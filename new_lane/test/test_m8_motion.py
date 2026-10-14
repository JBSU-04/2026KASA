"""2026-10-13 터널 정적·동적 강체 이동 판정(m8_motion.RigidMotion)."""
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from m8_motion import RigidMotion


def box(cx, cy, l=.6, w=.6, n=30, seed=0):
    r = np.random.RandomState(seed)
    return np.c_[cx + r.uniform(-l / 2, l / 2, n), cy + r.uniform(-w / 2, w / 2, n)]


def run(frames, dt=.1, **kw):
    m = RigidMotion(**kw);out = []
    for k, pts in enumerate(frames):
        out.append(m.update(pts, k * dt))
    return m, out


def test_crossing_person_becomes_candidate_then_dynamic():
    # 3 m/s 횡단(12:38:06 id 20 유형): 속도 분산과 무관하게 0.3 s 안 후보, 이후 동적.
    m, out = run([box(10., -3. + .3 * k, seed=k) for k in range(8)])
    first_cand = next(k for k, o in enumerate(out) if o[1])
    assert first_cand <= 3
    assert out[-1][0] == 'dynamic' and np.linalg.norm(out[-1][2]) > 2.


def test_static_object_with_center_jitter_stays_not_dynamic():
    # 보는 각도에 따라 한쪽 경계만 흔들리는 정적 물체(사고 차량 유형).
    frames = []
    for k in range(20):
        p = box(10., 0., l=1.9, w=2.8, n=60, seed=k)
        if k % 2:p = p[p[:, 0] < 10.3]  # 절반만 보임 -> 중심 이동
        frames.append(p)
    m, out = run(frames)
    assert all(o[0] != 'dynamic' and not o[1] for o in out)
    assert out[-1][0] == 'static'


def test_revealing_partial_view_is_not_motion():
    # 멀리서 일부만 보이다 드러남(12:38:06 id 5): 점 수·외곽 증가는 이동 아님.
    frames = [box(17., 51., l=.3, w=.3, n=8, seed=k) for k in range(3)]
    frames += [box(17.4, 51.4, l=1.2, w=.8, n=25, seed=10 + k) for k in range(6)]
    m, out = run(frames)
    assert all(o[0] != 'dynamic' for o in out)


def test_large_object_needs_double_displacement():
    # 큰 물체(외곽 >= 3 m)는 0.8 m 이상 함께 이동해야 동적.
    slow = [box(10. + .1 * k, 0., l=2., w=2., n=80, seed=k) for k in range(10)]
    m, out = run(slow)
    assert all(o[0] != 'dynamic' for o in out)
    # Same physical surface translates; independently random interiors do not
    # provide a surface correspondence contract.
    surface = box(10., 0., l=2., w=2., n=80, seed=42)
    fast = [surface + [.5 * k, 0.] for k in range(10)]
    m, out = run(fast)
    assert out[-1][0] == 'dynamic'


def test_dynamic_releases_after_stopping():
    frames = [box(10., -3. + .3 * k, seed=k) for k in range(6)] + [box(10., -1.5, seed=20 + k) for k in range(25)]
    m, out = run(frames)
    assert any(o[0] == 'dynamic' for o in out[:8])
    assert out[-1][0] == 'static'


def test_new_small_track_two_jitters_not_dynamic():
    # 16:11:00 id 21 유형: 생긴 지 0.5 s 인 작은 조각이 같은 방향으로 두 번 흔들림 -> 동적 아님(이력 0.6 s 미만).
    frames = [box(10., 0., l=.5, w=.5, n=10, seed=k) for k in range(2)]
    frames += [box(10.45, 0., l=.5, w=.5, n=10, seed=5), box(10.9, 0., l=.5, w=.5, n=10, seed=6)]
    frames += [box(10.9, 0., l=.5, w=.5, n=10, seed=10 + k) for k in range(10)]
    m, out = run(frames)
    assert all(o[0] != 'dynamic' for o in out)


def test_small_object_slow_jitter_not_candidate():
    # 작은 물체(외곽 < 1 m)는 1.5 m/s·0.3 m 이상이어야 후보.
    frames = [box(10. + .12 * k, 0., l=.4, w=.4, n=12, seed=k) for k in range(8)]  # 1.2 m/s
    m, out = run(frames)
    assert not any(o[1] for o in out)


# 2026-10-13(20:46:03, 20:47:07 백) 실제 지지점: 회피 중 면 추가·분할·다른 조각 대응으로 생긴 가짜 후보.
#   이 트랙들은 모두 정적 물체(사고 차량·입구 장애물 조각)였고, 후보 -> 예측 정지 1.2 s 가 5회 반복됐다.
import json as _json
_FIX = Path(__file__).resolve().parent / 'fixtures' / 'm8_motion_false_candidates_20261013.json'


def _replay(frames, **kw):
    m = RigidMotion(**kw);out = []
    for f in frames:
        out.append(m.update(np.asarray(f['points'], float), f['stamp'], arrival_time=f['arrival']))
    return out


def test_recorded_shape_changes_never_become_motion():
    for name, frames in _json.loads(_FIX.read_text()).items():
        out = _replay(frames)
        assert not any(o[1] for o in out), name
        assert all(o[0] != 'dynamic' for o in out), name


def test_recorded_shape_changes_were_false_candidates_with_bounds_only():
    # 표면 대응을 끄면(복귀 스위치) 외곽 min/max 만으로 다시 후보가 생긴다(공백·reveal·진행 규칙만으로는 못 막음).
    hits = [name for name, frames in _json.loads(_FIX.read_text()).items()
            if any(o[1] for o in _replay(frames, surface_translation_enabled=False))]
    assert len(hits) >= 2


def _face(cx, cy, size, r, spacing=.05):
    ys = np.arange(-size / 2, size / 2 + 1e-9, spacing);front = np.c_[np.full(len(ys), cx - size / 2), cy + ys]
    xs = np.arange(-size / 2, 0, spacing);side = np.c_[cx + xs, np.full(len(xs), cy - size / 2)]
    p = np.vstack([front, side]);return p + r.normal(0, .02, p.shape)


def test_real_crossing_surfaces_are_verified_quickly():
    # 라이다 앞면점 횡단(0.8 m 2 m/s, 1.8 m 차량 1.5 m/s(긴 면이 진행 방향으로 미끄러짐)): 후보·동적이 늦지 않다.
    for size, v, cand_by, dyn_by in ((.8, 2., 4, 8), (1.8, 1.5, 7, 8), (.5, 3., 3, 8)):
        firsts = []
        for seed in range(10):
            r = np.random.RandomState(seed);m = RigidMotion();c = d = None
            for k in range(20):
                t = k * .1 + r.uniform(-.01, .01)
                s, cand, _ = m.update(_face(12., -3. + v * k * .1, size, r), t, arrival_time=t + .05)
                c = k if cand and c is None else c
                d = k if s == 'dynamic' and d is None else d
            firsts.append((c, d))
        assert sum(c is not None and c <= cand_by for c, _ in firsts) >= 9, (size, v, firsts)
        dyn = [99 if d is None else d for _, d in firsts]
        assert np.median(dyn) <= dyn_by and max(dyn) <= 15, (size, v, firsts)


def test_tiny_flicker_surface_cannot_verify_motion():
    # 20:46:03 id 7 유형: 점 3~4개 바닥 깜빡임은 표면 대응으로 이동을 검증할 수 없다.
    m = RigidMotion();out = []
    for k in range(10):
        p = np.array([[10. + .3 * (k % 2), 5.], [10.5 + .3 * (k % 2), 5.], [10.2 + .3 * (k % 2), 5.2]])
        out.append(m.update(p, k * .12, arrival_time=k * .12 + .05))
    assert not any(o[1] for o in out) and all(o[0] != 'dynamic' for o in out)
