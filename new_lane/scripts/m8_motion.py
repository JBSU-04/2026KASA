"""2026-10-13 M8 정적·동적 판정: 세계 좌표 외곽 강체 이동 근거.

군집 중심·칼만 속도는 보는 각도에 따라 흔들린다(사고 차량 0.6~1.2 m/s 가짜 속도). 여기서는 물체 외곽
(x/y 각 축 min·max)이 '같은 방향으로 함께' 움직인 양만 이동으로 본다. 커지거나 쪼개지는 변화는 0.

  - dynamic   : 창(≥ window_min) 강체 이동 ≥ dyn_m 이 같은 방향으로 confirm 회 연속
                (외곽 길이+폭 ≥ large_extent 이면 dyn_m 2배)
  - candidate : 아직 dynamic 은 아니지만 짧은 창(≥ candidate_window)에서 강체 속도 ≥ candidate_mps
  - static    : static_window 이상 창에서 강체 이동 < static_m
  - unknown   : 그 밖
속도 분산(KF vel_std)은 쓰지 않는다(12:38:06 백 id 20: 분산이 커 끝까지 unknown).

2026-10-13(18:10:51, 18:12:04 백) 보강:
  - 관측 공백 > max_gap 이면 이동 근거(이력·적중)를 다시 쌓는다. 공백 0.8 s 뒤 트랙이 2.5 m 옆 다른 조각에 붙어
    (id 18) 동적, 공백 0.66 s 사이 두 관측만으로 후보(id 12)가 되어 정지했다. 10 Hz 로 보이는 실제 이동체는 공백이 없다.
  - 최근 reveal_recent_sec 안에 외곽이 커지던(revealing) 물체는 후보를 reveal_candidate_confirm 회 연속 요구한다.
    회피 중 보는 각도가 바뀌던 정적 장애물(id 5)이 1 프레임 후보 -> 예측 정지. 모양이 안정된 물체는 그대로 1 프레임.
  - 후보는 직전 관측 대비로도 같은 방향으로 나아가야 한다(필요 속도 x 간격의 candidate_progress 배).
    한 프레임만 튄 외곽은 '튈 때'와 '돌아올 때' 두 창에서 같은 크기 이동으로 보이지만 돌아온 뒤 진행은 0 이다.
"""
import math
import time

import numpy as np
from scipy.spatial import cKDTree


def reject_common_pose_shift(models):
    """Only established static references can veto one frame's common jump."""
    deltas=[m.evidence['pose_reference_delta'] for m in models
            if 'pose_reference_delta' in m.evidence]
    if len(deltas)<4:return False
    deltas=np.asarray(deltas);common=np.median(deltas,axis=0)
    if np.linalg.norm(common)<=.2 or (np.linalg.norm(deltas-common,axis=1)<.12).mean()<.75:return False
    for m in models:m.reject_common_pose_shift()
    return True


class RigidMotion(object):
    def __init__(self, dyn_m=.4, static_m=.15, large_extent=3., window_min=.3, static_window=.8,
                 candidate_window=.2, candidate_mps=1., confirm=2, release_sec=1., history_sec=1.5,
                 reveal_ratio=1.8, reveal_extent=.3, candidate_confirm=1,
                 net_m=.6, net_window=.6, candidate_min_m=.3, small_extent=1., small_candidate_mps=1.5,
                 max_gap=.3, reveal_recent_sec=.5, reveal_candidate_confirm=2, candidate_progress=.3, surface_translation_enabled=True, bidirectional_surface_enabled=True,
                 min_surface_points=6, stationary_surface_priority=True):
        self.dyn_m, self.static_m, self.large_extent = dyn_m, static_m, large_extent
        self.window_min, self.static_window = window_min, static_window
        self.candidate_window, self.candidate_mps = candidate_window, candidate_mps
        self.confirm, self.release_sec, self.history_sec = confirm, release_sec, history_sec
        self.reveal_ratio, self.reveal_extent = reveal_ratio, reveal_extent
        self.candidate_confirm, self.candidate_hits = candidate_confirm, 0
        # 2026-10-13(16:09:08, 16:11:00 백): 수명 순이동 0.1 m 인 작은 정적 조각이 같은 방향 흔들림 2회로 동적·후보가
        #   되어 정지 hold -> 회피 경로가 기준경로로 튀었다. 실제 이동은 한 방향으로 쌓이므로 순이동을 함께 본다.
        self.net_m, self.net_window = net_m, net_window
        self.candidate_min_m, self.small_extent, self.small_candidate_mps = candidate_min_m, small_extent, small_candidate_mps
        self.max_gap, self.reveal_recent_sec, self.reveal_candidate_confirm = max_gap, reveal_recent_sec, reveal_candidate_confirm
        self.candidate_progress = candidate_progress
        self.last_reveal = -1e9
        self.surface_translation_enabled = surface_translation_enabled
        self.bidirectional_surface_enabled = bidirectional_surface_enabled
        # 2026-10-13(20:46:03 백 id 7): 점 2~8개 바닥 깜빡임이 3점 대응만으로 '표면 이동'이 되어 후보.
        #   독립 표면점(2 cm 중복 제거)이 이 수 미만이면 이동을 검증할 수 없다(검사 불가 = 이동 근거 없음).
        self.min_surface_points = min_surface_points
        self.stationary_surface_priority = stationary_surface_priority
        self.surfaces = {}
        self._surface_used=False
        self._surface_detail={}
        self._surface_cache = {}
        self.coordinate_version = None
        self.last_stamp = None
        self.last_result = ('unknown', False, np.zeros(2))
        self.bounds = []
        self.hits = 0
        self.last_dir = None
        self.state = 'unknown'
        self.last_moving = -1e9
        self.velocity = np.zeros(2)
        self.evidence = {}

    @staticmethod
    def _axis(d0, d1):
        return math.copysign(min(abs(d0), abs(d1)), d0) if d0 * d1 > 0 else 0.

    def _shift(self, old, new):
        return np.array([self._axis(new[1] - old[1], new[2] - old[2]), self._axis(new[3] - old[3], new[4] - old[4])])

    def _revealing(self, old, new):
        """멀리서 일부만 보이던 물체가 드러나며 점 수·외곽이 커지는 변화(12:38:06 백 id 5)는 이동 근거가 아니다."""
        ext_old = (old[2] - old[1]) + (old[4] - old[3])
        ext_new = (new[2] - new[1]) + (new[4] - new[3])
        return new[5] > self.reveal_ratio * max(old[5], 1) or ext_new > ext_old + self.reveal_extent

    def _surface_pair(self, old_stamp, stamp):
        key=(old_stamp,stamp)
        if key in self._surface_cache:return self._surface_cache[key]
        a=self.surfaces.get(old_stamp);b=self.surfaces.get(stamp)
        if self.bidirectional_surface_enabled:
            # Vertical returns sharing XY are one surface feature, not votes.
            if a is not None:a=a[np.unique(np.round(a/.02),axis=0,return_index=True)[1]]
            if b is not None:b=b[np.unique(np.round(b/.02),axis=0,return_index=True)[1]]
        failure=(False,np.zeros(2),0.,float('inf'),'surface_unavailable')
        if a is None or b is None or min(len(a),len(b))<max(3,self.min_surface_points):return failure
        tree=getattr(self,'_surface_tree',None)
        if tree is None:
            tree=cKDTree(b);self._surface_tree=tree
        distance,index=tree.query(a)
        fixed_delta=np.median(b[index]-a,axis=0)
        # A stationary shared surface must not be replaced by a moving box fit.
        fixed=distance<=.12
        fixed_ratio=.5 if self.bidirectional_surface_enabled else .65
        slide=self._axis_slide(a,b)
        c=a-a.mean(0);_,axes=np.linalg.eigh(c.T@c)
        span=np.ptp(c@axes,axis=0)
        extended=self.stationary_surface_priority and max(span)>=6. and max(span)/max(min(span),.15)>=6.
        fixed_surface=(fixed.mean()>=fixed_ratio and len(set(index[fixed]))>=3 and np.linalg.norm(fixed_delta)<=.04)
        if extended and fixed_surface:
            self._surface_cache[key]=(False,np.zeros(2),float(fixed.mean()),float(np.median(distance)),'stationary_extended_surface')
            return self._surface_cache[key]
        if (fixed.mean()>=fixed_ratio and len(set(index[fixed]))>=3 and np.linalg.norm(fixed_delta)<=.04 and
                np.linalg.norm(slide)<=.06):
            self._surface_cache[key]=(False,np.zeros(2),float((distance<=.12).mean()),
                float(np.median(distance)),'stationary_shared_surface')
            return self._surface_cache[key]
        seeds=[np.median(b,axis=0)-np.median(a,axis=0),b.mean(0)-a.mean(0),
               b.min(0)-a.min(0),b.max(0)-a.max(0),slide]
        best=None
        for seed in seeds:
            delta=seed.copy()
            for _ in range(2):
                distance,index=tree.query(a+delta)
                good=distance<=.12
                if good.sum()<3:break
                delta=np.median(b[index[good]]-a[good],axis=0)
            distance,index=tree.query(a+delta);good=distance<=.12
            ratio=float(good.mean());residual=float(np.median(distance))
            if good.sum()<3 or ratio<.5 or residual>.12:continue
            # Bidirectional correspondence: multiple old points cannot all
            # claim one newly revealed point as a translated surface.
            if len(set(index[good])) < max(3, .5*good.sum()):continue
            if self.bidirectional_surface_enabled:
                # Reciprocal correspondences, restricted to the old surface's
                # translated region: newly revealed area is not a match veto.
                translated=a+delta
                reverse_tree=cKDTree(translated)
                reverse_distance,reverse_index=reverse_tree.query(b)
                eligible=np.all((b>=translated.min(0)-.12)&(b<=translated.max(0)+.12),axis=1)
                if not eligible.any() or (reverse_distance[eligible]<=.12).mean()<.5:continue
                reciprocal=good & (reverse_index[index]==np.arange(len(a)))
                if reciprocal.mean()<.5:continue
            score=(ratio,-residual)
            if best is None or score>best[0]:best=(score,delta,ratio,residual)
            if ratio>=.95 and residual<=.02:break
        if best is None:return failure
        delta,ratio,residual=best[1:]
        if extended:
            normal=axes[:,int(np.argmin(span))]
            if abs(float(delta@normal))<=.04:
                self._surface_cache[key]=(False,np.zeros(2),ratio,residual,'extended_surface_motion_unobservable')
                return self._surface_cache[key]
        valid=np.linalg.norm(delta)> .04 and np.linalg.norm(delta)/max(stamp-old_stamp,1e-6)<=6.
        result=(bool(valid),delta,ratio,residual,'matched_surface_translation' if valid else 'stationary_or_jump')
        self._surface_cache[key]=result
        return result

    def _axis_slide(self, a, b):
        """Shared face sliding along itself (aperture): both ends of the principal axis move together.

        A face parallel to the motion keeps nearest-neighbour distances small, so it looks
        stationary. Growth/shrink moves one end only and stays zero here.
        """
        c=a-a.mean(0)
        try:w,v=np.linalg.eigh(c.T@c)
        except np.linalg.LinAlgError:return np.zeros(2)
        axis=v[:,int(np.argmax(w))];pa=a@axis;pb=b@axis
        if pa.max()-pa.min()<.3:return np.zeros(2)
        return self._axis(pb.min()-pa.min(),pb.max()-pa.max())*axis

    def update(self, points_xy, now, arrival_time=None, coordinate_version=0):
        """One original scan timestamp; arrival time only validates freshness."""
        if not self.surface_translation_enabled:return self._update_bounds(points_xy,now)
        begin=time.perf_counter()
        arrival=now if arrival_time is None else arrival_time
        if not math.isfinite(now) or not math.isfinite(arrival) or not -.05<=arrival-now<=.35:
            self.bounds=[];self.surfaces={};self.hits=self.candidate_hits=0
            self.state='unknown';self.velocity=np.zeros(2);self.last_stamp=None
            self.evidence=dict(reason='stale_measurement',observation_stamp=now,
                               surface_translation_valid=False)
            return self.state,False,self.velocity.copy()
        if self.last_stamp is not None and now==self.last_stamp and coordinate_version==self.coordinate_version:
            return self.last_result[0],self.last_result[1],self.last_result[2].copy()
        if (self.last_stamp is not None and (now<self.last_stamp or now-self.last_stamp>self.max_gap+1e-6)
                or self.coordinate_version is not None and coordinate_version!=self.coordinate_version):
            self.bounds=[];self.surfaces={};self.hits=self.candidate_hits=0
            self.state='unknown';self.last_dir=None;self.velocity=np.zeros(2)
            self.last_moving=-1e9;self.last_reveal=-1e9
        p=np.asarray(points_xy,float).reshape(-1,2)
        if len(p)<3 or not np.isfinite(p).all():
            self.evidence=dict(reason='insufficient_surface',observation_stamp=now,surface_translation_valid=False)
            return 'unknown',False,np.zeros(2)
        previous_state=self.state
        previous_bound=self.bounds[-1] if self.bounds else None
        self.surfaces={t:q for t,q in self.surfaces.items() if now-t<=self.history_sec}
        # Bounded matching evidence; full points still determine observed bounds.
        self.surfaces[now]=p[np.linspace(0,len(p)-1,min(64,len(p)),dtype=int)]
        self._surface_cache={};self._surface_tree=None;self._surface_used=False;self._surface_detail={}
        self.evidence={}
        result=self._update_bounds(p,now)
        # Bounds may exchange faces or fitted fragments. Every NEW motion
        # claim, including non-revealing candidates, needs the same surface.
        if result[1] or result[0] == 'dynamic' or (self.stationary_surface_priority and previous_state=='dynamic'):
            prior=[t for t in self.surfaces if self.candidate_window-1e-6 <= now-t <= self.max_gap+1e-6]
            valid=False;delta=np.zeros(2)
            if prior:
                baseline=max(prior)
                valid,delta,ratio,residual,reason=self._surface_pair(baseline,now)
                self._surface_detail.update(surface_match_ratio=ratio,
                    surface_residual_m=residual if math.isfinite(residual) else None,
                    surface_match_reason=reason)
            if valid:
                self._surface_used=True
                vector=delta/max(now-baseline,1e-6)
                self.velocity=vector.copy()
                result=(result[0],result[1],vector)
            elif self.stationary_surface_priority and self._surface_detail.get('surface_match_reason')=='stationary_extended_surface':
                self.state='static';self.hits=self.candidate_hits=0;self.last_dir=None
                self.last_moving=-1e9;self.velocity=np.zeros(2)
                result=('static',False,self.velocity.copy())
                self._surface_detail['motion_retracted']='stationary_extended_surface'
            elif self.stationary_surface_priority and self._surface_detail.get('surface_match_reason')=='extended_surface_motion_unobservable':
                self.state='unknown';self.hits=self.candidate_hits=0;self.last_dir=None
                self.velocity=np.zeros(2);result=('unknown',False,self.velocity.copy())
            elif result[0] == 'static':
                self.state='static';self.velocity=np.zeros(2)
                result=('static',False,self.velocity.copy())
            elif previous_state == 'dynamic':
                # One partial view must not cancel a confirmed crossing.
                result=('dynamic',False,self.last_result[2].copy())
                self.state='dynamic';self.velocity=result[2].copy()
            else:
                self.state='static' if previous_state=='static' else 'unknown'
                self.hits=self.candidate_hits=0;self.velocity=np.zeros(2)
                result=(self.state,False,self.velocity.copy())
                self._surface_detail['candidate_rejected']='unverified_surface_motion'
        self.evidence.update(self._surface_detail)
        self.evidence['independent_scans']=len(self.bounds)
        self.evidence['reason']=('surface_translation' if self._surface_used else
            'shape_reveal_unverified' if self.evidence.get('revealing',False) else
            'rigid_candidate' if result[1] else 'rigid_'+result[0])
        self.evidence.update(observation_stamp=now,coordinate_version=coordinate_version,surface_translation_valid=self._surface_used,
                             surface_translation_source='matched_surface' if self._surface_used else 'rigid_bounds',
                             observation_age_sec=arrival-now)
        if previous_state=='static' and previous_bound is not None:
            delta=self._shift(previous_bound,self.bounds[-1])
            self.evidence['pose_reference_delta']=delta.tolist()
        self.evidence['added_compute_ms']=(time.perf_counter()-begin)*1000.
        self.last_stamp=now;self.coordinate_version=coordinate_version
        self.last_result=(result[0],result[1],result[2].copy())
        return result

    def reject_common_pose_shift(self):
        """A frame-wide displacement of established static references is not motion."""
        self.hits=self.candidate_hits=0;self.last_dir=None;self.last_moving=-1e9
        self.state='unknown';self.velocity=np.zeros(2)
        self.bounds=self.bounds[-1:]
        self.evidence.update(common_pose_shift=True,surface_translation_valid=False)
        self.last_result=(self.state,False,self.velocity.copy())

    def _matched_reveal(self, old, new):
        if not self.surface_translation_enabled:return False,None
        result=self._surface_pair(old[0],new[0])
        valid,delta,ratio,residual,reason=result
        if valid and self.bidirectional_surface_enabled:
            prior=[t for t in self.surfaces if self.candidate_window-1e-6<=new[0]-t<=self.max_gap+1e-6 and t!=old[0]]
            previous=max(prior) if prior else None
            if previous is not None and previous!=old[0]:
                step_ok,step,_,_,step_reason=self._surface_pair(previous,new[0])
                v=delta/max(new[0]-old[0],1e-6)
                sv=step/max(new[0]-previous,1e-6)
                aligned=(step_ok and np.dot(v,sv)>0 and
                         np.dot(v,sv)>=math.cos(math.radians(45))*np.linalg.norm(v)*np.linalg.norm(sv) and
                         np.linalg.norm(v-sv)<=max(.5,.5*np.linalg.norm(v)))
                if not aligned:valid=False;reason='inconsistent_surface_step_'+step_reason
        self._surface_detail=dict(surface_match_ratio=ratio,
            surface_residual_m=residual if math.isfinite(residual) else None,surface_match_reason=reason)
        return valid,delta

    def _update_bounds(self, points_xy, now):
        """points_xy: (N,2) 세계 좌표. 반환 (state, candidate, velocity)."""
        p = np.asarray(points_xy, float).reshape(-1, 2)
        if len(p) < 3:
            return self.state, False, self.velocity
        b = (now, p[:, 0].min(), p[:, 0].max(), p[:, 1].min(), p[:, 1].max(), len(p))
        if self.bounds and now - self.bounds[-1][0] > self.max_gap+(1e-6 if self.surface_translation_enabled else 0.):
            # 관측 공백: 같은 물체인지 이어서 확인할 수 없으므로 이동 근거를 다시 쌓는다(상태는 유지).
            self.bounds, self.hits, self.candidate_hits, self.last_dir = [], 0, 0, None
        self.bounds = [q for q in self.bounds if now - q[0] <= self.history_sec] + [b]
        extent = (b[2] - b[1]) + (b[4] - b[3])
        dyn_m = self.dyn_m * (2. if extent >= self.large_extent else 1.)
        candidate = False
        old = [q for q in self.bounds if now - q[0] >= self.window_min-(1e-6 if self.surface_translation_enabled else 0.)]
        if old:
            o = old[-1]
            dt = now - o[0]
            disp = self._shift(o, b)
            mag = float(np.linalg.norm(disp))
            revealing = self._revealing(o, b)
            surface_long=False
            if revealing:
                self.last_reveal = now
                surface_long, surface_delta=self._matched_reveal(o,b)
                if surface_long:
                    disp=surface_delta;mag=float(np.linalg.norm(disp))
            if mag >= dyn_m and (not revealing or surface_long):
                direction = disp / mag
                same = self.last_dir is None or float(direction @ self.last_dir) >= math.cos(math.radians(45.))
                self.hits = self.hits + 1 if same else 1
                self.last_dir = direction
                self.last_moving = now
                first = self.bounds[0]
                # 이력이 net_window 미만인 새 트랙은 동적 확정하지 않는다(후보까지만).
                net_ok = now - first[0] >= self.net_window and float(np.linalg.norm(self._shift(first, b))) >= max(self.net_m, dyn_m)
                if not net_ok and surface_long and now-first[0]>=self.net_window:
                    matched,net=self._matched_reveal(first,b)
                    net_ok=matched and float(np.linalg.norm(net))>=max(self.net_m,dyn_m)
                if surface_long:self._surface_used=True
                if self.hits >= self.confirm and net_ok:
                    self.state = 'dynamic'
                    self.velocity = disp / max(dt, 1e-3)
            else:
                self.hits = 0
                # 정적: static_window 이전 샘플과 비교해도 함께 움직인 양이 static_m 미만.
                far = [q for q in self.bounds if now - q[0] >= self.static_window]
                still = bool(far) and float(np.linalg.norm(self._shift(far[-1], b))) < self.static_m
                if still and now - self.last_moving >= self.release_sec:
                    self.state = 'static'
                    self.velocity = np.zeros(2)
                    self.last_dir = None
                elif self.state == 'dynamic' and not revealing and now - self.last_moving >= self.release_sec:
                    self.state = 'unknown'
                    self.velocity = np.zeros(2)
            self.evidence = dict(rigid_disp_m=round(mag, 3), window_s=round(dt, 3), extent_m=round(extent, 2),
                                 hits=self.hits, threshold_m=dyn_m, revealing=bool(revealing))
        short = [q for q in self.bounds if now - q[0] >= self.candidate_window-(1e-6 if self.surface_translation_enabled else 0.)]
        if self.state != 'dynamic' and short:
            o = short[-1]
            dt = now - o[0]
            disp = self._shift(o, b)
            dist = float(np.linalg.norm(disp))
            speed = dist / max(dt, 1e-3)
            need = self.candidate_mps * (2. if extent >= self.large_extent else 1.)
            if extent < self.small_extent:
                need = max(need, self.small_candidate_mps)
            progress_ok = False
            if dist > 1e-6 and len(self.bounds) >= 2:
                prev = self.bounds[-2]
                step = float(self._shift(prev, b) @ (disp / dist))
                progress_ok = step >= self.candidate_progress * need * (now - prev[0])
            revealing_short=self._revealing(o,b)
            surface_short=False
            if revealing_short:
                surface_short, surface_delta=self._matched_reveal(o,b)
                if surface_short:
                    disp=surface_delta;dist=float(np.linalg.norm(disp));speed=dist/max(dt,1e-3)
                    prev=self.bounds[-2]
                    step_valid,step=self._matched_reveal(prev,b)
                    progress_ok=(step_valid and dist>1e-6 and
                        float(step@(disp/dist))>=self.candidate_progress*need*(now-prev[0]))
            if speed >= need and dist >= self.candidate_min_m and progress_ok and (not revealing_short or surface_short):
                self.candidate_hits += 1
                need_hits = self.candidate_confirm
                if not surface_short and now - self.last_reveal <= self.reveal_recent_sec:
                    need_hits = max(need_hits, self.reveal_candidate_confirm)
                if self.candidate_hits >= need_hits:
                    candidate = True
                    if surface_short:self._surface_used=True
                    self.velocity = disp / max(dt, 1e-3)
            else:
                self.candidate_hits = 0
        self.evidence['candidate_motion_verified']=bool(candidate)
        return self.state, candidate, self.velocity
