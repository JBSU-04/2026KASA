#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""주행 bag 에서 제어 지연 사슬과 센서 주기를 재서 기준 범위와 비교한다.

환경(ROS PC, MORAI PC, 네트워크)이 바뀌었을 때 짧게 한 번 달리고 돌려 보면,
주행 성능 차이가 환경 지연 때문인지 파라미터 때문인지 바로 가를 수 있다.
주행 중 노드가 아니라 사후 분석 도구이며, 제어에는 영향을 주지 않는다.

    rosrun total_bringup env_delay_check.py <competition_*.bag> [...]

필요 토픽: /ctrl_cmd /odom /Competition_topic (+ 있으면 /comm/latency_ms).
competition.launch record:=true 로 받은 bag 이면 모두 들어 있다.

기준 범위(BASELINE)는 아래 4개 주행의 실측에서 잡았다.
    2026-09-18 대회장 15:47 / 16:08 / 16:27,  2026-10-01 연습 환경 17:36
                          대회장          연습
    조향명령 -> 바퀴각     35 ms           35 ms
    바퀴각 -> 요레이트     90~105 ms       110 ms
    실제 요 -> /odom 요    45~60 ms        50 ms
    합계                   180~190 ms      195 ms
    요 게인(모델 대비)     0.71~0.76       0.69
즉 두 환경의 지연 사슬은 사실상 같았다. 범위를 벗어나면 그 환경에서는
MPC 가중치(urban_control 등)를 다시 봐야 한다.
"""
import collections
import math
import sys

import numpy as np
import rosbag

# (하한, 상한). 실측 범위에 여유를 둔 값.
BASELINE = collections.OrderedDict([
    ('cmd_to_wheel_ms', (20.0, 55.0)),
    ('wheel_to_yaw_ms', (75.0, 130.0)),
    ('truth_to_odom_yaw_ms', (30.0, 75.0)),
    ('cmd_to_odom_yaw_ms', (160.0, 220.0)),
    ('yaw_gain_vs_model', (0.62, 0.85)),
    ('ctrl_cmd_gap_p99_ms', (0.0, 80.0)),
    ('ctrl_cmd_over_100ms_pct', (0.0, 0.5)),
])
RATE_BASELINE = collections.OrderedDict([
    ('/ctrl_cmd', (19.0, 21.0)),
    ('/odom', (48.0, 52.0)),
    ('/Competition_topic', (43.0, 52.0)),
    ('/imu', (28.0, 45.0)),
    ('/gps', (26.0, 32.0)),
    ('/image_jpeg/compressed', (27.0, 32.0)),
    ('/velodyne_points', (8.0, 10.5)),
])
LF = 3.0          # mpc_params.yaml Lf
MIN_SPEED = 5.0   # 이 속도 이상 구간만 사용 (저속은 요레이트 신호가 약하다)
STEP = 0.005


def _best_lag(a, b, mask, lo=-40, hi=120):
    """a 를 b 에 맞추는 지연[ms]. 표본이 부족하면 nan."""
    idx = np.where(mask)[0]
    idx = idx[(idx + lo >= 0) & (idx + hi < len(b))]
    if len(idx) < 400:
        return float('nan')
    best_lag, best_r = 0, -1.0
    for lag in range(lo, hi):
        r = np.corrcoef(a[idx], b[idx + lag])[0, 1]
        if r > best_r:
            best_lag, best_r = lag, r
    return best_lag * STEP * 1e3


def measure(path):
    cmd, odom, comp = [], [], []
    rates = collections.defaultdict(list)
    with rosbag.Bag(path) as bag:
        for topic, msg, t in bag.read_messages(
                topics=['/ctrl_cmd', '/odom', '/Competition_topic', '/comm/latency_ms']):
            ts = t.to_sec()
            if topic == '/ctrl_cmd':
                cmd.append((ts, msg.steering))
            elif topic == '/odom':
                q = msg.pose.pose.orientation
                odom.append((ts, math.atan2(2 * (q.w * q.z + q.x * q.y),
                                            1 - 2 * (q.y * q.y + q.z * q.z))))
            elif topic == '/Competition_topic':
                comp.append((ts, msg.velocity.x, msg.heading, msg.front_steer_angle))
            else:
                for status in msg.status:
                    for kv in status.values:
                        if kv.key == 'hz':
                            try:
                                rates[status.name].append(float(kv.value))
                            except ValueError:
                                pass
    if len(cmd) < 200 or len(odom) < 200 or len(comp) < 200:
        raise RuntimeError('필요 토픽이 부족하다 (/ctrl_cmd /odom /Competition_topic)')

    cmd, odom, comp = np.array(cmd), np.array(odom), np.array(comp)
    t0, t1 = comp[0, 0] + 1.0, comp[-1, 0] - 1.0
    grid = np.arange(t0, t1, STEP)
    speed = np.interp(grid, comp[:, 0], comp[:, 1])
    steer_cmd = np.interp(grid, cmd[:, 0], cmd[:, 1])
    # MORAI front_steer_angle 은 deg 이고 ctrl_cmd 와 부호가 반대다.
    wheel = -np.radians(np.interp(grid, comp[:, 0], comp[:, 3]))

    def smooth(x, n=21):
        return np.convolve(x, np.ones(n) / n, 'same')

    yaw_rate_odom = smooth(np.gradient(np.interp(grid, odom[:, 0], np.unwrap(odom[:, 1])), STEP))
    yaw_rate_true = smooth(np.gradient(
        np.interp(grid, comp[:, 0], np.unwrap(np.radians(comp[:, 2]))), STEP))
    moving = speed > MIN_SPEED

    out = collections.OrderedDict()
    out['cmd_to_wheel_ms'] = _best_lag(steer_cmd, wheel, moving)
    out['wheel_to_yaw_ms'] = _best_lag(speed * wheel, yaw_rate_true, moving)
    out['truth_to_odom_yaw_ms'] = _best_lag(yaw_rate_true, yaw_rate_odom, moving)
    out['cmd_to_odom_yaw_ms'] = _best_lag(speed * steer_cmd, yaw_rate_odom, moving)

    total = out['cmd_to_odom_yaw_ms']
    if total == total:
        shift = int(round(total / 1e3 / STEP))
        idx = np.where(moving)[0]
        idx = idx[idx + shift < len(grid)]
        model = speed * steer_cmd / LF
        out['yaw_gain_vs_model'] = float(np.polyfit(model[idx], yaw_rate_odom[idx + shift], 1)[0])
    else:
        out['yaw_gain_vs_model'] = float('nan')

    gap = np.diff(cmd[:, 0]) * 1e3
    out['ctrl_cmd_gap_p99_ms'] = float(np.percentile(gap, 99))
    out['ctrl_cmd_over_100ms_pct'] = float(100.0 * np.mean(gap > 100.0))

    rate_out = collections.OrderedDict()
    for name in RATE_BASELINE:
        key = 'comm/' + name
        if rates.get(key):
            rate_out[name] = float(np.median(rates[key]))
    return out, rate_out, float(comp[-1, 0] - comp[0, 0]), float(speed.max())


def _row(name, value, lo, hi, unit=''):
    if value != value:
        return '  [ -- ] %-28s 측정 불가 (속도 %.0f m/s 이상 구간 부족)' % (name, MIN_SPEED), False
    ok = lo <= value <= hi
    return '  [%s] %-28s %8.2f %s  (기준 %g ~ %g)' % (
        ' OK ' if ok else 'WARN', name, value, unit, lo, hi), not ok


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    worst = 0
    for path in sys.argv[1:]:
        try:
            delays, rates, duration, vmax = measure(path)
        except Exception as exc:  # bag 하나가 깨져도 나머지는 본다
            print('%s\n  실패: %s' % (path, exc))
            worst = max(worst, 2)
            continue
        print('%s  (%.0f s, 최고 %.1f m/s)' % (path, duration, vmax))
        warned = False
        for name, (lo, hi) in BASELINE.items():
            line, bad = _row(name, delays[name], lo, hi)
            print(line)
            warned |= bad
        for name, (lo, hi) in RATE_BASELINE.items():
            if name in rates:
                line, bad = _row(name + ' Hz', rates[name], lo, hi)
                print(line)
                warned |= bad
        if not rates:
            print('  (센서 주기: /comm/latency_ms 가 bag 에 없어 생략)')
        print('  => %s' % ('기준 범위를 벗어난 항목이 있다. 이 환경에서는 제어 튜닝을 다시 확인할 것.'
                           if warned else '지연/주기 모두 기준 범위 안.'))
        worst = max(worst, 1 if warned else 0)
    return worst


if __name__ == '__main__':
    sys.exit(main())
