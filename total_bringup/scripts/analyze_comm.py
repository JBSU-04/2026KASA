#!/usr/bin/env python3
"""백에서 UDP 지연과 토픽 Hz 를 뽑는다. comm_monitor 없이도 동작한다.

왜 두 벌인가.
  comm_monitor 는 주행 중 현장에서 보기 위한 것이고, 이 스크립트는 주행이
  끝난 뒤 백만 가지고 따지기 위한 것이다. 백에는 각 메시지의 수신시각(t)과
  송신측 header.stamp 가 모두 남으므로, comm_monitor 를 안 켰던 예전 백도
  그대로 분석할 수 있다. 대회장 수치를 집 수치와 비교할 때 이 쪽을 쓴다.

시계 offset 처리는 comm_monitor 와 같은 원리다.
  raw = 수신시각 - header.stamp 에는 두 PC 의 시계 차이가 통째로 섞인다.
  가장 한가했던 순간에는 전송지연이 0 에 가까우므로 raw 의 최솟값을
  offset 으로 보고, jitter = raw - offset 을 실제 지연 지표로 쓴다.

사용법
  python3 analyze_comm.py <bag> [<bag> ...]
  python3 analyze_comm.py --compare 집.bag 대회장.bag
"""
import argparse
import math
import sys

try:
    import rosbag
except ImportError as exc:  # pragma: no cover
    sys.exit('ROS 환경을 source 한 뒤 실행하세요: %s' % exc)


def pct(vals, p):
    if not vals:
        return None
    s = sorted(vals)
    k = (len(s) - 1) * p / 100.0
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    if lo == hi:
        return s[lo]
    return s[lo] * (hi - k) + s[hi] * (k - lo)


def analyze(path):
    """토픽별 {hz, gap_p95, offset, jit_med, jit_p95, n} 를 돌려준다."""
    bag = rosbag.Bag(path)
    recv = {}
    raw = {}
    for topic, msg, t in bag.read_messages():
        ts = t.to_sec()
        recv.setdefault(topic, []).append(ts)
        stamp = getattr(getattr(msg, 'header', None), 'stamp', None)
        if stamp is not None:
            st = stamp.to_sec()
            if st > 0:
                raw.setdefault(topic, []).append((ts - st) * 1000.0)
    dur = bag.get_end_time() - bag.get_start_time()
    bag.close()

    out = {}
    for topic, times in recv.items():
        n = len(times)
        if n < 2:
            continue
        times.sort()
        span = times[-1] - times[0]
        hz = (n - 1) / span if span > 1e-6 else 0.0
        gaps = [(times[i + 1] - times[i]) * 1000.0 for i in range(n - 1)]
        r = raw.get(topic)
        entry = {'n': n, 'hz': hz, 'gap_p95': pct(gaps, 95),
                 'gap_max': max(gaps), 'offset': None,
                 'jit_med': None, 'jit_p95': None, 'jit_max': None}
        if r:
            off = min(r)
            j = [v - off for v in r]
            entry.update(offset=off, jit_med=pct(j, 50),
                         jit_p95=pct(j, 95), jit_max=max(j))
        out[topic] = entry
    return out, dur


def fmt(v, w=7, d=1):
    return ('%*.*f' % (w, d, v)) if v is not None else ' ' * (w - 1) + '-'


def show(path, res, dur, only=None):
    print('=' * 78)
    print('%s   (%.1f s)' % (path.split('/')[-1], dur))
    print('%-30s %6s %8s %9s %9s %9s' %
          ('topic', 'Hz', 'gap p95', 'offset', 'jit p50', 'jit p95'))
    print('-' * 78)
    for topic in sorted(res):
        if only and not any(k in topic for k in only):
            continue
        e = res[topic]
        print('%-30s %6.1f %s %s %s %s' %
              (topic[:30], e['hz'], fmt(e['gap_p95'], 8),
               fmt(e['offset'], 9), fmt(e['jit_med'], 9),
               fmt(e['jit_p95'], 9)))
    print()
    print('  offset = 두 PC 시계차 추정(절대 지연 아님).')
    print('  jitter = raw - offset. 이게 실제로 밀린 양이다.')
    worst = [(e['jit_p95'], t) for t, e in res.items()
             if e['jit_p95'] is not None]
    if worst:
        j, t = max(worst)
        print('  최악 지터: %s  p95 %.1f ms' % (t, j))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--compare', action='store_true',
                    help='두 백의 주요 토픽을 나란히 비교')
    ap.add_argument('--only', nargs='*',
                    help='이 문자열이 든 토픽만 출력')
    args = ap.parse_args()

    results = [(p,) + analyze(p) for p in args.bags]
    for path, res, dur in results:
        show(path, res, dur, args.only)

    if args.compare and len(results) >= 2:
        (pa, ra, _), (pb, rb, _) = results[0], results[1]
        print('=' * 78)
        print('비교: %s  ->  %s' % (pa.split('/')[-1], pb.split('/')[-1]))
        print('%-28s %-20s %-20s' % ('topic', 'Hz (전 -> 후)',
                                     'jit p95 (전 -> 후)'))
        print('-' * 78)
        for topic in sorted(set(ra) & set(rb)):
            a, b = ra[topic], rb[topic]
            ja = fmt(a['jit_p95'], 7)
            jb = fmt(b['jit_p95'], 7)
            print('%-28s %7.1f -> %7.1f   %s -> %s' %
                  (topic[:28], a['hz'], b['hz'], ja, jb))


if __name__ == '__main__':
    main()
