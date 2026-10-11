#!/usr/bin/env python3
"""회전교차로 백 전체 트래커 재생 + 지표 (2026-10-06).

사용: replay_all.py PAIRS.txt OUT_DIR TAG [EXTRA_PARAM_YAML] [-j 4]
PAIRS.txt 각 줄: main.bag heavy.bag
회전교차로 상태(/mission/roundabout/state >= 1) 구간 앞 6 s ~ 끝 뒤 3 s 만 재생한다.
"""
import argparse
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

import rosbag

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tracker_metrics import analyze  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def window(main):
    bag = rosbag.Bag(main)
    t = [ts.to_sec() for _, m, ts in bag.read_messages(topics=['/mission/roundabout/state']) if m.state >= 1]
    return (min(t), max(t)) if t else None


def run(job):
    i, main, heavy, out, extra, bagdir = job
    main, heavy = os.path.join(bagdir, main), os.path.join(bagdir, heavy)
    w = window(main)
    start = min(rosbag.Bag(main).get_start_time(), rosbag.Bag(heavy).get_start_time())
    # 회전교차로 상태 시작 6 s 전부터, 상태 끝 + extra_after 까지(시뮬에서 자차가 녹화보다 늦게 진입해도 트랙이 있게).
    extra_after = float(os.environ.get('REPLAY_EXTRA_AFTER', '3'))
    env = dict(os.environ, REPLAY_PORT=str(11450 + i),
               PLAY_ARGS='-s %.1f -u %.1f' % (max(0.0, w[0] - 6 - start), w[1] - w[0] + 6 + extra_after))
    subprocess.run([os.path.join(HERE, 'replay_tracker.sh'), main, heavy, out] + ([extra] if extra else []),
                   env=env, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return main, out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pairs'); ap.add_argument('outdir'); ap.add_argument('tag')
    ap.add_argument('extra', nargs='?', default='')
    ap.add_argument('-j', type=int, default=4)
    ap.add_argument('--bagdir', default=os.path.expanduser('~/catkin_ws/bags'))
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)
    jobs = []
    for i, line in enumerate(open(a.pairs)):
        p = line.split()
        if len(p) >= 2:
            jobs.append((i % a.j, p[0], p[1], os.path.join(a.outdir, '%s_%s' % (a.tag, p[0][12:])), a.extra, a.bagdir))
    # 같은 포트를 동시에 쓰지 않도록 j 개씩 묶어 돌린다.
    results = []
    with ThreadPoolExecutor(a.j) as ex:
        for k in range(0, len(jobs), a.j):
            results += list(ex.map(run, jobs[k:k + a.j]))
    tot = {'fv': 0, 'so': 0, 'real': 0, 'nm': 0, 'delay': []}
    for main, out in results:
        if not os.path.exists(out):
            print(os.path.basename(main), 'NO OUTPUT'); continue
        r = analyze(out)
        tot['fv'] += len(r['false_vehicle']); tot['so'] += r['static_out']; tot['real'] += r['real']
        tot['nm'] += r['never_moved_real']; tot['delay'] += r['delay']
        print('%-40s false_vehicle %d static_out %2d real %2d never_moved %d %s' % (
            os.path.basename(main), len(r['false_vehicle']), r['static_out'], r['real'], r['never_moved_real'],
            r['false_vehicle'][:4]))
    import numpy as np
    d = np.array(tot['delay'])
    print('TOTAL false_vehicle %d static_out %d real %d never_moved %d delay p50 %.2f p90 %.2f max %.2f' % (
        tot['fv'], tot['so'], tot['real'], tot['nm'], np.percentile(d, 50), np.percentile(d, 90), d.max()))


if __name__ == '__main__':
    main()
