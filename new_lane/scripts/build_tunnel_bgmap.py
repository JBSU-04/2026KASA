#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""터널 라이다 배경맵 만들기 / 오프라인 평가.

장애물 없는 터널 주행의 원본 점군(/velodyne_points)과 정확한 자차 포즈로
'노면 위에 원래 있는 것'(벽, 턱, 구조물) 격자를 만든다. 실행 중에는
tunnel_wall_detector 가 이 맵에 없는 점을 새 물체로 보고 선감속한다.

포즈는 GPS 복귀 오차가 작은 것을 써야 한다(2026-09-29 이후 어댑터: 격자 보정
IMU yaw 추측항법, 재획득 오차 0.05 m 안팎). 예전 백의 /odom 은 음영구간 끝에서
0.5~2 m 틀어져 있으니, 새 어댑터로 재현한 포즈 백을 쓴다.

사용법
  # 맵 만들기 (heavy 백 : 포즈 백) 여러 쌍 가능
  build_tunnel_bgmap.py build --out ../config/tunnel_bgmap.npz \\
      heavy_2057.bag:pose_2057.bag heavy_1623.bag:pose_1623.bag
  # 평가: 맵에 없는 물체 검출을 스캔마다 출력
  build_tunnel_bgmap.py eval --map ../config/tunnel_bgmap.npz heavy.bag:pose.bag
"""
import argparse
import math
import os
import sys

import numpy as np
import rosbag

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tunnel_wall_detector import (TunnelBgMap, above_ground_points,  # noqa: E402
                                  pointcloud2_to_xyz_array)

SENSOR_X = 1.10
GROUND_Z = -1.75
TUNNEL_HEADING_DEG = -151.84


def quat_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def load_poses(pose_bag, topic):
    t, x, y, yaw, modes = [], [], [], [], []
    t_in = 0.0
    with rosbag.Bag(pose_bag) as b:
        for top, m, bt in b.read_messages(topics=[topic, '/localization/gps_mode', '/imu']):
            if top == '/imu':
                t_in = max(t_in, m.header.stamp.to_sec())
                continue
            if top == '/localization/gps_mode':
                if not modes or modes[-1][1] != m.data:
                    modes.append((bt.to_sec(), m.data))
                continue
            t.append(m.header.stamp.to_sec())
            x.append(m.pose.pose.position.x)
            y.append(m.pose.pose.position.y)
            yaw.append(quat_yaw(m.pose.pose.orientation))
    yaw = np.unwrap(np.array(yaw))
    t = np.array(t)
    if t_in > 0.0:
        # 재현 백: 입력(IMU)이 끝난 뒤 EKF 가 외삽한 포즈는 버린다.
        keep = t <= t_in
        t, x, y, yaw = t[keep], np.array(x)[keep], np.array(y)[keep], yaw[keep]
    return t, np.array(x), np.array(y), yaw, modes


def tunnel_window(modes, before, after):
    """GPS 동결 구간(앞뒤 여유 포함). 여러 번이면 가장 긴 것."""
    spans, start = [], None
    for ts, md in modes:
        if md == 'frozen' and start is None:
            start = ts
        elif md == 'normal' and start is not None:
            spans.append((start, ts))
            start = None
    if start is not None:
        spans.append((start, modes[-1][0] + 1e9))
    if not spans:
        return None
    a, c = max(spans, key=lambda sp: sp[1] - sp[0])
    return a - before, c + after


def scans(heavy_bag, pose, window, topic='/velodyne_points'):
    t, x, y, yaw, _ = pose
    with rosbag.Bag(heavy_bag) as b:
        for _, m, bt in b.read_messages(topics=[topic]):
            ts = m.header.stamp.to_sec()
            if window and not (window[0] <= ts <= window[1]):
                continue
            if ts < t[0] or ts > t[-1]:
                continue
            ex = float(np.interp(ts, t, x))
            ey = float(np.interp(ts, t, y))
            eyaw = float(np.interp(ts, t, yaw))
            yield ts, (ex, ey, eyaw), pointcloud2_to_xyz_array(m)


def to_world(p, pose):
    ex, ey, eyaw = pose
    c, s = math.cos(eyaw), math.sin(eyaw)
    bx = p[:, 0] + SENSOR_X
    return ex + c * bx - s * p[:, 1], ey + s * bx + c * p[:, 1]


def cmd_build(args):
    theta = math.radians(args.heading_deg)
    ct, st = math.cos(theta), math.sin(theta)
    all_u, all_v, all_h = [], [], []
    origin = None
    for pair in args.pairs:
        heavy, pose_bag = pair.split(':')
        pose = load_poses(pose_bag, args.pose_topic)
        win = tunnel_window(pose[4], args.before, args.after)
        print('%s: window %s' % (os.path.basename(heavy), win))
        n = 0
        for ts, ep, pts in scans(heavy, pose, win):
            q = above_ground_points(pts, GROUND_Z, x_min=args.x_min, x_max=args.x_max,
                                    y_abs_max=args.y_max, h_min=-0.3, h_max=args.h_max)
            if not len(q):
                continue
            wx, wy = to_world(q, ep)
            if origin is None:
                origin = (ep[0], ep[1])
            dx, dy = wx - origin[0], wy - origin[1]
            all_u.append(dx * ct + dy * st)
            all_v.append(-dx * st + dy * ct)
            all_h.append(q[:, 2])
            n += 1
        print('  %d scans' % n)
    u = np.concatenate(all_u)
    v = np.concatenate(all_v)
    h = np.concatenate(all_h)
    res = args.res
    iu = np.floor(u / res).astype(np.int64)
    iv = np.floor(v / res).astype(np.int64)
    du = int(round(args.dil_u / res))
    dv = int(round(args.dil_v / res))
    pad = max(du, dv) + 2
    iu0, iv0 = iu.min() - pad, iv.min() - pad
    nu, nv = iu.max() - iu0 + pad + 1, iv.max() - iv0 + pad + 1
    a, c = iu - iu0, iv - iv0
    hmax = np.full((nu, nv), -1.0, dtype=np.float32)
    obj = h >= args.obj_min_h
    np.maximum.at(hmax, (a[obj], c[obj]), h[obj].astype(np.float32))
    seen = np.zeros((nu, nv), dtype=bool)
    seen[a, c] = True
    from scipy import ndimage
    hmax_d = ndimage.maximum_filter(hmax, size=(2 * du + 1, 2 * dv + 1),
                                    mode='constant', cval=-1.0)
    seen_d = ndimage.binary_dilation(seen, structure=np.ones((5, 5), dtype=bool))
    np.savez_compressed(args.out, origin=np.array(origin), theta=np.array(theta),
                        res=np.array(res), i0=np.array([iu0, iv0]),
                        hmax=hmax_d, seen=seen_d, hmax_raw=hmax,
                        dil=np.array([args.dil_u, args.dil_v]))
    print('saved %s grid %dx%d (%.0f x %.0f m), obj cells %d, seen %d' % (
        args.out, nu, nv, nu * res, nv * res, int(np.count_nonzero(hmax > 0)),
        int(np.count_nonzero(seen_d))))


def cluster(q_xy, cell, min_pts):
    cells = np.floor(q_xy / cell).astype(np.int64)
    keys = cells[:, 0] * 100003 + cells[:, 1]
    uniq, inv, counts = np.unique(keys, return_inverse=True, return_counts=True)
    out = []
    for k in np.nonzero(counts >= min_pts)[0]:
        cp = q_xy[inv == k]
        out.append((float(np.mean(cp[:, 0])), float(np.mean(cp[:, 1])), int(counts[k])))
    return out


def load_route(pose_bag):
    wp = {}
    with rosbag.Bag(pose_bag) as b:
        for _, m, _ in b.read_messages(topics=['/final_waypoint']):
            for w in m.waypoints:
                wp[w.waypoint_index] = (w.pose.pose.position.x, w.pose.pose.position.y)
    if len(wp) < 2:
        return None
    idx = np.array(sorted(wp), dtype=np.float64)
    xy = np.array([wp[int(i)] for i in idx])
    tan = np.gradient(xy, axis=0)
    tan /= np.maximum(np.linalg.norm(tan, axis=1), 1e-9)[:, None]
    return xy, idx, tan


def load_profile(path):
    import yaml
    a = np.asarray(yaml.safe_load(open(path))['tunnel_wall_profile'], dtype=float).reshape(-1, 3)
    return a[:, 0], a[:, 1], a[:, 2]


def lateral_ok(route, prof, wx, wy, margin, default=(-4.5, 8.0)):
    if route is None:
        return True
    xy, idx, tan = route
    d2 = (xy[:, 0] - wx) ** 2 + (xy[:, 1] - wy) ** 2
    j = int(np.argmin(d2))
    if d2[j] > 15.0 ** 2:
        return False
    lat = -tan[j, 1] * (wx - xy[j, 0]) + tan[j, 0] * (wy - xy[j, 1])
    r, l = default
    ik, rk, lk = prof
    k = idx[j]
    if ik[0] <= k <= ik[-1]:
        if np.interp(k, ik, (rk < 0).astype(float)) >= 0.999:
            r = float(np.interp(k, ik, rk))
        if np.interp(k, ik, (lk > 0).astype(float)) >= 0.999:
            l = float(np.interp(k, ik, lk))
    return r - margin <= lat <= l + margin


def cmd_eval(args):
    bg = TunnelBgMap(args.map)
    prof = load_profile(args.profile)
    for pair in args.pairs:
        heavy, pose_bag = pair.split(':')
        pose = load_poses(pose_bag, args.pose_topic)
        route = load_route(pose_bag)
        win = tunnel_window(pose[4], args.before, args.after)
        t_first = None
        print('== %s window %s' % (os.path.basename(heavy), win))
        for ts, ep, pts in scans(heavy, pose, win):
            if t_first is None:
                t_first = ts
            q = above_ground_points(pts, GROUND_Z, x_min=args.x0 - SENSOR_X,
                                    x_max=args.x1 - SENSOR_X, y_abs_max=args.y_max,
                                    h_min=args.min_h, h_max=args.det_max_h)
            if not len(q):
                continue
            wx, wy = to_world(q, ep)
            nov, judged = bg.novel(wx, wy, q[:, 2], args.margin)
            cov = bg.covers(ep[0], ep[1])
            objs = []
            for ox, oy, k in cluster(q[nov, :2], args.cell, args.min_pts):
                c, s = math.cos(ep[2]), math.sin(ep[2])
                bx = ox + SENSOR_X
                if lateral_ok(route, prof, ep[0] + c * bx - s * oy, ep[1] + s * bx + c * oy,
                              args.lat_margin):
                    objs.append((ox, oy, k))
            if objs or args.verbose:
                desc = ' '.join('(%.1f,%+.1f n%d)' % (ox + SENSOR_X, oy, k) for ox, oy, k in
                                sorted(objs))
                print('%6.2f cov=%d judged=%4d novel=%3d %s' % (
                    ts - t_first, cov, int(np.count_nonzero(judged)),
                    int(np.count_nonzero(nov)), desc))


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd')
    for name in ('build', 'eval'):
        p = sub.add_parser(name)
        p.add_argument('pairs', nargs='+', help='heavy.bag:pose.bag')
        p.add_argument('--pose-topic', default='/ekf_pose_with_covariance')
        p.add_argument('--before', type=float, default=1.0)
        p.add_argument('--after', type=float, default=3.0)
        p.add_argument('--y-max', type=float, default=12.0)
    b = sub.choices['build']
    b.add_argument('--out', required=True)
    b.add_argument('--res', type=float, default=0.2)
    b.add_argument('--dil-u', type=float, default=0.8, help='진행방향 팽창 [m]')
    b.add_argument('--dil-v', type=float, default=0.4, help='횡방향 팽창 [m]')
    b.add_argument('--heading-deg', type=float, default=TUNNEL_HEADING_DEG)
    b.add_argument('--x-min', type=float, default=-15.0)
    b.add_argument('--x-max', type=float, default=45.0)
    b.add_argument('--h-max', type=float, default=3.0)
    b.add_argument('--obj-min-h', type=float, default=0.15)
    e = sub.choices['eval']
    e.add_argument('--map', required=True)
    e.add_argument('--x0', type=float, default=3.0)
    e.add_argument('--x1', type=float, default=35.0)
    e.add_argument('--min-h', type=float, default=0.25)
    e.add_argument('--det-max-h', type=float, default=2.5)
    e.add_argument('--margin', type=float, default=0.3)
    e.add_argument('--cell', type=float, default=1.0)
    e.add_argument('--min-pts', type=int, default=4)
    e.add_argument('--verbose', action='store_true')
    e.add_argument('--lat-margin', type=float, default=0.6)
    e.add_argument('--profile', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'tunnel_wall_profile.yaml'))
    args = ap.parse_args()
    if args.cmd == 'build':
        cmd_build(args)
    elif args.cmd == 'eval':
        cmd_eval(args)
    else:
        ap.print_help()


if __name__ == '__main__':
    main()
