#!/usr/bin/env python3
"""LiDAR -> 전방 카메라(cam0) 투영 검증.

왜 필요한가.
  고주로 원거리 차량 인지를 카메라로 보강하려면, 먼저 LiDAR 클러스터를
  이미지에 투영했을 때 실제 차량 위에 떨어지는지 확인해야 한다. 이게
  틀리면 이후 융합 로직이 전부 무의미하다.

확인 대상 (MORAI SensorInfo_2023_Hyundai_Ioniq5.json 실측값)
  cam0  : pos (1.900, 0.000, 1.200), pitch 2.0 deg, 1280x720, HFOV 90
  LiDAR : pos (1.100, 0.000, 1.330)
  -> 상대 오프셋 (+0.800, 0.000, -0.130), pitch 2.0 deg

두 가지 초점거리 가설을 같이 그린다.
  fx = 640  : HFOV 90 / 폭 1280 에서 기하학적으로 유도한 값
  fx = 320  : 센서 파일의 focalLengthpixel 필드에 적힌 값
어느 쪽이 맞는지는 눈으로 판별해야 한다.

사용법
  python3 verify_projection.py <heavy.bag> [--out DIR] [--n 8] [--pitch 2.0]
"""
import argparse
import os
import sys

import numpy as np

try:
    import rosbag
    import sensor_msgs.point_cloud2 as pc2
    import cv2
except ImportError as exc:  # pragma: no cover
    sys.exit('ROS 환경을 source 한 뒤 실행하세요: %s' % exc)


CAM_POS = np.array([1.900, 0.000, 1.200])
LIDAR_POS = np.array([1.100, 0.000, 1.330])  # 2026-09-17 실측 반영(1.300 -> 1.330)
IMG_W, IMG_H = 1280, 720
IMAGE_TOPIC = '/image_jpeg/compressed'
# 점군은 velodyne(센서) 프레임이라 라이다 원점 기준이다.
CLOUD_TOPICS = ('/nonground', '/velodyne_points')


def build_rotation(pitch_deg):
    """카메라 pitch 회전(차량 y축 기준). MORAI pitch 양수 = 아래를 향함."""
    p = np.radians(pitch_deg)
    cp, sp = np.cos(p), np.sin(p)
    # base_link(x 전방, y 좌, z 상) 에서 카메라를 pitch 만큼 숙인다.
    return np.array([
        [cp, 0.0, sp],
        [0.0, 1.0, 0.0],
        [-sp, 0.0, cp],
    ])


def project(points_lidar, pitch_deg, fx, fy=None):
    """points_lidar: (N,3) 라이다 원점 기준 -> (pixels, valid)."""
    fy = fx if fy is None else fy
    offset = CAM_POS - LIDAR_POS
    shifted = points_lidar - offset
    rotated = shifted @ build_rotation(pitch_deg).T

    x, y, z = rotated[:, 0], rotated[:, 1], rotated[:, 2]
    # base_link -> OpenCV 광학 좌표 (x 우, y 하, z 전방)
    x_opt, y_opt, z_opt = -y, -z, x

    valid = z_opt > 0.5
    denom = np.maximum(z_opt, 1e-6)
    u = fx * x_opt / denom + IMG_W / 2.0
    v = fy * y_opt / denom + IMG_H / 2.0
    valid &= (u >= 0) & (u < IMG_W) & (v >= 0) & (v < IMG_H)
    return np.stack([u, v], axis=1), valid


def read_cloud(msg):
    pts = np.array([p[:3] for p in pc2.read_points(
        msg, field_names=('x', 'y', 'z'), skip_nans=True)])
    return pts


def colorize(dist, dmax=60.0):
    t = np.clip(dist / dmax, 0.0, 1.0)
    return (int(255 * (1 - t)), int(80 + 120 * t), int(255 * t))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--out', default='/tmp/projection_check')
    ap.add_argument('--n', type=int, default=8, help='저장할 프레임 수')
    ap.add_argument('--pitch', type=float, default=2.0)
    ap.add_argument('--max-range', type=float, default=60.0)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    bag = rosbag.Bag(args.bag)

    topics = bag.get_type_and_topic_info().topics
    if IMAGE_TOPIC not in topics:
        sys.exit('%s 가 백에 없습니다. competition.launch 의 record_heavy 에 '
                 '이미지 토픽을 추가하고 다시 녹화하세요.' % IMAGE_TOPIC)
    cloud_topic = next((t for t in CLOUD_TOPICS if t in topics), None)
    if cloud_topic is None:
        sys.exit('점군 토픽이 없습니다: %s' % (CLOUD_TOPICS,))
    print('image=%s  cloud=%s' % (IMAGE_TOPIC, cloud_topic))

    clouds = [(t.to_sec(), m) for _, m, t in bag.read_messages(cloud_topic)]
    clouds.sort(key=lambda z: z[0])
    ctimes = np.array([c[0] for c in clouds])

    saved = 0
    for _, msg, t in bag.read_messages(IMAGE_TOPIC):
        if saved >= args.n:
            break
        ts = t.to_sec()
        idx = int(np.argmin(np.abs(ctimes - ts)))
        dt = abs(ctimes[idx] - ts)
        if dt > 0.10:
            continue

        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            continue
        if (img.shape[1], img.shape[0]) != (IMG_W, IMG_H):
            print('경고: 이미지 %dx%d 가 가정(%dx%d)과 다릅니다'
                  % (img.shape[1], img.shape[0], IMG_W, IMG_H))

        pts = read_cloud(clouds[idx][1])
        if pts.size == 0:
            continue
        rng = np.linalg.norm(pts[:, :2], axis=1)
        keep = (rng > 2.0) & (rng < args.max_range)
        pts, rng = pts[keep], rng[keep]

        for fx, tag, shift in ((640.0, 'fx=640', 0), (320.0, 'fx=320', 1)):
            canvas = img.copy()
            px, ok = project(pts, args.pitch, fx)
            for (u, v), d in zip(px[ok], rng[ok]):
                cv2.circle(canvas, (int(u), int(v)), 2, colorize(d, args.max_range), -1)
            cv2.putText(canvas, '%s  pitch=%.1f  dt=%.3fs  n=%d'
                        % (tag, args.pitch, dt, int(ok.sum())),
                        (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
            cv2.putText(canvas, '%s  pitch=%.1f  dt=%.3fs  n=%d'
                        % (tag, args.pitch, dt, int(ok.sum())),
                        (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            out = os.path.join(args.out, 'frame%02d_%s.jpg' % (saved, tag.replace('=', '')))
            cv2.imwrite(out, canvas)
            if shift == 1:
                print('saved %s (t=%.2f)' % (out, ts))
        saved += 1

    bag.close()
    if saved == 0:
        print('저장된 프레임이 없습니다. 이미지/점군 시간 동기를 확인하세요.')
    else:
        print('\n%d 프레임 저장: %s' % (saved, args.out))
        print('확인 요령: 점이 실제 차량/구조물 위에 얹혀야 한다.')
        print('  - 전체가 좌우로 퍼지면 fx 가 틀린 것 (640 vs 320 비교)')
        print('  - 전체가 위/아래로 밀리면 pitch 를 조정 (--pitch)')


if __name__ == '__main__':
    main()
