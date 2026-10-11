#!/usr/bin/env python3
"""YOLO-seg + 단안깊이 오브젝트 거리 추정 — 단일 파일 배포판 (MORAI CAMERA 90° FOV 학습).

의존성: torch, torchvision, opencv-python, numpy, ultralytics
동봉:   depth_unet.pt(깊이 모델, 960×1280 학습), yolov8n-seg.pt, scale.json

사용:
  python3 distance_estimator.py --input <이미지|디렉토리|글롭>          # CLI
  from distance_estimator import DistanceEstimator                     # 라이브러리
  est = DistanceEstimator()            # 같은 폴더의 가중치 자동 로드
  dets = est(bgr_image)                # [{cls, conf, box, dist}, ...]

주의:
- 깊이 모델은 MORAI 1280×960(HFOV 90°, fx=640) 카메라로 학습됨. 다른 해상도는
  폭 1280 리사이즈 + 상하 제로패딩으로 맞춤(1280×720 검증됨). FOV가 다르면 재학습 필요.
- TRT 엔진(.engine)은 빌드 머신 종속이라 미동봉 — 필요하면 대상 PC에서
  `yolo export model=yolov8n-seg.pt format=engine imgsz=1280` 로 재생성 후 --yolo 지정.
- 유효 거리 ~100 m (학습 GT 상한). 오브젝트 거리 MAE 1.9 m / 상대오차 9% (val 기준).
"""
import argparse
import glob
import os

import cv2
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision.models import resnet18

PKG_DIR = os.path.dirname(os.path.abspath(__file__))
IMNET_MEAN = np.array([0.485, 0.456, 0.406], np.float32)
IMNET_STD = np.array([0.229, 0.224, 0.225], np.float32)
COCO_KEEP = {0: "person", 1: "bicycle", 2: "car", 3: "motorcycle", 5: "bus", 7: "truck"}
TRAIN_W, TRAIN_H = 1280, 960


# ---------------- 깊이 모델 (ResNet18-UNet, log-depth) ----------------
class Up(nn.Module):
    def __init__(self, c_in, c_skip, c_out):
        super().__init__()
        self.conv = nn.Sequential(
            nn.Conv2d(c_in + c_skip, c_out, 3, padding=1), nn.BatchNorm2d(c_out), nn.ReLU(True),
            nn.Conv2d(c_out, c_out, 3, padding=1), nn.BatchNorm2d(c_out), nn.ReLU(True))

    def forward(self, x, skip):
        x = F.interpolate(x, size=skip.shape[-2:], mode="bilinear", align_corners=False)
        return self.conv(torch.cat([x, skip], 1))


class UNetDepth(nn.Module):
    def __init__(self):
        super().__init__()
        r = resnet18(weights=None)
        self.stem = nn.Sequential(r.conv1, r.bn1, r.relu)
        self.pool = r.maxpool
        self.l1, self.l2, self.l3, self.l4 = r.layer1, r.layer2, r.layer3, r.layer4
        self.up4 = Up(512, 256, 256)
        self.up3 = Up(256, 128, 128)
        self.up2 = Up(128, 64, 64)
        self.up1 = Up(64, 64, 64)
        self.head = nn.Sequential(nn.Conv2d(64, 32, 3, padding=1), nn.ReLU(True),
                                  nn.Conv2d(32, 1, 3, padding=1))

    def forward(self, x):
        s0 = self.stem(x)
        s1 = self.l1(self.pool(s0))
        s2 = self.l2(s1)
        s3 = self.l3(s2)
        s4 = self.l4(s3)
        d = self.up4(s4, s3)
        d = self.up3(d, s2)
        d = self.up2(d, s1)
        d = self.up1(d, s0)
        d = F.interpolate(d, scale_factor=2, mode="bilinear", align_corners=False)
        return self.head(d)  # log-depth


# ---------------- 거리 추출 ----------------
def roi_distance(depth, box):
    """bbox 하단 중앙(가로 안쪽 60%, 세로 55~95%) 중앙값 — 마스크 없을 때 폴백."""
    x1, y1, x2, y2 = [int(v) for v in box]
    w, h = x2 - x1, y2 - y1
    roi = depth[max(y1 + int(0.55 * h), 0): y1 + int(0.95 * h),
                max(x1 + int(0.2 * w), 0): x2 - int(0.2 * w)]
    if roi.size == 0:
        roi = depth[max(y1, 0):y2, max(x1, 0):x2]
    return float(np.median(roi)) if roi.size else float("nan")


def mask_distance(depth, polygon):
    """seg 폴리곤 내부 깊이 중앙값 — 배경 오염 최소."""
    m = np.zeros(depth.shape, np.uint8)
    cv2.fillPoly(m, [polygon.astype(np.int32)], 1)
    vals = depth[m.astype(bool)]
    return float(np.median(vals)) if vals.size >= 20 else float("nan")


class DistanceEstimator:
    def __init__(self, depth_ckpt=None, yolo_weights=None, conf=0.3, imgsz=1280,
                 device=None, hood_filter=True):
        self.dev = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.conf, self.imgsz, self.hood_filter = conf, imgsz, hood_filter
        ck = torch.load(depth_ckpt or os.path.join(PKG_DIR, "depth_unet.pt"),
                        map_location="cpu", weights_only=False)
        self.size = tuple(ck["meta"].get("size", [960, 1280]))
        self.depth_net = UNetDepth().to(self.dev)
        self.depth_net.load_state_dict(ck["model"])
        self.depth_net.eval()
        from ultralytics import YOLO
        # Default remains the original package model. Alternative weights
        # can be selected explicitly with --yolo or DistanceEstimator(...).
        w = yolo_weights or os.path.join(PKG_DIR, "yolov8n-seg.pt")
        # The fine-tuned best.pt/engine is a YOLOv8-seg model even though the
        # exported engine filename does not contain "seg".
        self.yolo = YOLO(w, task="segment")

    def _pad(self, img):
        """임의 해상도 → 폭 1280 + 상하 제로패딩 1280×960 (학습 calib 일치)."""
        h, w = img.shape[:2]
        if w != TRAIN_W:
            img = cv2.resize(img, (TRAIN_W, int(h * TRAIN_W / w)))
            h = img.shape[0]
        if h < TRAIN_H:
            top = (TRAIN_H - h) // 2
            img = cv2.copyMakeBorder(img, top, TRAIN_H - h - top, 0, 0,
                                     cv2.BORDER_CONSTANT, value=0)
            return img, top, top + h
        if h > TRAIN_H:
            top = (h - TRAIN_H) // 2
            return img[top: top + TRAIN_H], -top, TRAIN_H
        return img, 0, TRAIN_H

    @torch.no_grad()
    def predict_depth(self, rgb):
        """1280×960 RGB → 같은 해상도 깊이맵(m)."""
        img = cv2.resize(rgb, (self.size[1], self.size[0]),
                         interpolation=cv2.INTER_AREA).astype(np.float32)
        img = (img / 255.0 - IMNET_MEAN) / IMNET_STD
        t = torch.from_numpy(img.transpose(2, 0, 1))[None].to(self.dev)
        if self.dev == "cuda":
            with torch.autocast("cuda", torch.bfloat16):
                pred = torch.exp(self.depth_net(t).float())
        else:
            pred = torch.exp(self.depth_net(t))
        pred = F.interpolate(pred, size=rgb.shape[:2], mode="bilinear",
                             align_corners=False)
        return pred[0, 0].cpu().numpy()

    def __call__(self, bgr):
        """BGR 이미지 → [{cls, conf, box(x1,y1,x2,y2 패딩좌표), dist(m)}].

        box 좌표를 원본 좌표로 되돌리려면 y − pad_top (반환 dict 의 pad_top 참조).
        """
        padded, pad_top, content_bottom = self._pad(bgr)
        depth = self.predict_depth(cv2.cvtColor(padded, cv2.COLOR_BGR2RGB))
        r = self.yolo.predict(padded, conf=self.conf, imgsz=self.imgsz, verbose=False)[0]
        polys = r.masks.xy if r.masks is not None else [None] * len(r.boxes)
        dets = []
        for box, poly in zip(r.boxes, polys):
            c = int(box.cls.item())
            if c not in COCO_KEEP:
                continue
            xyxy = tuple(box.xyxy[0].tolist())
            dist = mask_distance(depth, poly) if poly is not None and len(poly) >= 3 \
                else roi_distance(depth, xyxy)
            if np.isnan(dist):
                dist = roi_distance(depth, xyxy)
            if self.hood_filter and xyxy[3] >= 0.95 * content_bottom and dist < 3.0:
                continue  # 자차 후드 오검출
            dets.append(dict(cls=COCO_KEEP[c], conf=round(float(box.conf), 2),
                             box=[round(v, 1) for v in xyxy], dist=round(float(dist), 2),
                             pad_top=pad_top))
        return dets, depth, padded


def run_ros(args):
    """Subscribe to sensor_msgs/CompressedImage and publish JSON detections."""
    import json
    import rospy
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String
    from morai_msgs.msg import EgoVehicleStatus
    from collections import defaultdict, deque
    import math

    rospy.init_node("yolo_distance_estimator", anonymous=False)
    detections_pub = rospy.Publisher(args.detections_topic, String, queue_size=1)
    ttc_pub = rospy.Publisher('/yolo_distance/ttc', String, queue_size=1)
    debug_pub = rospy.Publisher(args.debug_topic, CompressedImage, queue_size=1)
    combined_pub = (rospy.Publisher(args.combined_topic, CompressedImage, queue_size=1)
                    if args.combined_topic else None)
    estimator = DistanceEstimator(args.depth_ckpt, args.yolo, args.conf, args.imgsz)
    ego_speed = [0.0]
    distance_hist = defaultdict(lambda: deque(maxlen=8))
    # Keep the last valid TTC briefly through dropped YOLO frames.  The camera
    # stream can be bursty, so clearing TTC on every missed detection makes the
    # dashboard flash between a number and N/A.
    ttc_cache = {}

    def ego_callback(status):
        if math.isfinite(status.velocity.x):
            ego_speed[0] = max(0.0, float(status.velocity.x))

    rospy.Subscriber('/Competition_topic', EgoVehicleStatus, ego_callback, queue_size=1)
    latest_color = ["UNKNOWN"]

    def traffic_color_callback(msg):
        latest_color[0] = msg.data.strip().upper() or "UNKNOWN"

    def publish_combined(yolo_image, detections, header):
        combined = yolo_image.copy()
        for d in detections:
            x1, y1, x2, y2 = [int(v) for v in d["box"]]
            cv2.rectangle(combined, (x1, y1), (x2, y2), (0, 200, 255), 2)
            cv2.putText(combined, f'{d["cls"]} {d["dist"]:.1f}m',
                        (x1, max(y1 - 6, 14)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 200, 255), 2)
        cv2.putText(combined, "Traffic light: " + latest_color[0], (15, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
        ok, encoded = cv2.imencode(".jpg", combined, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if ok:
            out = CompressedImage()
            out.header = header
            out.format = "jpeg"
            out.data = encoded.tobytes()
            combined_pub.publish(out)

    def callback(msg):
        try:
            img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                rospy.logwarn_throttle(5.0, "Could not decode compressed image")
                return
            dets, _, debug = estimator(img)
            out = String()
            out.data = json.dumps({"header": {"stamp": msg.header.stamp.to_sec()},
                                   "detections": dets}, ensure_ascii=False)
            detections_pub.publish(out)
            ttc_objects = []
            stamp = msg.header.stamp.to_sec()
            for i, det in enumerate(dets):
                box = det.get('box', [])
                side = int((float(box[0]) + float(box[2])) * 0.5 / 160.0) if len(box) >= 4 else i
                key = '%s:%s' % (det.get('cls', 'object'), side)
                distance_hist[key].append((stamp, float(det['dist'])))
                hist = list(distance_hist[key]); closing = 0.0
                if len(hist) >= 2 and hist[-1][0] - hist[0][0] > 0.05:
                    closing = max(0.0, (hist[0][1] - hist[-1][1]) /
                                  (hist[-1][0] - hist[0][0]))
                ttc = float(det['dist']) / closing if closing > 0.15 else None
                if ttc is not None and math.isfinite(ttc):
                    ttc_cache[key] = (ttc, stamp)
                elif key in ttc_cache and stamp - ttc_cache[key][1] <= 1.0:
                    # Hold the last estimate for up to one second (not forever).
                    ttc = ttc_cache[key][0]
                ttc_objects.append({'id': key, 'cls': det.get('cls'),
                                    'distance': det['dist'],
                                    'closing_speed': round(closing, 2),
                                    'ttc': None if ttc is None else round(ttc, 2),
                                    'ego_speed': round(ego_speed[0], 2)})
            ttc_pub.publish(String(data=json.dumps({'stamp': stamp, 'objects': ttc_objects})))

            for d in dets:
                x1, y1, x2, y2 = [int(v) for v in d["box"]]
                cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 200, 255), 2)
                cv2.putText(debug, f'{d["cls"]} {d["dist"]:.1f}m',
                            (x1, max(y1 - 6, 14)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 200, 255), 2)
            # Keep the camera image intact and add a compact TTC dashboard on
            # the right.  This avoids putting long TTC text over the scene and
            # makes each tracked object easy to inspect in rqt_image_view/RViz.
            sidebar_w = 360
            sidebar = np.zeros((debug.shape[0], sidebar_w, 3), dtype=np.uint8)
            sidebar[:] = (28, 28, 28)
            cv2.putText(sidebar, "OBJECT TTC", (18, 38),
                        cv2.FONT_HERSHEY_DUPLEX, 0.9, (0, 230, 255), 2)
            cv2.line(sidebar, (15, 52), (sidebar_w - 15, 52), (90, 90, 90), 1)
            if not ttc_objects:
                cv2.putText(sidebar, "No objects", (18, 92),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 180, 180), 1)
            for row, obj in enumerate(ttc_objects):
                y = 88 + row * 58
                if y > sidebar.shape[0] - 20:
                    break
                ttc_val = obj.get('ttc')
                ttc_text = "N/A" if ttc_val is None else f"{float(ttc_val):.1f} s"
                # Red < 2 s, amber < 4 s, green otherwise; N/A is gray.
                if ttc_val is None:
                    color = (170, 170, 170)
                elif float(ttc_val) < 2.0:
                    color = (60, 60, 255)
                elif float(ttc_val) < 4.0:
                    color = (0, 190, 255)
                else:
                    color = (70, 220, 70)
                label = f"{obj.get('cls', 'object')}  {obj.get('distance', 0.0):.1f} m"
                cv2.putText(sidebar, label, (18, y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.62, (235, 235, 235), 1)
                cv2.putText(sidebar, f"TTC: {ttc_text}", (18, y + 27),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2)
            fused = np.hstack((debug, sidebar))
            ok, encoded = cv2.imencode(".jpg", fused, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                debug_msg = CompressedImage()
                debug_msg.header = msg.header
                debug_msg.format = "jpeg"
                debug_msg.data = encoded.tobytes()
                debug_pub.publish(debug_msg)
                if combined_pub is not None:
                    publish_combined(debug, dets, msg.header)
        except Exception as exc:
            rospy.logerr_throttle(5.0, "inference failed: %s", exc)

    rospy.Subscriber(args.ros_topic, CompressedImage, callback, queue_size=1,
                     buff_size=2**24)
    rospy.Subscriber("/traffic_light_color", String, traffic_color_callback,
                     queue_size=1)
    rospy.loginfo("YOLO distance estimator subscribed to %s", args.ros_topic)
    rospy.loginfo("detections: %s, debug: %s, combined: %s", args.detections_topic,
                  args.debug_topic, args.combined_topic)
    rospy.spin()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", default=None, help="이미지 파일/디렉토리/글롭 (ROS 모드에서는 생략)")
    ap.add_argument("--ros", action="store_true", help="ROS compressed-image subscriber mode")
    ap.add_argument("--ros-topic", default="/front/image_jpeg/compressed")
    ap.add_argument("--detections-topic", default="/yolo_distance/detections")
    ap.add_argument("--debug-topic", default="/yolo_distance/debug/compressed")
    ap.add_argument("--traffic-debug-topic", default="/traffic_light/debug/compressed")
    # The 2x2 compositor is the only publisher of /perception/debug/compressed.
    ap.add_argument("--combined-topic", default="")
    ap.add_argument("--depth-ckpt", default=None)
    ap.add_argument("--yolo", default=None)
    ap.add_argument("--conf", type=float, default=0.3)
    ap.add_argument("--imgsz", type=int, default=1280)
    ap.add_argument("--out", default="dist_out")
    args = ap.parse_args()

    if args.ros or args.input is None:
        run_ros(args)
        return

    if os.path.isdir(args.input):
        paths = sorted(glob.glob(os.path.join(args.input, "*.png")) +
                       glob.glob(os.path.join(args.input, "*.jpg")))
    else:
        paths = sorted(glob.glob(args.input)) or [args.input]

    est = DistanceEstimator(args.depth_ckpt, args.yolo, args.conf, args.imgsz)
    os.makedirs(args.out, exist_ok=True)
    for p in paths:
        img = cv2.imread(p, cv2.IMREAD_UNCHANGED)
        if img is None:
            continue
        if img.ndim == 3 and img.shape[-1] == 4:
            img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        dets, depth, padded = est(img)
        for d in dets:
            x1, y1, x2, y2 = [int(v) for v in d["box"]]
            cv2.rectangle(padded, (x1, y1), (x2, y2), (0, 200, 255), 2)
            cv2.putText(padded, f'{d["cls"]} {d["dist"]:.1f}m', (x1, max(y1 - 6, 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
            print(f'{os.path.basename(p)}  {d["cls"]:<10} conf={d["conf"]:.2f} '
                  f'dist={d["dist"]:.1f} m')
        cv2.imwrite(os.path.join(args.out,
                                 os.path.splitext(os.path.basename(p))[0] + "_dist.jpg"),
                    padded)
    print(f"\n결과: {args.out}/")


if __name__ == "__main__":
    main()
