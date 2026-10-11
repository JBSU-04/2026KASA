#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO 기반 신호등 색 판별 노드.

- 카메라 이미지에서 YOLO로 신호등 검출 후 bbox 내부 HSV 색 분석으로
  red / yellow / green 판별.
- 모델 클래스명이 색을 직접 담고 있으면(red_light 등) 그 클래스를 우선 사용,
  아니면 'traffic'/'light' 류 클래스 bbox에 HSV 판별 적용.
- /localization/stop_ahead 의 전방 신호등 거리로 게이팅: 신호등이 전방에
  없으면 추론을 쉬어 GPU 절약 (~always_infer:=true 로 해제 가능).

구독:
  ~image_topic (기본 /image_jpeg/compressed)
  /localization/stop_ahead (Float32MultiArray, dist<0 이면 전방 신호등 없음)
발행:
  /traffic_light_color (std_msgs/String: "red"|"yellow"|"green"|"none")
  /traffic_light/debug (sensor_msgs/Image) 오버레이 (~publish_debug:=true)
  /traffic_light/debug/compressed (sensor_msgs/CompressedImage) JPEG 오버레이
"""
import os
import sys

_ros1_py_deps = "/home/kuuve/catkin_ws/.ros1_py_deps/root/usr/lib/python3/dist-packages"
if os.path.isdir(_ros1_py_deps) and _ros1_py_deps not in sys.path:
    sys.path.insert(0, _ros1_py_deps)
# 이 노드는 ROS1 rospy를 쓰므로 ROS2 Humble 메시지 경로가 먼저 잡히면 깨진다.
sys.path[:] = [p for p in sys.path
               if "/opt/ros/humble" not in p and "/ros2_ws/install" not in p]

# ROS Noetic의 /usr/local OpenCV GUI 패키지와 사용자 headless OpenCV가
# 충돌하는 환경이 있어, 이 노드는 정상 동작하는 사용자 cv2를 우선 로드한다.
_user_site = os.path.expanduser("~/.local/lib/python3.8/site-packages")
if os.path.isdir(_user_site) and _user_site in sys.path:
    sys.path.remove(_user_site)
    sys.path.insert(0, _user_site)

import numpy as np
import cv2
import rospy
import torch
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import CompressedImage, Image

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


def classify_color_hsv(bgr):
    """신호등 bbox 크롭에서 점등 색 판별. 반환 red/yellow/green/none."""
    if bgr.size == 0:
        return "none"
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    lit = (s > 90) & (v > 110)   # 점등된(채도·명도 높은) 픽셀만
    red = int(np.count_nonzero(lit & ((h <= 10) | (h >= 170))))
    yellow = int(np.count_nonzero(lit & (h >= 15) & (h <= 35)))
    green = int(np.count_nonzero(lit & (h >= 40) & (h <= 95)))
    best = max(("red", red), ("yellow", yellow), ("green", green),
               key=lambda kv: kv[1])
    # 최소 픽셀 수: bbox 크기에 비례 (너무 작은 잡광 무시)
    min_px = max(6, bgr.shape[0] * bgr.shape[1] // 200)
    return best[0] if best[1] >= min_px else "none"


class TrafficLightDetector(object):
    def __init__(self):
        rospy.init_node("traffic_light_detector", anonymous=False)

        model_path = rospy.get_param(
            "~model_path",
            os.path.expanduser("~/catkin_ws/yolo_model/yolov8n.pt"))
        self.image_topic = rospy.get_param("~image_topic",
                                           "/image_jpeg/compressed")
        # 상향 보조 카메라(pitch 325 = -35deg, nose up). 빈 문자열이면 단일
        # 카메라로 동작한다 - 기존 구성과 완전히 같다.
        self.image_topic2 = rospy.get_param("~image_topic2", "")
        # 이 거리(m) 안쪽에서는 보조(상향) 카메라를 주로 믿고, 그보다 멀면
        # Front 가 주다.
        #   Front 1280x720 FOV90 -> 수직 half 29.4deg
        #   상향  640x480  FOV90 -> 수직 half 36.9deg (4:3 이라 더 세로로 넓다)
        #   장착 (x-0.20, z1.25) / pitch 325(=-35deg nose up).
        # 4m 높이 신호등 기준 상향은 -1.9~71.9deg 를 덮어 사실상 전 거리에서
        # 신호등을 담는다. Front 는 정지선 3.75m 앞부터 화각 위로 놓친다.
        # 그래도 원거리는 Front 가 주다 - 해상도가 2배라 램프 픽셀수가 2배고
        # (20m 에서 12px vs 6px), 상향 640x480 은 멀수록 색 판정이 불안하다.
        self.near_switch_dist = float(rospy.get_param("~near_switch_dist", 12.0))
        self.conf_thres = rospy.get_param("~conf", 0.35)
        self.always_infer = rospy.get_param("~always_infer", False)
        self.publish_debug = rospy.get_param("~publish_debug", True)
        self.debug_topic = rospy.get_param("~debug_topic",
                                           "/traffic_light/debug")
        self.debug_compressed_topic = rospy.get_param(
            "~debug_compressed_topic", "/traffic_light/debug/compressed")
        self.debug_jpeg_quality = int(rospy.get_param(
            "~debug_jpeg_quality", 85))
        self.max_fps = rospy.get_param("~max_fps", 10.0)
        # 데이터셋은 지도상 전방 신호등이 가까울 때만 저주기로 저장한다.
        self.collect_dataset = rospy.get_param("~collect_dataset", True)
        self.collect_max_dist = float(rospy.get_param("~collect_max_dist", 60.0))
        self.collect_fps = float(rospy.get_param("~collect_fps", 2.0))
        self.dataset_dir = os.path.expanduser(rospy.get_param(
            "~dataset_dir", "~/Downloads/kcity_traffic_light"))
        self.last_save_t = 0.0
        if self.collect_dataset:
            os.makedirs(self.dataset_dir, exist_ok=True)

        # 적색/황색은 즉시 반영하되 녹색 해제는 연속 검출로 확인한다.
        self.stable_color = "none"
        self.green_count = 0
        self.green_confirm_frames = int(rospy.get_param(
            "~green_confirm_frames", 5))
        # 검출이 끊겼을 때 직전 색을 몇 프레임까지 붙들지. 이 값을 넘기면
        # none 으로 푼다.
        #   실측 2026-09-14(12:15 백): 정지선 앞에 서자 신호등이 Front 화각을
        #   벗어나 검출이 끊겼는데, stabilize_color 가 none 을 받고도
        #   stable_color 를 유지해 red 를 462회(54초) 계속 내보냈다. 그 red 가
        #   map_localizer 의 stop_hold 까지 물려 영구 정지가 됐다.
        #   푼다고 위험해지지 않는다 - none 이면 localizer 가 '미상'으로
        #   보고 stop_on_unknown(기본 true)이 정지를 유지한다.
        self.none_release_frames = int(rospy.get_param(
            "~none_release_frames", 10))
        self.none_count = 0
        self.color_decision_mode = rospy.get_param(
            "~color_decision_mode", "confidence")
        priority_ids = rospy.get_param(
            "~green_priority_tl_ids", "C1256W000052")
        if isinstance(priority_ids, str):
            self.green_priority_tl_ids = {
                x.strip() for x in priority_ids.split(",") if x.strip()
            }
        else:
            self.green_priority_tl_ids = {
                str(x).strip() for x in priority_ids if str(x).strip()
            }
        self.priority_green_confirm_frames = int(rospy.get_param(
            "~priority_green_confirm_frames", 2))
        self.current_tl_id = ""
        requested_device = rospy.get_param("~device", "auto")
        self.device = ("cuda:0" if torch.cuda.is_available() else "cpu") \
            if requested_device == "auto" else requested_device
        if self.device.startswith("cuda") and not torch.cuda.is_available():
            rospy.logwarn("[tl_detector] CUDA unavailable, fallback to cpu")
            self.device = "cpu"
        self.imgsz = rospy.get_param("~imgsz", 640)
        self.use_half = rospy.get_param("~use_half", True) and self.device != "cpu"
        # 신호등으로 취급할 클래스명 부분문자열 (소문자 비교)
        self.tl_keywords = rospy.get_param(
            "~tl_keywords", "traffic,light,signal,red,green,yellow,left_arrow,arrow").split(",")

        if YOLO is None:
            rospy.logfatal("[tl_detector] ultralytics 미설치")
            raise SystemExit(1)
        if not os.path.isfile(model_path):
            # 경로가 틀렸을 때 조용히 죽지 말고, 같은 파일명을 워크스페이스에서
            # 찾아본다. 호스트/컨테이너 마운트 차이로 절대경로가 어긋나면
            # (실측 2026-07-26) 노드가 respawn 무한루프에 빠져 신호색이 영구
            # 미상 = 영구 정지가 됐다.
            found = None
            base = os.path.basename(model_path)
            for root in ("/home/kuuve/catkin_ws/src",
                         "/home/kuuve/catkin_ws/catkin_ws/src"):
                cand = os.path.join(root, "yolo_distance_pkg", "models", base)
                if os.path.isfile(cand):
                    found = cand
                    break
            if found is None:
                rospy.logfatal("[tl_detector] model not found: %s "
                               "(대체 경로 탐색도 실패)", model_path)
                raise SystemExit(1)
            rospy.logwarn("[tl_detector] model_path 불일치 — 대체 경로 사용: "
                          "%s -> %s", model_path, found)
            model_path = found
        if self.device != "cpu":
            torch.backends.cudnn.benchmark = True
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True
        pt_model_path = model_path
        engine = os.path.splitext(model_path)[0] + ".engine"
        if model_path.endswith(".pt") and os.path.exists(engine):
            rospy.loginfo("[tl_detector] TensorRT engine found: %s", engine)
            try:
                self.model = YOLO(engine, task="detect")
                _ = self.model.names
                model_path = engine
            except Exception as exc:
                rospy.logwarn("[tl_detector] TensorRT engine load failed: %s", exc)
                rospy.logwarn("[tl_detector] fallback to PyTorch model: %s",
                              pt_model_path)
                self.model = YOLO(pt_model_path, task="detect")
                model_path = pt_model_path
        else:
            self.model = YOLO(model_path, task="detect")
        if model_path.endswith(".pt"):
            self.model.to(self.device)
        self.names = {int(k): str(v).lower()
                      for k, v in (self.model.names or {}).items()}
        self.tl_class_ids = [i for i, n in self.names.items()
                             if any(k in n for k in self.tl_keywords)]
        rospy.loginfo("[tl_detector] model=%s device=%s half=%s tl_classes=%s",
                      model_path, self.device, self.use_half,
                      {i: self.names[i] for i in self.tl_class_ids})
        if not self.tl_class_ids:
            rospy.logwarn("[tl_detector] 신호등 클래스가 모델에 없음 — "
                          "모든 검출 bbox에 HSV 판별 적용")

        self.bridge = CvBridge() if CvBridge is not None else None
        self.tl_dist = -1.0
        self.last_infer_t = {}          # 카메라별 마지막 추론 시각
        self.cam_result = {}            # 카메라별 최근 (색, 신뢰도, 시각)
        self.cam_result_ttl = float(rospy.get_param("~cam_result_ttl", 0.5))
        self.frame_count = 0

        self.pub_color = rospy.Publisher("/traffic_light_color", String,
                                         queue_size=1)
        if self.publish_debug:
            self.pub_dbg = rospy.Publisher(self.debug_topic, Image,
                                           queue_size=1)
            self.pub_dbg_compressed = rospy.Publisher(
                self.debug_compressed_topic, CompressedImage, queue_size=1)

        rospy.Subscriber("/localization/stop_ahead", Float32MultiArray,
                         self.stop_cb, queue_size=1)
        self._subscribe_image(self.image_topic, "front")
        if self.image_topic2:
            self._subscribe_image(self.image_topic2, "up")
        rospy.Subscriber("/localization/stop_tl", String,
                         self.stop_tl_cb, queue_size=1)
        rospy.loginfo("[tl_detector] ready (image=%s, image2=%s, debug=%s, "
                      "compressed=%s)",
                      self.image_topic, self.image_topic2 or "-",
                      self.debug_topic, self.debug_compressed_topic)

    def _subscribe_image(self, topic, cam):
        """cam 은 'front'(정면) 또는 'up'(상향 보조)."""
        if "compressed" in topic:
            rospy.Subscriber(topic, CompressedImage,
                             lambda m, c=cam: self.img_cb(m, c),
                             queue_size=1, buff_size=2 ** 22)
        else:
            rospy.Subscriber(topic, Image,
                             lambda m, c=cam: self.img_cb(m, c),
                             queue_size=1, buff_size=2 ** 24)

    def stop_cb(self, msg):
        self.tl_dist = msg.data[0] if len(msg.data) > 0 else -1.0
        if self.tl_dist < 0.0:
            # Do not carry a red/yellow decision into a road section with no
            # mapped traffic light, or into the next intersection.
            self.stable_color = "none"
            self.green_count = 0
            self.pub_color.publish(String(data="none"))

    def stop_tl_cb(self, msg):
        self.current_tl_id = str(msg.data)

    def green_has_priority(self):
        return self.current_tl_id in self.green_priority_tl_ids

    def stabilize_color(self, detected):
        if detected in ("red", "yellow"):
            self.stable_color = detected
            self.green_count = 0
            self.none_count = 0
        elif detected == "green":
            self.green_count += 1
            self.none_count = 0
            required = (self.priority_green_confirm_frames
                        if self.green_has_priority()
                        else self.green_confirm_frames)
            if self.green_count >= required:
                self.stable_color = "green"
        else:
            # 검출 없음. 짧은 끊김은 직전 색을 유지하되(깜빡임 흡수),
            # 계속 안 보이면 none 으로 푼다 - 안 그러면 화각을 벗어난 뒤
            # 직전 red 가 영구히 남아 녹색으로 바뀌어도 못 움직인다.
            self.green_count = 0
            self.none_count += 1
            if self.none_count >= self.none_release_frames:
                self.stable_color = "none"
        return self.stable_color

    def fuse_cameras(self, now):
        """두 카메라의 최근 판정을 하나로 합친다.

        거리로 주 카메라를 고른다 - 근거리에서는 Front 가 신호등을 화각
        위로 놓치고(4m 높이 기준 8m 안쪽), 원거리에서는 상향 카메라가
        신호등을 화각 아래로 놓친다. 주 카메라가 색을 내면 그걸 쓰고,
        못 내면 다른 쪽으로 폴백한다.

        둘 다 색을 냈는데 서로 다르면 red/yellow 를 택한다 - 정지 쪽이
        안전하고, 한쪽이 지나가는 차나 반사를 green 으로 본 경우를 막는다.
        """
        fresh = {cam: v for cam, v in self.cam_result.items()
                 if now - v[2] <= self.cam_result_ttl}
        if not fresh:
            return "none"
        near = 0.0 <= self.tl_dist <= self.near_switch_dist
        primary, secondary = ("up", "front") if near else ("front", "up")

        pri = fresh.get(primary)
        sec = fresh.get(secondary)
        pri_c = pri[0] if pri and pri[0] != "none" else None
        sec_c = sec[0] if sec and sec[0] != "none" else None

        if self.green_has_priority() and "green" in (pri_c, sec_c):
            color = "green"
        elif pri_c and sec_c and pri_c != sec_c:
            color = pri_c if pri_c in ("red", "yellow") else sec_c
        else:
            color = pri_c or sec_c or "none"
        rospy.loginfo_throttle(
            0.5, "[tl_detector] fuse d=%.1f primary=%s front=%s up=%s -> %s "
            "(stable=%s)", self.tl_dist, primary,
            fresh.get("front", ("-", 0, 0))[0], fresh.get("up", ("-", 0, 0))[0],
            color, self.stable_color)
        return color

    def save_dataset_image(self, img, now, detected, stable):
        if (not self.collect_dataset or self.tl_dist < 0.0 or
                self.tl_dist > self.collect_max_dist or
                now - self.last_save_t < 1.0 / max(self.collect_fps, 0.1)):
            return
        self.last_save_t = now
        stamp_ms = int(now * 1000)
        name = "%d_d%05.1f_raw-%s_stable-%s.jpg" % (
            stamp_ms, self.tl_dist, detected, stable)
        path = os.path.join(self.dataset_dir, name)
        if not cv2.imwrite(path, img):
            rospy.logwarn_throttle(5.0, "[tl_detector] dataset save failed: %s", path)

    def show_debug_image(self, img):
        if self.publish_debug:
            stamp = rospy.Time.now()
            if self.bridge is not None:
                raw_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
                raw_msg.header.stamp = stamp
                raw_msg.header.frame_id = "camera"
                self.pub_dbg.publish(raw_msg)
            ok, encoded = cv2.imencode(
                ".jpg", img,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.debug_jpeg_quality])
            if ok:
                compressed_msg = CompressedImage()
                compressed_msg.header.stamp = stamp
                compressed_msg.header.frame_id = "camera"
                compressed_msg.format = "jpeg"
                compressed_msg.data = encoded.tobytes()
                self.pub_dbg_compressed.publish(compressed_msg)
    def img_cb(self, msg, cam="front"):
        now = rospy.Time.now().to_sec()
        # 프레임 제한은 카메라별로 건다. 하나로 묶으면 먼저 도착한 쪽이
        # 매번 창을 차지해 다른 쪽이 굶는다.
        if now - self.last_infer_t.get(cam, 0.0) < 1.0 / max(self.max_fps, 0.1):
            return
        self.last_infer_t[cam] = now

        # 지도상 전방 신호등이 없을 때는 JPEG decode/복사/디버그 재인코딩도
        # 생략한다. 색 상태 해제는 stop_cb에서 즉시 발행한다.
        if not self.always_infer and self.tl_dist < 0.0:
            self.pub_color.publish(String(data="none"))
            return

        if isinstance(msg, CompressedImage):
            img = cv2.imdecode(np.frombuffer(msg.data, np.uint8),
                               cv2.IMREAD_COLOR)
        else:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if img is None:
            rospy.logwarn_throttle(5.0, "[tl_detector] JPEG decode failed")
            return
        self.frame_count += 1
        dataset_img = img.copy() if self.collect_dataset else None

        try:
            res = self.model.predict(
                source=img, conf=self.conf_thres, imgsz=self.imgsz,
                device=self.device, half=self.use_half, verbose=False)[0]
        except Exception as exc:
            rospy.logerr_throttle(2.0, "[tl_detector] YOLO inference failed: %s", exc)
            cv2.putText(img, "YOLO ERROR: %s" % exc, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
            self.show_debug_image(img)
            return
        color = "none"
        best_conf = 0.0
        detected_colors = []
        boxes = res.boxes
        if boxes is not None:
            for b in boxes:
                cls = int(b.cls[0])
                conf = float(b.conf[0])
                if self.tl_class_ids and cls not in self.tl_class_ids:
                    continue
                name = self.names.get(cls, "")
                x1, y1, x2, y2 = [int(v) for v in b.xyxy[0]]
                # 클래스명에 색이 있으면 그대로, 없으면 HSV 판별
                if "red" in name:
                    c = "red"
                elif "green" in name:
                    c = "green"
                elif "yellow" in name:
                    c = "yellow"
                elif "arrow" in name:
                    c = classify_color_hsv(img[max(y1, 0):y2, max(x1, 0):x2])
                else:
                    c = classify_color_hsv(img[max(y1, 0):y2, max(x1, 0):x2])
                if c != "none":
                    detected_colors.append((c, conf))
                if self.publish_debug:
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 255), 2)
                    cv2.putText(img, "%s %.2f" % (c, conf), (x1, y1 - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        if detected_colors:
            # 마지막 좌회전 신호(C1256W000052)만 red+좌회전
            # green 동시 점등을 통과로 판정한다. 다른 교차로의
            # red 우선 정책은 그대로 유지한다.
            if self.green_has_priority() and any(
                    c == "green" for c, _ in detected_colors):
                color = max((item for item in detected_colors
                             if item[0] == "green"),
                            key=lambda item: item[1])[0]
            elif self.color_decision_mode == "priority":
                # 여러 램프가 동시에 보이는 실제 교차로용 보수 모드.
                for priority in ("red", "yellow", "green"):
                    candidates = [(c, cf) for c, cf in detected_colors
                                  if c == priority]
                    if candidates:
                        color = max(candidates, key=lambda x: x[1])[0]
                        break
            else:
                # MORAI 단일 신호등에서는 low-conf yellow/red 오검출 하나가
                # green 출발을 영구적으로 막지 않게 가장 확신도 높은 색을 사용.
                color = max(detected_colors, key=lambda x: x[1])[0]

        # 이 카메라의 결과를 기록하고, 두 카메라를 합쳐 최종 색을 정한다.
        best_conf = (max(cf for c, cf in detected_colors)
                     if detected_colors else 0.0)
        self.cam_result[cam] = (color, best_conf, now)
        detected_color = self.fuse_cameras(now)
        color = self.stabilize_color(detected_color)
        if dataset_img is not None:
            self.save_dataset_image(dataset_img, now, detected_color, color)
        self.pub_color.publish(String(data=color))
        if self.publish_debug:
            cv2.putText(img, "TL: %s raw:%s (d=%.1fm)" %
                        (color, detected_color, self.tl_dist),
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                        (0, 0, 255) if color == "red" else (0, 200, 0), 2)
            # 두 카메라가 같은 디버그 토픽으로 나가므로 어느 쪽 프레임인지
            # 반드시 표시한다.
            cv2.putText(img, "cam:%s frame:%d ros:%.2f" %
                        (cam, self.frame_count, now),
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (255, 255, 0), 2)
        self.show_debug_image(img)


if __name__ == "__main__":
    try:
        TrafficLightDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
