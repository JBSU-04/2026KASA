#!/usr/bin/env python3
"""
카메라 세그멘테이션으로 "장애물이 화면에 보이나"만 판단해서 즉시 속도 감속
트리거를 걸어주는 노드. 라이다 투영/위치 계산은 전혀 안 함 - 순수하게
"감지 여부"만 obstacle_avoidance_node에 넘긴다.

라이다는 근접해야 점이 충분히 찍혀서 확정(confirmed)되는 반면, 카메라는
훨씬 멀리서부터 장애물 형태를 인식할 수 있어서, 이 신호로 먼저 감속을
걸어두면 라이다가 확정될 때쯤엔 이미 느려진 상태로 반응할 여유가 생긴다.

감속(속도 70%)이 걸린 뒤의 실제 회피/복귀는 지금까지처럼 라이다 전용
(obstacle_avoidance_node)이 그대로 담당하고, 이 노드의 카메라 인식 결과는
회피 경로 계산에는 전혀 쓰이지 않는다 - 오직 "감속 트리거" 용도.
"""

import os
import sys
import rospy
import numpy as np
import cv2
import rospkg

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

# catkin이 devel/lib/yolo_test/에 만드는 dual_model_detector.py는 실제 코드가 아니라
# 소스 파일을 exec()으로 실행만 하는 "relay" 스텁이라, import로 쓰면(from ... import
# DualModelDetector) 클래스가 안 보여서 ImportError가 남. 실제 소스가 있는
# 패키지 디렉토리를 sys.path 맨 앞에 직접 넣어서 그쪽이 우선 잡히게 한다.
sys.path.insert(0, rospkg.RosPack().get_path('yolo_test'))
from dual_model_detector import DualModelDetector


class CameraSpeedKickNode:
    def __init__(self):
        rospy.init_node('camera_speed_kick_node')

        # 기본값: 패키지 경로 기준 weights/ (rosrun/roslaunch의 실행 cwd와
        # 무관하게 항상 올바른 경로를 찾도록 rospkg로 해석)
        pkg_path = rospkg.RosPack().get_path('yolo_test')
        default_weight_dir = os.path.join(pkg_path, 'weights') + os.sep
        weight_dir = rospy.get_param('~weight_dir', default_weight_dir)
        coco_weight = weight_dir + rospy.get_param('~coco_weight', 'yolo11s-seg.pt')
        custom_weight = weight_dir + rospy.get_param('~custom_weight', 'best.pt')

        # person, bicycle, car, motorcycle, bus, truck, dog
        self.coco_conf = rospy.get_param('~coco_conf', 0.5)
        self.custom_conf = rospy.get_param('~custom_conf', 0.5)

        # 화면 중 이 영역(가로 비율, 0~1) 안에 걸린 검출만 "내 진행방향에 관련 있다"고
        # 봄. 화면 양 끝(다른 차선/인도)에서 잡히는 건 무시해서 불필요한 감속을 줄인다.
        self.roi_x_min_ratio = rospy.get_param('~roi_x_min_ratio', 0.20)
        self.roi_x_max_ratio = rospy.get_param('~roi_x_max_ratio', 0.80)
        # 화면 맨 아래 이 비율(세로) 밑은 아예 무시 - 차량 본네트가 항상 잡혀서
        # 오탐(계속 SLOWDOWN ACTIVE)을 유발하던 걸 원천 차단.
        self.roi_y_max_ratio = rospy.get_param('~roi_y_max_ratio', 0.75)

        # 한 프레임만 튀어도 감속이 걸렸다 풀렸다 하지 않도록, 마지막 감지 후 이
        # 시간(s) 동안은 "감지됨" 상태를 유지.
        self.hold_sec = rospy.get_param('~hold_sec', 0.5)

        # 두 segmentation 모델을 매 카메라 프레임마다 실행하면 신호등 YOLO와
        # GPU를 경쟁한다. 대회 통합 런치에서는 M2/M3/M8에서만, 지정한 주기로
        # 추론한다. 단독 런치는 기존처럼 게이팅 없이 사용할 수 있다.
        self.section_gating_enabled = rospy.get_param(
            '~section_gating_enabled', False)
        self.section_active = not self.section_gating_enabled
        self.max_fps = float(rospy.get_param('~max_fps', 8.0))
        self.last_infer_time = 0.0

        # 오탐(false positive)이 계속 걸려서 평소 순항속도가 안 나오는 문제를 눈으로
        # 확인하기 위한 디버그 창. 기본 켜짐 - 끄려면 launch에서 false로.
        self.debug_show = rospy.get_param('~debug_show', True)

        rospy.loginfo("[camera_speed_kick] loading detection models...")
        self.detector = DualModelDetector(
            coco_weight=coco_weight,
            custom_weight=custom_weight,
            coco_classes=[0, 1, 2, 3, 5, 7, 16],
            iou_threshold=0.4,
            coco_conf=self.coco_conf,
            custom_conf=self.custom_conf,
            custom_exclude_names=['motorcycle'],
        )
        rospy.loginfo("[camera_speed_kick] models loaded.")

        self.last_detect_time = None
        self.pub = rospy.Publisher('/camera_detection/obstacle_present', Bool, queue_size=1)

        if self.section_gating_enabled:
            try:
                from mission_supervisor.msg import SectionState
                rospy.Subscriber('/mission/active_section', SectionState,
                                 self.section_callback, queue_size=1)
            except Exception as exc:
                rospy.logwarn('[camera_speed_kick] section gating unavailable: %s', exc)
                self.section_active = True

        rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.image_callback,
                          queue_size=1, buff_size=2**24, tcp_nodelay=True)

        rospy.Timer(rospy.Duration(0.1), self.publish_state)

        rospy.loginfo("[camera_speed_kick] started.")
        rospy.spin()

    def section_callback(self, msg):
        executors = list(getattr(msg, 'executors', []))
        self.section_active = (
            getattr(msg, 'section_id', '') in ('M2', 'M3', 'M8') or
            'static_obstacle' in executors or
            'dynamic_obstacle' in executors)
        if not self.section_active:
            self.last_detect_time = None

    def image_callback(self, msg):
        if not self.section_active:
            return
        now_sec = rospy.Time.now().to_sec()
        if now_sec - self.last_infer_time < 1.0 / max(self.max_fps, 0.1):
            return
        self.last_infer_time = now_sec

        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is None:
            return

        h, w = img.shape[:2]
        x_min = w * self.roi_x_min_ratio
        x_max = w * self.roi_x_max_ratio

        # 본네트가 항상 화면 맨 아래에 찍혀서 계속 오탐을 유발하므로, 그 구간은
        # 아예 모델에 안 보이게 잘라낸다. 위쪽부터 자르는 거라 x,y 좌표 원점은
        # 그대로라 draw()/트리거 판정에 별도 보정이 필요 없다.
        crop_h = int(h * self.roi_y_max_ratio)
        img_for_detect = img[:crop_h, :]

        detections = self.detector.detect(img_for_detect)
        found = False
        triggering_dets = []
        for det in detections:
            x1, y1, x2, y2 = det['box']
            cx = 0.5 * (x1 + x2)
            if x_min <= cx <= x_max:
                found = True
                triggering_dets.append(det)

        if found:
            self.last_detect_time = rospy.Time.now()

        if self.debug_show:
            self._show_debug(img, detections, triggering_dets, x_min, x_max, found)

    def _show_debug(self, img, detections, triggering_dets, x_min, x_max, found):
        if not os.environ.get('DISPLAY'):
            return
        try:
            out = self.detector.draw(img, detections)
            h = out.shape[0]
            # ROI 경계선 (이 안에 걸린 검출만 트리거로 인정)
            cv2.line(out, (int(x_min), 0), (int(x_min), h), (0, 255, 255), 1)
            cv2.line(out, (int(x_max), 0), (int(x_max), h), (0, 255, 255), 1)
            # 본네트 제외 경계선 (이 아래는 아예 모델에 안 들어감)
            cutoff_y = int(h * self.roi_y_max_ratio)
            cv2.line(out, (0, cutoff_y), (out.shape[1], cutoff_y), (255, 0, 255), 1)
            # 실제로 트리거를 유발한 박스는 빨간 테두리로 강조
            for det in triggering_dets:
                x1, y1, x2, y2 = [int(v) for v in det['box']]
                cv2.rectangle(out, (x1, y1), (x2, y2), (0, 0, 255), 3)
                cv2.putText(out, f"TRIGGER:{det['name']}:{det['conf']:.2f}", (x1, max(0, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            status = "SLOWDOWN ACTIVE" if found else "clear"
            color = (0, 0, 255) if found else (0, 200, 0)
            cv2.putText(out, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
            cv2.imshow("camera_speed_kick debug", out)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"[camera_speed_kick] debug_show exception: {e}")

    def publish_state(self, _event):
        now = rospy.Time.now()
        present = (self.last_detect_time is not None and
                   (now - self.last_detect_time).to_sec() < self.hold_sec)
        self.pub.publish(Bool(data=present))


if __name__ == '__main__':
    try:
        CameraSpeedKickNode()
    except rospy.ROSInterruptException:
        pass
