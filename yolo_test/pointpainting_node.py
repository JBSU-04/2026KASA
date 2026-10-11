#!/usr/bin/env python3
"""
PointPainting 방식 장애물 검증 노드.

세그멘테이션(YOLO-seg, dual model)으로 마스크를 얻고,
라이다 포인트를 카메라에 투영해서 그 마스크 안에 실제로 점이 찍히는지 확인.
마스크는 있는데 라이다 점이 하나도 안 들어오면 오탐(허공/그림자/포스터 등)일 가능성이 높고,
마스크 안에 라이다 점이 충분히 들어오면 실제 3D 물체(장애물)로 확정.

좌표계 흐름:
  라이다 포인트 (base_link 기준, x=전방 y=좌측 z=상방)
    -> base_link -> Camera 위치로 평행이동
    -> OpenCV 광학 좌표계로 축 변환 (x=우측, y=하방, z=전방)
    -> K 행렬로 픽셀 투영
    -> 세그멘테이션 마스크 폴리곤 내부인지 판정 (cv2.pointPolygonTest)
"""

import rospy
import numpy as np
import cv2

from sensor_msgs.msg import CompressedImage, PointCloud2
import sensor_msgs.point_cloud2 as pc2

from dual_model_detector import DualModelDetector


class PointPaintingNode:
    def __init__(self):
        rospy.init_node('pointpainting_node', anonymous=True)

        # ---- Camera intrinsic (640x480, HFOV=90deg) ----
        self.K = np.array([
            [320.0,   0.0, 320.0],
            [  0.0, 320.0, 240.0],
            [  0.0,   0.0,   1.0]
        ])
        self.img_w, self.img_h = 640, 480

        # ---- base_link 기준 카메라/라이다 장착 위치 (rotation은 identity 가정) ----
        camera_pos = np.array([1.9, 0.0, 1.2])
        lidar_pos = np.array([1.1, 0.0, 1.33])  # lidar_extrinsic.yaml (1.30 -> 1.33, 2026-09-17)
        self.t_lidar_to_camera = camera_pos - lidar_pos  # base_link 기준 오프셋

        # ---- 검출 모델 ----
        rospy.loginfo("Loading detection models...")
        self.detector = DualModelDetector(
            coco_weight='weights/yolo11s-seg.pt',
            custom_weight='weights/best.pt',
            coco_classes=[0, 1, 2, 3, 5, 7, 16],  # person, bicycle, car, motorcycle, bus, truck, dog
            iou_threshold=0.4,
            coco_conf=0.5,
            custom_conf=0.5,
            custom_exclude_names=['motorcycle'],
        )
        rospy.loginfo("Models loaded.")

        # ---- PointPainting 판정 파라미터 ----
        self.min_points_for_confirm = 5  # 마스크 안에 이 개수 이상 라이다 점이 있어야 "진짜 장애물"로 확정
        self.min_range = 1.5   # 이 거리(m) 이내 너무 가까운 점(차체 자체 반사 등)은 제외
        self.max_range = 15.0  # 이 거리(m) 이상 먼 점은 배경으로 간주하고 무시

        self.latest_lidar_points = None  # (N,3) base_link 기준 x,y,z

        rospy.Subscriber('/demo/nonground', PointCloud2, self.lidar_callback, queue_size=1)
        rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.image_callback,
                          queue_size=1, buff_size=2**24, tcp_nodelay=True)

        rospy.loginfo("PointPainting node started. Waiting for topics...")
        rospy.spin()

    def lidar_callback(self, msg):
        points = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append((p[0], p[1], p[2]))
        if points:
            arr = np.array(points, dtype=np.float32)
            # 관심 범위 필터: 전방(x>0)이고 max_range 이내인 점만 사용
            # (먼 건물/구조물이 화면을 가로지르는 줄무늬로 찍히는 것 방지)
            dist = np.linalg.norm(arr[:, :2], axis=1)  # x,y 평면 거리
            mask = (arr[:, 0] > 0) & (dist > self.min_range) & (dist < self.max_range)
            filtered = arr[mask]
            self.latest_lidar_points = filtered if len(filtered) > 0 else None
        else:
            self.latest_lidar_points = None

    def lidar_to_camera_optical(self, points_base):
        """points_base: (N,3) base_link 기준 좌표 -> (N,3) OpenCV 광학 좌표계"""
        shifted = points_base - self.t_lidar_to_camera  # 카메라 원점 기준으로 평행이동
        x, y, z = shifted[:, 0], shifted[:, 1], shifted[:, 2]
        x_opt = -y
        y_opt = -z
        z_opt = x
        return np.stack([x_opt, y_opt, z_opt], axis=1)

    def project_points(self, points_cam_opt):
        """(N,3) 카메라 광학좌표 -> (N,2) 픽셀 좌표. z<=0인 점은 뒤쪽이라 제외.
        반환: pixels (N,2), in_bounds (N,) bool
        """
        z = points_cam_opt[:, 2]
        valid = z > 0.05
        x = points_cam_opt[:, 0]
        y = points_cam_opt[:, 1]

        u = (self.K[0, 0] * x / np.maximum(z, 1e-6)) + self.K[0, 2]
        v = (self.K[1, 1] * y / np.maximum(z, 1e-6)) + self.K[1, 2]

        in_bounds = valid & (u >= 0) & (u < self.img_w) & (v >= 0) & (v < self.img_h)
        pixels = np.stack([u, v], axis=1)
        return pixels, in_bounds

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is None:
            return

        detections = self.detector.detect(img)
        out = img.copy()

        if self.latest_lidar_points is None or len(self.latest_lidar_points) == 0:
            cv2.imshow("PointPainting", out)
            cv2.waitKey(1)
            return

        cam_points = self.lidar_to_camera_optical(self.latest_lidar_points)
        pixels, in_bounds = self.project_points(cam_points)
        valid_pixels = pixels[in_bounds]  # (M,2)

        # ---- 디버그: 원본/변환 좌표 범위 확인 ----
        if not hasattr(self, '_dbg_last'):
            self._dbg_last = 0
        import time
        now = time.time()
        if now - self._dbg_last > 1.0:
            self._dbg_last = now
            raw = self.latest_lidar_points
            print("[RAW lidar] x range=(%.2f, %.2f) y range=(%.2f, %.2f) z range=(%.2f, %.2f) n=%d" % (
                raw[:, 0].min(), raw[:, 0].max(),
                raw[:, 1].min(), raw[:, 1].max(),
                raw[:, 2].min(), raw[:, 2].max(), len(raw)))
            print("[CAM optical] x range=(%.2f, %.2f) y range=(%.2f, %.2f) z range=(%.2f, %.2f)" % (
                cam_points[:, 0].min(), cam_points[:, 0].max(),
                cam_points[:, 1].min(), cam_points[:, 1].max(),
                cam_points[:, 2].min(), cam_points[:, 2].max()))
            print("[in_bounds] valid count=%d / %d" % (in_bounds.sum(), len(in_bounds)))
            if len(valid_pixels) > 0:
                print("[pixels sample]", valid_pixels[:5])

        confirmed_detections = []

        for det in detections:
            mask_poly = det['mask']

            # 마스크가 없으면(드묾) 박스를 폴리곤 대용으로 사용
            if mask_poly is None or len(mask_poly) < 3:
                x1, y1, x2, y2 = det['box']
                poly = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.float32)
            else:
                poly = mask_poly.astype(np.float32)

            # 이 폴리곤 안에 들어오는 라이다 점 개수 세기
            count_inside = 0
            for (u, v) in valid_pixels:
                result = cv2.pointPolygonTest(poly.reshape(-1, 1, 2), (float(u), float(v)), False)
                if result >= 0:
                    count_inside += 1

            is_confirmed = count_inside >= self.min_points_for_confirm

            det['lidar_point_count'] = count_inside
            det['confirmed'] = is_confirmed
            if is_confirmed:
                confirmed_detections.append(det)

            # 시각화: 확정되면 초록, 아니면 회색으로 구분
            color = (0, 255, 0) if is_confirmed else (120, 120, 120)
            pts = poly.astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(out, [pts], True, color, 2)

            x1, y1 = int(det['box'][0]), int(det['box'][1])
            label = "%s pts=%d %s" % (det['name'], count_inside,
                                       "CONFIRMED" if is_confirmed else "unconfirmed")
            cv2.putText(out, label, (x1, max(0, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # 라이다 점 전체를 이미지에 작은 점으로 표시 (디버깅용)
        for (u, v) in valid_pixels:
            cv2.circle(out, (int(u), int(v)), 1, (255, 0, 0), -1)

        cv2.imshow("PointPainting", out)
        cv2.waitKey(1)


if __name__ == '__main__':
    try:
        PointPaintingNode()
    except rospy.ROSInterruptException:
        pass
