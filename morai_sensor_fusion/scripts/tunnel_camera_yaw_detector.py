#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tunnel Camera Vanishing Point Yaw Detector

Uses perspective geometry of tunnel structural lines (lane markings,
ceiling arch ribs, wall base lines) to estimate the vanishing point (VP)
and calculate the vehicle's heading relative to the tunnel centerline.

Projective Geometry Principles:
- The horizontal position of the vanishing point (u_vp) in the camera image
  is strictly a function of vehicle orientation (yaw) and camera parameters (c_x, f_x).
- u_vp is completely invariant to lateral shift (X, Y) or lane changes.
- Yaw error relative to tunnel axis:
    delta_psi_cam = arctan2(u_vp - c_x, f_x)
- Global vehicle heading in map frame:
    psi_global = normalize_angle(tunnel_nominal_heading + delta_psi_cam)
"""

import math
import threading
import time
import cv2
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Float64, String


def normalize_angle(angle):
    """Normalize an angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class TunnelCameraYawDetector:
    def __init__(self):
        rospy.init_node('tunnel_camera_yaw_detector', anonymous=False)

        # Node parameters
        self.enabled = rospy.get_param('~enabled', True)
        self.image_topic = rospy.get_param('~image_topic', '/image_jpeg/compressed')
        self.gps_mode_topic = rospy.get_param('~gps_mode_topic', '/localization/gps_mode')
        self.camera_hfov_deg = float(rospy.get_param('~camera_hfov_deg', 90.0))
        self.tunnel_nominal_heading_deg = float(rospy.get_param('~tunnel_nominal_heading_deg', -151.84))
        self.tunnel_nominal_heading = math.radians(self.tunnel_nominal_heading_deg)

        # ROI configuration (fractions of image height and width)
        self.roi_top_ratio = float(rospy.get_param('~roi_top_ratio', 0.12))
        self.roi_bottom_ratio = float(rospy.get_param('~roi_bottom_ratio', 0.72))
        self.roi_left_ratio = float(rospy.get_param('~roi_left_ratio', 0.05))
        self.roi_right_ratio = float(rospy.get_param('~roi_right_ratio', 0.95))

        # Processing parameters
        self.downsample_factor = float(rospy.get_param('~downsample_factor', 1.0))
        self.min_line_length = float(rospy.get_param('~min_line_length', 25.0))
        self.min_line_angle_deg = float(rospy.get_param('~min_line_angle_deg', 15.0))
        self.max_line_angle_deg = float(rospy.get_param('~max_line_angle_deg', 85.0))
        self.min_line_count = int(rospy.get_param('~min_line_count', 6))
        self.min_intersections = int(rospy.get_param('~min_intersections', 8))
        self.ransac_distance_px = float(rospy.get_param('~ransac_distance_px', 80.0))
        self.ema_alpha = float(rospy.get_param('~ema_alpha', 0.25))
        self.max_delta_yaw_deg = float(rospy.get_param('~max_delta_yaw_deg', 20.0))
        self.publish_debug_image = rospy.get_param('~publish_debug_image', True)
        self.only_when_gps_frozen = rospy.get_param('~only_when_gps_frozen', True)

        # Geographic gate: straight tunnel boundaries in UTM Zone 52 X/Y coordinates
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')
        self.tunnel_min_x = float(rospy.get_param('~tunnel_min_x', 302530.0))
        self.tunnel_max_x = float(rospy.get_param('~tunnel_max_x', 302635.0))
        self.tunnel_min_y = float(rospy.get_param('~tunnel_min_y', 4123550.0))
        self.tunnel_max_y = float(rospy.get_param('~tunnel_max_y', 4123750.0))
        self.max_delta_jump_deg = float(rospy.get_param('~max_delta_jump_deg', 3.0))

        # Internal state
        self.lock = threading.Lock()
        self.gps_mode = 'normal'
        self.vehicle_x = None
        self.vehicle_y = None
        self.smooth_delta_psi = None
        self.last_valid_time = None
        self.valid_timeout = float(rospy.get_param('~valid_timeout', 0.5))

        # Line Segment Detector (OpenCV)
        self.lsd = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD)

        # Publishers
        self.yaw_error_pub = rospy.Publisher('/tunnel/camera_yaw_error', Float64, queue_size=5)
        self.yaw_valid_pub = rospy.Publisher('/tunnel/camera_yaw_valid', Bool, queue_size=5)
        self.global_yaw_pub = rospy.Publisher('/tunnel/camera_global_yaw', Float64, queue_size=5)

        if self.publish_debug_image:
            self.debug_image_pub = rospy.Publisher(
                '/tunnel/camera_vp_debug/compressed', CompressedImage, queue_size=1)
        else:
            self.debug_image_pub = None

        # Subscribers
        rospy.Subscriber(self.gps_mode_topic, String, self.gps_mode_callback, queue_size=5)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=2)

        if 'compressed' in self.image_topic:
            rospy.Subscriber(self.image_topic, CompressedImage, self.compressed_image_callback,
                             queue_size=1, buff_size=2**24)
        else:
            rospy.Subscriber(self.image_topic, Image, self.raw_image_callback,
                             queue_size=1, buff_size=2**24)

        rospy.loginfo(
            '[tunnel_cam_vp] Initialized. HFOV=%.1f deg, Nominal Heading=%.2f deg, '
            'Tunnel X=[%.1f, %.1f], Alpha=%.2f, FrozenOnly=%s',
            self.camera_hfov_deg, self.tunnel_nominal_heading_deg,
            self.tunnel_min_x, self.tunnel_max_x, self.ema_alpha,
            self.only_when_gps_frozen)

    def odom_callback(self, msg):
        with self.lock:
            self.vehicle_x = float(msg.pose.pose.position.x)
            self.vehicle_y = float(msg.pose.pose.position.y)

    def gps_mode_callback(self, msg):
        with self.lock:
            self.gps_mode = msg.data.lower()

    def raw_image_callback(self, msg):
        if not self.enabled:
            return
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
            if img.shape[2] == 4:
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            elif img.shape[2] == 1:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            self.process_frame(img, msg.header.stamp)
        except Exception as e:
            rospy.logerr_throttle(2.0, f'[tunnel_cam_vp] Raw image decode error: {e}')

    def compressed_image_callback(self, msg):
        if not self.enabled:
            return
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img is not None:
                self.process_frame(img, msg.header.stamp)
        except Exception as e:
            rospy.logerr_throttle(2.0, f'[tunnel_cam_vp] Compressed image decode error: {e}')

    def process_frame(self, img, stamp):
        h, w, _ = img.shape
        hfov_rad = math.radians(self.camera_hfov_deg)
        fx = (w / 2.0) / math.tan(hfov_rad / 2.0)
        cx = w / 2.0
        cy = h / 2.0

        with self.lock:
            current_gps_mode = self.gps_mode
            cur_x = self.vehicle_x
            cur_y = self.vehicle_y

        # If configured to only operate when GPS is frozen
        is_frozen = (current_gps_mode == 'frozen')
        if self.only_when_gps_frozen and not is_frozen:
            # Reset smooth state when outside tunnel to prevent lag when re-entering
            self.smooth_delta_psi = None
            self.yaw_valid_pub.publish(Bool(data=False))
            return

        # Geographic gate: immediately deactivate when outside the straight tunnel tube (X and Y)
        if cur_x is not None and (cur_x < self.tunnel_min_x or cur_x > self.tunnel_max_x):
            self.smooth_delta_psi = None
            self.yaw_valid_pub.publish(Bool(data=False))
            return
        if cur_y is not None and (cur_y < self.tunnel_min_y or cur_y > self.tunnel_max_y):
            self.smooth_delta_psi = None
            self.yaw_valid_pub.publish(Bool(data=False))
            return

        # Crop ROI
        v1 = max(0, int(h * self.roi_top_ratio))
        v2 = min(h, int(h * self.roi_bottom_ratio))
        u1 = max(0, int(w * self.roi_left_ratio))
        u2 = min(w, int(w * self.roi_right_ratio))

        roi = img[v1:v2, u1:u2]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        scale = self.downsample_factor
        if scale < 0.99:
            gray_proc = cv2.resize(gray, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        else:
            gray_proc = gray

        # LSD Line Detection
        lines, _, _, _ = self.lsd.detect(gray_proc)
        if lines is None or len(lines) < self.min_line_count:
            self.handle_detection_failure(stamp)
            return

        # Scale coordinates back to full image
        inv_scale = 1.0 / scale if scale < 0.99 else 1.0
        valid_lines = []
        for l in lines:
            x1 = l[0][0] * inv_scale + u1
            y1 = l[0][1] * inv_scale + v1
            x2 = l[0][2] * inv_scale + u1
            y2 = l[0][3] * inv_scale + v1
            dx, dy = x2 - x1, y2 - y1
            length = math.hypot(dx, dy)
            if length < self.min_line_length:
                continue
            angle = math.degrees(math.atan2(abs(dy), abs(dx)))
            if angle < self.min_line_angle_deg or angle > self.max_line_angle_deg:
                continue

            # Normalized line equation: A*x + B*y + C = 0
            A = dy
            B = -dx
            C = dx * y1 - dy * x1
            norm = math.hypot(A, B)
            if norm < 1e-6:
                continue
            valid_lines.append((A / norm, B / norm, C / norm, (x1, y1, x2, y2)))

        if len(valid_lines) < self.min_line_count:
            self.handle_detection_failure(stamp)
            return

        # Compute line pair intersections
        pts = []
        n_lines = len(valid_lines)
        for i in range(n_lines):
            A1, B1, C1, _ = valid_lines[i]
            for j in range(i + 1, n_lines):
                A2, B2, C2, _ = valid_lines[j]
                det = A1 * B2 - A2 * B1
                if abs(det) < 0.12:  # reject nearly parallel lines
                    continue
                px = (B1 * C2 - B2 * C1) / det
                py = (C1 * A2 - C2 * A1) / det

                # Filter intersection bounds
                if 0.10 * w <= px <= 0.90 * w and 0.05 * h <= py <= 0.65 * h:
                    pts.append((px, py))

        if len(pts) < self.min_intersections:
            self.handle_detection_failure(stamp)
            return

        pts = np.array(pts)
        u_med = np.median(pts[:, 0])
        v_med = np.median(pts[:, 1])

        dists = np.hypot(pts[:, 0] - u_med, pts[:, 1] - v_med)
        inliers = pts[dists <= self.ransac_distance_px]

        if len(inliers) < 5:
            self.handle_detection_failure(stamp)
            return

        u_vp = float(np.mean(inliers[:, 0]))
        v_vp = float(np.mean(inliers[:, 1]))

        # Projective geometry: delta_psi_cam = arctan2(u_vp - cx, fx)
        delta_psi_raw = math.atan2(u_vp - cx, fx)

        # Sanity check on delta yaw limit
        if abs(delta_psi_raw) > math.radians(self.max_delta_yaw_deg):
            rospy.logwarn_throttle(
                1.0, f'[tunnel_cam_vp] Delta yaw exceeds limit: {math.degrees(delta_psi_raw):.1f} deg')
            self.handle_detection_failure(stamp)
            return

        # Jump filter: reject sudden spikes caused by exit glare or scenery change
        if self.smooth_delta_psi is not None:
            jump = abs(normalize_angle(delta_psi_raw - self.smooth_delta_psi))
            if jump > math.radians(self.max_delta_jump_deg):
                rospy.logwarn_throttle(
                    1.0,
                    f'[tunnel_cam_vp] Delta yaw jump ({math.degrees(jump):.1f} deg) exceeds limit, rejected')
                self.handle_detection_failure(stamp)
                return

        # Temporal EMA filter
        if self.smooth_delta_psi is None:
            self.smooth_delta_psi = delta_psi_raw
        else:
            self.smooth_delta_psi = (self.ema_alpha * delta_psi_raw +
                                    (1.0 - self.ema_alpha) * self.smooth_delta_psi)

        self.last_valid_time = stamp if stamp.to_sec() > 0 else rospy.Time.now()

        # Calculate global vehicle heading
        global_yaw = normalize_angle(self.tunnel_nominal_heading + self.smooth_delta_psi)

        # Publish results
        self.yaw_error_pub.publish(Float64(data=self.smooth_delta_psi))
        self.yaw_valid_pub.publish(Bool(data=True))
        self.global_yaw_pub.publish(Float64(data=global_yaw))

        # Debug overlay image
        if self.publish_debug_image and self.debug_image_pub is not None:
            self.publish_debug_overlay(img, valid_lines, inliers, u_vp, v_vp, cx, cy,
                                       delta_psi_raw, self.smooth_delta_psi, global_yaw, stamp)

    def handle_detection_failure(self, stamp):
        now = stamp if stamp.to_sec() > 0 else rospy.Time.now()
        if self.last_valid_time is not None:
            dt = (now - self.last_valid_time).to_sec()
            if dt > self.valid_timeout:
                self.yaw_valid_pub.publish(Bool(data=False))
        else:
            self.yaw_valid_pub.publish(Bool(data=False))

    def publish_debug_overlay(self, img, lines, inliers, u_vp, v_vp, cx, cy,
                              raw_yaw, smooth_yaw, global_yaw, stamp):
        vis = img.copy()

        # Draw detected structural lines
        for _, _, _, (x1, y1, x2, y2) in lines:
            cv2.line(vis, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 1)

        # Draw inlier intersection points
        for px, py in inliers:
            cv2.circle(vis, (int(px), int(py)), 2, (255, 200, 0), -1)

        # Draw optical center vertical reference
        h, w, _ = vis.shape
        cv2.line(vis, (int(cx), 0), (int(cx), h), (0, 255, 255), 1, cv2.LINE_AA)

        # Draw Vanishing Point
        ivp_x, ivp_y = int(round(u_vp)), int(round(v_vp))
        cv2.circle(vis, (ivp_x, ivp_y), 7, (0, 0, 255), -1)
        cv2.drawMarker(vis, (ivp_x, ivp_y), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)

        # Information banner
        err_deg = math.degrees(smooth_yaw)
        raw_deg = math.degrees(raw_yaw)
        glob_deg = math.degrees(global_yaw)
        text1 = f"VP: ({ivp_x}, {ivp_y}) | Offset: {u_vp - cx:+.1f}px | Inliers: {len(inliers)}/{len(lines)}"
        text2 = f"Yaw Error: {err_deg:+.2f} deg (Raw: {raw_deg:+.2f}) | Global: {glob_deg:+.2f} deg"

        cv2.rectangle(vis, (10, 10), (620, 75), (0, 0, 0), -1)
        cv2.putText(vis, text1, (18, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(vis, text2, (18, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)

        # Compress and publish
        comp_msg = CompressedImage()
        comp_msg.header.stamp = stamp
        comp_msg.format = "jpeg"
        comp_msg.data = np.array(cv2.imencode('.jpg', vis)[1]).tobytes()
        self.debug_image_pub.publish(comp_msg)


if __name__ == '__main__':
    detector = TunnelCameraYawDetector()
    rospy.spin()
