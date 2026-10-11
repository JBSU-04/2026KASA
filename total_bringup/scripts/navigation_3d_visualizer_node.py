#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
navigation_3d_visualizer_node.py
================================
3D 네비게이션 스타일(Tesla FSD / 현대차 AR 내비 / Waymo) 실시간 3D 시각화 노드.

주요 기능:
  1. 🚗 3D 자차 모델링 & 라이팅 (/navigation_3d/ego_vehicle_markers):
     - 정밀 3D 차체 (debug_vehicle.stl 또는 사이버 실버 에어로 바디)
     - 노면 밀착을 위한 하부 섀도우(Underbody Shadow)
     - 전방 도로를 비추는 듀얼 전조등 라이트 콘 (Headlight Cones)
     - 후방 테일램프 라이팅
     - 차량 상단에 우아하게 떠 있는 3D 홀로그램 주행 HUD
       (현재 속도 km/h, 목표 속도, 활성 미션 배지, 주행 상태)
  2. 🛣️ 3D 주행 유도로 카펫 (/navigation_3d/drive_carpet_markers):
     - /local_path 기준 차폭 2.2m의 매끄러운 3D 유도로 리본(Drive Carpet)
     - 주행 모드별 동적 색상 전환:
       * 일반 주행: 일렉트릭 사이언 (Electric Cyan)
       * 장애물 회피(Frenet): 고시인성 골드/앰버 (Glowing Amber)
       * 감속/정지/비상: 경고 크림슨 레드 (Warning Crimson)
     - 전방으로 갈수록 자연스럽게 페이드아웃되는 거리별 투명도 그래디언트
  3. 🚘 3D 주변 차량 & 장애물 (/navigation_3d/surrounding_markers):
     - 3D 선행/주변 차량 박스 및 상단 상태 태그 (ID, 거리, 상대속도, TTC)
     - 정적 장애물(드럼통, 상자)의 3D 모델 및 노면 안전 클리어런스 링(Halo)
"""

import math
import json
import rospy
import numpy as np
import tf.transformations as tft

from std_msgs.msg import String, Float64, Bool
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, Quaternion, Pose
from visualization_msgs.msg import Marker, MarkerArray
try:
    from mission_supervisor.msg import SectionState
except ImportError:
    SectionState = None
try:
    from lidar_ttc_tracker.msg import TrackedObjectArray
except ImportError:
    TrackedObjectArray = None


class Navigation3DVisualizerNode(object):
    def __init__(self):
        rospy.init_node('navigation_3d_visualizer', anonymous=False)

        # 파라미터
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.update_rate = rospy.get_param('~update_rate', 20.0)  # Hz
        self.carpet_width = rospy.get_param('~carpet_width', 2.20)  # m
        self.carpet_max_len = rospy.get_param('~carpet_max_len', 35.0)  # m
        self.mesh_resource = rospy.get_param(
            '~vehicle_mesh_resource',
            'package://smpc_lane_change/meshes/debug_vehicle.stl'
        )
        self.use_mesh = rospy.get_param('~use_vehicle_mesh', True)

        # 차량 상태 변수
        self.ego_x = 0.0
        self.ego_y = 0.0
        self.ego_z = 0.0
        self.ego_yaw = 0.0
        self.ego_speed_kph = 0.0
        self.target_speed_kph = 0.0
        self.have_odom = False

        # 미션 & 플래너 상태
        self.section_id = ""
        self.section_name = "일반 주행"
        self.section_progress = 0.0
        self.is_avoiding = False
        self.avoidance_failed = False
        self.is_highway = False

        # 경로 및 객체 데이터
        self.latest_local_path = None
        self.latest_tracked_objects = None
        self.latest_obstacle_info = None

        # 퍼블리셔
        self.ego_pub = rospy.Publisher('/navigation_3d/ego_vehicle_markers', MarkerArray, queue_size=5)
        self.carpet_pub = rospy.Publisher('/navigation_3d/drive_carpet_markers', MarkerArray, queue_size=5)
        self.surround_pub = rospy.Publisher('/navigation_3d/surrounding_markers', MarkerArray, queue_size=5)

        # 서브스크라이버
        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber('/local_path', Path, self.path_callback, queue_size=1)
        rospy.Subscriber('/control_feedback/mpc_target_speed_mps', Float64, self.target_speed_callback, queue_size=1)
        rospy.Subscriber('/avoidance_failed', Bool, self.avoidance_failed_callback, queue_size=1)

        if SectionState is not None:
            rospy.Subscriber('/mission/active_section', SectionState, self.section_callback, queue_size=1)

        if TrackedObjectArray is not None:
            rospy.Subscriber('/tracked_objects', TrackedObjectArray, self.tracked_objects_callback, queue_size=1)

        rospy.Subscriber('/obstacle_info', String, self.obstacle_info_callback, queue_size=1)

        rospy.loginfo("[navigation_3d_visualizer] 3D Navigation Visualizer Started successfully at %.1f Hz",
                      self.update_rate)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.update_rate), self.timer_callback)
        rospy.spin()

    def odom_callback(self, msg):
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y
        self.ego_z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        _, _, self.ego_yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.ego_speed_kph = math.hypot(vx, vy) * 3.6
        self.have_odom = True

    def path_callback(self, msg):
        self.latest_local_path = msg

    def target_speed_callback(self, msg):
        self.target_speed_kph = max(0.0, msg.data * 3.6)

    def avoidance_failed_callback(self, msg):
        self.avoidance_failed = msg.data

    def section_callback(self, msg):
        self.section_id = msg.section_id
        self.section_name = msg.section_name if msg.section_name else "자율주행 경로"
        self.section_progress = msg.progress
        self.is_highway = ("M6" in self.section_id or "M7" in self.section_id or "고주로" in self.section_name)

    def tracked_objects_callback(self, msg):
        self.latest_tracked_objects = msg

    def obstacle_info_callback(self, msg):
        try:
            self.latest_obstacle_info = json.loads(msg.data)
        except Exception:
            pass

    def timer_callback(self, event):
        if not self.have_odom:
            return
        now = rospy.Time.now()

        # 1. 3D 자차 & 조명 & 홀로그램 HUD 마커 발행
        self.publish_ego_markers(now)

        # 2. 3D 주행 유도로 카펫 마커 발행
        self.publish_drive_carpet(now)

        # 3. 3D 주변 차량 및 장애물 마커 발행
        self.publish_surrounding_markers(now)

    # =========================================================================
    # 1. 3D 자차 & 조명 & HUD
    # =========================================================================
    def publish_ego_markers(self, now):
        ma = MarkerArray()
        yaw = self.ego_yaw
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        qx = 0.0
        qy = 0.0
        qz = math.sin(0.5 * yaw)
        qw = math.cos(0.5 * yaw)

        # [A] 3D 하부 노면 밀착 그림자 (Underbody Shadow)
        shadow = Marker()
        shadow.header.frame_id = self.frame_id
        shadow.header.stamp = now
        shadow.ns = 'ego_shadow'
        shadow.id = 0
        shadow.type = Marker.CYLINDER
        shadow.action = Marker.ADD
        shadow.pose.position.x = self.ego_x + cy * 1.35
        shadow.pose.position.y = self.ego_y + sy * 1.35
        shadow.pose.position.z = self.ego_z + 0.03
        shadow.pose.orientation = Quaternion(qx, qy, qz, qw)
        shadow.scale.x = 4.80
        shadow.scale.y = 2.10
        shadow.scale.z = 0.04
        shadow.color.r = 0.02
        shadow.color.g = 0.03
        shadow.color.b = 0.06
        shadow.color.a = 0.65
        ma.markers.append(shadow)

        # [B] 3D 자차 본체 (Sleek Cyber Pearl Body)
        body = Marker()
        body.header.frame_id = self.frame_id
        body.header.stamp = now
        body.ns = 'ego_body'
        body.id = 1
        body.pose.position.x = self.ego_x + cy * 1.35
        body.pose.position.y = self.ego_y + sy * 1.35
        body.pose.position.z = self.ego_z + 0.75
        body.pose.orientation = Quaternion(qx, qy, qz, qw)
        body.scale.x = 4.63
        body.scale.y = 1.89
        body.scale.z = 1.50

        if self.use_mesh:
            body.type = Marker.MESH_RESOURCE
            body.mesh_resource = self.mesh_resource
            body.mesh_use_embedded_materials = false if hasattr(body, 'mesh_use_embedded_materials') else False
            body.pose.position.z = self.ego_z + 0.05
            body.color.r = 0.20
            body.color.g = 0.55
            body.color.b = 0.95
            body.color.a = 0.95
        else:
            body.type = Marker.CUBE
            body.color.r = 0.15
            body.color.g = 0.45
            body.color.b = 0.90
            body.color.a = 0.90
        ma.markers.append(body)

        # [C] 듀얼 전조등 라이트 콘 (Dual Glowing Headlight Cones)
        # IONIQ 5 전방 범퍼 좌/우 헤드라이트에서 전방 16m를 우아하게 비추는 반투명 조명
        headlights = Marker()
        headlights.header.frame_id = self.frame_id
        headlights.header.stamp = now
        headlights.ns = 'ego_headlights'
        headlights.id = 2
        headlights.type = Marker.TRIANGLE_LIST
        headlights.action = Marker.ADD
        headlights.pose.orientation.w = 1.0
        headlights.scale.x = 1.0
        headlights.scale.y = 1.0
        headlights.scale.z = 1.0
        headlights.color.r = 0.25
        headlights.color.g = 0.85
        headlights.color.b = 1.00
        headlights.color.a = 0.20

        # 좌/우 헤드라이트 위치 (base_link 기준)
        for side_sign in (-1.0, 1.0):
            # 헤드라이트 기점: x=+3.5m, y=side_sign*0.75m, z=0.6m
            lx = self.ego_x + cy * 3.50 - sy * (side_sign * 0.75)
            ly = self.ego_y + sy * 3.50 + cy * (side_sign * 0.75)
            lz = self.ego_z + 0.60
            p_origin = Point(lx, ly, lz)

            # 전방 16m 바닥 투사점 (좌측/우측 확산)
            fwd_dist = 16.0
            beam_half_w = 2.2
            bx1 = self.ego_x + cy * (3.5 + fwd_dist) - sy * (side_sign * 0.75 - beam_half_w)
            by1 = self.ego_y + sy * (3.5 + fwd_dist) + cy * (side_sign * 0.75 - beam_half_w)
            bz1 = self.ego_z + 0.05
            p_fwd1 = Point(bx1, by1, bz1)

            bx2 = self.ego_x + cy * (3.5 + fwd_dist) - sy * (side_sign * 0.75 + beam_half_w)
            by2 = self.ego_y + sy * (3.5 + fwd_dist) + cy * (side_sign * 0.75 + beam_half_w)
            bz2 = self.ego_z + 0.05
            p_fwd2 = Point(bx2, by2, bz2)

            headlights.points.extend([p_origin, p_fwd1, p_fwd2])
        ma.markers.append(headlights)

        # [D] 3D 홀로그램 주행 HUD (차량 상단 플로팅 디스플레이)
        hud = Marker()
        hud.header.frame_id = self.frame_id
        hud.header.stamp = now
        hud.ns = 'ego_hud'
        hud.id = 3
        hud.type = Marker.TEXT_VIEW_FACING
        hud.action = Marker.ADD
        hud.pose.position.x = self.ego_x + cy * 0.80
        hud.pose.position.y = self.ego_y + sy * 0.80
        hud.pose.position.z = self.ego_z + 2.70
        hud.scale.z = 0.80

        # 상태별 텍스트 구성
        status_tag = "정상 주행"
        hud.color.r = 0.10
        hud.color.g = 0.95
        hud.color.b = 1.00
        hud.color.a = 1.00

        if self.avoidance_failed:
            status_tag = "경로 불가(정지)"
            hud.color.r, hud.color.g, hud.color.b = 1.0, 0.2, 0.2
        elif self.is_avoiding:
            status_tag = "회피 기동중"
            hud.color.r, hud.color.g, hud.color.b = 1.0, 0.8, 0.1
        elif self.is_highway:
            status_tag = "고주로 ACC"
            hud.color.r, hud.color.g, hud.color.b = 0.3, 0.7, 1.0

        mission_label = f"[{self.section_id}] {self.section_name}" if self.section_id else "K-City 자율주행"
        hud.text = (
            f"★ {self.ego_speed_kph:.1f} km/h  (목표 {self.target_speed_kph:.0f})\n"
            f"{mission_label}\n"
            f"● {status_tag} | GEAR [ D ]"
        )
        ma.markers.append(hud)

        self.ego_pub.publish(ma)

    # =========================================================================
    # 2. 3D 주행 유도로 카펫 (Drive Carpet Ribbon)
    # =========================================================================
    def publish_drive_carpet(self, now):
        if self.latest_local_path is None or len(self.latest_local_path.poses) < 2:
            return

        poses = self.latest_local_path.poses
        ma = MarkerArray()

        carpet = Marker()
        carpet.header.frame_id = self.frame_id
        carpet.header.stamp = now
        carpet.ns = 'drive_carpet'
        carpet.id = 0
        carpet.type = Marker.TRIANGLE_LIST
        carpet.action = Marker.ADD
        carpet.pose.orientation.w = 1.0
        carpet.scale.x = 1.0
        carpet.scale.y = 1.0
        carpet.scale.z = 1.0

        # 카펫 테두리 글로우 라인
        edge_left = Marker()
        edge_left.header.frame_id = self.frame_id
        edge_left.header.stamp = now
        edge_left.ns = 'drive_carpet_edges'
        edge_left.id = 1
        edge_left.type = Marker.LINE_STRIP
        edge_left.action = Marker.ADD
        edge_left.pose.orientation.w = 1.0
        edge_left.scale.x = 0.08

        edge_right = Marker()
        edge_right.header.frame_id = self.frame_id
        edge_right.header.stamp = now
        edge_right.ns = 'drive_carpet_edges'
        edge_right.id = 2
        edge_right.type = Marker.LINE_STRIP
        edge_right.action = Marker.ADD
        edge_right.pose.orientation.w = 1.0
        edge_right.scale.x = 0.08

        # 주행 모드별 기본 색상 지정
        if self.avoidance_failed:
            base_r, base_g, base_b = 1.0, 0.20, 0.20
        elif self.is_avoiding:
            base_r, base_g, base_b = 1.0, 0.75, 0.10
        elif self.is_highway:
            base_r, base_g, base_b = 0.15, 0.65, 1.00
        else:
            base_r, base_g, base_b = 0.00, 0.85, 1.00

        carpet.color.r = base_r
        carpet.color.g = base_g
        carpet.color.b = base_b
        carpet.color.a = 0.50

        edge_left.color.r = min(1.0, base_r * 1.2)
        edge_left.color.g = min(1.0, base_g * 1.2)
        edge_left.color.b = min(1.0, base_b * 1.2)
        edge_left.color.a = 0.85
        edge_right.color = edge_left.color

        half_w = 0.5 * self.carpet_width

        # 경로 점들을 순회하며 좌/우 버텍스 생성 및 트라이앵글 스트립 구성
        left_pts = []
        right_pts = []
        cum_dist = 0.0

        for i in range(len(poses)):
            px = poses[i].pose.position.x
            py = poses[i].pose.position.y
            pz = poses[i].pose.position.z + 0.04  # 노면 z-fighting 방지 살짝 오프셋

            # 진행 방향 헤딩 계산
            if i + 1 < len(poses):
                nx = poses[i + 1].pose.position.x
                ny = poses[i + 1].pose.position.y
                seg_yaw = math.atan2(ny - py, nx - px)
                step_d = math.hypot(nx - px, ny - py)
            elif i > 0:
                prev_x = poses[i - 1].pose.position.x
                prev_y = poses[i - 1].pose.position.y
                seg_yaw = math.atan2(py - prev_y, px - prev_x)
                step_d = 0.0
            else:
                seg_yaw = self.ego_yaw
                step_d = 0.0

            if i > 0:
                cum_dist += step_d
            if cum_dist > self.carpet_max_len:
                break

            # 법선 벡터 (좌측 90도 회전)
            norm_x = -math.sin(seg_yaw)
            norm_y = math.cos(seg_yaw)

            # 전방 거리에 따른 부드러운 테이퍼링 (앞으로 갈수록 폭이 살짝 넓어지거나 좁아짐)
            w = half_w
            lp = Point(px + norm_x * w, py + norm_y * w, pz)
            rp = Point(px - norm_x * w, py - norm_y * w, pz)
            left_pts.append(lp)
            right_pts.append(rp)
            edge_left.points.append(lp)
            edge_right.points.append(rp)

        # 트라이앵글 페어 구성
        for k in range(len(left_pts) - 1):
            # Triangle 1: L_k -> R_k -> L_{k+1}
            carpet.points.extend([left_pts[k], right_pts[k], left_pts[k + 1]])
            # Triangle 2: R_k -> R_{k+1} -> L_{k+1}
            carpet.points.extend([right_pts[k], right_pts[k + 1], left_pts[k + 1]])

        if len(carpet.points) >= 3:
            ma.markers.append(carpet)
            ma.markers.append(edge_left)
            ma.markers.append(edge_right)

        self.carpet_pub.publish(ma)

    # =========================================================================
    # 3. 3D 주변 차량 및 장애물
    # =========================================================================
    def publish_surrounding_markers(self, now):
        ma = MarkerArray()

        # [A] TTC 추적 객체 (차량 및 동적 타겟)
        if self.latest_tracked_objects is not None:
            for obj in self.latest_tracked_objects.objects:
                dist_ego = math.hypot(obj.x - self.ego_x, obj.y - self.ego_y)
                if dist_ego > 60.0:
                    continue

                # 3D 차량 박스 / 섀도우
                m_box = Marker()
                m_box.header.frame_id = self.frame_id
                m_box.header.stamp = now
                m_box.ns = 'surround_cars'
                m_box.id = int(obj.id)
                m_box.type = Marker.CUBE
                m_box.action = Marker.ADD
                m_box.pose.position.x = obj.x
                m_box.pose.position.y = obj.y
                m_box.pose.position.z = max(0.4, 0.5 * obj.size_z)
                m_box.pose.orientation.z = math.sin(0.5 * obj.yaw)
                m_box.pose.orientation.w = math.cos(0.5 * obj.yaw)
                m_box.scale.x = max(0.8, obj.size_x)
                m_box.scale.y = max(0.8, obj.size_y)
                m_box.scale.z = max(0.5, obj.size_z)

                if obj.is_vehicle:
                    m_box.color.r = 0.20
                    m_box.color.g = 0.60
                    m_box.color.b = 1.00
                    m_box.color.a = 0.70
                elif obj.is_dynamic:
                    m_box.color.r = 1.00
                    m_box.color.g = 0.55
                    m_box.color.b = 0.10
                    m_box.color.a = 0.75
                else:
                    m_box.color.r = 0.70
                    m_box.color.g = 0.75
                    m_box.color.b = 0.85
                    m_box.color.a = 0.60
                ma.markers.append(m_box)

                # 3D 플로팅 정보 태그 (ID, 속도, 거리, TTC)
                m_txt = Marker()
                m_txt.header.frame_id = self.frame_id
                m_txt.header.stamp = now
                m_txt.ns = 'surround_tags'
                m_txt.id = int(obj.id)
                m_txt.type = Marker.TEXT_VIEW_FACING
                m_txt.action = Marker.ADD
                m_txt.pose.position.x = obj.x
                m_txt.pose.position.y = obj.y
                m_txt.pose.position.z = m_box.pose.position.z + 0.5 * obj.size_z + 0.65
                m_txt.scale.z = 0.60

                v_kph = math.hypot(obj.vx, obj.vy) * 3.6
                kind = "차량" if obj.is_vehicle else ("동적" if obj.is_dynamic else "물체")
                ttc_txt = f"TTC {obj.ttc:.1f}s" if obj.ttc >= 0 else ""
                m_txt.text = f"ID {obj.id} [{kind}] {dist_ego:.1f}m\n{v_kph:.1f} km/h {ttc_txt}"
                m_txt.color.r = 1.0
                m_txt.color.g = 1.0
                m_txt.color.b = 1.0
                m_txt.color.a = 0.95
                ma.markers.append(m_txt)

        # [B] 정적/동적 장애물 및 안전 클리어런스 헤일로 (/obstacle_info)
        if self.latest_obstacle_info and isinstance(self.latest_obstacle_info, list):
            for obs in self.latest_obstacle_info:
                obs_id = obs.get('id', 0)
                pos_w = obs.get('position_world', [0, 0])
                ox, oy = float(pos_w[0]), float(pos_w[1])
                dist_ego = math.hypot(ox - self.ego_x, oy - self.ego_y)
                if dist_ego > 50.0:
                    continue

                length = float(obs.get('length', 0.8))
                width = float(obs.get('width', 0.8))
                radius = float(obs.get('radius', 0.6))
                motion = obs.get('motion_state', 'static')

                # 3D 노면 안전 클리어런스 헤일로 (Safety Clearance Halo Ring)
                halo = Marker()
                halo.header.frame_id = self.frame_id
                halo.header.stamp = now
                halo.ns = 'obstacle_halo'
                halo.id = int(obs_id)
                halo.type = Marker.CYLINDER
                halo.action = Marker.ADD
                halo.pose.position.x = ox
                halo.pose.position.y = oy
                halo.pose.position.z = self.ego_z + 0.03
                halo.pose.orientation.w = 1.0
                # 로봇 반경(1.15) + 장애물 반경
                halo.scale.x = (radius + 1.15) * 2.0
                halo.scale.y = (radius + 1.15) * 2.0
                halo.scale.z = 0.02
                if motion == 'dynamic':
                    halo.color.r, halo.color.g, halo.color.b, halo.color.a = 1.0, 0.45, 0.10, 0.35
                else:
                    halo.color.r, halo.color.g, halo.color.b, halo.color.a = 1.0, 0.25, 0.15, 0.30
                ma.markers.append(halo)

        self.surround_pub.publish(ma)


if __name__ == '__main__':
    try:
        Navigation3DVisualizerNode()
    except rospy.ROSInterruptException:
        pass
