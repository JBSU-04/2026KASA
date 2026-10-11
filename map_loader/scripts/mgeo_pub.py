#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import json
import math
import rospy

from geometry_msgs.msg import Point32, Point
from sensor_msgs.msg import PointCloud
from visualization_msgs.msg import Marker, MarkerArray

current_path = os.path.dirname(os.path.realpath(__file__))
# 구형 MGeo 모듈은 ``from utils...`` 절대 import를 사용한다. 시스템에
# 설치된 동명 utils 패키지보다 번들된 lib/mgeo/utils를 우선한다.
sys.path.insert(0, os.path.join(current_path, 'lib', 'mgeo'))
sys.path.insert(0, current_path)

from lib.mgeo.class_defs import *

class get_mgeo :

    def __init__(self):
        rospy.init_node('mgeo_pub', anonymous=True)
        self.link_pub = rospy.Publisher('link',PointCloud, queue_size=1)
        self.node_pub = rospy.Publisher('node',PointCloud, queue_size=1)
        # 내비식 지도 렌더링 (kcity_localizer map_localizer 와 동일 스타일):
        # 차선 도색(색/실선/점선/겹선) + 정지선 + 신호등 + 횡단보도. latched 1회.
        self.map_marker_pub = rospy.Publisher('/mgeo_map_markers', MarkerArray,
                                              queue_size=1, latch=True)

        # ~map: lib/mgeo_data/ 아래 폴더명 (kcity2025 는 kcity_localizer/data 심링크)
        map_name = rospy.get_param('~map', 'kcity2025')
        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data', map_name))
        self.data_dir = load_path
        mgeo_planner_map = MGeoPlannerMap.create_instance_from_json(load_path)

        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set
        self.nodes=node_set.nodes
        self.links=link_set.lines
        self.link_msg=self.getAllLinks()
        self.node_msg=self.getAllNode()

        marker_arr = self.build_map_markers()
        self.map_marker_pub.publish(marker_arr)
        rospy.loginfo('[mgeo_pub] /mgeo_map_markers %d markers (map=%s)',
                      len(marker_arr.markers), map_name)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.link_pub.publish(self.link_msg)
            self.node_pub.publish(self.node_msg)

            rate.sleep()

    def getAllLinks(self):
        all_link=PointCloud()
        all_link.header.frame_id='map'
        
        for link_idx in self.links :
            for link_point in self.links[link_idx].points:
                tmp_point=Point32()
                tmp_point.x=link_point[0]
                tmp_point.y=link_point[1]
                # kcity2025 는 origin z=0 이라 점 z가 고도(~30m)로 남아 rviz 에서
                # ego(z=0) 위에 떠 보인다 -> 시각화용이므로 평탄화
                tmp_point.z=0.0
                all_link.points.append(tmp_point)

        return all_link
    
    def getAllNode(self):
        all_node=PointCloud()
        all_node.header.frame_id='map'
        for node_idx in self.nodes :
            tmp_point=Point32()
            tmp_point.x=self.nodes[node_idx].point[0]
            tmp_point.y=self.nodes[node_idx].point[1]
            tmp_point.z=0.0
            all_node.points.append(tmp_point)

        return all_node

    # --------------------------------------------------------------- 내비식 지도
    # kcity_localizer/scripts/map_localizer.py 의 렌더링 이식 (동일 스타일 유지).
    LANE_COLORS = {
        "yellow": (1.0, 0.78, 0.05),
        "white": (0.92, 0.92, 0.92),
        "blue": (0.25, 0.5, 1.0),
        "curb": (0.55, 0.55, 0.6),      # sangam 연석
    }

    def _load_json(self, name):
        path = os.path.join(self.data_dir, name)
        if not os.path.exists(path):
            return []
        try:
            with open(path, 'r') as f:
                return json.load(f)
        except Exception as e:
            rospy.logwarn('[mgeo_pub] %s load failed: %s', name, e)
            return []

    def build_map_markers(self):
        arr = MarkerArray()
        self._append_lane_marking_markers(arr)
        self._append_hdmap_feature_markers(arr)
        return arr

    def _append_lane_marking_markers(self, arr):
        """lane_boundary_set 의 노면 도색 차선을 실제 색/형태로 렌더링.
        정지선(530)은 _append_hdmap_feature_markers 가 따로 그리므로 제외."""
        boundaries = self._load_json('lane_boundary_set.json')
        if not boundaries:
            return

        buckets = {}

        def emit(pts, color, shape, dash):
            segs = buckets.setdefault((color, shape), [])
            if shape == 'broken':
                segs.extend(self._dash_segments(pts, dash[0], dash[1]))
            else:
                for i in range(len(pts) - 1):
                    segs.append((pts[i], pts[i + 1]))

        for item in boundaries:
            if 530 in (item.get('lane_type') or []):
                continue
            raw = item.get('points') or []
            if len(raw) < 2:
                continue
            pts = [(float(p[0]), float(p[1])) for p in raw]
            # sangam 은 'Broken'/'Solid'/'Curb' 처럼 대문자 -> 소문자 정규화
            color = str((item.get('lane_color') or ['white'])[0]).lower()
            shape_tokens = str((item.get('lane_shape') or ['solid'])[0] or 'solid').lower().split()
            l1 = float(item.get('dash_interval_L1') or 0) or 3.0
            l2 = float(item.get('dash_interval_L2') or 0) or 5.0

            if len(shape_tokens) >= 2:
                # 겹선: 좌우로 반씩 벌려 두 줄 (예: 중앙 실선+점선)
                gap = float(item.get('double_line_interval') or 0) or 0.3
                left, right = self._offset_polyline(pts, gap * 0.5)
                emit(left, color, shape_tokens[0], (l1, l2))
                emit(right, color, shape_tokens[1], (l1, l2))
            else:
                emit(pts, color, shape_tokens[0], (l1, l2))

        mid = 20
        for (color, shape), segs in sorted(buckets.items()):
            if not segs:
                continue
            mk = Marker()
            mk.header.frame_id = 'map'
            mk.header.stamp = rospy.Time.now()
            mk.ns = 'lane_%s_%s' % (color, shape)
            mk.id = mid
            mid += 1
            mk.type = Marker.LINE_LIST
            mk.action = Marker.ADD
            mk.scale.x = 0.15
            r, g, b = self.LANE_COLORS.get(color, self.LANE_COLORS['white'])
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = r, g, b, 0.95
            mk.pose.orientation.w = 1.0
            for pa, pb in segs:
                mk.points.append(Point(pa[0], pa[1], 0.02))
                mk.points.append(Point(pb[0], pb[1], 0.02))
            arr.markers.append(mk)

    @staticmethod
    def _dash_segments(pts, paint, gap):
        """폴리라인을 아크길이 기준 도색(paint)/공백(gap) 반복으로 잘라 반환."""
        period = paint + gap
        segs = []
        s = 0.0
        for i in range(len(pts) - 1):
            ax, ay = pts[i]
            bx, by = pts[i + 1]
            seg_len = math.hypot(bx - ax, by - ay)
            if seg_len < 1e-9:
                continue
            s1 = s + seg_len
            k = int(s // period)
            while k * period < s1:
                p0 = max(s, k * period)
                p1 = min(s1, k * period + paint)
                if p1 > p0:
                    t0 = (p0 - s) / seg_len
                    t1 = (p1 - s) / seg_len
                    segs.append(((ax + (bx - ax) * t0, ay + (by - ay) * t0),
                                 (ax + (bx - ax) * t1, ay + (by - ay) * t1)))
                k += 1
            s = s1
        return segs

    @staticmethod
    def _offset_polyline(pts, half):
        """폴리라인을 진행방향 좌/우로 half 만큼 평행이동한 두 폴리라인 반환."""
        left, right = [], []
        n = len(pts)
        for i in range(n):
            a = pts[max(i - 1, 0)]
            b = pts[min(i + 1, n - 1)]
            dx, dy = b[0] - a[0], b[1] - a[1]
            norm = math.hypot(dx, dy) or 1.0
            nx, ny = -dy / norm, dx / norm
            left.append((pts[i][0] + nx * half, pts[i][1] + ny * half))
            right.append((pts[i][0] - nx * half, pts[i][1] - ny * half))
        return left, right

    def _append_hdmap_feature_markers(self, arr):
        traffic_lights = self._load_json('traffic_light_set.json')
        if traffic_lights:
            tl_pole = Marker()
            tl_pole.header.frame_id = 'map'
            tl_pole.header.stamp = rospy.Time.now()
            tl_pole.ns = 'traffic_light_poles'
            tl_pole.id = 10
            tl_pole.type = Marker.LINE_LIST
            tl_pole.action = Marker.ADD
            tl_pole.scale.x = 0.18
            tl_pole.color.r, tl_pole.color.g, tl_pole.color.b, tl_pole.color.a = 0.75, 0.75, 0.75, 0.95
            tl_pole.pose.orientation.w = 1.0

            tl_head = Marker()
            tl_head.header.frame_id = 'map'
            tl_head.header.stamp = rospy.Time.now()
            tl_head.ns = 'traffic_light_heads'
            tl_head.id = 11
            tl_head.type = Marker.CUBE_LIST
            tl_head.action = Marker.ADD
            tl_head.scale.x = 0.55
            tl_head.scale.y = 0.25
            tl_head.scale.z = 0.9
            tl_head.color.r, tl_head.color.g, tl_head.color.b, tl_head.color.a = 1.0, 0.15, 0.15, 0.95
            tl_head.pose.orientation.w = 1.0
            for item in traffic_lights:
                p = item.get('point')
                if p:
                    x = float(p[0])
                    y = float(p[1])
                    tl_pole.points.append(Point(x, y, 0.0))
                    tl_pole.points.append(Point(x, y, 2.4))
                    tl_head.points.append(Point(x, y, 2.4))
            arr.markers.append(tl_pole)
            arr.markers.append(tl_head)

        stop_lines = self._load_json('lane_boundary_set.json')
        if stop_lines:
            stop = Marker()
            stop.header.frame_id = 'map'
            stop.header.stamp = rospy.Time.now()
            stop.ns = 'stop_lines'
            stop.id = 12
            stop.type = Marker.CUBE_LIST
            stop.action = Marker.ADD
            stop.scale.x = 0.22
            stop.scale.y = 0.55
            stop.scale.z = 0.04
            stop.color.r, stop.color.g, stop.color.b, stop.color.a = 1.0, 1.0, 1.0, 0.95
            stop.pose.orientation.w = 1.0
            for item in stop_lines:
                if 530 not in (item.get('lane_type') or []):
                    continue
                pts = item.get('points') or []
                for i in range(len(pts) - 1):
                    ax, ay = float(pts[i][0]), float(pts[i][1])
                    bx, by = float(pts[i + 1][0]), float(pts[i + 1][1])
                    seg_len = math.hypot(bx - ax, by - ay)
                    samples = max(1, int(seg_len / 0.25))
                    for j in range(samples + 1):
                        t = float(j) / float(samples)
                        stop.points.append(Point(ax + t * (bx - ax), ay + t * (by - ay), 0.02))
            if stop.points:
                arr.markers.append(stop)

        crosswalks = self._load_json('singlecrosswalk_set.json')
        if crosswalks:
            cw = Marker()
            cw.header.frame_id = 'map'
            cw.header.stamp = rospy.Time.now()
            cw.ns = 'crosswalks'
            cw.id = 13
            cw.type = Marker.TRIANGLE_LIST
            cw.action = Marker.ADD
            cw.scale.x = cw.scale.y = cw.scale.z = 1.0
            cw.color.r, cw.color.g, cw.color.b, cw.color.a = 1.0, 0.9, 0.2, 0.35
            cw.pose.orientation.w = 1.0
            for item in crosswalks:
                pts = item.get('points') or []
                if len(pts) < 4:
                    continue
                poly = pts[:-1] if pts[0] == pts[-1] else pts
                if len(poly) < 3:
                    continue
                origin = poly[0]
                for i in range(1, len(poly) - 1):
                    a, b, c = origin, poly[i], poly[i + 1]
                    cw.points.append(Point(float(a[0]), float(a[1]), 0.01))
                    cw.points.append(Point(float(b[0]), float(b[1]), 0.01))
                    cw.points.append(Point(float(c[0]), float(c[1]), 0.01))
            if cw.points:
                arr.markers.append(cw)

if __name__ == '__main__':
    
    test_track=get_mgeo()

