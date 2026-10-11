#!/usr/bin/env python3
"""구간별 기준경로 코리도(2차, 즉시정지/회피 트리거 폭) RViz 시각화 (2026-10-05).

정적(M2, 0.csv static_zone_min_idx~split), 동적(M3, split~dynamic_zone_max_idx),
음영(M8, 3.csv tunnel_min_idx~tunnel_max_idx) 구간에서 /final_waypoint 앞부분을 따라
좌우 경계선 + 반투명 영역 + 라벨을 그린다. 폭은 obstacle_stop_node 가 실제로 쓰는
파라미터(path_corridor_radius / m3_path_corridor_radius / tunnel_path_corridor_radius)를
1 초마다 다시 읽어 그대로 보여 준다. 구간 밖이면 지운다.
출력: /reference_corridor/markers (visualization_msgs/MarkerArray, frame map)
"""
import math

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from waypoint_maker.msg import Lane

ZONES = {
    # 이름: (라벨, 색 r,g,b, 정지 노드 파라미터)
    'static': ('정적(M2) 기준경로 코리도', (0.20, 0.55, 1.00), 'path_corridor_radius'),
    'dynamic': ('동적(M3) 기준경로 코리도', (1.00, 0.55, 0.10), 'm3_path_corridor_radius'),
    'tunnel': ('음영(M8) 기준경로 코리도', (0.75, 0.30, 1.00), 'tunnel_path_corridor_radius'),
}


class ReferenceCorridorViz(object):
    def __init__(self):
        rospy.init_node('reference_corridor_viz')
        g = rospy.get_param
        self.static_min = int(g('~static_zone_min_idx', 2400))
        self.split = int(g('~static_dynamic_split_idx', 3700))
        self.dyn_max = int(g('~dynamic_zone_max_idx', 4900))
        self.tunnel_min = int(g('~tunnel_min_idx', 7000))
        self.tunnel_max = int(g('~tunnel_max_idx', 9000))
        self.length = float(g('~preview_m', 40.0))
        self.stop_ns = g('~stop_node_ns', '/obstacle_stop_node')
        self.default_width = float(g('~default_half_width', 1.4))
        self.widths = {}
        self.lane = None
        self.pub = rospy.Publisher('/reference_corridor/markers', MarkerArray, queue_size=1)
        rospy.Subscriber('/final_waypoint', Lane, self.cb, queue_size=1)
        rospy.Timer(rospy.Duration(1.0), self.refresh_widths)
        self.refresh_widths(None)

    def refresh_widths(self, _evt):
        for zone, (_, _, param) in ZONES.items():
            try:
                self.widths[zone] = float(rospy.get_param(self.stop_ns + '/' + param, self.default_width))
            except (TypeError, ValueError):
                self.widths[zone] = self.default_width

    def zone(self, lane, idx):
        if lane == 3 and self.tunnel_min <= idx <= self.tunnel_max:
            return 'tunnel'
        if lane == 0 and self.static_min <= idx <= self.split:
            return 'static'
        if lane == 0 and self.split < idx <= self.dyn_max:
            return 'dynamic'
        return None

    def cb(self, msg):
        out = MarkerArray()
        clear = Marker()
        clear.header.frame_id = 'map'
        clear.header.stamp = rospy.Time.now()
        clear.action = Marker.DELETEALL
        out.markers.append(clear)
        if len(msg.waypoints) < 2:
            self.pub.publish(out)
            return
        w0 = msg.waypoints[0]
        zone = self.zone(int(getattr(w0, 'lane_number', 0)), int(w0.waypoint_index))
        if zone is None:
            self.pub.publish(out)
            return
        label, (r, gg, b), _ = ZONES[zone]
        hw = self.widths.get(zone, self.default_width)
        pts = []
        acc = 0.0
        for i, w in enumerate(msg.waypoints):
            p = w.pose.pose.position
            if pts:
                acc += math.hypot(p.x - pts[-1][0], p.y - pts[-1][1])
                if acc > self.length:
                    break
            pts.append((p.x, p.y))
        if len(pts) < 2:
            self.pub.publish(out)
            return
        left, right = [], []
        for i, (x, y) in enumerate(pts):
            j2, j1 = min(i + 1, len(pts) - 1), max(i - 1, 0)
            tx, ty = pts[j2][0] - pts[j1][0], pts[j2][1] - pts[j1][1]
            n = math.hypot(tx, ty) or 1.0
            nx, ny = -ty / n, tx / n
            left.append(Point(x + nx * hw, y + ny * hw, 0.25))
            right.append(Point(x - nx * hw, y - ny * hw, 0.25))

        def base(ident, kind):
            m = Marker()
            m.header = clear.header
            m.ns = 'reference_corridor_' + zone
            m.id, m.type, m.action = ident, kind, Marker.ADD
            m.pose.orientation.w = 1.0
            m.color.r, m.color.g, m.color.b, m.color.a = r, gg, b, 0.95
            m.scale.x = 0.07
            m.lifetime = rospy.Duration(0.6)
            return m

        for ident, line in enumerate((left, right)):
            m = base(ident, Marker.LINE_STRIP)
            m.points = line
            out.markers.append(m)
        fill = base(2, Marker.TRIANGLE_LIST)
        fill.scale.x = fill.scale.y = fill.scale.z = 1.0
        fill.color.a = 0.12
        for i in range(len(pts) - 1):
            for p in (left[i], right[i], left[i + 1], right[i], right[i + 1], left[i + 1]):
                fill.points.append(Point(p.x, p.y, 0.24))
        out.markers.append(fill)
        text = base(3, Marker.TEXT_VIEW_FACING)
        text.pose.position = Point(pts[0][0], pts[0][1], 2.2)
        text.scale.z = 0.45
        text.text = '%s +/-%.2f m' % (label, hw)
        out.markers.append(text)
        self.pub.publish(out)


if __name__ == '__main__':
    ReferenceCorridorViz()
    rospy.spin()
