#!/usr/bin/env python3
"""RViz display of the measured right curb and the avoidance width limits.

obstacle_avoidance_node 가 실제로 쓰는 road_width_zones 파라미터와, 그 값의 근거인
실측 연석 경계(config/curb_edges.yaml)를 0.csv 기준경로 위에 같이 그린다.
한계선이 실측 연석에서 얼마나 떨어져 있는지, 어느 구간이 어느 주행 데이터로
정해졌는지를 한눈에 보려고 만들었다.

  노랑 실선 : 실측 우측 연석 (밝음 = 실측 칸 / 흐림 = 앞뒤 보간 칸)
  하늘 실선 : 실측 좌측 연석 (curb_edges 5번째 값이 있을 때)
  빨강 실선 : 우측 회피 상한 곡선 (차 중심이 갈 수 있는 한계 = 연석 - 0.95 - 0.15,
              road_width_profile 이 있으면 그 연속 곡선을 그린다)
  회색 실선 : 우측 개방 구간(교차로) - 고정 상한 max_road_width_right
  초록 실선 : 좌측 회피 상한 (같은 규칙, 상한 max_road_width_left)
  흰 글자   : 10 m(100 idx) 마다 우측 상한 값
"""
import os
import json

import rospkg
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray


def _load_path(csv_path):
    xs, ys = [], []
    with open(csv_path) as f:
        for line in f:
            p = line.strip().split(',')
            if len(p) < 3:
                continue
            try:
                xs.append(float(p[1]))
                ys.append(float(p[2]))
            except ValueError:
                continue
    return xs, ys


class CurbZoneVisualizer:
    def __init__(self):
        default_csv = os.path.join(rospkg.RosPack().get_path('waypoint_system'),
                                   'data', 'default', '0.csv')
        self.csv_path = rospy.get_param('~csv_path', default_csv)
        self.node_ns = rospy.get_param('~avoidance_node', '/obstacle_avoidance_node')
        self.step = int(rospy.get_param('~idx_step', 5))
        self.z = float(rospy.get_param('~z', 0.15))
        self.live_profile = None
        self.live_divider_edges = []
        self.live_max_left = self.live_max_right = None
        self.edges = rospy.get_param('~curb_edges', [])
        self.xs, self.ys = _load_path(self.csv_path)
        self.pub = rospy.Publisher('/obstacle_avoidance/curb_zone_markers', MarkerArray,
                                   queue_size=1, latch=True)
        self.boundary_sub = rospy.Subscriber(
            "/obstacle_avoidance/boundary_profile", String, self._boundary, queue_size=1)
        rospy.Timer(rospy.Duration(0.2), self._publish)
        self._publish(None)

    def _boundary(self, msg):
        try:
            data = json.loads(msg.data)
            profile = [float(v) for v in data['profile']]
            if len(profile) < 6 or len(profile) % 3:
                return
            self.live_profile = profile
            self.live_divider_edges = data.get('divider_edges') or []
            self.live_max_left = float(data['max_left'])
            self.live_max_right = float(data['max_right'])
        except (ValueError, KeyError, TypeError):
            return

    def _point(self, i, lateral):
        n = len(self.xs)
        i = max(0, min(n - 1, i))
        i0, i1 = max(0, i - 1), min(n - 1, i + 1)
        tx, ty = self.xs[i1] - self.xs[i0], self.ys[i1] - self.ys[i0]
        norm = (tx * tx + ty * ty) ** 0.5 or 1.0
        # d > 0 가 좌측 (adding_global_path 의 fx = ix - d*sin(yaw) 와 같은 규약)
        return Point(x=self.xs[i] - lateral * ty / norm,
                     y=self.ys[i] + lateral * tx / norm, z=self.z)

    def _marker(self, mid, ns, mtype, rgba, width):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = rospy.Time.now()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = width
        m.color = ColorRGBA(*rgba)
        return m

    def _line(self, mid, ns, a, b, lateral, rgba, width):
        m = self._marker(mid, ns, Marker.LINE_STRIP, rgba, width)
        last = min(b, len(self.xs) - 1)
        for i in list(range(a, last + 1, self.step)) + [last]:
            m.points.append(self._point(i, lateral))
        return m

    def _curb_lines(self, mid):
        """실측 연석: 칸 중심을 이어 한 줄로 그린다(연석 없는 칸에서 끊는다).
        우측은 노랑(실측 칸 밝게, 보간 칸 흐리게), 좌측(5번째 값)은 하늘색."""
        out = []
        for side in ('right', 'left'):
            seg = []
            seg_measured = None

            def flush():
                if len(seg) >= 2:
                    if side == 'right':
                        rgba = (1.0, 0.95, 0.1, 0.95) if seg_measured else (0.8, 0.75, 0.3, 0.55)
                    else:
                        rgba = (0.3, 0.8, 1.0, 0.9)
                    m = self._marker(mid + len(out), 'curb_measured_' + side,
                                     Marker.LINE_STRIP, rgba, 0.07)
                    m.points = list(seg)
                    out.append(m)

            for e in self.edges:
                a, b = int(e[0]), int(e[1])
                if side == 'right':
                    edge = float(e[2])
                    measured = '보간' not in str(e[3])
                else:
                    edge = float(e[4]) if len(e) > 4 else -1.0
                    measured = True
                    if self.live_profile is not None and '분리봉' in str(e[3]):
                        active = [v for v in self.live_divider_edges
                                  if int(v[0]) <= (a+b)//2 <= int(v[1])]
                        edge = float(active[0][2]) if active else -1.0
                if edge < 0.0:
                    flush()
                    seg, seg_measured = [], None
                    continue
                p = self._point((a + b) // 2, -edge if side == 'right' else edge)
                if seg_measured is not None and measured != seg_measured:
                    flush()
                    seg = seg[-1:]
                seg.append(p)
                seg_measured = measured
            flush()
        return out, mid + len(out)

    def _profile_markers(self, mid, prof, right_default, left_default):
        """연속 곡선(road_width_profile): 우측 한계선을 연석 있음(빨강)/개방(회색)
        으로 나눠 그리고, 좌측 한계선(초록)과 10 m 마다 값 라벨을 단다."""
        out = []
        n = len(prof) // 3
        knots = [(int(prof[k * 3]), float(prof[k * 3 + 1]), float(prof[k * 3 + 2])) for k in range(n)]
        # 우측: 개방 여부가 바뀌는 곳에서 선을 끊는다
        seg, seg_open = [], None
        def flush(points, is_open):
            if len(points) < 2:
                return
            m = self._marker(len(out) + mid, 'right_open' if is_open else 'right_limit',
                             Marker.LINE_STRIP,
                             (0.6, 0.6, 0.6, 0.6) if is_open else (1.0, 0.1, 0.1, 0.95),
                             0.06 if is_open else 0.10)
            m.points = points
            out.append(m)
        for i, r, _ in knots:
            is_open = r >= right_default - 1e-3
            p = self._point(i, -r)
            if seg_open is None or is_open == seg_open:
                seg.append(p)
            else:
                seg.append(p)
                flush(seg, seg_open)
                seg = [p]
            seg_open = is_open
        flush(seg, seg_open)
        # Known left limits connect only to known limits. Open space is not a measured 6m edge.
        left_points = []
        def flush_left():
            if len(left_points) >= 2:
                m = self._marker(len(out) + mid, 'left_limit', Marker.LINE_STRIP,
                                 (0.1, 0.9, 0.2, 0.9), 0.08)
                m.points = list(left_points)
                out.append(m)
        for i, _, l in knots:
            if l <= 0.0:
                flush_left()
                left_points = []
            else:
                left_points.append(self._point(i, l))
        flush_left()
        for i, r, l in knots:
            if i % 100 != 0:
                continue
            tm = self._marker(len(out) + mid, 'label', Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 0.9), 0.0)
            pp = self._point(i, -r - 0.7)
            pp.z = self.z + 0.8
            tm.pose.position = pp
            tm.scale.z = 0.5
            tm.text = '%d R %s' % (i, '개방' if r >= right_default - 1e-3 else '%.2f' % r)
            out.append(tm)
        return out

    def _publish(self, _event):
        if not self.xs:
            return
        prof = rospy.get_param(self.node_ns + '/road_width_profile', [])
        if self.live_profile is not None:
            prof = list(self.live_profile)
        zones = rospy.get_param(self.node_ns + '/road_width_zones', [])
        right_default = float(rospy.get_param(self.node_ns + '/max_road_width_right', 3.5))
        left_default = float(rospy.get_param(self.node_ns + '/max_road_width_left', 5.5))
        if self.live_max_left is not None:
            left_default = self.live_max_left
            right_default = self.live_max_right
        src_by_bin = {int(e[0]): str(e[3]) for e in self.edges}
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        curb, mid = self._curb_lines(0)
        arr.markers.extend(curb)
        if len(prof) >= 6:
            arr.markers.extend(self._profile_markers(mid, prof, right_default, left_default))
            self.pub.publish(arr)
            return
        for k in range(0, len(zones) - 3, 4):
            a, b = int(zones[k]), int(zones[k + 1])
            right, left = float(zones[k + 2]), float(zones[k + 3])
            if a >= len(self.xs):
                continue
            if right > 0.0:
                arr.markers.append(self._line(mid, 'right_limit', a, b, -right,
                                              (1.0, 0.1, 0.1, 0.95), 0.10)); mid += 1
            else:
                arr.markers.append(self._line(mid, 'right_open', a, b, -right_default,
                                              (0.6, 0.6, 0.6, 0.6), 0.06)); mid += 1
            left_eff = left if left > 0.0 else left_default
            arr.markers.append(self._line(mid, 'left_limit', a, b, left_eff,
                                          (0.1, 0.9, 0.2, 0.9), 0.08)); mid += 1
            srcs = sorted({src_by_bin[x] for x in range(a, b + 1, 25) if x in src_by_bin})
            t = self._marker(mid, 'label', Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 0.9), 0.0)
            p = self._point(a, -(right if right > 0.0 else right_default) - 0.7)
            p.z = self.z + 0.8
            t.pose.position = p
            t.scale.z = 0.55
            t.text = 'R %s  [%s]' % ('%.2f' % right if right > 0.0 else '개방',
                                     ','.join(srcs) if srcs else '-')
            arr.markers.append(t)
            mid += 1
        self.pub.publish(arr)


if __name__ == '__main__':
    rospy.init_node('curb_zone_visualizer')
    CurbZoneVisualizer()
    rospy.spin()
