#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
교차로 진입판단 노드 — MGeo 정지선 인지 + 교차교통 gap-acceptance.

로직
----
1) MGeo node_set 에서 on_stop_line=True 노드(정지선) 좌표를 로드.
2) 매 주기 자차 전방(heading 기준) lookahead 안의 최근접 정지선까지 거리 d_stop 산출
   -> 정지선이 없으면 개입 없음(NONE). 있으면 그 지점을 conflict zone 중심으로.
3) 교차교통 gap-acceptance:
   - 동적 트랙 중 자차 코리도 밖(교차 방향) 물체를 대상으로,
   - 자차의 교차구간 점유 시간창 [t_ego_arrive, t_ego_arrive + t_cross] 와
     물체의 conflict 도달시각 t_obj 가 PET(post-encroachment time) 여유 안에서 겹치면 위험.
   - 위험하면 HOLD(정지선 앞 정지 프로파일), 안전하면 NONE(진입 허용).
   자차 정지 상태(t_ego_arrive 무한)면 거리 기반: conflict 반경 안 접근 물체 있으면 HOLD.

교통신호(적색) 정지는 kcity_localizer(/localization/stop_required)가 MPC 경로로 이미 처리하므로
여기서는 '교차교통 gap' 만 담당한다. 출력은 arbiter 가 MPC·AEB 명령과 함께 융합.

입력 : /odom, /tracked_objects
출력 : ~out_topic(AebCommand, 기본 /intersection/command), /intersection/markers
"""
import os
import math
import json
import rospy
import rospkg
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

from lidar_ttc_tracker.msg import TrackedObjectArray
from aeb_safety.msg import AebCommand


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class IntersectionManager(object):
    def __init__(self):
        rospy.init_node('intersection_manager', anonymous=False)

        self.map_name = rospy.get_param('~map', 'kcity2025')
        self.stop_lookahead = rospy.get_param('~stop_lookahead', 40.0)
        self.stop_fwd_min = rospy.get_param('~stop_fwd_min', 2.0)   # 정지선이 이보다 뒤면 무시
        self.conflict_radius = rospy.get_param('~conflict_radius', 18.0)  # 교차로 크기
        self.cross_min_speed = rospy.get_param('~cross_min_speed', 1.5)   # 교차물체 최소속력
        self.pet_margin = rospy.get_param('~pet_margin', 2.0)       # 겹침 안전여유 [s]
        self.release_gap = rospy.get_param('~release_gap', 3.0)     # 진입 허용 최소 gap [s]
        self.lane_half = rospy.get_param('~lane_half_width', 1.75)
        self.comfort_decel = rospy.get_param('~comfort_decel', 2.0)
        self.stop_margin = rospy.get_param('~stop_margin', 4.0)
        self.ref_v = rospy.get_param('~ref_v', 12.0)

        self.stop_pts = self._load_stop_lines()

        self.have_ego = False
        self.ego_x = self.ego_y = 0.0
        self.ego_yaw = 0.0
        self.ego_v = 0.0
        self.objects = []
        self._holding = False

        out_topic = rospy.get_param('~out_topic', '/intersection/command')
        self.cmd_pub = rospy.Publisher(out_topic, AebCommand, queue_size=1)
        self.mk_pub = rospy.Publisher('/intersection/markers', MarkerArray, queue_size=1)

        rospy.Subscriber('/odom', Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber('/tracked_objects', TrackedObjectArray, self.objects_cb, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.tick)
        rospy.loginfo('[intersection_manager] ready. stop_lines=%d radius=%.0fm pet=%.1fs',
                      len(self.stop_pts), self.conflict_radius, self.pet_margin)

    def _load_stop_lines(self):
        """MGeo node_set 에서 on_stop_line 노드 좌표 [x,y] 리스트. 실패 시 빈 리스트(무개입)."""
        path = rospy.get_param('~node_set_path', '')
        if not path:
            try:
                base = rospkg.RosPack().get_path('map_loader')
                path = os.path.join(base, 'scripts', 'lib', 'mgeo_data',
                                    self.map_name, 'node_set.json')
            except Exception as e:
                rospy.logwarn('[intersection_manager] map_loader 경로 실패: %s', e)
                return []
        try:
            nodes = json.load(open(path))
        except Exception as e:
            rospy.logwarn('[intersection_manager] node_set 로드 실패(%s): %s', path, e)
            return []
        pts = []
        for n in nodes:
            if n.get('on_stop_line'):
                p = n.get('point')
                if p and len(p) >= 2:
                    pts.append((float(p[0]), float(p[1])))
        rospy.loginfo('[intersection_manager] 정지선 노드 %d개 로드', len(pts))
        return pts

    def odom_cb(self, msg):
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y
        self.ego_yaw = quat_to_yaw(msg.pose.pose.orientation)
        v = msg.twist.twist.linear.x
        self.ego_v = abs(v) if math.isfinite(v) else 0.0
        self.have_ego = True

    def objects_cb(self, msg):
        self.objects = list(msg.objects)

    def _nearest_stop_ahead(self):
        """자차 전방 lookahead 안 최근접 정지선 (d_stop, sx, sy) 또는 None."""
        c, s = math.cos(self.ego_yaw), math.sin(self.ego_yaw)
        best = None
        for (sx, sy) in self.stop_pts:
            dx, dy = sx - self.ego_x, sy - self.ego_y
            fwd = dx * c + dy * s
            lat = -dx * s + dy * c
            if fwd < self.stop_fwd_min or fwd > self.stop_lookahead:
                continue
            if abs(lat) > self.lane_half + 1.0:   # 내 진행 차선의 정지선만
                continue
            if best is None or fwd < best[0]:
                best = (fwd, sx, sy)
        return best

    def tick(self, _evt):
        cmd = AebCommand()
        cmd.header = Header(stamp=rospy.Time.now(), frame_id='map')
        cmd.source = 'intersection'
        cmd.stage = AebCommand.STAGE_NONE
        cmd.emergency = False
        cmd.target_speed = -1.0
        cmd.brake = 0.0
        cmd.decel = 0.0
        cmd.target_id = 0
        cmd.ttc = -1.0
        cmd.distance = -1.0
        cmd.reason = 'no_intersection'

        if not self.have_ego or not self.stop_pts:
            self.cmd_pub.publish(cmd)
            return

        near = self._nearest_stop_ahead()
        if near is None:
            self._holding = False
            cmd.reason = 'no_stopline_ahead'
            self.cmd_pub.publish(cmd)
            self._markers(cmd, None)
            return

        d_stop, cx, cy = near
        cmd.distance = d_stop

        # 자차 교차구간 점유 시간창
        eps = 0.5
        if self.ego_v > eps:
            t_arrive = d_stop / self.ego_v
        else:
            t_arrive = 0.0   # 정지선 근처 정지 상태 -> 지금 진입 시도로 간주
        t_cross = (2.0 * self.conflict_radius) / max(self.ego_v, 2.0)

        # 교차교통 gap-acceptance
        worst = None  # (min_gap, obj_id, t_obj)
        for o in self.objects:
            if not o.is_dynamic or o.speed < self.cross_min_speed:
                continue
            # conflict 중심까지 물체 거리/도달시각
            ox, oy = o.x - cx, o.y - cy
            dist_o = math.hypot(ox, oy)
            if dist_o > self.conflict_radius + o.speed * (t_arrive + t_cross + self.pet_margin):
                continue  # 시간창 내 도달 불가
            # 물체가 conflict 중심으로 접근 중인가 (속도·위치 내적<0)
            approaching = (ox * o.vx + oy * o.vy) < 0.0
            if not approaching:
                continue
            t_obj = dist_o / max(o.speed, 0.1)
            # 자차 점유창 [t_arrive, t_arrive+t_cross] 와 t_obj 의 PET 겹침
            gap = min(abs(t_obj - t_arrive), abs(t_obj - (t_arrive + t_cross)))
            overlap = (t_arrive - self.pet_margin) <= t_obj <= (t_arrive + t_cross + self.pet_margin)
            g = 0.0 if overlap else gap
            if worst is None or g < worst[0]:
                worst = (g, o.id, t_obj)

        unsafe = worst is not None and (worst[0] < self.pet_margin)
        # 히스테리시스: 한 번 HOLD 면 release_gap 이상 확보돼야 해제
        if self._holding:
            unsafe = worst is not None and (worst[0] < self.release_gap)

        if unsafe:
            self._holding = True
            eff = max(d_stop - self.stop_margin, 0.0)
            v_prof = math.sqrt(2.0 * self.comfort_decel * eff)
            cmd.stage = AebCommand.STAGE_HOLD
            cmd.target_speed = min(self.ref_v, v_prof) * 3.6
            cmd.decel = self.comfort_decel
            cmd.target_id = worst[1]
            cmd.ttc = worst[2]
            cmd.reason = 'HOLD cross id=%d t_obj=%.1f d_stop=%.1f' % (worst[1], worst[2], d_stop)
        else:
            self._holding = False
            cmd.reason = 'clear d_stop=%.1f' % d_stop

        self.cmd_pub.publish(cmd)
        self._markers(cmd, near)

    def _markers(self, cmd, near):
        ma = MarkerArray()
        if near is not None:
            d_stop, cx, cy = near
            z = Marker()
            z.header = cmd.header
            z.ns = 'xing_zone'
            z.id = 0
            z.type = Marker.CYLINDER
            z.action = Marker.ADD
            z.pose.position.x = cx
            z.pose.position.y = cy
            z.pose.position.z = 0.1
            z.pose.orientation.w = 1.0
            z.scale.x = z.scale.y = 2.0 * self.conflict_radius
            z.scale.z = 0.2
            if cmd.stage == AebCommand.STAGE_HOLD:
                z.color.r, z.color.g, z.color.b, z.color.a = 1.0, 0.2, 0.2, 0.25
            else:
                z.color.r, z.color.g, z.color.b, z.color.a = 0.2, 0.9, 0.3, 0.18
            z.lifetime = rospy.Duration(0.3)
            ma.markers.append(z)
        self.mk_pub.publish(ma)


if __name__ == '__main__':
    try:
        IntersectionManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
