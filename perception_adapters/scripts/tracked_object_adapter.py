#!/usr/bin/env python3
"""
tracked_object_adapter — 트래킹 정본을 각 미션의 메시지 타입으로 변환한다.

병합 후 서로 다른 TrackedObjectArray 가 3종 존재한다(계획안 §2).
lidar_ttc_tracker 것을 정본으로 두고, 나머지 소비자에게는 변환만 해서 준다.
메시지 정의를 통일하려고 각 미션 코드를 고치는 것보다 안전하다.

입력: /tracked_objects (lidar_ttc_tracker/TrackedObjectArray)
출력:
  /perception/roundabout/tracked_objects (mission_planner_msgs/TrackedObjectArray)
  /perception/shadow/tracked_objects     (non_gps_msgs/TrackedObjectArray)

두 출력은 각각 파라미터로 끌 수 있다. 회전교차로는 전용 트래커를 그대로
쓰는 것이 검증된 구성이므로 기본값은 꺼둔다.
"""
import math
import rospy
from lidar_ttc_tracker.msg import TrackedObjectArray as TtcArray
from mission_planner_msgs.msg import (TrackedObjectArray as MpArray,
                                      TrackedObject as MpObject)
from non_gps_msgs.msg import (TrackedObjectArray as NgArray,
                              TrackedObject as NgObject)


class TrackedObjectAdapter(object):
    def __init__(self):
        rospy.init_node('tracked_object_adapter')

        self.enable_roundabout = bool(rospy.get_param('~enable_roundabout', False))
        self.enable_shadow = bool(rospy.get_param('~enable_shadow', False))
        self.min_confidence = float(rospy.get_param('~min_confidence', 0.0))
        self.moved_hits = int(rospy.get_param('~moved_min_hits', 3))
        self.static_speed = float(rospy.get_param('~static_speed_threshold', 1.5))

        self.pub_mp = None
        self.pub_ng = None
        if self.enable_roundabout:
            self.pub_mp = rospy.Publisher('/perception/roundabout/tracked_objects',
                                          MpArray, queue_size=1)
        if self.enable_shadow:
            self.pub_ng = rospy.Publisher('/perception/shadow/tracked_objects',
                                          NgArray, queue_size=1)

        rospy.Subscriber('/tracked_objects', TtcArray, self.cb, queue_size=1)
        rospy.loginfo('[tracked_object_adapter] ready. roundabout=%s shadow=%s',
                      self.enable_roundabout, self.enable_shadow)

    @staticmethod
    def _axis_aligned_size(o):
        """yaw 정렬 bbox를 지도축 정렬 bbox로 근사한다."""
        c, s = abs(math.cos(o.yaw)), abs(math.sin(o.yaw))
        return (o.size_x * c + o.size_y * s,
                o.size_x * s + o.size_y * c,
                o.size_z)

    def cb(self, msg):
        if self.pub_mp is not None:
            out = MpArray()
            out.header = msg.header
            out.valid = True
            for o in msg.objects:
                # 버릴지 여부는 장애물성으로 판단한다. 차량성은 '어떤 모양인가'
                # 이고 낮은 장애물은 정당하게 0 점을 받는데, 그것으로 존재를
                # 부정하면 차 모양이 아닌 장애물이 통째로 사라진다.
                if o.obstacle_confidence < self.min_confidence:
                    continue
                t = MpObject()
                t.id = int(o.id)
                t.position.x, t.position.y, t.position.z = o.x, o.y, o.z
                t.velocity.x, t.velocity.y, t.velocity.z = o.vx, o.vy, 0.0
                sx, sy, sz = self._axis_aligned_size(o)
                t.size.x, t.size.y, t.size.z = sx, sy, sz
                t.confidence = float(o.vehicle_confidence)
                t.hit_count = int(o.hits)
                t.velocity_valid = bool(o.hits >= self.moved_hits)
                t.has_moved = bool(o.is_dynamic)
                out.objects.append(t)
            self.pub_mp.publish(out)

        if self.pub_ng is not None:
            out = NgArray()
            out.header = msg.header
            for o in msg.objects:
                t = NgObject()
                t.id = int(o.id)
                t.position.x, t.position.y, t.position.z = o.x, o.y, o.z
                t.velocity.x, t.velocity.y, t.velocity.z = o.vx, o.vy, 0.0
                sx, sy, sz = self._axis_aligned_size(o)
                t.size.x, t.size.y, t.size.z = sx, sy, sz
                t.confidence = float(o.vehicle_confidence)
                if o.hits < self.moved_hits:
                    t.motion_type = NgObject.UNKNOWN
                elif o.speed >= self.static_speed:
                    t.motion_type = NgObject.DYNAMIC
                else:
                    t.motion_type = NgObject.STATIC
                out.objects.append(t)
            self.pub_ng.publish(out)


if __name__ == '__main__':
    try:
        TrackedObjectAdapter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
