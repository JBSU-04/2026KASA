#!/usr/bin/env python3
"""
tl_bool_adapter — YOLO(예정)의 Bool 신호를 localizer 색 문자열로 변환

입력:  /traffic_light_stop (std_msgs/Bool)  true = 빨간불(정지해야 함)
출력:  /traffic_light_color (std_msgs/String) "red" / "green" — 10Hz 연속 발행

- YOLO 가 아직 없어서 Bool 이 한 번도 안 오면 기본 "green"(통과) 을 발행
  → map_localizer 의 '신호 미상이면 40m 내 정지' 규칙이 발동하지 않아
    신호등 없는 테스트 주행에 방해가 안 됨.
- 나중에 YOLO 는 std_msgs/Bool 로 /traffic_light_stop 만 쏘면 끝.
  (True 유지 = 계속 정지, False = 출발)
- 수동 테스트:
    rostopic pub /traffic_light_stop std_msgs/Bool "data: true"   # 정지
    rostopic pub /traffic_light_stop std_msgs/Bool "data: false"  # 출발
"""
import rospy
from std_msgs.msg import Bool, String


class TlBoolAdapter:
    def __init__(self):
        rospy.init_node("tl_bool_adapter")
        self.stop = False
        self.pub = rospy.Publisher("/traffic_light_color", String, queue_size=1)
        rospy.Subscriber("/traffic_light_stop", Bool, self.cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.tick)
        rospy.loginfo("[tl_bool] /traffic_light_stop(Bool) -> /traffic_light_color")
        rospy.spin()

    def cb(self, msg):
        if bool(msg.data) != self.stop:
            rospy.loginfo("[tl_bool] 신호 변경: %s", "RED(정지)" if msg.data else "GREEN(통과)")
        self.stop = bool(msg.data)

    def tick(self, _evt):
        self.pub.publish(String(data="red" if self.stop else "green"))


if __name__ == "__main__":
    try:
        TlBoolAdapter()
    except rospy.ROSInterruptException:
        pass
