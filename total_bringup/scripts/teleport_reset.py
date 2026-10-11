#!/usr/bin/env python3
import rospy
from morai_msgs.msg import MultiEgoSetting

def init_teleport():
    rospy.init_node('init_teleport_node')
    pub = rospy.Publisher('/Ego_Setting', MultiEgoSetting, queue_size=1)
    rospy.sleep(1.0) # Wait for network
    
    # K-City Start Point
    m = MultiEgoSetting()
    m.ego_index = 0
    m.pose_x = 9355.0
    m.pose_y = 19160.0
    m.pose_z = 0.0
    
    for _ in range(3):
        pub.publish(m)
        rospy.sleep(0.5)
    
    rospy.loginfo("Respawned at K-City Start Point")

if __name__ == '__main__':
    try:
        init_teleport()
    except rospy.ROSInterruptException:
        pass
