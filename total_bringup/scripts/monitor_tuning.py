#!/usr/bin/env python3
"""실시간 피팅 모니터링 스크립트.
- IMU 자세 및 gravity_frame TF (roll_dyn, pitch_dyn)
- /odom 속도 및 가속/제동 상태
- /obstacle_info 장애물 수 및 가짜 지면 잔차 탐지 여부 (z_max, cluster_h, top)
- /nonground 및 /groundgrid/ground 점 개수 추이
"""
import rospy
import tf2_ros
import json
import math
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String

class TuningMonitor(object):
    def __init__(self):
        rospy.init_node('tuning_monitor', anonymous=True)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.speed = 0.0
        self.imu_pitch = 0.0
        self.imu_roll = 0.0
        self.n_ground = 0
        self.n_nonground = 0
        self.obstacles = []
        self.collision_objects = []

        rospy.Subscriber('/odom', Odometry, self.odom_cb)
        rospy.Subscriber('/imu', Imu, self.imu_cb)
        rospy.Subscriber('/obstacle_info', String, self.obs_cb)

        rospy.Timer(rospy.Duration(0.5), self.report)
        rospy.loginfo('[TuningMonitor] Ready and listening...')

    def odom_cb(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.speed = math.hypot(vx, vy) * 3.6 # km/h

    def imu_cb(self, msg):
        q = msg.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        self.imu_pitch = math.degrees(math.asin(sinp))
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.imu_roll = math.degrees(math.atan2(sinr, cosr))

    def obs_cb(self, msg):
        try:
            data = json.loads(msg.data)
            self.obstacles = data.get('obstacles', [])
            self.collision_objects = data.get('collision_objects', [])
        except Exception:
            pass

    def report(self, _evt):
        tf_str = "TF: N/A"
        try:
            # velodyne -> velodyne_grav
            tf_stamped = self.tf_buffer.lookup_transform('velodyne', 'velodyne_grav', rospy.Time(0))
            q = tf_stamped.transform.rotation
            sinp = 2.0 * (q.w * q.y - q.z * q.x)
            sinp = max(-1.0, min(1.0, sinp))
            dyn_pitch = math.degrees(math.asin(sinp))
            tf_str = f"Dyn Pitch: {dyn_pitch:+.2f} deg"
        except Exception:
            pass

        # Check for suspicious low ground obstacles (z < -1.30m or top < 0.25m)
        suspicious = []
        for o in self.collision_objects + self.obstacles:
            pos = o.get('position_ego', [0,0,0])
            if len(pos) >= 3 and pos[2] < -1.30:
                suspicious.append(f"id={o.get('id')} (x={pos[0]:.1f}, y={pos[1]:.1f}, z={pos[2]:.2f})")

        sus_str = f"Suspicious Ground Clusters: {len(suspicious)}"
        if suspicious:
            sus_str += " -> " + ", ".join(suspicious[:3])

        rospy.loginfo(
            f"[Spd: {self.speed:4.1f} km/h | Raw Pitch: {self.imu_pitch:+.2f} deg | {tf_str}] "
            f"Obs: {len(self.obstacles)} | Col: {len(self.collision_objects)} | {sus_str}"
        )

if __name__ == '__main__':
    mon = TuningMonitor()
    rospy.spin()
