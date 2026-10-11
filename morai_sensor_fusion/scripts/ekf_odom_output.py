#!/usr/bin/env python3
"""Assemble Autoware EKF pose/twist outputs into canonical Odometry."""

import math
import threading

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class EkfOdomOutput:
    def __init__(self):
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.child_frame = rospy.get_param('~child_frame', 'base_link')
        self.output_timeout = float(rospy.get_param('~output_timeout', 0.20))
        self.max_position_variance = float(rospy.get_param('~max_position_variance', 25.0))
        self.always_publish_tf = bool(rospy.get_param('~always_publish_tf', True))
        self.lock = threading.RLock()
        self.pose = None
        self.twist = None
        self.pose_rx = None
        self.twist_rx = None
        self.sensor_valid = False

        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=5)
        self.fused_pub = rospy.Publisher('/odom_fused', Odometry, queue_size=5)
        self.valid_pub = rospy.Publisher('/localization/valid', Bool, queue_size=1, latch=True)
        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber('/ekf_pose_with_covariance', PoseWithCovarianceStamped,
                         self.pose_callback, queue_size=5)
        rospy.Subscriber('/ekf_twist_with_covariance', TwistWithCovarianceStamped,
                         self.twist_callback, queue_size=5)
        rospy.Subscriber('/localization/sensors_valid', Bool,
                         self.sensor_callback, queue_size=2)
        rate = float(rospy.get_param('~rate', 50.0))
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(rate, 1.0)), self.timer_callback)

    def pose_callback(self, message):
        with self.lock:
            self.pose = message
            self.pose_rx = rospy.Time.now()

    def twist_callback(self, message):
        with self.lock:
            self.twist = message
            self.twist_rx = rospy.Time.now()

    def sensor_callback(self, message):
        with self.lock:
            self.sensor_valid = bool(message.data)

    @staticmethod
    def finite_odometry(pose, twist):
        values = [pose.pose.pose.position.x, pose.pose.pose.position.y,
                  pose.pose.pose.orientation.x, pose.pose.pose.orientation.y,
                  pose.pose.pose.orientation.z, pose.pose.pose.orientation.w,
                  twist.twist.twist.linear.x, twist.twist.twist.angular.z,
                  pose.pose.covariance[0], pose.pose.covariance[7]]
        return all(math.isfinite(value) for value in values)

    def timer_callback(self, _event):
        now = rospy.Time.now()
        with self.lock:
            pose, twist = self.pose, self.twist
            pose_rx, twist_rx = self.pose_rx, self.twist_rx
            sensors = self.sensor_valid
        ready = pose is not None and twist is not None
        fresh = (ready and pose_rx is not None and twist_rx is not None
                 and (now - pose_rx).to_sec() <= self.output_timeout
                 and (now - twist_rx).to_sec() <= self.output_timeout)
        covariance_ok = (ready and pose.pose.covariance[0] <= self.max_position_variance
                         and pose.pose.covariance[7] <= self.max_position_variance)
        valid = bool(ready and fresh and sensors and covariance_ok
                     and self.finite_odometry(pose, twist))
        self.valid_pub.publish(Bool(data=valid))
        if not ready or not self.finite_odometry(pose, twist):
            return

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame
        odom.pose = pose.pose
        odom.twist = twist.twist
        self.odom_pub.publish(odom)
        self.fused_pub.publish(odom)
        if self.always_publish_tf or valid:
            p = odom.pose.pose.position
            q = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(
                (p.x, p.y, p.z), (q.x, q.y, q.z, q.w), now,
                self.child_frame, self.frame_id)


def main():
    rospy.init_node('ekf_odom_output')
    EkfOdomOutput()
    rospy.loginfo('[ekf_odom_output] publishing /odom_fused, /odom, localization validity and TF')
    rospy.spin()


if __name__ == '__main__':
    main()
