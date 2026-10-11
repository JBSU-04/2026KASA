#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""전방 신호등 정지선 접근 구간의 MORAI 카메라 원본 이미지 수집."""

import os
import threading

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray


class TrafficLightDatasetCollector(object):
    def __init__(self):
        rospy.init_node("traffic_light_dataset_collector")
        self.image_topic = rospy.get_param(
            "~image_topic", "/front/image_jpeg/compressed")
        self.stop_topic = rospy.get_param(
            "~stop_topic", "/localization/stop_ahead")
        self.output_dir = os.path.expanduser(rospy.get_param(
            "~output_dir", "~/Downloads/kcity_traffic_light"))
        self.max_dist = float(rospy.get_param("~max_dist", 60.0))
        self.save_fps = float(rospy.get_param("~save_fps", 2.0))
        self.jpeg_quality = int(rospy.get_param("~jpeg_quality", 95))

        os.makedirs(self.output_dir, exist_ok=True)
        self.stop_dist = -1.0
        self.last_save = 0.0
        self.saved = 0
        self.lock = threading.Lock()

        rospy.Subscriber(self.stop_topic, Float32MultiArray,
                         self.stop_cb, queue_size=1)
        rospy.Subscriber(self.image_topic, CompressedImage,
                         self.image_cb, queue_size=1, buff_size=2 ** 22)
        rospy.loginfo("[tl_dataset] collecting 0..%.1fm at %.1f fps -> %s",
                      self.max_dist, self.save_fps, self.output_dir)

    def stop_cb(self, msg):
        with self.lock:
            self.stop_dist = float(msg.data[0]) if msg.data else -1.0

    def image_cb(self, msg):
        now = rospy.Time.now().to_sec()
        with self.lock:
            dist = self.stop_dist
            if not (0.0 <= dist <= self.max_dist):
                return
            if now - self.last_save < 1.0 / max(self.save_fps, 0.1):
                return
            self.last_save = now

        image = cv2.imdecode(np.frombuffer(msg.data, np.uint8),
                             cv2.IMREAD_COLOR)
        if image is None:
            rospy.logwarn_throttle(5.0, "[tl_dataset] JPEG decode failed")
            return

        stamp_ns = msg.header.stamp.to_nsec()
        if stamp_ns <= 0:
            stamp_ns = int(now * 1000000000)
        filename = "%019d_d%05.1fm.jpg" % (stamp_ns, dist)
        path = os.path.join(self.output_dir, filename)
        ok = cv2.imwrite(path, image,
                         [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if ok:
            self.saved += 1
            rospy.loginfo_throttle(
                5.0, "[tl_dataset] saved=%d dist=%.1fm -> %s",
                self.saved, dist, self.output_dir)
        else:
            rospy.logerr_throttle(5.0, "[tl_dataset] save failed: %s", path)


if __name__ == "__main__":
    try:
        TrafficLightDatasetCollector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
