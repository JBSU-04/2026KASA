#!/usr/bin/env python3
"""
COCO 모델 + 커스텀 파인튜닝 모델을 합쳐서(교차 NMS 적용) 실시간 카메라에 적용하는 노드.
"""

import rospy
import numpy as np
import cv2

from sensor_msgs.msg import CompressedImage
from dual_model_detector import DualModelDetector


class DualModelTester:
    def __init__(self):
        rospy.init_node('dual_model_tester', anonymous=True)

        rospy.loginfo("Loading COCO + custom models...")
        self.detector = DualModelDetector(
            coco_weight='weights/yolo11s-seg.pt',
            custom_weight='weights/best.pt',
            coco_classes=[0, 1, 2, 3, 5, 7, 16],  # person, bicycle, car, motorcycle, bus, truck, dog
            iou_threshold=0.4,
            coco_conf=0.5,
            custom_conf=0.5,
            custom_exclude_names=['motorcycle'],
           
        )
        rospy.loginfo("Models loaded.")

        rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.image_callback,
                          queue_size=1, buff_size=2**24, tcp_nodelay=True)

        rospy.loginfo("Dual Model Tester started. Waiting for images...")
        rospy.spin()

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is None:
            return

        detections = self.detector.detect(img)
        annotated = self.detector.draw(img, detections)

        cv2.imshow("Dual Model Test", annotated)
        cv2.waitKey(1)


if __name__ == '__main__':
    try:
        DualModelTester()
    except rospy.ROSInterruptException:
        pass
