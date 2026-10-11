#!/usr/bin/env python3
"""
카메라 이미지(/image_jpeg/compressed)를 화면에 띄우고,
키보드 입력으로 이미지를 캡처/저장하는 노드. (수동 라벨링용 데이터 수집)

조작법:
  스페이스바 : 현재 프레임 저장
  a          : 자동 저장 모드 ON/OFF (켜면 --interval 초마다 자동 저장)
  q          : 종료
"""

import rospy
import numpy as np
import cv2
import os
import time
import argparse

from sensor_msgs.msg import CompressedImage


class ImageCapturer:
    def __init__(self, save_dir='captured_images', auto_interval=1.0):
        rospy.init_node('image_capturer', anonymous=True)

        self.save_dir = save_dir
        os.makedirs(self.save_dir, exist_ok=True)

        self.auto_interval = auto_interval   # 자동 저장 모드일 때 저장 간격(초)
        self.auto_mode = False
        self.last_auto_save = 0.0

        self.latest_img = None
        self.save_count = 0

        rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.image_callback,
                          queue_size=1, buff_size=2**24, tcp_nodelay=True)

        rospy.loginfo("Image Capturer started. save_dir=%s" % os.path.abspath(self.save_dir))
        rospy.loginfo("SPACE=저장  a=자동저장 토글  q=종료")

        self.run()

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is not None:
            self.latest_img = img

    def save_image(self, img):
        timestamp = time.strftime("%Y%m%d_%H%M%S") + ("_%03d" % (int(time.time() * 1000) % 1000))
        filename = os.path.join(self.save_dir, "img_%s.jpg" % timestamp)
        cv2.imwrite(filename, img)
        self.save_count += 1
        rospy.loginfo("[SAVED %d] %s" % (self.save_count, filename))

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.latest_img is not None:
                display_img = self.latest_img.copy()

                # 화면에 상태 표시 (자동모드 여부, 저장 개수)
                status_text = "AUTO ON" if self.auto_mode else "AUTO OFF"
                cv2.putText(display_img, "%s | saved: %d" % (status_text, self.save_count),
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                cv2.imshow("Image Capturer", display_img)
                key = cv2.waitKey(1) & 0xFF

                if key == ord(' '):
                    self.save_image(self.latest_img)
                elif key == ord('a'):
                    self.auto_mode = not self.auto_mode
                    rospy.loginfo("Auto mode: %s" % self.auto_mode)
                elif key == ord('q'):
                    rospy.loginfo("Exiting. Total saved: %d" % self.save_count)
                    break

                if self.auto_mode:
                    now = time.time()
                    if now - self.last_auto_save >= self.auto_interval:
                        self.save_image(self.latest_img)
                        self.last_auto_save = now

            rate.sleep()

        cv2.destroyAllWindows()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--save_dir', type=str, default='captured_images',
                         help='이미지 저장 폴더')
    parser.add_argument('--interval', type=float, default=1.0,
                         help='자동 저장 모드일 때 저장 간격(초)')
    # rospy가 실행 시 넘기는 인자와 충돌 방지 위해 known args만 파싱
    args, _ = parser.parse_known_args()

    try:
        ImageCapturer(save_dir=args.save_dir, auto_interval=args.interval)
    except rospy.ROSInterruptException:
        pass
