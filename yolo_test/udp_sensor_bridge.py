#!/usr/bin/env python3
import socket
import struct
import time
from threading import Event, Lock, Thread

import cv2
import numpy as np
import pynmea2
import rospy
from morai_msgs.msg import GPSMessage
from sensor_msgs.msg import CompressedImage, Imu


class CamUDPReceiver:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None
        self.event = Event()
        self.lock = Lock()
        self.latest_jpeg = None

    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(1)
        self.sock.bind((self.host, self.port))
        Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self.event.set()
        if self.sock:
            self.sock.close()

    def _loop(self):
        while not self.event.is_set():
            self._receive_one_frame()

    def _receive_one_frame(self):
        total_buffer = b''
        while not self.event.is_set():
            try:
                unit_block, _ = self.sock.recvfrom(65000)
                # 같은 포트로 카메라 프레임(헤더 'MOR', tail 'EI') 말고도
                # 장애물이 화면에 잡히면 별도의 오브젝트 박스 스트림(헤더 'BOX', tail 'EO')이
                # 섞여 들어온다. 타입 구분 없이 tail=='EI'까지 이어붙이면 BOX 패킷이 낀
                # MOR 프레임끼리 합쳐져서 깨진 JPEG가 만들어짐 -> MOR이 아니면 무시.
                if unit_block[:3] != b'MOR':
                    continue
                unit_data = unit_block[19:-2]
                unit_tail = unit_block[-2:]
                total_buffer += unit_data
                if unit_tail == b'EI':
                    # 패킷이 고정 크기(65000B)라 실제 JPEG보다 크면 뒤에 0으로
                    # 패딩된다. 그래서 EOI(\xff\xd9)를 뒤에서부터 찾아 그 지점까지만
                    # 잘라내야 한다 (버퍼 끝이 곧 EOI라고 가정하면 안 됨).
                    eoi_idx = total_buffer.rfind(b'\xff\xd9')
                    if total_buffer[:2] == b'\xff\xd8' and eoi_idx != -1:
                        with self.lock:
                            self.latest_jpeg = total_buffer[:eoi_idx + 2]
                    else:
                        rospy.logwarn_throttle(1.0, '[camera] invalid jpeg frame, dropped (len=%d)' % len(total_buffer))
                    return
            except socket.timeout:
                return
            except Exception as e:
                rospy.logwarn(f'[camera] recv error: {e}')
                return

    def get_jpeg(self):
        with self.lock:
            return self.latest_jpeg


class ImuUDPReceiver:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None
        self.event = Event()
        self.lock = Lock()
        self.data = None

    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(1)
        self.sock.bind((self.host, self.port))
        Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self.event.set()
        if self.sock:
            self.sock.close()

    def _loop(self):
        while not self.event.is_set():
            try:
                raw_data, _ = self.sock.recvfrom(65535)
                header = raw_data[0:9].decode()
                if header == '#IMUData$':
                    values = struct.unpack('10d', raw_data[33:113])
                    with self.lock:
                        # values = (qw, qx, qy, qz, wx, wy, wz, ax, ay, az)
                        self.data = values
            except socket.timeout:
                continue
            except Exception as e:
                rospy.logwarn(f'[imu] recv error: {e}')

    def get_data(self):
        with self.lock:
            return self.data


class GpsUDPReceiver:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None
        self.event = Event()
        self.lock = Lock()
        self.lat = None
        self.lon = None

    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(1)
        self.sock.bind((self.host, self.port))
        Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self.event.set()
        if self.sock:
            self.sock.close()

    def _loop(self):
        while not self.event.is_set():
            try:
                datas, _ = self.sock.recvfrom(8196)
                ascii_datas = datas.decode('ascii', errors='ignore')
                for line in ascii_datas.split('\r\n'):
                    if not line:
                        continue
                    try:
                        gpgga = pynmea2.parse(line)
                    except pynmea2.ParseError:
                        continue
                    if hasattr(gpgga, 'latitude') and hasattr(gpgga, 'longitude'):
                        with self.lock:
                            self.lat = gpgga.latitude
                            self.lon = gpgga.longitude
                        break
            except socket.timeout:
                continue
            except Exception as e:
                rospy.logwarn(f'[gps] recv error: {e}')

    def get_pose(self):
        with self.lock:
            return self.lat, self.lon


def main():
    rospy.init_node('udp_sensor_bridge')

    bind_host = rospy.get_param('~bind_host', '0.0.0.0')
    cam_port = rospy.get_param('~cam_port', 1232)
    imu_port = rospy.get_param('~imu_port', 1236)
    gps_port = rospy.get_param('~gps_port', 1234)
    rate_hz = rospy.get_param('~rate', 30)

    cam_pub = rospy.Publisher('/image_jpeg/compressed', CompressedImage, queue_size=1)
    imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)
    gps_pub = rospy.Publisher('/gps', GPSMessage, queue_size=10)

    cam_rx = CamUDPReceiver(bind_host, cam_port)
    imu_rx = ImuUDPReceiver(bind_host, imu_port)
    gps_rx = GpsUDPReceiver(bind_host, gps_port)

    cam_rx.start()
    imu_rx.start()
    gps_rx.start()
    rospy.loginfo(f'udp_sensor_bridge listening: cam={cam_port} imu={imu_port} gps={gps_port}')

    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        now = rospy.Time.now()

        jpeg = cam_rx.get_jpeg()
        if jpeg is not None:
            msg = CompressedImage()
            msg.header.stamp = now
            msg.format = 'jpeg'
            msg.data = jpeg
            cam_pub.publish(msg)

        imu_vals = imu_rx.get_data()
        if imu_vals is not None:
            qw, qx, qy, qz, wx, wy, wz, ax, ay, az = imu_vals
            msg = Imu()
            msg.header.stamp = now
            msg.header.frame_id = 'imu'
            msg.orientation.w = qw
            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz
            msg.angular_velocity.x = wx
            msg.angular_velocity.y = wy
            msg.angular_velocity.z = wz
            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az
            imu_pub.publish(msg)

        lat, lon = gps_rx.get_pose()
        if lat is not None:
            msg = GPSMessage()
            msg.header.stamp = now
            msg.latitude = lat
            msg.longitude = lon
            gps_pub.publish(msg)

        rate.sleep()

    cam_rx.stop()
    imu_rx.stop()
    gps_rx.stop()


if __name__ == '__main__':
    main()
