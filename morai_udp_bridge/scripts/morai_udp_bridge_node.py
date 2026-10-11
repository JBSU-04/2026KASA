#!/usr/bin/env python3
"""ROS1 bridge between MORAI 25.S4 UDP channels and existing ROS topics."""

import math
import socket
import threading
import time
from typing import Callable, Dict, Optional

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from morai_msgs.msg import CollisionData, CtrlCmd, EgoVehicleStatus, GPSMessage, ObjectStatus
from sensor_msgs.msg import CompressedImage, Imu
from std_msgs.msg import String, UInt8

from morai_udp_bridge.protocol import (
    CameraFrameAssembler,
    pack_ctrl_packet,
    parse_collision_packet,
    parse_competition_packet,
    parse_imu_packet_with_stamp,
    parse_nmea_datagram,
)


class UdpReceiver:
    def __init__(self, name: str, bind_ip: str, port: int,
                 callback: Callable[[bytes], None], allowed_ip: str = ''):
        self.name = name
        self.bind_ip = bind_ip
        self.port = port
        self.callback = callback
        self.allowed_ip = allowed_ip
        self.socket: Optional[socket.socket] = None
        self.thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()

    def start(self) -> None:
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        self.socket.settimeout(0.25)
        self.socket.bind((self.bind_ip, self.port))
        self.thread = threading.Thread(target=self._run, name='udp_{}'.format(self.name), daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        if self.socket is not None:
            self.socket.close()
        if self.thread is not None:
            self.thread.join(timeout=1.0)

    def _run(self) -> None:
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            try:
                data, source = self.socket.recvfrom(65535)
                if self.allowed_ip and source[0] != self.allowed_ip:
                    rospy.logwarn_throttle(
                        2.0, '[morai_udp_bridge] %s packet rejected from %s',
                        self.name, source[0])
                    continue
                self.callback(data)
            except socket.timeout:
                continue
            except OSError as error:
                if not self.stop_event.is_set():
                    rospy.logwarn_throttle(2.0, '[morai_udp_bridge] %s socket: %s', self.name, error)
            except Exception as error:  # 수신 스레드는 손상된 한 패킷 때문에 종료되면 안 된다.
                rospy.logwarn_throttle(2.0, '[morai_udp_bridge] %s decode: %s', self.name, error)


class ControlSender:
    def __init__(self, node: 'MoraiUdpBridge'):
        self.node = node
        self.simulator_ip = rospy.get_param('~network/simulator_ip', '192.168.0.1')
        self.simulator_port = int(rospy.get_param('~ports/control/simulator', 9093))
        self.local_ip = rospy.get_param('~network/bind_ip', '0.0.0.0')
        self.local_port = int(rospy.get_param('~ports/control/local', 9094))
        self.aux_size = int(rospy.get_param('~protocol/network_aux_bytes', 12))
        # 런타임 토글 가능하게 둔다. UDP 는 매 패킷에 CtrlMode 를 실으므로
        # 이 값을 고정하면 시뮬레이터의 키보드 조작이 계속 덮인다.
        #   rosparam set /morai_udp_bridge/control/ctrl_mode 1  -> 키보드
        #   rosparam set /morai_udp_bridge/control/ctrl_mode 2  -> 자율
        # 대회 주행 중에는 규정 2-8에 따라 2를 유지한다.
        self.ctrl_mode = int(rospy.get_param('~control/ctrl_mode', 2))
        self._mode_refresh_sec = float(
            rospy.get_param('~control/mode_refresh_sec', 0.2))
        self._mode_checked = 0.0
        self.gear = int(rospy.get_param('~control/gear', 4))
        # ★ 2026-10-01 후진 탈출: control_arbiter 가 /control/gear 로 기어를 요청한다.
        #   요청이 gear_timeout 이상 끊기면 기본 기어(D)로 돌아간다. 허용: 1 P, 2 R, 3 N, 4 D.
        self.gear_timeout = float(rospy.get_param('~control/gear_timeout', 0.3))
        self.gear_request = None
        self.gear_request_time = 0.0
        self.max_wheel_angle_rad = math.radians(
            float(rospy.get_param('~control/max_wheel_angle_deg', 40.0))
        )
        self.timeout = float(rospy.get_param('~control/command_timeout', 0.3))
        self.stale_brake = float(rospy.get_param('~control/stale_brake', 0.7))
        self.send_before_first = bool(rospy.get_param('~control/send_before_first_cmd', False))
        self.lock = threading.Lock()
        self.latest: Optional[CtrlCmd] = None
        self.latest_time = 0.0
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.local_ip, self.local_port))

        topic = rospy.get_param('~topics/control_command', '/ctrl_cmd')
        self.subscriber = rospy.Subscriber(topic, CtrlCmd, self._command_callback, queue_size=1)
        self.gear_subscriber = rospy.Subscriber(
            rospy.get_param('~topics/gear_request', '/control/gear'), UInt8,
            self._gear_callback, queue_size=1)
        send_rate = float(rospy.get_param('~control/send_rate', 50.0))
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(1.0, send_rate)), self._timer_callback)

    def close(self) -> None:
        self.timer.shutdown()
        self.socket.close()

    def _command_callback(self, message: CtrlCmd) -> None:
        with self.lock:
            self.latest = message
            self.latest_time = time.monotonic()

    def _gear_callback(self, message: UInt8) -> None:
        if int(message.data) not in (1, 2, 3, 4):
            return
        with self.lock:
            self.gear_request = int(message.data)
            self.gear_request_time = time.monotonic()

    def _current_gear(self) -> int:
        with self.lock:
            fresh = (self.gear_request is not None and
                     time.monotonic() - self.gear_request_time <= self.gear_timeout)
            gear = self.gear_request if fresh else self.gear
        return gear

    def _refresh_mode(self) -> None:
        now = time.monotonic()
        if (now - self._mode_checked) < self._mode_refresh_sec:
            return
        self._mode_checked = now
        try:
            mode = int(rospy.get_param('~control/ctrl_mode', self.ctrl_mode))
        except Exception:
            return
        if mode != self.ctrl_mode:
            rospy.logwarn('[morai_udp_bridge] CtrlMode %d -> %d (%s)',
                          self.ctrl_mode, mode,
                          'KeyBoard' if mode == 1 else 'AutoMode')
            self.ctrl_mode = mode

    def _timer_callback(self, _event) -> None:
        self._refresh_mode()
        with self.lock:
            message = self.latest
            age = time.monotonic() - self.latest_time
        stale = message is None or age > self.timeout
        if message is None and not self.send_before_first:
            return

        if stale:
            values = dict(
                longl_cmd_type=1,
                accel=0.0,
                brake=self.stale_brake,
                steering_rad=0.0,
                velocity_kph=0.0,
                acceleration_mps2=0.0,
            )
        else:
            values = dict(
                longl_cmd_type=message.longlCmdType,
                accel=message.accel,
                brake=message.brake,
                steering_rad=message.steering,
                velocity_kph=message.velocity,
                acceleration_mps2=message.acceleration,
            )
        if self.ctrl_mode == 1:
            # 수동(키보드) 모드에서는 우리 명령이 무시되지만, 전환 순간
            # 직전 가속이 남지 않도록 페달을 비운다.
            values.update(accel=0.0, brake=0.0, steering_rad=0.0,
                          velocity_kph=0.0, acceleration_mps2=0.0)

        packet = pack_ctrl_packet(
            **values,
            max_wheel_angle_rad=self.max_wheel_angle_rad,
            ctrl_mode=self.ctrl_mode,
            gear=self._current_gear(),
            aux_size=self.aux_size,
        )
        try:
            self.socket.sendto(packet, (self.simulator_ip, self.simulator_port))
            self.node.record('control_tx', stale=stale)
        except OSError as error:
            self.node.record_error('control_tx')
            rospy.logwarn_throttle(2.0, '[morai_udp_bridge] control send: %s', error)


class MoraiUdpBridge:
    def __init__(self):
        self.bind_ip = rospy.get_param('~network/bind_ip', '0.0.0.0')
        self.simulator_ip = rospy.get_param('~network/simulator_ip', '192.168.0.1')
        self.stats_lock = threading.Lock()
        self.stats: Dict[str, Dict[str, object]] = {}
        self.receivers = []
        self.control: Optional[ControlSender] = None
        self.camera_assembler = CameraFrameAssembler(
            int(rospy.get_param('~camera/max_frame_bytes', 8 * 1024 * 1024))
        )
        # 보조 카메라는 반드시 자기 assembler 를 가져야 한다. assembler 는
        # JPEG 조각을 인스턴스 버퍼(_buffer)에 쌓아 EI 꼬리에서 한 장으로
        # 맺는데, 두 카메라가 하나를 공유하면 서로의 조각이 섞여 두 스트림이
        # 같이 깨진다.
        self.camera2_assembler = CameraFrameAssembler(
            int(rospy.get_param('~camera/max_frame_bytes', 8 * 1024 * 1024))
        )
        # MORAI 25.S4는 GPS 한 fix를 RMC와 GGA 두 datagram으로 따로 보낸다
        # (1ms 미만 간격, lat/lon 동일). 둘 다 발행하면 20Hz 센서가 40Hz로
        # 보이고 EKF가 같은 측정을 독립 관측 2회로 반영해 위치를 과신한다.
        # GGA를 한 번이라도 받으면 그 뒤로는 RMC만 든 datagram을 버린다.
        self.gps_dedupe = bool(rospy.get_param('~gps/dedupe_duplicate_sentence', True))
        self.gps_gga_seen = False
        self.gps_duplicate_skipped = 0

        self.gps_pub = rospy.Publisher(
            rospy.get_param('~topics/gps', '/gps'), GPSMessage, queue_size=20
        )
        self.imu_pub = rospy.Publisher(
            rospy.get_param('~topics/imu', '/imu'), Imu, queue_size=50
        )
        self.camera_pub = rospy.Publisher(
            rospy.get_param('~topics/camera', '/image_jpeg/compressed'),
            CompressedImage,
            queue_size=2,
        )
        self.camera2_pub = rospy.Publisher(
            rospy.get_param('~topics/camera2', '/image_jpeg2/compressed'),
            CompressedImage,
            queue_size=2,
        )
        self.competition_pub = rospy.Publisher(
            rospy.get_param('~topics/competition_status', '/Competition_topic'),
            EgoVehicleStatus,
            queue_size=50,
        )
        self.link_id_pub = rospy.Publisher(
            rospy.get_param('~topics/current_link_id', '/morai/current_link_id'),
            String,
            queue_size=10,
        )
        # 현재 기어(UDP 상태 패킷). EgoVehicleStatus 에 기어 필드가 없어 따로 낸다.
        self.gear_pub = rospy.Publisher(
            rospy.get_param('~topics/current_gear', '/morai/gear'), UInt8, queue_size=10)
        self.collision_pub = rospy.Publisher(
            rospy.get_param('~topics/collision', '/CollisionData'),
            CollisionData,
            queue_size=10,
        )
        self.diagnostics_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)

        self.gps_frame = rospy.get_param('~frames/gps', 'gps')
        self.imu_frame = rospy.get_param('~frames/imu', 'imu')
        self.camera_frame = rospy.get_param('~frames/camera', 'camera')
        self.camera2_frame = rospy.get_param('~frames/camera2', 'camera2')
        self.ego_frame = rospy.get_param('~frames/ego', 'base_link')
        self.gps_east_offset = float(rospy.get_param('~gps/east_offset', 0.0))
        self.gps_north_offset = float(rospy.get_param('~gps/north_offset', 0.0))
        self.imu_orientation_covariance = float(
            rospy.get_param('~imu/orientation_covariance', 0.0025)
        )
        self.imu_angular_velocity_covariance = float(
            rospy.get_param('~imu/angular_velocity_covariance', 0.0004)
        )
        self.imu_linear_acceleration_covariance = float(
            rospy.get_param('~imu/linear_acceleration_covariance', 0.01)
        )
        self.imu_drop_non_increasing_stamp = bool(
            rospy.get_param('~imu/drop_non_increasing_stamp', True)
        )
        self.imu_min_stamp_step = float(rospy.get_param('~imu/min_stamp_step', 0.01))
        self.imu_last_stamp = None
        # MORAI PC 시계를 ROS PC 시계로 옮기는 보정. 자세한 근거는 _stamp 참조.
        # 한 대에서 돌릴 때는 offset 이 0 근처라 켜 두어도 해가 없다.
        self.clock_sync = bool(rospy.get_param('~clock_sync', True))
        self.clock_sync_alpha = float(rospy.get_param('~clock_sync_alpha', 0.001))
        self.clock_offset = None
        self.imu_duplicate_skipped = 0

        self._configure_channels()
        self.diagnostic_timer = rospy.Timer(rospy.Duration(2.0), self._publish_diagnostics)

    def _configure_channels(self) -> None:
        channel_specs = [
            ('competition', 'competition', 9099, self._competition_callback),
            ('collision', 'collision', 9092, self._collision_callback),
            ('gps', 'gps', 1234, self._gps_callback),
            ('imu', 'imu', 1236, self._imu_callback),
            ('camera', 'camera', 1232, self._camera_callback),
            ('camera2', 'camera2', 1238, self._camera2_callback),
        ]
        for channel, port_key, default_port, callback in channel_specs:
            if not bool(rospy.get_param('~enable/{}'.format(channel),
                                        not channel.startswith('camera'))):
                continue
            port = int(rospy.get_param('~ports/{}/listen'.format(port_key), default_port))
            allowed_ip = '' if self.simulator_ip in ('', '0.0.0.0') else self.simulator_ip
            receiver = UdpReceiver(channel, self.bind_ip, port, callback, allowed_ip)
            receiver.start()
            self.receivers.append(receiver)
            self.record(channel, enabled=True)
            rospy.loginfo('[morai_udp_bridge] %s UDP listening on %s:%d', channel, self.bind_ip, port)

        if bool(rospy.get_param('~enable/control', True)):
            self.control = ControlSender(self)
            rospy.loginfo(
                '[morai_udp_bridge] /ctrl_cmd UDP -> %s:%d (wheel limit %.1f deg)',
                self.control.simulator_ip,
                self.control.simulator_port,
                math.degrees(self.control.max_wheel_angle_rad),
            )
        else:
            rospy.loginfo(
                '[morai_udp_bridge] /ctrl_cmd UDP sender disabled; use ROS Bridge websocket '
                'for MORAI Ctrl Cmd'
            )

    def shutdown(self) -> None:
        self.diagnostic_timer.shutdown()
        if self.control is not None:
            self.control.close()
        for receiver in self.receivers:
            receiver.stop()

    def record(self, channel: str, **values) -> None:
        with self.stats_lock:
            status = self.stats.setdefault(channel, {'count': 0, 'errors': 0, 'last': 0.0})
            status['count'] = int(status['count']) + 1
            status['last'] = time.monotonic()
            status.update(values)

    def record_error(self, channel: str) -> None:
        with self.stats_lock:
            status = self.stats.setdefault(channel, {'count': 0, 'errors': 0, 'last': 0.0})
            status['errors'] = int(status['errors']) + 1

    def _stamp(self, secs: int = 0, nsecs: int = 0):
        """MORAI 패킷 시각을 ROS PC 시계로 옮겨서 돌려준다.

        왜 필요한가 - 두 PC 를 쓰면 stamp 가 섞인다.
          GPS 는 NMEA 에 쓸 만한 시각이 없어 수신시각(rospy.Time.now(), ROS PC
          시계)을 쓰고, IMU/Competition 은 패킷 안의 MORAI PC 시계를 쓴다.
          한 대에서 돌릴 때는 같은 시계라 문제가 없었다. 대회장처럼 PC 가
          둘이면 두 시계가 그대로 어긋난 채 EKF 로 들어간다.

          실측(2026-09-18 15:47 대회장 백, 250 s):
            /gps      stamp - 수신시각 중앙   -0.5 ms  (ROS 시계)
            /imu      stamp - 수신시각 중앙 +1077.3 ms (MORAI 시계)
          EKF 는 pose(GPS)와 twist(IMU)를 1.078 s 어긋난 상태로 융합했고,
          9.85 m/s 에서 이는 10.6 m 짜리 불일치다. 같은 백에서 /odom 헤딩
          오차가 std 1.633 deg, 28% 가 2 deg 초과, 최대 5.18 deg 였다
          (집에서는 0.05 deg 수준).

          게다가 MORAI 시계가 '앞서' 있으면 ekf_localizer 의
          delay_time = t_curr - stamp 가 음수가 되어 0 으로 잘린다
          (ekf_localizer.cpp:468, 614). 지연보상이 아예 동작하지 않는다.

        어떻게 고치는가.
          수신시각 - 패킷시각 의 최솟값을 시계 offset 으로 본다. 네트워크가
          가장 한가했던 순간에는 전송지연이 0 에 가까우므로 그때의 차이가
          순수한 시계차에 가장 가깝다(comm_monitor 와 같은 원리). 최솟값은
          한 번 잡히면 잘 안 변하므로 느린 드리프트에는 EMA 로 따라간다.
          보정 후에는 stamp 가 ROS 시계 기준이 되어 실제 전송지연만 남고,
          EKF 의 지연보상이 그 값으로 정상 동작한다.

          clock_sync 를 false 로 두면 기존(원본 stamp) 동작 그대로다.
        """
        if not (secs > 0 and 0 <= nsecs < 1_000_000_000):
            return rospy.Time.now()
        if not self.clock_sync:
            return rospy.Time(secs, nsecs)

        now = rospy.Time.now()
        packet_sec = secs + nsecs * 1e-9
        diff = now.to_sec() - packet_sec
        if self.clock_offset is None:
            self.clock_offset = diff
        elif diff < self.clock_offset:
            # 더 작은 지연을 봤다 = 더 나은 offset 추정치. 즉시 받아들인다.
            self.clock_offset = diff
        else:
            # 시계 드리프트를 아주 천천히 따라간다. 튀는 지연에 끌려가지
            # 않도록 계수를 작게 둔다.
            self.clock_offset += self.clock_sync_alpha * (diff - self.clock_offset)

        corrected = packet_sec + self.clock_offset
        # 보정 결과가 미래로 가지 않도록 막는다. 미래 stamp 는 EKF 에서
        # delay_time 음수 -> 0 클램프를 다시 유발한다.
        if corrected > now.to_sec():
            return now
        return rospy.Time.from_sec(corrected)

    def _competition_callback(self, data: bytes) -> None:
        try:
            state = parse_competition_packet(data)
            message = EgoVehicleStatus()
            message.header.stamp = self._stamp(state['secs'], state['nsecs'])
            message.header.frame_id = self.ego_frame
            message.unique_id = 0
            velocity = state['velocity_kph']
            message.velocity.x = velocity[0] / 3.6
            message.velocity.y = velocity[1] / 3.6
            message.velocity.z = velocity[2] / 3.6
            acceleration = state['acceleration_mps2']
            message.acceleration.x, message.acceleration.y, message.acceleration.z = acceleration
            angular_velocity = state['angular_velocity_deg_s']
            message.angular_velocity.x = math.radians(angular_velocity[0])
            message.angular_velocity.y = math.radians(angular_velocity[1])
            message.angular_velocity.z = math.radians(angular_velocity[2])
            message.heading = state['orientation_deg'][2]
            message.accel = state['accel']
            message.brake = state['brake']
            message.wheel_angle = state['wheel_angle_deg']
            message.front_steer_angle = state['wheel_angle_deg']
            self.competition_pub.publish(message)
            self.link_id_pub.publish(String(data=state['link_id']))
            self.gear_pub.publish(UInt8(data=int(state['gear']) & 0xFF))
            self.record('competition')
        except (ValueError, TypeError) as error:
            self.record_error('competition')
            rospy.logwarn_throttle(2.0, '[morai_udp_bridge] competition packet: %s', error)

    def _collision_callback(self, data: bytes) -> None:
        try:
            state = parse_collision_packet(data)
            message = CollisionData()
            message.header.stamp = self._stamp(state['secs'], state['nsecs'])
            message.header.frame_id = self.ego_frame
            objects = state['objects']
            if objects:
                offset = objects[0]['global_offset']
                message.global_offset_x, message.global_offset_y, message.global_offset_z = offset
            for item in objects:
                collision = ObjectStatus()
                collision.unique_id = item['unique_id']
                collision.type = item['type']
                collision.position.x, collision.position.y, collision.position.z = item['position']
                message.collision_object.append(collision)
            self.collision_pub.publish(message)
            self.record('collision')
        except (ValueError, TypeError) as error:
            self.record_error('collision')
            rospy.logwarn_throttle(2.0, '[morai_udp_bridge] collision packet: %s', error)

    def _gps_callback(self, data: bytes) -> None:
        fix = parse_nmea_datagram(data)
        if fix is None:
            self.record_error('gps')
            return
        if self.gps_dedupe:
            if fix.get('sentence') == 'GGA':
                if not self.gps_gga_seen:
                    self.gps_gga_seen = True
                    rospy.loginfo('[morai_udp_bridge] GPS GGA 확인 — 이후 중복 '
                                  'RMC datagram은 버린다 (EKF 이중 반영 방지)')
            elif self.gps_gga_seen:
                self.gps_duplicate_skipped += 1
                rospy.loginfo_throttle(
                    30.0, '[morai_udp_bridge] 중복 RMC %d개 무시 (GPS는 GGA만 발행)',
                    self.gps_duplicate_skipped)
                return
        message = GPSMessage()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.gps_frame
        message.latitude = fix['latitude']
        message.longitude = fix['longitude']
        message.altitude = fix['altitude']
        message.eastOffset = self.gps_east_offset
        message.northOffset = self.gps_north_offset
        message.status = fix['status']
        self.gps_pub.publish(message)
        self.record('gps')

    @staticmethod
    def _diagonal_covariance(value: float):
        return [value, 0.0, 0.0, 0.0, value, 0.0, 0.0, 0.0, value]

    def _imu_callback(self, data: bytes) -> None:
        try:
            stamp_tuple, values = parse_imu_packet_with_stamp(data)
            if stamp_tuple[0] > 0:
                if self.imu_last_stamp is not None:
                    stamp_sec = stamp_tuple[0] + 1e-9 * stamp_tuple[1]
                    last_sec = self.imu_last_stamp[0] + 1e-9 * self.imu_last_stamp[1]
                    stamp_step = stamp_sec - last_sec
                    if (self.imu_drop_non_increasing_stamp
                            and stamp_step < self.imu_min_stamp_step):
                        self.imu_duplicate_skipped += 1
                        rospy.loginfo_throttle(
                            30.0,
                            '[morai_udp_bridge] IMU burst %d개 무시 '
                            '(dt=%.4fs < %.4fs)',
                            self.imu_duplicate_skipped,
                            stamp_step,
                            self.imu_min_stamp_step)
                        return
                self.imu_last_stamp = stamp_tuple
            qw, qx, qy, qz, wx, wy, wz, ax, ay, az = values
            message = Imu()
            message.header.stamp = self._stamp(*stamp_tuple)
            message.header.frame_id = self.imu_frame
            message.orientation.w = qw
            message.orientation.x = qx
            message.orientation.y = qy
            message.orientation.z = qz
            message.angular_velocity.x = wx
            message.angular_velocity.y = wy
            message.angular_velocity.z = wz
            message.linear_acceleration.x = ax
            message.linear_acceleration.y = ay
            message.linear_acceleration.z = az
            message.orientation_covariance = self._diagonal_covariance(
                self.imu_orientation_covariance
            )
            message.angular_velocity_covariance = self._diagonal_covariance(
                self.imu_angular_velocity_covariance
            )
            message.linear_acceleration_covariance = self._diagonal_covariance(
                self.imu_linear_acceleration_covariance
            )
            self.imu_pub.publish(message)
            self.record('imu')
        except (ValueError, TypeError) as error:
            self.record_error('imu')
            rospy.logwarn_throttle(2.0, '[morai_udp_bridge] imu packet: %s', error)

    def _camera_callback(self, data: bytes) -> None:
        try:
            frame = self.camera_assembler.feed(data)
            if frame is None:
                return
            message = CompressedImage()
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = self.camera_frame
            message.format = 'jpeg'
            message.data = frame
            self.camera_pub.publish(message)
            self.record('camera')
        except ValueError as error:
            self.record_error('camera')
            rospy.logwarn_throttle(2.0, '[morai_udp_bridge] camera packet: %s', error)

    def _camera2_callback(self, data: bytes) -> None:
        try:
            frame = self.camera2_assembler.feed(data)
            if frame is None:
                return
            message = CompressedImage()
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = self.camera2_frame
            message.format = 'jpeg'
            message.data = frame
            self.camera2_pub.publish(message)
            self.record('camera2')
        except ValueError as error:
            self.record_error('camera2')
            rospy.logwarn_throttle(2.0, '[morai_udp_bridge] camera2 packet: %s', error)

    def _publish_diagnostics(self, _event) -> None:
        now = time.monotonic()
        with self.stats_lock:
            snapshot = {name: dict(values) for name, values in self.stats.items()}
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        for channel in sorted(snapshot):
            values = snapshot[channel]
            count = int(values.get('count', 0))
            errors = int(values.get('errors', 0))
            last = float(values.get('last', 0.0))
            age = now - last if last else float('inf')
            status = DiagnosticStatus()
            status.name = 'morai_udp_bridge/{}'.format(channel)
            status.hardware_id = 'MORAI-UDP'
            # Collision은 이벤트 토픽이라 무수신 상태가 정상이다.
            stale = channel not in ('collision',) and age > 1.0
            status.level = DiagnosticStatus.WARN if stale or errors else DiagnosticStatus.OK
            status.message = 'waiting for UDP' if stale else 'OK'
            status.values = [
                KeyValue(key='packets', value=str(count)),
                KeyValue(key='decode_errors', value=str(errors)),
                KeyValue(key='last_age_sec', value='inf' if not last else '{:.3f}'.format(age)),
            ]
            if 'stale' in values:
                status.values.append(KeyValue(key='control_watchdog', value=str(values['stale'])))
            array.status.append(status)
        self.diagnostics_pub.publish(array)


def main() -> None:
    rospy.init_node('morai_udp_bridge')
    bridge = MoraiUdpBridge()
    rospy.on_shutdown(bridge.shutdown)
    control_transport = 'UDP' if bool(rospy.get_param('~enable/control', False)) else 'ROSBridge'
    rospy.loginfo(
        '[morai_udp_bridge] ready; sensors/status use UDP, ctrl_cmd uses %s',
        control_transport,
    )
    rospy.spin()


if __name__ == '__main__':
    main()
