#!/usr/bin/env python3
"""MORAI UDP 통신 지연과 토픽 Hz 를 기록한다.

왜 필요한가.
  대회장에서는 주최측 PC 에서 MORAI 를 돌리고 우리 PC 로 UDP 를 받는다.
  집에서 쓰던 한 대짜리 구성과 달리 랜 구간이 끼므로, 문제가 생겼을 때
  "시뮬이 느린 건지, 랜이 느린 건지, 우리 노드가 밀린 건지"를 현장에서
  갈라야 한다. 백에 남겨두면 주행이 끝난 뒤에도 분석할 수 있다.

지연을 어떻게 재는가 - 시계 차이를 반드시 빼야 한다.
  MORAI 메시지에는 송신측 header.stamp 가 들어있다. 그래서
      raw = 수신시각(ROS) - header.stamp(MORAI)
  가 바로 지연처럼 보이지만, 두 PC 의 시계가 안 맞으면 그 offset 이
  통째로 섞인다. 실측(2026-09-18, 같은 PC 한 대)에서도 raw 중앙값이
  -11.7 ms 로 '음수 지연'이 나왔다 - MORAI 시계가 앞서 있다는 뜻이지
  지연이 음수인 게 아니다. 대회장처럼 PC 가 둘이면 이 offset 이 수십~수백
  ms 까지 벌어질 수 있다.

  그래서 두 가지를 따로 낸다.
    offset  : raw 의 최솟값 추정(이동창 최소). 네트워크가 가장 한가했던
              순간에는 실제 전송지연이 거의 0 이므로, 그때의 raw 가
              시계 offset 에 가장 가깝다.
    jitter  : raw - offset. 시계 차이를 제거한 '여유 지연'.
              이 값이 커지는 순간이 실제로 랜/시뮬이 밀린 순간이다.
  offset 자체는 절대 지연을 말해주지 않지만, jitter 는 PC 가 둘이든
  하나든 그대로 의미가 있다.

  덧붙여 topic 별 수신간격(Hz)과 그 흔들림(간격 p95)도 같이 낸다.
  헤더가 없는 토픽(/velodyne_points 등)은 지연을 못 재므로 Hz 만 낸다.

발행 토픽 (전부 가볍고 백에 남길 수 있다)
  /comm/latency_ms    diagnostic_msgs/DiagnosticArray - 토픽별 지연/Hz
  /comm/summary       std_msgs/String (JSON)          - 한 줄 요약

사용법
  rosrun total_bringup comm_monitor.py
  roslaunch total_bringup competition.launch record:=true   (자동 포함)
"""
import collections
import json
import math

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import String


class TopicStat(object):
    """한 토픽의 수신간격과 (헤더가 있으면) 지연을 모은다."""

    def __init__(self, name, window_sec, expected_hz):
        self.name = name
        self.expected_hz = float(expected_hz)
        self.window_sec = float(window_sec)
        # (수신시각, raw지연) - raw 는 헤더 없으면 None
        self.samples = collections.deque()
        self.last_recv = None
        self.gaps = collections.deque()
        self.count = 0
        self.has_header = False

    def add(self, recv_sec, stamp_sec):
        self.count += 1
        raw = None
        if stamp_sec is not None and stamp_sec > 0.0:
            raw = (recv_sec - stamp_sec) * 1000.0
            self.has_header = True
        self.samples.append((recv_sec, raw))
        if self.last_recv is not None:
            self.gaps.append((recv_sec, (recv_sec - self.last_recv) * 1000.0))
        self.last_recv = recv_sec
        self._trim(recv_sec)

    def _trim(self, now):
        cutoff = now - self.window_sec
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()
        while self.gaps and self.gaps[0][0] < cutoff:
            self.gaps.popleft()

    def report(self, now):
        """(hz, gap_p95_ms, offset_ms, jitter_med_ms, jitter_p95_ms, age_ms)"""
        self._trim(now)
        n = len(self.samples)
        hz = 0.0
        if n >= 2:
            span = self.samples[-1][0] - self.samples[0][0]
            if span > 1e-6:
                hz = (n - 1) / span

        gaps = sorted(g for _, g in self.gaps)
        gap_p95 = _pct(gaps, 95)

        raws = [r for _, r in self.samples if r is not None]
        offset = min(raws) if raws else None
        jit_med = jit_p95 = None
        if raws and offset is not None:
            j = sorted(r - offset for r in raws)
            jit_med = _pct(j, 50)
            jit_p95 = _pct(j, 95)

        age = None
        if self.last_recv is not None:
            age = (now - self.last_recv) * 1000.0
        return hz, gap_p95, offset, jit_med, jit_p95, age


def _pct(sorted_vals, pct):
    if not sorted_vals:
        return None
    k = (len(sorted_vals) - 1) * pct / 100.0
    lo = int(math.floor(k))
    hi = int(math.ceil(k))
    if lo == hi:
        return sorted_vals[lo]
    return sorted_vals[lo] * (hi - k) + sorted_vals[hi] * (k - lo)


class CommMonitor(object):
    def __init__(self):
        self.window_sec = float(rospy.get_param('~window_sec', 5.0))
        self.rate_hz = float(rospy.get_param('~rate', 2.0))
        # 기본 감시 대상. MORAI 에서 UDP 로 직접 넘어오는 것 위주로 잡고,
        # 우리 노드가 만든 /odom 을 하나 넣어 파이프라인 끝단도 본다.
        default = [
            '/Competition_topic:50',
            '/gps:30',
            '/imu:30',
            '/velodyne_points:10',
            '/image_jpeg/compressed:30',
            '/image_jpeg2/compressed:30',
            '/odom:50',
            '/ctrl_cmd:50',
        ]
        specs = rospy.get_param('~topics', default)
        self.stats = {}
        for spec in specs:
            name, _, hz = str(spec).partition(':')
            name = name.strip()
            if not name:
                continue
            try:
                exp = float(hz) if hz else 0.0
            except ValueError:
                exp = 0.0
            self.stats[name] = TopicStat(name, self.window_sec, exp)

        self.diag_pub = rospy.Publisher('/comm/latency_ms', DiagnosticArray,
                                        queue_size=2)
        self.sum_pub = rospy.Publisher('/comm/summary', String, queue_size=2)

        # 타입을 모르는 채 구독하기 위해 AnyMsg 를 쓴다. 역직렬화 없이
        # 헤더만 떼어내므로 큰 점군에도 부담이 적다.
        for name in self.stats:
            rospy.Subscriber(name, rospy.AnyMsg,
                             self._make_cb(name), queue_size=1)
        rospy.loginfo('[comm_monitor] %d개 토픽 감시, 창 %.1fs',
                      len(self.stats), self.window_sec)

    def _make_cb(self, name):
        def cb(msg):
            recv = rospy.Time.now().to_sec()
            stamp = _peek_header_stamp(msg)
            self.stats[name].add(recv, stamp)
        return cb

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self._publish()
            rate.sleep()

    def _publish(self):
        now = rospy.Time.now().to_sec()
        arr = DiagnosticArray()
        arr.header.stamp = rospy.Time.now()
        worst_jit = 0.0
        worst_name = ''
        summary = {}

        for name in sorted(self.stats):
            st = self.stats[name]
            hz, gap95, off, jmed, j95, age = st.report(now)
            s = DiagnosticStatus()
            s.name = 'comm/%s' % name
            s.hardware_id = 'morai_udp'
            kv = [KeyValue('hz', '%.2f' % hz),
                  KeyValue('count', str(st.count))]
            if st.expected_hz > 0:
                kv.append(KeyValue('expected_hz', '%.1f' % st.expected_hz))
            if gap95 is not None:
                kv.append(KeyValue('gap_p95_ms', '%.1f' % gap95))
            if age is not None:
                kv.append(KeyValue('age_ms', '%.1f' % age))
            if off is not None:
                kv.append(KeyValue('clock_offset_ms', '%.1f' % off))
            if jmed is not None:
                kv.append(KeyValue('jitter_med_ms', '%.1f' % jmed))
                kv.append(KeyValue('jitter_p95_ms', '%.1f' % j95))
                if j95 > worst_jit:
                    worst_jit, worst_name = j95, name
            s.values = kv

            # 판정: 수신이 끊겼거나 기대 Hz 의 절반 아래면 ERROR,
            # 지터가 100ms 넘으면 WARN.
            if st.count == 0 or (age is not None and age > 2000.0):
                s.level, s.message = DiagnosticStatus.ERROR, '수신 없음'
            elif st.expected_hz > 0 and hz < st.expected_hz * 0.5:
                s.level, s.message = DiagnosticStatus.ERROR, 'Hz 저하'
            elif j95 is not None and j95 > 100.0:
                s.level, s.message = DiagnosticStatus.WARN, '지연 흔들림'
            else:
                s.level, s.message = DiagnosticStatus.OK, 'OK'
            arr.status.append(s)

            summary[name] = {'hz': round(hz, 2)}
            if j95 is not None:
                summary[name]['jitter_p95_ms'] = round(j95, 1)
                summary[name]['clock_offset_ms'] = round(off, 1)

        self.diag_pub.publish(arr)
        summary['_worst'] = {'topic': worst_name,
                             'jitter_p95_ms': round(worst_jit, 1)}
        self.sum_pub.publish(String(data=json.dumps(summary,
                                                    ensure_ascii=False)))


def _peek_header_stamp(any_msg):
    """AnyMsg 버퍼 앞에서 std_msgs/Header 의 stamp 만 읽는다.

    ROS 직렬화에서 Header 를 첫 필드로 갖는 메시지는 버퍼가
    [uint32 seq][uint32 sec][uint32 nsec][uint32 frame_len]... 로 시작한다.
    타입을 몰라도 헤더 유무를 확신할 수 없으므로, frame_id 길이가
    상식적인 범위인지까지 확인해 오탐을 줄인다.
    """
    buf = any_msg._buff
    if buf is None or len(buf) < 16:
        return None
    try:
        import struct
        seq, sec, nsec, flen = struct.unpack('<IIII', buf[:16])
    except Exception:
        return None
    # 헤더가 아닌 메시지를 헤더로 오독하지 않도록 방어한다.
    if flen > 64 or len(buf) < 16 + flen:
        return None
    if sec < 1000000000 or sec > 4000000000:
        return None
    if nsec >= 1000000000:
        return None
    return sec + nsec * 1e-9


if __name__ == '__main__':
    rospy.init_node('comm_monitor')
    try:
        CommMonitor().spin()
    except rospy.ROSInterruptException:
        pass
