#!/usr/bin/env python3
"""
control_arbiter — 워크스페이스에서 /ctrl_cmd 를 발행하는 유일한 노드.

기존 aeb_arbiter 를 확장했다. 세 단계를 순서대로 적용한다.

  ① 소스 선택  구간이 차선주행(M8)이면 /control/m8_cmd, 아니면 /control/raw_cmd(MPC).
                전환 순간 조향이 튀지 않도록 blend_sec 동안 선형 보간한다.
  ② 안전 오버라이드  /aeb/command, /intersection/command 를 보수적으로 융합.
  ③ 속도 하드리미터  실측 속도가 상한을 넘으면 accel 을 끊고 제동한다.
                     구간이 speed_limit_exempt(고주로)면 해제한다.

불변식: 출력은 항상 longlCmdType = 1 (accel/brake).
규정 2-8이 cmd type 1만 허용하므로 type 2 경로를 구조적으로 차단한다.
기존 aeb_arbiter 는 속도캡을 type 2로 내보냈다 — 그 경로를 여기서 없앴다.

측위 무효(GPS 상실) 처리:
  일반 구간에서는 localization 이 무효면 정지한다. 그러나 M8은 GPS 가
  Blackout 되는 구간이고 차선주행이 그것을 전제로 동작하므로, 차선주행
  소스가 선택된 동안에는 이 조건을 적용하지 않는다.
"""
import rospy
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from aeb_safety.msg import AebCommand
from std_msgs.msg import Bool, Float64, UInt8
from mission_supervisor.msg import SectionState


class ControlArbiter(object):
    def __init__(self):
        rospy.init_node('control_arbiter')

        self.cmd_timeout = rospy.get_param('~cmd_timeout', 0.5)
        self.src_timeout = rospy.get_param('~source_timeout', 0.5)
        self.full_brake = rospy.get_param('~full_brake', 1.0)
        self.rate_hz = float(rospy.get_param('~rate', 50.0))
        self.blend_sec = float(rospy.get_param('~source_blend_sec', 1.5))
        # 차선주행 소스를 선택하게 만드는 executor 이름
        self.lane_executor = rospy.get_param('~lane_following_executor',
                                             'lane_following')

        # 속도 하드리미터 — 규정 60kph 초과는 즉시 15초 패널티다.
        self.limit_enabled = bool(rospy.get_param('/speed_limit/hard_limit_enabled',
                                  rospy.get_param('~speed_limit/enabled', True)))
        self.limit_mps = float(rospy.get_param('/speed_limit/hard_limit_mps',
                               rospy.get_param('~speed_limit/hard_limit_mps', 16.2)))
        self.limit_brake = float(rospy.get_param('/speed_limit/hard_limit_brake',
                                 rospy.get_param('~speed_limit/brake', 0.25)))
        self.limit_release = float(rospy.get_param('/speed_limit/hard_limit_release_mps',
                                   rospy.get_param('~speed_limit/release_margin_mps', 0.4)))
        # 미션 속도캡을 페달로 적용할지. 기본 false.
        # 캡은 constraint_to_speed_cap 이 /smpc/target_speed_mps 로 MPC 에
        # 직접 준다. 여기서 또 페달을 덮으면 MPC 와 종방향 권한이 겹쳐
        # 가속/제동이 서로를 밀어내며 차가 덜컹인다(2026-09-02 실측).
        self.cap_enabled = bool(rospy.get_param('~mission_cap/enabled', False))
        self.cap_brake_base = float(rospy.get_param('~mission_cap/brake_base', 0.20))
        self.cap_brake_gain = float(rospy.get_param('~mission_cap/brake_gain', 0.15))

        self.raw = None
        self.raw_t = 0.0
        self.m8 = None
        self.m8_t = 0.0
        self.aeb = None
        self.aeb_t = 0.0
        self.xing = None
        self.xing_t = 0.0
        # ★ 2026-10-01 후진 탈출(reverse_escape_node). 신선한 동안 이 명령이 소스가 되고
        #   요청 기어를 /control/gear 로 브리지에 넘긴다. 끊기면 기어는 D(4).
        #   후진 중에는 '회피 경로 없음(avoidance_failed)' AEB 만 무시한다 - 그게 후진의
        #   이유이기 때문. 동적 장애물 정지 등 다른 안전 명령은 그대로 제동한다.
        self.rev_timeout = float(rospy.get_param('~reverse_timeout', 0.3))
        self.rev = None
        self.rev_t = 0.0
        self.rev_gear = 4
        self.rev_gear_t = 0.0

        self.loc_valid = False
        self.loc_seen = False
        self.speed = 0.0
        self.speed_t = 0.0
        self.exempt = False
        self.lane_mode = False
        self.section_seen = False
        self.section_id = ""

        self.active_src = None
        self.blend_t0 = 0.0
        self.prev_steer = 0.0
        self.limiting = False
        self.mission_cap = -1.0     # <0 = 캡 없음
        self.mission_cap_t = 0.0

        self.pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.gear_pub = rospy.Publisher('/control/gear', UInt8, queue_size=1)
        rospy.Subscriber('/control/reverse_cmd', CtrlCmd, self.rev_cb, queue_size=1)
        rospy.Subscriber('/control/reverse_gear', UInt8, self.rev_gear_cb, queue_size=1)
        rospy.Subscriber('/control/raw_cmd', CtrlCmd, self.raw_cb, queue_size=1)
        rospy.Subscriber('/control/m8_cmd', CtrlCmd, self.m8_cb, queue_size=1)
        rospy.Subscriber('/aeb/command', AebCommand, self.aeb_cb, queue_size=1)
        rospy.Subscriber('/intersection/command', AebCommand, self.xing_cb, queue_size=1)
        rospy.Subscriber('/localization/valid', Bool, self.loc_cb, queue_size=1)
        rospy.Subscriber('/Competition_topic', EgoVehicleStatus, self.ego_cb, queue_size=1)
        rospy.Subscriber('/mission/active_section', SectionState, self.sec_cb, queue_size=1)
        rospy.Subscriber('/mission/speed_cap_mps', Float64, self.cap_cb, queue_size=1)

        rospy.Timer(rospy.Duration(1.0 / max(1.0, self.rate_hz)), self.tick)
        rospy.loginfo('[control_arbiter] ready. /ctrl_cmd 단독 발행 · '
                      'cmd type 1 고정 · 속도리미터 %s(%.1f m/s)',
                      'ON' if self.limit_enabled else 'OFF', self.limit_mps)

    # ---------- 입력 ----------

    def raw_cb(self, m):
        self.raw, self.raw_t = m, rospy.Time.now().to_sec()

    def m8_cb(self, m):
        self.m8, self.m8_t = m, rospy.Time.now().to_sec()

    def rev_cb(self, m):
        self.rev, self.rev_t = m, rospy.Time.now().to_sec()

    def rev_gear_cb(self, m):
        if int(m.data) in (2, 4):
            self.rev_gear, self.rev_gear_t = int(m.data), rospy.Time.now().to_sec()

    def reverse_active(self):
        return self.rev is not None and self._fresh(self.rev_t, self.rev_timeout)

    def aeb_cb(self, m):
        self.aeb, self.aeb_t = m, rospy.Time.now().to_sec()

    def xing_cb(self, m):
        self.xing, self.xing_t = m, rospy.Time.now().to_sec()

    def loc_cb(self, m):
        self.loc_valid, self.loc_seen = bool(m.data), True

    def ego_cb(self, m):
        self.speed, self.speed_t = float(m.velocity.x), rospy.Time.now().to_sec()

    def cap_cb(self, m):
        self.mission_cap, self.mission_cap_t = float(m.data), rospy.Time.now().to_sec()

    def sec_cb(self, m):
        self.section_id = getattr(m, 'section_id', '')
        self.exempt = bool(m.speed_limit_exempt)
        self.lane_mode = self.lane_executor in list(m.executors)
        self.section_seen = True

    # ---------- 유틸 ----------

    def _fresh(self, t, timeout):
        return (rospy.Time.now().to_sec() - t) < timeout

    def stop_command(self, steering=0.0):
        out = CtrlCmd()
        out.longlCmdType = 1
        out.accel = 0.0
        out.brake = self.full_brake
        out.steering = steering
        return out

    # ---------- ① 소스 선택 ----------

    def select_source(self):
        want_m8 = self.lane_mode and self.m8 is not None \
            and self._fresh(self.m8_t, self.src_timeout)
        if want_m8:
            return 'm8', self.m8
        if self.raw is not None and self._fresh(self.raw_t, self.src_timeout):
            return 'raw', self.raw
        # 선택된 소스가 없으면 남은 하나라도 쓴다(둘 다 stale 이면 None).
        if self.m8 is not None and self._fresh(self.m8_t, self.src_timeout):
            return 'm8', self.m8
        return None, None

    def blend_steering(self, steer):
        """소스 전환 직후 조향을 이전 값에서 서서히 옮긴다."""
        if self.blend_sec <= 0.0:
            return steer
        dt = rospy.Time.now().to_sec() - self.blend_t0
        if dt >= self.blend_sec:
            return steer
        w = 1.0 - (dt / self.blend_sec)      # 1 → 0
        return w * self.prev_steer + (1.0 - w) * steer

    # ---------- ② 안전 오버라이드 ----------

    def safety_cmds(self):
        out = []
        if self.aeb is not None and self._fresh(self.aeb_t, self.cmd_timeout):
            out.append(self.aeb)
        if self.xing is not None and self._fresh(self.xing_t, self.cmd_timeout):
            out.append(self.xing)
        return out

    # ---------- 주기 ----------

    def reverse_tick(self):
        """후진 탈출 명령 출력. 기어는 요청값(신선할 때), 아니면 D."""
        gear = self.rev_gear if self._fresh(self.rev_gear_t, self.rev_timeout) else 4
        self.gear_pub.publish(UInt8(data=gear))
        out = CtrlCmd()
        out.longlCmdType = 1
        out.accel = self.rev.accel
        out.brake = self.rev.brake
        out.steering = self.rev.steering
        cmds = [c for c in self.safety_cmds()
                if not (c.source == 'dynamic_obstacle_stop' and
                        c.reason in ('avoidance_failed',
                                     'dynamic_to_static_wait_reverse'))]
        if any(c.emergency or c.stage == AebCommand.STAGE_FULL for c in cmds):
            out.accel, out.brake = 0.0, self.full_brake
            rospy.logwarn_throttle(1.0, '[control_arbiter] 후진 중 안전 정지: %s',
                                   ', '.join('%s/%s' % (c.source, c.reason) for c in cmds))
        else:
            brakes = [c.brake for c in cmds
                      if c.stage in (AebCommand.STAGE_PB1, AebCommand.STAGE_PB2)]
            if brakes:
                out.accel, out.brake = 0.0, max(out.brake, max(brakes))
        out.velocity = 0.0
        out.acceleration = 0.0
        self.prev_steer = out.steering
        if self.active_src != 'reverse':
            rospy.loginfo('[control_arbiter] 제어 소스 전환: %s -> reverse', self.active_src)
            self.active_src = 'reverse'
            self.blend_t0 = rospy.Time.now().to_sec()
        self.pub.publish(out)

    def tick(self, _evt):
        if self.reverse_active():
            self.reverse_tick()
            return
        self.gear_pub.publish(UInt8(data=4))
        name, src = self.select_source()
        if src is None:
            self.pub.publish(self.stop_command(self.prev_steer))
            return

        if name != self.active_src:
            if self.active_src is not None:
                rospy.loginfo('[control_arbiter] 제어 소스 전환: %s -> %s',
                              self.active_src, name)
                self.blend_t0 = rospy.Time.now().to_sec()
            self.active_src = name

        # 측위 무효 정지 — 차선주행 중 및 M8(GPS Blackout) 구간에서는 적용하지 않는다.
        # M7B(터널 접근)도 함께 면제한다. GPS 음영은 구간 경계(idx 7000)에
        # 맞춰 시작하지 않고 터널 입구 전부터 흔들릴 수 있는데, 그때
        # 측위 무효 정지가 걸리면 터널 직전에 선다.
        sec = getattr(self, 'section_id', '')
        is_m8 = sec in ('M8', 'M7B') or (name == 'm8') or self.lane_mode
        if not is_m8 and self.loc_seen and not self.loc_valid:
            self.pub.publish(self.stop_command(src.steering))
            self.prev_steer = src.steering
            return

        out = CtrlCmd()
        out.longlCmdType = 1                      # 불변식
        out.accel = src.accel
        out.brake = src.brake
        out.steering = self.blend_steering(src.steering)

        # 소스가 속도명령(type 2)으로 왔다면 페달 명령이 비어 있을 수 있다.
        # 규정상 type 2를 내보낼 수 없으므로 그대로 통과시키지 않는다.
        if src.longlCmdType != 1:
            rospy.logwarn_throttle(
                5.0, '[control_arbiter] 소스 %s 가 type=%d 로 보냈다. '
                     'type 1 로 강제한다.', name, src.longlCmdType)

        # ② 안전 오버라이드
        cmds = self.safety_cmds()
        emergency = any(c.emergency or c.stage == AebCommand.STAGE_FULL for c in cmds)
        brakes = [c.brake for c in cmds
                  if c.stage in (AebCommand.STAGE_PB1, AebCommand.STAGE_PB2)]
        caps = [c.target_speed / 3.6 for c in cmds
                if c.stage in (AebCommand.STAGE_FCW, AebCommand.STAGE_HOLD)
                and c.target_speed >= 0.0]
        # 미션 제약에서 온 속도캡. ACC(behavior 레이어)가 안 떠 있어도
        # 미션이 차량에 도달하도록 여기서 페달 단으로 적용한다.
        if self.cap_enabled and self.mission_cap >= 0.0 \
                and self._fresh(self.mission_cap_t, self.cmd_timeout):
            caps.append(self.mission_cap)

        if emergency:
            out.accel, out.brake = 0.0, self.full_brake
        elif brakes:
            out.accel, out.brake = 0.0, max(brakes)
        elif caps and self._fresh(self.speed_t, self.cmd_timeout):
            # 속도캡을 type 2 로 바꾸지 않고 페달로 처리한다.
            cap = min(caps)
            if cap <= 0.05:
                # 정지 요구
                out.accel = 0.0
                out.brake = max(out.brake, self.full_brake)
            elif self.speed > cap:
                out.accel = 0.0
                out.brake = max(out.brake, min(
                    1.0, self.cap_brake_base + self.cap_brake_gain * (self.speed - cap)))

        # ③ 속도 하드리미터
        if self.limit_enabled and not self.exempt and not emergency \
                and self._fresh(self.speed_t, self.cmd_timeout):
            if self.speed > self.limit_mps:
                self.limiting = True
            elif self.speed < (self.limit_mps - self.limit_release):
                self.limiting = False
            if self.limiting:
                out.accel = 0.0
                out.brake = max(out.brake, self.limit_brake)
                rospy.logwarn_throttle(
                    2.0, '[control_arbiter] 속도리미터 작동: %.2f > %.2f m/s',
                    self.speed, self.limit_mps)

        out.velocity = 0.0
        out.acceleration = 0.0
        self.prev_steer = out.steering
        self.pub.publish(out)


if __name__ == '__main__':
    try:
        ControlArbiter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
