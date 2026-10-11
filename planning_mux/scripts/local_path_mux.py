#!/usr/bin/env python3
"""
local_path_mux — /local_path 의 유일한 publisher.

통합 전에는 4개 노드가 /local_path 를 각자 발행했다(계획안 §2). 후보를
/planning/local_path/<source> 로 내리고 여기서 하나만 승격한다.

우선순위는 파라미터로 준다. 기본값은 avoidance > waypoint 이며,
avoidance 는 그 구간의 executors 에 'static_obstacle' 이 있을 때만 승격된다.
구간 정보가 없으면(라우터 미기동) 후보 신선도만으로 판단한다.

전환 시 경로가 튀지 않도록 blend_points 개수만큼 선형 보간한다.
모든 후보가 stale 이면 마지막 유효 경로를 유지하고 경고한다 — 빈 경로를
내보내면 MPC 가 polyfit 최소점수를 못 채워 path-end stop 으로 급정지한다.
"""
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Float64
from mission_supervisor.msg import SectionState


class LocalPathMux(object):
    def __init__(self):
        rospy.init_node('local_path_mux')

        # [{name, topic, requires_executor(optional)}] 순서가 곧 우선순위
        self.sources = rospy.get_param('~sources', [
            {'name': 'avoidance',
             'topic': '/planning/local_path/avoidance',
             'requires_executor': 'static_obstacle'},
            {'name': 'waypoint',
             'topic': '/planning/local_path/waypoint'},
        ])
        self.timeout = float(rospy.get_param('~source_timeout', 0.5))
        self.rate_hz = float(rospy.get_param('~rate', 30.0))
        self.blend_points = int(rospy.get_param('~blend_points', 20))
        self.hold_on_stale = bool(rospy.get_param('~hold_on_stale', True))
        # 회피 후보는 장애물이 없으면 기준 경로를 그대로 흘려보낸다
        # (passthrough_length_m). 그래서 '실제로 벌어질 때만' 승격시키면
        # 구간 인덱스를 몰라도 안전하게 회피를 켤 수 있다.
        self.dev_enter_m = float(rospy.get_param('~deviation_enter_m', 0.35))
        self.dev_exit_m = float(rospy.get_param('~deviation_exit_m', 0.15))
        self.dev_confirm = int(rospy.get_param('~deviation_confirm', 3))
        self.dev_release = int(rospy.get_param('~deviation_release', 15))
        self.dev_samples = int(rospy.get_param('~deviation_samples', 25))
        # 편차로 회피 여부를 판별하려 했으나, obstacle_avoidance_node 는 원래
        # local_path_publisher 를 '대체'하는 노드라 장애물이 없어도 CSV 경로와
        # 2 m 이상 벌어진다(실측 중앙 2.33 m). 그래서 편차는 쓸 수 없다.
        # 대신 그 노드가 내는 명시적 플래그를 쓴다.
        #   /obstacle_avoidance/target_speed_mps
        #     9999.0 = 평소 주행(passthrough)   그 외 = 회피/복귀 중
        self.signal_topic = rospy.get_param(
            '~avoidance_signal_topic', '/obstacle_avoidance/target_speed_mps')
        self.signal_idle = float(rospy.get_param('~avoidance_signal_idle', 9000.0))
        self.signal_timeout = float(rospy.get_param('~avoidance_signal_timeout', 1.0))
        self.signal_val = None
        self.signal_t = 0.0
        self.deviating = {}
        self.dev_hi = {}
        self.dev_lo = {}

        self.latest = {}     # name -> (Path, stamp)
        # 편차 비교의 기준이 되는 후보(보통 CSV 경로)
        self.base_name = rospy.get_param('~base_source', 'waypoint')
        self.executors = []
        self.section_seen = False
        self.active_name = None
        self.last_out = None
        self.blend_left = 0

        self.pub = rospy.Publisher('/local_path', Path, queue_size=1)
        for s in self.sources:
            rospy.Subscriber(s['topic'], Path,
                             self._make_cb(s['name']), queue_size=1)
        rospy.Subscriber('/mission/active_section', SectionState,
                         self.sec_cb, queue_size=1)
        rospy.Subscriber(self.signal_topic, Float64, self.sig_cb, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / max(1.0, self.rate_hz)), self.tick)
        rospy.loginfo('[local_path_mux] ready. 후보 %d개 -> /local_path',
                      len(self.sources))

    def _make_cb(self, name):
        def cb(msg):
            self.latest[name] = (msg, rospy.Time.now().to_sec())
        return cb

    def sig_cb(self, msg):
        self.signal_val = float(msg.data)
        self.signal_t = rospy.Time.now().to_sec()

    def _avoidance_active(self):
        if self.signal_val is None:
            return False
        if (rospy.Time.now().to_sec() - self.signal_t) >= self.signal_timeout:
            return False
        return self.signal_val < self.signal_idle

    def sec_cb(self, msg):
        self.executors = list(msg.executors)
        self.section_seen = True

    @staticmethod
    def _max_lateral_dev(cand, ref, nsample):
        """cand 점들이 ref 폴리라인에서 얼마나 벗어났는지 최대값[m]."""
        if not cand.poses or not ref.poses:
            return 0.0
        rp = [(q.pose.position.x, q.pose.position.y) for q in ref.poses]
        step = max(1, len(cand.poses) // max(1, nsample))
        worst = 0.0
        for i in range(0, len(cand.poses), step):
            cx = cand.poses[i].pose.position.x
            cy = cand.poses[i].pose.position.y
            best = None
            for rx, ry in rp:
                d = (rx - cx) ** 2 + (ry - cy) ** 2
                if best is None or d < best:
                    best = d
            if best is not None:
                worst = max(worst, best ** 0.5)
        return worst

    def _deviating(self, name, cand):
        """기준 후보 대비 실제로 벗어나는 중인지. 히스테리시스 적용."""
        base = self.latest.get(self.base_name)
        if base is None:
            return False
        dev = self._max_lateral_dev(cand, base[0], self.dev_samples)
        hi = self.dev_hi.get(name, 0)
        lo = self.dev_lo.get(name, 0)
        if dev >= self.dev_enter_m:
            hi += 1; lo = 0
        elif dev <= self.dev_exit_m:
            lo += 1; hi = 0
        self.dev_hi[name], self.dev_lo[name] = hi, lo
        on = self.deviating.get(name, False)
        if not on and hi >= self.dev_confirm:
            on = True
            rospy.loginfo('[local_path_mux] %s 회피 시작 (편차 %.2f m)', name, dev)
        elif on and lo >= self.dev_release:
            on = False
            rospy.loginfo('[local_path_mux] %s 복귀 (편차 %.2f m)', name, dev)
        self.deviating[name] = on
        return on

    def _allowed(self, src):
        """구간 조건이 걸린 후보는 그 조건이 '확인될 때만' 승격한다.

        구간 정보가 없을 때 열어주면(이전 구현) section_router 를 안 띄운
        구성에서 회피 경로가 랩 내내 CSV 경로를 밀어낸다. 실제로 MPC 가
        3.1 m 짜리 Frenet 조각을 계속 추종했다. 정보가 없으면 막고
        기본 경로로 떨어지는 쪽이 맞다.
        """
        # 회피 노드가 '회피 중'이라고 말할 때만 승격한다.
        if src.get('promote_on_signal'):
            if not self._avoidance_active():
                return False

        need = src.get('requires_executor')
        if not need:
            return True
        if not self.section_seen:
            return False     # 구간 정보 없음 -> 기본 경로 사용
        return need in self.executors

    def select(self, now):
        for src in self.sources:
            name = src['name']
            entry = self.latest.get(name)
            if entry is None:
                continue
            path, stamp = entry
            if (now - stamp) >= self.timeout:
                continue
            if not path.poses:
                continue
            if not self._allowed(src):
                continue
            return name, path
        return None, None

    def blend(self, new_path):
        """전환 직후 blend_left 프레임 동안 이전 경로와 선형 보간한다."""
        if self.last_out is None or self.blend_left <= 0:
            return new_path
        w = self.blend_left / float(max(1, self.blend_points))
        n = min(len(new_path.poses), len(self.last_out.poses))
        out = Path()
        out.header = new_path.header
        out.poses = list(new_path.poses)
        for i in range(n):
            a = self.last_out.poses[i].pose.position
            b = new_path.poses[i].pose.position
            out.poses[i].pose.position.x = w * a.x + (1.0 - w) * b.x
            out.poses[i].pose.position.y = w * a.y + (1.0 - w) * b.y
        self.blend_left -= 1
        return out

    def tick(self, _evt):
        now = rospy.Time.now().to_sec()
        name, path = self.select(now)

        if path is None:
            if self.hold_on_stale and self.last_out is not None:
                rospy.logwarn_throttle(
                    1.0, '[local_path_mux] 모든 후보 stale — 마지막 경로 유지')
                self.pub.publish(self.last_out)
            else:
                rospy.logwarn_throttle(
                    1.0, '[local_path_mux] 유효한 경로 후보 없음')
            return

        if name != self.active_name:
            rospy.loginfo('[local_path_mux] 경로 전환: %s -> %s',
                          self.active_name, name)
            self.active_name = name
            self.blend_left = self.blend_points

        out = self.blend(path)
        self.last_out = out
        self.pub.publish(out)


if __name__ == '__main__':
    try:
        LocalPathMux()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
