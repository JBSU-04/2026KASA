#pragma once

#include <limits>
#include <functional>
#include <string>
#include <vector>

namespace mission_planner {

// 합류 흐름(원형 차로 등)을 1차원으로 편 좌표에서 차량 사이 틈(gap)을 고른다.
//   pos: 합류점(충돌점) = 0, 진행 방향 + (하류/지나감), - (상류/다가옴) [m]
//   ROS 와 회전교차로 기하에 의존하지 않는다 - 호출자가 좌표를 편다.
//
// 틈 = (앞차 leader, 뒤차 follower) 쌍. 각 틈마다 '지금부터 몇 초 뒤에
// 출발하면 들어갈 수 있는지'(depart_delay)를 예측으로 구한다.
//   - 흐름 차량: 등속 예측(앞차는 속도 -margin, 뒤차는 +margin 으로 불리하게)
//   - 자차: 지금 속도에서 accel 로 max_speed 까지(정지 출발이면 start_delay 후)
//   - 자차 앞끝이 합류 구역에 들어간 뒤 check_horizon 동안
//       앞차와: 간격 >= standstill_gap + front_headway * v_ego
//       뒤차와: 간격 >= standstill_gap + rear_headway * v_follower
struct StreamVehicle {
  int id = -1;
  double pos = 0.0;
  double speed = 0.0;
  double length = 4.5;
  // 순차 판단용 속도 범위(음수 = speed 사용). 앞차는 느린 쪽(늦게 빠짐),
  // 뒤차는 빠른 쪽(빨리 옴)으로 본다.
  double speed_lo = -1.0;
  double speed_hi = -1.0;
  double physical_speed = -1.0; // ACC uses physical speed, not nominal-ring arc speed.
};

struct EgoEntryModel {
  double distance_to_merge = 0.0;  // 자차 중심 -> 합류점 경로 거리 [m]
  double speed = 0.0;              // 현재 속도 [m/s]
  double accel = 2.2;              // 출발 가속 [m/s^2]
  double max_speed = 8.0;          // 합류 속도 상한 [m/s]
  double start_delay = 0.2;        // 정지 출발 반응 지연 [s]
  double length = 4.6;             // 자차 길이 [m]
  bool measured_profile = false;
  std::vector<double> profile_times, profile_speeds;
  bool departure_in_progress = false; // COMMIT: continue from measured motion.
};

struct GapSelectorParams {
  double front_headway = 1.0;      // 앞차와 시간 간격 [s]
  double rear_headway = 1.2;       // 뒤차와 시간 간격 [s]
  double standstill_gap = 2.0;     // 범퍼 간 최소 거리 [m]
  double check_horizon = 3.0;      // 합류 뒤 검사 시간 [s]
  double speed_margin = 0.5;       // 흐름 차량 속도 불확실성 [m/s]
  double entry_zone = 1.0;         // 자차 앞끝이 합류점 이만큼 앞이면 합류 구역 [m]
  double max_wait = 12.0;          // 출발 지연 탐색 범위 [s]
  double search_step = 0.1;        // 출발 지연 탐색 간격 [s]
  double sample_dt = 0.1;          // 검사 시간 간격 [s]
  double ring_length = 0.0;        // 원형이면 둘레 [m] (0 = 직선 흐름)
  double leader_follow_range = 30.0; // 이 안의 앞차는 합류 뒤 속도 상한으로 본다 [m]
};

struct GapCandidate {
  int leader_id = -1;      // -1 = 앞차 없음
  int follower_id = -1;    // -1 = 뒤차 없음
  int wrap = 0;            // 원형 한 바퀴 이동: +1 앞차, -1 뒤차 (래치 구분용)
  bool feasible = false;
  double depart_delay = std::numeric_limits<double>::infinity();
  double merge_time = std::numeric_limits<double>::infinity();
  double leader_pos = 0.0, leader_speed = 0.0, leader_length = 0.0;
  double follower_pos = 0.0, follower_speed = 0.0, follower_length = 0.0;
  // depart_delay=0 으로 출발했을 때 가장 부족한 쪽과 그 부족분 [m]
  std::string limit_side;  // "front" / "rear" / ""
  double limit_shortfall = 0.0;
  // depart_delay=0 일 때 쪽별 부족분 [m] (0 = 만족). 앞차만 부족하면 '앞차가
  // 먼저 들어가기를 기다리는' 틈이라 조금 일찍 출발해도 앞차 뒤로 맞춰 설 수 있다.
  double front_shortfall_now = 0.0;
  double rear_shortfall_now = 0.0;
};

// ★ 2026-09-30 순차 틈 판단(sequential). 틈을 도착 순서대로 하나씩 본다.
//   목표 틈 = 앞차 L 바로 뒤 공간(L=-1 이면 처음 다가오는 차 앞 공간).
//   - 출발: 앞차 통과와 front_headway를 만족하는 틈을 예약하고, 대기시간이
//     0일 때만 출발. 승인 전/후 동일한 가속·속도 상한·앞차 대기 모델을 쓴다.
//   - 뒤차: 예측 진입부터 check_horizon 동안(late_margin은 뒤차 위치에 가산)
//       간격 >= standstill_gap + rear_headway * v_f, 뒤차가 빠르면 TTC >= ttc_rear.
//   - 대기 중 안 되면 뒤차의 뒤 공간(L := 뒤차)을 본다. COMMIT은 차량 쌍을 고정한다.
struct MergeFootprint {
  bool valid = true;
  bool active = false;
  double pos = 0.0, half_length = 0.0, speed_ratio = 1.0;
  double x = 0.0, y = 0.0, yaw = 0.0;
};
struct SequentialParams {
  std::function<MergeFootprint(double)> project_ego;
  std::function<int(double,double)> motion_collision; // time, travel -> object ID, -1 if clear.
  bool observed_slot=false;
  bool separate_slot_observation=false; // Bounded capacity is geometric; visibility is a trajectory check.
  std::function<std::pair<double,double>(double)> observed_bounds;
  std::function<std::string(double,double)> observation_check;
  double spatial_front_gap = 2.5, spatial_rear_gap = 2.0;
  double front_gap=-1.0, rear_gap=-1.0; // Negative restores the legacy shared standstill gap.
  double approach_distance = -1.0;
  bool approach_accel_compensation = false;
  double approach_accel_upper = 4.0;
  double approach_time_buffer = 0.5;
  double approach_pos = 0.0, approach_half = 0.0;

  double integration_dt=.02;
  bool bound_profile_acceleration=false;
  bool check_after_rear_clear=false;
  bool require_rear_clear = false; // Check until the full ego body leaves the merge.
  bool hold_for_leader = true; // Only predict a yield-line hold if the command actually uses it.
  bool rear_yield = false; // Rear reacts only after ego nose reaches the conflict.
  bool rear_ttc_at_entry = false; // Cooperative case: full spacing trajectory still checked.
  double rear_reaction = 0.5, rear_decel = 2.0;
  double ttc_rear = 1.5;       // 뒤차 최소 TTC [s]
  double late_margin = 0.5;    // 뒤차 검사용 진입 늦음 여유 [s]
  // 앞차가 합류점에서 이 거리 안으로 와야 출발한다 [m] (0 = 끔). 앞차가 멀 때
  // 자차가 움직이면 NPC 가 '가까워졌다'고 보고 서행/정지해 등속 예측이 깨졌다
  // (12:57/14:48/14:49 백: 앞차 -12~-15 m 에서 출발 -> 0.8~2 m/s 까지 양보).
  double leader_start_distance = 0.0;
  bool adaptive_leader_start = false;
  double leader_start_max_distance = 12.0; // NPC 반응을 피하기 위한 출발 거리 상한
  double departure_confirm_time = 0.0;
  // COMMIT evaluates exactly this pair; disappearance/intrusion rejects it.
  bool lock_gap = false;
  int target_follower = -1;
  double comfortable_decel = 2.5;
  double abort_margin = 0.5;
  double stop_epsilon = 0.5;
  double hold_distance = std::numeric_limits<double>::infinity();
  double front_response = 0.0, front_max_decel = 0.0; // Same braking reserve as live lead constraint.
  double follow_standstill_gap = 4.0;
  double follow_time_gap = 0.0; // Zero disables the external ACC cap model.
  double follow_range = 60.0;
  int max_skip = 12;
  double pre_departure_delay = 0.0;
  double pre_departure_max_speed = 2.0;
  double pre_departure_stop_distance = 0.0;
  double pre_departure_contact_distance = -1.0;
  double pre_departure_response = 0.3;
  bool preparation_profile=false;
};

struct SequentialResult {
  bool preview_ready = false;
  int motion_collision_id = -1;
  double future_departure = -1.0;
  double future_front_margin = std::numeric_limits<double>::infinity();
  double future_rear_margin = std::numeric_limits<double>::infinity();
  bool feasible = false;
  bool approach_used = false;
  int target_leader = -1;      // 다음 주기에 넘길 목표(앞차 id, -1 = 첫 차 앞)
  int resolved_leader = -1;    // 이번 주기 처음 본 목표(패스 전)
  int resolved_follower = -1;  // 그 목표의 뒤차(-1 = 없음)
  GapCandidate gap;            // leader/follower/depart_delay 채움
  double rear_margin = 0.0;    // 뒤차 간격 여유(최소) [m]
  double rear_ttc = 0.0;       // 뒤차 TTC(최소) [s]
  double front_margin = 0.0;
  bool departure_ready = false; // Feasible from now, without borrowing future wait time.
  double clear_time=-1.0, observed_free_length=0.0;
  double physical_free_length=0.0;
  double preparation_speed=0.0, preparation_stop_distance=0.0, launch_speed_limit=0.0;
  std::string observation_failure;
  std::vector<int> skipped;    // 이번 주기에 패스한 틈의 뒤차 id
};

class GapSelector {
 public:
  // target_leader: 이전 목표(앞차 id). -2 = 처음, 없어졌으면 가장 가까운 하류 차량.
  static SequentialResult evaluateSequential(
      const std::vector<StreamVehicle>& vehicles, const EgoEntryModel& ego,
      const GapSelectorParams& params, const SequentialParams& seq,
      int target_leader);

  static SequentialResult evaluateSpatial(
      const std::vector<StreamVehicle>& vehicles, const EgoEntryModel& ego,
      const SequentialParams& seq, int target_leader);

  // 자차가 합류 구역(앞끝이 합류점 -entry_zone)에 닿는 시간(지금 출발).
  static double timeToZone(const EgoEntryModel& ego, const GapSelectorParams& params);

  // 모든 틈을 평가한다(앞차가 먼저, 즉 하류 쪽 틈부터).
  static std::vector<GapCandidate> evaluate(
      const std::vector<StreamVehicle>& vehicles, const EgoEntryModel& ego,
      const GapSelectorParams& params);

  // depart_delay 로 출발했을 때 이 틈이 되는지. 부족하면 side/shortfall 채움.
  static bool feasibleAt(const GapCandidate& gap, const EgoEntryModel& ego,
                         const GapSelectorParams& params, double depart_delay,
                         double* merge_time, std::string* side,
                         double* shortfall, double* front_shortfall = nullptr,
                         double* rear_shortfall = nullptr);

  // 자차가 출발 뒤 t 초에 간 거리와 속도(출발 지연 포함).
  static double entryAcceleration(const EgoEntryModel& ego, double speed);
  static void egoProfile(const EgoEntryModel& ego, double t_since_depart,
                         double* s, double* v);
};

// Speed in path metres/second to reach the entry only after front clearance.
double timedApproachSpeedCap(double remaining, double clearance_gap,
                            double leader_speed, double response,
                            double ego_speed = 0.0, double acceleration = 0.0);
}  // namespace mission_planner
