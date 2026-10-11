#pragma once

#include <deque>
#include <limits>
#include <map>
#include <set>

#include "mission_planner/types.hpp"
#include <string>

namespace mission_planner {

class RoundaboutMission {
 public:
  explicit RoundaboutMission(const RoundaboutConfig& config);

  void setConfig(const RoundaboutConfig& config);
  void reset(double now = 0.0);
  RoundaboutOutput update(const RoundaboutInput& input);
  RoundaboutState state() const { return state_; }

  static const char* stateName(RoundaboutState state);

 private:
  bool spatial_approach_braking_ = false;
  double preparation_since_=-1.0, preparation_scan_=-1.0, preparation_observation_=-1.0;
  int preparation_leader_=-2, preparation_follower_=-2;
  double approved_merge_speed_=0.0;
  bool bodyCollisionRisk(const RoundaboutInput& input, double speed_cap, int* object_id) const;
  struct GapResult {
    bool safe = false;
    bool preview_ready = false;
    double preview_stop_distance = 0.0;
    double future_departure = -1.0;
    double future_front = std::numeric_limits<double>::infinity();
    double future_rear = std::numeric_limits<double>::infinity();
    std::string switch_reason;
    double observed_free_length=0.0;
    double physical_gap_length=0.0, preparation_speed=0.0, launch_speed_limit=0.0;
    bool preparation_valid=false;
    std::string trajectory_observation="unknown";
    int blocking_object_id = -1;
    double next_npc_ttc = std::numeric_limits<double>::infinity();
    double front_gap = std::numeric_limits<double>::infinity();
    std::string reason;
    // 틈 선택 결과(gap_select_enabled)
    bool has_leader = false;
    int leader_id = -1;
    int follower_id = -1;
    double leader_pos = 0.0;
    double leader_speed = 0.0;
    double leader_length = 0.0;
    double depart_delay = std::numeric_limits<double>::infinity();
    double conflict_distance = std::numeric_limits<double>::infinity();
    int wrap = 0;
    double ego_zone_time = std::numeric_limits<double>::infinity();
    double ego_prediction_speed = 0.0;
    double entry_approach_cap = std::numeric_limits<double>::infinity();
    double ego_reference_time = 0.0;
    double ego_delay_remaining = 0.0;
    bool ego_leader_limited = false, ego_curve_limited = false;
    double circle_distance = std::numeric_limits<double>::infinity();  // 원 판정 반경까지 경로거리
    bool strict_go = false;   // depart_delay <= go_tolerance (앞차 허용 없이)
    // 뒤차 조건은 지금 만족하고 앞차 쪽만 commit_front_tolerance 이내로 기다리는 틈.
    // COMMIT 중에는 '진입할 때 고른 그 틈'일 때만 계속 진입 근거로 쓴다.
    double predicted_front = std::numeric_limits<double>::infinity(), predicted_rear = std::numeric_limits<double>::infinity();
    double current_rear = std::numeric_limits<double>::infinity(), rear_decel = 0.0, clear_time = 0.0;
    bool front_wait_ok = false;
  };

  struct LeadResult {
    bool active = false;
    int id = -1;
    double distance = 0.0;
    double speed = 0.0;
  };

  double distanceToYield(const RoundaboutInput& input) const;
  bool entryHeadingMatches(const EgoState& ego) const;
  bool egoInsideCircle(const EgoState& ego) const;
  bool egoEnteredMerge(const RoundaboutInput& input, double conflict_distance) const;
  double yieldRelaxation(double now) const;
  double relaxedThreshold(double configured, double floor,
                          double relaxation) const;
  GapResult evaluateGap(const RoundaboutInput& input,
                        double distance_to_yield) const;
  LeadResult findLeadVehicle(const RoundaboutInput& input);
  TrackedObject body_front_cache_;
  double body_front_cache_stamp_ = -1.0;
  bool seq_timed_approach_ = false, committed_timed_approach_ = false;
  GapResult evaluateGapSelect(const RoundaboutInput& input,
                              double distance_to_yield);
  GapResult evaluateDistance(const RoundaboutInput& input, double distance_to_yield);
  std::map<int, TrackedObject> distance_cache_;
  int distance_leader_ = -2, distance_follower_ = -2, distance_wrap_ = 0;
  double distance_bad_since_ = -1.0;
  GapResult evaluateTimeWindow(const RoundaboutInput& input,
                               double distance_to_yield);
  // 시간창 모델: 트랙별 첫 관측/마지막 실제 검출(hit_count 증가) 시각
  struct TrackMeta { double first_seen = 0.0; double last_hit_time = 0.0; int last_hits = -1; double last_seen = 0.0;
                     double stop_since = -1.0;
                     std::deque<std::pair<double, double>> lengths; double length = 4.5; };
  std::map<int, TrackMeta> track_meta_;
  double tw_stream_ok_since_ = -1.0;
  double egoTimeToDistance(double dist, double v0, double vmax) const;
  Point2 effectiveConflictPoint(const RoundaboutInput& input);
  double conflictDistance(const RoundaboutInput& input,
                          const Point2& conflict,
                          double distance_to_yield) const;
  bool isCirculatingObject(const TrackedObject& object, double* angle,
                           double* tangential_speed) const;
  void transition(RoundaboutState next, double now);
  MissionConstraint baseConstraint(double max_speed,
                                   const std::string& reason) const;

  RoundaboutConfig config_;
  RoundaboutState state_ = RoundaboutState::kIdle;
  double state_enter_time_ = 0.0;
  double safe_since_ = -1.0;
  int confirm_leader_=-2, confirm_follower_=-2;
  double confirm_scan_stamp_=-1.0, confirm_observation_stamp_=-1.0;
  double approval_time_=-1.0, approval_distance_=-1.0;
  double approved_free_=0.0, approved_entry_=0.0, approved_clear_=0.0, approved_physical_=0.0;
  double mandatory_stop_since_ = -1.0;
  bool mandatory_stop_completed_ = false;
  double commit_unsafe_since_ = -1.0;
  bool commit_aborted_ = false;  // COMMIT 중 양보 복귀했음(합류 구역 앞 정지)
  bool have_previous_ego_ = false;
  Point2 previous_ego_position_;
  std::string failsafe_reason_ = "localization invalid";
  // 틈 선택 래치(앞/뒤 차량 id). -2 = 래치 없음.
  int gap_latch_leader_ = -2;
  int gap_latch_follower_ = -2;
  int gap_latch_wrap_ = 0;
  int seq_target_leader_ = -2;   // 순차 판단 목표(앞차 id), -2 = 처음
  double seq_infeasible_since_ = -1.0;
  double seq_last_feasible_time_ = -1.0;
  // 흐름 속도(모든 순환 차량이 같은 등속) 추정: 차량별 긴 구간 평균의 중앙값.
  struct StreamTravel {
    double t0 = -1.0;      // 처음 관측 시각
    double travel = 0.0;   // 누적 이동 호 거리 [m]
    double last_pos = 0.0;
    double last_t = -1.0;
  };
  std::map<int, StreamTravel> stream_travel_;
  std::map<int, std::pair<double, double>> stream_speed_samples_;  // id -> (시각, 속도)
  double stream_speed_ = -1.0;   // <0 = 표본 없음
  double last_zone_lead_ = 0.0, last_zone_rear_ = 0.0, last_zone_speed_ = 0.0;
  double last_zone_relax_ = 0.0;
  // 차량별 길이 추정(zone 규칙). id -> (시각, 관측 길이) 표본, id -> (시각, 추정) prior 기억.
  std::map<int, std::deque<std::pair<double, double>>> length_samples_;
  std::map<int, std::pair<double, double>> length_estimates_;
  double length_prior_ = -1.0;   // <0 = 아직 없음(gs_zone_vehicle_length 사용)
  double streamVehicleLength(const TrackedObject& object, double stamp);
  void updateLengthPrior(double stamp);
  // 대기 완화량 [m]. YIELD 에서 갱신하고 COMMIT 동안 유지한다(고른 틈의 창이 넓어지지 않게).
  double zone_relax_ = 0.0;
  double mandatory_stop_completed_time_ = -1.0;
  int zone_pair_leader_ = -2, zone_pair_follower_ = -2;
  double zone_pair_ok_since_ = -1.0;
  bool zone_pair_confirmed_ = false;
  bool zone_preconfirm_active_ = false;
  // 상호 대기 해소: id -> (정지 시작 시각, 마지막 관측 시각)
  std::map<int, std::pair<double, double>> stationary_since_;
  std::set<int> yield_released_ids_;      // 마지막 평가에서 하드 차단에서 뺀 차량
  std::set<int> committed_released_ids_;  // COMMIT 으로 넘어갈 때 래치
  // 가림 보호(expiry guard)만 걸린 COMMIT 은 grace 동안 stale 로 보지 않는다.
  double last_stream_ok_time_ = -1.0;
  bool trackerStreamUsable(const RoundaboutInput& input) const;
  // 차량별 원호 위치 이력(시각, 펼친 원호 [m]) - 등속 추정용
  std::map<int, std::deque<std::pair<double, double>>> arc_hist_;
  std::map<int, std::deque<std::pair<double, double>>> speed_hist_;
  double estimateSpeed(int id, double now, double angle, double fallback);
  double circleEntryDistance(const RoundaboutInput& input) const;
  // 합류 구역/원 판정 반경 앞에 서는 거리(둘 중 가까운 것)
  double holdStopDistance(const GapResult& gap) const;
  // COMMIT 으로 넘어갈 때 고른 틈(앞차/뒤차/wrap). 없으면 committed_valid_ = false.
  bool committed_valid_ = false;
  int committed_leader_ = -2;
  int committed_follower_ = -2;
  int committed_wrap_ = 0;
  // 진입 중 승인 앞차의 마지막 확인 뒤끝 위치(합류점 기준 원호)/속도/시각.
  double committed_leader_rear_ = -1e9, committed_leader_speed_ = 0.0, committed_leader_seen_ = -1.0;
  bool commitGapOk(const GapResult& gap) const {
    if (config_.distance_enabled) return gap.safe && committed_valid_ && gap.leader_id==committed_leader_ && gap.follower_id==committed_follower_;
    if (config_.tw_enabled) return gap.front_wait_ok;
    if (config_.gs_sequential && state_ == RoundaboutState::kCommit) {
      return committed_valid_ && gap.front_wait_ok &&
             gap.leader_id == committed_leader_ &&
             gap.follower_id == committed_follower_ && gap.wrap == committed_wrap_;
    }
    // 틈이 바뀌었으면 원래 기준(go_tolerance)만. 앞차 쪽 허용은 진입한 그 틈에만.
    return gap.strict_go ||
           (gap.front_wait_ok && committed_valid_ &&
            gap.leader_id == committed_leader_ &&
            gap.follower_id == committed_follower_ && gap.wrap == committed_wrap_);
  }
  bool have_auto_conflict_ = false;
  // 마지막 틈 선택 결과(시각화용)
  bool last_gap_select_active_ = false;
  Point2 last_conflict_point_;
  double last_conflict_angle_ = 0.0;
  std::vector<RoundaboutGapViz> last_gaps_;
  std::vector<RoundaboutVehicleViz> last_stream_;
  Point2 auto_conflict_;
};

}  // namespace mission_planner
