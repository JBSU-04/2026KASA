#include <cmath>

#include <gtest/gtest.h>

#include "mission_planner/geometry.hpp"
#include "mission_planner/roundabout_mission.hpp"

namespace mission_planner {
namespace {

constexpr double kPi = 3.14159265358979323846;

RoundaboutConfig testConfig() {
  RoundaboutConfig config;
  config.enabled = true;
  config.counter_clockwise = true;
  config.center = {0.0, 0.0};
  config.yield_point = {10.0, -2.0};
  config.conflict_point = {10.0, 0.0};
  config.exit_point = {0.0, 10.0};
  config.entry_heading = 0.5 * kPi;
  config.inner_radius = 8.0;
  config.outer_radius = 12.0;
  config.nominal_radius = 10.0;
  config.activation_distance = 40.0;
  config.decision_distance = 20.0;
  config.approach_speed = 6.0;
  config.ego_merge_time = 2.0;
  config.rear_time_gap = 3.0;
  config.safe_confirm_seconds = 0.5;
  config.minimum_circulate_seconds = 0.5;
  config.exit_hold_seconds = 0.5;
  config.exit_detection_radius = 2.0;
  config.exit_clear_distance = 8.0;
  config.reset_jump_distance = 100.0;
  return config;
}

RoundaboutInput approachInput(double now) {
  RoundaboutInput input;
  input.now = now;
  input.ego.position = {10.0, -10.0};
  input.ego.yaw = 0.5 * kPi;
  input.ego.speed = 4.0;
  input.ego.valid = true;
  input.localization_valid = true;
  input.tracker_stream_valid = true;
  input.route = {{10.0, -30.0}, {10.0, -2.0}, {10.0, 0.0}, {9.8, 2.0}};
  return input;
}

TrackedObject circulatingObject(int id, double angle, double speed) {
  TrackedObject object;
  object.id = id;
  object.position = {10.0 * std::cos(angle), 10.0 * std::sin(angle)};
  const double tangent = angle + 0.5 * kPi;
  object.velocity_x = speed * std::cos(tangent);
  object.velocity_y = speed * std::sin(tangent);
  object.length = 4.0;
  object.velocity_valid = true;
  object.has_moved = true;
  return object;
}

TEST(Geometry, UsesDirectionalCircularArc) {
  EXPECT_NEAR(circleArcDistance(-0.25 * kPi, 0.0, 10.0, true),
              2.5 * kPi, 1.0e-9);
  EXPECT_NEAR(circleArcDistance(0.25 * kPi, 0.0, 10.0, false),
              2.5 * kPi, 1.0e-9);
  EXPECT_NEAR(normalizeAngle(3.0 * kPi), -kPi, 1.0e-9);
}

TEST(RoundaboutMission, EmptyRoundaboutCommitsAfterStableGap) {
  RoundaboutMission mission(testConfig());

  RoundaboutOutput first = mission.update(approachInput(0.0));
  EXPECT_EQ(first.state, RoundaboutState::kApproach);
  EXPECT_TRUE(first.constraint.stop_required);
  EXPECT_EQ(first.constraint.reason, "confirming safe gap");

  RoundaboutOutput confirmed = mission.update(approachInput(0.6));
  EXPECT_EQ(confirmed.state, RoundaboutState::kCommit);
  EXPECT_FALSE(confirmed.constraint.stop_required);

  RoundaboutInput inside = approachInput(0.8);
  inside.ego.position = {10.0, 0.0};
  RoundaboutOutput circulating = mission.update(inside);
  EXPECT_EQ(circulating.state, RoundaboutState::kCirculate);
}

TEST(RoundaboutMission, FastUpstreamVehicleBlocksEntry) {
  RoundaboutMission mission(testConfig());
  RoundaboutInput input = approachInput(0.0);
  input.objects.push_back(circulatingObject(42, -0.30, 5.0));

  const RoundaboutOutput output = mission.update(input);
  EXPECT_EQ(output.state, RoundaboutState::kYield);
  EXPECT_FALSE(output.gap_safe);
  EXPECT_TRUE(output.constraint.stop_required);
  EXPECT_EQ(output.blocking_object_id, 42);
  EXPECT_LT(output.next_npc_ttc, 1.0);
}

TEST(RoundaboutMission, RearTimeCanBeUsedAsTotalTtcThreshold) {
  RoundaboutConfig config = testConfig();
  config.rear_time_gap = 1.5;
  config.rear_time_is_total_ttc = true;
  RoundaboutMission mission(config);
  RoundaboutInput input = approachInput(0.0);
  // 충돌점까지 약 3초인 차량: 기존 merge+margin(약 4.8초)에는 막히지만
  // total-TTC 모드의 설정값 1.5초보다 크므로 진입 가능한 gap이다.
  input.objects.push_back(circulatingObject(43, -1.7, 5.0));

  const RoundaboutOutput output = mission.update(input);
  EXPECT_TRUE(output.gap_safe);
  EXPECT_EQ(output.blocking_object_id, -1);
  EXPECT_GT(output.next_npc_ttc, config.rear_time_gap);
}

TEST(RoundaboutMission, ReportsRawTtcAndAddsPredictionMarginToLimit) {
  RoundaboutConfig config = testConfig();
  config.rear_time_gap = 0.5;
  config.rear_time_is_total_ttc = true;
  config.prediction_uncertainty_time = 0.5;
  RoundaboutMission mission(config);
  RoundaboutInput input = approachInput(0.0);
  input.objects.push_back(circulatingObject(44, -0.5, 5.0));

  const RoundaboutOutput output = mission.update(input);
  EXPECT_FALSE(output.gap_safe);
  EXPECT_NEAR(output.next_npc_ttc, 0.6, 1.0e-9);
  EXPECT_NE(output.reason.find("TTC 0.60s < limit 1.00s"),
            std::string::npos);
  EXPECT_NE(output.reason.find("arc 3.00m / 5.00m/s"),
            std::string::npos);
}

TEST(RoundaboutMission, UsesAverageDepartureSpeedForFrontGap) {
  RoundaboutConfig config = testConfig();
  config.front_standstill_distance = 0.0;
  config.front_time_gap = 2.0;
  config.front_departure_speed_ratio = 0.5;

  RoundaboutInput blocked_input = approachInput(0.0);
  blocked_input.ego.speed = 0.0;
  blocked_input.objects.push_back(circulatingObject(45, 0.7, 5.0));
  const RoundaboutOutput blocked = RoundaboutMission(config).update(blocked_input);
  EXPECT_FALSE(blocked.gap_safe);
  EXPECT_NE(blocked.reason.find("gap 5.00m < limit 6.00m"),
            std::string::npos);

  RoundaboutInput safe_input = approachInput(0.0);
  safe_input.ego.speed = 0.0;
  safe_input.objects.push_back(circulatingObject(46, 0.9, 5.0));
  const RoundaboutOutput safe = RoundaboutMission(config).update(safe_input);
  EXPECT_TRUE(safe.gap_safe);
}

TEST(RoundaboutMission, RequiresConfigurableFullStopBeforeGapCommit) {
  RoundaboutConfig config = testConfig();
  config.mandatory_stop_seconds = 2.0;
  config.mandatory_stop_position_tolerance = 0.6;
  config.mandatory_stop_speed_threshold = 0.25;
  config.outer_radius = 10.0;
  RoundaboutMission mission(config);

  RoundaboutInput stopped = approachInput(0.0);
  stopped.ego.position = {10.0, -3.5};
  stopped.ego.speed = 0.0;
  RoundaboutOutput start = mission.update(stopped);
  EXPECT_EQ(start.state, RoundaboutState::kYield);
  EXPECT_TRUE(start.constraint.stop_required);
  EXPECT_NE(start.reason.find("mandatory stop 0.0/2.0s"), std::string::npos);

  stopped.now = 1.9;
  RoundaboutOutput holding = mission.update(stopped);
  EXPECT_EQ(holding.state, RoundaboutState::kYield);
  EXPECT_TRUE(holding.constraint.stop_required);

  stopped.now = 2.1;
  RoundaboutOutput completed = mission.update(stopped);
  EXPECT_EQ(completed.state, RoundaboutState::kYield);
  EXPECT_TRUE(completed.constraint.stop_required);
  EXPECT_EQ(completed.reason, "confirming safe gap");

  stopped.now = 2.7;
  RoundaboutOutput committed = mission.update(stopped);
  EXPECT_EQ(committed.state, RoundaboutState::kCommit);
  EXPECT_FALSE(committed.constraint.stop_required);
}

TEST(RoundaboutMission, StaleTrackerStreamFailsClosedBeforeEntry) {
  RoundaboutMission mission(testConfig());
  RoundaboutInput input = approachInput(0.0);
  input.tracker_stream_valid = false;

  const RoundaboutOutput output = mission.update(input);
  EXPECT_EQ(output.state, RoundaboutState::kYield);
  EXPECT_TRUE(output.constraint.stop_required);
  EXPECT_EQ(output.reason, "LiDAR tracker stream stale");
}

TEST(RoundaboutMission, StaticInfrastructureDoesNotBlockEntry) {
  RoundaboutMission mission(testConfig());
  RoundaboutInput input = approachInput(0.0);
  TrackedObject structure = circulatingObject(11, 0.15, 0.0);
  structure.has_moved = false;
  input.objects.push_back(structure);

  const RoundaboutOutput output = mission.update(input);
  EXPECT_TRUE(output.gap_safe);
  EXPECT_EQ(output.blocking_object_id, -1);
}

TEST(RoundaboutMission, UncertainNewTrackBlocksNearConflict) {
  RoundaboutMission mission(testConfig());
  RoundaboutInput input = approachInput(0.0);
  TrackedObject uncertain = circulatingObject(12, -0.1, 0.0);
  uncertain.velocity_valid = false;
  uncertain.has_moved = false;
  input.objects.push_back(uncertain);

  const RoundaboutOutput output = mission.update(input);
  EXPECT_FALSE(output.gap_safe);
  EXPECT_EQ(output.blocking_object_id, 12);
  EXPECT_EQ(output.reason, "object motion not confirmed");
}

TEST(RoundaboutMission, LongYieldRelaxesRearTtcToConfiguredFloor) {
  RoundaboutConfig config = testConfig();
  config.rear_time_is_total_ttc = true;
  config.rear_time_gap = 3.0;
  config.rear_ttc_relax_floor = 1.5;
  config.rear_ttc_relax_max = 1.5;
  config.gap_relax_start_seconds = 2.0;
  config.gap_relax_rate = 0.5;
  config.prediction_uncertainty_time = 0.5;
  RoundaboutMission mission(config);

  RoundaboutInput input = approachInput(1.0);
  input.objects.push_back(circulatingObject(50, -1.5, 5.0));
  EXPECT_FALSE(mission.update(input).gap_safe);

  input.now = 5.0;
  const RoundaboutOutput relaxed = mission.update(input);
  EXPECT_TRUE(relaxed.gap_safe);
  EXPECT_NEAR(relaxed.next_npc_ttc, 2.6, 1.0e-9);
}

TEST(RoundaboutMission, LongYieldRelaxesFrontGap) {
  RoundaboutConfig config = testConfig();
  config.rear_time_gap = 0.0;
  config.prediction_uncertainty_time = 0.0;
  config.front_time_gap = 2.0;
  config.front_time_relax_floor = 1.0;
  config.front_departure_speed_ratio = 0.5;
  config.front_standstill_distance = 0.0;
  config.gap_relax_start_seconds = 2.0;
  config.gap_relax_rate = 0.5;
  RoundaboutMission mission(config);

  RoundaboutInput input = approachInput(1.0);
  input.objects.push_back(circulatingObject(51, 0.7, 5.0));
  EXPECT_FALSE(mission.update(input).gap_safe);

  input.now = 5.0;
  const RoundaboutOutput relaxed = mission.update(input);
  EXPECT_TRUE(relaxed.gap_safe);
  EXPECT_NEAR(relaxed.front_gap, 5.0, 1.0e-9);
}

TEST(RoundaboutMission, LongYieldShortensSafeConfirmation) {
  RoundaboutConfig config = testConfig();
  config.safe_confirm_seconds = 1.0;
  config.safe_confirm_relax_floor = 0.2;
  config.gap_relax_start_seconds = 2.0;
  config.gap_relax_rate = 0.4;
  RoundaboutMission mission(config);

  RoundaboutInput blocked = approachInput(1.0);
  blocked.objects.push_back(circulatingObject(52, -0.3, 5.0));
  ASSERT_EQ(mission.update(blocked).state, RoundaboutState::kYield);

  RoundaboutInput clear = approachInput(5.0);
  EXPECT_EQ(mission.update(clear).state, RoundaboutState::kYield);
  clear.now = 5.21;
  EXPECT_EQ(mission.update(clear).state, RoundaboutState::kCommit);
}

TEST(RoundaboutMission, LongYieldDoesNotRelaxUncertainObjectBlock) {
  RoundaboutMission mission(testConfig());
  RoundaboutInput input = approachInput(1.0);
  TrackedObject uncertain = circulatingObject(53, -0.1, 0.0);
  uncertain.velocity_valid = false;
  uncertain.has_moved = false;
  input.objects.push_back(uncertain);
  ASSERT_FALSE(mission.update(input).gap_safe);

  input.now = 100.0;
  const RoundaboutOutput still_blocked = mission.update(input);
  EXPECT_FALSE(still_blocked.gap_safe);
  EXPECT_EQ(still_blocked.blocking_object_id, 53);
  EXPECT_EQ(still_blocked.reason, "object motion not confirmed");
}

TEST(RoundaboutMission, LocalizationLossStopsActiveMission) {
  RoundaboutMission mission(testConfig());
  mission.update(approachInput(0.0));

  RoundaboutInput invalid = approachInput(0.1);
  invalid.localization_valid = false;
  const RoundaboutOutput output = mission.update(invalid);
  EXPECT_EQ(output.state, RoundaboutState::kFailsafe);
  EXPECT_TRUE(output.constraint.stop_required);
  EXPECT_DOUBLE_EQ(output.constraint.max_speed, 0.0);
}

TEST(RoundaboutMission, CommitDoesNotReturnToYield) {
  RoundaboutMission mission(testConfig());
  mission.update(approachInput(0.0));
  ASSERT_EQ(mission.update(approachInput(0.6)).state,
            RoundaboutState::kCommit);

  RoundaboutInput blocked = approachInput(0.7);
  blocked.objects.push_back(circulatingObject(7, -0.2, 6.0));
  const RoundaboutOutput output = mission.update(blocked);
  EXPECT_EQ(output.state, RoundaboutState::kCommit);
  EXPECT_FALSE(output.constraint.stop_required);
}

TEST(RoundaboutMission, TrackerLossAfterCommitUsesDegradedSpeed) {
  RoundaboutConfig config = testConfig();
  config.degraded_speed = 1.5;
  config.tracker_stale_grace = 0.0;  // 순간 끊김 유예 없이 기존 동작 확인
  RoundaboutMission mission(config);
  mission.update(approachInput(0.0));
  ASSERT_EQ(mission.update(approachInput(0.6)).state,
            RoundaboutState::kCommit);

  RoundaboutInput stale = approachInput(0.7);
  stale.tracker_stream_valid = false;
  const RoundaboutOutput output = mission.update(stale);
  EXPECT_EQ(output.state, RoundaboutState::kCommit);
  EXPECT_DOUBLE_EQ(output.constraint.max_speed, 1.5);
  EXPECT_FALSE(output.constraint.stop_required);
}

TEST(RoundaboutMission, ClearsAfterSelectedExit) {
  RoundaboutMission mission(testConfig());
  mission.update(approachInput(0.0));
  mission.update(approachInput(0.6));

  RoundaboutInput inside = approachInput(0.8);
  inside.ego.position = {10.0, 0.0};
  ASSERT_EQ(mission.update(inside).state, RoundaboutState::kCirculate);

  RoundaboutInput at_exit = approachInput(1.4);
  at_exit.ego.position = {0.0, 10.0};
  ASSERT_EQ(mission.update(at_exit).state, RoundaboutState::kExit);

  RoundaboutInput clear = approachInput(2.0);
  clear.ego.position = {-10.0, 10.0};
  EXPECT_EQ(mission.update(clear).state, RoundaboutState::kIdle);
}

// 틈 선택: 들어가던 틈이 무너지면(늦게 잡힌 뒤차) 링 앞에서 설 수 있을 때 양보로 돌아가고
// 양보선이 아니라 합류 구역 앞에 세운다.
TEST(RoundaboutGapSelect, CommitAbortsWhenLateFollowerAppears) {
  RoundaboutConfig config = testConfig();
  config.yield_point = {10.0, -9.0};
  config.conflict_point = {10.0, 0.0};
  config.gap_select_enabled = true;
  config.mandatory_stop_seconds = 0.0;
  config.gs_confirm_seconds = 0.0;
  config.gs_commit_abort_confirm = 0.0;
  config.comfortable_decel = 2.5;
  RoundaboutMission mission(config);
  RoundaboutInput input = approachInput(0.0);
  input.ego.position = {10.0, -9.0};
  input.ego.speed = 0.0;
  input.route = {{10.0, -30.0}, {10.0, -9.0}, {10.0, 0.0}, {9.8, 2.0}};
  mission.update(input);
  input.now = 0.1;
  RoundaboutOutput out = mission.update(input);
  ASSERT_EQ(out.state, RoundaboutState::kCommit);
  // 출발 직후 합류점 6 m 상류에 8 m/s 차량이 새로 잡힌다.
  input.now = 0.3;
  input.ego.position = {10.0, -8.5};
  input.ego.speed = 1.0;
  TrackedObject late = circulatingObject(7, -0.6, 8.0);
  input.objects = {late};
  mission.update(input);
  input.now = 0.4;
  out = mission.update(input);
  EXPECT_EQ(out.state, RoundaboutState::kYield);
  EXPECT_TRUE(out.constraint.stop_required);
  // A cancelled entry must brake now rather than accelerate toward a later line.
  EXPECT_DOUBLE_EQ(out.constraint.stop_distance, 0.0);
  EXPECT_DOUBLE_EQ(out.constraint.max_speed, 0.0);
}

// 상류에서 다가오는 앞차를 먼저 보내는 틈으로 진입(COMMIT)하면, 앞차가
// 합류점을 지나기 전까지 합류 구역 앞에 정지 목표를 둔다(2026-09-30 12:34 백).
TEST(RoundaboutGapSelect, CommitHoldsBeforeZoneUntilUpstreamLeaderPasses) {
  RoundaboutConfig config = testConfig();
  config.yield_point = {10.0, -9.0};
  config.conflict_point = {10.0, 0.0};
  config.gap_select_enabled = true;
  config.mandatory_stop_seconds = 0.0;
  config.gs_confirm_seconds = 0.0;
  config.gs_front_headway = 0.3;
  config.gs_rear_headway = 0.5;
  config.gs_standstill_gap = 0.8;
  config.gs_check_horizon = 2.0;
  config.gs_speed_margin = 0.2;
  config.gs_go_tolerance_front = 12.0;  // 앞차 쪽 대기와 상관없이 진입시켜 본다
  config.gs_commit_front_tolerance = 12.0;
  config.gs_leader_release_margin = 0.0;  // 앞차 통과 예측 해제 없이 기존 동작 확인
  RoundaboutMission mission(config);
  RoundaboutInput input = approachInput(0.0);
  input.ego.position = {10.0, -9.0};
  input.ego.speed = 0.0;
  input.route = {{10.0, -30.0}, {10.0, -9.0}, {10.0, 0.0}, {9.8, 2.0}};
  // 앞차: 합류점(각 0) 상류 약 6 m(각 -0.6 rad), 4 m/s. 뒤차 없음.
  input.objects = {circulatingObject(3, -0.6, 4.0)};
  mission.update(input);
  input.now = 0.1;
  RoundaboutOutput out = mission.update(input);
  ASSERT_EQ(out.state, RoundaboutState::kCommit);
  EXPECT_TRUE(out.constraint.stop_required);
  EXPECT_GT(out.constraint.stop_distance, 1.0);
  // 앞차가 합류점을 충분히 지나면 정지 목표가 풀린다.
  input.now = 0.2;
  input.objects = {circulatingObject(3, 0.6, 4.0)};
  out = mission.update(input);
  EXPECT_EQ(out.state, RoundaboutState::kCommit);
  EXPECT_FALSE(out.constraint.stop_required);
}

// 2026-10-05: 진입 확정 뒤 앞차가 자차 도착 전에 합류점을 비우면(움직이는 앞차) 세우지 않는다.
//   앞차가 느리면(통과가 자차 도착보다 늦으면) 기존처럼 세운다.
TEST(RoundaboutGapSelect, CommitReleasesWhenLeaderClearsBeforeArrival) {
  for (double leader_speed : {4.0, 2.0}) {
    RoundaboutConfig config = testConfig();
    config.yield_point = {10.0, -9.0};
    config.conflict_point = {10.0, 0.0};
    config.gap_select_enabled = true;
    config.mandatory_stop_seconds = 0.0;
    config.gs_confirm_seconds = 0.0;
    config.gs_front_headway = 0.3;
    config.gs_rear_headway = 0.5;
    config.gs_standstill_gap = 0.8;
    config.gs_check_horizon = 2.0;
    config.gs_speed_margin = 0.2;
    config.gs_go_tolerance_front = 12.0;
    config.gs_commit_front_tolerance = 12.0;
    config.gs_leader_release_margin = 0.6;
    config.gs_leader_release_accel = 1.0;
    config.gs_leader_release_min_speed = 2.0;
    RoundaboutMission mission(config);
    RoundaboutInput input = approachInput(0.0);
    input.ego.position = {10.0, -9.0};
    input.ego.speed = 0.0;
    input.route = {{10.0, -30.0}, {10.0, -9.0}, {10.0, 0.0}, {9.8, 2.0}};
    input.objects = {circulatingObject(3, -0.6, leader_speed)};
    mission.update(input);
    input.now = 0.1;
    RoundaboutOutput out = mission.update(input);
    if (out.state != RoundaboutState::kCommit) continue;  // 느린 앞차는 진입 확정 전일 수 있다
    if (leader_speed >= 4.0)
      EXPECT_FALSE(out.constraint.stop_required) << out.constraint.reason;
    else
      EXPECT_TRUE(out.constraint.stop_required) << out.constraint.reason;
  }
}

// 2026-10-05: 접근/양보 중 트래커 순간 끊김(유예 이내)은 판단을 끊지 않는다.
TEST(RoundaboutMission, ShortTrackerStaleUsesGrace) {
  RoundaboutConfig config = testConfig();
  config.degraded_speed = 1.5;
  config.tracker_stale_grace = 0.3;
  RoundaboutMission mission(config);
  mission.update(approachInput(0.0));
  ASSERT_EQ(mission.update(approachInput(0.6)).state, RoundaboutState::kCommit);
  RoundaboutInput stale = approachInput(0.7);
  stale.tracker_stream_valid = false;
  stale.tracker_stream_fresh = true;
  const RoundaboutOutput output = mission.update(stale);
  EXPECT_EQ(output.state, RoundaboutState::kCommit);
  EXPECT_GT(output.constraint.max_speed, 1.5);
}

// 순차 판단: 앞차 대기 정지점은 원 판정 반경(outer + margin) 밖에 둔다.
//   (12:57 백: 정지점이 반경 안이라 대기하러 가다 CIRCULATE 로 넘어갔다)
TEST(RoundaboutGapSelect, SequentialHoldStopsOutsideCircle) {
  RoundaboutConfig config = testConfig();
  config.yield_point = {10.0, -9.0};
  config.conflict_point = {10.0, 0.0};
  config.gap_select_enabled = true;
  config.gs_sequential = true;
  config.gs_skip_confirm = 0.0;
  config.mandatory_stop_seconds = 0.0;
  config.gs_confirm_seconds = 0.0;
  config.gs_standstill_gap = 0.8;
  config.gs_front_headway = 0.3;
  config.gs_rear_headway = 0.5;
  config.gs_check_horizon = 2.0;
  config.gs_speed_margin = 0.2;
  config.gs_leader_release_margin = 0.0;
  RoundaboutMission mission(config);
  RoundaboutInput input = approachInput(0.0);
  input.ego.position = {10.0, -9.0};
  input.ego.speed = 0.0;
  input.route = {{10.0, -30.0}, {10.0, -9.0}, {10.0, 0.0}, {9.8, 2.0}};
  input.objects = {circulatingObject(3, -0.6, 6.0)};
  mission.update(input);
  input.now = 0.1;
  RoundaboutOutput out = mission.update(input);
  ASSERT_EQ(out.state, RoundaboutState::kCommit) << out.reason;
  ASSERT_TRUE(out.constraint.stop_required);
  // 반경 12 + 0.5 = 12.5 -> y = -7.5, 자차(y=-9)에서 1.5 m
  EXPECT_NEAR(out.constraint.stop_distance, 1.5, 0.2);
}

// 순차 판단: 목표 틈이 안 돼도 skip_confirm 동안은 패스하지 않는다(새 트랙 노이즈).
TEST(RoundaboutGapSelect, SequentialSkipNeedsConfirmation) {
  RoundaboutConfig config = testConfig();
  config.yield_point = {10.0, -9.0};
  config.conflict_point = {10.0, 0.0};
  config.gap_select_enabled = true;
  config.gs_sequential = true;
  config.mandatory_stop_seconds = 0.0;
  config.gs_confirm_seconds = 0.0;
  config.gs_skip_confirm = 0.3;
  RoundaboutMission mission(config);
  RoundaboutInput input = approachInput(0.0);
  input.ego.position = {10.0, -9.0};
  input.ego.speed = 0.0;
  input.route = {{10.0, -30.0}, {10.0, -9.0}, {10.0, 0.0}, {9.8, 2.0}};
  // 합류점 바로 상류 6 m 차량: 그 앞 틈은 안 된다.
  input.objects = {circulatingObject(3, -0.6, 4.0)};
  RoundaboutOutput out = mission.update(input);
  EXPECT_NE(out.reason.find("confirming skip"), std::string::npos) << out.reason;
  input.now = 0.2;
  out = mission.update(input);
  EXPECT_NE(out.reason.find("confirming skip"), std::string::npos) << out.reason;
  input.now = 0.4;
  out = mission.update(input);
  EXPECT_NE(out.reason.find("seq L3"), std::string::npos) << out.reason;
}

TEST(RoundaboutGapSelect, UpstreamSlotIsNotSkippedOnTransientRearRejection) {
  auto config = testConfig();
  config.yield_point = {10.0, -9.0};
  config.gap_select_enabled = true;
  config.gs_sequential = true;
  config.gs_confirm_seconds = 0.0;
  config.gs_skip_confirm = 0.3;
  RoundaboutMission mission(config);
  auto input = approachInput(0.0);
  input.ego.position = {10.0, -9.0};
  input.ego.speed = 0.0;
  input.objects = {circulatingObject(1, -1.0, 3.0)};
  mission.update(input);
  input.now = 0.4;
  auto out = mission.update(input); // Select the space behind vehicle 1.
  ASSERT_EQ(out.gap_leader_id, 1);
  input.objects.push_back(circulatingObject(2, -1.5, 8.0));
  input.now = 0.5;
  mission.update(input);
  input.now = 1.0; // Longer than skip_confirm, but leader is still upstream.
  out = mission.update(input);
  EXPECT_EQ(out.gap_leader_id, 1);
  EXPECT_EQ(out.gap_follower_id, 2);
  EXPECT_FALSE(out.gap_safe);
  EXPECT_TRUE(out.constraint.stop_required);
  input.objects[0] = circulatingObject(1, 0.0, 3.0);
  input.now = 1.1;
  out = mission.update(input);
  EXPECT_EQ(out.gap_leader_id, 2); // A genuinely missed slot can still advance.
}

// 미확정(속도 확인 전) 상류 트랙도 뒤차로 넣어 진입을 막는다.
TEST(RoundaboutGapSelect, UnconfirmedUpstreamTrackBlocksEntry) {
  RoundaboutConfig config = testConfig();
  config.yield_point = {10.0, -7.0};
  config.gap_select_enabled = true;
  config.mandatory_stop_seconds = 0.0;
  config.gs_confirm_seconds = 0.0;
  RoundaboutMission mission(config);
  RoundaboutInput input = approachInput(0.0);
  input.ego.position = {10.0, -7.0};  // 반경 12.2 m (링 밖), 합류점까지 7 m
  input.ego.speed = 0.0;
  TrackedObject young = circulatingObject(9, -0.9, 0.0);  // 상류 9 m
  young.velocity_valid = false;
  young.has_moved = false;
  young.confirmed = false;
  young.hit_count = 3;
  input.objects = {young};
  mission.update(input);
  input.now = 0.2;
  RoundaboutOutput out = mission.update(input);
  EXPECT_EQ(out.state, RoundaboutState::kYield);
  // 같은 물체가 오래 안 움직였으면(시설물) 무시하고 진입한다.
  input.objects[0].hit_count = 200;
  input.now = 0.4;
  mission.update(input);
  input.now = 0.6;
  out = mission.update(input);
  EXPECT_EQ(out.state, RoundaboutState::kCommit);
}

TEST(RoundaboutMission, UnsafeTargetRemainsVisibleWithoutEntryApproval) {
  RoundaboutConfig config = testConfig();
  config.gap_select_enabled = true;
  config.gs_sequential = true;
  config.gs_skip_confirm = 0.3;
  RoundaboutMission mission(config);
  auto input = approachInput(0.0);
  input.ego.speed = 0.0;
  input.objects = {circulatingObject(1, -1.0, 6.0),
                   circulatingObject(2, -1.5, 9.0)};
  const auto output = mission.update(input);
  EXPECT_FALSE(output.gap_safe);
  EXPECT_TRUE(output.constraint.stop_required);
  int selected = 0;
  for (const auto& gap : output.gaps) {
    if (!gap.selected) continue;
    ++selected;
    EXPECT_FALSE(gap.feasible);
    EXPECT_FALSE(gap.go);
  }
  EXPECT_EQ(selected, 1);
  ASSERT_FALSE(output.stream.empty());
  EXPECT_TRUE(output.stream.front().measured_position_valid);
  EXPECT_DOUBLE_EQ(output.stream.front().measured_position.x, input.objects.front().position.x);
}

TEST(RoundaboutGapSelect, RadialEntryAloneCannotReleaseYield) {
  auto config = testConfig();
  config.gap_select_enabled = true;
  config.gs_require_merge_progress = true;
  config.mandatory_stop_seconds = 1.0;
  RoundaboutMission mission(config);
  auto input = approachInput(0.0);
  input.ego.position = {10.0, -6.2}; // inside radius 12, still before merge zone
  input.ego.speed = 0.0;
  const auto out = mission.update(input);
  EXPECT_EQ(out.state, RoundaboutState::kYield);
  EXPECT_TRUE(out.constraint.stop_required);
  input.now = 0.1;
  input.ego.position = {10.0, -3.0}; // front has actually reached merge zone
  EXPECT_EQ(mission.update(input).state, RoundaboutState::kCirculate);
}

TEST(RoundaboutGapSelect, MpcLimitChangesPredictedArrivalWithoutChangingController) {
  auto config = testConfig();
  config.gap_select_enabled = true;
  config.gs_use_mpc_prediction = true;
  config.circulation_speed = 8.5;
  auto input = approachInput(0.0);
  input.ego.speed = 0.0;
  input.prediction_accel_limit = 1.8;
  input.prediction_speed_limit = 8.5;
  RoundaboutMission fast(config);
  const auto a = fast.update(input);
  input.prediction_speed_limit = 3.1;
  RoundaboutMission limited(config);
  const auto b = limited.update(input);
  EXPECT_NEAR(a.ego_prediction_speed, 8.5, 1e-9);
  EXPECT_NEAR(b.ego_prediction_speed, 3.1, 1e-9);
  // 6.7 m to the zone: accelerate at 1.8 to 3.1, then cruise.
  EXPECT_NEAR(b.ego_zone_time, 0.2 + 3.1 / 1.8 +
              (6.7 - 0.5 * 3.1 * 3.1 / 1.8) / 3.1, 1e-6);
  EXPECT_GT(b.ego_zone_time, a.ego_zone_time + 0.29);
  EXPECT_DOUBLE_EQ(b.constraint.max_speed, config.approach_speed);
}

TEST(RoundaboutGapSelect, MissingMpcFeedbackHoldsEntryThenRecovers) {
  auto config = testConfig();
  config.gap_select_enabled = true;
  config.gs_use_mpc_prediction = true;
  config.gs_confirm_seconds = 0.0;
  RoundaboutMission mission(config);
  auto input = approachInput(0.0);
  input.ego.speed = 0.0;
  const auto missing = mission.update(input);
  EXPECT_EQ(missing.state, RoundaboutState::kYield);
  EXPECT_TRUE(missing.constraint.stop_required);
  EXPECT_FALSE(missing.gap_safe);
  EXPECT_NE(missing.reason.find("MPC prediction feedback unavailable"), std::string::npos);
  input.now = 0.1;
  input.prediction_speed_limit = 3.1;
  EXPECT_EQ(mission.update(input).state, RoundaboutState::kCommit);
}

TEST(RoundaboutGapSelect, AbortBrakesDuringConfirmationAndCannotRecommitWhileMoving) {
  auto config = testConfig();
  config.gap_select_enabled = true;
  config.gs_require_merge_progress = true;
  config.gs_confirm_seconds = 0.0;
  config.gs_commit_abort_confirm = 0.2;
  RoundaboutMission mission(config);
  auto input = approachInput(0.0);
  input.ego.position = {10.0, -9.0};
  input.ego.speed = 0.0;
  ASSERT_EQ(mission.update(input).state, RoundaboutState::kCommit);
  input.now = 0.1;
  input.ego.position = {10.0, -6.2};
  input.ego.speed = 0.8;
  input.objects = {circulatingObject(7, -0.4, 8.0)};
  auto out = mission.update(input);
  EXPECT_EQ(out.state, RoundaboutState::kCommit);
  EXPECT_TRUE(out.constraint.stop_required);
  EXPECT_DOUBLE_EQ(out.constraint.max_speed, 0.0);
  input.now = 0.4;
  out = mission.update(input);
  EXPECT_EQ(out.state, RoundaboutState::kYield);
  EXPECT_DOUBLE_EQ(out.constraint.stop_distance, 0.0);
  input.now = 0.5;
  input.objects.clear(); // gap clear, but vehicle has not finished stopping
  out = mission.update(input);
  EXPECT_EQ(out.state, RoundaboutState::kYield);
  EXPECT_DOUBLE_EQ(out.constraint.max_speed, 0.0);
  input.now = 0.6;
  input.ego.speed = 0.0;
  EXPECT_EQ(mission.update(input).state, RoundaboutState::kCommit);
}

TEST(RoundaboutGapSelect, RepeatedLidarFrameDoesNotExtendVelocityHistory) {
  auto config = testConfig();
  config.gap_select_enabled = true;
  config.gs_sequential = true;
  config.gs_speed_min_span = 0.8;
  config.gs_speed_disagreement_limit = 1.0;
  RoundaboutMission mission(config);
  auto input = approachInput(0.0);
  input.ego.speed = 0.0;
  RoundaboutOutput out;
  for (int i = 0; i <= 5; ++i) {
    input.now = 0.2 * i;
    input.objects_stamp = input.now;
    // Position regression suggests 8 m/s; tracker says 4 m/s.
    input.objects = {circulatingObject(1, -2.0 + 0.16 * i, 4.0)};
    out = mission.update(input);
  }
  ASSERT_EQ(out.stream.size(), 1u);
  EXPECT_NEAR(out.stream[0].speed, 5.0, 1e-6);
  // Timer runs faster than LiDAR: use the sensor stamp, not timer time.
  for (int i = 1; i <= 4; ++i) {
    input.now = 1.0 + 0.05 * i;
    out = mission.update(input);
    ASSERT_EQ(out.stream.size(), 1u);
    EXPECT_NEAR(out.stream[0].speed, 5.0, 1e-6);
  }
}

TEST(RoundaboutGapSelect, DistantTargetIsWaitingNotRearUnsafeOrSkipped) {
  auto config = testConfig();
  config.gap_select_enabled = true;
  config.gs_sequential = true;
  config.gs_max_wait = 0.1;
  config.gs_leader_start_distance = 9.0;
  RoundaboutMission mission(config);
  auto input = approachInput(0.0);
  input.ego.speed = 0.0;
  input.objects = {circulatingObject(1, -2.0, 6.0)};
  for (int i = 0; i < 7; ++i) {
    input.now = i * 0.2;
    const auto out = mission.update(input);
    EXPECT_FALSE(out.gap_safe);
    EXPECT_TRUE(out.constraint.stop_required);
    // First reject the slot ahead of this car, then retain the slot behind it.
    if (i < 2) continue;
    EXPECT_NE(out.reason.find("exceeds prediction horizon"), std::string::npos);
    EXPECT_EQ(out.reason.find("unsafe, confirming skip"), std::string::npos);
  }
}

TEST(RoundaboutGapSelect, SingleEstimatorUsesSameArcUnitsForPositionAndSpeed) {
  auto config = testConfig();
  config.gap_select_enabled = true;
  config.gs_sequential = true;
  config.gs_use_tracker_velocity = true;
  RoundaboutMission mission(config);
  auto input = approachInput(0.0); input.ego.speed = 0.0;
  auto object = circulatingObject(8, -0.8, 5.4);
  object.position.x *= 0.9; object.position.y *= 0.9; // radius 9, nominal 10
  input.objects = {object};
  const auto out = mission.update(input);
  ASSERT_EQ(out.stream.size(), 1U);
  EXPECT_NEAR(out.stream[0].speed, 6.0, 1e-6);
}

TEST(RoundaboutGapSelect, PredictedPoseDoesNotBecomeSecondSpeedMeasurement) {
  auto config = testConfig();
  config.gap_select_enabled = true;
  config.gs_sequential = true;
  config.gs_use_tracker_velocity = true;
  RoundaboutMission mission(config);
  auto input = approachInput(0.0); input.ego.speed = 0.0;
  for (int i = 0; i <= 12; ++i) {
    input.now = i * 0.1; input.objects_stamp = input.now;
    // Position drift could fool a second regression; reported raw speed is stable.
    input.objects = {circulatingObject(8, -1.0 + i * 0.075, 5.4)};
    const auto out = mission.update(input);
    ASSERT_EQ(out.stream.size(), 1U);
    EXPECT_NEAR(out.stream[0].speed, 5.4, 1e-6);
  }
}

TEST(RoundaboutGapSelect, LeadFollowingKeepsPhysicalSpeedAfterArcConversion) {
  auto config = testConfig();
  config.gap_select_enabled = true;
  config.gs_sequential = true;
  config.gs_use_tracker_velocity = true;
  config.gs_confirm_seconds = 0.0;
  RoundaboutMission mission(config);
  auto input = approachInput(0.0); input.ego.speed = 0.0;
  auto object = circulatingObject(8, 0.5, 5.4);
  object.position.x *= 0.9; object.position.y *= 0.9;
  input.objects = {object};
  const auto out = mission.update(input);
  ASSERT_EQ(out.state, RoundaboutState::kCommit);
  ASSERT_TRUE(out.constraint.lead_vehicle_active);
  EXPECT_NEAR(out.stream[0].speed, 6.0, 1e-6);
  EXPECT_NEAR(out.stream[0].physical_speed, 5.4, 1e-6);
  EXPECT_NEAR(out.constraint.lead_speed, 5.4, 1e-6);
}

TEST(RoundaboutGapSelect, SequentialToleranceCannotApproveFutureDeparture) {
  auto config = testConfig();
  config.gap_select_enabled = config.gs_sequential = true;
  config.gs_use_tracker_velocity = true;
  config.gs_require_merge_progress = true;
  config.gs_skip_confirm = config.gs_confirm_seconds = 0.0;
  config.gs_seq_go_tolerance = 30.0; // Legacy permissive value must not bypass waiting.
  config.yield_point = {10.0, -9.0};
  RoundaboutMission mission(config);
  auto input = approachInput(0.0);
  input.ego.position = config.yield_point;
  input.ego.speed = 0.0;
  input.route = {{10.0, -30.0}, {10.0, -9.0}, {10.0, 0.0}, {9.8, 2.0}};
  input.objects = {circulatingObject(1, -1.6, 4.0)};
  const auto out = mission.update(input);
  EXPECT_EQ(out.state, RoundaboutState::kYield);
  EXPECT_FALSE(out.gap_safe);
  EXPECT_TRUE(out.constraint.stop_required);
  EXPECT_NE(out.reason.find("wait"), std::string::npos) << out.reason;
}

TEST(RoundaboutGapSelect, MissingCommittedFollowerStopsWithoutSelectingAnotherGap) {
  auto config = testConfig();
  config.gap_select_enabled = config.gs_sequential = true;
  config.gs_use_tracker_velocity = config.gs_require_merge_progress = true;
  config.gs_confirm_seconds = config.gs_skip_confirm = config.gs_commit_abort_confirm = 0.0;
  config.yield_point = {10.0, -9.0};
  RoundaboutMission mission(config);
  auto input = approachInput(0.0);
  input.ego.position = config.yield_point;
  input.ego.speed = 0.0;
  input.route = {{10.0, -30.0}, {10.0, -9.0}, {10.0, 0.0}, {9.8, 2.0}};
  input.objects = {circulatingObject(1, 0.5, 6.0), circulatingObject(2, -2.5, 1.0)};
  auto out = mission.update(input);
  ASSERT_EQ(out.state, RoundaboutState::kCommit) << out.reason;
  input.now = 0.1;
  input.objects.pop_back();
  out = mission.update(input);
  EXPECT_EQ(out.state, RoundaboutState::kYield);
  EXPECT_TRUE(out.constraint.stop_required);
  EXPECT_DOUBLE_EQ(out.constraint.max_speed, 0.0);
  EXPECT_NE(out.reason.find("target_missing"), std::string::npos) << out.reason;
  for (const auto& gap : out.gaps) {
    if (!gap.selected) continue;
    EXPECT_EQ(gap.leader_id, 1);
    EXPECT_EQ(gap.follower_id, 2);
    EXPECT_FALSE(gap.go);
  }
}

TEST(RoundaboutGapSelect, MeasuredSlowdownAddsFrontWaitingWithoutChangingReportedSpeed) {
  auto config = testConfig();
  config.gap_select_enabled = config.gs_sequential = config.gs_use_tracker_velocity = true;
  config.gs_require_merge_progress = true;
  config.gs_skip_confirm = 0.0;
  config.gs_leader_start_distance = 0.0;
  config.mandatory_stop_seconds = 100.0;
  config.yield_point = {10.0, -9.0};
  RoundaboutMission conservative(config);
  config.gs_leader_decel_horizon = 0.0;
  RoundaboutMission constant(config);
  auto input = approachInput(0.0);
  input.ego.position = config.yield_point;
  input.ego.speed = 0.0;
  input.route = {{10.0, -30.0}, {10.0, -9.0}, {10.0, 0.0}, {9.8, 2.0}};
  RoundaboutOutput slow, normal;
  for (int i = 0; i < 4; ++i) {
    input.now = input.objects_stamp = 0.2 * i;
    input.objects = {circulatingObject(1, -2.0 + i * 0.08, 6.0 - i * 0.5)};
    slow = conservative.update(input);
    normal = constant.update(input);
  }
  ASSERT_EQ(slow.stream.size(), 1u);
  EXPECT_NEAR(slow.stream.front().speed, 4.5, 1e-6);
  double slow_wait = -1.0, normal_wait = -1.0;
  for (const auto& gap : slow.gaps) if (gap.selected) slow_wait = gap.depart_delay;
  for (const auto& gap : normal.gaps) if (gap.selected) normal_wait = gap.depart_delay;
  ASSERT_GE(normal_wait, 0.0);
  EXPECT_GT(slow_wait, normal_wait);
  // Reusing the same sensor stamp cannot create a stronger slowdown trend.
  input.now += 0.1;
  const auto repeated = conservative.update(input);
  for (const auto& gap : repeated.gaps) {
    if (gap.selected) {
      EXPECT_DOUBLE_EQ(gap.depart_delay, slow_wait);
    }
  }
}

TEST(RoundaboutGapSelect, UnconfirmedFrontObjectCannotKeepCommittedGapApproved) {
  auto config = testConfig();
  config.gap_select_enabled = config.gs_sequential = true;
  config.gs_use_tracker_velocity = config.gs_require_merge_progress = true;
  config.gs_confirm_seconds = config.gs_skip_confirm = config.gs_commit_abort_confirm = 0.0;
  config.yield_point = {10.0, -9.0};
  RoundaboutMission mission(config);
  auto input = approachInput(0.0);
  input.ego.position = config.yield_point;
  input.ego.speed = 0.0;
  input.route = {{10.0, -30.0}, {10.0, -9.0}, {10.0, 0.0}, {9.8, 2.0}};
  input.objects = {circulatingObject(1, 0.5, 6.0), circulatingObject(2, -2.5, 1.0)};
  ASSERT_EQ(mission.update(input).state, RoundaboutState::kCommit);
  auto uncertain = circulatingObject(3, 0.1, 0.0);
  uncertain.confirmed = uncertain.velocity_valid = uncertain.has_moved = false;
  uncertain.hit_count = 1;
  input.objects.push_back(uncertain);
  input.now = 0.1;
  const auto out = mission.update(input);
  EXPECT_EQ(out.state, RoundaboutState::kYield);
  EXPECT_FALSE(out.gap_safe);
  EXPECT_DOUBLE_EQ(out.constraint.max_speed, 0.0);
  for (const auto& gap : out.gaps) EXPECT_FALSE(gap.go);
}

TEST(RoundaboutGapSelect, VehicleRoiRejectsInnerIslandSignWithFalseMotionFlag) {
  auto config = testConfig();
  config.vehicle_roi_inner_radius = 9.0;
  config.vehicle_roi_outer_radius = 11.0;
  config.gap_select_enabled = config.gs_sequential = true;
  config.gs_confirm_seconds = 0.0;
  config.gs_require_merge_progress = true;
  auto input = approachInput(0.0); input.ego.speed = 0.0;
  auto sign = circulatingObject(10, 0.5, 0.7);
  sign.position.x *= 0.85; sign.position.y *= 0.85;
  input.objects = {sign}; // A false has_moved flag must not admit an island object.
  RoundaboutMission mission(config);
  const auto out = mission.update(input);
  EXPECT_EQ(out.state, RoundaboutState::kCommit);
  EXPECT_TRUE(out.stream.empty());
  EXPECT_FALSE(out.constraint.lead_vehicle_active);
}

TEST(RoundaboutGapSelect, ProvenVehicleWithInvalidSpeedIsNotDroppedAsOldInfrastructure) {
  auto config = testConfig();
  config.gap_select_enabled = config.gs_sequential = true;
  config.gs_skip_confirm = 0.0;
  auto input = approachInput(0.0); input.ego.speed = 0.0;
  auto car = circulatingObject(7, -1.5, 5.0);
  car.confirmed = car.velocity_valid = false;
  car.has_moved = true; car.hit_count = 100;
  input.objects = {car};
  RoundaboutMission mission(config);
  const auto out = mission.update(input);
  ASSERT_EQ(out.stream.size(), 1u);
  EXPECT_EQ(out.stream.front().id, 7);
  EXPECT_FALSE(out.gap_safe);
  EXPECT_TRUE(out.constraint.stop_required);
}

// ---- 거리 기준 진입(zone 규칙): 차량별 길이, 대기 완화, COMMIT grace, 정지 차량 해제 ----

RoundaboutConfig zoneConfig() {
  auto config = testConfig();
  config.gap_select_enabled = config.gs_sequential = true;
  config.gs_zone_rule_enabled = true;
  config.gs_zone_confirm_seconds = 0.0;
  config.gs_commit_abort_enabled = false;
  config.gs_ego_length = 4.6;
  config.gs_entry_zone = 1.0;
  config.gs_standstill_gap = 0.8;
  config.gs_min_vehicle_length = 4.0;
  config.gs_zone_front_gap = 2.5;
  config.gs_zone_rear_gap = 2.0;
  config.gs_zone_front_time = 0.3;
  config.gs_zone_vehicle_length = 5.0;
  config.gs_velocity_hold_min_hits = 20;
  return config;
}

RoundaboutInput zoneInput(double now) {
  RoundaboutInput input = approachInput(now);
  input.ego.speed = 0.0;
  input.objects_stamp = now;
  return input;
}

// 합류점 상류 arc [m] 에 있는 순환 차량(반지름 10 m 라 각 = -arc / 10).
TrackedObject upstreamCar(int id, double arc, double speed, double length) {
  TrackedObject car = circulatingObject(id, -arc / 10.0, speed);
  car.length = length;
  car.hit_count = 50;
  car.confirmed = true;
  return car;
}

// 앞차(arc 12.6)와 뒤차(arc 12.6 + spacing)를 3 m/s 로 두고 어느 쪽 틈을 고르는지 본다.
int zoneChosenLeader(RoundaboutConfig config, double leader_length, double follower_length,
                     double spacing) {
  RoundaboutMission mission(config);
  RoundaboutInput input = zoneInput(0.0);
  input.objects = {upstreamCar(1, 12.6, 3.0, leader_length),
                   upstreamCar(2, 12.6 + spacing, 3.0, follower_length)};
  return mission.update(input).gap_leader_id;
}

TEST(RoundaboutZoneRule, WindowAndSlotWidthUseEachVehiclesOwnLength) {
  auto config = zoneConfig();
  config.gs_zone_length_estimate = true;
  config.gs_zone_length_min_samples = 1;
  config.gs_zone_length_max = 7.5;
  // v=3 m/s: 앞 간격 max(2.5, 0.8+1.0+0.3*3) = 2.7, 뒤 간격 2.0.
  //   W(4.5, 4.5) = 4.6 + 2.7 + 2.0 + 4.5 = 13.8,  W(7.0, 4.5) = 15.05.
  EXPECT_EQ(zoneChosenLeader(config, 4.5, 4.5, 14.5), 1);  // 틈 (1, 2) 통과
  EXPECT_EQ(zoneChosenLeader(config, 7.0, 4.5, 14.5), 2);  // 긴 앞차는 같은 간격이 모자라 건너뜀
  EXPECT_EQ(zoneChosenLeader(config, 4.5, 7.0, 14.5), 2);  // 긴 뒤차도 마찬가지
}

TEST(RoundaboutZoneRule, FixedLengthIsUsedWhenEstimationIsOff) {
  auto config = zoneConfig();
  config.gs_zone_length_estimate = false;
  // 관측 길이와 상관없이 기본 5.0 m: W = 4.6 + 2.7 + 2.0 + 5.0 = 14.3
  EXPECT_EQ(zoneChosenLeader(config, 7.0, 7.0, 14.5), 1);
  EXPECT_EQ(zoneChosenLeader(config, 4.5, 4.5, 14.0), 2);
}

TEST(RoundaboutZoneRule, LengthEstimateUsesPercentileAndPriorForPartialViews) {
  auto config = zoneConfig();
  config.gs_zone_length_estimate = true;
  config.gs_zone_length_percentile = 0.8;
  config.gs_zone_length_min_samples = 10;
  config.gs_zone_length_max = 7.5;
  RoundaboutMission mission(config);
  // 한 차량이 보이는 면에 따라 길이가 흔들린다(부분 관측 2.0~3.0, 정상 4.8~5.2).
  const double observed[] = {2.0, 3.0, 4.8, 5.0, 5.0, 5.0, 5.2, 5.0, 4.9, 5.1};
  RoundaboutOutput out;
  for (int i = 0; i < 10; ++i) {
    RoundaboutInput input = zoneInput(0.1 * i);
    input.objects = {upstreamCar(1, 30.0, 3.0, observed[i])};
    out = mission.update(input);
  }
  ASSERT_EQ(out.stream.size(), 1u);
  EXPECT_NEAR(out.stream.front().length, 5.0, 1e-9);  // 80 백분위수

  // 방금 생긴 차(표본 1 개, 부분 관측 3.2 m)는 스트림 prior(5.0)보다 작게 보지 않는다.
  RoundaboutInput input = zoneInput(1.0);
  input.objects = {upstreamCar(1, 27.0, 3.0, 5.0), upstreamCar(2, 20.0, 3.0, 3.2)};
  out = mission.update(input);
  double short_view = -1.0;
  for (const auto& v : out.stream) if (v.id == 2) short_view = v.length;
  EXPECT_NEAR(short_view, 5.0, 1e-9);

  // 합쳐진 클러스터(12 m)는 상한으로 자른다.
  for (int i = 0; i < 12; ++i) {
    RoundaboutInput merged = zoneInput(2.0 + 0.1 * i);
    merged.objects = {upstreamCar(3, 25.0, 3.0, 12.0)};
    out = mission.update(merged);
  }
  double merged_length = -1.0;
  for (const auto& v : out.stream) if (v.id == 3) merged_length = v.length;
  EXPECT_NEAR(merged_length, 7.5, 1e-9);
}

// 2026-10-05(20:02 백) 사전 확인 시뮬: 두 차가 3 m/s 로 합류점에 다가온다. jump_at 이후 뒤차가
// jump 만큼 앞으로 튄 것처럼 보인다(트랙 위치 튐). 첫 COMMIT 시각과 그때 고른 앞차를 돌려준다.
std::pair<double, int> zonePreconfirmRun(RoundaboutConfig config, double spacing,
                                         double jump_at = 1e9, double jump = 0.0) {
  config.gs_zone_length_estimate = false;
  config.gs_zone_vehicle_length = 4.5;   // W = 4.6 + 2.7 + 2.0 + 4.5 = 13.8
  RoundaboutMission mission(config);
  for (int k = 0; k <= 160; ++k) {
    const double t = 0.05 * k;
    RoundaboutInput input = zoneInput(t);
    const double lead_arc = 14.0 - 3.0 * t;   // 반지름 10 m(둘레 62.8): 뒤차도 반 바퀴(31.4) 안
    const double fol_arc = lead_arc + spacing - (t >= jump_at ? jump : 0.0);
    input.objects = {upstreamCar(1, lead_arc, 3.0, 4.5), upstreamCar(2, fol_arc, 3.0, 4.5)};
    const RoundaboutOutput out = mission.update(input);
    if (out.state == RoundaboutState::kCommit) return {t, out.gap_leader_id};
  }
  return {-1.0, -1};
}

// 출발 창 (14.5 - 13.8) / 3 = 0.23 s < 확인 0.3 s. 예전 방식(경계 통과 뒤 확인)은 이 틈을 놓치고,
// 사전 확인은 앞차가 경계를 지나는 순간 바로 들어간다.
TEST(RoundaboutZoneRule, PreconfirmTakesGapShorterThanConfirmTime) {
  auto config = zoneConfig();
  config.gs_zone_confirm_seconds = 0.3;
  config.gs_zone_preconfirm = false;
  const auto old_way = zonePreconfirmRun(config, 14.5);
  EXPECT_NE(old_way.second, 1) << "old commit t=" << old_way.first;
  config.gs_zone_preconfirm = true;
  const auto pre = zonePreconfirmRun(config, 14.5);
  EXPECT_EQ(pre.second, 1);
  EXPECT_GT(pre.first, 0.0);
}

// 확인을 마친 쌍은 간격 부족이 허용치(0.8 m) 이내인 튐에는 그대로 들어가고, 넘으면 건너뛴다.
TEST(RoundaboutZoneRule, ConfirmedPairToleratesSmallPositionJumpOnly) {
  auto config = zoneConfig();
  config.gs_zone_confirm_seconds = 0.3;
  config.gs_zone_preconfirm = true;
  config.gs_zone_pair_tolerance = 0.8;
  // 간격 14.3(여유 0.5) -> 1.0 s 뒤 0.9 m 튐(부족 0.4): 유지
  EXPECT_EQ(zonePreconfirmRun(config, 14.3, 1.0, 0.9).second, 1);
  // 1.5 m 튐(부족 1.0 > 0.8): 건너뜀
  EXPECT_NE(zonePreconfirmRun(config, 14.3, 1.0, 1.5).second, 1);
  // 허용치 0 이면 작은 튐에도 건너뜀(이전 동작)
  config.gs_zone_pair_tolerance = 0.0;
  EXPECT_NE(zonePreconfirmRun(config, 14.3, 1.0, 0.9).second, 1);
}

// 완화가 하한까지 다 되면 허용치는 0 이다(하한 아래로 들어가지 않는다).
TEST(RoundaboutZoneRule, PairToleranceIsZeroAtRelaxFloors) {
  auto config = zoneConfig();
  config.gs_zone_confirm_seconds = 0.3;
  config.gs_zone_preconfirm = true;
  config.gs_zone_pair_tolerance = 0.8;
  config.gs_zone_rear_gap_floor = config.gs_zone_rear_gap;   // 뒤 간격 남은 여유 없음(앞 여유는 빌리지 않는다)
  EXPECT_NE(zonePreconfirmRun(config, 14.3, 1.0, 0.9).second, 1);
}

TEST(RoundaboutZoneRule, LongYieldRelaxesGapsDownToFloors) {
  auto config = zoneConfig();
  config.gs_zone_length_estimate = false;
  config.gs_zone_vehicle_length = 4.5;
  config.gs_zone_relax_start = 5.0;
  config.gs_zone_relax_rate = 0.5;       // m/s
  config.gs_zone_rear_gap_floor = 1.0;
  config.gs_zone_front_gap_floor = 2.0;
  // 틈 12.8 m: 기본 W = 13.8 에는 모자라고, 하한까지 줄인 W = 4.6 + 2.0 + 1.0 + 4.5 = 12.1 에는 들어간다.
  RoundaboutMission mission(config);
  auto frame = [&](double now) {
    RoundaboutInput input = zoneInput(now);
    input.objects = {upstreamCar(1, 12.6, 3.0, 4.5), upstreamCar(2, 12.6 + 12.8, 3.0, 4.5)};
    return mission.update(input);
  };
  RoundaboutOutput out = frame(0.0);
  ASSERT_EQ(out.state, RoundaboutState::kYield);
  EXPECT_EQ(out.gap_leader_id, 2);                       // 틈 (1, 2) 는 건너뜀
  out = frame(5.5);                                       // 완화 0.25 m: 아직 부족
  EXPECT_EQ(out.gap_leader_id, 2);
  out = frame(7.5);                                       // 완화 1.25 m >= 1.0 필요
  EXPECT_EQ(out.gap_leader_id, 1);
  EXPECT_NE(out.reason.find("relax-"), std::string::npos);
  out = frame(30.0);                                      // 하한에서 멈춘다(1.7 m)
  EXPECT_EQ(out.gap_leader_id, 1);
  EXPECT_NE(out.reason.find("relax-1.7"), std::string::npos);
}

TEST(RoundaboutZoneRule, CommitGraceBridgesOcclusionGuardButNotMissingMessages) {
  RoundaboutConfig config = testConfig();
  config.degraded_speed = 1.5;
  config.tracker_guard_commit_grace = 1.2;
  RoundaboutMission mission(config);
  mission.update(approachInput(0.0));
  ASSERT_EQ(mission.update(approachInput(0.6)).state, RoundaboutState::kCommit);

  // 메시지는 신선하고 가림 보호만 걸림: grace 안에서는 감속하지 않는다.
  RoundaboutInput guarded = approachInput(0.7);
  guarded.tracker_stream_valid = false;
  guarded.tracker_stream_fresh = true;
  RoundaboutOutput out = mission.update(guarded);
  EXPECT_EQ(out.state, RoundaboutState::kCommit);
  EXPECT_GT(out.constraint.max_speed, 1.5);

  // 메시지가 끊기면(fresh=false) grace 와 상관없이 stale.
  RoundaboutInput lost = approachInput(0.8);
  lost.tracker_stream_valid = false;
  lost.tracker_stream_fresh = false;
  EXPECT_DOUBLE_EQ(mission.update(lost).constraint.max_speed, 1.5);

  // 가림 보호가 grace 보다 오래 이어지면 stale.
  RoundaboutInput expired = approachInput(2.0);
  expired.tracker_stream_valid = false;
  expired.tracker_stream_fresh = true;
  EXPECT_DOUBLE_EQ(mission.update(expired).constraint.max_speed, 1.5);
}

TEST(RoundaboutZoneRule, StationaryYieldingVehicleIsReleasedOnlyAfterWaiting) {
  auto config = zoneConfig();
  config.gs_zone_length_estimate = false;
  config.gs_stationary_yield_enabled = true;
  config.gs_stationary_yield_start = 5.0;
  config.gs_stationary_yield_confirm = 3.0;
  config.gs_stationary_yield_min_distance = 3.0;
  config.stationary_block_distance = 20.0;
  auto run = [&](bool car_between) {
    RoundaboutMission mission(config);
    RoundaboutOutput out;
    for (int t = 0; t <= 8; ++t) {
      RoundaboutInput input = zoneInput(static_cast<double>(t));
      // 합류점 9 m 상류에 서 있는 차(이동 이력 있음) = 자차를 기다리다 선 NPC.
      input.objects = {upstreamCar(7, 9.0, 0.0, 4.5)};
      if (car_between) input.objects.push_back(upstreamCar(8, 4.0, 0.0, 4.5));
      out = mission.update(input);
      if (t == 2) {
        EXPECT_EQ(out.state, RoundaboutState::kYield);
        EXPECT_NE(out.reason.find("stationary vehicle near conflict"), std::string::npos);
      }
    }
    return out;
  };
  RoundaboutOutput released = run(false);
  EXPECT_EQ(released.state, RoundaboutState::kCommit);
  // 기능을 끄면 8 초 기다려도 계속 막는다.
  config.gs_stationary_yield_enabled = false;
  EXPECT_EQ(run(false).state, RoundaboutState::kYield);
  config.gs_stationary_yield_enabled = true;
  // 그 차와 합류점 사이에 다른 차가 있으면 상호 대기가 아니므로 계속 막는다.
  RoundaboutOutput blocked = run(true);
  EXPECT_EQ(blocked.state, RoundaboutState::kYield);
  EXPECT_NE(blocked.reason.find("stationary vehicle near conflict"), std::string::npos);
}

TEST(RoundaboutZoneRule, ReleasedVehicleThatStartsMovingIsAnIntruder) {
  auto config = zoneConfig();
  config.gs_zone_length_estimate = false;
  config.gs_stationary_yield_enabled = true;
  config.gs_stationary_yield_start = 5.0;
  config.gs_stationary_yield_confirm = 3.0;
  config.gs_stationary_yield_min_distance = 3.0;
  RoundaboutMission mission(config);
  RoundaboutOutput out;
  for (int t = 0; t <= 8; ++t) {
    RoundaboutInput input = zoneInput(static_cast<double>(t));
    input.objects = {upstreamCar(7, 9.0, 0.0, 4.5)};
    out = mission.update(input);
  }
  ASSERT_EQ(out.state, RoundaboutState::kCommit);
  // 진입 중 그 차가 다시 움직이면(속도 3 m/s) 일반 차량으로 돌아가 구간 침범이 된다.
  RoundaboutInput moving = zoneInput(8.1);
  moving.ego.speed = 1.0;
  moving.objects = {upstreamCar(7, 8.7, 3.0, 4.5)};
  out = mission.update(moving);
  EXPECT_NE(out.reason.find("zone_intrusion"), std::string::npos);
}

TEST(RoundaboutMission, ExitFollowsRouteInsteadOfUnrelatedCircularVehicle) {
  auto config = testConfig();
  config.safe_confirm_seconds = 0.0;
  RoundaboutMission mission(config);
  auto input = approachInput(0.0);
  ASSERT_EQ(mission.update(input).state, RoundaboutState::kCommit);
  input.now = 0.2; input.ego.position = {10.0, 0.0};
  ASSERT_EQ(mission.update(input).state, RoundaboutState::kCirculate);
  input.now = 1.0; input.ego.position = {0.0, 10.0};
  input.route = {{10.0, 0.0}, {5.0, 8.0}, {0.0, 10.0}, {0.0, 30.0}};
  input.objects = {circulatingObject(8, 2.0, 0.0)};
  const auto leaving = mission.update(input);
  ASSERT_EQ(leaving.state, RoundaboutState::kExit);
  EXPECT_FALSE(leaving.constraint.lead_vehicle_active);
  auto stopped_exit_car = circulatingObject(9, 0.0, 0.0);
  stopped_exit_car.position = {0.0, 11.5};
  input.objects.push_back(stopped_exit_car);
  input.now = 1.1;
  const auto stopped = mission.update(input);
  EXPECT_TRUE(stopped.constraint.lead_vehicle_active);
  EXPECT_EQ(stopped.constraint.lead_vehicle_id, 9);
  EXPECT_DOUBLE_EQ(stopped.constraint.lead_speed, 0.0);
  EXPECT_DOUBLE_EQ(stopped.constraint.lead_distance, 0.0);
}

}  // namespace
}  // namespace mission_planner

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#include "mission_planner/distance_entry.hpp"
namespace mission_planner {
TEST(DistanceEntry,FrontCanClearBeforeOurArrivalWithoutAlreadyPassing){
  EgoEntryModel ego;ego.distance_to_merge=10;ego.speed=0;ego.accel=2;ego.max_speed=5;
  GapCandidate g;g.leader_id=1;g.leader_pos=-8;g.leader_speed=5;g.leader_length=4.6;
  auto result=evaluateDistanceEntry(g,ego,DistanceEntryParams{});
  EXPECT_TRUE(result.safe)<<result.reason;EXPECT_GE(result.front_min,2.5-1e-6);
}
TEST(DistanceEntry,RearYieldIsBoundedAndStartsOnlyAfterEntryReaction){
  EgoEntryModel ego;ego.distance_to_merge=10;ego.speed=0;ego.accel=2;ego.max_speed=5;
  GapCandidate g;g.follower_id=2;g.follower_pos=-29;g.follower_speed=6;g.follower_length=4.6;
  DistanceEntryParams p;EXPECT_FALSE(evaluateDistanceEntry(g,ego,p).safe);
  p.rear_yield=true;auto result=evaluateDistanceEntry(g,ego,p);ASSERT_TRUE(result.safe)<<result.reason;
  EXPECT_GT(result.required_decel,0);EXPECT_LE(result.required_decel,2);EXPECT_GE(result.rear_min,2-1e-6);
  p.rear_decel=.2;EXPECT_FALSE(evaluateDistanceEntry(g,ego,p).safe);
  p.rear_decel=2;p.reaction=5;EXPECT_FALSE(evaluateDistanceEntry(g,ego,p).safe);
}
TEST(DistanceEntry,FastCloseRearCannotBeExcusedByNpcCooperation){
  EgoEntryModel ego;ego.distance_to_merge=10;ego.speed=0;ego.accel=2;ego.max_speed=5;
  GapCandidate g;g.follower_id=2;g.follower_pos=-15;g.follower_speed=8;
  DistanceEntryParams p;p.rear_yield=true;EXPECT_FALSE(evaluateDistanceEntry(g,ego,p).safe);
}
TEST(DistanceEntry,TruckLengthAndPostClearHorizonAreIncluded){
  EgoEntryModel ego;ego.distance_to_merge=10;ego.speed=0;ego.accel=2;ego.max_speed=5;
  GapCandidate g;g.leader_id=1;g.leader_pos=-8;g.leader_speed=5;g.leader_length=10;
  auto result=evaluateDistanceEntry(g,ego,DistanceEntryParams{});EXPECT_FALSE(result.safe);
  g.leader_id=-1;g.follower_id=2;g.follower_pos=-29;g.follower_speed=6;
  DistanceEntryParams p;p.horizon=0;auto shorter=evaluateDistanceEntry(g,ego,p);p.horizon=1.5;auto longer=evaluateDistanceEntry(g,ego,p);
  EXPECT_GT(shorter.rear_min,longer.rear_min);EXPECT_GT(longer.clear_time,longer.entry_time);
}
TEST(DistanceEntry,MissingMpcPredictionCannotGrantClearRoadGo){
  auto c=testConfig();c.distance_enabled=true;c.gap_select_enabled=true;c.gs_use_mpc_prediction=true;
  c.mandatory_stop_seconds=0;c.distance_confirm=0;
  RoundaboutMission mission(c);auto input=approachInput(1);input.prediction_speed_limit=std::numeric_limits<double>::quiet_NaN();
  auto out=mission.update(input);EXPECT_FALSE(out.gap_safe);EXPECT_NE(out.reason.find("prediction unavailable"),std::string::npos);
}
}

namespace mission_planner {
TEST(DistanceEntry,FacilityInsideRingDoesNotBecomeLeaderAfterConfirmationWindow){
 auto c=testConfig();c.distance_enabled=true;c.gap_select_enabled=true;c.mandatory_stop_seconds=0;c.distance_confirm=0;
 RoundaboutMission mission(c);auto input=approachInput(1);auto o=circulatingObject(8,-.2,0);o.has_moved=false;o.confirmed=false;o.height=.03;o.length=2;o.hit_count=5;
 input.objects={o};auto out=mission.update(input);EXPECT_EQ(out.gap_leader_id,-1);EXPECT_TRUE(out.gap_safe);
}
TEST(DistanceEntry,LargeStoppedUnconfirmedBodyAtMergeCannotBeIgnored){
 auto c=testConfig();c.distance_enabled=true;c.gap_select_enabled=true;c.mandatory_stop_seconds=0;c.distance_confirm=0;
 RoundaboutMission mission(c);auto input=approachInput(1);auto o=circulatingObject(8,0,0);o.has_moved=false;o.confirmed=false;o.height=1.5;o.length=6;o.width=2;o.hit_count=5;
 input.objects={o};mission.update(input);input.now=3;auto out=mission.update(input);EXPECT_FALSE(out.gap_safe);
}
TEST(DistanceEntry,NewModelPreservesMissionGeometryAndRejectsOutsideVehicleRoi){
 auto c=testConfig();c.distance_enabled=true;c.gap_select_enabled=true;c.mandatory_stop_seconds=0;c.distance_confirm=0;c.vehicle_roi_inner_radius=8;c.vehicle_roi_outer_radius=12;
 RoundaboutMission mission(c);auto input=approachInput(1);auto o=circulatingObject(9,0,4);o.position={12.4,0};input.objects={o};auto out=mission.update(input);
 EXPECT_TRUE(out.stream.empty());EXPECT_TRUE(out.gap_safe);EXPECT_EQ(c.outer_radius,12);
}
TEST(DistanceEntry,ActualDetectedTruckIsNotClampedToPassengerLength){
 auto c=testConfig();c.distance_enabled=true;c.gap_select_enabled=true;c.mandatory_stop_seconds=0;c.distance_confirm=0;
 RoundaboutMission mission(c);auto input=approachInput(1);auto o=circulatingObject(9,.8,4);o.length=11;o.hit_count=10;input.objects={o};auto out=mission.update(input);
 ASSERT_EQ(out.stream.size(),1u);EXPECT_DOUBLE_EQ(out.stream[0].length,11);
}
}
namespace mission_planner {
TEST(DistanceEntry,CommitKeepsThePairAndOccludedRearUntilMeasuredLifetimeExpires){
 auto c=testConfig();c.distance_enabled=true;c.gap_select_enabled=true;c.mandatory_stop_seconds=0;c.distance_confirm=.2;c.distance_rear_yield=true;c.gs_ego_accel=2;c.gs_ego_max_speed=5;
 RoundaboutMission mission(c);auto input=approachInput(1);input.ego.speed=0;
 auto lead=circulatingObject(1,-.8,5),rear=circulatingObject(2,-2.9,5);lead.hit_count=rear.hit_count=10;input.objects={lead,rear};
 auto first=mission.update(input);ASSERT_TRUE(first.gap_safe)<<first.reason;
 input.now=1.25;auto out=mission.update(input);ASSERT_EQ(out.state,RoundaboutState::kCommit)<<out.reason;
 const int l=out.gap_leader_id,f=out.gap_follower_id;
 input.now=1.3;input.objects={lead};out=mission.update(input);EXPECT_EQ(out.gap_leader_id,l);EXPECT_EQ(out.gap_follower_id,f);
 input.now=4.4;out=mission.update(input);EXPECT_FALSE(out.gap_safe);
}
}
namespace mission_planner {
TEST(DistanceEntry,ApprovedPredictionDoesNotRunLegacyLeaderStop){
 auto c=testConfig();c.distance_enabled=true;c.gap_select_enabled=true;c.mandatory_stop_seconds=0;c.distance_confirm=.2;
 c.distance_rear_yield=true;c.gs_ego_accel=2;c.gs_ego_max_speed=5;c.gs_leader_release_margin=0;
 RoundaboutMission mission(c);auto input=approachInput(1);input.ego.speed=0;
 auto lead=circulatingObject(1,-.8,5),rear=circulatingObject(2,-2.9,5);lead.hit_count=rear.hit_count=10;input.objects={lead,rear};
 ASSERT_TRUE(mission.update(input).gap_safe);input.now=1.25;auto out=mission.update(input);
 ASSERT_EQ(out.state,RoundaboutState::kCommit)<<out.reason;
 EXPECT_FALSE(out.constraint.stop_required)<<out.reason;
 EXPECT_EQ(out.reason.find("waiting leader to pass"),std::string::npos);
}
}
namespace mission_planner {
TEST(DistanceEntry,PassedFollowerExpiresCommittedGapBeforeEgoMerge){
 auto c=testConfig();c.distance_enabled=true;c.gap_select_enabled=true;c.mandatory_stop_seconds=0;c.distance_confirm=.2;
 c.distance_rear_yield=true;c.gs_ego_accel=2;c.gs_ego_max_speed=5;c.gs_leader_release_margin=0;
 RoundaboutMission mission(c);auto input=approachInput(1);input.ego.speed=0;
 auto lead=circulatingObject(1,-.8,5),rear=circulatingObject(2,-2.9,5);lead.hit_count=rear.hit_count=10;input.objects={lead,rear};
 ASSERT_TRUE(mission.update(input).gap_safe);input.now=1.25;ASSERT_EQ(mission.update(input).state,RoundaboutState::kCommit);
 input.now=1.3;lead=circulatingObject(1,1.2,5);rear=circulatingObject(2,.2,5);lead.hit_count=rear.hit_count=11;input.objects={lead,rear};
 auto out=mission.update(input);EXPECT_FALSE(out.gap_safe);EXPECT_NE(out.reason.find("gap"),std::string::npos);
 for(const auto& g:out.gaps)if(g.leader_id==1&&g.follower_id==2)EXPECT_FALSE(g.go);
}
TEST(DistanceEntry,PassedSlotsAreNotShownAsSelectedTargetsWhileYielding){
 auto c=testConfig();c.distance_enabled=true;c.gap_select_enabled=true;c.mandatory_stop_seconds=0;c.distance_confirm=10;
 RoundaboutMission mission(c);auto input=approachInput(1);input.ego.speed=0;
 input.objects={circulatingObject(1,1.2,5),circulatingObject(2,.4,5)};
 auto out=mission.update(input);for(const auto& g:out.gaps)if(g.leader_id==1&&g.follower_id==2){EXPECT_FALSE(g.go);EXPECT_FALSE(g.selected);}
}
}
namespace mission_planner {
TEST(DistanceEntry,OutsideRingDoesNotUsePolarArcAsLeadDistance){
 auto c=testConfig();c.distance_enabled=true;c.gap_select_enabled=true;c.mandatory_stop_seconds=0;c.distance_confirm=.2;
 c.distance_rear_yield=true;c.gs_ego_accel=2;c.gs_ego_max_speed=5;
 RoundaboutMission mission(c);auto input=approachInput(1);input.ego.speed=0;
 auto lead=circulatingObject(1,-.8,5),rear=circulatingObject(2,-2.9,5);lead.hit_count=rear.hit_count=10;input.objects={lead,rear};
 ASSERT_TRUE(mission.update(input).gap_safe);input.now=1.25;ASSERT_EQ(mission.update(input).state,RoundaboutState::kCommit);
 input.now=1.3;lead=circulatingObject(1,-.4,5);lead.hit_count=11;input.objects={lead,rear};auto out=mission.update(input);
 EXPECT_FALSE(out.constraint.lead_vehicle_active)<<out.reason;
}
TEST(DistanceEntry,DownstreamLeadUsesEntryPathPlusConflictArc){
 auto c=testConfig();c.distance_enabled=true;c.gap_select_enabled=true;c.mandatory_stop_seconds=0;c.distance_confirm=.2;
 c.distance_rear_yield=true;c.gs_ego_accel=2;c.gs_ego_max_speed=5;c.gs_standstill_gap=.8;c.gs_prediction_follow_time_gap=0;
 RoundaboutMission mission(c);auto input=approachInput(1);input.ego.speed=0;
 auto lead=circulatingObject(1,-.8,5),rear=circulatingObject(2,-2.9,5);lead.hit_count=rear.hit_count=10;input.objects={lead,rear};
 ASSERT_TRUE(mission.update(input).gap_safe);input.now=1.25;ASSERT_EQ(mission.update(input).state,RoundaboutState::kCommit);
 input.now=1.3;lead=circulatingObject(1,.3,5);lead.hit_count=11;input.objects={lead,rear};auto out=mission.update(input);
 ASSERT_TRUE(out.constraint.lead_vehicle_active);EXPECT_EQ(out.constraint.lead_vehicle_id,1);
 EXPECT_NEAR(out.constraint.lead_distance,10.+3.-2.-.5*c.gs_ego_length,.01);
}
TEST(DistanceEntry,RingLeadKeepsBothVehicleHalfLengths){
 auto c=testConfig();c.distance_enabled=true;c.gap_select_enabled=true;c.mandatory_stop_seconds=0;
 c.gs_require_merge_progress=true;RoundaboutMission mission(c);
 auto input=approachInput(1);input.ego.position={10,0};input.ego.speed=3;
 auto lead=circulatingObject(1,.9,3);lead.hit_count=10;input.objects={lead};
 auto out=mission.update(input);ASSERT_TRUE(out.constraint.lead_vehicle_active)<<out.reason;
 EXPECT_NEAR(out.constraint.lead_distance,9.-2.-.5*c.gs_ego_length,.001);
 input.now=1.05;lead=circulatingObject(1,.35,3);lead.hit_count=11;input.objects={lead};
 out=mission.update(input);ASSERT_TRUE(out.constraint.lead_vehicle_active)<<out.reason;
 EXPECT_DOUBLE_EQ(out.constraint.lead_distance,0.);
}
TEST(DistanceEntry,EnteringEgoKeepsVehicleStillOccupyingConflictAsLead){
 auto c=testConfig();c.distance_enabled=true;c.gap_select_enabled=true;c.mandatory_stop_seconds=0;
 c.gs_require_merge_progress=true;RoundaboutMission mission(c);
 auto input=approachInput(1);input.ego.position={10,-2.5};input.ego.speed=3;
 auto lead=circulatingObject(1,.1,0);lead.hit_count=10;input.objects={lead};
 auto out=mission.update(input);ASSERT_TRUE(out.constraint.lead_vehicle_active)<<out.reason;
 EXPECT_EQ(out.constraint.lead_vehicle_id,1);EXPECT_DOUBLE_EQ(out.constraint.lead_distance,0.);
}
TEST(DistanceEntry,PredictionIncludesExternalHeadwayCap){
 EgoEntryModel ego;ego.distance_to_merge=8;ego.speed=0;ego.accel=2.2;ego.max_speed=8.5;
 GapCandidate g;g.leader_id=1;g.leader_pos=3;g.leader_length=4;g.leader_speed=5;
 DistanceEntryParams p;auto free_gap=g;free_gap.leader_id=-1;
 auto unconstrained=evaluateDistanceEntry(free_gap,ego,p);
 p.follow_time_gap=1.5;auto following=evaluateDistanceEntry(g,ego,p);
 EXPECT_GT(following.clear_time,unconstrained.clear_time);
 EXPECT_GE(following.entry_time,unconstrained.entry_time-1e-6);
 ASSERT_FALSE(following.motion.empty());
 double s,v;sampleEntryMotion(following,following.clear_time,&s,&v);
 EXPECT_NEAR(s,ego.distance_to_merge+.5*ego.length,.001);
}
TEST(DistanceEntry,FreshReplacementIdInheritsProofButSeparateVehicleDoesNot){
 auto c=testConfig();c.distance_enabled=true;c.gap_select_enabled=true;c.mandatory_stop_seconds=0;c.distance_confirm=10;
 RoundaboutMission mission(c);auto input=approachInput(1);input.ego.speed=0;
 auto old=circulatingObject(1,-.8,5);old.hit_count=10;old.height=1.;old.width=1.8;input.objects={old};mission.update(input);
 auto replacement=old;replacement.id=2;replacement.has_moved=false;replacement.hit_count=5;
 input.now=1.1;input.objects={replacement};auto out=mission.update(input);
 EXPECT_EQ(out.reason.find("unconfirmed vehicle 2"),std::string::npos);
 auto separate=circulatingObject(3,-.3,5);separate.has_moved=false;separate.hit_count=5;separate.height=1;separate.width=1.8;
 input.now=1.15;input.objects={replacement,separate};out=mission.update(input);
 for(const auto& vehicle:out.stream)EXPECT_NE(vehicle.id,3);
}
}

namespace mission_planner {
TEST(DistanceEntry, FailedWaitingPairImmediatelyPreviewsNextUpstreamGap) {
 auto c=testConfig();c.distance_enabled=true;c.gap_select_enabled=true;c.mandatory_stop_seconds=0;c.distance_confirm=.2;
 c.distance_pair_invalid=10;c.distance_rear_yield=true;c.gs_ego_accel=2;c.gs_ego_max_speed=5;
 RoundaboutMission mission(c);auto input=approachInput(1);input.ego.speed=0;
 auto lead=circulatingObject(1,-.8,5),rear=circulatingObject(2,-2.9,5),next=circulatingObject(3,-3.1,2);
 lead.hit_count=rear.hit_count=next.hit_count=10;input.objects={lead,rear,next};
 auto out=mission.update(input);ASSERT_TRUE(out.gap_safe)<<out.reason<<" entry="<<out.ego_zone_time<<" clear="<<out.ego_clear_time;
 ASSERT_EQ(out.gap_leader_id,1);ASSERT_EQ(out.gap_follower_id,2);
 input.now=1.1;lead=circulatingObject(1,.2,0);lead.hit_count=11;input.objects={lead,rear,next};out=mission.update(input);
 EXPECT_FALSE(out.gap_safe);EXPECT_EQ(out.gap_leader_id,2)<<out.reason;EXPECT_EQ(out.gap_follower_id,3);
 for(const auto& g:out.gaps)if(g.selected)EXPECT_TRUE(g.preview);
 input.now=1.2;lead=circulatingObject(1,-.8,5);lead.hit_count=12;input.objects={lead,rear,next};out=mission.update(input);
 EXPECT_TRUE(out.gap_safe)<<out.reason;EXPECT_EQ(out.gap_leader_id,1);
 EXPECT_EQ(out.state,RoundaboutState::kYield);
 input.now=1.3;EXPECT_EQ(mission.update(input).state,RoundaboutState::kYield);
 input.now=1.45;EXPECT_EQ(mission.update(input).state,RoundaboutState::kCommit);
}
TEST(DistanceEntry, SelectedLeadCarriesRoundaboutProfile){
 auto c=testConfig();c.distance_enabled=true;c.gap_select_enabled=true;c.mandatory_stop_seconds=0;c.distance_confirm=0;
 c.gs_prediction_follow_standstill_gap=2.5;c.gs_prediction_follow_time_gap=.5;
 RoundaboutMission mission(c);auto input=approachInput(1);input.ego.speed=0;
 input.objects={circulatingObject(1,.5,5)};auto out=mission.update(input);
 ASSERT_TRUE(out.constraint.lead_vehicle_active)<<out.reason;
 EXPECT_TRUE(out.constraint.lead_profile_override);EXPECT_DOUBLE_EQ(out.constraint.lead_standstill_distance,2.5);
 EXPECT_DOUBLE_EQ(out.constraint.lead_time_gap,.5);
}
}
namespace mission_planner {
TEST(DistanceEntry, TunedSpacingAcceptsBoundaryButRejectsSmallerBumperGap) {
 EgoEntryModel ego;ego.distance_to_merge=-2;ego.length=4;ego.speed=4;ego.max_speed=4;ego.departure_in_progress=true;
 GapCandidate gap;gap.leader_id=1;gap.leader_pos=8;gap.leader_speed=4;gap.leader_length=4;
 gap.follower_id=2;gap.follower_pos=-3.5;gap.follower_speed=4;gap.follower_length=4;
 DistanceEntryParams params;params.front_gap=2;params.rear_gap=1.5;params.speed_margin=0;
 auto out=evaluateDistanceEntry(gap,ego,params);ASSERT_TRUE(out.safe)<<out.reason;
 EXPECT_NEAR(out.front_min,2,1e-6);EXPECT_NEAR(out.rear_min,1.5,1e-6);
 gap.leader_pos=7.99;EXPECT_FALSE(evaluateDistanceEntry(gap,ego,params).safe);
 gap.leader_pos=8;gap.follower_pos=-3.49;EXPECT_FALSE(evaluateDistanceEntry(gap,ego,params).safe);
}
}
namespace mission_planner {
namespace {
EgoEntryModel measuredTestEgo(){
 EgoEntryModel e;e.measured_profile=true;e.speed=0;e.max_speed=8.5;e.start_delay=.2;
 e.profile_times={0,.3,.8,1.3,1.8,2.3,2.8,3.3};e.profile_speeds={0,.95,2.5,3.7,4.6,5.2,5.55,5.8};return e;
}
double motionArrival(const DistanceEntryResult& r,double distance){
 double lo=0,hi=30,s,v;sampleEntryMotion(r,hi,&s,&v);if(s<distance)return INFINITY;
 for(int i=0;i<40;++i){double m=(lo+hi)/2;sampleEntryMotion(r,m,&s,&v);if(s<distance)lo=m;else hi=m;}return hi;
}
}
TEST(DistanceEntry,MeasuredProfileMatchesSuccessfulEntryTimes){
 for(const auto& row:std::vector<std::vector<double>>{{9.839557,2.592,3.013,3.432},{9.656097,2.509,2.909,3.249}}){
  auto e=measuredTestEgo();e.distance_to_merge=row[0];auto r=evaluateDistanceEntry(GapCandidate{},e,DistanceEntryParams{});
  ASSERT_TRUE(r.safe)<<r.reason;
  EXPECT_NEAR(r.entry_time,row[1],.15);EXPECT_NEAR(motionArrival(r,row[0]),row[2],.15);EXPECT_NEAR(r.clear_time,row[3],.15);
 }
}
TEST(DistanceEntry,MeasuredContinuationUsesObservedSpeedWithoutNewDelay){
 auto e=measuredTestEgo();e.speed=3.7;e.departure_in_progress=true;e.distance_to_merge=6;
 auto r=evaluateDistanceEntry(GapCandidate{},e,DistanceEntryParams{});ASSERT_TRUE(r.safe);
 EXPECT_DOUBLE_EQ(r.delay_remaining,0.);double s,v;sampleEntryMotion(r,.1,&s,&v);
 EXPECT_GT(s,.37);EXPECT_NEAR(v,3.88,.02);
}
TEST(DistanceEntry,MeasuredProfileRespectsCurveAndLeaderLimits){
 auto e=measuredTestEgo();e.distance_to_merge=8;auto free=evaluateDistanceEntry(GapCandidate{},e,DistanceEntryParams{});
 e.max_speed=3;auto limited=evaluateDistanceEntry(GapCandidate{},e,DistanceEntryParams{});
 EXPECT_TRUE(limited.curve_limited);EXPECT_GT(limited.clear_time,free.clear_time);
 for(const auto& sample:limited.motion)EXPECT_LE(sample.speed,3.000001);
 e.max_speed=8.5;GapCandidate g;g.leader_id=1;g.leader_pos=3;g.leader_length=4;g.leader_speed=3;
 DistanceEntryParams p;p.follow_time_gap=.5;p.follow_standstill_gap=2.5;
 auto follow=evaluateDistanceEntry(g,e,p);EXPECT_TRUE(follow.leader_limited);EXPECT_GT(follow.clear_time,free.clear_time);
}
TEST(DistanceEntry,MeasuredProfileHasNoAccelerationPastLastSpeed){
 auto e=measuredTestEgo();e.distance_to_merge=80;auto r=evaluateDistanceEntry(GapCandidate{},e,DistanceEntryParams{});
 for(const auto& sample:r.motion)EXPECT_LE(sample.speed,5.800001);
 e.speed=6.3;e.departure_in_progress=true;r=evaluateDistanceEntry(GapCandidate{},e,DistanceEntryParams{});
 for(const auto& sample:r.motion)EXPECT_DOUBLE_EQ(sample.speed,6.3);
}
TEST(DistanceEntry,InvalidMeasuredProfileCannotApproveEntry){
 auto e=measuredTestEgo();e.profile_times[2]=.1;
 EXPECT_EQ(evaluateDistanceEntry(GapCandidate{},e,DistanceEntryParams{}).reason,"invalid_measured_profile");
}
}
namespace mission_planner {
TEST(DistanceEntry,BrakeReserveAccountsForDifferentVehicleBrakesAndReaction){
 const double gap=8,lead_speed=4,standstill=2.5,decel=2,reaction=.5,lead_decel=3,ego_speed=5;
 const double cap=frontBrakingSpeedCap(gap,lead_speed,standstill,decel,reaction,lead_decel,ego_speed);
 EXPECT_LE(cap*cap/(2*decel)+cap*reaction,gap-standstill+lead_speed*lead_speed/(2*lead_decel)+1e-9);
 EXPECT_LE(cap*cap/(2*decel)+ego_speed*reaction,gap-standstill+lead_speed*lead_speed/(2*lead_decel)+1e-9);
 EXPECT_DOUBLE_EQ(frontBrakingSpeedCap(0,0,standstill,decel,reaction,lead_decel,ego_speed),0.);
 EXPECT_DOUBLE_EQ(frontBrakingSpeedCap(8,4,2.5,2.5,0),std::sqrt(16+5*5.5));
}
TEST(DistanceEntry,ClosedLoopMeasuredEntryMaintainsFrontClearance){
 // Exercise the 2.5 m / 0.35 s profile and 0.2 s contact timing reserve with delayed braking.
 // The rejected 2.0 m / 0.25 s experiment is documented in the analysis report.
 for(int approval_policy:{0,1,2,3,4,5})for(double actual_decel:{2.5,2.0})for(int scenario:{0,1,2})
 for(double npc_speed:{3.5,4.2,5.5})for(double onset:{.4,1.,1.5})
 for(double response_delay:{.2,.3})for(double leader_decel:{1.5,2.,3.}){
  const double headway=approval_policy>=4?.35:.5;
  const bool clearance_only=approval_policy!=0;
  SCOPED_TRACE(::testing::Message()<<"policy="<<approval_policy<<" clearance="<<clearance_only<<" npc="<<npc_speed<<" onset="<<onset<<" response="<<response_delay<<" leader_decel="<<leader_decel);
  const bool slowing_leader=scenario!=0;
  auto c=testConfig();c.distance_enabled=true;c.gap_select_enabled=true;c.gs_require_merge_progress=true;
  c.distance_commit_rear_yield_lock=approval_policy==2;c.distance_clearance_only=clearance_only;c.distance_measured_profile=true;auto model=measuredTestEgo();c.distance_profile_times=model.profile_times;c.distance_profile_speeds=model.profile_speeds;
  c.gs_speed_margin=.2;c.comfortable_decel=2.;c.distance_front_response=.5;c.distance_front_max_decel=3.;c.distance_front_gap=2;c.distance_rear_gap=1.5;c.distance_confirm=.15;c.distance_rear_yield=true;
  c.gs_prediction_follow_standstill_gap=2.5;c.gs_prediction_follow_time_gap=headway;c.gs_standstill_gap=.8;
  if (approval_policy >= 3) {
    c.distance_enabled=false;c.gs_sequential=true;c.gs_zone_rule_enabled=false;
    c.gs_route_body_projection=approval_policy>=4;c.gs_ego_width=1.9;
    c.gs_timed_entry_approach=approval_policy==5;
    c.gs_approach_time_buffer=.2;c.gs_approach_accel_compensation=true;
    c.gs_rear_ttc_at_entry=false; // Deployed full-trajectory rear TTC check.
    c.gs_commit_rear_yield_lock=true;c.gs_measured_entry_profile=true;c.gs_use_mpc_prediction=true;
    c.gs_front_headway=approval_policy>=4?.05:.3;c.gs_seq_rear_headway=.2;c.gs_ttc_rear=1.;c.gs_late_margin=.2;
    c.gs_check_horizon=1.5;c.gs_confirm_seconds=.1;c.gs_leader_start_distance=8;
    c.gs_leader_decel_horizon=1.;c.gs_leader_decel_limit=1.5;
  }
  c.mandatory_stop_seconds=1;c.yield_point={10,-10};c.gs_commit_abort_enabled=true;
  RoundaboutMission mission(c);auto input=approachInput(0);input.route={{10,-30},{10,0}};
  input.prediction_speed_limit=8.5;input.prediction_accel_limit=3.2;
  for(int j=1;j<=40;++j)input.route.push_back({10*std::cos(j*.025),10*std::sin(j*.025)});
  double s=-10,v=0,leader=-7,leader_v=npc_speed,rear=-27,rear_v=npc_speed,commit=-1,min_gap=1e9,brake_since=-1;bool cleared=false;
  for(int i=0;i<600;++i){double now=i*.05;input.now=now;input.ego.position=s<0?Point2{10,s}:Point2{10*std::cos(s/10),10*std::sin(s/10)};
   input.ego.yaw=kPi/2+std::max(0.,s)/10;input.ego.speed=v;
   auto l=circulatingObject(1,leader/10,leader_v),r=circulatingObject(2,rear/10,rear_v);l.hit_count=r.hit_count=i/2+10;
   input.objects={l,r};auto out=mission.update(input);
   if(out.state==RoundaboutState::kCommit&&commit<0)commit=now;
   double cap=out.constraint.max_speed<0?8.5:out.constraint.max_speed;
   if(out.constraint.stop_required)cap=std::min(cap,std::sqrt(2*c.comfortable_decel*std::max(0.,out.constraint.stop_distance)));
   if(out.constraint.lead_vehicle_active){double gap=std::max(0.,out.constraint.lead_distance-c.gs_prediction_follow_standstill_gap);
    cap=std::min(cap,std::min(std::sqrt(out.constraint.lead_speed*out.constraint.lead_speed+2*c.comfortable_decel*gap),
      std::max(0.,out.constraint.lead_speed+(gap-headway*v)/headway)));}
   if(cap<v&&brake_since<0)brake_since=now;
   if(cap>=v)brake_since=-1;
   const bool brake_delay=brake_since>=0&&now-brake_since<response_delay;
   const double next=(commit>=0&&now-commit<.23)||brake_delay?v:(v<cap?std::min(cap,v+3.2*.05):std::max(cap,v-actual_decel*.05));
   s+=.5*(v+next)*.05;v=next;
   if(slowing_leader&&commit>=0&&now-commit>onset){
    if(scenario==2&&now-commit>onset+1.)leader_v=std::min(npc_speed,leader_v+2*.05);
    else leader_v=std::max(0.,leader_v-leader_decel*.05);
   }
   leader+=leader_v*.05;
   // NPCs share one lane: the rear must queue behind a stopped leader rather
   // than drive through it and become a fictitious new closest front car.
   const double rear_gap=std::max(0.,leader-rear-6.);
   const double rear_cap=std::min(npc_speed,std::min(std::sqrt(leader_v*leader_v+4*rear_gap),
     std::max(0.,leader_v+(rear_gap-.5*rear_v)/.5)));
   const double rear_next=rear_v<rear_cap?std::min(rear_cap,rear_v+.1):std::max(rear_cap,rear_v-.15);
   rear+=.5*(rear_v+rear_next)*.05;rear_v=rear_next;
   if (approval_policy>=4) {
    const Point2 ec=s<0?Point2{10,s}:Point2{10*std::cos(s/10),10*std::sin(s/10)};
    const Point2 lc{10*std::cos(leader/10),10*std::sin(leader/10)};
    const double ey=kPi/2+std::max(0.,s)/10,ly=kPi/2+leader/10;
    double separation=-1e9;
    for(double axis:{ey,ey+kPi/2,ly,ly+kPi/2}) {
      const double between=std::abs((lc.x-ec.x)*std::cos(axis)+(lc.y-ec.y)*std::sin(axis));
      const double e_support=2.3*std::abs(std::cos(ey-axis))+.95*std::abs(std::sin(ey-axis));
      const double l_support=2.*std::abs(std::cos(ly-axis))+.95*std::abs(std::sin(ly-axis));
      separation=std::max(separation,between-e_support-l_support);
    }
    min_gap=std::min(min_gap,separation);
   } else if(s+2.3>=0)min_gap=std::min(min_gap,leader-2.-s-2.3);
   if(s>=2.3){cleared=true;break;}
  }
  EXPECT_GE(min_gap,approval_policy>=4?.3:1.5)<<"front contact, scenario="<<scenario<<" slowing="<<slowing_leader<<" headway="<<headway<<" decel="<<actual_decel;
  if(scenario!=1){EXPECT_TRUE(cleared)<<"scenario="<<scenario;EXPECT_GE(commit,0.);// TTC follows its own sequential slot selection; require a bounded
    // departure after the mandatory stop, preserving the same 1.5m minimum.
    EXPECT_LE(commit,approval_policy>=3?4.0:3.1);}
 }
}
}

namespace mission_planner {
TEST(DistanceEntry,ClearanceRemovesOnlyPostMergeAdmissionHorizon){
 EgoEntryModel e;e.distance_to_merge=0;e.length=4;e.speed=1;e.max_speed=1;e.accel=2;e.departure_in_progress=true;
 GapCandidate g;g.follower_id=2;g.follower_pos=-6.05;g.follower_length=.1;g.follower_speed=2;
 DistanceEntryParams p;p.speed_margin=0;p.rear_gap=1.5;p.horizon=1.5;
 auto old=evaluateDistanceEntry(g,e,p);EXPECT_FALSE(old.safe);EXPECT_EQ(old.reason,"rear_space");
 p.clearance_only=true;auto simple=evaluateDistanceEntry(g,e,p);ASSERT_TRUE(simple.safe)<<simple.reason;
 EXPECT_NEAR(simple.rear_min,2.,1e-6);EXPECT_NEAR(simple.clear_time,2.,1e-6);
 p.horizon=30;auto same=evaluateDistanceEntry(g,e,p);EXPECT_EQ(same.safe,simple.safe);
 EXPECT_NEAR(same.rear_min,simple.rear_min,1e-6);EXPECT_NEAR(same.clear_time,simple.clear_time,1e-6);
}
TEST(DistanceEntry,ClearanceSolvesRearBrakingBeforeFullStop){
 auto e=measuredTestEgo();e.distance_to_merge=8;
 DistanceEntryParams p;p.clearance_only=true;p.speed_margin=0;p.rear_yield=true;p.rear_gap=1.5;p.reaction=.2;
 auto free=evaluateDistanceEntry(GapCandidate{},e,p);const double dt=free.clear_time-free.entry_time-p.reaction;
 ASSERT_GT(dt,0.);
 GapCandidate g;g.follower_id=2;g.follower_speed=4;g.follower_length=4;
 const double reduction=.1;g.follower_pos=-4*free.clear_time-2-p.rear_gap+reduction;
 auto r=evaluateDistanceEntry(g,e,p);ASSERT_TRUE(r.safe)<<r.reason;
 EXPECT_NEAR(r.required_decel,2*reduction/(dt*dt),1e-5);EXPECT_NEAR(r.rear_min,p.rear_gap,1e-5);
 DistanceEntryParams old=p;old.clearance_only=false;old.horizon=0;
 auto reference=evaluateDistanceEntry(g,e,old);ASSERT_TRUE(reference.safe)<<reference.reason;
 EXPECT_NEAR(r.required_decel,reference.required_decel,1e-4);
}
TEST(DistanceEntry,ClearanceSolvesRearBrakingAfterFullStop){
 EgoEntryModel e;e.distance_to_merge=0;e.length=4;e.speed=1;e.max_speed=1;e.accel=2;e.departure_in_progress=true;
 GapCandidate g;g.follower_id=2;g.follower_pos=-2.35;g.follower_length=.1;g.follower_speed=1;
 DistanceEntryParams p;p.clearance_only=true;p.speed_margin=0;p.rear_yield=true;p.rear_gap=1.5;p.reaction=.5;p.rear_decel=2;
 auto r=evaluateDistanceEntry(g,e,p);ASSERT_TRUE(r.safe)<<r.reason;
 EXPECT_NEAR(r.required_decel,5./3.,1e-5);EXPECT_NEAR(r.rear_min,1.5,1e-5);
 p.rear_decel=1.;EXPECT_FALSE(evaluateDistanceEntry(g,e,p).safe);
 p.rear_decel=100.;p.reaction=3.;EXPECT_FALSE(evaluateDistanceEntry(g,e,p).safe);
}
TEST(DistanceEntry,ClearanceStillRejectsFrontSpaceAndMissedMergeHorizon){
 auto e=measuredTestEgo();e.distance_to_merge=8;DistanceEntryParams p;p.clearance_only=true;
 GapCandidate g;g.leader_id=1;g.leader_pos=0;g.leader_length=4;g.leader_speed=1;
 EXPECT_FALSE(evaluateDistanceEntry(g,e,p).safe);
 e.distance_to_merge=1000;EXPECT_EQ(evaluateDistanceEntry(GapCandidate{},e,p).reason,"prediction_horizon");
}
}

namespace mission_planner {
TEST(DistanceEntry,ClearanceUsesEntryFrontGapWhileKeepingPredictedLeadCap){
 EgoEntryModel e;e.distance_to_merge=0;e.length=4;e.speed=2;e.max_speed=2;e.accel=2;e.departure_in_progress=true;
 GapCandidate g;g.leader_id=1;g.leader_pos=6;g.leader_length=4;g.leader_speed=1;
 DistanceEntryParams p;p.front_gap=2;p.speed_margin=0;p.horizon=0;
 // Constant-accel legacy with a leader limits ego speed; use an observed
 // measured speed above the capped profile to exercise continuing motion.
 e.measured_profile=true;e.profile_times={0,1};e.profile_speeds={0,2};
 auto old=evaluateDistanceEntry(g,e,p);EXPECT_FALSE(old.safe);
 p.clearance_only=true;auto r=evaluateDistanceEntry(g,e,p);ASSERT_TRUE(r.safe)<<r.reason;
 EXPECT_NEAR(r.front_min,2.,1e-6);
 p.follow_time_gap=.5;p.follow_standstill_gap=2.5;p.front_response_time=.5;p.front_max_decel=3;
 r=evaluateDistanceEntry(g,e,p);EXPECT_TRUE(r.leader_limited);
}
}

namespace mission_planner {
namespace {
RoundaboutConfig lockedEntryTestConfig(){
 auto c=testConfig();c.distance_enabled=true;c.gap_select_enabled=true;c.gs_require_merge_progress=true;
 c.distance_clearance_only=true;c.distance_commit_rear_yield_lock=true;c.distance_rear_yield=true;
 c.distance_measured_profile=true;auto e=measuredTestEgo();c.distance_profile_times=e.profile_times;c.distance_profile_speeds=e.profile_speeds;
 c.gs_prediction_follow_standstill_gap=2.5;c.gs_prediction_follow_time_gap=.5;
 c.comfortable_decel=2;c.distance_front_response=.5;c.distance_front_max_decel=3;
 c.distance_front_gap=3;c.distance_rear_gap=1;c.distance_confirm=0;c.mandatory_stop_seconds=0;c.yield_point={10,-10};return c;
}
RoundaboutInput lockedEntryTestInput(){
 auto input=approachInput(1);input.ego.speed=0;
 auto leader=circulatingObject(1,-.7,4.2),rear=circulatingObject(2,-2.7,4.2);
 leader.hit_count=rear.hit_count=10;input.objects={leader,rear};return input;
}
}
TEST(DistanceEntry,ApprovedCooperativeEntryIsNotCancelledByRearReevaluation){
 auto c=lockedEntryTestConfig();RoundaboutMission mission(c);auto input=lockedEntryTestInput();
 ASSERT_EQ(mission.update(input).state,RoundaboutState::kCommit);
 auto leader=circulatingObject(1,-.6,4.2),rear=circulatingObject(2,-1.1,8.);
 leader.hit_count=rear.hit_count=11;input.objects={leader,rear};input.now=1.4;
 auto out=mission.update(input);EXPECT_EQ(out.state,RoundaboutState::kCommit)<<out.reason;
 EXPECT_FALSE(out.constraint.stop_required)<<out.reason;EXPECT_GT(out.constraint.max_speed,0);
 EXPECT_NE(out.reason.find("approval latched"),std::string::npos);
 input.now=1.8;out=mission.update(input);EXPECT_EQ(out.state,RoundaboutState::kCommit)<<out.reason;
 EXPECT_FALSE(out.constraint.stop_required)<<out.reason;
}
TEST(DistanceEntry,ApprovedCooperativeEntryStillBrakesForActualFrontAndLocalizationLoss){
 auto c=lockedEntryTestConfig();RoundaboutMission mission(c);auto input=lockedEntryTestInput();
 ASSERT_EQ(mission.update(input).state,RoundaboutState::kCommit);
 input.now=1.4;input.ego.position={10,-5};input.ego.speed=3;
 auto front=circulatingObject(1,.1,0);front.hit_count=11;input.objects={front};
 auto out=mission.update(input);ASSERT_TRUE(out.constraint.lead_vehicle_active)<<out.reason;
 EXPECT_DOUBLE_EQ(out.constraint.max_speed,0.);
 input.now=1.5;input.localization_valid=false;out=mission.update(input);
 EXPECT_EQ(out.state,RoundaboutState::kFailsafe);EXPECT_TRUE(out.constraint.stop_required);EXPECT_DOUBLE_EQ(out.constraint.max_speed,0.);
}
TEST(DistanceEntry,ApprovalLockRetainsTrackerDegradationProtection){
 auto c=lockedEntryTestConfig();c.tracker_guard_commit_grace=0;c.degraded_speed=1.5;
 RoundaboutMission mission(c);auto input=lockedEntryTestInput();ASSERT_EQ(mission.update(input).state,RoundaboutState::kCommit);
 input.now=1.4;input.tracker_stream_valid=false;input.tracker_stream_fresh=false;
 auto out=mission.update(input);EXPECT_LE(out.constraint.max_speed,1.5);EXPECT_NE(out.reason.find("tracker degraded"),std::string::npos);
}
}

namespace mission_planner {
TEST(DistanceEntry,LockedEntryDoesNotExpireWhileActualFrontProtectionHolds){
 auto c=lockedEntryTestConfig();c.commit_timeout_seconds=6;RoundaboutMission mission(c);
 auto input=lockedEntryTestInput();ASSERT_EQ(mission.update(input).state,RoundaboutState::kCommit);
 input.now=8;input.ego.position={10,-5};input.ego.speed=0;
 auto front=circulatingObject(1,.1,0);front.hit_count=12;input.objects={front};
 auto out=mission.update(input);EXPECT_EQ(out.state,RoundaboutState::kCommit)<<out.reason;
 ASSERT_TRUE(out.constraint.lead_vehicle_active);EXPECT_DOUBLE_EQ(out.constraint.max_speed,0.);
 input.now=8.1;out=mission.update(input);EXPECT_EQ(out.state,RoundaboutState::kCommit);EXPECT_DOUBLE_EQ(out.constraint.max_speed,0.);
}
}

namespace mission_planner {
TEST(DistanceEntry,LockedEntryRetainsCachedFrontWhenRearGapIsUnsafe){
 auto c=lockedEntryTestConfig();RoundaboutMission mission(c);auto input=lockedEntryTestInput();
 ASSERT_EQ(mission.update(input).state,RoundaboutState::kCommit);
 input.now=1.4;input.ego.position={10,-5};input.ego.speed=0;
 auto front=circulatingObject(1,.1,0);front.hit_count=11;input.objects={front};
 ASSERT_TRUE(mission.update(input).constraint.lead_vehicle_active);
 input.now=1.5;auto rear=circulatingObject(2,-2.7,8);rear.hit_count=12;input.objects={rear};
 auto out=mission.update(input);EXPECT_EQ(out.state,RoundaboutState::kCommit);
 ASSERT_TRUE(out.constraint.lead_vehicle_active)<<out.reason;EXPECT_EQ(out.constraint.lead_vehicle_id,1);
 EXPECT_DOUBLE_EQ(out.constraint.max_speed,0.);
}
}

namespace mission_planner {
TEST(RoundaboutGapSelect, TtcApprovalLockKeepsCommitButRetainsFrontAndLocalizationProtection) {
 auto c=lockedEntryTestConfig();c.distance_enabled=false;c.gs_sequential=true;
 c.gs_zone_rule_enabled=false;c.gs_commit_rear_yield_lock=true;c.gs_measured_entry_profile=true;
 c.gs_confirm_seconds=0;c.gs_leader_start_distance=0;
 RoundaboutMission mission(c);auto input=lockedEntryTestInput();input.objects.clear();
 ASSERT_EQ(mission.update(input).state,RoundaboutState::kCommit);
 input.now=1.4;auto rear=circulatingObject(2,-.1,8);rear.hit_count=10;input.objects={rear};
 auto out=mission.update(input);EXPECT_EQ(out.state,RoundaboutState::kCommit)<<out.reason;
 EXPECT_FALSE(out.constraint.stop_required)<<out.reason;
 input.now=8;out=mission.update(input);EXPECT_EQ(out.state,RoundaboutState::kCommit)<<out.reason;
 input.now=8.1;input.ego.position={10,-5};input.ego.speed=3;
 auto front=circulatingObject(1,.1,0);front.has_moved=true;front.confirmed=true;front.velocity_valid=true;front.hit_count=12;
 input.objects={front};out=mission.update(input);
 EXPECT_TRUE(out.constraint.lead_vehicle_active);EXPECT_NEAR(out.constraint.max_speed,0,1e-6);
 input.localization_valid=false;input.now=8.2;
 EXPECT_EQ(mission.update(input).state,RoundaboutState::kFailsafe);
}
TEST(RoundaboutGapSelect, ProjectedEntryRetainsFrontWhenSpeedOrDetectionDrops) {
 auto c=lockedEntryTestConfig();c.distance_enabled=false;c.gs_sequential=true;
 c.gs_zone_rule_enabled=false;c.gs_commit_rear_yield_lock=true;c.gs_measured_entry_profile=true;
 c.gs_route_body_projection=true;c.gs_confirm_seconds=0;c.gs_leader_start_distance=0;c.tw_ghost_age=1;
 RoundaboutMission mission(c);auto input=lockedEntryTestInput();input.objects.clear();
 input.route={{10,-30},{10,0}};
 for(int i=1;i<=40;++i)input.route.push_back({10*std::cos(i*.025),10*std::sin(i*.025)});
 ASSERT_EQ(mission.update(input).state,RoundaboutState::kCommit);
 input.now=1.4;input.ego.position={10,-5};input.ego.speed=3;
 auto front=circulatingObject(1,.1,0);front.hit_count=12;input.objects={front};
 auto out=mission.update(input);ASSERT_DOUBLE_EQ(out.constraint.max_speed,0);
 input.now=1.5;front.velocity_valid=false;front.confirmed=false;
 front.velocity_x=40;front.hit_count=13;input.objects={front};
 out=mission.update(input);EXPECT_EQ(out.state,RoundaboutState::kCommit);
 EXPECT_DOUBLE_EQ(out.constraint.max_speed,0)<<"Invalid speed must not remove a proven front";
 input.now=1.6;input.objects.clear();out=mission.update(input);
 EXPECT_DOUBLE_EQ(out.constraint.max_speed,0)<<"A short detection dropout must retain front braking";
 input.now=2.4;out=mission.update(input);EXPECT_DOUBLE_EQ(out.constraint.max_speed,0);
 input.now=2.55;out=mission.update(input);
 EXPECT_GT(out.constraint.max_speed,0)<<"Synthetic cache publications must not renew the real-detection lifetime";
}
TEST(RoundaboutGapSelect, TimedApproachKeepsChosenPolicyAndHoldsMissingFront) {
 auto c=lockedEntryTestConfig();c.distance_enabled=false;c.gs_sequential=true;
 c.gs_zone_rule_enabled=false;c.gs_commit_rear_yield_lock=true;c.gs_measured_entry_profile=true;
 c.gs_route_body_projection=true;c.gs_timed_entry_approach=true;c.gs_confirm_seconds=0;
 c.gs_use_mpc_prediction=true;c.gs_use_tracker_velocity=true;c.gs_check_horizon=1.5;
 c.gs_front_headway=.05;c.gs_seq_rear_headway=.2;c.gs_ttc_rear=1;c.gs_late_margin=.2;
 c.gs_standstill_gap=.5;c.gs_speed_margin=.2;c.gs_leader_start_distance=8;
 c.center={0,0};c.nominal_radius=18.338;c.inner_radius=16;c.outer_radius=20;
 c.yield_point={3.327553,-24.808677};c.conflict_point={4.335901,-20.967604};
 RoundaboutMission mission(c);auto input=approachInput(1);
 input.prediction_speed_limit=7;input.prediction_accel_limit=3.2;
 input.ego.position=c.yield_point;input.ego.speed=0;input.ego.yaw=78.92*kPi/180;
 input.route.clear();
 for(int i=0;i<=160;++i)input.route.push_back({c.yield_point.x+.1*i*std::cos(input.ego.yaw),c.yield_point.y+.1*i*std::sin(input.ego.yaw)});
 const double origin=std::atan2(c.conflict_point.y,c.conflict_point.x);
 const auto car=[&](int id,double pos) {
   const double angle=origin+pos/c.nominal_radius;
   auto o=circulatingObject(id,angle,4.7);
   o.position={c.nominal_radius*std::cos(angle),c.nominal_radius*std::sin(angle)};o.hit_count=12;return o;
 };
 input.objects={car(1,-7.75),car(2,-30.5)};
 auto out=mission.update(input);
 ASSERT_EQ(out.state,RoundaboutState::kCommit)<<out.reason;
 ASSERT_NE(out.reason.find("timing front clearance"),std::string::npos)<<out.reason;
 EXPECT_GT(out.constraint.max_speed,0);EXPECT_LT(out.constraint.max_speed,3);
 input.now=1.1;input.objects={car(2,-30.0)};out=mission.update(input);
 EXPECT_EQ(out.state,RoundaboutState::kCommit);
 EXPECT_DOUBLE_EQ(out.constraint.max_speed,0)<<"Lost front must not release the timed approach cap";
 input.now=1.2;input.objects={car(1,-6.8),car(2,-29.5)};out=mission.update(input);
 EXPECT_GT(out.constraint.max_speed,0);
 EXPECT_NE(out.reason.find("timing front clearance"),std::string::npos)<<"Chosen approach policy must remain locked";
}
TEST(RoundaboutGapSelect, SeparateRadialFootprintsCannotBecomeZeroDistanceLead) {
 auto c=lockedEntryTestConfig();c.distance_enabled=false;c.gs_sequential=true;
 c.gs_zone_rule_enabled=false;c.gs_commit_rear_yield_lock=true;c.gs_measured_entry_profile=true;
 c.gs_route_body_projection=true;c.gs_confirm_seconds=0;c.inner_radius=7;
 RoundaboutMission mission(c);auto input=lockedEntryTestInput();input.objects.clear();
 input.route={{10,-30},{10,0}};
 for(int i=1;i<=40;++i)input.route.push_back({10*std::cos(i*.025),10*std::sin(i*.025)});
 ASSERT_EQ(mission.update(input).state,RoundaboutState::kCommit);
 input.now=1.1;input.ego.position={10,0};input.ego.yaw=kPi/2;input.ego.speed=2;
 auto car=circulatingObject(1,.1,2);car.position={7.5*std::cos(.1),7.5*std::sin(.1)};
 car.hit_count=12;input.objects={car};auto out=mission.update(input);
 ASSERT_EQ(out.state,RoundaboutState::kCirculate)<<out.reason;
 EXPECT_GT(out.constraint.max_speed,0)<<"Same angular sector is not body overlap across separated radii";
}
}

namespace mission_planner {
TEST(DistanceEntry, MovingLeaderCannotCreateClearanceBelowMinimumBumperGap) {
 EXPECT_DOUBLE_EQ(frontBrakingSpeedCap(0,4,2.5,2,.5,3,3),0);
 EXPECT_DOUBLE_EQ(frontBrakingSpeedCap(2.5,4,2.5,2,.5,3,3),0);
 EXPECT_GT(frontBrakingSpeedCap(6,4,2.5,2,.5,3,3),0);
}
}

namespace mission_planner {
TEST(RoundaboutSpatial, RollingApproachHasNoMandatoryStopAndKeepsMpcIndependent) {
  auto c=testConfig();c.gap_select_enabled=true;c.gs_sequential=true;
  c.gs_spatial_gap_enabled=true;c.gs_route_body_projection=true;c.gs_timed_entry_approach=true;
  c.gs_confirm_seconds=0;c.mandatory_stop_seconds=0;c.yield_roll_speed=2;c.yield_go_distance=4;
  c.gs_approach_decel=1;c.gs_commit_rear_yield_lock=true;c.yield_point={10,-10};
  RoundaboutMission mission(c);auto in=approachInput(0);
  in.ego.position={10,-16};in.ego.speed=4;
  auto out=mission.update(in);
  EXPECT_EQ(out.state,RoundaboutState::kYield)<<out.reason;
  EXPECT_FALSE(out.constraint.stop_required);
  EXPECT_NEAR(out.constraint.max_speed,std::sqrt(8.),.01);
  in.now=.1;in.ego.position={10,-12};in.ego.speed=2;
  out=mission.update(in);
  EXPECT_EQ(out.state,RoundaboutState::kCommit)<<out.reason;
  EXPECT_FALSE(out.constraint.stop_required);
  EXPECT_GT(out.constraint.max_speed,c.yield_roll_speed);
}
}

namespace mission_planner {
TEST(RoundaboutSpatial,ClosedLoopSpatialEntryMaintainsFrontClearance){
 // Exercise the 2.5 m / 0.35 s profile and 0.2 s contact timing reserve with delayed braking.
 // The rejected 2.0 m / 0.25 s experiment is documented in the analysis report.
 for(int approval_policy:{6,7})for(double actual_decel:{2.5,2.0})for(int scenario:{0,1,2})
 for(double npc_speed:{3.5,4.2,5.5})for(double onset:{.4,1.,1.5})
 for(double response_delay:{.2,.3})for(double leader_decel:{1.5,2.,3.}){
  const double headway=approval_policy>=4?.35:.5;
  const bool clearance_only=approval_policy!=0;
  SCOPED_TRACE(::testing::Message()<<"policy="<<approval_policy<<" clearance="<<clearance_only<<" npc="<<npc_speed<<" onset="<<onset<<" response="<<response_delay<<" leader_decel="<<leader_decel);
  const bool slowing_leader=scenario!=0;
  auto c=testConfig();c.distance_enabled=true;c.gap_select_enabled=true;c.gs_require_merge_progress=true;
  c.distance_commit_rear_yield_lock=approval_policy==2;c.distance_clearance_only=clearance_only;c.distance_measured_profile=true;auto model=measuredTestEgo();c.distance_profile_times=model.profile_times;c.distance_profile_speeds=model.profile_speeds;
  c.gs_speed_margin=.2;c.comfortable_decel=2.;c.distance_front_response=.5;c.distance_front_max_decel=3.;c.distance_front_gap=2;c.distance_rear_gap=1.5;c.distance_confirm=.15;c.distance_rear_yield=true;
  c.gs_prediction_follow_standstill_gap=2.5;c.gs_prediction_follow_time_gap=headway;c.gs_standstill_gap=.8;
  if (approval_policy >= 3) {
    c.distance_enabled=false;c.gs_sequential=true;c.gs_zone_rule_enabled=false;
    c.gs_route_body_projection=approval_policy>=4;c.gs_ego_width=1.9;
    c.gs_timed_entry_approach=true;c.gs_spatial_gap_enabled=true;
    c.gs_approach_time_buffer=.2;c.gs_approach_accel_compensation=true;
    c.gs_rear_ttc_at_entry=false; // Deployed full-trajectory rear TTC check.
    c.gs_commit_rear_yield_lock=true;c.gs_measured_entry_profile=true;c.gs_use_mpc_prediction=true;
    c.gs_front_headway=approval_policy>=4?.05:.3;c.gs_seq_rear_headway=.2;c.gs_ttc_rear=1.;c.gs_late_margin=.2;
    c.gs_check_horizon=1.5;c.gs_confirm_seconds=.1;c.gs_leader_start_distance=8;
    c.gs_leader_decel_horizon=1.;c.gs_leader_decel_limit=1.5;
  }
  c.mandatory_stop_seconds=0;c.yield_roll_speed=2;c.yield_go_distance=4;c.yield_point={10,-10};c.gs_commit_abort_enabled=true;
  RoundaboutMission mission(c);auto input=approachInput(0);input.route={{10,-30},{10,0}};
  input.prediction_speed_limit=8.5;input.prediction_accel_limit=3.2;
  for(int j=1;j<=40;++j)input.route.push_back({10*std::cos(j*.025),10*std::sin(j*.025)});
  double s=-10,v=0,leader=-7,leader_v=npc_speed,rear=-27,rear_v=npc_speed,commit=-1,min_gap=1e9,brake_since=-1;bool cleared=false;
  for(int i=0;i<600;++i){double now=i*.05;input.now=now;input.ego.position=s<0?Point2{10,s}:Point2{10*std::cos(s/10),10*std::sin(s/10)};
   input.ego.yaw=kPi/2+std::max(0.,s)/10;input.ego.speed=v;
   auto l=circulatingObject(1,leader/10,leader_v),r=circulatingObject(2,rear/10,rear_v);l.hit_count=r.hit_count=i/2+10;
   input.objects={l,r};auto out=mission.update(input);
   if(out.state==RoundaboutState::kCommit&&commit<0)commit=now;
   double cap=out.constraint.max_speed<0?8.5:out.constraint.max_speed;
   if(out.constraint.stop_required)cap=std::min(cap,std::sqrt(2*c.comfortable_decel*std::max(0.,out.constraint.stop_distance)));
   if(out.constraint.lead_vehicle_active){double gap=std::max(0.,out.constraint.lead_distance-c.gs_prediction_follow_standstill_gap);
    cap=std::min(cap,std::min(std::sqrt(out.constraint.lead_speed*out.constraint.lead_speed+2*c.comfortable_decel*gap),
      std::max(0.,out.constraint.lead_speed+(gap-headway*v)/headway)));}
   if(cap<v&&brake_since<0)brake_since=now;
   if(cap>=v)brake_since=-1;
   const bool brake_delay=brake_since>=0&&now-brake_since<response_delay;
   const double next=(commit>=0&&now-commit<.23)||brake_delay?v:(v<cap?std::min(cap,v+3.2*.05):std::max(cap,v-actual_decel*.05));
   s+=.5*(v+next)*.05;v=next;
   if(slowing_leader&&commit>=0&&now-commit>onset){
    if(scenario==2&&now-commit>onset+1.)leader_v=std::min(npc_speed,leader_v+2*.05);
    else leader_v=std::max(0.,leader_v-leader_decel*.05);
   }
   leader+=leader_v*.05;
   // NPCs share one lane: the rear must queue behind a stopped leader rather
   // than drive through it and become a fictitious new closest front car.
   const double rear_gap=std::max(0.,leader-rear-6.);
   const double rear_cap=std::min(npc_speed,std::min(std::sqrt(leader_v*leader_v+4*rear_gap),
     std::max(0.,leader_v+(rear_gap-.5*rear_v)/.5)));
   const double rear_next=rear_v<rear_cap?std::min(rear_cap,rear_v+.1):std::max(rear_cap,rear_v-.15);
   rear+=.5*(rear_v+rear_next)*.05;rear_v=rear_next;
   if (approval_policy>=4) {
    const Point2 ec=s<0?Point2{10,s}:Point2{10*std::cos(s/10),10*std::sin(s/10)};
    const Point2 lc{10*std::cos(leader/10),10*std::sin(leader/10)};
    const double ey=kPi/2+std::max(0.,s)/10,ly=kPi/2+leader/10;
    double separation=-1e9;
    for(double axis:{ey,ey+kPi/2,ly,ly+kPi/2}) {
      const double between=std::abs((lc.x-ec.x)*std::cos(axis)+(lc.y-ec.y)*std::sin(axis));
      const double e_support=2.3*std::abs(std::cos(ey-axis))+.95*std::abs(std::sin(ey-axis));
      const double l_support=2.*std::abs(std::cos(ly-axis))+.95*std::abs(std::sin(ly-axis));
      separation=std::max(separation,between-e_support-l_support);
    }
    min_gap=std::min(min_gap,separation);
   } else if(s+2.3>=0)min_gap=std::min(min_gap,leader-2.-s-2.3);
   if(s>=2.3){cleared=true;break;}
  }
  EXPECT_GE(min_gap,approval_policy>=4?.3:1.5)<<"front contact, scenario="<<scenario<<" slowing="<<slowing_leader<<" headway="<<headway<<" decel="<<actual_decel;
  if(scenario!=1){EXPECT_TRUE(cleared)<<"scenario="<<scenario;EXPECT_GE(commit,0.);// TTC follows its own sequential slot selection; require a bounded
    // departure after the mandatory stop, preserving the same 1.5m minimum.
    EXPECT_LE(commit,approval_policy>=3?4.0:3.1);}
 }
}
}

namespace mission_planner {
TEST(RoundaboutSpatial, PassedCommittedSlotFollowsItsRearOrHoldsBeforeContact) {
 for(double next_rear:{-25.,-10.}) {
 auto c=lockedEntryTestConfig();c.gs_spatial_gap_enabled=true;c.distance_enabled=false;c.gs_sequential=true;
 c.gs_zone_rule_enabled=false;c.gs_commit_rear_yield_lock=true;c.gs_measured_entry_profile=true;
 c.gs_route_body_projection=true;c.gs_timed_entry_approach=true;c.gs_confirm_seconds=0;
 c.gs_use_mpc_prediction=true;c.gs_use_tracker_velocity=true;c.gs_check_horizon=1.5;
 c.gs_front_headway=.05;c.gs_seq_rear_headway=.2;c.gs_ttc_rear=1;c.gs_late_margin=.2;
 c.gs_standstill_gap=.5;c.gs_speed_margin=.2;c.gs_leader_start_distance=8;
 c.center={0,0};c.nominal_radius=18.338;c.inner_radius=16;c.outer_radius=20;
 c.yield_point={3.327553,-24.808677};c.conflict_point={4.335901,-20.967604};
 RoundaboutMission mission(c);auto input=approachInput(1);
 input.prediction_speed_limit=7;input.prediction_accel_limit=3.2;
 input.ego.position=c.yield_point;input.ego.speed=0;input.ego.yaw=78.92*kPi/180;
 input.route.clear();
 for(int i=0;i<=160;++i)input.route.push_back({c.yield_point.x+.1*i*std::cos(input.ego.yaw),c.yield_point.y+.1*i*std::sin(input.ego.yaw)});
 const double origin=std::atan2(c.conflict_point.y,c.conflict_point.x);
 const auto car=[&](int id,double pos) {
   const double angle=origin+pos/c.nominal_radius;
   auto o=circulatingObject(id,angle,4.7);
   o.position={c.nominal_radius*std::cos(angle),c.nominal_radius*std::sin(angle)};o.hit_count=12;return o;
 };
 input.objects={car(1,2.0),car(2,-30.5)};
 auto out=mission.update(input);
 ASSERT_EQ(out.state,RoundaboutState::kCommit)<<out.reason;

 input.now=1.1;input.objects={car(1,12),car(2,0),car(3,next_rear)};
 out=mission.update(input);EXPECT_EQ(out.state,RoundaboutState::kCommit);
 if(next_rear<-20) {
   bool found=false;for(const auto& gap:out.gaps)if(gap.selected){found=true;EXPECT_EQ(gap.leader_id,2);EXPECT_EQ(gap.follower_id,3);}
   EXPECT_TRUE(found);EXPECT_GT(out.constraint.max_speed,0);
 } else {
   EXPECT_DOUBLE_EQ(out.constraint.max_speed,0)<<"Cannot enter the expired slot when next slot is narrow";
 }
 }
}

}

namespace mission_planner {
TEST(RoundaboutSpatial, FarObservationDoesNotCapUrbanSpeedButNearApproachSlows) {
  auto c=testConfig();c.gap_select_enabled=true;c.gs_sequential=true;
  c.gs_spatial_gap_enabled=true;c.gs_route_body_projection=true;c.gs_timed_entry_approach=true;
  c.gs_confirm_seconds=0;c.mandatory_stop_seconds=0;c.yield_roll_speed=2;c.yield_go_distance=4;
  c.gs_approach_slow_distance=18;c.gs_approach_decel=2.5;c.approach_speed=7;
  c.yield_point={10,-10};c.activation_distance=60;c.decision_distance=60;
  RoundaboutMission mission(c);auto in=approachInput(0);
  in.route={{10,-80},{10,-2},{10,0},{9.8,2}};
  in.ego.position={10,-35};in.ego.speed=8;
  auto out=mission.update(in);EXPECT_EQ(out.state,RoundaboutState::kYield);
  EXPECT_DOUBLE_EQ(out.constraint.max_speed,-1.0)<<"Observation must not impose the old 7m/s cap";
  EXPECT_FALSE(out.constraint.stop_required);
  in.now=.1;in.ego.position={10,-20};in.ego.speed=6;
  out=mission.update(in);
  EXPECT_NEAR(out.constraint.max_speed,std::sqrt(34.),.01);
  EXPECT_FALSE(out.constraint.stop_required);
}
TEST(RoundaboutSpatial, FarUnknownGapRetainsYieldStopTargetWithoutSpeedCap) {
  auto c=testConfig();c.gap_select_enabled=true;c.gs_sequential=true;c.gs_spatial_gap_enabled=true;
  c.gs_route_body_projection=true;c.gs_timed_entry_approach=true;
  c.mandatory_stop_seconds=0;c.yield_roll_speed=2;c.yield_go_distance=4;
  c.yield_point={10,-10};c.activation_distance=c.decision_distance=60;
  RoundaboutMission mission(c);auto in=approachInput(0);
  in.route={{10,-80},{10,-2},{10,0},{9.8,2}};
  in.ego.position={10,-35};in.ego.speed=8;in.tracker_stream_fresh=false;
  in.tracker_stream_valid=false;
  auto out=mission.update(in);
  EXPECT_DOUBLE_EQ(out.constraint.max_speed,-1.0);
  EXPECT_TRUE(out.constraint.stop_required);
  EXPECT_GT(out.constraint.stop_distance,20.0);
}
}

namespace mission_planner {
TEST(RoundaboutEntryGuard, AdaptiveBrakingStartsForActualSpeedAndCannotReleaseMidApproach) {
  auto c=testConfig();c.gap_select_enabled=true;c.gs_sequential=true;c.gs_spatial_gap_enabled=true;
  c.gs_adaptive_approach=true;c.gs_approach_decel=2.5;c.gs_approach_response=.3;
  c.comfortable_decel=2.0;
  c.gs_spatial_arrival_check=true;c.gs_route_body_projection=true;c.gs_timed_entry_approach=true;
  c.mandatory_stop_seconds=0;c.yield_roll_speed=2;c.yield_go_distance=4;
  c.yield_point={10,-10};c.activation_distance=c.decision_distance=60;
  RoundaboutMission mission(c);auto in=approachInput(0);
  in.route={{10,-90},{10,-2},{10,0},{9.8,2}};in.ego.position={10,-70};in.ego.speed=13.5;
  auto out=mission.update(in);EXPECT_DOUBLE_EQ(out.constraint.max_speed,-1);
  in.now=.1;in.ego.position={10,-50};out=mission.update(in);
  EXPECT_GT(out.constraint.max_speed,7);EXPECT_LT(out.constraint.max_speed,13.5);
  EXPECT_TRUE(out.constraint.stop_required)<<"Prediction is not approval before the rolling gate";
  const double first=out.constraint.max_speed;
  in.now=.2;in.ego.position={10,-40};in.ego.speed=10;
  out=mission.update(in);EXPECT_GE(out.constraint.max_speed,0);EXPECT_LT(out.constraint.max_speed,first);
  mission.reset(.3);in.now=.3;in.ego.speed=8;out=mission.update(in);
  EXPECT_DOUBLE_EQ(out.constraint.max_speed,-1)<<"Reset must clear the braking latch";
}
TEST(RoundaboutEntryGuard, RearIsNotAccLeaderButImminentRearBodyCollisionStillStops) {
  auto c=lockedEntryTestConfig();c.gs_body_collision_guard=true;c.gs_route_body_projection=true;
  c.gs_confirm_seconds=0;RoundaboutMission mission(c);auto in=lockedEntryTestInput();
  in.route={{10,-30},{10,0}};for(int i=1;i<=40;++i)in.route.push_back({10*std::cos(i*.025),10*std::sin(i*.025)});
  in.objects.clear();ASSERT_EQ(mission.update(in).state,RoundaboutState::kCommit);
  in.now=1.1;in.ego.position={10,0};in.ego.yaw=kPi/2;in.ego.speed=2;
  auto rear=circulatingObject(2,-.7,2);rear.hit_count=12;in.objects={rear};
  auto out=mission.update(in);EXPECT_NE(out.constraint.lead_vehicle_id,2);EXPECT_GT(out.constraint.max_speed,0);
  in.now=1.2;rear=circulatingObject(2,-.35,8);rear.hit_count=13;in.objects={rear};
  out=mission.update(in);EXPECT_NE(out.constraint.lead_vehicle_id,2);
  EXPECT_DOUBLE_EQ(out.constraint.max_speed,0);EXPECT_NE(out.reason.find("body collision guard"),std::string::npos);
}
TEST(RoundaboutEntryGuard, BoxSeparationChecksCornersInsteadOfOnlyArcOverlap) {
  EXPECT_GT(boxSeparation({0,0},0,4.6,1.9,{-3,4},0,4.5,1.9),2.0);
  EXPECT_LT(boxSeparation({0,0},0,4.6,1.9,{-3,0},0,4.5,1.9),0);
  EXPECT_LT(boxSeparation({0,0},0,4.6,1.9,{2,1},.6,4.5,1.9),0);
}
TEST(RoundaboutEntryGuard, WorldBoundsAreNotRotatedTwiceByBodyGuard) {
  auto c=lockedEntryTestConfig();c.gs_body_collision_guard=true;c.gs_route_body_projection=true;
  c.gs_collision_horizon=.1;c.gs_confirm_seconds=0;
  RoundaboutMission mission(c);auto in=lockedEntryTestInput();
  in.route={{10,-30},{10,0}};
  for(int i=1;i<=40;++i)in.route.push_back({10*std::cos(i*.025),10*std::sin(i*.025)});
  in.objects.clear();ASSERT_EQ(mission.update(in).state,RoundaboutState::kCommit);
  in.now=1.1;in.ego.position={10,0};in.ego.yaw=kPi/2;in.ego.speed=0;
  auto lead=circulatingObject(2,.45,2);lead.hit_count=12;
  lead.length=4.6;lead.width=1.9;lead.aabb_x=4.6;lead.aabb_y=1.9;in.objects={lead};
  auto out=mission.update(in);
  EXPECT_EQ(out.reason.find("body collision guard"),std::string::npos)<<out.reason;
  // The same measured box, placed on the ego, must still trigger protection.
  in.now=1.2;lead.position=in.ego.position;in.objects={lead};out=mission.update(in);
  EXPECT_NE(out.reason.find("body collision guard"),std::string::npos)<<out.reason;
}
}

namespace mission_planner {
TEST(RoundaboutEntryGuard, ArrivalRejectsCurrentWideSlotAndKeepsApprovedPair) {
 for(double rear_pos:{-12.,-50.}) {
 auto c=lockedEntryTestConfig();c.gs_spatial_gap_enabled=true;c.gs_spatial_arrival_check=true;c.distance_enabled=false;c.gs_sequential=true;
 c.gs_zone_rule_enabled=false;c.gs_commit_rear_yield_lock=true;c.gs_measured_entry_profile=true;
 c.gs_route_body_projection=true;c.gs_timed_entry_approach=true;c.gs_confirm_seconds=0;
 c.gs_use_mpc_prediction=true;c.gs_use_tracker_velocity=true;c.gs_check_horizon=1.5;
 c.gs_front_headway=.05;c.gs_seq_rear_headway=.2;c.gs_ttc_rear=1;c.gs_late_margin=.2;
 c.gs_standstill_gap=.5;c.gs_speed_margin=.2;c.gs_leader_start_distance=8;
 c.center={0,0};c.nominal_radius=18.338;c.inner_radius=16;c.outer_radius=20;
 c.yield_point={3.327553,-24.808677};c.conflict_point={4.335901,-20.967604};
 RoundaboutMission mission(c);auto input=approachInput(1);
 input.prediction_speed_limit=7;input.prediction_accel_limit=3.2;
 input.ego.position=c.yield_point;input.ego.speed=0;input.ego.yaw=78.92*kPi/180;
 input.route.clear();
 for(int i=0;i<=160;++i)input.route.push_back({c.yield_point.x+.1*i*std::cos(input.ego.yaw),c.yield_point.y+.1*i*std::sin(input.ego.yaw)});
 const double origin=std::atan2(c.conflict_point.y,c.conflict_point.x);
 const auto car=[&](int id,double pos) {
   const double angle=origin+pos/c.nominal_radius;
   auto o=circulatingObject(id,angle,4.7);
   o.position={c.nominal_radius*std::cos(angle),c.nominal_radius*std::sin(angle)};o.hit_count=12;return o;
 };
 input.objects={car(1,2),car(2,rear_pos)};
 if(rear_pos>-20)input.objects.push_back(car(3,-26));
 auto out=mission.update(input);
 if(rear_pos>-20) {
   bool unsafe_pair_approved=false;for(const auto& g:out.gaps)if(g.selected&&g.leader_id==1&&g.follower_id==2&&out.state==RoundaboutState::kCommit)unsafe_pair_approved=true;
   EXPECT_FALSE(unsafe_pair_approved)<<out.reason;
   // The old rear has passed: no approval may be obtained from the expired
   // pair, and subsequent decisions must move on instead of waiting on it.
   input.objects={car(1,20),car(2,4),car(3,-10),car(4,-23)};
   for(int i=1;i<=4;++i){input.now=1+.1*i;out=mission.update(input);}
   bool selected=false;
   for(const auto& g:out.gaps)if(g.selected){selected=true;EXPECT_NE(g.leader_id,1)<<out.reason;}
   EXPECT_TRUE(selected)<<out.reason;
 } else {
   ASSERT_EQ(out.state,RoundaboutState::kCommit)<<out.reason;
   EXPECT_NE(out.reason.find("approval latched"),std::string::npos);
   EXPECT_GT(out.constraint.max_speed,0);
   input.now=1.1;input.objects={car(1,5),car(2,-1),car(3,-25)};
   out=mission.update(input);EXPECT_EQ(out.state,RoundaboutState::kCommit);
   for(const auto& g:out.gaps)if(g.selected){EXPECT_EQ(g.leader_id,1);EXPECT_EQ(g.follower_id,2);}
   EXPECT_EQ(out.reason.find("timing front clearance"),std::string::npos);
 }
 }
}

}

namespace mission_planner {
TEST(RoundaboutEntryGuard,ClosedLoopArrivalEntryMaintainsFrontClearance){
 // Exercise the 2.5 m / 0.35 s profile and 0.2 s contact timing reserve with delayed braking.
 // The rejected 2.0 m / 0.25 s experiment is documented in the analysis report.
 for(int approval_policy:{6})for(double actual_decel:{2.5,2.0})for(int scenario:{0,1,2})
 for(double npc_speed:{3.5,4.2,5.5})for(double onset:{.4,1.,1.5})
 for(double response_delay:{.2,.3})for(double leader_decel:{1.5,2.,3.}){
  const double headway=approval_policy>=4?.35:.5;
  const bool clearance_only=approval_policy!=0;
  SCOPED_TRACE(::testing::Message()<<"policy="<<approval_policy<<" clearance="<<clearance_only<<" npc="<<npc_speed<<" onset="<<onset<<" response="<<response_delay<<" leader_decel="<<leader_decel);
  const bool slowing_leader=scenario!=0;
  auto c=testConfig();c.distance_enabled=true;c.gap_select_enabled=true;c.gs_require_merge_progress=true;
  c.distance_commit_rear_yield_lock=approval_policy==2;c.distance_clearance_only=clearance_only;c.distance_measured_profile=true;auto model=measuredTestEgo();c.distance_profile_times=model.profile_times;c.distance_profile_speeds=model.profile_speeds;
  c.gs_speed_margin=.2;c.comfortable_decel=2.;c.distance_front_response=.5;c.distance_front_max_decel=3.;c.distance_front_gap=2;c.distance_rear_gap=1.5;c.distance_confirm=.15;c.distance_rear_yield=true;
  c.gs_prediction_follow_standstill_gap=2.5;c.gs_prediction_follow_time_gap=headway;c.gs_standstill_gap=.8;
  if (approval_policy >= 3) {
    c.distance_enabled=false;c.gs_sequential=true;c.gs_zone_rule_enabled=false;
    c.gs_route_body_projection=approval_policy>=4;c.gs_ego_width=1.9;
    c.gs_timed_entry_approach=true;c.gs_spatial_gap_enabled=true;
    c.gs_spatial_arrival_check=true;c.gs_body_collision_guard=true;c.gs_adaptive_approach=true;c.gs_approach_decel=2.5;
    c.gs_opportunity_enabled=approval_policy==7;
    c.gs_approach_time_buffer=.2;c.gs_approach_accel_compensation=true;
    c.gs_rear_ttc_at_entry=false; // Deployed full-trajectory rear TTC check.
    c.gs_commit_rear_yield_lock=true;c.gs_measured_entry_profile=true;c.gs_use_mpc_prediction=true;
    c.gs_front_headway=approval_policy>=4?.05:.3;c.gs_seq_rear_headway=.2;c.gs_ttc_rear=1.;c.gs_late_margin=.2;
    c.gs_check_horizon=1.5;c.gs_confirm_seconds=.1;c.gs_leader_start_distance=8;
    c.gs_leader_decel_horizon=1.;c.gs_leader_decel_limit=1.5;
  }
  c.mandatory_stop_seconds=0;c.yield_roll_speed=2;c.yield_go_distance=4;c.yield_point={10,-10};c.gs_commit_abort_enabled=true;
  RoundaboutMission mission(c);auto input=approachInput(0);input.route={{10,-30},{10,0}};
  input.prediction_speed_limit=8.5;input.prediction_accel_limit=3.2;
  for(int j=1;j<=40;++j)input.route.push_back({10*std::cos(j*.025),10*std::sin(j*.025)});
  double s=-10,v=0,leader=-7,leader_v=npc_speed,rear=-27,rear_v=npc_speed,commit=-1,min_gap=1e9,brake_since=-1;bool cleared=false;
  for(int i=0;i<600;++i){double now=i*.05;input.now=now;input.ego.position=s<0?Point2{10,s}:Point2{10*std::cos(s/10),10*std::sin(s/10)};
   input.ego.yaw=kPi/2+std::max(0.,s)/10;input.ego.speed=v;
   auto l=circulatingObject(1,leader/10,leader_v),r=circulatingObject(2,rear/10,rear_v);l.hit_count=r.hit_count=i/2+10;
   input.objects={l,r};auto out=mission.update(input);
   if(out.state==RoundaboutState::kCommit&&commit<0)commit=now;
   double cap=out.constraint.max_speed<0?8.5:out.constraint.max_speed;
   if(out.constraint.stop_required)cap=std::min(cap,std::sqrt(2*c.comfortable_decel*std::max(0.,out.constraint.stop_distance)));
   if(out.constraint.lead_vehicle_active){double gap=std::max(0.,out.constraint.lead_distance-c.gs_prediction_follow_standstill_gap);
    cap=std::min(cap,std::min(std::sqrt(out.constraint.lead_speed*out.constraint.lead_speed+2*c.comfortable_decel*gap),
      std::max(0.,out.constraint.lead_speed+(gap-headway*v)/headway)));}
   if(cap<v&&brake_since<0)brake_since=now;
   if(cap>=v)brake_since=-1;
   const bool brake_delay=brake_since>=0&&now-brake_since<response_delay;
   const double next=(commit>=0&&now-commit<.23)||brake_delay?v:(v<cap?std::min(cap,v+3.2*.05):std::max(cap,v-actual_decel*.05));
   s+=.5*(v+next)*.05;v=next;
   if(slowing_leader&&commit>=0&&now-commit>onset){
    if(scenario==2&&now-commit>onset+1.)leader_v=std::min(npc_speed,leader_v+2*.05);
    else leader_v=std::max(0.,leader_v-leader_decel*.05);
   }
   leader+=leader_v*.05;
   // NPCs share one lane: the rear must queue behind a stopped leader rather
   // than drive through it and become a fictitious new closest front car.
   const double rear_gap=std::max(0.,leader-rear-6.);
   const double rear_cap=std::min(npc_speed,std::min(std::sqrt(leader_v*leader_v+4*rear_gap),
     std::max(0.,leader_v+(rear_gap-.5*rear_v)/.5)));
   const double rear_next=rear_v<rear_cap?std::min(rear_cap,rear_v+.1):std::max(rear_cap,rear_v-.15);
   rear+=.5*(rear_v+rear_next)*.05;rear_v=rear_next;
   if (approval_policy>=4) {
    const Point2 ec=s<0?Point2{10,s}:Point2{10*std::cos(s/10),10*std::sin(s/10)};
    const Point2 lc{10*std::cos(leader/10),10*std::sin(leader/10)};
    const double ey=kPi/2+std::max(0.,s)/10,ly=kPi/2+leader/10;
    double separation=-1e9;
    for(double axis:{ey,ey+kPi/2,ly,ly+kPi/2}) {
      const double between=std::abs((lc.x-ec.x)*std::cos(axis)+(lc.y-ec.y)*std::sin(axis));
      const double e_support=2.3*std::abs(std::cos(ey-axis))+.95*std::abs(std::sin(ey-axis));
      const double l_support=2.*std::abs(std::cos(ly-axis))+.95*std::abs(std::sin(ly-axis));
      separation=std::max(separation,between-e_support-l_support);
    }
    min_gap=std::min(min_gap,separation);
   } else if(s+2.3>=0)min_gap=std::min(min_gap,leader-2.-s-2.3);
   if(s>=2.3){cleared=true;break;}
  }
  EXPECT_GE(min_gap,approval_policy>=4?.3:1.5)<<"front contact, scenario="<<scenario<<" slowing="<<slowing_leader<<" headway="<<headway<<" decel="<<actual_decel;
  if(scenario!=1){EXPECT_TRUE(cleared)<<"scenario="<<scenario;EXPECT_GE(commit,0.);// TTC follows its own sequential slot selection; require a bounded
    // departure after the mandatory stop, preserving the same 1.5m minimum.
    EXPECT_LE(commit,5.0); // New arrival-space check adds a bounded hold; do not weaken clearance.
  }
 }
}
}

namespace mission_planner {
TEST(RoundaboutOpportunity, DuplicateFragmentsDoNotSplitGapButDistinctVehiclesStay) {
  for(int mode:{0,1,2}) {
    auto c=testConfig();c.gap_select_enabled=true;c.gs_sequential=true;
    c.gs_spatial_gap_enabled=true;c.gs_spatial_arrival_check=true;c.gs_opportunity_enabled=true;
    c.gs_route_body_projection=true;c.gs_timed_entry_approach=true;
    auto in=approachInput(1);auto parent=circulatingObject(4,-.6,4);
    parent.has_moved=true;parent.confirmed=true;parent.hit_count=12;
    parent.aabb_x=4.6;parent.aabb_y=1.9;parent.length=4.6;parent.width=1.9;
    auto fragment=parent;fragment.id=5;fragment.hit_count=3;fragment.has_moved=false;
    fragment.confirmed=false;fragment.velocity_valid=false;fragment.aabb_x=fragment.aabb_y=1.5;
    fragment.length=fragment.width=1.5;fragment.position.x+=mode==1?3.0:.2;
    if(mode==2){fragment.has_moved=true;fragment.confirmed=true;fragment.hit_count=12;}
    in.objects={parent,fragment};RoundaboutMission mission(c);auto out=mission.update(in);
    if(mode==0){ASSERT_EQ(out.duplicate_track_ids.size(),1u);EXPECT_EQ(out.duplicate_track_ids.front(),5);
      for(const auto& car:out.stream)EXPECT_NE(car.id,5);}
    else EXPECT_TRUE(out.duplicate_track_ids.empty());
  }
}
TEST(RoundaboutOpportunity, FragmentGeometryStillTriggersCollisionProtection) {
  auto c=lockedEntryTestConfig();c.distance_enabled=false;c.gs_sequential=true;c.gs_commit_rear_yield_lock=true;
  c.gs_body_collision_guard=true;c.gs_route_body_projection=true;
  c.gs_spatial_gap_enabled=true;c.gs_spatial_arrival_check=true;c.gs_opportunity_enabled=true;
  c.gs_collision_horizon=.1;c.gs_confirm_seconds=0;
  RoundaboutMission mission(c);auto in=lockedEntryTestInput();
  in.route={{10,-30},{10,0}};for(int i=1;i<=40;++i)in.route.push_back({10*std::cos(i*.025),10*std::sin(i*.025)});
  in.objects.clear();ASSERT_EQ(mission.update(in).state,RoundaboutState::kCommit);
  in.now=1.1;in.ego.position={10,0};in.ego.yaw=kPi/2;in.ego.speed=0;
  auto parent=circulatingObject(4,std::atan2(3.5,10.),2);parent.position={10,3.5};parent.hit_count=12;
  parent.aabb_x=4.6;parent.aabb_y=1.9;parent.length=4.6;parent.width=1.9;
  in.objects={parent};auto out=mission.update(in);
  EXPECT_EQ(out.reason.find("body collision guard"),std::string::npos)<<out.reason;
  auto fragment=parent;fragment.id=5;fragment.position.y=3.3;fragment.aabb_x=fragment.aabb_y=1.9;
  fragment.hit_count=3;fragment.has_moved=false;fragment.confirmed=false;fragment.velocity_valid=false;
  in.now=1.2;in.objects={parent,fragment};out=mission.update(in);
  ASSERT_EQ(out.duplicate_track_ids.size(),1u);
  EXPECT_NE(out.reason.find("body collision guard id5"),std::string::npos)<<out.reason;
}
TEST(RoundaboutOpportunity, SafeRollingSlotCanCommitBeforeOldFourMetreGate) {
 auto c=testConfig();c.gap_select_enabled=true;c.gs_sequential=true;
 c.gs_spatial_gap_enabled=true;c.gs_spatial_arrival_check=true;c.gs_opportunity_enabled=true;
 c.gs_route_body_projection=true;c.gs_confirm_seconds=.1;c.mandatory_stop_seconds=0;
 c.yield_roll_speed=2;c.yield_go_distance=4;c.yield_point={10,-10};
 RoundaboutMission mission(c);auto in=approachInput(1);
 in.ego.position={10,-16};in.ego.speed=3;in.objects.clear();
 in.route={{10,-30},{10,0}};for(int i=1;i<=80;++i)in.route.push_back({10*std::cos(i*.025),10*std::sin(i*.025)});
 auto out=mission.update(in);EXPECT_NE(out.state,RoundaboutState::kCommit);
 in.now=1.11;out=mission.update(in);
 for(const auto& g:out.gaps)if(g.selected)EXPECT_TRUE(out.gap_safe)<<g.failure_reason;
 EXPECT_EQ(out.state,RoundaboutState::kCommit)<<out.reason;
 EXPECT_GT(out.constraint.max_speed,c.yield_roll_speed);
}
}

namespace mission_planner {
TEST(RoundaboutOpportunity, KeepsApproachingFrontUntilWindowExpiresButSkipsPassedRear) {
 auto c=lockedEntryTestConfig();c.distance_enabled=false;c.gs_sequential=true;c.gs_zone_rule_enabled=false;
 c.gs_spatial_gap_enabled=true;c.gs_spatial_arrival_check=true;c.gs_opportunity_enabled=true;
 c.gs_commit_rear_yield_lock=true;c.gs_measured_entry_profile=true;c.gs_use_mpc_prediction=true;
 c.gs_route_body_projection=true;c.gs_timed_entry_approach=true;c.gs_confirm_seconds=.1;
 c.gs_check_horizon=1.5;c.gs_leader_start_distance=8;c.gs_speed_margin=.2;
 c.center={0,0};c.nominal_radius=18.338;c.inner_radius=16;c.outer_radius=20;
 c.yield_point={3.327553,-24.808677};c.conflict_point={4.335901,-20.967604};
 auto input=approachInput(1);input.prediction_speed_limit=7;input.prediction_accel_limit=3.2;
 input.ego.position=c.yield_point;input.ego.speed=0;input.ego.yaw=78.92*kPi/180;input.route.clear();
 for(int i=0;i<=160;++i)input.route.push_back({c.yield_point.x+.1*i*std::cos(input.ego.yaw),c.yield_point.y+.1*i*std::sin(input.ego.yaw)});
 const double origin=std::atan2(c.conflict_point.y,c.conflict_point.x);
 const auto car=[&](int id,double pos) {
   const double angle=origin+pos/c.nominal_radius;auto o=circulatingObject(id,angle,4.7);
   o.position={c.nominal_radius*std::cos(angle),c.nominal_radius*std::sin(angle)};o.hit_count=12;return o;
 };
 input.objects={car(1,2),car(2,-12),car(3,-26)};RoundaboutMission mission(c);
 auto out=mission.update(input);
 for(int i=1;i<=6;++i){input.now=1+.1*i;out=mission.update(input);}
 EXPECT_NE(out.state,RoundaboutState::kCommit)<<out.reason;
 EXPECT_EQ(out.gap_leader_id,1)<<out.reason;
 input.now=1.7;input.objects={car(1,16),car(2,5),car(3,-25)};out=mission.update(input);
 EXPECT_NE(out.gap_leader_id,1)<<out.reason;
}
}

namespace mission_planner {
TEST(RoundaboutOpportunity, AccOnlyDoesNotStopForRearCollisionPredictionAfterApproval) {
 auto c=lockedEntryTestConfig();c.distance_enabled=false;c.gs_sequential=true;c.gs_commit_rear_yield_lock=true;
 c.gs_spatial_gap_enabled=true;c.gs_spatial_arrival_check=true;c.gs_opportunity_enabled=true;
 c.gs_route_body_projection=true;c.gs_timed_entry_approach=true;c.gs_confirm_seconds=0;
 c.gs_body_collision_guard=true;c.gs_commit_acc_only=true;
 RoundaboutMission mission(c);auto in=lockedEntryTestInput();in.objects.clear();
 in.route={{10,-30},{10,0}};for(int i=1;i<=80;++i)in.route.push_back({10*std::cos(i*.025),10*std::sin(i*.025)});
 ASSERT_EQ(mission.update(in).state,RoundaboutState::kCommit);
 in.now=1.1;in.ego.position={10,0};in.ego.yaw=kPi/2;in.ego.speed=2;
 auto rear=circulatingObject(2,-.35,8);rear.hit_count=12;in.objects={rear};
 const auto out=mission.update(in);EXPECT_NE(out.constraint.lead_vehicle_id,2);
 EXPECT_GT(out.constraint.max_speed,0);EXPECT_FALSE(out.constraint.stop_required)<<out.reason;
 EXPECT_EQ(out.reason.find("body collision guard"),std::string::npos);
}
TEST(RoundaboutOpportunity, ApprovalMustBeContinuousAndResetsAfterTrackerFailure) {
 auto c=testConfig();c.gap_select_enabled=true;c.gs_sequential=true;c.require_tracker_stream=true;
 c.gs_spatial_gap_enabled=true;c.gs_spatial_arrival_check=true;c.gs_opportunity_enabled=true;
 c.gs_route_body_projection=true;c.gs_confirm_seconds=.1;c.mandatory_stop_seconds=0;
 c.yield_roll_speed=2;c.yield_point={10,-10};RoundaboutMission mission(c);auto in=approachInput(1);
 in.ego.position={10,-16};in.ego.speed=3;in.objects.clear();
 in.route={{10,-30},{10,0}};for(int i=1;i<=80;++i)in.route.push_back({10*std::cos(i*.025),10*std::sin(i*.025)});
 EXPECT_NE(mission.update(in).state,RoundaboutState::kCommit);
 in.now=1.05;in.tracker_stream_valid=false;in.tracker_stream_fresh=false;
 EXPECT_NE(mission.update(in).state,RoundaboutState::kCommit);
 in.now=1.1;in.tracker_stream_valid=true;in.tracker_stream_fresh=true;
 EXPECT_NE(mission.update(in).state,RoundaboutState::kCommit);
 in.now=1.15;EXPECT_NE(mission.update(in).state,RoundaboutState::kCommit);
 in.now=1.21;EXPECT_EQ(mission.update(in).state,RoundaboutState::kCommit);
}
}

namespace mission_planner {
TEST(RoundaboutSpatialOnly, SlotSpaceApprovesWithoutArrivalFeedbackOrRollingGate) {
 for(bool spatial_only:{false,true}) for(double free_space:{9.0,9.2}) {
  auto c=testConfig();c.gap_select_enabled=true;c.gs_sequential=true;
  c.gs_spatial_gap_enabled=true;c.gs_spatial_arrival_check=true;c.gs_opportunity_enabled=true;
  c.gs_spatial_only_admission=spatial_only;c.gs_commit_acc_only=true;c.gs_body_collision_guard=true;
  c.gs_use_mpc_prediction=true;c.gs_route_body_projection=true;c.gs_timed_entry_approach=true;
  c.gs_confirm_seconds=.1;c.mandatory_stop_seconds=0;c.yield_roll_speed=2;
  c.auto_conflict_point=false;c.yield_point={10,-10};c.gs_spatial_front_gap=2.5;c.gs_spatial_rear_gap=2.;
  auto in=approachInput(1);in.ego.position={10,-20};in.ego.speed=6;
  in.route={{10,-30},{10,0}};
  for(int i=1;i<=80;++i)in.route.push_back({10*std::cos(i*.025),10*std::sin(i*.025)});
  // 4m vehicles: centre spacing minus 4m equals the measured free slot.
  in.objects={circulatingObject(1,0.,4),circulatingObject(2,(-4.-free_space)/10.,8)};
  for(auto& object:in.objects){object.confirmed=true;object.hit_count=12;}
  RoundaboutMission mission(c);auto out=mission.update(in);
  EXPECT_NE(out.state,RoundaboutState::kCommit);
  in.now=1.05;EXPECT_NE(mission.update(in).state,RoundaboutState::kCommit);
  in.now=1.11;out=mission.update(in);
  if(spatial_only && free_space>=9.1)EXPECT_EQ(out.state,RoundaboutState::kCommit)<<out.reason;
  else EXPECT_NE(out.state,RoundaboutState::kCommit)<<out.reason;
 }
}
}

namespace mission_planner {
namespace {
RoundaboutConfig observedEntryConfig() {
 auto c=testConfig();c.gap_select_enabled=true;c.gs_sequential=true;c.gs_zone_rule_enabled=false;
 c.gs_spatial_gap_enabled=true;c.gs_spatial_arrival_check=true;c.gs_opportunity_enabled=true;
 c.gs_observed_entry_enabled=true;c.gs_opportunity_rear_yield=true;c.gs_commit_rear_yield_lock=true;
 c.gs_commit_acc_only=true;c.gs_body_collision_guard=true;c.gs_route_body_projection=true;c.gs_timed_entry_approach=true;
 c.gs_use_mpc_prediction=true;c.gs_confirm_seconds=.1;c.mandatory_stop_seconds=0;c.yield_roll_speed=2;
 c.gs_ego_start_delay=.24;c.gs_ego_accel=3;c.gs_entry_commit_distance=8;c.auto_conflict_point=false;
 c.center={0,0};c.nominal_radius=18.338;c.inner_radius=16;c.outer_radius=20;
 c.yield_point={3.327553,-24.808677};c.conflict_point={4.335901,-20.967604};
 return c;
}
void observedScan(RoundaboutInput& in,double stamp) {
 auto frame=std::make_shared<RoundaboutObservation>();frame->stamp=stamp;
 frame->min_x=frame->min_y=-25;frame->width=100;frame->resolution=.5;
 frame->cells.assign(10000,ObservationCell::Free);in.observation.add(frame);
}
RoundaboutInput observedEntryInput(const RoundaboutConfig& c) {
 auto in=approachInput(1);in.ego.position=c.yield_point;in.ego.speed=0;
 in.ego.yaw=78.92*kPi/180;in.route.clear();
 for(int i=0;i<=180;++i)in.route.push_back({c.yield_point.x+.1*i*std::cos(in.ego.yaw),c.yield_point.y+.1*i*std::sin(in.ego.yaw)});
 in.prediction_speed_limit=7;in.prediction_accel_limit=3.2;in.objects_stamp=1.;return in;
}
TrackedObject observedCar(const RoundaboutConfig& c,int id,double arc,double speed) {
 const double angle=std::atan2(c.conflict_point.y,c.conflict_point.x)+arc/c.nominal_radius;
 auto o=circulatingObject(id,angle,speed);o.position={c.nominal_radius*std::cos(angle),c.nominal_radius*std::sin(angle)};
 o.hit_count=12;o.height=1.5;o.width=1.9;return o;
}
}
TEST(RoundaboutObservedEntry, MissingObservationNeverApprovesEmptyTracks) {
 auto c=observedEntryConfig();RoundaboutMission mission(c);auto in=observedEntryInput(c);
 auto out=mission.update(in);EXPECT_FALSE(out.gap_safe);EXPECT_NE(out.state,RoundaboutState::kCommit);
 in.now=1.3;in.objects_stamp=1.3;out=mission.update(in);
 EXPECT_FALSE(out.gap_safe);EXPECT_NE(out.state,RoundaboutState::kCommit);
}
TEST(RoundaboutObservedEntry, OpenSlotNeedsThreeScansAndDistinctTrackerConfirmation) {
 auto c=observedEntryConfig();RoundaboutMission mission(c);auto in=observedEntryInput(c);
 observedScan(in,.9);observedScan(in,1.);
 EXPECT_FALSE(mission.update(in).gap_safe);
 in.now=1.1;in.objects_stamp=1.1;observedScan(in,1.1);auto out=mission.update(in);
 ASSERT_TRUE(out.gap_safe)<<out.reason<<" entry="<<out.ego_zone_time<<" clear="<<out.ego_clear_time;EXPECT_NE(out.state,RoundaboutState::kCommit);
 in.now=1.25;out=mission.update(in);EXPECT_NE(out.state,RoundaboutState::kCommit); // same cached tracker scan
 in.now=1.26;in.objects_stamp=1.26;observedScan(in,1.26);out=mission.update(in);
 ASSERT_EQ(out.state,RoundaboutState::kCommit)<<out.reason;
 EXPECT_TRUE(out.approval_latched);EXPECT_DOUBLE_EQ(out.approval_time,1.26);
 EXPECT_TRUE(std::isfinite(out.observed_free_length));
}
TEST(RoundaboutObservedEntry, FarEmptyRoundaboutCannotCommitAndPhysicalRadiusIsNotApproval) {
 auto c=observedEntryConfig();RoundaboutMission mission(c);auto in=observedEntryInput(c);
 in.ego.position={c.yield_point.x-32*std::cos(in.ego.yaw),c.yield_point.y-32*std::sin(in.ego.yaw)};
 in.route.insert(in.route.begin(),in.ego.position);in.ego.speed=11.5;
 for(int i=0;i<6;++i){in.now=1.+.1*i;in.objects_stamp=in.now;observedScan(in,in.now);
  const auto out=mission.update(in);EXPECT_NE(out.state,RoundaboutState::kCommit);EXPECT_FALSE(out.approval_latched);}
 RoundaboutMission unapproved(c);in=observedEntryInput(c);in.ego.position={18.338,0};in.ego.yaw=kPi/2;
 in.route={{18.338,-5},{18.338,0},{18,2}};in.observation.frames.clear();
 EXPECT_NE(unapproved.update(in).state,RoundaboutState::kCirculate);
}
TEST(RoundaboutObservedEntry, ChangingVehiclePairRestartsConfirmation) {
 auto c=observedEntryConfig();RoundaboutMission mission(c);auto in=observedEntryInput(c);
 in.objects={observedCar(c,1,16,4),observedCar(c,2,-26,4)};
 for(double t:{.8,.9,1.})observedScan(in,t);
 auto out=mission.update(in);ASSERT_TRUE(out.gap_safe)<<out.reason<<" entry="<<out.ego_zone_time<<" clear="<<out.ego_clear_time;
 in.now=1.05;in.objects_stamp=1.05;observedScan(in,1.05);in.objects[0].id=11;in.objects[1].id=12;
 out=mission.update(in);ASSERT_TRUE(out.gap_safe)<<out.reason<<" entry="<<out.ego_zone_time<<" clear="<<out.ego_clear_time;EXPECT_NE(out.state,RoundaboutState::kCommit);
 in.now=1.11;in.objects_stamp=1.11;observedScan(in,1.11);EXPECT_NE(mission.update(in).state,RoundaboutState::kCommit);
 in.now=1.16;in.objects_stamp=1.16;observedScan(in,1.16);out=mission.update(in);
 EXPECT_EQ(out.state,RoundaboutState::kCommit)<<out.reason;EXPECT_EQ(out.gap_leader_id,11);
}
TEST(RoundaboutObservedEntry, ApprovalCannotBeCancelledDuringResponseDelayOrByNewRear) {
 auto c=observedEntryConfig();RoundaboutMission mission(c);auto in=observedEntryInput(c);
 for(double t:{.8,.9,1.})observedScan(in,t);
 EXPECT_NE(mission.update(in).state,RoundaboutState::kCommit);
 in.now=1.11;in.objects_stamp=1.11;observedScan(in,1.11);
 ASSERT_EQ(mission.update(in).state,RoundaboutState::kCommit);
 in.observation.frames.clear();in.objects={observedCar(c,44,-1,8)};
 in.now=1.12;in.objects_stamp=1.12;auto out=mission.update(in);
 EXPECT_EQ(out.state,RoundaboutState::kCommit)<<out.reason;EXPECT_TRUE(out.approval_latched);
 EXPECT_FALSE(out.constraint.stop_required)<<out.reason;EXPECT_EQ(out.gap_follower_id,-1);
 in.now=1.2;out=mission.update(in);EXPECT_EQ(out.state,RoundaboutState::kCommit);
 EXPECT_EQ(out.reason.find("body collision guard"),std::string::npos);
}
TEST(RoundaboutObservedEntry, StaleObservationOrMpcFeedbackBlocksBeforeApproval) {
 auto c=observedEntryConfig();RoundaboutMission mission(c);auto in=observedEntryInput(c);
 for(double t:{.8,.9,1.})observedScan(in,t);
 in.now=1.7;in.objects_stamp=1.7;EXPECT_FALSE(mission.update(in).gap_safe);
 in=observedEntryInput(c);for(double t:{.8,.9,1.})observedScan(in,t);
 in.prediction_speed_limit=std::numeric_limits<double>::quiet_NaN();
 RoundaboutMission no_feedback(c);EXPECT_FALSE(no_feedback.update(in).gap_safe);
}
}

namespace mission_planner {
TEST(RoundaboutObservedEntry, FrontAccStillTracksAStoppedLeaderAfterApproval) {
 auto c=observedEntryConfig();c.gs_measured_entry_profile=true;RoundaboutMission mission(c);auto in=observedEntryInput(c);
 for(double t:{.8,.9,1.})observedScan(in,t);
 mission.update(in);in.now=1.11;in.objects_stamp=1.11;observedScan(in,1.11);
 ASSERT_EQ(mission.update(in).state,RoundaboutState::kCommit);
 // Move to the circulating lane and put a proven stopped car directly ahead.
 const double angle=std::atan2(c.conflict_point.y,c.conflict_point.x);
 in.now=1.2;in.ego.position={c.nominal_radius*std::cos(angle),c.nominal_radius*std::sin(angle)};
 in.ego.yaw=angle+kPi/2;in.ego.speed=2;in.objects={observedCar(c,99,4.,0)};
 in.objects_stamp=in.now;in.observation.frames.clear();const auto out=mission.update(in);
 ASSERT_TRUE(out.approval_latched);EXPECT_LE(out.constraint.max_speed,.1)<<out.reason;
 EXPECT_FALSE(out.constraint.lead_vehicle_active); // projected ACC already owns the cap, avoid a second scalar ACC
 EXPECT_EQ(out.constraint.lead_vehicle_id,99);EXPECT_NE(out.state,RoundaboutState::kYield);
 EXPECT_EQ(out.reason.find("gap unsafe"),std::string::npos);
}
TEST(RoundaboutObservedEntry, NewTrackerFrameWithoutNewObservationDoesNotCommit) {
 auto c=observedEntryConfig();RoundaboutMission mission(c);auto in=observedEntryInput(c);
 for(double t:{.8,.9,1.})observedScan(in,t);
 ASSERT_TRUE(mission.update(in).gap_safe);
 in.now=1.15;in.objects_stamp=1.15;EXPECT_NE(mission.update(in).state,RoundaboutState::kCommit);
 observedScan(in,1.16);in.now=1.16;in.objects_stamp=1.16;
 EXPECT_EQ(mission.update(in).state,RoundaboutState::kCommit);
}
}

namespace mission_planner {
TEST(RoundaboutObservedEntry, LocalizationRecoveryKeepsDepartureApprovalWithoutRecheckingGap) {
 auto c=observedEntryConfig();c.require_localization=true;RoundaboutMission mission(c);auto in=observedEntryInput(c);
 for(double t:{.8,.9,1.})observedScan(in,t);
 mission.update(in);in.now=1.11;in.objects_stamp=1.11;observedScan(in,1.11);
 ASSERT_EQ(mission.update(in).state,RoundaboutState::kCommit);
 in.now=1.12;in.localization_valid=false;in.observation.frames.clear();
 EXPECT_EQ(mission.update(in).state,RoundaboutState::kFailsafe);
 in.now=1.13;in.localization_valid=true;in.objects={observedCar(c,44,-1,8)};
 const auto out=mission.update(in);EXPECT_EQ(out.state,RoundaboutState::kCommit)<<out.reason;
 EXPECT_TRUE(out.approval_latched);EXPECT_FALSE(out.constraint.stop_required)<<out.reason;
 EXPECT_DOUBLE_EQ(out.approval_time,1.11);
}
TEST(RoundaboutObservedEntry, UnapprovedRecoveryInsideRingDoesNotBypassAdmission) {
 auto c=observedEntryConfig();c.require_localization=true;RoundaboutMission mission(c);auto in=observedEntryInput(c);
 in.localization_valid=false;EXPECT_EQ(mission.update(in).state,RoundaboutState::kFailsafe);
 in.now=1.1;in.localization_valid=true;in.ego.position={c.nominal_radius,0};in.ego.yaw=kPi/2;
 in.route={{c.nominal_radius,-5},{c.nominal_radius,0},{c.nominal_radius-.2,2}};
 const auto out=mission.update(in);EXPECT_NE(out.state,RoundaboutState::kCirculate);
 EXPECT_FALSE(out.approval_latched);EXPECT_FALSE(out.gap_safe);
}
}

namespace mission_planner {
namespace {
RoundaboutConfig preparationConfig() {
 auto c=observedEntryConfig();c.gs_preparation_enabled=true;c.gs_spatial_front_gap=2.;c.gs_spatial_rear_gap=1.5;
 c.gs_prediction_follow_standstill_gap=2.;c.gs_prediction_follow_time_gap=.35;return c;
}
RoundaboutInput preparationInput(const RoundaboutConfig& c) {
 auto in=observedEntryInput(c);
 in.ego.position.x-=3*std::cos(in.ego.yaw);in.ego.position.y-=3*std::sin(in.ego.yaw);
 in.route.insert(in.route.begin(),in.ego.position);in.ego.speed=.5;
 in.objects={observedCar(c,1,-14,4),observedCar(c,2,-40,4)};
 for(auto& car:in.objects)car.confirmed=true;
 return in;
}
}
void preparationScan(RoundaboutInput& in,double stamp) {
 auto frame=std::make_shared<RoundaboutObservation>();frame->stamp=stamp;
 frame->resolution=.5;frame->min_x=frame->min_y=-40;frame->width=160;
 frame->cells.assign(25600,ObservationCell::Free);in.observation.add(frame);
}
TEST(RoundaboutPreparation, WaitingTargetLeavesThreeMetresWithoutMandatoryStop) {
 auto c=preparationConfig();RoundaboutMission mission(c);auto in=preparationInput(c);
 in.ego.position.x-=3*std::cos(in.ego.yaw);in.ego.position.y-=3*std::sin(in.ego.yaw);
 in.route.insert(in.route.begin(),in.ego.position);in.objects.clear();
 const auto out=mission.update(in);EXPECT_FALSE(out.approval_latched);
 EXPECT_NEAR(out.constraint.stop_distance,3.,.01);EXPECT_TRUE(out.constraint.stop_required);
 EXPECT_EQ(out.entry_phase,"waiting");
}
TEST(RoundaboutPreparation, PreparationNeedsDistinctScansAndStopsWhenObservationDisappears) {
 auto c=preparationConfig();RoundaboutMission mission(c);auto in=preparationInput(c);
 for(double t:{.8,.9,1.})preparationScan(in,t);
 auto out=mission.update(in);
 ASSERT_GT(out.acceleration_start_delay,0.)<<out.reason;
 EXPECT_NE(out.entry_phase,"preparing");EXPECT_FALSE(out.approval_latched);
 in.now=1.12;in.objects_stamp=1.12;preparationScan(in,1.12);out=mission.update(in);
 ASSERT_EQ(out.entry_phase,"preparing")<<out.reason;
 EXPECT_FALSE(out.approval_latched);EXPECT_LE(out.constraint.max_speed,1.5);
 EXPECT_GT(out.constraint.stop_distance,0.);
 in.now=1.13;in.observation.frames.clear();out=mission.update(in);
 EXPECT_EQ(out.entry_phase,"waiting");EXPECT_FALSE(out.approval_latched);
}
TEST(RoundaboutPreparation, ApprovalRemainsFixedWithNewRearAndMissingObservations) {
 auto c=preparationConfig();RoundaboutMission mission(c);auto in=observedEntryInput(c);
 for(double t:{.8,.9,1.})preparationScan(in,t);
 mission.update(in);in.now=1.11;in.objects_stamp=1.11;preparationScan(in,1.11);
 ASSERT_EQ(mission.update(in).state,RoundaboutState::kCommit);
 in.now=1.12;in.objects={observedCar(c,99,-1,8)};in.objects_stamp=in.now;in.observation.frames.clear();
 const auto out=mission.update(in);EXPECT_TRUE(out.approval_latched);EXPECT_EQ(out.state,RoundaboutState::kCommit);
 EXPECT_FALSE(out.constraint.stop_required);EXPECT_EQ(out.entry_phase,"committed");
}
}

namespace mission_planner {
TEST(RoundaboutPreparation, ChangingPairRestartsPreparationConfirmation) {
 auto c=preparationConfig();RoundaboutMission mission(c);auto in=preparationInput(c);
 for(double t:{.8,.9,1.})preparationScan(in,t);
 mission.update(in);
 in.now=1.05;in.objects_stamp=in.now;preparationScan(in,in.now);in.objects[0].id=11;in.objects[1].id=12;
 EXPECT_NE(mission.update(in).entry_phase,"preparing");
 in.now=1.12;in.objects_stamp=in.now;preparationScan(in,in.now);
 EXPECT_NE(mission.update(in).entry_phase,"preparing");
 in.now=1.17;in.objects_stamp=in.now;preparationScan(in,in.now);
 EXPECT_EQ(mission.update(in).entry_phase,"preparing");
}
TEST(RoundaboutPreparation, UnconfirmedApproachingFrontCannotOwnApproval) {
 auto c=preparationConfig();RoundaboutMission mission(c);auto in=preparationInput(c);
 in.objects[0].confirmed=false;in.objects[0].has_moved=false;in.objects[0].velocity_valid=false;in.objects[0].hit_count=3;
 for(double t:{.8,.9,1.})preparationScan(in,t);
 auto out=mission.update(in);EXPECT_FALSE(out.approval_latched);
 in.now=1.12;in.objects_stamp=in.now;preparationScan(in,in.now);out=mission.update(in);
 EXPECT_FALSE(out.approval_latched);EXPECT_NE(out.entry_phase,"preparing");
}
}

namespace mission_planner {
TEST(RoundaboutPreparation, LocalizationLossRestartsPreparationConfirmationBeforeApproval) {
 auto c=preparationConfig();c.require_localization=true;RoundaboutMission mission(c);auto in=preparationInput(c);
 for(double t:{.8,.9,1.})preparationScan(in,t);
 mission.update(in);in.now=1.12;in.objects_stamp=in.now;preparationScan(in,in.now);
 ASSERT_EQ(mission.update(in).entry_phase,"preparing");
 in.now=1.13;in.localization_valid=false;
 EXPECT_EQ(mission.update(in).state,RoundaboutState::kFailsafe);
 in.now=1.14;in.localization_valid=true;in.objects_stamp=in.now;preparationScan(in,in.now);
 EXPECT_NE(mission.update(in).entry_phase,"preparing");
 in.now=1.25;in.objects_stamp=in.now;preparationScan(in,in.now);
 EXPECT_EQ(mission.update(in).entry_phase,"preparing");
}
}
