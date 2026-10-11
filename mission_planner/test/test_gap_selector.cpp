#include <gtest/gtest.h>
#include <cmath>

#include "mission_planner/gap_selector.hpp"

namespace mission_planner {
namespace {

EgoEntryModel standingEgo() {
  EgoEntryModel ego;
  ego.distance_to_merge = 7.3;  // K-City 양보선 -> 합류점
  ego.speed = 0.0;
  ego.accel = 2.2;
  ego.max_speed = 8.5;
  ego.start_delay = 0.2;
  ego.length = 4.6;
  return ego;
}

GapSelectorParams ringParams() {
  GapSelectorParams p;
  p.ring_length = 2.0 * 3.14159265358979323846 * 17.955;
  return p;
}

StreamVehicle vehicle(int id, double pos, double speed) {
  StreamVehicle v;
  v.id = id;
  v.pos = pos;
  v.speed = speed;
  v.length = 4.5;
  return v;
}

const GapCandidate* earliest(const std::vector<GapCandidate>& gaps) {
  const GapCandidate* best = nullptr;
  for (const GapCandidate& g : gaps)
    if (g.feasible && (!best || g.depart_delay < best->depart_delay)) best = &g;
  return best;
}

TEST(GapSelector, EmptyRingGoesNow) {
  const auto gaps = GapSelector::evaluate({}, standingEgo(), ringParams());
  ASSERT_EQ(gaps.size(), 1u);
  EXPECT_TRUE(gaps[0].feasible);
  EXPECT_DOUBLE_EQ(gaps[0].depart_delay, 0.0);
}

TEST(GapSelector, VehicleJustPassedAllowsImmediateGo) {
  // 합류점을 막 지난 차량(앞차)만 있으면 바로 출발.
  const auto gaps = GapSelector::evaluate({vehicle(6, 3.2, 6.2)}, standingEgo(),
                                          ringParams());
  const GapCandidate* best = earliest(gaps);
  ASSERT_NE(best, nullptr);
  EXPECT_DOUBLE_EQ(best->depart_delay, 0.0);
  EXPECT_EQ(best->leader_id, 6);
}

TEST(GapSelector, CloseFastFollowerMakesUsWaitBehindIt) {
  // 12 m 상류에서 8 m/s 로 오는 차량: 지금은 앞에 못 끼고, 지나간 뒤 출발.
  const auto gaps = GapSelector::evaluate({vehicle(7, -12.0, 8.0)},
                                          standingEgo(), ringParams());
  const GapCandidate* best = earliest(gaps);
  ASSERT_NE(best, nullptr);
  EXPECT_EQ(best->leader_id, 7);
  EXPECT_GE(best->depart_delay, 0.4);
  EXPECT_LT(best->depart_delay, 4.0);
}

TEST(GapSelector, FarFollowerLeavesGapNow) {
  const auto gaps = GapSelector::evaluate({vehicle(8, -55.0, 6.0)},
                                          standingEgo(), ringParams());
  const GapCandidate* best = earliest(gaps);
  ASSERT_NE(best, nullptr);
  EXPECT_EQ(best->follower_id, 8);
  EXPECT_DOUBLE_EQ(best->depart_delay, 0.0);
}

TEST(GapSelector, PicksGapBetweenPlatoonVehicles) {
  // 차량 A 가 막 지나가는 중, B 는 멀리 뒤: A/B 사이 틈에 바로 들어간다.
  const auto gaps = GapSelector::evaluate(
      {vehicle(1, 1.0, 6.0), vehicle(2, -48.0, 6.0)}, standingEgo(),
      ringParams());
  const GapCandidate* best = earliest(gaps);
  ASSERT_NE(best, nullptr);
  EXPECT_EQ(best->leader_id, 1);
  EXPECT_EQ(best->follower_id, 2);
  EXPECT_DOUBLE_EQ(best->depart_delay, 0.0);
}

TEST(GapSelector, StoppedLeaderAtConflictBlocks) {
  // 합류점 바로 앞에 선 차량: 앞차 간격이 영영 안 난다.
  const auto gaps = GapSelector::evaluate({vehicle(3, 4.0, 0.0)},
                                          standingEgo(), ringParams());
  EXPECT_EQ(earliest(gaps), nullptr);
}

TEST(GapSelector, ApproachingLeaderOnlyLimitsFrontSide) {
  // 상류에서 다가오는 앞차(14 m, 5 m/s)를 먼저 보내고 멀리 있는 뒤차(45 m) 앞으로
  // 들어가는 틈: 지금 출발하면 앞쪽만 부족하고 뒤쪽은 만족한다.
  GapSelectorParams p = ringParams();  // K-City 설정값
  p.front_headway = 0.3;
  p.rear_headway = 0.5;
  p.standstill_gap = 0.8;
  p.check_horizon = 2.0;
  p.speed_margin = 0.2;
  const auto gaps = GapSelector::evaluate(
      {vehicle(1, -14.0, 5.0), vehicle(2, -45.0, 5.0)}, standingEgo(), p);
  const GapCandidate* g = nullptr;
  for (const GapCandidate& c : gaps)
    if (c.leader_id == 1 && c.follower_id == 2) g = &c;
  ASSERT_NE(g, nullptr);
  ASSERT_TRUE(g->feasible);
  EXPECT_GT(g->depart_delay, 0.0);
  EXPECT_GT(g->front_shortfall_now, 0.0);
  EXPECT_DOUBLE_EQ(g->rear_shortfall_now, 0.0);
}

GapSelectorParams kcityParams() {
  GapSelectorParams p = ringParams();
  p.front_headway = 0.3;
  p.rear_headway = 0.5;
  p.standstill_gap = 0.8;
  p.check_horizon = 2.0;
  p.speed_margin = 0.2;
  return p;
}

TEST(GapSelectorSequential, FollowsLeaderWhenFollowerIsFar) {
  // 앞차(10 m 상류, 6 m/s) 뒤, 뒤차(45 m) 앞 틈: 앞차가 빠지는 시각에 맞춰 출발.
  SequentialParams sp;
  const auto r = GapSelector::evaluateSequential(
      {vehicle(1, -10.0, 6.0), vehicle(2, -45.0, 6.0)}, standingEgo(), kcityParams(),
      sp, 1);
  ASSERT_TRUE(r.feasible);
  EXPECT_EQ(r.gap.leader_id, 1);
  EXPECT_EQ(r.gap.follower_id, 2);
  EXPECT_TRUE(r.skipped.empty());
  // 앞차 뒤끝이 합류점을 0.8 m 지나는 시각 - 자차 도착 시간 >= 0
  EXPECT_GE(r.gap.depart_delay, 0.0);
}

TEST(GapSelectorSequential, SkipsGapWithCloseFollowerAndTakesNextOne) {
  // 앞차(10 m) 바로 뒤 뒤차(20 m) - 뒤차 TTC 를 못 지킨다 -> 패스하고
  // 뒤차 뒤(멀리 60 m 까지 빈) 공간을 목표로.
  SequentialParams sp;
  const auto r = GapSelector::evaluateSequential(
      {vehicle(1, -10.0, 6.0), vehicle(2, -20.0, 6.0), vehicle(3, -60.0, 6.0)},
      standingEgo(), kcityParams(), sp, 1);
  ASSERT_TRUE(r.feasible);
  ASSERT_EQ(r.skipped.size(), 1u);
  EXPECT_EQ(r.skipped[0], 2);
  EXPECT_EQ(r.gap.leader_id, 2);
  EXPECT_EQ(r.gap.follower_id, 3);
  EXPECT_EQ(r.target_leader, 2);
}

TEST(GapSelectorSequential, DoesNotGoBackToSkippedGap) {
  // 목표가 이미 L2 로 넘어갔으면, 앞 틈(L1/F2)이 좋아 보여도 되돌아가지 않는다.
  SequentialParams sp;
  const auto r = GapSelector::evaluateSequential(
      {vehicle(1, -5.0, 6.0), vehicle(2, -40.0, 6.0), vehicle(3, -80.0, 6.0)},
      standingEgo(), kcityParams(), sp, 2);
  ASSERT_TRUE(r.feasible);
  EXPECT_EQ(r.gap.leader_id, 2);
}

TEST(GapSelectorSequential, WaitsUntilLeaderIsNearConflict) {
  // A restrictive gate must add waiting beyond front clearance alone.
  SequentialParams sp;
  sp.leader_start_distance = 6.0;
  const auto r = GapSelector::evaluateSequential(
      {vehicle(1, -15.0, 6.0), vehicle(2, -80.0, 6.0)}, standingEgo(), kcityParams(),
      sp, 1);
  ASSERT_TRUE(r.feasible);
  EXPECT_GE(r.gap.depart_delay, (15.0 - 6.0) / 5.8);
  SequentialParams off;
  const auto r2 = GapSelector::evaluateSequential(
      {vehicle(1, -15.0, 6.0), vehicle(2, -80.0, 6.0)}, standingEgo(), kcityParams(),
      off, 1);
  EXPECT_LT(r2.gap.depart_delay, r.gap.depart_delay);
}

TEST(GapSelectorSequential, AdaptiveStartMatchesArrivalButStaysBounded) {
  SequentialParams fixed;
  fixed.leader_start_distance = 6.0;
  SequentialParams adaptive = fixed;
  adaptive.adaptive_leader_start = true;
  adaptive.leader_start_max_distance = 12.0;
  adaptive.departure_confirm_time = 0.1;
  const auto cars = std::vector<StreamVehicle>{vehicle(1, -10.9, 6.0),
                                               vehicle(2, -80.0, 6.0)};
  const auto old = GapSelector::evaluateSequential(cars, standingEgo(), kcityParams(), fixed, 1);
  const auto now = GapSelector::evaluateSequential(cars, standingEgo(), kcityParams(), adaptive, 1);
  ASSERT_TRUE(old.feasible);
  ASSERT_TRUE(now.feasible);
  EXPECT_LT(now.gap.depart_delay, old.gap.depart_delay);
  EXPECT_GT(old.gap.depart_delay, 0.3);
  EXPECT_GT(now.gap.depart_delay, 0.0); // Adaptive gate cannot bypass front clearance.
  const auto far = GapSelector::evaluateSequential(
      {vehicle(1, -30.0, 10.0), vehicle(2, -90.0, 10.0)},
      standingEgo(), kcityParams(), adaptive, 1);
  ASSERT_TRUE(far.feasible);
  EXPECT_GE(far.gap.depart_delay, (30.0 - 12.0) / 9.8);
}

TEST(GapSelectorSequential, AdaptiveStartRetainsRearRejectionAndDiagnostics) {
  SequentialParams sp;
  sp.leader_start_distance = 9.0;
  sp.adaptive_leader_start = true;
  sp.max_skip = 0;
  const auto r = GapSelector::evaluateSequential(
      {vehicle(1, -10.0, 6.0), vehicle(2, -15.0, 9.0)},
      standingEgo(), kcityParams(), sp, 1);
  EXPECT_FALSE(r.feasible);
  EXPECT_FALSE(r.gap.feasible);
  EXPECT_EQ(r.gap.leader_id, 1);
  EXPECT_EQ(r.gap.follower_id, 2);
  EXPECT_LT(r.rear_margin, 0.0);
}

TEST(GapSelectorSequential, DoesNotReserveDistantSlotBeyondWaitHorizon) {
  auto p = kcityParams();
  p.max_wait = 2.0;
  SequentialParams seq;
  seq.leader_start_distance = 9.0;
  const auto result = GapSelector::evaluateSequential(
      {vehicle(1, -90.0, 6.0)}, standingEgo(), p, seq, 1);
  EXPECT_FALSE(result.feasible);
  EXPECT_EQ(result.target_leader, 1);
  EXPECT_GT(result.gap.depart_delay, p.max_wait);
  EXPECT_EQ(result.gap.limit_side, "wait_horizon");
}

TEST(GapSelectorSequential, LargerRetainedVehicleRequiresLaterFrontClearance) {
  auto car = vehicle(1, -10.0, 6.0);
  auto truck = car; truck.length = 8.7;
  SequentialParams seq;
  auto a = GapSelector::evaluateSequential({car}, standingEgo(), kcityParams(), seq, 1);
  auto b = GapSelector::evaluateSequential({truck}, standingEgo(), kcityParams(), seq, 1);
  ASSERT_TRUE(a.feasible); ASSERT_TRUE(b.feasible);
  EXPECT_GT(b.gap.depart_delay, a.gap.depart_delay);
}

TEST(GapSelectorSequential, MovingCommitDoesNotReapplyDepartureGate) {
  auto ego = standingEgo();
  ego.speed = 2.4;
  ego.distance_to_merge = 5.1;
  ego.departure_in_progress = true;
  SequentialParams seq;
  seq.leader_start_distance = 1.0; // Intentionally restrictive departure gate.
  auto p = kcityParams();
  const auto result = GapSelector::evaluateSequential(
      {vehicle(1, -5.0, 5.56), vehicle(2, -60.0, 5.56)}, ego, p, seq, 1);
  ASSERT_TRUE(result.feasible);
  const double t_clear = (p.standstill_gap + 2.25 + 5.0) / (5.56 - p.speed_margin);
  EXPECT_GE(result.gap.depart_delay,
            std::max(0.0, t_clear - GapSelector::timeToZone(ego, p)));
  EXPECT_GE(result.gap.merge_time, t_clear);
  // Occupied front zone still requires waiting, even after departure.
  EXPECT_GT(result.gap.depart_delay, 0.0);
}

TEST(GapSelectorSequential, MovingCommitStillRejectsCloseFastRear) {
  auto ego = standingEgo();
  ego.speed = 2.4;
  ego.distance_to_merge = 5.1;
  ego.departure_in_progress = true;
  SequentialParams seq;
  seq.max_skip = 0;
  const auto result = GapSelector::evaluateSequential(
      {vehicle(1, -5.0, 5.56), vehicle(2, -13.0, 8.0)},
      ego, kcityParams(), seq, 1);
  EXPECT_FALSE(result.feasible);
  EXPECT_LT(result.rear_margin, 0.0);
}

TEST(GapSelector, CommitCreepUsesCurrentSpeedWithoutReactionDelay) {
  auto ego = standingEgo();
  ego.speed = 0.3;
  ego.departure_in_progress = true;
  double s, v;
  GapSelector::egoProfile(ego, 0.1, &s, &v);
  EXPECT_NEAR(s, 0.3 * 0.1 + 0.5 * ego.accel * 0.01, 1e-9);
  EXPECT_NEAR(v, 0.3 + ego.accel * 0.1, 1e-9);
}

TEST(GapSelectorSequential, ApprovalAndCommitUseSameCappedStopProfile) {
  auto ego = standingEgo();
  ego.distance_to_merge = 9.7;
  ego.start_delay = 0.0; // Isolate state change from elapsed reaction time.
  SequentialParams seq;
  seq.max_skip = 0;
  seq.hold_distance = 5.9;
  const std::vector<StreamVehicle> cars{vehicle(1, -5.0, 3.0),
                                      vehicle(2, -70.0, 6.0)};
  const auto before = GapSelector::evaluateSequential(cars, ego, kcityParams(), seq, 1);
  ego.departure_in_progress = true;
  seq.lock_gap = true;
  seq.target_follower = 2;
  const auto after = GapSelector::evaluateSequential(cars, ego, kcityParams(), seq, 1);
  ASSERT_TRUE(before.feasible);
  ASSERT_TRUE(before.departure_ready);
  ASSERT_TRUE(after.feasible);
  EXPECT_NEAR(before.gap.merge_time, after.gap.merge_time, 0.021);
  EXPECT_NEAR(before.rear_margin, after.rear_margin, 0.15);
  EXPECT_GT(before.gap.merge_time, GapSelector::timeToZone(ego, kcityParams()));
}

TEST(GapSelectorSequential, WaitingSlotIsNotDepartureReady) {
  SequentialParams seq;
  seq.max_skip = 0;
  seq.leader_start_distance = 6.5;
  const auto result = GapSelector::evaluateSequential(
      {vehicle(1, -7.5, 6.0), vehicle(2, -80.0, 6.0)},
      standingEgo(), kcityParams(), seq, 1);
  ASSERT_TRUE(result.feasible);
  EXPECT_GT(result.gap.depart_delay, 0.0);
  EXPECT_LT(result.gap.depart_delay, 0.3);
  EXPECT_FALSE(result.departure_ready);
}

TEST(GapSelectorSequential, LockedFollowerStaysSelectedAfterPassingMerge) {
  auto ego = standingEgo();
  ego.departure_in_progress = true;
  SequentialParams seq;
  seq.lock_gap = true;
  seq.target_follower = 2;
  const auto result = GapSelector::evaluateSequential(
      {vehicle(1, 15.0, 6.0), vehicle(2, 1.0, 6.0), vehicle(3, -60.0, 6.0)},
      ego, kcityParams(), seq, 1);
  EXPECT_FALSE(result.feasible);
  EXPECT_EQ(result.gap.leader_id, 1);
  EXPECT_EQ(result.gap.follower_id, 2);
  EXPECT_TRUE(result.skipped.empty());
}

TEST(GapSelectorSequential, MissingCommittedVehicleDoesNotBecomeOpenGap) {
  SequentialParams seq;
  seq.lock_gap = true;
  seq.target_follower = 2;
  for (const auto& cars : std::vector<std::vector<StreamVehicle>>{
           {vehicle(1, 5.0, 6.0)}, {vehicle(2, -60.0, 6.0)}, {}}) {
    const auto result = GapSelector::evaluateSequential(cars, standingEgo(), kcityParams(), seq, 1);
    EXPECT_FALSE(result.feasible);
    EXPECT_EQ(result.gap.leader_id, 1);
    EXPECT_EQ(result.gap.follower_id, 2);
    EXPECT_EQ(result.gap.limit_side, "target_missing");
  }
}

TEST(GapSelectorSequential, NewVehicleInsideCommittedSlotInvalidatesIt) {
  SequentialParams seq;
  seq.lock_gap = true;
  seq.target_follower = 2;
  const auto result = GapSelector::evaluateSequential(
      {vehicle(1, 5.0, 6.0), vehicle(2, -60.0, 6.0), vehicle(3, -20.0, 6.0)},
      standingEgo(), kcityParams(), seq, 1);
  EXPECT_FALSE(result.feasible);
  EXPECT_EQ(result.gap.limit_side, "gap_intrusion");
  EXPECT_EQ(result.gap.follower_id, 2);
  EXPECT_TRUE(result.skipped.empty());
}

TEST(GapSelectorSequential, SlowingLeaderDelaysDepartureAndCannotBeDrivenThrough) {
  auto lead = vehicle(1, -9.0, 6.0);
  SequentialParams seq;
  seq.max_skip = 0;
  const auto normal = GapSelector::evaluateSequential(
      {lead}, standingEgo(), kcityParams(), seq, 1);
  lead.speed_lo = 2.0;
  const auto slow = GapSelector::evaluateSequential(
      {lead}, standingEgo(), kcityParams(), seq, 1);
  ASSERT_TRUE(normal.feasible);
  ASSERT_TRUE(slow.feasible);
  EXPECT_GT(slow.gap.depart_delay, normal.gap.depart_delay);
  EXPECT_GE(slow.gap.merge_time, (9.0 + 2.25 + 0.8) / 1.8);
  lead.speed_lo = 0.0;
  const auto stopped = GapSelector::evaluateSequential(
      {lead}, standingEgo(), kcityParams(), seq, 1);
  EXPECT_FALSE(stopped.feasible);
  EXPECT_EQ(stopped.target_leader, 1);
}

TEST(GapSelectorSequential, FastMovingEgoCannotTeleportToFrontHoldPoint) {
  auto ego = standingEgo();
  ego.distance_to_merge = 4.0;
  ego.speed = 6.0;
  ego.departure_in_progress = true;
  SequentialParams seq;
  seq.lock_gap = true;
  seq.target_follower = -1;
  const auto result = GapSelector::evaluateSequential(
      {vehicle(1, -5.0, 3.0)}, ego, kcityParams(), seq, 1);
  EXPECT_FALSE(result.feasible);
}

TEST(GapSelectorSequential, AccProfileDoesNotAlsoHardCapEgoToLeaderSpeed) {
  auto ego = standingEgo();
  ego.distance_to_merge = 9.7;
  auto leader = vehicle(1, 8.0, 3.0);
  leader.physical_speed = 3.0;
  const std::vector<StreamVehicle> cars{leader, vehicle(2, -100.0, 6.0)};
  SequentialParams seq;
  seq.max_skip = 0;
  const auto fixed_cap = GapSelector::evaluateSequential(cars, ego, kcityParams(), seq, 1);
  seq.follow_time_gap = 1.5;
  seq.follow_standstill_gap = 4.0;
  const auto actual_acc = GapSelector::evaluateSequential(cars, ego, kcityParams(), seq, 1);
  ASSERT_TRUE(fixed_cap.departure_ready);
  ASSERT_TRUE(actual_acc.departure_ready);
  EXPECT_LT(actual_acc.gap.merge_time, fixed_cap.gap.merge_time - 0.3);
  EXPECT_GE(actual_acc.rear_ttc, seq.ttc_rear);
}

TEST(GapSelectorSequential, CommandAdapterAccCapDelaysArrivalAndReducesRearMargin) {
  auto ego = standingEgo();
  ego.distance_to_merge = 9.7;
  auto leader = vehicle(1, -5.0, 3.0);
  leader.physical_speed = 3.0;
  const std::vector<StreamVehicle> cars{leader, vehicle(2, -100.0, 6.0)};
  SequentialParams seq;
  seq.max_skip = 0;
  const auto unrestricted = GapSelector::evaluateSequential(cars, ego, kcityParams(), seq, 1);
  seq.follow_time_gap = 1.5;
  seq.follow_standstill_gap = 4.0;
  const auto following = GapSelector::evaluateSequential(cars, ego, kcityParams(), seq, 1);
  ASSERT_TRUE(unrestricted.feasible);
  ASSERT_TRUE(following.feasible);
  EXPECT_GT(following.gap.merge_time, unrestricted.gap.merge_time + 0.2);
  EXPECT_LT(following.rear_margin, unrestricted.rear_margin);
}

}  // namespace
}  // namespace mission_planner

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

namespace mission_planner {
TEST(GapSelectorMeasured, ArrivalUsesMeasuredAccelerationAndMpcSpeedCap) {
  EgoEntryModel ego; ego.distance_to_merge=10; ego.length=4; ego.start_delay=.2;
  ego.accel=3; ego.max_speed=7;
  GapSelectorParams p; p.entry_zone=1;
  const double constant=GapSelector::timeToZone(ego,p);
  ego.measured_profile=true; ego.profile_times={0,.3,.8,1.3,1.8,2.3,2.8,3.3};
  ego.profile_speeds={0,.95,2.5,3.7,4.6,5.2,5.55,5.8};
  const double measured=GapSelector::timeToZone(ego,p);
  EXPECT_GT(measured,constant);
  double s,v;GapSelector::egoProfile(ego,measured,&s,&v);
  EXPECT_NEAR(s,7,.01);
  ego.max_speed=2;EXPECT_GT(GapSelector::timeToZone(ego,p),measured);
  GapSelector::egoProfile(ego,10,&s,&v);EXPECT_LE(v,2);
  ego.speed=3;ego.departure_in_progress=true;
  EXPECT_LT(GapSelector::timeToZone(ego,p),measured);
}
TEST(GapSelectorMeasured, InvalidProfileFallsBackWithoutNan) {
  EgoEntryModel ego;ego.distance_to_merge=10;ego.measured_profile=true;
  ego.profile_times={0,0};ego.profile_speeds={0,3};
  EXPECT_TRUE(std::isfinite(GapSelector::timeToZone(ego,GapSelectorParams{})));
}
}

namespace mission_planner {
TEST(GapSelectorMeasured, CooperativeRearRequiresPostEntryReactionAndBrakingCapability) {
 auto ego=standingEgo();ego.accel=2;ego.max_speed=5;
 auto p=ringParams();p.standstill_gap=.8;p.front_headway=.3;p.rear_headway=.2;p.check_horizon=2;p.speed_margin=.2;
 SequentialParams seq;seq.follow_standstill_gap=2;seq.follow_time_gap=.5;
 seq.ttc_rear=1;seq.late_margin=.2;seq.max_skip=0;seq.leader_start_distance=0;
 bool newly_reachable=false;
 for(double rear_pos=-40;rear_pos<=-5;rear_pos+=.5){
  std::vector<StreamVehicle> stream{vehicle(1,5,3),vehicle(2,rear_pos,6)};
  const auto normal=GapSelector::evaluateSequential(stream,ego,p,seq,1);
  auto yielding=seq;yielding.rear_yield=true;yielding.rear_reaction=.5;yielding.rear_decel=2;
  const auto cooperative=GapSelector::evaluateSequential(stream,ego,p,yielding,1);
  newly_reachable |= cooperative.departure_ready && !normal.departure_ready;
  yielding.rear_decel=0;
  EXPECT_EQ(GapSelector::evaluateSequential(stream,ego,p,yielding,1).departure_ready,normal.departure_ready);
  yielding.rear_decel=2;yielding.rear_reaction=30;
  EXPECT_EQ(GapSelector::evaluateSequential(stream,ego,p,yielding,1).departure_ready,normal.departure_ready);
 }
 EXPECT_TRUE(newly_reachable);
}
}

namespace mission_planner {
TEST(GapSelectorMeasured, ShortPostEntryHorizonCannotSkipRearBodyCollision) {
 EgoEntryModel ego;ego.distance_to_merge=6;ego.length=4;ego.speed=1;ego.max_speed=1;ego.departure_in_progress=true;
 GapSelectorParams p;p.entry_zone=1;p.check_horizon=0;p.standstill_gap=.5;p.speed_margin=0;p.rear_headway=0;p.max_wait=2;
 SequentialParams seq;seq.max_skip=0;seq.ttc_rear=1;seq.late_margin=0;
 auto rear=vehicle(2,-30,5);rear.length=4;
 const auto partial=GapSelector::evaluateSequential({rear},ego,p,seq,-1);
 ASSERT_TRUE(partial.departure_ready);
 seq.require_rear_clear=true;
 const auto full=GapSelector::evaluateSequential({rear},ego,p,seq,-1);
 EXPECT_FALSE(full.departure_ready);
 EXPECT_TRUE(full.rear_margin < 0 || full.rear_ttc < seq.ttc_rear);
}
}

namespace mission_planner {
TEST(GapSelectorMeasured, UsesProjectedBodyInsteadOfFullLengthOnCrossingAxis) {
 EgoEntryModel ego;ego.distance_to_merge=0;ego.length=4;ego.speed=1;ego.max_speed=1;ego.departure_in_progress=true;
 GapSelectorParams p;p.standstill_gap=.5;p.front_headway=0;p.rear_headway=0;p.speed_margin=0;p.check_horizon=1;
 SequentialParams seq;seq.max_skip=0;seq.follow_time_gap=0;seq.require_rear_clear=true;seq.hold_for_leader=false;
 auto front=vehicle(1,4,1);front.length=4;auto rear=vehicle(2,-20,0);
 const auto length_only=GapSelector::evaluateSequential({front,rear},ego,p,seq,1);
 EXPECT_FALSE(length_only.feasible);
 seq.project_ego=[](double s){MergeFootprint b;b.active=true;b.pos=s;b.half_length=.95;b.speed_ratio=1;return b;};
 const auto body=GapSelector::evaluateSequential({front,rear},ego,p,seq,1);
 EXPECT_TRUE(body.departure_ready);EXPECT_GE(body.rear_margin,0);
 seq.project_ego=[](double s){MergeFootprint b;b.active=true;b.pos=s;b.half_length=.95;b.valid=s<1;return b;};
 EXPECT_FALSE(GapSelector::evaluateSequential({front,rear},ego,p,seq,1).feasible);
 seq.project_ego=[](double){return MergeFootprint{};};
 EXPECT_FALSE(GapSelector::evaluateSequential({front,rear},ego,p,seq,1).feasible);
}
}
TEST(TimedApproach, DelaysContactWithoutManufacturingStoppedLeaderClearance) {
  EXPECT_DOUBLE_EQ(mission_planner::timedApproachSpeedCap(4,-8,4,.5),1.6);
  EXPECT_GT(mission_planner::timedApproachSpeedCap(4,-8,4,.2),
            mission_planner::timedApproachSpeedCap(4,-8,4,.5));
  EXPECT_DOUBLE_EQ(mission_planner::timedApproachSpeedCap(4,-8,0,.2),0);
  EXPECT_DOUBLE_EQ(mission_planner::timedApproachSpeedCap(4,-8,0,.5),0);
  EXPECT_DOUBLE_EQ(mission_planner::timedApproachSpeedCap(0,-8,4,.5),0);
  EXPECT_TRUE(std::isinf(mission_planner::timedApproachSpeedCap(4,1,0,.5)));
  EXPECT_LT(mission_planner::timedApproachSpeedCap(2,-8,4,.5),
            mission_planner::timedApproachSpeedCap(4,-8,4,.5));
}
TEST(TimedApproach, EntryTtcStillRequiresFullCooperativeBrakeClearance) {
  using namespace mission_planner;
  EgoEntryModel ego;ego.distance_to_merge=0;ego.length=4;ego.speed=ego.max_speed=2;
  ego.departure_in_progress=true;
  GapSelectorParams p;p.standstill_gap=.5;p.front_headway=p.rear_headway=0;
  p.speed_margin=0;p.max_wait=0;p.check_horizon=2;p.entry_zone=0;
  StreamVehicle rear;rear.id=2;rear.pos=-8;rear.length=4;rear.speed=rear.speed_hi=5;
  SequentialParams s;s.max_skip=0;s.require_rear_clear=true;s.rear_yield=true;
  s.rear_reaction=.5;s.rear_decel=3;s.ttc_rear=1;s.late_margin=0;
  EXPECT_FALSE(GapSelector::evaluateSequential({rear},ego,p,s,-1).feasible);
  s.rear_ttc_at_entry=true;
  auto safe=GapSelector::evaluateSequential({rear},ego,p,s,-1);
  ASSERT_TRUE(safe.feasible);EXPECT_GE(safe.rear_margin,.49);EXPECT_GE(safe.rear_ttc,1);
  s.rear_decel=2;
  EXPECT_FALSE(GapSelector::evaluateSequential({rear},ego,p,s,-1).feasible);
  s.rear_decel=3;rear.pos=-6;
  EXPECT_FALSE(GapSelector::evaluateSequential({rear},ego,p,s,-1).feasible);
  rear.pos=-8;s.rear_yield=false;
  EXPECT_FALSE(GapSelector::evaluateSequential({rear},ego,p,s,-1).feasible);
}

TEST(TimedApproach, AccelerationRampReachesContactAtClearanceTime) {
  using mission_planner::timedApproachSpeedCap;
  const double remaining=4.0, time=2.5, acceleration=2.0, speed=.4;
  const double cap=timedApproachSpeedCap(remaining,-8,4,.5,speed,acceleration);
  EXPECT_GT(cap,remaining/time);
  const double ramp=(cap-speed)/acceleration;
  ASSERT_LT(ramp,time);
  EXPECT_NEAR(speed*ramp+.5*acceleration*ramp*ramp+cap*(time-ramp),remaining,1e-9);
  EXPECT_DOUBLE_EQ(timedApproachSpeedCap(4,-8,0,.2,speed,acceleration),0);
  EXPECT_DOUBLE_EQ(timedApproachSpeedCap(4,-8,4,.5,3,acceleration),1.6);
  EXPECT_TRUE(std::isinf(timedApproachSpeedCap(4,-.1,4,.2,0,acceleration)));
}

namespace mission_planner {
TEST(SpatialGap, UsesBumperSpaceAndIgnoresVelocityForApproval) {
  SequentialParams p;p.approach_pos=0;p.leader_start_distance=8;
  auto ego=standingEgo();
  for(double speed:{0.,4.,20.}) {
    const auto r=GapSelector::evaluateSpatial({vehicle(1,-5,speed),vehicle(2,-22,speed)},ego,p,1);
    ASSERT_TRUE(r.departure_ready);EXPECT_EQ(r.gap.limit_side,"spatial_gap");
    EXPECT_NEAR(r.rear_margin,17-4.5-4.6-2.5-2.,1e-9);
  }
  const auto tight=GapSelector::evaluateSpatial({vehicle(1,-5,4),vehicle(2,-17,4)},ego,p,1);
  EXPECT_FALSE(tight.feasible);EXPECT_EQ(tight.gap.limit_side,"slot_space");
}
TEST(SpatialGap, RejectsPassedSlotAndAllowsTheNextSlot) {
  SequentialParams p;p.approach_pos=0;p.leader_start_distance=8;
  const auto cars=std::vector<StreamVehicle>{vehicle(1,15,4),vehicle(2,-1,4),vehicle(3,-25,4)};
  const auto old=GapSelector::evaluateSpatial(cars,standingEgo(),p,1);
  EXPECT_FALSE(old.feasible);EXPECT_EQ(old.gap.limit_side,"slot_passed");
  const auto next=GapSelector::evaluateSpatial(cars,standingEgo(),p,2);
  EXPECT_TRUE(next.departure_ready);EXPECT_EQ(next.gap.follower_id,3);
}
TEST(SpatialGap, LargerVehicleReducesFreeSpaceWithoutSpeedPrediction) {
  SequentialParams p;auto truck=vehicle(1,-5,4);truck.length=12;
  EXPECT_FALSE(GapSelector::evaluateSpatial({truck,vehicle(2,-22,4)},standingEgo(),p,1).feasible);
}
TEST(SpatialGap, EmptyRingAndMissingLockedFrontAreDifferent) {
  SequentialParams p;
  EXPECT_TRUE(GapSelector::evaluateSpatial({},standingEgo(),p,-2).departure_ready);
  p.lock_gap=true;p.target_follower=2;
  const auto missing=GapSelector::evaluateSpatial({vehicle(2,-22,4)},standingEgo(),p,1);
  EXPECT_FALSE(missing.feasible);EXPECT_EQ(missing.gap.limit_side,"target_missing");
  const auto intrusion=GapSelector::evaluateSpatial({vehicle(1,10,4),vehicle(2,-22,4),vehicle(3,0,4)},standingEgo(),p,1);
  EXPECT_FALSE(intrusion.feasible);EXPECT_EQ(intrusion.gap.limit_side,"gap_intrusion");
}
TEST(SpatialGap, InvalidGeometryCannotBecomeEmptyGap) {
  SequentialParams p;auto bad=vehicle(1,-5,4);bad.length=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(GapSelector::evaluateSpatial({bad},standingEgo(),p,-2).feasible);
}

} // namespace mission_planner

namespace mission_planner {
TEST(SequentialOpportunity, ChecksReactionMotionBeforeScheduledDeparture) {
  auto ego=standingEgo();ego.speed=2;
  auto p=ringParams();p.max_wait=0;
  SequentialParams seq;seq.lock_gap=true;seq.target_follower=-1;
  seq.pre_departure_delay=.5;seq.pre_departure_max_speed=0;
  seq.pre_departure_stop_distance=0;seq.pre_departure_response=.3;
  seq.motion_collision=[](double time,double travel){return time<.5 && travel>.4?99:-1;};
  auto out=GapSelector::evaluateSequential({},ego,p,seq,-1);
  EXPECT_FALSE(out.departure_ready);EXPECT_EQ(out.motion_collision_id,99);
}
}

namespace mission_planner {
TEST(ObservedArrival, MeasuredProfileRespectsMpcAccelerationAndChecksAfterRearClearance) {
 EgoEntryModel ego;ego.distance_to_merge=8;ego.speed=0;ego.accel=2.2;ego.max_speed=7;ego.start_delay=.24;
 ego.measured_profile=true;ego.profile_times={0,1};ego.profile_speeds={0,4};
 GapSelectorParams p;p.entry_zone=0;p.max_wait=0;p.check_horizon=.5;
 SequentialParams seq;seq.require_rear_clear=true;seq.check_after_rear_clear=true;
 seq.bound_profile_acceleration=true;seq.integration_dt=.1;seq.ttc_rear=0;
 double last_sample=-1.;seq.motion_collision=[&](double time,double){last_sample=time;return -1;};
 const auto result=GapSelector::evaluateSequential({},ego,p,seq,-1);ASSERT_TRUE(result.departure_ready);
 EXPECT_GE(result.gap.merge_time,std::sqrt(2*(ego.distance_to_merge-.5*ego.length)/ego.accel)+ego.start_delay-.1);
 EXPECT_GE(last_sample,result.clear_time+p.check_horizon-1e-6);
}
}

namespace mission_planner {
TEST(PreparationGap, CapacityUses81MetresWithoutClippingBoundedPairToVisibility) {
 EgoEntryModel ego;ego.length=4.6;SequentialParams seq;seq.spatial_front_gap=2.;seq.spatial_rear_gap=1.5;
 seq.observed_slot=true;seq.separate_slot_observation=true;
 seq.observed_bounds=[](double){return std::make_pair(-3.,3.);};
 auto lead=vehicle(1,6.3,4),rear=vehicle(2,-6.3,4);lead.length=rear.length=4.5;
 auto out=GapSelector::evaluateSpatial({lead,rear},ego,seq,1);
 EXPECT_TRUE(out.feasible);EXPECT_NEAR(out.physical_free_length,8.1,1e-6);
 EXPECT_LT(out.observed_free_length,8.1); // Visibility must be checked separately before GO.
 rear.pos+=.02;EXPECT_FALSE(GapSelector::evaluateSpatial({lead,rear},ego,seq,1).feasible);
}
TEST(PreparationGap, IndependentFrontAndRearMinimumsAreEnforced) {
 EgoEntryModel ego;ego.distance_to_merge=0;ego.speed=ego.max_speed=4.;
 GapSelectorParams p;p.max_wait=0;p.speed_margin=0;p.front_headway=p.rear_headway=0;p.check_horizon=.5;
 SequentialParams seq;seq.lock_gap=true;seq.target_follower=2;seq.require_rear_clear=true;
 seq.front_gap=2.;seq.rear_gap=1.5;seq.ttc_rear=0;seq.late_margin=0;seq.hold_for_leader=false;
 seq.project_ego=[](double s){MergeFootprint b;b.active=true;b.pos=s;b.half_length=2.3;return b;};
 auto lead=vehicle(1,6.6,4),rear=vehicle(2,-6.1,4);lead.length=rear.length=4.5;
 ASSERT_TRUE(GapSelector::evaluateSequential({lead,rear},ego,p,seq,1).departure_ready);
 lead.pos-=.3;auto out=GapSelector::evaluateSequential({lead,rear},ego,p,seq,1);
 EXPECT_FALSE(out.departure_ready);EXPECT_LT(out.front_margin,0);EXPECT_GE(out.rear_margin,0);
 lead.pos+=.3;rear.pos+=.3;out=GapSelector::evaluateSequential({lead,rear},ego,p,seq,1);
 EXPECT_FALSE(out.departure_ready);EXPECT_GE(out.front_margin,0);EXPECT_LT(out.rear_margin,0);
}
TEST(PreparationProfile, CreepVelocityShortensMergeAndCannotCrossBeforeScheduledAcceleration) {
 auto ego=standingEgo();ego.distance_to_merge=10;ego.max_speed=4;
 auto p=ringParams();p.max_wait=0;p.check_horizon=.5;
 SequentialParams seq;seq.lock_gap=true;seq.target_follower=-1;seq.require_rear_clear=true;
 seq.check_after_rear_clear=true;seq.bound_profile_acceleration=true;seq.integration_dt=.1;seq.ttc_rear=0;
 seq.preparation_profile=true;seq.pre_departure_delay=1.5;seq.pre_departure_stop_distance=2.;
 seq.motion_collision=[](double t,double s){return t<1.5 && s>2.05?99:-1;};
 seq.pre_departure_max_speed=0;auto stationary=GapSelector::evaluateSequential({},ego,p,seq,-1);
 ASSERT_TRUE(stationary.departure_ready);
 seq.pre_departure_max_speed=1.5;auto rolling=GapSelector::evaluateSequential({},ego,p,seq,-1);
 ASSERT_TRUE(rolling.departure_ready);EXPECT_LT(rolling.clear_time,stationary.clear_time);
 seq.observation_check=[](double,double s){return s>1?std::string("trajectory_observation_unknown"):std::string();};
 auto blocked=GapSelector::evaluateSequential({},ego,p,seq,-1);
 EXPECT_FALSE(blocked.departure_ready);EXPECT_EQ(blocked.observation_failure,"trajectory_observation_unknown");
}
}
