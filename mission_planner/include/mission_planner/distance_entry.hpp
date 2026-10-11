#pragma once
#include "mission_planner/gap_selector.hpp"
namespace mission_planner {
struct DistanceEntryParams {
  double front_gap = 2.5, rear_gap = 2.0, horizon = 1.5;
  double reaction = 0.5, rear_decel = 2.0, speed_margin = 0.2;
  bool rear_yield = false;
  bool clearance_only = false;
  double follow_standstill_gap = 4.0, follow_time_gap = 0.0;
  double leader_release_gap = 0.8, comfortable_decel = 2.5;
  double front_response_time = 0.0;
  double front_max_decel = 0.0;
  // 2026-10-11: 앞차가 합류점을 지나기 전부터 '합류점 기준 앞차 뒤끝'을 자차 경로의
  //   가상 앞차로 보고 추종한다(앞차 통과에 맞춘 출발). false 면 통과 뒤에만 추종.
  bool project_leader = false;
};
struct EntryMotionSample { double time, position, speed; };
struct DistanceEntryResult {
  bool safe = false;
  double front_min = std::numeric_limits<double>::infinity();
  double rear_min = std::numeric_limits<double>::infinity();
  double front_now = std::numeric_limits<double>::infinity();
  double rear_now = std::numeric_limits<double>::infinity();
  double required_decel = 0.0, entry_time = 0.0, clear_time = 0.0;
  std::string reason;
  std::vector<EntryMotionSample> motion;
  double reference_time = 0.0;
  double delay_remaining = 0.0;
  bool leader_limited = false, curve_limited = false;
};
double frontBrakingSpeedCap(double gap, double lead_speed, double standstill,
                            double decel, double reaction, double lead_decel = 0.0,
                            double ego_speed = 0.0);
void sampleEntryMotion(const DistanceEntryResult& result, double time,
                      double* position, double* speed);
// Sample the same measured-motion/start-delay profile used by the existing gap
// selector. Trajectory mode checks the interval through clearance+horizon;
// clearance mode checks front space at entry and rear space at full clearance.
DistanceEntryResult evaluateDistanceEntry(const GapCandidate& gap,
    const EgoEntryModel& ego, const DistanceEntryParams& params);
}
