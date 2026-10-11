// Read-only offline evaluator; consumes recorded entry snapshots on stdin.
#include <iostream>
#include "mission_planner/gap_selector.hpp"

int main() {
  using namespace mission_planner;
  GapSelectorParams p;
  SequentialParams seq;
  EgoEntryModel ego;
  std::cin >> p.front_headway >> p.rear_headway >> p.standstill_gap
           >> p.check_horizon >> p.speed_margin >> p.entry_zone >> p.max_wait
           >> seq.ttc_rear >> seq.late_margin >> seq.leader_start_distance
           >> seq.adaptive_leader_start >> seq.leader_start_max_distance
           >> seq.departure_confirm_time >> seq.comfortable_decel >> seq.abort_margin
           >> seq.follow_standstill_gap >> seq.follow_time_gap
           >> ego.accel >> ego.start_delay >> ego.length;
  if (!std::cin) return 1;
  seq.max_skip = 0;
  int leader, follower, count;
  double stamp;
  while (std::cin >> stamp >> ego.distance_to_merge >> ego.speed >> ego.max_speed
                  >> leader >> follower >> count) {
    std::vector<StreamVehicle> cars(count);
    for (auto& car : cars)
      std::cin >> car.id >> car.pos >> car.speed >> car.length >> car.speed_lo
               >> car.speed_hi >> car.physical_speed;
    if (!std::cin) return 1;
    seq.lock_gap = true;
    seq.target_follower = follower;
    ego.departure_in_progress = false;
    auto result = GapSelector::evaluateSequential(cars, ego, p, seq, leader);
    std::cout << stamp << ' ' << result.departure_ready << ' ' << result.feasible
              << ' ' << result.gap.depart_delay << ' ' << result.gap.merge_time
              << ' ' << result.rear_margin << ' ' << result.rear_ttc << '\n';
  }
  return 0;
}
