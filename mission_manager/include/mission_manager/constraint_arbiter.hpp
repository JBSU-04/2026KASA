#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace mission_manager {

struct Constraint {
  std::uint8_t source = 0;
  std::uint8_t priority = 0;
  bool active = false;
  double max_speed = -1.0;
  bool stop_required = false;
  double stop_distance = 0.0;
  bool lead_vehicle_active = false;
  int lead_vehicle_id = -1;
  double lead_distance = 0.0;
  double lead_speed = 0.0;
  bool lead_profile_override = false;
  double lead_standstill_distance = 0.0;
  double lead_time_gap = 0.0;
  double comfortable_decel = 0.0;
  double emergency_decel = 0.0;
  std::string reason;
};

class ConstraintArbiter {
 public:
  Constraint merge(const std::vector<Constraint>& constraints) const;
};

}  // namespace mission_manager
