#include "mission_manager/constraint_arbiter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace mission_manager {

Constraint ConstraintArbiter::merge(
    const std::vector<Constraint>& constraints) const {
  Constraint output;
  double comfortable_decel = std::numeric_limits<double>::infinity();
  bool have_source = false;
  std::ostringstream reasons;

  for (const Constraint& constraint : constraints) {
    if (!constraint.active) {
      continue;
    }
    if (have_source) {
      output.source = 255U;
      reasons << "; ";
    } else {
      output.source = constraint.source;
      have_source = true;
    }
    output.active = true;
    output.priority = std::max(output.priority, constraint.priority);
    if (constraint.max_speed >= 0.0 &&
        (output.max_speed < 0.0 || constraint.max_speed < output.max_speed)) {
      output.max_speed = constraint.max_speed;
    }
    if (constraint.stop_required &&
        (!output.stop_required ||
         constraint.stop_distance < output.stop_distance)) {
      output.stop_required = true;
      output.stop_distance = std::max(0.0, constraint.stop_distance);
    }
    if (constraint.lead_vehicle_active &&
        (!output.lead_vehicle_active ||
         constraint.lead_distance < output.lead_distance)) {
      output.lead_vehicle_active = true;
      output.lead_vehicle_id = constraint.lead_vehicle_id;
      output.lead_distance = std::max(0.0, constraint.lead_distance);
      output.lead_speed = std::max(0.0, constraint.lead_speed);
      output.lead_profile_override = constraint.lead_profile_override;
      output.lead_standstill_distance = constraint.lead_standstill_distance;
      output.lead_time_gap = constraint.lead_time_gap;
    }
    if (constraint.comfortable_decel > 0.0) {
      comfortable_decel =
          std::min(comfortable_decel, constraint.comfortable_decel);
    }
    output.emergency_decel =
        std::max(output.emergency_decel, constraint.emergency_decel);
    reasons << (constraint.reason.empty() ? "active constraint"
                                          : constraint.reason);
  }

  if (!output.active) {
    output.reason = "no active mission";
    return output;
  }
  output.comfortable_decel = std::isfinite(comfortable_decel)
                                 ? comfortable_decel
                                 : 2.5;
  if (output.emergency_decel <= 0.0) {
    output.emergency_decel = 5.0;
  }
  output.reason = reasons.str();
  return output;
}

}  // namespace mission_manager
