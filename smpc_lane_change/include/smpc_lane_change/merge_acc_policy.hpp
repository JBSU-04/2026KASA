#pragma once
#include <algorithm>
#include <cmath>
#include <smpc_lane_change/BehaviorLongitudinalRequest.h>
#include <smpc_lane_change/BehaviorLongitudinalStatus.h>
#include <smpc_lane_change/TargetVehicleSet.h>
namespace smpc_lane_change {
inline TargetVehicle selectedMergeLeader(const BehaviorLongitudinalRequest& request,
                                        const TargetVehicleSet& targets, double max_age) {
  TargetVehicle out; out.valid=false;
  if(!request.merge_gap_selected || request.merge_leader_id<0 ||
     request.current_lane_id!=targets.current_lane_id || request.target_lane_id!=targets.target_lane_id) return out;
  auto pick=[&](const TargetVehicle& v) {
    if(v.valid && v.unique_id==request.merge_leader_id && v.lane_id==targets.target_lane_id &&
       std::isfinite(v.observation_age) && v.observation_age>=0 && v.observation_age<=max_age) out=v;
  };
  pick(targets.target_front);pick(targets.target_rear);
  for(const auto& v:targets.nearby_vehicles) pick(v);
  return out;
}
inline double independentLeadCap(const BehaviorLongitudinalStatus& status,
                                  const TargetVehicleSet& targets,
                                  double now, double received, double timeout, double fallback) {
  const double age=now-status.header.stamp.toSec(), rx=now-received;
  if(status.header.stamp.isZero() || age<0 || age>timeout || rx<0 || rx>timeout ||
     status.current_lane_id!=targets.current_lane_id || status.target_lane_id!=targets.target_lane_id ||
     !status.independent_lead_limit_active || !std::isfinite(status.independent_lead_speed_cap_mps)) return fallback;
  return std::min(fallback,std::max(0.,status.independent_lead_speed_cap_mps));
}
inline double mergeFollowSpeed(double gap, double lead_speed, double desired,
                               double close_gain, double gap_gain, double catchup,
                               double minimum, double maximum) {
  if(!std::isfinite(gap) || !std::isfinite(lead_speed)) return minimum;
  const double gain=gap<desired ? close_gain : gap_gain;
  return std::clamp(std::min(lead_speed+gain*(gap-desired),lead_speed+catchup),minimum,maximum);
}
}
