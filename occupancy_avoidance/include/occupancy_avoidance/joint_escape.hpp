#pragma once
#include "occupancy_avoidance/core.hpp"
#include <functional>
namespace occupancy_avoidance {
struct EscapePlan {
 bool valid=false;std::string reason="no_path";
 std::vector<Reference> reverse,forward;std::vector<double> reverse_steer;
 double reverse_distance=0,compute_ms=0,minimum_clearance=1e9;size_t expanded=0;
};
// ROS1 port of Apollo Hybrid A* motion expansion; original ReedShepp is linked.
EscapePlan jointEscape(const Pose&,const std::vector<Reference>& goals,
 const std::function<bool(const Reference&,bool,double)>& clear,double wheelbase,
 double steer_limit,double max_reverse,double budget,bool forward_only=false);
}
