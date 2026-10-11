#pragma once
#include <vector>
// Interface-only adapter for optional Apollo corner constraints. The upstream
// math source is unchanged; the vehicle sweep is validated in our core.
namespace apollo { namespace planning {
struct InterpolatedConstraint {
  int left_index=0,right_index=0;
  double left_weight=0,right_weight=0,lower_bound=0,upper_bound=0;
};
using ObsCornerConstraints=std::vector<InterpolatedConstraint>;
struct ADCVertexConstraints:std::vector<InterpolatedConstraint>{double front_edge_to_center=0;};
}}
