#pragma once
#include <cmath>

namespace roundabout_lidar_tracker {
struct RadialRoi {
  double center_x = 0.0;
  double center_y = 0.0;
  double inner_radius = 0.0;
  double outer_radius = 100.0;
  bool contains(double x, double y, double padding = 0.0) const {
    const double radius = std::hypot(x - center_x, y - center_y);
    return std::isfinite(radius) && radius >= std::fmax(0.0, inner_radius - padding) &&
           radius <= outer_radius + padding;
  }
};
}  // namespace roundabout_lidar_tracker
