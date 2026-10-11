#include "mission_planner/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mission_planner {
double boxSeparation(const Point2& a,double ayaw,double alength,double awidth,
                     const Point2& b,double byaw,double blength,double bwidth) {
  double separation=-1e9;
  for(double axis:{ayaw,ayaw+1.5707963267948966,byaw,byaw+1.5707963267948966}) {
    const double between=std::abs((b.x-a.x)*std::cos(axis)+(b.y-a.y)*std::sin(axis));
    const double as=.5*alength*std::abs(std::cos(ayaw-axis))+.5*awidth*std::abs(std::sin(ayaw-axis));
    const double bs=.5*blength*std::abs(std::cos(byaw-axis))+.5*bwidth*std::abs(std::sin(byaw-axis));
    separation=std::max(separation,between-as-bs);
  }
  return separation;
}

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

}  // namespace

double positiveModulo(double value, double modulus) {
  if (modulus <= 0.0) {
    return value;
  }
  const double result = std::fmod(value, modulus);
  return result < 0.0 ? result + modulus : result;
}

double normalizeAngle(double angle) {
  return positiveModulo(angle + kPi, kTwoPi) - kPi;
}

double distance(const Point2& lhs, const Point2& rhs) {
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

double radiusFrom(const Point2& center, const Point2& point) {
  return distance(center, point);
}

double angleFrom(const Point2& center, const Point2& point) {
  return std::atan2(point.y - center.y, point.x - center.x);
}

double directionalAngleDelta(double from, double to,
                             bool counter_clockwise) {
  return counter_clockwise ? positiveModulo(to - from, kTwoPi)
                           : positiveModulo(from - to, kTwoPi);
}

double circleArcDistance(double from, double to, double radius,
                         bool counter_clockwise) {
  return std::max(0.0, radius) *
         directionalAngleDelta(from, to, counter_clockwise);
}

double tangentYaw(double angle, bool counter_clockwise) {
  return normalizeAngle(angle + (counter_clockwise ? 0.5 * kPi
                                                    : -0.5 * kPi));
}

PolylineProjection projectToPolyline(const std::vector<Point2>& path,
                                     const Point2& point) {
  PolylineProjection result;
  if (path.size() < 2U) {
    return result;
  }

  double cumulative_s = 0.0;
  double best_squared_distance = std::numeric_limits<double>::infinity();

  for (std::size_t index = 0; index + 1U < path.size(); ++index) {
    const Point2& start = path[index];
    const Point2& end = path[index + 1U];
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double segment_squared = dx * dx + dy * dy;
    const double segment_length = std::sqrt(segment_squared);
    if (segment_length < 1.0e-9) {
      continue;
    }

    const double raw_ratio =
        ((point.x - start.x) * dx + (point.y - start.y) * dy) /
        segment_squared;
    const double ratio = std::max(0.0, std::min(1.0, raw_ratio));
    const Point2 projected{start.x + ratio * dx, start.y + ratio * dy};
    const double error_x = point.x - projected.x;
    const double error_y = point.y - projected.y;
    const double squared_distance = error_x * error_x + error_y * error_y;

    if (squared_distance < best_squared_distance) {
      best_squared_distance = squared_distance;
      result.valid = true;
      result.s = cumulative_s + ratio * segment_length;
      result.distance = std::sqrt(squared_distance);
      result.point = projected;
      result.yaw = std::atan2(dy, dx);
    }
    cumulative_s += segment_length;
  }

  return result;
}

}  // namespace mission_planner
