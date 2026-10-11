#pragma once

#include <vector>

#include "mission_planner/types.hpp"

namespace mission_planner {

struct PolylineProjection {
  bool valid = false;
  double s = 0.0;
  double distance = 0.0;
  Point2 point;
  double yaw = 0.0;
};

double boxSeparation(const Point2& a,double ayaw,double alength,double awidth,
                     const Point2& b,double byaw,double blength,double bwidth);
double normalizeAngle(double angle);
double positiveModulo(double value, double modulus);
double distance(const Point2& lhs, const Point2& rhs);
double radiusFrom(const Point2& center, const Point2& point);
double angleFrom(const Point2& center, const Point2& point);
double directionalAngleDelta(double from, double to, bool counter_clockwise);
double circleArcDistance(double from, double to, double radius,
                         bool counter_clockwise);
double tangentYaw(double angle, bool counter_clockwise);
PolylineProjection projectToPolyline(const std::vector<Point2>& path,
                                     const Point2& point);

}  // namespace mission_planner
