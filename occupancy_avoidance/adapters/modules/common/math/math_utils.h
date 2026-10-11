#pragma once
#include <cmath>
#include <utility>
namespace apollo { namespace common { namespace math {
inline double NormalizeAngle(double a){return std::atan2(std::sin(a),std::cos(a));}
inline std::pair<double,double> Cartesian2Polar(double x,double y){return {std::hypot(x,y),std::atan2(y,x)};}
}}}
