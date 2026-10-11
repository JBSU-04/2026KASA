// Local geometry adapter. Cloud points are double precision local coordinates.
#pragma once
#include <algorithm>
#include <limits>
#include "modules/perception/lidar_tracking/tracker/common/tracked_object.h"
namespace apollo { namespace perception { namespace algorithm {
inline void CalculateBBoxSizeCenter2DXY(const base::PointDCloud& cloud,
    const Eigen::Vector3f& direction, Eigen::Vector3f* size,
    Eigen::Vector3d* center, float minimum_edge_length) {
  Eigen::Vector3d u(direction.x(), direction.y(), 0.0); u.normalize();
  Eigen::Vector3d v(-u.y(), u.x(), 0.0);
  Eigen::Vector3d lo = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d hi = -lo;
  for (const auto& p : cloud) {
    Eigen::Vector3d q(p.dot(u), p.dot(v), p.z());
    lo = lo.cwiseMin(q); hi = hi.cwiseMax(q);
  }
  const Eigen::Vector3d mid = (lo + hi) * 0.5;
  *center = u * mid.x() + v * mid.y() + Eigen::Vector3d(0,0,mid.z());
  *size = (hi-lo).cwiseMax(double(minimum_edge_length)).cast<float>();
}
template<class T> Eigen::Matrix<T,3,1> Calculate2DXYProjectVector(
    const Eigen::Matrix<T,3,1>& value, const Eigen::Matrix<T,3,1>& direction) {
  const T norm = direction.template head<2>().squaredNorm();
  if (norm < T(1e-12)) return Eigen::Matrix<T,3,1>::Zero();
  Eigen::Matrix<T,3,1> result = direction *
      (value.template head<2>().dot(direction.template head<2>()) / norm);
  result.z() = 0; return result;
}
}}}
