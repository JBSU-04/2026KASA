// KUUVe C ABI adapter. Actual velocity candidate calculations are Apollo originals.
#include <cmath>
#include "modules/perception/lidar_tracking/tracker/measurement/measurement_collection.h"
#include "modules/perception/common/algorithm/geometry/basic.h"
using namespace apollo::perception;
extern "C" int apollo_velocity_candidates(const double* old_points, int old_n,
    const double* new_points, int new_n, double yaw, double dt, double* result) {
  if (!old_points || !new_points || !result || old_n < 2 || new_n < 2 ||
      !std::isfinite(dt) || dt < 0.001 || !std::isfinite(yaw)) return -1;
  try {
    auto old = std::make_shared<lidar::TrackedObject>();
    auto current = std::make_shared<lidar::TrackedObject>();
    for (int j=0; j<2; ++j) {
      auto obj = j ? current : old;
      const double* data = j ? new_points : old_points;
      int n = j ? new_n : old_n;
      for (int i=0; i<n; ++i) {
        Eigen::Vector3d p(data[3*i], data[3*i+1], data[3*i+2]);
        if (!p.allFinite()) return -2;
        obj->object_ptr->lidar_supplement.cloud_world.push_back(p);
        obj->anchor_point += p;
      }
      obj->anchor_point /= n;
      obj->belief_anchor_point = obj->anchor_point;
      obj->output_direction = Eigen::Vector3d(std::cos(yaw),std::sin(yaw),0);
      Eigen::Vector3f size;
      algorithm::CalculateBBoxSizeCenter2DXY(obj->object_ptr->lidar_supplement.cloud_world,
          obj->output_direction.cast<float>(), &size, &obj->center, 0.01f);
      obj->size = size.cast<double>(); obj->output_size = obj->size;
      obj->output_center = obj->center;
    }
    lidar::MeasureAnchorPointVelocity(current, old, dt);
    lidar::MeasureBboxCenterVelocity(current, old, dt);
    lidar::MeasureBboxCornerVelocity(current, old, dt);
    std::array<Eigen::Vector3d,6> candidates;
    candidates[0] = current->measured_barycenter_velocity;
    candidates[1] = current->measured_center_velocity;
    for (int i=0;i<4;++i) candidates[i+2]=current->measured_corners_velocity[i];
    for (int i=0;i<6;++i) for(int k=0;k<2;++k) result[2*i+k]=candidates[i][k];
    return 0;
  } catch (...) { return -3; }
}
