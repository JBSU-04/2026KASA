// Local ABI adapter, NOT an Apollo original header. Only fields used by the
// unmodified measurement_collection.cc are represented; no CyberRT dependency.
#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <memory>
#include <vector>
namespace apollo { namespace perception {
namespace base {
using PointDCloud = std::vector<Eigen::Vector3d>;
struct Object { struct { PointDCloud cloud_world; } lidar_supplement; };
}
namespace lidar {
struct TrackedObject {
  std::shared_ptr<base::Object> object_ptr = std::make_shared<base::Object>();
  Eigen::Vector3d anchor_point = Eigen::Vector3d::Zero();
  Eigen::Vector3d belief_anchor_point = Eigen::Vector3d::Zero();
  Eigen::Vector3d output_direction = Eigen::Vector3d::UnitX();
  Eigen::Vector3d output_center = Eigen::Vector3d::Zero();
  Eigen::Vector3d output_size = Eigen::Vector3d::Ones();
  Eigen::Vector3d size = Eigen::Vector3d::Ones();
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d detection_center = Eigen::Vector3d::Zero();
  Eigen::Vector3d measured_barycenter_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d measured_center_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d measured_history_center_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d measured_nearest_corner_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d measured_detection_center_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d measured_detection_history_center_velocity = Eigen::Vector3d::Zero();
  std::array<Eigen::Vector3d,4> corners, detection_corners;
  std::array<Eigen::Vector3d,4> measured_corners_velocity;
  std::array<Eigen::Vector3d,4> measured_history_corners_velocity;
  std::array<Eigen::Vector3d,4> measured_detection_history_corners_velocity;
  Eigen::Affine3d sensor_to_local_pose = Eigen::Affine3d::Identity();
};
using TrackedObjectPtr = std::shared_ptr<TrackedObject>;
using TrackedObjectConstPtr = std::shared_ptr<const TrackedObject>;
}}}
