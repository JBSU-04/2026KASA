#pragma once
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <limits>
#include <string>
#include <vector>

namespace tunnel_xicp {
using Cloud = pcl::PointCloud<pcl::PointNormal>;
enum class State { INVALID = 0, DEGRADED = 1, VALID = 2 };
struct Policy {
  size_t min_matches = 100;
  double min_ratio = .6, max_rms = .1, max_lateral = .6;
  double max_yaw = 3.0 * M_PI / 180.0, observable_min = .95;
};
struct Result {
  State state = State::INVALID;
  std::string reason = "not_processed";
  bool left = false, right = false, longitudinal = false, lateral = false, yaw = false;
  double longitudinal_score = 0, lateral_score = 0, yaw_score = 0;
  double rms = std::numeric_limits<double>::infinity(), ratio = 0;
  double lateral_delta = 0, yaw_delta = 0, processing_ms = 0;
  double matching_ms = 0, verification_ms = 0;
  size_t input_points = 0, matched_points = 0, reference_points = 0;
  int iterations = 0, constraints = 0;
  bool official_xicp_called = false;
  Eigen::Matrix4f raw_transform = Eigen::Matrix4f::Identity();
  Eigen::Matrix4d comparison_pose = Eigen::Matrix4d::Identity();
  std::vector<Eigen::Vector3d> reading_matches, reference_matches;
};
void classify(Result& r, const Policy& policy);
Eigen::Matrix4d comparisonPose(const Eigen::Matrix4d& initial, double lateral,
                               double yaw, const Eigen::Vector3d& road_lateral);
class Backend {
 public:
  explicit Backend(const std::string& config);
  ~Backend();
  Result align(const Cloud& reading, const Cloud& reference,
               const Eigen::Matrix4d& initial, double road_heading, const Policy& policy);
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
