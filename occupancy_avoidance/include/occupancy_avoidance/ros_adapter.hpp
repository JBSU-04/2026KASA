#pragma once
#include "occupancy_avoidance/core.hpp"
#include <ros/ros.h>
#include <nav_msgs/Path.h>
namespace occupancy_avoidance {
// Internal contract: path, geometry generation and status are acquired together.
// This object never publishes final control/path/stop outputs in embedded mode.
class RosAdapter {
 public:
  struct Output {
    Result result;nav_msgs::Path path;
    bool valid=false,recovery=false;
    std::string rejection;
    double geometry_stamp=-1,plan_stamp=-1;
    uint32_t plan_id=0;uint64_t geometry_digest=0,scene_id=0,coordinate_version=0;
  };
  explicit RosAdapter(ros::NodeHandle params,bool embedded=true);
  ~RosAdapter();
  Output consume(const Pose&,bool recovery,bool tunnel=false);
  void published(uint32_t plan_id);
  void rejected(uint32_t plan_id,const std::string& reason);
 private:
  struct Impl;std::unique_ptr<Impl> impl_;
};
}
