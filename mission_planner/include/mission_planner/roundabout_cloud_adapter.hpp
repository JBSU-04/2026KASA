#pragma once

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "mission_planner/roundabout_observation.hpp"

namespace mission_planner {

// The complete gravity-aligned scan and /nonground share velodyne_ground.
// Use exactly the tracker's sensor extrinsic + odometry yaw transform; applying
// pitch again to gravity-aligned data would double-correct the scan.
struct ObservationTransform {
  double x=0,y=0,z=0,yaw=0,sensor_x=1.1,sensor_y=0,sensor_z=1.33,sensor_yaw=0;
  ObservationPoint origin() const {
    return {x+std::cos(yaw)*sensor_x-std::sin(yaw)*sensor_y,
            y+std::sin(yaw)*sensor_x+std::cos(yaw)*sensor_y,z+sensor_z};
  }
};
inline bool observationPoints(const sensor_msgs::PointCloud2& cloud,
                              const ObservationTransform& tf,
                              std::vector<ObservationPoint>* points) {
  points->clear();
  // Reject malformed layouts rather than turning a decode failure into a clear scan.
  for(const std::string name:{"x","y","z"}) {
    const auto field=std::find_if(cloud.fields.begin(),cloud.fields.end(),
        [&](const sensor_msgs::PointField& f){return f.name==name;});
    if(field==cloud.fields.end() || field->datatype!=sensor_msgs::PointField::FLOAT32 ||
        field->count!=1 || field->offset+sizeof(float)>cloud.point_step)return false;
  }
  const size_t row=static_cast<size_t>(cloud.width)*cloud.point_step;
  if(cloud.is_bigendian || cloud.row_step!=row || cloud.data.size()!=row*cloud.height)return false;
  if(cloud.width==0 || cloud.height==0)return true; // valid empty non-ground scan
  try {
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud,"x"),y(cloud,"y"),z(cloud,"z");
    const double c=std::cos(tf.yaw+tf.sensor_yaw),s=std::sin(tf.yaw+tf.sensor_yaw);
    const auto origin=tf.origin();points->reserve(cloud.width*cloud.height);
    for(;x!=x.end();++x,++y,++z) {
      if(!std::isfinite(*x)||!std::isfinite(*y)||!std::isfinite(*z))continue;
      points->push_back({origin.x+c*(*x)-s*(*y),origin.y+s*(*x)+c*(*y),origin.z+(*z)});
    }
  } catch(const std::runtime_error&) {return false;}
  return true;
}

} // namespace mission_planner
