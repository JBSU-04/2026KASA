#include <algorithm>
#include <string>

#include <non_gps_msgs/MissionCommand.h>
#include <non_gps_msgs/TrackedObject.h>
#include <non_gps_msgs/TrackedObjectArray.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

class DynamicObstacleMissionNode
{
public:
  DynamicObstacleMissionNode() : pnh_("~")
  {
    pnh_.param<std::string>("input_topic", input_topic_, "/lidar_perception/tracked_objects");
    pnh_.param<std::string>("output_topic", output_topic_, "/mission/dynamic_obstacle/command");
    pnh_.param("priority", priority_, 100);
    pnh_.param("target_speed", target_speed_, 0.0);
    pnh_.param("steering_offset", steering_offset_, 0.0);
    pnh_.param("brake", brake_, 1.0);
    pnh_.param("roi_x_min", roi_x_min_, 0.5);
    pnh_.param("roi_x_max", roi_x_max_, 30.0);
    pnh_.param("roi_y_min", roi_y_min_, -2.0);
    pnh_.param("roi_y_max", roi_y_max_, 2.0);
    pnh_.param("roi_z_min", roi_z_min_, -1.2);
    pnh_.param("roi_z_max", roi_z_max_, 5.0);
    pub_ = nh_.advertise<non_gps_msgs::MissionCommand>(output_topic_, 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/dynamic_obstacle/visualization", 1);
    sub_ = nh_.subscribe(input_topic_, 1, &DynamicObstacleMissionNode::callback, this);
  }

private:

  bool boxIntersectsRoi(const non_gps_msgs::TrackedObject &object) const
  {
    const double half_x = std::max(object.size.x, 0.0) * 0.5;
    const double half_y = std::max(object.size.y, 0.0) * 0.5;
    const double half_z = std::max(object.size.z, 0.0) * 0.5;
    return object.position.x - half_x <= roi_x_max_ &&
           object.position.x + half_x >= roi_x_min_ &&
           object.position.y - half_y <= roi_y_max_ &&
           object.position.y + half_y >= roi_y_min_ &&
           object.position.z - half_z <= roi_z_max_ &&
           object.position.z + half_z >= roi_z_min_;
  }

  void callback(const non_gps_msgs::TrackedObjectArray::ConstPtr &objects)
  {
    bool found = false;
    double min_dist = std::numeric_limits<double>::infinity();
    
    for (const auto &object : objects->objects) {
      if (object.motion_type == non_gps_msgs::TrackedObject::DYNAMIC &&
          boxIntersectsRoi(object)) {
        found = true;
        double dist = std::hypot(object.position.x, object.position.y);
        if (dist < min_dist) {
            min_dist = dist;
        }
      }
    }

    
    // Visualize ROI
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker m;
    m.header.frame_id = "ego";  // Assuming lidar/ego frame
    m.header.stamp = ros::Time::now();
    m.ns = "roi_box";
    m.id = 0;
    m.type = visualization_msgs::Marker::CUBE;
    m.action = visualization_msgs::Marker::ADD;
    
    m.scale.x = roi_x_max_ - roi_x_min_;
    m.scale.y = roi_y_max_ - roi_y_min_;
    m.scale.z = roi_z_max_ - roi_z_min_;
    
    m.pose.position.x = roi_x_min_ + m.scale.x / 2.0;
    m.pose.position.y = roi_y_min_ + m.scale.y / 2.0;
    m.pose.position.z = roi_z_min_ + m.scale.z / 2.0;
    m.pose.orientation.w = 1.0;
    
    if (found) {
        m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 0.5;
    } else {
        m.color.r = 0.0; m.color.g = 0.0; m.color.b = 1.0; m.color.a = 0.3; // Blue
    }
    
    markers.markers.push_back(m);
    marker_pub_.publish(markers);

    non_gps_msgs::MissionCommand command;

    command.header = objects->header;
    command.source = "dynamic_obstacle";
    command.active = found;
    command.priority = static_cast<uint8_t>(priority_);
    
    if (found) {
        // Distance-based Adaptive Cruise Control (ACC)
        double safe_dist = 6.0; // Desired stopping distance behind the vehicle
        double k_p = 2.0;       // Gain (km/h per meter)
        
        if (min_dist <= safe_dist) {
            command.target_speed = 0.0;
            command.brake = brake_; // Brake hard if too close
        } else {
            // Speed up proportionally to the distance error, capped at target_speed_
            double acc_speed = (min_dist - safe_dist) * k_p;
            command.target_speed = std::max(0.0, std::min(acc_speed, target_speed_));
            command.brake = 0.0;
        }
    } else {
        command.target_speed = target_speed_;
        command.brake = 0.0;
    }
    command.steering_offset = steering_offset_;
    pub_.publish(command);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Publisher marker_pub_;
  std::string input_topic_, output_topic_;
  int priority_;
  double target_speed_, steering_offset_, brake_;
  double roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_, roi_z_min_, roi_z_max_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_obstacle_mission_node");
  DynamicObstacleMissionNode node;
  ros::spin();
  return 0;
}
