#include <algorithm>
#include <string>

#include <non_gps_msgs/MissionCommand.h>
#include <non_gps_msgs/TrackedObject.h>
#include <non_gps_msgs/TrackedObjectArray.h>
#include <ros/ros.h>

class StaticObstacleMissionNode
{
public:
  StaticObstacleMissionNode() : pnh_("~")
  {
    pnh_.param<std::string>("input_topic", input_topic_, "/lidar_perception/tracked_objects");
    pnh_.param<std::string>("output_topic", output_topic_, "/mission/static_obstacle/command");
    pnh_.param("priority", priority_, 50);
    pnh_.param("target_speed", target_speed_, 20.0);
    pnh_.param("steering_offset", steering_offset_, 0.0);
    pnh_.param("brake", brake_, 0.0);
    pnh_.param("roi_x_min", roi_x_min_, 0.5);
    pnh_.param("roi_x_max", roi_x_max_, 30.0);
    pnh_.param("roi_y_min", roi_y_min_, -1.2);
    pnh_.param("roi_y_max", roi_y_max_, 1.2);
    pnh_.param("roi_z_min", roi_z_min_, -1.2);
    pnh_.param("roi_z_max", roi_z_max_, 1.5);
    pub_ = nh_.advertise<non_gps_msgs::MissionCommand>(output_topic_, 1);
    sub_ = nh_.subscribe(input_topic_, 1, &StaticObstacleMissionNode::callback, this);
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
    double dynamic_steering_offset = 0.0;
    
    for (const auto &object : objects->objects) {
      if (object.motion_type == non_gps_msgs::TrackedObject::STATIC &&
          boxIntersectsRoi(object)) {
        found = true;
        // Simple avoidance: if object is on the left (y > 0), steer right (negative offset).
        // If object is on the right (y < 0), steer left (positive offset).
        if (object.position.y > 0.0) {
            dynamic_steering_offset = -0.5; // Steer right
        } else {
            dynamic_steering_offset = 0.5;  // Steer left
        }
        break; // Just avoid the first found object for simplicity
      }
    }

    non_gps_msgs::MissionCommand command;
    command.header = objects->header;
    command.source = "static_obstacle";
    command.active = found;
    command.priority = static_cast<uint8_t>(priority_);
    command.target_speed = target_speed_;
    command.steering_offset = found ? dynamic_steering_offset : steering_offset_;
    command.brake = brake_;
    pub_.publish(command);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  std::string input_topic_, output_topic_;
  int priority_;
  double target_speed_, steering_offset_, brake_;
  double roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_, roi_z_min_, roi_z_max_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "static_obstacle_mission_node");
  StaticObstacleMissionNode node;
  ros::spin();
  return 0;
}
