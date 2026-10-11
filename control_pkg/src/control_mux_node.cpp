#include <algorithm>
#include <string>

#include <non_gps_msgs/ControlCommand.h>
#include <non_gps_msgs/MissionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

class ControlMuxNode
{
public:
  ControlMuxNode() : pnh_("~")
  {
    pnh_.param<std::string>("lane_steering_topic", lane_steering_topic_, "/lane_perception/steering_angle");
    pnh_.param<std::string>("lane_valid_topic", lane_valid_topic_, "/lane_perception/valid");
    pnh_.param<std::string>("static_mission_topic", static_topic_, "/mission/static_obstacle/command");
    pnh_.param<std::string>("dynamic_mission_topic", dynamic_topic_, "/mission/dynamic_obstacle/command");
    pnh_.param<std::string>("control_command_topic", output_topic_, "/control/command");
    pnh_.param("default_speed", default_speed_, 60.0);
    pnh_.param("default_accel", default_accel_, 0.2);
    pnh_.param("stop_when_lane_invalid", stop_when_lane_invalid_, false);
    pnh_.param("command_timeout", command_timeout_, 0.5);

    steering_sub_ = nh_.subscribe(lane_steering_topic_, 1, &ControlMuxNode::steeringCallback, this);
    lane_valid_sub_ = nh_.subscribe(lane_valid_topic_, 1, &ControlMuxNode::laneValidCallback, this);
    static_sub_ = nh_.subscribe(static_topic_, 1, &ControlMuxNode::staticCallback, this);
    dynamic_sub_ = nh_.subscribe(dynamic_topic_, 1, &ControlMuxNode::dynamicCallback, this);
    pub_ = nh_.advertise<non_gps_msgs::ControlCommand>(output_topic_, 1);
    timer_ = nh_.createTimer(ros::Duration(0.025), &ControlMuxNode::timerCallback, this);
  }

private:
  void steeringCallback(const std_msgs::Float64::ConstPtr &msg) { steering_ = msg->data; }
  void laneValidCallback(const std_msgs::Bool::ConstPtr &msg) { lane_valid_ = msg->data; }
  void staticCallback(const non_gps_msgs::MissionCommand::ConstPtr &msg)
  {
    static_command_ = *msg;
    static_stamp_ = ros::Time::now();
  }
  void dynamicCallback(const non_gps_msgs::MissionCommand::ConstPtr &msg)
  {
    dynamic_command_ = *msg;
    dynamic_stamp_ = ros::Time::now();
  }

  bool freshAndActive(const non_gps_msgs::MissionCommand &command, const ros::Time &stamp) const
  {
    return command.active && !stamp.isZero() &&
           (ros::Time::now() - stamp).toSec() <= command_timeout_;
  }

  void timerCallback(const ros::TimerEvent &)
  {
    non_gps_msgs::ControlCommand output;
    output.header.stamp = ros::Time::now();
    output.target_speed = default_speed_;
    output.steering_angle = steering_;
    output.accel = default_accel_;
    output.brake = 0.0;
    output.source = "lane_following";

    const non_gps_msgs::MissionCommand *selected = nullptr;
    if (freshAndActive(static_command_, static_stamp_)) selected = &static_command_;
    if (freshAndActive(dynamic_command_, dynamic_stamp_) &&
        (!selected || dynamic_command_.priority > selected->priority)) {
      selected = &dynamic_command_;
    }

    if (selected) {
      output.target_speed = selected->target_speed;
      output.steering_angle += selected->steering_offset;
      output.brake = selected->brake;
      output.accel = selected->brake > 0.0 ? 0.0 : default_accel_;
      output.source = selected->source;
    } else if (stop_when_lane_invalid_ && !lane_valid_) {
      output.target_speed = 0.0;
      output.accel = 0.0;
      output.brake = 1.0;
      output.source = "lane_invalid_safety";
    }

    // Cornering speed reduction logic
    if (output.target_speed > 10.0) {
        double abs_steer_deg = std::abs(output.steering_angle * 180.0 / M_PI);
        if (abs_steer_deg > 5.0) {
            double speed_drop = (abs_steer_deg - 5.0) * 0.5; // Drop 0.5 km/h per degree over 5
            output.target_speed = std::max(10.0, output.target_speed - speed_drop);
        }
    }
    pub_.publish(output);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber steering_sub_, lane_valid_sub_, static_sub_, dynamic_sub_;
  ros::Publisher pub_;
  ros::Timer timer_;
  std::string lane_steering_topic_, lane_valid_topic_, static_topic_, dynamic_topic_, output_topic_;
  double default_speed_, default_accel_, command_timeout_, steering_ = 0.0;
  bool stop_when_lane_invalid_, lane_valid_ = false;
  non_gps_msgs::MissionCommand static_command_, dynamic_command_;
  ros::Time static_stamp_, dynamic_stamp_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_mux_node");
  ControlMuxNode node;
  ros::spin();
  return 0;
}
