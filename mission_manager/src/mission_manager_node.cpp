#include <algorithm>
#include <mutex>
#include <string>
#include <vector>

#include <XmlRpcValue.h>
#include <boost/bind/bind.hpp>
#include <mission_planner_msgs/BehaviorConstraint.h>
#include <ros/ros.h>

#include "mission_manager/constraint_arbiter.hpp"

namespace mission_manager {

class MissionManagerNode {
 public:
  MissionManagerNode() : nh_(), pnh_("~") {
    loadParameters();
    inputs_.resize(input_topics_.size());
    subscribers_.reserve(input_topics_.size());
    for (std::size_t index = 0; index < input_topics_.size(); ++index) {
      subscribers_.push_back(nh_.subscribe<
          mission_planner_msgs::BehaviorConstraint>(
          input_topics_[index], 5,
          boost::bind(&MissionManagerNode::constraintCallback, this,
                      boost::placeholders::_1, index)));
    }
    output_pub_ = nh_.advertise<mission_planner_msgs::BehaviorConstraint>(
        output_topic_, 2);
    timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_hz_),
                             &MissionManagerNode::timerCallback, this);
    ROS_INFO_STREAM("[mission_manager] " << input_topics_.size()
                    << " mission inputs -> " << output_topic_);
  }

 private:
  struct InputState {
    Constraint constraint;
    ros::Time received;
    bool received_once = false;
    bool was_active = false;
  };

  void loadParameters() {
    pnh_.param<std::string>("output_topic", output_topic_,
                            "/mission/behavior_constraint");
    pnh_.param<double>("update_rate_hz", update_rate_hz_, 20.0);
    pnh_.param<double>("input_timeout", input_timeout_, 0.5);
    pnh_.param<bool>("fail_closed_after_active", fail_closed_after_active_,
                     true);
    update_rate_hz_ = std::max(1.0, update_rate_hz_);
    input_timeout_ = std::max(0.05, input_timeout_);

    XmlRpc::XmlRpcValue topics;
    if (pnh_.getParam("input_topics", topics) &&
        topics.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for (int index = 0; index < topics.size(); ++index) {
        if (topics[index].getType() == XmlRpc::XmlRpcValue::TypeString) {
          input_topics_.push_back(static_cast<std::string>(topics[index]));
        }
      }
    }
    if (input_topics_.empty()) {
      input_topics_.push_back("/mission/constraints/roundabout");
    }
  }

  static Constraint fromRos(
      const mission_planner_msgs::BehaviorConstraint& message) {
    Constraint constraint;
    constraint.source = message.source;
    constraint.priority = message.priority;
    constraint.active = message.active;
    constraint.max_speed = message.max_speed;
    constraint.stop_required = message.stop_required;
    constraint.stop_distance = message.stop_distance;
    constraint.lead_vehicle_active = message.lead_vehicle_active;
    constraint.lead_vehicle_id = message.lead_vehicle_id;
    constraint.lead_distance = message.lead_distance;
    constraint.lead_speed = message.lead_speed;
    constraint.lead_time_gap = message.lead_time_gap;
    constraint.lead_standstill_distance = message.lead_standstill_distance;
    constraint.lead_profile_override = message.lead_profile_override;
    constraint.comfortable_decel = message.comfortable_decel;
    constraint.emergency_decel = message.emergency_decel;
    constraint.reason = message.reason;
    return constraint;
  }

  static mission_planner_msgs::BehaviorConstraint toRos(
      const Constraint& constraint, const ros::Time& stamp) {
    mission_planner_msgs::BehaviorConstraint message;
    message.header.stamp = stamp;
    message.source = constraint.source;
    message.priority = constraint.priority;
    message.active = constraint.active;
    message.max_speed = constraint.max_speed;
    message.stop_required = constraint.stop_required;
    message.stop_distance = constraint.stop_distance;
    message.lead_vehicle_active = constraint.lead_vehicle_active;
    message.lead_vehicle_id = constraint.lead_vehicle_id;
    message.lead_distance = constraint.lead_distance;
    message.lead_speed = constraint.lead_speed;
    message.lead_time_gap = constraint.lead_time_gap;
    message.lead_standstill_distance = constraint.lead_standstill_distance;
    message.lead_profile_override = constraint.lead_profile_override;
    message.comfortable_decel = constraint.comfortable_decel;
    message.emergency_decel = constraint.emergency_decel;
    message.reason = constraint.reason;
    return message;
  }

  static Constraint staleEmergency(const std::string& topic) {
    Constraint constraint;
    constraint.source = 255U;
    constraint.priority = 200U;
    constraint.active = true;
    constraint.max_speed = 0.0;
    constraint.stop_required = true;
    constraint.stop_distance = 0.0;
    constraint.comfortable_decel = 3.0;
    constraint.emergency_decel = 6.0;
    constraint.reason = "active mission input stale: " + topic;
    return constraint;
  }

  void constraintCallback(
      const mission_planner_msgs::BehaviorConstraint::ConstPtr& message,
      std::size_t index) {
    std::lock_guard<std::mutex> lock(mutex_);
    InputState& input = inputs_[index];
    input.constraint = fromRos(*message);
    input.received = ros::Time::now();
    input.received_once = true;
    input.was_active = message->active;
  }

  void timerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    std::vector<Constraint> constraints;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      constraints.reserve(inputs_.size());
      for (std::size_t index = 0; index < inputs_.size(); ++index) {
        const InputState& input = inputs_[index];
        const bool fresh = input.received_once &&
            (now - input.received).toSec() <= input_timeout_;
        if (fresh) {
          constraints.push_back(input.constraint);
        } else if (fail_closed_after_active_ && input.was_active) {
          constraints.push_back(staleEmergency(input_topics_[index]));
        }
      }
    }
    const Constraint merged = arbiter_.merge(constraints);
    output_pub_.publish(toRos(merged, now));
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::vector<ros::Subscriber> subscribers_;
  ros::Publisher output_pub_;
  ros::Timer timer_;
  std::mutex mutex_;
  ConstraintArbiter arbiter_;
  std::vector<InputState> inputs_;
  std::vector<std::string> input_topics_;
  std::string output_topic_;
  double update_rate_hz_ = 20.0;
  double input_timeout_ = 0.5;
  bool fail_closed_after_active_ = true;
};

}  // namespace mission_manager

int main(int argc, char** argv) {
  ros::init(argc, argv, "mission_manager");
  mission_manager::MissionManagerNode node;
  ros::spin();
  return 0;
}
