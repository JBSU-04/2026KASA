#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

#include <morai_msgs/EgoVehicleStatus.h>
#include <non_gps_msgs/MissionCommand.h>
#include <non_gps_msgs/TrackedObjectArray.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8MultiArray.h>

class LaneObstacleMissionNode
{
public:
  LaneObstacleMissionNode() : pnh_("~")
  {
    pnh_.param<std::string>("roi_states_topic", roi_states_topic_, "/obstacle_roi_states");
    pnh_.param<std::string>("tracked_objects_topic", tracked_objects_topic_, "/lidar_perception/tracked_objects");
    pnh_.param<std::string>("current_lane_topic", current_lane_topic_, "/current_lane");
    pnh_.param<std::string>("ego_status_topic", ego_status_topic_, "/Competition_topic");
    pnh_.param<std::string>("output_topic", output_topic_, "/mission/dynamic_obstacle/command");
    pnh_.param<std::string>("lane_change_topic", lane_change_topic_, "/lane_change_cmd");
    
    pnh_.param("priority", priority_, 100);
    pnh_.param("lane_change_timeout", lane_change_timeout_, 5.0);
    pnh_.param("lane_change_obstacle_ignore_duration", lane_change_obstacle_ignore_duration_, 3.0);
    pnh_.param("lane_change_completed_ignore_duration", lane_change_completed_ignore_duration_, 2.0);
    
    // For ACC
    pnh_.param("acc_safe_dist", acc_safe_dist_, 8.0);
    pnh_.param("acc_kp", acc_kp_, 2.0);
    pnh_.param("acc_max_speed", acc_max_speed_, 50.0);
    
    // Ego lane boundaries for finding objects
    pnh_.param("ego_lane_y_min", ego_lane_y_min_, -1.9);
    pnh_.param("ego_lane_y_max", ego_lane_y_max_, 1.9);

    roi_sub_ = nh_.subscribe(roi_states_topic_, 1, &LaneObstacleMissionNode::roiCallback, this);
    tracked_sub_ = nh_.subscribe(tracked_objects_topic_, 1, &LaneObstacleMissionNode::trackedCallback, this);
    lane_sub_ = nh_.subscribe(current_lane_topic_, 1, &LaneObstacleMissionNode::laneCallback, this);
    ego_sub_ = nh_.subscribe(ego_status_topic_, 1, &LaneObstacleMissionNode::egoCallback, this);
    
    command_pub_ = nh_.advertise<non_gps_msgs::MissionCommand>(output_topic_, 1);
    lane_change_pub_ = nh_.advertise<std_msgs::Int32>(lane_change_topic_, 1);
    
    timer_ = nh_.createTimer(ros::Duration(0.05), &LaneObstacleMissionNode::timerCallback, this);
  }

private:
  enum class State { DRIVING, DECELERATING, JUDGING, LANE_CHANGING, ACC_FOLLOWING, BLOCKED };
  static constexpr int LEFT_LANE = 1;
  static constexpr int RIGHT_LANE = 2;

  void roiCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg)
  {
    if (msg->data.size() >= 3) {
      for (size_t i = 0; i < 3; ++i) roi_occupied_[i] = msg->data[i] != 0;
      roi_stamp_ = ros::Time::now();
    }
  }

  void trackedCallback(const non_gps_msgs::TrackedObjectArray::ConstPtr &msg)
  {
    tracked_objects_ = *msg;
    tracked_stamp_ = ros::Time::now();
  }

  void laneCallback(const std_msgs::Int32::ConstPtr &msg)
  {
    if (msg->data == LEFT_LANE || msg->data == RIGHT_LANE) {
      current_lane_ = msg->data;
    }
  }

  void egoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr &msg)
  {
    ego_speed_ = std::abs(msg->velocity.x);
  }

  bool ownLaneOccupied() const { return roi_occupied_[1]; }

  bool otherLaneOccupied() const
  {
    return current_lane_ == RIGHT_LANE ? roi_occupied_[0] : roi_occupied_[2];
  }

  void timerCallback(const ros::TimerEvent &)
  {
    ros::Time now = ros::Time::now();

    switch (state_) {
      case State::DRIVING:
        if (now < obstacle_ignore_until_) {
          // Ignore obstacles and just drive
          break;
        }
        if (ownLaneOccupied()) {
          bool is_dynamic = false;
          for (const auto &obj : tracked_objects_.objects) {
            if (obj.position.y >= ego_lane_y_min_ && obj.position.y <= ego_lane_y_max_ && 
                obj.motion_type == non_gps_msgs::TrackedObject::DYNAMIC) {
              is_dynamic = true; break;
            }
          }
          if (is_dynamic) {
            state_ = State::ACC_FOLLOWING;
            ROS_INFO("[MISSION] Moving object detected! Switching directly to ACC.");
          } else {
            state_ = State::DECELERATING;
            ROS_WARN("[MISSION] Obstacle detected! Decelerating...");
          }
        } else if (current_lane_ == LEFT_LANE && !roi_occupied_[2]) {
          // Return to right lane if it's empty
          beginLaneChange(now);
          ROS_INFO("[MISSION] Right lane is clear. Returning to origin lane.");
        }
        break;

      case State::DECELERATING:
        if (!ownLaneOccupied()) {
          state_ = State::DRIVING; // False alarm or obstacle disappeared
        } else {
          bool is_dynamic = false;
          for (const auto &obj : tracked_objects_.objects) {
            if (obj.position.y >= ego_lane_y_min_ && obj.position.y <= ego_lane_y_max_ && 
                obj.motion_type == non_gps_msgs::TrackedObject::DYNAMIC) {
              is_dynamic = true; break;
            }
          }
          if (is_dynamic) {
            state_ = State::ACC_FOLLOWING;
            ROS_INFO("[MISSION] Object is moving! Canceling stop and engaging ACC.");
          } else if (ego_speed_ < 1.0) {
            state_ = State::JUDGING;
            judge_start_time_ = now;
            ROS_INFO("[MISSION] Stopped. Judging object type (STATIC/DYNAMIC)...");
          }
        }
        break;

      case State::JUDGING:
        if (!ownLaneOccupied()) {
          state_ = State::DRIVING;
        } else if ((now - judge_start_time_).toSec() > 0.5) {
          // Wait 0.5s to gather tracking data, then decide
          classifyAndDecide(now);
        }
        break;

      case State::ACC_FOLLOWING:
        if (!ownLaneOccupied()) {
          ROS_INFO("[MISSION] Dynamic object left the lane. Resuming DRIVING.");
          state_ = State::DRIVING;
        } else {
          bool has_dynamic = false;
          for (const auto &obj : tracked_objects_.objects) {
            if (obj.position.y >= ego_lane_y_min_ && obj.position.y <= ego_lane_y_max_ && 
                obj.motion_type == non_gps_msgs::TrackedObject::DYNAMIC) {
              has_dynamic = true; break;
            }
          }
          if (!has_dynamic) {
            ROS_WARN("[MISSION] Dynamic object lost, but lane still occupied! Reverting to DECELERATING.");
            state_ = State::DECELERATING;
          }
        }
        break;

      case State::BLOCKED:
        if (!ownLaneOccupied()) {
          state_ = State::DRIVING;
        } else if (!otherLaneOccupied()) {
          ROS_INFO("[MISSION] Other lane is now clear. Changing lane!");
          beginLaneChange(now);
        }
        break;

      case State::LANE_CHANGING:
        if (current_lane_ != lane_change_origin_) {
          state_ = State::DRIVING;
          obstacle_ignore_until_ = now + ros::Duration(lane_change_completed_ignore_duration_);
          ROS_INFO("[MISSION] Lane change completed!");
        } else if ((now - lane_change_started_).toSec() >= lane_change_timeout_) {
          state_ = State::DRIVING;
          ROS_WARN("[MISSION] Lane change timed out!");
        }
        break;
    }

    publishCommand(now);
  }

  void classifyAndDecide(const ros::Time &now)
  {
    bool has_dynamic = false;
    // Check tracked objects in our lane
    for (const auto &obj : tracked_objects_.objects) {
      if (obj.position.y >= ego_lane_y_min_ && obj.position.y <= ego_lane_y_max_) {
        if (obj.motion_type == non_gps_msgs::TrackedObject::DYNAMIC) {
          has_dynamic = true;
          break;
        }
      }
    }

    if (has_dynamic) {
      ROS_INFO("[MISSION] Identified as DYNAMIC. Switching to ACC_FOLLOWING.");
      state_ = State::ACC_FOLLOWING;
    } else {
      ROS_INFO("[MISSION] Identified as STATIC.");
      if (otherLaneOccupied()) {
        ROS_WARN("[MISSION] Other lane is blocked! Waiting...");
        state_ = State::BLOCKED;
      } else {
        ROS_INFO("[MISSION] Other lane clear. Changing lane.");
        beginLaneChange(now);
      }
    }
  }

  void beginLaneChange(const ros::Time &now)
  {
    std_msgs::Int32 cmd;
    cmd.data = current_lane_ == RIGHT_LANE ? -1 : 1;
    lane_change_pub_.publish(cmd);
    
    lane_change_origin_ = current_lane_;
    lane_change_started_ = now;
    obstacle_ignore_until_ = now + ros::Duration(lane_change_obstacle_ignore_duration_);
    state_ = State::LANE_CHANGING;
  }

  void publishDrivingCommand(const ros::Time &now)
  {
    non_gps_msgs::MissionCommand cmd;
    cmd.header.stamp = now;
    cmd.source = "lane_obstacle";
    cmd.active = false; 
    command_pub_.publish(cmd);
  }

  void publishCommand(const ros::Time &now)
  {
    non_gps_msgs::MissionCommand cmd;
    cmd.header.stamp = now;
    cmd.source = "lane_obstacle";
    cmd.active = true;
    cmd.priority = priority_;
    cmd.steering_offset = 0.0;

    if (state_ == State::DRIVING) {
      cmd.active = false;
    } else if (state_ == State::DECELERATING || state_ == State::JUDGING || state_ == State::BLOCKED) {
      cmd.target_speed = 0.0;
      cmd.brake = 0.7; // Stronger brake for safer stop
    } else if (state_ == State::LANE_CHANGING) {
      cmd.target_speed = 30.0; // Safe speed for lane change
      cmd.brake = 0.0;
    } else if (state_ == State::ACC_FOLLOWING) {
      double min_dist = std::numeric_limits<double>::infinity();
      for (const auto &obj : tracked_objects_.objects) {
        if (obj.position.y >= ego_lane_y_min_ && obj.position.y <= ego_lane_y_max_) {
          if (obj.motion_type == non_gps_msgs::TrackedObject::DYNAMIC) {
            double dist = std::hypot(obj.position.x, obj.position.y);
            if (dist < min_dist) min_dist = dist;
          }
        }
      }
      
      if (std::isinf(min_dist)) {
        cmd.target_speed = 0.0; 
        cmd.brake = 0.7;
      } else {
        double min_brake_dist = 5.0; // Hard limit

        if (min_dist <= min_brake_dist) {
          cmd.target_speed = 0.0;
          cmd.brake = 0.7; // Hard stop
        } else {
          // Unify the speed target calculation for ANY distance above 5m
          // If min_dist = 15m, target_speed = (15 - 5) * 4 = 40 km/h
          // If min_dist = 10m, target_speed = (10 - 5) * 4 = 20 km/h
          double acc_speed = (min_dist - min_brake_dist) * acc_kp_;
          cmd.target_speed = std::max(0.0, std::min(acc_speed, acc_max_speed_));
          
          // Speed-matching active brake (Trust the brakes a little while driving!)
          // If we are going faster than the calculated target_speed, smoothly tap the brake
          double ego_speed_kph = ego_speed_ * 3.6;
          if (ego_speed_kph > cmd.target_speed + 1.0) { // Small margin to prevent flickering
            double speed_error = ego_speed_kph - cmd.target_speed;
            cmd.brake = std::min(0.7, speed_error * 0.05); // Smooth proportional brake
          } else {
            cmd.brake = 0.0;
          }
        }
      }
    }

    command_pub_.publish(cmd);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber roi_sub_, tracked_sub_, lane_sub_, ego_sub_;
  ros::Publisher command_pub_, lane_change_pub_;
  ros::Timer timer_;
  
  std::string roi_states_topic_, tracked_objects_topic_, current_lane_topic_, ego_status_topic_;
  std::string output_topic_, lane_change_topic_;
  
  int priority_;
  double lane_change_timeout_, lane_change_obstacle_ignore_duration_, lane_change_completed_ignore_duration_;
  double acc_safe_dist_, acc_kp_, acc_max_speed_;
  double ego_lane_y_min_, ego_lane_y_max_;

  std::array<bool, 3> roi_occupied_{{false, false, false}};
  non_gps_msgs::TrackedObjectArray tracked_objects_;
  int current_lane_ = RIGHT_LANE;
  int lane_change_origin_ = RIGHT_LANE;
  double ego_speed_ = 0.0;
  State state_ = State::DRIVING;
  
  ros::Time roi_stamp_, tracked_stamp_, judge_start_time_, lane_change_started_, obstacle_ignore_until_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lane_obstacle_mission_node");
  LaneObstacleMissionNode node;
  ros::spin();
  return 0;
}
