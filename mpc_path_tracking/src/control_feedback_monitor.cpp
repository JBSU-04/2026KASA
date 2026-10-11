#include <cmath>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>

#include <morai_msgs/CtrlCmd.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

class ControlFeedbackMonitor {
public:
  ControlFeedbackMonitor()
      : nh_(), pnh_("~"), have_cmd_(false), have_ego_(false),
        have_target_speed_(false), have_curve_speed_limit_(false),
        have_path_curvature_(false), have_mission_speed_target_(false),
        cmd_count_(0), ego_count_(0) {
    pnh_.param<std::string>("ctrl_cmd_topic", ctrl_cmd_topic_, "/ctrl_cmd");
    pnh_.param<std::string>("feedback_topic", feedback_topic_,
                            "/Competition_topic");
    pnh_.param<std::string>("target_speed_topic", target_speed_topic_,
                            "/control_feedback/mpc_target_speed_mps");
    pnh_.param<std::string>("curve_speed_limit_topic", curve_speed_limit_topic_,
                            "/control_feedback/curvature_speed_limit_mps");
    pnh_.param<std::string>("path_curvature_topic", path_curvature_topic_,
                            "/control_feedback/path_curvature_for_speed_limit");
    pnh_.param<std::string>("mission_speed_topic", mission_speed_topic_,
                            "/control_feedback/mission_speed_target_mps");
    pnh_.param<std::string>("output_dir", output_dir_, std::string(""));
    pnh_.param("report_rate_hz", report_rate_hz_, 20.0);
    pnh_.param("log_command_feedback", log_command_feedback_, false);
    if (report_rate_hz_ <= 0.0) {
      ROS_WARN_STREAM("[control_feedback_monitor] report_rate_hz must be positive; "
                      << "using 20 Hz");
      report_rate_hz_ = 20.0;
    }

    cmd_sub_ = nh_.subscribe(ctrl_cmd_topic_, 20,
                             &ControlFeedbackMonitor::cmdCallback, this);
    ego_sub_ = nh_.subscribe(feedback_topic_, 100,
                             &ControlFeedbackMonitor::egoCallback, this);
    target_speed_sub_ = nh_.subscribe(
        target_speed_topic_, 20, &ControlFeedbackMonitor::targetSpeedCallback,
        this);
    curve_speed_limit_sub_ = nh_.subscribe(
        curve_speed_limit_topic_, 20,
        &ControlFeedbackMonitor::curveSpeedLimitCallback, this);
    path_curvature_sub_ = nh_.subscribe(
        path_curvature_topic_, 20,
        &ControlFeedbackMonitor::pathCurvatureCallback, this);
    mission_speed_sub_ = nh_.subscribe(
        mission_speed_topic_, 20,
        &ControlFeedbackMonitor::missionSpeedCallback, this);

    cmd_speed_pub_ = nh_.advertise<std_msgs::Float64>(
        "/control_feedback/cmd_speed_mps", 10);
    actual_speed_pub_ = nh_.advertise<std_msgs::Float64>(
        "/control_feedback/actual_speed_mps", 10);
    speed_error_pub_ = nh_.advertise<std_msgs::Float64>(
        "/control_feedback/speed_error_mps", 10);
    cmd_steer_pub_ = nh_.advertise<std_msgs::Float64>(
        "/control_feedback/cmd_steering", 10);
    front_steer_pub_ = nh_.advertise<std_msgs::Float64>(
        "/control_feedback/front_steer", 10);
    wheel_angle_pub_ = nh_.advertise<std_msgs::Float64>(
        "/control_feedback/wheel_angle", 10);

    openCsv();
    sample_timer_ = nh_.createWallTimer(
        ros::WallDuration(1.0 / report_rate_hz_),
        &ControlFeedbackMonitor::sampleTimerCallback, this);
    ROS_INFO_STREAM("[control_feedback_monitor] cmd=" << ctrl_cmd_topic_
                    << ", feedback=" << feedback_topic_
                    << ", target_speed=" << target_speed_topic_
                    << ", curve_speed_limit=" << curve_speed_limit_topic_
                    << ", path_curvature=" << path_curvature_topic_
                    << ", mission_speed=" << mission_speed_topic_
                    << ", CSV sample rate=" << report_rate_hz_ << " Hz");
  }

private:
  void cmdCallback(const morai_msgs::CtrlCmd::ConstPtr& msg) {
    last_cmd_ = *msg;
    last_cmd_receive_time_ = ros::WallTime::now();
    have_cmd_ = true;
    ++cmd_count_;
    publishIfFinite(cmd_speed_pub_, commandedSpeedMps());
    publish(cmd_steer_pub_, last_cmd_.steering);
    if (have_ego_) publishSpeedError();

    if (have_ego_ && log_command_feedback_) {
      const double feedback_speed_mps = actualSpeedMps();
      ROS_INFO_THROTTLE(0.2,
          "[control feedback] type=%d velocity_cmd=%.2f km/h, throttle=%.3f, brake=%.3f | "
          "speed=%.3f m/s | steer cmd=%.5f, front=%.5f, wheel=%.5f",
          last_cmd_.longlCmdType, last_cmd_.velocity, last_cmd_.accel,
          last_cmd_.brake, feedback_speed_mps, last_cmd_.steering,
          last_ego_.front_steer_angle, last_ego_.wheel_angle);
    } else {
      ROS_WARN_THROTTLE(2.0,
          "[control_feedback_monitor] recording /ctrl_cmd, but waiting for /Competition_topic");
    }
  }

  void targetSpeedCallback(const std_msgs::Float64::ConstPtr& msg) {
    last_target_speed_mps_ = msg->data;
    last_target_speed_receive_time_ = ros::WallTime::now();
    have_target_speed_ = true;

    if (have_cmd_ && last_cmd_.longlCmdType != 2) {
      publishIfFinite(cmd_speed_pub_, commandedSpeedMps());
      if (have_ego_) publishSpeedError();
    }
  }

  void curveSpeedLimitCallback(const std_msgs::Float64::ConstPtr& msg) {
    last_curve_speed_limit_mps_ = msg->data;
    last_curve_speed_limit_receive_time_ = ros::WallTime::now();
    have_curve_speed_limit_ = true;
  }

  void pathCurvatureCallback(const std_msgs::Float64::ConstPtr& msg) {
    last_path_curvature_for_speed_limit_ = msg->data;
    last_path_curvature_receive_time_ = ros::WallTime::now();
    have_path_curvature_ = true;
  }

  void missionSpeedCallback(const std_msgs::Float64::ConstPtr& msg) {
    last_mission_speed_target_mps_ = msg->data;
    last_mission_speed_receive_time_ = ros::WallTime::now();
    have_mission_speed_target_ = true;
  }

  void egoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& msg) {
    last_ego_ = *msg;
    last_ego_receive_time_ = ros::WallTime::now();
    have_ego_ = true;
    ++ego_count_;
    publish(actual_speed_pub_, actualSpeedMps());
    publish(front_steer_pub_, last_ego_.front_steer_angle);
    publish(wheel_angle_pub_, last_ego_.wheel_angle);
    if (have_cmd_) {
      publishSpeedError();
    } else {
      ROS_WARN_THROTTLE(2.0,
          "[control_feedback_monitor] receiving /Competition_topic, but no /ctrl_cmd. "
          "No command/feedback pairs can be recorded.");
    }
  }

  double actualSpeedMps() const {
    return std::hypot(last_ego_.velocity.x, last_ego_.velocity.y);
  }

  double commandedSpeedMps() const {
    if (!have_cmd_) return std::numeric_limits<double>::quiet_NaN();
    if (last_cmd_.longlCmdType == 2) return last_cmd_.velocity / 3.6;
    if (have_target_speed_) return last_target_speed_mps_;
    return std::numeric_limits<double>::quiet_NaN();
  }

  void publishSpeedError() {
    const double cmd_speed_mps = commandedSpeedMps();
    if (std::isfinite(cmd_speed_mps)) {
      publish(speed_error_pub_, cmd_speed_mps - actualSpeedMps());
    }
  }

  // Sampling on a wall timer makes the CSV regular and independent of which
  // topic is published faster.  It also prevents a sparse /ctrl_cmd topic from
  // producing an almost empty file while the simulator is running.
  void sampleTimerCallback(const ros::WallTimerEvent&) {
    if (!have_cmd_ || !have_ego_) {
      ROS_WARN_THROTTLE(2.0,
          "[control_feedback_monitor] CSV waiting for both topics: /ctrl_cmd=%llu, "
          "/Competition_topic=%llu",
          static_cast<unsigned long long>(cmd_count_),
          static_cast<unsigned long long>(ego_count_));
      return;
    }
    writeCsv(actualSpeedMps(), true);
  }

  void openCsv() {
    if (output_dir_.empty()) return;

    const std::time_t now = std::time(NULL);
    std::tm local_time;
    localtime_r(&now, &local_time);
    char timestamp[32];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &local_time);

    csv_path_ = output_dir_ + "/control_feedback_" + timestamp + ".csv";
    csv_.open(csv_path_.c_str());
    if (!csv_.is_open()) {
      ROS_ERROR_STREAM("[control_feedback_monitor] cannot write " << csv_path_);
      return;
    }

    start_time_ = ros::WallTime::now();
    csv_ << "elapsed_s,cmd_recv_elapsed_s,feedback_recv_elapsed_s,"
            "cmd_age_s,feedback_age_s,feedback_available,feedback_stamp_s,"
            "command_type,ctrl_velocity_kph,cmd_speed_mps,"
            "target_speed_available,target_speed_mps,target_speed_age_s,"
            "mission_speed_available,mission_speed_target_mps,mission_speed_age_s,"
            "curve_speed_limit_available,curve_speed_limit_mps,curve_speed_limit_age_s,"
            "path_curvature_available,path_curvature_for_speed_limit,path_curvature_age_s,"
            "speed_error_mps,"
            "ctrl_acceleration,ctrl_accel,ctrl_brake,actual_speed_mps,"
            "ctrl_steering,wheel_angle,front_steer,actual_accel_x,"
            "feedback_accel,feedback_brake\n";
    ROS_INFO_STREAM("[control_feedback_monitor] recording " << csv_path_);
  }

  void writeCsv(double feedback_speed_mps, bool feedback_available) {
    if (!csv_.is_open()) return;

    const ros::WallTime now = ros::WallTime::now();
    const double elapsed = (now - start_time_).toSec();
    const double cmd_recv_elapsed = (last_cmd_receive_time_ - start_time_).toSec();
    const double feedback_recv_elapsed =
        feedback_available ? (last_ego_receive_time_ - start_time_).toSec()
                           : std::numeric_limits<double>::quiet_NaN();
    const double cmd_age = (now - last_cmd_receive_time_).toSec();
    const double feedback_age =
        feedback_available ? (now - last_ego_receive_time_).toSec()
                           : std::numeric_limits<double>::quiet_NaN();
    const double cmd_speed_mps = commandedSpeedMps();
    const double target_speed_age =
        have_target_speed_ ? (now - last_target_speed_receive_time_).toSec()
                           : std::numeric_limits<double>::quiet_NaN();
    const double curve_speed_limit_age =
        have_curve_speed_limit_
            ? (now - last_curve_speed_limit_receive_time_).toSec()
            : std::numeric_limits<double>::quiet_NaN();
    const double mission_speed_age =
        have_mission_speed_target_
            ? (now - last_mission_speed_receive_time_).toSec()
            : std::numeric_limits<double>::quiet_NaN();
    const double path_curvature_age =
        have_path_curvature_
            ? (now - last_path_curvature_receive_time_).toSec()
            : std::numeric_limits<double>::quiet_NaN();
    const double speed_error_mps =
        feedback_available && std::isfinite(cmd_speed_mps)
            ? cmd_speed_mps - feedback_speed_mps
                           : std::numeric_limits<double>::quiet_NaN();

    csv_ << std::fixed << std::setprecision(6)
         << elapsed << ',' << cmd_recv_elapsed << ',' << feedback_recv_elapsed
         << ',' << cmd_age << ',' << feedback_age << ','
         << (feedback_available ? 1 : 0) << ','
         << (feedback_available ? last_ego_.header.stamp.toSec()
                                : std::numeric_limits<double>::quiet_NaN()) << ','
         << last_cmd_.longlCmdType << ',' << last_cmd_.velocity << ','
         << cmd_speed_mps << ',' << (have_target_speed_ ? 1 : 0) << ','
         << (have_target_speed_ ? last_target_speed_mps_
                                : std::numeric_limits<double>::quiet_NaN()) << ','
         << target_speed_age << ','
         << (have_mission_speed_target_ ? 1 : 0) << ','
         << (have_mission_speed_target_ ? last_mission_speed_target_mps_
                                        : std::numeric_limits<double>::quiet_NaN()) << ','
         << mission_speed_age << ','
         << (have_curve_speed_limit_ ? 1 : 0) << ','
         << (have_curve_speed_limit_ ? last_curve_speed_limit_mps_
                                     : std::numeric_limits<double>::quiet_NaN()) << ','
         << curve_speed_limit_age << ','
         << (have_path_curvature_ ? 1 : 0) << ','
         << (have_path_curvature_ ? last_path_curvature_for_speed_limit_
                                  : std::numeric_limits<double>::quiet_NaN()) << ','
         << path_curvature_age << ','
         << speed_error_mps << ','
         << last_cmd_.acceleration << ',' << last_cmd_.accel << ','
         << last_cmd_.brake << ',' << feedback_speed_mps << ','
         << last_cmd_.steering << ','
         << (feedback_available ? last_ego_.wheel_angle
                                : std::numeric_limits<double>::quiet_NaN()) << ','
         << (feedback_available ? last_ego_.front_steer_angle
                                : std::numeric_limits<double>::quiet_NaN()) << ','
         << (feedback_available ? last_ego_.acceleration.x
                                : std::numeric_limits<double>::quiet_NaN()) << ','
         << (feedback_available ? last_ego_.accel
                                : std::numeric_limits<double>::quiet_NaN()) << ','
         << (feedback_available ? last_ego_.brake
                                : std::numeric_limits<double>::quiet_NaN()) << '\n';
    csv_.flush();
  }

  static void publish(ros::Publisher& publisher, double value) {
    std_msgs::Float64 msg;
    msg.data = value;
    publisher.publish(msg);
  }

  static void publishIfFinite(ros::Publisher& publisher, double value) {
    if (std::isfinite(value)) publish(publisher, value);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber cmd_sub_;
  ros::Subscriber ego_sub_;
  ros::Subscriber target_speed_sub_;
  ros::Subscriber curve_speed_limit_sub_;
  ros::Subscriber path_curvature_sub_;
  ros::Subscriber mission_speed_sub_;
  ros::WallTimer sample_timer_;
  ros::Publisher cmd_speed_pub_;
  ros::Publisher actual_speed_pub_;
  ros::Publisher speed_error_pub_;
  ros::Publisher cmd_steer_pub_;
  ros::Publisher front_steer_pub_;
  ros::Publisher wheel_angle_pub_;
  std::string ctrl_cmd_topic_;
  std::string feedback_topic_;
  std::string target_speed_topic_;
  std::string curve_speed_limit_topic_;
  std::string path_curvature_topic_;
  std::string mission_speed_topic_;
  std::string output_dir_;
  std::string csv_path_;
  std::ofstream csv_;
  ros::WallTime start_time_;
  ros::WallTime last_cmd_receive_time_;
  ros::WallTime last_ego_receive_time_;
  ros::WallTime last_target_speed_receive_time_;
  ros::WallTime last_curve_speed_limit_receive_time_;
  ros::WallTime last_path_curvature_receive_time_;
  ros::WallTime last_mission_speed_receive_time_;
  double report_rate_hz_;
  bool log_command_feedback_;
  bool have_cmd_;
  bool have_ego_;
  bool have_target_speed_;
  bool have_curve_speed_limit_;
  bool have_path_curvature_;
  bool have_mission_speed_target_;
  std::uint64_t cmd_count_;
  std::uint64_t ego_count_;
  morai_msgs::CtrlCmd last_cmd_;
  morai_msgs::EgoVehicleStatus last_ego_;
  double last_target_speed_mps_;
  double last_curve_speed_limit_mps_;
  double last_path_curvature_for_speed_limit_;
  double last_mission_speed_target_mps_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "control_feedback_monitor");
  ControlFeedbackMonitor monitor;
  ros::spin();
  return 0;
}
