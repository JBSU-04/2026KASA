#include <algorithm>
#include <cstdint>
#include <cmath>
#include <filesystem>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <waypoint_maker/Lane.h>
#include <waypoint_maker/State.h>

#include <smpc_lane_change/LaneChangeDecision.h>
#include <smpc_lane_change/LaneChangeStatus.h>
#include <smpc_lane_change/lane_path.hpp>

namespace fs = std::filesystem;

class LaneChangeSupervisor {
 public:
  LaneChangeSupervisor() : nh_(), pnh_("~") {
    pnh_.param("confirmation_count", confirmation_count_, 3);
    pnh_.param("cooldown_sec", cooldown_sec_, 1.0);
    pnh_.param("lane_change_duration_sec", lane_change_duration_sec_, 4.0);
    pnh_.param<std::string>("waypoint_directory", waypoint_directory_, "");
    pnh_.param("lane_start_extension_m", lane_start_extension_m_, 80.0);
    pnh_.param("lane_start_extension_spacing_m",
               lane_start_extension_spacing_m_, 0.1);
    pnh_.param<std::string>("odom_topic", odom_topic_, "/odom");
    pnh_.param("completion_use_pose_check", completion_use_pose_check_, true);
    pnh_.param("completion_min_active_sec", completion_min_active_sec_, 2.0);
    pnh_.param("completion_timeout_sec", completion_timeout_sec_, 8.0);
    pnh_.param("completion_lateral_d_m", completion_lateral_d_m_, 0.75);
    pnh_.param("completion_confirmation_count", completion_confirmation_count_, 5);
    pnh_.param("completion_aligning_lateral_d_m",
               completion_aligning_lateral_d_m_, 1.35);
    pnh_.param("completion_aligning_yaw_error_rad",
               completion_aligning_yaw_error_rad_, 0.60);
    pnh_.param("completion_failed_source_lateral_d_m",
               completion_failed_source_lateral_d_m_, 0.90);
    pnh_.param("completion_lane_distance_margin_m",
               completion_lane_distance_margin_m_, 0.30);
    pnh_.param("completion_hard_timeout_sec", completion_hard_timeout_sec_, 0.0);
    pnh_.param("accept_source_max_abs_d_m", accept_source_max_abs_d_m_, 4.0);
    pnh_.param("accept_target_max_abs_d_m", accept_target_max_abs_d_m_, 6.0);
    pnh_.param("timeout_off_active_lanes_abs_d_m",
               timeout_off_active_lanes_abs_d_m_,
               6.0);
    pnh_.param("recovery_nearest_lane_max_abs_d_m",
               recovery_nearest_lane_max_abs_d_m_,
               6.0);
    pnh_.param<std::string>("final_waypoint_topic", final_waypoint_topic_,
                            "/final_waypoint");
    pnh_.param("target_path_min_points", target_path_min_points_, 4);
    pnh_.param("path_switch_timeout_sec", path_switch_timeout_sec_, 2.0);
    pnh_.param("motion_start_speed_mps", motion_start_speed_mps_, 0.20);
    pnh_.param("motion_start_displacement_m",
               motion_start_displacement_m_, 0.30);
    pnh_.param("motion_start_timeout_sec", motion_start_timeout_sec_, 0.0);

    target_path_min_points_ = std::max(4, target_path_min_points_);

    loadLanes();

    decision_sub_ = nh_.subscribe("/smpc/decision", 10,
                                  &LaneChangeSupervisor::decisionCallback, this);
    state_sub_ = nh_.subscribe("/gps_state", 10,
                               &LaneChangeSupervisor::stateCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 10,
                              &LaneChangeSupervisor::odomCallback, this);
    final_waypoint_sub_ = nh_.subscribe(
        final_waypoint_topic_, 10,
        &LaneChangeSupervisor::finalWaypointCallback, this);
    path_pub_ = nh_.advertise<std_msgs::Int32>("/path_number", 1, true);
    status_pub_ = nh_.advertise<smpc_lane_change::LaneChangeStatus>(
        "/smpc/lane_change_status", 1, true);
    debug_pub_ = nh_.advertise<std_msgs::String>(
        "/smpc/lane_change_supervisor_debug", 10);
    timer_ = nh_.createTimer(ros::Duration(0.05),
                             &LaneChangeSupervisor::timerCallback, this);
  }

 private:
  bool loadLanes() {
    lanes_.clear();
    if (waypoint_directory_.empty() || !fs::is_directory(waypoint_directory_)) {
      ROS_WARN("[lane_change_supervisor] waypoint_directory invalid: '%s'; "
               "falling back to timer-only completion",
               waypoint_directory_.c_str());
      return false;
    }

    std::vector<fs::path> files;
    for (const auto& entry : fs::directory_iterator(waypoint_directory_)) {
      if (entry.is_regular_file() && entry.path().extension() == ".csv") {
        files.push_back(entry.path());
      }
    }
    std::sort(files.begin(), files.end(), [](const fs::path& a, const fs::path& b) {
      try {
        return std::stoi(a.stem().string()) < std::stoi(b.stem().string());
      } catch (...) {
        return a.string() < b.string();
      }
    });

    for (const auto& file : files) {
      int lane_id = -1;
      try {
        lane_id = std::stoi(file.stem().string());
      } catch (...) {
        continue;
      }
      smpc_lane_change::LanePath lane;
      if (lane.loadCsv(file.string())) {
        if (lane_id > 0) {
          bool curved_enabled = true;
          double offset = 3.5, join_error = 1.5, join_yaw = 0.35;
          ros::param::param<bool>("/local_path_publisher_node/path_switch_target_offset_prelude_enabled_", curved_enabled, true);
          ros::param::param<double>("/local_path_publisher_node/path_switch_lane_offset_m_", offset, 3.5);
          ros::param::param<double>("/local_path_publisher_node/path_switch_target_offset_prelude_max_join_error_m_", join_error, 1.5);
          ros::param::param<double>("/local_path_publisher_node/path_switch_target_offset_prelude_max_join_heading_error_rad_", join_yaw, 0.35);
          const auto previous = lanes_.find(lane_id - 1);
          const bool curved = curved_enabled && previous != lanes_.end() &&
              lane.prependOffsetExtension(previous->second, lane_start_extension_m_,
                                          offset, join_error, join_yaw);
          if (!curved) lane.prependLinearExtension(lane_start_extension_m_,
                                                   lane_start_extension_spacing_m_);
          ROS_INFO("lane %d prelude: %s", lane_id, curved ? "source curve offset" : "linear fallback");
        }
        lanes_[lane_id] = std::move(lane);
      }
    }

    lanes_loaded_ = !lanes_.empty();
    if (lanes_loaded_) {
      ROS_INFO("[lane_change_supervisor] loaded %zu lane CSVs for completion check",
               lanes_.size());
    } else {
      ROS_WARN("[lane_change_supervisor] no lane CSVs loaded; "
               "falling back to timer-only completion");
    }
    return lanes_loaded_;
  }

  const smpc_lane_change::LanePath* laneFor(int lane_id) const {
    const auto it = lanes_.find(lane_id);
    return it == lanes_.end() ? nullptr : &it->second;
  }

  static double yawFromQuaternion(const geometry_msgs::Quaternion& q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static double closestYawError(double yaw, double lane_yaw) {
    const double e0 = std::abs(smpc_lane_change::wrapToPi(yaw - lane_yaw));
    const double e1 = std::abs(smpc_lane_change::wrapToPi(yaw - lane_yaw - M_PI));
    return std::min(e0, e1);
  }

  struct LanePose {
    bool valid{false};
    double d{std::numeric_limits<double>::infinity()};
    double abs_d{std::numeric_limits<double>::infinity()};
    double yaw_error{std::numeric_limits<double>::infinity()};
  };

  struct TimeoutClassification {
    std::string state{"unknown"};
    LanePose source;
    LanePose target;
  };

  struct NearestLanePose {
    bool valid{false};
    int lane_id{0};
    LanePose pose;
  };

  // The path number command is intentionally issued immediately after an
  // accepted decision, including when ego is stopped at the source-path end.
  // Completion timing, however, must not begin until the local-path publisher
  // has acknowledged the target path and ego actually starts progressing.
  enum class ActivationPhase {
    kInactive,
    kPathSwitchRequested,
    kPathSwitched,
    kTargetPathValid,
    kMotionStarted,
  };

  static const char* activationPhaseName(ActivationPhase phase) {
    switch (phase) {
      case ActivationPhase::kInactive:
        return "INACTIVE";
      case ActivationPhase::kPathSwitchRequested:
        return "PATH_SWITCH_REQUESTED";
      case ActivationPhase::kPathSwitched:
        return "PATH_SWITCHED";
      case ActivationPhase::kTargetPathValid:
        return "TARGET_LOCAL_PATH_VALID";
      case ActivationPhase::kMotionStarted:
        return "MOTION_STARTED";
    }
    return "UNKNOWN";
  }

  LanePose projectEgoToLane(int lane_id) const {
    LanePose out;
    const auto* lane = laneFor(lane_id);
    if (!lane || !have_odom_) return out;

    const auto projection = lane->project(ego_x_, ego_y_);
    if (!projection.valid) return out;

    out.valid = true;
    out.d = projection.d;
    out.abs_d = std::abs(projection.d);
    out.yaw_error = closestYawError(ego_yaw_, projection.yaw);
    return out;
  }

  NearestLanePose nearestLaneToEgo() const {
    NearestLanePose best;
    if (!have_odom_) return best;

    for (const auto& lane_pair : lanes_) {
      const int lane_id = lane_pair.first;
      const auto projection = lane_pair.second.project(ego_x_, ego_y_);
      if (!projection.valid) continue;

      LanePose pose;
      pose.valid = true;
      pose.d = projection.d;
      pose.abs_d = std::abs(projection.d);
      pose.yaw_error = closestYawError(ego_yaw_, projection.yaw);

      if (!best.valid || pose.abs_d < best.pose.abs_d) {
        best.valid = true;
        best.lane_id = lane_id;
        best.pose = pose;
      }
    }

    return best;
  }

  static std::string poseText(const char* label, const LanePose& pose) {
    char buf[160];
    if (!pose.valid) {
      std::snprintf(buf, sizeof(buf), "%s=invalid", label);
    } else {
      std::snprintf(buf, sizeof(buf), "%s d=%.3f yaw=%.3f",
                    label, pose.d, pose.yaw_error);
    }
    return std::string(buf);
  }

  bool yawLooksLaneAligned(const TimeoutClassification& cls) const {
    double best_yaw_error = std::numeric_limits<double>::infinity();
    if (cls.source.valid) {
      best_yaw_error = std::min(best_yaw_error, cls.source.yaw_error);
    }
    if (cls.target.valid) {
      best_yaw_error = std::min(best_yaw_error, cls.target.yaw_error);
    }
    return best_yaw_error <= completion_aligning_yaw_error_rad_;
  }

  std::string progressStateWithYaw(const TimeoutClassification& cls,
                                   const std::string& base_state) const {
    return base_state +
           (yawLooksLaneAligned(cls) ? "_yaw_consistent" : "_yaw_unsettled");
  }

  bool activeLaneProjectionsTooFar(const TimeoutClassification& cls) const {
    double best_active_abs_d = std::numeric_limits<double>::infinity();
    if (cls.source.valid) best_active_abs_d = std::min(best_active_abs_d, cls.source.abs_d);
    if (cls.target.valid) best_active_abs_d = std::min(best_active_abs_d, cls.target.abs_d);
    return best_active_abs_d > timeout_off_active_lanes_abs_d_m_;
  }

  TimeoutClassification classifyTimeoutPose() const {
    TimeoutClassification out;
    out.source = projectEgoToLane(source_lane_);
    out.target = projectEgoToLane(target_lane_);

    const bool target_complete = out.target.valid &&
        out.target.abs_d <= completion_lateral_d_m_;
    if (target_complete) {
      out.state = "complete";
      return out;
    }

    const bool target_near = out.target.valid &&
        out.target.abs_d <= completion_aligning_lateral_d_m_;
    if (target_near) {
      out.state =
          out.target.yaw_error <= completion_aligning_yaw_error_rad_
              ? "post_change_aligning"
              : "post_change_near_target_yaw_unsettled";
      return out;
    }

    const bool source_stable = out.source.valid &&
        out.source.abs_d <= completion_failed_source_lateral_d_m_;
    const bool source_closer = out.source.valid &&
        (!out.target.valid ||
         out.source.abs_d + completion_lane_distance_margin_m_ < out.target.abs_d);
    if (source_stable && source_closer) {
      out.state = "failed_source_lane";
      return out;
    }

    if (activeLaneProjectionsTooFar(out)) {
      out.state = "off_active_lanes";
      return out;
    }

    if (out.source.valid && out.target.valid) {
      if (out.target.abs_d + completion_lane_distance_margin_m_ < out.source.abs_d) {
        out.state = progressStateWithYaw(out, "lane_change_in_progress_target_side");
      } else if (out.source.abs_d + completion_lane_distance_margin_m_ < out.target.abs_d) {
        out.state = progressStateWithYaw(out, "lane_change_in_progress_source_side");
      } else {
        out.state = progressStateWithYaw(out, "lane_change_in_progress_between_lanes");
      }
      return out;
    }

    if (out.target.valid) {
      out.state = progressStateWithYaw(out, "lane_change_in_progress_target_only");
    } else if (out.source.valid) {
      out.state = progressStateWithYaw(out, "lane_change_in_progress_source_only");
    }
    return out;
  }

  void publishDebug(const std::string& text) const {
    std_msgs::String msg;
    msg.data = text;
    debug_pub_.publish(msg);
  }

  void setActivationPhase(ActivationPhase phase, const std::string& detail) {
    if (activation_phase_ == phase) return;
    activation_phase_ = phase;
    const std::string text =
        std::string("phase=") + activationPhaseName(phase) + " " + detail;
    publishDebug(text);
    ROS_INFO("[lane_change_supervisor] %s", text.c_str());
  }

  void resetActivationState() {
    activation_phase_ = ActivationPhase::kInactive;
    path_switch_confirmed_ = false;
    target_path_valid_ = false;
    completion_start_ = ros::Time(0);
    completion_clock_started_ = false;
    required_state_callback_seq_ = 0;
    required_final_waypoint_callback_seq_ = 0;
    last_target_path_point_count_ = 0;
    have_motion_origin_ = false;
  }

  bool hasMotionProgress() const {
    const double speed = std::hypot(ego_vx_, ego_vy_);
    if (motion_start_speed_mps_ > 0.0 && speed >= motion_start_speed_mps_) {
      return true;
    }
    if (!have_motion_origin_ || !have_odom_) return false;
    const double dx = ego_x_ - motion_origin_x_;
    const double dy = ego_y_ - motion_origin_y_;
    return motion_start_displacement_m_ > 0.0 &&
           std::hypot(dx, dy) >= motion_start_displacement_m_;
  }

  void maybeStartCompletionClock() {
    if (!active_ || completion_clock_started_) return;
    if (!path_switch_confirmed_ || !target_path_valid_) return;

    if (activation_phase_ != ActivationPhase::kTargetPathValid) {
      setActivationPhase(
          ActivationPhase::kTargetPathValid,
          "target_lane=" + std::to_string(target_lane_) +
              " points=" + std::to_string(last_target_path_point_count_));
    }

    if (!hasMotionProgress()) return;

    completion_start_ = ros::Time::now();
    completion_clock_started_ = true;
    setActivationPhase(ActivationPhase::kMotionStarted,
                       "completion_clock_started");
  }

  void stateCallback(const waypoint_maker::State::ConstPtr& msg) {
    ++state_callback_seq_;
    // /gps_state.lane_number is the active waypoint path number, not a reliable
    // physical lane-completion signal during blending. Keep source_lane_ stable
    // while active; switch current_lane_ only when the completion check passes.
    if (active_ &&
        state_callback_seq_ > required_state_callback_seq_ &&
        msg->lane_number == target_lane_) {
      if (!path_switch_confirmed_) {
        path_switch_confirmed_ = true;
        setActivationPhase(
            ActivationPhase::kPathSwitched,
            "active_path=" + std::to_string(msg->lane_number));
      }
      maybeStartCompletionClock();
    } else if (!active_) {
      current_lane_ = msg->lane_number;
      publishStatus(false);
    }
    have_lane_ = true;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    ego_x_ = msg->pose.pose.position.x;
    ego_y_ = msg->pose.pose.position.y;
    ego_yaw_ = yawFromQuaternion(msg->pose.pose.orientation);
    ego_vx_ = msg->twist.twist.linear.x;
    ego_vy_ = msg->twist.twist.linear.y;
    have_odom_ = true;
    maybeStartCompletionClock();
  }

  void finalWaypointCallback(const waypoint_maker::Lane::ConstPtr& msg) {
    ++final_waypoint_callback_seq_;
    if (!active_ ||
        final_waypoint_callback_seq_ <= required_final_waypoint_callback_seq_ ||
        msg->waypoints.empty()) {
      return;
    }

    const int path_lane = msg->waypoints.front().lane_number;
    if (path_lane != target_lane_ ||
        static_cast<int>(msg->waypoints.size()) < target_path_min_points_) {
      return;
    }

    if (!target_path_valid_) {
      target_path_valid_ = true;
      last_target_path_point_count_ = static_cast<int>(msg->waypoints.size());
      publishDebug("target_local_path_valid lane=" +
                   std::to_string(path_lane) + " points=" +
                   std::to_string(last_target_path_point_count_));
    }
    maybeStartCompletionClock();
  }

  void decisionCallback(const smpc_lane_change::LaneChangeDecision::ConstPtr& msg) {
    if (!have_lane_ || active_) return;
    const bool valid_request = msg->request &&
        msg->mode == smpc_lane_change::LaneChangeDecision::CHANGE_LEFT &&
        msg->current_lane_id == current_lane_ &&
        msg->target_lane_id == current_lane_ + 1;

    if (!valid_request) {
      consecutive_ = 0;
      return;
    }
    if ((ros::Time::now() - last_switch_).toSec() < cooldown_sec_) return;

    ++consecutive_;
    if (consecutive_ < confirmation_count_) return;

    if (completion_use_pose_check_ && lanes_loaded_ && have_odom_) {
      const LanePose source_pose = projectEgoToLane(current_lane_);
      const LanePose target_pose = projectEgoToLane(msg->target_lane_id);
      const bool accept_pose =
          source_pose.valid &&
          target_pose.valid &&
          source_pose.abs_d <= accept_source_max_abs_d_m_ &&
          target_pose.abs_d <= accept_target_max_abs_d_m_;
      if (!accept_pose) {
        consecutive_ = 0;
        ROS_WARN_THROTTLE(
            1.0,
            "[lane_change_supervisor] reject CHANGE_LEFT: ego not near source/target lanes "
            "src lane=%d d=%s target lane=%d d=%s",
            current_lane_,
            source_pose.valid ? std::to_string(source_pose.d).c_str() : "invalid",
            msg->target_lane_id,
            target_pose.valid ? std::to_string(target_pose.d).c_str() : "invalid");
        return;
      }
    }

    source_lane_ = current_lane_;
    target_lane_ = msg->target_lane_id;
    active_ = true;
    completion_consecutive_ = 0;
    last_completion_d_ = std::numeric_limits<double>::infinity();
    last_completion_yaw_error_ = std::numeric_limits<double>::infinity();
    change_start_ = ros::Time::now();
    resetActivationState();
    required_state_callback_seq_ = state_callback_seq_;
    required_final_waypoint_callback_seq_ = final_waypoint_callback_seq_;
    have_motion_origin_ = have_odom_;
    motion_origin_x_ = ego_x_;
    motion_origin_y_ = ego_y_;
    setActivationPhase(ActivationPhase::kPathSwitchRequested,
                       "source=" + std::to_string(source_lane_) +
                           " target=" + std::to_string(target_lane_));

    std_msgs::Int32 path;
    path.data = target_lane_;
    path_pub_.publish(path);
    publishStatus(true);
    // 쿨다운 기준을 '변경 시작'으로 잡으면 변경 자체가 2~3초 걸리므로
    // 끝나기도 전에 쿨다운이 만료된다 - 연속 변경 사이에 인지 여유가
    // 사실상 0 이 된다.
    //   실측 2026-09-14(13:00 백): 1->2 변경이 t=19.8 시작, t=22.8 완료(3.0s).
    //   쿨다운 1.0s 는 t=20.8 에 이미 풀려서, 완료 0.1초 뒤인 t=22.9 에
    //   곧바로 2->3 을 요청했다. 새 옆 차로를 관측할 시간이 없었다.
    // 완료 시각(completeChange)에서 다시 재므로 여기서는 건드리지 않는다.
    consecutive_ = 0;
    ROS_WARN("[lane_change_supervisor] accepted CHANGE_LEFT: path_number %d -> %d (%s)",
             source_lane_, target_lane_, msg->reason.c_str());
  }

  bool poseCompletionReady(double elapsed_sec) {
    if (elapsed_sec < completion_min_active_sec_) {
      completion_consecutive_ = 0;
      return false;
    }

    const auto* target_lane = laneFor(target_lane_);
    if (!target_lane || !have_odom_) {
      completion_consecutive_ = 0;
      return false;
    }

    const auto projection = target_lane->project(ego_x_, ego_y_);
    if (!projection.valid) {
      completion_consecutive_ = 0;
      return false;
    }

    last_completion_d_ = projection.d;
    last_completion_yaw_error_ = closestYawError(ego_yaw_, projection.yaw);
    const bool within_target_lane =
        std::abs(last_completion_d_) <= completion_lateral_d_m_;

    if (within_target_lane) {
      ++completion_consecutive_;
    } else {
      completion_consecutive_ = 0;
    }

    return completion_consecutive_ >= completion_confirmation_count_;
  }

  void finishLaneChange(const std::string& reason, double elapsed_sec, bool forced) {
    active_ = false;
    current_lane_ = target_lane_;
    completion_consecutive_ = 0;
    resetActivationState();
    publishStatus(false);
    // 다음 변경까지의 쿨다운은 '변경이 끝난 시점'부터 잰다. 새 옆 차로를
    // 관측할 시간을 확보하는 것이 목적이라, 시작 시점부터 재면 의미가 없다.
    last_switch_ = ros::Time::now();
    if (forced) {
      ROS_WARN("[lane_change_supervisor] lane change forced complete: lane=%d elapsed=%.2f "
               "reason=%s d=%.3f yaw_err=%.3f",
               current_lane_, elapsed_sec, reason.c_str(),
               last_completion_d_, last_completion_yaw_error_);
    } else {
      ROS_INFO("[lane_change_supervisor] lane change pose complete: lane=%d elapsed=%.2f "
               "reason=%s d=%.3f yaw_err=%.3f",
               current_lane_, elapsed_sec, reason.c_str(),
               last_completion_d_, last_completion_yaw_error_);
    }
  }

  void abortLaneChange(const std::string& reason,
                       double elapsed_sec,
                       const TimeoutClassification& cls,
                       bool recover_to_nearest_lane = false) {
    active_ = false;
    int recovery_lane = source_lane_;
    NearestLanePose nearest;
    if (recover_to_nearest_lane) {
      nearest = nearestLaneToEgo();
      if (nearest.valid &&
          nearest.pose.abs_d <= recovery_nearest_lane_max_abs_d_m_) {
        recovery_lane = nearest.lane_id;
      }
    }
    current_lane_ = recovery_lane;
    completion_consecutive_ = 0;
    consecutive_ = 0;
    resetActivationState();

    std_msgs::Int32 path;
    path.data = recovery_lane;
    path_pub_.publish(path);
    publishStatus(false);
    last_switch_ = ros::Time::now();

    const std::string detail =
        "state=" + cls.state + " " +
        poseText("src", cls.source) + " " +
        poseText("tgt", cls.target) +
        (recover_to_nearest_lane && nearest.valid
             ? " recovery_lane=" + std::to_string(nearest.lane_id) + " " +
                   poseText("nearest", nearest.pose)
             : "");
    publishDebug("abort " + reason + " " + detail);
    ROS_WARN("[lane_change_supervisor] lane change aborted: lane=%d elapsed=%.2f "
             "reason=%s %s",
             current_lane_, elapsed_sec, reason.c_str(), detail.c_str());
  }

  void handleCompletionTimeout(double elapsed) {
    const TimeoutClassification cls = classifyTimeoutPose();
    const std::string detail =
        "state=" + cls.state + " " +
        poseText("src", cls.source) + " " +
        poseText("tgt", cls.target);
    publishDebug("timeout " + detail);

    if (cls.state == "complete") {
      last_completion_d_ = cls.target.d;
      last_completion_yaw_error_ = cls.target.yaw_error;
      finishLaneChange("timeout_pose_complete", elapsed, false);
      return;
    }

    if (cls.state == "failed_source_lane") {
      abortLaneChange("timeout_pose_failed_source_lane", elapsed, cls);
      return;
    }

    if (cls.state == "off_active_lanes") {
      abortLaneChange("timeout_pose_off_active_lanes", elapsed, cls, true);
      return;
    }

    const bool hard_timeout =
        completion_hard_timeout_sec_ > 0.0 &&
        elapsed >= completion_hard_timeout_sec_;
    if (hard_timeout) {
      if (cls.target.valid && (!cls.source.valid || cls.target.abs_d <= cls.source.abs_d)) {
        last_completion_d_ = cls.target.d;
        last_completion_yaw_error_ = cls.target.yaw_error;
        finishLaneChange("hard_timeout_nearest_target", elapsed, true);
      } else {
        abortLaneChange("hard_timeout_nearest_source", elapsed, cls);
      }
      return;
    }

    ROS_WARN_THROTTLE(
        1.0,
        "[lane_change_supervisor] lane change timeout classified as %s; "
        "keeping active. %s",
        cls.state.c_str(), detail.c_str());
    publishStatus(true);
  }

  void timerCallback(const ros::TimerEvent&) {
    if (!active_) return;

    const ros::Time now = ros::Time::now();
    if (!completion_clock_started_) {
      const double activation_elapsed = (now - change_start_).toSec();
      const bool target_path_ready = path_switch_confirmed_ && target_path_valid_;

      // This is deliberately a path-readiness watchdog, not the maneuver
      // completion timeout.  When target path data is unavailable, returning
      // to the source path is safer than leaving MPC on an unusable path.
      if (!target_path_ready && path_switch_timeout_sec_ > 0.0 &&
          activation_elapsed >= path_switch_timeout_sec_) {
        const TimeoutClassification cls = classifyTimeoutPose();
        abortLaneChange("target_path_switch_timeout", activation_elapsed, cls);
        return;
      }

      // A stopped ego is allowed to wait indefinitely by default (0 disables
      // this watchdog).  This avoids treating an intentional standstill at a
      // lane end as a failed lane change before the controller can depart.
      if (target_path_ready && motion_start_timeout_sec_ > 0.0 &&
          activation_elapsed >= motion_start_timeout_sec_) {
        const TimeoutClassification cls = classifyTimeoutPose();
        abortLaneChange("target_path_ready_but_motion_not_started",
                        activation_elapsed, cls);
        return;
      }

      publishStatus(true);
      return;
    }

    const double elapsed = (now - completion_start_).toSec();
    const bool can_pose_check =
        completion_use_pose_check_ && lanes_loaded_ && have_odom_;

    if (can_pose_check && poseCompletionReady(elapsed)) {
      finishLaneChange("target_lane_pose_confirmed", elapsed, false);
      return;
    }

    if (!can_pose_check && elapsed >= lane_change_duration_sec_) {
      finishLaneChange("timer_fallback_no_pose_check", elapsed, false);
      return;
    }

    if (can_pose_check &&
        completion_timeout_sec_ > 0.0 &&
        elapsed >= completion_timeout_sec_) {
      handleCompletionTimeout(elapsed);
      return;
    }

    publishStatus(true);
  }

  void publishStatus(bool active) {
    smpc_lane_change::LaneChangeStatus status;
    status.header.stamp = ros::Time::now();
    status.header.frame_id = "map";
    status.active = active;
    status.source_lane_id = active ? source_lane_ : current_lane_;
    status.target_lane_id = active ? target_lane_ : current_lane_ + 1;
    status.elapsed_sec = active && completion_clock_started_
                             ? (ros::Time::now() - completion_start_).toSec()
                             : 0.0;
    status.expected_duration_sec = lane_change_duration_sec_;
    status_pub_.publish(status);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber decision_sub_, state_sub_, odom_sub_, final_waypoint_sub_;
  ros::Publisher path_pub_, status_pub_, debug_pub_;
  ros::Timer timer_;

  std::map<int, smpc_lane_change::LanePath> lanes_;
  std::string waypoint_directory_;
  double lane_start_extension_m_{80.0};
  double lane_start_extension_spacing_m_{0.1};
  std::string odom_topic_{"/odom"};
  std::string final_waypoint_topic_{"/final_waypoint"};
  bool lanes_loaded_{false};

  bool have_lane_{false};
  bool have_odom_{false};
  bool active_{false};
  int current_lane_{0};
  int source_lane_{0};
  int target_lane_{1};
  int confirmation_count_{3};
  int consecutive_{0};
  int completion_confirmation_count_{5};
  int completion_consecutive_{0};
  double cooldown_sec_{1.0};
  double lane_change_duration_sec_{4.0};
  bool completion_use_pose_check_{true};
  double completion_min_active_sec_{2.0};
  double completion_timeout_sec_{8.0};
  double completion_lateral_d_m_{0.75};
  double completion_aligning_lateral_d_m_{1.35};
  double completion_aligning_yaw_error_rad_{0.60};
  double completion_failed_source_lateral_d_m_{0.90};
  double completion_lane_distance_margin_m_{0.30};
  double completion_hard_timeout_sec_{0.0};
  double accept_source_max_abs_d_m_{4.0};
  double accept_target_max_abs_d_m_{6.0};
  double timeout_off_active_lanes_abs_d_m_{6.0};
  double recovery_nearest_lane_max_abs_d_m_{6.0};
  int target_path_min_points_{4};
  double path_switch_timeout_sec_{2.0};
  double motion_start_speed_mps_{0.20};
  double motion_start_displacement_m_{0.30};
  double motion_start_timeout_sec_{0.0};
  double ego_x_{0.0};
  double ego_y_{0.0};
  double ego_yaw_{0.0};
  double ego_vx_{0.0};
  double ego_vy_{0.0};
  double motion_origin_x_{0.0};
  double motion_origin_y_{0.0};
  bool have_motion_origin_{false};
  uint64_t state_callback_seq_{0};
  uint64_t final_waypoint_callback_seq_{0};
  uint64_t required_state_callback_seq_{0};
  uint64_t required_final_waypoint_callback_seq_{0};
  bool path_switch_confirmed_{false};
  bool target_path_valid_{false};
  int last_target_path_point_count_{0};
  ActivationPhase activation_phase_{ActivationPhase::kInactive};
  double last_completion_d_{std::numeric_limits<double>::infinity()};
  double last_completion_yaw_error_{std::numeric_limits<double>::infinity()};
  ros::Time last_switch_{0.0};
  ros::Time change_start_{0.0};
  ros::Time completion_start_{0.0};
  bool completion_clock_started_{false};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "lane_change_supervisor");
  LaneChangeSupervisor node;
  ros::spin();
  return 0;
}
