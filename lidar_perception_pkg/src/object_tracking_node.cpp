#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <non_gps_msgs/TrackedObject.h>
#include <non_gps_msgs/TrackedObjectArray.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class ObjectTrackingNode
{
public:
  ObjectTrackingNode() : pnh_("~"), tf_listener_(tf_buffer_)
  {
    pnh_.param<std::string>("odom_frame", odom_frame_, "odom");
    pnh_.param<std::string>("input_topic", input_topic_, "/cluster_boxes");
    pnh_.param<std::string>("output_topic", output_topic_, "/lidar_perception/tracked_objects");
    pnh_.param<std::string>("visualization_topic", visualization_topic_,
                            "/lidar_perception/tracked_boxes");
    pnh_.param<std::string>("ego_status_topic", ego_status_topic_, "/Competition_topic");
    pnh_.param("association_max_distance", association_max_distance_, 2.5);
    pnh_.param("min_size_similarity", min_size_similarity_, 0.25);
    pnh_.param("iou_weight", iou_weight_, 1.0);
    pnh_.param("size_weight", size_weight_, 0.5);
    pnh_.param("velocity_alpha", velocity_alpha_, 0.5);
    pnh_.param("static_speed_threshold", static_speed_threshold_, 0.2);
    pnh_.param("dynamic_speed_threshold", dynamic_speed_threshold_, 0.5);
    pnh_.param("motion_confirm_frames", motion_confirm_frames_, 3);
    pnh_.param("track_timeout", track_timeout_, 0.5);
    pnh_.param("use_ego_motion_compensation", use_ego_motion_compensation_, true);
    pnh_.param("allow_static_without_ego", allow_static_without_ego_, true);
    pnh_.param("wheel_base", wheel_base_, 2.8);
    pnh_.param("ego_status_timeout", ego_status_timeout_, 0.5);
    pnh_.param("steer_angle_scale", steer_angle_scale_, 1.0);
    pnh_.param("ego_speed_scale", ego_speed_scale_, 1.0);
    pnh_.param("yaw_rate_is_deg", yaw_rate_is_deg_, false); // Usually rad/s in ROS
    pnh_.param("ego_buffer_duration", ego_buffer_duration_, 2.0);
    pnh_.param("velocity_window_size", velocity_window_size_, 5);
    pnh_.param("dynamic_confirm_frames", dynamic_confirm_frames_, motion_confirm_frames_);
    pnh_.param("static_confirm_frames", static_confirm_frames_, motion_confirm_frames_);

    tracked_pub_ = nh_.advertise<non_gps_msgs::TrackedObjectArray>(output_topic_, 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(visualization_topic_, 1);
    sub_ = nh_.subscribe(input_topic_, 1, &ObjectTrackingNode::callback, this);
    ego_sub_ = nh_.subscribe(ego_status_topic_, 50, &ObjectTrackingNode::egoStatusCallback, this);
  }

private:
  struct Detection
  {
    geometry_msgs::Point position;
    geometry_msgs::Vector3 size;
  };

  struct Track
  {
    int32_t id = -1;
    geometry_msgs::Point position;
    geometry_msgs::Vector3 size;
    geometry_msgs::Vector3 velocity;
    ros::Time last_seen;
    uint8_t motion_type = non_gps_msgs::TrackedObject::UNKNOWN;
    int static_votes = 0;
    int dynamic_votes = 0;
    int hits = 1;
    float confidence = 0.0f;
    std::deque<geometry_msgs::Vector3> velocity_history;
  };

  struct EgoState
  {
    ros::Time stamp;
    double speed = 0.0;
    double yaw_rate = 0.0;
    bool valid = false;
  };

  struct EgoMotion
  {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    bool valid = false;
  };

  struct AssociationCandidate
  {
    int32_t track_id = -1;
    size_t detection_index = 0;
    double score = std::numeric_limits<double>::infinity();
  };

  static double centerDistance2d(const geometry_msgs::Point &a,
                                 const geometry_msgs::Point &b)
  {
    return std::hypot(a.x - b.x, a.y - b.y);
  }

  static double dimensionSimilarity(double a, double b)
  {
    a = std::max(a, 0.01);
    b = std::max(b, 0.01);
    return std::min(a, b) / std::max(a, b);
  }

  static double sizeSimilarity(const geometry_msgs::Vector3 &a,
                               const geometry_msgs::Vector3 &b)
  {
    return (dimensionSimilarity(a.x, b.x) +
            dimensionSimilarity(a.y, b.y) +
            dimensionSimilarity(a.z, b.z)) / 3.0;
  }

  static double bevIou(const geometry_msgs::Point &a_position,
                       const geometry_msgs::Vector3 &a_size,
                       const geometry_msgs::Point &b_position,
                       const geometry_msgs::Vector3 &b_size)
  {
    const double a_min_x = a_position.x - a_size.x * 0.5;
    const double a_max_x = a_position.x + a_size.x * 0.5;
    const double a_min_y = a_position.y - a_size.y * 0.5;
    const double a_max_y = a_position.y + a_size.y * 0.5;
    const double b_min_x = b_position.x - b_size.x * 0.5;
    const double b_max_x = b_position.x + b_size.x * 0.5;
    const double b_min_y = b_position.y - b_size.y * 0.5;
    const double b_max_y = b_position.y + b_size.y * 0.5;

    const double intersection_x = std::max(0.0, std::min(a_max_x, b_max_x) -
                                                  std::max(a_min_x, b_min_x));
    const double intersection_y = std::max(0.0, std::min(a_max_y, b_max_y) -
                                                  std::max(a_min_y, b_min_y));
    const double intersection = intersection_x * intersection_y;
    const double a_area = std::max(a_size.x, 0.01) * std::max(a_size.y, 0.01);
    const double b_area = std::max(b_size.x, 0.01) * std::max(b_size.y, 0.01);
    const double union_area = a_area + b_area - intersection;
    return union_area > 0.0 ? intersection / union_area : 0.0;
  }

  double associationScore(const Track &track,
                          const geometry_msgs::Point &predicted_position,
                          const Detection &detection) const
  {
    const double distance = centerDistance2d(predicted_position, detection.position);
    const double similarity = sizeSimilarity(track.size, detection.size);
    if (distance > association_max_distance_ || similarity < min_size_similarity_) {
      return std::numeric_limits<double>::infinity();
    }
    const double iou = bevIou(predicted_position, track.size,
                              detection.position, detection.size);
    return distance + iou_weight_ * (1.0 - iou) +
           size_weight_ * (1.0 - similarity);
  }

  void updateMotionState(Track &track, double speed)
  {
    if (speed >= dynamic_speed_threshold_) {
      track.dynamic_votes = std::min(track.dynamic_votes + 1, dynamic_confirm_frames_ * 2);
      track.static_votes = std::max(track.static_votes - 1, 0);
    } else if (speed <= static_speed_threshold_) {
      track.static_votes = std::min(track.static_votes + 1, static_confirm_frames_ * 2);
      track.dynamic_votes = std::max(track.dynamic_votes - 1, 0);
    } else {
      track.static_votes = std::max(track.static_votes - 1, 0);
      track.dynamic_votes = std::max(track.dynamic_votes - 1, 0);
    }

    const int dynamic_required = std::max(dynamic_confirm_frames_, 1);
    const int static_required = std::max(static_confirm_frames_, 1);
    if (track.dynamic_votes >= dynamic_required) {
      track.motion_type = non_gps_msgs::TrackedObject::DYNAMIC;
      track.confidence = std::min(1.0f,
          static_cast<float>(track.dynamic_votes) / static_cast<float>(dynamic_required * 2));
    } else if (track.static_votes >= static_required) {
      track.motion_type = non_gps_msgs::TrackedObject::STATIC;
      track.confidence = std::min(1.0f,
          static_cast<float>(track.static_votes) / static_cast<float>(static_required * 2));
    } else if (track.motion_type == non_gps_msgs::TrackedObject::DYNAMIC &&
               track.dynamic_votes > 0) {
      track.confidence = static_cast<float>(track.dynamic_votes) /
                         static_cast<float>(dynamic_required);
    } else if (track.motion_type == non_gps_msgs::TrackedObject::STATIC &&
               track.static_votes > 0) {
      track.confidence = static_cast<float>(track.static_votes) /
                         static_cast<float>(static_required);
    } else {
      track.motion_type = non_gps_msgs::TrackedObject::UNKNOWN;
      track.confidence = 0.0f;
    }
  }

  void updateStaticStateWithoutEgo(Track &track, double apparent_speed)
  {
    if (allow_static_without_ego_ && apparent_speed <= static_speed_threshold_) {
      track.static_votes = std::min(track.static_votes + 1, static_confirm_frames_ * 2);
      track.dynamic_votes = 0;
      const int required = std::max(static_confirm_frames_, 1);
      if (track.static_votes >= required) {
        track.motion_type = non_gps_msgs::TrackedObject::STATIC;
        track.confidence = std::min(
            1.0f, static_cast<float>(track.static_votes) / static_cast<float>(required * 2));
      } else {
        track.motion_type = non_gps_msgs::TrackedObject::UNKNOWN;
        track.confidence = 0.0f;
      }
      return;
    }

    track.motion_type = non_gps_msgs::TrackedObject::UNKNOWN;
    track.confidence = 0.0f;
    track.static_votes = 0;
    track.dynamic_votes = 0;
  }

  static double degToRad(double deg)
  {
    return deg * M_PI / 180.0;
  }

  void egoStatusCallback(const morai_msgs::EgoVehicleStatus::ConstPtr &msg)
  {
    EgoState state;
    state.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    state.speed = msg->velocity.x * ego_speed_scale_;

    double steer_deg = msg->front_steer_angle;
    if (std::abs(steer_deg) < 1e-3) {
      steer_deg = msg->wheel_angle;
    }
    const double steer_rad = degToRad(steer_deg * steer_angle_scale_);
    const double safe_wheel_base = std::max(std::abs(wheel_base_), 0.1);
    const double bicycle_yaw_rate = state.speed / safe_wheel_base * std::tan(steer_rad);
    
    double reported_yaw_rate = msg->angular_velocity.z;
    if (yaw_rate_is_deg_) {
      reported_yaw_rate = degToRad(reported_yaw_rate);
    }
    
    state.yaw_rate = std::abs(reported_yaw_rate) > 1e-3 ? reported_yaw_rate
                                                        : bicycle_yaw_rate;
    state.valid = std::isfinite(state.speed) && std::isfinite(state.yaw_rate);
    if (!state.valid) return;

    if (!ego_history_.empty() && state.stamp < ego_history_.back().stamp) {
      const auto position = std::lower_bound(
          ego_history_.begin(), ego_history_.end(), state.stamp,
          [](const EgoState &sample, const ros::Time &stamp) {
            return sample.stamp < stamp;
          });
      ego_history_.insert(position, state);
    } else if (!ego_history_.empty() && state.stamp == ego_history_.back().stamp) {
      ego_history_.back() = state;
    } else {
      ego_history_.push_back(state);
    }

    const ros::Time newest = ego_history_.back().stamp;
    while (!ego_history_.empty() &&
           (newest - ego_history_.front().stamp).toSec() > ego_buffer_duration_) {
      ego_history_.pop_front();
    }
  }

  bool egoStateAt(const ros::Time &stamp, EgoState &state) const
  {
    if (ego_history_.empty()) return false;

    if (stamp <= ego_history_.front().stamp) {
      if ((ego_history_.front().stamp - stamp).toSec() > ego_status_timeout_) return false;
      state = ego_history_.front();
      state.stamp = stamp;
      return true;
    }
    if (stamp >= ego_history_.back().stamp) {
      if ((stamp - ego_history_.back().stamp).toSec() > ego_status_timeout_) return false;
      state = ego_history_.back();
      state.stamp = stamp;
      return true;
    }

    const auto upper = std::lower_bound(
        ego_history_.begin(), ego_history_.end(), stamp,
        [](const EgoState &sample, const ros::Time &time) {
          return sample.stamp < time;
        });
    const auto lower = std::prev(upper);
    const double interval = (upper->stamp - lower->stamp).toSec();
    if (interval <= 0.0 || interval > ego_status_timeout_ * 2.0) return false;

    const double ratio = (stamp - lower->stamp).toSec() / interval;
    state.stamp = stamp;
    state.speed = lower->speed + ratio * (upper->speed - lower->speed);
    state.yaw_rate = lower->yaw_rate + ratio * (upper->yaw_rate - lower->yaw_rate);
    state.valid = true;
    return true;
  }

  EgoMotion egoMotionBetween(const ros::Time &from, const ros::Time &to) const
  {
    EgoMotion motion;
    if (to <= from) return motion;
    if (!use_ego_motion_compensation_) {
      motion.valid = true;
      return motion;
    }

    EgoState start, finish;
    if (!egoStateAt(from, start) || !egoStateAt(to, finish)) return motion;

    std::vector<EgoState> samples;
    samples.push_back(start);
    for (const EgoState &sample : ego_history_) {
      if (sample.stamp > from && sample.stamp < to) samples.push_back(sample);
    }
    samples.push_back(finish);

    for (size_t i = 1; i < samples.size(); ++i) {
      const double dt = (samples[i].stamp - samples[i - 1].stamp).toSec();
      if (dt <= 0.0) continue;
      const double speed = 0.5 * (samples[i - 1].speed + samples[i].speed);
      const double yaw_rate = 0.5 * (samples[i - 1].yaw_rate + samples[i].yaw_rate);
      const double mid_yaw = motion.yaw + 0.5 * yaw_rate * dt;
      motion.x += speed * std::cos(mid_yaw) * dt;
      motion.y += speed * std::sin(mid_yaw) * dt;
      motion.yaw += yaw_rate * dt;
    }
    motion.valid = true;
    return motion;
  }

  static geometry_msgs::Point transformToCurrentFrame(const geometry_msgs::Point &point,
                                                       const EgoMotion &motion)
  {
    geometry_msgs::Point transformed = point;
    if (!motion.valid) return transformed;
    const double shifted_x = point.x - motion.x;
    const double shifted_y = point.y - motion.y;
    const double cos_yaw = std::cos(motion.yaw);
    const double sin_yaw = std::sin(motion.yaw);
    transformed.x = cos_yaw * shifted_x + sin_yaw * shifted_y;
    transformed.y = -sin_yaw * shifted_x + cos_yaw * shifted_y;
    return transformed;
  }

  geometry_msgs::Point predictedPosition(const Track &track,
                                         const ros::Time &stamp,
                                         const EgoMotion &motion) const
  {
    geometry_msgs::Point predicted = track.position;
    const double dt = std::max(0.0, (stamp - track.last_seen).toSec());
    if (motion.valid) {
      predicted.x += track.velocity.x * dt;
      predicted.y += track.velocity.y * dt;
      return transformToCurrentFrame(predicted, motion);
    }
    return predicted;
  }

  static double median(std::vector<double> values)
  {
    if (values.empty()) return 0.0;
    const size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    if (values.size() % 2 == 1) return values[middle];
    const double upper = values[middle];
    std::nth_element(values.begin(), values.begin() + middle - 1, values.begin() + middle);
    return 0.5 * (values[middle - 1] + upper);
  }

  static void rotateVelocity(geometry_msgs::Vector3 &velocity, double yaw)
  {
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const double x = cos_yaw * velocity.x + sin_yaw * velocity.y;
    velocity.y = -sin_yaw * velocity.x + cos_yaw * velocity.y;
    velocity.x = x;
  }

  void updateTrack(Track &track, const Detection &detection,
                   const ros::Time &stamp, const EgoMotion &motion)
  {
    const double dt = (stamp - track.last_seen).toSec();
    if (dt > 0.05) {
      const geometry_msgs::Point expected_position = motion.valid
          ? transformToCurrentFrame(track.position, motion)
          : track.position;
          
      geometry_msgs::Vector3 instantaneous;
      instantaneous.x = (detection.position.x - expected_position.x) / dt;
      instantaneous.y = (detection.position.y - expected_position.y) / dt;
      instantaneous.z = (detection.position.z - expected_position.z) / dt;

      if (motion.valid) {
        rotateVelocity(track.velocity, motion.yaw);
        for (geometry_msgs::Vector3 &sample : track.velocity_history) {
          rotateVelocity(sample, motion.yaw);
        }
      }
      track.velocity_history.push_back(instantaneous);
      while (track.velocity_history.size() > static_cast<size_t>(std::max(velocity_window_size_, 1))) {
        track.velocity_history.pop_front();
      }

      std::vector<double> x_values, y_values, z_values;
      x_values.reserve(track.velocity_history.size());
      y_values.reserve(track.velocity_history.size());
      z_values.reserve(track.velocity_history.size());
      for (const geometry_msgs::Vector3 &sample : track.velocity_history) {
        x_values.push_back(sample.x);
        y_values.push_back(sample.y);
        z_values.push_back(sample.z);
      }
      geometry_msgs::Vector3 filtered;
      filtered.x = median(x_values);
      filtered.y = median(y_values);
      filtered.z = median(z_values);

      const double alpha = track.velocity_history.size() == 1
                               ? 1.0
                               : std::max(0.0, std::min(1.0, velocity_alpha_));
      track.velocity.x = alpha * filtered.x + (1.0 - alpha) * track.velocity.x;
      track.velocity.y = alpha * filtered.y + (1.0 - alpha) * track.velocity.y;
      track.velocity.z = alpha * filtered.z + (1.0 - alpha) * track.velocity.z;
      const double speed = std::hypot(track.velocity.x, track.velocity.y);
      if (motion.valid || !use_ego_motion_compensation_) {
        updateMotionState(track, speed);
      } else {
        updateStaticStateWithoutEgo(track, speed);
      }
    }
    track.position = detection.position;
    track.size = detection.size;
    track.last_seen = stamp;
    track.hits++;
  }

  Track createTrack(const Detection &detection, const ros::Time &stamp)
  {
    Track track;
    track.id = next_track_id_++;
    track.position = detection.position;
    track.size = detection.size;
    track.last_seen = stamp;
    return track;
  }

  // detectionsFromMarkers removed, logic moved to callback.

  void publish(const std_msgs::Header &header, const ros::Time &now)
  {
    non_gps_msgs::TrackedObjectArray objects;
    objects.header = header;
    objects.header.stamp = now;

    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clear;
    clear.header = objects.header;
    clear.ns = "tracked_boxes";
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);

    for (const auto &item : tracks_) {
      const Track &track = item.second;
      if ((now - track.last_seen).toSec() > track_timeout_) continue;

      non_gps_msgs::TrackedObject object;
      object.id = track.id;
      object.position = track.position;
      object.size = track.size;
      object.velocity = track.velocity;
      object.motion_type = track.motion_type;
      object.confidence = track.confidence;
      objects.objects.push_back(object);

      visualization_msgs::Marker box;
      box.header = objects.header;
      box.ns = "tracked_boxes";
      box.id = track.id;
      box.type = visualization_msgs::Marker::CUBE;
      box.action = visualization_msgs::Marker::ADD;
      box.pose.position = track.position;
      box.pose.orientation.w = 1.0;
      // Make the markers larger and more visible in RViz
      box.scale.x = track.size.x + 0.5;
      box.scale.y = track.size.y + 0.5;
      box.scale.z = std::max(track.size.z, 1.0);
      if (track.motion_type == non_gps_msgs::TrackedObject::STATIC) {
        box.color.a = 0.55;
        box.color.g = 1.0;
      } else if (track.motion_type == non_gps_msgs::TrackedObject::DYNAMIC) {
        box.color.a = 0.55;
        box.color.r = 1.0;
      } else {
        box.color.a = 0.0; // Hide UNKNOWN state (yellowish boxes)
      }
      box.lifetime = ros::Duration(track_timeout_);
      markers.markers.push_back(box);

      visualization_msgs::Marker label;
      label.header = objects.header;
      label.ns = "tracked_box_labels";
      label.id = track.id;
      label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::Marker::ADD;
      label.pose.position = track.position;
      label.pose.position.z += track.size.z * 0.5 + 0.3;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.35;
      label.color.a = 1.0;
      label.color.r = label.color.g = label.color.b = 1.0;
      const char *state = track.motion_type == non_gps_msgs::TrackedObject::STATIC ? "STATIC" :
                          track.motion_type == non_gps_msgs::TrackedObject::DYNAMIC ? "DYNAMIC" : "UNKNOWN";
      double speed_kmh = std::hypot(track.velocity.x, track.velocity.y) * 3.6;
      label.text = "ID " + std::to_string(track.id) + " " + state +
                   " " + std::to_string(speed_kmh).substr(0, 4) + "km/h";
      label.lifetime = ros::Duration(track_timeout_);
      markers.markers.push_back(label);
    }
    tracked_pub_.publish(objects);
    marker_pub_.publish(markers);
  }

  void callback(const visualization_msgs::MarkerArray::ConstPtr &msg)
  {
    if (msg->markers.empty()) return;
    std_msgs::Header header = msg->markers[0].header;
    if (header.frame_id.empty()) header.frame_id = "velodyne";

    geometry_msgs::TransformStamped transform;
    ros::Time tf_time = header.stamp.isZero() ? ros::Time::now() : header.stamp;
    try {
      transform = tf_buffer_.lookupTransform(odom_frame_, header.frame_id, tf_time, ros::Duration(0.05));
    } catch (tf2::TransformException &ex) {
      ROS_WARN_THROTTLE(1.0, "TF error in object tracking: %s", ex.what());
      return;
    }

    std::vector<Detection> detections;
    for (const auto &marker : msg->markers) {
      if (marker.action != visualization_msgs::Marker::ADD || marker.type != visualization_msgs::Marker::CUBE) continue;
      Detection detection;
      geometry_msgs::PointStamped pt_in, pt_out;
      
      pt_in.point = marker.pose.position;
      tf2::doTransform(pt_in, pt_out, transform);
      detection.position = pt_out.point;

      detection.size = marker.scale;
      detections.push_back(detection);
    }
    header.frame_id = odom_frame_;

    const ros::Time now = tf_time;

    std::vector<bool> detection_used(detections.size(), false);
    std::unordered_map<int32_t, bool> track_used;
    std::unordered_map<int32_t, EgoMotion> track_motions;
    std::vector<int32_t> track_ids;
    track_ids.reserve(tracks_.size());
    for (const auto &item : tracks_) track_ids.push_back(item.first);

    std::vector<AssociationCandidate> candidates;
    for (const int32_t track_id : track_ids) {
      const Track &track = tracks_.at(track_id);
      const EgoMotion motion = egoMotionBetween(track.last_seen, now);
      track_motions[track_id] = motion;
      const geometry_msgs::Point predicted = predictedPosition(track, now, motion);
      for (size_t i = 0; i < detections.size(); ++i) {
        const double score = associationScore(track, predicted, detections[i]);
        if (std::isfinite(score)) {
          AssociationCandidate candidate;
          candidate.track_id = track_id;
          candidate.detection_index = i;
          candidate.score = score;
          candidates.push_back(candidate);
        }
      }
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const AssociationCandidate &a, const AssociationCandidate &b) {
                return a.score < b.score;
              });
    for (const AssociationCandidate &candidate : candidates) {
      if (track_used[candidate.track_id] || detection_used[candidate.detection_index]) continue;
      Track &track = tracks_.at(candidate.track_id);
      updateTrack(track, detections[candidate.detection_index], now,
                  track_motions.at(candidate.track_id));
      track_used[candidate.track_id] = true;
      detection_used[candidate.detection_index] = true;
    }

    for (size_t i = 0; i < detections.size(); ++i) {
      if (!detection_used[i]) {
        Track track = createTrack(detections[i], now);
        tracks_[track.id] = track;
      }
    }

    for (auto it = tracks_.begin(); it != tracks_.end();) {
      if ((now - it->second.last_seen).toSec() > track_timeout_) {
        it = tracks_.erase(it);
      } else {
        ++it;
      }
    }
    publish(header, now);
  }

  ros::NodeHandle nh_, pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber sub_, ego_sub_;
  ros::Publisher tracked_pub_, marker_pub_;
  std::string input_topic_, output_topic_, visualization_topic_, odom_frame_, ego_status_topic_;
  double association_max_distance_, min_size_similarity_, iou_weight_, size_weight_;
  double velocity_alpha_, static_speed_threshold_, dynamic_speed_threshold_, track_timeout_;
  double wheel_base_, ego_status_timeout_, steer_angle_scale_, ego_buffer_duration_;
  double ego_speed_scale_;
  bool use_ego_motion_compensation_, allow_static_without_ego_, yaw_rate_is_deg_;
  int motion_confirm_frames_, dynamic_confirm_frames_, static_confirm_frames_;
  int velocity_window_size_;
  int32_t next_track_id_ = 0;
  std::unordered_map<int32_t, Track> tracks_;
  std::deque<EgoState> ego_history_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_tracking_node");
  ObjectTrackingNode node;
  ros::spin();
  return 0;
}
