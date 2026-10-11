#pragma once

#include <cstdint>
#include <deque>
#include <limits>
#include <vector>

namespace roundabout_lidar_tracker {

struct Detection2D {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double size_x = 0.0;
  double size_y = 0.0;
  double size_z = 0.0;
  double confidence = 1.0;
  // 링 진행 방향으로 중심 대비 클러스터 뒤끝/앞끝 원호 오프셋 [m] (NaN = 미제공 -> 중심만 사용).
  double arc_lo = std::numeric_limits<double>::quiet_NaN();
  double arc_hi = std::numeric_limits<double>::quiet_NaN();
};

struct Track2D {
  int id = -1;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double velocity_x = 0.0;
  double velocity_y = 0.0;
  double size_x = 0.0;
  double size_y = 0.0;
  double size_z = 0.0;
  double confidence = 0.0;
  std::uint32_t hit_count = 0U;
  bool velocity_valid = false;
  bool has_moved = false;
  double measurement_age = 0.0;  // RViz prediction diagnostics, not a new measurement.
};

struct TrackerConfig {
  double association_distance = 3.0;
  double position_alpha = 0.75;
  double velocity_alpha = 0.4;
  bool circular_velocity_enabled = false;
  double circular_center_x = 0.0;
  double circular_center_y = 0.0;
  int velocity_window_size = 7;
  double velocity_min_span = 0.12;
  bool robust_circular_velocity = false;
  double velocity_window_seconds = 1.8;
  double velocity_pair_min_span = 0.6;
  double velocity_extent_tolerance = 0.5;  // Comparable visible body extent [m].
  double velocity_hold_seconds = 0.6; // Brief face-change gap for already moving tracks.
  double size_alpha = 0.5;
  double confirmed_hold_seconds = 0.0;  // Disabled by default for other users.
  double retained_confidence_floor = 0.45;
  double confirmed_size_shrink_alpha = 0.02;
  double expiry_guard_seconds = 1.0;
  double expiry_guard_conflict_angle = std::numeric_limits<double>::quiet_NaN();
  bool counter_clockwise = true;
  double stale_seconds = 0.6;
  double max_prediction_seconds = 0.3;
  double max_object_speed = 30.0;
  int min_confirm_hits = 3;
  int min_velocity_hits = 4;
  double moving_speed_threshold = 0.7;
  double moving_distance_threshold = 1.0;
  int moving_confirm_hits = 3;
  // 2026-10-06 차량 증명: 순간 속도/원점 거리 대신, 원시 검출의 링 진행 거리로 '움직이는 차량'을 확정한다.
  //   창(proof_window_seconds) 안에서 진행 방향 이동 >= proof_progress, 역행 <= proof_reversal,
  //   반경 흔들림 <= proof_radial_ratio * 진행. 확정 뒤 revoke_reversal 넘게 역행하면 취소(끌려갔다 돌아온 정지물).
  bool vehicle_proof_enabled = false;
  double proof_window_seconds = 2.0;
  double proof_progress = 2.0;
  double proof_reversal = 0.5;
  double proof_radial_ratio = 0.6;
  int proof_min_samples = 4;
  double revoke_reversal = 1.0;
};

class CentroidTracker {
 public:
  explicit CentroidTracker(const TrackerConfig& config);

  void setConfig(const TrackerConfig& config);
  void update(const std::vector<Detection2D>& detections, double now);
  std::vector<Track2D> tracks(double now) const;
  void reset();
  bool occlusionUnresolved(double now) const;

 private:
  struct AngleSample {
    double time = 0.0;
    double angle = 0.0;
    double visible_extent = 0.0;
  };

  struct TrackState {
    int id = -1;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double velocity_x = 0.0;
    double velocity_y = 0.0;
    double angular_velocity = 0.0;
    double size_x = 0.0;
    double size_y = 0.0;
    double size_z = 0.0;
    double observed_body_length = 0.0;
    double confidence = 0.0;
    double origin_x = 0.0;
    double origin_y = 0.0;
    double last_detection_x = 0.0;
    double last_detection_y = 0.0;
    double last_update = 0.0;
    double last_measurement = 0.0;
    double last_velocity_evidence = -std::numeric_limits<double>::infinity();
    std::uint32_t hits = 0U;
    int moving_hits = 0;
    bool has_moved = false;
    bool velocity_estimate_valid = false;
    bool has_velocity_evidence = false;
    std::deque<AngleSample> angle_history;
    struct ProgressSample { double time; double arc; double radius; double lo; double hi; };
    std::deque<ProgressSample> progress_history;  // 원시 검출의 진행 방향 원호 위치 [m]
    double proof_arc_max = -std::numeric_limits<double>::infinity();
    bool proven = false;
  };
  void updateVehicleProof(TrackState& track, const Detection2D& raw, double now) const;

  bool retainable(const TrackState& track) const;
  double lifetime(const TrackState& track) const;
  double last_confirmed_expiry_ = -1.0;
  double last_frame_time_ = -std::numeric_limits<double>::infinity();
  TrackerConfig config_;
  std::vector<TrackState> tracks_;
  int next_id_ = 1;
};

}  // namespace roundabout_lidar_tracker
