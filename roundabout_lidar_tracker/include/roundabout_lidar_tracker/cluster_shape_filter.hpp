#pragma once

#include <vector>

namespace roundabout_lidar_tracker {

struct ClusterPoint3D {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct ClusterShape {
  bool valid = false;
  double center_x = 0.0;
  double center_y = 0.0;
  double center_z = 0.0;
  double size_x = 0.0;
  double size_y = 0.0;
  double height = 0.0;
  double oriented_length = 0.0;
  double oriented_width = 0.0;
  double yaw = 0.0;
  double aspect_ratio = 0.0;
};

struct VehicleShapeFilterConfig {
  double min_length = 0.4;
  double max_length = 8.0;
  double min_width = 0.15;
  double max_width = 6.0;
  double min_height = 0.2;
  double max_height = 4.0;

  // 가드레일/중앙분리대처럼 길고 낮은 선형 클러스터를 제거한다.
  bool reject_low_linear = true;
  double linear_min_length = 2.0;
  double linear_max_width = 0.45;
  double linear_max_height = 1.3;
  double linear_min_aspect_ratio = 6.0;

  // ★ 지붕만 보이는 차량(2026-09-29, 12:50 백).
  //   중앙섬에 가려 상류 차량이 라이다 한 줄(지붕)로만 찍히면 높이 폭이
  //   0.03 m 라 min_height 에서 탈락했다(-42 m 부터 보였는데 -25 m 에서야 추적).
  //   높이 폭 대신 '지면 위 윗면 높이'로 차량 지붕인지 본다. 원형 차로 띠
  //   (inner~outer) 안의 클러스터만 대상. 정지 시설물은 미션의 이동 확인이 거른다.
  bool thin_top_enabled = false;
  double thin_top_min_top = 0.9;      // 지면 위 윗면 최소 높이 [m]
  double thin_top_max_top = 2.2;      // 지면 위 윗면 최대 높이 [m]
  double thin_top_min_length = 0.8;   // 최소 길이 [m]
  int thin_top_min_points = 5;        // 최소 점 수(복셀 후)
};

ClusterShape measureClusterShape(const std::vector<ClusterPoint3D>& points);

bool isVehicleShape(const ClusterShape& shape,
                    const VehicleShapeFilterConfig& config);

// 높이 폭이 min_height 미만이지만 차량 지붕으로 볼 수 있는지.
//   top_above_ground: 클러스터 최고점의 지면 위 높이 [m]
bool isThinTopVehicle(const ClusterShape& shape, double top_above_ground,
                      int point_count, const VehicleShapeFilterConfig& config);

}  // namespace roundabout_lidar_tracker
