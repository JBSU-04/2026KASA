#include <cmath>

#include <gtest/gtest.h>

#include "roundabout_lidar_tracker/cluster_shape_filter.hpp"
#include "roundabout_lidar_tracker/radial_roi.hpp"

namespace roundabout_lidar_tracker {
namespace {

TEST(RadialRoi, RecordedSignsAreOutsideWhileVehicleCentersRemainInside) {
  RadialRoi roi;
  roi.center_x = 302492.838; roi.center_y = 4124488.413;
  roi.inner_radius = 15.8; roi.outer_radius = 20.5;
  EXPECT_FALSE(roi.contains(302507.497, 4124489.373)); // 21:27:56 ID10
  EXPECT_FALSE(roi.contains(302507.486, 4124489.053)); // 21:28:52 ID11
  for (double radius : {16.2, 17.0, 18.338, 19.7})
    EXPECT_TRUE(roi.contains(roi.center_x + radius, roi.center_y));
  EXPECT_FALSE(roi.contains(roi.center_x + 21.0, roi.center_y));
}

TEST(RadialRoi, PointPaddingPreservesVehicleSidesWithoutAdmittingTheirCenters) {
  RadialRoi roi;
  roi.inner_radius = 15.8; roi.outer_radius = 20.5;
  EXPECT_TRUE(roi.contains(15.0, 0.0, 1.2));
  EXPECT_FALSE(roi.contains(15.0, 0.0));
  EXPECT_TRUE(roi.contains(21.5, 0.0, 1.2));
  EXPECT_FALSE(roi.contains(21.5, 0.0));
}

std::vector<ClusterPoint3D> boxPoints(double length, double width,
                                     double height, double yaw) {
  std::vector<ClusterPoint3D> points;
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  for (int ix = -5; ix <= 5; ++ix) {
    for (int iy = -3; iy <= 3; ++iy) {
      for (int iz = 0; iz <= 2; ++iz) {
        const double local_x = 0.5 * length * ix / 5.0;
        const double local_y = 0.5 * width * iy / 3.0;
        points.push_back({cosine * local_x - sine * local_y,
                          sine * local_x + cosine * local_y,
                          0.5 * height * iz});
      }
    }
  }
  return points;
}

VehicleShapeFilterConfig filterConfig() {
  VehicleShapeFilterConfig config;
  config.min_length = 1.0;
  config.max_length = 6.5;
  config.min_width = 0.4;
  config.max_width = 3.5;
  config.min_height = 0.45;
  config.max_height = 3.0;
  config.linear_min_length = 2.0;
  config.linear_max_width = 0.45;
  config.linear_max_height = 1.3;
  config.linear_min_aspect_ratio = 6.0;
  return config;
}

TEST(ClusterShapeFilter, AcceptsRotatedPassengerVehicle) {
  const ClusterShape shape =
      measureClusterShape(boxPoints(4.7, 1.9, 1.5, 0.7));
  EXPECT_TRUE(shape.valid);
  EXPECT_NEAR(shape.oriented_length, 4.7, 0.1);
  EXPECT_NEAR(shape.oriented_width, 1.9, 0.1);
  EXPECT_TRUE(isVehicleShape(shape, filterConfig()));
}

TEST(ClusterShapeFilter, RejectsLongLowMedianBarrier) {
  const ClusterShape shape =
      measureClusterShape(boxPoints(6.0, 0.42, 0.9, 1.1));
  EXPECT_TRUE(shape.valid);
  EXPECT_GT(shape.aspect_ratio, 10.0);
  EXPECT_FALSE(isVehicleShape(shape, filterConfig()));
}

TEST(ClusterShapeFilter, RejectsSmallPostCluster) {
  const ClusterShape shape =
      measureClusterShape(boxPoints(0.5, 0.4, 1.0, 0.0));
  EXPECT_FALSE(isVehicleShape(shape, filterConfig()));
}

TEST(ClusterShapeFilter, LargeVehicleProfileAcceptsDumpTruckButRejectsBarrier) {
  auto config = filterConfig();
  config.max_length = 14.0;
  config.max_height = 4.5;
  const auto truck = measureClusterShape(boxPoints(8.7, 2.7, 3.3, 0.8));
  EXPECT_FALSE(isVehicleShape(truck, filterConfig()));
  EXPECT_TRUE(isVehicleShape(truck, config));
  EXPECT_FALSE(isVehicleShape(measureClusterShape(boxPoints(10.0, 0.42, 0.6, 0.8)), config));
  EXPECT_FALSE(isVehicleShape(measureClusterShape(boxPoints(16.0, 2.7, 3.3, 0.8)), config));
}

}  // namespace
}  // namespace roundabout_lidar_tracker

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
