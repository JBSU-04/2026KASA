#include "tunnel_xicp/backend.hpp"
#include <gtest/gtest.h>
#include <chrono>
#include <algorithm>
#include <iostream>
using namespace tunnel_xicp;
int main(int argc,char** argv){::testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}

#ifndef XICP_CONFIG_PATH
#error XICP_CONFIG_PATH required
#endif
static Cloud corridor(bool both=true) {
  Cloud c;
  for(int side: {-1,1}) {
    if(!both&&side<0)continue;
    for(int x=-60;x<=60;++x)for(int z=0;z<9;++z) {
      pcl::PointNormal p;p.x=x*.2f;p.y=side*5.f;p.z=z*.2f;
      p.normal_x=0;p.normal_y=side;p.normal_z=0;c.push_back(p);
    }
  }
  return c;
}
static Cloud sensorCloud(const Cloud& reference,const Eigen::Matrix4d& pose) {
  Cloud c;for(auto p:reference) {
    Eigen::Vector3d q=(pose.inverse()*Eigen::Vector4d(p.x,p.y,p.z,1)).head<3>();
    Eigen::Vector3d n=pose.block<3,3>(0,0).transpose()*Eigen::Vector3d(p.normal_x,p.normal_y,p.normal_z);
    p.x=q.x();p.y=q.y();p.z=q.z();p.normal_x=n.x();p.normal_y=n.y();p.normal_z=n.z();c.push_back(p);
  }return c;
}
TEST(Xicp, OfficialEqualitySolverAndCorridorDegeneracy) {
  Backend b(XICP_CONFIG_PATH);auto reference=corridor();Eigen::Matrix4d initial=Eigen::Matrix4d::Identity();initial(1,3)=.12;
  auto r=b.align(reference,reference,initial,0,Policy());
  EXPECT_TRUE(r.official_xicp_called)<<r.reason;EXPECT_GT(r.iterations,0);EXPECT_GT(r.constraints,0);
  EXPECT_FALSE(r.longitudinal);EXPECT_TRUE(r.lateral);EXPECT_TRUE(r.yaw);
  EXPECT_EQ(r.state,State::VALID)<<r.reason;EXPECT_NEAR(r.lateral_delta,-.12,.03);EXPECT_LT(r.rms,.03);
}
TEST(Xicp, SingleWallCannotBecomeValid) {
  Backend b(XICP_CONFIG_PATH);auto c=corridor(false);auto r=b.align(c,c,Eigen::Matrix4d::Identity(),0,Policy());
  EXPECT_TRUE(r.official_xicp_called)<<r.reason;EXPECT_NE(r.state,State::VALID);EXPECT_FALSE(r.right);
}
TEST(Xicp, RotatedMapUsesRoadAxesNotWorldAxes) {
  Backend b(XICP_CONFIG_PATH);auto c=corridor();const double heading=-151.84*M_PI/180;
  auto rotation=Eigen::AngleAxisd(heading,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  for(auto& p:c){Eigen::Vector3d x=rotation*Eigen::Vector3d(p.x,p.y,p.z),n=rotation*Eigen::Vector3d(p.normal_x,p.normal_y,p.normal_z);
    p.x=x.x();p.y=x.y();p.z=x.z();p.normal_x=n.x();p.normal_y=n.y();p.normal_z=n.z();}
  auto r=b.align(c,c,Eigen::Matrix4d::Identity(),heading,Policy());
  EXPECT_TRUE(r.official_xicp_called)<<r.reason;EXPECT_FALSE(r.longitudinal);EXPECT_TRUE(r.lateral);EXPECT_TRUE(r.yaw);EXPECT_EQ(r.state,State::VALID)<<r.reason;
}
TEST(Xicp, AvoidanceYawIsNotMapYawError) {
  Backend b(XICP_CONFIG_PATH);auto reference=corridor();Eigen::Matrix4d pose=Eigen::Matrix4d::Identity();
  pose.block<3,3>(0,0)=Eigen::AngleAxisd(17*M_PI/180,Eigen::Vector3d::UnitZ()).toRotationMatrix();pose(1,3)=1.8;
  auto reading=sensorCloud(reference,pose);auto r=b.align(reading,reference,pose,0,Policy());
  EXPECT_TRUE(r.official_xicp_called)<<r.reason;EXPECT_NEAR(r.yaw_delta,0,.01);EXPECT_NEAR(r.lateral_delta,0,.03);
}
TEST(Xicp, UnsupportedAxesNeverReachComparisonPose) {
  Eigen::Matrix4d pose=Eigen::Matrix4d::Identity();pose(0,3)=12;pose(2,3)=1.4;
  pose.block<3,3>(0,0)=(Eigen::AngleAxisd(.2,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(.03,Eigen::Vector3d::UnitY())).toRotationMatrix();
  auto result=comparisonPose(pose,.1,.02,Eigen::Vector3d::UnitY());
  EXPECT_DOUBLE_EQ(result(0,3),12);EXPECT_DOUBLE_EQ(result(2,3),1.4);
  EXPECT_NEAR((result.block<3,3>(0,0).col(2)-Eigen::AngleAxisd(.02,Eigen::Vector3d::UnitZ())*pose.block<3,3>(0,0).col(2)).norm(),0,1e-12);
}
TEST(Xicp, LargeCorrectionPoorMatchAndUnobservabilityRejected) {
  Result r;r.official_xicp_called=true;r.matched_points=200;r.ratio=.9;r.rms=.02;r.left=r.right=r.lateral=r.yaw=true;
  r.lateral_delta=.61;classify(r,Policy());EXPECT_EQ(r.state,State::INVALID);
  r.lateral_delta=0;r.rms=.11;classify(r,Policy());EXPECT_EQ(r.state,State::INVALID);
  r.rms=.02;r.yaw=false;classify(r,Policy());EXPECT_EQ(r.state,State::DEGRADED);
  r.yaw=true;r.ratio=.59;classify(r,Policy());EXPECT_EQ(r.state,State::INVALID);
  r.ratio=.9;r.official_xicp_called=false;classify(r,Policy());EXPECT_EQ(r.state,State::INVALID);
}
TEST(Xicp, OutsideMapAndEmptyScan) {
  Backend b(XICP_CONFIG_PATH);auto ref=corridor();auto reading=ref;for(auto& p:reading)p.y+=20;
  auto r=b.align(reading,ref,Eigen::Matrix4d::Identity(),0,Policy());EXPECT_EQ(r.state,State::INVALID);
  r=b.align(Cloud(),ref,Eigen::Matrix4d::Identity(),0,Policy());EXPECT_FALSE(r.official_xicp_called);
}
TEST(Xicp, RepeatedFrameDoesNotLeakConfidence) {
  Backend b(XICP_CONFIG_PATH);auto full=corridor(),half=corridor(false);Policy p;
  auto a=b.align(full,full,Eigen::Matrix4d::Identity(),0,p);auto r=b.align(half,full,Eigen::Matrix4d::Identity(),0,p);
  EXPECT_EQ(a.state,State::VALID);EXPECT_NE(r.state,State::VALID);
}
TEST(Xicp, PerformanceReportNotVehicleBenchmark) {
  Backend b(XICP_CONFIG_PATH);auto c=corridor();std::vector<double> times;
  for(int i=0;i<10;++i){auto r=b.align(c,c,Eigen::Matrix4d::Identity(),0,Policy());EXPECT_TRUE(r.official_xicp_called);times.push_back(r.processing_ms);}
  std::sort(times.begin(),times.end());std::cout<<"SYNTHETIC backend p95_ms="<<times.back()<<std::endl;
}
TEST(Xicp, CachedMapUpdatesWhenSameSizeGeometryChanges) {
  Backend b(XICP_CONFIG_PATH);auto reading=corridor(),reference=reading;
  auto first=b.align(reading,reference,Eigen::Matrix4d::Identity(),0,Policy());
  for(auto& p:reference)p.y+=.2;
  auto updated=b.align(reading,reference,Eigen::Matrix4d::Identity(),0,Policy());
  EXPECT_EQ(first.state,State::VALID);EXPECT_EQ(updated.state,State::VALID)<<updated.reason;
  EXPECT_NEAR(updated.lateral_delta,.2,.03);EXPECT_GT(updated.matching_ms,0);
  EXPECT_GT(updated.verification_ms,0);
}
