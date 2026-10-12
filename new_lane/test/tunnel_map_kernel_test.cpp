#include <gtest/gtest.h>
#include "new_lane/tunnel_map_kernel.hpp"
TEST(TunnelPose,ExtrapolatesMotionInsteadOfRelabelingOldPosition){
 tunnel_map::Pose p; p.stamp=10;p.x=302600;p.y=4123700;p.yaw=0;p.speed=12;p.wz=0;p.twist_valid=true;
 tunnel_map::Pose out; ASSERT_TRUE(tunnel_map::extrapolatePose(p,10.08,out));EXPECT_NEAR(out.x,p.x+.96,1e-8);
 EXPECT_NEAR(out.y,p.y,1e-8);EXPECT_FALSE(tunnel_map::extrapolatePose(p,10.081,out));
 p.twist_valid=false;EXPECT_FALSE(tunnel_map::extrapolatePose(p,10.02,out));
}
#include <unistd.h>
int main(int argc,char**argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
using namespace tunnel_map;
namespace {
void syntheticMap(Map&m){
  for(double x=-2;x<32;x+=.05)for(double y=-4;y<4;y+=.05){m.floor->push_back(pcl::PointXYZ(x,y,-.435));m.points->push_back(pcl::PointXYZ(x,y,-.435));}
  *m.floor_xy=*m.floor;for(auto&p:*m.floor_xy)p.z=0;
  for(double x=0;x<30;x+=.2)for(double z=-.4;z<2.8;z+=.2)m.points->push_back(pcl::PointXYZ(x,3,z));
  m.tree.setInputCloud(m.points);m.floor_tree.setInputCloud(m.floor_xy);m.indexFloor();
}
}
TEST(TunnelMap,FloorAndLowObjectAreSeparate){Map m;syntheticMap(m);Pose p;auto r=classify({V(10,0,-1.765),V(10,0,-1.675)},p,&m,V(1.1,0,1.33));ASSERT_EQ(r.size(),2u);EXPECT_TRUE(r[0].bg);EXPECT_FALSE(r[1].bg);EXPECT_TRUE(r[1].candidate);EXPECT_NEAR(r[1].height,.09,.00001);}
TEST(TunnelMap,WallAdjacentLowObjectSurvives){Map m;syntheticMap(m);auto r=classify({V(10,2.9,-1.665)},Pose(),&m,V(1.1,0,1.33));EXPECT_FALSE(r[0].bg);EXPECT_TRUE(r[0].candidate);}
TEST(TunnelMap,SameXYDifferentZDoesNotMatch){Map m;syntheticMap(m);EXPECT_GT(m.nearest(V(10,0,.3)),.12);}
TEST(TunnelMap,UnknownFloorRemainsEvidence){auto r=classify({V(10,0,-1.6),V(10,.1,-1.6),V(10,.2,-1.6)},Pose(),nullptr,V(1.1,0,1.33));Confirmation c;c.update(r,10);EXPECT_EQ(r[0].label,2);c.update(r,10.1);EXPECT_EQ(r[0].label,4);}
TEST(TunnelMap,DuplicateScanCannotConfirm){auto r=classify({V(10,0,-1.6),V(10,.1,-1.6),V(10,.2,-1.6)},Pose(),nullptr,V(1.1,0,1.33));Confirmation c;c.update(r,10);c.update(r,10);EXPECT_EQ(r[0].label,2);c.update(r,10.1);EXPECT_EQ(r[0].label,4);}
TEST(TunnelMap,TwoPointsCannotConfirm){auto r=classify({V(10,0,-1.6),V(10,.1,-1.6)},Pose(),nullptr,V(1.1,0,1.33));Confirmation c;c.update(r,10);c.update(r,10.1);EXPECT_EQ(r[0].label,2);}
TEST(TunnelMap,CeilingDoesNotBecomeCandidate){auto r=classify({V(10,0,2.0)},Pose(),nullptr,V(1.1,0,1.33));EXPECT_FALSE(r[0].candidate);EXPECT_EQ(r[0].label,0);}
TEST(TunnelMap,GradeAndCrossfallStayGround){std::vector<V>p;for(double x=0;x<30;x+=.05)for(double y=-4;y<4;y+=.05)p.emplace_back(x,y,-1.765+.015*x+.025*y);auto r=classify(p,Pose(),nullptr,V(1.1,0,1.33));size_t supported=0;for(const auto&q:r)if(q.valid){++supported;EXPECT_NEAR(q.height,0,.01);EXPECT_FALSE(q.candidate);}EXPECT_GT(supported,p.size()*.8);}
TEST(TunnelMap,LocalUTMOriginKeepsPrecision){Map m;syntheticMap(m);m.origin=V(302665.263837,4123727.039868,0);V p=m.origin+V(10,0,-.435);EXPECT_LT(m.nearest(p),1e-6);EXPECT_NEAR(m.ground(p),-.435,1e-6);}
TEST(TunnelMap,MissingBinaryFailsExplicitly){Map m;EXPECT_THROW(m.load("/tmp/nonexistent_m8_map.bin"),std::runtime_error);}

TEST(TunnelMap,IndexedVoxelDistanceAgreesWithExactThresholdAndReusesQuery){
 Map m;syntheticMap(m);m.indexMap();
 for(double x=-.11;x<20;x+=.137)for(double y=2.8;y<3.2;y+=.013){V p(x,y,.331);
  EXPECT_EQ(m.matches(p,.12),m.nearest(p)<=.12);EXPECT_EQ(m.matches(p,.055),m.nearest(p)<=.055);}
 m.beginScan();V p(10.,3.,.3);bool first=m.matches(p,.12);size_t hits=m.cache_hits;
 EXPECT_EQ(m.matches(p,.12),first);EXPECT_EQ(m.cache_hits,hits+1);
 m.beginScan();EXPECT_EQ(m.cache_hits,0u);m.voxel_search=false;EXPECT_EQ(m.matches(p,.12),m.nearest(p)<=.12);
}

TEST(TunnelMap,BoundedMembershipMatchesExactDistance){Map m;syntheticMap(m);for(double x=2;x<12;x+=.137)for(double z=-.5;z<.3;z+=.031){V p(x,.13,z);EXPECT_EQ(m.matches(p,.055),m.nearest(p)<=.055);EXPECT_EQ(m.matches(p,.12),m.nearest(p)<=.12);}}
TEST(TunnelMap,MissingFloorSupportStaysUnknown){Map m;syntheticMap(m);EXPECT_TRUE(std::isnan(m.ground(V(100,100,0))));}
TEST(TunnelMap,FloorBucketsAgreeWithExactSupport){Map m;syntheticMap(m);for(double x=-1;x<30;x+=.137){V p(x,.131,-.3);EXPECT_NEAR(m.ground(p),-.435,1e-6);}}

TEST(TunnelMap,UnknownCannotBePromotedByRepetitionWhenPolicyDisabled){
 auto rows=classify({V(10,0,-1.6),V(10,.1,-1.6),V(10,.2,-1.6)},Pose(),nullptr,V(1.1,0,1.33));
 Confirmation confirmation;confirmation.allow_unknown_ground=false;
 confirmation.update(rows,10);confirmation.update(rows,10.1);EXPECT_EQ(rows[0].label,2);
}
TEST(TunnelMap,OnlyCommonTwoWallOffsetCanAlign){
 Map m;for(double x=0;x<30;x+=.1)for(double z=0;z<2;z+=.1){m.points->push_back(pcl::PointXYZ(x,3,z));m.points->push_back(pcl::PointXYZ(x,-3,z));}
 m.tree.setInputCloud(m.points);std::vector<V> scan;
 for(double x=1;x<30;x+=.1){scan.emplace_back(x,3.15,.5);scan.emplace_back(x,-2.85,.5);}
 auto shift=wallAlignment(scan,Pose(),m,V::Zero());ASSERT_TRUE(shift.valid);EXPECT_NEAR(shift.lateral,-.15,.001);
 scan.clear();for(double x=1;x<30;x+=.1)scan.emplace_back(x,3.15,.5);
 EXPECT_FALSE(wallAlignment(scan,Pose(),m,V::Zero()).valid);
}
TEST(TunnelMap,DifferentOffsetsOnEachWallCannotAlign){
 Map m;for(double x=0;x<30;x+=.1){m.points->push_back(pcl::PointXYZ(x,3,.5));m.points->push_back(pcl::PointXYZ(x,-3,.5));}m.tree.setInputCloud(m.points);
 std::vector<V>scan;for(double x=1;x<30;x+=.1){scan.emplace_back(x,3.15,.5);scan.emplace_back(x,-3.15,.5);}
 EXPECT_FALSE(wallAlignment(scan,Pose(),m,V::Zero()).valid);
}
TEST(TunnelDetectionUnion,RawMismatchCannotReintroduceGround){
 Row r;r.sensor=V(24.,0.,-1.5);r.world=V(302620.,4123600.,0.);r.valid=true;r.height=.2;r.candidate=true;r.label=3;
 EXPECT_TRUE(mergeDetectionRows({}, {r}).empty());
 auto rows=mergeDetectionRows({}, {r},true);ASSERT_EQ(rows.size(),1u);EXPECT_DOUBLE_EQ(rows[0].sensor.x(),24.);
}
TEST(TunnelDetectionUnion,ExactDuplicatesOnlyAndUnknownGroundNotConfirmed){
 Row r;r.sensor=V(24.,0.,-1.5);r.valid=true;r.candidate=true;r.label=2;r.height=.2;
 Row close=r;close.sensor.x()+=.001;
 Row unknown=r;unknown.valid=false;unknown.label=4;unknown.sensor.x()+=1;
 Row bg=r;bg.bg=true;bg.sensor.x()+=2;
 auto rows=mergeDetectionRows({r},{r,close,unknown,bg},true);ASSERT_EQ(rows.size(),2u);EXPECT_DOUBLE_EQ(rows[1].sensor.x(),24.001);EXPECT_EQ(rows[1].label,2);
}

TEST(TunnelDetectionUnion,SharedGroundHeightCannotBeOverwrittenByMapFloor){
 Row near;near.sensor=V(24.,0.,-1.7);near.height=.01;near.valid=true;
 Row mapped=near;mapped.height=.09;mapped.candidate=true;mapped.label=3;
 auto rows=mergeDetectionRows({near},{mapped});ASSERT_EQ(rows.size(),1u);EXPECT_DOUBLE_EQ(rows[0].height,.01);EXPECT_EQ(rows[0].label,0);
}
TEST(TunnelDetectionUnion,ActualLowNongroundIsPreservedWithoutMapEvidence){
 Row low;low.sensor=V(18.,.3,-1.6);low.height=.16;low.valid=true;
 auto rows=mergeDetectionRows({low},{});ASSERT_EQ(rows.size(),1u);EXPECT_DOUBLE_EQ(rows[0].height,.16);
}
