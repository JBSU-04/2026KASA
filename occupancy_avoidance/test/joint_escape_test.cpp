#include <gtest/gtest.h>
#include "occupancy_avoidance/joint_escape.hpp"
#include <cmath>
using namespace occupancy_avoidance;
TEST(JointEscape, ForwardFirstAndDenseSweep){
 Pose p;std::vector<Reference> goals{{12,0,0,0}};size_t samples=0;
 auto r=jointEscape(p,goals,[&](const Reference&,bool reverse,double){++samples;return !reverse;},3.,.3,10.,.1,true);
 ASSERT_TRUE(r.valid)<<r.reason;EXPECT_TRUE(r.reverse.empty());ASSERT_GT(r.forward.size(),100u);EXPECT_GT(samples,100u);
 for(size_t i=1;i<r.forward.size();++i)EXPECT_LE(std::hypot(r.forward[i].x-r.forward[i-1].x,r.forward[i].y-r.forward[i-1].y),.101);
}
TEST(JointEscape, ReverseThenForwardEscapesBlockedForwardPose){
 Pose p;std::vector<Reference> goals{{15,2.5,0,0},{15,-2.5,0,0}};
 auto clear=[](const Reference& p,bool,double){
   // Full oriented body, 0.2 m obstacle clearance and physical wall inset.
   for(double a=-.99;a<4.05;a+=.1)for(double b=-1.15;b<=1.15;b+=.1){
    double x=p.x+std::cos(p.yaw)*a-std::sin(p.yaw)*b,y=p.y+std::sin(p.yaw)*a+std::cos(p.yaw)*b;
    if(std::abs(y)>5.5||(x>=4.5&&x<=7.5&&std::abs(y)<=1.2))return false;
   }return true;
 };
 auto r=jointEscape(p,goals,clear,3.,.3,10.,.5,false);
 ASSERT_TRUE(r.valid)<<r.reason<<" expanded="<<r.expanded;ASSERT_GT(r.reverse.size(),1u);ASSERT_GT(r.forward.size(),1u);
 EXPECT_LE(r.reverse_distance,10.);EXPECT_NEAR(r.reverse.back().x,r.forward.front().x,1e-6);
 EXPECT_NEAR(r.reverse.back().y,r.forward.front().y,1e-6);
 for(const auto& q:r.reverse)EXPECT_TRUE(clear(q,true,0));for(const auto& q:r.forward)EXPECT_TRUE(clear(q,false,0));
}
TEST(JointEscape, UnknownRearCannotAuthorizeReverse){
 Pose p;std::vector<Reference> goals{{-5,0,0,0}};
 auto r=jointEscape(p,goals,[](const Reference& q,bool reverse,double){return !reverse&&q.x>=0&&q.x<.05;},3.,.3,10.,.2,false);
 EXPECT_FALSE(r.valid);EXPECT_EQ(r.reason,"no_connected_escape");
}
TEST(JointEscape, TimeoutIsNotNoPath){
 Pose p;auto r=jointEscape(p,{{12,0,0,0}},[](const Reference&,bool,double){return true;},3.,.3,10.,0.,false);
 EXPECT_FALSE(r.valid);EXPECT_EQ(r.reason,"search_timeout");
}
