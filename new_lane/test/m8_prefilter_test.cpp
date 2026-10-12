#include <gtest/gtest.h>
#include "new_lane/m8_prefilter.hpp"
using namespace m8_prefilter;
TEST(PathIndex,DoubleUtmPrecisionAndExactDistance){PathIndex p({{302500,4124000},{302501,4124000},{302502,4124000}});auto r=p.nearest({302501.01,4124000.02});EXPECT_EQ(r.first,1);EXPECT_NEAR(r.second,.0005,1e-10);}
TEST(CommonFilter,PreservesOriginalHeightAndSparseLowPoints){Config c;c.self_x_max=2;Sample p{15,0,-1.6,.08};EXPECT_TRUE(keepCommon(p,c,nullptr,{0,0},0));EXPECT_FLOAT_EQ(p[3],.08);}
TEST(CommonFilter,MatchesRoiAndReferencePointCorridor){Config c;PathIndex path({{0,0},{10,0},{20,0}});EXPECT_TRUE(keepCommon({8.9,0,-1.4,.3},c,&path,{0,0},0));EXPECT_FALSE(keepCommon({8.9,8,-1.4,.3},c,&path,{0,0},0));EXPECT_FALSE(keepCommon({8.9,0,-1.8,0},c,&path,{0,0},0));}
TEST(CommonFilter,RotatedEgoAndWorldRouteAgree){Config c;PathIndex path({{302500,4124000},{302500,4124010},{302500,4124020}});EXPECT_TRUE(keepCommon({8.9,0,-1.4,.3},c,&path,{302500,4124000},M_PI/2));}
TEST(CommonFilter,MissingRouteNeverInventsObservedClearance){Config c;c.skip_missing_path=false;EXPECT_FALSE(keepCommon({10,0,-1.4,.3},c,nullptr,{0,0},0));}
static PathIndex road(){std::vector<XY>p;for(int i=0;i<100;++i)p.push_back({i*.1,0});return PathIndex(p);}
// 2026-10-12 벽·연석 제한 구역: 경계 밖 제거 + 띠 안 세로 벽면 제거.
static std::vector<std::array<double,3>>prof(){return {{0,-3,4},{99,-3,4}};}
TEST(WallZone,RemovesEverythingBeyondBoundary){auto r=road();std::vector<Sample>p{{2,-3.5,0,.12},{2,-3.2,0,2.},{2,4.3,0,.5},{2,-2.,0,.12}};WallZoneStats st;
 auto k=wallZoneKeep(p,r,prof(),{0,0},0,0,0,0,WallZone(),&st);EXPECT_FALSE(k[0]);EXPECT_FALSE(k[1]);EXPECT_FALSE(k[2]);EXPECT_TRUE(k[3]);EXPECT_EQ(st.outside,3u);}
TEST(WallZone,VerticalWallInBandRemovedOverTwoMetres){auto r=road();std::vector<Sample>p;for(int i=0;i<30;++i)for(float h:{.1f,.6f,1.2f})p.push_back({float(.5+i*.1),-2.8,0,h});
 auto k=wallZoneKeep(p,r,prof(),{0,0},0,0,0,0,WallZone());for(bool v:k)EXPECT_FALSE(v);}
TEST(WallZone,ShortTallObjectAtWallIsKept){auto r=road();std::vector<Sample>p;for(int i=0;i<6;++i)for(float h:{.1f,.6f,1.2f})p.push_back({float(2+i*.1),-2.8,0,h});
 auto k=wallZoneKeep(p,r,prof(),{0,0},0,0,0,0,WallZone());for(bool v:k)EXPECT_TRUE(v);}
TEST(WallZone,LowObstacleBodyInLaneSurvivesNextToWall){auto r=road();std::vector<Sample>p;for(int i=0;i<30;++i)for(float h:{.1f,.6f,1.2f})p.push_back({float(.5+i*.1),-2.8,0,h});
 size_t n=p.size();p.push_back({2,-2.2,0,.45});p.push_back({2,-1.8,0,.45});auto k=wallZoneKeep(p,r,prof(),{0,0},0,0,0,0,WallZone());EXPECT_TRUE(k[n]);EXPECT_TRUE(k[n+1]);}
TEST(WallZone,LongLowObstacleAlongWallIsKept){auto r=road();std::vector<Sample>p;for(int i=0;i<30;++i)for(float h:{.05f,.25f,.45f})p.push_back({float(.5+i*.1),-2.8,0,h});
 auto k=wallZoneKeep(p,r,prof(),{0,0},0,0,0,0,WallZone());for(bool v:k)EXPECT_TRUE(v);}
TEST(WallZone,ThinRingWallBesideEgoRemoved){auto r=road();std::vector<Sample>p;for(int i=0;i<30;++i)p.push_back({float(.5+i*.1),-2.9,0,.8f});
 auto k=wallZoneKeep(p,r,prof(),{0,0},0,0,0,0,WallZone());for(bool v:k)EXPECT_FALSE(v);}
TEST(WallZone,OpenEntranceHasNoBoundary){auto r=road();std::vector<Sample>p{{2,-5,0,.12}};auto k=wallZoneKeep(p,r,{{0,0,0},{50,-3,4},{99,-3,4}},{0,0},0,0,0,0,WallZone());EXPECT_TRUE(k[0]);}
TEST(WallZone,DrOffsetShiftsBoundary){auto r=road();std::vector<Sample>p{{2,-3.5,0,.3}};
 // offset -1.2: 실제 횡 = -3.5 + 1.2 = -2.3 (벽 안쪽) -> 유지.
 EXPECT_TRUE(wallZoneKeep(p,r,prof(),{0,0},0,0,0,-1.2,WallZone())[0]);EXPECT_FALSE(wallZoneKeep(p,r,prof(),{0,0},0,0,0,0,WallZone())[0]);}

int main(int argc,char**argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
