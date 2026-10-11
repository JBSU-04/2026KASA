#include <gtest/gtest.h>
#include "mission_planner/roundabout_observation.hpp"
#include "mission_planner/gap_selector.hpp"
#include "mission_planner/geometry.hpp"
#include "mission_planner/roundabout_cloud_adapter.hpp"

namespace mission_planner {
TEST(RoundaboutObservation, GroundReturnCarvesOnlyMeasuredBeamAndObstacleOverridesFree) {
  const ObservationPoint origin{0,0,1.754};
  auto map=buildRoundaboutObservation(10,0,12,.5,1.,origin,0.,{{20,0,0}},{});
  EXPECT_EQ(map->at(8,0),ObservationCell::Free);
  EXPECT_EQ(map->at(8,2),ObservationCell::Unknown);
  EXPECT_EQ(map->at(21,0),ObservationCell::Unknown);
  EXPECT_NE(map->at(20,0),ObservationCell::Occupied); // ground is not a car
  map=buildRoundaboutObservation(10,0,12,.5,1.,origin,0.,{{20,0,0}},{{8,0,.5}});
  EXPECT_EQ(map->at(8,0),ObservationCell::Occupied);
}
TEST(RoundaboutObservation, RayStopsAtForegroundReturnLeavingItsShadowUnknown) {
  auto map=buildRoundaboutObservation(10,0,12,.5,1.,{0,0,1.754},0.,{{8,0,1.0}},{});
  EXPECT_EQ(map->at(6,0),ObservationCell::Free);
  EXPECT_EQ(map->at(8,0),ObservationCell::Occupied);
  EXPECT_EQ(map->at(10,0),ObservationCell::Unknown);
}
TEST(RoundaboutObservation, ThreeDistinctFreshScansAreRequiredAndTimeResetClearsHistory) {
  ObservationHistory h;
  for(double t:{1.,1.1}) {auto f=std::make_shared<RoundaboutObservation>();f->stamp=t;h.add(f);}
  EXPECT_FALSE(h.stable(1.1));
  auto f=std::make_shared<RoundaboutObservation>();f->stamp=1.1;
  EXPECT_FALSE(h.add(f));EXPECT_FALSE(h.stable(1.2));
  f=std::make_shared<RoundaboutObservation>();f->stamp=1.2;h.add(f);
  EXPECT_TRUE(h.stable(1.2));EXPECT_FALSE(h.stable(1.8));
  f=std::make_shared<RoundaboutObservation>();f->stamp=.5;h.add(f);
  EXPECT_EQ(h.frames.size(),1U);EXPECT_FALSE(h.stable(.5));
}
TEST(RoundaboutObservation, OpenGapIsFiniteAndCannotPassWithoutObservedSpace) {
  EgoEntryModel ego;SequentialParams p;p.observed_slot=true;
  auto r=GapSelector::evaluateSpatial({},ego,p,-2);
  EXPECT_FALSE(r.feasible);EXPECT_EQ(r.gap.limit_side,"observation_unknown");
  p.observed_bounds=[](double){return std::make_pair(-4.,4.);};
  r=GapSelector::evaluateSpatial({},ego,p,-2);EXPECT_FALSE(r.feasible);
  p.observed_bounds=[](double){return std::make_pair(-6.,6.);};
  r=GapSelector::evaluateSpatial({},ego,p,-2);ASSERT_TRUE(r.feasible);
  EXPECT_DOUBLE_EQ(r.observed_free_length,12.);EXPECT_TRUE(std::isfinite(r.rear_margin));
}
} // namespace mission_planner

int main(int argc,char** argv) {
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}

namespace mission_planner {
TEST(RoundaboutObservation, CloudAdapterDistinguishesInvalidLayoutFromEmptyNonGround) {
 sensor_msgs::PointCloud2 cloud;ObservationTransform tf;std::vector<ObservationPoint> points;
 EXPECT_FALSE(observationPoints(cloud,tf,&points));
 sensor_msgs::PointCloud2Modifier modifier(cloud);modifier.setPointCloud2FieldsByString(1,"xyz");modifier.resize(0);
 EXPECT_TRUE(observationPoints(cloud,tf,&points));EXPECT_TRUE(points.empty());
 modifier.resize(1);sensor_msgs::PointCloud2Iterator<float> x(cloud,"x"),y(cloud,"y"),z(cloud,"z");
 *x=1;*y=2;*z=.3;EXPECT_TRUE(observationPoints(cloud,tf,&points));ASSERT_EQ(points.size(),1U);
 EXPECT_NEAR(points[0].x,2.1,1e-6);EXPECT_NEAR(points[0].z,1.63,1e-6);
 cloud.data.pop_back();EXPECT_FALSE(observationPoints(cloud,tf,&points));
}
}

namespace mission_planner {
TEST(RoundaboutObservation, SweptBodyKeepsUnknownAndUntrackedOccupancySeparateFromProvenTraffic) {
 RoundaboutObservation map;map.min_x=map.min_y=-10;map.width=40;map.resolution=.5;
 map.cells.assign(1600,ObservationCell::Free);
 ObservationBox current{0,0,0,4.6,1.9},future{4,0,0,4.6,1.9},car{5,0,0,4.,2.};
 EXPECT_EQ(observedFootprint(map,future,current,{}),ObservationCell::Free);
 map.set(5,0,ObservationCell::Unknown);
 EXPECT_EQ(observedFootprint(map,future,current,{}),ObservationCell::Unknown);
 map.set(5,0,ObservationCell::Occupied);
 EXPECT_EQ(observedFootprint(map,future,current,{}),ObservationCell::Occupied);
 EXPECT_EQ(observedFootprint(map,future,current,{car}),ObservationCell::Free);
 map.set(3,0,ObservationCell::Occupied);
 car.x=6;
 EXPECT_EQ(observedFootprint(map,future,current,{car}),ObservationCell::Occupied);
}
}

namespace mission_planner {
TEST(RoundaboutObservation, CachedAxesSeparationMatchesOriginalSatAcrossRotationsAndOffsets) {
 for(double a:{-3.,-1.5,0.,.7,2.9})for(double b:{-2.7,-.2,0.,1.1,3.})
  for(double x:{-8.,-1.,0.,.3,6.})for(double y:{-3.,0.,.1,4.}) {
   ObservationBox ego{0,0,a,4.6,1.9},npc{x,y,b,7.,2.8};
   EXPECT_NEAR(ego.separation(npc),boxSeparation({0,0},a,4.6,1.9,{x,y},b,7.,2.8),1e-9);
  }
}
}
