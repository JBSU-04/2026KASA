#include <gtest/gtest.h>

#include "mission_manager/constraint_arbiter.hpp"

namespace mission_manager {

TEST(ConstraintArbiter, NoActiveMissionPassesThrough) {
  ConstraintArbiter arbiter;
  const Constraint result = arbiter.merge({Constraint()});
  EXPECT_FALSE(result.active);
  EXPECT_LT(result.max_speed, 0.0);
}

TEST(ConstraintArbiter, CombinesMostRestrictiveLimits) {
  Constraint speed;
  speed.source = 1;
  speed.priority = 50;
  speed.active = true;
  speed.max_speed = 6.0;
  speed.comfortable_decel = 2.5;
  speed.reason = "roundabout speed";

  Constraint stop;
  stop.source = 2;
  stop.priority = 100;
  stop.active = true;
  stop.max_speed = 4.0;
  stop.stop_required = true;
  stop.stop_distance = 8.0;
  stop.comfortable_decel = 2.0;
  stop.reason = "crosswalk stop";

  const Constraint result = ConstraintArbiter().merge({speed, stop});
  EXPECT_TRUE(result.active);
  EXPECT_EQ(result.source, 255);
  EXPECT_EQ(result.priority, 100);
  EXPECT_DOUBLE_EQ(result.max_speed, 4.0);
  EXPECT_TRUE(result.stop_required);
  EXPECT_DOUBLE_EQ(result.stop_distance, 8.0);
  EXPECT_DOUBLE_EQ(result.comfortable_decel, 2.0);
}

TEST(ConstraintArbiter, ChoosesNearestLeadVehicle) {
  Constraint far;
  far.active = true;
  far.lead_vehicle_active = true;
  far.lead_vehicle_id = 10;
  far.lead_distance = 20.0;

  Constraint near = far;
  near.lead_vehicle_id = 11;
  near.lead_distance = 7.0;

  const Constraint result = ConstraintArbiter().merge({far, near});
  EXPECT_TRUE(result.lead_vehicle_active);
  EXPECT_EQ(result.lead_vehicle_id, 11);
  EXPECT_DOUBLE_EQ(result.lead_distance, 7.0);
}

TEST(ConstraintArbiter, ProfileBelongsToSelectedLead) {
  Constraint roundabout;roundabout.active=true;roundabout.lead_vehicle_active=true;
  roundabout.lead_vehicle_id=10;roundabout.lead_distance=8;
  roundabout.lead_profile_override=true;roundabout.lead_standstill_distance=2.5;roundabout.lead_time_gap=.5;
  Constraint other;other.active=true;other.lead_vehicle_active=true;
  other.lead_vehicle_id=20;other.lead_distance=12;
  auto out=ConstraintArbiter().merge({other,roundabout});
  EXPECT_TRUE(out.lead_profile_override);EXPECT_DOUBLE_EQ(out.lead_time_gap,.5);
  other.lead_distance=4;out=ConstraintArbiter().merge({roundabout,other});
  EXPECT_EQ(out.lead_vehicle_id,20);EXPECT_FALSE(out.lead_profile_override);
  EXPECT_DOUBLE_EQ(out.lead_standstill_distance,0);
}

}  // namespace mission_manager

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
