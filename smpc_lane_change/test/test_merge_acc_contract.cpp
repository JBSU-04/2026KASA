#include <smpc_lane_change/merge_acc_policy.hpp>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace smpc_lane_change;
void require(bool condition,const char* why) {if(!condition) throw std::runtime_error(why);}
int main() {
  TargetVehicleSet t;t.current_lane_id=1;t.target_lane_id=2;
  t.target_front.valid=true;t.target_front.unique_id=7;t.target_front.lane_id=2;t.target_front.observation_age=.1;
  TargetVehicle leader=t.target_front;leader.unique_id=8;t.nearby_vehicles.push_back(leader);
  BehaviorLongitudinalRequest r;r.current_lane_id=1;r.target_lane_id=2;r.merge_gap_selected=true;
  r.merge_follower_id=7;r.merge_leader_id=8;
  require(selectedMergeLeader(r,t,.3).unique_id==8,"select slot leader, not overtaken follower");
  r.merge_leader_id=-1;require(!selectedMergeLeader(r,t,.3).valid,"open front has no lead");
  r.merge_leader_id=8;t.nearby_vehicles[0].observation_age=.4;
  require(!selectedMergeLeader(r,t,.3).valid,"stale leader is unavailable");
  t.nearby_vehicles[0].observation_age=.1;r.target_lane_id=3;
  require(!selectedMergeLeader(r,t,.3).valid,"old lane request cannot select leader");
  BehaviorLongitudinalStatus s;s.header.stamp=ros::Time(10);s.current_lane_id=1;s.target_lane_id=2;
  s.independent_lead_limit_active=true;s.independent_lead_speed_cap_mps=13.;
  s.behavior_cap_applied=true;s.applied_speed_cap_mps=8.;s.final_target_speed_mps=8.;
  require(independentLeadCap(s,t,10.1,10.,.35,25.)==13.,"yield output must not become independent cap");
  require(independentLeadCap(s,t,10.5,10.,.35,25.)==25.,"stale status");
  require(independentLeadCap(s,t,9.9,10.,.35,25.)==25.,"future status");
  s.independent_lead_speed_cap_mps=std::numeric_limits<double>::quiet_NaN();
  require(independentLeadCap(s,t,10.1,10.,.35,25.)==25.,"nonfinite ceiling");
  require(mergeFollowSpeed(8.5,15.,8.5,.2,.5,2.,0.,25.)==15.,"8.5m equilibrium");
  require(mergeFollowSpeed(20.,15.,8.5,.2,.5,2.,0.,25.)==17.,"catchup bound");
  require(mergeFollowSpeed(3.,15.,8.5,.2,.5,2.,0.,25.)<15.,"close gap reduces speed");
  require(mergeFollowSpeed(8.5,15.,8.5,.2,.5,2.,0.,12.)==12.,"own ACC cap remains binding");
  std::cout << "12 merge ACC contract scenarios passed\n";
}
