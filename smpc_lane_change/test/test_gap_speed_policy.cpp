#include <smpc_lane_change/gap_speed_policy.hpp>
#include <smpc_lane_change/merge_acc_policy.hpp>
#include <stdexcept>
#include <iostream>
using namespace smpc_lane_change;
void require(bool value,const char* why){if(!value)throw std::runtime_error(why);}
TargetVehicle vehicle(int id,int lane,double speed,double s) {
 TargetVehicle v;v.valid=true;v.unique_id=id;v.lane_id=lane;v.velocity_valid=true;
 v.velocity_std=.5;v.observation_age=0;v.v_long=speed;v.s=s;return v;
}
int main() {
 GapSpeedPolicy planner,acc;TargetVehicleSet set;set.current_lane_id=1;set.target_lane_id=2;
 set.header.stamp=ros::Time(10);set.target_front=vehicle(30,2,14,100);
 set.target_rear=vehicle(27,2,13,60);set.current_front=vehicle(5,1,20,120);
 planner.update(set);acc.update(set);
 require(planner.get(set.target_front,10.1).speed==14,"measured speed, not 11m/s");
 require(planner.get(set.current_front,10.1).speed==20,"different lane independent");
 require(planner.get(set.target_rear,10.1).speed==13,"same lane need not equal");
 set.header.stamp=ros::Time(10.2);set.target_front.velocity_valid=false;set.target_front.s=102.8;
 planner.update(set);acc.update(set);
 require(planner.get(set.target_front,10.2).source=="trusted_history","use true trusted history");
 require(planner.get(set.target_front,10.2).speed==acc.get(set.target_front,10.2).speed,"consumer parity");
 require(planner.get(set.target_front,10.2).evidence_stamp==10,"original trusted timestamp");
 planner.update(set);
 require(!planner.get(set.target_front,10.6).known,"repeated frame cannot freshen history");
 require(!planner.get(vehicle(999,2,0,0),10.2).known,"new ID not inherited automatically");
 GapSpeedPolicy positional;
 for(int i=0;i<=8;++i){set.header.stamp=ros::Time(20+i*.1);set.target_front=vehicle(42,2,0,100+1.4*i);set.target_front.velocity_valid=false;positional.update(set);}
 auto e=positional.get(set.target_front,20.8);
 require(e.known && e.source=="position_history" && std::abs(e.speed-14)<1e-6,"position history independent of ego speed");
 set.target_front.lane_id=3;set.header.stamp=ros::Time(20.9);positional.update(set);
 require(!positional.get(set.target_front,20.9).known,"lane change resets s coordinate history");
 require(mergeFollowSpeed(30,14,8.5,.2,.5,6,0,25)==20,"selected slot catchup6");
 require(mergeFollowSpeed(30,14,8.5,.2,.5,2,0,25)==16,"ordinary follow remains2");
 require(mergeFollowSpeed(8.5,14,8.5,.2,.5,6,0,25)==14,"8.5m equilibrium");
 require(mergeFollowSpeed(5,14,8.5,.2,.5,6,0,25)<14,"close front still slows");
 require(mergeFollowSpeed(30,14,8.5,.2,.5,6,0,15)==15,"independent ceiling binding");
 BehaviorLongitudinalRequest r;r.active=true;r.merge_gap_selected=true;r.current_lane_id=1;r.target_lane_id=2;
 r.action=r.ACCELERATE;r.reason="smpc_v3_gap_align_accel";r.requested_accel_mps2=3;r.header.stamp=ros::Time(30);
 require(gapAccelRequest(r,set,30.1,30,.35,true),"explicit selected-slot acceleration");
 require(!gapAccelRequest(r,set,30.4,30,.35,true),"stale request rejected");
 require(!gapAccelRequest(r,set,30.1,30,.35,false),"unknown speed cannot authorize acceleration");
 r.reason="smpc_v3_prepare_ACCEL_THEN_CHANGE";
 require(!gapAccelRequest(r,set,30.1,30,.35,true),"other behavior accelerations untouched");
 set.header.stamp=ros::Time(5);set.target_front=vehicle(70,2,9,10);positional.update(set);
 require(positional.get(set.target_front,5).speed==9,"simulation clock reset starts new history");
 std::cout<<"20 gap speed and preparation contract scenarios passed\n";
}
