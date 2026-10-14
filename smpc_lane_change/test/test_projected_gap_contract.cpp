#include <algorithm>
#include <array>
#include <cstdint>
#include <deque>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <numeric>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <smpc_lane_change/BehaviorLongitudinalRequest.h>
#include <smpc_lane_change/merge_acc_policy.hpp>
#include <smpc_lane_change/gap_speed_policy.hpp>
#include <smpc_lane_change/BehaviorLongitudinalStatus.h>
#include <smpc_lane_change/BehaviorPlan.h>
#include <smpc_lane_change/LaneChangeDecision.h>
#include <smpc_lane_change/TargetVehicle.h>
#include <smpc_lane_change/TargetVehicleSet.h>
#include <smpc_lane_change/lane_path.hpp>
#include <mission_supervisor/SectionState.h>
#include <waypoint_system/PathSwitchPreview.h>
#include <waypoint_system/spatial_blend.hpp>

// Exercise the actual decision implementation; no ROS callbacks are spun.
#define private public
#define main production_decision_main
#include "../src/smpc_decision_node.cpp"
#undef main
#undef private
#include <stdexcept>
void require(bool v,const char* why){if(!v)throw std::runtime_error(why);}
int main(int argc,char** argv) {
 ros::init(argc,argv,"isolated_projected_gap_contract",ros::init_options::AnonymousName);
 ros::Time::setNow(ros::Time(50));
 SmpcDecisionNode node;
 ros::Time::setNow(ros::Time(50));
 node.lanes_.clear();
 const std::string csv="/tmp/highway_projected_contract.csv";
 {std::ofstream f(csv);f<<"0,0,0\n1,1000,0\n";}
 smpc_lane_change::LanePath lane;require(lane.loadCsv(csv),"synthetic lane");node.lanes_[2]=lane;
 node.fixed_target_gap_gate_enabled_=true;node.target_front_safe_gap_m_=6.2;
 node.target_rear_safe_gap_m_=11;node.target_front_min_ttc_sec_=1;node.target_rear_min_ttc_sec_=1.2;
 node.target_rear_min_headway_sec_=.8;node.deadline_relax_enabled_=false;node.wait_relax_enabled_=false;
 node.npc_longitudinal_event_modes_enabled_=false;node.lateral_mode_enabled_=false;
 node.behavior_execution_delay_sec_=0;node.ego_speed_mps_=15.6;
 smpc_lane_change::TargetVehicleSet t;t.current_lane_id=1;t.target_lane_id=2;
 t.ego_s_current=100;t.distance_to_lane_end=100;
 auto make=[](int id,double s){smpc_lane_change::TargetVehicle v;v.valid=true;v.unique_id=id;v.lane_id=2;
 v.s=s;v.x=s;v.y=0;v.yaw=0;v.length=3.3;v.width=1.8;v.half_s=1.65;v.velocity_valid=false;
 v.velocity_std=1.5;v.v_long=0;v.delta_s=s-100;v.front_delta_s=v.delta_s+1.65;v.rear_delta_s=v.delta_s-1.65;return v;};
 // 34s bag's approximate bumper gaps: front19.195m,rear13.318m.
 t.target_front=make(30,123.245);t.target_rear=make(27,82.632);
 for(int i=0;i<=8;++i) {
  auto observation=t;observation.header.stamp=ros::Time(49.2+.1*i);
  observation.target_front.s-=14*(.8-.1*i);observation.target_rear.s-=14*(.8-.1*i);
  node.gap_speed_policy_.update(observation);
 }
 require(std::abs(node.gapVehicleSpeed(t.target_front)-14)<1e-6,"trajectory uses observed history, not ego/zero");
 require(node.gapFlowSpeedFor(0)<0,"fixed11 convergence disabled");
 node.gap_fixed_flow_enabled_=true;node.gap_select_flow_speed_lanes_={0};node.gap_select_flow_speed_mps_=11;
 require(node.gapFlowSpeedFor(0)==11,"explicit fixed-flow rollback");node.gap_fixed_flow_enabled_=false;
 SmpcDecisionNode::ActionSpec spec;spec.change_left=true;spec.action=SmpcDecisionNode::BehaviorAction::kChangeNow;
 spec.lane_change_start_sec=0;spec.lane_change_duration_sec=4;
 auto trajectory=[&](double speed){std::vector<SmpcDecisionNode::PredPose> out;
 for(int i=0;i<=20;++i){SmpcDecisionNode::PredPose p;p.valid=true;p.t=i*.1;p.x=100+speed*p.t;p.y=0;p.yaw=0;p.v=speed;out.push_back(p);}return out;};
 const bool safe=node.projectedTargetGapTtcSafe(t,trajectory(16),spec,false);
 if(!safe)throw std::runtime_error("observed-speed safe trajectory rejected: "+node.projected_gap_rejection_);
 require(!node.projectedTargetGapTtcSafe(t,trajectory(25),spec,false),"excessive acceleration toward front remains rejected");
 require(node.projected_gap_rejection_.find("id=30")!=std::string::npos,"reject includes vehicle ID");
 require(node.projected_gap_rejection_.find("source=position_history")!=std::string::npos,"reject includes speed source");
 t.target_front.s=106;t.target_front.x=106;
 // Renew a real observed close lead: same policy must reject the collision.
 t.header.stamp=ros::Time(50.1);t.target_front.velocity_valid=true;t.target_front.velocity_std=.3;t.target_front.v_long=14;
 node.gap_speed_policy_.update(t);ros::Time::setNow(ros::Time(50.1));
 require(!node.projectedTargetGapTtcSafe(t,trajectory(16),spec,false),"real close front remains rejected");
 std::remove(csv.c_str());std::cout<<"7 actual projected-gate scenarios passed\n";
}
