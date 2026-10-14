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
#include "smpc_lane_change/overtake_policy.hpp"

namespace fs = std::filesystem;

// SMPC-inspired lane-change decision node v2.
//
// This is still competition-practical rather than a full Gurobi/SOCP
// implementation of the paper.  The important upgrade from v1 is that the node
// no longer evaluates only scalar gap evolution.  It now:
//   1) loads the same lane CSVs as target_selector,
//   2) rolls out ego KEEP and CHANGE_LEFT candidate trajectories in map/UTM,
//   3) predicts target vehicles in Frenet s,d with multiple modes,
//   4) evaluates bbox collision probability across the whole horizon, and
//   5) publishes the lower-cost safe decision.
class SmpcDecisionNode {
 public:
  SmpcDecisionNode() : nh_(), pnh_("~") {
    pnh_.param("enabled", enabled_, true);
    pnh_.param("publish_rate_hz", publish_rate_hz_, 20.0);
    pnh_.param("target_timeout_sec", target_timeout_sec_, 0.5);
    pnh_.param<std::string>("waypoint_directory", waypoint_directory_, "");
    pnh_.param("lane_start_extension_m", lane_start_extension_m_, 80.0);
    pnh_.param("lane_start_extension_spacing_m",
               lane_start_extension_spacing_m_, 0.1);
    pnh_.param<std::string>("active_section_topic", active_section_topic_,
                            "/mission/active_section");
    pnh_.param("section_gate_enabled", section_gate_enabled_, true);
    pnh_.param("lane_change_min_index_csv", lane_change_min_index_csv_, 0);
    pnh_.param("lane_change_min_index", lane_change_min_index_, 11000);
    pnh_.param("lane_change_max_index_csv", lane_change_max_index_csv_, 3);
    pnh_.param("lane_change_max_index", lane_change_max_index_, -1);
    pnh_.param("section_gate_timeout_sec", section_gate_timeout_sec_, 1.0);

    pnh_.param("horizon_steps", horizon_steps_, 40);
    pnh_.param("prediction_dt_sec", prediction_dt_sec_, 0.2);
    pnh_.param("risk_epsilon", risk_epsilon_, 0.04);
    pnh_.param("urgent_risk_epsilon", urgent_risk_epsilon_, 0.15);
    // Keep /smpc/targets.nearby_vehicles available for RViz/diagnostics, but
    // allow A/B testing whether they participate in behavior-risk evaluation.
    pnh_.param("risk_include_nearby_vehicles", risk_include_nearby_vehicles_, true);
    pnh_.param("probabilistic_risk_gate_enabled", probabilistic_risk_gate_enabled_, true);

    pnh_.param("ego_length_m", ego_length_m_, 4.8);
    pnh_.param("ego_width_m", ego_width_m_, 1.9);
    pnh_.param<std::string>("mission_speed_topic",
                            mission_speed_topic_,
                            "/control_feedback/mission_speed_target_mps");
    pnh_.param("mission_speed_timeout_sec", mission_speed_timeout_sec_, 0.5);
    pnh_.param("mission_speed_fallback_mps", mission_speed_fallback_mps_, 10.0);
    pnh_.param("ego_prediction_use_acc_target_speed",
               ego_prediction_use_acc_target_speed_, true);
    pnh_.param<std::string>("ego_prediction_acc_target_speed_topic",
                            ego_prediction_acc_target_speed_topic_,
                            "/smpc/target_speed_mps");
    pnh_.param("ego_prediction_acc_target_speed_timeout_sec",
               ego_prediction_acc_target_speed_timeout_sec_, 0.5);
    pnh_.param("standstill_gap_m", standstill_gap_m_, 5.0);
    pnh_.param("time_headway_sec", time_headway_sec_, 1.0);
    pnh_.param("current_front_trigger_gap_m", current_front_trigger_gap_m_, 35.0);
    pnh_.param("current_front_safe_gap_m", current_front_safe_gap_m_, 7.0);
    pnh_.param("target_front_safe_gap_m", target_front_safe_gap_m_, 14.0);
    pnh_.param("target_rear_safe_gap_m", target_rear_safe_gap_m_, 9.0);
    pnh_.param("fixed_target_gap_gate_enabled", fixed_target_gap_gate_enabled_, true);
    pnh_.param("prepare_merge_front_safe_gap_m",
               prepare_merge_front_safe_gap_m_,
               12.0);
    pnh_.param("prepare_merge_rear_safe_gap_m",
               prepare_merge_rear_safe_gap_m_,
               7.0);
    pnh_.param("target_front_min_ttc_sec", target_front_min_ttc_sec_, 3.0);
    pnh_.param("target_rear_min_ttc_sec", target_rear_min_ttc_sec_, 3.0);
    pnh_.param("target_rear_min_headway_sec", target_rear_min_headway_sec_, 1.2);
    pnh_.param("clear_zone_ttc_relax_enabled", clear_zone_ttc_relax_enabled_, true);
    pnh_.param("clear_zone_front_min_ttc_sec", clear_zone_front_min_ttc_sec_, 1.2);
    pnh_.param("clear_zone_rear_min_ttc_sec", clear_zone_rear_min_ttc_sec_, 1.5);
    pnh_.param("clear_zone_rear_min_headway_sec",
               clear_zone_rear_min_headway_sec_, 0.6);
    pnh_.param("clear_zone_projected_rear_min_gap_m",
               clear_zone_projected_rear_min_gap_m_, 8.0);
    pnh_.param("clear_zone_merge_accel_min_gain_mps",
               clear_zone_merge_accel_min_gain_mps_, 0.5);
    // 점유영역(앞 8m / 뒤 15m)이 비어 있으면 시간지표 risk 문턱도 함께 푼다.
    // TTC/headway 만 풀고 risk_epsilon 을 그대로 두면, 간격이 넉넉해도
    // "앞차가 급제동할 수 있다"는 시나리오 확률만으로 계속 막힌다.
    //   실측 2026-09-15(16:49 백) t=23.0: 양보 완료 후 앞차 10.2 m,
    //   뒤 비어 있음, ttc_ok 1/1, zone_ttc_relaxed=1 이었는데
    //   legacy_change=0.295 > risk_eps=0.150 으로 거부됐다.
    // 궤적 겹침(clearance)과 최소 범퍼 간격은 그대로 유지되므로,
    // 실제로 겹치는 변경은 이 완화로도 통과하지 못한다.
    pnh_.param("clear_zone_risk_epsilon", clear_zone_risk_epsilon_, 0.35);
    pnh_.param("urgent_merge_front_safe_gap_m", urgent_merge_front_safe_gap_m_, 9.0);
    pnh_.param("urgent_merge_rear_safe_gap_m", urgent_merge_rear_safe_gap_m_, 5.5);
    pnh_.param("emergency_merge_front_safe_gap_m", emergency_merge_front_safe_gap_m_, 6.5);
    pnh_.param("emergency_merge_rear_safe_gap_m", emergency_merge_rear_safe_gap_m_, 4.5);
    pnh_.param("emergency_merge_risk_epsilon", emergency_merge_risk_epsilon_, 0.25);
    pnh_.param("deadline_relax_enabled", deadline_relax_enabled_, true);
    pnh_.param("deadline_relax_start_distance_m",
               deadline_relax_start_distance_m_, 120.0);
    pnh_.param("deadline_relax_full_distance_m",
               deadline_relax_full_distance_m_, 30.0);
    pnh_.param("deadline_relax_front_gap_floor_m",
               deadline_relax_front_gap_floor_m_, 8.0);
    pnh_.param("deadline_relax_rear_gap_floor_m",
               deadline_relax_rear_gap_floor_m_, 12.0);
    pnh_.param("deadline_relax_front_ttc_floor_sec",
               deadline_relax_front_ttc_floor_sec_, 2.0);
    pnh_.param("deadline_relax_rear_ttc_floor_sec",
               deadline_relax_rear_ttc_floor_sec_, 2.5);
    pnh_.param("deadline_relax_rear_headway_floor_sec",
               deadline_relax_rear_headway_floor_sec_, 0.8);
    // 시간 기반 완화. 차선변경을 원하는데(desire_change) 확정을 못 받은
    // 상태가 이어지면, 대기 시간에 비례해 안전영역 요구치를 낮춘다.
    // 거리 기반(deadline_relax_*)과 같은 0~1 스케일을 쓰고 둘 중 큰 값을
    // 채택하므로, 차선 끝이 멀어서 거리 완화가 0 인 구간에서도 동작한다.
    pnh_.param("wait_relax_enabled", wait_relax_enabled_, true);
    // 이 시간(초)까지는 완화 없음 - 유예 구간.
    pnh_.param("wait_relax_grace_sec", wait_relax_grace_sec_, 3.0);
    // 유예 이후 이 시간(초)에 걸쳐 0 -> 1 로 선형 완화.
    pnh_.param("wait_relax_ramp_sec", wait_relax_ramp_sec_, 3.0);
    pnh_.param("merge_priority_enabled", merge_priority_enabled_, true);
    pnh_.param("merge_priority_rear_safe_gap_m", merge_priority_rear_safe_gap_m_, 4.5);
    pnh_.param("merge_priority_rear_min_gap_m", merge_priority_rear_min_gap_m_, 4.0);
    pnh_.param("merge_priority_rear_max_closing_mps",
               merge_priority_rear_max_closing_mps_, 3.5);
    pnh_.param("merge_priority_rear_min_ttc_sec",
               merge_priority_rear_min_ttc_sec_, 1.8);
    pnh_.param("merge_priority_rear_risk_scale", merge_priority_rear_risk_scale_, 0.10);
    pnh_.param("merge_priority_nearby_rear_risk_scale",
               merge_priority_nearby_rear_risk_scale_, 0.05);
    pnh_.param("low_speed_rear_safety_enabled", low_speed_rear_safety_enabled_, true);
    pnh_.param("low_speed_rear_ego_speed_mps", low_speed_rear_ego_speed_mps_, 2.0);
    pnh_.param("low_speed_rear_safe_gap_m", low_speed_rear_safe_gap_m_, 10.0);
    pnh_.param("low_speed_rear_min_ttc_sec", low_speed_rear_min_ttc_sec_, 3.0);
    pnh_.param("ego_lane_priority_enabled", ego_lane_priority_enabled_, true);
    pnh_.param("ego_lane_priority_current_max_abs_d_m",
               ego_lane_priority_current_max_abs_d_m_, 1.25);
    pnh_.param("ego_lane_priority_front_hard_gap_m",
               ego_lane_priority_front_hard_gap_m_, 8.0);
    pnh_.param("ego_lane_priority_side_risk_scale",
               ego_lane_priority_side_risk_scale_, 0.08);

    pnh_.param("lane_width_m", lane_width_m_, 3.5);
    pnh_.param("lane_change_duration_sec", lane_change_duration_sec_, 2.5);
    pnh_.param("use_waypoint_linear_blend", use_waypoint_linear_blend_, true);
    pnh_.param("waypoint_blend_steps", waypoint_blend_steps_, 17);
    pnh_.param("waypoint_blend_rate_hz", waypoint_blend_rate_hz_, 25.0);
    pnh_.param("waypoint_blend_duration_sec", waypoint_blend_duration_sec_, 0.0);
    pnh_.param("lane_center_decay_tau_sec", lane_center_decay_tau_sec_, 1.2);

    // waypoint_system is the owner of the reference that the low-level MPC
    // will receive.  For a lane-change candidate, consume its preview instead
    // of independently blending two CSV lane centre-lines in this node.
    pnh_.param("ego_reference_preview_enabled", ego_reference_preview_enabled_, true);
    pnh_.param("ego_reference_preview_required_for_change",
               ego_reference_preview_required_for_change_, true);
    pnh_.param("ego_reference_preview_allow_fallback",
               ego_reference_preview_allow_fallback_, true);
    pnh_.param<std::string>("ego_reference_preview_topic",
                            ego_reference_preview_topic_, "/path_switch_preview");
    pnh_.param("ego_reference_preview_timeout_sec",
               ego_reference_preview_timeout_sec_, 0.50);
    pnh_.param("ego_reference_preview_motion_start_speed_mps",
               ego_reference_preview_motion_start_speed_mps_, 0.20);
    pnh_.param("ego_reference_preview_motion_start_distance_m",
               ego_reference_preview_motion_start_distance_m_, 0.30);
    pnh_.param("stopped_launch_connector_speed_cap_mps",
               stopped_launch_connector_speed_cap_mps_, 2.0);
    pnh_.param("stopped_launch_connector_max_accel_mps2",
               stopped_launch_connector_max_accel_mps2_, 1.5);
    pnh_.param("stopped_launch_connector_max_preswitch_distance_m",
               stopped_launch_connector_max_preswitch_distance_m_, 0.50);
    // A change candidate is not safe merely because its frozen source-lane
    // trajectory has low risk. It must reach the target lane within the
    // evaluated horizon. This rejects a stopped normal-BLEND whose clock has
    // not started; stopped endpoint changes instead use CONNECTOR preview.
    pnh_.param("ego_reference_preview_require_target_completion",
               ego_reference_preview_require_target_completion_, true);
    pnh_.param("ego_reference_preview_target_completion_max_abs_d_m",
               ego_reference_preview_target_completion_max_abs_d_m_, 0.75);
    ego_reference_preview_timeout_sec_ =
        std::max(0.0, ego_reference_preview_timeout_sec_);
    ego_reference_preview_motion_start_speed_mps_ =
        std::max(0.0, ego_reference_preview_motion_start_speed_mps_);
    ego_reference_preview_motion_start_distance_m_ =
        std::max(0.0, ego_reference_preview_motion_start_distance_m_);
    stopped_launch_connector_speed_cap_mps_ =
        std::max(0.0, stopped_launch_connector_speed_cap_mps_);
    stopped_launch_connector_max_accel_mps2_ =
        std::max(0.0, stopped_launch_connector_max_accel_mps2_);
    stopped_launch_connector_max_preswitch_distance_m_ =
        std::max(0.0, stopped_launch_connector_max_preswitch_distance_m_);

    // High-level preparation candidates.  These are evaluated in the
    // decision node, while the low-level longitudinal command remains owned
    // by ACC through BehaviorLongitudinalRequest.
    pnh_.param("behavior_planner_enabled", behavior_planner_enabled_, true);
    pnh_.param("behavior_execute_decel_hold", behavior_execute_decel_hold_, false);
    pnh_.param("behavior_execute_accel", behavior_execute_accel_, false);
    pnh_.param("behavior_require_legacy_risk_gate",
               behavior_require_legacy_risk_gate_, false);
    pnh_.param("behavior_min_signed_clearance_m",
               behavior_min_signed_clearance_m_, 0.0);
    pnh_.param("behavior_execution_delay_sec", behavior_execution_delay_sec_, 0.20);
    pnh_.param("behavior_post_merge_buffer_sec",
               behavior_post_merge_buffer_sec_, 2.0);
    pnh_.param("behavior_decel_accel_mps2", behavior_decel_accel_mps2_, -1.0);
    pnh_.param("behavior_yield_comfort_max_decel_mps2",
               behavior_yield_comfort_max_decel_mps2_, 0.8);
    pnh_.param("behavior_yield_max_speed_drop_mps",
               behavior_yield_max_speed_drop_mps_, 3.0);
    pnh_.param("side_vehicle_yield_use_safe_gaps", side_vehicle_yield_use_safe_gaps_, true);
    pnh_.param("side_vehicle_yield_window_m", side_vehicle_yield_window_m_, 16.0);
    pnh_.param("side_vehicle_yield_front_window_m", side_vehicle_yield_front_window_m_, side_vehicle_yield_window_m_);
    pnh_.param("side_vehicle_yield_rear_window_m", side_vehicle_yield_rear_window_m_, side_vehicle_yield_window_m_);
    pnh_.param("side_vehicle_yield_dynamic_window_enabled",
               side_vehicle_yield_dynamic_window_enabled_, true);
    pnh_.param("side_vehicle_yield_front_window_max_m",
               side_vehicle_yield_front_window_max_m_, 24.0);
    pnh_.param("side_vehicle_yield_rear_window_max_m",
               side_vehicle_yield_rear_window_max_m_, 30.0);
    pnh_.param("side_vehicle_yield_ego_speed_wide_mps",
               side_vehicle_yield_ego_speed_wide_mps_, 5.0);
    pnh_.param("side_vehicle_yield_ego_speed_narrow_mps",
               side_vehicle_yield_ego_speed_narrow_mps_, 15.0);
    pnh_.param("side_vehicle_yield_commit_enabled",
               side_vehicle_yield_commit_enabled_, true);
    pnh_.param("side_vehicle_yield_commit_max_duration_sec",
               side_vehicle_yield_commit_max_duration_sec_, 15.0);
    pnh_.param("side_vehicle_yield_min_speed_mps", side_vehicle_yield_min_speed_mps_, 10.0);
    pnh_.param("side_vehicle_yield_prep_accel_mps2", side_vehicle_yield_prep_accel_mps2_, -1.0);
    pnh_.param("target_front_early_yield_enabled",
               target_front_early_yield_enabled_, true);
    pnh_.param("target_front_early_yield_window_m",
               target_front_early_yield_window_m_, 30.0);
    pnh_.param("target_front_early_yield_min_closing_mps",
               target_front_early_yield_min_closing_mps_, 1.0);
    pnh_.param("target_front_early_yield_max_velocity_std_mps",
               target_front_early_yield_max_velocity_std_mps_, 3.0);
    pnh_.param("target_front_early_yield_max_observation_age_sec",
               target_front_early_yield_max_observation_age_sec_, 0.5);
    pnh_.param("target_front_early_yield_time_headway_sec",
               target_front_early_yield_time_headway_sec_, 1.0);
    pnh_.param("target_front_early_yield_gap_gain",
               target_front_early_yield_gap_gain_, 0.4);
    pnh_.param("target_front_early_yield_prep_accel_mps2",
               target_front_early_yield_prep_accel_mps2_, -1.2);
    pnh_.param("target_front_early_yield_min_speed_drop_mps",
               target_front_early_yield_min_speed_drop_mps_, 0.3);
    pnh_.param("side_vehicle_yield_adaptive_front_decel_enabled",
               side_vehicle_yield_adaptive_front_decel_enabled_, true);
    pnh_.param("side_vehicle_yield_adaptive_front_stop_margin_m",
               side_vehicle_yield_adaptive_front_stop_margin_m_, 2.0);
    pnh_.param("side_vehicle_rear_yield_speed_drop_mps",
               side_vehicle_rear_yield_speed_drop_mps_, 2.0);
    pnh_.param("behavior_hold_accel_mps2", behavior_hold_accel_mps2_, 0.0);
    pnh_.param("behavior_accel_accel_mps2", behavior_accel_accel_mps2_, 1.0);
    pnh_.param("behavior_delay_cost_weight", behavior_delay_cost_weight_, 1.0);
    pnh_.param("behavior_accel_cost_weight", behavior_accel_cost_weight_, 0.25);
    pnh_.param<std::string>("behavior_status_topic", behavior_status_topic_,
                            "/smpc/behavior_longitudinal_status");
    pnh_.param("behavior_status_timeout_sec", behavior_status_timeout_sec_, 0.35);
    if (!pnh_.getParam("behavior_yield_decel_candidates_mps2",
                       behavior_yield_decel_candidates_mps2_)) {
      behavior_yield_decel_candidates_mps2_ = {behavior_decel_accel_mps2_};
    }
    behavior_yield_comfort_max_decel_mps2_ =
        std::max(0.05, behavior_yield_comfort_max_decel_mps2_);
    behavior_yield_max_speed_drop_mps_ =
        std::max(0.0, behavior_yield_max_speed_drop_mps_);
    side_vehicle_rear_yield_speed_drop_mps_ =
        std::max(0.5, side_vehicle_rear_yield_speed_drop_mps_);
    side_vehicle_yield_front_window_m_ =
        std::max(0.0, side_vehicle_yield_front_window_m_);
    side_vehicle_yield_rear_window_m_ =
        std::max(0.0, side_vehicle_yield_rear_window_m_);
    side_vehicle_yield_front_window_max_m_ = std::max(
        side_vehicle_yield_front_window_m_,
        side_vehicle_yield_front_window_max_m_);
    side_vehicle_yield_rear_window_max_m_ = std::max(
        side_vehicle_yield_rear_window_m_,
        side_vehicle_yield_rear_window_max_m_);
    side_vehicle_yield_ego_speed_wide_mps_ =
        std::max(0.0, side_vehicle_yield_ego_speed_wide_mps_);
    side_vehicle_yield_ego_speed_narrow_mps_ = std::max(
        side_vehicle_yield_ego_speed_wide_mps_ + 0.1,
        side_vehicle_yield_ego_speed_narrow_mps_);
    side_vehicle_yield_commit_max_duration_sec_ =
        std::max(0.0, side_vehicle_yield_commit_max_duration_sec_);
    target_front_early_yield_window_m_ =
        std::max(0.0, target_front_early_yield_window_m_);
    target_front_early_yield_min_closing_mps_ =
        std::max(0.0, target_front_early_yield_min_closing_mps_);
    target_front_early_yield_max_velocity_std_mps_ =
        std::max(0.0, target_front_early_yield_max_velocity_std_mps_);
    target_front_early_yield_max_observation_age_sec_ =
        std::max(0.0, target_front_early_yield_max_observation_age_sec_);
    target_front_early_yield_time_headway_sec_ =
        std::max(0.0, target_front_early_yield_time_headway_sec_);
    target_front_early_yield_gap_gain_ =
        std::max(0.0, target_front_early_yield_gap_gain_);
    target_front_early_yield_prep_accel_mps2_ = -std::clamp(
        std::abs(target_front_early_yield_prep_accel_mps2_), 0.05,
        behavior_yield_comfort_max_decel_mps2_);
    side_vehicle_yield_adaptive_front_stop_margin_m_ =
        std::max(0.0, side_vehicle_yield_adaptive_front_stop_margin_m_);
    // 뒤차 양보는 그 차가 실제로 나를 따라잡고 있을 때만 의미가 있다.
    // 이 값(m/s)보다 느리게 접근하면(= 나보다 느리면) 양보하지 않는다.
    pnh_.param("side_vehicle_rear_yield_min_closing_mps",
               side_vehicle_rear_yield_min_closing_mps_, 0.5);
    pnh_.param("slow_rear_overtake_enabled", slow_rear_overtake_enabled_, true);
    pnh_.param("slow_rear_latch_enabled", slow_rear_latch_enabled_, true);
    pnh_.param("slow_rear_deadline_check_enabled", slow_rear_deadline_check_enabled_, true);
    pnh_.param("slow_rear_overtake_deadline_margin_m",
               slow_rear_overtake_deadline_margin_m_, 5.0);
    pnh_.param("slow_rear_overtake_accel_mps2", slow_rear_overtake_accel_mps2_, 1.0);
    pnh_.param("slow_rear_overtake_speed_gain_mps", slow_rear_overtake_speed_gain_mps_, 3.0);
    pnh_.param("slow_rear_overtake_max_sim_sec", slow_rear_overtake_max_sim_sec_, 10.0);
    pnh_.param("slow_rear_latch_timeout_sec", slow_rear_latch_timeout_sec_, 12.0);
    pnh_.param("slow_rear_latch_lost_sec", slow_rear_latch_lost_sec_, 1.0);
    pnh_.param("gap_select_enabled", gap_select_enabled_, true);
    // Read the same deployed ACC parameters, never a second fitted copy.
    nh_.param("/smpc_acc_speed_planner/yield_follow_min_desired_gap_m",merge_follow_gap_m_,8.5);
    nh_.param("/smpc_acc_speed_planner/yield_follow_close_gap_gain",merge_follow_close_gain_,.2);
    nh_.param("/smpc_acc_speed_planner/yield_follow_gap_gain",merge_follow_gain_,.5);
    nh_.param("/smpc_acc_speed_planner/yield_follow_max_catchup_mps",merge_follow_catchup_mps_,2.0);
    pnh_.param("side_vehicle_yield_front_min_headway_sec",
               side_vehicle_yield_front_min_headway_sec_, 0.5);
    pnh_.param("untrusted_near_gap_m", untrusted_near_gap_m_, 20.0);
    pnh_.param("untrusted_std_margin_cap_mps", untrusted_std_margin_cap_mps_, 2.0);
    pnh_.param("streak_count_new_targets_only", streak_count_new_targets_only_, true);
    pnh_.param("gap_select_publish_markers", gap_select_publish_markers_, true);
    pnh_.param("gap_select_current_lanes", gap_select_current_lanes_, std::vector<int>{0, 1, 2});
    pnh_.param("gap_select_open_gap_extra_m", gap_select_open_gap_extra_m_, 5.0);
    pnh_.param("gap_select_inner_margin_m", gap_select_inner_margin_m_, 3.0);
    pnh_.param("gap_select_position_gain", gap_select_position_gain_, 0.3);
    pnh_.param("gap_select_max_catchup_mps", gap_select_max_catchup_mps_, 4.0);
    pnh_.param("gap_select_min_speed_mps", gap_select_min_speed_mps_, 5.0);
    pnh_.param("gap_select_align_margin_m", gap_select_align_margin_m_, 1.0);
    pnh_.param("gap_select_align_speed_tol_mps", gap_select_align_speed_tol_mps_, 2.0);
    pnh_.param("gap_select_slot_margin_m", gap_select_slot_margin_m_, 1.0);
    pnh_.param("gap_select_min_slot_m", gap_select_min_slot_m_, 1.0);
    pnh_.param("gap_select_max_time_sec", gap_select_max_time_sec_, 12.0);
    pnh_.param("gap_response_enabled", gap_response_enabled_, true);
    pnh_.param("gap_shared_speed_enabled", gap_shared_speed_enabled_, true);
    pnh_.param("gap_selected_max_catchup_mps", gap_selected_max_catchup_mps_, 6.0);
    pnh_.param("gap_fixed_flow_enabled", gap_fixed_flow_enabled_, false);
    pnh_.param("gap_select_accel_mps2", gap_select_accel_mps2_, 1.2);
    // 2026-10-13 추월(기준 차량 앞 틈) 가속과 기준 차량 가속 판단(overtake_policy.hpp).
    pnh_.param("gap_select_overtake_accel_mps2", gap_select_overtake_accel_mps2_, 4.0);
    pnh_.param("gap_ref_accel_window_sec", gap_ref_accel_window_sec_, 1.0);
    pnh_.param("gap_ref_accel_deadband_mps2", overtake_policy_.deadband, 0.6);
    pnh_.param("gap_overtake_rel_accel_min_mps2", overtake_policy_.rel_min, 1.0);
    pnh_.param("gap_overtake_abort_confirm_sec", overtake_policy_.confirm_sec, 0.5);
    pnh_.param("gap_ref_speed_cap_mps", gap_ref_speed_cap_mps_, 27.8);
    pnh_.param("gap_overtake_accel_policy_enabled", gap_overtake_accel_policy_enabled_, true);
    overtake_policy_.ego_overtake_accel = gap_select_overtake_accel_mps2_;
    pnh_.param("gap_select_decel_mps2", gap_select_decel_mps2_, 1.5);
    // 구간별 제한속도 표(gapSpeedCapAt 주석). 상태별 속도와 구역 상한은 MPC 가 쓰는
    // 전역 파라미터(mpc_params.yaml)를 그대로 읽어 두 노드의 값이 어긋나지 않게 한다.
    pnh_.param("gap_select_flow_speed_lanes", gap_select_flow_speed_lanes_, std::vector<int>{});
    pnh_.param("gap_select_flow_speed_mps", gap_select_flow_speed_mps_, -1.0);
    pnh_.param("gap_select_flow_accel_mps2", gap_select_flow_accel_mps2_, 0.8);
    pnh_.param("gap_select_overtake_min_margin_m", gap_select_overtake_min_margin_m_, 0.0);
    pnh_.param("gap_speed_profile_enabled", gap_speed_profile_enabled_, true);
    pnh_.param("gap_speed_profile_lookahead_m", gap_speed_profile_lookahead_m_, 80.0);
    pnh_.param("lane_index_spacing_m", lane_index_spacing_m_, 0.1);
    pnh_.param("speed_exempt_start_csv", speed_exempt_start_csv_, 0);
    pnh_.param("speed_exempt_start_index", speed_exempt_start_index_, 11124);
    pnh_.param("speed_exempt_end_csv", speed_exempt_end_csv_, 3);
    pnh_.param("speed_exempt_end_index", speed_exempt_end_index_, 6196);
    nh_.param("mission_speed/default_mps", state_speed_default_mps_, 16.0);
    nh_.param("speed_limit/urban_max_mps", zone_urban_max_mps_, 16.0);
    nh_.param("speed_limit/highway_max_mps", zone_highway_max_mps_, 25.0);
    state_speed_mps_.assign(13, state_speed_default_mps_);
    for (size_t i = 0; i < state_speed_mps_.size(); ++i) {
      nh_.param("mission_speed/state_" + std::to_string(i) + "_mps", state_speed_mps_[i],
                state_speed_default_mps_);
    }
    pnh_.param("gap_select_effort_weight", gap_select_effort_weight_, 0.2);
    pnh_.param("gap_select_switch_cost_sec", gap_select_switch_cost_sec_, 1.5);
    pnh_.param("gap_select_match_dist_m", gap_select_match_dist_m_, 6.0);
    pnh_.param("gap_select_hold_sec", gap_select_hold_sec_, 0.6);
    pnh_.param("gap_select_switch_confirm_frames", gap_select_switch_confirm_frames_, 3);
    pnh_.param("gap_select_effort_switch_m", gap_select_effort_switch_m_, 10.0);
    pnh_.param("gap_select_cmd_rate_mps2", gap_select_cmd_rate_mps2_, 3.0);
    pnh_.param<std::string>("gap_select_side_policy", gap_select_side_policy_, "speed");
    pnh_.param("gap_select_own_lead_enabled", gap_select_own_lead_enabled_, true);
    pnh_.param("gap_select_own_lead_max_gap_m", gap_select_own_lead_max_gap_m_, 80.0);
    pnh_.param("gap_select_own_lead_standstill_m", gap_select_own_lead_standstill_m_, 6.0);
    pnh_.param("gap_select_own_lead_time_gap_sec", gap_select_own_lead_time_gap_sec_, 0.6);
    pnh_.param("gap_select_own_lead_gain", gap_select_own_lead_gain_, 0.3);
    pnh_.param("gap_select_own_lead_overtake_margin_mps", gap_select_own_lead_overtake_margin_mps_, 2.0);
    pnh_.param("gap_select_side_speed_deadband_mps", gap_select_side_speed_deadband_mps_, 1.0);
    pnh_.param("gap_select_ignore_stationary_behind_mps",
               gap_select_ignore_stationary_behind_mps_, 2.0);
    pnh_.param("lane_deadline_midpoint_enabled", lane_deadline_midpoint_enabled_, true);
    pnh_.param("lane_deadline_blend_fraction", lane_deadline_blend_fraction_, 0.5);
    pnh_.param("lane_deadline_margin_m", lane_deadline_margin_m_, 5.0);
    pnh_.param("lane_end_standstill_merge_enabled", lane_end_standstill_merge_enabled_, true);
    pnh_.param("lane_end_standstill_merge_max_speed_mps",
               lane_end_standstill_merge_max_speed_mps_, 1.0);
    pnh_.param("lane_end_standstill_rear_min_gap_m",
               lane_end_standstill_rear_min_gap_m_, 20.0);
    pnh_.param("lane_end_standstill_rear_min_ttc_sec",
               lane_end_standstill_rear_min_ttc_sec_, 2.5);
    pnh_.param("lead_follow_gap_gain", lead_follow_gap_gain_, 0.4);
    pnh_.param("lead_follow_min_cap_mps", lead_follow_min_cap_mps_, 5.0);
    pnh_.param("unknown_velocity_gap_fallback_enabled",
               unknown_velocity_gap_fallback_enabled_, true);
    pnh_.param("unknown_velocity_worst_closing_mps",
               unknown_velocity_worst_closing_mps_, 4.0);
    behavior_yield_decel_candidates_mps2_.erase(
        std::remove_if(behavior_yield_decel_candidates_mps2_.begin(),
                       behavior_yield_decel_candidates_mps2_.end(),
                       [](double a) { return !std::isfinite(a) || a >= -1e-3; }),
        behavior_yield_decel_candidates_mps2_.end());
    for (double& accel : behavior_yield_decel_candidates_mps2_) {
      accel = std::max(accel, -behavior_yield_comfort_max_decel_mps2_);
    }
    std::sort(behavior_yield_decel_candidates_mps2_.begin(),
              behavior_yield_decel_candidates_mps2_.end(), std::greater<double>());
    behavior_yield_decel_candidates_mps2_.erase(
        std::unique(behavior_yield_decel_candidates_mps2_.begin(),
                    behavior_yield_decel_candidates_mps2_.end()),
        behavior_yield_decel_candidates_mps2_.end());
    if (behavior_yield_decel_candidates_mps2_.empty()) {
      behavior_yield_decel_candidates_mps2_ = {
          -behavior_yield_comfort_max_decel_mps2_};
    }
    if (!pnh_.getParam("behavior_lc_start_times_sec", behavior_lc_start_times_sec_)) {
      behavior_lc_start_times_sec_ = {0.4, 0.8, 1.2, 1.6, 2.0};
    }
    behavior_lc_start_times_sec_.erase(
        std::remove_if(behavior_lc_start_times_sec_.begin(),
                       behavior_lc_start_times_sec_.end(),
                       [](double t) { return !std::isfinite(t) || t <= 0.0; }),
        behavior_lc_start_times_sec_.end());
    std::sort(behavior_lc_start_times_sec_.begin(), behavior_lc_start_times_sec_.end());
    behavior_lc_start_times_sec_.erase(
        std::unique(behavior_lc_start_times_sec_.begin(),
                    behavior_lc_start_times_sec_.end()),
        behavior_lc_start_times_sec_.end());

    pnh_.param("front_brake_probability", front_brake_probability_, 0.25);
    pnh_.param("rear_accel_probability", rear_accel_probability_, 0.25);
    pnh_.param("min_front_brake_probability", min_front_brake_probability_, 0.05);
    pnh_.param("max_front_brake_probability", max_front_brake_probability_, 0.65);
    pnh_.param("min_rear_accel_probability", min_rear_accel_probability_, 0.05);
    pnh_.param("max_rear_accel_probability", max_rear_accel_probability_, 0.65);
    pnh_.param("front_brake_accel_mps2", front_brake_accel_mps2_, -2.0);
    pnh_.param("rear_accel_mps2", rear_accel_mps2_, 1.5);
    pnh_.param("npc_longitudinal_event_modes_enabled",
               npc_longitudinal_event_modes_enabled_, false);
    pnh_.param("projected_gap_ttc_horizon_sec",
               projected_gap_ttc_horizon_sec_, 2.0);
    pnh_.param("nominal_target_accel_mps2", nominal_target_accel_mps2_, 0.0);
    pnh_.param("ego_assumed_accel_mps2", ego_assumed_accel_mps2_, 0.0);
    pnh_.param("ego_prediction_use_mission_speed_rollout",
               ego_prediction_use_mission_speed_rollout_,
               true);
    pnh_.param("ego_prediction_max_accel_mps2", ego_prediction_max_accel_mps2_, 2.5);
    pnh_.param("ego_prediction_max_decel_mps2", ego_prediction_max_decel_mps2_, 2.0);
    pnh_.param("probability_ttc_safe_sec", probability_ttc_safe_sec_, 5.0);
    pnh_.param("probability_ttc_critical_sec", probability_ttc_critical_sec_, 1.5);
    pnh_.param("probability_rel_speed_scale_mps", probability_rel_speed_scale_mps_, 8.0);
    pnh_.param("probability_accel_scale_mps2", probability_accel_scale_mps2_, 3.0);
    pnh_.param("observed_accel_alpha", observed_accel_alpha_, 0.35);
    pnh_.param("max_observed_accel_mps2", max_observed_accel_mps2_, 6.0);
    // 속도 추정이 불확실할 때 관측 가속도를 신뢰하지 않는다.
    //   실측 2026-09-15(16:49 백) target_front id0:
    //     t=24.27 v=10.94 (std 1.387) -> t=24.79 v=6.95 (std 1.015)
    //     0.52초에 -4.0 m/s -> 관측 가속도 -7.7 m/s^2 (측정 노이즈)
    //   그 값이 decel_pressure 를 최대(1.0)로 밀어 front_brake 확률을
    //   0.25 -> 0.37+ 로 올렸고, "앞차 급제동" 시나리오 가중으로
    //   legacy_change=0.295 > risk_eps=0.150 이 되어 차선변경이 막혔다.
    //   TTC 는 같은 구간에서 4.0초로 여유였다(요구 1.2) - 즉 TTC 가 아니라
    //   이 경로가 원인이었다.
    // velocity_std 가 이 값을 넘으면 그 프레임의 raw 가속도를 버린다.
    pnh_.param("observed_accel_max_velocity_std",
               observed_accel_max_velocity_std_, 0.8);

    pnh_.param("lateral_mode_enabled", lateral_mode_enabled_, true);
    pnh_.param("lateral_mode_probability", lateral_mode_probability_, 0.03);
    pnh_.param("max_lateral_mode_probability", max_lateral_mode_probability_, 0.08);
    pnh_.param("lateral_mode_v_threshold_mps", lateral_mode_v_threshold_mps_, 0.90);
    pnh_.param("yaw_lateral_threshold_rad", yaw_lateral_threshold_rad_, 0.28);
    pnh_.param("yaw_lateral_probability_gain", yaw_lateral_probability_gain_, 0.04);
    pnh_.param("max_lateral_prediction_mps", max_lateral_prediction_mps_, 0.45);
    pnh_.param("lateral_mode_allow_nearby", lateral_mode_allow_nearby_, false);
    pnh_.param("lateral_mode_allow_rear", lateral_mode_allow_rear_, false);
    pnh_.param("lateral_mode_allow_yaw_only", lateral_mode_allow_yaw_only_, false);
    pnh_.param("lateral_mode_min_abs_d_m", lateral_mode_min_abs_d_m_, 1.40);
    pnh_.param("lateral_mode_require_yaw_agreement",
               lateral_mode_require_yaw_agreement_, true);
    pnh_.param("lateral_mode_require_away_from_center",
               lateral_mode_require_away_from_center_, true);
    pnh_.param("tv_yaw_rate_weight", tv_yaw_rate_weight_, 0.05);
    pnh_.param("tv_csv_yaw_weight", tv_csv_yaw_weight_, 0.90);
    pnh_.param("tv_no_lateral_probability_floor",
               tv_no_lateral_probability_floor_, 0.90);
    pnh_.param("tv_yaw_rate_horizon_sec", tv_yaw_rate_horizon_sec_, 0.8);
    pnh_.param("max_abs_tv_yaw_rate", max_abs_tv_yaw_rate_, 0.6);

    pnh_.param("prediction_sigma_base_m", prediction_sigma_base_m_, 0.5);
    pnh_.param("prediction_sigma_growth_mps", prediction_sigma_growth_mps_, 0.35);
    pnh_.param("prediction_lateral_sigma_base_m", prediction_lateral_sigma_base_m_, 0.25);
    pnh_.param("prediction_lateral_sigma_growth_mps", prediction_lateral_sigma_growth_mps_, 0.12);
    pnh_.param("collision_margin_m", collision_margin_m_, 0.45);

    pnh_.param("lane_change_base_cost", lane_change_base_cost_, 0.5);
    pnh_.param("risk_cost_weight", risk_cost_weight_, 80.0);
    pnh_.param("gap_cost_weight", gap_cost_weight_, 2.5);
    pnh_.param("blocked_cost_weight", blocked_cost_weight_, 35.0);
    pnh_.param("lane_end_prepare_cost", lane_end_prepare_cost_, 60.0);
    pnh_.param("lane_end_urgent_cost", lane_end_urgent_cost_, 180.0);
    pnh_.param("lane_end_emergency_cost", lane_end_emergency_cost_, 500.0);
    pnh_.param("decision_hysteresis_cost", decision_hysteresis_cost_, 0.1);
    pnh_.param("commit_hysteresis_enabled", commit_hysteresis_enabled_, true);
    pnh_.param("commit_bias_cost", commit_bias_cost_, 7.0);
    pnh_.param("trusted_speed_gate_enabled", trusted_speed_gate_enabled_, true);
    pnh_.param("trusted_speed_max_std_mps", trusted_speed_max_std_mps_, 1.0);
    pnh_.param("trusted_speed_std_margin_scale",
               trusted_speed_std_margin_scale_, 1.0);
    pnh_.param("untrusted_speed_std_margin_scale",
               untrusted_speed_std_margin_scale_, 2.0);
    pnh_.param("commit_hold_sec", commit_hold_sec_, 2.0);
    pnh_.param("debug_publish_marker", debug_publish_marker_, true);
    pnh_.param("debug_marker_lifetime_sec", debug_marker_lifetime_sec_, 0.5);
    pnh_.param("debug_marker_height_m", debug_marker_height_m_, 4.0);
    pnh_.param("debug_prediction_publish_markers", debug_prediction_publish_markers_, true);
    pnh_.param("debug_prediction_marker_lifetime_sec", debug_prediction_marker_lifetime_sec_, 0.5);
    pnh_.param("debug_prediction_line_width_m", debug_prediction_line_width_m_, 0.25);
    pnh_.param("debug_prediction_z_m", debug_prediction_z_m_, 0.35);
    pnh_.param("debug_prediction_footprint_height_m", debug_prediction_footprint_height_m_, 0.15);
    pnh_.param("debug_prediction_ego_footprint_stride", debug_prediction_ego_footprint_stride_, 5);
    pnh_.param("debug_prediction_target_footprint_stride", debug_prediction_target_footprint_stride_, 5);
    pnh_.param("debug_prediction_target_max_vehicles", debug_prediction_target_max_vehicles_, 20);
    pnh_.param("debug_prediction_show_target_modes", debug_prediction_show_target_modes_, true);
    pnh_.param("debug_prediction_show_labels", debug_prediction_show_labels_, true);
    pnh_.param("debug_yield_zone_publish_markers", debug_yield_zone_publish_markers_, true);
    pnh_.param("change_feasible_confirm_count", change_feasible_confirm_count_, 3);

    lanes_loaded_ = loadLanes();
    if (!lanes_loaded_) {
      ROS_WARN("[smpc_decision] failed to load lane CSVs from '%s'; v2 decision will publish KEEP only",
               waypoint_directory_.c_str());
    }

    targets_sub_ = nh_.subscribe("/smpc/targets", 10, &SmpcDecisionNode::targetsCallback, this);
    odom_sub_ = nh_.subscribe("/odom", 10, &SmpcDecisionNode::odomCallback, this);
    mission_speed_sub_ =
        nh_.subscribe(mission_speed_topic_, 10, &SmpcDecisionNode::missionSpeedCallback, this);
    acc_target_speed_sub_ = nh_.subscribe(
        ego_prediction_acc_target_speed_topic_, 10,
        &SmpcDecisionNode::accTargetSpeedCallback, this);
    behavior_status_sub_ = nh_.subscribe(
        behavior_status_topic_, 10, &SmpcDecisionNode::behaviorStatusCallback, this);
    reference_preview_sub_ = nh_.subscribe(
        ego_reference_preview_topic_, 10,
        &SmpcDecisionNode::referencePreviewCallback, this);
    active_section_sub_ = nh_.subscribe(
        active_section_topic_, 10, &SmpcDecisionNode::activeSectionCallback, this);
    decision_pub_ = nh_.advertise<smpc_lane_change::LaneChangeDecision>("/smpc/decision", 10);
    behavior_plan_pub_ = nh_.advertise<smpc_lane_change::BehaviorPlan>("/smpc/behavior_plan", 10);
    behavior_request_pub_ = nh_.advertise<smpc_lane_change::BehaviorLongitudinalRequest>(
        "/smpc/behavior_longitudinal_request", 10);
    debug_pub_ = nh_.advertise<std_msgs::String>("/smpc/smpc_decision_debug", 10);
    debug_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/smpc/decision_debug_marker", 1);
    prediction_marker_pub_ =
        nh_.advertise<visualization_msgs::MarkerArray>("/smpc/prediction_markers", 1);
    yield_zone_marker_pub_ =
        nh_.advertise<visualization_msgs::MarkerArray>("/smpc/yield_zone_markers", 1);
    keep_path_pub_ = nh_.advertise<nav_msgs::Path>("/smpc/ego_keep_prediction", 1);
    change_path_pub_ = nh_.advertise<nav_msgs::Path>("/smpc/ego_change_prediction", 1);

    const double period = 1.0 / std::max(1.0, publish_rate_hz_);
    timer_ = nh_.createTimer(ros::Duration(period), &SmpcDecisionNode::timerCallback, this);

    const double horizon_sec = std::max(1, horizon_steps_) * prediction_dt_sec_;
    const double max_lc_start = behavior_lc_start_times_sec_.empty()
        ? 0.0
        : behavior_lc_start_times_sec_.back();
    const double required_candidate_horizon =
        behavior_execution_delay_sec_ + max_lc_start + egoChangeBlendDuration() +
        behavior_post_merge_buffer_sec_;
    if (behavior_planner_enabled_ && horizon_sec + 1e-6 < required_candidate_horizon) {
      ROS_WARN("[smpc_decision] behavior horizon %.2fs is shorter than the requested "
               "candidate coverage %.2fs; late t_LC candidates will be skipped",
               horizon_sec, required_candidate_horizon);
    }
    ROS_INFO("[smpc_decision] v3 enabled=%s lanes=%zu horizon=%d dt=%.2f eps=%.3f behavior=%s risk_nearby=%s ref_preview=%s%s",
             enabled_ ? "true" : "false", lanes_.size(), horizon_steps_, prediction_dt_sec_,
             risk_epsilon_, behavior_planner_enabled_ ? "true" : "false",
             risk_include_nearby_vehicles_ ? "true" : "false",
             ego_reference_preview_enabled_ ? "true" : "false",
             ego_reference_preview_required_for_change_ ? "/required" : "");
  }

 private:
  struct TvMode {
    std::string name;
    double probability{1.0};
    double accel_mps2{0.0};
    double lateral_rate_mps{0.0};
  };

  struct VehicleHistory {
    bool initialized{false};
    double v_long{0.0};
    double filtered_accel{0.0};
    ros::Time stamp;
  };

  struct PredPose {
    bool valid{false};
    double t{0.0};
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double s{0.0};
    double d{0.0};
    double v{0.0};
  };

  struct LongitudinalRollout {
    double v{0.0};
    double ds{0.0};
  };

  struct ReferencePolyline {
    std::vector<geometry_msgs::Pose> poses;
    std::vector<double> arc_m;

    bool hasPoint() const {
      return !poses.empty() && poses.size() == arc_m.size();
    }

    bool valid() const {
      return poses.size() >= 2 && poses.size() == arc_m.size() &&
          std::isfinite(arc_m.back()) && arc_m.back() > 1e-4;
    }
  };

  struct ReferencePreview {
    bool valid{false};
    uint8_t mode{waypoint_system::PathSwitchPreview::INVALID};
    int source_lane_id{-1};
    int target_lane_id{-1};
    double blend_duration_sec{0.0};
    double spatial_blend_length_m{0.0};
    uint32_t blend_max_points{0};
    double connector_join_arc_m{std::numeric_limits<double>::quiet_NaN()};
    ReferencePolyline source;
    ReferencePolyline target;
    ReferencePolyline connector;
    ros::Time header_stamp;
    ros::Time received_stamp;
  };

  struct OrientedBox {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double half_l{0.0};
    double half_w{0.0};
  };

  struct CollisionEval {
    double probability{0.0};
    double signed_clearance{std::numeric_limits<double>::infinity()};
  };

  enum class BehaviorAction {
    kKeep,
    kWaitForGap,
    kChangeNow,
    kYieldDecelThenChange,
    kHoldThenChange,
    kAccelThenChange,
  };

  struct ActionSpec {
    BehaviorAction action{BehaviorAction::kKeep};
    double prep_accel_mps2{0.0};
    double lane_change_start_sec{-1.0};
    double lane_change_duration_sec{0.0};
    bool change_left{false};
    std::string name{"keep"};
  };

  struct ModeRiskTrace {
    std::string name;
    double probability{0.0};
    std::vector<double> conditional_risk_by_step;
  };

  struct VehicleRiskTrace {
    int unique_id{-1};
    std::string role;
    double priority_scale{1.0};
    std::vector<double> mixed_risk_by_step;
    std::vector<ModeRiskTrace> modes;
  };

  struct TrajectoryEval {
    bool valid{false};
    // Legacy risk preserves the previous mode-max aggregation for shadow
    // comparison and conservative rollout gating during the first stage.
    double risk{1.0};
    double peak_step_risk{1.0};
    std::size_t peak_step{0};
    double min_signed_clearance{std::numeric_limits<double>::infinity()};
    std::vector<double> risk_by_step;
    std::vector<double> min_signed_clearance_by_step;
    std::vector<VehicleRiskTrace> vehicle_risk_traces;
    std::string detail;
  };

  struct CandidateEval {
    ActionSpec spec;
    TrajectoryEval trajectory;
    double target_speed_mps{0.0};
    double cost{std::numeric_limits<double>::infinity()};
    bool time_risk_safe{false};
    bool legacy_risk_safe{false};
    bool clearance_safe{false};
    bool projected_gap_ttc_safe{false};
    bool longitudinal_model_compatible{false};
    bool feasible{false};
  };

  void targetsCallback(const smpc_lane_change::TargetVehicleSet::ConstPtr& msg) {
    targets_ = *msg;
    gap_speed_policy_.update(*msg);
    targets_stamp_ = ros::Time::now();
    have_targets_ = true;
    updateVehicleHistories(*msg);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    ego_x_ = msg->pose.pose.position.x;
    ego_y_ = msg->pose.pose.position.y;
    const auto& q = msg->pose.pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    ego_yaw_ = std::atan2(siny_cosp, cosy_cosp);
    ego_speed_mps_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    ego_speed_slope_.add(msg->header.stamp.isZero() ? ros::Time::now().toSec() : msg->header.stamp.toSec(),
                         ego_speed_mps_, 2.0);
    have_ego_ = true;
  }

  void missionSpeedCallback(const std_msgs::Float64::ConstPtr& msg) {
    if (!std::isfinite(msg->data)) return;
    mission_speed_target_mps_ = std::max(0.0, msg->data);
    mission_speed_stamp_ = ros::Time::now();
    have_mission_speed_target_ = true;
  }

  void activeSectionCallback(const mission_supervisor::SectionState::ConstPtr& msg) {
    in_exempt_section_ = msg->speed_limit_exempt;
    section_has_smpc_executor_ =
        std::find(msg->executors.begin(), msg->executors.end(), "smpc") != msg->executors.end();
    active_section_csv_ = msg->csv;
    active_section_index_ = msg->index;
    active_section_index_valid_ = msg->index_valid;
    active_section_stamp_ = ros::Time::now();
    have_active_section_ = true;
  }

  // 목표 차로 차량의 종방향 속도를 신뢰도와 함께 읽는다.
  //
  // velocity_std 가 문턱을 넘거나 velocity_valid 가 false 면 추정이 크게
  // 틀릴 수 있으므로(실측 RMS 1.45~1.8 m/s), 자차 속도를 대신 쓴다.
  // 그러면 상대속도가 0 이 되어 '상대속도 때문에' 추가로 감속하거나
  // 가속하지 않고, 거리(gap) 기반 판단만 남는다.
  double leadTrustedLongSpeed(const smpc_lane_change::TargetVehicle& v) const {
    if(gap_shared_speed_enabled_) return gapVehicleSpeed(v);
    if (!std::isfinite(v.v_long)) return ego_speed_mps_;
    if (!trusted_speed_gate_enabled_) return v.v_long;
    const bool std_ok = !std::isfinite(v.velocity_std) ||
        trusted_speed_max_std_mps_ <= 0.0 ||
        v.velocity_std <= trusted_speed_max_std_mps_;
    if (v.velocity_valid && std_ok) return v.v_long;
    return ego_speed_mps_;
  }

  // 속도 불확실성을 폐쇄속도에 얹는 안전 마진.
  //
  // 기존에는 무조건 2*velocity_std 를 더했다. 그런데 std 가 신뢰 구간
  // (<= trusted_speed_max_std_mps) 안이면 실측 RMS 가 1.05 m/s 로 작아
  // 2 배까지 부풀릴 근거가 약하다 - 그만큼 없는 위험을 만들어 차선변경이
  // 불필요하게 막힌다(2026-09-15 20:38 백: 간격이 벌어지는 중인데도
  // projected_gap_ttc 가 계속 0).
  // 신뢰 구간에서는 배수를 낮추고, 그 밖에서는 기존 2 배를 유지한다.
  double velocityStdMargin(const smpc_lane_change::TargetVehicle& v) const {
    const double sd = std::max(0.0, v.velocity_std);
    if (!std::isfinite(sd)) return 0.0;
    const bool trusted = trusted_speed_gate_enabled_ && v.velocity_valid &&
        trusted_speed_max_std_mps_ > 0.0 && sd <= trusted_speed_max_std_mps_;
    const double k = trusted ? trusted_speed_std_margin_scale_
                             : untrusted_speed_std_margin_scale_;
    return k * sd;
  }

  // 차선변경 '준비'(틈 맞추기 속도 조절)만 하는 구간인가.
  //   missions.yaml 의 highway_approach 처럼 속도 예외는 아니지만 smpc 실행기가 켜진 구간.
  //   실행(laneChangeAllowedHere)은 여전히 속도 예외 구간에서만 된다. 목표 차로와 나란한
  //   구간이 106 m 뿐이라(0.csv 11124~12186), 그 전에 본선 속도와 틈을 맞춰 놓아야 한다
  //   (2026-10-01 21:37 백: 판단이 137 m 전에야 시작돼 차로 끝에서 정지).
  bool lanePrepOnlyHere() const {
    if (!section_gate_enabled_ || !have_active_section_) return false;
    if (section_gate_timeout_sec_ > 0.0 &&
        (ros::Time::now() - active_section_stamp_).toSec() > section_gate_timeout_sec_) {
      return false;
    }
    return !in_exempt_section_ && section_has_smpc_executor_;
  }

  // ---- 구간별 제한속도 표 ----
  // 틈 시뮬레이션(evaluateMergeGap)은 속도 상한으로 '지금 위치의' 미션 속도 하나를 끝까지
  // 썼다. 앞으로 지나갈 구간의 제한이 달라도 반영되지 않아,
  //   - 예외 구간 직전에서는 16 m/s 고정이라 11124 이후 가속분을 못 본다(앞지르기 틈을 불가능으로),
  //   - 고주로 직선(25 m/s)에서는 곧 나오는 곡선(20)이나 고주로 끝(16)에서도 25 까지
  //     가속한다고 본다(실제로 못 가는 틈을 가능으로).
  // 차로 CSV 의 mission_state 열과 규정 예외 구간으로 idx 별 제한속도를 구해 쓴다.
  bool inSpeedExemptZone(int lane, int idx) const {
    if (lane < speed_exempt_start_csv_ || lane > speed_exempt_end_csv_) return false;
    if (lane == speed_exempt_start_csv_ && idx < speed_exempt_start_index_) return false;
    if (lane == speed_exempt_end_csv_ && idx > speed_exempt_end_index_) return false;
    return true;
  }

  // MPC 의 missionStateTargetSpeedMps 와 같은 규칙: min(상태별 속도, 구역 상한).
  double speedLimitAt(int lane, int idx) const {
    const double zone =
        inSpeedExemptZone(lane, idx) ? zone_highway_max_mps_ : zone_urban_max_mps_;
    double state_v = state_speed_default_mps_;
    const auto it = lane_states_.find(lane);
    if (it != lane_states_.end() && !it->second.empty()) {
      const int i = std::clamp(idx, 0, static_cast<int>(it->second.size()) - 1);
      const int st = it->second[i];
      if (st >= 0 && st < static_cast<int>(state_speed_mps_.size())) state_v = state_speed_mps_[st];
    }
    return std::max(0.0, std::min(state_v, zone));
  }

  bool gapSpeedProfileUsable(const smpc_lane_change::TargetVehicleSet& t) const {
    return gap_speed_profile_enabled_ && have_active_section_ && active_section_index_valid_ &&
        active_section_csv_ == t.current_lane_id &&
        lane_states_.find(t.current_lane_id) != lane_states_.end() &&
        (ros::Time::now() - active_section_stamp_).toSec() <= 1.0;
  }

  // 자차가 지금 위치에서 pos[m] 나아간 지점에서 낼 수 있는 속도 상한.
  // 그 지점의 제한속도, 그리고 앞쪽에 더 낮은 제한이 있으면 거기까지 감속을 마칠 수 있는 속도.
  double gapSpeedCapAt(int lane, double pos) const {
    const double a = std::max(0.1, gap_select_decel_mps2_);
    const double spacing = std::max(1e-3, lane_index_spacing_m_);
    double cap = std::numeric_limits<double>::infinity();
    for (double x = 0.0; x <= gap_speed_profile_lookahead_m_ + 1e-6; x += 2.0) {
      const int idx = active_section_index_ +
          static_cast<int>(std::lround((std::max(0.0, pos) + x) / spacing));
      const double lim = speedLimitAt(lane, idx);
      cap = std::min(cap, std::sqrt(lim * lim + 2.0 * a * x));
    }
    return cap;
  }

  // 앞지르기 방향 규칙용: 제한속도 안에서 가속했을 때 차로 끝(또는 max_time)까지의 평균 속도.
  double gapOvertakeSpeed(const smpc_lane_change::TargetVehicleSet& t, double fallback) const {
    if (!gapSpeedProfileUsable(t)) return fallback;
    const double dt = 0.1;
    const double end = std::isfinite(t.distance_to_lane_end)
        ? std::max(0.0, t.distance_to_lane_end) : std::numeric_limits<double>::infinity();
    double v = std::max(0.0, ego_speed_mps_), pos = 0.0, time = 0.0;
    while (time < gap_select_max_time_sec_ && pos < end) {
      const double cap = independentAccLeadCap(t, gapSpeedCapAt(t.current_lane_id, pos));
      const double dv = std::clamp(cap - v, -gap_select_decel_mps2_ * dt,
                                   gap_select_accel_mps2_ * dt);
      pos += (v + 0.5 * dv) * dt;
      v += dv;
      time += dt;
    }
    return time > 1e-6 ? pos / time : fallback;
  }

  // 차로 CSV 의 mission_state 열(4번째)을 idx 순서로 읽는다.
  static std::vector<int> loadLaneStates(const std::string& path) {
    std::vector<int> states;
    std::ifstream in(path);
    std::string line;
    while (std::getline(in, line)) {
      std::stringstream ss(line);
      std::string cell;
      std::vector<std::string> cols;
      while (std::getline(ss, cell, ',')) cols.push_back(cell);
      if (cols.size() < 4) continue;
      try {
        const int idx = std::stoi(cols[0]);
        const int st = static_cast<int>(std::lround(std::stod(cols[3])));
        if (idx < 0) continue;
        if (idx >= static_cast<int>(states.size())) {
          states.resize(idx + 1, states.empty() ? st : states.back());
        }
        states[idx] = st;
      } catch (...) {
        continue;
      }
    }
    return states;
  }

  bool laneChangeAllowedHere() const {
    if (!section_gate_enabled_) return true;
    if (!have_active_section_) return false;
    if (section_gate_timeout_sec_ > 0.0 &&
        (ros::Time::now() - active_section_stamp_).toSec() >
            section_gate_timeout_sec_) {
      return false;
    }
    if (!in_exempt_section_) return false;
    // 합류 시작 전 조기 변경 금지.
    //
    // M6 은 판단 시간을 벌려고 idx 10600 부터 무장하는데, 그 자체가
    // '지금 변경해도 된다'는 뜻은 아니다. 실측(2026-09-15 20:25 백):
    // M6 무장(10500) 23 m 뒤인 idx 10831 에서 이미 0->1 을 실행해
    // 합류구간(~12185)이 끝나기 한참 전에 차로를 옮겼다.
    // 사유는 smpc_v2_lane_end_traj_safe - target_selector 가 CSV 전체 길이
    // (1218.6 m) 기준으로 distance_to_lane_end=159 m 를 내면서 마감 임박
    // 경로를 탔기 때문이다. 그 차로 끝은 합류 구간 끝과 다른 값이다.
    // 무장 위치와 별개로 실행 가능한 최소 지점을 따로 둔다.
    if (lane_change_min_index_ >= 0 && active_section_index_valid_ &&
        active_section_csv_ == lane_change_min_index_csv_ &&
        active_section_index_ < lane_change_min_index_) {
      return false;
    }
    // 이 csv 의 이 idx 이후에는 새 차선변경을 하지 않는다(2026-09-29,
    // 3.csv 5200 이후 톨게이트 구조물). ACC 중계는 constraint_to_speed_cap
    // 의 acc_disable_ranges 가 같은 구간에서 끈다.
    if (lane_change_max_index_ >= 0 && active_section_index_valid_ &&
        active_section_csv_ == lane_change_max_index_csv_ &&
        active_section_index_ >= lane_change_max_index_) {
      return false;
    }
    return true;
  }

  void accTargetSpeedCallback(const std_msgs::Float64::ConstPtr& msg) {
    if (!std::isfinite(msg->data)) return;
    acc_target_speed_mps_ = std::max(0.0, msg->data);
    acc_target_speed_stamp_ = ros::Time::now();
    have_acc_target_speed_ = true;
  }

  void behaviorStatusCallback(
      const smpc_lane_change::BehaviorLongitudinalStatus::ConstPtr& msg) {
    behavior_status_ = *msg;
    behavior_status_stamp_ = ros::Time::now();
    have_behavior_status_ = true;
  }

  static ReferencePolyline makeReferencePolyline(
      const std::vector<geometry_msgs::Pose>& poses) {
    ReferencePolyline out;
    out.poses = poses;
    out.arc_m.resize(out.poses.size(), 0.0);
    if (out.poses.size() < 2) return out;
    for (std::size_t i = 1; i < out.poses.size(); ++i) {
      const double dx = out.poses[i].position.x - out.poses[i - 1].position.x;
      const double dy = out.poses[i].position.y - out.poses[i - 1].position.y;
      const double ds = std::hypot(dx, dy);
      if (!std::isfinite(ds)) {
        out.poses.clear();
        out.arc_m.clear();
        return out;
      }
      out.arc_m[i] = out.arc_m[i - 1] + ds;
    }
    return out;
  }

  void referencePreviewCallback(
      const waypoint_system::PathSwitchPreview::ConstPtr& msg) {
    ReferencePreview preview;
    preview.valid = msg->valid;
    preview.mode = msg->mode;
    preview.source_lane_id = msg->source_path_number;
    preview.target_lane_id = msg->target_path_number;
    preview.blend_duration_sec = std::max(0.0, msg->blend_duration_sec);
    preview.spatial_blend_length_m = msg->spatial_blend_length_m;
    preview.blend_max_points = msg->blend_max_points;
    preview.connector_join_arc_m = msg->connector_join_arc_m;
    preview.source = makeReferencePolyline(msg->source_path);
    preview.target = makeReferencePolyline(msg->target_path);
    preview.connector = makeReferencePolyline(msg->connector_path);
    preview.header_stamp = msg->header.stamp;
    preview.received_stamp = ros::Time::now();
    reference_preview_ = std::move(preview);
    have_reference_preview_ = true;
  }

  double currentTargetSpeedMps() const {
    if (have_mission_speed_target_ &&
        (mission_speed_timeout_sec_ <= 0.0 ||
         (ros::Time::now() - mission_speed_stamp_).toSec() <= mission_speed_timeout_sec_)) {
      return mission_speed_target_mps_;
    }
    return mission_speed_fallback_mps_;
  }

  double independentAccLeadCap(const smpc_lane_change::TargetVehicleSet& t,
                               double fallback) const {
    if(!have_behavior_status_) return fallback;
    return smpc_lane_change::independentLeadCap(behavior_status_,t,ros::Time::now().toSec(),
        behavior_status_stamp_.toSec(),behavior_status_timeout_sec_,fallback);
  }

  // ACC can be holding ego at the pre-end wait line while mission speed still
  // asks for cruise.  Use the lower fresh target for safety rollout so a
  // stopped ego is not modelled as immediately accelerating into a fast rear.
  double egoPredictionTargetSpeedMps() const {
    const double mission_target = currentTargetSpeedMps();
    if (!ego_prediction_use_acc_target_speed_ || !have_acc_target_speed_) {
      return mission_target;
    }
    if (ego_prediction_acc_target_speed_timeout_sec_ > 0.0 &&
        (ros::Time::now() - acc_target_speed_stamp_).toSec() >
            ego_prediction_acc_target_speed_timeout_sec_) {
      return mission_target;
    }
    return std::min(mission_target, acc_target_speed_mps_);
  }

  // CHANGE_NOW releases the preparation YIELD cap.  If the fresh ACC target
  // is low only because ACC has acknowledged that exact cap, predict the
  // executable merge with the uncapped mission target.  A genuine lead,
  // lane-end or other ACC restriction is still retained because the status
  // marks it as an external speed limit.
  double egoChangePredictionTargetSpeedMps(
      const smpc_lane_change::TargetVehicleSet& t,
      const ActionSpec& spec) const {
    const double constrained_target = egoPredictionTargetSpeedMps();
    if (spec.action != BehaviorAction::kChangeNow) {
      return constrained_target;
    }
    const ros::Time now = ros::Time::now();
    const bool status_fresh = have_behavior_status_ &&
        (behavior_status_timeout_sec_ <= 0.0 ||
         (now - behavior_status_stamp_).toSec() <= behavior_status_timeout_sec_);
    // Keep a newly-approved launch model stable while ACC ramps its filtered
    // target.  Without this short latch, the first CHANGE_LEFT cycle releases
    // the cap, the next cycle sees only ~0.15 m/s of recovery, and the merge
    // candidate incorrectly falls back to "no acceleration".
    const bool committed_launch = commit_hysteresis_enabled_ &&
        commit_target_lane_id_ == t.target_lane_id &&
        commit_target_lane_id_ >= 0 &&
        (now - commit_stamp_).toSec() <= commit_hold_sec_;
    const bool own_cap_is_only_limit =
        behaviorStatusConfirmsLastIssuedPreparation(t) || committed_launch;
    const bool external_limit_active =
        status_fresh && behavior_status_.external_speed_limit_active;
    if (!own_cap_is_only_limit || external_limit_active) {
      return constrained_target;
    }
    // 양보 cap 을 풀 때 미션속도(21 m/s)로 바로 올리지 않는다.
    //
    // 여기서 예측한 속도는 '변경 중 자차가 어디까지 갈 것인가'에 그대로
    // 쓰인다. 21 로 두면 실제로는 앞차 때문에 낼 수 없는 속도로 전진한다고
    // 보게 되어, 목표 차로 앞차를 따라잡는 것으로 예측되고 clearance 가
    // 음수로 떨어진다(2026-09-15 20:17 백에서 관측된 패턴).
    // 현재/목표 차로 앞차를 실제로 추종할 수 있는 속도로 상한을 둔다.
    const double release_target =
        std::max(constrained_target, currentTargetSpeedMps());
    const double follow_cap = leadFollowSpeedCapMps(t);
    return std::min(release_target, follow_cap);
  }

  // 앞차를 추종할 때 낼 수 있는 속도 상한.
  //
  // 현재 차로 앞차와 목표 차로 앞차 중 더 제약이 큰 쪽을 쓴다.
  // 간격이 목표(standstill + headway*v)보다 넉넉하면 앞차 속도보다 조금
  // 빠르게 갈 수 있고, 빠듯하면 앞차 속도로 수렴한다. 속도를 신뢰할 수
  // 없는 차량은 상한 계산에서 제외한다(leadTrustedLongSpeed 가 ego 를
  // 돌려주므로 자연히 무제한에 가까워진다).
  double leadFollowSpeedCapMps(
      const smpc_lane_change::TargetVehicleSet& t) const {
    double cap = std::numeric_limits<double>::infinity();
    const auto consider = [&](const smpc_lane_change::TargetVehicle& v) {
      if (!v.valid) return;
      const double gap = frontBumperGap(v);
      if (!std::isfinite(gap) || gap <= 0.0) return;
      const double lead_v = std::max(0.0, leadTrustedLongSpeed(v));
      const double desired =
          standstill_gap_m_ + time_headway_sec_ * ego_speed_mps_;
      const double gap_err = gap - desired;
      // 간격 여유를 속도 여유로 환산한다(ACC 의 gap_gain 과 같은 형태).
      const double allowance =
          std::max(0.0, lead_follow_gap_gain_ * gap_err);
      cap = std::min(cap, lead_v + allowance);
    };
    consider(t.current_front);
    consider(t.target_front);
    if (!std::isfinite(cap)) return std::numeric_limits<double>::infinity();
    return std::max(cap, lead_follow_min_cap_mps_);
  }

  void updateOneVehicleHistory(const smpc_lane_change::TargetVehicle& v,
                               const ros::Time& stamp) {
    if (!v.valid || v.unique_id < 0) return;
    // 판단용 가속도는 속도 표준편차로 거르지 않고 최근 window 초 기울기로 잡음을 흡수한다.
    if (std::isfinite(v.v_long)) ref_speed_slope_[v.unique_id].add(stamp.toSec(), v.v_long, 2.0);
    auto& h = vehicle_history_[v.unique_id];
    if (!h.initialized) {
      h.initialized = true;
      h.v_long = v.v_long;
      h.filtered_accel = 0.0;
      h.stamp = stamp;
      return;
    }

    const double dt = (stamp - h.stamp).toSec();
    // 속도 추정이 흔들리는 프레임은 가속도 계산에서 통째로 뺀다. 두 프레임의
    // 속도 차로 만드는 값이라 노이즈가 dt 로 나뉘며 증폭된다(0.52초에 4 m/s
    // 튀면 -7.7 m/s^2). alpha=0.35 EMA 는 그 한 번을 5프레임 넘게 끌고 간다.
    const bool velocity_trustworthy =
        v.velocity_valid && std::isfinite(v.velocity_std) &&
        (observed_accel_max_velocity_std_ <= 0.0 ||
         v.velocity_std <= observed_accel_max_velocity_std_);
    if (dt > 1e-3 && dt < 1.0 && velocity_trustworthy) {
      const double raw_accel = std::clamp((v.v_long - h.v_long) / dt,
                                          -max_observed_accel_mps2_,
                                          max_observed_accel_mps2_);
      const double alpha = std::clamp(observed_accel_alpha_, 0.0, 1.0);
      h.filtered_accel = alpha * raw_accel + (1.0 - alpha) * h.filtered_accel;
    }

    h.v_long = v.v_long;
    h.stamp = stamp;
  }

  void updateVehicleHistories(const smpc_lane_change::TargetVehicleSet& t) {
    const ros::Time stamp = t.header.stamp.isZero() ? ros::Time::now() : t.header.stamp;
    const smpc_lane_change::TargetVehicle candidates[3] = {
        t.current_front, t.target_front, t.target_rear};
    for (const auto& v : candidates) updateOneVehicleHistory(v, stamp);
    for (const auto& v : t.nearby_vehicles) updateOneVehicleHistory(v, stamp);
  }

  bool loadLanes() {
    lanes_.clear();
    lane_states_.clear();
    if (waypoint_directory_.empty() || !fs::is_directory(waypoint_directory_)) return false;

    std::vector<fs::path> files;
    for (const auto& entry : fs::directory_iterator(waypoint_directory_)) {
      if (entry.is_regular_file() && entry.path().extension() == ".csv") files.push_back(entry.path());
    }
    std::sort(files.begin(), files.end(), [](const fs::path& a, const fs::path& b) {
      try {
        return std::stoi(a.stem().string()) < std::stoi(b.stem().string());
      } catch (...) {
        return a.filename().string() < b.filename().string();
      }
    });

    for (const auto& file : files) {
      int lane_id = -1;
      try {
        lane_id = std::stoi(file.stem().string());
      } catch (...) {
        continue;
      }
      smpc_lane_change::LanePath lane;
      if (lane.loadCsv(file.string())) {
        lane_states_[lane_id] = loadLaneStates(file.string());
        if (lane_id > 0) {
          bool curved_enabled = true;
          double offset = 3.5, join_error = 1.5, join_yaw = 0.35;
          ros::param::param<bool>("/local_path_publisher_node/path_switch_target_offset_prelude_enabled_", curved_enabled, true);
          ros::param::param<double>("/local_path_publisher_node/path_switch_lane_offset_m_", offset, 3.5);
          ros::param::param<double>("/local_path_publisher_node/path_switch_target_offset_prelude_max_join_error_m_", join_error, 1.5);
          ros::param::param<double>("/local_path_publisher_node/path_switch_target_offset_prelude_max_join_heading_error_rad_", join_yaw, 0.35);
          const auto previous = lanes_.find(lane_id - 1);
          const bool curved = curved_enabled && previous != lanes_.end() &&
              lane.prependOffsetExtension(previous->second, lane_start_extension_m_,
                                          offset, join_error, join_yaw);
          if (!curved) lane.prependLinearExtension(lane_start_extension_m_,
                                                   lane_start_extension_spacing_m_);
          ROS_INFO("lane %d prelude: %s", lane_id, curved ? "source curve offset" : "linear fallback");
        }
        lanes_[lane_id] = std::move(lane);
      }
    }
    return !lanes_.empty();
  }

  const smpc_lane_change::LanePath* laneFor(int lane_id) const {
    const auto it = lanes_.find(lane_id);
    return it == lanes_.end() ? nullptr : &it->second;
  }

  static double normalCdf(double z) {
    return 0.5 * std::erfc(-z / std::sqrt(2.0));
  }

  static double clampProbability(double p) {
    return std::clamp(p, 0.0, 1.0);
  }

  static double smoothstep(double x) {
    x = std::clamp(x, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
  }

  static double angleLerp(double a, double b, double alpha) {
    return smpc_lane_change::wrapToPi(a + alpha * smpc_lane_change::wrapToPi(b - a));
  }

  static double projectPolylineArc(const ReferencePolyline& path,
                                   double x,
                                   double y) {
    if (!path.hasPoint()) return std::numeric_limits<double>::quiet_NaN();
    if (path.poses.size() == 1) return 0.0;
    double best_sq = std::numeric_limits<double>::infinity();
    double best_arc = 0.0;
    for (std::size_t i = 1; i < path.poses.size(); ++i) {
      const auto& a = path.poses[i - 1].position;
      const auto& b = path.poses[i].position;
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double len_sq = dx * dx + dy * dy;
      if (len_sq < 1e-8) continue;
      const double u = std::clamp(((x - a.x) * dx + (y - a.y) * dy) / len_sq,
                                  0.0, 1.0);
      const double px = a.x + u * dx;
      const double py = a.y + u * dy;
      const double sq = (x - px) * (x - px) + (y - py) * (y - py);
      if (sq < best_sq) {
        best_sq = sq;
        best_arc = path.arc_m[i - 1] +
            u * (path.arc_m[i] - path.arc_m[i - 1]);
      }
    }
    return std::isfinite(best_sq) ? best_arc : std::numeric_limits<double>::quiet_NaN();
  }

  static PredPose samplePolyline(const ReferencePolyline& path, double arc_m) {
    PredPose out;
    if (!path.hasPoint() || !std::isfinite(arc_m) || arc_m < -1e-4 ||
        arc_m > path.arc_m.back() + 1e-4) {
      return out;
    }
    if (path.poses.size() == 1) {
      if (std::abs(arc_m) > 1e-4) return out;
      const auto& pose = path.poses.front();
      out.valid = true;
      out.x = pose.position.x;
      out.y = pose.position.y;
      const auto& q = pose.orientation;
      out.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      return out;
    }
    arc_m = std::clamp(arc_m, 0.0, path.arc_m.back());
    const auto it = std::upper_bound(path.arc_m.begin(), path.arc_m.end(), arc_m);
    const std::size_t hi = std::clamp<std::size_t>(
        static_cast<std::size_t>(std::distance(path.arc_m.begin(), it)),
        1, path.arc_m.size() - 1);
    const std::size_t lo = hi - 1;
    const double segment = std::max(1e-6, path.arc_m[hi] - path.arc_m[lo]);
    const double u = std::clamp((arc_m - path.arc_m[lo]) / segment, 0.0, 1.0);
    const auto& a = path.poses[lo].position;
    const auto& b = path.poses[hi].position;
    out.valid = true;
    out.x = (1.0 - u) * a.x + u * b.x;
    out.y = (1.0 - u) * a.y + u * b.y;
    // Use the geometric tangent.  The same tangent is written by
    // local_path_publisher after it blends x/y, so yaw cannot describe a
    // different curve from the collision footprint.
    out.yaw = std::atan2(b.y - a.y, b.x - a.x);
    return out;
  }

  static ReferencePolyline blendReferencePreview(const ReferencePreview& preview,
                                                  double alpha) {
    ReferencePolyline out;
    if (!preview.source.valid() || !preview.target.valid()) return out;
    alpha = std::clamp(alpha, 0.0, 1.0);
    const std::size_t common = std::min(preview.source.poses.size(),
                                        preview.target.poses.size());
    const std::size_t max_points = preview.blend_max_points > 0
        ? static_cast<std::size_t>(preview.blend_max_points) : common;
    const std::size_t blend_size = std::min(common, max_points);
    if (blend_size < 2) return out;

    out.poses.reserve(preview.target.poses.size());
    for (std::size_t i = 0; i < blend_size; ++i) {
      geometry_msgs::Pose p = preview.target.poses[i];
      p.position.x = (1.0 - alpha) * preview.source.poses[i].position.x +
          alpha * preview.target.poses[i].position.x;
      p.position.y = (1.0 - alpha) * preview.source.poses[i].position.y +
          alpha * preview.target.poses[i].position.y;
      p.position.z = (1.0 - alpha) * preview.source.poses[i].position.z +
          alpha * preview.target.poses[i].position.z;
      out.poses.push_back(p);
    }
    for (std::size_t i = blend_size; i < preview.target.poses.size(); ++i) {
      out.poses.push_back(preview.target.poses[i]);
    }
    out.arc_m.resize(out.poses.size(), 0.0);
    for (std::size_t i = 1; i < out.poses.size(); ++i) {
      const double dx = out.poses[i].position.x - out.poses[i - 1].position.x;
      const double dy = out.poses[i].position.y - out.poses[i - 1].position.y;
      const double ds = std::hypot(dx, dy);
      if (!std::isfinite(ds)) {
        out.poses.clear();
        out.arc_m.clear();
        return out;
      }
      out.arc_m[i] = out.arc_m[i - 1] + ds;
    }
    return out;
  }

  bool referencePreviewFreshFor(const smpc_lane_change::TargetVehicleSet& t) const {
    if (!ego_reference_preview_enabled_) return false;
    if (!have_reference_preview_ || !reference_preview_.valid) return false;
    const ros::Time now = ros::Time::now();
    const auto fresh = [&](const ros::Time& stamp) {
      return ego_reference_preview_timeout_sec_ <= 0.0 ||
          (!stamp.isZero() && (now - stamp).toSec() <= ego_reference_preview_timeout_sec_);
    };
    if (!fresh(reference_preview_.received_stamp)) return false;
    if (!reference_preview_.header_stamp.isZero() &&
        !fresh(reference_preview_.header_stamp)) return false;
    if (reference_preview_.source_lane_id != t.current_lane_id ||
        reference_preview_.target_lane_id != t.target_lane_id) return false;
    if (reference_preview_.mode == waypoint_system::PathSwitchPreview::BLEND) {
      return reference_preview_.source.valid() && reference_preview_.target.valid() &&
          reference_preview_.blend_duration_sec > 1e-3;
    }
    if (reference_preview_.mode == waypoint_system::PathSwitchPreview::CONNECTOR) {
      return reference_preview_.source.hasPoint() && reference_preview_.connector.valid() &&
          std::isfinite(reference_preview_.connector_join_arc_m) &&
          reference_preview_.connector_join_arc_m > 1e-3 &&
          reference_preview_.connector_join_arc_m <=
              reference_preview_.connector.arc_m.back() + 1e-3;
    }
    if (reference_preview_.mode == waypoint_system::PathSwitchPreview::DIRECT) {
      return reference_preview_.source.valid() && reference_preview_.target.valid();
    }
    return false;
  }

  static const char* referencePreviewModeName(uint8_t mode) {
    switch (mode) {
      case waypoint_system::PathSwitchPreview::BLEND:
        return "blend";
      case waypoint_system::PathSwitchPreview::CONNECTOR:
        return "connector";
      case waypoint_system::PathSwitchPreview::DIRECT:
        return "direct";
      default:
        return "invalid";
    }
  }

  static double constantAccelDistance(double v0, double accel, double tau) {
    v0 = std::max(0.0, v0);
    tau = std::max(0.0, tau);
    if (accel < -1e-6) {
      const double t_stop = v0 / -accel;
      const double used_t = std::min(tau, t_stop);
      return std::max(0.0, v0 * used_t + 0.5 * accel * used_t * used_t);
    }
    return std::max(0.0, v0 * tau + 0.5 * accel * tau * tau);
  }

  LongitudinalRollout egoLongitudinalRollout(double tau,
                                             double target_speed_mps) const {
    LongitudinalRollout out;
    const double v0 = std::max(0.0, ego_speed_mps_);
    tau = std::max(0.0, tau);

    if (!ego_prediction_use_mission_speed_rollout_) {
      out.v = std::max(0.0, v0 + ego_assumed_accel_mps2_ * tau);
      out.ds = constantAccelDistance(v0, ego_assumed_accel_mps2_, tau);
      return out;
    }

    const double v_ref = std::max(0.0, target_speed_mps);
    constexpr double kSpeedEps = 0.05;

    double accel = 0.0;
    if (v_ref > v0 + kSpeedEps) {
      accel = gap_response_enabled_ && gap_latch_valid_
        ? std::max(0.0,gap_select_accel_mps2_) : std::max(0.0, ego_prediction_max_accel_mps2_);
    } else if (v_ref < v0 - kSpeedEps) {
      accel = -(gap_response_enabled_ && gap_latch_valid_
        ? std::max(0.0,gap_select_decel_mps2_) : std::max(0.0, ego_prediction_max_decel_mps2_));
    }

    if (std::abs(accel) < 1e-6) {
      out.v = v0;
      out.ds = v0 * tau;
      return out;
    }

    const double t_to_ref = std::max(0.0, (v_ref - v0) / accel);
    const double accel_t = std::min(tau, t_to_ref);
    out.v = std::max(0.0, v0 + accel * accel_t);
    out.ds = std::max(0.0, v0 * accel_t + 0.5 * accel * accel_t * accel_t);

    if (tau > accel_t) {
      out.v = v_ref;
      out.ds += v_ref * (tau - accel_t);
    }

    return out;
  }

  double egoChangeBlendDuration() const {
    if (ego_reference_preview_enabled_ && have_reference_preview_ &&
        reference_preview_.valid &&
        reference_preview_.mode == waypoint_system::PathSwitchPreview::BLEND &&
        reference_preview_.blend_duration_sec > 1e-3) {
      return reference_preview_.blend_duration_sec;
    }
    if (waypoint_blend_duration_sec_ > 1e-3) {
      return waypoint_blend_duration_sec_;
    }
    if (use_waypoint_linear_blend_ && waypoint_blend_steps_ > 0 && waypoint_blend_rate_hz_ > 1e-3) {
      return static_cast<double>(waypoint_blend_steps_) / waypoint_blend_rate_hz_;
    }
    return lane_change_duration_sec_;
  }

  double egoChangeAlpha(double tau, double start_sec, double duration_sec) const {
    const double duration = std::max(0.1, duration_sec);
    const double raw = std::clamp((tau - std::max(0.0, start_sec)) / duration, 0.0, 1.0);
    return use_waypoint_linear_blend_ ? raw : smoothstep(raw);
  }

  double effectiveLaneChangeStartSec(const ActionSpec& spec) const {
    if (!spec.change_left) return std::numeric_limits<double>::infinity();
    return std::max(0.0, spec.lane_change_start_sec) +
           std::max(0.0, behavior_execution_delay_sec_);
  }

  // A YIELD command is refreshed at 20 Hz while its future lane change is
  // repeatedly re-evaluated.  Anchor the speed floor to the beginning of
  // that continuous episode, rather than to the current speed every cycle;
  // otherwise a harmless, short yield can ratchet a highway-speed vehicle all
  // the way down to a crawl without a new safety reason.
  double behaviorYieldSpeedFloorMps(const ActionSpec& spec) const {
    if (spec.action != BehaviorAction::kYieldDecelThenChange) return 0.0;
    const double reference_speed =
        (last_behavior_action_ == BehaviorAction::kYieldDecelThenChange &&
         std::isfinite(behavior_yield_episode_start_speed_mps_))
            ? behavior_yield_episode_start_speed_mps_
            : std::max(0.0, ego_speed_mps_);
    return std::max(0.0, reference_speed - behavior_yield_max_speed_drop_mps_);
  }

  LongitudinalRollout boundedPreparationRollout(double duration_sec,
                                                double requested_accel_mps2,
                                                double speed_cap_mps,
                                                double minimum_speed_mps) const {
    LongitudinalRollout out;
    const double v0 = std::max(0.0, ego_speed_mps_);
    const double duration = std::max(0.0, duration_sec);
    const double speed_cap = std::max(0.0, speed_cap_mps);
    const double accel = requested_accel_mps2;

    if (duration <= 1e-6) {
      out.v = v0;
      return out;
    }

    // HOLD means "do not add a longitudinal maneuver", not an instantaneous
    // jump to the latest ACC reference.  The candidate eligibility check
    // below rejects a HOLD/YIELD plan when normal ACC is already braking, so
    // keeping v0 here matches the separate behavior-rate contract in ACC.
    if (std::abs(accel) <= 1e-6) {
      out.v = v0;
      out.ds = v0 * duration;
      return out;
    }

    if (accel > 0.0) {
      if (v0 >= speed_cap) {
        out.v = speed_cap;
        out.ds = speed_cap * duration;
        return out;
      }
      const double t_to_cap = (speed_cap - v0) / accel;
      const double accel_time = std::clamp(t_to_cap, 0.0, duration);
      out.v = std::min(speed_cap, v0 + accel * accel_time);
      out.ds = v0 * accel_time + 0.5 * accel * accel_time * accel_time;
      if (duration > accel_time) out.ds += out.v * (duration - accel_time);
      return out;
    }

    // For a renewed YIELD request, the cap advances with each receding-
    // horizon cycle.  Its conservative closed-loop model is therefore a
    // persistent requested deceleration, limited only by zero speed.  The
    // normal ACC reference is checked separately before such a plan can be
    // executed, so it must not become an instantaneous lower-speed jump here.
    const double speed_floor = std::clamp(minimum_speed_mps, 0.0, v0);
    const double t_to_floor = (v0 - speed_floor) / std::max(1e-6, -accel);
    const double accel_time = std::min(duration, t_to_floor);
    out.v = std::max(speed_floor, v0 + accel * accel_time);
    out.ds = std::max(0.0, v0 * accel_time + 0.5 * accel * accel_time * accel_time);
    if (duration > accel_time) out.ds += out.v * (duration - accel_time);
    return out;
  }

  LongitudinalRollout actionLongitudinalRollout(double tau,
                                                double safe_speed_cap_mps,
                                                const ActionSpec& spec) const {
    if (spec.action == BehaviorAction::kKeep ||
        spec.action == BehaviorAction::kWaitForGap ||
        spec.action == BehaviorAction::kChangeNow) {
      return egoLongitudinalRollout(tau, safe_speed_cap_mps);
    }

    // A preparation request is re-evaluated at 20 Hz.  If it remains
    // selected, ACC receives a renewed cap and therefore continues the
    // requested yield/hold action instead of silently recovering at t_LC.
    // Model that persistent worst case across the horizon; it avoids
    // underestimating a fast rear vehicle after a deferred lane change.
    return boundedPreparationRollout(std::max(0.0, tau),
                                     spec.prep_accel_mps2,
                                     safe_speed_cap_mps,
                                     behaviorYieldSpeedFloorMps(spec));
  }

  LongitudinalRollout connectorLaunchRollout(double tau,
                                             double safe_speed_cap_mps,
                                             const ActionSpec& spec,
                                             double lane_change_start_sec) const {
    const double start = std::max(0.0, lane_change_start_sec);
    if (tau <= start + 1e-6) {
      return actionLongitudinalRollout(tau, safe_speed_cap_mps, spec);
    }

    // Once waypoint_system has switched to a valid endpoint connector, ACC
    // applies its dedicated connector cap.  Model the same 0 -> cap launch
    // instead of retaining the pre-switch lane-end target of 0 m/s.
    const LongitudinalRollout pre_switch =
        actionLongitudinalRollout(start, safe_speed_cap_mps, spec);
    const double elapsed = tau - start;
    const double accel = std::max(0.0, stopped_launch_connector_max_accel_mps2_);
    const double cap = std::max(0.0, stopped_launch_connector_speed_cap_mps_);
    const double v0 = std::max(0.0, pre_switch.v);

    LongitudinalRollout out;
    if (accel <= 1e-6 || v0 >= cap - 1e-6) {
      out.v = v0;
      out.ds = pre_switch.ds + v0 * elapsed;
      return out;
    }

    const double accel_time = std::min(elapsed, (cap - v0) / accel);
    out.v = std::min(cap, v0 + accel * accel_time);
    out.ds = pre_switch.ds + v0 * accel_time +
        0.5 * accel * accel_time * accel_time;
    if (elapsed > accel_time) out.ds += out.v * (elapsed - accel_time);
    return out;
  }

  double connectorTravelTimeSec(double initial_speed_mps,
                                double distance_m) const {
    const double distance = std::max(0.0, distance_m);
    if (distance <= 1e-6) return 0.0;
    const double v0 = std::max(0.0, initial_speed_mps);
    const double accel = std::max(0.0, stopped_launch_connector_max_accel_mps2_);
    const double cap = std::max(0.0, stopped_launch_connector_speed_cap_mps_);
    if (v0 >= cap - 1e-6) {
      return v0 > 1e-6 ? distance / v0 : std::numeric_limits<double>::infinity();
    }
    if (accel <= 1e-6) return std::numeric_limits<double>::infinity();
    const double t_to_cap = (cap - v0) / accel;
    const double d_to_cap = v0 * t_to_cap + 0.5 * accel * t_to_cap * t_to_cap;
    if (distance <= d_to_cap) {
      return (-v0 + std::sqrt(std::max(0.0, v0 * v0 + 2.0 * accel * distance))) /
          accel;
    }
    return t_to_cap + (distance - d_to_cap) / std::max(1e-6, cap);
  }

  double actionTargetSpeedMps(double safe_speed_cap_mps,
                              const ActionSpec& spec) const {
    if (spec.action == BehaviorAction::kKeep ||
        spec.action == BehaviorAction::kWaitForGap ||
        spec.action == BehaviorAction::kChangeNow) {
      return std::max(0.0, safe_speed_cap_mps);
    }
    return actionLongitudinalRollout(effectiveLaneChangeStartSec(spec),
                                     safe_speed_cap_mps, spec).v;
  }

  double vehicleHalfS(const smpc_lane_change::TargetVehicle& v) const {
    return v.half_s > 1e-3 ? v.half_s : 0.5 * std::max(0.0, v.length);
  }

  double frontEdgeDeltaS(const smpc_lane_change::TargetVehicle& v) const {
    return v.front_delta_s != 0.0 ? v.front_delta_s : v.delta_s + vehicleHalfS(v);
  }

  double rearEdgeDeltaS(const smpc_lane_change::TargetVehicle& v) const {
    return v.rear_delta_s != 0.0 ? v.rear_delta_s : v.delta_s - vehicleHalfS(v);
  }

  double frontBumperGap(const smpc_lane_change::TargetVehicle& front) const {
    return rearEdgeDeltaS(front) - 0.5 * ego_length_m_;
  }

  double rearBumperGap(const smpc_lane_change::TargetVehicle& rear) const {
    return -frontEdgeDeltaS(rear) - 0.5 * ego_length_m_;
  }

  double observedLongitudinalAccel(const smpc_lane_change::TargetVehicle& v) const {
    if (v.unique_id < 0) return 0.0;
    const auto it = vehicle_history_.find(v.unique_id);
    if (it == vehicle_history_.end() || !it->second.initialized) return 0.0;
    return it->second.filtered_accel;
  }

  static double pressure01(double value, double low, double high) {
    if (high <= low) return value >= high ? 1.0 : 0.0;
    return std::clamp((value - low) / (high - low), 0.0, 1.0);
  }

  double adaptiveEventProbability(const smpc_lane_change::TargetVehicle& v) const {
    const bool rear = v.role.find("rear") != std::string::npos;
    const double accel_obs = observedLongitudinalAccel(v);

    if (rear) {
      const double gap = std::max(0.0, rearBumperGap(v));
      const double closing = std::max(0.0, leadTrustedLongSpeed(v) - ego_speed_mps_);
      const double ttc = closing > 0.1 ? gap / closing : std::numeric_limits<double>::infinity();
      const double ttc_pressure = std::isfinite(ttc)
          ? pressure01(probability_ttc_safe_sec_ - ttc,
                       0.0,
                       probability_ttc_safe_sec_ - probability_ttc_critical_sec_)
          : 0.0;
      const double speed_pressure =
          pressure01(closing, 0.0, probability_rel_speed_scale_mps_);
      const double accel_pressure =
          pressure01(std::max(0.0, accel_obs), 0.0, probability_accel_scale_mps2_);
      const double near_pressure =
          pressure01(target_rear_safe_gap_m_ - gap, 0.0, target_rear_safe_gap_m_);

      const double p = rear_accel_probability_ +
          0.25 * ttc_pressure +
          0.18 * speed_pressure +
          0.12 * accel_pressure +
          0.10 * near_pressure;
      return std::clamp(p, min_rear_accel_probability_, max_rear_accel_probability_);
    }

    const double gap = std::max(0.0, frontBumperGap(v));
    const double desired_gap = std::max(
        current_front_safe_gap_m_, standstill_gap_m_ + time_headway_sec_ * ego_speed_mps_);
    const double closing = std::max(0.0, ego_speed_mps_ - leadTrustedLongSpeed(v));
    const double ttc = closing > 0.1 ? gap / closing : std::numeric_limits<double>::infinity();
    const double ttc_pressure = std::isfinite(ttc)
        ? pressure01(probability_ttc_safe_sec_ - ttc,
                     0.0,
                     probability_ttc_safe_sec_ - probability_ttc_critical_sec_)
        : 0.0;
    const double speed_pressure =
        pressure01(closing, 0.0, probability_rel_speed_scale_mps_);
    const double decel_pressure =
        pressure01(std::max(0.0, -accel_obs), 0.0, probability_accel_scale_mps2_);
    const double near_pressure =
        pressure01(desired_gap - gap, 0.0, desired_gap);

    const double p = front_brake_probability_ +
        0.25 * ttc_pressure +
        0.18 * speed_pressure +
        0.12 * decel_pressure +
        0.10 * near_pressure;
    return std::clamp(p, min_front_brake_probability_, max_front_brake_probability_);
  }

  double gapPenalty(double gap, double desired_gap) const {
    if (!std::isfinite(gap)) return 0.0;
    const double shortage = std::max(0.0, desired_gap - gap);
    return gap_cost_weight_ * shortage * shortage / std::max(1.0, desired_gap);
  }

  double laneEndPressureCost(const smpc_lane_change::TargetVehicleSet& t) const {
    if (!t.lane_change_required) return 0.0;
    if (t.emergency_stop_required) return lane_end_emergency_cost_;
    if (t.lane_change_urgent) return lane_end_urgent_cost_;
    if (t.lane_change_prepare) return lane_end_prepare_cost_;
    return 0.0;
  }

  double yieldZoneWindow(bool front) const {
    return yieldZoneWindowAt(front, ego_speed_mps_);
  }

  double yieldZoneWindowAt(bool front, double ego_speed_mps) const {
    const double base = front ? side_vehicle_yield_front_window_m_
                              : side_vehicle_yield_rear_window_m_;
    if (!side_vehicle_yield_dynamic_window_enabled_) return base;
    const double upper = front ? side_vehicle_yield_front_window_max_m_
                               : side_vehicle_yield_rear_window_max_m_;
    const double speed_span = side_vehicle_yield_ego_speed_narrow_mps_ -
                              side_vehicle_yield_ego_speed_wide_mps_;
    const double ratio = speed_span > 1.0e-3
        ? std::clamp((ego_speed_mps - side_vehicle_yield_ego_speed_wide_mps_) /
                         speed_span, 0.0, 1.0)
        : 1.0;
    const double window = base + ratio * (upper - base);
    // 앞 영역은 속도 비례 하한(시간 간격)을 둔다. 고정 4.2 m 는 15 m/s 에서 0.28 s 라
    //   17:15 백: 4.4 m/s 느린 앞차가 5.6 m 앞인데 '영역 비어 있음'으로 변경 -> 충돌.
    if (front) {
      return std::max(window, std::max(0.0, side_vehicle_yield_front_min_headway_sec_) *
                                  std::max(0.0, ego_speed_mps));
    }
    return window;
  }

  double activeTargetFrontSafeGap(const smpc_lane_change::TargetVehicleSet& t) const {
    if (!fixed_target_gap_gate_enabled_) return yieldZoneWindow(true);
    if (t.emergency_stop_required) return emergency_merge_front_safe_gap_m_;
    const double base = t.lane_change_urgent ? urgent_merge_front_safe_gap_m_
        : (t.lane_change_prepare ? prepare_merge_front_safe_gap_m_
                                 : target_front_safe_gap_m_);
    return interpolateDeadlineRequirement(
        base, deadline_relax_front_gap_floor_m_, deadlineRelaxation(t));
  }

  double activeTargetRearSafeGap(const smpc_lane_change::TargetVehicleSet& t) const {
    if (!fixed_target_gap_gate_enabled_) return yieldZoneWindow(false);
    if (t.emergency_stop_required) return emergency_merge_rear_safe_gap_m_;
    const double base = t.lane_change_urgent ? urgent_merge_rear_safe_gap_m_
        : (t.lane_change_prepare ? prepare_merge_rear_safe_gap_m_
                                 : target_rear_safe_gap_m_);
    return interpolateDeadlineRequirement(
        base, deadline_relax_rear_gap_floor_m_, deadlineRelaxation(t));
  }

  // 차선변경을 원하는데 확정을 못 받고 대기한 시간을 0~1 완화값으로 바꾼다.
  // grace 까지는 0, 그 뒤 ramp 동안 1 까지 선형 증가.
  void updateWaitRelaxation(bool waiting) {
    if (!wait_relax_enabled_ || !waiting) {
      wait_relax_since_ = ros::Time(0.0);
      wait_relax_value_ = 0.0;
      return;
    }
    const ros::Time now = ros::Time::now();
    if (wait_relax_since_.isZero()) {
      wait_relax_since_ = now;
      wait_relax_value_ = 0.0;
      return;
    }
    const double waited = (now - wait_relax_since_).toSec();
    const double ramp = std::max(1.0e-3, wait_relax_ramp_sec_);
    wait_relax_value_ =
        std::clamp((waited - std::max(0.0, wait_relax_grace_sec_)) / ramp,
                   0.0, 1.0);
  }

  double deadlineRelaxation(const smpc_lane_change::TargetVehicleSet& t) const {
    double distance_relax = 0.0;
    if (deadline_relax_enabled_ && t.lane_change_required &&
        std::isfinite(t.distance_to_lane_end)) {
      const double start = std::max(deadline_relax_start_distance_m_,
                                    deadline_relax_full_distance_m_ + 1.0e-3);
      const double full = std::max(0.0, deadline_relax_full_distance_m_);
      distance_relax = std::clamp((start - t.distance_to_lane_end) /
                                  std::max(start - full, 1.0e-3), 0.0, 1.0);
    }
    // 거리 완화와 대기시간 완화 중 큰 쪽을 쓴다. 차선 끝이 아직 멀어
    // 거리 완화가 0 인 구간에서도 오래 못 끼면 요구치가 내려간다.
    // 완화는 gap/TTC/headway 요구치를 floor 값까지만 낮추고
    // (interpolateDeadlineRequirement), 궤적 충돌 검사 자체는 건드리지
    // 않으므로 실제로 겹치는 변경은 여전히 거부된다.
    if (!t.lane_change_required) return distance_relax;
    return std::max(distance_relax, wait_relax_value_);
  }

  static double interpolateDeadlineRequirement(double normal, double floor,
                                                double relaxation) {
    const double bounded_floor = std::min(normal, std::max(0.0, floor));
    return normal + std::clamp(relaxation, 0.0, 1.0) *
                        (bounded_floor - normal);
  }

  double activeTargetFrontMinTtc(const smpc_lane_change::TargetVehicleSet& t,
                                 bool clear_zone = false) const {
    const double normal = interpolateDeadlineRequirement(
        target_front_min_ttc_sec_, deadline_relax_front_ttc_floor_sec_,
        deadlineRelaxation(t));
    return clear_zone && clear_zone_ttc_relax_enabled_
        ? std::min(normal, std::max(0.0, clear_zone_front_min_ttc_sec_))
        : normal;
  }

  double activeTargetRearMinTtc(const smpc_lane_change::TargetVehicleSet& t,
                                bool clear_zone = false) const {
    const double normal = interpolateDeadlineRequirement(
        target_rear_min_ttc_sec_, deadline_relax_rear_ttc_floor_sec_,
        deadlineRelaxation(t));
    return clear_zone && clear_zone_ttc_relax_enabled_
        ? std::min(normal, std::max(0.0, clear_zone_rear_min_ttc_sec_))
        : normal;
  }

  double activeTargetRearMinHeadway(const smpc_lane_change::TargetVehicleSet& t,
                                    bool clear_zone = false) const {
    const double normal = interpolateDeadlineRequirement(
        target_rear_min_headway_sec_, deadline_relax_rear_headway_floor_sec_,
        deadlineRelaxation(t));
    return clear_zone && clear_zone_ttc_relax_enabled_
        ? std::min(normal, std::max(0.0, clear_zone_rear_min_headway_sec_))
        : normal;
  }

  double activeRiskEpsilon(const smpc_lane_change::TargetVehicleSet& t) const {
    if (t.emergency_stop_required) {
      return std::max(urgent_risk_epsilon_, emergency_merge_risk_epsilon_);
    }
    if (t.lane_change_urgent) return urgent_risk_epsilon_;
    return risk_epsilon_;
  }

  static bool roleContains(const smpc_lane_change::TargetVehicle& v,
                           const std::string& token) {
    return v.role.find(token) != std::string::npos;
  }

  static bool laneEndPressureActive(const smpc_lane_change::TargetVehicleSet& t) {
    return t.lane_change_prepare || t.lane_change_urgent || t.emergency_stop_required;
  }

  bool hasAdjacentSideVehicleInTargetLane(
      const smpc_lane_change::TargetVehicleSet& t,
      double front_window_m,
      double rear_window_m,
      double* side_veh_speed = nullptr,
      double* side_veh_delta_s = nullptr,
      bool* rear_vehicle_in_zone = nullptr,
      double* side_veh_bumper_gap = nullptr,
      int* side_veh_id = nullptr) const {
    double min_abs_ds = std::numeric_limits<double>::infinity();
    bool found = false;
    bool found_rear = false;
    double found_speed = 0.0;
    double found_ds = 0.0;
    double found_bumper_gap = 0.0;
    int found_id = -1;

    auto check_veh = [&](const smpc_lane_change::TargetVehicle& v) {
      if (!v.valid || v.lane_id != t.target_lane_id) return;
      // 전방: frontBumperGap이 front_window_m 이내 (안전 간격 미확보)
      // 후방: rearBumperGap이 rear_window_m 이내 (안전 간격 미확보)
      // 자차와 종방향으로 겹치는 경우(bumper gap < 0)도 당연히 안전 간격 미확보 구역에 포함
      bool in_window = false;
      if (v.delta_s >= 0.0) {
        in_window = (frontBumperGap(v) <= front_window_m);
      } else {
        in_window = (rearBumperGap(v) <= rear_window_m);
      }
      if (in_window) {
        if (std::abs(v.delta_s) < min_abs_ds) {
          min_abs_ds = std::abs(v.delta_s);
          found = true;
          found_rear = v.delta_s < -2.0;
          // 속도 추정 신뢰도를 반영한다.
          //
          // 실측(2026-09-16 15:18 백, 중앙차분 기준 n=523): velocity_std 가
          //   0.0~1.0 -> RMS 1.05 m/s
          //   1.0~2.0 -> RMS 1.45
          //   2.0~    -> RMS 1.8
          // 로 1.0 을 넘으면 급격히 나빠진다. 아래 양보 목표속도 계산은
          // side_veh_speed 를 기준점으로 쓰므로(commanded = side_veh_speed
          // + 0.5*gap_err), 추정이 틀리면 목표속도가 통째로 틀어진다.
          // 신뢰할 수 없으면 '자차와 같은 속도'로 두어, 상대속도 기반
          // 추가 감속을 요구하지 않고 거리 기반 판단에 맡긴다.
          found_speed = leadTrustedLongSpeed(v);
          found_ds = v.delta_s;
          found_bumper_gap = v.delta_s >= 0.0
              ? frontBumperGap(v) : rearBumperGap(v);
          found_id = v.unique_id;
        }
      }
    };

    if (t.target_front.valid) check_veh(t.target_front);
    if (t.target_rear.valid) check_veh(t.target_rear);
    for (const auto& nb : t.nearby_vehicles) {
      check_veh(nb);
    }

    if (found) {
      if (side_veh_speed) *side_veh_speed = found_speed;
      if (side_veh_delta_s) *side_veh_delta_s = found_ds;
      if (side_veh_bumper_gap) *side_veh_bumper_gap = found_bumper_gap;
      if (side_veh_id) *side_veh_id = found_id;
    }
    if (rear_vehicle_in_zone) *rear_vehicle_in_zone = found_rear;
    return found;
  }

  bool isRearLikeVehicle(const smpc_lane_change::TargetVehicle& v) const {
    return roleContains(v, "rear");
  }

  bool targetVelocityTrusted(
      const smpc_lane_change::TargetVehicle& v) const {
    if(gap_shared_speed_enabled_) return gap_speed_policy_.get(v,ros::Time::now().toSec()).known;
    if (!v.velocity_valid || !std::isfinite(v.v_long) ||
        !std::isfinite(v.velocity_std) || v.observation_age > 0.3) {
      return false;
    }
    return !trusted_speed_gate_enabled_ || trusted_speed_max_std_mps_ <= 0.0 ||
        v.velocity_std <= trusted_speed_max_std_mps_;
  }

  bool lowSpeedRearSafetyActive() const {
    return low_speed_rear_safety_enabled_ &&
        ego_speed_mps_ <= std::max(0.0, low_speed_rear_ego_speed_mps_);
  }

  // 속도를 신뢰할 수 없어도 가까운 차량(범퍼 간격 untrusted_near_gap_m 이내)은
  // 원시 속도로 TTC/예측 간격을 검사한다. 17:15 백: std 1.21(>1.0) 이라 '영역이 비면
  // 통과' 규칙으로 TTC 0.8 s 짜리 앞차를 통과시켰다.
  bool untrustedNearVehicle(const smpc_lane_change::TargetVehicle& v, bool front) const {
    if (!v.valid || targetVelocityTrusted(v) || !std::isfinite(v.v_long)) return false;
    const double gap = front ? frontBumperGap(v) : rearBumperGap(v);
    return std::isfinite(gap) && gap <= std::max(0.0, untrusted_near_gap_m_);
  }

  // 속도 오차 여유의 상한. 불신 트랙 std 가 45 같은 값이면 여유가 폭주해 항상 막힌다.
  // 신뢰 트랙은 std <= 1.0 이라 영향 없음.
  double cappedStdMargin(const smpc_lane_change::TargetVehicle& v) const {
    return std::min(velocityStdMargin(v), std::max(0.0, untrusted_std_margin_cap_mps_));
  }

  bool targetFrontTtcSafe(const smpc_lane_change::TargetVehicleSet& t,
                          const smpc_lane_change::TargetVehicle& front,
                          bool clear_zone = false) const {
    if (!front.valid) return true;
    if (!targetVelocityTrusted(front) && !untrustedNearVehicle(front, true)) {
      // The ego-speed-scaled occupancy box is the primary gate.  Outside that
      // box an unreliable lidar derivative must not veto an otherwise clear
      // merge; geometry/risk checks still run on the executable trajectory.
      return clear_zone;
    }
    const double closing = std::max(0.0, ego_speed_mps_ - (gap_shared_speed_enabled_ ? gapVehicleSpeed(front) : front.v_long) + cappedStdMargin(front));
    if (closing <= 0.1) return true;
    const double gap = std::max(0.0, frontBumperGap(front));
    return gap / closing >= std::max(0.0, activeTargetFrontMinTtc(t, clear_zone));
  }

  bool targetRearTtcSafe(const smpc_lane_change::TargetVehicleSet& t,
                         const smpc_lane_change::TargetVehicle& rear,
                         bool clear_zone = false) const {
    if (!rear.valid) return true;
    if (!targetVelocityTrusted(rear) && !untrustedNearVehicle(rear, false)) {
      return clear_zone;
    }
    const double closing = std::max(0.0, (gap_shared_speed_enabled_ ? gapVehicleSpeed(rear) : rear.v_long) - ego_speed_mps_ + cappedStdMargin(rear));
    if (closing <= 0.1) return true;
    // Once the enlarged low-speed occupancy box is clear, TTC is deliberately
    // only a loose secondary check.  The CHANGE_NOW rollout below models the
    // acceleration that starts as the preparation cap is released.
    const double required_ttc = lowSpeedRearSafetyActive() && !clear_zone
        ? std::max(activeTargetRearMinTtc(t, clear_zone), low_speed_rear_min_ttc_sec_)
        : activeTargetRearMinTtc(t, clear_zone);
    const double gap = std::max(0.0, rearBumperGap(rear));
    return gap / closing >= std::max(0.0, required_ttc);
  }

  bool targetRearHeadwaySafe(const smpc_lane_change::TargetVehicleSet& t,
                             const smpc_lane_change::TargetVehicle& rear,
                             bool clear_zone = false) const {
    if (!rear.valid) return true;
    if (!targetVelocityTrusted(rear) && clear_zone) {
      return true;
    }
    const double gap = std::max(0.0, rearBumperGap(rear));
    const double rear_speed = gap_shared_speed_enabled_ ? gapVehicleSpeed(rear) : std::max(0.0, rear.v_long);
    const double reference_speed = std::max(std::max(0.0, ego_speed_mps_), rear_speed);
    const double required_gap =
        std::max(0.0, activeTargetRearMinHeadway(t, clear_zone)) * reference_speed;
    return gap >= required_gap;
  }

  bool mergePriorityRearEligible(const smpc_lane_change::TargetVehicleSet& t,
                                 const smpc_lane_change::TargetVehicle& v) const {
    if (!merge_priority_enabled_ || !laneEndPressureActive(t) || !v.valid) return false;
    if (!isRearLikeVehicle(v)) return false;
    // A stopped/slow ego cannot safely claim priority over a fast rear vehicle:
    // it needs time to launch and finish the lateral blend.
    if (lowSpeedRearSafetyActive()) return false;

    const double gap = rearBumperGap(v);
    if (!std::isfinite(gap) || gap < merge_priority_rear_min_gap_m_) return false;

    const double closing = std::max(0.0, leadTrustedLongSpeed(v) - ego_speed_mps_);
    const double ttc = closing > 0.1
        ? gap / closing
        : std::numeric_limits<double>::infinity();
    if (closing > merge_priority_rear_max_closing_mps_ &&
        ttc < merge_priority_rear_min_ttc_sec_) {
      return false;
    }

    return true;
  }

  double mergePriorityRiskScale(const smpc_lane_change::TargetVehicleSet* t,
                                const smpc_lane_change::TargetVehicle& v,
                                bool change_left) const {
    if (!t) return 1.0;

    double scale = 1.0;
    if (egoLanePriorityEligible(*t, v, change_left)) {
      scale = std::min(
          scale, std::clamp(ego_lane_priority_side_risk_scale_, 0.0, 1.0));
    }

    if (change_left && mergePriorityRearEligible(*t, v)) {
      if (roleContains(v, "nearby")) {
        scale = std::min(
            scale, std::clamp(merge_priority_nearby_rear_risk_scale_, 0.0, 1.0));
      } else {
        scale = std::min(
            scale, std::clamp(merge_priority_rear_risk_scale_, 0.0, 1.0));
      }
    }
    return scale;
  }

  double effectiveTargetRearSafeGap(const smpc_lane_change::TargetVehicleSet& t,
                                    const smpc_lane_change::TargetVehicle& rear,
                                    double base_gap) const {
    double required_gap = base_gap;
    if (lowSpeedRearSafetyActive()) {
      required_gap = std::max(required_gap, std::max(0.0, low_speed_rear_safe_gap_m_));
    }
    if (!mergePriorityRearEligible(t, rear)) return required_gap;
    return std::min(required_gap, std::max(0.0, merge_priority_rear_safe_gap_m_));
  }

  bool egoLanePriorityEligible(const smpc_lane_change::TargetVehicleSet& t,
                               const smpc_lane_change::TargetVehicle& v,
                               bool change_left) const {
    if (!ego_lane_priority_enabled_ || !v.valid) return false;

    const bool current_front = roleContains(v, "current_front");
    const bool nearby = roleContains(v, "nearby");
    if (!current_front && !nearby) return false;

    // If the vehicle is already very close in front of ego, it is no longer a
    // right-of-way negotiation problem; it is a hard safety constraint.
    const double front_gap = frontBumperGap(v);
    if (std::isfinite(front_gap) &&
        front_gap <= ego_lane_priority_front_hard_gap_m_) {
      return false;
    }

    // A vehicle selected as current_front only because its box touches the
    // current-lane boundary should not make ego yield until its center is
    // actually established near the lane center.
    if (current_front) {
      return std::abs(v.d) > ego_lane_priority_current_max_abs_d_m_;
    }

    // For nearby vehicles, preserve full weight for ego's current lane and for
    // the target lane while ego is actively evaluating a lane change.  Other
    // lanes are treated as yielding side traffic during lane-end pressure.
    if (v.lane_id == t.current_lane_id &&
        std::abs(v.d) <= ego_lane_priority_current_max_abs_d_m_) {
      return false;
    }
    if (v.lane_id == t.current_lane_id &&
        std::abs(v.d) > ego_lane_priority_current_max_abs_d_m_) {
      return true;
    }
    if (change_left && v.lane_id == t.target_lane_id) return false;

    return laneEndPressureActive(t);
  }

  std::vector<smpc_lane_change::TargetVehicle> uniqueVehicles(
      const smpc_lane_change::TargetVehicleSet& t) const {
    std::vector<smpc_lane_change::TargetVehicle> out;
    auto add_unique = [&out](const smpc_lane_change::TargetVehicle& v) {
      if (!v.valid) return;
      bool duplicate = false;
      for (const auto& prev : out) {
        if (v.unique_id >= 0 && prev.unique_id == v.unique_id) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) out.push_back(v);
    };

    add_unique(t.current_front);
    add_unique(t.target_front);
    add_unique(t.target_rear);
    if (risk_include_nearby_vehicles_) {
      for (const auto& v : t.nearby_vehicles) add_unique(v);
    }
    return out;
  }

  double laneYawAtVehicle(const smpc_lane_change::TargetVehicle& v) const {
    const auto* lane = laneFor(v.lane_id);
    if (!lane) return v.yaw;
    const auto sample = lane->sample(v.s, v.d);
    return sample.valid ? sample.yaw : v.yaw;
  }

  bool lateralModeAllowedForVehicle(const smpc_lane_change::TargetVehicle& v) const {
    if (!lateral_mode_enabled_) return false;
    const bool nearby = v.role.find("nearby") != std::string::npos;
    const bool rear = v.role.find("rear") != std::string::npos;
    if (nearby && !lateral_mode_allow_nearby_) return false;
    if (rear && !lateral_mode_allow_rear_) return false;
    if (std::abs(v.d) < lateral_mode_min_abs_d_m_) return false;
    return true;
  }

  std::vector<TvMode> modesForVehicle(const smpc_lane_change::TargetVehicle& v) const {
    const bool rear = v.role.find("rear") != std::string::npos;
    // NPC 종방향 이벤트 모드(앞차 급제동 / 뒤차 급가속) 사용 여부.
    //
    // 대회 NPC 는 급제동·급가속을 하지 않는다. 그런데 이 모드가 있으면
    // 등속으로 잘 가고 있는 차량도 '3.5초 내내 -1.2 m/s^2 로 감속한다'는
    // 시나리오로 함께 평가되어, 실제로는 벌어지는 간격이 좁아지는 것으로
    // 예측된다.
    //   실측(2026-09-15 20:38 백): 옆 차로 점유영역이 전 구간 비었고
    //   (zone_ttc_relaxed=1) risk/clearance/TTC 가 모두 통과인데
    //   projected_gap_ttc 만 0 이라 28초 내내 변경을 못 했다.
    //   target_front(id13)는 등속 10.4 m/s 로 간격이 14.2 -> 20.1 m 로
    //   벌어지는 중이었는데도 급제동 가정 때문에 +2.6초에 7.02 m 로
    //   예측돼 실패했다.
    // 끄면 nominal(등속) 한 가지만 남아 확률 1.0 이 된다. 횡방향 모드
    // (차선변경 관측)는 아래에서 그대로 유지된다 - 그건 최악 가정이 아니라
    // 실제 관측된 거동이다.
    const double event_probability =
        npc_longitudinal_event_modes_enabled_ ? adaptiveEventProbability(v) : 0.0;
    const double event_accel = rear ? rear_accel_mps2_ : front_brake_accel_mps2_;

    std::vector<TvMode> modes;
    modes.push_back({"nominal", 1.0 - event_probability, nominal_target_accel_mps2_, 0.0});
    if (npc_longitudinal_event_modes_enabled_) {
      modes.push_back({rear ? "rear_accel" : "front_brake", event_probability, event_accel, 0.0});
    }

    const double lane_yaw = laneYawAtVehicle(v);
    const double yaw_error = smpc_lane_change::wrapToPi(v.yaw - lane_yaw);
    double inferred_lat_rate = v.v_lat;
    if (lateral_mode_allow_yaw_only_ &&
        std::abs(inferred_lat_rate) < lateral_mode_v_threshold_mps_ &&
        std::abs(yaw_error) > yaw_lateral_threshold_rad_) {
      inferred_lat_rate = v.v_long * std::sin(yaw_error);
    }
    const double yaw_implied_lat_rate = v.v_long * std::sin(yaw_error);

    bool lateral_observed =
        lateralModeAllowedForVehicle(v) &&
        std::abs(inferred_lat_rate) >= lateral_mode_v_threshold_mps_ &&
        std::abs(yaw_error) >= yaw_lateral_threshold_rad_;
    if (lateral_observed && lateral_mode_require_yaw_agreement_ &&
        inferred_lat_rate * yaw_implied_lat_rate <= 0.0) {
      lateral_observed = false;
    }
    if (lateral_observed && lateral_mode_require_away_from_center_ &&
        v.d * inferred_lat_rate <= 0.0) {
      lateral_observed = false;
    }
    if (lateral_observed) {
      const double over_v = std::max(0.0, std::abs(inferred_lat_rate) - lateral_mode_v_threshold_mps_);
      const double over_yaw = std::max(0.0, std::abs(yaw_error) - yaw_lateral_threshold_rad_);
      const double max_p_lat_by_straight_floor =
          std::max(0.0, 1.0 - std::clamp(tv_no_lateral_probability_floor_, 0.0, 1.0));
      const double max_allowed_lateral_probability =
          std::min(max_lateral_mode_probability_, max_p_lat_by_straight_floor);
      const double p_lat = clampProbability(std::min(
          max_allowed_lateral_probability,
          lateral_mode_probability_ + 0.08 * over_v + yaw_lateral_probability_gain_ * over_yaw));
      if (p_lat > 1e-6) {
        for (auto& mode : modes) mode.probability *= (1.0 - p_lat);
        modes.push_back({"observed_lateral", p_lat, nominal_target_accel_mps2_,
                         std::clamp(inferred_lat_rate,
                                    -max_lateral_prediction_mps_, max_lateral_prediction_mps_)});
      }
    }

    const double total = std::max(
        1e-6, std::accumulate(modes.begin(), modes.end(), 0.0,
                              [](double acc, const TvMode& mode) {
                                return acc + std::max(0.0, mode.probability);
                              }));
    for (auto& mode : modes) mode.probability = clampProbability(mode.probability / total);
    return modes;
  }

  std::vector<PredPose> buildEgoTrajectoryFromReferencePreview(
      const smpc_lane_change::TargetVehicleSet& t,
      const ActionSpec& spec) const {
    std::vector<PredPose> traj;
    if (!referencePreviewFreshFor(t)) return traj;

    const ReferencePreview preview = reference_preview_;
    const auto* current_lane = laneFor(t.current_lane_id);
    const auto* target_lane = laneFor(t.target_lane_id);
    if (!current_lane || !target_lane || !spec.change_left) return traj;

    const double source_origin_arc =
        projectPolylineArc(preview.source, ego_x_, ego_y_);
    const double target_origin_arc =
        projectPolylineArc(preview.target, ego_x_, ego_y_);
    const double connector_origin_arc = preview.connector.valid()
        ? projectPolylineArc(preview.connector, ego_x_, ego_y_)
        : std::numeric_limits<double>::quiet_NaN();
    if (!std::isfinite(source_origin_arc) ||
        (preview.mode != waypoint_system::PathSwitchPreview::CONNECTOR &&
         !std::isfinite(target_origin_arc)) ||
        (preview.mode == waypoint_system::PathSwitchPreview::CONNECTOR &&
         !std::isfinite(connector_origin_arc))) {
      return traj;
    }

    const double target_speed_mps =
        egoChangePredictionTargetSpeedMps(t, spec);
    const double lane_change_start_sec = effectiveLaneChangeStartSec(spec);
    const LongitudinalRollout pre_switch_rollout = actionLongitudinalRollout(
        lane_change_start_sec, target_speed_mps, spec);
    ReferencePolyline spatial_plan;
    if (preview.mode == waypoint_system::PathSwitchPreview::BLEND &&
        preview.spatial_blend_length_m > 0.0) {
      auto source = preview.source.poses;
      const double start_arc=source_origin_arc+pre_switch_rollout.ds;
      auto it=std::lower_bound(preview.source.arc_m.begin(), preview.source.arc_m.end(), start_arc);
      if (it==preview.source.arc_m.end()) return {};
      source.erase(source.begin(), source.begin()+std::distance(preview.source.arc_m.begin(),it));
      const ReferencePolyline delayed_source = makeReferencePolyline(source);
      if (!delayed_source.valid()) return {};
      // The preview length was computed at the current pose.  CHANGE_NOW is
      // executed after behavior_execution_delay_sec, when waypoint_system
      // will publish a shorter 95%-of-remaining blend.  Recompute that exact
      // contract here; otherwise the shifted source can be a few centimetres
      // shorter than the old blend and spatialBlend returns an empty path.
      const double effective_blend_length = std::min(
          preview.spatial_blend_length_m,
          0.95 * delayed_source.arc_m.back());
      if (effective_blend_length <= 1.0) return {};
      spatial_plan=makeReferencePolyline(waypoint_system::spatialBlend(
          source,preview.target.poses,effective_blend_length));
      if (!spatial_plan.valid()) return {};
      const double finish_tau=std::max(1,horizon_steps_)*prediction_dt_sec_-
          std::max(0.0,behavior_post_merge_buffer_sec_);
      const auto finish=actionLongitudinalRollout(finish_tau,target_speed_mps,spec);
      if (finish.ds-pre_switch_rollout.ds < effective_blend_length+2.0) return {};
    }
    if (preview.mode == waypoint_system::PathSwitchPreview::CONNECTOR &&
        pre_switch_rollout.ds >
            std::max(0.0, stopped_launch_connector_max_preswitch_distance_m_)) {
      // A connector preview is anchored at the current stopped ego pose.  Do
      // not certify a delayed candidate after it has already travelled away
      // from that anchor; the next 20 Hz preview will provide the new pose.
      return traj;
    }
    if (preview.mode == waypoint_system::PathSwitchPreview::CONNECTOR) {
      const double join_distance = std::max(
          0.0, preview.connector_join_arc_m - connector_origin_arc);
      const double completion_time = connectorTravelTimeSec(
          pre_switch_rollout.v, join_distance);
      const double horizon_sec = std::max(1, horizon_steps_) * prediction_dt_sec_;
      if (!std::isfinite(completion_time) ||
          lane_change_start_sec + completion_time + behavior_post_merge_buffer_sec_ >
              horizon_sec + 1e-6) {
        // A short target tail is not permission to clamp at its endpoint.
        // Require enough path/time to observe the completed connector and its
        // post-merge buffer before calling the stopped launch safe.
        return traj;
      }
    }

    double motion_started_at_sec = std::numeric_limits<double>::infinity();
    traj.reserve(std::max(1, horizon_steps_));
    for (int k = 1; k <= std::max(1, horizon_steps_); ++k) {
      const double tau = prediction_dt_sec_ * static_cast<double>(k);
      const bool after_switch = tau >= lane_change_start_sec - 1e-6;
      const LongitudinalRollout rollout =
          preview.mode == waypoint_system::PathSwitchPreview::CONNECTOR
          ? connectorLaunchRollout(tau, target_speed_mps, spec,
                                   lane_change_start_sec)
          : actionLongitudinalRollout(tau, target_speed_mps, spec);

      PredPose p;
      if (!after_switch) {
        p = samplePolyline(preview.source, source_origin_arc + rollout.ds);
      } else if (preview.mode == waypoint_system::PathSwitchPreview::CONNECTOR) {
        const double after_switch_distance = std::max(
            0.0, rollout.ds - pre_switch_rollout.ds);
        p = samplePolyline(preview.connector,
                           connector_origin_arc + after_switch_distance);
      } else if (preview.mode == waypoint_system::PathSwitchPreview::DIRECT) {
        p = samplePolyline(preview.target, target_origin_arc + rollout.ds);
      } else if (preview.mode == waypoint_system::PathSwitchPreview::BLEND &&
                 preview.spatial_blend_length_m > 0.0) {
        p = samplePolyline(spatial_plan,std::max(0.0,rollout.ds-pre_switch_rollout.ds));
      } else if (preview.mode == waypoint_system::PathSwitchPreview::BLEND) {
        const double after_switch_distance = std::max(
            0.0, rollout.ds - pre_switch_rollout.ds);
        if (!std::isfinite(motion_started_at_sec) &&
            (rollout.v >= ego_reference_preview_motion_start_speed_mps_ ||
             after_switch_distance >=
                 ego_reference_preview_motion_start_distance_m_)) {
          motion_started_at_sec = tau;
        }
        const double alpha = std::isfinite(motion_started_at_sec)
            ? std::clamp((tau - motion_started_at_sec) /
                             std::max(0.1, preview.blend_duration_sec),
                         0.0, 1.0)
            : 0.0;
        const ReferencePolyline blended = blendReferencePreview(preview, alpha);
        p = samplePolyline(blended, source_origin_arc + rollout.ds);
      } else {
        return {};
      }

      if (!p.valid) return {};
      p.t = tau;
      p.v = rollout.v;
      const auto projection = (after_switch ? target_lane : current_lane)->project(p.x, p.y);
      if (projection.valid) {
        p.s = projection.s;
        p.d = projection.d;
      }
      traj.push_back(p);
    }

    if (ego_reference_preview_require_target_completion_) {
      const PredPose& terminal = traj.back();
      const auto terminal_target_projection =
          target_lane->project(terminal.x, terminal.y);
      if (!terminal_target_projection.valid ||
          std::abs(terminal_target_projection.d) >
              std::max(0.0,
                       ego_reference_preview_target_completion_max_abs_d_m_)) {
        // In particular, a stopped BLEND stays on the source path because its
        // motion clock never starts. Returning an empty trajectory makes the
        // CHANGE_LEFT candidate infeasible rather than certifying that state.
        return {};
      }
    }
    return traj;
  }

  std::vector<PredPose> buildEgoTrajectory(const smpc_lane_change::TargetVehicleSet& t,
                                           const ActionSpec& spec) const {
    std::vector<PredPose> traj;
    // target == current 는 target_selector 가 '차선변경 불필요'를 표현하는 값이다
    // (최종 차선 도달 시 target_lane_id = current_lane_id). 실제로 바꿀 차선이
    // 없으므로 프리뷰를 요구하는 것 자체가 무의미하고, 그대로 두면 고속도로
    // 종료 후 매 사이클 WARN 만 쌓인다(실측 2026-09-07: 3->3/1->1/2->2 로 89회).
    // 아래쪽에도 같은 가드가 있지만 프리뷰 검사 뒤라 먼저 걸리지 않았다.
    if (spec.change_left && t.target_lane_id == t.current_lane_id) return traj;
    if (spec.change_left && ego_reference_preview_enabled_) {
      if (referencePreviewFreshFor(t)) {
        const auto preview_traj = buildEgoTrajectoryFromReferencePreview(t, spec);
        if (!preview_traj.empty()) {
          return preview_traj;
        }
      }
      if (ego_reference_preview_required_for_change_ && !ego_reference_preview_allow_fallback_) {
        ROS_WARN_THROTTLE(
            1.0,
            "[smpc_decision] refusing CHANGE_LEFT without a fresh matching "
            "path-switch preview (expected %d -> %d)",
            t.current_lane_id, t.target_lane_id);
        return traj;
      }
      if (ego_reference_preview_required_for_change_) {
        ROS_WARN_THROTTLE(
            2.0,
            "[smpc_decision] path-switch preview unavailable/invalid for %d -> %d; "
            "falling back to internal centerline rollout",
            t.current_lane_id, t.target_lane_id);
      }
    }
    const auto* current_lane = laneFor(t.current_lane_id);
    const auto* target_lane = laneFor(t.target_lane_id);
    if (!current_lane || !target_lane) return traj;

    const auto ego_current = current_lane->project(ego_x_, ego_y_);
    const auto ego_target = target_lane->project(ego_x_, ego_y_);
    if (!ego_current.valid || !ego_target.valid) return traj;
    if (spec.change_left && t.target_lane_id == t.current_lane_id) return traj;

    const double ego_rollout_target_speed_mps =
        egoChangePredictionTargetSpeedMps(t, spec);
    const double lane_change_start_sec = effectiveLaneChangeStartSec(spec);
    const double lane_change_duration_sec = spec.lane_change_duration_sec > 1e-3
        ? spec.lane_change_duration_sec
        : egoChangeBlendDuration();
    traj.reserve(std::max(1, horizon_steps_));
    for (int k = 1; k <= std::max(1, horizon_steps_); ++k) {
      const double tau = prediction_dt_sec_ * static_cast<double>(k);
      const LongitudinalRollout rollout =
          actionLongitudinalRollout(tau, ego_rollout_target_speed_mps, spec);
      const double predicted_v = rollout.v;
      const double ds = rollout.ds;

      PredPose p;
      p.t = tau;
      p.v = predicted_v;

      if (!spec.change_left) {
        const double d_decay = std::exp(-tau / std::max(0.1, lane_center_decay_tau_sec_));
        const double sample_s = ego_current.s + ds;
        // LanePath::sample intentionally clamps to the CSV endpoint.  A
        // clamped stationary pose with nonzero velocity is not a valid future
        // state, particularly for a deferred merge near the lane end.
        if (sample_s > current_lane->length() + 1e-6) return {};
        const auto sample = current_lane->sample(sample_s, ego_current.d * d_decay);
        if (!sample.valid) return {};
        p.valid = true;
        p.x = sample.x;
        p.y = sample.y;
        p.s = sample.s;
        p.d = sample.d;
        p.yaw = sample.yaw;
      } else {
        const double alpha = egoChangeAlpha(tau, lane_change_start_sec,
                                            lane_change_duration_sec);
        const double d_decay = std::exp(-tau / std::max(0.1, lane_center_decay_tau_sec_));
        // Before t_LC the candidate must remain on the current physical lane,
        // rather than jumping to a centered source-lane sample.
        smpc_lane_change::LaneSample cur;
        smpc_lane_change::LaneSample target;
        if (alpha < 1.0 - 1e-6) {
          const double cur_s = ego_current.s + ds;
          if (cur_s > current_lane->length() + 1e-6) return {};
          const double cur_d = ego_current.d * (1.0 - alpha) * d_decay;
          cur = current_lane->sample(cur_s, cur_d);
          if (!cur.valid) return {};
        }
        if (alpha > 1e-6) {
          const double target_s = ego_target.s + ds;
          if (target_s > target_lane->length() + 1e-6) return {};
          target = target_lane->sample(target_s, 0.0);
          if (!target.valid) return {};
        }
        p.valid = true;
        if (alpha <= 1e-6) {
          p.x = cur.x;
          p.y = cur.y;
          p.s = cur.s;
          p.d = cur.d;
          p.yaw = cur.yaw;
        } else if (alpha >= 1.0 - 1e-6) {
          p.x = target.x;
          p.y = target.y;
          p.s = target.s;
          p.d = target.d;
          p.yaw = target.yaw;
        } else {
          p.x = (1.0 - alpha) * cur.x + alpha * target.x;
          p.y = (1.0 - alpha) * cur.y + alpha * target.y;
          p.s = (1.0 - alpha) * cur.s + alpha * target.s;
          p.d = (1.0 - alpha) * cur.d + alpha * target.d;
          p.yaw = angleLerp(cur.yaw, target.yaw, alpha);
        }
      }
      traj.push_back(p);
    }
    return traj;
  }

  PredPose predictTargetPose(const smpc_lane_change::TargetVehicle& v,
                             const TvMode& mode,
                             double tau) const {
    PredPose p;
    const auto* lane = laneFor(v.lane_id);
    if (!lane) return p;

    // A noisy lidar velocity must not dominate the trajectory gate after the
    // distance occupancy box has already passed.  Use the same reliability
    // policy as TTC: an untrusted derivative falls back to ego speed, leaving
    // the measured geometric gap as the primary evidence.
    const double initial_v_long = targetVelocityTrusted(v)
        ? (gap_shared_speed_enabled_ ? gapVehicleSpeed(v) : std::max(0.0, v.v_long))
        : std::max(0.0, ego_speed_mps_);
    const double predicted_v_long =
        std::max(0.0, initial_v_long + mode.accel_mps2 * tau);
    const double ds = constantAccelDistance(initial_v_long, mode.accel_mps2, tau);
    double d = v.d;
    if (std::abs(mode.lateral_rate_mps) > 1e-3) {
      d += mode.lateral_rate_mps * tau;
    } else {
      d *= std::exp(-tau / std::max(0.1, lane_center_decay_tau_sec_));
    }
    d = std::clamp(d, -1.25 * lane_width_m_, 1.25 * lane_width_m_);

    const double sample_s = v.s + ds;
    if (sample_s > lane->length() + 1e-6) return p;
    const auto sample = lane->sample(sample_s, d);
    if (!sample.valid) return p;

    const double yaw_from_frenet_vel =
        smpc_lane_change::wrapToPi(sample.yaw +
                                  std::atan2(mode.lateral_rate_mps,
                                             std::max(0.1, predicted_v_long)));
    const double yaw_rate_limited = std::clamp(
        v.yaw_rate, -max_abs_tv_yaw_rate_, max_abs_tv_yaw_rate_);
    const double yaw_from_rate = smpc_lane_change::wrapToPi(
        v.yaw + yaw_rate_limited * std::min(tau, tv_yaw_rate_horizon_sec_));

    p.valid = true;
    p.t = tau;
    p.x = sample.x;
    p.y = sample.y;
    p.s = sample.s;
    p.d = sample.d;
    p.v = predicted_v_long;
    const double dynamic_yaw = angleLerp(yaw_from_frenet_vel, yaw_from_rate,
                                        std::clamp(tv_yaw_rate_weight_, 0.0, 1.0));
    p.yaw = angleLerp(dynamic_yaw, sample.yaw,
                      std::clamp(tv_csv_yaw_weight_, 0.0, 1.0));
    return p;
  }

  static double axisExtent(const OrientedBox& box, double ax, double ay) {
    const double ux = std::cos(box.yaw);
    const double uy = std::sin(box.yaw);
    const double vx = -std::sin(box.yaw);
    const double vy = std::cos(box.yaw);
    return box.half_l * std::abs(ux * ax + uy * ay) +
           box.half_w * std::abs(vx * ax + vy * ay);
  }

  CollisionEval collisionProbability(const PredPose& ego,
                                     const PredPose& tv,
                                     const smpc_lane_change::TargetVehicle& vehicle) const {
    CollisionEval out;
    const OrientedBox ego_box{
        ego.x, ego.y, ego.yaw, 0.5 * ego_length_m_, 0.5 * ego_width_m_};
    const OrientedBox tv_box{
        tv.x, tv.y, tv.yaw, 0.5 * std::max(0.1, vehicle.length),
        0.5 * std::max(0.1, vehicle.width)};

    const std::array<std::array<double, 2>, 4> axes{{
        {std::cos(ego_box.yaw), std::sin(ego_box.yaw)},
        {-std::sin(ego_box.yaw), std::cos(ego_box.yaw)},
        {std::cos(tv_box.yaw), std::sin(tv_box.yaw)},
        {-std::sin(tv_box.yaw), std::cos(tv_box.yaw)},
    }};

    const double dx = tv_box.x - ego_box.x;
    const double dy = tv_box.y - ego_box.y;
    const double sigma_long = std::max(
        1e-3, prediction_sigma_base_m_ + prediction_sigma_growth_mps_ * ego.t);
    const double sigma_lat = std::max(
        1e-3, prediction_lateral_sigma_base_m_ +
                   prediction_lateral_sigma_growth_mps_ * ego.t);

    double collision_prob = 1.0;
    double best_separating_gap = -std::numeric_limits<double>::infinity();
    for (const auto& axis : axes) {
      const double ax = axis[0];
      const double ay = axis[1];
      const double center_distance = std::abs(dx * ax + dy * ay);
      const double gap = center_distance -
          axisExtent(ego_box, ax, ay) - axisExtent(tv_box, ax, ay);
      best_separating_gap = std::max(best_separating_gap, gap);

      const double ego_long_axis_alignment =
          std::abs(std::cos(ego_box.yaw) * ax + std::sin(ego_box.yaw) * ay);
      const double tv_long_axis_alignment =
          std::abs(std::cos(tv_box.yaw) * ax + std::sin(tv_box.yaw) * ay);
      const double longness = std::max(ego_long_axis_alignment, tv_long_axis_alignment);
      const double sigma = longness * sigma_long + (1.0 - longness) * sigma_lat;
      const double p_axis_overlap = normalCdf((collision_margin_m_ - gap) / sigma);
      collision_prob = std::min(collision_prob, p_axis_overlap);
    }

    out.probability = clampProbability(collision_prob);
    out.signed_clearance = best_separating_gap - collision_margin_m_;
    return out;
  }

  TrajectoryEval evaluateTrajectory(
      const std::vector<PredPose>& ego_traj,
      const std::vector<smpc_lane_change::TargetVehicle>& vehicles,
      const smpc_lane_change::TargetVehicleSet* merge_context = nullptr,
      bool change_left = false,
      double evaluation_end_sec = std::numeric_limits<double>::infinity()) const {
    TrajectoryEval out;
    if (ego_traj.empty()) {
      out.detail = "empty_ego_traj";
      return out;
    }

    out.valid = true;
    const std::size_t steps = ego_traj.size();
    std::vector<double> no_collision_probability_by_step(steps, 1.0);
    out.risk_by_step.assign(steps, 0.0);
    out.min_signed_clearance_by_step.assign(
        steps, std::numeric_limits<double>::infinity());

    // Retain the legacy result in out.risk while the time-indexed gate is
    // calibrated.  The new result is out.risk_by_step / peak_step_risk.
    double legacy_no_collision_probability = 1.0;

    for (const auto& vehicle : vehicles) {
      const auto modes = modesForVehicle(vehicle);
      VehicleRiskTrace vehicle_trace;
      vehicle_trace.unique_id = vehicle.unique_id;
      vehicle_trace.role = vehicle.role;
      vehicle_trace.priority_scale =
          mergePriorityRiskScale(merge_context, vehicle, change_left);
      vehicle_trace.mixed_risk_by_step.assign(steps, 0.0);

      double legacy_expected_vehicle_risk = 0.0;

      for (const auto& mode : modes) {
        ModeRiskTrace mode_trace;
        mode_trace.name = mode.name;
        mode_trace.probability = clampProbability(mode.probability);
        mode_trace.conditional_risk_by_step.assign(steps, 0.0);
        double legacy_mode_risk = 0.0;

        for (std::size_t k = 0; k < steps; ++k) {
          const auto& ego = ego_traj[k];
          // A lane-change candidate owns the merge only until the lateral
          // transition and post-merge settling buffer have finished.  Beyond
          // that point the 20 Hz receding-horizon ACC/lane decision takes
          // over.  Testing the full prediction horizon here rejected usable
          // gaps because a rear vehicle could catch ego several seconds after
          // the merge had already completed.
          if (change_left && ego.t > evaluation_end_sec + 1e-6) break;
          const auto tv = predictTargetPose(vehicle, mode, ego.t);
          // Never turn a prediction that ran out of CSV coverage into zero
          // collision risk.  The caller must keep/stop or provide route
          // continuation before accepting a deferred lane-change candidate.
          if (!tv.valid) {
            out.valid = false;
            out.detail = "target_prediction_out_of_coverage";
            return out;
          }
          const auto collision = collisionProbability(ego, tv, vehicle);
          const double conditional_risk = clampProbability(collision.probability);
          mode_trace.conditional_risk_by_step[k] = conditional_risk;
          vehicle_trace.mixed_risk_by_step[k] +=
              mode_trace.probability * conditional_risk;
          legacy_mode_risk = std::max(legacy_mode_risk, conditional_risk);
          out.min_signed_clearance = std::min(out.min_signed_clearance,
                                              collision.signed_clearance);
          out.min_signed_clearance_by_step[k] = std::min(
              out.min_signed_clearance_by_step[k], collision.signed_clearance);
        }
        legacy_expected_vehicle_risk += mode_trace.probability * legacy_mode_risk;
        vehicle_trace.modes.push_back(std::move(mode_trace));
      }

      legacy_expected_vehicle_risk = clampProbability(
          legacy_expected_vehicle_risk * vehicle_trace.priority_scale);
      legacy_no_collision_probability *= (1.0 - legacy_expected_vehicle_risk);
      for (std::size_t k = 0; k < steps; ++k) {
        vehicle_trace.mixed_risk_by_step[k] = clampProbability(
            vehicle_trace.mixed_risk_by_step[k] * vehicle_trace.priority_scale);
        no_collision_probability_by_step[k] *=
            (1.0 - vehicle_trace.mixed_risk_by_step[k]);
      }
      out.vehicle_risk_traces.push_back(std::move(vehicle_trace));
    }

    out.risk = clampProbability(1.0 - legacy_no_collision_probability);
    out.peak_step_risk = 0.0;
    out.peak_step = 0;
    for (std::size_t k = 0; k < steps; ++k) {
      out.risk_by_step[k] = clampProbability(1.0 - no_collision_probability_by_step[k]);
      if (out.risk_by_step[k] > out.peak_step_risk) {
        out.peak_step_risk = out.risk_by_step[k];
        out.peak_step = k;
      }
    }
    if (!std::isfinite(out.min_signed_clearance)) {
      out.min_signed_clearance = std::numeric_limits<double>::infinity();
    }
    return out;
  }

  void publishPredictionPath(const std::vector<PredPose>& traj,
                             const ros::Publisher& pub,
                             const std_msgs::Header& source_header) const {
    nav_msgs::Path path;
    path.header = source_header;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();
    path.poses.reserve(traj.size());
    for (const auto& p : traj) {
      if (!p.valid) continue;
      geometry_msgs::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = p.x;
      pose.pose.position.y = p.y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.z = std::sin(0.5 * p.yaw);
      pose.pose.orientation.w = std::cos(0.5 * p.yaw);
      path.poses.push_back(pose);
    }
    pub.publish(path);
  }

  static geometry_msgs::Point markerPoint(double x, double y, double z) {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
  }

  static void setMarkerColor(visualization_msgs::Marker& marker,
                             double r, double g, double b, double a) {
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
  }

  ros::Duration predictionMarkerLifetime() const {
    return ros::Duration(std::max(0.0, debug_prediction_marker_lifetime_sec_));
  }

  std_msgs::Header predictionMarkerHeader(const std_msgs::Header& source_header) const {
    std_msgs::Header header = source_header;
    header.frame_id = "map";
    header.stamp = ros::Time::now();
    return header;
  }

  void appendTrajectoryLine(visualization_msgs::MarkerArray& array,
                            const std_msgs::Header& header,
                            const std::string& ns,
                            int id,
                            const std::vector<PredPose>& traj,
                            double r,
                            double g,
                            double b,
                            double a,
                            double line_width_scale = 1.0) const {
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = std::max(0.02, debug_prediction_line_width_m_ * line_width_scale);
    marker.lifetime = predictionMarkerLifetime();
    setMarkerColor(marker, r, g, b, a);
    for (const auto& p : traj) {
      if (!p.valid) continue;
      marker.points.push_back(markerPoint(p.x, p.y, debug_prediction_z_m_));
    }
    if (marker.points.size() >= 2) array.markers.push_back(marker);
  }

  void appendFootprints(visualization_msgs::MarkerArray& array,
                        const std_msgs::Header& header,
                        const std::string& ns,
                        int base_id,
                        const std::vector<PredPose>& traj,
                        double length,
                        double width,
                        int stride,
                        double r,
                        double g,
                        double b,
                        double a) const {
    const int safe_stride = std::max(1, stride);
    int marker_index = 0;
    for (std::size_t i = 0; i < traj.size(); i += static_cast<std::size_t>(safe_stride)) {
      const auto& p = traj[i];
      if (!p.valid) continue;
      visualization_msgs::Marker marker;
      marker.header = header;
      marker.ns = ns;
      marker.id = base_id + marker_index;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = p.x;
      marker.pose.position.y = p.y;
      marker.pose.position.z = debug_prediction_z_m_ + 0.5 * debug_prediction_footprint_height_m_;
      marker.pose.orientation.z = std::sin(0.5 * p.yaw);
      marker.pose.orientation.w = std::cos(0.5 * p.yaw);
      marker.scale.x = std::max(0.1, length);
      marker.scale.y = std::max(0.1, width);
      marker.scale.z = std::max(0.02, debug_prediction_footprint_height_m_);
      marker.lifetime = predictionMarkerLifetime();
      setMarkerColor(marker, r, g, b, a);
      array.markers.push_back(marker);
      ++marker_index;
    }
  }

  void appendPredictionLabel(visualization_msgs::MarkerArray& array,
                             const std_msgs::Header& header,
                             const std::string& ns,
                             int id,
                             const PredPose& pose,
                             const std::string& text,
                             double r,
                             double g,
                             double b,
                             double a) const {
    if (!debug_prediction_show_labels_ || !pose.valid) return;
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pose.x;
    marker.pose.position.y = pose.y;
    marker.pose.position.z = debug_prediction_z_m_ + 1.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.75;
    marker.text = text;
    marker.lifetime = predictionMarkerLifetime();
    setMarkerColor(marker, r, g, b, a);
    array.markers.push_back(marker);
  }

  static bool lastValidPose(const std::vector<PredPose>& traj, PredPose& out) {
    for (auto it = traj.rbegin(); it != traj.rend(); ++it) {
      if (it->valid) {
        out = *it;
        return true;
      }
    }
    return false;
  }

  void targetModeColor(const TvMode& mode,
                       double& r,
                       double& g,
                       double& b,
                       double& a) const {
    a = 0.85;
    if (mode.name == "nominal") {
      r = 0.20;
      g = 1.00;
      b = 0.75;
    } else if (mode.name.find("brake") != std::string::npos) {
      r = 1.00;
      g = 0.35;
      b = 0.20;
    } else if (mode.name.find("accel") != std::string::npos) {
      r = 0.25;
      g = 0.55;
      b = 1.00;
    } else {
      r = 0.95;
      g = 0.45;
      b = 1.00;
    }
  }

  std::vector<PredPose> buildTargetTrajectory(const smpc_lane_change::TargetVehicle& vehicle,
                                              const TvMode& mode) const {
    std::vector<PredPose> traj;
    traj.reserve(std::max(1, horizon_steps_));
    for (int k = 1; k <= std::max(1, horizon_steps_); ++k) {
      const double tau = prediction_dt_sec_ * static_cast<double>(k);
      const PredPose pose = predictTargetPose(vehicle, mode, tau);
      if (pose.valid) traj.push_back(pose);
    }
    return traj;
  }

  void appendTargetPredictionMarkers(
      visualization_msgs::MarkerArray& array,
      const std_msgs::Header& header,
      const std::vector<smpc_lane_change::TargetVehicle>& vehicles) const {
    const int max_vehicles = std::max(0, debug_prediction_target_max_vehicles_);
    const int vehicle_count = std::min(static_cast<int>(vehicles.size()), max_vehicles);
    for (int vehicle_index = 0; vehicle_index < vehicle_count; ++vehicle_index) {
      const auto& vehicle = vehicles[vehicle_index];
      const auto modes = modesForVehicle(vehicle);
      for (std::size_t mode_index = 0; mode_index < modes.size(); ++mode_index) {
        if (!debug_prediction_show_target_modes_ && mode_index > 0) continue;
        const auto& mode = modes[mode_index];
        const auto traj = buildTargetTrajectory(vehicle, mode);
        double r = 0.0, g = 1.0, b = 0.0, a = 0.85;
        targetModeColor(mode, r, g, b, a);
        if (mode_index > 0) a *= 0.75;

        const int marker_base = vehicle_index * 20 + static_cast<int>(mode_index);
        appendTrajectoryLine(array, header, "smpc_tv_prediction_lines", marker_base,
                             traj, r, g, b, a, mode_index == 0 ? 1.0 : 0.65);

        if (mode_index == 0) {
          appendFootprints(array, header, "smpc_tv_prediction_footprints",
                           vehicle_index * 100, traj, vehicle.length, vehicle.width,
                           debug_prediction_target_footprint_stride_, r, g, b, 0.22);
        }

        PredPose label_pose;
        if (lastValidPose(traj, label_pose)) {
          std::ostringstream ss;
          ss.setf(std::ios::fixed);
          ss.precision(2);
          ss << "TV " << vehicle.unique_id << " " << mode.name
             << "\np=" << mode.probability;
          appendPredictionLabel(array, header, "smpc_tv_prediction_labels", marker_base,
                                label_pose, ss.str(), r, g, b, 0.95);
        }
      }
    }
  }

  void publishPredictionMarkers(const smpc_lane_change::TargetVehicleSet& t,
                                const std::vector<PredPose>& keep_traj,
                                const std::vector<PredPose>& change_traj,
                                const std::vector<smpc_lane_change::TargetVehicle>& vehicles) const {
    if (!debug_prediction_publish_markers_) return;

    visualization_msgs::MarkerArray array;
    const auto header = predictionMarkerHeader(t.header);

    visualization_msgs::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::Marker::DELETEALL;
    array.markers.push_back(clear);

    appendTrajectoryLine(array, header, "smpc_ego_keep_prediction_line", 0,
                         keep_traj, 0.20, 0.75, 1.00, 0.95, 1.1);
    appendTrajectoryLine(array, header, "smpc_ego_change_prediction_line", 0,
                         change_traj, 1.00, 0.75, 0.10, 0.95, 1.4);
    appendFootprints(array, header, "smpc_ego_keep_prediction_footprints", 0,
                     keep_traj, ego_length_m_, ego_width_m_,
                     debug_prediction_ego_footprint_stride_, 0.20, 0.75, 1.00, 0.16);
    appendFootprints(array, header, "smpc_ego_change_prediction_footprints", 0,
                     change_traj, ego_length_m_, ego_width_m_,
                     debug_prediction_ego_footprint_stride_, 1.00, 0.75, 0.10, 0.20);

    PredPose keep_end;
    if (lastValidPose(keep_traj, keep_end)) {
      appendPredictionLabel(array, header, "smpc_ego_prediction_labels", 0,
                            keep_end, "ego keep", 0.20, 0.75, 1.00, 0.95);
    }
    PredPose change_end;
    if (lastValidPose(change_traj, change_end)) {
      appendPredictionLabel(array, header, "smpc_ego_prediction_labels", 1,
                            change_end, "ego change", 1.00, 0.75, 0.10, 0.95);
    }

    appendTargetPredictionMarkers(array, header, vehicles);
    prediction_marker_pub_.publish(array);
  }

  static const char* behaviorActionName(BehaviorAction action) {
    switch (action) {
      case BehaviorAction::kKeep: return "KEEP";
      case BehaviorAction::kWaitForGap: return "WAIT_FOR_GAP";
      case BehaviorAction::kChangeNow: return "CHANGE_NOW";
      case BehaviorAction::kYieldDecelThenChange: return "YIELD_DECEL_THEN_CHANGE";
      case BehaviorAction::kHoldThenChange: return "HOLD_THEN_CHANGE";
      case BehaviorAction::kAccelThenChange: return "ACCEL_THEN_CHANGE";
    }
    return "KEEP";
  }

  static uint8_t behaviorPlanActionMessageValue(BehaviorAction action) {
    switch (action) {
      case BehaviorAction::kWaitForGap:
        return smpc_lane_change::BehaviorPlan::WAIT_FOR_GAP;
      case BehaviorAction::kChangeNow:
        return smpc_lane_change::BehaviorPlan::CHANGE_NOW;
      case BehaviorAction::kYieldDecelThenChange:
        return smpc_lane_change::BehaviorPlan::YIELD_DECEL_THEN_CHANGE;
      case BehaviorAction::kHoldThenChange:
        return smpc_lane_change::BehaviorPlan::HOLD_THEN_CHANGE;
      case BehaviorAction::kAccelThenChange:
        return smpc_lane_change::BehaviorPlan::ACCEL_THEN_CHANGE;
      case BehaviorAction::kKeep:
      default:
        return smpc_lane_change::BehaviorPlan::KEEP;
    }
  }

  static uint8_t behaviorRequestActionMessageValue(BehaviorAction action) {
    switch (action) {
      case BehaviorAction::kWaitForGap:
        return smpc_lane_change::BehaviorLongitudinalRequest::WAIT_FOR_GAP;
      case BehaviorAction::kYieldDecelThenChange:
        return smpc_lane_change::BehaviorLongitudinalRequest::YIELD;
      case BehaviorAction::kHoldThenChange:
        return smpc_lane_change::BehaviorLongitudinalRequest::HOLD;
      case BehaviorAction::kAccelThenChange:
        return smpc_lane_change::BehaviorLongitudinalRequest::ACCELERATE;
      case BehaviorAction::kKeep:
      case BehaviorAction::kChangeNow:
      default:
        return smpc_lane_change::BehaviorLongitudinalRequest::NONE;
    }
  }

  bool candidateFitsHorizon(const ActionSpec& spec) const {
    if (!spec.change_left) return true;
    const double required = effectiveLaneChangeStartSec(spec) +
        std::max(0.1, spec.lane_change_duration_sec) +
        std::max(0.0, behavior_post_merge_buffer_sec_);
    const double available = std::max(1, horizon_steps_) * prediction_dt_sec_;
    return required <= available + 1e-6;
  }

  // 차선변경에 필요한 남은 차로 길이.
  //
  // 중간 기준(기본, 2026-09-26): 실행지연 + 블렌드의 lane_deadline_blend_fraction(0.5)
  // 지점까지 간 뒤에도 lane_deadline_margin_m 이 남아야 한다. 블렌드 절반이면 자차
  // 중심이 차선을 넘어 대부분 목표 차로 쪽에 있다. waypoint_system 은 남은 차로 길이에
  // 맞춰 블렌드를 줄이므로(13:48 백: 남은 46.6 m -> 블렌드 44.5 m, 최소 20 m) 경로
  // 자체는 끝까지 만들어진다. 변경 중 충돌/간격 검사는 그대로 적용된다.
  //   이전 기준(블렌드 전체 + 합류 후 1 s + 차 길이 ~= 5.2 s x 속도)은 0번 차로
  //   합류 구간 135 m 중 앞 55 m 만 결정에 쓸 수 있어, 13:48 백에서 옆 차가 빠진
  //   16.1 s(남은 72.6 m, 요구 76 m)에 변경을 놓쳤다.
  double laneDeadlineRequiredDistance(
      const smpc_lane_change::TargetVehicleSet& t, const ActionSpec& spec) const {
    const bool preview_blend = referencePreviewFreshFor(t) &&
        reference_preview_.spatial_blend_length_m > 0.0;
    if (lane_deadline_midpoint_enabled_) {
      const double fraction = std::clamp(lane_deadline_blend_fraction_, 0.0, 1.0);
      const double start = effectiveLaneChangeStartSec(spec);
      const double required = preview_blend
          ? actionLongitudinalRollout(start, egoPredictionTargetSpeedMps(), spec).ds +
                fraction * reference_preview_.spatial_blend_length_m
          : actionLongitudinalRollout(
                start + fraction * std::max(0.1, spec.lane_change_duration_sec),
                egoPredictionTargetSpeedMps(), spec).ds;
      return required + std::max(0.0, lane_deadline_margin_m_);
    }
    const double finish = effectiveLaneChangeStartSec(spec) +
        std::max(0.1, spec.lane_change_duration_sec) +
        std::max(0.0, behavior_post_merge_buffer_sec_);
    const auto rollout = actionLongitudinalRollout(
        finish, egoPredictionTargetSpeedMps(), spec);
    // Require room for the complete maneuver, settling and ego footprint.
    const double required_distance = preview_blend
        ? actionLongitudinalRollout(effectiveLaneChangeStartSec(spec),
              egoPredictionTargetSpeedMps(),spec).ds +
              reference_preview_.spatial_blend_length_m +
              rollout.v * std::max(0.0,behavior_post_merge_buffer_sec_)
        : rollout.ds;
    return required_distance + ego_length_m_;
  }

  // 같은 요구거리를 다른 속도 v 에서 추정한다(블렌드 길이는 속도에 비례).
  double laneDeadlineRequiredDistanceAtSpeed(
      const smpc_lane_change::TargetVehicleSet& t, double v) const {
    v = std::max(0.0, v);
    double blend = std::max(0.1, egoChangeBlendDuration()) * v;
    if (referencePreviewFreshFor(t) &&
        reference_preview_.spatial_blend_length_m > 0.0 && ego_speed_mps_ > 1.0) {
      blend = reference_preview_.spatial_blend_length_m * v / ego_speed_mps_;
    }
    if (lane_deadline_midpoint_enabled_) {
      return v * std::max(0.0, behavior_execution_delay_sec_) +
          std::clamp(lane_deadline_blend_fraction_, 0.0, 1.0) * blend +
          std::max(0.0, lane_deadline_margin_m_);
    }
    return v * std::max(0.0, behavior_execution_delay_sec_) + blend +
        v * std::max(0.0, behavior_post_merge_buffer_sec_) + ego_length_m_;
  }

  // 차로 끝에 멈춰 선 상태(비상 플래그 + 거의 정지)에서는 남은 길이가 0 에 가까워
  // 어떤 요구거리도 만족할 수 없다. 이때 합류는 waypoint_system 의 endpoint
  // connector 가 맡으므로 거리 검사를 건너뛴다(간격/TTC/궤적 검사는 그대로).
  //   실측 2026-09-26(13:48 백): 25 s 부터 남은 0.9 m 에서 정지, 간격이 비어도
  //   (위험도 0, TTC/예측 통과) 이 검사 하나로 10 초 넘게 변경 불가였다.
  bool laneEndStandstillMergeActive(
      const smpc_lane_change::TargetVehicleSet& t) const {
    return lane_end_standstill_merge_enabled_ && t.emergency_stop_required &&
        ego_speed_mps_ <= std::max(0.0, lane_end_standstill_merge_max_speed_mps_);
  }

  bool candidateFitsLaneDeadline(
      const smpc_lane_change::TargetVehicleSet& t, const ActionSpec& spec) const {
    if (!spec.change_left) return true;
    if (t.current_lane_id < 0 || t.current_lane_id >= 3 ||
        t.target_lane_id != t.current_lane_id + 1) return false;
    if (laneEndStandstillMergeActive(t)) return true;
    if (!std::isfinite(t.distance_to_lane_end)) return false;
    const double required_distance = laneDeadlineRequiredDistance(t, spec);
    return std::isfinite(required_distance) &&
        required_distance <= t.distance_to_lane_end;
  }


  // ===== 목표 차로 틈 선택(2026-09-26) =====
  // 목표 차로 차량을 위치순으로 늘어놓고, 연속한 두 차 사이(및 맨 앞/맨 뒤 열린 틈)에
  // 자차가 안전영역(앞 front_window, 뒤 rear_window)을 지키며 들어갈 수 있는 자리를
  // 만든다. 각 자리에 자차를 맞추는 속도 제어를 짧게 시뮬레이션해, 차로 끝(중간 기준)
  // 전에 맞출 수 있는 자리 중 가장 빨리 맞는 곳을 고른다. 고른 틈은 앞/뒤 차량 id 로
  // 고정하고, 그 틈의 속도 + 위치 오차 보정으로 종방향 명령을 낸다.
  // 실제 변경 허가는 기존 CHANGE_NOW 게이트(안전영역/TTC/궤적/차로 끝)가 그대로 한다.
  struct GapVehicle {
    int id{-1};
    double rear_edge{0.0};   // 자차 중심 기준 종방향 위치
    double front_edge{0.0};
    double v{0.0};
    bool speed_known{false};  // 신뢰/최근 신뢰/위치 변화로 속도를 앎(새 트랙 가정값이 아님)
    double accel{0.0};        // 2026-10-13 최근 1 s 속도 기울기(데드밴드 적용)
  };

  struct MergeGap {
    int follower_id{-1};     // -1: 뒤가 열린 틈(맨 뒤 차량 뒤)
    double leader_rear_edge0{std::numeric_limits<double>::infinity()};
    int leader_id{-1};       // -1: 앞이 열린 틈(맨 앞 차량 앞)
    double lo0{-std::numeric_limits<double>::infinity()};  // 자차 중심 허용 하한(현재)
    double hi0{std::numeric_limits<double>::infinity()};   // 자차 중심 허용 상한(현재)
    double v_lo{0.0};        // 하한이 움직이는 속도(뒤차 속도)
    double v_hi{0.0};        // 상한이 움직이는 속도(앞차 속도)
    bool feasible{false};    // 맞출 수 있고 차로 끝(중간 기준)도 만족
    bool aligns{false};      // 시뮬레이션 안에 맞출 수는 있음(차로 끝은 넘을 수도 있음)
    double overshoot_m{std::numeric_limits<double>::infinity()};  // 차로 끝 초과 거리(<=0 이면 만족)
    bool aligned_now{false};
    bool speed_known{true};  // 경계 차량 속도를 모두 앎
    double align_time{std::numeric_limits<double>::infinity()};
    double end_speed{0.0};
    double cost{std::numeric_limits<double>::infinity()};
    // 시뮬레이션에 쓴 속도 상한의 범위(디버그용).
    double vlim_min{std::numeric_limits<double>::infinity()};
    double vlim_max{0.0};

    // 경계 차량이 흐름 속도(v_flow)로 a_flow 만큼씩 수렴한다고 본다. v_flow < 0 이면 등속.
    //   합류 구간의 본선 차량은 곡선을 빠져나오며 9 m/s 로 발견된 뒤 흐름(~11 m/s)까지
    //   가속한다(2026-10-01 22:25 백 id6: 2.8 초 동안 9.0 -> 11.2 m/s). 등속 가정에서는
    //   그 차 앞 틈이 '가능(여유 12 m)'으로 나왔다가 매 주기 여유가 줄어 2.8 초 뒤 불가능이
    //   됐고, 자차는 그 차 바로 옆에 있었다. 뒤차 12.8 / 앞차 9.0 인 틈은 '닫힌다'로 버려졌다.
    double v_flow{-1.0};
    double a_flow{0.0};

    bool sameKey(int f, int l) const { return follower_id == f && leader_id == l; }
    double edgeSpeed(double v0, double time) const {
      if (v_flow < 0.0 || a_flow <= 0.0) return v0;
      const double dv = v_flow - v0;
      if (time >= std::abs(dv) / a_flow) return v_flow;
      return v0 + std::copysign(a_flow * time, dv);
    }
    double edgeTravel(double v0, double time) const {
      if (v_flow < 0.0 || a_flow <= 0.0) return v0 * time;
      const double dv = v_flow - v0;
      const double tc = std::abs(dv) / a_flow;
      const double a = std::copysign(a_flow, dv);
      if (time <= tc) return v0 * time + 0.5 * a * time * time;
      return v0 * tc + 0.5 * a * tc * tc + v_flow * (time - tc);
    }
    double vLo(double time) const { return a_lo != 0.0 ? accSpeed(v_lo, a_lo, time) : edgeSpeed(v_lo, time); }
    double vHi(double time) const { return a_hi != 0.0 ? accSpeed(v_hi, a_hi, time) : edgeSpeed(v_hi, time); }
    // 2026-10-13 경계 차량별 가속도(데드밴드 적용). 0 이면 기존 흐름/등속 모델.
    double a_lo{0.0};
    double a_hi{0.0};
    double v_edge_max{27.8};
    double accSpeed(double v0, double a, double time) const {
      const double vt = a > 0.0 ? std::max(v0, v_edge_max) : 0.0;
      return a > 0.0 ? std::min(vt, v0 + a * time) : std::max(vt, v0 + a * time);
    }
    double accTravel(double v0, double a, double time) const {
      const double vt = a > 0.0 ? std::max(v0, v_edge_max) : 0.0;
      const double tc = std::max(0.0, (vt - v0) / a);
      if (time <= tc) return v0 * time + 0.5 * a * time * time;
      return v0 * tc + 0.5 * a * tc * tc + vt * (time - tc);
    }
    double lo(double time) const { return lo0 + (a_lo != 0.0 ? accTravel(v_lo, a_lo, time) : edgeTravel(v_lo, time)); }
    double hi(double time) const { return hi0 + (a_hi != 0.0 ? accTravel(v_hi, a_hi, time) : edgeTravel(v_hi, time)); }
    double speed() const {
      if (follower_id >= 0 && leader_id >= 0) return 0.5 * (v_lo + v_hi);
      return follower_id >= 0 ? v_lo : v_hi;
    }
  };

  // 틈 선택용 차량 속도. 신뢰 속도 -> 최근(3 s) 신뢰 속도 -> 차로 s 변화 평균(0.5 s 이상)
  // -> 원시 속도(자차 +-5 m/s 제한) 순. 불신 속도를 자차 속도로 두면 상대 이동이 0 이
  // 돼 느린 차를 앞지르는 틈이 '불가능'으로 나왔다(14:41 백 id1: 실제 11 m/s).
  // 원시 속도를 그대로 쓰면 새 트랙에서 6 m/s 대로 튄다.
  struct GapSpeedHistory {
    double trusted_v{0.0};
    ros::Time trusted_stamp;
    std::deque<std::pair<ros::Time, double>> s_hist;
    ros::Time last_seen;
  };

  double gapVehicleSpeed(const smpc_lane_change::TargetVehicle& v,
                         bool* known = nullptr) const {
    if (gap_shared_speed_enabled_) {
      const auto estimate=gap_speed_policy_.get(v,ros::Time::now().toSec());
      if(known) *known=estimate.known;
      return estimate.known ? estimate.speed : std::max(0.,ego_speed_mps_);
    }
    if (known) *known = true;
    const ros::Time now = ros::Time::now();
    const bool trusted = targetVelocityTrusted(v);
    if (v.unique_id < 0) {
      if (trusted) return std::max(0.0, v.v_long);
      if (known) *known = false;
      return std::isfinite(v.v_long)
          ? std::clamp(v.v_long, std::max(0.0, ego_speed_mps_ - 5.0), ego_speed_mps_ + 5.0)
          : ego_speed_mps_;
    }
    auto& h = gap_speed_history_[v.unique_id];
    h.last_seen = now;
    if (std::isfinite(v.s)) {
      if (h.s_hist.empty() || (now - h.s_hist.back().first).toSec() >= 0.05) {
        h.s_hist.emplace_back(now, v.s);
      }
      while (h.s_hist.size() > 2 && (now - h.s_hist.front().first).toSec() > 1.5) {
        h.s_hist.pop_front();
      }
    }
    if (trusted) {
      h.trusted_v = std::max(0.0, v.v_long);
      h.trusted_stamp = now;
      return h.trusted_v;
    }
    if (!h.trusted_stamp.isZero() && (now - h.trusted_stamp).toSec() <= 3.0) {
      return h.trusted_v;
    }
    if (h.s_hist.size() >= 2) {
      const double dt = (h.s_hist.back().first - h.s_hist.front().first).toSec();
      if (dt >= 0.5) {
        return std::max(0.0, (h.s_hist.back().second - h.s_hist.front().second) / dt);
      }
    }
    if (known) *known = false;
    // 이력 없는 새 트랙(0.5 s 미만): 자차 속도(상대 이동 없음)로 둔다. 트래커 초기 속도는
    // 낮게 나와(16:02 백 첫 프레임 4.9/6.5 m/s) 한 프레임 감속 명령을 만들었다.
    // 서 있는 물체 필터는 원시 속도를 따로 본다(targetLaneGapVehicles).
    return ego_speed_mps_;
  }

  void pruneGapSpeedHistory() const {
    const ros::Time now = ros::Time::now();
    for (auto it = gap_speed_history_.begin(); it != gap_speed_history_.end();) {
      if ((now - it->second.last_seen).toSec() > 5.0) it = gap_speed_history_.erase(it);
      else ++it;
    }
  }

  // 기준 차량 가속도(데드밴드 적용). raw 에 원값. 이력이 모자라면 0(등속 가정, 지금과 같음).
  double refAccel(int id, double now, double* raw = nullptr) const {
    if (raw) *raw = 0.0;
    if (!gap_overtake_accel_policy_enabled_ || id < 0) return 0.0;
    const auto it = ref_speed_slope_.find(id);
    double a = 0.0;
    if (it == ref_speed_slope_.end() || now - it->second.lastStamp() > 0.5 ||
        !it->second.slope(now, gap_ref_accel_window_sec_, 4, a)) return 0.0;
    if (raw) *raw = a;
    return smpc_lane_change::accelDeadband(a, overtake_policy_.deadband);
  }
  double egoAccelMeasured(double now, bool* valid) const {
    double a = 0.0;
    *valid = now - ego_speed_slope_.lastStamp() <= 0.5 &&
             ego_speed_slope_.slope(now, gap_ref_accel_window_sec_, 4, a);
    return *valid ? a : 0.0;
  }

  std::vector<GapVehicle> targetLaneGapVehicles(
      const smpc_lane_change::TargetVehicleSet& t) const {
    pruneGapSpeedHistory();
    std::vector<GapVehicle> out;
    std::set<int> seen;
    const auto add = [&](const smpc_lane_change::TargetVehicle& v) {
      if (!v.valid || v.lane_id != t.target_lane_id) return;
      if (v.unique_id >= 0 && !seen.insert(v.unique_id).second) return;
      GapVehicle g;
      g.id = v.unique_id;
      g.rear_edge = rearEdgeDeltaS(v);
      g.front_edge = frontEdgeDeltaS(v);
      g.v = gapVehicleSpeed(v, &g.speed_known);
      g.accel = refAccel(v.unique_id, ros::Time::now().toSec());
      // 자차 뒤에 거의 서 있는 물체는 뒤에서 다가올 수 없으므로 틈을 나누지 않는다.
      //   16:02 백: 목표 차로 40~60 m 뒤 0 m/s 미확정 물체(target_far_candidate id2/3/4)가
      //   잡혔다 사라졌다 하며 같은 자리를 2/1, 4/1, -1/1 로 바꿔 놓았다.
      if (g.front_edge < 0.0 &&
          (g.v < gap_select_ignore_stationary_behind_mps_ ||
           (std::isfinite(v.v_long) && v.v_long < gap_select_ignore_stationary_behind_mps_))) {
        return;
      }
      out.push_back(g);
    };
    add(t.target_front);
    add(t.target_rear);
    for (const auto& nb : t.nearby_vehicles) add(nb);
    std::sort(out.begin(), out.end(), [](const GapVehicle& a, const GapVehicle& b) {
      return a.rear_edge + a.front_edge < b.rear_edge + b.front_edge;
    });
    return out;
  }

  // 자차 중심이 가야 할 자리(자차 현재 중심 기준 종방향 위치). 이미 자리 안이면
  // 현재 위치를 그대로 쓰고(불필요한 가감속 없음), 밖이면 가장 가까운 안쪽 지점.
  double gapTargetPos(const MergeGap& gap, double time, double ego_pos) const {
    const double inner = std::max(0.0, gap_select_inner_margin_m_);
    const bool has_lo = std::isfinite(gap.lo0);
    const bool has_hi = std::isfinite(gap.hi0);
    if (has_lo && has_hi) {
      const double lo = gap.lo(time);
      const double hi = gap.hi(time);
      const double in = std::min(inner, 0.5 * std::max(0.0, hi - lo));
      return std::clamp(ego_pos, lo + in, hi - in);
    }
    if (has_lo) return std::max(ego_pos, gap.lo(time) + gap_select_open_gap_extra_m_);
    return std::min(ego_pos, gap.hi(time) - gap_select_open_gap_extra_m_);
  }

  // 틈 자체의 기준 위치(고정된 틈을 id 가 바뀌어도 이어 찾을 때 사용).
  double gapReferencePos(const MergeGap& gap) const {
    if (std::isfinite(gap.lo0) && std::isfinite(gap.hi0)) return 0.5 * (gap.lo0 + gap.hi0);
    return std::isfinite(gap.lo0) ? gap.lo0 : gap.hi0;
  }

  // 자리를 목표로 하는 속도 명령(시뮬레이션과 실제 명령이 같은 식을 쓴다).
  //   앞 경계 허용 상한 = 앞차 속도 + gain x (앞 경계 - 안쪽여유 - 자차 위치)
  //   뒤 경계 허용 하한 = 뒤차 속도 + gain x (뒤 경계 + 안쪽여유 - 자차 위치)
  // 경계에서 멀면 제한이 사실상 없고, 가까워질수록 그 차 속도로 수렴한다(경계 도달
  // 시간 약 1/gain 초 보장). 명령은 허용 상한(mission 속도 이하)이라 자리 안에서
  // 불필요하게 감속하지 않고, 앞 경계를 넘어서 있으면 감속, 뒤차보다 빨라야 하면
  // 상한을 열어 ACC 가 가속한다.
  //   16:02 백: 앞차가 45 m 앞인데도 '앞차보다 2 m/s 이상 빠르면 안 됨' 조건 때문에
  //   느린 옆 차를 앞지르는 틈이 8.8 s(불가능)로 계산됐다.
  double gapUpperSpeed(const MergeGap& gap, double time, double ego_pos) const {
    if (!std::isfinite(gap.hi0)) return std::numeric_limits<double>::infinity();
    const double edge = std::isfinite(gap.lo0)
        ? gap.hi(time) - std::min(std::max(0.0, gap_select_inner_margin_m_),
                                  0.5 * std::max(0.0, gap.hi(time) - gap.lo(time)))
        : gap.hi(time) - gap_select_open_gap_extra_m_;
    return gap.vHi(time) + gap_select_position_gain_ * (edge - ego_pos);
  }

  double gapLowerSpeed(const MergeGap& gap, double time, double ego_pos) const {
    if (!std::isfinite(gap.lo0)) return -std::numeric_limits<double>::infinity();
    const double edge = std::isfinite(gap.hi0)
        ? gap.lo(time) + std::min(std::max(0.0, gap_select_inner_margin_m_),
                                  0.5 * std::max(0.0, gap.hi(time) - gap.lo(time)))
        : gap.lo(time) + gap_select_open_gap_extra_m_;
    return gap.vLo(time) + gap_select_position_gain_ * (edge - ego_pos);
  }

  double gapAlignSpeedCommand(const MergeGap& gap, double time, double ego_pos,
                              double ego_v, double v_max) const {
    (void)ego_v;
    v_max = std::max(0.0, v_max);
    const double v_min = std::min(v_max, std::max(0.0, gap_select_min_speed_mps_));
    const double catchup = std::max(0.0, gap_select_max_catchup_mps_);
    const double upper = gapUpperSpeed(gap, time, ego_pos);
    const double lower = gapLowerSpeed(gap, time, ego_pos);
    double v = std::min(v_max, upper);
    if (std::isfinite(lower) && std::isfinite(upper) && lower > upper) {
      v = 0.5 * (lower + upper);  // 자리가 좁아지는 중: 두 요구의 중간
    }
    // 경계 차량 속도 기준 상대속도 한도.
    double ref_lo = std::numeric_limits<double>::infinity();
    double ref_hi = -std::numeric_limits<double>::infinity();
    if (std::isfinite(gap.lo0)) { ref_lo = std::min(ref_lo, gap.vLo(time)); ref_hi = std::max(ref_hi, gap.vLo(time)); }
    if (std::isfinite(gap.hi0)) { ref_lo = std::min(ref_lo, gap.vHi(time)); ref_hi = std::max(ref_hi, gap.vHi(time)); }
    if (std::isfinite(ref_lo)) v = std::clamp(v, ref_lo - catchup, std::max(ref_hi + catchup, v_min));
    if(gap.leader_id>=0 && std::isfinite(gap.leader_rear_edge0)) {
      const double bumper_gap=gap.leader_rear_edge0+gap.edgeTravel(gap.v_hi,time)-ego_pos-.5*ego_length_m_;
      v=std::min(v,smpc_lane_change::mergeFollowSpeed(bumper_gap,gap.vHi(time),
          merge_follow_gap_m_,merge_follow_close_gain_,merge_follow_gain_,gap_response_enabled_ ? gap_selected_max_catchup_mps_ : merge_follow_catchup_mps_,v_min,v_max));
    }
    return std::clamp(v, v_min, v_max);
  }

  bool gapAligned(const MergeGap& gap, double time, double ego_pos, double ego_v) const {
    const double margin = std::max(0.0, gap_select_align_margin_m_);
    const double tol = std::max(0.0, gap_select_align_speed_tol_mps_);
    // 위치는 자리 안, 속도는 경계까지 거리로 정한 허용 범위 안(+-tol).
    if (std::isfinite(gap.lo0) && ego_pos < gap.lo(time) + margin) return false;
    if (std::isfinite(gap.hi0) && ego_pos > gap.hi(time) - margin) return false;
    if (std::isfinite(gap.hi0) &&
        ego_v > gap.vHi(time) + gap_select_position_gain_ *
                               std::max(0.0, gap.hi(time) - margin - ego_pos) + tol) return false;
    if (std::isfinite(gap.lo0) &&
        ego_v < gap.vLo(time) - gap_select_position_gain_ *
                               std::max(0.0, ego_pos - gap.lo(time) - margin) - tol) return false;
    return true;
  }

  void evaluateMergeGap(const smpc_lane_change::TargetVehicleSet& t, MergeGap& gap,
                        double v_max) const {
    gap.feasible = false;
    const double dt = 0.1;
    double v = std::max(0.0, ego_speed_mps_);
    double pos = 0.0;
    double effort = 0.0;
    // 같은 차로 앞차가 있으면 ACC 가 그 이상 가속하지 못한다(17:59 백: 목표 차로 차를
    // 앞지르는 틈을 25 m/s 가속 가정으로 '가능' 판정했지만 16 m/s 앞차 때문에 못 감).
    const auto& own = t.current_front;
    const double own_gap0 = own.valid ? frontBumperGap(own) : std::numeric_limits<double>::infinity();
    const bool own_lead = gap_select_own_lead_enabled_ && own.valid &&
        std::isfinite(own_gap0) && own_gap0 > -1.0 && own_gap0 <= gap_select_own_lead_max_gap_m_;
    const double own_v = own_lead ? std::max(0.0, leadTrustedLongSpeed(own)) : 0.0;
    const bool use_profile = gapSpeedProfileUsable(t);
    for (double time = 0.0; time <= gap_select_max_time_sec_ + 1e-6; time += dt) {
      if (gap.hi(time) - gap.lo(time) < std::max(0.0, gap_select_min_slot_m_)) return;
      // 차로 끝을 넘어도 계속 본다: 가능한 틈이 없을 때 best effort 후보(초과 거리)로 쓴다.
      if (gapAligned(gap, time, pos, v)) {
        const double required = pos + laneDeadlineRequiredDistanceAtSpeed(t, v);
        gap.aligns = true;
        gap.overshoot_m = required - t.distance_to_lane_end;
        gap.feasible = gap.overshoot_m <= 0.0;
        gap.aligned_now = time < 1e-6;
        gap.align_time = time;
        gap.end_speed = v;
        gap.cost = time + gap_select_effort_weight_ * effort;
        return;
      }
      // 속도 상한은 지금 위치의 값이 아니라, 시뮬레이션 속 그 지점의 제한속도를 쓴다.
      const double v_cap = independentAccLeadCap(t, use_profile ? gapSpeedCapAt(t.current_lane_id, pos) : v_max);
      gap.vlim_min = std::min(gap.vlim_min, v_cap);
      gap.vlim_max = std::max(gap.vlim_max, v_cap);
      double v_cmd = gapAlignSpeedCommand(gap, time, pos, v, v_cap);
      if (own_lead) {
        const double own_gap = own_gap0 + own_v * time - pos;
        const double desired = gap_select_own_lead_standstill_m_ +
            gap_select_own_lead_time_gap_sec_ * v;
        v_cmd = std::min(v_cmd, std::max(0.0, own_v + gap_select_own_lead_gain_ * (own_gap - desired)));
      }
      const double dv = std::clamp(v_cmd - v, -gap_select_decel_mps2_ * dt,
                                   std::max(gap_select_accel_mps2_, gap_select_overtake_accel_mps2_) * dt);
      effort += std::abs(dv);
      pos += (v + 0.5 * dv) * dt;
      v += dv;
    }
  }

  // 이 차로에서 틈을 볼 때 쓰는 목표 차로 흐름 속도. 설정되지 않은 차로는 -1(등속 가정).
  double gapFlowSpeedFor(int current_lane_id) const {
    if (!gap_fixed_flow_enabled_) return -1.0;
    if (gap_select_flow_speed_mps_ <= 0.0 || gap_select_flow_accel_mps2_ <= 0.0) return -1.0;
    return std::find(gap_select_flow_speed_lanes_.begin(), gap_select_flow_speed_lanes_.end(),
                     current_lane_id) != gap_select_flow_speed_lanes_.end()
        ? gap_select_flow_speed_mps_ : -1.0;
  }

  std::vector<MergeGap> buildMergeGaps(const smpc_lane_change::TargetVehicleSet& t,
                                       double v_max,
                                       std::vector<GapVehicle>* veh_out = nullptr) const {
    std::vector<MergeGap> gaps;
    const auto veh = targetLaneGapVehicles(t);
    if (veh_out) *veh_out = veh;
    if (veh.empty()) return gaps;
    const double half_len = 0.5 * ego_length_m_;
    const double v_ref = std::max(ego_speed_mps_, 0.0);
    const double front_window = yieldZoneWindowAt(true, v_ref);
    const double rear_window = yieldZoneWindowAt(false, v_ref);
    const double slot_margin = std::max(0.0, gap_select_slot_margin_m_);
    const auto lower_from = [&](const GapVehicle& f) {
      return f.front_edge + rear_window + half_len + slot_margin;
    };
    const auto upper_from = [&](const GapVehicle& l) {
      return l.rear_edge - front_window - half_len - slot_margin;
    };
    // 맨 뒤 차량 뒤(앞이 막힌 열린 틈)
    {
      MergeGap g;
      g.leader_id = veh.front().id;
      g.leader_rear_edge0=veh.front().rear_edge;
      g.hi0 = upper_from(veh.front());
      g.v_hi = veh.front().v;
      g.v_lo = g.v_hi;
      g.a_hi = g.a_lo = veh.front().accel;
      g.speed_known = veh.front().speed_known;
      gaps.push_back(g);
    }
    for (size_t i = 0; i + 1 < veh.size(); ++i) {
      MergeGap g;
      g.follower_id = veh[i].id;
      g.leader_id = veh[i + 1].id;
      g.leader_rear_edge0=veh[i+1].rear_edge;
      g.lo0 = lower_from(veh[i]);
      g.hi0 = upper_from(veh[i + 1]);
      g.v_lo = veh[i].v;
      g.v_hi = veh[i + 1].v;
      g.a_lo = veh[i].accel;
      g.a_hi = veh[i + 1].accel;
      g.speed_known = veh[i].speed_known && veh[i + 1].speed_known;
      gaps.push_back(g);
    }
    // 맨 앞 차량 앞(뒤가 막힌 열린 틈)
    {
      MergeGap g;
      g.follower_id = veh.back().id;
      g.lo0 = lower_from(veh.back());
      g.v_lo = veh.back().v;
      g.v_hi = g.v_lo;
      g.a_lo = g.a_hi = veh.back().accel;
      g.speed_known = veh.back().speed_known;
      gaps.push_back(g);
    }
    const double flow = gapFlowSpeedFor(t.current_lane_id);
    for (auto& g : gaps) {
      g.v_flow = flow;
      g.a_flow = gap_select_flow_accel_mps2_;
      g.v_edge_max = gap_ref_speed_cap_mps_;
      evaluateMergeGap(t, g, v_max);
    }
    return gaps;
  }

  // 느린 목표 차로 뒤차를 앞질러 뒤 안전영역을 비운 뒤에도 차선변경이 차로 끝 전에
  // 끝나는지 추정한다. 뒤차는 등속, 자차는 accel 로 v_cap 까지(0 이면 속도 유지).
  bool slowRearOvertakeFits(const smpc_lane_change::TargetVehicleSet& t,
                            const smpc_lane_change::TargetVehicle& rear,
                            double accel, double v_cap, double margin) const {
    if (!rear.valid || !std::isfinite(rear.v_long) ||
        !std::isfinite(t.distance_to_lane_end)) return false;
    const double v_rear0 = gap_shared_speed_enabled_ ? gapVehicleSpeed(rear) : std::max(0.0, rear.v_long);
    // 2026-10-13 뒤차가 가속 중이면(우리 추월 가속보다 빠르거나 상대 가속이 작음) 추월하지 않는다.
    const double a_rear = refAccel(rear.unique_id, ros::Time::now().toSec());
    if (gap_overtake_accel_policy_enabled_ &&
        !smpc_lane_change::overtakeAllowedByAccel(a_rear, overtake_policy_)) return false;
    const double gap0 = rearBumperGap(rear);
    const double dt = 0.1;
    double v = std::max(0.0, ego_speed_mps_);
    double s = 0.0;
    double s_rear = 0.0;
    for (double time = 0.0; time <= slow_rear_overtake_max_sim_sec_ + 1e-6; time += dt) {
      if (gap0 + (s - s_rear) > yieldZoneWindowAt(false, v)) {
        return s + laneDeadlineRequiredDistanceAtSpeed(t, v) + margin <=
            t.distance_to_lane_end;
      }
      const double v_next = std::min(std::max(v, v_cap), v + std::max(0.0, accel) * dt);
      s += 0.5 * (v + v_next) * dt;
      v = v_next;
      const double v_rear = std::clamp(v_rear0 + a_rear * time, 0.0, std::max(v_rear0, gap_ref_speed_cap_mps_));
      s_rear += v_rear * dt;
      if (s >= t.distance_to_lane_end) return false;
    }
    return false;
  }

  std::vector<ActionSpec> buildBehaviorCandidates() const {
    const double duration = egoChangeBlendDuration();
    std::vector<ActionSpec> out;
    out.push_back({BehaviorAction::kChangeNow, 0.0, 0.0, duration, true, "change_now"});
    if (!behavior_planner_enabled_) return out;

    for (const double start_sec : behavior_lc_start_times_sec_) {
      out.push_back({BehaviorAction::kHoldThenChange,
                     behavior_hold_accel_mps2_, start_sec, duration, true,
                     "hold_then_change"});
      // Evaluate the lightest comfortable yield first.  The lexicographic
      // policy still gives every valid HOLD candidate priority over YIELD;
      // within YIELD, cost selects the smallest deceleration that makes a
      // future lane change safe.
      for (const double decel : behavior_yield_decel_candidates_mps2_) {
        out.push_back({BehaviorAction::kYieldDecelThenChange,
                       decel, start_sec, duration, true,
                       "yield_decel_then_change"});
      }
      // Positive-acceleration candidates must not win the action selection
      // while their ACC execution path is intentionally disabled.  Otherwise
      // a non-executed shadow action can hide a real YIELD/HOLD candidate.
      if (behavior_execute_accel_) {
        out.push_back({BehaviorAction::kAccelThenChange,
                       behavior_accel_accel_mps2_, start_sec, duration, true,
                       "accel_then_change"});
      }
    }
    return out;
  }

  bool timeIndexedRiskSafe(const TrajectoryEval& eval, double allowed_risk) const {
    if (!eval.valid || eval.risk_by_step.empty()) return false;
    // When the occupancy-zone/TTC policy is selected, risk remains telemetry
    // and a cost term. Physical trajectory overlap is still rejected by
    // clearanceSafe(), independently of this probabilistic threshold.
    if (!probabilistic_risk_gate_enabled_) return true;
    return std::all_of(eval.risk_by_step.begin(), eval.risk_by_step.end(),
                       [allowed_risk](double risk) {
                         return std::isfinite(risk) && risk <= allowed_risk;
                       });
  }

  bool clearanceSafe(const TrajectoryEval& eval) const {
    if (!eval.valid) return false;
    return std::all_of(eval.min_signed_clearance_by_step.begin(),
                       eval.min_signed_clearance_by_step.end(),
                       [this](double clearance) {
                         // +inf means no relevant vehicle contributed a
                         // clearance at that step.  NaN/-inf are invalid
                         // numerical states and must fail closed.
                         return (std::isinf(clearance) && clearance > 0.0) ||
                             (std::isfinite(clearance) &&
                              clearance >= behavior_min_signed_clearance_m_);
                       });
  }

  double firstRiskViolationTimeSec(const TrajectoryEval& eval,
                                   double allowed_risk) const {
    for (std::size_t k = 0; k < eval.risk_by_step.size(); ++k) {
      if (eval.risk_by_step[k] > allowed_risk) {
        return prediction_dt_sec_ * static_cast<double>(k + 1);
      }
    }
    return -1.0;
  }

  bool behaviorStatusConfirmsLastIssuedPreparation(
      const smpc_lane_change::TargetVehicleSet& t) const {
    // This acknowledgement identifies the already-applied command that
    // explains a lower current ACC baseline.  It deliberately does not try
    // to pre-approve the candidate currently being evaluated: that candidate
    // has not been published yet and may receive a new command revision.
    if (!have_behavior_status_ || last_behavior_plan_id_ == 0 ||
        last_behavior_command_revision_ == 0 ||
        !last_behavior_command_active_) {
      return false;
    }
    const ros::Time now = ros::Time::now();
    if (behavior_status_timeout_sec_ > 0.0 &&
        (now - behavior_status_stamp_).toSec() > behavior_status_timeout_sec_) {
      return false;
    }
    if (behavior_status_.header.stamp.isZero() ||
        behavior_status_.header.stamp > now + ros::Duration(0.05) ||
        (behavior_status_timeout_sec_ > 0.0 &&
         (now - behavior_status_.header.stamp).toSec() > behavior_status_timeout_sec_)) {
      return false;
    }
    constexpr double kSpeedCapMatchToleranceMps = 0.10;
    const auto matches_issued_command =
        [&](uint32_t plan_id, uint32_t command_revision, bool command_active,
            double command_speed_cap_mps) {
          return command_active && plan_id == last_behavior_plan_id_ &&
              plan_id != 0 && command_revision != 0 &&
              behavior_status_.plan_id == plan_id &&
              behavior_status_.command_revision == command_revision &&
              std::isfinite(command_speed_cap_mps) &&
              std::abs(behavior_status_.applied_speed_cap_mps -
                       command_speed_cap_mps) <= kSpeedCapMatchToleranceMps;
        };
    return behavior_status_.behavior_cap_applied &&
        !behavior_status_.external_speed_limit_active &&
        behavior_status_.current_lane_id == t.current_lane_id &&
        behavior_status_.target_lane_id == t.target_lane_id &&
        std::isfinite(behavior_status_.applied_speed_cap_mps) &&
        (matches_issued_command(last_behavior_command_plan_id_,
                                last_behavior_command_revision_,
                                last_behavior_command_active_,
                                last_behavior_command_speed_cap_mps_) ||
         // Decision and ACC timers are asynchronous.  A fresh status can
         // legitimately describe the immediately preceding material command
         // for one cycle while ACC is consuming the latest request.
         matches_issued_command(previous_behavior_command_plan_id_,
                                previous_behavior_command_revision_,
                                previous_behavior_command_active_,
                                previous_behavior_command_speed_cap_mps_));
  }

  bool preparationLongitudinalModelCompatible(
      const smpc_lane_change::TargetVehicleSet& t,
      const ActionSpec& spec,
      double base_speed_cap_mps) const {
    if (spec.action == BehaviorAction::kKeep ||
        spec.action == BehaviorAction::kWaitForGap ||
        spec.action == BehaviorAction::kChangeNow) {
      return true;
    }

    // A preparation action owns only the extra longitudinal cap.  If normal
    // ACC is already asking ego to decelerate, its lead/lane-end logic may
    // legitimately use a stronger brake rate than the behavior request.
    // Do not execute a future candidate whose model cannot represent that
    // safety override; it remains visible as ordinary KEEP/replanning.
    constexpr double kCompatibilityToleranceMps = 0.10;
    if (!std::isfinite(base_speed_cap_mps)) return false;
    if (base_speed_cap_mps + kCompatibilityToleranceMps >= ego_speed_mps_) {
      return true;
    }

    // A lower ACC target is normally an external lead/lane-end brake and
    // makes this simple behavior rollout incompatible.  The only exception
    // is an explicit fresh acknowledgement that the lower target is this
    // same plan's rate-limited behavior cap; without this handshake, a
    // renewed YIELD would oscillate between selected and cancelled.
    return behaviorStatusConfirmsLastIssuedPreparation(t);
  }

  bool projectedTargetGapTtcSafe(
      const smpc_lane_change::TargetVehicleSet& t,
      const std::vector<PredPose>& ego_traj,
      const ActionSpec& spec,
      bool clear_zone = false) const {
    if (!spec.change_left || ego_traj.empty()) return false;

    const double lane_change_start = effectiveLaneChangeStartSec(spec);
    // 이 게이트의 예측 창은 차선변경이 실제로 진행되는 동안(lane_change_duration)
    // 으로 한정한다. post_merge_buffer 는 여기에 더하지 않는다.
    //
    // 다른 평가(risk/clearance)와 달리 여기는 '미래 한 순간이라도 위반하면
    // 불허'인 하드 게이트다. 그런데 예측은 자차가 ACC 목표속도로 계속
    // 달린다고 가정하고 앞차의 미래 감속은 모른다 - 멀리 볼수록 그 가정
    // 오차가 누적돼 실제로는 오지 않을 위반을 만들어낸다. 합류가 끝난 뒤의
    // 간격 유지는 ACC 가 맡는 영역이지, 변경 허가 시점에 미리 확정할 일이
    // 아니다.
    //   실측(2026-09-15 20:17 백, t=24.0): 양보영역이 비고(zone_ttc_relaxed=1)
    //   risk=1.000, clearance=inf, TTC 전부 통과인데 이 게이트만 0 이었다.
    //   전개해 보면 +4.2s 까지 통과하다 +5.0s(=3.5+1.0 창의 끝부분)에 간격이
    //   6.82m 로 7.0 을 밑돌아 실패했다. 그 결과 변경이 4.3 초 지연되고
    //   자차는 ACC 되먹임으로 10.8 -> 5.9 m/s 까지 끌려 내려갔다.
    // 예측 창 길이는 lane_change_duration 과 분리한다.
    //
    // lane_change_duration(3.5s)은 MPC 블렌딩 시간이라 곡률/승차감 기준으로
    // 정해지는 값인데, 그걸 그대로 위험 예측 창으로 쓰면 '경로를 완만하게
    // 하려고 3.5 로 올렸더니 위험 판정이 같이 길어져 변경이 더 보수적으로
    // 바뀌는' 결합이 생긴다. 두 값은 목적이 다르므로 따로 둔다.
    // projected_gap_ttc_horizon_sec > 0 이면 그 값을, 0 이하면 기존처럼
    // lane_change_duration 을 쓴다.
    const double projected_horizon_sec = projected_gap_ttc_horizon_sec_ > 1e-3
        ? projected_gap_ttc_horizon_sec_
        : std::max(0.1, spec.lane_change_duration_sec);
    const double safety_window_end = lane_change_start + projected_horizon_sec;
    const bool merge_accel_predicted = ego_traj.size() >= 2 &&
        ego_traj.back().v >= ego_traj.front().v +
            std::max(0.0, clear_zone_merge_accel_min_gain_mps_);
    // At low ego speed the wide distance box replaces a strict instantaneous
    // TTC gate only when the executable CHANGE_NOW trajectory actually gains
    // speed.  Otherwise a slow lateral merge in front of a rear vehicle still
    // fails closed.
    if (clear_zone && lowSpeedRearSafetyActive() && !merge_accel_predicted) {
      return false;
    }
    const double rear_safety_window_end =
        clear_zone && merge_accel_predicted
            ? lane_change_start + std::max(0.1, spec.lane_change_duration_sec) +
                  std::max(0.0, behavior_post_merge_buffer_sec_)
            : safety_window_end;
    const double front_required_gap = activeTargetFrontSafeGap(t);
    double rear_required_gap = activeTargetRearSafeGap(t);
    if (clear_zone && fixed_target_gap_gate_enabled_) {
      rear_required_gap = std::min(
          rear_required_gap,
          std::max(0.0, clear_zone_projected_rear_min_gap_m_));
    }
    const double front_required_ttc = activeTargetFrontMinTtc(t, clear_zone);
    const double rear_required_ttc = activeTargetRearMinTtc(t, clear_zone);
    const double rear_required_headway = fixed_target_gap_gate_enabled_
        ? activeTargetRearMinHeadway(t, clear_zone) : 0.0;
    // Do not apply a merge-priority relaxation in a future hard gate.  It is
    // safer to wait for a fresh CHANGE_NOW decision than to pre-authorize a
    // reduced rear gap several seconds ahead.
    projected_gap_rejection_="none";
    const auto check_vehicle = [&](const smpc_lane_change::TargetVehicle& vehicle,
                                   bool front) {
      const auto reject=[&](const char* why,double time,double gap,double required) {
        const auto estimate=gap_speed_policy_.get(vehicle,ros::Time::now().toSec());
        std::ostringstream detail;
        detail<<why<<",id="<<vehicle.unique_id<<",t="<<time<<",gap="<<gap<<",required="<<required
          <<",speed="<<(gap_shared_speed_enabled_ && estimate.known ? estimate.speed : vehicle.v_long)
          <<",source="<<(gap_shared_speed_enabled_ ? estimate.source : "legacy")
          <<",known="<<estimate.known;
        projected_gap_rejection_=detail.str();return false;
      };
      if (!vehicle.valid) return true;
      // 속도를 신뢰할 수 없을 때.
      //
      // 기존에는 즉시 false 였다. 그런데 LiDAR 트래커의 velocity_valid 는
      // 프레임마다 뒤집힌다 - 실측(2026-09-16 15:59 백, t>=14):
      //   target_front 통과 91%, target_rear 85%, 둘 다 78%.
      // 즉 22% 프레임에서 이 게이트가 강제로 0 이 되고,
      // confirmation_count(3 연속)를 요구하므로 확정이 사실상 불가능했다.
      // 그 구간의 실제 간격은 tf 15~27 m(요구 7~9.6), tr 11~21 m(요구
      // 10~17)로 2 배 이상 여유였고 zone_ttc_relaxed 도 계속 1 이었다.
      //
      // 속도를 모른다고 '위험하다'고 단정할 이유는 없다. 대신 최악을
      // 가정해서(상대속도가 자차 속도만큼 벌어질 수 있다고 보고) 검사하고,
      // 그래도 요구 간격을 지키면 통과시킨다. 간격이 빠듯하면 종전처럼
      // 거부되므로 안전 여유는 유지된다.
      const bool velocity_trusted = targetVelocityTrusted(vehicle) ||
          untrustedNearVehicle(vehicle, front);  // 가까우면 원시 속도로 아래 검사
      if (!velocity_trusted) {
        // The enlarged distance box is intentionally the primary gate when
        // lidar velocity is unreliable.  Do not recreate a noisy TTC veto
        // with an arbitrary worst-closing estimate after that box is clear.
        if (clear_zone) return true;
        if (!unknown_velocity_gap_fallback_enabled_) return false;
        const double gap_now = front ? frontBumperGap(vehicle)
                                     : rearBumperGap(vehicle);
        const double required = front ? front_required_gap : rear_required_gap;
        // 예측 창 동안 상대속도가 최대 이만큼 벌어질 수 있다고 본다.
        const double worst_closing =
            std::max(0.0, unknown_velocity_worst_closing_mps_);
        const double vehicle_window_end =
            front ? safety_window_end : rear_safety_window_end;
        const double window =
            std::max(0.0, vehicle_window_end - lane_change_start);
        const double worst_gap = gap_now - worst_closing * window;
        return worst_gap > required || reject("unknown_speed_gap",window,worst_gap,required);
      }
      // npc_longitudinal_event_modes_enabled_ 가 false 면 여기 오는 modes 는
      // nominal(등속) 하나뿐이다(modesForVehicle 주석 참고). 이 게이트는
      // 확률과 무관하게 모든 모드의 통과를 요구하므로, 이벤트 모드가 살아
      // 있으면 최악 시나리오 하나로 변경이 영구 차단된다.
      const auto modes = modesForVehicle(vehicle);
      const double vehicle_window_end =
          front ? safety_window_end : rear_safety_window_end;
      for (const auto& ego : ego_traj) {
        if (!ego.valid || ego.t + 1e-6 < lane_change_start) continue;
        if (ego.t > vehicle_window_end + 1e-6) break;
        for (const auto& mode : modes) {
          const PredPose tv = predictTargetPose(vehicle, mode, ego.t);
          if (!tv.valid) return reject("target_coverage",ego.t,0,0);

          const double ex = std::cos(ego.yaw);
          const double ey = std::sin(ego.yaw);
          const double longitudinal = (tv.x - ego.x) * ex + (tv.y - ego.y) * ey;
          const OrientedBox ego_box{
              ego.x, ego.y, ego.yaw, 0.5 * ego_length_m_, 0.5 * ego_width_m_};
          const OrientedBox tv_box{
              tv.x, tv.y, tv.yaw, 0.5 * std::max(0.1, vehicle.length),
              0.5 * std::max(0.1, vehicle.width)};
          const double ego_extent = axisExtent(ego_box, ex, ey);
          const double tv_extent = axisExtent(tv_box, ex, ey);
          const double tv_forward_speed =
              std::max(0.0, tv.v * std::cos(smpc_lane_change::wrapToPi(tv.yaw - ego.yaw)));

          if (front) {
            const double gap = longitudinal - ego_extent - tv_extent;
            if (!(gap > front_required_gap)) return reject("front_gap",ego.t,gap,front_required_gap);
            const double closing = std::max(0.0, ego.v - tv_forward_speed + cappedStdMargin(vehicle));
            if (closing > 0.1 &&
                gap / closing < std::max(0.0, front_required_ttc)) {
              return reject("front_ttc",ego.t,gap/closing,front_required_ttc);
            }
          } else {
            const double gap = -longitudinal - ego_extent - tv_extent;
            double required_gap = rear_required_gap;
            if (!clear_zone && low_speed_rear_safety_enabled_ &&
                ego.v <= std::max(0.0, low_speed_rear_ego_speed_mps_)) {
              required_gap = std::max(required_gap,
                                      std::max(0.0, low_speed_rear_safe_gap_m_));
            }
            if (!(gap > required_gap)) return reject("rear_gap",ego.t,gap,required_gap);
            const double closing = std::max(0.0, tv_forward_speed - ego.v + cappedStdMargin(vehicle));
            const double required_ttc = (!clear_zone &&
                                         low_speed_rear_safety_enabled_ &&
                                         ego.v <= std::max(0.0, low_speed_rear_ego_speed_mps_))
                ? std::max(rear_required_ttc, low_speed_rear_min_ttc_sec_)
                : rear_required_ttc;
            if (closing > 0.1 && gap / closing < std::max(0.0, required_ttc)) {
              return reject("rear_ttc",ego.t,gap/closing,required_ttc);
            }
            const double headway_reference_speed = std::max(ego.v, tv_forward_speed);
            if (gap < std::max(0.0, rear_required_headway) *
                          headway_reference_speed) {
              return reject("rear_headway",ego.t,gap,std::max(0.,rear_required_headway)*headway_reference_speed);
            }
          }
        }
      }
      return true;
    };

    return check_vehicle(t.target_front, true) &&
           check_vehicle(t.target_rear, false);
  }

  CandidateEval evaluateCandidate(
      const smpc_lane_change::TargetVehicleSet& t,
      const std::vector<smpc_lane_change::TargetVehicle>& vehicles,
      const ActionSpec& spec,
      double allowed_risk,
      double target_gap_cost) const {
    CandidateEval out;
    out.spec = spec;
    if (!candidateFitsHorizon(spec) || !candidateFitsLaneDeadline(t, spec)) return out;

    const double base_speed_cap_mps = egoPredictionTargetSpeedMps();
    out.longitudinal_model_compatible =
        preparationLongitudinalModelCompatible(t, spec, base_speed_cap_mps);
    if (!out.longitudinal_model_compatible) return out;

    const auto trajectory = buildEgoTrajectory(t, spec);
    const double safety_window_end = effectiveLaneChangeStartSec(spec) +
        std::max(0.1, spec.lane_change_duration_sec) +
        std::max(0.0, behavior_post_merge_buffer_sec_);
    out.trajectory = evaluateTrajectory(
        trajectory, vehicles, &t, spec.change_left, safety_window_end);
    out.target_speed_mps = actionTargetSpeedMps(base_speed_cap_mps, spec);
    out.time_risk_safe = timeIndexedRiskSafe(out.trajectory, allowed_risk);
    out.legacy_risk_safe = !behavior_require_legacy_risk_gate_ ||
        (out.trajectory.valid && out.trajectory.risk <= allowed_risk);
    out.clearance_safe = clearanceSafe(out.trajectory);
    out.projected_gap_ttc_safe =
        out.trajectory.valid && projectedTargetGapTtcSafe(t, trajectory, spec);
    out.feasible = out.time_risk_safe && out.legacy_risk_safe && out.clearance_safe &&
        out.projected_gap_ttc_safe;
    if (!out.trajectory.valid) return out;

    const double preparation_time = std::max(0.0, spec.lane_change_start_sec);
    out.cost = lane_change_base_cost_ + target_gap_cost +
        risk_cost_weight_ * out.trajectory.peak_step_risk +
        behavior_delay_cost_weight_ * preparation_time +
        behavior_accel_cost_weight_ * spec.prep_accel_mps2 * spec.prep_accel_mps2;
    return out;
  }

  bool behaviorCommandEnabled(const ActionSpec& spec) const {
    switch (spec.action) {
      case BehaviorAction::kWaitForGap:
      case BehaviorAction::kYieldDecelThenChange:
      case BehaviorAction::kHoldThenChange:
        return behavior_execute_decel_hold_;
      case BehaviorAction::kAccelThenChange:
        return behavior_execute_accel_;
      case BehaviorAction::kKeep:
      case BehaviorAction::kChangeNow:
      default:
        return false;
    }
  }

  void publishBehaviorOutputs(const smpc_lane_change::TargetVehicleSet& t,
                              const CandidateEval& selected,
                              bool now_feasible,
                              bool future_feasible,
                              double allowed_risk,
                              const std::string& reason) {
    const double reported_lane_change_start_sec = selected.spec.change_left
        ? effectiveLaneChangeStartSec(selected.spec)
        : -1.0;
    const bool longitudinal_command_enabled = behaviorCommandEnabled(selected.spec);

    // Keep one ID for a continuous preparation episode.  ACC and decision
    // timers are asynchronous, so incrementing this value every 50 ms would
    // turn a legitimate one-cycle status delay into a false cancellation.
    const bool same_behavior_contract =
        last_behavior_plan_id_ != 0 &&
        last_behavior_action_ == selected.spec.action &&
        last_behavior_current_lane_id_ == t.current_lane_id &&
        last_behavior_target_lane_id_ == t.target_lane_id;
    if (selected.spec.action == BehaviorAction::kYieldDecelThenChange) {
      if (!same_behavior_contract ||
          !std::isfinite(behavior_yield_episode_start_speed_mps_)) {
        behavior_yield_episode_start_speed_mps_ = std::max(0.0, ego_speed_mps_);
      }
    } else {
      behavior_yield_episode_start_speed_mps_ =
          std::numeric_limits<double>::quiet_NaN();
    }
    if (!same_behavior_contract) {
      ++behavior_plan_id_;
      if (behavior_plan_id_ == 0) ++behavior_plan_id_;
      last_behavior_plan_id_ = behavior_plan_id_;
      last_behavior_action_ = selected.spec.action;
      last_behavior_current_lane_id_ = t.current_lane_id;
      last_behavior_target_lane_id_ = t.target_lane_id;
    }

    // plan_id identifies the continuous preparation episode, while this
    // revision identifies the exact cap/timing command within that episode.
    // In particular, a new t_LC or speed cap must not be mistaken for the
    // older request when ACC reports its applied cap back to SMPC.
    constexpr double kCommandStartToleranceSec = 0.05;
    constexpr double kCommandSpeedCapToleranceMps = 0.10;
    const bool same_command_contract =
        last_behavior_command_revision_ != 0 &&
        last_behavior_command_plan_id_ == last_behavior_plan_id_ &&
        last_behavior_command_action_ == selected.spec.action &&
        last_behavior_command_active_ == longitudinal_command_enabled &&
        last_behavior_command_current_lane_id_ == t.current_lane_id &&
        last_behavior_command_target_lane_id_ == t.target_lane_id &&
        last_behavior_merge_leader_id_ == (gap_latch_valid_ ? gap_latch_leader_id_ : -1) &&
        last_behavior_merge_follower_id_ == (gap_latch_valid_ ? gap_latch_follower_id_ : -1) &&
        std::isfinite(last_behavior_command_lane_change_start_sec_) &&
        std::isfinite(last_behavior_command_speed_cap_mps_) &&
        std::abs(last_behavior_command_lane_change_start_sec_ -
                 reported_lane_change_start_sec) <= kCommandStartToleranceSec &&
        std::abs(last_behavior_command_speed_cap_mps_ - selected.target_speed_mps) <=
            kCommandSpeedCapToleranceMps;
    if (!same_command_contract) {
      previous_behavior_command_plan_id_ = last_behavior_command_plan_id_;
      previous_behavior_command_revision_ = last_behavior_command_revision_;
      previous_behavior_command_active_ = last_behavior_command_active_;
      previous_behavior_command_speed_cap_mps_ = last_behavior_command_speed_cap_mps_;
      ++behavior_command_revision_;
      if (behavior_command_revision_ == 0) ++behavior_command_revision_;
      last_behavior_command_plan_id_ = last_behavior_plan_id_;
      last_behavior_command_revision_ = behavior_command_revision_;
      last_behavior_command_action_ = selected.spec.action;
      last_behavior_command_active_ = longitudinal_command_enabled;
      last_behavior_command_current_lane_id_ = t.current_lane_id;
      last_behavior_command_target_lane_id_ = t.target_lane_id;
      last_behavior_command_lane_change_start_sec_ = reported_lane_change_start_sec;
      last_behavior_command_speed_cap_mps_ = selected.target_speed_mps;
      last_behavior_merge_leader_id_ = gap_latch_valid_ ? gap_latch_leader_id_ : -1;
      last_behavior_merge_follower_id_ = gap_latch_valid_ ? gap_latch_follower_id_ : -1;
    }

    smpc_lane_change::BehaviorPlan plan;
    plan.header = t.header;
    plan.header.stamp = ros::Time::now();
    plan.action = behaviorPlanActionMessageValue(selected.spec.action);
    plan.plan_id = last_behavior_plan_id_;
    plan.command_revision = last_behavior_command_revision_;
    plan.now_feasible = now_feasible;
    plan.future_feasible = future_feasible;
    plan.longitudinal_command_enabled = longitudinal_command_enabled;
    plan.current_lane_id = t.current_lane_id;
    plan.target_lane_id = t.target_lane_id;
    plan.preparation_accel_mps2 = selected.spec.prep_accel_mps2;
    plan.target_speed_mps = selected.target_speed_mps;
    // Expose the same physical transition start that the trajectory uses,
    // including supervisor/path-switch latency.  The raw candidate delay is
    // deliberately internal so consumers cannot treat t_LC as a reservation.
    plan.lane_change_start_sec = reported_lane_change_start_sec;
    plan.lane_change_duration_sec = selected.spec.lane_change_duration_sec;
    plan.horizon_sec = std::max(1, horizon_steps_) * prediction_dt_sec_;
    plan.selected_cost = selected.cost;
    plan.peak_risk = selected.trajectory.peak_step_risk;
    plan.legacy_risk = selected.trajectory.risk;
    plan.first_risk_violation_time_sec =
        firstRiskViolationTimeSec(selected.trajectory, allowed_risk);
    plan.min_signed_clearance_m = selected.trajectory.min_signed_clearance;
    plan.risk_by_step = selected.trajectory.risk_by_step;
    plan.reason = reason;
    plan.merge_gap_selected = gap_latch_valid_ &&
        gap_latch_current_lane_id_ == t.current_lane_id && gap_latch_target_lane_id_ == t.target_lane_id;
    plan.merge_leader_id = plan.merge_gap_selected ? gap_latch_leader_id_ : -1;
    plan.merge_follower_id = plan.merge_gap_selected ? gap_latch_follower_id_ : -1;
    behavior_plan_pub_.publish(plan);

    smpc_lane_change::BehaviorLongitudinalRequest request;
    request.header = plan.header;
    request.action = behaviorRequestActionMessageValue(selected.spec.action);
    request.plan_id = last_behavior_plan_id_;
    request.command_revision = last_behavior_command_revision_;
    request.active = plan.longitudinal_command_enabled;
    request.current_lane_id = t.current_lane_id;
    request.target_lane_id = t.target_lane_id;
    request.speed_cap_mps = selected.target_speed_mps;
    request.requested_accel_mps2 = selected.spec.prep_accel_mps2;
    request.planned_lc_start_sec = reported_lane_change_start_sec;
    request.candidate_cost = selected.cost;
    request.peak_risk = selected.trajectory.peak_step_risk;
    request.reason = reason;
    request.merge_gap_selected = plan.merge_gap_selected;
    request.merge_leader_id = plan.merge_leader_id;
    request.merge_follower_id = plan.merge_follower_id;
    request.merge_leader.valid = false;
    const auto pick_merge_leader = [&](const smpc_lane_change::TargetVehicle& v) {
      if (v.valid && v.unique_id == request.merge_leader_id && v.lane_id == t.target_lane_id)
        request.merge_leader = v;
    };
    if (request.merge_gap_selected && request.merge_leader_id >= 0) {
      pick_merge_leader(t.target_front); pick_merge_leader(t.target_rear);
      for (const auto& v : t.nearby_vehicles) pick_merge_leader(v);
    }
    behavior_request_pub_.publish(request);
  }

  void publishKeep(const smpc_lane_change::TargetVehicleSet& t,
                   const std::string& reason,
                   double keep_cost,
                   double change_cost,
                   double collision_risk) {
    smpc_lane_change::LaneChangeDecision d;
    d.header = t.header;
    d.mode = smpc_lane_change::LaneChangeDecision::KEEP;
    d.request = false;
    d.current_lane_id = t.current_lane_id;
    d.target_lane_id = t.target_lane_id;
    d.target_speed_mps = currentTargetSpeedMps();
    d.keep_cost = keep_cost;
    d.change_cost = change_cost;
    d.collision_risk = collision_risk;
    d.reason = reason;
    decision_pub_.publish(d);
  }

  void publishChange(const smpc_lane_change::TargetVehicleSet& t,
                     const std::string& reason,
                     double keep_cost,
                     double change_cost,
                     double collision_risk) {
    smpc_lane_change::LaneChangeDecision d;
    d.header = t.header;
    d.mode = smpc_lane_change::LaneChangeDecision::CHANGE_LEFT;
    d.request = true;
    d.current_lane_id = t.current_lane_id;
    d.target_lane_id = t.target_lane_id;
    d.target_speed_mps = currentTargetSpeedMps();
    d.keep_cost = keep_cost;
    d.change_cost = change_cost;
    d.collision_risk = collision_risk;
    d.reason = reason;
    decision_pub_.publish(d);
  }

  void publishDebug(const std::string& text) {
    std_msgs::String msg;
    msg.data = text;
    debug_pub_.publish(msg);
    if (debug_publish_marker_) {
      publishDebugMarker(text);
    }
  }

  std::string wrapDebugText(const std::string& text) const {
    constexpr std::size_t kMaxLine = 72;
    std::istringstream input(text);
    std::ostringstream wrapped;
    std::string word;
    std::size_t line_len = 0;
    while (input >> word) {
      if (line_len > 0 && line_len + 1 + word.size() > kMaxLine) {
        wrapped << '\n';
        line_len = 0;
      } else if (line_len > 0) {
        wrapped << ' ';
        ++line_len;
      }
      wrapped << word;
      line_len += word.size();
    }
    return wrapped.str();
  }

  void publishDebugMarker(const std::string& text) const {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "smpc_decision_debug";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = ego_x_;
    marker.pose.position.y = ego_y_;
    marker.pose.position.z = debug_marker_height_m_;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.9;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 0.95;
    marker.lifetime = ros::Duration(std::max(0.0, debug_marker_lifetime_sec_));
    marker.text = wrapDebugText(text);
    debug_marker_pub_.publish(marker);
  }

  void publishYieldZoneMarkers(const smpc_lane_change::TargetVehicleSet& t,
                               bool side_veh_in_yield_zone,
                               bool change_feasible,
                               bool change_confirmed,
                               double yield_front_window,
                               double yield_rear_window) const {
    if (!debug_yield_zone_publish_markers_) return;
    visualization_msgs::MarkerArray marker_array;

    const auto* target_lane = laneFor(t.target_lane_id);
    if (!target_lane || target_lane->empty() ||
        t.target_lane_id == t.current_lane_id || !t.lane_change_required) {
      visualization_msgs::Marker del;
      del.header.frame_id = "map";
      del.header.stamp = ros::Time::now();
      del.action = visualization_msgs::Marker::DELETEALL;
      marker_array.markers.push_back(del);
      yield_zone_marker_pub_.publish(marker_array);
      return;
    }

    const auto proj = target_lane->project(ego_x_, ego_y_);
    if (!proj.valid) return;

    // Yield zone: front_window ahead of ego front bumper, rear_window behind ego rear bumper
    const double s_front = proj.s + ego_length_m_ * 0.5 + yield_front_window;
    const double s_rear = proj.s - (ego_length_m_ * 0.5 + yield_rear_window);
    const double s_center = (s_front + s_rear) * 0.5;
    const double total_length = std::max(1.0, s_front - s_rear);
    const auto center_sample = target_lane->sample(s_center);
    if (!center_sample.valid) return;

    const ros::Time now = ros::Time::now();
    const ros::Duration lifetime(0.3);

    const bool preview_not_ready =
        !referencePreviewFreshFor(t) &&
        ego_reference_preview_required_for_change_ &&
        !ego_reference_preview_allow_fallback_;

    // Color code. Keep this in sync with the status text below so that an
    // empty geometric zone is not mistaken for an approved lane change.
    // Red:     a vehicle occupies the immediate yield zone (BLOCKED)
    // Orange:  executable path preview is unavailable (PATH NOT READY)
    // Yellow:  path exists, but gap/TTC/risk checks are unsafe (WAIT SAFE GAP)
    // Cyan:    safe now, waiting for consecutive-frame confirmation
    // Green:   all checks confirmed (LC READY)
    float r = 0.1f, g = 1.0f, b = 0.2f, a = 0.35f;
    if (side_veh_in_yield_zone) {
      r = 1.0f; g = 0.1f; b = 0.1f; a = 0.45f;
    } else if (preview_not_ready) {
      r = 1.0f; g = 0.35f; b = 0.0f; a = 0.42f;
    } else if (!change_feasible) {
      r = 1.0f; g = 0.8f; b = 0.1f; a = 0.35f;
    } else if (!change_confirmed) {
      r = 0.0f; g = 0.85f; b = 1.0f; a = 0.38f;
    }

    // 1. Semi-transparent box on target lane
    visualization_msgs::Marker box_marker;
    box_marker.header.frame_id = "map";
    box_marker.header.stamp = now;
    box_marker.ns = "yield_zone_box";
    box_marker.id = 0;
    box_marker.type = visualization_msgs::Marker::CUBE;
    box_marker.action = visualization_msgs::Marker::ADD;
    box_marker.pose.position.x = center_sample.x;
    box_marker.pose.position.y = center_sample.y;
    box_marker.pose.position.z = 0.2;
    const double half_yaw = center_sample.yaw * 0.5;
    box_marker.pose.orientation.z = std::sin(half_yaw);
    box_marker.pose.orientation.w = std::cos(half_yaw);
    box_marker.scale.x = total_length;
    box_marker.scale.y = lane_width_m_;
    box_marker.scale.z = 0.15;
    box_marker.color.r = r;
    box_marker.color.g = g;
    box_marker.color.b = b;
    box_marker.color.a = a;
    box_marker.lifetime = lifetime;
    marker_array.markers.push_back(box_marker);

    // 2. Boundary border line strip
    visualization_msgs::Marker border_marker;
    border_marker.header.frame_id = "map";
    border_marker.header.stamp = now;
    border_marker.ns = "yield_zone_border";
    border_marker.id = 1;
    border_marker.type = visualization_msgs::Marker::LINE_STRIP;
    border_marker.action = visualization_msgs::Marker::ADD;
    border_marker.pose.orientation.w = 1.0;
    border_marker.scale.x = 0.18;
    border_marker.color.r = r;
    border_marker.color.g = g;
    border_marker.color.b = b;
    border_marker.color.a = 0.9f;
    border_marker.lifetime = lifetime;

    const double half_w = lane_width_m_ * 0.5;
    const auto p_front_left = target_lane->sample(s_front, half_w);
    const auto p_front_right = target_lane->sample(s_front, -half_w);
    const auto p_rear_right = target_lane->sample(s_rear, -half_w);
    const auto p_rear_left = target_lane->sample(s_rear, half_w);

    auto to_pt = [](const smpc_lane_change::LaneSample& s) {
      geometry_msgs::Point pt;
      pt.x = s.x; pt.y = s.y; pt.z = 0.25;
      return pt;
    };
    if (p_front_left.valid && p_front_right.valid && p_rear_right.valid && p_rear_left.valid) {
      border_marker.points.push_back(to_pt(p_front_left));
      border_marker.points.push_back(to_pt(p_front_right));
      border_marker.points.push_back(to_pt(p_rear_right));
      border_marker.points.push_back(to_pt(p_rear_left));
      border_marker.points.push_back(to_pt(p_front_left));
      marker_array.markers.push_back(border_marker);
    }

    // 3. Status text marker
    visualization_msgs::Marker text_marker;
    text_marker.header.frame_id = "map";
    text_marker.header.stamp = now;
    text_marker.ns = "yield_zone_text";
    text_marker.id = 2;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.pose.position.x = center_sample.x;
    text_marker.pose.position.y = center_sample.y;
    text_marker.pose.position.z = 2.8;
    text_marker.pose.orientation.w = 1.0;
    text_marker.scale.z = 0.9;
    text_marker.color.r = 1.0f;
    text_marker.color.g = 1.0f;
    text_marker.color.b = 1.0f;
    text_marker.color.a = 0.95f;
    text_marker.lifetime = lifetime;

    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss.precision(1);
    ss << "Safe Zone [Front +" << yield_front_window << "m / Rear -" << yield_rear_window << "m]\n";
    if (side_veh_in_yield_zone) {
      ss << "[BLOCKED] Side Veh in Zone -> YIELD DECEL (TTC/Headway)";
    } else if (change_confirmed) {
      ss << "[CLEAR] Confirmed (" << change_feasible_streak_ << "/" << change_feasible_confirm_count_ << ") -> LC READY";
    } else if (change_feasible) {
      ss << "[CONFIRMING] Hysteresis (" << change_feasible_streak_ << "/" << change_feasible_confirm_count_ << ")";
    } else {
      if (preview_not_ready) {
        ss << "[WAIT] Path Preview Not Ready";
      } else {
        ss << "[WAIT] Waiting Safe Gap";
      }
    }
    text_marker.text = ss.str();
    marker_array.markers.push_back(text_marker);

    yield_zone_marker_pub_.publish(marker_array);
  }

  // ===== 틈 선택 모드 시각화 (/smpc/yield_zone_markers) =====
  //   틈 자리: 선택=초록(진함), 가능=하늘색, 불가능=회색. 열린 틈은 40 m 까지만 그림.
  //   목표 차로 차량: 선택 틈의 뒤차=주황, 앞차=보라, 나머지=흰색 (+ id, 속도).
  //   자차 목표 위치: 노란 기둥 + 자차 위치에서의 화살표. 상태 글자는 자차 위.
  void appendLaneStrip(visualization_msgs::MarkerArray& arr, const smpc_lane_change::LanePath& lane,
                       const std::string& ns, int id, double s_a, double s_b, double half_w,
                       float r, float g, float b, float fill_a, float line_a, double line_w,
                       double z) const {
    if (s_b < s_a) std::swap(s_a, s_b);
    if (s_b - s_a < 0.1) return;
    const ros::Time now = ros::Time::now();
    const ros::Duration lifetime(0.3);
    const int n = std::max(1, static_cast<int>(std::ceil((s_b - s_a) / 2.0)));
    std::vector<smpc_lane_change::LaneSample> left, right;
    for (int i = 0; i <= n; ++i) {
      const double si = s_a + (s_b - s_a) * i / n;
      const auto l = lane.sample(si, half_w);
      const auto r2 = lane.sample(si, -half_w);
      if (!l.valid || !r2.valid) return;
      left.push_back(l);
      right.push_back(r2);
    }
    const auto pt = [z](const smpc_lane_change::LaneSample& q) {
      geometry_msgs::Point p; p.x = q.x; p.y = q.y; p.z = z; return p;
    };
    visualization_msgs::Marker fill;
    fill.header.frame_id = "map";
    fill.header.stamp = now;
    fill.ns = ns + "_fill";
    fill.id = id;
    fill.type = visualization_msgs::Marker::TRIANGLE_LIST;
    fill.action = visualization_msgs::Marker::ADD;
    fill.pose.orientation.w = 1.0;
    fill.scale.x = fill.scale.y = fill.scale.z = 1.0;
    fill.color.r = r; fill.color.g = g; fill.color.b = b; fill.color.a = fill_a;
    fill.lifetime = lifetime;
    for (int i = 0; i < n; ++i) {
      fill.points.push_back(pt(left[i]));
      fill.points.push_back(pt(right[i]));
      fill.points.push_back(pt(left[i + 1]));
      fill.points.push_back(pt(right[i]));
      fill.points.push_back(pt(right[i + 1]));
      fill.points.push_back(pt(left[i + 1]));
    }
    arr.markers.push_back(fill);
    visualization_msgs::Marker border = fill;
    border.ns = ns + "_border";
    border.type = visualization_msgs::Marker::LINE_STRIP;
    border.points.clear();
    border.scale.x = line_w;
    border.scale.y = border.scale.z = 0.0;
    border.color.a = line_a;
    for (const auto& q : left) border.points.push_back(pt(q));
    for (auto it = right.rbegin(); it != right.rend(); ++it) border.points.push_back(pt(*it));
    border.points.push_back(pt(left.front()));
    border.points.back().z = z + 0.01;
    arr.markers.push_back(border);
  }

  void publishGapSelectMarkers(const smpc_lane_change::TargetVehicleSet& t,
                               const std::vector<MergeGap>& gaps,
                               const std::vector<GapVehicle>& vehicles,
                               bool gap_selected, bool gap_hold_last,
                               const MergeGap& selected,
                               double speed_cmd, bool change_feasible,
                               bool change_confirmed) const {
    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker del;
    del.header.frame_id = "map";
    del.header.stamp = ros::Time::now();
    del.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(del);
    const auto* lane = laneFor(t.target_lane_id);
    if (!lane || lane->empty()) { yield_zone_marker_pub_.publish(arr); return; }
    const auto proj = lane->project(ego_x_, ego_y_);
    if (!proj.valid) { yield_zone_marker_pub_.publish(arr); return; }
    const double s0 = proj.s;   // 자차 중심의 목표 차로 s (틈 좌표의 0)
    const double half_w = 0.5 * lane_width_m_;
    const double open_len = 40.0;
    const ros::Time now = ros::Time::now();
    const ros::Duration lifetime(0.3);

    int id = 0;
    for (const auto& g : gaps) {
      const bool sel = gap_selected && g.sameKey(selected.follower_id, selected.leader_id);
      double lo = std::isfinite(g.lo0) ? g.lo0 : g.hi0 - open_len;
      double hi = std::isfinite(g.hi0) ? g.hi0 : g.lo0 + open_len;
      if (hi <= lo) continue;
      if (sel) {
        appendLaneStrip(arr, *lane, "gap_selected", id++, s0 + lo, s0 + hi, half_w * 0.9,
                        0.1f, 1.0f, 0.3f, 0.35f, 1.0f, 0.3, 0.22);
      } else if (g.feasible) {
        appendLaneStrip(arr, *lane, "gap_feasible", id++, s0 + lo, s0 + hi, half_w * 0.8,
                        0.0f, 0.8f, 1.0f, 0.12f, 0.7f, 0.1, 0.18);
      } else {
        appendLaneStrip(arr, *lane, "gap_infeasible", id++, s0 + lo, s0 + hi, half_w * 0.8,
                        0.6f, 0.6f, 0.6f, 0.08f, 0.5f, 0.06, 0.16);
      }
    }

    int vid = 0;
    for (const auto& v : vehicles) {
      const double sc = s0 + 0.5 * (v.rear_edge + v.front_edge);
      const auto c = lane->sample(sc);
      if (!c.valid) continue;
      float r = 0.9f, g = 0.9f, b = 0.9f;
      if (gap_selected && v.id >= 0 && v.id == selected.follower_id) { r = 1.0f; g = 0.55f; b = 0.0f; }
      if (gap_selected && v.id >= 0 && v.id == selected.leader_id) { r = 0.75f; g = 0.2f; b = 1.0f; }
      visualization_msgs::Marker box;
      box.header.frame_id = "map";
      box.header.stamp = now;
      box.ns = "gap_vehicle";
      box.id = vid;
      box.type = visualization_msgs::Marker::CUBE;
      box.action = visualization_msgs::Marker::ADD;
      box.pose.position.x = c.x;
      box.pose.position.y = c.y;
      box.pose.position.z = 0.8;
      box.pose.orientation.z = std::sin(0.5 * c.yaw);
      box.pose.orientation.w = std::cos(0.5 * c.yaw);
      box.scale.x = std::max(1.0, v.front_edge - v.rear_edge);
      box.scale.y = 1.9;
      box.scale.z = 1.5;
      box.color.r = r; box.color.g = g; box.color.b = b; box.color.a = 0.8f;
      box.lifetime = lifetime;
      arr.markers.push_back(box);
      visualization_msgs::Marker txt = box;
      txt.ns = "gap_vehicle_text";
      txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      txt.pose.position.z = 2.3;
      txt.pose.orientation.z = 0.0;
      txt.pose.orientation.w = 1.0;
      txt.scale.x = txt.scale.y = 0.0;
      txt.scale.z = 0.8;
      txt.color.a = 1.0f;
      std::ostringstream vs;
      vs.setf(std::ios::fixed);
      vs.precision(1);
      vs << "id" << v.id << " " << v.v << "m/s";
      txt.text = vs.str();
      arr.markers.push_back(txt);
      ++vid;
    }

    const auto ego_c = lane->sample(s0);
    if (gap_selected && ego_c.valid) {
      const double target_rel = gapTargetPos(selected, 0.0, 0.0);
      const auto tc = lane->sample(s0 + target_rel);
      if (tc.valid) {
        visualization_msgs::Marker cyl;
        cyl.header.frame_id = "map";
        cyl.header.stamp = now;
        cyl.ns = "gap_target";
        cyl.id = 0;
        cyl.type = visualization_msgs::Marker::CYLINDER;
        cyl.action = visualization_msgs::Marker::ADD;
        cyl.pose.position.x = tc.x;
        cyl.pose.position.y = tc.y;
        cyl.pose.position.z = 1.25;
        cyl.pose.orientation.w = 1.0;
        cyl.scale.x = cyl.scale.y = 0.8;
        cyl.scale.z = 2.5;
        cyl.color.r = 1.0f; cyl.color.g = 0.95f; cyl.color.b = 0.0f; cyl.color.a = 0.9f;
        cyl.lifetime = lifetime;
        arr.markers.push_back(cyl);
        if (std::abs(target_rel) > 0.5) {
          visualization_msgs::Marker arrow = cyl;
          arrow.ns = "gap_target_arrow";
          arrow.type = visualization_msgs::Marker::ARROW;
          arrow.pose = geometry_msgs::Pose();
          arrow.pose.orientation.w = 1.0;
          arrow.scale.x = 0.25;
          arrow.scale.y = 0.6;
          arrow.scale.z = 0.8;
          geometry_msgs::Point a, bpt;
          a.x = ego_c.x; a.y = ego_c.y; a.z = 1.0;
          bpt.x = tc.x; bpt.y = tc.y; bpt.z = 1.0;
          arrow.points = {a, bpt};
          arr.markers.push_back(arrow);
        }
      }
    }

    if (ego_c.valid) {
      visualization_msgs::Marker txt;
      txt.header.frame_id = "map";
      txt.header.stamp = now;
      txt.ns = "gap_status";
      txt.id = 0;
      txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      txt.action = visualization_msgs::Marker::ADD;
      txt.pose.position.x = ego_c.x;
      txt.pose.position.y = ego_c.y;
      txt.pose.position.z = 3.6;
      txt.pose.orientation.w = 1.0;
      txt.scale.z = 0.9;
      txt.color.r = txt.color.g = txt.color.b = 1.0f;
      txt.color.a = 0.95f;
      txt.lifetime = lifetime;
      int feasible = 0;
      for (const auto& g : gaps) feasible += g.feasible ? 1 : 0;
      std::ostringstream ss;
      ss.setf(std::ios::fixed);
      ss.precision(1);
      ss << "GAP SELECT ";
      if (!gap_selected) {
        ss << (gap_hold_last ? "[HOLD LAST CMD]"
               : feasible > 0 ? "[SELECTING]" : "[NO FEASIBLE GAP -> fallback]");
      } else {
        ss << (selected.follower_id >= 0 ? std::to_string(selected.follower_id) : std::string("open"))
           << " / "
           << (selected.leader_id >= 0 ? std::to_string(selected.leader_id) : std::string("open"))
           << (selected.aligned_now ? "  [ALIGNED]" : "  [ALIGNING ")
           ;
        if (!selected.aligned_now) ss << selected.align_time << "s]";
        if (!selected.feasible) ss << " BEST EFFORT +" << selected.overshoot_m << "m";
      }
      if (gap_selected || gap_hold_last) {
        ss << "\ncmd " << speed_cmd << " m/s (ego " << ego_speed_mps_ << ")"
           << (speed_cmd < ego_speed_mps_ - 0.3 ? " DECEL"
               : speed_cmd > ego_speed_mps_ + 0.3 ? " ACCEL" : " HOLD");
      } else {
        ss << "\nego " << ego_speed_mps_ << " m/s";
      }
      ss << "  gaps " << feasible << "/" << gaps.size();
      if (gap_rule_vehicle_id_ >= 0) {
        ss << "\nref id" << gap_rule_vehicle_id_ << " " << gap_rule_vehicle_speed_ << "m/s -> "
           << (gap_rule_side_ > 0 ? "AHEAD (overtake)" : gap_rule_side_ < 0 ? "BEHIND (let pass)"
                                  : "by cost")
           << " [" << gap_select_side_policy_ << "]";
      }
      ss << "\nLC gate: "
         << (change_confirmed ? "READY" : change_feasible ? "CONFIRMING" : "WAIT");
      txt.text = ss.str();
      arr.markers.push_back(txt);
    }
    yield_zone_marker_pub_.publish(arr);
  }

  void timerCallback(const ros::TimerEvent&) {
    if (!enabled_ || !have_ego_) return;

    const bool fresh_targets = have_targets_ &&
        (ros::Time::now() - targets_stamp_).toSec() <= target_timeout_sec_;
    if (!fresh_targets) return;

    const auto& t = targets_;
    ActionSpec keep_spec;
    keep_spec.action = BehaviorAction::kKeep;
    keep_spec.name = "keep";
    CandidateEval keep_candidate;
    keep_candidate.spec = keep_spec;
    keep_candidate.target_speed_mps = egoPredictionTargetSpeedMps();
    keep_candidate.cost = 0.0;

    if (!lanes_loaded_) {
      publishBehaviorOutputs(t, keep_candidate, false, false, 1.0,
                             "smpc_v3_no_lane_csv_keep");
      publishKeep(t, "smpc_v3_no_lane_csv_keep", 0.0, lane_end_emergency_cost_, 1.0);
      publishDebug("v3 disabled: lane CSVs not loaded");
      return;
    }

    const auto vehicles = uniqueVehicles(t);
    ActionSpec change_now_spec;
    change_now_spec.action = BehaviorAction::kChangeNow;
    change_now_spec.lane_change_start_sec = 0.0;
    change_now_spec.lane_change_duration_sec = egoChangeBlendDuration();
    change_now_spec.change_left = true;
    change_now_spec.name = "change_now";

    const auto keep_traj = buildEgoTrajectory(t, keep_spec);
    const auto change_traj = buildEgoTrajectory(t, change_now_spec);
    publishPredictionPath(keep_traj, keep_path_pub_, t.header);
    publishPredictionPath(change_traj, change_path_pub_, t.header);
    publishPredictionMarkers(t, keep_traj, change_traj, vehicles);

    const TrajectoryEval keep_eval = evaluateTrajectory(keep_traj, vehicles, &t, false);
    const double change_safety_window_end =
        effectiveLaneChangeStartSec(change_now_spec) +
        std::max(0.1, change_now_spec.lane_change_duration_sec) +
        std::max(0.0, behavior_post_merge_buffer_sec_);
    const TrajectoryEval change_eval = evaluateTrajectory(
        change_traj, vehicles, &t, true, change_safety_window_end);
    keep_candidate.trajectory = keep_eval;

    const double current_safe_gap = std::max(
        current_front_safe_gap_m_, standstill_gap_m_ + time_headway_sec_ * ego_speed_mps_);
    const double current_gap = t.current_front.valid
        ? frontBumperGap(t.current_front)
        : std::numeric_limits<double>::infinity();
    const double target_front_gap = t.target_front.valid
        ? frontBumperGap(t.target_front)
        : std::numeric_limits<double>::infinity();
    const double target_rear_gap = t.target_rear.valid
        ? rearBumperGap(t.target_rear)
        : std::numeric_limits<double>::infinity();

    const bool current_front_priority =
        egoLanePriorityEligible(t, t.current_front, false);
    const double effective_current_gap = current_front_priority
        ? std::numeric_limits<double>::infinity()
        : current_gap;
    const bool current_blocked = t.current_front.valid &&
        !current_front_priority &&
        current_gap < current_front_trigger_gap_m_;
    const bool endpoint_trigger =
        t.lane_change_required || t.lane_change_prepare || t.lane_change_urgent || t.emergency_stop_required;
    const double allowed_risk = activeRiskEpsilon(t);
    const double active_target_front_safe_gap = activeTargetFrontSafeGap(t);
    const double active_target_rear_safe_gap = activeTargetRearSafeGap(t);
    const double effective_target_rear_safe_gap = fixed_target_gap_gate_enabled_
        ? effectiveTargetRearSafeGap(t, t.target_rear, active_target_rear_safe_gap)
        : active_target_rear_safe_gap;
    const bool target_rear_priority =
        t.target_rear.valid && effective_target_rear_safe_gap < active_target_rear_safe_gap;

    const bool target_lane_is_left = t.target_lane_id == t.current_lane_id + 1;
    double side_veh_speed = 0.0;
    double side_veh_delta_s = 0.0;
    double side_veh_bumper_gap = 0.0;
    bool rear_vehicle_in_yield_zone = false;
    const bool use_fixed_safe_gaps = side_vehicle_yield_use_safe_gaps_ &&
                                     fixed_target_gap_gate_enabled_;
    double yield_front_window = use_fixed_safe_gaps
        ? active_target_front_safe_gap : side_vehicle_yield_front_window_m_;
    double yield_rear_window = use_fixed_safe_gaps
        ? effective_target_rear_safe_gap : side_vehicle_yield_rear_window_m_;
    if (side_vehicle_yield_dynamic_window_enabled_ &&
        !use_fixed_safe_gaps) {
      // Target velocity is intentionally excluded: lidar velocity estimates
      // are least reliable at long range, so the box is sized by ego speed
      // alone.
      //
      // 2026-09-18: 방향을 뒤집었다. 이전에는 저속에서 영역을 키웠는데,
      // 실제로 위험한 쪽은 반대다. 고속일수록 같은 시간에 더 많이 좁혀지고
      // 제동거리도 길어지므로 영역이 커져야 하고, 저속에서는 상대적으로
      // 여유가 있어 영역을 줄여야 합류가 막히지 않는다.
      //   wide (저속 기준속도) 이하 -> 최소 영역
      //   narrow(고속 기준속도) 이상 -> 최대 영역
      // 파라미터 이름은 호환을 위해 유지하되, 이제 wide_mps 가 '좁아지는
      // 하한 속도', narrow_mps 가 '넓어지는 상한 속도'를 뜻한다.
      yield_front_window = yieldZoneWindow(true);
      yield_rear_window = yieldZoneWindow(false);
    }
    int side_veh_id = -1;
    const bool side_veh_in_yield_zone = target_lane_is_left &&
        hasAdjacentSideVehicleInTargetLane(t, yield_front_window, yield_rear_window,
                                           &side_veh_speed, &side_veh_delta_s,
                                           &rear_vehicle_in_yield_zone,
                                           &side_veh_bumper_gap, &side_veh_id);
    // The ego-speed-scaled occupancy box is the driver's primary visual gate.
    // Once it is clear, use a less conservative TTC/headway threshold
    // while retaining executable-path, minimum-gap and overlap guards.
    const bool clear_zone_ttc_relaxed =
        clear_zone_ttc_relax_enabled_ && !side_veh_in_yield_zone;
    const bool target_front_ttc_safe =
        targetFrontTtcSafe(t, t.target_front, clear_zone_ttc_relaxed);
    const bool target_rear_ttc_safe =
        targetRearTtcSafe(t, t.target_rear, clear_zone_ttc_relaxed);
    const bool target_rear_headway_safe =
        !fixed_target_gap_gate_enabled_ ||
        targetRearHeadwaySafe(t, t.target_rear, clear_zone_ttc_relaxed);
    // 점유영역이 비었으면 risk 문턱도 함께 완화한다(위 clear_zone_risk_epsilon
    // 주석 참고). 긴급/비상 상황에서 이미 더 큰 값을 쓰고 있으면 그쪽을 둔다.
    const double effective_allowed_risk =
        (clear_zone_ttc_relaxed && clear_zone_ttc_relax_enabled_)
            ? std::max(allowed_risk, clear_zone_risk_epsilon_)
            : allowed_risk;
    // The highway route has a fixed end and every left change is mandatory.
    // When a target-lane vehicle is behind us inside the exclusion zone, do
    // not spend the remaining route trying to pull ahead. Give it the slot and
    // prepare to merge behind it.
    // 뒤차가 실제로 나를 따라잡고 있을 때만 양보한다. 나보다 느린 차에
    // 양보하면 가만히 있어도 벌어질 간격을 만들려고 감속하게 되고, 그 차는
    // 영영 오지 않아 교착에 빠진다.
    //   실측 2026-09-14(19:26 백): 목표 차로 뒤차 id2 가 11.0 m/s 로 자차
    //   15.4 m/s 보다 4.1 m/s 느린데 yield_rear_vehicle_pass 가 걸려
    //   자차가 15.4 -> 0 m/s 로 완전 정지했다. 간격은 이미 -6.7 -> -21.7 m
    //   로 벌어지고 있던 상황이라 양보할 이유가 없었다.
    const double rear_closing_mps =
        t.target_rear.valid ? (leadTrustedLongSpeed(t.target_rear) - ego_speed_mps_) : 0.0;
    const bool rear_vehicle_actually_closing =
        !t.target_rear.valid ||
        rear_closing_mps > side_vehicle_rear_yield_min_closing_mps_;
    if (side_vehicle_yield_commit_current_lane_id_ != t.current_lane_id ||
        side_vehicle_yield_commit_target_lane_id_ != t.target_lane_id) {
      side_vehicle_yield_committed_id_ = -1;
      side_vehicle_yield_commit_stamp_ = ros::Time();
    }
    side_vehicle_yield_commit_current_lane_id_ = t.current_lane_id;
    side_vehicle_yield_commit_target_lane_id_ = t.target_lane_id;
    if (side_vehicle_yield_commit_enabled_ && side_veh_in_yield_zone &&
        side_veh_id >= 0 && side_veh_delta_s >= 0.0 &&
        side_vehicle_yield_committed_id_ < 0) {
      // Once yielding starts for a vehicle beside/in front of us, keep the
      // same maneuver when projection changes its role to target_rear.  The
      // 19:51 bag otherwise changed -2.5 to -1.0 m/s^2 after one cycle.
      side_vehicle_yield_committed_id_ = side_veh_id;
      side_vehicle_yield_commit_stamp_ = ros::Time::now();
    }
    bool side_vehicle_yield_commit_active =
        side_vehicle_yield_commit_enabled_ && side_veh_in_yield_zone &&
        side_veh_id >= 0 && side_veh_id == side_vehicle_yield_committed_id_;
    if (side_vehicle_yield_commit_active &&
        side_vehicle_yield_commit_max_duration_sec_ > 0.0 &&
        (ros::Time::now() - side_vehicle_yield_commit_stamp_).toSec() >
            side_vehicle_yield_commit_max_duration_sec_) {
      side_vehicle_yield_commit_active = false;
    }
    if (!side_vehicle_yield_commit_active &&
        (!side_veh_in_yield_zone || side_veh_id != side_vehicle_yield_committed_id_)) {
      side_vehicle_yield_committed_id_ = -1;
      side_vehicle_yield_commit_stamp_ = ros::Time();
    }
    const bool rear_side_vehicle_requires_yield_base =
        side_veh_in_yield_zone && rear_vehicle_in_yield_zone &&
        (rear_vehicle_actually_closing || side_vehicle_yield_commit_active);

    // 느린 뒤차 앞서가기 조건.
    //
    // 양보영역을 점유한 차량이 자차보다 뒤에 있고(delta_s < 0) 자차보다
    // 느리면, 양보 감속은 교착만 만든다. 이때는 감속하지 않고 그대로
    // 앞서간 뒤 그 차 앞으로 합류한다.
    //
    // 단, '앞이 충분히 비어 있을 때만' 허용한다(사용자 요구). 앞쪽
    // 양보영역(yield_front_window) 안에 차가 있으면 앞서갈 공간이 없다.
    // target_front 가 그 창 밖이거나 아예 없어야 한다.
    const double slow_rear_front_clearance =
        t.target_front.valid ? frontBumperGap(t.target_front)
                             : std::numeric_limits<double>::infinity();
    const bool slow_rear_front_clear =
        slow_rear_front_clearance >= yield_front_window;
    const bool slow_rear_overtake_raw =
        slow_rear_overtake_enabled_ &&
        side_veh_in_yield_zone &&
        rear_vehicle_in_yield_zone &&
        t.target_rear.valid && t.target_rear.velocity_valid &&
        std::isfinite(t.target_rear.v_long) &&
        std::isfinite(t.target_rear.velocity_std) &&
        t.target_rear.velocity_std <= trusted_speed_max_std_mps_ &&
        std::isfinite(t.target_rear.observation_age) &&
        t.target_rear.observation_age >= 0.0 &&
        t.target_rear.observation_age <= 0.3 &&
        !rear_vehicle_actually_closing &&      // 뒤차가 접근 중이 아님(= 더 느림)
        !side_vehicle_yield_commit_active &&   // 이미 양보를 시작했으면 유지
        side_veh_delta_s < 0.0 &&              // 실제로 자차보다 뒤
        slow_rear_front_clear;

    // 느린 뒤차: 앞지르기/양보 결정을 차량 id 별로 고정한다(2026-09-26, 13:48 백).
    //   실측: 옆 차로 id2(10.5~11 m/s)가 자차(14.8 m/s) 옆~뒤에 있을 때
    //   overtake_slow_rear / HOLD / yield_adjacent 가 프레임마다 바뀌고, HOLD 상한이
    //   ACC 목표(=직전 상한)를 따라 내려가기만 해 14.8 -> 13.5 m/s 로 느려졌다.
    //   앞지르지도 양보하지도 못한 채 4 초를 쓰고 차로 끝 거리 검사에 걸렸다.
    // 처음 판단할 때, 앞질러 뒤 안전영역을 비운 뒤에도 차선변경이 차로 끝 전에
    // 끝나는지 추정한다(가속 -> 속도 유지 순). 안 되면 그 차에 양보해 먼저 보내고,
    // 앞으로 빠지면 ACC 의 양보 후 따라가기가 이어받는다.
    // 앞지르기 중 가능성이 사라지면 양보로 한 번만 바꾼다(반대 방향 전환 없음).
    const ros::Time slow_rear_now = ros::Time::now();
    if (slow_rear_latch_current_lane_id_ != t.current_lane_id ||
        slow_rear_latch_target_lane_id_ != t.target_lane_id ||
        !slow_rear_latch_enabled_) {
      slow_rear_latch_id_ = -1;
    }
    slow_rear_latch_current_lane_id_ = t.current_lane_id;
    slow_rear_latch_target_lane_id_ = t.target_lane_id;
    bool slow_rear_latched_present = slow_rear_latch_id_ >= 0 &&
        side_veh_in_yield_zone && side_veh_id == slow_rear_latch_id_;
    if (slow_rear_latched_present) slow_rear_latch_last_seen_ = slow_rear_now;
    if (slow_rear_latch_id_ >= 0) {
      const bool expired =
          (slow_rear_now - slow_rear_latch_stamp_).toSec() > slow_rear_latch_timeout_sec_ ||
          (slow_rear_now - slow_rear_latch_last_seen_).toSec() > slow_rear_latch_lost_sec_;
      // 앞지르기 중 뒤차가 실제로 따라붙기 시작하면 기존 뒤차 양보 로직에 맡긴다.
      const bool overtake_rear_closing = slow_rear_latch_mode_ == SlowRearMode::kOvertake &&
          slow_rear_latched_present && rear_vehicle_actually_closing;
      // 양보한 차가 자차 앞으로 나가면 양보 고정을 푼다(이후 앞 양보영역/ACC 추종).
      const bool yielded_vehicle_passed = slow_rear_latch_mode_ == SlowRearMode::kYield &&
          slow_rear_latched_present && side_veh_delta_s >= 0.0;
      if (expired || overtake_rear_closing || yielded_vehicle_passed) {
        slow_rear_latch_id_ = -1;
        slow_rear_latched_present = false;
      }
    }
    const double slow_rear_mission_speed = currentTargetSpeedMps();
    const double slow_rear_accel_cap = std::min(
        slow_rear_mission_speed, ego_speed_mps_ + std::max(0.0, slow_rear_overtake_speed_gain_mps_));
    if (slow_rear_latch_enabled_ && slow_rear_latch_id_ < 0 && slow_rear_overtake_raw &&
        side_veh_id >= 0 && t.target_rear.unique_id == side_veh_id) {
      slow_rear_latch_id_ = side_veh_id;
      slow_rear_latch_stamp_ = slow_rear_now;
      slow_rear_latch_last_seen_ = slow_rear_now;
      slow_rear_latched_present = true;
      const double margin = std::max(0.0, slow_rear_overtake_deadline_margin_m_);
      if (!slow_rear_deadline_check_enabled_) {
        slow_rear_latch_mode_ = SlowRearMode::kOvertake;
        slow_rear_overtake_cap_mps_ = ego_speed_mps_;
      } else if (slow_rear_accel_cap > ego_speed_mps_ + 0.3 &&
                 slowRearOvertakeFits(t, t.target_rear, slow_rear_overtake_accel_mps2_,
                                      slow_rear_accel_cap, margin)) {
        slow_rear_latch_mode_ = SlowRearMode::kOvertake;
        slow_rear_overtake_cap_mps_ = slow_rear_accel_cap;
      } else if (slowRearOvertakeFits(t, t.target_rear, 0.0, ego_speed_mps_, margin)) {
        slow_rear_latch_mode_ = SlowRearMode::kOvertake;
        slow_rear_overtake_cap_mps_ = ego_speed_mps_;
      } else {
        slow_rear_latch_mode_ = SlowRearMode::kYield;
      }
    } else if (slow_rear_deadline_check_enabled_ &&
               slow_rear_latch_mode_ == SlowRearMode::kOvertake &&
               slow_rear_latched_present && t.target_rear.valid &&
               t.target_rear.unique_id == slow_rear_latch_id_ &&
               targetVelocityTrusted(t.target_rear)) {
      // 재확인은 여유(margin) 없이 한다: 처음 결정보다 느슨하게 둬 경계에서 흔들리지 않게.
      const bool accelerating = slow_rear_overtake_cap_mps_ > ego_speed_mps_ + 0.3;
      const bool still_fits = slowRearOvertakeFits(
          t, t.target_rear, accelerating ? slow_rear_overtake_accel_mps2_ : 0.0,
          std::max(ego_speed_mps_, slow_rear_overtake_cap_mps_), 0.0);
      if (!still_fits) slow_rear_latch_mode_ = SlowRearMode::kYield;
    }
    const bool slow_rear_overtake_latched = slow_rear_latched_present &&
        slow_rear_latch_mode_ == SlowRearMode::kOvertake;
    const bool slow_rear_yield_latched = slow_rear_latched_present &&
        slow_rear_latch_mode_ == SlowRearMode::kYield && side_veh_delta_s < 0.0;
    const bool slow_rear_overtake_active = slow_rear_overtake_latched ||
        (slow_rear_overtake_raw && slow_rear_latch_id_ < 0);
    const bool rear_side_vehicle_requires_yield =
        rear_side_vehicle_requires_yield_base || slow_rear_yield_latched;
    // 고정된 앞지르기/양보는 behavior 후보(HOLD/YIELD_THEN_CHANGE)보다 우선한다.
    const bool slow_rear_latch_command =
        slow_rear_overtake_latched || slow_rear_yield_latched;
    // Keep the immediate 8 m / 15 m approval box unchanged, but begin a
    // controlled longitudinal preparation when a slower target-front vehicle
    // will make the predicted merge unsafe.  This condition is deliberately
    // separate from occupancy: expanding the approval box would only prolong
    // WAIT_FOR_GAP.  Invalid/stale velocity never triggers this early path.
    const double target_front_closing_mps = t.target_front.valid
        ? ego_speed_mps_ - leadTrustedLongSpeed(t.target_front) : 0.0;
    const bool target_front_velocity_fresh = t.target_front.valid &&
        t.target_front.velocity_valid &&
        std::isfinite(t.target_front.velocity_std) &&
        t.target_front.velocity_std <=
            target_front_early_yield_max_velocity_std_mps_ &&
        std::isfinite(t.target_front.observation_age) &&
        t.target_front.observation_age <=
            target_front_early_yield_max_observation_age_sec_;
    const bool target_front_requires_early_yield =
        target_front_early_yield_enabled_ && target_lane_is_left &&
        target_front_velocity_fresh && !side_veh_in_yield_zone &&
        target_front_gap > yield_front_window &&
        target_front_gap <= target_front_early_yield_window_m_ &&
        target_front_closing_mps >=
            target_front_early_yield_min_closing_mps_;
    const bool deterministic_gap_safe =
        !side_veh_in_yield_zone &&
        (!t.target_front.valid || target_front_gap > active_target_front_safe_gap) &&
        (!t.target_rear.valid || target_rear_gap > effective_target_rear_safe_gap) &&
        target_front_ttc_safe && target_rear_ttc_safe && target_rear_headway_safe;
    const bool change_time_risk_safe =
        timeIndexedRiskSafe(change_eval, effective_allowed_risk);
    const bool change_legacy_risk_safe = change_eval.valid &&
        (!behavior_require_legacy_risk_gate_ ||
         change_eval.risk <= effective_allowed_risk);
    const bool change_clearance_safe = clearanceSafe(change_eval);
    const bool chance_gap_safe = change_eval.valid && change_time_risk_safe &&
        change_legacy_risk_safe && change_clearance_safe;
    // CHANGE_NOW checks the occupied zone and TTC, and still requires an
    // executable trajectory without physical overlap through the merge.
    const bool change_projected_gap_ttc_safe = change_eval.valid &&
        projectedTargetGapTtcSafe(
            t, change_traj, change_now_spec, clear_zone_ttc_relaxed);
    const std::string change_projected_rejection=change_eval.valid
        ? projected_gap_rejection_ : "trajectory_invalid,"+change_eval.detail;
    // 차로 끝 정지 합류: 거리 검사를 건너뛰는 대신 목표 차로 뒤차에 대해 별도 기준을
    // 둔다. 정지 상태에서는 안전영역(저속 7 m)과 완화 TTC(0.6 s)가 너무 느슨하다.
    //   13:48 백 재생: 뒤차 13 m / 9~11 m/s 접근 중에 CHANGE 가 확정됐다.
    const bool standstill_merge = laneEndStandstillMergeActive(t);
    bool standstill_rear_ok = true;
    if (standstill_merge) {
      const auto check_rear = [&](const smpc_lane_change::TargetVehicle& v) {
        if (!v.valid || v.lane_id != t.target_lane_id || v.delta_s >= 0.0) return;
        if (!std::isfinite(v.v_long)) { standstill_rear_ok = false; return; }
        const double gap = rearBumperGap(v);
        const double closing = std::max(0.0, leadTrustedLongSpeed(v) - ego_speed_mps_);
        if (gap < lane_end_standstill_rear_min_gap_m_ ||
            (closing > 0.1 && gap / closing < lane_end_standstill_rear_min_ttc_sec_)) {
          standstill_rear_ok = false;
        }
      };
      check_rear(t.target_rear);
      for (const auto& nb : t.nearby_vehicles) check_rear(nb);
    }
    const bool change_feasible =
        standstill_rear_ok &&
        candidateFitsHorizon(change_now_spec) &&
        candidateFitsLaneDeadline(t, change_now_spec) &&
        !side_veh_in_yield_zone &&
        target_lane_is_left && change_eval.valid && deterministic_gap_safe &&
        chance_gap_safe && change_projected_gap_ttc_safe;

    // [User Request] 차선변경시 양보 구간(앞 8m, 뒤 15m) 및 차선변경 가능 조건이
    // 히스테리시스 3프레임 연속으로 충족해야 변경 가능하도록 카운트
    // 새 목표 차량 데이터가 들어온 프레임만 센다. 17:15 백: 3 프레임 중 둘이 10 ms
    // 간격(같은 관측)이라 확인이 0.2 s 만에 끝났다.
    if (change_feasible) {
      if (!streak_count_new_targets_only_ || targets_stamp_ != last_streak_targets_stamp_) {
        change_feasible_streak_ =
            std::min(change_feasible_confirm_count_, change_feasible_streak_ + 1);
        last_streak_targets_stamp_ = targets_stamp_;
      }
    } else {
      change_feasible_streak_ = 0;
    }
    const bool change_confirmed = (change_feasible_streak_ >= change_feasible_confirm_count_);


    double keep_cost = risk_cost_weight_ * keep_eval.risk +
        gapPenalty(effective_current_gap, current_safe_gap) +
        laneEndPressureCost(t);
    if (!keep_eval.valid) keep_cost += lane_end_emergency_cost_;
    if (current_blocked) keep_cost += blocked_cost_weight_;

    const double target_gap_cost =
        gapPenalty(target_front_gap, active_target_front_safe_gap) +
        gapPenalty(target_rear_gap, effective_target_rear_safe_gap);
    double change_cost = lane_change_base_cost_ +
        risk_cost_weight_ * change_eval.risk +
        target_gap_cost;
    if (!target_lane_is_left) change_cost += lane_end_emergency_cost_;
    if (!change_eval.valid) change_cost += lane_end_emergency_cost_;
    if (!deterministic_gap_safe) change_cost += lane_end_urgent_cost_;
    if (!chance_gap_safe) change_cost += lane_end_urgent_cost_;
    if (!change_projected_gap_ttc_safe) change_cost += lane_end_urgent_cost_;

    // 커밋 히스테리시스.
    //
    // 한 번 변경을 요청한 뒤에는 같은 목표 차로 후보에 보너스를 주고, 그
    // 상태가 유지되는 동안 반대 결정(KEEP)에는 벌점을 준다. 매 사이클
    // 독립적으로 비용을 비교하면, 변경을 시작한 뒤 관측이 조금만 흔들려도
    // 판단이 뒤집혀 차로 사이에서 어정쩡하게 멈출 수 있다.
    //   src_최적(kcity2026 highway_trajectory _score)의 같은 장치를 가져온
    //   것이다. 그쪽은 committed 후보에 -7.0, 다른 후보에 +9.0 을 준다.
    // commit_bias 는 '이미 가던 방향을 계속하는' 쪽으로만 작용하므로,
    // 안전 게이트(change_confirmed / feasible)를 우회하지 않는다 - 위험해
    // 지면 change_confirmed 가 떨어져 어차피 실행되지 않는다.
    const bool commit_active = commit_hysteresis_enabled_ &&
        commit_target_lane_id_ == t.target_lane_id &&
        commit_target_lane_id_ >= 0 &&
        (ros::Time::now() - commit_stamp_).toSec() <= commit_hold_sec_;
    const double commit_bias = commit_active ? commit_bias_cost_ : 0.0;

    const bool lane_change_allowed = laneChangeAllowedHere();
    const bool change_wanted = endpoint_trigger || current_blocked ||
        (change_cost + decision_hysteresis_cost_ - commit_bias < keep_cost);
    const bool desire_change = lane_change_allowed && change_wanted;
    const bool request_change = desire_change && change_confirmed;
    // 준비 전용 구간(lanePrepOnlyHere): 변경 요청은 내지 않고 틈 맞추기 속도 조절만 한다.
    const bool prep_only = !lane_change_allowed && change_wanted && lanePrepOnlyHere();

    // 목표 차로 틈 선택(buildMergeGaps 주석). 고른 틈은 앞/뒤 차량 id 로 고정하고,
    // 새 후보가 switch_cost 이상 빨리 맞을 때만 바꾼다.
    const double gap_v_max = independentAccLeadCap(t, currentTargetSpeedMps());
    const bool gap_select_applicable =
        gap_select_enabled_ && ((desire_change && !change_confirmed) || prep_only) &&
        target_lane_is_left &&
        std::isfinite(t.distance_to_lane_end) && !laneEndStandstillMergeActive(t) &&
        std::find(gap_select_current_lanes_.begin(), gap_select_current_lanes_.end(),
                  t.current_lane_id) != gap_select_current_lanes_.end();
    std::vector<MergeGap> merge_gaps;
    std::vector<GapVehicle> gap_vehicles;
    MergeGap selected_gap;
    bool gap_selected = false;
    bool gap_selecting = false;  // 틈 선택 적용 중이지만 경계 차량 속도 확인 대기
    if (gap_latch_current_lane_id_ != t.current_lane_id ||
        gap_latch_target_lane_id_ != t.target_lane_id || !gap_select_applicable) {
      gap_latch_valid_ = false;
      gap_pending_valid_ = false;
      gap_side_vehicle_id_ = -1;
      gap_side_ = 0;
    }
    gap_latch_current_lane_id_ = t.current_lane_id;
    gap_latch_target_lane_id_ = t.target_lane_id;
    if (gap_select_applicable) {
      merge_gaps = buildMergeGaps(t, gap_v_max, &gap_vehicles);
      // 한 모드 원칙(16:02 백): 목표 차로에 차량이 있으면 가능한 틈이 없어도 기존
      // 양보/앞지르기 로직으로 넘어가지 않는다. 가능한 틈(차로 끝 만족)이 있으면 그중
      // 최선, 없으면 맞출 수는 있는 틈 중 차로 끝 초과가 가장 작은 틈(best effort)을
      // 계속 추구한다. 경계에서 가능/불가능이 오가며 두 로직이 번갈아 명령하던 것을 막는다.
      const auto better = [](const MergeGap& a, const MergeGap& b) {
        if (a.feasible != b.feasible) return a.feasible;
        if (a.feasible) return a.cost < b.cost;
        return a.overshoot_m < b.overshoot_m;
      };
      const MergeGap* cost_best = nullptr;
      for (const auto& g : merge_gaps) {
        if (!g.aligns) continue;
        if (!cost_best || better(g, *cost_best)) cost_best = &g;
      }
      // 방향 규칙(gap_select_side_policy). 목표 차로에서 자차에 가장 가까운 차량(기준 차량)
      // 을 두고, speed: 나보다 느리면 그 차 앞 틈(추월), 빠르면 그 차 뒤 틈(보내기).
      // yield: 항상 그 차 뒤 틈(보낸 뒤, 그 차와 그 뒤 차량 사이 틈을 다시 본다).
      // 고른 쪽이 차로 끝 안에 불가능하고 반대쪽이 가능하면 반대쪽, 둘 다 불가능하면
      // 고른 쪽을 best effort 로 추구한다. 방향은 기준 차량이 바뀔 때까지 유지.
      const GapVehicle* ref_vehicle = nullptr;
      for (const auto& v : gap_vehicles) {
        const double c = 0.5 * (v.rear_edge + v.front_edge);
        if (!ref_vehicle ||
            std::abs(c) < std::abs(0.5 * (ref_vehicle->rear_edge + ref_vehicle->front_edge))) {
          ref_vehicle = &v;
        }
      }
      int side = 0;  // +1: 기준 차량 앞, -1: 뒤, 0: 시간 비용으로
      if (ref_vehicle && gap_select_side_policy_ != "cost") {
        if (ref_vehicle->id >= 0 && ref_vehicle->id == gap_side_vehicle_id_ && gap_side_ != 0) {
          side = gap_side_;
        } else if (gap_select_side_policy_ == "yield") {
          side = -1;
        } else {
          // 앞지를 수 있는 최고 속도: 같은 차로 앞차가 가까우면 그 차 속도 근처가 한계.
          double overtake_speed = gapOvertakeSpeed(t, gap_v_max);
          const auto& own = t.current_front;
          if (gap_select_own_lead_enabled_ && own.valid) {
            const double og = frontBumperGap(own);
            if (std::isfinite(og) && og > -1.0 && og <= gap_select_own_lead_max_gap_m_) {
              overtake_speed = std::min(overtake_speed,
                  std::max(0.0, leadTrustedLongSpeed(own)) + gap_select_own_lead_overtake_margin_mps_);
            }
          }
          const double db = gap_select_side_speed_deadband_mps_;
          // 흐름 속도가 설정된 차로에서는 기준 차량이 곧 그 속도까지 올라온다고 보고 비교한다.
          const double ref_v = std::max(ref_vehicle->v, gapFlowSpeedFor(t.current_lane_id));
          if (ref_v > ego_speed_mps_ + db || ref_v >= overtake_speed - db) {
            side = -1;   // 기준 차량이 더 빠르거나, 앞차 때문에 앞지를 수 없음 -> 보내기
          } else if (ref_v < ego_speed_mps_ - db) {
            side = +1;
          }
        }
      }
      // 2026-10-13 기준 차량 가속 판단(overtake_policy.hpp). 14:44:35 백: 추월로 정한 기준 차량이
      //   1.8 m/s^2 로 같이 가속해 나란히 달리다 차로 끝에서 섰다(자차 실측 0.5~1.0 m/s^2).
      //   - 처음 고를 때: 기준 차량이 가속 중이고 우리 추월 가속보다 빠르거나 상대 가속이 작으면 뒤로.
      //   - 추월 중: 자차 실측 가속과 비교해 같은 조건이 confirm_sec 이어지면 포기하고 뒤 틈으로.
      //   포기 상태는 그 기준 차량이 앞으로 지나가거나 바뀔 때까지 유지(앞 틈 재선택 금지).
      bool overtake_blocked = false;
      dbg_ot_abort_ = "none";
      {
        const double now_ot = ros::Time::now().toSec();
        double a_ref_raw = 0.0;
        const double a_ref = ref_vehicle ? refAccel(ref_vehicle->id, now_ot, &a_ref_raw) : 0.0;
        bool ego_valid = false;
        const double a_ego = egoAccelMeasured(now_ot, &ego_valid);
        dbg_a_ref_ = a_ref; dbg_a_ref_raw_ = a_ref_raw; dbg_a_ego_ = a_ego; dbg_a_ego_valid_ = ego_valid;
        if (gap_overtake_accel_policy_enabled_ && ref_vehicle && ref_vehicle->id >= 0 &&
            gap_select_side_policy_ != "cost") {
          const double half_len = 0.5 * ego_length_m_;
          const bool ego_passed = ref_vehicle->front_edge < -half_len - 1.0;  // 자차가 완전히 앞질렀음
          const bool ref_passed = ref_vehicle->rear_edge > half_len + 1.0;    // 기준 차량이 완전히 앞으로 지나감
          const bool latched_overtake = side > 0 && ref_vehicle->id == gap_side_vehicle_id_ && gap_side_ > 0;
          const std::string why = latched_overtake
              ? smpc_lane_change::overtakeAbortReason(a_ref, a_ego, ego_valid, overtake_policy_) : std::string();
          const bool was_aborted = overtake_monitor_.aborted && overtake_monitor_.ref_id == ref_vehicle->id;
          const bool aborted = overtake_monitor_.update(ref_vehicle->id, now_ot, why, ego_passed, ref_passed,
                                                        overtake_policy_);
          if (aborted) {
            overtake_blocked = true;
            dbg_ot_abort_ = overtake_monitor_.reason;
            if (!was_aborted) {
              gap_latch_valid_ = false;     // 앞 틈 고정을 풀고 뒤 틈으로 다시 고른다
              gap_pending_valid_ = false;
              ROS_WARN("[smpc_decision] overtake abort ref=%d (%s): a_ref=%.2f a_ego=%.2f%s",
                       ref_vehicle->id, overtake_monitor_.reason.c_str(), a_ref, a_ego, ego_valid ? "" : "(cmd)");
            }
          } else if (side > 0 && !latched_overtake && !ego_passed &&
                     !smpc_lane_change::overtakeAllowedByAccel(a_ref, overtake_policy_)) {
            overtake_blocked = true;
            dbg_ot_abort_ = "initial_ref_accel";
          }
          if (overtake_blocked) {
            side = -1;
            gap_side_vehicle_id_ = ref_vehicle->id;
            gap_side_ = -1;
          }
        }
      }
      const MergeGap* best = cost_best;
      if (side != 0 && ref_vehicle) {
        const MergeGap* preferred = nullptr;
        const MergeGap* opposite = nullptr;
        for (const auto& g : merge_gaps) {
          if (!g.aligns || ref_vehicle->id < 0) continue;
          const bool ahead_of_ref = g.follower_id == ref_vehicle->id;
          const bool behind_ref = g.leader_id == ref_vehicle->id;
          if ((side > 0 && ahead_of_ref) || (side < 0 && behind_ref)) preferred = &g;
          if ((side > 0 && behind_ref) || (side < 0 && ahead_of_ref)) opposite = &g;
        }
        if (overtake_blocked) opposite = nullptr;  // 가속하는 기준 차량 앞 틈(추월)은 고르지 않는다
        // 앞 틈(앞지르기)은 차로 끝까지 여유가 overtake_min_margin 이상 남을 때만 고른다.
        // 여유가 작으면 상대 차량이 조금만 더 빨라져도 불가능이 되고, 그때는 이미 그 차
        // 옆이라 뒤로 빠질 거리도 없다(22:25 백: 여유 12.3 m 로 시작해 2.8 초 뒤 불가능).
        const bool preferred_ok = preferred && preferred->feasible &&
            (side < 0 ||
             preferred->overshoot_m <= -std::max(0.0, gap_select_overtake_min_margin_m_));
        if (preferred_ok) best = preferred;
        else if (opposite && opposite->feasible) best = opposite;
        else if (preferred && opposite) {
          best = preferred->overshoot_m <= opposite->overshoot_m ? preferred : opposite;
        } else if (preferred) best = preferred;
        else if (opposite) best = opposite;
      }
      gap_rule_side_ = side;
      gap_rule_vehicle_id_ = ref_vehicle ? ref_vehicle->id : -1;
      gap_rule_vehicle_speed_ = ref_vehicle ? ref_vehicle->v : 0.0;
      // 고정된 틈 찾기: 같은 id 쌍 > 앞차 id 같음(그 차 뒤 자리) 또는 뒤차 id 같음(그 차
      // 앞 자리) > 예측 위치 match_dist 이내. 먼 차량이 잡혔다 사라져도 같은 자리로 본다.
      const MergeGap* latched = nullptr;
      if (gap_latch_valid_) {
        double predicted_target = std::numeric_limits<double>::quiet_NaN();
        if (!gap_latch_stamp_.isZero()) {
          const double dt_latch = std::clamp(
              (ros::Time::now() - gap_latch_stamp_).toSec(), 0.0, 1.0);
          predicted_target = gap_latch_target_pos_ +
              (gap_latch_gap_speed_ - ego_speed_mps_) * dt_latch;
        }
        int best_rank = 99;
        double best_dist = std::numeric_limits<double>::infinity();
        for (const auto& g : merge_gaps) {
          if (!g.aligns) continue;
          int rank = 99;
          if (g.sameKey(gap_latch_follower_id_, gap_latch_leader_id_)) rank = 0;
          else if ((gap_latch_leader_id_ >= 0 && g.leader_id == gap_latch_leader_id_) ||
                   (gap_latch_follower_id_ >= 0 && g.follower_id == gap_latch_follower_id_)) rank = 1;
          const double dist = std::isfinite(predicted_target)
              ? std::abs(gapReferencePos(g) - predicted_target)
              : std::numeric_limits<double>::infinity();
          if (rank == 99 && dist <= gap_select_match_dist_m_) rank = 2;
          if (rank < best_rank || (rank == best_rank && rank < 99 && dist < best_dist)) {
            best_rank = rank;
            best_dist = dist;
            latched = rank < 99 ? &g : latched;
          }
        }
      }
      const bool latch_recent = gap_latch_valid_ &&
          (ros::Time::now() - gap_latch_stamp_).toSec() <= gap_select_hold_sec_;
      if (latched) {
        // 규칙 방향과 반대편 틈에 고정돼 있고 규칙 쪽 틈이 가능하면 교체(17:59 백:
        // 속도 불확실 때 '앞' 틈을 고정한 뒤 기준 차량이 더 빠른 게 확인됐는데도 유지).
        const bool latched_opposite = side != 0 && ref_vehicle && ref_vehicle->id >= 0 &&
            ((side < 0 && latched->follower_id == ref_vehicle->id) ||
             (side > 0 && latched->leader_id == ref_vehicle->id));
        const bool switch_wanted = best && best != latched &&
            ((latched_opposite && best->feasible) ||
             (best->feasible && !latched->feasible) ||
             (best->feasible && latched->feasible &&
              best->cost < latched->cost - gap_select_switch_cost_sec_) ||
             (!best->feasible && !latched->feasible &&
              best->overshoot_m < latched->overshoot_m - gap_select_effort_switch_m_));
        selected_gap = *latched;
        gap_selected = true;
        if (switch_wanted) {
          // 교체는 switch_confirm_frames 연속으로 같은 틈이 더 좋을 때만.
          if (gap_pending_valid_ &&
              best->sameKey(gap_pending_follower_id_, gap_pending_leader_id_)) {
            ++gap_pending_count_;
          } else {
            gap_pending_valid_ = true;
            gap_pending_follower_id_ = best->follower_id;
            gap_pending_leader_id_ = best->leader_id;
            gap_pending_count_ = 1;
          }
          if (gap_pending_count_ >= std::max(1, gap_select_switch_confirm_frames_)) {
            selected_gap = *best;
            gap_pending_valid_ = false;
          }
        } else {
          gap_pending_valid_ = false;
        }
      } else if (!latch_recent && best) {
        // 처음 선택(또는 고정 틈을 hold_sec 넘게 못 찾음). 틈 경계 차량 속도를 아직
        // 모르면(새 트랙) 고르지 않고 현재 속도를 유지한다(gap_selecting). 새 트랙은
        // 자차 속도로 가정돼, 15:52/16:02 백에서 첫 0.5 s 에 틀린 틈을 고정했다.
        bool any_unknown = false;
        for (const auto& g : merge_gaps) any_unknown = any_unknown || (g.aligns && !g.speed_known);
        if (!any_unknown) {
          selected_gap = *best;
          gap_selected = true;
          gap_pending_valid_ = false;
        } else {
          gap_selecting = true;
        }
      }
      // 고정된 틈을 hold_sec 안에서 잠깐 못 찾으면 gap_hold_last 로 직전 명령 유지.
      if (gap_selected && side != 0 && ref_vehicle) {
        gap_side_vehicle_id_ = ref_vehicle->id;
        gap_side_ = side;
      }
      if (gap_selected) {
        gap_latch_valid_ = true;
        gap_latch_follower_id_ = selected_gap.follower_id;
        gap_latch_leader_id_ = selected_gap.leader_id;
        gap_latch_target_pos_ = gapReferencePos(selected_gap);
        gap_latch_gap_speed_ = selected_gap.speed();
        gap_latch_stamp_ = ros::Time::now();
      } else if (gap_latch_valid_ &&
                 (ros::Time::now() - gap_latch_stamp_).toSec() > gap_select_hold_sec_) {
        gap_latch_valid_ = false;
      }
    }
    // 고정된 틈을 잠깐(hold_sec) 못 찾으면 직전 명령을 유지한다. 한 프레임 빠졌다고
    // 앞지르기/양보 로직으로 넘어가 명령이 번갈아 나오는 것을 막는다.
    const bool gap_hold_last = gap_select_applicable && !gap_selected && !gap_selecting &&
        gap_latch_valid_ && gap_last_command_valid_;
    // 틈 명령 속도: 선택된 틈 -> 틈 식, 직전 명령 유지 -> 직전 값, 선택 대기 -> 현재 속도.
    // 변화율은 gap_select_cmd_rate_mps2 로 제한한다(같은 자리가 먼 차량 가시성에 따라
    // 두 차 사이/열린 틈으로 바뀌며 명령이 오가던 것 흡수, 14:41 백 12.7 <-> 16.1).
    double gap_speed_cmd_raw = 0.0;
    if (gap_selected) {
      gap_speed_cmd_raw = gapAlignSpeedCommand(selected_gap, 0.0, 0.0, ego_speed_mps_, gap_v_max);
    } else if (gap_hold_last) {
      gap_speed_cmd_raw = gap_last_speed_cmd_;
    } else if (gap_selecting) {
      gap_speed_cmd_raw = std::min(ego_speed_mps_, gap_v_max);
    }
    const bool gap_command_active = gap_selected || gap_hold_last || gap_selecting;
    double gap_speed_cmd = gap_speed_cmd_raw;
    if (gap_command_active) {
      const ros::Time now_cmd = ros::Time::now();
      const double dt_cmd = gap_last_command_valid_ && !gap_last_cmd_stamp_.isZero()
          ? (now_cmd - gap_last_cmd_stamp_).toSec() : -1.0;
      const double prev = (dt_cmd > 0.0 && dt_cmd < 0.5) ? gap_last_filtered_cmd_ : ego_speed_mps_;
      const double dt_step = (dt_cmd > 0.0 && dt_cmd < 0.5) ? dt_cmd : 0.05;
      const double step = std::max(0.0, gap_select_cmd_rate_mps2_) * dt_step;
      // 추월(기준 차량 앞 틈) 중 가속은 gap_select_overtake_accel_mps2(4.0) 까지 올린다.
      const double step_up = gap_rule_side_ > 0
          ? std::max(step, std::max(0.0, gap_select_overtake_accel_mps2_) * dt_step) : step;
      gap_speed_cmd = std::clamp(gap_speed_cmd_raw, prev - step, prev + step_up);
      gap_last_filtered_cmd_ = gap_speed_cmd;
      gap_last_cmd_stamp_ = now_cmd;
    }
    if (gap_selected) {
      gap_last_speed_cmd_ = gap_speed_cmd_raw;
      gap_last_align_time_ = selected_gap.align_time;
      gap_last_command_valid_ = true;
    } else if (gap_selecting) {
      gap_last_command_valid_ = true;
    } else if (!gap_hold_last) {
      gap_last_command_valid_ = false;
    }
    int gap_feasible_count = 0;
    for (const auto& g : merge_gaps) gap_feasible_count += g.feasible ? 1 : 0;

    // 시각화: 틈 선택 모드(틈 고름/직전 명령 유지)면 기존 안전영역 마커 대신 틈 마커를
    // 같은 토픽(/smpc/yield_zone_markers)에 낸다.
    // 틈 선택이 적용되는 동안(목표 차로에 차량이 있음)은 틈이 확정 전이어도 틈 마커를
    // 유지해 두 시각화가 프레임마다 번갈아 나오지 않게 한다.
    if (gap_select_publish_markers_ &&
        (gap_command_active || (gap_select_applicable && !merge_gaps.empty()))) {
      publishGapSelectMarkers(t, merge_gaps, gap_vehicles, gap_selected, gap_hold_last,
                              selected_gap, gap_speed_cmd, change_feasible, change_confirmed);
    } else {
      publishYieldZoneMarkers(t, side_veh_in_yield_zone, change_feasible, change_confirmed,
                              yield_front_window, yield_rear_window);
    }

    // 변경 의사가 확정된 시점부터 커밋을 유지한다. 의사가 사라지거나 목표
    // 차로가 바뀌면 즉시 해제해, 지나간 기회에 발목이 잡히지 않게 한다.
    if (request_change && t.target_lane_id >= 0) {
      commit_target_lane_id_ = t.target_lane_id;
      commit_stamp_ = ros::Time::now();
    } else if (!desire_change) {
      commit_target_lane_id_ = -1;
    }

    // 이번 사이클 결과로 다음 사이클의 완화값을 갱신한다. '변경을 원하는데
    // 확정을 못 받은' 시간만 센다 - 확정되거나(=곧 실행) 변경 의사가
    // 사라지면 리셋한다. 완화값은 사이클 시작 시점에 이미 gap/TTC 계산에
    // 쓰였으므로, 여기서의 갱신은 다음 사이클(50 ms 뒤)에 반영된다.
    updateWaitRelaxation(desire_change && !change_confirmed);

    CandidateEval change_now_candidate;
    change_now_candidate.spec = change_now_spec;
    change_now_candidate.trajectory = change_eval;
    change_now_candidate.target_speed_mps =
        egoChangePredictionTargetSpeedMps(t, change_now_spec);
    change_now_candidate.time_risk_safe = change_time_risk_safe;
    change_now_candidate.legacy_risk_safe = change_legacy_risk_safe;
    change_now_candidate.clearance_safe = change_clearance_safe;
    change_now_candidate.projected_gap_ttc_safe = change_projected_gap_ttc_safe;
    change_now_candidate.feasible = change_confirmed;
    change_now_candidate.cost = change_cost;

    // Compare executable, collision-safe candidates that finish before the
    // lane deadline. A mandatory merge must not always prefer HOLD over a
    // feasible speed adjustment merely because of its action type.
    CandidateEval best_hold_candidate;
    CandidateEval best_yield_candidate;
    CandidateEval best_accel_candidate;
    bool have_hold_candidate = false;
    bool have_yield_candidate = false;
    bool have_accel_candidate = false;
    if (desire_change && !change_confirmed && target_lane_is_left && behavior_planner_enabled_) {
      const auto candidates = buildBehaviorCandidates();
      for (const auto& candidate : candidates) {
        if (candidate.action == BehaviorAction::kChangeNow) continue;
        const CandidateEval evaluated =
            evaluateCandidate(t, vehicles, candidate, allowed_risk, target_gap_cost);
        if (!evaluated.feasible || !behaviorCommandEnabled(candidate)) continue;
        switch (candidate.action) {
          case BehaviorAction::kHoldThenChange:
            if (rear_side_vehicle_requires_yield) break;
            if (!have_hold_candidate || evaluated.cost < best_hold_candidate.cost) {
              best_hold_candidate = evaluated;
              have_hold_candidate = true;
            }
            break;
          case BehaviorAction::kYieldDecelThenChange:
            if (!have_yield_candidate || evaluated.cost < best_yield_candidate.cost) {
              best_yield_candidate = evaluated;
              have_yield_candidate = true;
            }
            break;
          case BehaviorAction::kAccelThenChange:
            if (rear_side_vehicle_requires_yield) break;
            if (!have_accel_candidate || evaluated.cost < best_accel_candidate.cost) {
              best_accel_candidate = evaluated;
              have_accel_candidate = true;
            }
            break;
          case BehaviorAction::kKeep:
          case BehaviorAction::kWaitForGap:
          case BehaviorAction::kChangeNow:
            break;
        }
      }
    }
    CandidateEval best_future_candidate;
    bool have_future_candidate = false;
    const auto consider = [&](bool available, const CandidateEval& candidate) {
      if (available && (!have_future_candidate || candidate.cost < best_future_candidate.cost)) {
        best_future_candidate=candidate;
        have_future_candidate=true;
      }
    };
    consider(have_hold_candidate,best_hold_candidate);
    consider(have_yield_candidate,best_yield_candidate);
    consider(have_accel_candidate,best_accel_candidate);
    const bool future_feasible = change_confirmed || have_future_candidate;

    std::ostringstream debug;
    debug.setf(std::ios::fixed);
    debug.precision(3);
    const double p_current_front = t.current_front.valid
        ? adaptiveEventProbability(t.current_front)
        : -1.0;
    const double p_target_front = t.target_front.valid
        ? adaptiveEventProbability(t.target_front)
        : -1.0;
    const double p_target_rear = t.target_rear.valid
        ? adaptiveEventProbability(t.target_rear)
        : -1.0;
    const bool reference_preview_ready = referencePreviewFreshFor(t);
    debug << "v3 risk legacy_keep=" << keep_eval.risk
          << " legacy_change=" << change_eval.risk
          << " step_peak_keep=" << keep_eval.peak_step_risk
          << " change=" << change_eval.peak_step_risk
          << " change_peak_t=" << prediction_dt_sec_ * (change_eval.peak_step + 1)
          << " change_first_violation=" << firstRiskViolationTimeSec(change_eval, allowed_risk)
          << " clear keep=" << keep_eval.min_signed_clearance
          << " change=" << change_eval.min_signed_clearance
          << " p cf=" << p_current_front
          << " tf=" << p_target_front
          << " tr=" << p_target_rear
          << " gaps cur=" << current_gap
          << " tf=" << target_front_gap
          << " tr=" << target_rear_gap
          << " req_gap tf=" << active_target_front_safe_gap
          << " tr=" << active_target_rear_safe_gap
          << " eff_tr=" << effective_target_rear_safe_gap
          << " relax=" << deadlineRelaxation(t)
          << " wait_relax=" << wait_relax_value_
          << " waited=" << (wait_relax_since_.isZero() ? 0.0
                            : (ros::Time::now() - wait_relax_since_).toSec())
          << " zone_m f=" << yield_front_window
          << " r=" << yield_rear_window
          << " zone_ttc_relaxed=" << (clear_zone_ttc_relaxed ? 1 : 0)
          << " req_ttc tf=" << activeTargetFrontMinTtc(t, clear_zone_ttc_relaxed)
          << " tr=" << activeTargetRearMinTtc(t, clear_zone_ttc_relaxed)
          << " rear_hw=" << activeTargetRearMinHeadway(t, clear_zone_ttc_relaxed)
          << " ttc_ok tf=" << (target_front_ttc_safe ? 1 : 0)
          << " tr=" << (target_rear_ttc_safe ? 1 : 0)
          << " rear_hw_ok=" << (target_rear_headway_safe ? 1 : 0)
          << " projected_gap_ttc_ok=" << (change_projected_gap_ttc_safe ? 1 : 0)
          << " projected_reject=" << change_projected_rejection
          << " low_spd_rear=" << (lowSpeedRearSafetyActive() ? 1 : 0)
          << " risk_eps=" << effective_allowed_risk
          << " rear_priority=" << (target_rear_priority ? 1 : 0)
          << " side_priority=" << (current_front_priority ? 1 : 0)
          << " vehicles=" << vehicles.size()
          << " nearby=" << t.nearby_vehicles.size()
          << " v_ego=" << ego_speed_mps_
          << " v_ref=" << currentTargetSpeedMps()
          << " v_rollout=" << egoPredictionTargetSpeedMps()
          << " v_merge="
          << egoChangePredictionTargetSpeedMps(t, change_now_spec)
          << " ego_rollout=" << (ego_prediction_use_mission_speed_rollout_ ? 1 : 0)
          << " ref_preview=" << (reference_preview_ready ? 1 : 0)
          << "/" << referencePreviewModeName(reference_preview_.mode)
          << " ref_pts=" << reference_preview_.source.poses.size()
          << ":" << reference_preview_.target.poses.size()
          << ":" << reference_preview_.connector.poses.size()
          << " ref_join=" << reference_preview_.connector_join_arc_m
          << " cost keep=" << keep_cost
          << " change=" << change_cost
          << " feasible=" << (change_feasible ? 1 : 0)
          << " confirmed=" << (change_confirmed ? 1 : 0)
          << " streak=" << change_feasible_streak_
          << " desire=" << (desire_change ? 1 : 0)
          << " future=" << (future_feasible ? 1 : 0)
          << " dl_ok=" << (candidateFitsLaneDeadline(t, change_now_spec) ? 1 : 0)
          << " dl_req=" << laneDeadlineRequiredDistance(t, change_now_spec)
          << " dend=" << t.distance_to_lane_end
          << " standstill_merge=" << (standstill_merge ? 1 : 0)
          << " standstill_rear_ok=" << (standstill_rear_ok ? 1 : 0)
          << " slow_rear=" << (slow_rear_latch_id_ < 0 ? "none"
                               : slow_rear_latch_mode_ == SlowRearMode::kYield ? "yield"
                                                                                : "overtake")
          << ":" << slow_rear_latch_id_
          << " ot_cap=" << slow_rear_overtake_cap_mps_
          << " gap=" << (gap_selected ? std::to_string(selected_gap.follower_id) + "/" +
                                         std::to_string(selected_gap.leader_id)
                                      : std::string("none"))
          << " gap_t=" << (gap_selected ? selected_gap.align_time : -1.0)
          << " gap_vcmd=" << gap_speed_cmd
          << " gap_vgap=" << (gap_selected ? selected_gap.speed() : -1.0)
          << " gaps=" << gap_feasible_count << "/" << merge_gaps.size()
          << " gap_rule=" << gap_rule_vehicle_id_ << ":" << gap_rule_side_
          << " a_ref=" << dbg_a_ref_ << " a_ref_raw=" << dbg_a_ref_raw_
          << " a_ego_meas=" << (dbg_a_ego_valid_ ? dbg_a_ego_ : std::numeric_limits<double>::quiet_NaN())
          << " a_rel=" << ((dbg_a_ego_valid_ ? dbg_a_ego_ : gap_select_overtake_accel_mps2_) - dbg_a_ref_)
          << " ot_abort=" << dbg_ot_abort_
          << " gap_eff=" << ((gap_selected && !selected_gap.feasible) ? 1 : 0)
          << " gap_over=" << (gap_selected ? selected_gap.overshoot_m : 0.0)
          << " gap_vlim=" << ((gap_selected && std::isfinite(selected_gap.vlim_min))
                                  ? selected_gap.vlim_min : -1.0)
          << "~" << (gap_selected ? selected_gap.vlim_max : -1.0)
          << " prep_only=" << (prep_only ? 1 : 0)
          << " gap_flow=" << gapFlowSpeedFor(t.current_lane_id);
    // 틈 목록: 뒤id/앞id:가능:맞추는시간:하한:상한:뒤속도:앞속도
    debug << " gl=";
    for (const auto& g : merge_gaps) {
      debug << g.follower_id << "/" << g.leader_id << ":" << (g.feasible ? 1 : 0) << ":"
            << (std::isfinite(g.align_time) ? g.align_time : -1.0) << ":"
            << (std::isfinite(g.lo0) ? g.lo0 : -999.0) << ":"
            << (std::isfinite(g.hi0) ? g.hi0 : 999.0) << ":" << g.v_lo << ":" << g.v_hi << ";";
    }
    if (have_future_candidate) {
      debug << " selected=" << behaviorActionName(best_future_candidate.spec.action)
            << " t_lc=" << best_future_candidate.spec.lane_change_start_sec
            << " a_prep=" << best_future_candidate.spec.prep_accel_mps2
            << " v_cap=" << best_future_candidate.target_speed_mps
            << " c=" << best_future_candidate.cost
            << " peak=" << best_future_candidate.trajectory.peak_step_risk;
    }
    publishDebug(debug.str());

    if (request_change) {
      std::string decision_reason;
      if (endpoint_trigger) {
        decision_reason = "smpc_v2_lane_end_traj_safe";
      } else if (current_blocked) {
        decision_reason = "smpc_v2_current_front_blocked_traj_safe";
      } else {
        decision_reason = "smpc_v3_lower_expected_traj_cost";
      }
      publishBehaviorOutputs(t, change_now_candidate, true, true, allowed_risk,
                             decision_reason);
      publishChange(t, decision_reason, keep_cost, change_cost, change_eval.risk);
      return;
    }

    if (gap_command_active) {
      // 고른 틈에 맞추는 종방향 명령. 감속이 필요하면 YIELD(감속률 지정),
      // 아니면 HOLD 상한(자차보다 높으면 ACC 가 그 속도까지 가속).
      CandidateEval gap_candidate = keep_candidate;
      const bool decel = gap_speed_cmd < ego_speed_mps_ - 0.3;
      const bool accel = gap_response_enabled_ && gap_selected && selected_gap.speed_known &&
          gap_speed_cmd > ego_speed_mps_ + 0.3;
      gap_candidate.spec.action = decel ? BehaviorAction::kYieldDecelThenChange
          : accel ? BehaviorAction::kAccelThenChange : BehaviorAction::kHoldThenChange;
      gap_candidate.spec.prep_accel_mps2 = decel ? -std::abs(gap_select_decel_mps2_)
          : accel ? std::abs(gap_rule_side_ > 0 ? gap_select_overtake_accel_mps2_ : gap_select_accel_mps2_) : 0.0;
      gap_candidate.spec.change_left = true;
      gap_candidate.spec.lane_change_start_sec =
          gap_selected ? selected_gap.align_time : gap_last_align_time_;
      gap_candidate.spec.lane_change_duration_sec = egoChangeBlendDuration();
      gap_candidate.spec.name = "gap_align";
      gap_candidate.target_speed_mps = gap_speed_cmd;
      gap_candidate.cost = gap_selected ? selected_gap.cost : 0.0;
      const std::string gap_reason = dbg_ot_abort_ != "none"
          ? std::string("smpc_v3_gap_overtake_abort_") + dbg_ot_abort_
          : std::string("smpc_v3_gap_align_") +
          (gap_selecting ? "selecting" : !gap_selected ? "keep_last"
                         : selected_gap.aligned_now ? "hold" : decel ? "decel" : "accel");
      publishBehaviorOutputs(t, gap_candidate, false, true, allowed_risk, gap_reason);
      publishKeep(t, gap_reason, keep_cost, change_cost, change_eval.risk);
      return;
    }

    if (desire_change && have_future_candidate && !slow_rear_latch_command) {
      const std::string preparation_reason =
          std::string("smpc_v3_prepare_") + behaviorActionName(best_future_candidate.spec.action);
      publishBehaviorOutputs(t, best_future_candidate, false, true, allowed_risk,
                             preparation_reason);
      publishKeep(t, preparation_reason, keep_cost, best_future_candidate.cost,
                   best_future_candidate.trajectory.risk);
      return;
    }

    if (desire_change && !change_confirmed) {
      // Changing is desired, but neither CHANGE_NOW nor a future candidate
      // has passed its safety gates (or still waiting for hysteresis confirmation).
      // Preserve the longitudinal policy and keep observing.
      CandidateEval wait_candidate = keep_candidate;
      wait_candidate.spec.action = BehaviorAction::kWaitForGap;
      wait_candidate.spec.prep_accel_mps2 = 0.0;
      wait_candidate.spec.lane_change_start_sec = -1.0;
      wait_candidate.spec.lane_change_duration_sec = 0.0;
      wait_candidate.spec.change_left = false;
      wait_candidate.spec.name = "wait_for_gap";
      wait_candidate.target_speed_mps = egoPredictionTargetSpeedMps();

      std::string wait_reason = "smpc_v3_wait_for_gap_change_unsafe";
      if (!target_lane_is_left) wait_reason = "smpc_v3_wait_for_gap_invalid_target_lane";
      else if (!change_eval.valid) wait_reason = "smpc_v3_wait_for_gap_no_change_trajectory";
      else if (!deterministic_gap_safe) wait_reason = "smpc_v3_wait_for_gap_deterministic_gap_unsafe";
      else if (!chance_gap_safe) wait_reason = "smpc_v3_wait_for_gap_chance_risk_high";
      else if (!change_projected_gap_ttc_safe) {
        wait_reason = "smpc_v3_wait_for_gap_projected_gap_ttc_unsafe";
      } else if (!standstill_rear_ok) {
        wait_reason = "smpc_v3_wait_for_gap_standstill_rear_unsafe";
      } else if (!candidateFitsLaneDeadline(t, change_now_spec)) {
        wait_reason = "smpc_v3_wait_for_gap_lane_end_too_close";
      }

      // 옆 차선(목표 차선)에 인접 차량이 있어 지금 당장 변경하지 못하거나 확인 대기 중인 경우:
      const bool target_lane_blocked_by_adjacent =
          side_veh_in_yield_zone ||
          (target_lane_is_left &&
           ((t.target_front.valid && target_front_gap <= active_target_front_safe_gap) ||
            (t.target_rear.valid && target_rear_gap <= effective_target_rear_safe_gap)));

      if (change_feasible && !change_confirmed) {
        // 3프레임 연속 충족 확인 중인 경우: 현재 속도를 안정적으로 유지(HOLD)하면서 안전 확인 대기
        wait_candidate.spec.action = BehaviorAction::kHoldThenChange;
        wait_candidate.spec.prep_accel_mps2 = 0.0;
        wait_candidate.target_speed_mps = egoPredictionTargetSpeedMps();
        wait_candidate.spec.name = "wait_for_gap_confirming_hysteresis";
        wait_reason = "smpc_v3_wait_for_gap_confirming_hysteresis";
      } else if (side_veh_in_yield_zone &&
                 (side_veh_delta_s >= 0.0 || rear_vehicle_actually_closing ||
                  side_vehicle_yield_commit_active || slow_rear_yield_latched)) {
        // 목표 차선의 인접 차량에는 양보해서 그 차량 뒤 슬롯으로 합류한다.
        // 특히 후방 차량이 안전 영역 안에 들어오면 HOLD/ACCEL로 앞지르려 하지
        // 않고 즉시 속도 상한을 낮춰 상대 차량을 먼저 보낸다.
        // 뒤차만 있는데 나보다 느리면 이 블록을 건너뛴다. 앞차가 점유한
        // 경우에는 뒤차 상태와 무관하게 아래 거리 기반 감속을 적용한다.
        wait_candidate.spec.action = BehaviorAction::kYieldDecelThenChange;
        wait_candidate.spec.prep_accel_mps2 = side_vehicle_yield_prep_accel_mps2_;
        if (side_vehicle_yield_adaptive_front_decel_enabled_ &&
            !rear_side_vehicle_requires_yield && side_veh_delta_s > 0.0) {
          const double closing_speed =
              std::max(0.0, ego_speed_mps_ - side_veh_speed);
          const double braking_distance = std::max(
              1.0, side_veh_bumper_gap -
                       side_vehicle_yield_adaptive_front_stop_margin_m_);
          const double required_decel =
              closing_speed * closing_speed / (2.0 * braking_distance);
          wait_candidate.spec.prep_accel_mps2 = -std::min(
              behavior_yield_comfort_max_decel_mps2_,
              std::max(std::abs(side_vehicle_yield_prep_accel_mps2_),
                       required_decel));
        }
        const double yield_floor = std::max(
            side_vehicle_yield_min_speed_mps_,
            behaviorYieldSpeedFloorMps(wait_candidate.spec));
        if (rear_side_vehicle_requires_yield) {
          // Tracker의 순간 v_long 오차가 커도 양보가 취소되지 않도록 자차 속도
          // 기준으로 상한을 내린다. 연속 YIELD의 최저 속도는 episode 기반
          // behavior_yield_max_speed_drop_mps가 제한한다.
          wait_candidate.target_speed_mps = std::max(
              yield_floor, ego_speed_mps_ - side_vehicle_rear_yield_speed_drop_mps_);
          wait_candidate.spec.name = "wait_for_gap_yield_rear_vehicle";
          wait_reason = slow_rear_yield_latched && !rear_side_vehicle_requires_yield_base
              ? "smpc_v3_yield_slow_rear_vehicle_pass"
              : side_vehicle_yield_commit_active && !rear_vehicle_actually_closing
              ? "smpc_v3_yield_committed_side_vehicle_pass"
              : "smpc_v3_yield_rear_vehicle_pass";
        } else {
          double target_yield_speed = ego_speed_mps_ - 2.0;
          if (side_veh_speed > 2.0) {
            // 상대차 후방 목표 간격 15m (TTC/거리 비례 제어)
            const double desired_gap_m = 15.0;
            const double cur_gap_m = std::max(0.0, side_veh_bumper_gap);
            const double gap_err_m = cur_gap_m - desired_gap_m; // 음수면 15m보다 가까움
            const double closing_speed = std::max(0.0, ego_speed_mps_ - side_veh_speed);
            const double ttc = (closing_speed > 0.2) ? (cur_gap_m / closing_speed) : 99.0;

            // 기본 추종 속도: 상대차 속도 + 간격 오차 게인(0.5)
            double commanded_yield_v = side_veh_speed + 0.5 * gap_err_m;

            // TTC 위험 시(TTC < 5.0s) 확실한 제동 감속 유도
            if (ttc < 5.0 && closing_speed > 0.5) {
              commanded_yield_v = std::min(commanded_yield_v, side_veh_speed - 1.5);
            }
            target_yield_speed = std::clamp(commanded_yield_v, yield_floor, ego_speed_mps_ - 0.5);
          } else {
            target_yield_speed = std::max(yield_floor, ego_speed_mps_ - 2.0);
          }
          wait_candidate.target_speed_mps = target_yield_speed;
          wait_candidate.spec.name = "wait_for_gap_yield_ttc";
          wait_reason = "smpc_v3_yield_side_vehicle_ttc";
        }
      } else if (target_front_requires_early_yield) {
        wait_candidate.spec.action = BehaviorAction::kYieldDecelThenChange;
        wait_candidate.spec.prep_accel_mps2 =
            target_front_early_yield_prep_accel_mps2_;
        const double desired_gap = std::max(
            active_target_front_safe_gap,
            standstill_gap_m_ + target_front_early_yield_time_headway_sec_ *
                                   ego_speed_mps_);
        const double commanded_speed = leadTrustedLongSpeed(t.target_front) +
            target_front_early_yield_gap_gain_ *
                (target_front_gap - desired_gap);
        const double yield_floor = std::max(
            side_vehicle_yield_min_speed_mps_,
            behaviorYieldSpeedFloorMps(wait_candidate.spec));
        wait_candidate.target_speed_mps = std::clamp(
            commanded_speed, yield_floor,
            std::max(yield_floor,
                     ego_speed_mps_ -
                         target_front_early_yield_min_speed_drop_mps_));
        wait_candidate.spec.name = "wait_for_gap_early_target_front_yield";
        wait_reason = "smpc_v3_yield_early_target_front";
      } else if (slow_rear_overtake_active) {
        // 느린 뒤차는 양보하지 않고 앞서간다.
        //
        // 목표 차로 점유 차량이 (a) 자차보다 뒤에 있고 (b) 자차보다 느리면,
        // 감속해서 보내주려 해도 그 차는 오지 않는다 - 가만히 있어도 간격이
        // 벌어지는데 감속하면 오히려 교착에 빠진다.
        //   실측 2026-09-14(19:26 백): 뒤차가 4.1 m/s 느린데 양보가 걸려
        //   자차가 15.4 -> 0 m/s 로 완전 정지했다.
        // 단, 앞쪽이 비어 있을 때만 허용한다. 앞 차량이 양보영역 안에
        // 있으면 앞서갈 공간 자체가 없으므로 이 분기를 타지 않는다.
        // 여기서는 감속 요청을 걸지 않고 현재 속도를 유지한다(HOLD).
        // 가속까지 요청하지는 않는다 - ACC 가 앞차/차로 상황에 맞춰
        // 스스로 올리게 두는 편이 안전하다.
        wait_candidate.spec.action = BehaviorAction::kHoldThenChange;
        wait_candidate.spec.prep_accel_mps2 = 0.0;
        wait_candidate.target_speed_mps = egoPredictionTargetSpeedMps();
        if (slow_rear_overtake_latched) {
          // 고정된 앞지르기는 결정 시점의 상한을 유지한다. egoPredictionTargetSpeedMps
          // 는 ACC 목표(=직전 상한)를 따라가 한 번 내려가면 올라오지 못한다.
          wait_candidate.target_speed_mps = std::min(
              slow_rear_mission_speed,
              std::max(wait_candidate.target_speed_mps, slow_rear_overtake_cap_mps_));
        }
        wait_candidate.spec.name = "wait_for_gap_overtake_slow_rear";
        wait_reason = "smpc_v3_overtake_slow_rear";
      } else if (target_lane_blocked_by_adjacent) {
        wait_candidate.spec.action = BehaviorAction::kYieldDecelThenChange;
        wait_candidate.spec.prep_accel_mps2 = -1.0;
        const double yield_floor = behaviorYieldSpeedFloorMps(wait_candidate.spec);
        wait_candidate.target_speed_mps = std::max(yield_floor, ego_speed_mps_ - 2.0);
        wait_candidate.spec.name = "wait_for_gap_yield_adjacent";
        wait_reason += "_yield_adjacent";
      }
      publishBehaviorOutputs(t, wait_candidate, false, false, allowed_risk, wait_reason);
      publishKeep(t, wait_reason, keep_cost, change_cost, change_eval.risk);
      return;
    }

    publishBehaviorOutputs(t, keep_candidate, change_confirmed, future_feasible,
                           allowed_risk, "smpc_v3_keep_lower_expected_traj_cost");
    publishKeep(t, "smpc_v3_keep_lower_expected_traj_cost",
                keep_cost, change_cost, keep_eval.risk);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber targets_sub_, odom_sub_, mission_speed_sub_, acc_target_speed_sub_,
      behavior_status_sub_, reference_preview_sub_, active_section_sub_;
  ros::Publisher decision_pub_, behavior_plan_pub_, behavior_request_pub_;
  ros::Publisher debug_pub_, debug_marker_pub_, prediction_marker_pub_, yield_zone_marker_pub_;
  ros::Publisher keep_path_pub_, change_path_pub_;
  ros::Timer timer_;

  std::string active_section_topic_{"/mission/active_section"};
  bool section_gate_enabled_{true};
  double section_gate_timeout_sec_{1.0};
  bool have_active_section_{false};
  bool in_exempt_section_{false};
  bool section_has_smpc_executor_{false};
  // 목표 차로 흐름 속도 가정(MergeGap::v_flow)
  smpc_lane_change::GapSpeedPolicy gap_speed_policy_;
  bool gap_response_enabled_{true}, gap_shared_speed_enabled_{true}, gap_fixed_flow_enabled_{false};
  double gap_selected_max_catchup_mps_{6.0};
  mutable std::string projected_gap_rejection_{"none"};
  std::vector<int> gap_select_flow_speed_lanes_;
  double gap_select_flow_speed_mps_{-1.0};
  double gap_select_flow_accel_mps2_{0.8};
  double gap_select_overtake_min_margin_m_{0.0};
  // 구간별 제한속도 표(gapSpeedCapAt)
  bool gap_speed_profile_enabled_{true};
  double gap_speed_profile_lookahead_m_{80.0};
  double lane_index_spacing_m_{0.1};
  int speed_exempt_start_csv_{0};
  int speed_exempt_start_index_{11124};
  int speed_exempt_end_csv_{3};
  int speed_exempt_end_index_{6196};
  double state_speed_default_mps_{16.0};
  double zone_urban_max_mps_{16.0};
  double zone_highway_max_mps_{25.0};
  std::vector<double> state_speed_mps_;
  std::map<int, std::vector<int>> lane_states_;
  ros::Time active_section_stamp_;
  int active_section_csv_{-1};
  int active_section_index_{-1};
  bool active_section_index_valid_{false};
  // 이 csv 에서 이 idx 이전에는 차선변경을 실행하지 않는다(laneChangeAllowedHere).
  int lane_change_min_index_csv_{0};
  int lane_change_max_index_csv_{3};
  int lane_change_max_index_{-1};
  int lane_change_min_index_{11000};

  smpc_lane_change::TargetVehicleSet targets_;
  ros::Time targets_stamp_;
  bool have_targets_{false};
  bool have_ego_{false};
  bool have_mission_speed_target_{false};
  bool have_acc_target_speed_{false};
  bool have_behavior_status_{false};
  bool have_reference_preview_{false};
  double ego_x_{0.0};
  double ego_y_{0.0};
  double ego_yaw_{0.0};
  double ego_speed_mps_{0.0};

  std::map<int, smpc_lane_change::LanePath> lanes_;
  std::unordered_map<int, VehicleHistory> vehicle_history_;
  // 2026-10-13 기준 차량·자차 가속(최근 window 초 속도 기울기) 이력과 추월 판단 상태.
  std::unordered_map<int, smpc_lane_change::SpeedSlope> ref_speed_slope_;
  smpc_lane_change::SpeedSlope ego_speed_slope_;
  smpc_lane_change::OvertakeAccelPolicy overtake_policy_;
  smpc_lane_change::OvertakeAbortMonitor overtake_monitor_;
  double gap_select_overtake_accel_mps2_{4.0};
  double gap_ref_accel_window_sec_{1.0};
  double gap_ref_speed_cap_mps_{27.8};
  bool gap_overtake_accel_policy_enabled_{true};
  double dbg_a_ref_{0.0}, dbg_a_ref_raw_{0.0}, dbg_a_ego_{0.0};
  bool dbg_a_ego_valid_{false};
  std::string dbg_ot_abort_{"none"};
  std::string waypoint_directory_;
  double lane_start_extension_m_{80.0};
  double lane_start_extension_spacing_m_{0.1};
  bool lanes_loaded_{false};

  bool enabled_{true};
  double publish_rate_hz_{20.0};
  double target_timeout_sec_{0.5};
  bool debug_yield_zone_publish_markers_{true};
  int change_feasible_confirm_count_{3};
  int change_feasible_streak_{0};
  int horizon_steps_{40};
  double prediction_dt_sec_{0.2};
  double risk_epsilon_{0.04};
  double urgent_risk_epsilon_{0.15};
  bool risk_include_nearby_vehicles_{true};
  bool probabilistic_risk_gate_enabled_{true};

  double ego_length_m_{4.8};
  double ego_width_m_{1.9};
  std::string mission_speed_topic_{"/control_feedback/mission_speed_target_mps"};
  double mission_speed_timeout_sec_{0.5};
  double mission_speed_fallback_mps_{10.0};
  double mission_speed_target_mps_{10.0};
  ros::Time mission_speed_stamp_;
  bool ego_prediction_use_acc_target_speed_{true};
  std::string ego_prediction_acc_target_speed_topic_{"/smpc/target_speed_mps"};
  double ego_prediction_acc_target_speed_timeout_sec_{0.5};
  double acc_target_speed_mps_{0.0};
  ros::Time acc_target_speed_stamp_;
  smpc_lane_change::BehaviorLongitudinalStatus behavior_status_;
  ros::Time behavior_status_stamp_;
  ReferencePreview reference_preview_;
  double standstill_gap_m_{5.0};
  double time_headway_sec_{1.0};
  double current_front_trigger_gap_m_{35.0};
  double current_front_safe_gap_m_{7.0};
  double target_front_safe_gap_m_{14.0};
  double target_rear_safe_gap_m_{9.0};
  bool fixed_target_gap_gate_enabled_{true};
  double prepare_merge_front_safe_gap_m_{12.0};
  double prepare_merge_rear_safe_gap_m_{7.0};
  double target_front_min_ttc_sec_{3.0};
  double target_rear_min_ttc_sec_{3.0};
  double target_rear_min_headway_sec_{1.2};
  bool clear_zone_ttc_relax_enabled_{true};
  double clear_zone_front_min_ttc_sec_{1.2};
  double clear_zone_rear_min_ttc_sec_{1.5};
  double clear_zone_rear_min_headway_sec_{0.6};
  double clear_zone_projected_rear_min_gap_m_{8.0};
  double clear_zone_merge_accel_min_gain_mps_{0.5};
  double clear_zone_risk_epsilon_{0.35};
  double urgent_merge_front_safe_gap_m_{9.0};
  double urgent_merge_rear_safe_gap_m_{5.5};
  double emergency_merge_front_safe_gap_m_{6.5};
  double emergency_merge_rear_safe_gap_m_{4.5};
  double emergency_merge_risk_epsilon_{0.25};
  bool deadline_relax_enabled_{true};
  double deadline_relax_start_distance_m_{120.0};
  double deadline_relax_full_distance_m_{30.0};
  double deadline_relax_front_gap_floor_m_{8.0};
  double deadline_relax_rear_gap_floor_m_{12.0};
  double deadline_relax_front_ttc_floor_sec_{2.0};
  double deadline_relax_rear_ttc_floor_sec_{2.5};
  double deadline_relax_rear_headway_floor_sec_{0.8};
  bool wait_relax_enabled_{true};
  double wait_relax_grace_sec_{3.0};
  double wait_relax_ramp_sec_{3.0};
  // 차선변경을 원하기 시작한 시각. 확정되거나 원하지 않게 되면 0 으로 리셋.
  ros::Time wait_relax_since_{0.0};
  double wait_relax_value_{0.0};
  bool merge_priority_enabled_{true};
  double merge_priority_rear_safe_gap_m_{4.5};
  double merge_priority_rear_min_gap_m_{4.0};
  double merge_priority_rear_max_closing_mps_{3.5};
  double merge_priority_rear_min_ttc_sec_{1.8};
  double merge_priority_rear_risk_scale_{0.10};
  double merge_priority_nearby_rear_risk_scale_{0.05};
  bool low_speed_rear_safety_enabled_{true};
  double low_speed_rear_ego_speed_mps_{2.0};
  double low_speed_rear_safe_gap_m_{10.0};
  double low_speed_rear_min_ttc_sec_{3.0};
  bool ego_lane_priority_enabled_{true};
  double ego_lane_priority_current_max_abs_d_m_{1.25};
  double ego_lane_priority_front_hard_gap_m_{8.0};
  double ego_lane_priority_side_risk_scale_{0.08};

  double lane_width_m_{3.5};
  double lane_change_duration_sec_{2.5};
  bool use_waypoint_linear_blend_{true};
  int waypoint_blend_steps_{17};
  double waypoint_blend_rate_hz_{25.0};
  double waypoint_blend_duration_sec_{0.0};
  double lane_center_decay_tau_sec_{1.2};
  bool ego_reference_preview_enabled_{true};
  bool ego_reference_preview_required_for_change_{true};
  bool ego_reference_preview_allow_fallback_{true};
  std::string ego_reference_preview_topic_{"/path_switch_preview"};
  double ego_reference_preview_timeout_sec_{0.50};
  double ego_reference_preview_motion_start_speed_mps_{0.20};
  double ego_reference_preview_motion_start_distance_m_{0.30};
  double stopped_launch_connector_speed_cap_mps_{2.0};
  double stopped_launch_connector_max_accel_mps2_{1.5};
  double stopped_launch_connector_max_preswitch_distance_m_{0.50};
  bool ego_reference_preview_require_target_completion_{true};
  double ego_reference_preview_target_completion_max_abs_d_m_{0.75};

  bool behavior_planner_enabled_{true};
  bool behavior_execute_decel_hold_{false};
  bool behavior_execute_accel_{false};
  bool behavior_require_legacy_risk_gate_{false};
  double behavior_min_signed_clearance_m_{0.0};
  double behavior_execution_delay_sec_{0.20};
  double behavior_post_merge_buffer_sec_{1.5};
  double behavior_decel_accel_mps2_{-1.0};
  double behavior_yield_comfort_max_decel_mps2_{0.8};
  double behavior_yield_max_speed_drop_mps_{3.0};
  bool side_vehicle_yield_use_safe_gaps_{true};
  double side_vehicle_yield_window_m_{16.0};
  double side_vehicle_yield_front_window_m_{6.0};
  double side_vehicle_yield_rear_window_m_{16.0};
  bool side_vehicle_yield_dynamic_window_enabled_{true};
  double side_vehicle_yield_front_window_max_m_{24.0};
  double side_vehicle_yield_rear_window_max_m_{30.0};
  double side_vehicle_yield_ego_speed_wide_mps_{5.0};
  double side_vehicle_yield_ego_speed_narrow_mps_{15.0};
  bool side_vehicle_yield_commit_enabled_{true};
  double side_vehicle_yield_commit_max_duration_sec_{15.0};
  int side_vehicle_yield_committed_id_{-1};
  int side_vehicle_yield_commit_current_lane_id_{std::numeric_limits<int>::min()};
  int side_vehicle_yield_commit_target_lane_id_{std::numeric_limits<int>::min()};
  ros::Time side_vehicle_yield_commit_stamp_;
  double side_vehicle_yield_min_speed_mps_{10.0};
  double side_vehicle_yield_prep_accel_mps2_{-1.0};
  bool target_front_early_yield_enabled_{true};
  double target_front_early_yield_window_m_{30.0};
  double target_front_early_yield_min_closing_mps_{1.0};
  double target_front_early_yield_max_velocity_std_mps_{3.0};
  double target_front_early_yield_max_observation_age_sec_{0.5};
  double target_front_early_yield_time_headway_sec_{1.0};
  double target_front_early_yield_gap_gain_{0.4};
  double target_front_early_yield_prep_accel_mps2_{-1.2};
  double target_front_early_yield_min_speed_drop_mps_{0.3};
  bool side_vehicle_yield_adaptive_front_decel_enabled_{true};
  double side_vehicle_yield_adaptive_front_stop_margin_m_{2.0};
  double side_vehicle_rear_yield_speed_drop_mps_{2.0};
  double side_vehicle_rear_yield_min_closing_mps_{0.5};
  // 느린 뒤차 앞서가기(slow_rear_overtake_active 참고).
  bool slow_rear_overtake_enabled_{true};
  enum class SlowRearMode { kOvertake, kYield };
  bool slow_rear_latch_enabled_{true};
  bool slow_rear_deadline_check_enabled_{true};
  double slow_rear_overtake_deadline_margin_m_{5.0};
  double slow_rear_overtake_accel_mps2_{1.0};
  double slow_rear_overtake_speed_gain_mps_{3.0};
  double slow_rear_overtake_max_sim_sec_{10.0};
  double slow_rear_latch_timeout_sec_{12.0};
  double slow_rear_latch_lost_sec_{1.0};
  int slow_rear_latch_id_{-1};
  SlowRearMode slow_rear_latch_mode_{SlowRearMode::kOvertake};
  double slow_rear_overtake_cap_mps_{0.0};
  ros::Time slow_rear_latch_stamp_;
  ros::Time slow_rear_latch_last_seen_;
  int slow_rear_latch_current_lane_id_{-1};
  int slow_rear_latch_target_lane_id_{-1};
  double merge_follow_gap_m_{8.5},merge_follow_close_gain_{.2},merge_follow_gain_{.5},merge_follow_catchup_mps_{2.0};
  bool gap_select_enabled_{true};
  double side_vehicle_yield_front_min_headway_sec_{0.5};
  double untrusted_near_gap_m_{20.0};
  double untrusted_std_margin_cap_mps_{2.0};
  bool streak_count_new_targets_only_{true};
  ros::Time last_streak_targets_stamp_;
  bool gap_select_publish_markers_{true};
  std::vector<int> gap_select_current_lanes_{0, 1, 2};
  double gap_select_open_gap_extra_m_{5.0};
  double gap_select_inner_margin_m_{3.0};
  double gap_select_position_gain_{0.3};
  double gap_select_max_catchup_mps_{4.0};
  double gap_select_min_speed_mps_{5.0};
  double gap_select_align_margin_m_{1.0};
  double gap_select_align_speed_tol_mps_{2.0};
  double gap_select_slot_margin_m_{1.0};
  double gap_select_min_slot_m_{1.0};
  double gap_select_max_time_sec_{12.0};
  double gap_select_accel_mps2_{1.2};
  double gap_select_decel_mps2_{1.5};
  double gap_select_effort_weight_{0.2};
  double gap_select_switch_cost_sec_{1.5};
  double gap_select_match_dist_m_{6.0};
  double gap_select_hold_sec_{0.6};
  int gap_select_switch_confirm_frames_{3};
  double gap_select_effort_switch_m_{10.0};
  double gap_select_cmd_rate_mps2_{3.0};
  std::string gap_select_side_policy_{"speed"};
  bool gap_select_own_lead_enabled_{true};
  double gap_select_own_lead_max_gap_m_{80.0};
  double gap_select_own_lead_standstill_m_{6.0};
  double gap_select_own_lead_time_gap_sec_{0.6};
  double gap_select_own_lead_gain_{0.3};
  double gap_select_own_lead_overtake_margin_mps_{2.0};
  double gap_select_side_speed_deadband_mps_{1.0};
  int gap_side_vehicle_id_{-1};
  int gap_side_{0};
  int gap_rule_side_{0};
  int gap_rule_vehicle_id_{-1};
  double gap_rule_vehicle_speed_{0.0};
  double gap_last_filtered_cmd_{0.0};
  ros::Time gap_last_cmd_stamp_;
  double gap_select_ignore_stationary_behind_mps_{2.0};
  bool gap_pending_valid_{false};
  int gap_pending_follower_id_{-1};
  int gap_pending_leader_id_{-1};
  int gap_pending_count_{0};
  mutable std::unordered_map<int, GapSpeedHistory> gap_speed_history_;
  bool gap_latch_valid_{false};
  double gap_latch_target_pos_{0.0};
  double gap_latch_gap_speed_{0.0};
  ros::Time gap_latch_stamp_;
  bool gap_last_command_valid_{false};
  double gap_last_speed_cmd_{0.0};
  double gap_last_align_time_{0.0};
  int last_behavior_merge_leader_id_{-1}, last_behavior_merge_follower_id_{-1};
  int gap_latch_follower_id_{-1};
  int gap_latch_leader_id_{-1};
  int gap_latch_current_lane_id_{-1};
  int gap_latch_target_lane_id_{-1};
  bool lane_deadline_midpoint_enabled_{true};
  double lane_deadline_blend_fraction_{0.5};
  double lane_deadline_margin_m_{5.0};
  bool lane_end_standstill_merge_enabled_{true};
  double lane_end_standstill_merge_max_speed_mps_{1.0};
  double lane_end_standstill_rear_min_gap_m_{20.0};
  double lane_end_standstill_rear_min_ttc_sec_{2.5};
  // 차선변경 예측 속도의 앞차 추종 상한(leadFollowSpeedCapMps).
  double lead_follow_gap_gain_{0.4};
  double lead_follow_min_cap_mps_{5.0};
  // 속도 미신뢰 시 간격 기반 폴백(projectedTargetGapTtcSafe check_vehicle).
  bool unknown_velocity_gap_fallback_enabled_{true};
  double unknown_velocity_worst_closing_mps_{4.0};
  double behavior_hold_accel_mps2_{0.0};
  double behavior_accel_accel_mps2_{1.0};
  std::vector<double> behavior_yield_decel_candidates_mps2_{-0.8};
  double behavior_delay_cost_weight_{1.0};
  double behavior_accel_cost_weight_{0.25};
  std::vector<double> behavior_lc_start_times_sec_{0.4, 0.8, 1.2, 1.6, 2.0};
  std::string behavior_status_topic_{"/smpc/behavior_longitudinal_status"};
  double behavior_status_timeout_sec_{0.35};
  uint32_t behavior_plan_id_{0};
  uint32_t last_behavior_plan_id_{0};
  uint32_t behavior_command_revision_{0};
  uint32_t last_behavior_command_plan_id_{0};
  uint32_t last_behavior_command_revision_{0};
  uint32_t previous_behavior_command_plan_id_{0};
  uint32_t previous_behavior_command_revision_{0};
  BehaviorAction last_behavior_action_{BehaviorAction::kKeep};
  double behavior_yield_episode_start_speed_mps_{
      std::numeric_limits<double>::quiet_NaN()};
  int last_behavior_current_lane_id_{std::numeric_limits<int>::min()};
  int last_behavior_target_lane_id_{std::numeric_limits<int>::min()};
  BehaviorAction last_behavior_command_action_{BehaviorAction::kKeep};
  bool last_behavior_command_active_{false};
  bool previous_behavior_command_active_{false};
  int last_behavior_command_current_lane_id_{std::numeric_limits<int>::min()};
  int last_behavior_command_target_lane_id_{std::numeric_limits<int>::min()};
  double last_behavior_command_lane_change_start_sec_{
      std::numeric_limits<double>::quiet_NaN()};
  double last_behavior_command_speed_cap_mps_{
      std::numeric_limits<double>::quiet_NaN()};
  double previous_behavior_command_speed_cap_mps_{
      std::numeric_limits<double>::quiet_NaN()};

  double front_brake_probability_{0.25};
  double rear_accel_probability_{0.25};
  double min_front_brake_probability_{0.05};
  double max_front_brake_probability_{0.65};
  double min_rear_accel_probability_{0.05};
  double max_rear_accel_probability_{0.65};
  double front_brake_accel_mps2_{-2.0};
  double rear_accel_mps2_{1.5};
  // NPC 종방향 이벤트 모드(앞차 급제동/뒤차 급가속) 사용 여부.
  bool npc_longitudinal_event_modes_enabled_{false};
  // projectedTargetGapTtcSafe 의 예측 창(초). 0 이하면 lane_change_duration 사용.
  double projected_gap_ttc_horizon_sec_{2.0};
  double nominal_target_accel_mps2_{0.0};
  double ego_assumed_accel_mps2_{0.0};
  bool ego_prediction_use_mission_speed_rollout_{true};
  double ego_prediction_max_accel_mps2_{2.5};
  double ego_prediction_max_decel_mps2_{2.0};
  double probability_ttc_safe_sec_{5.0};
  double probability_ttc_critical_sec_{1.5};
  double probability_rel_speed_scale_mps_{8.0};
  double probability_accel_scale_mps2_{3.0};
  double observed_accel_alpha_{0.35};
  double max_observed_accel_mps2_{6.0};
  double observed_accel_max_velocity_std_{0.8};

  bool lateral_mode_enabled_{true};
  double lateral_mode_probability_{0.03};
  double max_lateral_mode_probability_{0.08};
  double lateral_mode_v_threshold_mps_{0.90};
  double yaw_lateral_threshold_rad_{0.28};
  double yaw_lateral_probability_gain_{0.04};
  double max_lateral_prediction_mps_{0.45};
  bool lateral_mode_allow_nearby_{false};
  bool lateral_mode_allow_rear_{false};
  bool lateral_mode_allow_yaw_only_{false};
  double lateral_mode_min_abs_d_m_{1.40};
  bool lateral_mode_require_yaw_agreement_{true};
  bool lateral_mode_require_away_from_center_{true};
  double tv_yaw_rate_weight_{0.05};
  double tv_csv_yaw_weight_{0.90};
  double tv_no_lateral_probability_floor_{0.90};
  double tv_yaw_rate_horizon_sec_{0.8};
  double max_abs_tv_yaw_rate_{0.6};

  double prediction_sigma_base_m_{0.5};
  double prediction_sigma_growth_mps_{0.35};
  double prediction_lateral_sigma_base_m_{0.25};
  double prediction_lateral_sigma_growth_mps_{0.12};
  double collision_margin_m_{0.45};

  double lane_change_base_cost_{0.5};
  double risk_cost_weight_{80.0};
  double gap_cost_weight_{2.5};
  double blocked_cost_weight_{35.0};
  double lane_end_prepare_cost_{60.0};
  double lane_end_urgent_cost_{180.0};
  double lane_end_emergency_cost_{500.0};
  double decision_hysteresis_cost_{0.1};
  // 커밋 히스테리시스(위 commit_bias 참고).
  bool commit_hysteresis_enabled_{true};
  double commit_bias_cost_{7.0};
  // 목표 차로 차량 속도 추정 신뢰도 게이트(leadTrustedLongSpeed / velocityStdMargin).
  bool trusted_speed_gate_enabled_{true};
  double trusted_speed_max_std_mps_{1.0};
  double trusted_speed_std_margin_scale_{1.0};
  double untrusted_speed_std_margin_scale_{2.0};
  double commit_hold_sec_{2.0};
  int commit_target_lane_id_{-1};
  ros::Time commit_stamp_;
  bool debug_publish_marker_{true};
  double debug_marker_lifetime_sec_{0.5};
  double debug_marker_height_m_{4.0};
  bool debug_prediction_publish_markers_{true};
  double debug_prediction_marker_lifetime_sec_{0.5};
  double debug_prediction_line_width_m_{0.25};
  double debug_prediction_z_m_{0.35};
  double debug_prediction_footprint_height_m_{0.15};
  int debug_prediction_ego_footprint_stride_{5};
  int debug_prediction_target_footprint_stride_{5};
  int debug_prediction_target_max_vehicles_{20};
  bool debug_prediction_show_target_modes_{true};
  bool debug_prediction_show_labels_{true};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "smpc_decision");
  SmpcDecisionNode node;
  ros::spin();
  return 0;
}
