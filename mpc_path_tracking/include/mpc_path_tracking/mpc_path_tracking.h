#ifndef ROSCONTROL_H
#define ROSCONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <waypoint_maker/Lane.h>
#include <waypoint_maker/Waypoint.h>
#include <waypoint_maker/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <std_msgs/Int32.h>	
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

#include <std_msgs/Float32.h>
#include <mission_supervisor/SectionState.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <cmath>
#include <math.h>
#include <numeric>
#include <array>
#include <string>

#include <sensor_msgs/Imu.h>
#include <morai_msgs/CtrlCmd.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/CollisionData.h>
#include <new_platform_control/new_platform_control.h>

#include <new_lane/lidar_topic_msg.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include "ros/time.h"

#include <mpc/mpc.h>
#include <mpc_path_tracking/steering_delay.h>

#define INTER_TIME_PLUS 1000000000
#define INTER_TIME_MIN 90000000
#define INTER_TIME_MAX 200000000
#define INTER_SPEED_TIME_MAX 3600000000
#define SOME_MIN 0.01
#define SOME_MAX 0.15

class ROSCONTROL: public MPC {
	
protected:
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

    const double WHEEL_BASE = 3.00;
	const double MAX_SEARCH_DIST = 10.0;

    double ex_x_, ex_y_;
    unsigned int ex_time_;
    int inter_time_;

    double cur_course_;
    double cur_speed_;
	double cur_accel_;

	double v_lpf_alpha_;            
	double v_diff_thresh_;          // |v_twist - v_pos| 의심 임계값 (m/s)
	double v_jump_thresh_;          // twist 점프 임계값
	double twist_max_;              // twist 최대 허용
	double teleport_dist_thresh_;   // 순간이동 의심 거리 (m)

	bool have_v_filt_;
	double v_filt_;

	bool have_prev_twist_;
	double prev_twist_;

    int waypoints_size_;
	int waypoint_min_;
	int closest_waypoint_;

    bool is_pose_;
    bool is_lane_;
    bool is_course_;
	bool is_control_;

	double dist_;
	int current_mission_state_;
	int loader_number_;

	double cte_cost_weight_info;
		double epsi_cost_weight_info;
		double v_weight_info;
		double delta_cost_weight_info;
		double v_target_cost_weight_info;
	double delta_change_cost_weight_info;
	double v_target_change_cost_weight_info;
	double referance_v_info;
	double k_v_info;
	double dt_info;
	int mpc_step_info;

	int get_gear;
    int get_speed;
    int get_steer;
    int get_brake;
	double Get_Kmh_Speed_;
	double Get_MPH_Speed_;
	const double GEAR_RATIO = 5.0;
    const double WHEEL_RADIOUS = 0.29;
	double Get_steer_;

	double ref_v_nominal_{0.0};          // 기본 목표 속도(m/s)
	double speed_upper_nominal_{0.0};    // 기본 속도 상한(m/s)
	double speed_lower_nominal_{0.0};    // 기본 속도 하한(m/s)
	double speed_limit_urban_max_mps_{15.0};   // 도심 최고속도 상한(m/s)
	double speed_limit_highway_max_mps_{25.0}; // 고주로 최고속도 상한(m/s)

	bool is_acc;
	int acc_index;

	// External real-distance ACC speed limit from smpc_lane_change.
	bool external_acc_enabled_{true};
	double external_acc_target_speed_mps_{0.0};
	double external_acc_timeout_sec_{0.5};
	bool have_external_acc_target_{false};
	ros::Time external_acc_target_stamp_;

    geometry_msgs::PoseStamped cur_pose_;

    vector<waypoint_maker::Waypoint> waypoints_;
    vector<waypoint_maker::Waypoint> speed_waypoints_;

		ros::Publisher ackermann_pub_;
		ros::Publisher rviz_predicted_path_;
		ros::Publisher vis_pub_;
		ros::Publisher mpc_target_speed_pub_;
		ros::Publisher curvature_speed_limit_pub_;
        ros::Publisher curvature_capacity_speed_pub_;
		ros::Publisher path_curvature_for_speed_limit_pub_;
		ros::Publisher mission_speed_target_pub_;
		ros::Publisher is_highway_mode_pub_;

	ros::Subscriber odom_sub_;
	ros::Subscriber lane_sub_;
	ros::Subscriber final_waypoint_sub_;
	ros::Subscriber state_sub_;
    ros::Subscriber lidar_topic_sub_;
	ros::Subscriber external_acc_speed_sub_;
	ros::Subscriber erp42_status_sub_;
		ros::Subscriber competition_sub_;
		ros::Subscriber avoidance_speed_sub_;
		ros::Subscriber static_speed_scale_sub_;
		ros::Subscriber active_section_sub_;

		bool in_exempt_section_{false};
		std::string active_section_id_;
        std::string avoidance_section_key_;
		ros::Time last_active_section_time_;
		bool have_active_section_{false};

		// obstacle_avoidance_node가 회피/복귀 중일 때만 target_speed를 실어 보내는
		// 토픽. min(ref_v, 이 값)으로 curvature_speed와 같은 방식으로 적용한다.
		double avoidance_speed_limit_mps_{9999.0};
		ros::Time last_avoidance_speed_time_;
		bool have_avoidance_speed_{false};

		// 정적/unknown 장애물이 confirmed 되는 즉시(회피 판단과 무관하게) 원래
		// 속도의 이 배율로 감속. 복잡한 로직 없이 ref_v에 단순 곱셈으로 적용.
		double static_speed_scale_{9999.0};
		ros::Time last_static_speed_scale_time_;
		bool have_static_speed_scale_{false};
		// 회피 진입 순간(rising edge)에만 target_speed로 즉시 스냅하기 위한 이전 상태 기억.
		// 회피 -> 평소 복귀는 기존 방식(점진적) 그대로 둔다.
		bool avoidance_speed_limit_was_active_{false};
		bool static_speed_scale_was_active_{false};

	morai_msgs::CtrlCmd ackermann_msg_;
	PlatformConnector platform_;

	tf::TransformBroadcaster tf_broadcaster_; // TF Broadcaster 선언

	// MPC Variables
	float steer;

	float velocity_actual;
    float steering_angle;
	double target_velocity;

	double input_steer_;
	double input_speed_;
	double rk_;
	int prev_mission_state_;

		// MORAI longitudinal-control configuration and feedback.
	int longitudinal_command_type_{1};
	double throttle_kp_{0.25};
	double throttle_ki_{0.05};
	double throttle_integral_limit_{4.0};
		double throttle_accel_limit_{0.60};
		double throttle_brake_limit_{0.60};
		// static_speed_scale(카메라/라이다 감속 킥) 걸릴 때만 쓰는 별도 브레이크 세기.
		// 평소 순항속도 변화(커브 등)는 부드럽게, 킥은 더 세게 밟고 싶을 때 사용.
		double kick_brake_limit_{0.60};
		// M6/M7(고주로, speed_limit_exempt) 구간 전용 가속 페달 상한. 도심 값(0.60)은
		// M2/M3 저속(36km/h) 기준으로 검증됐고, 그 값 그대로면 고주로 목표(25 m/s)까지
		// 가속이 못 따라간다.
		double throttle_accel_limit_exempt_{1.0};
		// 브레이크가 한 사이클 만에 확 튀는(노이즈성) 걸 막기 위한 초당 최대 증가량.
		// obstacle_stop_node의 실제 위험 정지는 이 경로를 안 거치므로 안전에 영향 없음.
		double brake_rate_limit_per_sec_{3.0};
		double last_commanded_brake_{0.0};
		double throttle_integral_{0.0};
		double throttle_integral_band_mps_{0.0};
		double throttle_pedal_deadband_{0.0};
		// 목표속도 램프 피드포워드(0 = 끔). 페달 = gain * 목표가속도 / (페달 1.0 의 실측 가속도).
		double throttle_ff_gain_{0.0};
		double throttle_ff_accel_per_pedal_{3.5};
		double throttle_ff_tau_s_{0.3};
		double throttle_ff_min_accel_{0.6};
		double throttle_ff_taper_mps_{1.0};
		double throttle_ff_steer_zero_rad_{0.10};
		// 규정 3-2 속도 보호: 속도 예외 구간(determineIsHighway) 밖에서 실제 속도가 cut 를 넘으면 가속 0,
		// brake_at 을 넘으면 brake 를 건다. 60 km/h 를 한 번만 넘어도 15 s 패널티라 목표와 무관하게 막는다.
		// (speed_limit/hard_limit_* 는 control_arbiter 용인데 대회 launch 는 arbiter 를 띄우지 않는다.)
		bool legal_guard_enabled_{true};
		double legal_guard_cut_mps_{16.2};
		double legal_guard_brake_at_mps_{16.4};
		double legal_guard_brake_{0.3};
		double ff_prev_target_mps_{-1.0};
		double ff_target_accel_{0.0};
		double steering_command_sign_{-1.0};
		double steering_command_rate_limit_radps_{1.0};
		bool steering_output_lpf_enabled_{false};
		double steering_output_lpf_alpha_{1.0};
		double steering_output_lpf_alpha_high_speed_{0.75};
		double previous_steering_command_rad_{0.0};
		bool have_previous_steering_command_{false};
		// 조향 지연보상(민성 0929 이식). 0 이면 꺼짐(기존 0.05 s 단일 예측만 사용).
		// 매 주기 getParamCached 로 다시 읽으므로 rosparam set 으로 실행 중 바꿀 수 있다.
		double steering_actuator_delay_s_{0.0};
		// 이력 적분에 쓸 Lf. 0 이하면 MPC 모델 Lf 를 그대로 쓴다.
		double steering_delay_model_lf_{0.0};
		std::deque<mpc_tracking::TimedSteering> steering_history_;
		ros::Publisher steering_delay_pub_;
		// 지연보상 v2(조건 불변): 상태 지연(실측) + MPC 계산 지연(실측) + 조향 지연(τ0 + 프레임 대기).
		// delay_comp/enabled 가 true 면 위의 고정 τ(steer/actuator_delay_s) 경로 대신 이것을 쓴다.
		bool dc_enabled_{false};
		double dc_state_k_{1.0}, dc_state_c_s_{0.010}, dc_state_max_s_{0.12};
		double dc_act_base_s_{0.06}, dc_act_frame_k_{0.5}, dc_model_lf_{4.2};
		double dc_fallback_period_s_{0.020}, dc_compute_max_s_{0.10};
		double dc_period_s_{-1.0}, dc_last_imu_stamp_{0.0};
		std::deque<double> dc_intervals_;
		// 실효 Lf 실시간 측정: MORAI 상태 패킷(같은 패킷의 실제 바퀴각·요레이트·속도)으로
		// Lf = v*delta/yaw_rate 를 곡선에서만 천천히 갱신한다. 차량 고유값이라 조건과 무관해야 한다.
		bool dc_lf_online_{true};
		double dc_test_extra_compute_s_{0.0};   // 시험 전용: 느린 PC 의 계산 시간을 흉내 낸다(기본 0)
		double dc_lf_est_{-1.0}, dc_lf_x_{0.0}, dc_lf_y_{0.0}, dc_lf_last_stamp_{0.0};
		ros::Time dc_last_imu_rx_;
		double dc_compute_s_{0.03};
		ros::WallTime dc_pred_wall_;
		bool dc_pred_pending_{false};
		ros::Time last_odom_stamp_;
		double odom_yaw_rate_{0.0};
		// 상태 입력: odom = ekf_odom_output 가 자기 50 Hz 타이머로 다시 찍어 내는 /odom,
		// ekf = 같은 값을 EKF 원본 토픽에서 바로 받는다(재발행 대기 8~18 ms 가 없고 stamp 가 EKF 계산 시각).
		std::string state_source_{"odom"};
		ros::Subscriber ekf_pose_sub_, ekf_twist_sub_;
		bool ekf_twist_ok_{false};
		double ekf_yaw_rate_{0.0};
		double ekf_vx_{0.0};
		// MPC 입력 위치 필터(state_filter/enabled). GPS(NMEA)가 0.15~0.19 m 계단으로 와서 생기는 EKF 위치·헤딩
		// 떨림을 직선에서만 거른다: 차속·요레이트로 적분하고 횡위치·헤딩만 EKF 쪽으로 tau 로 맞춘다(진행방향은 EKF).
		// 커브·회전 중, 정지·후진, GPS 음영/복귀 중에는 EKF 값을 그대로 쓴다.
		bool sf_enabled_{false};
		double sf_pos_tau_s_{1.0}, sf_yaw_tau_s_{1.0};
		double sf_kappa_full_{0.004}, sf_kappa_zero_{0.012}, sf_preview_s_{2.5};
		double sf_wz_full_{0.03}, sf_wz_zero_{0.10};
		double sf_s_up_rate_{0.5}, sf_s_last_t_{0.0};
		double sf_max_lat_m_{0.04}, sf_max_yaw_rad_{0.3 * M_PI / 180.0};
		double sf_reset_pos_m_{0.3}, sf_reset_yaw_rad_{3.0 * M_PI / 180.0};
		bool sf_init_{false};
		double sf_x_{0.0}, sf_y_{0.0}, sf_yaw_{0.0}, sf_last_stamp_{0.0}, sf_straightness_{0.0};
		int sf_reset_count_{0};
		std::string gps_mode_{"normal"};
		ros::Subscriber gps_mode_sub_;
		ros::Publisher sf_debug_pub_;
		ros::Subscriber dc_imu_sub_;
		ros::Publisher dc_debug_pub_;
		bool have_competition_feedback_{false};
		double competition_speed_mps_{0.0};
		double competition_wheel_angle_deg_{0.0};
		bool mission_speed_enabled_{true};
		double mission_speed_default_mps_{0.0};
		std::array<double, 13> mission_speed_state_mps_{};
		double mission_speed_rate_limit_up_mps2_{1.8};
		double mission_speed_rate_limit_down_mps2_{2.2};
		double filtered_mission_speed_target_mps_{0.0};
		bool have_filtered_mission_speed_target_{false};
		bool curvature_speed_enabled_{true};
		double curvature_lateral_accel_limit_mps2_{2.3};
        double urban_curvature_lateral_accel_limit_mps2_{2.3};
        double urban_curvature_speed_rate_up_mps2_{1.0};
		double avoidance_lateral_accel_limit_mps2_{3.0};
		double avoidance_curvature_min_speed_mps_{3.0};
		// 회피 곡률 속도 프로파일을 쓰는 구간(M2, 음영구간 M8).
		std::vector<std::string> avoidance_sections_{"M2", "M8"};
		double curvature_min_speed_mps_{3.5};
		double curvature_max_speed_mps_{0.0};
		double curvature_preview_distance_m_{28.0};
		double preview_distance_urban_m_{28.0};
		double preview_distance_highway_m_{50.0};
		double curvature_min_lookahead_m_{0.5};
		double curvature_deadband_inv_m_{0.01};
		double curvature_speed_rate_up_mps2_{1.0};
		double curvature_speed_rate_down_mps2_{3.0};
		int curvature_sample_count_{40};
		double curvature_percentile_{0.90};
		bool curvature_use_waypoint_rk_{true};
		bool curvature_fallback_to_polyfit_{true};
		bool have_filtered_curve_speed_limit_{false};
		double filtered_curve_speed_limit_mps_{0.0};
		double last_curve_speed_limit_mps_{0.0};
		double last_path_curvature_for_speed_limit_{0.0};
		bool path_end_stop_enabled_{true};
		int path_end_min_polyfit_points_{4};
		// 경로 끝 접근 감속(위 modelPredictiveController 참고).
		// trigger 는 /local_path 창 길이(약 45 m)보다 짧아야 평상시 주행에
		// 영향이 없다. decel 1.5 m/s^2 면 14.9 m/s 에서 약 74 m 가 필요한데,
		// 창이 45 m 뿐이라 초기 clamp 가 다소 급해진다 - 그래도 끝점 0.3 m
		// 앞에서 처음 제동하는 기존 동작보다는 훨씬 완만하다.
		double path_end_approach_trigger_m_{40.0};
		double path_end_approach_decel_mps2_{1.5};
		double path_end_stop_margin_m_{1.0};

		double missionStateTargetSpeedMps() const;
		bool determineIsHighway() const;

public:
	ros::Time ex_stamp;
    ROSCONTROL() 
	{
		initSetup();
	}

	~ROSCONTROL() 
	{
		waypoints_.clear();
	}

    void initSetup()
    {
		ros::Time::init();
		MPC mpc;
		
        ex_x_ = 0.0;
		ex_y_ = 0.0;
		ex_time_ = 0;
        inter_time_ = 0;

        cur_course_ = .0;
        cur_speed_ = .0;

        waypoints_size_ = 0;
		waypoint_min_ = -1;
		closest_waypoint_ = -1;

        is_pose_ = false;
        is_lane_ = false;
        is_course_ = false;
		is_control_ = false;

		dist_ = 100.0;
		current_mission_state_ = -1;
		loader_number_ = 0;

		steer = 0.0;
		steering_angle = 0.0;
		target_velocity = 0.0;

		input_speed_ = 0.0;
		input_steer_ = 0.0;
		rk_ = 0.0;
			prev_mission_state_ = 0;
			is_acc = false;
			acc_index = 0;

		ref_v_nominal_ = ref_v;
		speed_upper_nominal_ = speed_upperBound;
		speed_lower_nominal_ = speed_lowerBound;

		nh_.param<double>("speed_limit/urban_max_mps", speed_limit_urban_max_mps_, 15.0);
		nh_.param<double>("urban_control/max_speed_mps", speed_limit_urban_max_mps_, speed_limit_urban_max_mps_);
		nh_.param<double>("speed_limit/highway_max_mps", speed_limit_highway_max_mps_, 25.0);
		nh_.param<double>("highway_control/max_speed_mps", speed_limit_highway_max_mps_, speed_limit_highway_max_mps_);
		ROS_INFO("[MPC] speed limits configured: urban=%.1f m/s (%.1f km/h), highway=%.1f m/s (%.1f km/h)",
				 speed_limit_urban_max_mps_, speed_limit_urban_max_mps_ * 3.6,
				 speed_limit_highway_max_mps_, speed_limit_highway_max_mps_ * 3.6);

		nh_.param<bool>("external_acc/enabled", external_acc_enabled_, true);
		nh_.param<double>("external_acc/timeout_sec", external_acc_timeout_sec_, 0.5);
		nh_.param<bool>("path_end_stop/enabled", path_end_stop_enabled_, true);
		nh_.param<int>("path_end_stop/min_polyfit_points", path_end_min_polyfit_points_, 4);
		nh_.param<double>("path_end_stop/approach_trigger_m", path_end_approach_trigger_m_, 40.0);
		nh_.param<double>("path_end_stop/approach_decel_mps2", path_end_approach_decel_mps2_, 1.5);
		nh_.param<double>("path_end_stop/stop_margin_m", path_end_stop_margin_m_, 1.0);
		if (path_end_min_polyfit_points_ < 4) {
			ROS_WARN("[MPC] path_end_stop/min_polyfit_points=%d is too small for cubic polyfit; using 4",
					 path_end_min_polyfit_points_);
			path_end_min_polyfit_points_ = 4;
		}

		ackermann_pub_ = nh_.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 1);
		nh_.param<int>("longitudinal/command_type", longitudinal_command_type_, 1);
		nh_.param<double>("longitudinal/throttle_kp", throttle_kp_, 0.25);
		nh_.param<double>("longitudinal/throttle_ki", throttle_ki_, 0.05);
			nh_.param<double>("longitudinal/throttle_integral_limit", throttle_integral_limit_, 4.0);
			nh_.param<double>("longitudinal/throttle_accel_limit", throttle_accel_limit_, 0.60);
			nh_.param<double>("longitudinal/throttle_accel_limit_exempt", throttle_accel_limit_exempt_, 1.0);
			nh_.param<double>("longitudinal/throttle_brake_limit", throttle_brake_limit_, 0.60);
			nh_.param<double>("longitudinal/kick_brake_limit", kick_brake_limit_, 0.60);
			nh_.param<double>("longitudinal/brake_rate_limit_per_sec", brake_rate_limit_per_sec_, 3.0);
			nh_.param<double>("longitudinal/throttle_pedal_deadband", throttle_pedal_deadband_, 0.0);
			nh_.param<double>("longitudinal/throttle_integral_band_mps", throttle_integral_band_mps_, 0.0);
			nh_.param<double>("longitudinal/throttle_ff_gain", throttle_ff_gain_, 0.0);
			nh_.param<double>("longitudinal/throttle_ff_accel_per_pedal_mps2", throttle_ff_accel_per_pedal_, 3.5);
			nh_.param<double>("longitudinal/throttle_ff_tau_s", throttle_ff_tau_s_, 0.3);
			nh_.param<double>("longitudinal/throttle_ff_min_accel_mps2", throttle_ff_min_accel_, 0.6);
			nh_.param<double>("longitudinal/throttle_ff_taper_mps", throttle_ff_taper_mps_, 1.0);
			throttle_ff_taper_mps_ = std::max(0.1, throttle_ff_taper_mps_);
			nh_.param<double>("longitudinal/throttle_ff_steer_zero_rad", throttle_ff_steer_zero_rad_, 0.10);
			throttle_ff_steer_zero_rad_ = std::max(0.01, throttle_ff_steer_zero_rad_);
			nh_.param<bool>("longitudinal/legal_guard/enabled", legal_guard_enabled_, true);
			nh_.param<double>("longitudinal/legal_guard/cut_mps", legal_guard_cut_mps_, 16.2);
			nh_.param<double>("longitudinal/legal_guard/brake_at_mps", legal_guard_brake_at_mps_, 16.4);
			nh_.param<double>("longitudinal/legal_guard/brake", legal_guard_brake_, 0.3);
			throttle_ff_accel_per_pedal_ = std::max(0.5, throttle_ff_accel_per_pedal_);
			throttle_ff_tau_s_ = std::max(0.0, throttle_ff_tau_s_);
		nh_.param<double>("steer/command_sign", steering_command_sign_, -1.0);
		nh_.param<double>("steer/command_rate_limit_radps", steering_command_rate_limit_radps_, 1.0);
		nh_.param<bool>("steer/output_lpf_enabled", steering_output_lpf_enabled_, false);
		nh_.param<double>("steer/output_lpf_alpha", steering_output_lpf_alpha_, 1.0);
		steering_output_lpf_alpha_ = clampValue(steering_output_lpf_alpha_, 0.0, 1.0);
		nh_.param<double>("steer/output_lpf_alpha_high_speed", steering_output_lpf_alpha_high_speed_, 0.75);
		steering_output_lpf_alpha_high_speed_ = clampValue(steering_output_lpf_alpha_high_speed_, 0.0, 1.0);
		nh_.param<double>("steer/actuator_delay_s", steering_actuator_delay_s_, 0.0);
		steering_actuator_delay_s_ = clampValue(steering_actuator_delay_s_, 0.0, 0.5);
		nh_.param<double>("steer/delay_model_lf", steering_delay_model_lf_, 0.0);
		ROS_INFO("[MPC] steering delay compensation: delay=%.3f s (0=off), integration Lf=%.2f",
				 steering_actuator_delay_s_,
				 steering_delay_model_lf_ > 0.0 ? steering_delay_model_lf_ : Lf);
		nh_.param<bool>("mission_speed/enabled", mission_speed_enabled_, true);
		nh_.param<double>("mission_speed/default_mps", mission_speed_default_mps_, ref_v_nominal_);
		mission_speed_state_mps_.fill(mission_speed_default_mps_);
		for (size_t i = 0; i < mission_speed_state_mps_.size(); ++i) {
			nh_.param<double>("mission_speed/state_" + std::to_string(i) + "_mps",
							  mission_speed_state_mps_[i],
							  mission_speed_default_mps_);
		}
		nh_.param<double>("mission_speed/rate_limit_up_mps2", mission_speed_rate_limit_up_mps2_, 1.8);
		nh_.param<double>("mission_speed/rate_limit_down_mps2", mission_speed_rate_limit_down_mps2_, 2.2);
		nh_.param<bool>("curvature_speed/enabled", curvature_speed_enabled_, true);
		nh_.param<double>("curvature_speed/lateral_accel_limit_mps2", curvature_lateral_accel_limit_mps2_, 2.3);
		nh_.param<double>("avoidance_control/lateral_accel_limit_mps2", avoidance_lateral_accel_limit_mps2_, 3.0);
		nh_.param<double>("avoidance_control/curvature_min_speed_mps", avoidance_curvature_min_speed_mps_, 3.0);
		nh_.param("avoidance_control/sections", avoidance_sections_, std::vector<std::string>{"M2", "M8"});
		nh_.param<double>("curvature_speed/min_speed_mps", curvature_min_speed_mps_, 3.5);
		nh_.param<double>("curvature_speed/max_speed_mps", curvature_max_speed_mps_, 0.0);
		nh_.param<double>("urban_control/preview_distance_m", preview_distance_urban_m_, 28.0);
		nh_.param<double>("highway_control/preview_distance_m", preview_distance_highway_m_, 50.0);
		nh_.param<double>("curvature_speed/preview_distance_m", curvature_preview_distance_m_, preview_distance_urban_m_);
		nh_.param<double>("curvature_speed/min_lookahead_m", curvature_min_lookahead_m_, 0.5);
		nh_.param<double>("curvature_speed/deadband_inv_m", curvature_deadband_inv_m_, 0.01);
		nh_.param<double>("curvature_speed/rate_limit_up_mps2", curvature_speed_rate_up_mps2_, 1.0);
        nh_.param<double>("urban_control/curvature_lateral_accel_limit_mps2",
                          urban_curvature_lateral_accel_limit_mps2_, curvature_lateral_accel_limit_mps2_);
        nh_.param<double>("urban_control/curvature_rate_limit_up_mps2",
                          urban_curvature_speed_rate_up_mps2_, curvature_speed_rate_up_mps2_);
		nh_.param<double>("curvature_speed/rate_limit_down_mps2", curvature_speed_rate_down_mps2_, 3.0);
		nh_.param<int>("curvature_speed/sample_count", curvature_sample_count_, 40);
		nh_.param<double>("curvature_speed/percentile", curvature_percentile_, 0.90);
		nh_.param<bool>("curvature_speed/use_waypoint_rk", curvature_use_waypoint_rk_, true);
		nh_.param<bool>("curvature_speed/fallback_to_polyfit", curvature_fallback_to_polyfit_, true);
		curvature_percentile_ = clampValue(curvature_percentile_, 0.0, 1.0);
			rviz_predicted_path_ = nh_.advertise<nav_msgs::Path>("/rviz_predicted_path", 1);
			vis_pub_ = nh_.advertise<visualization_msgs::Marker>("pose",1);
			mpc_target_speed_pub_ = nh_.advertise<std_msgs::Float64>(
				"/control_feedback/mpc_target_speed_mps", 10);
			curvature_speed_limit_pub_ = nh_.advertise<std_msgs::Float64>(
				"/control_feedback/curvature_speed_limit_mps", 10);
            curvature_capacity_speed_pub_ = nh_.advertise<std_msgs::Float64>(
                "/control_feedback/curvature_capacity_speed_mps", 10);
			path_curvature_for_speed_limit_pub_ = nh_.advertise<std_msgs::Float64>(
				"/control_feedback/path_curvature_for_speed_limit", 10);
			mission_speed_target_pub_ = nh_.advertise<std_msgs::Float64>(
				"/control_feedback/mission_speed_target_mps", 10);
			is_highway_mode_pub_ = nh_.advertise<std_msgs::Bool>(
				"/control_feedback/is_highway_mode", 10);
			steering_delay_pub_ = nh_.advertise<std_msgs::Float64>(
				"/control_feedback/steering_delay_s", 10);
			// [period_s, odom_age_s, state_delay_s, compute_s, actuation_s, enabled]
			dc_debug_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(
				"/control_feedback/delay_comp", 10);
			nh_.param<bool>("delay_comp/enabled", dc_enabled_, false);
			nh_.param<double>("delay_comp/state_k", dc_state_k_, 1.0);
			nh_.param<double>("delay_comp/state_c_s", dc_state_c_s_, 0.010);
			nh_.param<double>("delay_comp/state_max_s", dc_state_max_s_, 0.12);
			nh_.param<double>("delay_comp/actuation_base_s", dc_act_base_s_, 0.06);
			nh_.param<double>("delay_comp/actuation_frame_k", dc_act_frame_k_, 0.5);
			nh_.param<double>("delay_comp/model_lf", dc_model_lf_, 4.2);
			nh_.param<double>("delay_comp/fallback_period_s", dc_fallback_period_s_, 0.020);
			nh_.param<double>("delay_comp/compute_max_s", dc_compute_max_s_, 0.10);
			nh_.param<bool>("delay_comp/lf_online", dc_lf_online_, true);
			nh_.param<double>("delay_comp/test_extra_compute_s", dc_test_extra_compute_s_, 0.0);
			dc_imu_sub_ = nh_.subscribe("/imu", 50, &ROSCONTROL::DelayImuCallback, this);
			ROS_INFO("[MPC] delay_comp %s: d_state=%.2f*T+%.3f+odom_age (max %.2f), tau_act=%.3f+%.2f*T, Lf=%.2f, fallback T=%.3f",
					 dc_enabled_ ? "ON" : "off", dc_state_k_, dc_state_c_s_, dc_state_max_s_,
					 dc_act_base_s_, dc_act_frame_k_, dc_model_lf_, dc_fallback_period_s_);

		nh_.param<std::string>("state_input/source", state_source_, std::string("odom"));
		if (state_source_ == "ekf") {
			ekf_pose_sub_ = nh_.subscribe("/ekf_pose_with_covariance", 1, &ROSCONTROL::EkfPoseCallback, this);
			ekf_twist_sub_ = nh_.subscribe("/ekf_twist_with_covariance", 1, &ROSCONTROL::EkfTwistCallback, this);
		} else {
			state_source_ = "odom";
			odom_sub_ = nh_.subscribe("/odom", 1, &ROSCONTROL::OdomCallback, this);
		}
		ROS_INFO("[MPC] state input: %s", state_source_ == "ekf"
				 ? "EKF raw (/ekf_pose_with_covariance + /ekf_twist_with_covariance)" : "/odom");
		nh_.param<bool>("state_filter/enabled", sf_enabled_, false);
		nh_.param<double>("state_filter/pos_tau_s", sf_pos_tau_s_, 1.0);
		nh_.param<double>("state_filter/yaw_tau_s", sf_yaw_tau_s_, 1.0);
		nh_.param<double>("state_filter/kappa_full", sf_kappa_full_, 0.004);
		nh_.param<double>("state_filter/kappa_zero", sf_kappa_zero_, 0.012);
		nh_.param<double>("state_filter/preview_s", sf_preview_s_, 2.5);
		nh_.param<double>("state_filter/yaw_rate_full_radps", sf_wz_full_, 0.03);
		nh_.param<double>("state_filter/yaw_rate_zero_radps", sf_wz_zero_, 0.10);
		nh_.param<double>("state_filter/straightness_up_rate_per_s", sf_s_up_rate_, 0.5);
		nh_.param<double>("state_filter/max_lat_offset_m", sf_max_lat_m_, 0.04);
		double sf_max_yaw_deg = 0.3;
		nh_.param<double>("state_filter/max_yaw_offset_deg", sf_max_yaw_deg, 0.3);
		sf_max_yaw_rad_ = sf_max_yaw_deg * M_PI / 180.0;
		nh_.param<double>("state_filter/reset_pos_m", sf_reset_pos_m_, 0.3);
		double sf_reset_yaw_deg = 3.0;
		nh_.param<double>("state_filter/reset_yaw_deg", sf_reset_yaw_deg, 3.0);
		sf_reset_yaw_rad_ = sf_reset_yaw_deg * M_PI / 180.0;
		if (sf_enabled_) {
			gps_mode_sub_ = nh_.subscribe("/localization/gps_mode", 5, &ROSCONTROL::GpsModeCallback, this);
			// [straightness, lateral EKF-filter (m), yaw EKF-filter (deg), reset count, gps normal]
			sf_debug_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/control_feedback/state_filter", 10);
		}
		ROS_INFO("[MPC] state filter %s: tau pos %.2f s, yaw %.2f s, straight kappa %.4f..%.4f over %.1f s, yaw rate %.2f..%.2f rad/s, ramp %.2f /s, max offset %.3f m / %.2f deg, reset %.2f m / %.1f deg",
				 sf_enabled_ ? "ON" : "off", sf_pos_tau_s_, sf_yaw_tau_s_, sf_kappa_full_, sf_kappa_zero_,
				 sf_preview_s_, sf_wz_full_, sf_wz_zero_, sf_s_up_rate_, sf_max_lat_m_, sf_max_yaw_deg,
				 sf_reset_pos_m_, sf_reset_yaw_deg);
		lane_sub_ =nh_.subscribe("/local_path", 1, &ROSCONTROL::PathCallback, this);
		final_waypoint_sub_ = nh_.subscribe("/final_waypoint", 1, &ROSCONTROL::FinalWaypointCallback, this);
		state_sub_ = nh_.subscribe("/gps_state",1,&ROSCONTROL::StateCallback,this);
		lidar_topic_sub_=  nh_.subscribe("lidar_topic", 1, &ROSCONTROL::LidarLaneCallback, this);
		external_acc_speed_sub_ = nh_.subscribe("/smpc/target_speed_mps", 10,
			&ROSCONTROL::ExternalAccSpeedCallback, this);
		erp42_status_sub_ = nh_.subscribe("/erp42_status", 10, &ROSCONTROL::ErpCallback, this);
			competition_sub_ = nh_.subscribe("/Competition_topic", 100, &ROSCONTROL::CompetitionCallback, this);
		avoidance_speed_sub_ = nh_.subscribe("/obstacle_avoidance/target_speed_mps", 5,
			&ROSCONTROL::AvoidanceSpeedCallback, this);
		static_speed_scale_sub_ = nh_.subscribe("/obstacle_avoidance/static_speed_scale", 5,
			&ROSCONTROL::StaticSpeedScaleCallback, this);
		active_section_sub_ = nh_.subscribe("/mission/active_section", 10,
			&ROSCONTROL::ActiveSectionCallback, this);

		ROS_INFO("[MPC] longitudinal command type=%d (1=throttle, 2=velocity, 3=acceleration)",
				 longitudinal_command_type_);
		ROS_INFO("[MPC] curvature speed limit %s: source=%s, a_lat=%.2f m/s^2, min=%.2f m/s, preview=%.1f m",
				 curvature_speed_enabled_ ? "enabled" : "disabled",
				 curvature_use_waypoint_rk_ ? "final_waypoint.rk" : "mpc_polyfit",
				 curvature_lateral_accel_limit_mps2_, curvature_min_speed_mps_,
				 curvature_preview_distance_m_);
		ROS_INFO("[MPC] mission-state speed %s: default=%.2f m/s",
				 mission_speed_enabled_ ? "enabled" : "disabled",
				 mission_speed_default_mps_);
		ROS_INFO("[MPC] path-end stop %s: min_polyfit_points=%d, approach trigger=%.1f m, decel=%.2f m/s^2, margin=%.1f m",
				 path_end_stop_enabled_ ? "enabled" : "disabled",
				 path_end_min_polyfit_points_,
				 path_end_approach_trigger_m_, path_end_approach_decel_mps2_,
				 path_end_stop_margin_m_);

    }
	void ErpCallback(const erp_driver::erpStatusMsg::ConstPtr &msg){
        
        get_gear = msg->gear;
        get_speed = msg->speed;
        get_steer = msg->steer;
        get_brake = msg->brake;

        Get_steer_ = get_steer/100 * (M_PI/180); //radian

        Get_Kmh_Speed_ = (get_speed / 60.0) * (2.0 * WHEEL_RADIOUS * M_PI * 3.6);
		Get_MPH_Speed_ = Get_Kmh_Speed_ / 3.6;
       
    }

	static inline double applyACC_kmh(double spd_state_kmh, bool is_acc, int acc_index)
	{
		double input_speed_kmh = spd_state_kmh;

		if (!is_acc) return input_speed_kmh;

		if (acc_index > 150) {
			return spd_state_kmh;
		}
		else if (acc_index <= 150 && acc_index > 80) {
			const double x = static_cast<double>(acc_index);
			const double sig = 1.0 / (1.0 + std::exp(-0.017 * (x - 60.0)));

			double delta = 0.0;

			if (spd_state_kmh == 20.0) delta = (83.3 * sig - 68.67);
			else if (spd_state_kmh == 18.0) delta = (75.0 * sig - 61.8);
			else if (spd_state_kmh == 16.0) delta = (66.7 * sig - 54.93);
			else if (spd_state_kmh == 14.0) delta = (58.3 * sig - 48.067);
			else if (spd_state_kmh == 12.0) delta = (50.0 * sig - 41.2);

			// 누적 금지: 매 주기 spd_state 기준으로 재계산
			input_speed_kmh = spd_state_kmh + delta;
			return input_speed_kmh;
		}
		else if (acc_index <= 80 && acc_index > 0) {
			return 0.0;
		}

		return input_speed_kmh;
	}

	// EKF 원본 입력. /odom 과 값은 같고(ekf_odom_output 은 그대로 복사만 한다), 그 노드의
	// 유한값 검사만 똑같이 둔다. EKF 는 같은 주기에 pose -> twist 순으로 내고 MPC 는 루프마다
	// 콜백을 몰아서 처리하므로 odom_yaw_rate_ 는 solve 시점에 같은 주기의 twist 값이 된다.
	void EkfTwistCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg)
	{
		ekf_twist_ok_ = std::isfinite(msg->twist.twist.linear.x) && std::isfinite(msg->twist.twist.angular.z);
		if (!ekf_twist_ok_) return;
		ekf_yaw_rate_ = msg->twist.twist.angular.z;
		odom_yaw_rate_ = ekf_yaw_rate_;
		ekf_vx_ = msg->twist.twist.linear.x;
	}

	void GpsModeCallback(const std_msgs::String::ConstPtr &msg)
	{
		gps_mode_ = msg->data;
	}

	void EkfPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
	{
		const auto &p = msg->pose.pose;
		const bool finite = std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
			std::isfinite(p.orientation.x) && std::isfinite(p.orientation.y) &&
			std::isfinite(p.orientation.z) && std::isfinite(p.orientation.w) &&
			std::isfinite(msg->pose.covariance[0]) && std::isfinite(msg->pose.covariance[7]);
		if (!ekf_twist_ok_ || !finite) return;
		nav_msgs::Odometry::Ptr odom(new nav_msgs::Odometry);
		odom->header = msg->header;
		odom->pose = msg->pose;
		odom->twist.twist.angular.z = ekf_yaw_rate_;
		OdomCallback(odom);
	}

    void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
	{
		ros::Time cur_stamp = odom_msg->header.stamp;
		cur_pose_.pose.position = odom_msg->pose.pose.position;
		//
		cur_pose_.pose.orientation = odom_msg->pose.pose.orientation;

		// map -> base_link TF is owned by odom_information.
		// Do not publish the same transform from the controller.
		double cur_time_sec = cur_stamp.toSec();
		double ex_time_sec = ex_stamp.toSec();

		double dt = cur_time_sec - ex_time_sec;
		if (dt <= 1e-3) dt = 0.1;

		if (dt > SOME_MIN && dt < SOME_MAX){
			cur_speed_ = getSpeed(ex_x_, ex_y_,
								odom_msg->pose.pose.position.x,
								odom_msg->pose.pose.position.y,
								dt);
			
		}

		// update prev (항상 업데이트: 점프 감지 안정적)

		ex_stamp = cur_stamp;
		last_odom_stamp_ = cur_stamp;
		odom_yaw_rate_ = std::isfinite(odom_msg->twist.twist.angular.z) ? odom_msg->twist.twist.angular.z : 0.0;
		if (state_source_ == "odom" && std::isfinite(odom_msg->twist.twist.linear.x)) {
			ekf_vx_ = odom_msg->twist.twist.linear.x;
		}
		ex_x_ = cur_pose_.pose.position.x;
		ex_y_ = cur_pose_.pose.position.y;
		is_pose_ = true;

		tf::Quaternion q(odom_msg->pose.pose.orientation.x,
						odom_msg->pose.pose.orientation.y,
						odom_msg->pose.pose.orientation.z,
						odom_msg->pose.pose.orientation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll,pitch,yaw);
		// yaw 는 라디안 
		cur_course_ = yaw;

		is_course_ = true;

		// 					odom_msg->pose.pose.orientation.y,
		// 					odom_msg->pose.pose.orientation.z,

		// 				 odom_msg->pose.pose.orientation.y,
		// 				 odom_msg->pose.pose.orientation.z,

	}

	void PathCallback(const nav_msgs::Path::ConstPtr& path_msg)
	{
	    waypoints_.clear();
	    std::vector<waypoint_maker::Waypoint>().swap(waypoints_);

	    int index = 0;
	    for (const auto& pose_stamped : path_msg->poses) {
	        waypoint_maker::Waypoint wp;

	        wp.waypoint_index = index++;
	        wp.pose = pose_stamped;  // geometry_msgs::PoseStamped 그대로 복사
	        
	        waypoints_.push_back(wp);
	    }

	    waypoints_size_ = waypoints_.size();
	    is_lane_ = !waypoints_.empty();
	}

	void FinalWaypointCallback(const waypoint_maker::Lane::ConstPtr& lane_msg)
	{
	    speed_waypoints_.clear();
	    std::vector<waypoint_maker::Waypoint>().swap(speed_waypoints_);
	    speed_waypoints_.reserve(lane_msg->waypoints.size());

	    for (const auto& wp : lane_msg->waypoints) {
	        speed_waypoints_.push_back(wp);
	    }
	}

	void broadcastTransform()
	{

		geometry_msgs::TransformStamped transform;
		transform.header.stamp = ros::Time::now();  
		transform.header.frame_id = "map";          
		transform.child_frame_id = "base_link";     

		transform.transform.translation.x = cur_pose_.pose.position.x;
		transform.transform.translation.y = cur_pose_.pose.position.y;
		transform.transform.translation.z = cur_pose_.pose.position.z;

		geometry_msgs::Quaternion orientation = cur_pose_.pose.orientation;

		if (std::isnan(orientation.x) || std::isnan(orientation.y) ||
			std::isnan(orientation.z) || std::isnan(orientation.w) ||
			(orientation.x == 0 && orientation.y == 0 &&
			orientation.z == 0 && orientation.w == 0)) {
			
			double yaw = cur_course_; 
			
			tf::Quaternion tf_quat;
			tf_quat.setRPY(0.0, 0.0, yaw);  
			
			orientation.x = tf_quat.x();
			orientation.y = tf_quat.y();
			orientation.z = tf_quat.z();
			orientation.w = tf_quat.w();
		}

		transform.transform.rotation = orientation;

		tf_broadcaster_.sendTransform(transform);
	}

	void StateCallback(const waypoint_maker::State::ConstPtr &state_msg)
	{
		dist_ = state_msg->dist;
		current_mission_state_ = state_msg->current_state;
		loader_number_ = state_msg->lane_number;
	}

	void LidarLaneCallback(const new_lane::lidar_topic_msg::ConstPtr &msg)
	{
		is_acc = msg->ACC_flag;
        acc_index = msg->ACC_index_gap;

	}

	void ExternalAccSpeedCallback(const std_msgs::Float64::ConstPtr &msg)
	{
		if (!std::isfinite(msg->data)) return;
		external_acc_target_speed_mps_ = std::max(0.0, msg->data);
		external_acc_target_stamp_ = ros::Time::now();
		have_external_acc_target_ = true;
	}

		// MORAI 센서 갱신 주기 T: /imu 새 샘플(브리지가 같은 stamp 는 버린다) 간격의 EMA.
		// MORAI 는 프레임마다 한 번만 센서를 갱신하므로 T 는 그 PC 의 시뮬레이터 프레임 간격이다.
		void DelayImuCallback(const sensor_msgs::Imu::ConstPtr &msg)
		{
			const double stamp = msg->header.stamp.toSec();
			if (stamp <= 0.0) return;
			if (dc_last_imu_stamp_ > 0.0) {
				const double d = stamp - dc_last_imu_stamp_;
				if (d <= 1e-6) return;
				if (d > 0.005 && d < 0.2) {
					// 최근 15개 간격의 중앙값: 시작 직후 큰 간격이나 순간 튐에 끌려가지 않는다.
					dc_intervals_.push_back(d);
					while (dc_intervals_.size() > 15) dc_intervals_.pop_front();
					if (dc_intervals_.size() >= 5) {
						std::vector<double> sorted(dc_intervals_.begin(), dc_intervals_.end());
						std::nth_element(sorted.begin(), sorted.begin() + sorted.size() / 2, sorted.end());
						dc_period_s_ = sorted[sorted.size() / 2];
					}
				}
			}
			dc_last_imu_stamp_ = stamp;
			dc_last_imu_rx_ = ros::Time::now();
		}

		void updateEffectiveLf(const morai_msgs::EgoVehicleStatus &msg)
		{
			const double stamp = msg.header.stamp.toSec();
			if (stamp <= dc_lf_last_stamp_ + 1e-6) return;          // MORAI 가 같은 샘플을 두 번 보낸다
			const double dt = dc_lf_last_stamp_ > 0.0 ? std::min(stamp - dc_lf_last_stamp_, 0.2) : 0.05;
			dc_lf_last_stamp_ = stamp;
			const double v = std::hypot(msg.velocity.x, msg.velocity.y);
			// wheel_angle 은 deg, /ctrl_cmd 와 부호가 반대 -> MPC 부호(yaw_rate = -v*delta/Lf) 와 같다.
			// MORAI 상태 패킷의 angular_velocity.z 는 ROS(반시계 +) 와 부호가 반대다(실측 비 -1.00).
			const double delta = msg.wheel_angle * M_PI / 180.0;
			const double yaw_rate = -msg.angular_velocity.z;
			if (!std::isfinite(v) || !std::isfinite(delta) || !std::isfinite(yaw_rate)) return;
			const double a = 1.0 - std::exp(-dt / 0.5);                  // 0.5 s 저역통과(정상 상태 비)
			dc_lf_x_ += a * (-v * delta - dc_lf_x_);
			dc_lf_y_ += a * (yaw_rate - dc_lf_y_);
			if (v < 3.0 || std::fabs(dc_lf_y_) < 0.08 || dc_lf_x_ * dc_lf_y_ <= 0.0) return;   // 곡선에서만
			const double lf = dc_lf_x_ / dc_lf_y_;
			if (lf < 3.0 || lf > 6.0) return;
			const double b = 1.0 - std::exp(-dt / 3.0);                  // 3 s 시정수로 천천히
			dc_lf_est_ = dc_lf_est_ <= 0.0 ? lf : dc_lf_est_ + b * (lf - dc_lf_est_);
		}

		void CompetitionCallback(const morai_msgs::EgoVehicleStatus::ConstPtr &msg)
		{
			competition_speed_mps_ = std::hypot(msg->velocity.x, msg->velocity.y);
			// In this MORAI setup wheel_angle is reported in degrees and uses the
			// opposite sign convention from /ctrl_cmd.steering.
			competition_wheel_angle_deg_ = msg->wheel_angle;
			have_competition_feedback_ = true;
			updateEffectiveLf(*msg);
		}

		void AvoidanceSpeedCallback(const std_msgs::Float64::ConstPtr &msg)
		{
			avoidance_speed_limit_mps_ = msg->data;
			last_avoidance_speed_time_ = ros::Time::now();
			have_avoidance_speed_ = true;
		}

		void StaticSpeedScaleCallback(const std_msgs::Float64::ConstPtr &msg)
		{
			static_speed_scale_ = msg->data;
			last_static_speed_scale_time_ = ros::Time::now();
			have_static_speed_scale_ = true;
		}

		void ActiveSectionCallback(const mission_supervisor::SectionState::ConstPtr &msg)
		{
			in_exempt_section_ = msg->speed_limit_exempt;
			active_section_id_ = msg->section_id;
            // cruise section ID and all numerical MPC fitting stay unchanged.
            const bool entry = msg->index_valid && active_section_id_=="M7B" &&
                std::find(msg->executors.begin(),msg->executors.end(),"static_obstacle")!=msg->executors.end() &&
                std::find(msg->executors.begin(),msg->executors.end(),"dynamic_obstacle")!=msg->executors.end();
            avoidance_section_key_ = entry ? "M8" : active_section_id_;
			last_active_section_time_ = ros::Time::now();
			have_active_section_ = true;
		}

		void publishTargetSpeed(double target_speed_mps)
		{
			std_msgs::Float64 msg;
			msg.data = target_speed_mps;
			mpc_target_speed_pub_.publish(msg);
		}

			static double clampValue(double value, double lower, double upper)
		{
			return std::max(lower, std::min(value, upper));
		}

		void publishFloat64(ros::Publisher& publisher, double value)
		{
			std_msgs::Float64 msg;
			msg.data = value;
			publisher.publish(msg);
		}

		double polynomialCurvatureAbs(const Eigen::VectorXd& coeffs, double x) const
		{
			if (coeffs.size() < 3) return 0.0;

			double dy = coeffs[1];
			double ddy = 2.0 * coeffs[2];
			if (coeffs.size() >= 4) {
				dy += 2.0 * coeffs[2] * x + 3.0 * coeffs[3] * x * x;
				ddy += 6.0 * coeffs[3] * x;
			} else {
				dy += 2.0 * coeffs[2] * x;
			}

			const double denominator = std::pow(1.0 + dy * dy, 1.5);
			if (denominator < 1e-9) return 0.0;
			return std::fabs(ddy / denominator);
		}

		double percentileCurvatureSample(std::vector<double>& curvature_samples) const
		{
			if (curvature_samples.empty()) return 0.0;

			std::sort(curvature_samples.begin(), curvature_samples.end());
			const double percentile =
				clampValue(curvature_percentile_, 0.0, 1.0);
			const int index = static_cast<int>(
				std::round(percentile * static_cast<double>(curvature_samples.size() - 1)));
			return curvature_samples[
				std::max(0, std::min(index, static_cast<int>(curvature_samples.size() - 1)))];
		}

		double previewWaypointRkForSpeedLimit() const
		{
			if (speed_waypoints_.size() < 2) return 0.0;

			double path_s = 0.0;
			std::vector<double> curvature_samples;
			curvature_samples.reserve(speed_waypoints_.size());

			for (size_t i = 1; i < speed_waypoints_.size(); ++i) {
				const auto& prev = speed_waypoints_[i - 1].pose.pose.position;
				const auto& curr = speed_waypoints_[i].pose.pose.position;
				const double ds = std::hypot(curr.x - prev.x, curr.y - prev.y);
				if (!std::isfinite(ds)) continue;

				path_s += ds;
				if (path_s < curvature_min_lookahead_m_) continue;
				if (path_s > curvature_preview_distance_m_) break;

				const double kappa_abs = std::fabs(speed_waypoints_[i].rk);
				if (std::isfinite(kappa_abs)) {
					curvature_samples.push_back(kappa_abs);
				}
			}

			return percentileCurvatureSample(curvature_samples);
		}

		double previewPolyfitCurvatureForSpeedLimit(const Eigen::VectorXd& coeffs,
													const Eigen::VectorXd& vehicle_waypoints_x) const
		{
			double max_x = 0.0;
			for (int i = 0; i < vehicle_waypoints_x.size(); ++i) {
				const double x = vehicle_waypoints_x[i];
				if (std::isfinite(x) && x > max_x) max_x = x;
			}

			const double x_start = std::max(0.0, curvature_min_lookahead_m_);
			const double x_end = std::min(curvature_preview_distance_m_, max_x);
			if (x_end <= x_start + 1e-3) return 0.0;

			const int samples = std::max(2, curvature_sample_count_);
			std::vector<double> curvature_samples;
			curvature_samples.reserve(samples);
			for (int i = 0; i < samples; ++i) {
				const double ratio =
					(samples == 1) ? 0.0 : static_cast<double>(i) / static_cast<double>(samples - 1);
				const double x = x_start + ratio * (x_end - x_start);
				const double kappa_abs = polynomialCurvatureAbs(coeffs, x);
				if (std::isfinite(kappa_abs)) {
					curvature_samples.push_back(kappa_abs);
				}
			}

			return percentileCurvatureSample(curvature_samples);
		}

		double computeCurvatureSpeedLimit(const Eigen::VectorXd& coeffs,
										  const Eigen::VectorXd& vehicle_waypoints_x)
		{
			const double active_nominal_limit = std::min(ref_v, speed_upperBound);
			const double nominal_limit =
				(curvature_max_speed_mps_ > 0.0)
					? std::min(curvature_max_speed_mps_, active_nominal_limit)
					: active_nominal_limit;
			const double safe_nominal_limit = std::max(0.0, nominal_limit);
			const bool m2_avoidance = have_active_section_ &&
				(ros::Time::now() - last_active_section_time_).toSec() < 0.5 &&
				std::find(avoidance_sections_.begin(), avoidance_sections_.end(),
						  avoidance_section_key_) != avoidance_sections_.end() &&
				have_avoidance_speed_ &&
				(ros::Time::now() - last_avoidance_speed_time_).toSec() < 0.5 &&
				avoidance_speed_limit_mps_ < 500.0;
			const double min_limit = std::min(m2_avoidance
				? avoidance_curvature_min_speed_mps_ : curvature_min_speed_mps_,
				safe_nominal_limit);

			double preview_curvature = 0.0;
			if (curvature_use_waypoint_rk_) {
				preview_curvature = previewWaypointRkForSpeedLimit();
			}
			if ((!curvature_use_waypoint_rk_ ||
				 (curvature_fallback_to_polyfit_ &&
				  preview_curvature <= curvature_deadband_inv_m_)) &&
				curvature_fallback_to_polyfit_) {
				preview_curvature = previewPolyfitCurvatureForSpeedLimit(coeffs, vehicle_waypoints_x);
			}
			last_path_curvature_for_speed_limit_ = preview_curvature;

			// The existing avoidance_control profile owns the M2-only path
			// tracking tune.  Its lateral acceleration cap applies only while
			// a fresh avoidance path/speed command is active in M2; ordinary
			// urban corners keep the established curvature limit.
            const bool ordinary_urban = !is_highway_mode && !m2_avoidance;
            const double lateral_accel_limit = m2_avoidance
                ? avoidance_lateral_accel_limit_mps2_
                : (ordinary_urban ? urban_curvature_lateral_accel_limit_mps2_
                                  : curvature_lateral_accel_limit_mps2_);
            const double curve_rate_up = ordinary_urban
                ? urban_curvature_speed_rate_up_mps2_ : curvature_speed_rate_up_mps2_;
			double raw_limit = safe_nominal_limit;
			if (curvature_speed_enabled_ &&
				lateral_accel_limit > 0.0 &&
				last_path_curvature_for_speed_limit_ > curvature_deadband_inv_m_) {
				raw_limit = std::sqrt(lateral_accel_limit /
									  last_path_curvature_for_speed_limit_);
			}
            // Geometry capacity for future departure prediction. It deliberately
            // excludes the instantaneous mission ramp; actual controls below are unchanged.
            double capacity = std::max(0.0, speed_upper_nominal_);
            const double profile_ceiling = is_highway_mode
                ? speed_limit_highway_max_mps_ : speed_limit_urban_max_mps_;
            if (profile_ceiling > 0.0) capacity = std::min(capacity, profile_ceiling);
            if (curvature_max_speed_mps_ > 0.0)
                capacity = std::min(capacity, curvature_max_speed_mps_);
            if (curvature_speed_enabled_ && lateral_accel_limit > 0.0 &&
                last_path_curvature_for_speed_limit_ > curvature_deadband_inv_m_)
                capacity = std::min(capacity, std::max(
                    m2_avoidance ? avoidance_curvature_min_speed_mps_ : curvature_min_speed_mps_,
                    std::sqrt(lateral_accel_limit / last_path_curvature_for_speed_limit_)));
            publishFloat64(curvature_capacity_speed_pub_, capacity);
			raw_limit = clampValue(raw_limit, min_limit, safe_nominal_limit);

			if (!have_filtered_curve_speed_limit_) {
				filtered_curve_speed_limit_mps_ = raw_limit;
				have_filtered_curve_speed_limit_ = true;
			} else {
				const double up_step =
					std::max(0.0, curve_rate_up) * dt;
				const double down_step =
					std::max(0.0, curvature_speed_rate_down_mps2_) * dt;
				filtered_curve_speed_limit_mps_ =
					clampValue(raw_limit,
							   filtered_curve_speed_limit_mps_ - down_step,
							   filtered_curve_speed_limit_mps_ + up_step);
				filtered_curve_speed_limit_mps_ =
					clampValue(filtered_curve_speed_limit_mps_, min_limit, safe_nominal_limit);
			}

			last_curve_speed_limit_mps_ = curvature_speed_enabled_
				? filtered_curve_speed_limit_mps_
				: safe_nominal_limit;

			publishFloat64(curvature_speed_limit_pub_, last_curve_speed_limit_mps_);
			publishFloat64(path_curvature_for_speed_limit_pub_, last_path_curvature_for_speed_limit_);

			return last_curve_speed_limit_mps_;
		}

		double makeSteeringCommand(double mpc_steering_rad)
		{
			double command = steering_command_sign_ * mpc_steering_rad;
			command = clampValue(command, steer_lowerBound, steer_upperBound);

			if (steering_command_rate_limit_radps_ > 0.0 && have_previous_steering_command_) {
				const double max_step = steering_command_rate_limit_radps_ * dt;
				command = clampValue(command,
									 previous_steering_command_rad_ - max_step,
									 previous_steering_command_rad_ + max_step);
			}

			if (steering_output_lpf_enabled_ && have_previous_steering_command_) {
				double spd = have_competition_feedback_ ? competition_speed_mps_ : cur_speed_;
				double effective_lpf_alpha = steering_output_lpf_alpha_;
				if (spd <= 11.0) {
					effective_lpf_alpha = steering_output_lpf_alpha_;
				} else if (spd >= 25.0) {
					effective_lpf_alpha = steering_output_lpf_alpha_high_speed_;
				} else {
					const double r = (spd - 11.0) / (25.0 - 11.0);
					effective_lpf_alpha = steering_output_lpf_alpha_ + r * (steering_output_lpf_alpha_high_speed_ - steering_output_lpf_alpha_);
				}
				command = effective_lpf_alpha * command
						+ (1.0 - effective_lpf_alpha) * previous_steering_command_rad_;
				command = clampValue(command, steer_lowerBound, steer_upperBound);
			}

			previous_steering_command_rad_ = command;
			have_previous_steering_command_ = true;
			// 지연보상용 명령 이력(MPC 부호 기준). 1 s 넘게 끊기거나 시계가 역행하면 비운다.
			const double now = ros::Time::now().toSec();
			if (!steering_history_.empty() &&
				(now < steering_history_.back().time_s || now - steering_history_.back().time_s > 1.0)) {
				steering_history_.clear();
			}
			if (std::fabs(steering_command_sign_) > 1e-6) {
				steering_history_.push_back({now, command / steering_command_sign_});
			}
			if (dc_pred_pending_) {
				// 예측 시점 -> 명령 확정까지 걸린 실제 시간. 올라갈 때는 바로, 내려갈 때는 천천히 따라간다.
				const double sample = std::min((ros::WallTime::now() - dc_pred_wall_).toSec(), dc_compute_max_s_);
				dc_compute_s_ = sample > dc_compute_s_ ? sample : dc_compute_s_ + 0.1 * (sample - dc_compute_s_);
				dc_pred_pending_ = false;
			}
			while (steering_history_.size() > 1 && steering_history_[1].time_s < now - 1.0) {
				steering_history_.pop_front();
			}
			return command;
		}

		void setThrottleCommand(double target_speed_mps, double actual_speed_mps, double dt,
								 double brake_limit = -1.0, double accel_limit = -1.0)
	{
		if (brake_limit < 0.0) brake_limit = throttle_brake_limit_;
		if (accel_limit < 0.0) accel_limit = throttle_accel_limit_;

		if (target_speed_mps <= 0.05) {
			throttle_integral_ = 0.0;
			ff_prev_target_mps_ = 0.0;
			ff_target_accel_ = 0.0;
			ackermann_msg_.accel = 0.0;
			double desired_stop_brake = brake_limit;
			const double max_brake_increase = brake_rate_limit_per_sec_ * dt;
			if (desired_stop_brake > last_commanded_brake_ + max_brake_increase) {
				desired_stop_brake = last_commanded_brake_ + max_brake_increase;
			}
			last_commanded_brake_ = desired_stop_brake;
			ackermann_msg_.brake = desired_stop_brake;
			return;
		}

		const double speed_error = target_speed_mps - actual_speed_mps;
		// band > 0 이면 목표 근처에서만 적분한다(가속·감속 램프 중 와인드업 없이 정속 오차만 없앤다).
		// 0 이면 기존처럼 항상 적분.
		if (throttle_integral_band_mps_ <= 0.0 || std::fabs(speed_error) < throttle_integral_band_mps_) {
			throttle_integral_ += speed_error * dt;
		}
		// 목표보다 빠른데 양(+)의 적분이 남아 있으면 지운다. 없으면 고주로(20 m/s 유지 페달)에서 쌓인 적분이
		// 예외 구간 끝(목표 16.0)을 지나서도 페달을 밀어 16.1~16.28 m/s 로 5 s 달렸다(M7B).
		if (throttle_integral_band_mps_ > 0.0 && speed_error < -0.1 && throttle_integral_ > 0.0) {
			throttle_integral_ = 0.0;
		}
		throttle_integral_ = clampValue(throttle_integral_, -throttle_integral_limit_,
											 throttle_integral_limit_);

		// 목표속도 램프 피드포워드: 목표가 올라가는 기울기만큼 페달을 미리 준다.
		// PI 만으로는 3 m/s^2 램프에 페달 0.9 가 필요해 오차가 ~4 m/s 벌어진 뒤에야 따라갔다.
		// 올라갈 때만 쓰고(감속은 기존 PI·브레이크 그대로), 목표가 일정하면 0 이라 정속 거동은 같다.
		// - 정속 중 목표 잡음(평활 기울기 p95 0.17, p99 1.2 m/s^2)은 버리고 램프(중앙값 2.5)만 쓴다.
		// - 목표와의 오차가 taper 보다 작아지면 비례해서 줄인다. 없으면 램프 끝에서 응답 지연만큼
		//   목표를 넘었다(출발 가속 끝 16.0 -> 16.6 m/s).
		double pedal_ff = 0.0;
		if (throttle_ff_gain_ > 0.0) {
			if (ff_prev_target_mps_ >= 0.0) {
				const double accel_raw = (target_speed_mps - ff_prev_target_mps_) / dt;
				ff_target_accel_ += dt / (throttle_ff_tau_s_ + dt) * (accel_raw - ff_target_accel_);
			}
			const double accel_ff = ff_target_accel_ > throttle_ff_min_accel_
				? std::min(ff_target_accel_, throttle_ff_accel_per_pedal_) : 0.0;
			const double taper = clampValue(speed_error / throttle_ff_taper_mps_, 0.0, 1.0);
			// 조향 중(곡선 출구)에는 쓰지 않는다. 곡선 정확도는 출구에서 늦게 가속하던 PI 거동에 맞춰져 있어
			// 출구에서 바로 밟으면 바깥으로 밀렸다(R11 0.43 m, 중앙선 접촉).
			const double steer_taper = clampValue(
				1.0 - std::fabs(steering_angle) / throttle_ff_steer_zero_rad_, 0.0, 1.0);
			pedal_ff = throttle_ff_gain_ * taper * steer_taper * accel_ff / throttle_ff_accel_per_pedal_;
		}
		ff_prev_target_mps_ = target_speed_mps;

		const double pedal = throttle_kp_ * speed_error + throttle_ki_ * throttle_integral_ + pedal_ff;

		// 데드밴드: |pedal| 이 이 값보다 작으면 가속·제동 모두 0(타행).
		// 없으면 pedal 이 0을 지날 때마다 가속<->제동이 뒤집혀 차가 덜컹인다.
		// 루프가 느려 추종을 못 하던 시절에는 오차가 항상 한쪽이라 안 드러났다.
		// 기본 0.0 = 기존 거동 그대로. 켜려면 0.03~0.05 부터 시도한다.
		if (throttle_pedal_deadband_ > 0.0
			&& std::fabs(pedal) < throttle_pedal_deadband_) {
			ackermann_msg_.accel = 0.0;
			ackermann_msg_.brake = 0.0;
			last_commanded_brake_ = 0.0;
			return;
		}

		ackermann_msg_.accel = clampValue(pedal, 0.0, accel_limit);
		double desired_brake = clampValue(-pedal, 0.0, brake_limit);

		// 한 사이클짜리 노이즈(target_velocity는 안정적인데도 순간적으로 브레이크가
		// 훅 튀는 현상)를 걸러내기 위해 브레이크 증가 속도를 제한. 실제 위험한
		// 정지는 obstacle_stop_node가 이 경로를 거치지 않고 /ctrl_cmd를 직접
		// 덮어써서 즉시 정지시키니까(brake=1.0), 여기서 부드럽게 램프업해도
		// 안전에는 영향 없음.
		const double max_brake_increase = brake_rate_limit_per_sec_ * dt;
		if (desired_brake > last_commanded_brake_ + max_brake_increase) {
			desired_brake = last_commanded_brake_ + max_brake_increase;
		}
		last_commanded_brake_ = desired_brake;
		ackermann_msg_.brake = desired_brake;
	}

		void publishControlCommand()
	{
		ackermann_msg_.longlCmdType = 1;
		ackermann_msg_.steering = makeSteeringCommand(steering_angle);

		const double actual_speed = have_competition_feedback_
			? competition_speed_mps_ : cur_speed_;
		const double brake_limit_now =
			static_speed_scale_was_active_ ? kick_brake_limit_ : throttle_brake_limit_;
		const bool is_highway = determineIsHighway();
		const double accel_limit_now =
			is_highway ? throttle_accel_limit_exempt_ : throttle_accel_limit_;
		setThrottleCommand(target_velocity, actual_speed, 0.05, brake_limit_now, accel_limit_now);
		if (legal_guard_enabled_ && !is_highway) {
			if (actual_speed > legal_guard_cut_mps_) {
				ackermann_msg_.accel = 0.0;
			}
			if (actual_speed > legal_guard_brake_at_mps_ && ackermann_msg_.brake < legal_guard_brake_) {
				ackermann_msg_.brake = legal_guard_brake_;
				last_commanded_brake_ = legal_guard_brake_;
			}
		}
		ackermann_msg_.velocity = 0.0;
		ackermann_msg_.acceleration = 0.0;

		ackermann_pub_.publish(ackermann_msg_);
	}

	void publishPathEndStop(const std::string& reason)
	{
		target_velocity = 0.0;
		is_control_ = true;

		cmd_speed_kmh_ = 0.0;
		cmd_steer_deg_ = (-steering_angle) * 180.0 / M_PI;
		cmd_valid_ = true;

		publishTargetSpeed(target_velocity);
		publishControlCommand();

		ROS_WARN_THROTTLE(1.0,
			"[MPC] path-end stop (%s): /local_path points=%d, required >=%d for cubic polyfit",
			reason.c_str(), waypoints_size_, path_end_min_polyfit_points_);
	}

    void getClosestWaypoint(geometry_msgs::PoseStamped current_pose) 
	{
		if (!waypoints_.empty()) 
		{
			double dist_min = MAX_SEARCH_DIST;
			for (int i = 0; i < waypoints_.size(); i++)
			{
				double dist = calcPlaneDist(current_pose, waypoints_[i].pose);
				if (dist < dist_min) 
				{
					dist_min = dist;
					waypoint_min_ = i;
				}
			}
			closest_waypoint_ = waypoint_min_;
		}
		else cout << "------ NO CLOSEST WAYPOINT -------" << endl;
	}

	double calcPlaneDist(const geometry_msgs::PoseStamped pose1, const geometry_msgs::PoseStamped pose2) 
	{
		return sqrt(pow(pose1.pose.position.x - pose2.pose.position.x, 2) + pow(pose1.pose.position.y - pose2.pose.position.y, 2));
	}

	double getSpeed(double& ex_x, double& ex_y, double cur_x, double cur_y, double dt)
	{
        double distance = sqrt(pow((cur_x - ex_x ), 2) + pow((cur_y - ex_y), 2));
        double speed = distance / dt;
		
        return speed;
	}

	double getAccel(double& ex_x, double& ex_y, double cur_x, double cur_y) {
		double distance = sqrt(pow((cur_x - ex_x ), 2) + pow((cur_y - ex_y), 2));
		double speed = distance / inter_time_;
		double a = speed / inter_time_ * INTER_SPEED_TIME_MAX;

		return a;
	}

	double getCmdSpeedKmh() const { return cmd_speed_kmh_; }
	double getCmdSteerDeg() const { return cmd_steer_deg_; }

	static inline double wrapToPi(double a)
	{
	while (a > M_PI) a -= 2.0 * M_PI;
	while (a < -M_PI) a += 2.0 * M_PI;
	return a;
	}

	void modelPredictiveController() {
		is_control_ = false;
		
    	getClosestWaypoint(cur_pose_);

		const bool have_pose_and_course = is_pose_ && is_course_;
		if (have_pose_and_course &&
			(!is_lane_ || waypoints_size_ < path_end_min_polyfit_points_)) {
			if (path_end_stop_enabled_) {
				publishPathEndStop(is_lane_ ? "too_few_local_path_points" : "no_local_path");
			} else {
				ROS_WARN_THROTTLE(1.0,
					"[MPC] skip control: /local_path points=%d, required >=%d for cubic polyfit",
					waypoints_size_, path_end_min_polyfit_points_);
			}
			is_pose_ = false;
			is_course_ = false;
			return;
		}

    	if(is_pose_ && is_course_ && is_lane_ && waypoints_size_ >= path_end_min_polyfit_points_) 
		{
    	    is_control_ = true;

    	    double px = ex_x_;
    	    double py = ex_y_;

    	    double psi = cur_course_;
			double velocity = have_competition_feedback_ ? competition_speed_mps_ : cur_speed_;
			double dt_ = 0.05;

			// MPC 입력 위치 필터(기본 꺼짐): 직선에서 GPS 계단으로 생긴 EKF 위치·헤딩 잔떨림만 걸러낸다.
			// 새 EKF 샘플마다 차속·요레이트로 적분하고, EKF 쪽으로 tau*직선도 로 맞춘다(직선도 0 이면 EKF 그대로).
			if (sf_enabled_) {
				const double stamp = last_odom_stamp_.toSec();
				const double dt_f = stamp - sf_last_stamp_;
				const double ekf_yaw = psi;
				bool reset = !sf_init_ || gps_mode_ != "normal" || ekf_vx_ < 0.3 || dt_f < 0.0 || dt_f > 0.5;
				if (!reset && dt_f > 1e-4) {
					const double yaw_mid = sf_yaw_ + 0.5 * odom_yaw_rate_ * dt_f;
					sf_x_ += velocity * std::cos(yaw_mid) * dt_f;
					sf_y_ += velocity * std::sin(yaw_mid) * dt_f;
					sf_yaw_ = wrapToPi(sf_yaw_ + odom_yaw_rate_ * dt_f);
					const double s = clampValue(sf_straightness_, 0.0, 1.0);
					const double tau_p = sf_pos_tau_s_ * s;
					const double tau_y = sf_yaw_tau_s_ * s;
					const double a_p = tau_p > 1e-3 ? 1.0 - std::exp(-dt_f / tau_p) : 1.0;
					const double a_y = tau_y > 1e-3 ? 1.0 - std::exp(-dt_f / tau_y) : 1.0;
					// 진행방향은 EKF 를 그대로 따른다(조향과 무관하고, 고속에서 EKF 시각 어긋남만큼 차이가 커져
					// 리셋을 일으켰다). 횡위치·헤딩만 tau 로 맞춘다.
					const double c = std::cos(sf_yaw_), sn = std::sin(sf_yaw_);
					const double ex = px - sf_x_, ey = py - sf_y_;
					const double d_along = ex * c + ey * sn;
					const double d_lat = -ex * sn + ey * c;
					sf_x_ += d_along * c - a_p * d_lat * sn;
					sf_y_ += d_along * sn + a_p * d_lat * c;
					sf_yaw_ = wrapToPi(sf_yaw_ + a_y * wrapToPi(ekf_yaw - sf_yaw_));
					// EKF 와의 차이 상한: 출발·커브 직후 EKF 헤딩이 1~2 deg 틀린 동안 그 헤딩으로 적분해 위치가 밀리는 것을
					// 막는다. GPS 계단 잡음(횡 +-2~4 cm, 헤딩 +-0.2 deg)만 걸러지도록 이 폭 밖으로는 못 나가게 한다.
					{
						const double dyaw = wrapToPi(ekf_yaw - sf_yaw_);
						if (std::fabs(dyaw) > sf_max_yaw_rad_) {
							sf_yaw_ = wrapToPi(ekf_yaw - std::copysign(sf_max_yaw_rad_, dyaw));
						}
						const double c2 = std::cos(sf_yaw_), s2 = std::sin(sf_yaw_);
						const double lat = -(px - sf_x_) * s2 + (py - sf_y_) * c2;
						const double over = lat - clampValue(lat, -sf_max_lat_m_, sf_max_lat_m_);
						sf_x_ -= over * s2;
						sf_y_ += over * c2;
					}
					if (std::fabs((1.0 - a_p) * d_lat) > sf_reset_pos_m_ ||
						std::fabs(wrapToPi(ekf_yaw - sf_yaw_)) > sf_reset_yaw_rad_) {
						reset = true;
						++sf_reset_count_;
					}
				}
				if (reset) {
					sf_x_ = px;
					sf_y_ = py;
					sf_yaw_ = ekf_yaw;
					sf_init_ = true;
				}
				sf_last_stamp_ = stamp;
				std_msgs::Float64MultiArray sf_dbg;
				sf_dbg.data = {sf_straightness_,
							   -(px - sf_x_) * std::sin(sf_yaw_) + (py - sf_y_) * std::cos(sf_yaw_),
							   wrapToPi(ekf_yaw - sf_yaw_) * 180.0 / M_PI,
							   static_cast<double>(sf_reset_count_),
							   gps_mode_ == "normal" ? 1.0 : 0.0};
				sf_debug_pub_.publish(sf_dbg);
				px = sf_x_;
				py = sf_y_;
				psi = sf_yaw_;
			}

			const bool is_highway = determineIsHighway();

			this->is_highway_mode = is_highway;
			this->actual_speed_mps = velocity;
			curvature_preview_distance_m_ = is_highway ? preview_distance_highway_m_ : preview_distance_urban_m_;

			std_msgs::Bool hwy_msg;
			hwy_msg.data = is_highway;
			is_highway_mode_pub_.publish(hwy_msg);

		// 속도 적응형 Polyfit 거리:
		// - 도심(Urban): 저속 급커브 12m ~ 도심 직선(54 km/h) 19.5~24m로 기존 커브 0오차 피팅 100% 보존.
		// - 고주로(Highway): 90 km/h (25 m/s) 시 N=30 (1.5초) 예측 지평선 37.5m 전체를 포함하도록
		//   최대 42m까지 피팅하여 3차 다항식 외삽(Extrapolation) 발산에 의한 고속 조향 흔들림을 원천 제거.
		const double kMaxPolyfitDistanceM = is_highway
			? std::max(24.0, std::min(42.0, velocity * 1.6))
			: std::max(12.0, std::min(24.0, velocity * 1.3));
		int fit_points_count = 0;
		double accum_s = 0.0;
		for (int i = 0; i < waypoints_size_; i++) {
			fit_points_count++;
			if (i > 0) {
				accum_s += std::hypot(waypoints_[i].pose.pose.position.x - waypoints_[i - 1].pose.pose.position.x,
									  waypoints_[i].pose.pose.position.y - waypoints_[i - 1].pose.pose.position.y);
			}
			if (accum_s >= kMaxPolyfitDistanceM && fit_points_count >= path_end_min_polyfit_points_) {
				break;
			}
		}
		if (fit_points_count < path_end_min_polyfit_points_) {
			fit_points_count = std::min(waypoints_size_, path_end_min_polyfit_points_);
		}

		VectorXd vehicle_waypoints_x(fit_points_count);
		VectorXd vehicle_waypoints_y(fit_points_count);

		for (int i = 0; i < fit_points_count; i++) 
		{
			double diff_x = waypoints_[i].pose.pose.position.x - px;
			double diff_y = waypoints_[i].pose.pose.position.y - py;
			vehicle_waypoints_x[i] = diff_x * cos(-psi) - diff_y * sin(-psi);
			vehicle_waypoints_y[i] = diff_y * cos(-psi) + diff_x * sin(-psi);
		}

		auto coeffs = polyfit(vehicle_waypoints_x, vehicle_waypoints_y, 3);

		// 곡률 preview 전용 별도 fit.
		// 추종용 coeffs 는 근거리 정밀도를 위해 짧게(<=24m) 유지해야 한다 - 늘리면
		// R=20m 커브에서 0~5m 피팅오차가 3.6cm -> 62cm 로 악화된다.
		// 반면 커브 감속은 v^2/(2a) 만큼 미리 봐야 하므로(15 m/s 기준 23m) 짧은
		// fit 으로는 늦는다. 13.6 m/s 진입에서 17.6m 까지만 보여
		// 감속이 커브 안에서 시작됐고 조향 40deg 포화 + cte 8.9m 로 발산했다.
		// 그래서 속도를 보는 쪽만 긴 fit 을 따로 만든다.
		Eigen::VectorXd preview_coeffs = coeffs;
		Eigen::VectorXd preview_x = vehicle_waypoints_x;
		{
			const double kPreviewFitDistanceM =
				std::max(kMaxPolyfitDistanceM, std::min(60.0, velocity * 2.5));
			if (kPreviewFitDistanceM > kMaxPolyfitDistanceM + 1e-3) {
				int pv_count = 0;
				double pv_s = 0.0;
				for (int i = 0; i < waypoints_size_; i++) {
					pv_count++;
					if (i > 0) {
						pv_s += std::hypot(
							waypoints_[i].pose.pose.position.x - waypoints_[i - 1].pose.pose.position.x,
							waypoints_[i].pose.pose.position.y - waypoints_[i - 1].pose.pose.position.y);
					}
					if (pv_s >= kPreviewFitDistanceM && pv_count >= path_end_min_polyfit_points_) break;
				}
				if (pv_count > fit_points_count && pv_count >= path_end_min_polyfit_points_) {
					VectorXd pvx(pv_count), pvy(pv_count);
					for (int i = 0; i < pv_count; i++) {
						double dx = waypoints_[i].pose.pose.position.x - px;
						double dy = waypoints_[i].pose.pose.position.y - py;
						pvx[i] = dx * cos(-psi) - dy * sin(-psi);
						pvy[i] = dy * cos(-psi) + dx * sin(-psi);
					}
					preview_coeffs = polyfit(pvx, pvy, 3);
					preview_x = pvx;
				}
			}
		}

			if (sf_enabled_) {
				// 다음 주기 필터용 직선도: 앞으로 preview_s 동안 경로 평균 곡률(긴 preview fit)과 실측 요레이트.
				const double x_end = std::max(1.0, std::min(std::max(1.0, velocity) * sf_preview_s_,
															 preview_x.size() > 0 ? preview_x.maxCoeff() : 1.0));
				double ksum = 0.0;
				const int n = 12;
				for (int i = 0; i <= n; ++i) {
					const double xs = x_end * i / n;
					const double d1 = preview_coeffs[1] + 2.0 * preview_coeffs[2] * xs + 3.0 * preview_coeffs[3] * xs * xs;
					const double d2 = 2.0 * preview_coeffs[2] + 6.0 * preview_coeffs[3] * xs;
					ksum += std::fabs(d2) / std::pow(1.0 + d1 * d1, 1.5);
				}
				const double kmean = ksum / (n + 1);
				const double s_k = clampValue((sf_kappa_zero_ - kmean) / std::max(1e-6, sf_kappa_zero_ - sf_kappa_full_), 0.0, 1.0);
				const double s_w = clampValue((sf_wz_zero_ - std::fabs(odom_yaw_rate_)) / std::max(1e-6, sf_wz_zero_ - sf_wz_full_), 0.0, 1.0);
				// 커브 직후 1~2 s 동안 EKF 헤딩이 0.5~1 deg 어긋나 있어, 필터가 바로 켜지면 그 오차를 붙잡고 적분해
				// 횡위치가 밀린다. 그래서 직선도는 천천히만 올린다.
				const double now_s = ros::Time::now().toSec();
				const double dt_s = sf_s_last_t_ > 0.0 ? clampValue(now_s - sf_s_last_t_, 0.0, 0.5) : 0.0;
				sf_s_last_t_ = now_s;
				sf_straightness_ = std::min(s_k * s_w, sf_straightness_ + std::max(0.0, sf_s_up_rate_) * dt_s);
			}

			// ✅ (B) FIT 로그는 여기 (polyfit 직후)

			const double cte = polyeval(coeffs, 0);
    	    const double epsi = -atan(coeffs[1]); // const add

			// ===== ACC 적용: Solve 한 번에만 ref_v / speed_upperBound 잠깐 낮추기 =====
			const double ref_v_saved = ref_v;
			const double speed_ub_saved = speed_upperBound;
			const double speed_lb_saved = speed_lowerBound;

			const double current_max_speed_limit = is_highway ? speed_limit_highway_max_mps_ : speed_limit_urban_max_mps_;
			if (current_max_speed_limit > 0.0) {
				ref_v = std::min(ref_v, current_max_speed_limit);
				speed_upperBound = std::min(speed_upperBound, current_max_speed_limit);
				if (speed_lowerBound > speed_upperBound) {
					speed_lowerBound = speed_upperBound;
				}
			}

			bool applied_mission_speed = false;
			const double raw_mission_speed_target_mps = missionStateTargetSpeedMps();
			const double actual_speed_now = velocity;

			if (!have_filtered_mission_speed_target_) {
				filtered_mission_speed_target_mps_ = std::max(0.0, actual_speed_now);
				if (filtered_mission_speed_target_mps_ < 1.0) {
					filtered_mission_speed_target_mps_ = std::min(raw_mission_speed_target_mps, 3.0);
				}
				have_filtered_mission_speed_target_ = true;
			} else {
				// 정지/초저속 상태에서 출발할 때 급발진 방지 (현재 속도 + 3.0 m/s 이내로 캡)
				if (actual_speed_now < 1.0 && filtered_mission_speed_target_mps_ > actual_speed_now + 3.0) {
					filtered_mission_speed_target_mps_ = actual_speed_now + 3.0;
				}

				const double up_step = std::max(0.0, mission_speed_rate_limit_up_mps2_) * dt_;
				const double down_step = std::max(0.0, mission_speed_rate_limit_down_mps2_) * dt_;
				filtered_mission_speed_target_mps_ = clampValue(
					raw_mission_speed_target_mps,
					filtered_mission_speed_target_mps_ - down_step,
					filtered_mission_speed_target_mps_ + up_step);
			}
			const double mission_speed_target_mps = filtered_mission_speed_target_mps_;

			publishFloat64(mission_speed_target_pub_, mission_speed_target_mps);
			if (mission_speed_enabled_) {
				ref_v = mission_speed_target_mps;
				speed_upperBound = std::min(speed_upperBound, mission_speed_target_mps);
				if (speed_lowerBound > speed_upperBound) {
					speed_lowerBound = speed_upperBound;
				}
				applied_mission_speed = true;
			}

			bool applied_curve_speed_limit = false;
			const double curve_speed_limit_mps =
				computeCurvatureSpeedLimit(preview_coeffs, preview_x);
			if (curvature_speed_enabled_) {
				ref_v = std::min(ref_v, curve_speed_limit_mps);
				speed_upperBound = std::min(speed_upperBound, curve_speed_limit_mps);
				if (speed_lowerBound > speed_upperBound) {
					speed_lowerBound = speed_upperBound;
				}
				applied_curve_speed_limit = true;
			}

			// obstacle_avoidance_node가 회피/복귀 중일 때 보내는 target_speed 제한.
			// 0.5초 이내에 수신된 값만 유효로 보고, curvature_speed와 동일하게 min()으로 적용.
			bool applied_avoidance_speed_limit = false;
			if (have_avoidance_speed_ &&
				(ros::Time::now() - last_avoidance_speed_time_).toSec() < 0.5 &&
				avoidance_speed_limit_mps_ < 500.0) {
				ref_v = std::min(ref_v, avoidance_speed_limit_mps_);
				speed_upperBound = std::min(speed_upperBound, avoidance_speed_limit_mps_);
				if (speed_lowerBound > speed_upperBound) {
					speed_lowerBound = speed_upperBound;
				}
				applied_avoidance_speed_limit = true;
			}
			this->is_avoidance_mode = applied_avoidance_speed_limit;

			// 정적/unknown 장애물이 confirmed 되거나 카메라가 감지하면 즉시(회피
			// 판단 대기 없이) 절대 목표속도(static_speed_scale_, m/s)로 제한.
			// min()이라 이미 그보다 느리면 자동으로 무시(더 안 줄어듦)되고,
			// 다시 그 속도보다 빨라지면 그때 다시 걸린다.
			bool applied_static_speed_scale = false;
			double static_speed_scale_target_mps = 0.0;
			if (have_static_speed_scale_ &&
				(ros::Time::now() - last_static_speed_scale_time_).toSec() < 0.5 &&
				static_speed_scale_ < 500.0) {
				ref_v = std::min(ref_v, static_speed_scale_);
				speed_upperBound = std::min(speed_upperBound, static_speed_scale_);
				if (speed_lowerBound > speed_upperBound) {
					speed_lowerBound = speed_upperBound;
				}
				applied_static_speed_scale = true;
				static_speed_scale_target_mps = ref_v;
			}

			bool applied_acc_limit = false;

			// New ACC path: target speed is computed from actual longitudinal gap and
			// lead-vehicle speed in smpc_lane_change. Keep the legacy index ACC as a
			// fallback; when both are active, the lower speed limit wins.
			const bool external_acc_fresh = external_acc_enabled_ &&
				have_external_acc_target_ &&
				(ros::Time::now() - external_acc_target_stamp_).toSec() <= external_acc_timeout_sec_;
			if (external_acc_fresh) {
				const double acc_ms = std::max(0.0, external_acc_target_speed_mps_);
				if (acc_ms < 0.01) {
					target_velocity = 0.0;
					is_control_ = true;
					cmd_speed_kmh_ = 0.0;
					cmd_steer_deg_ = (-steering_angle) * 180.0 / M_PI;
					cmd_valid_ = true;
					publishTargetSpeed(target_velocity);
					ref_v = ref_v_saved;
					speed_upperBound = speed_ub_saved;
					speed_lowerBound = speed_lb_saved;
					publishControlCommand();
					is_pose_ = false;
					is_course_ = false;
					return;
				}
				ref_v = std::min(ref_v, acc_ms);
				speed_upperBound = std::min(speed_upperBound, acc_ms);
				speed_lowerBound = 0.0;
				applied_acc_limit = true;
			}

			if (is_acc) {
				//morai
				double spd_state_kmh = cur_speed_ * 3.6;
				//erp

				spd_state_kmh = std::round(spd_state_kmh / 2.0) * 2.0;
				if (spd_state_kmh < 12.0) spd_state_kmh = 12.0;
				if (spd_state_kmh > 20.0) spd_state_kmh = 20.0;

				double acc_kmh = applyACC_kmh(spd_state_kmh, true, acc_index);

				if (acc_kmh > 20.0) acc_kmh = 20.0;
				if (acc_kmh < 0.01)  acc_kmh = 0.0;

				const double acc_ms = acc_kmh / 3.6;

				if (acc_ms < 0.01) {
					target_velocity = 0.0;
					is_control_ = true;

					cmd_speed_kmh_ = 0.0;

					// 조향 부호는 아래 publish 와 같게 맞춘다(MORAI 최종 명령은 부호가 반대).
					cmd_steer_deg_ = (-steering_angle) * 180.0 / M_PI;
					cmd_valid_ = true;
					publishTargetSpeed(target_velocity);
					ref_v = ref_v_saved;
					speed_upperBound = speed_ub_saved;
					speed_lowerBound = speed_lb_saved;
					publishControlCommand();
					is_pose_ = false;
					is_course_ = false;
					return;
				}

				ref_v = std::min(ref_v, acc_ms);
				speed_upperBound = std::min(speed_upperBound, acc_ms);
				speed_lowerBound = 0.0;
				applied_acc_limit = true;
			}

    	    Eigen::VectorXd state(6);
				const double actual_steering =
					(have_previous_steering_command_ && std::fabs(steering_command_sign_) > 1e-6)
						? previous_steering_command_rad_ / steering_command_sign_
						: steering_angle;
				previous_delta_cost_reference = actual_steering;
				state = predict_FutureState(velocity, actual_steering, cte, epsi, target_velocity, dt_);
				if (dc_enabled_) {
					// 지연보상 v2: 조건(PC 부하, 센서 주기, 계산 시간)이 바뀌어도 같은 결과를 내도록
					// PC 에 따라 바뀌는 지연은 매 주기 실측하고, 차량 고유 지연만 고정값으로 둔다.
					const ros::Time now_t = ros::Time::now();
					const double now = now_t.toSec();
					double period = dc_period_s_;
					if (period <= 0.0 || dc_last_imu_rx_.isZero() || (now_t - dc_last_imu_rx_).toSec() > 0.5) {
						period = dc_fallback_period_s_;
					}
					period = clampValue(period, 0.005, 0.2);
					const double odom_age = last_odom_stamp_.isZero() ? 0.0
						: clampValue((now_t - last_odom_stamp_).toSec(), 0.0, 0.2);
					const double state_delay = clampValue(dc_state_k_ * period + dc_state_c_s_ + odom_age,
						0.0, dc_state_max_s_);
					const double compute = clampValue(dc_compute_s_, 0.0, dc_compute_max_s_);
					const double actuation = std::max(0.0, dc_act_base_s_ + dc_act_frame_k_ * period);
					if (!steering_history_.empty() &&
						(now < steering_history_.back().time_s || now - steering_history_.back().time_s > 1.0)) {
						steering_history_.clear();
					}
					// 1) /odom 은 state_delay 만큼 늦다: 실측 속도·요레이트로 현재 시점까지 당긴다.
					const auto lag_pose = mpc_tracking::measuredMotion(velocity, odom_yaw_rate_, state_delay);
					// 2) 지금부터 새 명령이 차량에 반영될 때까지(계산 + 조향 지연) 차량은 이미 보낸
					//    명령(계산 중에는 직전 명령 유지)을 따른다: 명령 이력을 실효 Lf 로 적분한다.
					double lf = dc_model_lf_ > 0.0 ? dc_model_lf_ : Lf;
					if (dc_lf_online_ && dc_lf_est_ > 0.0) lf = clampValue(dc_lf_est_, 3.5, 5.0);
					const auto act_pose = mpc_tracking::predictSteeringDelay(
						velocity, lf, now + compute, actuation + compute, steering_history_);
					const auto pose = mpc_tracking::compose(lag_pose, act_pose);
					const auto errors = mpc_tracking::trackingErrorsAt(
						pose, std::array<double, 4>{{coeffs[0], coeffs[1], coeffs[2], coeffs[3]}});
					state << pose.x, pose.y, pose.yaw, velocity, errors[0], wrapToPi(errors[1]);
					dc_pred_wall_ = ros::WallTime::now();
					dc_pred_pending_ = true;
					std_msgs::Float64MultiArray dbg;
					dbg.data = {period, odom_age, state_delay, compute, actuation, 1.0, lf};
					dc_debug_pub_.publish(dbg);
					publishFloat64(steering_delay_pub_, state_delay + compute + actuation);
				} else {
				// 조향 지연보상(민성 0929): 이미 파이프라인에 들어간 명령을 delay 만큼
				// 적분한 미래 자세에서 MPC 를 시작하고, 그 자세에서 cte/epsi 를 다시 계산한다.
				{
					double delay = steering_actuator_delay_s_;
					nh_.getParamCached("steer/actuator_delay_s", delay);
					delay = clampValue(delay, 0.0, 0.5);
					if (std::fabs(delay - steering_actuator_delay_s_) > 1e-9) {
						ROS_INFO("[MPC] steering delay compensation changed: %.3f -> %.3f s",
								 steering_actuator_delay_s_, delay);
						steering_actuator_delay_s_ = delay;
					}
					double model_lf = steering_delay_model_lf_;
					nh_.getParamCached("steer/delay_model_lf", model_lf);
					steering_delay_model_lf_ = model_lf;
					if (delay > 0.0) {
						const double now = ros::Time::now().toSec();
						if (!steering_history_.empty() &&
							(now < steering_history_.back().time_s || now - steering_history_.back().time_s > 1.0)) {
							steering_history_.clear();
						}
						const double lf = model_lf > 0.0 ? model_lf : Lf;
						const auto pose = mpc_tracking::predictSteeringDelay(
							velocity, lf, now, delay, steering_history_);
						const auto errors = mpc_tracking::trackingErrorsAt(
							pose, std::array<double, 4>{{coeffs[0], coeffs[1], coeffs[2], coeffs[3]}});
						state << pose.x, pose.y, pose.yaw, velocity,
							errors[0], wrapToPi(errors[1]);
					}
					publishFloat64(steering_delay_pub_, delay);
				}
				}

    	    auto output = Solve(state, coeffs);
			if (dc_test_extra_compute_s_ > 0.0) ros::WallDuration(std::min(dc_test_extra_compute_s_, 0.2)).sleep();
			
    	    steering_angle = output[0];
			target_velocity = output[1];

			// 회피 스위치가 이번 사이클에 막 켜졌으면(rising edge), MPC의 점진적
			// 속도 수렴을 기다리지 않고 target_speed로 즉시 스냅한다.
			// 회피 -> 평소 복귀는 여기서 건드리지 않고 기존 방식(점진적) 그대로.
			if (applied_avoidance_speed_limit && !avoidance_speed_limit_was_active_) {
				target_velocity = avoidance_speed_limit_mps_;
			}
			avoidance_speed_limit_was_active_ = applied_avoidance_speed_limit;

			// static_speed_scale(카메라/라이다 감속 킥)도 같은 이유로 rising edge에
			// 즉시 스냅한다. 안 그러면 v_target_change_cost_weight 때문에 MPC가
			// 서서히 수렴해서 "감지되자마자 확 줄어드는" 느낌이 전혀 안 남.
			if (applied_static_speed_scale && !static_speed_scale_was_active_) {
				target_velocity = static_speed_scale_target_mps;
			}
			static_speed_scale_was_active_ = applied_static_speed_scale;

			if (current_max_speed_limit > 0.0) {
				target_velocity = std::min(target_velocity, current_max_speed_limit);
			}

			// 경로 끝 접근 감속.
			//
			// path-end 정지(publishPathEndStop)는 /local_path 가 4점 미만이 돼야
			// 발동하는데, waypoint 간격이 0.1 m 라 그 시점은 끝점 0.3 m 앞이다.
			// 14.9 m/s 로 달려와서 거기서 처음 제동하면 한참 지나쳐서 선다.
			// 남은 호장이 제동거리보다 짧아지면 미리 속도를 눌러, 끝점에
			// 0 m/s 로 도달하게 한다.
			//   v_allow = sqrt(2 * a * max(0, 남은거리 - margin))
			// /local_path 는 앞쪽 일부만 담고 있으므로, 창이 가득 찬 동안에는
			// 남은 호장이 창 길이로 잘려 이 clamp 가 걸리지 않는다. 실제로
			// 경로 끝이 창 안에 들어온 뒤에만(끝점 약 45 m 전) 동작한다.
			if (path_end_stop_enabled_ && waypoints_size_ >= path_end_min_polyfit_points_) {
				double remaining_s = 0.0;
				for (int i = 1; i < waypoints_size_; i++) {
					remaining_s += std::hypot(
						waypoints_[i].pose.pose.position.x - waypoints_[i - 1].pose.pose.position.x,
						waypoints_[i].pose.pose.position.y - waypoints_[i - 1].pose.pose.position.y);
				}
				// 창이 가득 찬(=끝이 아직 안 보이는) 상태에서는 건드리지 않는다.
				if (remaining_s < path_end_approach_trigger_m_) {
					const double brake_s = std::max(0.0, remaining_s - path_end_stop_margin_m_);
					const double v_allow = std::sqrt(
						2.0 * std::max(0.1, path_end_approach_decel_mps2_) * brake_s);
					if (v_allow < target_velocity) {
						target_velocity = v_allow;
						ROS_WARN_THROTTLE(0.5,
							"[MPC] path-end approach: remaining=%.1f m -> v_target=%.1f m/s",
							remaining_s, target_velocity);
					}
				}
			}

			publishTargetSpeed(target_velocity);

			if (applied_mission_speed || applied_curve_speed_limit || applied_acc_limit ||
				applied_avoidance_speed_limit || applied_static_speed_scale || (current_max_speed_limit > 0.0)) {
				ref_v = ref_v_saved;
				speed_upperBound = speed_ub_saved;
				speed_lowerBound = speed_lb_saved;
			}

			std::vector<double> predicted_x;
			std::vector<double> predicted_y;

			for (size_t i = 2; i < output.size(); i += 2) {
    			predicted_x.push_back(output[i]);        // x-coordinate
    			predicted_y.push_back(output[i + 1]);    // y-coordinate
			}

			ROS_INFO_THROTTLE(1.0,
				"[MPC %s] v=%.1f m/s (tgt=%.1f) | Weights: cte=%.0f, epsi=%.0f, delta=%.0f, delta_chg=%.0f | Preview=%.1f m | Cost=%.3f",
				is_highway ? "HIGHWAY" : "URBAN",
				velocity,
				target_velocity,
				cte_cost_effective_weight,
				epsi_cost_effective_weight,
				delta_cost_effective_weight,
				delta_change_cost_effective_weight,
				curvature_preview_distance_m_,
				cost);

    	    // 차량 좌표계 변환된 경로 rviz
    	    nav_msgs::Path predicted_path_rviz;
			predicted_path_rviz.header.frame_id = "base_link";
			predicted_path_rviz.header.stamp = ros::Time::now();

    	    if(N!=0)
			{
				int idx_rowsize = 0;

    	        for(int i=0;i<N;i++)
    	        {
    	            geometry_msgs::PoseStamped loc;
    	            loc.header.frame_id = "base_link";
    	            loc.header.stamp = ros::Time::now();
    	            loc.pose.position.x = predicted_x[i];
    	            loc.pose.position.y = predicted_y[i];

    	            loc.pose.position.z = 0.0;
    	                predicted_path_rviz.poses.emplace_back(loc);
    	        }
			}
			rviz_predicted_path_.publish(predicted_path_rviz);
    	}

		if (is_control_)
    	{
	    	publishControlCommand();
			
    	}
    	is_pose_ = false;
		is_course_ = false;

	}
private:
  double cmd_speed_kmh_ = 0.0;   // PlatformConnector 입력용 (km/h)
  double cmd_steer_deg_ = 0.0;   // PlatformConnector 입력용 (deg, 좌회전 음수)
  bool   cmd_valid_ = false;

};

#endif
