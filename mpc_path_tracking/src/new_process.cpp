#include <mpc_path_tracking/mpc_path_tracking.h>

#include <cmath>

bool ROSCONTROL::determineIsHighway() const
{
	// 1. Waypoint index 및 CSV 번호 확인 (speed_waypoints_ 및 loader_number_)
	int wp_idx = -1;
	int wp_lane = loader_number_;
	if (!speed_waypoints_.empty()) {
		wp_idx = speed_waypoints_.front().waypoint_index;
		wp_lane = speed_waypoints_.front().lane_number;
	}

	// 2. 도심 / 고주로 명시적 구간 전환 규칙:
	// - 0.csv: idx 11124 미만은 도심(Urban), idx 11124부터 고주로(Highway)
	//   2026-10-01: 11000 -> 11124. 규정상 60 km/h 예외는 링크 A2256W000411 시작점
	//   (0.csv idx 11124)부터다. 11000 은 그보다 12.4 m 앞이라 상한이 미리 25 m/s 로 풀렸다.
	// - 1.csv, 2.csv: 고주로 차선변경 및 주행 구간 (Highway)
	// - 3.csv: idx 0 ~ 6196은 고주로 구간(Highway, 규정 끝 = A2256W000153 끝점), idx 6197부터는 M8 음영구간 및 도심 복귀(Urban)
	if (wp_lane == 0) {
		if (wp_idx >= 11124) {
			return true;
		}
		return false;
	} else if (wp_lane == 1 || wp_lane == 2) {
		return true;
	} else if (wp_lane == 3) {
		if (wp_idx >= 0 && wp_idx < 6197) {
			return true;
		} else if (wp_idx >= 6197) {
			return false;
		}
	}

	// 3. 폴백 (SectionState 및 mission_state 기반)
	const bool section_fresh = have_active_section_ &&
		(ros::Time::now() - last_active_section_time_).toSec() < 1.0;
	if (section_fresh) {
		return in_exempt_section_;
	}

	return (current_mission_state_ == 3);
}

double ROSCONTROL::missionStateTargetSpeedMps() const
{
	const bool is_hwy = determineIsHighway();
	const double current_max_speed_limit = is_hwy ? speed_limit_highway_max_mps_ : speed_limit_urban_max_mps_;
	const double effective_speed_upper = (current_max_speed_limit > 0.0)
		? std::min(speed_upper_nominal_, current_max_speed_limit)
		: speed_upper_nominal_;

	const double nominal_speed =
		std::max(speed_lower_nominal_, std::min(ref_v_nominal_, effective_speed_upper));

	if (!mission_speed_enabled_) {
		return nominal_speed;
	}

	// 고주로 구간(0.csv idx 11124 이상, 1/2.csv, 3.csv 6197 미만)에서는
	// 고주로 목표속도(state_3_mps = 25.0 m/s = 90 km/h)를 기본 적용.
	//
	// 단 CSV 가 고주로 안에서 3 이외의 state 를 명시적으로 찍어둔 구간에서는
	// 그 값을 존중한다. 예전엔 여기서 무조건 state_3 을 return 해버려서
	// 아래 switch 가 아예 실행되지 않았고, CSV 를 다시 태깅해도 계속
	// 25.0 m/s 가 나갔다(실측 2026-09-05 16:14 백: state=4 구간 내내
	// mission_speed_target 25.00 고정).
	//   3.csv idx 4000~5299 = state 4 : 곡선 구간 70 km/h
	//   3.csv idx 5300~6197 = state 0 : 도심 복귀 감속 시작(54 km/h)
	// 즉 고주로 판정은 MPC 코스트 가중치(highway 세트)에만 계속 쓰이고,
	// 목표속도는 CSV 태그가 결정한다.
	if (is_hwy && current_mission_state_ == 3) {
		return clampValue(mission_speed_state_mps_[3], speed_lower_nominal_, effective_speed_upper);
	}

	double target_speed = mission_speed_default_mps_;

	// waypoint_system CSV 4번째 컬럼(mission_state)이 /gps_state.current_state로
	// 들어오고, 여기서 state별 목표속도를 선택한다.
	switch (current_mission_state_) {
		case 0:
			target_speed = mission_speed_state_mps_[0];
			break;
		case 1:
			target_speed = mission_speed_state_mps_[1];
			break;
		case 2:
			target_speed = mission_speed_state_mps_[2];
			break;
		case 3:
			target_speed = mission_speed_state_mps_[3];
			break;
		case 4:
			target_speed = mission_speed_state_mps_[4];
			break;
		case 5:
			target_speed = mission_speed_state_mps_[5];
			break;
		case 6:
			target_speed = mission_speed_state_mps_[6];
			break;
		case 7:
			target_speed = mission_speed_state_mps_[7];
			break;
		case 8:
			target_speed = mission_speed_state_mps_[8];
			break;
		case 9:
			target_speed = mission_speed_state_mps_[9];
			break;
		case 10:
			target_speed = mission_speed_state_mps_[10];
			break;
		case 11:
			target_speed = mission_speed_state_mps_[11];
			break;
		case 12:
			target_speed = mission_speed_state_mps_[12];
			break;
		default:
			target_speed = mission_speed_default_mps_;
			break;
	}

	if (!std::isfinite(target_speed) || target_speed <= 0.0) {
		target_speed = nominal_speed;
	}

	return clampValue(target_speed, speed_lower_nominal_, effective_speed_upper);
}
