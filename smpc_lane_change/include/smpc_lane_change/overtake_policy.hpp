#pragma once
// 2026-10-13 차로 진입 추월 판단: 기준 차량(목표 차로에서 자차에 가장 가까운 차량)의 가속을 본다.
//   14:44:35 백: 기준 차량 id 2 를 추월(앞 틈)로 정했는데 id 2 가 1.8 m/s^2 로 같이 가속해
//   자차(실측 0.5~1.0 m/s^2)와 나란히 달리다 차로 끝에서 섰다. 갭 판단은 기준 차량 등속을 가정했다.
//
// 가속도 추정: 최근 window 초 v_long 의 최소제곱 기울기. 프레임 차분 EMA 는 등속 차량에서도
//   p95 2.7 m/s^2 로 튀었고(속도 std 중앙값 0.92), 1.0 s 기울기는 p95 0.61 m/s^2 였다(고주로 백 2개).
//   데드밴드(기본 0.6)보다 작은 값은 가속 아님(0)으로 본다.
#include <cmath>
#include <deque>
#include <string>
#include <utility>

namespace smpc_lane_change {

class SpeedSlope {
 public:
  void add(double t, double v, double keep_sec = 2.0) {
    if (!std::isfinite(t) || !std::isfinite(v)) return;
    if (!samples_.empty() && t < samples_.back().first - 1e-6) samples_.clear();  // 시간 되감김(백 재생)
    if (!samples_.empty() && t - samples_.back().first < 1e-4) samples_.back().second = v;
    else samples_.emplace_back(t, v);
    while (!samples_.empty() && t - samples_.front().first > keep_sec) samples_.pop_front();
  }
  // now 기준 최근 window 초 표본이 min_n 개 이상이고 표본 시간 폭이 window 의 절반 이상이면 기울기.
  bool slope(double now, double window, int min_n, double& out) const {
    double n = 0, st = 0, sv = 0, stt = 0, stv = 0, t_lo = 1e18, t_hi = -1e18;
    for (const auto& s : samples_) {
      if (now - s.first > window || s.first > now + 1e-6) continue;
      const double t = s.first - now;
      n += 1; st += t; sv += s.second; stt += t * t; stv += t * s.second;
      t_lo = std::fmin(t_lo, s.first); t_hi = std::fmax(t_hi, s.first);
    }
    if (n < min_n || t_hi - t_lo < 0.5 * window) return false;
    const double den = n * stt - st * st;
    if (std::abs(den) < 1e-9) return false;
    out = (n * stv - st * sv) / den;
    return std::isfinite(out);
  }
  double lastStamp() const { return samples_.empty() ? -1e18 : samples_.back().first; }

 private:
  std::deque<std::pair<double, double>> samples_;
};

inline double accelDeadband(double a, double deadband) {
  return std::abs(a) < deadband ? 0.0 : a;
}

struct OvertakeAccelPolicy {
  double ego_overtake_accel{4.0};  // 추월 가속 명령(m/s^2)
  double deadband{0.6};            // 기준 차량 가속 데드밴드
  double rel_min{1.0};             // 자차 - 기준 차량 상대 가속 최소
  double confirm_sec{0.5};         // 진행 중 포기 확인 시간
};

// 처음 방향을 고를 때: 기준 차량이 가속 중이고 우리 추월 가속보다 빠르거나 상대 가속이 작으면 추월 금지.
inline bool overtakeAllowedByAccel(double a_ref, const OvertakeAccelPolicy& p) {
  if (a_ref < p.deadband) return true;
  return !(a_ref >= p.ego_overtake_accel || p.ego_overtake_accel - a_ref < p.rel_min);
}

// 진행 중: 자차 실측 가속(a_ego, 없으면 명령값)과 비교한 포기 사유. 없으면 빈 문자열.
inline std::string overtakeAbortReason(double a_ref, double a_ego, bool ego_valid,
                                       const OvertakeAccelPolicy& p) {
  if (a_ref < p.deadband) return "";
  const double ae = ego_valid ? a_ego : p.ego_overtake_accel;
  if (a_ref >= ae) return "ref_faster";
  if (ae - a_ref < p.rel_min) return "rel_small";
  return "";
}

// 기준 차량별 진행 중 포기 상태. 포기하면 그 기준 차량이 바뀌거나(사라짐) 앞으로 지나갈 때까지 유지.
struct OvertakeAbortMonitor {
  int ref_id{-1};
  double since{-1.0};
  bool aborted{false};
  std::string reason;

  void reset(int id) { ref_id = id; since = -1.0; aborted = false; reason.clear(); }

  // why: 이번 주기 포기 사유(빈 문자열이면 없음). ego_passed: 자차가 이미 완전히 앞질렀음.
  // ref_passed: 기준 차량이 자차 앞으로 완전히 지나감(양보 완료) -> 포기 상태 해제.
  bool update(int id, double now, const std::string& why, bool ego_passed, bool ref_passed,
              const OvertakeAccelPolicy& p) {
    if (id != ref_id) reset(id);
    if (ref_passed) { since = -1.0; aborted = false; reason.clear(); return false; }
    if (aborted) return true;
    if (ego_passed || why.empty()) { since = -1.0; return false; }
    if (since < 0.0) since = now;
    if (now - since >= p.confirm_sec) { aborted = true; reason = why; }
    return aborted;
  }
};

}  // namespace smpc_lane_change
