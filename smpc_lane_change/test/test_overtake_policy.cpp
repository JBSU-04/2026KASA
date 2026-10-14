// 2026-10-13 차로 진입 추월 판단(overtake_policy.hpp) 회귀 테스트.
#include <smpc_lane_change/overtake_policy.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace smpc_lane_change;
void require(bool value, const char* why) { if (!value) throw std::runtime_error(why); }
int main() {
  OvertakeAccelPolicy p;  // 4.0 / deadband 0.6 / rel_min 1.0 / confirm 0.5
  // 1) 1.0 s 기울기 + 데드밴드: 잡음 섞인 등속(+-0.3 m/s)은 0, 1.8 m/s^2 가속은 그대로.
  SpeedSlope steady, accel;
  for (int k = 0; k <= 10; ++k) {
    const double t = 10.0 + 0.1 * k;
    steady.add(t, 10.0 + ((k % 2) ? 0.3 : -0.3));
    accel.add(t, 10.0 + 1.8 * (t - 10.0));
  }
  double a = 0.0;
  require(steady.slope(11.0, 1.0, 4, a), "steady slope");
  require(accelDeadband(a, p.deadband) == 0.0, "steady inside deadband");
  require(accel.slope(11.0, 1.0, 4, a) && std::abs(a - 1.8) < 1e-6, "accel slope");
  // 2) 처음 방향: 등속 기준 차량은 추월 가능, 3.5 m/s^2 가속(4.0-3.5 < 1.0)은 추월 금지.
  require(overtakeAllowedByAccel(0.0, p), "steady ref overtake ok");
  require(overtakeAllowedByAccel(1.8, p), "4.0 vs 1.8 relative 2.2 ok at start");
  require(!overtakeAllowedByAccel(3.5, p), "relative accel too small");
  require(!overtakeAllowedByAccel(4.2, p), "ref faster than our overtake accel");
  // 3) 진행 중: 14:44:35 유형(자차 실측 0.56, 기준 1.78) -> ref_faster, 0.5 s 뒤 포기.
  require(overtakeAbortReason(1.78, 0.56, true, p) == "ref_faster", "ref faster than measured ego");
  require(overtakeAbortReason(1.8, 2.5, true, p) == "rel_small", "relative 0.7 < 1.0");
  require(overtakeAbortReason(0.5, 0.2, true, p).empty(), "ref in deadband never aborts");
  require(overtakeAbortReason(1.8, 0.0, false, p).empty(), "no ego measurement -> command 4.0, relative 2.2 ok");
  OvertakeAbortMonitor m;
  require(!m.update(2, 38.0, "ref_faster", false, false, p), "not yet confirmed");
  require(!m.update(2, 38.3, "ref_faster", false, false, p), "0.3 s");
  require(m.update(2, 38.5, "ref_faster", false, false, p) && m.reason == "ref_faster", "aborted at 0.5 s");
  // 4) 포기 유지: 조건이 사라져도 같은 기준 차량이면 계속 포기(추월 재전환 금지).
  require(m.update(2, 39.0, "", false, false, p), "abort latched for same ref");
  // 기준 차량이 앞으로 지나가면 해제, 다른 기준 차량이면 새로 시작.
  require(!m.update(2, 40.0, "", false, true, p) && !m.aborted, "released after ref passed");
  m.update(2, 41.0, "ref_faster", false, false, p);
  require(!m.update(7, 41.2, "", false, false, p) && !m.aborted, "new ref resets");
  // 5) 끊긴 조건은 확인 시간을 다시 센다, 자차가 이미 앞질렀으면 포기하지 않는다.
  OvertakeAbortMonitor n;
  n.update(3, 1.0, "rel_small", false, false, p);
  n.update(3, 1.3, "", false, false, p);
  require(!n.update(3, 1.6, "rel_small", false, false, p), "confirm restarts after gap");
  require(!n.update(3, 2.5, "rel_small", true, false, p), "ego already passed");
  std::cout << "overtake policy ok\n";
  return 0;
}
