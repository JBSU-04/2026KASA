// 회전교차로 진입 준폐루프 시뮬레이션 (2026-10-06).
//   자차: 녹화된 자차 궤적(경로)을 따라, 판단기 명령(속도 상한, 정지 목표, 앞차 추종)대로 가감속한다.
//   주변 차량: 트래커 출력 백(녹화 또는 오프라인 재생)의 트랙을 시각에 맞춰 그대로 넣는다(자차에 반응하지 않음).
//   평가: 진입 확정 시각, 취소 횟수, 양보선 대기, 합류·순환 중 확정 차량과의 최소 간격.
// 사용: sim_roundabout_entry MAIN.bag TRACKS.bag SITE_YAML [--topic /replay/tracked_objects] [--verbose]
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <mission_planner_msgs/TrackedObjectArray.h>
#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <yaml-cpp/yaml.h>

#include "mission_planner/roundabout_mission.hpp"

using namespace mission_planner;

RoundaboutConfig loadConfig(const YAML::Node& site);  // replay_roundabout_bag.cpp 와 같은 로더(아래 include)

#define main replay_main_unused
#include "replay_roundabout_bag.cpp"
#undef main

namespace {
struct RoutePoint { double x, y, s, t, v; };

double wrapPi(double a) { return std::atan2(std::sin(a), std::cos(a)); }
}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "usage: sim_roundabout_entry MAIN.bag TRACKS.bag SITE_YAML [--topic T] [--verbose]\n";
    return 1;
  }
  std::string topic = "/replay/tracked_objects";
  bool verbose = false;
  for (int i = 4; i < argc; ++i) {
    const std::string o(argv[i]);
    if (o == "--topic" && i + 1 < argc) topic = argv[++i];
    else if (o == "--verbose") verbose = true;
  }
  ros::Time::init();
  const auto site = YAML::LoadFile(argv[3]);
  const RoundaboutConfig config = loadConfig(site);
  double min_confidence = 0.4;
  if (site["tracked_objects"]) read(site["tracked_objects"], "min_confidence", min_confidence);

  // 1) 녹화 자차 궤적 -> 경로(0.3 m 간격)
  std::vector<RoutePoint> route;
  {
    rosbag::Bag bag(argv[1], rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(std::vector<std::string>{"/odom"}));
    for (const auto& m : view) {
      auto o = m.instantiate<nav_msgs::Odometry>();
      if (!o) continue;
      const double x = o->pose.pose.position.x, y = o->pose.pose.position.y;
      const double v = std::hypot(o->twist.twist.linear.x, o->twist.twist.linear.y);
      if (route.empty()) { route.push_back({x, y, 0.0, m.getTime().toSec(), v}); continue; }
      const double ds = std::hypot(x - route.back().x, y - route.back().y);
      if (ds >= 0.3) route.push_back({x, y, route.back().s + ds, m.getTime().toSec(), v});
    }
  }
  // 2) 트랙
  std::vector<std::pair<double, std::vector<TrackedObject>>> frames;
  std::vector<bool> frame_valid;
  {
    rosbag::Bag bag(argv[2], rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(std::vector<std::string>{topic}));
    for (const auto& m : view) {
      auto a = m.instantiate<mission_planner_msgs::TrackedObjectArray>();
      if (!a) continue;
      std::vector<TrackedObject> objs;
      for (const auto& o : a->objects) {
        if (o.confidence < min_confidence) continue;
        TrackedObject t;
        t.id = o.id; t.position = {o.position.x, o.position.y};
        t.velocity_x = o.velocity.x; t.velocity_y = o.velocity.y;
        t.length = std::max(0.1, std::max(o.size.x, o.size.y));
        t.has_moved = o.has_moved; t.velocity_valid = o.velocity_valid;
        t.confirmed = o.has_moved && o.velocity_valid; t.hit_count = static_cast<int>(o.hit_count);
        objs.push_back(t);
      }
      frames.emplace_back(m.getTime().toSec(), objs);
      frame_valid.push_back(a->valid);
    }
  }
  if (route.size() < 10 || frames.empty()) { std::cerr << "no data\n"; return 1; }

  auto poseAt = [&](double s, double& x, double& y, double& yaw) {
    std::size_t i = 1;
    while (i + 1 < route.size() && route[i].s < s) ++i;
    const auto& a = route[i - 1]; const auto& b = route[i];
    const double f = std::min(1.0, std::max(0.0, (s - a.s) / std::max(1e-6, b.s - a.s)));
    x = a.x + f * (b.x - a.x); y = a.y + f * (b.y - a.y);
    yaw = std::atan2(b.y - a.y, b.x - a.x);
  };
  // 시작: 녹화 자차가 양보선 activation_distance + 5 m 지점을 지나는 순간(녹화 속도 그대로)
  std::size_t start_i = 0;
  for (std::size_t i = 0; i < route.size(); ++i) {
    if (std::hypot(route[i].x - config.yield_point.x, route[i].y - config.yield_point.y) <=
        config.activation_distance + 15.0) { start_i = i; break; }
  }
  double s = route[start_i].s, v = route[start_i].v, t = route[start_i].t;
  const double t_end = std::min(frames.back().first, route.back().t + 30.0);
  RoundaboutMission mission(config);
  std::size_t fi = 0;
  int prev_state = -1, aborts = 0;
  double t_line = -1.0, t_commit = -1.0, t_circ = -1.0, t_exit = -1.0;
  double min_dist_merge = 1e9, min_front_gap = 1e9, min_rear_gap = 1e9, min_rear_headway = 1e9;
  int worst_merge_id = -1;
  const double dt = 0.05;
  const double ego_len = config.gs_ego_length;
  while (t < t_end && s < route.back().s - 1.0) {
    while (fi + 1 < frames.size() && frames[fi + 1].first <= t) ++fi;
    RoundaboutInput in;
    in.now = t;
    double x, y, yaw;
    poseAt(s, x, y, yaw);
    in.ego.position = {x, y}; in.ego.yaw = yaw; in.ego.speed = v; in.ego.valid = true;
    in.localization_valid = true;
    for (std::size_t i = 0; i < route.size(); ++i)
      if (route[i].s >= s - 5.0 && route[i].s <= s + 80.0) in.route.push_back({route[i].x, route[i].y});
    in.objects = frames[fi].second;
    in.objects_stamp = frames[fi].first;
    in.tracker_stream_fresh = t - frames[fi].first <= 0.5;
    in.tracker_stream_valid = frame_valid[fi] && in.tracker_stream_fresh;
    in.prediction_speed_limit = config.circulation_speed;
    in.prediction_accel_limit = 3.0;
    const RoundaboutOutput out = mission.update(in);
    const int st = static_cast<int>(out.state);
    if (st != prev_state) {
      if (prev_state == 3 && st == 2) ++aborts;
      if (st == 3 && t_commit < 0.0) t_commit = t;
      if (st == 3 && std::getenv("SIM_COMMIT_DUMP")) {
        const double ca = std::atan2(config.conflict_point.y - config.center.y, config.conflict_point.x - config.center.x);
        for (const auto& o : in.objects) {
          const double oa = std::atan2(o.position.y - config.center.y, o.position.x - config.center.x);
          const double pp = (config.counter_clockwise ? 1 : -1) * wrapPi(oa - ca) * config.nominal_radius;
          std::cout << "      OBJ id " << o.id << " p " << pp << " r "
                    << std::hypot(o.position.x - config.center.x, o.position.y - config.center.y)
                    << " v " << std::hypot(o.velocity_x, o.velocity_y) << " moved " << o.has_moved
                    << " vvalid " << o.velocity_valid << " hits " << o.hit_count << " len " << o.length << "\n";
        }
      }
      if (st == 4 && t_circ < 0.0) t_circ = t;
      if (st == 5 && t_exit < 0.0) t_exit = t;
      if (verbose) std::cout << t - route[start_i].t << " state=" << st << " v=" << v << " dy=" << out.distance_to_yield << " " << out.reason << "\n";
      prev_state = st;
    }
    if (t_line < 0.0 && st >= 1 && out.distance_to_yield <= 1.5) t_line = t;
    static double last_print = -1.0;
    if (verbose && st == 2 && t - last_print >= 1.0) {
      last_print = t;
      std::cout << "   " << t - route[start_i].t << " v=" << v << " " << out.reason << "\n";
    }
    if (st == 0 && t_exit > 0.0) break;  // 미션 종료

    // 속도 명령
    double vlim = st == 0 ? std::max(v, 8.0) : config.circulation_speed;
    const MissionConstraint& c = out.constraint;
    if (c.active || st != 0) {
      if (c.max_speed >= 0.0) vlim = std::min(vlim, c.max_speed);
      if (c.stop_required) vlim = std::min(vlim, std::sqrt(std::max(0.0, 2.0 * 4.0 * (c.stop_distance - 0.3))));
      if (c.lead_vehicle_active) {
        const double gap_des = 3.0 + 1.0 * v;
        vlim = std::min(vlim, std::max(0.0, c.lead_speed + 0.8 * (c.lead_distance - gap_des)));
      }
    }
    if (std::getenv("SIM_TRACE") && out.distance_to_yield < 8.0 && st >= 1 && st <= 3)
      std::cout << "      t " << t - route[start_i].t << " st " << st << " dy " << out.distance_to_yield << " v " << v
                << " stop " << c.stop_required << " sd " << c.stop_distance << " max " << c.max_speed
                << " vlim " << vlim << "\n";
    v += std::max(-6.0 * dt, std::min(3.0 * dt, vlim - v));  // 실측 접근 감속 ~7 m/s^2
    v = std::max(0.0, v);
    s += v * dt;
    t += dt;

    // 간격 평가(확정 차량만, 링 띠 안)
    const double ego_r = std::hypot(x - config.center.x, y - config.center.y);
    const double ego_a = std::atan2(y - config.center.y, x - config.center.x);
    const double sign = config.counter_clockwise ? 1.0 : -1.0;
    for (const auto& o : frames[fi].second) {
      if (!o.has_moved) continue;
      const double orad = std::hypot(o.position.x - config.center.x, o.position.y - config.center.y);
      if (orad < config.inner_radius - 1.0 || orad > config.outer_radius + 1.0) continue;
      const double dcen = std::hypot(o.position.x - x, o.position.y - y);
      if (st >= 3 && dcen < min_dist_merge) { min_dist_merge = dcen; worst_merge_id = o.id; }
      if (st >= 4 || (st == 3 && ego_r <= config.outer_radius + 1.0)) {
        const double oa = std::atan2(o.position.y - config.center.y, o.position.x - config.center.x);
        const double arc = sign * wrapPi(oa - ego_a) * config.nominal_radius;
        const double bumper = std::abs(arc) - 0.5 * (ego_len + std::max(3.5, o.length));
        if (std::abs(arc) < 25.0) {
          if (std::getenv("SIM_GAPS") && bumper < 1.0)
            std::cout << "      GAP t " << t - route[start_i].t << " st " << st << " id " << o.id << (arc > 0 ? " front " : " rear ")
                      << bumper << " ego_v " << v << " obj_v " << std::hypot(o.velocity_x, o.velocity_y) << "\n";
          if (arc > 0) min_front_gap = std::min(min_front_gap, bumper);
          else {
            min_rear_gap = std::min(min_rear_gap, bumper);
            const double ov = std::hypot(o.velocity_x, o.velocity_y);
            if (ov > 1.0) min_rear_headway = std::min(min_rear_headway, bumper / ov);
          }
        }
      }
    }
  }
  const double t0 = route[start_i].t;
  std::cout.setf(std::ios::fixed);
  std::cout.precision(2);
  std::cout << "line " << (t_line < 0 ? -1 : t_line - t0) << " commit " << (t_commit < 0 ? -1 : t_commit - t0)
            << " wait " << (t_commit < 0 || t_line < 0 ? -1 : t_commit - t_line)
            << " circ " << (t_circ < 0 ? -1 : t_circ - t0) << " exit " << (t_exit < 0 ? -1 : t_exit - t0)
            << " aborts " << aborts << " min_center_dist " << min_dist_merge << " (id " << worst_merge_id << ")"
            << " min_front_gap " << min_front_gap << " min_rear_gap " << min_rear_gap
            << " min_rear_headway " << min_rear_headway << "\n";
}
