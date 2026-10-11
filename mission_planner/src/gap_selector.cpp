#include "mission_planner/gap_selector.hpp"
#include "mission_planner/distance_entry.hpp"

#include <algorithm>
#include <cmath>

namespace mission_planner {
double timedApproachSpeedCap(double remaining, double clearance_gap,
                            double leader_speed, double response,
                            double ego_speed, double acceleration) {
  if (!std::isfinite(remaining) || !std::isfinite(clearance_gap) ||
      !std::isfinite(leader_speed) || !std::isfinite(response)) return 0.0;
  if (clearance_gap >= 0.0) return std::numeric_limits<double>::infinity();
  if (leader_speed <= 0.01 || remaining <= 0.0) return 0.0;
  const double time = -clearance_gap / leader_speed + std::max(0.0,response);
  const double average = remaining / time;
  // A speed target is reached through the existing acceleration ramp. Account
  // for its lost travel, without changing that ramp or crossing before time.
  if (!std::isfinite(ego_speed) || !std::isfinite(acceleration) ||
      ego_speed < 0.0 || acceleration <= 0.0 || average <= ego_speed) return average;
  const double extra = remaining - ego_speed*time;
  const double discriminant = acceleration*acceleration*time*time - 2.0*acceleration*extra;
  if (discriminant <= 0.0) return std::numeric_limits<double>::infinity();
  return ego_speed + 2.0*acceleration*extra /
      (acceleration*time + std::sqrt(discriminant));
}

namespace {
bool usableMeasuredProfile(const EgoEntryModel& ego) {
  if (!ego.measured_profile || ego.profile_times.size() < 2 ||
      ego.profile_times.size() != ego.profile_speeds.size() ||
      ego.profile_times.front() != 0.0 || ego.profile_speeds.front() != 0.0) return false;
  for (size_t i = 0; i < ego.profile_times.size(); ++i) {
    if (!std::isfinite(ego.profile_times[i]) || !std::isfinite(ego.profile_speeds[i]) ||
        (i && (ego.profile_times[i] <= ego.profile_times[i-1] ||
               ego.profile_speeds[i] <= ego.profile_speeds[i-1]))) return false;
  }
  return true;
}
}
double GapSelector::entryAcceleration(const EgoEntryModel& ego, double speed) {
  if (!usableMeasuredProfile(ego)) return std::max(0.1, ego.accel);
  auto it = std::upper_bound(ego.profile_speeds.begin(), ego.profile_speeds.end(), speed);
  if (it == ego.profile_speeds.end()) return 0.0;
  const size_t i = std::max<size_t>(1, it - ego.profile_speeds.begin());
  return (ego.profile_speeds[i] - ego.profile_speeds[i-1]) /
         (ego.profile_times[i] - ego.profile_times[i-1]);
}

void GapSelector::egoProfile(const EgoEntryModel& ego, double t, double* s,
                             double* v) {
  const double v0 = std::max(0.0, ego.speed);
  const double vmax = std::max(v0, ego.max_speed);
  const double a = std::max(0.1, ego.accel);
  // 거의 서 있으면 반응 지연 뒤 출발한다.
  const double delay = !ego.departure_in_progress && v0 < 0.5 ? std::max(0.0, ego.start_delay) : 0.0;
  const double tt = t - delay;
  if (tt <= 0.0) {
    *s = v0 * std::max(0.0, t);
    *v = v0;
    return;
  }
  if (usableMeasuredProfile(ego)) {
    *s = v0 * delay;
    *v = v0;
    const double cap = std::max(v0, std::min(vmax, ego.profile_speeds.back()));
    double remaining = tt;
    // Integrate the measured piecewise acceleration exactly. Gap searches
    // call this repeatedly; stepping the whole profile per sample is wasteful.
    while (remaining > 1e-9 && *v < cap - 1e-9) {
      auto it = std::upper_bound(ego.profile_speeds.begin(), ego.profile_speeds.end(), *v);
      if (it == ego.profile_speeds.end()) break;
      const double acceleration = entryAcceleration(ego, *v);
      const double boundary = std::min(cap, *it);
      const double dt = std::min(remaining, (boundary - *v) / acceleration);
      *s += *v * dt + 0.5 * acceleration * dt * dt;
      *v = std::min(boundary, *v + acceleration * dt);
      remaining -= dt;
    }
    *s += *v * remaining;
    return;
  }
  const double t_acc = (vmax - v0) / a;
  const double s_delay = v0 * delay;
  if (tt <= t_acc) {
    *s = s_delay + v0 * tt + 0.5 * a * tt * tt;
    *v = v0 + a * tt;
  } else {
    *s = s_delay + v0 * t_acc + 0.5 * a * t_acc * t_acc + vmax * (tt - t_acc);
    *v = vmax;
  }
}

bool GapSelector::feasibleAt(const GapCandidate& gap, const EgoEntryModel& ego,
                             const GapSelectorParams& p, double td,
                             double* merge_time, std::string* side,
                             double* shortfall, double* front_shortfall,
                             double* rear_shortfall) {
  const double dt = std::max(0.02, p.sample_dt);
  const double half_ego = 0.5 * std::max(0.1, ego.length);
  const double zone = std::max(0.0, p.entry_zone);
  // 기다리는 동안 자차는 서 있다(출발 지연 td 뒤 정지 출발). td=0 이면
  // 현재 속도에서 바로 가속한다.
  EgoEntryModel model = ego;
  if (td > 1e-6)
    model.speed = 0.0;
  const double lead_v = std::max(0.0, gap.leader_speed - p.speed_margin);
  // 가까운 앞차가 있으면 합류 뒤 자차는 그 속도를 넘지 않는다(ACC 추종).
  //   이걸 빼면 자차가 느린 앞차를 계속 가속해 따라잡는 것으로 계산돼
  //   앞차 간격이 몇 초 뒤 무너지고, 뒤차 간격은 실제보다 낙관적이 된다.
  if (gap.leader_id >= 0 && gap.leader_pos < p.leader_follow_range)
    model.max_speed = std::min(model.max_speed, std::max(1.0, lead_v));
  const double foll_v = std::max(0.0, gap.follower_speed + p.speed_margin);
  const double half_lead = 0.5 * std::max(0.1, gap.leader_length);
  const double half_foll = 0.5 * std::max(0.1, gap.follower_length);

  double t_in = -1.0;
  double worst = 0.0;
  double worst_front = 0.0, worst_rear = 0.0;
  std::string worst_side;
  bool ok = true;
  // 합류 구역 진입 시각을 찾고, 그때부터 check_horizon 동안 검사한다.
  const double t_limit = td + 30.0;
  for (double t = td; t <= t_limit; t += dt) {
    double s = 0.0, v = 0.0;
    egoProfile(model, t - td, &s, &v);
    const double pe = s - ego.distance_to_merge;  // 자차 중심의 흐름 좌표
    if (t_in < 0.0) {
      if (pe + half_ego < -zone)
        continue;
      t_in = t;
    }
    if (t > t_in + p.check_horizon)
      break;
    if (gap.leader_id >= 0) {
      const double pl = gap.leader_pos + lead_v * t;
      const double g = (pl - half_lead) - (pe + half_ego);
      const double need = p.standstill_gap + p.front_headway * v;
      if (g < need) {
        ok = false;
        worst_front = std::max(worst_front, need - g);
        if (need - g > worst) { worst = need - g; worst_side = "front"; }
      }
    }
    if (gap.follower_id >= 0) {
      const double pf = gap.follower_pos + foll_v * t;
      const double g = (pe - half_ego) - (pf + half_foll);
      const double need = p.standstill_gap + p.rear_headway * foll_v;
      if (g < need) {
        ok = false;
        worst_rear = std::max(worst_rear, need - g);
        if (need - g > worst) { worst = need - g; worst_side = "rear"; }
      }
    }
  }
  if (merge_time) *merge_time = t_in;
  if (side) *side = worst_side;
  if (shortfall) *shortfall = worst;
  if (front_shortfall) *front_shortfall = worst_front;
  if (rear_shortfall) *rear_shortfall = worst_rear;
  return ok && t_in >= 0.0;
}

std::vector<GapCandidate> GapSelector::evaluate(
    const std::vector<StreamVehicle>& vehicles_in, const EgoEntryModel& ego,
    const GapSelectorParams& p) {
  std::vector<StreamVehicle> v = vehicles_in;
  // 하류(앞) -> 상류(뒤) 순.
  std::sort(v.begin(), v.end(), [](const StreamVehicle& a,
                                   const StreamVehicle& b) {
    return a.pos > b.pos;
  });
  std::vector<GapCandidate> gaps;
  const size_t n = v.size();
  auto make = [&](const StreamVehicle* lead, double lead_shift,
                  const StreamVehicle* foll, double foll_shift) {
    GapCandidate g;
    if (lead) {
      g.leader_id = lead->id;
      g.leader_pos = lead->pos + lead_shift;
      g.leader_speed = lead->speed;
      g.leader_length = lead->length;
    }
    if (foll) {
      g.follower_id = foll->id;
      g.follower_pos = foll->pos + foll_shift;
      g.follower_speed = foll->speed;
      g.follower_length = foll->length;
    }
    g.wrap = lead_shift > 0.0 ? 1 : (foll_shift < 0.0 ? -1 : 0);
    gaps.push_back(g);
  };
  const bool ring = p.ring_length > 1.0 && n > 0;
  // 맨 앞 틈: 앞차 = (원형이면) 맨 뒤 차가 한 바퀴 앞, 뒤차 = 맨 앞 차.
  if (n == 0) {
    make(nullptr, 0.0, nullptr, 0.0);
  } else {
    make(ring ? &v[n - 1] : nullptr, ring ? p.ring_length : 0.0, &v[0], 0.0);
    for (size_t k = 1; k < n; ++k)
      make(&v[k - 1], 0.0, &v[k], 0.0);
    // 맨 뒤 틈: 앞차 = 맨 뒤 차, 뒤차 = (원형이면) 맨 앞 차가 한 바퀴 뒤.
    make(&v[n - 1], 0.0, ring ? &v[0] : nullptr,
         ring ? -p.ring_length : 0.0);
  }
  // 원형에서 차가 한 대면 두 틈이 같은 차량 쌍이다: '그 차 앞'(뒤차=그 차)과
  // '그 차 뒤'(앞차=그 차). 둘 다 남기고 wrap 으로 구분한다.

  const double step = std::max(0.02, p.search_step);
  for (GapCandidate& g : gaps) {
    double mt = 0.0, sf = 0.0;
    std::string side;
    // 지금 출발할 때의 부족분(표시용).
    feasibleAt(g, ego, p, 0.0, &mt, &g.limit_side, &g.limit_shortfall,
               &g.front_shortfall_now, &g.rear_shortfall_now);
    for (double td = 0.0; td <= p.max_wait + 1e-9; td += step) {
      if (feasibleAt(g, ego, p, td, &mt, &side, &sf)) {
        g.feasible = true;
        g.depart_delay = td;
        g.merge_time = mt;
        break;
      }
    }
  }
  return gaps;
}

double GapSelector::timeToZone(const EgoEntryModel& ego,
                               const GapSelectorParams& p) {
  const double need = ego.distance_to_merge - 0.5 * std::max(0.1, ego.length) -
                      std::max(0.0, p.entry_zone);
  if (need <= 0.0) return 0.0;
  if (usableMeasuredProfile(ego)) {
    double s, v; egoProfile(ego, 30.0, &s, &v);
    if (s < need) return std::numeric_limits<double>::infinity();
    double lo = 0.0, hi = 30.0;
    for (int i = 0; i < 30; ++i) {
      const double mid = 0.5 * (lo + hi); egoProfile(ego, mid, &s, &v);
      if (s < need) lo = mid; else hi = mid;
    }
    return hi;
  }
  const double v0 = std::max(0.0, ego.speed);
  const double vmax = std::max(v0, ego.max_speed);
  const double accel = std::max(0.1, ego.accel);
  const double delay = !ego.departure_in_progress && v0 < 0.5
      ? std::max(0.0, ego.start_delay) : 0.0;
  if (v0 > 0.0 && need <= v0 * delay) return need / v0;
  const double remaining = need - v0 * delay;
  const double accel_time = (vmax - v0) / accel;
  const double accel_distance = v0 * accel_time + 0.5 * accel * accel_time * accel_time;
  const double travel = remaining <= accel_distance
      ? (std::sqrt(v0 * v0 + 2.0 * accel * remaining) - v0) / accel
      : accel_time + (remaining - accel_distance) / std::max(0.01, vmax);
  return std::min(30.0, delay + travel);
}

SequentialResult GapSelector::evaluateSpatial(
    const std::vector<StreamVehicle>& vehicles, const EgoEntryModel& ego,
    const SequentialParams& seq, int target_leader) {
  SequentialResult out;
  out.approach_used = seq.approach_distance >= 0.0;
  if (!std::isfinite(ego.length) || ego.length<=0.0 ||
      !std::isfinite(seq.spatial_front_gap) || seq.spatial_front_gap<0.0 ||
      !std::isfinite(seq.spatial_rear_gap) || seq.spatial_rear_gap<0.0 ||
      !std::isfinite(seq.approach_pos)) {
    out.gap.limit_side="invalid_geometry";return out;
  }
  std::vector<StreamVehicle> cars=vehicles;
  for (const auto& c:cars) {
    if (!std::isfinite(c.pos) || !std::isfinite(c.length) || c.length<=0.0) {
      out.gap.limit_side="invalid_geometry";return out;
    }
  }
  std::sort(cars.begin(),cars.end(),[](const StreamVehicle& a,const StreamVehicle& b){return a.pos>b.pos;});
  const StreamVehicle* lead=nullptr;const StreamVehicle* rear=nullptr;
  if (seq.lock_gap) {
    for (const auto& c:cars) {
      if(c.id==target_leader)lead=&c;
      if(c.id==seq.target_follower)rear=&c;
    }
    if ((target_leader>=0&&!lead)||(seq.target_follower>=0&&!rear)) {
      out.gap.leader_id=out.target_leader=out.resolved_leader=target_leader;
      out.gap.follower_id=out.resolved_follower=seq.target_follower;
      out.gap.limit_side="target_missing";return out;
    }
    for(const auto& c:cars) {
      if(c.id==target_leader||c.id==seq.target_follower)continue;
      if((!lead||c.pos<lead->pos)&&(!rear||c.pos>rear->pos)) {
        out.gap.limit_side="gap_intrusion";return out;
      }
    }
  } else {
    auto it=std::find_if(cars.begin(),cars.end(),[&](const StreamVehicle& c){return c.id==target_leader;});
    if (target_leader==-2 || (target_leader>=0 && it==cars.end())) {
      // Closest downstream car bounds the currently arriving slot.
      for(const auto& c:cars)if(c.pos>=seq.approach_pos)lead=&c;
      it=lead?std::find_if(cars.begin(),cars.end(),[&](const StreamVehicle& c){return c.id==lead->id;}):cars.end();
    } else if(it!=cars.end())lead=&*it;
    if (lead) {if(it+1!=cars.end())rear=&*(it+1);}
    else if(!cars.empty())rear=&cars.front();
  }
  auto& g=out.gap;
  g.leader_id=out.target_leader=out.resolved_leader=lead?lead->id:-1;
  g.follower_id=out.resolved_follower=rear?rear->id:-1;
  if(lead){g.leader_pos=lead->pos;g.leader_length=lead->length;g.leader_speed=lead->speed;}
  if(rear){g.follower_pos=rear->pos;g.follower_length=rear->length;g.follower_speed=rear->speed;}
  const double need=std::max(.1,ego.length)+std::max(0.,seq.spatial_front_gap)+std::max(0.,seq.spatial_rear_gap);
  double low=rear?rear->pos+.5*rear->length:-std::numeric_limits<double>::infinity();
  double high=lead?lead->pos-.5*lead->length:std::numeric_limits<double>::infinity();
  out.physical_free_length=high-low;
  if(seq.observed_slot && !(seq.separate_slot_observation && lead && rear)) {
    if(!seq.observed_bounds){g.limit_side="observation_unknown";return out;}
    // A bounded pair is measured at its free-space centre; open slots are
    // measured at the actual merge, never given infinite capacity.
    const double anchor=lead&&rear?.5*(low+high):seq.approach_pos;
    const auto visible=seq.observed_bounds(anchor);
    low=std::max(low,visible.first);high=std::min(high,visible.second);
    if(high<=low){g.limit_side="observation_unknown";return out;}
  }
  const double free=high-low;
  out.observed_free_length=free;
  if(seq.observed_slot && seq.separate_slot_observation && lead && rear && seq.observed_bounds) {
    const auto measured=seq.observed_bounds(.5*(low+high));
    out.observed_free_length=std::max(0.,std::min(high,measured.second)-std::max(low,measured.first));
  }
  out.rear_margin=free-need;
  g.front_shortfall_now=std::max(0.,need-free);
  if(lead&&rear&&rear->pos>=lead->pos){g.limit_side="gap_order";return out;}
  if(free<need){g.limit_side="slot_space";return out;}
  if(!seq.lock_gap&&rear&&rear->pos+.5*rear->length>=seq.approach_pos-std::max(0.,seq.spatial_rear_gap)) {
    g.limit_side="slot_passed";return out;
  }
  if(!seq.lock_gap&&lead&&seq.leader_start_distance>0. &&
      lead->pos<seq.approach_pos-seq.leader_start_distance) {
    g.limit_side="leader_far";return out;
  }
  out.feasible=out.departure_ready=g.feasible=true;
  g.limit_side="spatial_gap";
  return out;
}

SequentialResult GapSelector::evaluateSequential(
    const std::vector<StreamVehicle>& vehicles, const EgoEntryModel& ego,
    const GapSelectorParams& p, const SequentialParams& seq, int target_leader) {
  SequentialResult out;
  out.approach_used = seq.approach_distance >= 0.0;
  std::vector<StreamVehicle> up, down;
  for (const StreamVehicle& v : vehicles) {
    (v.pos < 0.0 ? up : down).push_back(v);
  }
  // 상류: 가까운 것부터(pos 큰 것부터), 하류: 가까운 것부터.
  std::sort(up.begin(), up.end(),
            [](const StreamVehicle& a, const StreamVehicle& b) { return a.pos > b.pos; });
  std::sort(down.begin(), down.end(),
            [](const StreamVehicle& a, const StreamVehicle& b) { return a.pos < b.pos; });
  auto find = [](const std::vector<StreamVehicle>& list, int id) -> int {
    for (size_t i = 0; i < list.size(); ++i)
      if (list[i].id == id) return static_cast<int>(i);
    return -1;
  };
  int target = target_leader;
  if (!seq.lock_gap && (target == -2 ||
      (target >= 0 && find(up, target) < 0 && find(down, target) < 0))) {
    target = down.empty() ? -1 : down.front().id;
  }
  out.resolved_leader = target;
  const double s0 = seq.front_gap>=0.0?seq.front_gap:p.standstill_gap;
  const double rear_gap = seq.rear_gap>=0.0?seq.rear_gap:p.standstill_gap;
  const double half_ego = 0.5 * std::max(0.1, ego.length);
  const double zone = std::max(0.0, p.entry_zone);
  for (int iter = 0; iter <= seq.max_skip; ++iter) {
    const StreamVehicle* ld = nullptr;
    const StreamVehicle* fl = nullptr;
    if (seq.lock_gap) {
      out.target_leader = out.resolved_leader = target;
      out.resolved_follower = seq.target_follower;
      out.gap.leader_id = target;
      out.gap.follower_id = seq.target_follower;
      for (const auto& car : vehicles) {
        if (car.id == target) ld = &car;
        if (car.id == seq.target_follower) fl = &car;
      }
      if ((target >= 0 && !ld) || (seq.target_follower >= 0 && !fl)) {
        out.gap.limit_side = "target_missing";
        return out;
      }
      // An unseen vehicle appearing inside the chosen slot invalidates it.
      for (const auto& car : vehicles) {
        if (car.id == target || car.id == seq.target_follower) continue;
        if ((!ld || car.pos < ld->pos) && (!fl || car.pos > fl->pos)) {
          out.gap.limit_side = "gap_intrusion";
          return out;
        }
      }
      if (ld && fl && fl->pos >= ld->pos) {
        out.gap.limit_side = "gap_order";
        return out;
      }
    } else if (target < 0) {
      if (!up.empty()) fl = &up.front();
    } else {
      const int ku = find(up, target);
      if (ku >= 0) {
        ld = &up[ku];
        if (ku + 1 < static_cast<int>(up.size())) fl = &up[ku + 1];
      } else {
        ld = &down[find(down, target)];
        if (!up.empty()) fl = &up.front();
      }
    }
    if (iter == 0) out.resolved_follower = fl != nullptr ? fl->id : -1;
    // Use the same capped approach model before and after approval.
    double t_clear = 0.0;
    double lead_v = 0.0;
    EgoEntryModel model = ego;
    if (ld != nullptr) {
      lead_v = std::max(0.0, (ld->speed_lo >= 0.0 ? ld->speed_lo : ld->speed) -
                                  p.speed_margin);
      const double dist = s0 + 0.5 * std::max(0.1, ld->length) - ld->pos;
      if (dist > 0.0) t_clear = lead_v > 0.01 ? dist / lead_v : p.max_wait + 30.0;
      if (seq.follow_time_gap <= 0.0 && ld->pos < p.leader_follow_range)
        model.max_speed = std::min(model.max_speed, std::max(0.1, lead_v));
    }
    const double t_zone = timeToZone(model, p);
    double front_ready_time = t_clear;
    if (ld && lead_v > 0.01) {
      // One integration step of clearance avoids a discretized arrival
      // rounding ahead of the continuous front-clearance prediction.
      const double front_need = s0 + p.front_headway * std::max(model.speed, model.max_speed) +
                                lead_v * 0.02;
      front_ready_time = std::max(t_clear,
          (front_need - zone + 0.5 * ld->length - ld->pos) / lead_v);
    }
    // With the real command profile, waiting/braking for the front is already
    // simulated. Do not also cap entry speed to the leader and delay departure
    // as though we must reach the zone without ever slowing or waiting.
    double td = seq.follow_time_gap > 0.0 ? 0.0 : std::max(0.0, front_ready_time - t_zone);
    double start_distance = seq.leader_start_distance;
    if (ld != nullptr && seq.adaptive_leader_start && start_distance > 0.0) {
      // Match the near-leader gate to our arrival time; a fixed gate can make
      // us wait until a previously feasible slot is no longer reachable.
      const double arrival_distance = lead_v *
          (t_zone + std::max(0.0, seq.departure_confirm_time)) -
          0.5 * std::max(0.1, ld->length) - s0;
      start_distance = std::max(start_distance, std::min(
          std::max(start_distance, seq.leader_start_max_distance), arrival_distance));
    }
    if (!ego.departure_in_progress && ld != nullptr && start_distance > 0.0 && ld->pos < -start_distance) {
      td = std::max(td, (-start_distance - ld->pos) / std::max(0.5, lead_v));
    }
    if (td > p.max_wait) {
      out.target_leader = target;
      out.gap.leader_id = ld ? ld->id : -1;
      out.gap.follower_id = fl ? fl->id : -1;
      out.gap.leader_pos = ld ? ld->pos : 0.0;
      out.gap.leader_speed = ld ? ld->speed : 0.0;
      out.gap.leader_length = ld ? ld->length : 0.0;
      out.gap.follower_pos = fl ? fl->pos : 0.0;
      out.gap.follower_speed = fl ? fl->speed : 0.0;
      out.gap.follower_length = fl ? fl->length : 0.0;
      out.gap.depart_delay = td;
      out.gap.limit_side = "wait_horizon";
      return out;
    }
    // Predict the command we actually issue: acceleration/rate limit, the
    // stop-distance braking envelope until the leader clears, then restart.
    // Both YIELD and COMMIT use this simulation from measured pose/speed.
    double rear_margin = 1e9, rear_ttc = 1e9;
    double arrival_time = -1.0;
    double front_margin = 1e9;
    auto predict = [&]() {
      rear_margin = rear_ttc = front_margin = 1e9;
      out.observation_failure.clear();out.motion_collision_id=-1;
      out.clear_time=-1.0;
      arrival_time = -1.0;
      bool valid_front = true;
      bool rear_cleared = false;
      bool rear_ttc_checked = false;
      double actual_entry_time = -1.0;
      double s = 0.0;
      double v = td > 1e-6 && !ego.departure_in_progress ? 0.0 : model.speed;
      const double departure = ego.departure_in_progress ? 0.0 : td + std::max(0.0,seq.pre_departure_delay);
      double reaction = !ego.departure_in_progress && v < 0.5
          ? std::max(0.0, model.start_delay) : 0.0;
      const double dt = std::max(.01,std::min(.1,seq.integration_dt));
      double rear_clear_time=-1.0;bool started_before_go=false;
      const double hold_s = std::max(0.0, std::min(seq.hold_distance,
          ego.distance_to_merge - half_ego - zone - std::max(0.0, seq.abort_margin)));
      const double decel = std::max(0.5, seq.comfortable_decel);
      for (double t = seq.pre_departure_delay>0.0 ? 0.0 : departure;
           t <= departure + 30.0; t += dt) {
        if(seq.preparation_profile && t>=departure && started_before_go)reaction=0.;
        const double pe = s - ego.distance_to_merge;
        const auto body = seq.project_ego ? seq.project_ego(s) : MergeFootprint{};
        if (seq.project_ego && !body.valid) {valid_front=false;break;}
        if (seq.observation_check) {
          out.observation_failure=seq.observation_check(t,s);
          if(!out.observation_failure.empty()){valid_front=false;break;}
        }
        if (seq.motion_collision) {
          const int collision=seq.motion_collision(t,s);
          if(collision>=0) {out.motion_collision_id=collision;valid_front=false;break;}
        }
        const double occupied_pos = seq.project_ego ? body.pos : pe;
        const double occupied_half = seq.project_ego ? body.half_length : half_ego;
        const double along_speed = seq.project_ego ? std::max(0.0, v * body.speed_ratio) : v;
        const bool in_merge = seq.project_ego ? body.active : pe + half_ego >= -zone - 1e-6;
        if (pe - half_ego >= 0.0) {rear_cleared=true;if(rear_clear_time<0){rear_clear_time=t;out.clear_time=t;}}
        if (actual_entry_time < 0.0 && (seq.project_ego ? body.active : pe + half_ego >= 0.0)) actual_entry_time = t;
        if (in_merge) {
          if (arrival_time < 0.0) arrival_time = t;
          if (ld) {
            const double gap = ld->pos + lead_v * t - 0.5 * ld->length - occupied_pos - occupied_half;
            front_margin = std::min(front_margin, gap - s0 - p.front_headway * along_speed);
            if (seq.hold_for_leader && t + 1e-6 < t_clear) valid_front = false;
          }
          if (fl) {
            const double vf = std::max(0.0,
                (fl->speed_hi >= 0.0 ? fl->speed_hi : fl->speed) + p.speed_margin);
            // A late ego arrival is equivalent to advancing the follower.
            const double braking_elapsed = seq.rear_yield && actual_entry_time >= 0.0
                ? std::max(0.0, t - actual_entry_time - std::max(0.0, seq.rear_reaction)) : 0.0;
            const double a = std::max(0.0, seq.rear_decel);
            const double bt = a > 0.0 ? std::min(braking_elapsed, vf / a) : 0.0;
            const double rear_speed = std::max(0.0, vf - a * bt);
            const double reduction = a * (0.5 * bt * bt + bt * (braking_elapsed - bt));
            const double pf = fl->pos + vf * (t + std::max(0.0, seq.late_margin)) - reduction;
            const double gap = occupied_pos - occupied_half - pf - 0.5 * fl->length;
            rear_margin = std::min(rear_margin, gap - rear_gap - p.rear_headway * rear_speed);
            if (rear_speed > along_speed + 1e-6 &&
                (!seq.rear_ttc_at_entry || !seq.rear_yield || !rear_ttc_checked))
              rear_ttc = std::min(rear_ttc, std::max(0.0, gap) / (rear_speed - along_speed));
            rear_ttc_checked = true;
          }
          // Running minima cannot recover after a violation. Stop invalid
          // deployed predictions early instead of integrating thirty seconds.
          if (seq.require_rear_clear && (!valid_front || front_margin < 0.0 ||
              rear_margin < 0.0 || rear_ttc < seq.ttc_rear)) break;
          if (t>=departure && (!seq.require_rear_clear || rear_cleared) &&
              t >= (seq.check_after_rear_clear ? rear_clear_time : arrival_time) + p.check_horizon) break;
        }
        // Full-body clearance can occur just as the footprint leaves the
        // circulating band. Finish the post-clear horizon outside in_merge too.
        if(seq.check_after_rear_clear && rear_clear_time>=0.0 &&
            t>=rear_clear_time+p.check_horizon)break;
        double cap = std::max(0.1, model.max_speed);
        if(t<departure && seq.pre_departure_delay>0.0) {
          const double room=std::max(0.0,seq.pre_departure_stop_distance-s);
          const double response=seq.preparation_profile?std::max(0.,seq.pre_departure_response):0.;
          const double preparation_cap=std::max(0.,std::sqrt(decel*decel*response*response+2.*decel*room)-decel*response);
          cap=std::min(cap,std::min(seq.pre_departure_max_speed,preparation_cap));
          if(ld && seq.pre_departure_contact_distance>=0.0) {
            const double needed=seq.approach_pos+seq.approach_half+.5*ld->length+
                std::max(s0,seq.follow_standstill_gap);
            if(!seq.preparation_profile)cap=std::min(cap,timedApproachSpeedCap(seq.pre_departure_contact_distance-s-.2,
                ld->pos+lead_v*t-needed,lead_v,seq.approach_time_buffer,v,
                seq.approach_accel_compensation?seq.approach_accel_upper:0.0));
          }
        }
        if (!seq.preparation_profile && seq.project_ego && !body.active && ld && seq.approach_distance >= 0.0) {
          const double needed = seq.approach_pos + seq.approach_half + 0.5*ld->length +
              std::max(s0,seq.follow_standstill_gap);
          cap = std::min(cap,timedApproachSpeedCap(
              seq.approach_distance-s-0.2,ld->pos+lead_v*t-needed,
              lead_v,seq.approach_time_buffer,v,
              seq.approach_accel_compensation ? std::min(std::max(.1,model.accel),entryAcceleration(model,v)) : 0.0));
        }
        if (seq.hold_for_leader && ld && t < t_clear) {
          const double room = std::max(0.0, hold_s - s);
          cap = std::min(cap, room < seq.stop_epsilon ? 0.0 : std::sqrt(2.0 * decel * room));
        }
        if (ld && seq.follow_time_gap > 0.0) {
          const double pl = ld->pos + lead_v * t;
          const double gap = pl - 0.5 * ld->length - occupied_pos - occupied_half;
          // This is the same activation and speed cap used by the command
          // adapter for the selected leader during entry.
          if ((seq.project_ego ? body.active : pl > -0.5 * ld->length) && gap < seq.follow_range) {
            const double usable = std::max(0.0, gap - seq.follow_standstill_gap);
            const double follow_v = !seq.project_ego && ld->physical_speed >= 0.0 && ld->speed > 0.01
                ? ld->physical_speed * lead_v / ld->speed : lead_v;
            const double ratio=seq.project_ego?std::max(0.1,body.speed_ratio):1.0;
            const double braking = seq.front_response > 0.0
                ? frontBrakingSpeedCap(gap, follow_v, seq.follow_standstill_gap,
                    std::max(.5,decel*ratio), seq.front_response, seq.front_max_decel, along_speed)
                : std::sqrt(follow_v * follow_v + 2.0 * decel * usable);
            const double headway = std::max(0.0, follow_v +
                (usable - seq.follow_time_gap * along_speed) / std::max(0.1, seq.follow_time_gap));
            cap = std::min(cap, std::min(braking, headway)/ratio);
          }
        }
        if (seq.require_rear_clear && ld && lead_v <= 0.01 && v <= 0.01 &&
            cap <= 0.01 && t >= departure + reaction) {valid_front=false;break;}
        double next_v = v;
        const bool moving_before_go=t<departure && seq.pre_departure_delay>0.0 &&
            t>=(model.speed<.5?std::max(0.0,model.start_delay):0.0)+
                (seq.preparation_profile?std::max(0.,seq.departure_confirm_time):0.);
        if ((t >= departure + reaction || moving_before_go) &&
            !(cap<v && seq.pre_departure_delay>0.0 && t<seq.pre_departure_response))
          next_v = std::max(0.0, std::min(v + (seq.bound_profile_acceleration?std::min(entryAcceleration(model,v),std::max(.1,model.accel)):entryAcceleration(model,v)) * dt,
                                        std::max(cap, v - decel * dt)));
        if(moving_before_go && next_v>.05)started_before_go=true;
        s += 0.5 * (v + next_v) * dt;
        v = next_v;
      }
      return valid_front && (!seq.require_rear_clear || rear_cleared) && arrival_time >= 0.0 && front_margin >= 0.0 &&
             rear_margin >= 0.0 && rear_ttc >= seq.ttc_rear;
    };
    bool ok = predict();
    if (!ok && !seq.preparation_profile && !ego.departure_in_progress && model.speed <= 0.25 && seq.follow_time_gap > 0.0) {
      // A bad instant must not discard a wide slot that becomes reachable
      // shortly after the front clears. Search this same pair before skipping.
      const double initial_delay = td;
      const double initial_rear = rear_margin, initial_ttc = rear_ttc;
      const double initial_front = front_margin, initial_arrival = arrival_time;
      double deadline = p.max_wait;
      if (fl) {
        const double vf = std::max(0.01,
            (fl->speed_hi >= 0.0 ? fl->speed_hi : fl->speed) + p.speed_margin);
        deadline = std::min(deadline,
            (-zone - 2.0 * half_ego - 0.5 * fl->length - s0 - fl->pos) / vf);
      }
      for (td = initial_delay + std::max(0.02, p.search_step);
           td <= deadline + 1e-9; td += std::max(0.02, p.search_step)) {
        if ((ok = predict())) break;
      }
      if (!ok) {
        td = initial_delay;
        rear_margin = initial_rear; rear_ttc = initial_ttc;
        front_margin = initial_front; arrival_time = initial_arrival;
      }
    }
    // Keep the evaluated target and rejection metrics for diagnostics/display.
    out.gap = GapCandidate();
    out.gap.leader_id = ld ? ld->id : -1;
    out.gap.follower_id = fl ? fl->id : -1;
    out.gap.leader_pos = ld ? ld->pos : 0.0;
    out.gap.leader_length = ld ? ld->length : 0.0;
    out.gap.follower_pos = fl ? fl->pos : 0.0;
    out.gap.follower_length = fl ? fl->length : 0.0;
    out.gap.depart_delay = td;
    out.target_leader = target;
    out.gap.limit_side = front_margin < 0.0 ? "front" : "rear";
    out.gap.front_shortfall_now = std::max(0.0, -front_margin);
    out.gap.rear_shortfall_now = std::max(0.0, -rear_margin);
    out.rear_margin = rear_margin;
    out.front_margin = front_margin;
    out.rear_ttc = rear_ttc;
    if (ok) {
      out.feasible = true;
      out.departure_ready = td <= 1e-6;
      out.target_leader = target;
      GapCandidate& g = out.gap;
      g.feasible = true;
      g.limit_side.clear();
      g.depart_delay = td;
      g.merge_time = arrival_time;
      if (ld) {
        g.leader_id = ld->id; g.leader_pos = ld->pos;
        g.leader_speed = ld->speed; g.leader_length = ld->length;
      }
      if (fl) {
        g.follower_id = fl->id; g.follower_pos = fl->pos;
        g.follower_speed = fl->speed; g.follower_length = fl->length;
      }
      out.rear_margin = rear_margin;
      out.rear_ttc = rear_ttc;
      return out;
    }
    // 패스: 뒤차의 뒤 공간
    if (seq.lock_gap || !fl) return out;
    out.skipped.push_back(fl->id);
    target = fl->id;
  }
  out.target_leader = target;
  return out;
}

}  // namespace mission_planner
