#pragma once
#include <algorithm>
#include <cmath>
#include <deque>
#include <map>
#include <string>
#include <smpc_lane_change/TargetVehicleSet.h>
#include <smpc_lane_change/BehaviorLongitudinalRequest.h>
namespace smpc_lane_change {
struct GapSpeedEstimate {
  double speed{0.0}, observation_stamp{0.0}, evidence_stamp{0.0};
  bool known{false};
  std::string source{"unknown"};
};
// Both decision and selected-gap ACC feed this policy the ORIGINAL target
// header. Timer callbacks never add observations or refresh a trusted sample.
class GapSpeedPolicy {
  struct History {
    int lane{-1}; double last{0}, trusted_stamp{0}, trusted{0};
    std::deque<std::pair<double,double>> positions;
  };
  double last_frame_{0.0};
  std::map<int,History> history_;
  std::map<int,GapSpeedEstimate> estimates_;
 public:
  void update(const TargetVehicleSet& set) {
    const double frame=set.header.stamp.toSec();
    if(frame<=0) return;
    if(last_frame_>0 && frame<last_frame_-.08) {history_.clear();estimates_.clear();}
    last_frame_=frame;
    auto add=[&](const TargetVehicle& v) {
      if(!v.valid || v.unique_id<0 || !std::isfinite(v.observation_age) || v.observation_age<0) return;
      const double stamp=frame-v.observation_age;
      auto& h=history_[v.unique_id];
      if(h.lane!=v.lane_id) { h=History{};h.lane=v.lane_id; }
      if(stamp<=h.last+1e-4) return;
      h.last=stamp;
      GapSpeedEstimate out;out.observation_stamp=stamp;
      if(v.observation_age<=.3 && v.velocity_valid && std::isfinite(v.v_long) &&
         std::isfinite(v.velocity_std) && v.velocity_std>=0 && v.velocity_std<=1.) {
        h.trusted=std::max(0.,v.v_long);h.trusted_stamp=stamp;
        out={h.trusted,stamp,stamp,true,"measured"};
      }
      if(v.observation_age<=.3 && std::isfinite(v.s)) {
        h.positions.emplace_back(stamp,v.s);
        while(h.positions.size()>2 && stamp-h.positions.front().first>1.5) h.positions.pop_front();
      }
      if(!out.known && v.observation_age<=.3 && h.trusted_stamp>0 && stamp-h.trusted_stamp<=3.)
        out={h.trusted,stamp,h.trusted_stamp,true,"trusted_history"};
      if(!out.known && v.observation_age<=.3 && h.positions.size()>=3 && stamp-h.positions.front().first>=.5) {
        double mt=0,ms=0,num=0,den=0;
        for(const auto& p:h.positions){mt+=p.first-stamp;ms+=p.second;}
        mt/=h.positions.size();ms/=h.positions.size();
        for(const auto& p:h.positions){double dt=p.first-stamp-mt;num+=dt*(p.second-ms);den+=dt*dt;}
        const double speed=den>0 ? num/den : -1.;
        double error=0;
        for(const auto& p:h.positions){double e=p.second-ms-speed*(p.first-stamp-mt);error+=e*e;}
        if(std::isfinite(speed) && speed>=0 && std::sqrt(error/h.positions.size())<=.75)
          out={speed,stamp,stamp,true,"position_history"};
      }
      estimates_[v.unique_id]=out;
    };
    add(set.current_front);add(set.target_front);add(set.target_rear);
    for(const auto& v:set.nearby_vehicles) add(v);
    for(auto it=history_.begin();it!=history_.end();)
      if(frame-it->second.last>5.) {estimates_.erase(it->first);it=history_.erase(it);} else ++it;
  }
  GapSpeedEstimate get(const TargetVehicle& v,double now) const {
    const auto it=estimates_.find(v.unique_id);
    if(it==estimates_.end()) return {};
    auto out=it->second;
    if(!v.valid || now-out.observation_stamp<-.08 || now-out.observation_stamp>.3 ||
       !std::isfinite(v.observation_age) || v.observation_age>.3) {out.known=false;out.source="stale";}
    return out;
  }
};
inline bool gapAccelRequest(const BehaviorLongitudinalRequest& r,const TargetVehicleSet& t,
                            double now,double received,double timeout,bool speed_known) {
  return speed_known && r.active && r.merge_gap_selected && r.action==BehaviorLongitudinalRequest::ACCELERATE &&
    r.reason=="smpc_v3_gap_align_accel" && r.current_lane_id==t.current_lane_id &&
    r.target_lane_id==t.target_lane_id && t.current_lane_id>=0 && t.current_lane_id<3 &&
    t.target_lane_id==t.current_lane_id+1 && now-r.header.stamp.toSec()>=0 &&
    now-r.header.stamp.toSec()<=timeout && now-received>=0 && now-received<=timeout &&
    std::isfinite(r.requested_accel_mps2) && r.requested_accel_mps2>0;
}
}
