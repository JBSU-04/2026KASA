#include "occupancy_avoidance/joint_escape.hpp"
#include "modules/planning/planning_open_space/coarse_trajectory_generator/reeds_shepp_path.h"
#include <queue>
#include <unordered_map>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <tuple>
namespace occupancy_avoidance {
namespace {
double wrap(double h){return std::atan2(std::sin(h),std::cos(h));}
struct Node {Reference p;int phase=0,parent=-1;double reverse=0,g=0,steer=0,travel=0,time=0;std::vector<Reference> segment;};
struct Entry {double cost;int i;bool operator<(const Entry& b)const{return cost>b.cost;}};
struct Rs:apollo::planning::ReedShepp {
 using ReedShepp::ReedShepp;
 using ReedShepp::GenerateRSPs;
 using ReedShepp::GenerateLocalConfigurations;
};
}
EscapePlan jointEscape(const Pose& pose,const std::vector<Reference>& goals,
 const std::function<bool(const Reference&,bool,double)>& clear,double wheelbase,
 double limit,double max_reverse,double budget,bool forward_only){
 EscapePlan out;const auto start=std::chrono::steady_clock::now();
 auto elapsed=[&]{return std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();};
 if(goals.empty()){out.reason="no_safe_exit_state";return out;}
 apollo::common::VehicleParam vehicle;vehicle.set_wheel_base(wheelbase);vehicle.set_max_steer_angle(limit);
 apollo::planning::WarmStartConfig config;config.set_step_size(.08);
 Rs rs(vehicle,config);
 std::vector<Node> nodes(1);nodes[0].p={pose.x,pose.y,pose.yaw,0};
 if(!clear(nodes[0].p,false,0)){out.reason="start_body_collision";return out;}
 auto heuristic=[&](const Reference& p){double h=1e9;for(const auto& g:goals)h=std::min(h,std::hypot(g.x-p.x,g.y-p.y)+.5*std::abs(wrap(g.yaw-p.yaw)));return h;};
 std::priority_queue<Entry> queue;queue.push({heuristic(nodes[0].p),0});
 std::unordered_map<std::string,double> seen;
 std::vector<Reference> rs_tail;std::vector<bool> rs_gear;int winner=-1;
 const double c0=std::cos(pose.yaw),s0=std::sin(pose.yaw);
 while(!queue.empty()&&elapsed()<budget){
  int index=queue.top().i;queue.pop();Node n=nodes[index];++out.expanded;
  double dx=n.p.x-pose.x,dy=n.p.y-pose.y;
  std::string key=std::to_string(int(std::round((c0*dx+s0*dy)/.25)))+":"+std::to_string(int(std::round((-s0*dx+c0*dy)/.25)))+":"+std::to_string(int(std::round(wrap(n.p.yaw)/.0872664626)))+":"+std::to_string(n.phase)+":"+std::to_string(int(n.reverse/.5));
  auto old=seen.find(key);if(old!=seen.end()&&old->second<=n.g)continue;seen[key]=n.g;
  // Analytic expansion uses the UNMODIFIED original Apollo RS generator.
  // Reject multi-cusp candidates before collision checks, preserve <= one R->D.
  if(out.expanded%12==1){
   for(const auto& goal:goals){if(elapsed()>=budget)break;
    auto a=std::make_shared<apollo::planning::Node3d>(n.p.x,n.p.y,n.p.yaw);a->SetDirec(n.phase!=1);
    auto b=std::make_shared<apollo::planning::Node3d>(goal.x,goal.y,goal.yaw);
    std::vector<apollo::planning::ReedSheppPath> paths;if(!rs.GenerateRSPs(a,b,&paths))continue;
    std::sort(paths.begin(),paths.end(),[](const auto& a,const auto& b){return a.total_length<b.total_length;});
    for(auto& path:paths){
     if(!rs.GenerateLocalConfigurations(a,b,&path))continue;
     int phase=n.phase;double reverse=n.reverse,travel=n.travel,time=n.time;bool ok=path.x.size()>1;
     for(size_t j=1;j<path.x.size()&&ok;++j){
      double sample_ds=std::hypot(path.x[j]-path.x[j-1],path.y[j]-path.y[j-1]);if(sample_ds<1e-7)continue;
      const bool back=!path.gear[j];if(back){if(forward_only||phase==2)ok=false;else phase=1;}else phase=2;
      double ds=std::hypot(path.x[j]-path.x[j-1],path.y[j]-path.y[j-1]);
      reverse+=back?ds:0;travel+=ds;time+=ds/(back?1.:2.); if(reverse>max_reverse||ds>.101||!clear({path.x[j],path.y[j],path.phi[j],0},back,time))ok=false;
     }
     if(ok&&phase==2){winner=index;for(size_t j=1;j<path.x.size();++j){rs_tail.push_back({path.x[j],path.y[j],path.phi[j],0});rs_gear.push_back(path.gear[j]);}break;}
     if(elapsed()>=budget)break;
    }if(winner>=0)break;
   }if(winner>=0)break;
  }
  for(bool back:{false,true}){
   if(back&&(forward_only||n.phase==2))continue;
   for(double steering:{0.,-limit,-limit*.5,limit*.5,limit}){
    Node next;next.p=n.p;next.phase=back?1:2;next.parent=index;next.reverse=n.reverse;next.travel=n.travel;next.time=n.time;next.steer=steering;
    bool ok=true;for(int j=0;j<8;++j){
     const double ds=back?-.1:.1;double dh=ds*std::tan(steering)/wheelbase;
     next.p.x+=ds*std::cos(next.p.yaw+dh*.5);next.p.y+=ds*std::sin(next.p.yaw+dh*.5);next.p.yaw=wrap(next.p.yaw+dh);
     next.travel+=.1;next.time+=back?.1:.05;next.reverse+=back?.1:0;
     double xx=next.p.x-pose.x,yy=next.p.y-pose.y;
     if(next.reverse>max_reverse||next.travel>70||std::hypot(xx,yy)>50||!clear(next.p,back,next.time)){ok=false;break;}
     next.segment.push_back(next.p);
    }if(!ok)continue;
    next.g=n.g+.8*(back?1.5:1.)+.1*std::abs(steering)+.2*std::abs(steering-n.steer)+(n.phase==1&&!back?5.:0.);
    nodes.push_back(std::move(next));queue.push({nodes.back().g+heuristic(nodes.back().p),int(nodes.size()-1)});
   }
  }
 }
 if(winner>=0){
  std::vector<int> chain;for(int i=winner;i>0;i=nodes[i].parent)chain.push_back(i);std::reverse(chain.begin(),chain.end());
  Reference last={pose.x,pose.y,pose.yaw,0};bool had_forward=false;
  for(int i:chain){for(const auto& p:nodes[i].segment){
   if(nodes[i].phase==1){if(out.reverse.empty())out.reverse.push_back(last);out.reverse.push_back(p);}
   else{if(out.forward.empty())out.forward.push_back(last);out.forward.push_back(p);had_forward=true;}last=p;}}
  for(size_t j=0;j<rs_tail.size();++j){const auto& p=rs_tail[j];if(!rs_gear[j]){if(had_forward){out.reason="invalid_gear_sequence";return out;}if(out.reverse.empty())out.reverse.push_back(last);out.reverse.push_back(p);}else{if(out.forward.empty())out.forward.push_back(last);out.forward.push_back(p);had_forward=true;}last=p;}
  for(size_t j=0;j<out.reverse.size();++j){double steering=0;if(j+1<out.reverse.size()){double ds=std::hypot(out.reverse[j+1].x-out.reverse[j].x,out.reverse[j+1].y-out.reverse[j].y);out.reverse_distance+=ds;steering=std::atan(-wheelbase*wrap(out.reverse[j+1].yaw-out.reverse[j].yaw)/std::max(.001,ds));}out.reverse_steer.push_back(steering);}
  out.valid=out.forward.size()>1;out.reason=out.reverse.empty()?"forward_hybrid_clear":"reverse_forward_hybrid_clear";
 }else out.reason=elapsed()>=budget?"search_timeout":"no_connected_escape";
 out.compute_ms=elapsed()*1000;return out;
}
}
