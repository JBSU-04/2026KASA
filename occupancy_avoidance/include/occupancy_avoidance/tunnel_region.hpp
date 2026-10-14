#pragma once
#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <map>
#include <cmath>

namespace occupancy_avoidance {
// The router owns mission bounds; preparing perception never arms control.
struct TunnelRegion {
  int lane=3, first=7201, last=8850, preview=100, shadow_first=7201, exit_hold=100;
  bool contains(int csv,int idx) const { return csv==lane&&idx>=first&&idx<=last; }
  bool prepares(int csv,int idx) const { return csv==lane&&idx>=first-preview&&idx<=last; }
  // 2026-10-13(12:08:53, 12:09:56 백): 엔진은 구간 끝 뒤 exit_hold 까지 경로를 맡을 수 있는데 입력(사전 필터·지도
  //   필터·엔진 입력)은 구간 끝에서 멈춰 stale_lidar -> 회피 실패 -> AEB 로 idx 8890 에서 섰다. 입력과 소유는 이 범위로 맞춘다.
  bool serves(int csv,int idx) const { return csv==lane&&idx>=first-preview&&idx<=last+exit_hold; }
  static TunnelRegion load(const ros::NodeHandle& nh) {
    TunnelRegion r; XmlRpc::XmlRpcValue sections;
    nh.param("/tunnel_recovery/perception_prepare_idx",r.preview,100);
    nh.param("/tunnel_recovery/exit_hold_idx",r.exit_hold,100);if(r.exit_hold<0)r.exit_hold=0;
    if(r.preview<0) throw std::runtime_error("negative tunnel perception preparation range");
    if(nh.getParam("/section_router/sections",sections)) {
      for(int i=0;i<sections.size();++i) {
        auto& s=sections[i];
        if(s.hasMember("id")&&std::string(s["id"])=="M8") {
          r.lane=int(s["csv"]);r.first=int(s["idx"][0]);r.last=int(s["idx"][1]);
          if(r.first>r.last) throw std::runtime_error("invalid M8 bounds");
          r.shadow_first=r.first;
          bool early=false;nh.param("/tunnel_recovery/approach_control_enabled",early,false);
          if(early){
            std::string path;double control=40.,prepare=60.;int anchor=r.first;
            nh.param("/tunnel_recovery/approach_anchor_idx",anchor,r.first);
            nh.getParam("/tunnel_recovery/approach_route_csv",path);
            nh.param("/tunnel_recovery/control_prepare_distance_m",control,40.);
            nh.param("/tunnel_recovery/perception_prepare_distance_m",prepare,60.);
            std::ifstream file(path);if(!file||control<0||prepare<control)throw std::runtime_error("invalid M8 approach route/distance");
            std::map<int,std::pair<double,double>> points;std::string line;
            while(std::getline(file,line)){std::stringstream row(line);std::string id,x,y;if(std::getline(row,id,',')&&std::getline(row,x,',')&&std::getline(row,y,','))points[std::stoi(id)]={std::stod(x),std::stod(y)};}
            if(!points.count(anchor))throw std::runtime_error("M8 anchor absent from approach route");
            int c=anchor,p=anchor,start=r.first;double distance=0;
            for(int j=0;j<sections.size();++j)if(std::string(sections[j]["id"])=="M7B"&&int(sections[j]["csv"])==r.lane)start=int(sections[j]["idx"][0]);
            for(int j=anchor-1;j>=0&&points.count(j);--j){const auto a=points.at(j),b=points.at(j+1);double step=std::hypot(a.first-b.first,a.second-b.second);if(!std::isfinite(step)||step<=0)throw std::runtime_error("invalid M8 route segment");distance+=step;if(distance<=control+1e-6)c=j;if(distance<=prepare+1e-6)p=j;else break;}
            r.first=std::max(start,c);r.preview=r.first-std::max(start,p);
          }
          ROS_INFO("[M8 region] csv=%d control=%d..%d perception=%d..%d",r.lane,r.first,r.last,r.first-r.preview,r.last);
          return r;
        }
      }
      throw std::runtime_error("section_router has no M8 section");
    }
    ROS_WARN("[M8 region] no router configuration; using standalone bounds %d..%d",r.first,r.last);
    return r;
  }
};
}
