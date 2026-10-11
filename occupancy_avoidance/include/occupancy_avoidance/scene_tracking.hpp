#pragma once
#include <cmath>
#include <map>
#include <set>
#include <vector>
#include <algorithm>
#include <string>
namespace occupancy_avoidance {
struct SurfaceTrack {
  int id=0,static_hits=0,moving_hits=0;double appearance=-1,stamp=-1,x=0,y=0,vx=0,vy=0;
  std::vector<std::pair<double,double>> surface;std::string motion="unknown";
};
// Match real observed surfaces. Retained boxes and centroids are never movement evidence.
class SurfaceTracker {
  int next_=2000000;std::map<int,SurfaceTrack> tracks_;std::set<int> used_;
 public:
  void rebase(double dx,double dy){for(auto&kv:tracks_){kv.second.x+=dx;kv.second.y+=dy;for(auto&q:kv.second.surface){q.first+=dx;q.second+=dy;}}}
  void begin(double stamp){used_.clear();for(auto it=tracks_.begin();it!=tracks_.end();)if(stamp-it->second.appearance>10||stamp<it->second.appearance)it=tracks_.erase(it);else ++it;}
  SurfaceTrack update(double stamp,const std::vector<std::pair<double,double>>& all,const std::vector<std::pair<double,double>>& observed){
    double x=0,y=0;for(const auto&q:all){x+=q.first;y+=q.second;}if(all.empty())return {};x/=all.size();y/=all.size();
    int id=0;double best=1.5;for(const auto&kv:tracks_)if(!used_.count(kv.first)){double d=std::hypot(x-kv.second.x,y-kv.second.y);if(d<best){best=d;id=kv.first;}}
    SurfaceTrack out=id?tracks_[id]:SurfaceTrack();if(!id)out.id=next_++;
    double dt=stamp-out.stamp;out.motion="unknown";
    if(observed.size()>=3&&out.surface.size()>=3&&dt>=.06&&dt<=.35){
      double ax=0,ay=0,bx=0,by=0;for(auto q:observed){ax+=q.first;ay+=q.second;}for(auto q:out.surface){bx+=q.first;by+=q.second;}
      double dx=ax/observed.size()-bx/out.surface.size(),dy=ay/observed.size()-by/out.surface.size();
      auto overlap=[&](double sx,double sy){std::map<std::pair<int,int>,std::vector<std::pair<double,double>>>bins;for(auto old:out.surface)bins[{int(std::floor((old.first+sx)/.2)),int(std::floor((old.second+sy)/.2))}].push_back({old.first+sx,old.second+sy});int hits=0;
        for(auto q:observed){int ix=std::floor(q.first/.2),iy=std::floor(q.second/.2);bool found=false;for(int di=-1;di<=1&&!found;++di)for(int dj=-1;dj<=1&&!found;++dj){auto it=bins.find({ix+di,iy+dj});if(it!=bins.end())for(auto old:it->second)if(std::hypot(q.first-old.first,q.second-old.second)<.16){found=true;break;}}hits+=found;}return double(hits)/observed.size();};
      double stationary=overlap(0,0),shifted=overlap(dx,dy);double vx=dx/dt,vy=dy/dt;
      bool moves=stationary<.35&&shifted>.75&&shifted>stationary+.35&&std::hypot(dx,dy)>.10&&std::hypot(vx,vy)<8;
      bool consistent=out.moving_hits==0||(vx*out.vx+vy*out.vy>0&&std::hypot(vx-out.vx,vy-out.vy)<2.);
      if(moves&&consistent){out.moving_hits++;out.static_hits=0;out.vx=vx;out.vy=vy;if(out.moving_hits>=2)out.motion="dynamic";}
      else if(stationary>=.5){out.static_hits++;out.moving_hits=0;out.vx=out.vy=0;if(out.static_hits>=1)out.motion="static";}
      else{out.static_hits=out.moving_hits=0;out.vx=out.vy=0;}
    }else if(observed.empty()){out.motion=out.static_hits&&stamp-out.stamp<=.35?"static":"unknown";}
    out.x=x;out.y=y;out.appearance=stamp;
    // A hidden scan cannot replace the last measured surface or its timestamp.
    if(!observed.empty()){out.surface=observed;out.stamp=stamp;}
    if(out.stamp<0)out.stamp=stamp;tracks_[out.id]=out;used_.insert(out.id);return out;
  }
};
}
