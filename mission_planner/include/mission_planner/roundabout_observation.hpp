#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <vector>

namespace mission_planner {

struct ObservationPoint { double x=0.0, y=0.0, z=0.0; };
enum class ObservationCell : std::uint8_t { Unknown=0, Free=1, Occupied=2 };

// A scan-local map, not a persistent free-space assumption. Rays terminate at
// actual returns. No return, occlusion, missing fields and stale scans are UNKNOWN.
class RoundaboutObservation {
 public:
  double stamp=-1.0, resolution=.5, min_x=0.0, min_y=0.0;
  int width=0;
  std::vector<ObservationCell> cells;

  ObservationCell at(double x,double y) const {
    if(!std::isfinite(x)||!std::isfinite(y)||width<=0||resolution<=0)return ObservationCell::Unknown;
    const int ix=static_cast<int>(std::floor((x-min_x)/resolution));
    const int iy=static_cast<int>(std::floor((y-min_y)/resolution));
    if(ix<0||iy<0||ix>=width||iy>=width)return ObservationCell::Unknown;
    return cells.at(static_cast<size_t>(iy*width+ix));
  }
  void set(double x,double y,ObservationCell state) {
    const int ix=static_cast<int>(std::floor((x-min_x)/resolution));
    const int iy=static_cast<int>(std::floor((y-min_y)/resolution));
    if(ix<0||iy<0||ix>=width||iy>=width)return;
    auto& cell=cells[static_cast<size_t>(iy*width+ix)];
    if(cell!=ObservationCell::Occupied)cell=state;
  }
  bool clearArc(double cx,double cy,double radius,double angle,double sign,
                double pos,double radial_half) const {
    for(double dr : {-radial_half,0.0,radial_half}) {
      const double a=angle+sign*pos/radius;
      if(at(cx+(radius+dr)*std::cos(a),cy+(radius+dr)*std::sin(a))!=ObservationCell::Free)return false;
    }
    return true;
  }
  // Contiguous observed free arc, limited to half a turn to avoid wrap aliases.
  std::pair<double,double> bounds(double cx,double cy,double radius,double angle,
                                 double sign,double anchor,double radial_half) const {
    if(radius<=0||!clearArc(cx,cy,radius,angle,sign,anchor,radial_half))return {anchor,anchor};
    double lo=anchor,hi=anchor;const double limit=3.141592653589793*radius;
    for(double d=resolution;d<limit;d+=resolution) {
      if(!clearArc(cx,cy,radius,angle,sign,anchor-d,radial_half))break;
      lo=anchor-d;
    }
    for(double d=resolution;d<limit;d+=resolution) {
      if(!clearArc(cx,cy,radius,angle,sign,anchor+d,radial_half))break;
      hi=anchor+d;
    }
    return {lo,hi};
  }
};

inline std::shared_ptr<RoundaboutObservation> buildRoundaboutObservation(
    double cx,double cy,double extent,double resolution,double stamp,
    const ObservationPoint& origin,double ground_z,
    const std::vector<ObservationPoint>& returns,
    const std::vector<ObservationPoint>& nonground) {
  auto out=std::make_shared<RoundaboutObservation>();
  out->stamp=stamp;out->resolution=std::max(.1,resolution);
  out->min_x=cx-extent;out->min_y=cy-extent;
  out->width=static_cast<int>(std::ceil(2*extent/out->resolution));
  out->cells.assign(static_cast<size_t>(out->width*out->width),ObservationCell::Unknown);
  const double xmax=out->min_x+out->width*out->resolution;
  const double ymax=out->min_y+out->width*out->resolution;
  const auto finite=[](const ObservationPoint& p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);};
  if(!finite(origin)||!std::isfinite(ground_z))return out;
  for(const auto& p:returns) {
    if(!finite(p))continue;
    const double dx=p.x-origin.x,dy=p.y-origin.y,dz=p.z-origin.z;
    const double length=std::hypot(dx,dy);if(length<.1)continue;
    // Clip each beam to the small intersection map before traversing it.
    double begin=0.0,end=1.0;
    bool outside=false;
    for(int axis=0;axis<2;++axis) {
      const double s=axis?origin.y:origin.x,d=axis?dy:dx;
      const double low=axis?out->min_y:out->min_x,high=axis?ymax:xmax;
      if(std::abs(d)<1e-9){if(s<low||s>=high)outside=true;continue;}
      double a=(low-s)/d,b=(high-s)/d;if(a>b)std::swap(a,b);
      begin=std::max(begin,a);end=std::min(end,b);
    }
    if(outside||begin>end)continue;
    // Exclude the return's own cell and a short buffer from FREE carving.
    end=std::min(end,1.0-out->resolution/length);
    for(double f=std::max(0.0,begin);f<=end;f+=out->resolution*.5/length) {
      const double height=origin.z+f*dz-ground_z;
      if(height>=.2&&height<=2.5)out->set(origin.x+f*dx,origin.y+f*dy,ObservationCell::Free);
    }
    if(p.z-ground_z>=.2&&p.z-ground_z<=3.0)out->set(p.x,p.y,ObservationCell::Occupied);
  }
  // Non-ground endpoints override ray evidence, including untracked clusters.
  for(const auto& p:nonground)if(finite(p)&&p.z-ground_z>=.15&&p.z-ground_z<=3.0)
    out->set(p.x,p.y,ObservationCell::Occupied);
  return out;
}

struct ObservationHistory {
  std::deque<std::shared_ptr<const RoundaboutObservation>> frames;
  bool add(const std::shared_ptr<const RoundaboutObservation>& frame) {
    if(!frame || frame->stamp<0)return false;
    if(!frames.empty()&&frame->stamp==frames.back()->stamp)return false;
    if(!frames.empty()&&frame->stamp<frames.back()->stamp)frames.clear();
    frames.push_back(frame);while(frames.size()>3)frames.pop_front();return true;
  }
  bool fresh(double now,double timeout=.5) const {
    return !frames.empty()&&now>=frames.back()->stamp&&now-frames.back()->stamp<=timeout;
  }
  bool stable(double now,double timeout=.5) const {
    if(frames.size()<3||!fresh(now,timeout))return false;
    for(size_t i=1;i<frames.size();++i)if(frames[i]->stamp-frames[i-1]->stamp>timeout)return false;
    return now-frames.front()->stamp<=2*timeout;
  }
};

struct ObservationBox {
  double x=0.0,y=0.0,yaw=0.0,length=0.0,width=0.0;
  bool contains(double px,double py,double padding=0.0) const {
    const double dx=px-x,dy=py-y,c=std::cos(yaw),s=std::sin(yaw);
    return std::abs(c*dx+s*dy)<=.5*length+padding &&
           std::abs(-s*dx+c*dy)<=.5*width+padding;
  }
  double separation(const ObservationBox& other) const {
    const double ca=std::cos(yaw),sa=std::sin(yaw),cb=std::cos(other.yaw),sb=std::sin(other.yaw);
    const double c=std::abs(ca*cb+sa*sb),s=std::abs(sa*cb-ca*sb),dx=other.x-x,dy=other.y-y;
    return std::max(std::max(std::abs(dx*ca+dy*sa)-.5*(length+c*other.length+s*other.width),
                            std::abs(-dx*sa+dy*ca)-.5*(width+s*other.length+c*other.width)),
                    std::max(std::abs(dx*cb+dy*sb)-.5*(other.length+c*length+s*width),
                            std::abs(-dx*sb+dy*cb)-.5*(other.width+s*length+c*width)));
  }
};

// Check the actual swept footprint, independently of the capacity of a slot.
// Proven traffic owns its measured cells; its future occupancy is checked by
// the motion predictor. Untracked occupancy and unknown cells remain blockers.
inline ObservationCell observedFootprint(const RoundaboutObservation& map,
    const ObservationBox& body,const ObservationBox& current_ego,
    const std::vector<ObservationBox>& traffic) {
  if(map.width<=0 || map.resolution<=0)return ObservationCell::Unknown;
  const double c=std::abs(std::cos(body.yaw)),s=std::abs(std::sin(body.yaw));
  const double hx=.5*(c*body.length+s*body.width),hy=.5*(s*body.length+c*body.width);
  bool unknown=false;
  for(double x=body.x-hx;x<=body.x+hx+1e-6;x+=map.resolution*.5)
    for(double y=body.y-hy;y<=body.y+hy+1e-6;y+=map.resolution*.5) {
      if(!body.contains(x,y) || current_ego.contains(x,y,.1))continue;
      bool dynamic=false;
      for(const auto& box:traffic)if(box.contains(x,y)){dynamic=true;break;}
      if(dynamic)continue;
      const auto cell=map.at(x,y);
      if(cell==ObservationCell::Occupied)return cell;
      if(cell==ObservationCell::Unknown)unknown=true;
    }
  return unknown?ObservationCell::Unknown:ObservationCell::Free;
}

} // namespace mission_planner
