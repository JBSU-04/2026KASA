#include "tunnel_xicp/backend.hpp"
#include <pointmatcher/PointMatcher.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <chrono>
#include <algorithm>
#include <fstream>
#include <cmath>
#include <stdexcept>
#include <cstring>

namespace tunnel_xicp {
using PM = PointMatcher<float>;
struct OfficialICP : PM::ICP {
  bool configured() const {
    return localizabilityDetectionParameters.degeneracyAwarenessMethod ==
      PM::DegeneracyAwarenessMethod::kOptimizedEqualityConstraints &&
      localizabilityDetectionParameters.enforceLocalizabilityMethod_;
  }
};
struct Backend::Impl {
  OfficialICP icp;
  PM::DataPoints reference;
  pcl::KdTreeFLANN<pcl::PointNormal> tree;
  uint64_t fingerprint=0;
  bool initialized=false;
};
static uint64_t fingerprint(const Cloud& cloud){
  uint64_t value=1469598103934665603ULL;
  for(const auto& p:cloud)for(float x:{p.x,p.y,p.z,p.normal_x,p.normal_y,p.normal_z}){
    uint32_t bits;std::memcpy(&bits,&x,sizeof(bits));value=(value^bits)*1099511628211ULL;
  }
  return value^cloud.size();
}
static PM::DataPoints dataPoints(const Cloud& c) {
  PM::Matrix f(4,c.size()),n(3,c.size());
  for(size_t i=0;i<c.size();++i) {
    f.col(i)<<c[i].x,c[i].y,c[i].z,1.f;
    n.col(i)<<c[i].normal_x,c[i].normal_y,c[i].normal_z;
  }
  PM::DataPoints::Labels fl,nl;
  fl.emplace_back("x",1);fl.emplace_back("y",1);fl.emplace_back("z",1);fl.emplace_back("pad",1);
  nl.emplace_back("normals",3);
  return PM::DataPoints(f,fl,n,nl);
}
Backend::Backend(const std::string& config):impl_(new Impl) {
  std::ifstream file(config);
  if(!file)throw std::runtime_error("cannot open X-ICP configuration: "+config);
  impl_->icp.loadFromYaml(file);
  if(!impl_->icp.configured())throw std::runtime_error("official X-ICP detection/equality solver not enabled");
}
Backend::~Backend()=default;
void classify(Result& r,const Policy& p) {
  r.state=State::INVALID;
  if(!r.official_xicp_called){r.reason="xicp_not_executed";return;}
  if(!r.raw_transform.allFinite()||!std::isfinite(r.rms)||!std::isfinite(r.lateral_delta)||!std::isfinite(r.yaw_delta)){r.reason="nonfinite_result";return;}
  if(r.matched_points<p.min_matches){r.reason="insufficient_correspondences";return;}
  if(std::abs(r.lateral_delta)>p.max_lateral||std::abs(r.yaw_delta)>p.max_yaw){r.reason="correction_rejected";return;}
  if(r.ratio<p.min_ratio||r.rms>p.max_rms){r.reason="poor_map_alignment";return;}
  if(!r.lateral||!r.yaw){r.state=State::DEGRADED;r.reason="unobservable_lateral_or_yaw";return;}
  if(!r.left||!r.right){r.state=State::DEGRADED;r.reason="single_wall";return;}
  r.state=State::VALID;r.reason="both_walls_observable_lateral_yaw";
}
Eigen::Matrix4d comparisonPose(const Eigen::Matrix4d& initial,double lat,double yaw,const Eigen::Vector3d& lateral) {
  Eigen::Matrix4d pose=initial;
  pose.block<3,1>(0,3)+=lateral*lat;
  pose.block<3,3>(0,0)=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix()*initial.block<3,3>(0,0);
  return pose;
}
Result Backend::align(const Cloud& reading,const Cloud& reference,const Eigen::Matrix4d& initial,double road_heading,const Policy& policy) {
  const auto begin=std::chrono::steady_clock::now();
  Result r;r.input_points=reading.size();r.reference_points=reference.size();r.raw_transform=initial.cast<float>();r.comparison_pose=initial;
  if(reading.size()<policy.min_matches||reference.size()<policy.min_matches){r.reason="insufficient_input_or_map";return r;}
  if(!initial.allFinite()){r.reason="invalid_initial_pose";return r;}
  const Eigen::Vector3d forward(std::cos(road_heading),std::sin(road_heading),0),lateral(-forward.y(),forward.x(),0);
  try {
    const auto hash=fingerprint(reference);
    if(!impl_->initialized||impl_->fingerprint!=hash){
      impl_->reference=dataPoints(reference);
      if(!impl_->icp.initReference(impl_->reference))throw std::runtime_error("official reference initialization failed");
      impl_->tree.setInputCloud(reference.makeShared());
      impl_->fingerprint=hash;impl_->initialized=true;
    }
    const auto matching_begin=std::chrono::steady_clock::now();
    r.raw_transform=impl_->icp.compute(dataPoints(reading),impl_->reference,initial.cast<float>(),false);
    r.matching_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-matching_begin).count();
    const auto verification_begin=std::chrono::steady_clock::now();
    r.official_xicp_called=true;r.iterations=impl_->icp.getTotalNumberOfIterations();r.constraints=impl_->icp.getEqualityConstraintSize();
    // These are the OFFICIAL detected eigen-directions in the registration frame.
    const Eigen::Matrix<double,3,6> dirs=impl_->icp.getDegenerateDirections().cast<double>();
    if(!dirs.allFinite())throw std::runtime_error("nonfinite official observability directions");
    auto score=[&](const Eigen::Vector3d& v,int offset) {
      double lost=0;
      for(int j=offset;j<offset+3;++j){Eigen::Vector3d d=dirs.col(j);if(d.squaredNorm()>1e-8)lost+=std::pow(v.dot(d.normalized()),2);}
      return std::clamp(1.0-lost,0.0,1.0);
    };
    r.longitudinal_score=score(forward,3);r.lateral_score=score(lateral,3);r.yaw_score=score(Eigen::Vector3d::UnitZ(),0);
    r.longitudinal=r.longitudinal_score>=policy.observable_min;r.lateral=r.lateral_score>=policy.observable_min;r.yaw=r.yaw_score>=policy.observable_min;
    const Eigen::Matrix4d aligned=r.raw_transform.cast<double>();
    r.lateral_delta=lateral.dot(aligned.block<3,1>(0,3)-initial.block<3,1>(0,3));
    Eigen::Matrix3d rot=aligned.block<3,3>(0,0)*initial.block<3,3>(0,0).transpose();r.yaw_delta=std::atan2(rot(1,0),rot(0,0));
    r.comparison_pose=comparisonPose(initial,r.lateral?r.lateral_delta:0,r.yaw?r.yaw_delta:0,lateral);
    double sum=0;int left=0,right=0;std::vector<int> idx(1);std::vector<float> distance(1);
    for(const auto& p:reading) {
      Eigen::Vector4d x(p.x,p.y,p.z,1);x=aligned*x;pcl::PointNormal query;query.x=x.x();query.y=x.y();query.z=x.z();
      if(impl_->tree.nearestKSearch(query,1,idx,distance)<1||distance[0]>.25f)continue;
      const auto& q=reference[idx[0]];Eigen::Vector3d ref(q.x,q.y,q.z),normal(q.normal_x,q.normal_y,q.normal_z);
      if(!normal.allFinite()||normal.norm()<.9)continue;
      double residual=normal.normalized().dot(x.head<3>()-ref);sum+=residual*residual;++r.matched_points;
      if(p.y>0)++left;else ++right;
      if(r.reading_matches.size()<100){r.reading_matches.push_back(x.head<3>());r.reference_matches.push_back(ref);}
    }
    r.left=left>=20;r.right=right>=20;r.ratio=double(r.matched_points)/reading.size();
    r.rms=r.matched_points?std::sqrt(sum/r.matched_points):std::numeric_limits<double>::infinity();classify(r,policy);
    r.verification_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-verification_begin).count();
  }catch(const std::exception& e){r.state=State::INVALID;r.reason=std::string("xicp_exception: ")+e.what();}
  r.processing_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();return r;
}
}
