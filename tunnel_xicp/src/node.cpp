#include "tunnel_xicp/backend.hpp"
#include <tunnel_xicp/XicpStatus.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <mission_supervisor/SectionState.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <chrono>
#include <sstream>
#include <algorithm>
#include <atomic>
#include <map>
#include <array>

namespace tx=tunnel_xicp;
using boost::property_tree::ptree;
static constexpr const char* kRevision="0fbe4175ea205a271f85287abfd8048e5f7dd32a";
struct Pose {ros::Time stamp;Eigen::Vector3d position;Eigen::Quaterniond q;};
struct Mask {Eigen::Vector2d xy;double radius;};
static Eigen::Quaterniond quaternion(const geometry_msgs::Quaternion& q){return Eigen::Quaterniond(q.w,q.x,q.y,q.z);}
static double yaw(const Eigen::Quaterniond& q){auto m=q.toRotationMatrix();return std::atan2(m(1,0),m(0,0));}
static std::vector<double> array(const ptree& tree,const std::string& key){std::vector<double>a;for(auto& p:tree.get_child(key))a.push_back(p.second.get_value<double>());return a;}

class Observer {
 public:
  Observer():nh_(),pnh_("~"),listener_(tf_) {
    bool only=true;pnh_.param("observation_only",only,true);
    if(!only)throw std::runtime_error("This node is observation-only. Control mode is not implemented.");
    pnh_.param<std::string>("map_frame",map_frame_,"map");pnh_.param<std::string>("base_frame",base_frame_,"base_link");
    param("max_input_age_sec",max_age_,.3);param("pose_max_gap_sec",pose_gap_,.08);
    param("reference_radius_m",radius_,40);param("wall_map_gate_m",gate_,.8);
    param("wall_z_min",zmin_,0);param("wall_z_max",zmax_,1.5);
    param("wall_x_min",xmin_,1);param("wall_x_max",xmax_,35);
    param("wall_y_abs_min",ymin_,1.5);param("wall_y_abs_max",ymax_,12);
    param("obstacle_mask_margin_m",mask_margin_,.5);param("processing_warn_ms",warn_ms_,30);
    int max_points=800,min_matches=100;pnh_.param("max_input_points",max_points,800);pnh_.param("min_matched_points",min_matches,100);
    max_points_=max_points;policy_.min_matches=min_matches;
    param("min_correspondence_ratio",policy_.min_ratio,.6);param("max_residual_rms_m",policy_.max_rms,.1);
    param("max_lateral_correction_m",policy_.max_lateral,.6);param("observable_projection_min",policy_.observable_min,.95);
    double yaw_deg;param("max_yaw_correction_deg",yaw_deg,3);policy_.max_yaw=yaw_deg*M_PI/180;
    if(max_points<100||min_matches<1||min_matches>max_points||max_age_<=0||pose_gap_<=0||radius_<=0||gate_<=0)
      throw std::runtime_error("invalid observer parameters");
    pnh_.getParam("active_sections",sections_);if(sections_.empty())sections_={"M8"};
    std::string config,map_file,metadata;
    pnh_.getParam("icp_config",config);pnh_.getParam("reference_map",map_file);pnh_.getParam("map_metadata",metadata);
    // Missing map must stay INVALID and must never silently switch to ordinary ICP.
    try {
      ptree meta;boost::property_tree::read_json(metadata,meta);
      if(meta.get<int>("format_version")!=1||meta.get<std::string>("frame")!=map_frame_)throw std::runtime_error("incompatible map metadata");
      auto origin=array(meta,"origin"),offset=array(meta,"sensor_offset");
      if(origin.size()!=3||offset.size()!=3)throw std::runtime_error("invalid map origin/extrinsics");
      origin_=Eigen::Vector3d(origin[0],origin[1],origin[2]);expected_offset_=Eigen::Vector3d(offset[0],offset[1],offset[2]);
      heading_=meta.get<double>("road_heading_rad");source_frame_=meta.get<std::string>("source_frame");
      if(!origin_.allFinite()||!expected_offset_.allFinite()||!std::isfinite(heading_)||source_frame_.empty())throw std::runtime_error("nonfinite map metadata");
      map_.reset(new tx::Cloud);
      if(pcl::io::loadPCDFile(map_file,*map_)<0||map_->size()<policy_.min_matches)throw std::runtime_error("missing/empty measured wall PCD");
      if(map_->size()!=meta.get<size_t>("points"))throw std::runtime_error("map metadata point count mismatch");
      for(const auto& p:*map_){Eigen::Vector3d n(p.normal_x,p.normal_y,p.normal_z);if(!n.allFinite()||std::abs(n.norm()-1)>.02||std::abs(n.z())>.3||!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))throw std::runtime_error("invalid map wall point/normal");}
      map_tree_.setInputCloud(map_);
      backend_.reset(new tx::Backend(config));
    }catch(const std::exception& e){map_error_=e.what();ROS_ERROR_STREAM("[tunnel_xicp] INVALID map/backend: "<<map_error_);}
    status_pub_=nh_.advertise<tx::XicpStatus>("/tunnel/xicp/status",1);
    yaw_pub_=nh_.advertise<geometry_msgs::PoseStamped>("/tunnel/xicp/yaw_measurement",1);
    pose_pub_=nh_.advertise<geometry_msgs::PoseStamped>("/tunnel/xicp/comparison_pose",1);
    marker_pub_=nh_.advertise<visualization_msgs::MarkerArray>("/tunnel/xicp/markers",1);
    std::string cloud_topic,pose_topic;pnh_.param<std::string>("cloud_topic",cloud_topic,"/perception/points_gravity");pnh_.param<std::string>("pose_topic",pose_topic,"/odom");
    cloud_sub_=nh_.subscribe(cloud_topic,1,&Observer::cloud,this);pose_sub_=nh_.subscribe(pose_topic,200,&Observer::odom,this);
    section_sub_=nh_.subscribe("/mission/active_section",1,&Observer::section,this);
    obstacles_sub_=nh_.subscribe("/obstacle_info",1,&Observer::obstacles,this);
    limits_sub_=nh_.subscribe("/tunnel/wall_limits",1,&Observer::limits,this);
    // Reports a stopped input stream even if no new cloud callback arrives.
    watchdog_=nh_.createTimer(ros::Duration(.2),&Observer::watchdog,this);
    worker_=std::thread(&Observer::work,this);
    ROS_INFO("[tunnel_xicp] official X-ICP, OBSERVATION ONLY; no TF/control publishers");
  }
  ~Observer(){stopping_=true;cv_.notify_all();if(worker_.joinable())worker_.join();}
 private:
  void param(const std::string& name,double& value,double def){pnh_.param(name,value,def);}
  void odom(const nav_msgs::OdometryConstPtr& m){
    if(m->header.frame_id!=map_frame_||m->header.stamp.isZero())return;
    auto q=quaternion(m->pose.pose.orientation);Eigen::Vector3d pos(m->pose.pose.position.x,m->pose.pose.position.y,m->pose.pose.position.z);
    if(!q.coeffs().allFinite()||q.norm()<.9||q.norm()>1.1||!pos.allFinite())return;
    std::lock_guard<std::mutex>lock(mu_);
    if(!poses_.empty()&&m->header.stamp<poses_.back().stamp)return;
    if(!poses_.empty()&&m->header.stamp==poses_.back().stamp)poses_.pop_back();
    poses_.push_back({m->header.stamp,pos,q.normalized()});
    while(!poses_.empty()&&(m->header.stamp-poses_.front().stamp).toSec()>3)poses_.pop_front();
  }
  void section(const mission_supervisor::SectionStateConstPtr& m){std::lock_guard<std::mutex>lock(mu_);section_=m->section_id;section_time_=ros::Time::now();}
  void limits(const std_msgs::Float64MultiArrayConstPtr& m){std::lock_guard<std::mutex>lock(mu_);old_limits_=m->data;limits_time_=ros::Time::now();}
  void obstacles(const std_msgs::StringConstPtr& m){
    std::vector<Mask> masks;
    try{
      ptree tree;std::istringstream stream(m->data);boost::property_tree::read_json(stream,tree);
      auto obs=tree.get_child_optional("collision_objects");if(!obs)obs=tree.get_child_optional("obstacles");
      if(obs)for(auto& item:*obs){auto w=array(item.second,"position_world");if(w.size()<2)continue;double l=item.second.get<double>("length",0),b=item.second.get<double>("width",0);
        if(std::isfinite(w[0])&&std::isfinite(w[1])&&std::isfinite(l)&&std::isfinite(b))masks.push_back({Eigen::Vector2d(w[0]-origin_.x(),w[1]-origin_.y()),.5*std::hypot(l,b)+mask_margin_});}
    }catch(const std::exception&){ROS_WARN_THROTTLE(2,"[tunnel_xicp] invalid obstacle mask message");return;}
    std::lock_guard<std::mutex>lock(mu_);masks_=masks;mask_time_=ros::Time::now();
  }
  void cloud(const sensor_msgs::PointCloud2ConstPtr& m){std::lock_guard<std::mutex>lock(mu_);if(pending_)++overwritten_;pending_=m;last_cloud_time_=ros::Time::now();cv_.notify_one();}
  bool poseAt(const ros::Time& stamp,Pose& out){
    if(poses_.empty())return false;
    auto hi=std::lower_bound(poses_.begin(),poses_.end(),stamp,[](const Pose& p,const ros::Time& s){return p.stamp<s;});
    if(hi==poses_.begin()||hi==poses_.end()){
      auto& p=hi==poses_.end()?poses_.back():*hi;if(std::abs((stamp-p.stamp).toSec())>pose_gap_)return false;out=p;out.stamp=stamp;return true;
    }
    auto& b=*hi;auto& a=*(hi-1);double span=(b.stamp-a.stamp).toSec();
    if(span>2*pose_gap_||span<=0)return false;double u=(stamp-a.stamp).toSec()/span;
    out={stamp,a.position*(1-u)+b.position*u,a.q.slerp(u,b.q)};return true;
  }
  void publishInvalid(const ros::Time& stamp,const std::string& reason){tx::Result r;r.reason=reason;publish(stamp,r);clearMarkers(stamp);}
  void watchdog(const ros::TimerEvent&){ros::Time t;{std::lock_guard<std::mutex>lock(mu_);t=last_cloud_time_;}
    if(t.isZero()||(ros::Time::now()-t).toSec()>max_age_)publishInvalid(ros::Time::now(),"cloud_stream_stale");}
  void work(){while(!stopping_&&ros::ok()){
    sensor_msgs::PointCloud2ConstPtr m;Pose pose;bool pose_ok,active;std::vector<Mask> masks;std::vector<double> old;
    {std::unique_lock<std::mutex>lock(mu_);cv_.wait(lock,[&]{return stopping_||bool(pending_);});if(stopping_)break;m=pending_;pending_.reset();
      active=(ros::Time::now()-section_time_).toSec()<=1&&std::find(sections_.begin(),sections_.end(),section_)!=sections_.end();
      pose_ok=poseAt(m->header.stamp,pose);if((ros::Time::now()-mask_time_).toSec()<.3)masks=masks_;
      if((ros::Time::now()-limits_time_).toSec()<.3)old=old_limits_;}
    const auto begin=std::chrono::steady_clock::now();
    if(!active){publishInvalid(m->header.stamp,"outside_tunnel_or_stale_section");continue;}
    if(!backend_){publishInvalid(m->header.stamp,"map_or_backend_unavailable: "+map_error_);continue;}
    double age=(ros::Time::now()-m->header.stamp).toSec();
    if(m->header.stamp.isZero()||age>max_age_||age<-.05){publishInvalid(m->header.stamp,"stale_or_future_cloud");continue;}
    if(!pose_ok){publishInvalid(m->header.stamp,"time_matched_pose_unavailable");continue;}
    if(m->header.frame_id!=source_frame_){publishInvalid(m->header.stamp,"wrong_gravity_frame");continue;}
    try{process(*m,pose,masks,old,begin);}catch(const std::exception& e){publishInvalid(m->header.stamp,std::string("input_exception: ")+e.what());}
  }}
  void process(const sensor_msgs::PointCloud2& m,const Pose& pose,const std::vector<Mask>& masks,const std::vector<double>& old,const std::chrono::steady_clock::time_point& begin){
    if(m.is_bigendian||m.point_step==0||m.row_step!=uint64_t(m.width)*m.point_step||m.data.size()!=uint64_t(m.row_step)*m.height)throw std::runtime_error("unsupported or malformed cloud layout");
    for(const char* name:{"x","y","z"}){
      auto field=std::find_if(m.fields.begin(),m.fields.end(),[&](const sensor_msgs::PointField& f){return f.name==name;});
      if(field==m.fields.end()||field->datatype!=sensor_msgs::PointField::FLOAT32||field->count!=1||uint64_t(field->offset)+4>m.point_step)throw std::runtime_error("invalid xyz cloud fields");
    }
    auto tf=tf_.lookupTransform(base_frame_,source_frame_,m.header.stamp,ros::Duration(0));
    Eigen::Vector3d offset(tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z);
    auto ext_q=quaternion(tf.transform.rotation);
    if((offset-expected_offset_).norm()>.01||ext_q.angularDistance(Eigen::Quaterniond::Identity())>.001){publishInvalid(m.header.stamp,"map_sensor_extrinsic_mismatch");return;}
    Eigen::Matrix4d base=Eigen::Matrix4d::Identity();base.block<3,3>(0,0)=Eigen::AngleAxisd(yaw(pose.q),Eigen::Vector3d::UnitZ()).toRotationMatrix();
    base.block<3,1>(0,3)=pose.position-origin_;
    Eigen::Matrix4d initial=base;initial.block<3,1>(0,3)+=base.block<3,3>(0,0)*offset;
    tx::Cloud ref;for(auto& p:*map_)if((Eigen::Vector3d(p.x,p.y,p.z)-initial.block<3,1>(0,3)).squaredNorm()<=radius_*radius_)ref.push_back(p);
    if(ref.size()<policy_.min_matches){publishInvalid(m.header.stamp,"outside_reference_map");return;}
    tx::Cloud reading;
    sensor_msgs::PointCloud2ConstIterator<float> x(m,"x"),y(m,"y"),z(m,"z");
    std::vector<int>idx(1);std::vector<float>dist(1);
    struct Voxel{Eigen::Vector3d sum=Eigen::Vector3d::Zero();size_t count=0;};
    std::map<std::array<int64_t,3>,Voxel> voxels;
    for(;x!=x.end();++x,++y,++z){
      if(!std::isfinite(*x)||!std::isfinite(*y)||!std::isfinite(*z)||*x<xmin_||*x>xmax_||std::abs(*y)<ymin_||std::abs(*y)>ymax_||*z<zmin_||*z>zmax_)continue;
      // Key ordering spans longitudinal cells then both sides; bounded lookup
      // count is applied BEFORE expensive map queries, not after them.
      const std::array<int64_t,3> key{{int64_t(std::floor(*x/.25)),
                                     int64_t(std::floor(*y/.25)),
                                     int64_t(std::floor(*z/.25))}};
      auto& v=voxels[key];v.sum+=Eigen::Vector3d(*x,*y,*z);++v.count;
    }
    size_t vi=0,selected=0;const size_t lookup_cap=max_points_*2;
    for(const auto& entry:voxels){
      const size_t i=vi++;if(voxels.size()>lookup_cap&&i<selected*voxels.size()/lookup_cap)continue;++selected;
      Eigen::Vector3d sensor=entry.second.sum/entry.second.count;
      Eigen::Vector4d world=initial*Eigen::Vector4d(sensor.x(),sensor.y(),sensor.z(),1);bool masked=false;
      for(auto& o:masks)if((world.head<2>()-o.xy).squaredNorm()<o.radius*o.radius){masked=true;break;}if(masked)continue;
      pcl::PointNormal query;query.x=world.x();query.y=world.y();query.z=world.z();
      if(map_tree_.nearestKSearch(query,1,idx,dist)<1||dist[0]>gate_*gate_)continue;
      const auto& q=(*map_)[idx[0]];Eigen::Vector3d n=initial.block<3,3>(0,0).transpose()*Eigen::Vector3d(q.normal_x,q.normal_y,q.normal_z);
      pcl::PointNormal p;p.x=sensor.x();p.y=sensor.y();p.z=sensor.z();p.normal_x=n.x();p.normal_y=n.y();p.normal_z=n.z();reading.push_back(p);
    }
    if(reading.size()>max_points_){tx::Cloud small;for(size_t i=0;i<max_points_;++i)small.push_back(reading[i*reading.size()/max_points_]);reading.swap(small);}
    tx::Result r=backend_->align(reading,*map_,initial,heading_,policy_);
    // Comparison pose is BASE, not sensor pose; preserve IMU/odom attitude and height.
    Eigen::Vector3d lat(-std::sin(heading_),std::cos(heading_),0);
    Eigen::Matrix4d actual=Eigen::Matrix4d::Identity();actual.block<3,3>(0,0)=pose.q.toRotationMatrix();actual.block<3,1>(0,3)=pose.position-origin_;
    r.comparison_pose=tx::comparisonPose(actual,r.lateral?r.lateral_delta:0,r.yaw?r.yaw_delta:0,lat);
    r.processing_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
    if((ros::Time::now()-m.header.stamp).toSec()>max_age_){r.state=tx::State::INVALID;r.reason="result_expired_during_processing";}
    publish(m.header.stamp,r);visualize(m,reading,ref,initial,r,old);
    if(r.processing_ms>warn_ms_)ROS_WARN_THROTTLE(2,"[tunnel_xicp] processing above budget: %.1f ms",r.processing_ms);
  }
  void publish(const ros::Time& stamp,const tx::Result& r){
    tx::XicpStatus s;s.header.stamp=stamp;s.header.frame_id=map_frame_;s.state=uint8_t(r.state);s.reason=r.reason;s.observation_only=true;
    s.left_observed=r.left;s.right_observed=r.right;s.longitudinal_observable=r.longitudinal;s.lateral_observable=r.lateral;s.yaw_observable=r.yaw;
    s.longitudinal_observability=r.longitudinal_score;s.lateral_observability=r.lateral_score;s.yaw_observability=r.yaw_score;
    s.residual_rms=r.rms;s.correspondence_ratio=r.ratio;s.input_points=r.input_points;s.matched_points=r.matched_points;s.reference_points=r.reference_points;
    s.iterations=r.iterations;s.equality_constraints=r.constraints;s.lateral_correction=r.lateral_delta;s.yaw_correction=r.yaw_delta;s.processing_ms=r.processing_ms;
    s.matching_ms=r.matching_ms;s.verification_ms=r.verification_ms;
    s.preprocessing_ms=std::max(0.,r.processing_ms-r.matching_ms-r.verification_ms);
    s.overwritten_scans=overwritten_.load();s.backend_revision=kRevision;status_pub_.publish(s);
    if(r.state!=tx::State::INVALID){geometry_msgs::PoseStamped p;p.header=s.header;auto pos=r.comparison_pose.block<3,1>(0,3)+origin_;p.pose.position.x=pos.x();p.pose.position.y=pos.y();p.pose.position.z=pos.z();
      auto q=Eigen::Quaterniond(r.comparison_pose.block<3,3>(0,0));p.pose.orientation.x=q.x();p.pose.orientation.y=q.y();p.pose.orientation.z=q.z();p.pose.orientation.w=q.w();pose_pub_.publish(p);if(r.state==tx::State::VALID&&r.yaw&&r.left&&r.right&&ros::Time::now()-stamp<=ros::Duration(max_age_))yaw_pub_.publish(p);}
    ROS_INFO_THROTTLE(2,"[tunnel_xicp] observe state=%u %s rms=%.3f ratio=%.2f lat=%.3f yaw=%.2fdeg %.1fms",s.state,s.reason.c_str(),s.residual_rms,s.correspondence_ratio,s.lateral_correction,s.yaw_correction*180/M_PI,s.processing_ms);
  }
  void clearMarkers(const ros::Time& stamp){visualization_msgs::MarkerArray a;visualization_msgs::Marker m;m.header.frame_id=map_frame_;m.header.stamp=stamp;m.action=visualization_msgs::Marker::DELETEALL;a.markers.push_back(m);marker_pub_.publish(a);}
  void visualize(const sensor_msgs::PointCloud2& msg,const tx::Cloud& reading,const tx::Cloud& ref,const Eigen::Matrix4d& initial,const tx::Result& r,const std::vector<double>& old){
    visualization_msgs::MarkerArray a;
    auto make=[&](int id,int type,float red,float green,float blue){visualization_msgs::Marker m;m.header.stamp=msg.header.stamp;m.header.frame_id=map_frame_;m.ns="xicp_observer";m.id=id;m.type=type;m.action=visualization_msgs::Marker::ADD;m.pose.orientation.w=1;m.scale.x=.06;m.scale.y=.06;m.color.r=red;m.color.g=green;m.color.b=blue;m.color.a=.8;m.lifetime=ros::Duration(.3);return m;};
    auto point=[&](const Eigen::Vector3d& v){geometry_msgs::Point p;p.x=v.x()+origin_.x();p.y=v.y()+origin_.y();p.z=v.z()+origin_.z();return p;};
    auto reference=make(0,visualization_msgs::Marker::POINTS,.5,.5,.5);for(auto& p:ref)reference.points.push_back(point(Eigen::Vector3d(p.x,p.y,p.z)));a.markers.push_back(reference);
    auto observed=make(1,visualization_msgs::Marker::POINTS,0,.8,1);for(auto& p:reading)observed.points.push_back(point((initial*Eigen::Vector4d(p.x,p.y,p.z,1)).head<3>()));a.markers.push_back(observed);
    auto fitted=make(2,visualization_msgs::Marker::POINTS,r.state==tx::State::INVALID?1:0,r.state==tx::State::VALID?1:.5,0);
    for(auto& p:reading)fitted.points.push_back(point((r.raw_transform.cast<double>()*Eigen::Vector4d(p.x,p.y,p.z,1)).head<3>()));a.markers.push_back(fitted);
    auto links=make(3,visualization_msgs::Marker::LINE_LIST,1,.8,0);links.scale.x=.01;
    for(size_t i=0;i<r.reading_matches.size();++i){links.points.push_back(point(r.reading_matches[i]));links.points.push_back(point(r.reference_matches[i]));}a.markers.push_back(links);
    if(old.size()==5&&old[0]>.5){auto lines=make(4,visualization_msgs::Marker::LINE_LIST,1,0,1);lines.scale.x=.04;
      for(int side=0;side<2;++side){double slope=old[3+side],b=(side?old[2]:-old[1])*std::sqrt(1+slope*slope);for(double x:{1.,15.})lines.points.push_back(point((initial*Eigen::Vector4d(x,slope*x+b,0,1)).head<3>()));}a.markers.push_back(lines);}
    auto text=make(5,visualization_msgs::Marker::TEXT_VIEW_FACING,1,1,1);text.scale.z=.4;text.pose.position=point(initial.block<3,1>(0,3)+Eigen::Vector3d(0,0,2));text.text="OBSERVE ONLY: "+r.reason;a.markers.push_back(text);marker_pub_.publish(a);
  }
  ros::NodeHandle nh_,pnh_;tf2_ros::Buffer tf_;tf2_ros::TransformListener listener_;
  ros::Publisher status_pub_,pose_pub_,yaw_pub_,marker_pub_;ros::Subscriber cloud_sub_,pose_sub_,section_sub_,obstacles_sub_,limits_sub_;ros::Timer watchdog_;
  std::mutex mu_;std::condition_variable cv_;std::thread worker_;std::atomic<bool>stopping_{false};std::atomic<uint64_t>overwritten_{0};
  sensor_msgs::PointCloud2ConstPtr pending_;std::deque<Pose>poses_;std::vector<Mask>masks_;std::vector<double>old_limits_;
  std::string section_,map_frame_,base_frame_,source_frame_,map_error_;std::vector<std::string>sections_;
  ros::Time section_time_,mask_time_,limits_time_,last_cloud_time_;
  Eigen::Vector3d origin_=Eigen::Vector3d::Zero(),expected_offset_=Eigen::Vector3d::Zero();
  double heading_=0,max_age_,pose_gap_,radius_,gate_,zmin_,zmax_,xmin_,xmax_,ymin_,ymax_,mask_margin_,warn_ms_;
  size_t max_points_;tx::Policy policy_;tx::Cloud::Ptr map_;std::unique_ptr<tx::Backend>backend_;
  pcl::KdTreeFLANN<pcl::PointNormal> map_tree_;
};
int main(int argc,char** argv){ros::init(argc,argv,"tunnel_xicp_observer");try{Observer observer;ros::spin();}catch(const std::exception& e){ROS_FATAL_STREAM(e.what());return 1;}return 0;}
