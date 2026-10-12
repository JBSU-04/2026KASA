#include "new_lane/m8_prefilter.hpp"
#include "new_lane/tunnel_map_kernel.hpp"
#include "occupancy_avoidance/tunnel_region.hpp"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <nav_msgs/Odometry.h>
#include <waypoint_maker/Lane.h>
#include <mission_supervisor/SectionState.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <json/json.h>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <atomic>
#include <deque>
#include <sstream>
#include <fstream>
using namespace m8_prefilter;
class Prefilter {
 ros::NodeHandle nh_,private_{"~"};occupancy_avoidance::TunnelRegion region_;
 ros::Timer config_timer_;ros::Subscriber cloud_sub_,pose_sub_,route_sub_,section_sub_,offset_sub_;ros::Publisher points_pub_,compute_pub_,queue_pub_,age_pub_,state_pub_;
 Config cfg_;std::shared_ptr<PathIndex>road_,path_;std::vector<std::array<double,3>>walls_;WallZone zone_;std::atomic<double>wall_offset_{0},wall_offset_stamp_{-1e9};WallZoneStats last_zone_;double path_stamp_=-1,path_spacing_=.5;
 std::mutex config_mutex_,input_mutex_,pose_mutex_,path_mutex_;std::condition_variable wake_;std::thread worker_;
 sensor_msgs::PointCloud2ConstPtr pending_;std::chrono::steady_clock::time_point queued_;
 std::deque<tunnel_map::Pose>poses_;std::atomic<bool>active_{false},quit_{false};std::atomic<double>section_stamp_{-1};std::atomic<unsigned>skipped_{0};double last_scan_=-1,last_debug_=-1;
 bool poseAt(double t,tunnel_map::Pose&out){std::lock_guard<std::mutex>lock(pose_mutex_);if(poses_.empty())return false;
  auto i=std::lower_bound(poses_.begin(),poses_.end(),t,[](const tunnel_map::Pose&p,double v){return p.stamp<v;});
  if(i!=poses_.end()&&std::abs(i->stamp-t)<1e-6){out=*i;return true;}
  if(i==poses_.begin()||i==poses_.end())return tunnel_map::extrapolatePose(i==poses_.end()?poses_.back():*i,t,out);
  auto a=*(i-1),b=*i;if(b.stamp-a.stamp>.16||std::min(t-a.stamp,b.stamp-t)>.08)return false;
  double u=(t-a.stamp)/(b.stamp-a.stamp);out=a;out.stamp=t;out.x+=u*(b.x-a.x);out.y+=u*(b.y-a.y);out.z+=u*(b.z-a.z);out.yaw+=u*std::atan2(std::sin(b.yaw-a.yaw),std::cos(b.yaw-a.yaw));return true;
 }
 void pose(const nav_msgs::Odometry::ConstPtr&m){const auto&p=m->pose.pose.position;const auto&q=m->pose.pose.orientation;tunnel_map::Pose v;
  v.stamp=m->header.stamp.toSec();v.x=p.x;v.y=p.y;v.z=p.z;v.yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));v.speed=m->twist.twist.linear.x;v.wz=m->twist.twist.angular.z;v.twist_valid=std::isfinite(v.speed+v.wz);
  std::lock_guard<std::mutex>lock(pose_mutex_);if(!poses_.empty()&&v.stamp<poses_.back().stamp)poses_.clear();poses_.push_back(v);while(poses_.size()>500)poses_.pop_front();
 }
 void route(const waypoint_maker::Lane::ConstPtr&m){std::vector<XY>all;for(const auto&w:m->waypoints)all.push_back({w.pose.pose.position.x,w.pose.pose.position.y});if(all.size()<2)return;
  std::vector<double>steps;for(size_t i=1;i<all.size();++i)steps.push_back(std::hypot(all[i][0]-all[i-1][0],all[i][1]-all[i-1][1]));
  double med=median(steps);size_t step=med>1e-6?std::max(1,int(path_spacing_/med)):1;
  std::vector<XY>p;for(size_t i=0;i<all.size();i+=step)p.push_back(all[i]);if(p.size()<2)p=all;
  {std::lock_guard<std::mutex>lock(path_mutex_);path_stamp_=ros::Time::now().toSec();if(path_&&path_->points==p)return;}
  auto tree=std::make_shared<PathIndex>(std::move(p));std::lock_guard<std::mutex>lock(path_mutex_);path_=tree;
 }
 void cloud(const sensor_msgs::PointCloud2ConstPtr&m){if(!active_)return;
  {std::lock_guard<std::mutex>lock(input_mutex_);if(pending_)++skipped_;pending_=m;queued_=std::chrono::steady_clock::now();}wake_.notify_one();
 }
 void diagnostic(const std::string&reason,double stamp,double wait,double compute,double age,size_t count=0){
  std_msgs::Float64 v;v.data=wait;queue_pub_.publish(v);v.data=compute;compute_pub_.publish(v);v.data=age;age_pub_.publish(v);
  double now=ros::Time::now().toSec();if(now-last_debug_<.5)return;last_debug_=now;
  Json::Value j;j["reason"]=reason;j["stamp"]=stamp;j["queue_ms"]=wait;j["compute_ms"]=compute;j["output_age_ms"]=age;j["points"]=Json::UInt64(count);j["wall_outside_removed"]=last_zone_.outside;j["wall_shape_removed"]=last_zone_.shape;j["skipped_scans"]=unsigned(skipped_);j["input"]="groundgrid_nonground";j["waits_for_detector"]=false;j["waits_for_far"]=false;j["waits_for_map"]=false;
  std_msgs::String text;text.data=Json::FastWriter().write(j);state_pub_.publish(text);
 }
 void reloadConfig(const ros::TimerEvent&){
  XmlRpc::XmlRpcValue root;if(!nh_.getParam("/obstacle_perception_node",root))return;
  std::lock_guard<std::mutex>lock(config_mutex_);Config c=cfg_;
  auto apply=[&](XmlRpc::XmlRpcValue&v){auto read=[&](const char*k,double&value){if(!v.hasMember(k))return;auto type=v[k].getType();if(type==XmlRpc::XmlRpcValue::TypeInt)value=int(v[k]);else if(type==XmlRpc::XmlRpcValue::TypeDouble)value=double(v[k]);};
   read("min_range",c.min_range);read("max_range",c.max_range);read("roi_x_min",c.x_min);read("roi_x_max",c.x_max);read("roi_y_min",c.y_min);read("roi_y_max",c.y_max);read("roi_z_min",c.z_min);read("roi_z_max",c.z_max);read("self_x_min",c.self_x_min);read("self_x_max",c.self_x_max);read("self_y_min",c.self_y_min);read("self_y_max",c.self_y_max);read("path_corridor_gather_half_width",c.gather_half);};
  apply(root);if(root.hasMember("mission_filter_profiles")&&root["mission_filter_profiles"].hasMember("static"))apply(root["mission_filter_profiles"]["static"]);
  cfg_=c;
 }
 void process(const sensor_msgs::PointCloud2ConstPtr&m,double wait){Config cfg;{std::lock_guard<std::mutex>lock(config_mutex_);cfg=cfg_;}auto begin=std::chrono::steady_clock::now();double stamp=m->header.stamp.toSec(),now=ros::Time::now().toSec();
  if(now<last_scan_-.05)last_scan_=-1;
  if(!active_||now-section_stamp_>1||now-stamp>.35||now-stamp<-.05||stamp<=last_scan_){diagnostic("expired_or_inactive",stamp,wait,0,(now-stamp)*1000);return;}
  if(m->header.frame_id!="velodyne_ground"||m->is_bigendian)throw std::runtime_error("unsupported filtered frame/layout");
  tunnel_map::Pose p;if(!poseAt(stamp,p)){diagnostic("scan_pose_unavailable",stamp,wait,0,(now-stamp)*1000);return;}
  int offsets[4]={-1,-1,-1,-1};const char*names[]={"x","y","z","ground_relative"};for(const auto&f:m->fields)for(int i=0;i<4;++i)if(f.name==names[i]&&f.datatype==sensor_msgs::PointField::FLOAT32)offsets[i]=f.offset;
  for(int off:offsets)if(off<0||off+4>int(m->point_step))throw std::runtime_error("missing/invalid GroundGrid height");
  std::shared_ptr<PathIndex>path;{std::lock_guard<std::mutex>lock(path_mutex_);if(now-path_stamp_<1.)path=path_;}
  std::vector<Sample>rows;rows.reserve(m->width*m->height);XY origin{p.x,p.y};
  for(size_t row=0;row<m->height;++row)for(size_t col=0;col<m->width;++col){size_t base=row*m->row_step+col*m->point_step;if(base+m->point_step>m->data.size())throw std::runtime_error("truncated GroundGrid cloud");Sample q;for(int i=0;i<4;++i)std::memcpy(&q[i],m->data.data()+base+offsets[i],4);if(keepCommon(q,cfg,path.get(),origin,p.yaw))rows.push_back(q);}
  // 벽·연석 제한 구역: 경계 밖 점과 경계 안쪽 띠의 세로 벽면 점을 제거한다(벽은 장애물 입력이 아니다).
  double offset=now-wall_offset_stamp_<=.5?double(wall_offset_):0.;
  auto keep=wallZoneKeep(rows,*road_,walls_,origin,p.yaw,cfg.sensor_x,cfg.sensor_y,offset,zone_,&last_zone_);
  sensor_msgs::PointCloud2 out;out.header=m->header;out.height=1;out.is_dense=true;out.is_bigendian=false;out.point_step=24;
  const char*out_names[]={"x","y","z","ground_relative","m2_filtered","m2_prefilter"};for(int i=0;i<6;++i){sensor_msgs::PointField f;f.name=out_names[i];f.offset=i*4;f.datatype=f.FLOAT32;f.count=1;out.fields.push_back(f);}
  out.data.reserve(rows.size()*24);for(size_t i=0;i<rows.size();++i)if(keep[i]){float v[6]={rows[i][0],rows[i][1],rows[i][2],rows[i][3],1,1};auto bytes=reinterpret_cast<const uint8_t*>(v);out.data.insert(out.data.end(),bytes,bytes+24);++out.width;}out.row_step=out.width*24;
  // No map lookup, clustering, tracking, JSON geometry or planning before publish.
  now=ros::Time::now().toSec();if(now-stamp>.35){diagnostic("processing_expired",stamp,wait,0,(now-stamp)*1000);return;}
  points_pub_.publish(out);last_scan_=stamp;
  double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();diagnostic("valid",stamp,wait,ms,(now-stamp)*1000,out.width);
 }
 public:
 Prefilter(){
  // Identical standalone defaults to ObstaclePerceptionNode; launch M2 profile overrides them.
  cfg_.min_range=1.5;cfg_.max_range=15.;cfg_.x_max=20.;cfg_.y_min=-2.8;cfg_.y_max=2.8;
  cfg_.z_min=-1.45;cfg_.z_max=2.5;cfg_.self_x_min=-2.09;cfg_.self_x_max=2.95;cfg_.self_y_min=-.85;cfg_.self_y_max=.85;cfg_.gather_half=4.;
  nh_.param("/obstacle_perception_node/path_corridor_decimate_m",path_spacing_,.5);region_=occupancy_avoidance::TunnelRegion::load(nh_);
  auto param=[&](const char*n,double&v){nh_.param(std::string("/obstacle_perception_node/")+n,v,v);};
  param("min_range",cfg_.min_range);param("max_range",cfg_.max_range);param("roi_x_min",cfg_.x_min);param("roi_x_max",cfg_.x_max);param("roi_y_min",cfg_.y_min);param("roi_y_max",cfg_.y_max);param("roi_z_min",cfg_.z_min);param("roi_z_max",cfg_.z_max);param("self_x_min",cfg_.self_x_min);param("self_x_max",cfg_.self_x_max);param("self_y_min",cfg_.self_y_min);param("self_y_max",cfg_.self_y_max);param("path_corridor_gather_half_width",cfg_.gather_half);
  nh_.param("/lidar_extrinsic/x",cfg_.sensor_x,cfg_.sensor_x);nh_.param("/lidar_extrinsic/y",cfg_.sensor_y,cfg_.sensor_y);nh_.param("/obstacle_perception_node/use_path_corridor",cfg_.use_corridor,true);nh_.param("/obstacle_perception_node/path_corridor_skip_if_no_path",cfg_.skip_missing_path,true);
  XmlRpc::XmlRpcValue profile;if(nh_.getParam("/obstacle_perception_node/mission_filter_profiles/static",profile)){
   auto read=[&](const char*k,double&v){if(profile.hasMember(k))v=profile[k].getType()==XmlRpc::XmlRpcValue::TypeInt?double(int(profile[k])):double(profile[k]);};
   read("roi_x_min",cfg_.x_min);read("roi_x_max",cfg_.x_max);read("roi_y_min",cfg_.y_min);read("roi_y_max",cfg_.y_max);read("roi_z_min",cfg_.z_min);read("roi_z_max",cfg_.z_max);read("path_corridor_gather_half_width",cfg_.gather_half);
  }
  std::string csv;private_.param<std::string>("wall_route",csv,"");std::ifstream file(csv);if(!file)throw std::runtime_error("wall route unavailable");std::vector<XY>road;std::string line;
  while(std::getline(file,line)){std::stringstream row(line);std::string id,x,y;if(std::getline(row,id,',')&&std::getline(row,x,',')&&std::getline(row,y,','))road.push_back({std::stod(x),std::stod(y)});}if(road.size()<2)throw std::runtime_error("wall route empty");road_=std::make_shared<PathIndex>(std::move(road));
  std::vector<double>flat;if(!private_.getParam("tunnel_wall_profile",flat)||flat.size()%3||flat.size()<6)throw std::runtime_error("physical wall profile unavailable");for(size_t i=0;i<flat.size();i+=3)walls_.push_back({flat[i],flat[i+1],flat[i+2]});
  private_.param("wall_margin_m",zone_.margin,zone_.margin);private_.param("wall_band_m",zone_.band,zone_.band);private_.param("wall_min_extent_m",zone_.min_extent,zone_.min_extent);private_.param("wall_min_top_m",zone_.min_top,zone_.min_top);
  private_.param("wall_min_run_m",zone_.min_run,zone_.min_run);private_.param("wall_cell_m",zone_.cell,zone_.cell);
  {std::vector<double>steps;for(size_t i=1;i<road_->points.size();++i)steps.push_back(std::hypot(road_->points[i][0]-road_->points[i-1][0],road_->points[i][1]-road_->points[i-1][1]));zone_.spacing=std::max(1e-3,median(steps));}
  offset_sub_=nh_.subscribe<std_msgs::Float64>("/tunnel/wall_offset",1,[this](const std_msgs::Float64::ConstPtr&m){if(std::isfinite(m->data)){wall_offset_=m->data;wall_offset_stamp_=ros::Time::now().toSec();}});
  points_pub_=nh_.advertise<sensor_msgs::PointCloud2>("/tunnel/filtered_points",1);compute_pub_=nh_.advertise<std_msgs::Float64>("/tunnel/preprocess_compute_ms",1);queue_pub_=nh_.advertise<std_msgs::Float64>("/tunnel/preprocess_queue_ms",1);age_pub_=nh_.advertise<std_msgs::Float64>("/tunnel/preprocess_output_age_ms",1);state_pub_=nh_.advertise<std_msgs::String>("/tunnel/preprocess_state",1);
  pose_sub_=nh_.subscribe("/odom",100,&Prefilter::pose,this);route_sub_=nh_.subscribe("/final_waypoint",1,&Prefilter::route,this);
  section_sub_=nh_.subscribe<mission_supervisor::SectionState>("/mission/active_section",1,[this](const mission_supervisor::SectionState::ConstPtr&m){active_=m->index_valid&&region_.prepares(m->csv,m->index);section_stamp_=ros::Time::now().toSec();});
  std::string input;nh_.param<std::string>("/obstacle_perception_node/lidar_topic",input,"/nonground");cloud_sub_=nh_.subscribe(input,1,&Prefilter::cloud,this,ros::TransportHints().tcpNoDelay());
  config_timer_=nh_.createTimer(ros::Duration(1.),&Prefilter::reloadConfig,this);
  worker_=std::thread([this](){while(!quit_){sensor_msgs::PointCloud2ConstPtr m;double wait=0;{std::unique_lock<std::mutex>lock(input_mutex_);wake_.wait(lock,[&](){return quit_||pending_;});if(quit_)break;m=pending_;pending_.reset();wait=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-queued_).count();}try{process(m,wait);}catch(const std::exception&e){diagnostic(e.what(),m->header.stamp.toSec(),wait,0,(ros::Time::now().toSec()-m->header.stamp.toSec())*1000);ROS_ERROR_THROTTLE(2,"M8 prefilter: %s",e.what());}}});
  ROS_INFO("M8 C++ shared prefilter: detector/map/far independent; input=%s",input.c_str());
 }
 ~Prefilter(){quit_=true;wake_.notify_all();if(worker_.joinable())worker_.join();}
};
int main(int argc,char**argv){ros::init(argc,argv,"tunnel_prefilter");try{Prefilter node;ros::AsyncSpinner spinner(2);spinner.start();ros::waitForShutdown();}catch(const std::exception&e){ROS_FATAL("M8 prefilter startup: %s",e.what());return 1;}return 0;}
