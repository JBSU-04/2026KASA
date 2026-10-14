#include "new_lane/tunnel_map_kernel.hpp"
#include "occupancy_avoidance/tunnel_region.hpp"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <mission_supervisor/SectionState.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <json/json.h>
#include <atomic>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <mutex>
#include <thread>
using namespace tunnel_map;
class Filter {
  ros::Subscriber preslow_sub_;ros::Publisher preslow_pub_,preslow_ms_;sensor_msgs::PointCloud2ConstPtr pending_preslow_;Confirmation preslow_confirmation_;bool repaired_preslow_=true;
  // 2026-10-13: 본 처리의 배경 조회 보정(X-ICP 정합 + 벽 정렬)을 선서행 비교에도 같은 스캔 기준으로 쓴다.
  double lookup_dx_=0,lookup_dy_=0,lookup_stamp_=-1e9;
  ros::NodeHandle nh_,pnh_{"~"};ros::Subscriber odom_,cloud_,nonground_,section_,registration_,near_filtered_,far_filtered_;ros::Publisher classified_,foreground_,background_,state_,map_pub_,filtered_,filter_state_;ros::Timer map_timer_;
  occupancy_avoidance::TunnelRegion region_;
  Map map_;bool map_valid_=false,enabled_=true;Confirmation confirmation_;V sensor_{1.1,0,1.33};double sensor_height_=1.765,max_age_=.35,min_height_=.08;
  bool common_wall_alignment_=true,detector_mask_=true,unified_input_=true,legacy_raw_detection_=false;WallAlignment wall_alignment_;
  bool filtered_pipeline_=false,join_only_=false;ros::Publisher mission_points_;
  // 2026-10-12 원거리 필터 점(/tunnel/filtered_far, x 12~40 m)도 같은 지도로 분류한다. 선서행(25 m) 전용 출력이고
  //   근거리 검출·엔진 입력(/tunnel/map_classified)에는 섞지 않는다(원거리 노면 오검출이 정지로 번지지 않게).
  bool far_classification_=true;ros::Subscriber far_cloud_;ros::Publisher classified_far_;Confirmation far_confirmation_;sensor_msgs::PointCloud2ConstPtr pending_far_;
  std::map<double,sensor_msgs::PointCloud2ConstPtr> near_cache_,far_cache_;double combined_stamp_=-1;
  ros::Publisher total_ms_,queue_ms_;double raw_compute_ms_=0,raw_compute_stamp_=-1;
  std::string heading_mode_;std::string map_path_;std::atomic<bool>active_{false},quit_{false};std::atomic<double>section_time_{-1};std::thread worker_;
  std::mutex pose_mutex_,queue_mutex_;std::condition_variable cv_;sensor_msgs::PointCloud2ConstPtr pending_,pending_ng_;std::deque<Pose>lookup_cache_;std::deque<std::pair<double,std::vector<Row>>> residual_cache_;std::deque<Pose>poses_;double registration_stamp_=-1;Eigen::Vector2d delta_=Eigen::Vector2d::Zero();
  bool poseAt(double stamp,Pose&out){std::lock_guard<std::mutex>lock(pose_mutex_);if(poses_.empty())return false;
    auto i=std::lower_bound(poses_.begin(),poses_.end(),stamp,[](const Pose&p,double s){return p.stamp<s;});
    if(i!=poses_.end()&&std::abs(i->stamp-stamp)<1e-6){out=*i;return true;}if(i==poses_.begin()||i==poses_.end()){auto nearest=i==poses_.end()?poses_.back():*i;return extrapolatePose(nearest,stamp,out);}
    auto a=*(i-1),b=*i;if(b.stamp-a.stamp>.16||std::min(stamp-a.stamp,b.stamp-stamp)>.08)return false;double u=(stamp-a.stamp)/(b.stamp-a.stamp);double dyaw=std::atan2(std::sin(b.yaw-a.yaw),std::cos(b.yaw-a.yaw));
    out={stamp,a.x+u*(b.x-a.x),a.y+u*(b.y-a.y),a.z+u*(b.z-a.z),a.yaw+u*dyaw};return true;}
  void odom(const nav_msgs::Odometry::ConstPtr&m){const auto&p=m->pose.pose.position;const auto&q=m->pose.pose.orientation;Pose v{m->header.stamp.toSec(),p.x,p.y,p.z,std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))};v.speed=m->twist.twist.linear.x;v.wz=m->twist.twist.angular.z;v.twist_valid=std::isfinite(v.speed+v.wz);std::lock_guard<std::mutex>lock(pose_mutex_);if(!poses_.empty()&&v.stamp<poses_.back().stamp){poses_.clear();registration_stamp_=-1;}poses_.push_back(v);while(poses_.size()>500)poses_.pop_front();}
  void registration(const geometry_msgs::PoseStamped::ConstPtr&m){if(heading_mode_!="existing")return;double stamp=m->header.stamp.toSec(),age=ros::Time::now().toSec()-stamp;if(age<-.05||age>.3)return;Pose p;if(!poseAt(stamp,p))return;Eigen::Vector2d d(m->pose.position.x-p.x,m->pose.position.y-p.y);if(!d.allFinite()||d.norm()>.6)return;std::lock_guard<std::mutex>lock(pose_mutex_);registration_stamp_=stamp;delta_=d;}
  void cloud(const sensor_msgs::PointCloud2::ConstPtr&m){if(!enabled_||!active_||ros::Time::now().toSec()-section_time_>1)return;{std::lock_guard<std::mutex>lock(queue_mutex_);pending_=m;}cv_.notify_one();}
  void filteredInput(const sensor_msgs::PointCloud2ConstPtr&m,bool near){
    if(!filtered_pipeline_||!active_||m->header.frame_id!="velodyne_ground")return;
    const double stamp=m->header.stamp.toSec();if(ros::Time::now().toSec()-stamp>max_age_)return;
    std::lock_guard<std::mutex>lock(queue_mutex_);if(combined_stamp_>ros::Time::now().toSec()+.05){combined_stamp_=-1;near_cache_.clear();far_cache_.clear();pending_.reset();}auto& cache=near?near_cache_:far_cache_;cache[stamp]=m;
    while(cache.size()>5)cache.erase(cache.begin());
    // M2 contract: far temporal proposals never gate fresh near detection.
    // Far objects independently reach /obstacle_info through the existing merger.
    // Only include same-scan far supports if they have already arrived; a late
    // far result cannot overwrite a newer scene or count that scan twice.
    if(!near||stamp<=combined_stamp_)return;
    auto a=m;
    sensor_msgs::PointCloud2ConstPtr b;
    auto far_it=far_cache_.find(stamp);
    if(far_it!=far_cache_.end())b=far_it->second;
    if(a->point_step!=20||(b&&(a->header.frame_id!=b->header.frame_id||b->point_step!=20)))return;
    sensor_msgs::PointCloud2Ptr combined(new sensor_msgs::PointCloud2(*a));combined->height=1;combined->width=0;combined->data.clear();
    std::set<std::array<float,3>>seen;
    for(const auto& source:{a,b})if(source)for(size_t row=0;row<source->height;++row)for(size_t col=0;col<source->width;++col){
      const size_t off=row*source->row_step+col*20;if(off+20>source->data.size())return;
      std::array<float,3>key;std::memcpy(key.data(),source->data.data()+off,12);
      if(!std::isfinite(key[0]+key[1]+key[2])||!seen.insert(key).second)continue;
      combined->data.insert(combined->data.end(),source->data.begin()+off,source->data.begin()+off+20);++combined->width;
    }
    combined->row_step=20*combined->width;combined_stamp_=stamp;mission_points_.publish(combined);if(!join_only_)pending_=combined;
    near_cache_.erase(near_cache_.begin(),near_cache_.upper_bound(stamp));far_cache_.erase(far_cache_.begin(),far_cache_.upper_bound(stamp));cv_.notify_one();
  }
  void nonground(const sensor_msgs::PointCloud2::ConstPtr&m){if(!detector_mask_||!active_||ros::Time::now().toSec()-section_time_>1)return;{std::lock_guard<std::mutex>lock(queue_mutex_);pending_ng_=m;}cv_.notify_one();}
  void filterNonground(const sensor_msgs::PointCloud2ConstPtr&m){
    const auto begin=std::chrono::steady_clock::now();double stamp=m->header.stamp.toSec();
    Pose lookup;bool found=false;for(const auto&p:lookup_cache_)if(std::abs(p.stamp-stamp)<1e-6){lookup=p;found=true;break;}
    double age=ros::Time::now().toSec()-stamp;bool valid=found&&map_valid_&&age>=-.05&&age<=max_age_&&m->header.frame_id=="velodyne_ground";
    sensor_msgs::PointCloud2 out=*m;int offsets[3]={-1,-1,-1};for(const auto&f:m->fields)for(int i=0;i<3;++i)if(f.name==std::string(1,"xyz"[i])&&f.datatype==sensor_msgs::PointField::FLOAT32)offsets[i]=f.offset;
    valid=valid&&!m->is_bigendian&&offsets[0]>=0&&offsets[1]>=0&&offsets[2]>=0;
    size_t removed=0;
    if(valid){out.height=1;out.width=0;out.data.clear();out.data.reserve(m->data.size());double c=std::cos(lookup.yaw),sn=std::sin(lookup.yaw);
      for(size_t row=0;row<m->height;++row)for(size_t col=0;col<m->width;++col){size_t base=row*m->row_step+col*m->point_step;if(base+m->point_step>m->data.size())throw std::runtime_error("truncated nonground");V q;
        for(int i=0;i<3;++i){if(offsets[i]+4>int(m->point_step))throw std::runtime_error("bad nonground fields");float a;std::memcpy(&a,m->data.data()+base+offsets[i],4);q[i]=a;}
        V b=q+sensor_,w(lookup.x+c*b.x()-sn*b.y(),lookup.y+sn*b.x()+c*b.y(),lookup.z+b.z());
        if(q.allFinite()&&q.x()>=-15&&q.x()<=40&&std::abs(q.y())<=12&&q.z()+sensor_height_<=3.2){
          double floor=map_.ground(w),h=w.z()-floor;
          double threshold=std::isfinite(floor)&&h<.25?map_.floor_distance:map_.distance;
          if(map_.matches(w,threshold)){++removed;continue;}}

        out.data.insert(out.data.end(),m->data.begin()+base,m->data.begin()+base+m->point_step);++out.width;}
      out.row_step=out.width*out.point_step;}
    size_t residual_count=0;
    if(valid&&unified_input_){
      std::vector<Row> observed;int ho=-1;for(const auto&f:out.fields)if(f.name=="ground_relative"&&f.datatype==sensor_msgs::PointField::FLOAT32)ho=f.offset;
      observed.reserve(out.width*out.height);
      for(size_t row=0;row<out.height;++row)for(size_t col=0;col<out.width;++col){size_t base=row*out.row_step+col*out.point_step;Row r;
        for(int i=0;i<3;++i){float v;std::memcpy(&v,out.data.data()+base+offsets[i],4);r.sensor[i]=v;}
        if(ho>=0&&ho+4<=int(out.point_step)){float v;std::memcpy(&v,out.data.data()+base+ho,4);r.height=v;r.valid=std::isfinite(v);}else r.height=r.sensor.z()+sensor_height_;
        observed.push_back(r);
      }
      const std::vector<Row>*raw=nullptr;for(const auto&entry:residual_cache_)if(std::abs(entry.first-stamp)<1e-6){raw=&entry.second;break;}
      if(raw){auto rows=mergeDetectionRows(observed,*raw,legacy_raw_detection_);residual_count=rows.size()>observed.size()?rows.size()-observed.size():0;publishCloud(filtered_,m->header,rows,0);}
      else {if(active_)filtered_.publish(out);} // Missing data cannot invent residual evidence.
    }else if(active_)filtered_.publish(out);
    Json::Value state;state["stamp"]=stamp;state["map_valid"]=valid;state["removed"]=Json::UInt64(removed);state["bundled_residual_points"]=Json::UInt64(residual_count);state["unified_input"]=unified_input_;state["reason"]=valid?"verified_3d_background_only":"unknown_background_passthrough";state["compute_ms"]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();state["cache_hits"]=Json::UInt64(map_.cache_hits);state["cache_queries"]=Json::UInt64(map_.cache_queries);std_msgs::String st;st.data=Json::FastWriter().write(state);filter_state_.publish(st);if(std::abs(raw_compute_stamp_-stamp)<1e-6){std_msgs::Float64 value;value.data=raw_compute_ms_+state["compute_ms"].asDouble();total_ms_.publish(value);}
  }
  std::vector<V> xyz(const sensor_msgs::PointCloud2&m){int off[3]={-1,-1,-1};for(const auto&f:m.fields)for(int i=0;i<3;++i)if(f.name==std::string(1,"xyz"[i])&&f.datatype==sensor_msgs::PointField::FLOAT32)off[i]=f.offset;
    if(off[0]<0||off[1]<0||off[2]<0||m.is_bigendian)throw std::runtime_error("unsupported XYZ/endian");std::vector<V>p;p.reserve(m.width*m.height);
    for(size_t row=0;row<m.height;++row)for(size_t col=0;col<m.width;++col){size_t base=row*m.row_step+col*m.point_step;V q;for(int i=0;i<3;++i){if(base+off[i]+4>m.data.size())throw std::runtime_error("truncated cloud");float a;std::memcpy(&a,m.data.data()+base+off[i],4);q[i]=a;}if(q.allFinite())p.push_back(q);}return p;}
  Pose lookupPose(const std::vector<V>&p,const Pose&pose,double stamp,double&age){Pose out=pose;Eigen::Vector2d d;double time;{std::lock_guard<std::mutex>lock(pose_mutex_);d=delta_;time=registration_stamp_;}age=time<0?INFINITY:stamp-time;if(!map_valid_||age<-.05||age>.3||d.norm()>.6)return out;
    std::vector<V>wall;for(const auto&q:p)if(q.z()>=0&&q.z()<=1.5&&std::abs(q.y())>=1.5&&std::abs(q.y())<=12&&q.x()>=1&&q.x()<=35)wall.push_back(q);if(wall.size()<100)return out;
    double c=std::cos(pose.yaw),s=std::sin(pose.yaw);std::vector<double>a,b;int matched0=0,matched1=0;size_t stride=std::max(size_t(1),wall.size()/500);
    for(size_t i=0;i<wall.size();i+=stride){V q=wall[i]+sensor_,w(pose.x+c*q.x()-s*q.y(),pose.y+s*q.x()+c*q.y(),pose.z+q.z());double d0=map_.nearest(w);w.head<2>()+=d;double d1=map_.nearest(w);a.push_back(d0);b.push_back(d1);matched0+=d0<=map_.distance;matched1+=d1<=map_.distance;}
    if(matched1>matched0+.02*a.size()&&median(b)<median(a)){out.x+=d.x();out.y+=d.y();}return out;}
  void publishCloud(ros::Publisher&pub,const std_msgs::Header&header,const std::vector<Row>&rows,int mask){sensor_msgs::PointCloud2 out;out.header=header;out.height=1;out.point_step=24;out.is_dense=false;const char*names[]={"x","y","z","map_ground","map_label","ground_relative"};for(int i=0;i<6;++i){sensor_msgs::PointField f;f.name=names[i];f.offset=4*i;f.datatype=f.FLOAT32;f.count=1;out.fields.push_back(f);}out.data.reserve(rows.size()*24);
    for(const auto&r:rows){bool fg=!r.bg&&!r.inside&&((r.valid&&r.height>=.08)||!r.valid);if(mask==1&&!fg)continue;if(mask==2&&!r.bg)continue;float a[6]={float(r.sensor.x()),float(r.sensor.y()),float(r.sensor.z()),float(r.ground),float(r.label),float(r.valid?r.height:r.sensor.z()+sensor_height_)};const auto*b=reinterpret_cast<const uint8_t*>(a);out.data.insert(out.data.end(),b,b+24);++out.width;}out.row_step=out.width*out.point_step;pub.publish(out);}
  void process(const sensor_msgs::PointCloud2ConstPtr&m){auto begin=std::chrono::steady_clock::now();double stamp=m->header.stamp.toSec(),age=ros::Time::now().toSec()-stamp;if(age<-.05||age>max_age_||m->header.frame_id!="velodyne_ground"||!active_)return;map_.beginScan();std_msgs::Float64 queued;queued.data=age*1000.;queue_ms_.publish(queued);Pose pose;if(!poseAt(stamp,pose))return;auto points=xyz(*m);double registration_age;Pose lookup=lookupPose(points,pose,stamp,registration_age);
    wall_alignment_=WallAlignment();
    if(common_wall_alignment_&&map_valid_){wall_alignment_=wallAlignment(points,lookup,map_,sensor_);
      if(wall_alignment_.valid&&std::abs(wall_alignment_.lateral)>.01){
        Pose trial=lookup;trial.x-=std::sin(lookup.yaw)*wall_alignment_.lateral;trial.y+=std::cos(lookup.yaw)*wall_alignment_.lateral;
        // Require improvement against existing 3D matching distance, never widen it.
        int before=0,after=0,counts[2]={0,0};double c=std::cos(lookup.yaw),sn=std::sin(lookup.yaw);
        const size_t stride=std::max(size_t(1),points.size()/1200);
        for(size_t i=0;i<points.size();i+=stride){const auto&q=points[i];if(q.x()<1||q.x()>35||std::abs(q.y())<2||q.z()<-.5||q.z()>1.5)continue;
          V b=q+sensor_,w(lookup.x+c*b.x()-sn*b.y(),lookup.y+sn*b.x()+c*b.y(),lookup.z+b.z());
          before+=map_.matches(w,map_.distance);w.x()+=trial.x-lookup.x;w.y()+=trial.y-lookup.y;after+=map_.matches(w,map_.distance);++counts[q.y()>0?0:1];}
        if(after>before+.10*(counts[0]+counts[1]))lookup=trial;else wall_alignment_.valid=false;
      }}lookup_dx_=lookup.x-pose.x;lookup_dy_=lookup.y-pose.y;lookup_stamp_=stamp;std::vector<Row> rows;
    if(filtered_pipeline_){
      const double c=std::cos(lookup.yaw),sn=std::sin(lookup.yaw);rows.reserve(points.size());
      for(size_t i=0;i<points.size();++i){Row v;v.sensor=points[i];float h;std::memcpy(&h,m->data.data()+i*m->point_step+12,4);
        v.height=h;v.valid=std::isfinite(h);v.ground=v.sensor.z()-h;V b=v.sensor+sensor_;
        v.world=V(lookup.x+c*b.x()-sn*b.y(),lookup.y+sn*b.x()+c*b.y(),lookup.z+b.z());
        v.bg=map_valid_&&(map_.wall_surface_background?map_.backgroundMatches(v.world,h,v.valid):map_.matches(v.world,map_.distance));
        v.candidate=map_valid_&&v.valid&&!v.bg&&h>=min_height_&&h<=2.8;
        v.label=v.bg?1:(v.candidate?2:0);rows.push_back(v);
      }
    }else rows=classify(points,lookup,map_valid_?&map_:nullptr,sensor_,sensor_height_,min_height_);
    // Registration changes background lookup only, not object motion coordinates.
    for(auto&r:rows){r.world.x()+=pose.x-lookup.x;r.world.y()+=pose.y-lookup.y;}confirmation_.update(rows,stamp);
    residual_cache_.emplace_back(stamp,rows);while(residual_cache_.size()>10)residual_cache_.pop_front();
    lookup_cache_.push_back(lookup);while(lookup_cache_.size()>10)lookup_cache_.pop_front();
    if(ros::Time::now().toSec()-stamp>max_age_||!active_)return;publishCloud(classified_,m->header,rows,0);publishCloud(foreground_,m->header,rows,1);if(background_.getNumSubscribers())publishCloud(background_,m->header,rows,2);
    Json::Value st;st["backend"]="cpp";st["input_contract"]=filtered_pipeline_?"m2_filtered_near_far":"legacy_raw";st["map_version"]=map_path_;st["stamp"]=stamp;st["map_valid"]=map_valid_;int bg=0,cand=0,confirmed=0,unknown=0,cgu=0;for(const auto&r:rows){bg+=r.bg;cand+=r.candidate;confirmed+=r.label>=3;unknown+=!r.bg&&!r.valid;cgu+=r.label==4;}st["background"]=bg;st["candidate"]=cand;st["confirmed"]=confirmed;st["confirmed_ground_unknown"]=cgu;st["unknown"]=unknown;st["registration_age_sec"]=std::isfinite(registration_age)?Json::Value(registration_age):Json::Value();st["wall_alignment_valid"]=wall_alignment_.valid;st["wall_alignment_lateral_m"]=wall_alignment_.lateral;st["wall_alignment_left_support"]=wall_alignment_.left;st["wall_alignment_right_support"]=wall_alignment_.right;st["lookup_delta_xy"].append(lookup.x-pose.x);st["lookup_delta_xy"].append(lookup.y-pose.y);st["compute_ms"]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();raw_compute_ms_=st["compute_ms"].asDouble();raw_compute_stamp_=stamp;std_msgs::String msg;msg.data=Json::FastWriter().write(st);state_.publish(msg);}
  void processPreslow(const sensor_msgs::PointCloud2ConstPtr&m){
    auto begin=std::chrono::steady_clock::now();double stamp=m->header.stamp.toSec();Pose pose;
    const double age=ros::Time::now().toSec()-stamp;
    if(!map_valid_||m->header.frame_id!="velodyne_ground"||!poseAt(stamp,pose)||!active_||age<-.05||age>max_age_)return;
    int ho=-1;for(const auto&f:m->fields)if(f.name=="ground_relative"&&f.datatype==sensor_msgs::PointField::FLOAT32)ho=f.offset;
    if(ho<0||ho+4>int(m->point_step))return;
    auto points=xyz(*m);if(points.size()!=size_t(m->width)*m->height)return;
    // Background lookup only (same registration/wall alignment as the main scan); without it an empty wall
    // under DR lateral drift mismatches the map and becomes a preslow candidate.
    Pose lookup=pose;if(std::abs(stamp-lookup_stamp_)<=.2){lookup.x+=lookup_dx_;lookup.y+=lookup_dy_;}
    map_.beginScan();double c=std::cos(lookup.yaw),sn=std::sin(lookup.yaw);std::vector<Row>rows;rows.reserve(points.size());
    for(size_t i=0;i<points.size();++i){Row v;v.sensor=points[i];float h;size_t row=i/m->width,col=i%m->width;std::memcpy(&h,m->data.data()+row*m->row_step+col*m->point_step+ho,4);
      v.height=h;v.valid=std::isfinite(h);v.ground=v.sensor.z()-h;V b=v.sensor+sensor_;
      v.world=V(lookup.x+c*b.x()-sn*b.y(),lookup.y+sn*b.x()+c*b.y(),lookup.z+b.z());
      // Preserve the existing floor-specific tolerance; a 9cm object
      // must not match road merely because wall tolerance is 12cm.
      v.bg=map_.backgroundMatches(v.world,h,v.valid);v.candidate=v.valid&&!v.bg&&h>=min_height_&&h<=2.8;
      v.label=v.bg?1:(v.candidate?2:0);rows.push_back(v);}
    for(auto&r:rows){r.world.x()+=pose.x-lookup.x;r.world.y()+=pose.y-lookup.y;}  // 확인 이력은 본 처리처럼 자세 좌표
    preslow_confirmation_.update(rows,stamp);
    if(ros::Time::now().toSec()-stamp>max_age_||!active_)return;
    publishCloud(preslow_pub_,m->header,rows,0);
    std_msgs::Float64 ms;ms.data=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();preslow_ms_.publish(ms);
  }
  void farCloud(const sensor_msgs::PointCloud2::ConstPtr&m){if(!enabled_||!active_||ros::Time::now().toSec()-section_time_>1)return;{std::lock_guard<std::mutex>lock(queue_mutex_);pending_far_=m;}cv_.notify_one();}
  void processFar(const sensor_msgs::PointCloud2ConstPtr&m){double stamp=m->header.stamp.toSec(),age=ros::Time::now().toSec()-stamp;
    if(age<-.05||age>max_age_||m->header.frame_id!="velodyne_ground"||!active_||!map_valid_||m->is_bigendian)return;
    Pose lookup;bool found=false;for(const auto&p:lookup_cache_)if(std::abs(p.stamp-stamp)<1e-6){lookup=p;found=true;break;}
    Pose pose;if(!poseAt(stamp,pose))return;if(!found)lookup=pose;
    int ho=-1;for(const auto&f:m->fields)if(f.name=="ground_relative"&&f.datatype==sensor_msgs::PointField::FLOAT32)ho=f.offset;if(ho<0||ho+4>int(m->point_step))return;
    auto points=xyz(*m);if(points.size()!=size_t(m->width)*m->height)return; // xyz drops non-finite rows; keep row order exact
    map_.beginScan();const double c=std::cos(lookup.yaw),sn=std::sin(lookup.yaw);std::vector<Row>rows;rows.reserve(points.size());
    for(size_t i=0;i<points.size();++i){size_t row=i/m->width,col=i%m->width;Row v;v.sensor=points[i];float h;std::memcpy(&h,m->data.data()+row*m->row_step+col*m->point_step+ho,4);
      v.height=h;v.valid=std::isfinite(h);v.ground=v.sensor.z()-h;V b=v.sensor+sensor_;
      v.world=V(lookup.x+c*b.x()-sn*b.y(),lookup.y+sn*b.x()+c*b.y(),lookup.z+b.z());
      v.bg=map_.backgroundMatches(v.world,h,v.valid);v.candidate=v.valid&&!v.bg&&h>=min_height_&&h<=2.8;v.label=v.bg?1:(v.candidate?2:0);rows.push_back(v);}
    for(auto&r:rows){r.world.x()+=pose.x-lookup.x;r.world.y()+=pose.y-lookup.y;}far_confirmation_.update(rows,stamp);
    if(ros::Time::now().toSec()-stamp>max_age_)return;publishCloud(classified_far_,m->header,rows,0);}
  void mapView(const ros::TimerEvent&){if(!active_||!map_valid_||!map_pub_.getNumSubscribers())return;Pose pose;{std::lock_guard<std::mutex>lock(pose_mutex_);if(poses_.empty())return;pose=poses_.back();}visualization_msgs::Marker m;m.header.frame_id="map";m.header.stamp=ros::Time::now();m.ns="active_gps_3d_map";m.id=0;m.type=m.POINTS;m.action=m.ADD;m.pose.orientation.w=1;m.scale.x=m.scale.y=.12;m.color.r=.15;m.color.g=.45;m.color.b=1;m.color.a=.45;m.lifetime=ros::Duration(2.);std::set<std::array<int,3>>seen;
    for(const auto&p:*map_.points){double x=p.x+map_.origin.x(),y=p.y+map_.origin.y();if(std::abs(x-pose.x)>45||std::abs(y-pose.y)>45)continue;std::array<int,3>key{int(std::floor(p.x/.3)),int(std::floor(p.y/.3)),int(std::floor(p.z/.3))};if(!seen.insert(key).second)continue;geometry_msgs::Point q;q.x=x;q.y=y;q.z=p.z+map_.origin.z();m.points.push_back(q);if(m.points.size()>=30000)break;}visualization_msgs::MarkerArray a;a.markers.push_back(m);map_pub_.publish(a);}
 public:
  Filter(){pnh_.param("join_only",join_only_,false);nh_.param("/tunnel_recovery/filtered_pipeline_enabled",filtered_pipeline_,false);nh_.param("/tunnel_recovery/wall_surface_background_enabled",map_.wall_surface_background,true);nh_.param("/tunnel_recovery/raw_detection_residual_enabled",legacy_raw_detection_,false);nh_.param<std::string>("/tunnel_recovery/heading_mode",heading_mode_,"existing");region_=occupancy_avoidance::TunnelRegion::load(nh_);bool master=true;nh_.param("/tunnel_recovery/enabled",master,true);pnh_.param("enabled",enabled_,true);enabled_&=master;pnh_.param<std::string>("map_binary",map_path_,"");pnh_.param("background_distance_m",map_.distance,.12);pnh_.param("floor_distance_m",map_.floor_distance,.055);pnh_.param("min_height_m",min_height_,.08);pnh_.param("max_scan_age_sec",max_age_,.35);pnh_.param("min_points",confirmation_.min_points,3);pnh_.param("confirm_scans",confirmation_.scans,2);pnh_.param("observation_memory_sec",confirmation_.memory,.35);pnh_.param("common_wall_alignment",common_wall_alignment_,true);nh_.param("/tunnel_recovery/unified_detection_input_enabled",unified_input_,true);nh_.param("/tunnel_recovery/verified_background_filter_enabled",detector_mask_,true);pnh_.param("allow_unknown_ground_confirmation",confirmation_.allow_unknown_ground,false);nh_.param("/lidar_extrinsic/z_from_ground",sensor_height_,1.765);pnh_.param("sensor_x",sensor_.x(),1.1);nh_.param("/lidar_extrinsic/y",sensor_.y(),0.);pnh_.param("sensor_z",sensor_.z(),1.33);
    pnh_.param("voxel_search_enabled",map_.voxel_search,true);
    total_ms_=nh_.advertise<std_msgs::Float64>("/tunnel/map_total_processing_ms",1);queue_ms_=nh_.advertise<std_msgs::Float64>("/tunnel/map_input_age_ms",1);
    try{if(enabled_&&!join_only_){map_.load(map_path_);map_valid_=true;}}catch(const std::exception&e){ROS_ERROR("[tunnel_map_filter] %s; UNKNOWN retained",e.what());}
    classified_=nh_.advertise<sensor_msgs::PointCloud2>("/tunnel/map_classified",1);foreground_=nh_.advertise<sensor_msgs::PointCloud2>("/tunnel/map_foreground",1);background_=nh_.advertise<sensor_msgs::PointCloud2>("/tunnel/map_background",1);state_=nh_.advertise<std_msgs::String>("/tunnel/map_state",1);map_pub_=nh_.advertise<visualization_msgs::MarkerArray>("/tunnel/static_map",1);
    std::string nonground_input; pnh_.param<std::string>("nonground_input",nonground_input,"/groundgrid/nonground_exact");
    filtered_=nh_.advertise<sensor_msgs::PointCloud2>("/tunnel/nonground_filtered",1);filter_state_=nh_.advertise<std_msgs::String>("/tunnel/nonground_filter_state",1);if(!filtered_pipeline_&&!join_only_)nonground_=nh_.subscribe(nonground_input,1,&Filter::nonground,this);
    if(join_only_)mission_points_=nh_.advertise<sensor_msgs::PointCloud2>("/tunnel/filtered_points",1);
    if(join_only_&&filtered_pipeline_){near_filtered_=nh_.subscribe<sensor_msgs::PointCloud2>("/tunnel/filtered_near",1,[this](const sensor_msgs::PointCloud2ConstPtr&m){filteredInput(m,true);});far_filtered_=nh_.subscribe<sensor_msgs::PointCloud2>("/tunnel/filtered_far",1,[this](const sensor_msgs::PointCloud2ConstPtr&m){filteredInput(m,false);});}
    odom_=nh_.subscribe("/odom",100,&Filter::odom,this);if(!join_only_)cloud_=nh_.subscribe(filtered_pipeline_?"/tunnel/filtered_points":"/perception/points_gravity",1,&Filter::cloud,this);registration_=nh_.subscribe("/tunnel/xicp/yaw_measurement",1,&Filter::registration,this);section_=nh_.subscribe<mission_supervisor::SectionState>("/mission/active_section",1,[this](const mission_supervisor::SectionState::ConstPtr&m){active_=m->index_valid&&region_.serves(m->csv,m->index);section_time_=ros::Time::now().toSec();});map_timer_=nh_.createTimer(ros::Duration(1.),&Filter::mapView,this);
    pnh_.param("far_classification",far_classification_,true);far_confirmation_=confirmation_;
    if(filtered_pipeline_&&!join_only_&&far_classification_){classified_far_=nh_.advertise<sensor_msgs::PointCloud2>("/tunnel/map_classified_far",1);bool verified=false;nh_.param("/tunnel_recovery/verified_wall_surface_only_enabled",verified,false);far_cloud_=nh_.subscribe(verified?"/tunnel/filtered_background_input":"/tunnel/filtered_far",1,&Filter::farCloud,this);}
    nh_.param("/tunnel_recovery/repair_preslow_policy_enabled",repaired_preslow_,true);
    if(!join_only_&&repaired_preslow_){preslow_confirmation_=confirmation_;preslow_confirmation_.allow_unknown_ground=false;
      preslow_pub_=nh_.advertise<sensor_msgs::PointCloud2>("/tunnel/preslow_classified",1);
      preslow_ms_=nh_.advertise<std_msgs::Float64>("/tunnel/preslow_map_processing_ms",1);
      preslow_sub_=nh_.subscribe<sensor_msgs::PointCloud2>("/tunnel/preslow_input",1,[this](const sensor_msgs::PointCloud2ConstPtr&m){if(!active_)return;{std::lock_guard<std::mutex>lock(queue_mutex_);pending_preslow_=m;}cv_.notify_one();});}
    worker_=std::thread([this](){while(!quit_){sensor_msgs::PointCloud2ConstPtr msg,ng,far,pre;{std::unique_lock<std::mutex>lock(queue_mutex_);cv_.wait_for(lock,std::chrono::milliseconds(10),[this](){return pending_||pending_far_||pending_preslow_||quit_||(pending_ng_&&(ros::Time::now().toSec()-pending_ng_->header.stamp.toSec()>max_age_||std::any_of(lookup_cache_.begin(),lookup_cache_.end(),[this](const Pose&p){return std::abs(p.stamp-pending_ng_->header.stamp.toSec())<1e-6;})));});if(quit_)break;msg=pending_;pending_.reset();far=pending_far_;pending_far_.reset();pre=pending_preslow_;pending_preslow_.reset();}
      if(msg)try{process(msg);}catch(const std::exception&e){ROS_WARN_THROTTLE(2,"[tunnel_map_filter] %s",e.what());}
      if(pre)try{processPreslow(pre);}catch(const std::exception&e){ROS_WARN_THROTTLE(2,"[M8 preslow] %s",e.what());}
      if(far)try{processFar(far);}catch(const std::exception&e){ROS_WARN_THROTTLE(2,"[tunnel_map_filter far] %s",e.what());}
      {std::lock_guard<std::mutex>lock(queue_mutex_);if(pending_ng_){double stamp=pending_ng_->header.stamp.toSec();bool ready=std::any_of(lookup_cache_.begin(),lookup_cache_.end(),[stamp](const Pose&p){return std::abs(p.stamp-stamp)<1e-6;});if(ready||ros::Time::now().toSec()-stamp>max_age_){ng=pending_ng_;pending_ng_.reset();}}}
      if(ng)try{filterNonground(ng);}catch(const std::exception&e){ROS_WARN_THROTTLE(2,"[tunnel_nonground_filter] %s",e.what());}
    }});}
  ~Filter(){quit_=true;cv_.notify_all();if(worker_.joinable())worker_.join();}
};
int main(int argc,char**argv){ros::init(argc,argv,"tunnel_map_filter");Filter f;ros::AsyncSpinner spinner(2);spinner.start();ros::waitForShutdown();return 0;}
