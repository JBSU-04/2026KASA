#pragma once
// ROS1 C++ port of the existing Autoware distance compare-map adapter.
// Provenance/license: new_lane/vendor/autoware_compare_map. Added local floor
// validation and scan-stamped evidence; no background learning at runtime.
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <map>
#include <set>
#include <stdexcept>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <cstring>
namespace tunnel_map {
using V=Eigen::Vector3d;using Key=std::pair<int,int>;
inline double median(std::vector<double> a){if(a.empty())return NAN;std::sort(a.begin(),a.end());return (a[(a.size()-1)/2]+a[a.size()/2])*.5;}
struct Pose{double stamp=0,x=0,y=0,z=0,yaw=0,speed=0,wz=0;bool twist_valid=false;};
inline bool extrapolatePose(const Pose&p,double stamp,Pose&out){
  double dt=stamp-p.stamp;if(std::abs(dt)>.08+1e-9||!p.twist_valid||!std::isfinite(p.speed+p.wz))return false;
  out=p;out.stamp=stamp;double yaw=p.yaw+p.wz*dt;
  if(std::abs(p.wz)>1e-6){out.x+=p.speed/p.wz*(std::sin(yaw)-std::sin(p.yaw));out.y+=p.speed/p.wz*(-std::cos(yaw)+std::cos(p.yaw));}
  else{out.x+=p.speed*dt*std::cos(p.yaw);out.y+=p.speed*dt*std::sin(p.yaw);}
  out.yaw=yaw;return true;
}
struct Row{V sensor,world;double height=NAN,ground=NAN;int label=0;bool valid=false,bg=false,inside=false,candidate=false;};
// Exact sensor-coordinate union. Raw candidates remain observations, not a
// forced static certificate. Unknown floor and verified background stay out.
inline std::vector<Row> mergeDetectionRows(const std::vector<Row>& nonground,const std::vector<Row>& raw,bool legacy_raw=false){
  std::vector<Row> out;out.reserve(nonground.size()+raw.size());
  std::map<std::array<double,3>,size_t> seen;
  auto add=[&](const Row&r,bool map_height){
    if(!r.sensor.allFinite())return;
    const std::array<double,3> key{{r.sensor.x(),r.sensor.y(),r.sensor.z()}};
    auto found=seen.find(key);
    if(found==seen.end()){seen[key]=out.size();out.push_back(r);}
    else if(map_height&&legacy_raw)out[found->second]=r;
  };
  for(const auto&r:nonground)add(r,false);
  // Map mismatch is not proof of being above ground. The shared GroundGrid
  // height and selection remain authoritative for the detector. Raw remains
  // available as map evidence, never reintroducing rejected terrain by default.
  if(legacy_raw)for(const auto&r:raw)if(r.valid&&!r.bg&&!r.inside&&r.candidate&&(r.label==2||r.label==3))add(r,true);
  return out;
}

class Map {
 public:
  // Autoware voxel-distance compare-map: occupied voxel first, exact radius
  // query otherwise. Sparse layout avoids dense leaf allocation for tunnels.
  struct K3 {int x,y,z;bool operator==(const K3&o)const{return x==o.x&&y==o.y&&z==o.z;}};
  struct KHash {size_t operator()(const K3&k)const{return size_t(uint32_t(k.x))*73856093u^size_t(uint32_t(k.y))*19349663u^size_t(uint32_t(k.z))*83492791u;}};
  struct Query {uint32_t x,y,z;uint64_t radius;bool operator==(const Query&o)const{return x==o.x&&y==o.y&&z==o.z&&radius==o.radius;}};
  struct QHash {size_t operator()(const Query&k)const{return size_t(k.x)*73856093u^size_t(k.y)*19349663u^size_t(k.z)*83492791u^size_t(k.radius);}};
  bool voxel_search=true;double voxel_leaf=.05;
  bool wall_surface_background=false;
  std::unordered_set<K3,KHash> map_voxels;
  mutable std::unordered_map<Query,bool,QHash> match_cache;
  mutable size_t cache_hits=0,cache_queries=0;
  mutable std::unordered_map<uint64_t,double> ground_cache;
  static K3 voxel(const pcl::PointXYZ&p,double leaf){return {int(std::floor(p.x/leaf)),int(std::floor(p.y/leaf)),int(std::floor(p.z/leaf))};}
  void indexMap(){map_voxels.clear();map_voxels.reserve(points->size());for(const auto&p:*points)map_voxels.insert(voxel(p,voxel_leaf));}
  void beginScan(){match_cache.clear();ground_cache.clear();cache_hits=cache_queries=0;}
  V origin=V::Zero();double distance=.12,floor_distance=.055;
  pcl::PointCloud<pcl::PointXYZ>::Ptr points{new pcl::PointCloud<pcl::PointXYZ>},floor{new pcl::PointCloud<pcl::PointXYZ>},floor_xy{new pcl::PointCloud<pcl::PointXYZ>};
  pcl::KdTreeFLANN<pcl::PointXYZ> tree,floor_tree;
  std::unordered_map<uint64_t,std::vector<int>> floor_bins;
  static uint64_t floorKey(int x,int y){return (uint64_t(uint32_t(x))<<32)|uint32_t(y);}
  void indexFloor(){
    floor_bins.clear();floor_bins.reserve(floor->size()/32);
    for(size_t i=0;i<floor->size();++i){const auto&p=(*floor)[i];floor_bins[floorKey(int(std::floor(p.x/.5)),int(std::floor(p.y/.5)))].push_back(int(i));}
  }
  void load(const std::string& path){
    std::ifstream f(path,std::ios::binary);char magic[8];uint64_t n=0,m=0;f.read(magic,8);
    if(!f||std::string(magic,7)!="TMAP3D1")throw std::runtime_error("invalid tunnel map binary");
    f.read(reinterpret_cast<char*>(origin.data()),24);f.read(reinterpret_cast<char*>(&n),8);f.read(reinterpret_cast<char*>(&m),8);
    if(n>10000000||m>10000000||!origin.allFinite())throw std::runtime_error("invalid tunnel map bounds");
    auto read=[&](auto cloud,uint64_t count){cloud->reserve(count);for(uint64_t i=0;i<count;++i){float q[3];f.read(reinterpret_cast<char*>(q),12);if(!f||!std::isfinite(q[0]+q[1]+q[2]))throw std::runtime_error("truncated/nonfinite tunnel map");cloud->push_back(pcl::PointXYZ(q[0],q[1],q[2]));}};
    read(points,n);read(floor,m);*floor_xy=*floor;for(auto&p:*floor_xy)p.z=0;
    if(points->empty()||floor->size()<3)throw std::runtime_error("empty map/floor");tree.setInputCloud(points);floor_tree.setInputCloud(floor_xy);indexFloor();indexMap();
  }
  double nearest(const V&w)const {std::vector<int> ids(1);std::vector<float>d(1);pcl::PointXYZ q((w-origin).x(),(w-origin).y(),(w-origin).z());if(tree.nearestKSearch(q,1,ids,d)!=1)return INFINITY;return std::sqrt(d[0]);}
  // Background classification needs only a bounded membership query. Exact
  // unbounded nearest searches are kept for registration diagnostics, not scans.
  bool matches(const V&w,double radius)const {
    thread_local std::vector<int> ids;thread_local std::vector<float> distances;
    pcl::PointXYZ q((w-origin).x(),(w-origin).y(),(w-origin).z());
    if(!std::isfinite(q.x+q.y+q.z)||!std::isfinite(radius)||radius<0)return false;
    Query key;std::memcpy(&key.x,&q.x,4);std::memcpy(&key.y,&q.y,4);std::memcpy(&key.z,&q.z,4);std::memcpy(&key.radius,&radius,8);
    ++cache_queries;auto old=match_cache.find(key);if(old!=match_cache.end()){++cache_hits;return old->second;}
    // A 5cm cell diagonal is <0.12m. Smaller floor thresholds use exact search.
    bool close=voxel_search&&radius>=std::sqrt(3.)*voxel_leaf+1e-6&&map_voxels.count(voxel(q,voxel_leaf));
    if(!close)close=tree.radiusSearch(q,radius,ids,distances,1)>0;
    if(match_cache.size()<250000)match_cache.emplace(key,close);
    return close;
  }
  bool backgroundMatches(const V&w,double height,bool height_valid)const {
    const double radius=height_valid&&height<.25?floor_distance:distance;
    if(matches(w,radius))return true;
    if(!wall_surface_background||!height_valid||height<.08||height>=.25)return false;
    const double floor_z=ground(w);if(!std::isfinite(floor_z))return false;
    // Never enlarge the road-removal radius: accept only verified elevated
    // map surfaces. Curved walls/columns need no plane fit or shape threshold.
    std::vector<int>ids;std::vector<float>d;
    pcl::PointXYZ q((w-origin).x(),(w-origin).y(),(w-origin).z());
    if(tree.radiusSearch(q,distance,ids,d)==0)return false;
    for(int id:ids)if((*points)[id].z+origin.z()-floor_z>=.08)return true;
    return false;
  }
  double ground(const V&w)const {
    // Exact nearest three within the existing 1.2 m support radius, using
    // immutable local XY buckets. No point thinning or floor averaging.
    if(floor_bins.empty())return NAN;
    const float x=w.x()-origin.x(),y=w.y()-origin.y();
    uint32_t xb,yb;std::memcpy(&xb,&x,4);std::memcpy(&yb,&y,4);uint64_t cache_key=(uint64_t(xb)<<32)|yb;
    auto cached=ground_cache.find(cache_key);if(cached!=ground_cache.end())return cached->second;
    int bx=int(std::floor(x/.5)),by=int(std::floor(y/.5));
    std::array<float,3> best{{INFINITY,INFINITY,INFINITY}};
    std::array<int,3> ids{{-1,-1,-1}};
    for(int ring=0;ring<=3;++ring){
      for(int dx=-ring;dx<=ring;++dx)for(int dy=-ring;dy<=ring;++dy){
        if(std::max(std::abs(dx),std::abs(dy))!=ring)continue;
        auto it=floor_bins.find(floorKey(bx+dx,by+dy));if(it==floor_bins.end())continue;
        for(int id:it->second){const auto&p=(*floor)[id];float d=(p.x-x)*(p.x-x)+(p.y-y)*(p.y-y);if(d>1.44f||d>=best[2])continue;
          int j=2;while(j>0&&d<best[j-1]){best[j]=best[j-1];ids[j]=ids[j-1];--j;}best[j]=d;ids[j]=id;}
      }
      double outside=std::min({double(x)-(bx-ring)*.5,(bx+ring+1)*.5-double(x),double(y)-(by-ring)*.5,(by+ring+1)*.5-double(y)});
      if(outside*outside>=std::min(double(best[2]),1.44))break;
    }
    if(ids[2]<0){ground_cache[cache_key]=NAN;return NAN;}
    std::array<double,3> z{{(*floor)[ids[0]].z,(*floor)[ids[1]].z,(*floor)[ids[2]].z}};
    std::sort(z.begin(),z.end());double height=z[2]-z[0]>=.12?NAN:z[1]+origin.z();ground_cache[cache_key]=height;return height;
  }

};
inline void localGround(std::vector<Row>&rows,double sensor_height){
  std::map<Key,std::vector<V>>cells;std::map<Key,std::vector<size_t>>targets;
  for(size_t i=0;i<rows.size();++i){const auto&p=rows[i].sensor;if(p.x()< -15||p.x()>40||std::abs(p.y())>12)continue;
    targets[{int(std::floor(p.x()/6)),int(std::floor(p.y()/4))}].push_back(i);
    if(p.z()>-sensor_height-1&&p.z()<-sensor_height+.8)cells[{int(std::floor(p.x())),int(std::floor(p.y()))}].push_back(p);}
  std::vector<V>seeds;
  for(auto&kv:cells){auto&v=kv.second;if(v.size()<3)continue;std::sort(v.begin(),v.end(),[](const V&a,const V&b){return a.z()<b.z();});double rank=.3*(v.size()-1);size_t lo=std::floor(rank),hi=std::ceil(rank);double z=v[lo].z()*(hi-rank)+v[hi].z()*(rank-lo);if(lo==hi)z=v[lo].z();std::vector<double>x,y,h;for(const auto&p:v)if(p.z()<=z){x.push_back(p.x());y.push_back(p.y());h.push_back(p.z());}seeds.emplace_back(median(x),median(y),median(h));}
  for(auto&kv:targets){Eigen::Vector2d center((kv.first.first+.5)*6,(kv.first.second+.5)*4);std::vector<V>near;for(const auto&p:seeds)if(std::abs(p.x()-center.x())<=9&&std::abs(p.y()-center.y())<=6)near.push_back(p);if(near.size()<12)continue;
    std::vector<bool>keep(near.size(),true);Eigen::Vector3d coef=V::Zero();bool supported=true;
    for(int iter=0;iter<4;++iter){size_t count=std::count(keep.begin(),keep.end(),true);if(count<12){supported=false;break;}Eigen::MatrixXd A(count,3);Eigen::VectorXd z(count);size_t j=0;for(size_t i=0;i<near.size();++i)if(keep[i]){A.row(j)<<near[i].x()-center.x(),near[i].y()-center.y(),1.;z[j++]=near[i].z();}coef=A.colPivHouseholderQr().solve(z);for(size_t i=0;i<near.size();++i)keep[i]=std::abs(near[i].z()-V(near[i].x()-center.x(),near[i].y()-center.y(),1).dot(coef))<=.12;}
    if(!supported||std::count(keep.begin(),keep.end(),true)<12||coef.head<2>().norm()>.12)continue;
    double x0=1e9,x1=-1e9,y0=1e9,y1=-1e9,error=0;int count=0;for(size_t i=0;i<near.size();++i)if(keep[i]){x0=std::min(x0,near[i].x());x1=std::max(x1,near[i].x());y0=std::min(y0,near[i].y());y1=std::max(y1,near[i].y());double e=near[i].z()-V(near[i].x()-center.x(),near[i].y()-center.y(),1).dot(coef);error+=e*e;++count;}
    if(x1-x0<2||y1-y0<1||std::sqrt(error/count)>.07)continue;
    for(size_t i:kv.second){auto&r=rows[i];double local=(r.sensor.z()-V(r.sensor.x()-center.x(),r.sensor.y()-center.y(),1).dot(coef))/std::sqrt(1+coef.head<2>().squaredNorm());bool restore=r.valid&&std::abs(local)<=.04&&r.height>=.08&&r.height<=.30;
      if(!r.valid||std::abs(local-r.height)<=.10||restore){r.ground=r.sensor.z()-local;r.height=local;r.valid=true;} }
  }
}
struct History{double stamp=-1;int hits=0;Eigen::Vector2d center=Eigen::Vector2d::Zero();};
class Confirmation {
 public:
  int min_points=3,scans=2;bool allow_unknown_ground=true;double memory=.35;std::map<Key,History>history;
  void update(std::vector<Row>&rows,double stamp){
    for(auto it=history.begin();it!=history.end();)if(stamp-it->second.stamp<0||stamp-it->second.stamp>memory)it=history.erase(it);else++it;
    std::map<Key,std::vector<size_t>>groups;for(size_t i=0;i<rows.size();++i)if(rows[i].candidate)groups[{int(std::floor(rows[i].world.x()/.5)),int(std::floor(rows[i].world.y()/.5))}].push_back(i);
    std::set<Key>visited,used;for(const auto&kv:groups){if(visited.count(kv.first))continue;std::vector<Key>stack{kv.first};visited.insert(kv.first);std::vector<size_t>indices;
      while(!stack.empty()){Key k=stack.back();stack.pop_back();auto&v=groups.at(k);indices.insert(indices.end(),v.begin(),v.end());for(int x=-1;x<=1;++x)for(int y=-1;y<=1;++y){Key q{k.first+x,k.second+y};if(groups.count(q)&&!visited.count(q)){visited.insert(q);stack.push_back(q);}}}
      if(indices.size()<size_t(min_points))continue;Eigen::Vector2d center=Eigen::Vector2d::Zero(),lo=Eigen::Vector2d::Constant(1e12),hi=-lo;for(size_t i:indices){Eigen::Vector2d p=rows[i].world.head<2>();center+=p;lo=lo.cwiseMin(p);hi=hi.cwiseMax(p);}center/=indices.size();double best=std::max(.6,(hi-lo).norm()*.5);Key key{int(std::floor(center.x()/.5)),int(std::floor(center.y()/.5))};bool found=false;History previous;
      for(const auto&h:history)if(!used.count(h.first)){double d=(center-h.second.center).norm();if(d<=best){best=d;key=h.first;previous=h.second;found=true;}}
      if(!found)previous=History();int hits=previous.hits+(stamp>previous.stamp+1e-6?1:0);history[key]={stamp,hits,center};used.insert(key);
      for(size_t i:indices)rows[i].label=hits>=scans?(rows[i].valid?3:(allow_unknown_ground?4:2)):2;
    }
  }
};
struct WallAlignment {bool valid=false;double lateral=0,left_mad=0,right_mad=0;int left=0,right=0;};
// A small common translation of BOTH walls is evidence of map/pose mismatch.
// Independent objects do not establish a correction; yaw/localization is untouched.
inline WallAlignment wallAlignment(const std::vector<V>&points,const Pose&pose,const Map&map,const V&sensor){
  WallAlignment out;std::vector<double> shifts[2],along[2];
  const double c=std::cos(pose.yaw),s=std::sin(pose.yaw);
  size_t stride=std::max(size_t(1),points.size()/1200);
  for(size_t i=0;i<points.size();i+=stride){const V&q=points[i];
    if(q.x()<1||q.x()>35||std::abs(q.y())<2||std::abs(q.y())>12||q.z()<-.5||q.z()>1.5)continue;
    V b=q+sensor,w(pose.x+c*b.x()-s*b.y(),pose.y+s*b.x()+c*b.y(),pose.z+b.z());
    std::vector<int> ids(1);std::vector<float> distance(1);V local=w-map.origin;
    if(map.tree.nearestKSearch(pcl::PointXYZ(local.x(),local.y(),local.z()),1,ids,distance)!=1||distance[0]>.35*.35)continue;
    const auto&p=(*map.points)[ids[0]];V target=map.origin+V(p.x,p.y,p.z);
    if(std::abs(target.z()-w.z())>.08)continue;
    int side=q.y()>0?0:1;shifts[side].push_back(-s*(target.x()-w.x())+c*(target.y()-w.y()));along[side].push_back(q.x());
  }
  out.left=shifts[0].size();out.right=shifts[1].size();
  if(out.left<30||out.right<30)return out;
  double med[2],mad[2];for(int side=0;side<2;++side){
    if(*std::max_element(along[side].begin(),along[side].end())-*std::min_element(along[side].begin(),along[side].end())<8)return out;
    med[side]=median(shifts[side]);std::vector<double>d;for(double v:shifts[side])d.push_back(std::abs(v-med[side]));mad[side]=median(d);}
  out.left_mad=mad[0];out.right_mad=mad[1];out.lateral=(med[0]+med[1])*.5;
  out.valid=std::abs(med[0]-med[1])<=.04&&std::max(mad[0],mad[1])<=.04&&std::abs(out.lateral)<=.20;
  return out;
}
inline std::vector<Row> classify(const std::vector<V>&xyz,const Pose&p,Map*map,const V&sensor,double sensor_height=1.765,double min_height=.08){
  std::vector<Row>rows;rows.reserve(xyz.size());double c=std::cos(p.yaw),s=std::sin(p.yaw);
  for(const auto&q:xyz){if(!q.allFinite())continue;Row r;r.sensor=q;V b=q+sensor;r.world=V(p.x+c*b.x()-s*b.y(),p.y+s*b.x()+c*b.y(),p.z+b.z());r.inside=b.x()>-.79&&b.x()<3.85&&std::abs(b.y())<.95;
    bool roi=q.x()>=-15&&q.x()<=40&&std::abs(q.y())<=12&&q.z()+sensor_height<=3.2;
    if(map&&roi){double g=map->ground(r.world);r.valid=std::isfinite(g);if(r.valid){r.height=r.world.z()-g;r.ground=q.z()-r.height;}r.bg=map->backgroundMatches(r.world,r.height,r.valid);}
    rows.push_back(r);}
  // Preserve the Python restore condition without a second floor lookup.
  std::vector<double>before;before.reserve(rows.size());for(const auto&r:rows)before.push_back(r.height);localGround(rows,sensor_height);
  for(size_t i=0;i<rows.size();++i){auto&r=rows[i];double h=r.valid?r.height:r.sensor.z()+sensor_height;bool restore=std::isfinite(before[i])&&std::abs(h)<=.04&&before[i]>=.08&&before[i]<=.30;
    if(restore&&map){V query=r.world;query.z()+=h-before[i];r.bg|=map->matches(query,map->floor_distance);}
    bool roi=r.sensor.x()>=-15&&r.sensor.x()<=40&&std::abs(r.sensor.y())<=12;r.candidate=roi&&!r.bg&&!r.inside&&h>=min_height&&h<=2.8;r.label=r.bg?1:(r.candidate?2:0);}
  return rows;
}
}
