// M8 double-precision numerical kernels. No classification or inferred free space.
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>
#include <numeric>
#include <limits>
#include <unordered_map>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
using P=std::array<double,3>;
struct Rays {
  struct Node {int id,left=-1,right=-1,axis=0;};
  P origin;std::vector<P> dirs;std::vector<double> ranges;std::vector<Node> nodes;
  int build(std::vector<int>&ids,int a,int b,int axis){if(a==b)return -1;int mid=(a+b)/2;
    std::nth_element(ids.begin()+a,ids.begin()+mid,ids.begin()+b,[&](int x,int y){return dirs[x][axis]<dirs[y][axis];});
    int n=nodes.size();nodes.push_back({ids[mid],-1,-1,axis});int l=build(ids,a,mid,(axis+1)%3),r=build(ids,mid+1,b,(axis+1)%3);nodes[n].left=l;nodes[n].right=r;return n;}
  void nearest(int n,const P&q,std::vector<std::pair<double,int>>&best)const{if(n<0)return;const auto&v=nodes[n];double d=0;
    for(int k=0;k<3;++k)d+=(q[k]-dirs[v.id][k])*(q[k]-dirs[v.id][k]);best.emplace_back(d,v.id);std::sort(best.begin(),best.end());if(best.size()>4)best.pop_back();
    double gap=q[v.axis]-dirs[v.id][v.axis];nearest(gap<0?v.left:v.right,q,best);if(best.size()<4||gap*gap<=best.back().first)nearest(gap<0?v.right:v.left,q,best);}
};
namespace bg=boost::geometry;namespace bi=boost::geometry::index;
using XY=bg::model::d2::point_xy<double>;using Box=bg::model::box<XY>;using Item=std::pair<Box,int>;
struct Route {std::vector<std::array<double,4>> segments;bi::rtree<Item,bi::quadratic<16>> tree;};
static double segment(const std::array<double,4>&s,double x,double y){double vx=s[2]-s[0],vy=s[3]-s[1],l=vx*vx+vy*vy;
  double u=l>0?std::max(0.,std::min(1.,((x-s[0])*vx+(y-s[1])*vy)/l)):0;return std::hypot(x-s[0]-u*vx,y-s[1]-u*vy);}
extern "C" {
void* m8_rays_create(const double*o,const double*p,int n){try{auto*r=new Rays;r->origin={o[0],o[1],o[2]};
  for(int i=0;i<n;++i){P d;double l=0;for(int k=0;k<3;++k){d[k]=p[3*i+k]-o[k];l+=d[k]*d[k];}l=std::sqrt(l);if(!std::isfinite(l)||l<=.2)continue;for(auto&v:d)v/=l;r->dirs.push_back(d);r->ranges.push_back(l);}
  std::vector<int>ids(r->dirs.size());std::iota(ids.begin(),ids.end(),0);r->nodes.reserve(ids.size());r->build(ids,0,ids.size(),0);return r;}catch(...){return nullptr;}}
void m8_rays_destroy(void*p){delete static_cast<Rays*>(p);}
int m8_rays_clear(void*ptr,const double*p,int n){auto*r=static_cast<Rays*>(ptr);if(!r||r->dirs.empty()||n<=0)return 0;
  for(int i=0;i<n;++i){P delta,q;double length=0;for(int k=0;k<3;++k){delta[k]=p[3*i+k]-r->origin[k];length+=delta[k]*delta[k];}length=std::sqrt(length);if(!std::isfinite(length)||length<.2)return 0;for(int k=0;k<3;++k)q[k]=delta[k]/length;
    std::vector<std::pair<double,int>>best;best.reserve(5);r->nearest(0,q,best);bool free=false;
    for(auto b:best){int j=b.second;double along=0,transverse=0;for(int k=0;k<3;++k)along+=delta[k]*r->dirs[j][k];for(int k=0;k<3;++k)transverse+=std::pow(delta[k]-along*r->dirs[j][k],2);
      if(transverse<=.08*.08&&along>0&&r->ranges[j]>along+.20){free=true;break;}}
    if(!free)return 0;}return 1;}
void* m8_route_create(const double*p,int n){try{auto*r=new Route;std::vector<Item>items;for(int i=0;i+1<n;++i){std::array<double,4>s{{p[2*i],p[2*i+1],p[2*i+2],p[2*i+3]}};
  if(!std::isfinite(s[0]+s[1]+s[2]+s[3])){delete r;return nullptr;}r->segments.push_back(s);items.emplace_back(Box(XY(std::min(s[0],s[2]),std::min(s[1],s[3])),XY(std::max(s[0],s[2]),std::max(s[1],s[3]))),i);}
  r->tree=decltype(r->tree)(items);return r;}catch(...){return nullptr;}}
void m8_route_destroy(void*p){delete static_cast<Route*>(p);}
double m8_route_distance(void*ptr,const double*p,int n){auto*r=static_cast<Route*>(ptr);double best=std::numeric_limits<double>::infinity();if(!r||r->segments.empty())return best;
  for(int i=0;i<n;++i){double x=p[2*i],y=p[2*i+1];if(!std::isfinite(x+y))continue;std::vector<Item>near;r->tree.query(bi::nearest(XY(x,y),1),std::back_inserter(near));double bound=segment(r->segments[near[0].second],x,y);
    near.clear();r->tree.query(bi::intersects(Box(XY(x-bound,y-bound),XY(x+bound,y+bound))),std::back_inserter(near));for(auto s:near)bound=std::min(bound,segment(r->segments[s.second],x,y));best=std::min(best,bound);}return best;}
// Exact XY radius connectivity. Each small bin is a complete connected set;
// one crossing edge connects two complete sets. Every input point is retained.
int m8_components(const double*p,int n,double eps,int*out){try{
  if(n<0||eps<=0)return 0;double leaf=eps/std::sqrt(2.);
  auto key=[](int x,int y){return (uint64_t(uint32_t(x))<<32)|uint32_t(y);};
  std::unordered_map<uint64_t,std::vector<int>> bins;std::vector<int>parent(n);std::iota(parent.begin(),parent.end(),0);
  for(int i=0;i<n;++i)bins[key(int(std::floor(p[2*i]/leaf)),int(std::floor(p[2*i+1]/leaf)))].push_back(i);
  auto find=[&](int i){while(parent[i]!=i){parent[i]=parent[parent[i]];i=parent[i];}return i;};
  for(auto&kv:bins)for(int i:kv.second)parent[i]=kv.second.front();
  for(auto&kv:bins){int x=int(int32_t(kv.first>>32)),y=int(int32_t(kv.first));
    for(int dx=-2;dx<=2;++dx)for(int dy=-2;dy<=2;++dy){auto k=key(x+dx,y+dy);if(k<=kv.first)continue;auto it=bins.find(k);if(it==bins.end())continue;
      int a=find(kv.second.front()),b=find(it->second.front());if(a==b)continue;bool hit=false;
      for(int i:kv.second){for(int j:it->second){double vx=p[2*i]-p[2*j],vy=p[2*i+1]-p[2*j+1];if(vx*vx+vy*vy<=eps*eps){hit=true;break;}}if(hit)break;}
      if(hit)parent[a]=b;
    }}
  std::unordered_map<int,int> first;
  for(int i=0;i<n;++i){int root=find(i);if(!first.count(root))first[root]=i;}
  for(int i=0;i<n;++i)out[i]=first[find(i)];return 1;
}catch(...){return 0;}}
int m8_sweep(const double*p,int n,const double*path,int m,double ex,double ey,double yaw,double horizon){try{
  if(m<2)return -1;for(int k=0;k+1<m;++k)if(std::hypot(path[2*k+2]-path[2*k],path[2*k+3]-path[2*k+1])<1e-6)return -1;double pad=.2+.1*.7072,front=3.85+pad,rear=.79+pad,half=.95+pad;
  auto hit=[&](double x,double y,double angle,int i){double c=std::cos(angle),s=std::sin(angle),dx=p[2*i]-x,dy=p[2*i+1]-y,a=c*dx+s*dy,d=-s*dx+c*dy;return a>=-rear&&a<=front&&std::abs(d)<=half;};
  for(int i=0;i<n;++i)if(hit(ex,ey,yaw,i))return 1;
  if(n==0)return 0;
  std::vector<double>len(m-1),projection(m-1);int start=0;double nearest=INFINITY;
  for(int k=0;k+1<m;++k){double dx=path[2*k+2]-path[2*k],dy=path[2*k+3]-path[2*k+1];len[k]=std::hypot(dx,dy);if(len[k]<1e-6)return -1;
    double u=std::max(0.,std::min(1.,((ex-path[2*k])*dx+(ey-path[2*k+1])*dy)/(len[k]*len[k])));projection[k]=u;
    double d=std::hypot(ex-path[2*k]-u*dx,ey-path[2*k+1]-u*dy);if(d<nearest){nearest=d;start=k;}}
  double tx=(path[2*m-2]-path[2*m-4])/len.back(),ty=(path[2*m-1]-path[2*m-3])/len.back();bool beyond=true;
  for(int i=0;i<n;++i)if((p[2*i]-path[2*m-2])*tx+(p[2*i+1]-path[2*m-1])*ty<=3.85){beyond=false;break;}if(beyond)return -1;
  std::vector<std::pair<XY,int>>items;for(int i=0;i<n;++i)items.emplace_back(XY(p[2*i],p[2*i+1]),i);bi::rtree<std::pair<XY,int>,bi::quadratic<16>>tree(items);
  double travelled=0,radius=std::hypot(front,half);
  for(int k=start;k+1<m;++k){double a=std::atan2(path[2*k+3]-path[2*k+1],path[2*k+2]-path[2*k]),begin=k==start?projection[k]*len[k]:0;
    for(double d=begin;d<=len[k]+1e-6;d+=.1){double x=path[2*k]+d*std::cos(a),y=path[2*k+1]+d*std::sin(a);std::vector<std::pair<XY,int>>near;
      tree.query(bi::intersects(Box(XY(x-radius,y-radius),XY(x+radius,y+radius))),std::back_inserter(near));for(auto q:near)if(hit(x,y,a,q.second))return 1;}
    travelled+=len[k]-begin;if(travelled>=horizon)break;}
  return 0;
}catch(...){return -1;}}
}
