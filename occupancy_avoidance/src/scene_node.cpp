#include "occupancy_avoidance/ros_adapter.hpp"
int main(int argc,char**argv){ros::init(argc,argv,"tunnel_perception_scene");ros::NodeHandle p("~");p.setParam("scene_only",true);
try{occupancy_avoidance::RosAdapter node(p,false);ros::spin();}catch(const std::exception&e){ROS_FATAL("tunnel scene: %s",e.what());return 1;}return 0;}
