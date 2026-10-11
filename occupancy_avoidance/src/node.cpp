#include "occupancy_avoidance/ros_adapter.hpp"
int main(int argc,char**argv){ros::init(argc,argv,"occupancy_avoidance_node");
  try{occupancy_avoidance::RosAdapter node(ros::NodeHandle("~"),false);ros::spin();}
  catch(const std::exception&e){ROS_FATAL("occupancy: %s",e.what());return 1;}return 0;}
