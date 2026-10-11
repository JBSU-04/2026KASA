/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
#include <morai_msgs/GPSMessage.h>

using namespace gps_common;

static ros::Publisher odom_back_pub;
static ros::Publisher odom_front_pub;

static std::string frame_id;
static bool append_zone = false;

static std::string gps_back_topic;
static std::string gps_front_topic;
static std::string odom_back_topic;
static std::string odom_front_topic;

static std::string child_frame_id_back;
static std::string child_frame_id_front;

static void publishOdomFromGps(const morai_msgs::GPSMessage& fix, ros::Publisher& pub, const std::string& child_frame_id)
{
//  if (fix.status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
//    ROS_DEBUG_THROTTLE(60,"No fix.");
//    return;
//  }

  if (fix.header.stamp == ros::Time(0)) {
    return;
  }

  double northing= 0.0, easting = 0.0;
  std::string zone;
  LLtoUTM(fix.latitude, fix.longitude, northing, easting, zone);

  nav_msgs::Odometry odom;
  odom.header.stamp = fix.header.stamp;

  // frame_id 결정
  if (frame_id.empty()) {
    if (append_zone) odom.header.frame_id = fix.header.frame_id + "/utm_" + zone;
    else             odom.header.frame_id = fix.header.frame_id;
  } else {
    if (append_zone) odom.header.frame_id = frame_id + "/utm_" + zone;
    else             odom.header.frame_id = frame_id;
  }

  odom.child_frame_id = child_frame_id;

  // UTM (easting=x, northing=y)
  odom.pose.pose.position.x = easting;
  odom.pose.pose.position.y = northing;
  odom.pose.pose.position.z = fix.altitude;  // 2D면 안 써도 무방

  // yaw는 state_information에서 만들 것 → 여기서는 identity
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 1.0;

  pub.publish(odom);

}
static void gpsBackCallBack(const morai_msgs::GPSMessage::ConstPtr& msg){
  publishOdomFromGps(*msg, odom_back_pub, child_frame_id_back);
}
static void gpsFrontCallBack(const morai_msgs::GPSMessage::ConstPtr& msg){
  publishOdomFromGps(*msg, odom_front_pub, child_frame_id_front);
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "utm_odometry_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Time::init();
  pnh.param<std::string>("frame_id", frame_id, "");
  pnh.param<bool>("append_zone", append_zone, false);
   
  pnh.param<std::string>("topics/gps_back", gps_back_topic, std::string("/gps_back"));
  pnh.param<std::string>("topics/gps_front", gps_front_topic, std::string("/gps_front"));
  
  pnh.param<std::string>("topics/odom_back", odom_back_topic, std::string("/odom_back"));
  pnh.param<std::string>("topics/odom_front", odom_front_topic, std::string("/odom_front"));

  pnh.param<std::string>("child_frame_id_back",  child_frame_id_back,  std::string("gps_back"));
  pnh.param<std::string>("child_frame_id_front", child_frame_id_front, std::string("gps_front"));

  odom_back_pub = nh.advertise<nav_msgs::Odometry>(odom_back_topic, 10);
  odom_front_pub = nh.advertise<nav_msgs::Odometry>(odom_front_topic, 10);

  ros::Subscriber sub_back  = nh.subscribe(gps_back_topic,  10, gpsBackCallBack);
  ros::Subscriber sub_front = nh.subscribe(gps_front_topic, 10, gpsFrontCallBack);
  
  ros::spin();

  return 0;
}

