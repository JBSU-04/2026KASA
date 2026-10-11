#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <numeric>

#include <new_lane/lidar_topic_msg.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>

#include <nav_msgs/Odometry.h>
#include <math.h>
#include <algorithm>
#include <tuple>
// #include <morai_msgs/CtrlCmd.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

#include <waypoint_maker/Lane.h>
#include <waypoint_maker/Waypoint.h>
#include <waypoint_maker/State.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include "new_lane/frenet_optimal_trajectory.hpp"
#include "new_lane/frenetROS_obst.hpp"
#include <new_lane/cubic_spline_planner.hpp>
#include <waypoint_maker/State.h>


#define _USE_MATH_DEFINES

using namespace std;
using vecD = vector<double>;
typedef pcl::PointXYZ PointType;

struct PointXYZV{
  int id;
  float width;
  float height;
  float z_height;
  float x;
  float y;
  float z;
  float dist;
  float angle;
  double vel;
};// PointXYZV 구조체 정의

class LaneChanger{

private:
	//ros
	ros::NodeHandle nh_;

	//publisher		
	ros::Publisher pub_marker_;
	ros::Publisher pub_center_;
	ros::Publisher pub_center_local;
	ros::Publisher pub_points_;
	ros::Publisher lidar_global_pub;
	ros::Publisher frenet_path;
	ros::Publisher frenet_path_2;
	ros::Publisher pub_offset_path1;
	ros::Publisher pub_offset_path2;
	ros::Publisher global_path;
	ros::Publisher frenet_pub_;
	ros::Publisher pub_lane_;
	ros::Publisher pub_frenet_points;
	ros::Publisher pub_lidar_topic;
	ros::Publisher pub_path_number_;

	
	//subscriber
	ros::Subscriber sub_;
	ros::Subscriber lane_sub_;
	// ros::Subscriber imu_sub_;
	// ros::Subscriber course_sub_;
	ros::Subscriber odom_sub_;
	ros::Subscriber state_sub_;
	// ros::Subscriber closest_index_sub_;


	//msg
	geometry_msgs::Point wp;
	std_msgs::Float64 motor_speed;
	std_msgs::Float64 motor_angle;
	
	//various
	float steering_angle_;
	float speed_;

	//param
	double sepe;
	double str_;
	
	double cluster_tolerance_;
	int cluster_minsize_;
	int cluster_maxsize_;

	float x_min;
	float x_max;
	float y_min;
	float y_max;
	float z_min;
	float z_max;
	
	
	//local point
	int prev_center_size = 0;
	float local_dist;
	
	//Decision
	int dynamic_cnt = 0;
	bool dynamic_flag = false;
	bool static_flag = false;

	//Circle Decision
	float is_circle1 = 0.0;
	float is_circle2 = 0.0;
	float is_circle3 = 0.0;
	int obs_index1 = 0;
	int obs_index2 = 0;
	int obs_index1_c = 0;
	int obs_index2_c = 0;


	//waypoints
	vector<waypoint_maker::Waypoint> waypoints_;
	int waypoints_size_;
	bool is_lane_ = false;
	double cur_course_rad_ = 0.0;
	geometry_msgs::PoseStamped cur_pose_;
	geometry_msgs::PoseArray cur_pose_global;

	vecD rx, ry, ryaw, rk;
	double ds = 0.1; // ds represents the step size for cubic_spline
	double bot_yaw, bot_v;

	FrenetPath path;
	FrenetPath path_2;
	FrenetPath lp;
	double s0, c_d, c_d_d, c_d_dd, c_speed;
	vecD frenet_x, frenet_y, frenet_x2, frenet_y2;
	double D_OFFSET;
	double LINEAR_SPEED;
	// geometry_msgs::PoseStamped frenet_pose_;
	waypoint_maker::Waypoint frenet_pose_;
	vector<waypoint_maker::Waypoint> frenet_points_;
	waypoint_maker::Lane frenet_msg;
	vecD rx2, ry2, ryaw2, rk2;
	vecD rx3, ry3, ryaw3, rk3;
	

	//local to global obstacle
	vecD global_x, global_y, global_z;
	vector<pcl::PointXYZ> global_obs;
	// vector<int> is_obs_index1;
	// vector<int> is_obs_index2;
	vector<float> is_obs_dist;

	//lane change
	// waypoint_maker::State lane_msg;
	bool is_obs1 = false;
	bool is_obs2 = false;
	bool is_obs1_c = false;
	bool is_obs2_c = false;
	bool my_line_obs = false;
	bool other_line_obs = false;
	bool obs_left_line = false;
	int lane_num = 0;
	int lane_number_ = 0;
	int current_state_ = 0;
	int lane_cnt1 = 0;
	int lane_cnt2 = 0;
	int acc_false_cnt_ = 0;
	int acc_false_debounce_count_ = 10; // 3~4 

	// int lane_cnt1_c = 0;
	// int lane_cnt2_c = 0;

	// path offset
    vector<waypoint_maker::Waypoint> offset_path1;
	// vector<geometry_msgs::Point> offset_path2;
    vecD offset_path1_x;
    vecD offset_path1_y;
	vecD offset_path2_x;
    vecD offset_path2_y;
	double real_D_OFFSET = 0;

	//

	tf2_ros::StaticTransformBroadcaster broadcaster;
	tf2_ros::StaticTransformBroadcaster broadcaster2;

    geometry_msgs::TransformStamped static_transformStamped;
    geometry_msgs::TransformStamped static_transformStamped2;

	new_lane::lidar_topic_msg lidar_topic_msg;

	// ACC
	int closest_index = 0;
	ros::Time start_time;
	ros::Time end_time;
	ros::Duration sec_duration;
	float other_car_speed = 0.0;
	bool crusie_dist_time = false;
	bool crusie_one_time = false;
	vector<int> before_index1;
	vector<int> before_index2;
	int acc_index1;
	int acc_index2; //off set
	int acc_index1_c;
	// int acc_index2_c; //off set
	bool acc_flag = false;


	//vector	
	vector<pcl::PointCloud<pcl::PointXYZ>> clustered_;
	vector<PointXYZV> center_;
	vector<float> distance;//vector for distance using angle to index
	vector<PointType> local_point;

    //pointcloud
	pcl::PointCloud<PointType>::Ptr msgCloud;

public:
	void initSetup();
	void pointCallback(const sensor_msgs::PointCloud2::ConstPtr &scan);
	void LaneCallback(const waypoint_maker::Lane::ConstPtr &lane_msg);
	void ImuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg);
	void CourseCallback(const std_msgs::Float64::ConstPtr &course_msg);
	void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);
	void state_Callback(const waypoint_maker::State::ConstPtr &msg);
	void closest_index_Callback(const std_msgs::Int32::ConstPtr &msg);
	void frenetPath();
	void Local_to_Global(vector<PointXYZV> input_points);
	void clustering(const pcl::PointCloud<PointType>::Ptr cloud_in);
	// void pt_center();
	void visualize_center(vector<pcl::PointXYZ> input_points);
	void visualize_center_local(vector<PointXYZV> input_points);
	void visualize(geometry_msgs::Point input_point);
	void run();
	void CircleDecision1(vecD x1, vecD y1, vecD x2, vecD y2, float r);
	void CircleDecision2(vecD x1, vecD y1, vecD x2, vecD y2, float r);
	bool CircleDecision1_c(vecD x1, vecD y1, vecD x2, vecD y2, float r);
	bool CircleDecision2_c(vecD x1, vecD y1, vecD x2, vecD y2, float r);
	float calcul_angle(float x, float y);
	float obs_min_speed(vector<pcl::PointXYZ> input_points);
	void LaneChage_Decision(bool my_obs, bool other_obs);
	geometry_msgs::Point calculateNormal(const waypoint_maker::Waypoint& p1, const waypoint_maker::Waypoint& p2);
	void offsetPath(const vector<waypoint_maker::Waypoint>& before_path, double offset);
	int find_closest_waypoint(vector<pcl::PointXYZ> obs_points, int index, vector<waypoint_maker::Waypoint> path);
	void only_ACC(bool my_obs);
	void only_ACC_2(bool my_obs);

	// bool CircleDecisionACC(vecD x1, vecD x2, double a, double b);

	static bool cmp(const PointXYZV &v1, const PointXYZV &v2);
	
    LaneChanger() {
	    initSetup();
		msgCloud.reset(new pcl::PointCloud<PointType>());
    }

};
