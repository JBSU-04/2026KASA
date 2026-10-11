#include <ros/ros.h>
#include <iostream>
#include <array>
#include <vector>
#include <cmath>
#include <numeric>


#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
#include <std_msgs/UInt8MultiArray.h>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>

#include <math.h>
#include <algorithm>
#include <limits>
#include <tuple>


#define _USE_MATH_DEFINES

using namespace std;
typedef pcl::PointXYZ PointType;


class LaneChanger{

private:
	//ros
	ros::NodeHandle nh_;

	//publisher	
		ros::Publisher pub_obstacle_;
		ros::Publisher pub_marker_;
				ros::Publisher pub_center_;
					ros::Publisher pub_center_points_;
					ros::Publisher pub_clustered_points_;
					ros::Publisher pub_cluster_boxes_;
					ros::Publisher pub_obstacle_roi_;
					ros::Publisher pub_obstacle_boxes_;
					ros::Publisher pub_obstacle_roi_states_;
					ros::Publisher pub_points_;
	//subscriber
	ros::Subscriber sub_;

		//msg
		std_msgs::Header latest_cloud_header_;
		geometry_msgs::Point wp;
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
				float obstacle_x_min;
				float obstacle_x_max;
				std::array<float, 3> obstacle_roi_y_min;
				std::array<float, 3> obstacle_roi_y_max;
				float obstacle_z_min;
				float obstacle_z_max;
				float obstacle_min_height;
				float obstacle_max_height;
				int obstacle_confirm_frames;
				int obstacle_clear_frames;
				std::array<int, 3> obstacle_detect_count_{{0, 0, 0}};
				std::array<int, 3> obstacle_clear_count_{{0, 0, 0}};
				std::array<bool, 3> obstacle_state_{{false, false, false}};


		//vector	
    vector<pcl::PointCloud<pcl::PointXYZ>> clustered_;
		vector<pcl::PointXYZ> center_;
		vector<float> distance;//vector for distance using angle to index

    //pointcloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr msgCloud;

	public:
				void initSetup();
					void pointCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);
								void clustering();
								bool cluster_intersects_obstacle_roi(const pcl::PointCloud<pcl::PointXYZ> &cluster);
								bool cluster_intersects_obstacle_roi(const pcl::PointCloud<pcl::PointXYZ> &cluster, size_t roi_index);
									void pt_center();
								void visualize_center(vector<pcl::PointXYZ> input_points);
									void publish_center_points(vector<pcl::PointXYZ> input_points);
									void publish_clustered_points();
									void publish_cluster_boxes();
									void publish_obstacle_roi();
									void publish_obstacle_boxes();
									void visualize(geometry_msgs::Point input_point);

	void run();
	void Static();

		    LaneChanger() {
		    initSetup();
			msgCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
	    }

	};
