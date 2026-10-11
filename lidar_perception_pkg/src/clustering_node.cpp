#include "lidar_perception_pkg/clustering_node.h"


void LaneChanger::initSetup() {
	//publisher
	pub_obstacle_ = nh_.advertise<std_msgs::Bool>("obstacle_detected", 1);
	pub_points_ = nh_.advertise<sensor_msgs::PointCloud2>("roi_points", 10);
	pub_center_ = nh_.advertise<visualization_msgs::Marker>("center_points", 10);
	pub_center_points_ = nh_.advertise<sensor_msgs::PointCloud2>("cluster_centers", 10);
	pub_clustered_points_ = nh_.advertise<sensor_msgs::PointCloud2>("clustered_points", 10);
	pub_cluster_boxes_ = nh_.advertise<visualization_msgs::MarkerArray>("cluster_boxes", 10);
	pub_obstacle_roi_ = nh_.advertise<visualization_msgs::MarkerArray>("obstacle_rois", 10);
	pub_obstacle_boxes_ = nh_.advertise<visualization_msgs::MarkerArray>("obstacle_boxes", 10);
	pub_obstacle_roi_states_ = nh_.advertise<std_msgs::UInt8MultiArray>("obstacle_roi_states", 1);
	pub_marker_ = nh_.advertise<visualization_msgs::Marker>("wayPoint", 10);
	
		//param for static
		ros::param::get("~cluster_tolerance", cluster_tolerance_);
		ros::param::get("~cluster_minsize", cluster_minsize_);
		ros::param::get("~cluster_maxsize", cluster_maxsize_);
		ros::param::get("~str_",str_);
	ros::param::get("~sepe",sepe);
	
	
		ros::param::get("~x_min_st", x_min);
			ros::param::get("~x_max_st", x_max);
			ros::param::get("~y_min_st", y_min);
			ros::param::get("~y_max_st", y_max);
					ros::param::get("~z_min_st", z_min);
					ros::param::get("~z_max_st", z_max);
					ros::param::param<float>("~obstacle_x_min", obstacle_x_min, x_min);
					ros::param::param<float>("~obstacle_x_max", obstacle_x_max, x_max);
					ros::param::param<float>("~obstacle_roi_0_y_min", obstacle_roi_y_min[0], 2.0f);
					ros::param::param<float>("~obstacle_roi_0_y_max", obstacle_roi_y_max[0], 6.0f);
					ros::param::param<float>("~obstacle_roi_1_y_min", obstacle_roi_y_min[1], -2.0f);
					ros::param::param<float>("~obstacle_roi_1_y_max", obstacle_roi_y_max[1], 2.0f);
					ros::param::param<float>("~obstacle_roi_2_y_min", obstacle_roi_y_min[2], -6.0f);
					ros::param::param<float>("~obstacle_roi_2_y_max", obstacle_roi_y_max[2], -2.0f);
					ros::param::param<float>("~obstacle_z_min", obstacle_z_min, z_min);
					ros::param::param<float>("~obstacle_z_max", obstacle_z_max, z_max);
					ros::param::param<float>("~obstacle_min_height", obstacle_min_height, 0.15f);
					ros::param::param<float>("~obstacle_max_height", obstacle_max_height, 3.0f);
					ros::param::param<int>("~obstacle_confirm_frames", obstacle_confirm_frames, 2);
					ros::param::param<int>("~obstacle_clear_frames", obstacle_clear_frames, 1);

				std::string pointcloud_topic = "/nonground";
			ros::param::get("~pointcloud_topic", pointcloud_topic);
		
		//subscriber
		sub_ = nh_.subscribe(pointcloud_topic, 1, &LaneChanger::pointCallback, this);
		
	}

void LaneChanger::clustering() {
				if (msgCloud->empty()) {
					clustered_.clear();
					return;
				}

						pcl::PointCloud<pcl::PointXYZ> inputCloud = *msgCloud;
						std::vector<int> valid_indices;
						pcl::removeNaNFromPointCloud(inputCloud, inputCloud, valid_indices);

						// ========
							pcl::PointCloud<pcl::PointXYZ> tmpCloud;
							pcl::PointCloud<pcl::PointXYZ> tmpCloud2;
							pcl::PointCloud<pcl::PointXYZ> passCloud;

						pcl::PassThrough<pcl::PointXYZ> pass;

							pass.setInputCloud(inputCloud.makeShared());
					pass.setFilterFieldName("x"); // axis x
					pass.setFilterLimits(x_min,x_max);
					pass.setFilterLimitsNegative(false);
			pass.filter(tmpCloud);

			pass.setInputCloud(tmpCloud.makeShared());
			pass.setFilterFieldName("y"); // axis y
				pass.setFilterLimits(y_min, y_max);
				pass.setFilterLimitsNegative(false);
				pass.filter(tmpCloud2);

				pass.setInputCloud(tmpCloud2.makeShared());
				pass.setFilterFieldName("z"); // axis z
				pass.setFilterLimits(z_min, z_max);
				pass.setFilterLimitsNegative(false);
				pass.filter(passCloud);

				sensor_msgs::PointCloud2 filteredOutput;
				pcl::toROSMsg(passCloud, filteredOutput);
				filteredOutput.header = latest_cloud_header_;
				pub_points_.publish(filteredOutput);

				// Populate point cloud...
				pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
				kdtree->setInputCloud(passCloud.makeShared());

			std::vector<pcl::PointIndices> clusterIndices;
			pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

		ec.setClusterTolerance(cluster_tolerance_);
		ec.setMinClusterSize(cluster_minsize_);    // set Minimum Cluster Size
		ec.setMaxClusterSize(cluster_maxsize_); // set Maximum Cluster Size

					ec.setSearchMethod(kdtree);
					ec.setInputCloud(passCloud.makeShared());

			ec.extract(clusterIndices);

		clustered_.clear();
		pcl::PointCloud<pcl::PointXYZ> tmpCloud4;

		int cluster_idx = 0;

		for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {
			tmpCloud4.clear();

						for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
							tmpCloud4.push_back(passCloud[*pit]);
						}

                // 관심 영역(ROI)을 벗어나는 클러스터는 아예 무시 (RViz 깔끔하게 유지 및 불필요한 트래킹 방지)
                if (cluster_intersects_obstacle_roi(tmpCloud4)) {
				    clustered_.push_back(tmpCloud4);
                }
			}

					publish_clustered_points();

									
								}

				bool LaneChanger::cluster_intersects_obstacle_roi(
						const pcl::PointCloud<pcl::PointXYZ> &cluster, size_t roi_index) {
					if (cluster.empty()) {
						return false;
					}
					if (roi_index >= obstacle_roi_y_min.size()) {
						return false;
					}

					float min_x = std::numeric_limits<float>::max();
					float min_y = std::numeric_limits<float>::max();
					float min_z = std::numeric_limits<float>::max();
					float max_x = std::numeric_limits<float>::lowest();
					float max_y = std::numeric_limits<float>::lowest();
					float max_z = std::numeric_limits<float>::lowest();

					for (const auto &point : cluster.points) {
						min_x = std::min(min_x, point.x);
						min_y = std::min(min_y, point.y);
						min_z = std::min(min_z, point.z);
						max_x = std::max(max_x, point.x);
						max_y = std::max(max_y, point.y);
						max_z = std::max(max_z, point.z);
					}

					const float obstacle_height = max_z - min_z;
					if (obstacle_height < obstacle_min_height || obstacle_height > obstacle_max_height) {
						return false;
					}

					// 터널 벽면, 가드레일 등 비정상적으로 길거나 넓은(10m 이상) 클러스터는 벽으로 간주하고 무시
					const float obstacle_length = max_x - min_x;
					const float obstacle_width = max_y - min_y;
					if (obstacle_length > 10.0 || obstacle_width > 10.0) {
						return false;
					}

					return min_x <= obstacle_x_max && max_x >= obstacle_x_min &&
						   min_y <= obstacle_roi_y_max[roi_index] &&
						   max_y >= obstacle_roi_y_min[roi_index] &&
						   min_z <= obstacle_z_max && max_z >= obstacle_z_min;
				}

				bool LaneChanger::cluster_intersects_obstacle_roi(
						const pcl::PointCloud<pcl::PointXYZ> &cluster) {
					for (size_t roi_index = 0; roi_index < obstacle_roi_y_min.size(); ++roi_index) {
						if (cluster_intersects_obstacle_roi(cluster, roi_index)) {
							return true;
						}
					}
					return false;
				}

				void LaneChanger::pt_center() {
		pcl::PointXYZ Pt_center;
		float center_x = 0;
		float center_y = 0;
		float center_z = 0;
	float size = 0;

	center_.clear();

		for (size_t i = 0; i < clustered_.size(); i++) {
			center_x = 0;
			center_y = 0;
			center_z = 0;

			size = clustered_[i].size();
			if(size==0){
				size=1;
			}

	  		for (size_t j = 0; j < size; j++) {
	       		center_x += clustered_[i].points[j].x / size;
				center_y += clustered_[i].points[j].y / size;
				center_z += clustered_[i].points[j].z / size;
			}

			Pt_center.x = center_x;
			Pt_center.y = center_y;
			Pt_center.z = center_z;
			center_.push_back(Pt_center);
		}
	}

	void LaneChanger::pointCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) {
				latest_cloud_header_ = cloud_msg->header;
				pcl::fromROSMsg(*cloud_msg, *msgCloud);
		}


void LaneChanger::run(){
	clustering();
	pt_center();

	std::array<bool, 3> raw_obstacle_detected{{false, false, false}};
	for (const auto &cluster : clustered_) {
		for (size_t roi_index = 0; roi_index < raw_obstacle_detected.size(); ++roi_index) {
			if (cluster_intersects_obstacle_roi(cluster, roi_index)) {
				raw_obstacle_detected[roi_index] = true;
			}
		}
	}

	for (size_t roi_index = 0; roi_index < raw_obstacle_detected.size(); ++roi_index) {
		if (raw_obstacle_detected[roi_index]) {
			obstacle_detect_count_[roi_index]++;
			obstacle_clear_count_[roi_index] = 0;
		} else {
			obstacle_clear_count_[roi_index]++;
			obstacle_detect_count_[roi_index] = 0;
		}

		if (obstacle_detect_count_[roi_index] >= std::max(obstacle_confirm_frames, 1)) {
			obstacle_state_[roi_index] = true;
		}
		if (obstacle_clear_count_[roi_index] >= std::max(obstacle_clear_frames, 1)) {
			obstacle_state_[roi_index] = false;
		}
	}

	std_msgs::Bool obstacle_msg;
	obstacle_msg.data = obstacle_state_[0] || obstacle_state_[1] || obstacle_state_[2];
	pub_obstacle_.publish(obstacle_msg);

	std_msgs::UInt8MultiArray roi_states_msg;
	roi_states_msg.data.reserve(obstacle_state_.size());
	for (bool occupied : obstacle_state_) {
		roi_states_msg.data.push_back(occupied ? 1 : 0);
	}
	pub_obstacle_roi_states_.publish(roi_states_msg);

	publish_center_points(center_);
	publish_cluster_boxes();
	publish_obstacle_roi();
	publish_obstacle_boxes();
	visualize_center(center_);
}


void LaneChanger::visualize_center(vector<pcl::PointXYZ> input_points) {
		visualization_msgs::Marker centers;
		geometry_msgs::Point point;

		centers.header = latest_cloud_header_;
		if (centers.header.frame_id.empty()) {
			centers.header.frame_id = "velodyne";
		}
		centers.header.stamp = ros::Time::now();
		centers.ns = "centers";
		centers.action = visualization_msgs::Marker::ADD;
		centers.pose.orientation.w = 1.0;
		centers.id = 2;
		centers.type = visualization_msgs::Marker::POINTS;
		centers.scale.x = 0.3; 
		centers.scale.y = 0.3;
		centers.color.a = 1.0;
		centers.color.g = 1.0f;

			for (int i = 0; i < input_points.size(); i++) {
				point.x = input_points[i].x;
				point.y = input_points[i].y;
				point.z = input_points[i].z;

				centers.points.push_back(point);
			}
	 	    pub_center_.publish(centers);
	}

	void LaneChanger::publish_center_points(vector<pcl::PointXYZ> input_points) {
			pcl::PointCloud<pcl::PointXYZ> center_cloud;
			center_cloud.header.frame_id = latest_cloud_header_.frame_id;
			center_cloud.is_dense = true;
			center_cloud.reserve(input_points.size());

			for (const auto &point : input_points) {
				center_cloud.push_back(point);
			}

			center_cloud.width = center_cloud.points.size();
			center_cloud.height = 1;

			sensor_msgs::PointCloud2 center_msg;
			pcl::toROSMsg(center_cloud, center_msg);
			center_msg.header = latest_cloud_header_;
			pub_center_points_.publish(center_msg);
	}

			void LaneChanger::publish_clustered_points() {
					pcl::PointCloud<pcl::PointXYZ> clustered_cloud;
					clustered_cloud.header.frame_id = latest_cloud_header_.frame_id;
					clustered_cloud.is_dense = true;

					size_t total_size = 0;
					for (const auto &cluster : clustered_) {
						total_size += cluster.points.size();
					}
					clustered_cloud.reserve(total_size);

					for (const auto &cluster : clustered_) {
						for (const auto &point : cluster.points) {
							clustered_cloud.push_back(point);
						}
			}

			clustered_cloud.width = clustered_cloud.points.size();
			clustered_cloud.height = 1;

			sensor_msgs::PointCloud2 clustered_msg;
			pcl::toROSMsg(clustered_cloud, clustered_msg);
			clustered_msg.header = latest_cloud_header_;
				pub_clustered_points_.publish(clustered_msg);
			}

		void LaneChanger::publish_cluster_boxes() {
				visualization_msgs::MarkerArray boxes;

			for (size_t i = 0; i < clustered_.size(); ++i) {
				if (clustered_[i].empty()) {
					continue;
				}

				float min_x = std::numeric_limits<float>::max();
				float min_y = std::numeric_limits<float>::max();
				float min_z = std::numeric_limits<float>::max();
				float max_x = std::numeric_limits<float>::lowest();
				float max_y = std::numeric_limits<float>::lowest();
				float max_z = std::numeric_limits<float>::lowest();

				for (const auto &point : clustered_[i].points) {
					min_x = std::min(min_x, point.x);
					min_y = std::min(min_y, point.y);
					min_z = std::min(min_z, point.z);
					max_x = std::max(max_x, point.x);
					max_y = std::max(max_y, point.y);
					max_z = std::max(max_z, point.z);
				}

				visualization_msgs::Marker box;
				box.header = latest_cloud_header_;
				box.ns = "cluster_boxes";
				box.id = static_cast<int>(i);
				box.type = visualization_msgs::Marker::CUBE;
				box.action = visualization_msgs::Marker::ADD;
				box.pose.position.x = (min_x + max_x) * 0.5;
				box.pose.position.y = (min_y + max_y) * 0.5;
				box.pose.position.z = (min_z + max_z) * 0.5;
				box.pose.orientation.w = 1.0;
				box.scale.x = std::max(max_x - min_x, 0.05f);
				box.scale.y = std::max(max_y - min_y, 0.05f);
				box.scale.z = std::max(max_z - min_z, 0.05f);
				box.color.a = 0.35;
				box.color.r = 1.0;
				box.color.g = 0.35;
				box.color.b = 0.05;
				box.lifetime = ros::Duration(0.5);
				boxes.markers.push_back(box);
			}

			visualization_msgs::Marker clear_marker;
			clear_marker.header = latest_cloud_header_;
			clear_marker.ns = "cluster_boxes";
			clear_marker.action = visualization_msgs::Marker::DELETEALL;
			boxes.markers.insert(boxes.markers.begin(), clear_marker);

			pub_cluster_boxes_.publish(boxes);
		}

			void LaneChanger::publish_obstacle_roi() {
				visualization_msgs::MarkerArray rois;
				for (size_t roi_index = 0; roi_index < obstacle_roi_y_min.size(); ++roi_index) {
					visualization_msgs::Marker roi;
					roi.header = latest_cloud_header_;
					if (roi.header.frame_id.empty()) roi.header.frame_id = "velodyne";
					roi.header.stamp = ros::Time::now();
					roi.ns = "obstacle_rois";
					roi.id = static_cast<int>(roi_index);
					roi.type = visualization_msgs::Marker::CUBE;
					roi.action = visualization_msgs::Marker::ADD;
					roi.pose.position.x = (obstacle_x_min + obstacle_x_max) * 0.5;
					roi.pose.position.y = (obstacle_roi_y_min[roi_index] + obstacle_roi_y_max[roi_index]) * 0.5;
					roi.pose.position.z = (obstacle_z_min + obstacle_z_max) * 0.5;
					roi.pose.orientation.w = 1.0;
					roi.scale.x = std::max(obstacle_x_max - obstacle_x_min, 0.05f);
					roi.scale.y = std::max(obstacle_roi_y_max[roi_index] - obstacle_roi_y_min[roi_index], 0.05f);
					roi.scale.z = std::max(obstacle_z_max - obstacle_z_min, 0.05f);
					roi.color.a = obstacle_state_[roi_index] ? 0.35 : 0.12;
					roi.color.r = roi_index == 2 ? 1.0 : 0.1;
					roi.color.g = roi_index == 1 ? 1.0 : 0.3;
					roi.color.b = roi_index == 0 ? 1.0 : 0.1;
					roi.lifetime = ros::Duration(0.0);
					rois.markers.push_back(roi);

					visualization_msgs::Marker label;
					label.header = roi.header;
					label.ns = "obstacle_roi_labels";
					label.id = static_cast<int>(roi_index);
					label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
					label.action = visualization_msgs::Marker::ADD;
					label.pose.position.x = obstacle_x_min + 1.0;
					label.pose.position.y = roi.pose.position.y;
					label.pose.position.z = obstacle_z_max + 0.5;
					label.pose.orientation.w = 1.0;
					label.scale.z = 0.6;
					label.color.a = 1.0;
					label.color.r = label.color.g = label.color.b = 1.0;
					label.text = "ROI " + std::to_string(roi_index);
					label.lifetime = ros::Duration(0.0);
					rois.markers.push_back(label);
				}
				pub_obstacle_roi_.publish(rois);
			}

		void LaneChanger::publish_obstacle_boxes() {
			visualization_msgs::MarkerArray boxes;

			for (size_t i = 0; i < clustered_.size(); ++i) {
				if (clustered_[i].empty() || !cluster_intersects_obstacle_roi(clustered_[i])) {
					continue;
				}

				float min_x = std::numeric_limits<float>::max();
				float min_y = std::numeric_limits<float>::max();
				float min_z = std::numeric_limits<float>::max();
				float max_x = std::numeric_limits<float>::lowest();
				float max_y = std::numeric_limits<float>::lowest();
				float max_z = std::numeric_limits<float>::lowest();

				for (const auto &point : clustered_[i].points) {
					min_x = std::min(min_x, point.x);
					min_y = std::min(min_y, point.y);
					min_z = std::min(min_z, point.z);
					max_x = std::max(max_x, point.x);
					max_y = std::max(max_y, point.y);
					max_z = std::max(max_z, point.z);
				}

				visualization_msgs::Marker box;
				box.header = latest_cloud_header_;
				box.ns = "obstacle_boxes";
				box.id = static_cast<int>(i);
				box.type = visualization_msgs::Marker::CUBE;
				box.action = visualization_msgs::Marker::ADD;
				box.pose.position.x = (min_x + max_x) * 0.5;
				box.pose.position.y = (min_y + max_y) * 0.5;
				box.pose.position.z = (min_z + max_z) * 0.5;
				box.pose.orientation.w = 1.0;
				box.scale.x = std::max(max_x - min_x, 0.05f);
				box.scale.y = std::max(max_y - min_y, 0.05f);
				box.scale.z = std::max(max_z - min_z, 0.05f);
				box.color.a = 0.65;
				box.color.r = 1.0;
				box.color.g = 1.0;
				box.color.b = 0.0;
				box.lifetime = ros::Duration(0.5);
				boxes.markers.push_back(box);
			}

			visualization_msgs::Marker clear_marker;
			clear_marker.header = latest_cloud_header_;
			clear_marker.ns = "obstacle_boxes";
			clear_marker.action = visualization_msgs::Marker::DELETEALL;
			boxes.markers.insert(boxes.markers.begin(), clear_marker);

			pub_obstacle_boxes_.publish(boxes);
		}

		void LaneChanger::visualize(geometry_msgs::Point input_point) { //static 알고리즘 구현하면서 추출한 내 path rviz 용도
		// visualize waypoint
		visualization_msgs::Marker way_point;
		geometry_msgs::Point point;

		way_point.header.frame_id = "velodyne";
		way_point.header.stamp = ros::Time::now();
		way_point.ns = "points_and_lines";
		way_point.action = visualization_msgs::Marker::ADD;
		way_point.pose.orientation.w = 1.0;
		way_point.id = 1;
		way_point.type = visualization_msgs::Marker::POINTS;
		way_point.scale.x = 0.05; 
		way_point.scale.y = 0.05;
		way_point.color.a = 1.0;
		way_point.color.r = 1.0f;

		way_point.points.push_back(input_point);
	
		pub_marker_.publish(way_point);
    }

int main(int argc, char **argv) {	
    ros::init(argc, argv, "clustering_node");

    LaneChanger lc;
	ros::Rate loop_rate(10);


	while(ros::ok()){		
		ros::spinOnce();
		lc.run();
		loop_rate.sleep();
	}
	return 0;

}
