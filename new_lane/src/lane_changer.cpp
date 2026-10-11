#include "new_lane/lane_changer.h"
#include <std_msgs/Int32.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


void LaneChanger::initSetup() {
	// morai_msgs::CtrlCmd ackermann_msg_;
	
	//publisher
	// ackermann_pub_ = nh_.advertise<morai_msgs::CtrlCmd>("ctrl_cmd", 1);

	pub_points_ = nh_.advertise<sensor_msgs::PointCloud2>("passed_points", 10);
	pub_center_ = nh_.advertise<visualization_msgs::Marker>("center_points", 10);
	pub_center_local = nh_.advertise<visualization_msgs::Marker>("center_points_local", 10);
	pub_marker_ = nh_.advertise<visualization_msgs::Marker>("wayPoint", 10);
	lidar_global_pub = nh_.advertise<geometry_msgs::PoseArray>("/lidar_global",1);
	frenet_path = nh_.advertise<nav_msgs::Path>("/frenet_path", 1);   // Publishes frenet path
	frenet_path_2 = nh_.advertise<nav_msgs::Path>("/frenet_path_2", 1);   // Publishes frenet path
	pub_offset_path1 = nh_.advertise<nav_msgs::Path>("/offset_path1", 1); // right
	pub_offset_path2 = nh_.advertise<nav_msgs::Path>("/offset_path2", 1); // left
	global_path = nh_.advertise<nav_msgs::Path>("/global_path", 1);   // Publishes global path
	pub_lane_ = nh_.advertise<waypoint_maker::State>("/lane_number_msg_", 1);
	pub_frenet_points = nh_.advertise<waypoint_maker::Lane>("frenet_points", 1);
	pub_lidar_topic = nh_.advertise<new_lane::lidar_topic_msg>("/lidar_topic", 1);
	pub_path_number_ = nh_.advertise<std_msgs::Int32>("/path_number", 1);


	//subscriber
	sub_ = nh_.subscribe("/demo/nonground", 1, &LaneChanger::pointCallback, this);
	lane_sub_ = nh_.subscribe("/final_waypoint", 1, &LaneChanger::LaneCallback, this);
	// imu_sub_ = nh_.subscribe("/imu",1,&LaneChanger::ImuCallback,this);
	state_sub_ = nh_.subscribe("/gps_state", 1, &LaneChanger::state_Callback, this);
	odom_sub_ = nh_.subscribe("/odom", 1, &LaneChanger::OdomCallback,this);
	// course_sub_ = nh_.subscribe("course_path", 1, &LaneChanger::CourseCallback,this);
	// closest_index_sub_ = nh_.subscribe("/closest_index", 1, &LaneChanger::closest_index_Callback, this);

	
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

	//load parameters
	ros::param::get("/path/max_speed", MAX_SPEED);
	ros::param::get("/path/max_accel", MAX_ACCEL);
	ros::param::get("/path/max_curvature", MAX_CURVATURE);
	ros::param::get("/path/max_road_width", MAX_ROAD_WIDTH);
	ros::param::get("/path/d_road_w", D_ROAD_W);
	ros::param::get("/path/dt", DT);
	ros::param::get("/path/maxt", MAXT);
	ros::param::get("/path/mint", MINT);
	ros::param::get("/path/target_speed", TARGET_SPEED);
	ros::param::get("/path/d_t_s", D_T_S);
	ros::param::get("/path/n_s_sample", N_S_SAMPLE);
	ros::param::get("/path/robot_radius", ROBOT_RADIUS);
	ros::param::get("/path/max_lat_vel", MAX_LAT_VEL);
	ros::param::get("/path/min_lat_vel", MIN_LAT_VEL);
	ros::param::get("/path/d_d_ns", D_D_NS);
	ros::param::get("/path/max_shift_d", MAX_SHIFT_D);

	ros::param::get("/cost/kj", KJ);
	ros::param::get("/cost/kt", KT);
	ros::param::get("/cost/kd", KD);
	ros::param::get("/cost/kd_v", KD_V);
	ros::param::get("/cost/klon", KLON);
	ros::param::get("/cost/klat", KLAT);
	
	ros::param::get("/frenet/d_offset", D_OFFSET);
	ros::param::get("/frenet/linear_speed", LINEAR_SPEED);
}

void LaneChanger::clustering(const pcl::PointCloud<PointType>::Ptr cloud_in) {
		
		//Voxelization -----------
		pcl::VoxelGrid<pcl::PointXYZ> vox;

		// sensor_msgs
		vox.setInputCloud(cloud_in);
		vox.setLeafSize(0.15, 0.15, 0.15);
		// vox.setLeafSize(voxel_size, voxel_size, voxel_size);
		vox.filter(*cloud_in);

		pcl::PassThrough<pcl::PointXYZ> pass;

		pass.setInputCloud(cloud_in);
		pass.setFilterFieldName("x"); // axis x
		pass.setFilterLimits(x_min,x_max);
		// pass.setFilterLimitsNegative(true);
		pass.setFilterLimitsNegative(false);
		pass.filter(*cloud_in);

		pass.setInputCloud(cloud_in);
		pass.setFilterFieldName("y"); // axis y
		pass.setFilterLimits(y_min, y_max);
		// pass.setFilterLimitsNegative(true);
		pass.setFilterLimitsNegative(false);
		pass.filter(*cloud_in);
		
		pass.setInputCloud(cloud_in);
		pass.setFilterFieldName("z"); // axis z
		pass.setFilterLimits(z_min, z_max);
		// pass.setFilterLimitsNegative(true);
		pass.setFilterLimitsNegative(false);
		pass.filter(*cloud_in);

		sensor_msgs::PointCloud2 filteredOutput;
		pcl::toROSMsg(*cloud_in, filteredOutput);
		filteredOutput.header.frame_id = "map";
		pub_points_.publish(filteredOutput);

		// Populate point cloud...
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
		if(!cloud_in -> empty()) kdtree->setInputCloud(cloud_in);//kdtree에 zCloud를 입력으로 설정하여 넣음

		std::vector<pcl::PointIndices> clusterIndices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

		ec.setClusterTolerance(cluster_tolerance_);
		ec.setMinClusterSize(cluster_minsize_);    // set Minimum Cluster Size
		ec.setMaxClusterSize(cluster_maxsize_); // set Maximum Cluster Size

		ec.setSearchMethod(kdtree);
		ec.setInputCloud(cloud_in);

		ec.extract(clusterIndices);

		clustered_.clear();
		// pcl::PointCloud<pcl::PointXYZ> tmpCloud4;


		int cluster_idx = 0;
		center_.clear();


		for (vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {
			pcl::PointCloud<PointType>::Ptr cluster_cloud (new pcl::PointCloud<PointType>);
			for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
				cluster_cloud->points.emplace_back(cloud_in->points[*pit]);
			}// cloud_in에서 클러스터링한 결과를 cluster_cloud에 할당
			PointXYZV P;
			pcl::PointXYZ min_pt, max_pt;
			pcl::getMinMax3D (*cluster_cloud, min_pt, max_pt);
			P.width = max_pt.x - min_pt.x;
			P.height = max_pt.y - min_pt.y;
			P.z_height = max_pt.z - min_pt.z;
			P.x = (max_pt.x + min_pt.x)/2;
			P.y = (max_pt.y + min_pt.y)/2;
			P.z = (max_pt.z + min_pt.z)/2;
			P.id = -1;
			P.vel = 0;
			P.dist = sqrt(pow(P.x,2)+pow(P.y,2));
			if(P.x > -2.0 && P.x < 3.0 && P.y > -0.85 && P.y < 0.85){} // x:1.4, y:-0.85~).85
			else{
				if(P.z_height > -1.5){	// 0.3
					center_.emplace_back(P);
				}
			}

			//static
			sort(center_.begin(),center_.end(),cmp);
		}
	cout << "center_ size: " << center_.size() << endl;
}


bool LaneChanger::cmp(const PointXYZV &v1, const PointXYZV &v2){
	return v1.dist < v2.dist;
}

void LaneChanger::pointCallback(const sensor_msgs::PointCloud2::ConstPtr &scan) {
		pcl::fromROSMsg(*scan, *msgCloud);
}

void LaneChanger::state_Callback(const waypoint_maker::State::ConstPtr &msg){
	lane_number_ = msg->lane_number;
	current_state_ = msg->current_state;
	// cout<<"lane_number : "<<lane_number_<< endl;
}

// void LaneChanger::closest_index_Callback(const std_msgs::Int32::ConstPtr &msg){
// 	closest_index = msg->data;
// }

// void LaneChanger::CourseCallback(const std_msgs::Float64::ConstPtr &course_msg){
// 	cur_course_rad_ = course_msg -> data;
// }

void LaneChanger::LaneCallback(const waypoint_maker::Lane::ConstPtr &lane_msg)
{
	waypoints_.clear();
	W_X.clear();
	W_Y.clear();
	vector<waypoint_maker::Waypoint>().swap(waypoints_);
	waypoints_ = lane_msg->waypoints;
	waypoints_size_ = waypoints_.size();
	// waypoints_size_ = 100;
	if (waypoints_size_ != 0) is_lane_ = true;
	// cout << "!!!!!!!!!! waypoint loaded !!!!!!!!!!" << endl;
	for(int i=0; i< waypoints_size_; i++){
		W_X.push_back(waypoints_[i].pose.pose.position.x);
		W_Y.push_back(waypoints_[i].pose.pose.position.y);
	}
}


void LaneChanger::OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
	cur_pose_.header = odom_msg->header;
	cur_pose_.pose.position = odom_msg->pose.pose.position;

	tf2::Quaternion q(
		odom_msg->pose.pose.orientation.x,
		odom_msg->pose.pose.orientation.y,
		odom_msg->pose.pose.orientation.z,
		odom_msg->pose.pose.orientation.w
	);
	double roll, pitch, yaw;
	tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
	cur_course_rad_ = yaw;

	// cout << "position X: " << cur_pose_.pose.position.x << endl;
	// cout << "position Y: " << cur_pose_.pose.position.y << endl;
}


float LaneChanger::calcul_angle(float x, float y) {
	float ang;
	ang = atan2f(y, x)*180/M_PI;//from radian to degree
	return ang;
}


void LaneChanger::CircleDecision1(vecD x1, vecD y1, vecD x2, vecD y2,float r){	// my line
	is_obs1 = false;
	// cout << "CircleDecision1" << endl;

	if(x1.size() == 0 || x2.size() == 0){
		is_obs1 = false;
		// cout << "11111"<<endl;
		lane_cnt1 = 0;
	}
	

	else if(x1.size() != 0 && x2.size() != 0){
		for(int j=0; j<x1.size(); j++){
			for(int i=0; i<x2.size(); i++){
				is_circle1 = sqrt(pow(x1[j]-x2[i], 2) + pow(y1[j]-y2[i], 2));
				if(is_circle1 < r) {
					cout << "!!!!!!!!! my line obstacle !!!!!!!!" << endl;
					is_obs1 = true;
					obs_index1 = j;
					lane_cnt1 ++;
					break;
				}
				else if(is_circle1 > r){
					is_obs1 = false;
				}
			}
			if(is_obs1) break;
		}
	}
	if(is_obs1){
		my_line_obs = true;
		cout << "내 장애물 확인" << endl;
	}
	else{
		my_line_obs = false;
		lane_cnt1 = 0;
	}
}

void LaneChanger::CircleDecision2(vecD x1, vecD y1, vecD x2, vecD y2,float r){	// 오른쪽 장애물
	is_obs2 = false;

	if(x1.size() == 0 || x2.size() == 0) {
		is_obs2 = false;
		lane_cnt2 = 0;
	}

	else if(x1.size() != 0){
		for(int j=0; j<x1.size(); j++){
			for(int i=0; i<x2.size(); i++){
				is_circle2 = sqrt(pow(x1[j]-x2[i], 2) + pow(y1[j]-y2[i], 2));
				if(is_circle2 < r) {
					cout << "!!!!!!!!! other obstacle !!!!!!!!" << endl;
					is_obs2 = true;
					obs_index2 = j;
					lane_cnt2 ++;
					break;
				}
				else if(is_circle2 > r){
					is_obs2 = false;

				}
			}
			if(is_obs2) break;
		}
	}
	if(is_obs2){
		other_line_obs = true;
		cout << "다른장애물 확인" << endl;
	}
	else{
		other_line_obs = false;
		lane_cnt2 = 0;
	}
}

bool LaneChanger::CircleDecision1_c(vecD x1, vecD y1, vecD x2, vecD y2,float r){	// 오른쪽 장애물
	is_obs1_c = false;

	if(x1.size() == 0 || x2.size() == 0) {
		// cout << "11111"<<endl;
		is_obs1_c = false;
		// lane_cnt1_c = 0;
	}

	else if(x1.size() != 0){
		// cout << "111111"<<endl;
		for(int j=0; j<x1.size(); j++){
			for(int i=0; i<x2.size(); i++){
				is_circle2 = sqrt(pow(x1[j]-x2[i], 2) + pow(y1[j]-y2[i], 2));
				if(is_circle2 < r) {
					is_obs1_c = true;
					obs_index1_c = j;
					// lane_cnt1_c ++;
					break;
				}
				else if(is_circle2 > r){
					is_obs1_c = false;

				}
			}
			if(is_obs1_c) break;
		}
	}
	return is_obs1_c;
}

bool LaneChanger::CircleDecision2_c(vecD x1, vecD y1, vecD x2, vecD y2,float r){	// 오른쪽 장애물
	is_obs2_c = false;

	if(x1.size() == 0 || x2.size() == 0) {
		is_obs2_c = false;
		// lane_cnt2_c = 0;
	}

	else if(x1.size() != 0){
		for(int j=0; j<x1.size(); j++){
			for(int i=0; i<x2.size(); i++){
				is_circle2 = sqrt(pow(x1[j]-x2[i], 2) + pow(y1[j]-y2[i], 2));
				if(is_circle2 < r) {
					is_obs2_c = true;
					// obs_index2_c = j;
					// lane_cnt2_c ++;
					break;
				}
				else if(is_circle2 > r){
					is_obs2_c = false;

				}
			}
			if(is_obs2_c) break;
		}
	}
	return is_obs2_c;
}

// void LaneChanger::LaneChage_Decision(bool my_obs, bool other_obs){	// lane_num 0 : 중앙, lane_num 1 : 오른쪽, lane_num 2 : 왼쪽
// 	// 중앙
// 	if(lane_num == 0){
// 		if(my_obs){
// 			if(other_obs){
// 				lane_num = lane_number_ ;
// 				acc_index1 = find_closest_waypoint(global_obs, obs_index1, waypoints_);
// 				cout << "acc_index1 : " << acc_index1 << endl;
// 				acc_flag = true;
// 			}
// 			else{
// 				if(lane_cnt1 > 1){
// 					lane_cnt1 = 0;
// 					lane_cnt2 = 0;

// 					lane_num = 1;
// 					acc_flag = false;	
// 				}
// 				else {
// 					lane_num = 0;
// 				}	
// 			}		
// 		}
// 		else {
// 			lane_num = 0;
// 			acc_flag = false;

// 		}
// 	}
// 	else if (lane_num == 1){
// 		if(my_obs){
// 			if(other_obs){
// 				lane_num = lane_number_ ;
// 				acc_index1 = find_closest_waypoint(global_obs, obs_index1, waypoints_);
// 				cout << "acc_index1 : " << acc_index1 << endl;
// 				acc_flag = true;
// 			}
// 			else{
// 				if(lane_cnt1 > 1){
// 					lane_cnt1 = 0;
// 					lane_cnt2 = 0;

// 					lane_num = 0;
// 					acc_flag = false;
// 				}
// 				else {
// 					lane_num = 1;
// 				}	
// 			}		
// 		}
// 		else {
// 			lane_num = 1;
// 			acc_flag = false;


// 		}

// 	}
// 	// cout << "lane_num : " << lane_num << endl;
// 	lidar_topic_msg.lane_num = lane_num;
// 	lidar_topic_msg.ACC_speed = 0; // no using
// 	lidar_topic_msg.ACC_flag = acc_flag;
// 	lidar_topic_msg.ACC_index_gap = acc_index1;
// 	pub_lidar_topic.publish(lidar_topic_msg);

// 	std_msgs::Int32 path_msg;
// 	path_msg.data = lane_num;
// 	pub_path_number_.publish(path_msg);


// }

// void LaneChanger::LaneChage_Decision(bool my_obs, bool other_obs){
// 	auto acc_false_debounce = [&]() {
// 		if (acc_flag) {
// 			acc_false_cnt_++;
// 			if (acc_false_cnt_ >= acc_false_debounce_count_) {
// 				acc_flag = false;
// 				acc_false_cnt_ = 0;
// 			}
// 		} else {
// 			acc_false_cnt_ = 0;
// 		}
// 	};

// 	if(lane_num == 0){
// 		if(my_obs){
// 			if(other_obs){
// 				lane_num = lane_number_;
// 				acc_index1 = find_closest_waypoint(global_obs, obs_index1, waypoints_);
// 				cout << "acc_index1 : " << acc_index1 << endl;
// 				acc_flag = true;
// 				acc_false_cnt_ = 0;
// 			}
// 			else{
// 				if(lane_cnt1 > 1){
// 					lane_num = 1;
// 					lane_cnt1 = 0;
// 					lane_cnt2 = 0;
// 					// lane_num = 1;
// 					acc_false_debounce();
// 				}
// 				else {
// 					lane_num = 0;
// 				}
// 			}
// 		}
// 		else {
// 			lane_num = 0;
// 			acc_false_debounce();
// 		}
// 	}
// 	else if (lane_num == 1){
// 		if(my_obs){
// 			if(other_obs){
// 				lane_num = lane_number_;
// 				acc_index1 = find_closest_waypoint(global_obs, obs_index1, waypoints_);
// 				cout << "acc_index1 : " << acc_index1 << endl;
// 				acc_flag = true;
// 				acc_false_cnt_ = 0;
// 			}
// 			else{
// 				if(lane_cnt1 > 1){
// 					lane_num = 0;
// 					lane_cnt1 = 0;
// 					lane_cnt2 = 0;
// 					// lane_num = 0;
// 					acc_false_debounce();
// 				}
// 				else {
// 					lane_num = 1;
// 				}
// 			}
// 		}
// 		else {
// 			lane_num = 1;
// 			acc_false_debounce();
// 		}
// 	}

// 	lidar_topic_msg.lane_num = lane_num;
// 	lidar_topic_msg.ACC_speed = 0;
// 	lidar_topic_msg.ACC_flag = acc_flag;
// 	lidar_topic_msg.ACC_index_gap = acc_index1;
// 	pub_lidar_topic.publish(lidar_topic_msg);

// 	std_msgs::Int32 path_msg;
// 	path_msg.data = lane_num;
// 	pub_path_number_.publish(path_msg);
// }


void LaneChanger::LaneChage_Decision(bool my_obs, bool other_obs){
	const int lane_before = lane_num;

	auto acc_false_debounce = [&]() {
		if (acc_flag) {
			acc_false_cnt_++;
			if (acc_false_cnt_ >= acc_false_debounce_count_) {
				acc_flag = false;
				acc_false_cnt_ = 0;
			}
		} else {
			acc_false_cnt_ = 0;
		}
	};

	if(lane_num == 0){
		if(my_obs){

			if(other_obs){
				lane_num = lane_number_;
				acc_index1 = find_closest_waypoint(global_obs, obs_index1, waypoints_);
				cout << "acc_index1 : " << acc_index1 << endl;
				acc_flag = true;
				acc_false_cnt_ = 0;
			}

			else{
				if(lane_cnt1 > 1){
					lane_num = 1;
					lane_cnt1 = 0;
					lane_cnt2 = 0;
					// lane_num = 1;
					acc_false_debounce();
				}
				else {
					lane_num = 0;
					acc_false_debounce();

				}
			}
		}
		else {
			lane_num = 0;
			acc_false_debounce();
		}
	}
	else if (lane_num == 1){
		if(my_obs){
			if(other_obs){
				lane_num = lane_number_;
				acc_index1 = find_closest_waypoint(global_obs, obs_index1, waypoints_);
				cout << "acc_index1 : " << acc_index1 << endl;
				acc_flag = true;
				acc_false_cnt_ = 0;
			}
			else{
				if(lane_cnt1 > 1){
					lane_num = 0;
					lane_cnt1 = 0;
					lane_cnt2 = 0;
					// lane_num = 0;
					acc_false_debounce();
				}
				else {
					lane_num = 1;
					acc_false_debounce();
				}
			}
		}
		else {
			lane_num = 1;
			acc_false_debounce();
		}
	}

	// ACC가 true인 동안은 차선 변경 금지: 이번 프레임 시작 차선 유지
	if (acc_flag) {
		lane_num = lane_before;
	}

	lidar_topic_msg.lane_num = lane_num;
	lidar_topic_msg.ACC_speed = 0;
	lidar_topic_msg.ACC_flag = acc_flag;
	lidar_topic_msg.ACC_index_gap = acc_index1;
	pub_lidar_topic.publish(lidar_topic_msg);

	std_msgs::Int32 path_msg;
	path_msg.data = lane_num;
	pub_path_number_.publish(path_msg);
}




void LaneChanger::only_ACC(bool my_obs){	// lane_num 0 : 오른쪽, lane_num 2 : 왼쪽
	// 중앙
	if(my_obs){
		if(lane_cnt1 > 2){
			lane_cnt1 = 0;
			
			// cout << "----ACC----" << endl;
			lane_num = lane_number_ ;
			acc_index1 = find_closest_waypoint(global_obs, obs_index1, waypoints_);
			cout << "acc_index1 : " << acc_index1 << endl;
			acc_flag = true;
		}			
	}
	else {
		acc_flag = false;
		lane_cnt1 = 0;
	}
	// cout << "lane_num : " << lane_num << endl;
	lidar_topic_msg.lane_num = lane_num;
	lidar_topic_msg.ACC_speed = 0; // no using
	lidar_topic_msg.ACC_flag = acc_flag;
	lidar_topic_msg.ACC_index_gap = acc_index1;
	pub_lidar_topic.publish(lidar_topic_msg);

}


void LaneChanger::only_ACC_2(bool my_obs){	// lane_num 0 : 오른쪽, lane_num 2 : 왼쪽
	lane_cnt1 = 0;
	lane_cnt2 = 0;
	if(my_obs){
		// cout << "----ACC----" << endl;
		lane_num = lane_number_ ;
		acc_index1_c = find_closest_waypoint(global_obs, obs_index1_c, waypoints_);
		cout << "acc_index1_c : " << acc_index1_c << endl;
		acc_flag = true;
	}
	else {
		acc_flag = false;
	}
	// cout << "lane_num : " << lane_num << endl;
	lidar_topic_msg.lane_num = lane_num;
	lidar_topic_msg.ACC_speed = 0; // no using
	lidar_topic_msg.ACC_flag = acc_flag;
	lidar_topic_msg.ACC_index_gap = acc_index1_c;
	pub_lidar_topic.publish(lidar_topic_msg);

}

void LaneChanger::Local_to_Global(vector<PointXYZV> input_points){
	global_obs.clear();
	double yaw = cur_course_rad_;
	pcl::PointXYZ obs;
	if(input_points.size() != 0){
		for(int i=0; i<input_points.size(); i++){
			geometry_msgs::Pose pose_global;
		
			pose_global.position.x = cur_pose_.pose.position.x +  ((input_points[i].x+1.04) * cos(yaw)) - ((input_points[i].y) * sin(yaw));
			pose_global.position.y = cur_pose_.pose.position.y +  ((input_points[i].x+1.04) * sin(yaw)) + ((input_points[i].y) * cos(yaw));
			pose_global.position.z = 0;
			
			global_x.emplace_back(pose_global.position.x);
			global_y.emplace_back(pose_global.position.y);
			global_z.emplace_back(pose_global.position.z);

			obs.x = pose_global.position.x;
			obs.y = pose_global.position.y;
			obs.z = pose_global.position.z;

			global_obs.emplace_back(obs);
			cur_pose_global.poses.emplace_back(pose_global);
		}
	}
	// cout << "global obs size : " << global_obs.size() << endl;
    lidar_global_pub.publish(cur_pose_global);
}


int LaneChanger::find_closest_waypoint(vector<pcl::PointXYZ> obs_points, int index, vector<waypoint_maker::Waypoint> path){
	// cout << "---find_closest_waypoint()---" << endl;
	float between_dist = 0.0;
	int tmp_index = 0;
	// int now_index = closest_index;	// closest_wapoint의 index 받아오기
	vector<float> all_dist;
	if(obs_points.size() != 0 && path.size() != 0){
		all_dist.clear();

		for(int i=0; i<path.size(); i++){
			between_dist = sqrt(pow(obs_points[index].x-path[i].pose.pose.position.x, 2) + pow(obs_points[index].y-path[i].pose.pose.position.y, 2));
			all_dist.push_back(between_dist);

		}
		float min_dist = 9999.9;
		cout << "all_dist size : " << all_dist.size() << endl;
		for(int j=0; j < all_dist.size(); j++){
			if(min_dist > all_dist[j]){
				min_dist = all_dist[j];
				tmp_index = j;
			}
		}
		cout << "tmp_index : " << tmp_index << endl;
	}
	else{
		all_dist.clear();
	}
	return tmp_index;
}


// 법선 벡터 계산
geometry_msgs::Point LaneChanger::calculateNormal(const waypoint_maker::Waypoint& p1, const waypoint_maker::Waypoint& p2) {
    geometry_msgs::Point normal;
    normal.x = p2.pose.pose.position.y - p1.pose.pose.position.y;
    normal.y = -(p2.pose.pose.position.x - p1.pose.pose.position.x);
    double length = sqrt(pow(p1.pose.pose.position.x - p2.pose.pose.position.x, 2) + pow(p1.pose.pose.position.y - p2.pose.pose.position.y, 2));
	if(length == 0) length = 1;
    normal.x /= length;
    normal.y /= length;	//	단위 길이로
    return normal;
}

// 다각형 오프셋 함수
void LaneChanger::offsetPath(const vector<waypoint_maker::Waypoint>& before_path, double offset) {	// lane_num 0 = right, lane_num 1 = left
	offset_path1_x.clear();
	offset_path1_y.clear();
	offset_path1.clear();

	if(before_path.size() != 0){
		if(lane_number_ == 0){		// 왼쪽으로 밀어줌
			real_D_OFFSET = offset;
			cout << "real_D_OFFSET : " << real_D_OFFSET << endl;
				int n = before_path.size();
				for (int i = 0; i < n-1; i++) {
					const waypoint_maker::Waypoint& p1 = before_path[i];
					const waypoint_maker::Waypoint& p2 = before_path[(i + 1)];
					geometry_msgs::Point normal = calculateNormal(p1, p2);

					waypoint_maker::Waypoint offsetPoint1;
					offsetPoint1.pose.pose.position.x = p1.pose.pose.position.x + normal.x * real_D_OFFSET;
					offsetPoint1.pose.pose.position.y = p1.pose.pose.position.y + normal.y * real_D_OFFSET;

					offset_path1_x.push_back(offsetPoint1.pose.pose.position.x);
					offset_path1_y.push_back(offsetPoint1.pose.pose.position.y);

					offset_path1.push_back(offsetPoint1);
				}
				nav_msgs::Path offset_path_msg1;

				offset_path_msg1.header.frame_id = "map";
				offset_path_msg1.poses.resize(offset_path1.size());

				for (int i = 0; i < offset_path1.size(); i++){
					geometry_msgs::PoseStamped off;
					off.pose.position.x = offset_path1[i].pose.pose.position.x;
					off.pose.position.y = offset_path1[i].pose.pose.position.y;
					offset_path_msg1.poses[i] = off;
				}
				pub_offset_path1.publish(offset_path_msg1);
			}

		else if(lane_number_ == 1){
			real_D_OFFSET = -1*offset;
			cout << "real_D_OFFSET : " << real_D_OFFSET << endl;
			int n = before_path.size();
			for (int i = 0; i < n-1; i++) {	// 오른쪽으로 밀어줌
				const waypoint_maker::Waypoint& p1 = before_path[i];
				const waypoint_maker::Waypoint& p2 = before_path[(i + 1)];
				geometry_msgs::Point normal = calculateNormal(p1, p2);

				waypoint_maker::Waypoint offsetPoint1;
				offsetPoint1.pose.pose.position.x = p1.pose.pose.position.x + normal.x * real_D_OFFSET;
				offsetPoint1.pose.pose.position.y = p1.pose.pose.position.y + normal.y * real_D_OFFSET;

				offset_path1_x.push_back(offsetPoint1.pose.pose.position.x);
				offset_path1_y.push_back(offsetPoint1.pose.pose.position.y);

				offset_path1.push_back(offsetPoint1);
			}
			nav_msgs::Path offset_path_msg1;

			offset_path_msg1.header.frame_id = "map";
			offset_path_msg1.poses.resize(offset_path1.size());

			for (int i = 0; i < offset_path1.size(); i++){
				geometry_msgs::PoseStamped off;
				off.pose.position.x = offset_path1[i].pose.pose.position.x;
				off.pose.position.y = offset_path1[i].pose.pose.position.y;
				offset_path_msg1.poses[i] = off;
			}
			pub_offset_path1.publish(offset_path_msg1);
		}

		//reference path visualization
		nav_msgs::Path global_path_msg;
		
		// Global path pushed into the message
		if(W_X.size() != 0){
			global_path_msg.header.frame_id = "map";
			global_path_msg.poses.resize(W_X.size());
			for (int i = 0; i < W_X.size(); i++)
			{
				geometry_msgs::PoseStamped loc;
				loc.pose.position.x = W_X[i];
				loc.pose.position.y = W_Y[i];
				global_path_msg.poses[i] = loc;
			}
			global_path.publish(global_path_msg);
		}
	}
}

void LaneChanger::run(){
	
	clustering(msgCloud);
	Local_to_Global(center_);
	cout << "lidar_lane_num : " << lane_num << endl;
	cout << "gps_lane_number_ : " << lane_number_ << endl;
	cout << "current_state_ : " << current_state_ << endl;

	
	if(current_state_ == 1 || current_state_ == 2 || current_state_ == 3 || current_state_ == 4 || current_state_ == 5 || current_state_ == 6 || current_state_ == 8 || current_state_ == 9 || current_state_ == 10){
		// 교차로 상황에 사용
		offsetPath(waypoints_, D_OFFSET);
		CircleDecision1(global_x, global_y, W_X, W_Y, 1.8);
		only_ACC(my_line_obs);
		
	}
	else {
		// 일반 주행에 사용
		if(lane_num == lane_number_){
			double new_offset = 0.0;
			double r_dist = 0.0;
			if(current_state_ == 0 || current_state_ == 11){
				new_offset = 3.0;
				r_dist = 1.8;
			}
			else if(current_state_ == 7){
				new_offset = 2.7;
				r_dist = 1.15;
			}
			else{
				new_offset = 2.55;
				r_dist = 1.1;
			}

			offsetPath(waypoints_, new_offset);
			if(lane_number_ == 0){
						
				CircleDecision1(global_x, global_y, W_X, W_Y, r_dist);	// 1.45
				CircleDecision2(global_x, global_y, offset_path1_x, offset_path1_y,r_dist);
				
				
				// CircleDecision1_c(global_x, global_y, W_X, W_Y, 1.9);
				// CircleDecision2_c(global_x, global_y, offset_path1_x, offset_path1_y,1.9);

				// if(is_obs1_c && is_obs2_c){
				// 	only_ACC_2(true
				// 	cout << "!!!center_ACC!!!" << endl;
				// }
				// else{
				LaneChage_Decision(my_line_obs, other_line_obs);
				// }
			}
			else if(lane_number_ == 1){
				
				CircleDecision1(global_x, global_y, W_X, W_Y, r_dist);
				CircleDecision2(global_x, global_y, offset_path1_x, offset_path1_y,r_dist);
				
				
				// CircleDecision1_c(global_x, global_y, W_X, W_Y, 1.9);
				// CircleDecision2_c(global_x, global_y, offset_path1_x, offset_path1_y,1.9);

				// if(is_obs1_c && is_obs2_c){
				// 	only_ACC_2(true);
				// 	cout << "!!!center_ACC!!!" << endl;
				// }
				// else{
				LaneChage_Decision(my_line_obs, other_line_obs);
				// }			
			}
		}
	}
	visualize_center(global_obs);
	visualize_center_local(center_);
	global_x.clear();
	global_y.clear();
	
}


void LaneChanger::visualize_center(vector<pcl::PointXYZ> input_points) {
		visualization_msgs::Marker centers;
		geometry_msgs::Point point;

		centers.header.frame_id = "map";
		centers.header.stamp = ros::Time::now();
		centers.ns = "centers";
		centers.action = visualization_msgs::Marker::ADD;
		centers.pose.orientation.w = 1.0;
		centers.id = 2;
		centers.type = visualization_msgs::Marker::POINTS;
		centers.scale.x = 0.8; 
		centers.scale.y = 0.8;
		centers.color.a = 1.0;
		centers.color.g = 1.0f;
		centers.color.r = 1.0f;

		for (int i = 0; i < input_points.size(); i++) {
			point.x = input_points[i].x;
			point.y = input_points[i].y;
			point.z = 0;

			centers.points.push_back(point);
		}
	    pub_center_.publish(centers);
}

void LaneChanger::visualize_center_local(vector<PointXYZV> input_points) {
		visualization_msgs::Marker centers;
		geometry_msgs::Point point;

		centers.header.frame_id = "map";
		centers.header.stamp = ros::Time::now();
		centers.ns = "centers_l";
		centers.action = visualization_msgs::Marker::ADD;
		centers.pose.orientation.w = 1.0;
		centers.id = 1;
		centers.type = visualization_msgs::Marker::POINTS;
		centers.scale.x = 0.3; 
		centers.scale.y = 0.3;
		centers.color.a = 1.0;
		centers.color.g = 1.0f;
		centers.color.r = 1.0f;

		for (int i = 0; i < input_points.size(); i++) {
			point.x = input_points[i].x;
			point.y = input_points[i].y;
			point.z = input_points[i].z;

			centers.points.push_back(point);
		}
	    pub_center_local.publish(centers);
}

void LaneChanger::visualize(geometry_msgs::Point input_point) {
		// visualize waypoint
		visualization_msgs::Marker way_point;
		geometry_msgs::Point point;

		way_point.header.frame_id = "map";
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
    ros::init(argc, argv, "lane_changer_node");

    LaneChanger lc;
	ros::Rate loop_rate(10);


	while(ros::ok()){		
		ros::spinOnce();
		lc.run();
		loop_rate.sleep();
	}
	return 0;

}
