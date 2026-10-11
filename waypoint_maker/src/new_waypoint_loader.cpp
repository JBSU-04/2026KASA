#include <ros/ros.h>

//msgs
#include <geometry_msgs/PoseStamped.h> 
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <waypoint_maker/Lane.h> //For Parking Mission
#include <waypoint_maker/Waypoint.h> // Waypoints
#include <waypoint_maker/State.h> //For Mission (미션에 필요한 msg 헤더 모음)
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <algorithm>
#include <numeric> 


#include <vector>
#include <string>


// For CSV file loading
#include <fstream>
#include <sys/types.h> // 시스템 자료형 타입 정리용 헤더
#include <dirent.h> // 디렉토리 다루기 위한 헤더


// header for tf
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <algorithm>
#include <ackermann_msgs/AckermannDriveStamped.h>


//<<====================================>>//


using namespace std;

/*
	waypoint loader algorithm : 

	1. initsetup()
		1-1 get_csvs_fromDirectory() : save all csv file's name in vector.
		1-2 get_new_waypooints() : open CSV files and save waypoints in vector.
	2. subscribe the Current_pose and the Lane_number
	3. poseprocess()
		3-1 getClosesttWaypoint() : find the closest waypoint 
		3-2 getFinalWaypoint() : decide 50 waypoints to publish.
		3-3 mission_state_ 설정
		3-4 state & waypoint publish
	4. publish waypoints

*/

class WaypointLoader {
private:
	int state_inspection_;	
	int current_mission_state_;


//ROS
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	ofstream os_;

	// publisher	
	ros::Publisher waypoint_pub_;
	ros::Publisher state_pub_;
	ros::Publisher rviz_global_path;
	ros::Publisher rviz_currnet_path_;

	
	// subscriber
	ros::Subscriber pose_sub_;
	ros::Subscriber lane_number_sub_;
	ros::Subscriber imu_sub_;
	ros::Subscriber course_sub_;

	tf::TransformBroadcaster tf_odom_;
	tf::TransformBroadcaster map2imu;
    tf::Quaternion quar_ego;
	tf::TransformListener listener;
	tf::StampedTransform imu_transform;

	//msg
	geometry_msgs::PoseStamped current_pose;
	waypoint_maker::Lane lane_msg_;
	waypoint_maker::State state_msg_;
	nav_msgs::Path past_path_rviz_;
	vector<geometry_msgs::PoseStamped> past_path_;

	sensor_msgs::Imu prev_imu;
    ros::Time prev_time;

	std_msgs::Float64 course_msg_;


//Functions
	//get_csvs_fromDirectory() & getNewWaypoints()
	ifstream is_;
	std::vector<std::string> all_csv_;
	vector< vector<waypoint_maker::Waypoint> > all_new_waypoints_;
	vector<waypoint_maker::Waypoint> new_waypoints_;
	vector<int> lane_size_;  

	vector< vector<int> > all_state_index_;

	//getClosestWaypoint()
	vector<int> closest_waypoint_candidates;
	double max_search_dist_;
	double min_search_dist_;
	double dist_;
	int closest_waypoint_;
	int ex_closest_waypoint_;
	
	//getFinalWaypoint()
	int size_;
	int final_size_;
	vector<waypoint_maker::Waypoint> final_waypoints_;

	//spline
	vector<waypoint_maker::Waypoint> ryaw;
	vector<waypoint_maker::Waypoint> rk;


	const char* PATH_CSV_PATH = "/home/kuuve/catkin_ws/src/data/";

//Mission
	//Parking mission
	int lane_number_;
	int ex_lane_number_;
	bool is_onlane_;
	bool is_state_;
	bool not_first;


	//parameters for Parking mission
	const double WHEEL_BASE = 1.04;
	
	double cur_course_;
	double cur_speed_; 
	bool is_course_;
	bool is_course_gps_;
	vector<double> heading_vec;

	string imu_frame;
	bool heading_init_;
	double initial_yaw_;
	double gps_yaw_;

//IMU test
	ros::Publisher imu_test_flag_pub_;
	ros::Publisher imu_test_path_pub_;
	std_msgs::Bool imu_test_flag_msgs_;
	float imu_test_dist_;
	float imu_test_ex_ryaw_;
	float imu_test_cur_ryaw_;
	bool imu_test_range_flag_;
	vector<float> imu_test_vec_; 
	float imu_test_all_vec_;
	float imu_test_avg_;

	// CTE Calculation
	double cte_test_dist_;
	vector<double> cte_test_vec_; 
	double cte_test_all_vec_;
	double cte_test_avg_;
	bool avg_cal_flag_;

	// EPSI Calculation
	double epsi_test_yaw_;
	vector<double> epsi_test_vec_; 
	double epsi_test_all_vec_;
	double epsi_test_avg_;

	double yaw_accel;
	vector<double> yaw_accel_test_vec_; 
	double yaw_accel_test_all_vec_;
	double yaw_accel_test_avg_;

	double roll_accel;
	vector<double> roll_accel_test_vec_; 
	double roll_accel_test_all_vec_;
	double roll_accel_test_avg_;

	double pitch_accel;
	vector<double> pitch_accel_test_vec_; 
	double pitch_accel_test_all_vec_;
	double pitch_accel_test_avg_;

	double yaw_jerk;
	vector<double> yaw_jerk_test_vec_; 
	double yaw_jerk_test_all_vec_;
	double yaw_jerk_test_avg_;

	double roll_jerk;
	vector<double> roll_jerk_test_vec_; 
	double roll_jerk_test_all_vec_;
	double roll_jerk_test_avg_;

	double pitch_jerk;
	vector<double> pitch_jerk_test_vec_; 
	double pitch_jerk_test_all_vec_;
	double pitch_jerk_test_avg_;

	int count_;

public:	
	WaypointLoader() {
		initSetup();
		ROS_INFO("WAYPOINT LOADER INITIALIZED.");
	}
	
	~WaypointLoader() {
		os_.close();
		ROS_INFO("WAYPOINT LOADER TERMINATED.");
	}
	
	void initSetup() {
		private_nh_.getParam("/waypoint_loader_node/state_inspection", state_inspection_);

		// publisher
		waypoint_pub_ = nh_.advertise<waypoint_maker::Lane>("final_waypoints", 1);
		state_pub_ = nh_.advertise<waypoint_maker::State>("gps_state",1);

		// rviz
		rviz_global_path = nh_.advertise<nav_msgs::Path>("/rviz_global_path", 1);
		rviz_currnet_path_ = nh_.advertise<nav_msgs::Path>("/rviz_currnet_path_", 1);

		// subscriber
		pose_sub_ = nh_.subscribe("odom_back", 10, &WaypointLoader::poseCallback, this);
		// imu_sub_ = nh_.subscribe("imu/data",1,&WaypointLoader::ImuCallback,this);
		lane_number_sub_ = nh_.subscribe("lane_number_msg_", 1, &WaypointLoader::lane_number_Callback, this);

		course_sub_ = nh_.subscribe("course_path", 1, &WaypointLoader::CourseCallback, this);

		os_.open("/home/kuuve/catkin_ws/src/test_data/test_data.csv", ios::app);
		os_<<setprecision(numeric_limits<double>::digits10+2);

		final_size_ = 400;
		size_ = 0;
		//getClosestWaypoint()
		max_search_dist_ = 10.0;
		min_search_dist_ = 0.5;
		current_mission_state_ = state_inspection_; 

		closest_waypoint_ = 0;
		lane_number_ = 0;
		ex_lane_number_ = 0;
		is_onlane_ = false;
		is_state_ = false;

		cur_course_ = .0;
		cur_speed_ = .0;
		is_course_ = false;
		initial_yaw_ = 0.0;
		gps_yaw_ = 0.0;


		count_ = 0;

		get_csvs_fromDirectory();
		getNewWaypoints();
		loader_rviz();
	}
	
//Functions in initSetup
	void get_csvs_fromDirectory() // /home/kuuve/data/ 안에 있는 여러개의 csv파일 이름을 벡터에 저장
	{
	
		DIR* dirp = opendir(PATH_CSV_PATH); //디렉토리 저장

		if (dirp == NULL){
			perror("UNABLE TO OPEN FOLDER");
			return;
		}

		struct dirent* dp; 
		while((dp = readdir(dirp)) != NULL){ //디렉토리 안의 파일 이름 저장
			string address(PATH_CSV_PATH);

			string filename (dp->d_name);

			address.append(filename);
			if (filename.size() > 2){ //디렉토리 open시 ..과 .를 막기 위한 방어코드
				all_csv_.emplace_back(address);
				ROS_INFO("SAVED CSV");
			}
		}
		sort(all_csv_.begin(),all_csv_.end()); //정렬
		closedir(dirp);
	}

	void getNewWaypoints() { // 가져온 여러개의 CSV내 데이터를 벡터에 저장
	/*
		정리하고자 하는 벡터
		all_new_waypoints_ : Waypoint 모음 벡터
		all_state_index_ : mission state 모음 벡터
		lane_size_ : Vector 길이 모음 벡터
	*/

		string str_buf;
		int pos;
		vector<int> state_index_;
		vector<waypoint_maker::Waypoint> temp_new_waypoints;
		waypoint_maker::Waypoint temp_waypoint;

		for(auto i = 0; i < all_csv_.size(); i++){ //파일 갯수만큼

			state_index_.emplace_back(5); // current_mission_state 0번 시작 : 반드시 5 
										  // 이유 : getClosestWaypoint()의 조사 시작 범위를 위해
										  // ctrl+f --> void getClosestWaypoint()
						
			is_.open(all_csv_[i]); //파일 열기

			cout << "OPEN CSV" << all_csv_[i] << endl;
			
			temp_waypoint.mission_state = 0;
			// 이 부분 내용 알아보기
			while(getline(is_, str_buf)) {//파일 내용을 str_buf에 저장

				if(str_buf != "") { // temp_waypoint = [index, x, y, ryaw, rk, mission_state]
				//index
					pos = str_buf.find(",");
					temp_waypoint.waypoint_index = stoi(str_buf.substr(0, pos));

					//x
					str_buf = str_buf.substr(++pos);
					pos = str_buf.find(","); 
					temp_waypoint.pose.pose.position.x = stod(str_buf.substr(0, pos));

					//y
					str_buf = str_buf.substr(++pos);
					pos = str_buf.find(",");
					temp_waypoint.pose.pose.position.y = stod(str_buf.substr(0, pos));

					//mission state
					str_buf = str_buf.substr(++pos);
					pos = str_buf.find(",");
					if(temp_waypoint.mission_state != stoi(str_buf.substr(0, pos)))
					{
						state_index_.emplace_back(temp_waypoint.waypoint_index);

					} //mission state 변화시 따로 저장
					temp_waypoint.mission_state = stoi(str_buf.substr(0, pos));
				
					//ryaw
					str_buf = str_buf.substr(++pos);
					pos = str_buf.find(",");
					temp_waypoint.ryaw = stod(str_buf.substr(0, pos));
					
					//rk
					str_buf = str_buf.substr(++pos);
					pos = str_buf.find(",");
					temp_waypoint.rk = stod(str_buf.substr(0, pos));

					
					temp_new_waypoints.emplace_back(temp_waypoint);

				}
			}
			is_.close();
			
			size_ = temp_new_waypoints.size();
			lane_size_.emplace_back(size_); //lane_size 정리
				
			ROS_INFO("%d WAYPOINTS HAVE BEEN SAVED.", size_);

			all_new_waypoints_.emplace_back(temp_new_waypoints); //all_new_waypoints_ 정리
			all_state_index_.emplace_back(state_index_);//all_state_index_ 정리

			temp_new_waypoints.clear();
			state_index_.clear();
		}

		//0번 행 : 전체 경로
		//1번 행 ~  : 주차 경로
		new_waypoints_.assign(all_new_waypoints_[lane_number_].begin(), all_new_waypoints_[lane_number_].end());
		size_ = lane_size_[lane_number_];		
	}


	void poseProcess(){
		final_waypoints_.clear();
		
		getClosestWaypoint(current_pose);//내 위치 기준 closest waypoint 찾기
		getFinalWaypoint();//최종 50개의 waypoint 결정
		
		/*
			parking_state_(default : -3)

			-2 부터 주차 시작, 주차시 mission_state는 parking_state_에 의해 결정
			mission_state_ = parking_state_ + 18 이 되도록 설정

			parking_state_: -3 ~ 1			mission_state_
			-3 : 평시 주행							15
			-2 : 주차 판단			    			16
			-1 : 각도 전진			    			17
			 0 : 후진 주차					  		18
			 1 : 복귀 전진 					 		19
		*/

		current_mission_state_ = final_waypoints_[lane_number_].mission_state;
		
		if(!is_state_ && closest_waypoint_candidates.empty()){
			ROS_INFO("closest_waypoint_candidates are not Founded.");
		    current_mission_state_ = state_inspection_;
		}//중간에서 실행시 current_mission_state_ 수신하지 못하는 오류 방어 코드 -> 원인 아직 파악 못함 -> 로직 수정 필요
		closest_waypoint_candidates.clear();
		//dist_ 결정
		dist_ = calcPlaneDist(current_pose, new_waypoints_[all_state_index_[lane_number_][current_mission_state_+1]].pose);
		//현위치와 다음 mission_state 간의 거리 (각종 미션에 사용)

		

		state_msg_.dist = dist_;

		state_msg_.current_state = current_mission_state_;
		state_msg_.lane_number = lane_number_;
		state_pub_.publish(state_msg_);//미션 관련 정보 publish
		
		lane_msg_.waypoints = final_waypoints_;
		lane_msg_.onlane = is_onlane_;		
		waypoint_pub_.publish(lane_msg_);//waypoint publish

		int final_waypoints_size = lane_msg_.waypoints.size();
		// ROS_INFO("FINAL WAYPOINTS NUMBER=%d PUBLISHED.", final_waypoints_size);	
	}


	/*
		getClosestWaypoint()
		1. 후보군 선정
		2. 최종 cloest_waypoint 선정
	*/
	void getClosestWaypoint(geometry_msgs::PoseStamped current_pose) 
	{
		
		//1. 후보군 선정
		//조사 시작 범위 : (현 mission state 시작점) - 4  ~ (현 mission state + 2)번 구역까지
		//조사시 mission_state 
		/*  ex)
			current mission state 3번 : 67 ~ 190 이라면
			조사 시작 범위 = 63 ~

			예외)
			current mission state 0번 : 0 ~ 
			조사 시작 범위 = -4 ~
			-> 오류 발생

			해결 : 	state_index_.emplace_back(5)
				-> current mission state 0번 : 5 ~
				   조사 시작 범위 : 1 ~
		*/
		int t_index = all_state_index_[lane_number_][current_mission_state_]-4;
		// cout << "==================================================================================" << endl;
		// cout << "search_startIndex : " << t_index << endl;
		// cout << "current_lane : " << lane_number_ << endl;
		// cout << "currnet_mission_state : " << current_mission_state_ << endl;
		// cout << "==================================================================================" << endl;

		for(int i = t_index; i < size_; i++) {
			double dist = calcPlaneDist(current_pose, new_waypoints_[i].pose);
			int t_state_check = new_waypoints_[i].mission_state - current_mission_state_;

			if (dist < max_search_dist_ && (t_state_check == 2 || t_state_check == 1 || t_state_check == 0)){
				closest_waypoint_candidates.emplace_back(i);
			}
			
			if(dist > max_search_dist_ && t_state_check > 2) break;
		}
		
		//2. 후보 중 최종 cloest_waypoint 선정
		if(!closest_waypoint_candidates.empty()) {
			int waypoint_min = -1;
			double dist_min = max_search_dist_; 
			for(int i=0;i<closest_waypoint_candidates.size();i++) {
				double dist = calcPlaneDist(current_pose, new_waypoints_[closest_waypoint_candidates[i]].pose);
				if(dist < dist_min) {
					dist_min = dist;
					waypoint_min = closest_waypoint_candidates[i];
				}
			}

			closest_waypoint_ = waypoint_min;
			
			ROS_INFO("CLOSEST WAYPOINT INDEX=%d, X=%f, Y=%f", closest_waypoint_, new_waypoints_[closest_waypoint_].pose.pose.position.x, new_waypoints_[closest_waypoint_].pose.pose.position.y);
			is_state_ = true;//middle start
		}
		else ROS_INFO("THERE IS NO CLOSEST WAYPOINT CANDIDATE.");

		
	}

// getFinalWaypoint()
	//최종 250개의 waypoint 선정
	void getFinalWaypoint(){ 
		if((size_ - closest_waypoint_ - 1) < final_size_) final_size_ = size_ - closest_waypoint_ - 1;
		
	 	for(int i=0;i<final_size_;i++) {
			final_waypoints_.emplace_back(new_waypoints_[closest_waypoint_ + i]);
		}
	}

	//Function in getClosetWaypoint()
	double calcPlaneDist(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2) {
		return sqrt(pow(pose1.pose.position.x - pose2.pose.position.x, 2) + pow(pose1.pose.position.y - pose2.pose.position.y, 2));
	} 



//Callback function
	void poseCallback(const nav_msgs::Odometry::ConstPtr &msg) {
		current_pose.pose.position = msg->pose.pose.position;
		lane_msg_.header = msg->header;		
	}

	// void CourseCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &course_msg) {	// 기존
	// 	double temp_course = 0.0;

	// 	temp_course = course_msg->drive.steering_angle;

	// 	is_course_gps_ = true;
		
	// 	if(is_course_gps_ && !heading_init_){ // default = 10;
	// 		heading_vec.push_back((double)temp_course);
	// 		//gps - atan 좌표계 통일 -> temp_course : 진북 기준, 시계 방향, 0 ~ π / 반시계 방향, 0 ~ -π	

	// 		cur_course_ = -1 * temp_course + 90;
	// 		if(cur_course_ < 180){
	// 			cur_course_ += 360;
	// 		}			


	// 		// cur_course_ = temp_course;
	// 		is_course_ = true;
	// 		is_course_gps_ = false;
	// 	}
	// 	if(!heading_init_ &&  heading_vec.size() >= 10 && !not_first){  // default = 10;
	// 		double max = *max_element(heading_vec.begin(), heading_vec.end());
	// 		double min = *min_element(heading_vec.begin(), heading_vec.end());
	// 		if(abs(max - min) > 180){
	// 			for(int i = 0; i < heading_vec.size(); i++){
	// 				if(heading_vec[i] > 180){
	// 					heading_vec[i] = heading_vec[i] - 360;
	// 				}
	// 			}
	// 		}
	// 	// 평균값 사용
	// 		for(int i = 0; i < heading_vec.size();i++){
	// 			initial_yaw_ += heading_vec[i];
	// 		}
	// 		initial_yaw_ /= heading_vec.size();
	// 		initial_yaw_ *= (M_PI/180);
	// 		heading_init_ = true;

	// 		not_first = true;
	// 		// 분산 활용해서 확룰 가장 높은거 선택하는 코드 작성하기
	// 	}
	// 	if(!heading_init_ && not_first){
	// 		initial_yaw_ = temp_course * M_PI / 180;
	// 		heading_init_ = true;
	// 	}


	// 	//cout << "heading init      : " << heading_init_ << endl;
	// 	//cout << "inital yaw(deg)   : " << initial_yaw_*180/M_PI << endl;
	// 	//cout << "heading vector size" << heading_vec.size() << endl;
	// 	//cout << "GPS Yaw is : " << cur_course_ << endl;
	// 	gps_yaw_ = temp_course;
	// }


	// void CourseCallback(const std_msgs::Float32::ConstPtr& msg)
	// {
	//     double temp_course = msg->data;   // [deg] 단위로 들어옴
	//     is_course_gps_ = true;

	//     /* --- 초기화 절차 --- */
	//     if (is_course_gps_ && !heading_init_) {
	//         heading_vec.push_back(temp_course);

	//         // 진북 기준 시계·반시계 좌표계 변환
	//         cur_course_ = -temp_course + 90.0;
	//         if (cur_course_ < 180.0)
	//             cur_course_ += 360.0;

	//         is_course_     = true;
	//         is_course_gps_ = false;
	//     }

	//     /* --- 초기 yaw 확정 --- */
	//     if (!heading_init_ && heading_vec.size() >= 10 && !not_first) {
	//         double max_v = *std::max_element(heading_vec.begin(), heading_vec.end());
	//         double min_v = *std::min_element(heading_vec.begin(), heading_vec.end());

	//         if (std::abs(max_v - min_v) > 180.0) {
	//             for (double& v : heading_vec)
	//                 if (v > 180.0) v -= 360.0;
	//         }

	//         initial_yaw_ = std::accumulate(heading_vec.begin(), heading_vec.end(), 0.0)
	//                        / heading_vec.size()
	//                        * (M_PI / 180.0);

	//         heading_init_ = true;
	//         not_first     = true;
	//     }

	//     if (!heading_init_ && not_first) {
	//         initial_yaw_  = temp_course * M_PI / 180.0;
	//         heading_init_ = true;
	//     }

	//     gps_yaw_ = temp_course;   // 최종 저장
	// }


	// void ImuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg)
	// {
	// 	tf::Quaternion imu_quaternion;
	// 	tf::quaternionMsgToTF(imu_msg->orientation, imu_quaternion);
	// 	imu_frame = imu_msg->header.frame_id;
	// 	double roll, pitch, yaw;

	// 	if (heading_init_){
	// 		ros::Time curr_time = imu_msg->header.stamp;
	// 		double dt = (curr_time - prev_time).toSec();

	// 		if (is_course_ && dt > 0) {
	// 			double wx = imu_msg->angular_velocity.x; // Roll 차량의 앞뒤 방향을 중심으로 좌우로 회전
	// 			double wy = imu_msg->angular_velocity.y; // Pitch 차량의 좌우 방향을 중심으로 앞뒤로 기울어짐 
	// 			double wz = imu_msg->angular_velocity.z; // Yaw 차량이 좌우로 도는 회전 (요잉, 차량의 회전 방향)

	// 			double prev_wx = prev_imu.angular_velocity.x;
	// 			double prev_wy = prev_imu.angular_velocity.y;
	// 			double prev_wz = prev_imu.angular_velocity.z;

	// 			double alpha_x = (wx - prev_wx) / dt;
	// 			double alpha_y = (wy - prev_wy) / dt;
	// 			double alpha_z = (wz - prev_wz) / dt;

	// 			static bool has_prev_alpha = false;
	// 			static double prev_alpha_x = 0, prev_alpha_y = 0, prev_alpha_z = 0;
	// 			double angular_jerk_x = 0, angular_jerk_y = 0, angular_jerk_z = 0;

	// 			if (has_prev_alpha) {
	// 				angular_jerk_x = (alpha_x - prev_alpha_x) / dt;
	// 				angular_jerk_y = (alpha_y - prev_alpha_y) / dt;
	// 				angular_jerk_z = (alpha_z - prev_alpha_z) / dt;
	// 			}

	// 			ROS_INFO("Angular Accel: Ax=%.3f m/s², Ay=%.3f m/s², Az=%.3f m/s²", alpha_x, alpha_y, alpha_z);

	// 			if (has_prev_alpha) {
	// 				ROS_INFO("Angular Jerk: Jαx=%.3f m/s³, Jαy=%.3f m/s³, Jαz=%.3f m/s³", angular_jerk_x, angular_jerk_y, angular_jerk_z);
	// 			}

	// 			prev_alpha_x = alpha_x;
	// 			prev_alpha_y = alpha_y;
	// 			prev_alpha_z = alpha_z;

	// 			yaw_accel = alpha_z;
	// 			roll_accel = alpha_x;
	// 			pitch_accel = alpha_y;

	// 			yaw_jerk = angular_jerk_z;
	// 			roll_jerk = angular_jerk_x;
	// 			pitch_jerk = angular_jerk_y;

	// 			has_prev_alpha = true;
	// 		}
			
	// 		// if(!imu_initial_){
	// 		// 	double roll_,pitch_,yaw_;
				
    //         // 	tf::Matrix3x3(imu_quaternion).getRPY(roll_, pitch_, yaw_);
	// 		// 	imu_initial_yaw_ = yaw_;
	// 		// 	imu_initial_ = true;
	// 		// }
	// 		is_course_ = true;
	// 		prev_imu = *imu_msg;
    //     	prev_time = curr_time;

	// 		try
	// 		{
	// 			listener.waitForTransform("imu_link", "gps", ros::Time(0), ros::Duration(1.0));
	// 			listener.lookupTransform("imu_link", "gps", ros::Time(0), imu_transform);
	// 		}
	// 		catch (tf::TransformException& ex)
	// 		{
	// 			ROS_ERROR("%s", ex.what());
	// 		}

	// 		tf::Quaternion transformed_quaternion =  imu_transform.getRotation() * imu_quaternion;
	// 		tf::Matrix3x3(transformed_quaternion).getRPY(roll, pitch, yaw);
	// 		// 변환된 map 프레임에서의 헤딩 값
	// 		//pp_cur_course_ = yaw * 180 / M_PI; 
	// 		cur_course_ = -(yaw * 180 / M_PI)+90.0;

	// 		if(cur_course_ >= 180){
	// 			cur_course_ -= 360;
	// 		}
	// 		// cout << "cur_course" << cur_course_ << endl;
	// 		// cout << "imu transform :: " << imu_transform.getRotation() << endl;
	// 		// cout << "===== IMU CallBack =====" << endl;
	// 		//cout << "imu_cur_course = " << cur_course_ * M_PI / 180<< endl;
	// 		// cout << "gps_cur_course = " << gps_yaw_ << endl;
	// 	}
	// }

	void CourseCallback(const std_msgs::Float64::ConstPtr &course_msg) 
	{
		cur_course_ = course_msg->data;
		// cur_speed_ = course_msg->drive.speed;
		is_course_ = true;
	}
	
	void lane_number_Callback(const waypoint_maker::State::ConstPtr &msg){
		lane_number_ = msg->lane_number;
	}


	void loader_rviz()
	{
		//1. 경로 총갯수 파악
		int total_size = 0;
		for(int i =0; i< all_csv_.size();i++)
		{
			total_size+=all_new_waypoints_[i].size();
		}

		nav_msgs::Path global_path_rviz;
		global_path_rviz.header.frame_id = "base_link";
		global_path_rviz.header.stamp = ros::Time::now();
		// global_path_rviz.poses.resize(total_size);
		if(all_csv_.size()!=0)
		{
			int idx_rowsize = 0;
			for (int i = 0; i < all_csv_.size(); i++)
			{

				for(int j=0;j<all_new_waypoints_[i].size();j++)
				{
					geometry_msgs::PoseStamped loc;
					loc.header.frame_id = "base_link";
					loc.header.stamp = ros::Time::now();
					loc.pose.position.x = (double)all_new_waypoints_[i][j].pose.pose.position.x;
					loc.pose.position.y = (double)all_new_waypoints_[i][j].pose.pose.position.y;
					// cout << "loc.pose.position.x" << (double)loc.pose.position.x << endl;
					// cout << "all new waypoints[i][j] : " << (double)all_new_waypoints_[i][j].pose.pose.position.x<<endl;
					// ROS_INFO("loc x : %lf",loc.pose.position.x);
					// ROS_INFO("loc y : %lf",loc.pose.position.y);
					loc.pose.position.z = 0.0;
					if(j % 10 == 0){
						global_path_rviz.poses.emplace_back(loc);
					}
				}
					idx_rowsize += all_new_waypoints_[i].size();
			}
		}
		// visualize_path(current_pose);
		rviz_global_path.publish(global_path_rviz);

		past_path_.emplace_back(current_pose);

		past_path_rviz_.header.frame_id = "base_link";
        past_path_rviz_.header.stamp = ros::Time::now();
        past_path_rviz_.poses = past_path_;

		rviz_currnet_path_.publish(past_path_rviz_);
	}

	// void visualize_path(geometry_msgs::PoseStamped current_pose){
    //     nav_msgs::Path imu_test_path_rviz;
    //     imu_test_path_rviz.header.frame_id = "map";
    //     imu_test_path_rviz.header.stamp = ros::Time::now();
    //     geometry_msgs::PoseStamped pose;
    //     pose.header.frame_id = "map";
    //     pose.header.stamp = ros::Time::now();

    //     // current_pose.pose.position.x = pose.pose.position.x;
    //     // current_pose.pose.position.y = pose.pose.position.y;
    //     // current_pose.pose.position.z = 0.0;
    //     // current_pose.header.frame_id = "map";
    //     // current_pose.header.stamp = ros::Time::now();
		
    //     imu_test_path_rviz.poses.emplace_back(current_pose);
    //     imu_test_path_pub_.publish(imu_test_path_rviz);
    // }

};

int main(int argc, char** argv) {
	ros::init(argc, argv, "waypoint_loader"); //노드명 -> 생성자 void init setup
	WaypointLoader wl;//생성자 void init setup
	ros::Rate loop_rate(20);
	
	while(ros::ok()) {
		ros::spinOnce();
		wl.poseProcess(); 
		wl.loader_rviz();
		loop_rate.sleep();	
	}
	return 0;
}
