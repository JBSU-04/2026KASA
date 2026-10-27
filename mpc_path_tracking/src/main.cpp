#include <ros/ros.h>
#include <mpc_path_tracking/mpc_path_tracking.h>
#include <std_msgs/Float64.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "mpc_path_tracking");

	ROSCONTROL mpc;
	ros::NodeHandle nh;
	ros::Publisher controller_time_pub =
		nh.advertise<std_msgs::Float64>("/mpc/controller_time_ms", 10);
	ros::Rate loop_rate(20);

	while(ros::ok()) 
	{
		ros::spinOnce();
		const ros::WallTime controller_begin = ros::WallTime::now();
		mpc.modelPredictiveController();
		const double controller_time_ms =
			(ros::WallTime::now() - controller_begin).toSec() * 1000.0;

		std_msgs::Float64 controller_time_msg;
		controller_time_msg.data = controller_time_ms;
		controller_time_pub.publish(controller_time_msg);

		loop_rate.sleep();
	}
	return 0;
}
