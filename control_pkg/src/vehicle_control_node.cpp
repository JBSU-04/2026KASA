// vehicle_control_node — 음영구간(M8) 전용 종방향 실행기.
//
// 변경 이유: 이전 구현은 brake <= 0.1 인 정상 주행 구간에서 longlCmdType = 2
// (Velocity) 를 썼다. 규정 2-8은 "cmd type 은 1번(accel, brake)"만 허용하므로
// 음영구간 주행 대부분이 위반이 된다. 여기서 type 1 로 고정하고, 목표속도는
// 페달 PI 로 추종한다.
//
// 조향은 손대지 않는다. sliding_window 가 낸 값을 그대로 통과시킨다 —
// 팀이 피팅한 게인을 살리는 것이 이 구조(B안)의 핵심이다.
//
// 출력 토픽 기본값은 /control/m8_cmd 다. /ctrl_cmd 의 유일한 publisher 는
// control_arbiter 이며, 구간이 차선주행일 때만 이 명령을 채택한다.
#include <algorithm>
#include <string>

#include <morai_msgs/CtrlCmd.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <non_gps_msgs/ControlCommand.h>
#include <ros/ros.h>

class VehicleControlNode
{
public:
  VehicleControlNode() : pnh_("~")
  {
    pnh_.param<std::string>("control_command_topic", input_topic_, "/control/command");
    pnh_.param<std::string>("ctrl_cmd_topic", output_topic_, "/control/m8_cmd");
    pnh_.param<std::string>("ego_status_topic", ego_topic_, "/Competition_topic");

    // MPC 의 longitudinal 게인을 출발점으로 삼는다.
    pnh_.param("throttle_kp", kp_, 0.25);
    pnh_.param("throttle_ki", ki_, 0.03);
    pnh_.param("throttle_integral_limit", i_limit_, 2.0);
    pnh_.param("throttle_accel_limit", accel_limit_, 0.60);
    pnh_.param("throttle_brake_limit", brake_limit_, 1.00);
    pnh_.param("speed_timeout", speed_timeout_, 0.5);
    // 목표보다 이만큼 이상 빠르면 가속을 끊고 제동으로 넘어간다 [m/s]
    pnh_.param("brake_deadband_mps", brake_deadband_, 0.5);

    pub_ = nh_.advertise<morai_msgs::CtrlCmd>(output_topic_, 10);
    sub_ = nh_.subscribe(input_topic_, 1, &VehicleControlNode::callback, this);
    ego_sub_ = nh_.subscribe(ego_topic_, 1, &VehicleControlNode::egoCallback, this);

    ROS_INFO("[vehicle_control] %s -> %s (cmd type 1 fixed, pedal PI)",
             input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void egoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr &msg)
  {
    speed_mps_ = msg->velocity.x;
    speed_stamp_ = ros::Time::now();
  }

  void callback(const non_gps_msgs::ControlCommand::ConstPtr &input)
  {
    morai_msgs::CtrlCmd output;
    output.longlCmdType = 1;  // 규정 2-8: accel/brake 만 허용
    output.steering = static_cast<float>(input->steering_angle);
    output.velocity = 0.0;
    output.acceleration = 0.0;

    const ros::Time now = ros::Time::now();
    const bool speed_fresh =
        !speed_stamp_.isZero() && (now - speed_stamp_).toSec() < speed_timeout_;

    // 미션이 제동을 요구하면 PI 를 우회한다.
    if (input->brake > 0.1)
    {
      integral_ = 0.0;
      output.accel = 0.0;
      output.brake = std::min(brake_limit_, static_cast<double>(input->brake));
      publish(output, input, speed_fresh);
      return;
    }

    const double target_mps = input->target_speed / 3.6;  // 입력은 km/h

    if (!speed_fresh)
    {
      // 실측 속도를 못 받으면 적분하지 않고 입력 accel 을 그대로 쓴다.
      ROS_WARN_THROTTLE(2.0, "[vehicle_control] %s stale — open loop",
                        ego_topic_.c_str());
      integral_ = 0.0;
      output.accel = std::min(accel_limit_, static_cast<double>(input->accel));
      output.brake = 0.0;
      publish(output, input, speed_fresh);
      return;
    }

    const double err = target_mps - speed_mps_;
    const double dt = last_stamp_.isZero() ? 0.05 : (now - last_stamp_).toSec();
    last_stamp_ = now;

    if (err > -brake_deadband_)
    {
      integral_ += err * std::max(0.0, std::min(dt, 0.2));
      integral_ = std::max(-i_limit_, std::min(i_limit_, integral_));
      const double u = kp_ * err + ki_ * integral_;
      output.accel = std::max(0.0, std::min(accel_limit_, u));
      output.brake = 0.0;
    }
    else
    {
      // 목표보다 충분히 빠르다 — 적분을 풀고 제동한다.
      integral_ = 0.0;
      output.accel = 0.0;
      output.brake = std::max(0.0, std::min(brake_limit_, 0.15 * (-err)));
    }

    publish(output, input, speed_fresh);
  }

  void publish(const morai_msgs::CtrlCmd &out,
               const non_gps_msgs::ControlCommand::ConstPtr &input,
               bool speed_fresh)
  {
    pub_.publish(out);
    ROS_INFO_THROTTLE(0.5,
                      "[vehicle_control] src=%s tgt=%.1fkph cur=%.1fkph "
                      "accel=%.2f brake=%.2f steer=%.3f%s",
                      input->source.c_str(), input->target_speed,
                      speed_mps_ * 3.6, out.accel, out.brake, out.steering,
                      speed_fresh ? "" : " [open-loop]");
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_, ego_sub_;
  ros::Publisher pub_;
  std::string input_topic_, output_topic_, ego_topic_;

  double kp_{0.25}, ki_{0.03}, i_limit_{2.0};
  double accel_limit_{0.6}, brake_limit_{1.0};
  double speed_timeout_{0.5}, brake_deadband_{0.5};
  double integral_{0.0};
  double speed_mps_{0.0};
  ros::Time speed_stamp_, last_stamp_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vehicle_control_node");
  VehicleControlNode node;
  ros::spin();
  return 0;
}
