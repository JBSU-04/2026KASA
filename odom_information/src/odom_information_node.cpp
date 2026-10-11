#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <morai_msgs/EgoVehicleStatus.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <string>
#include <std_msgs/Float32.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


class OdomInformationNode
{
public:
  OdomInformationNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh)
  , pnh_(pnh)
  , have_back_(false)
  , have_front_(false)
  , have_imu_(false)
  , yaw_initialized_(false)
  , yaw_(0.0)
  , imu_yaw_(0.0)
  {
    // ---- common params ----
    pnh_.param<std::string>("mode", mode_, std::string("dual_gps"));

    pnh_.param<std::string>("topics/odom_back", odom_back_topic_, std::string("/odom_back"));
    pnh_.param<std::string>("topics/odom_front", odom_front_topic_, std::string("/odom_front"));
    pnh_.param<std::string>("topics/imu", imu_topic_, std::string("/imu"));
    pnh_.param<std::string>("topics/odom_out", odom_out_topic_, std::string("/odom"));

    pnh_.param<std::string>("frames/parent", parent_frame_, std::string("map"));
    pnh_.param<std::string>("frames/child", child_frame_, std::string("base_link"));
    pnh_.param<bool>("publish_tf", publish_tf_, true);
    

    pnh_.param<std::string>("topics/cur_speed", cur_speed_topic_, std::string("/cur_speed"));
    pnh_.param<std::string>("topics/competition", competition_topic_,
                            std::string("/Competition_topic"));
    pnh_.param<bool>("cur_speed_is_kmh", cur_speed_is_kmh_, true);
    pnh_.param<double>("speed_scale", speed_scale_, 1.0);
    pnh_.param<double>("speed_stale_sec", speed_stale_sec_, 0.5);
    pnh_.param<bool>("competition_speed_enabled", competition_speed_enabled_, true);
    pnh_.param<double>("competition_speed_scale", competition_speed_scale_, 1.0);
    pnh_.param<double>("competition_speed_stale_sec", competition_speed_stale_sec_,
                       speed_stale_sec_);
    pnh_.param<double>("competition_speed_max_mps", competition_speed_max_mps_, 60.0);

    cur_speed_sub_ = nh_.subscribe(cur_speed_topic_, 50, &OdomInformationNode::curSpeedCallback, this);
    if (competition_speed_enabled_) {
      competition_sub_ = nh_.subscribe(competition_topic_, 50,
                                        &OdomInformationNode::competitionCallback, this);
    }

    // debug print (모드별로 param 네임스페이스 다르게 읽음)
    // 기본값은 안전하게 세팅
    print_yaw_ = true;
    print_period_ = 0.2;

    // yaw filter params (모드별)
    yaw_offset_ = 0.0;
    yaw_lpf_alpha_ = 0.2;

    // dual gps only
    min_baseline_ = 0.8;
    max_baseline_ = 3.3;

    if (mode_ == "dual_gps")
    {
      pnh_.param<double>("dual_gps/yaw_offset", yaw_offset_, 0.0);
      pnh_.param<double>("dual_gps/min_baseline", min_baseline_, 0.8);
      pnh_.param<double>("dual_gps/max_baseline", max_baseline_, 3.3);
      pnh_.param<double>("dual_gps/yaw_lpf_alpha", yaw_lpf_alpha_, 0.2);
      pnh_.param<bool>("dual_gps/print_yaw", print_yaw_, true);
      pnh_.param<double>("dual_gps/print_period", print_period_, 0.2);
    }
    else if (mode_ == "gps_imu")
    {
      pnh_.param<double>("gps_imu/yaw_offset", yaw_offset_, 0.0);
      pnh_.param<double>("gps_imu/yaw_lpf_alpha", yaw_lpf_alpha_, 0.2);
      pnh_.param<bool>("gps_imu/print_yaw", print_yaw_, true);
      pnh_.param<double>("gps_imu/print_period", print_period_, 0.2);
    }
    else
    {
      ROS_WARN_STREAM("[odom_information] unknown mode=" << mode_ << " -> fallback to dual_gps");
      mode_ = "dual_gps";
    }

    odom_back_sub_ = nh_.subscribe(odom_back_topic_, 10, &OdomInformationNode::OdombackCallback, this);

    if (mode_ == "dual_gps")
    {
      odom_front_sub_ = nh_.subscribe(odom_front_topic_, 10, &OdomInformationNode::OdomfrontCallback, this);
    }
    imu_sub_ = nh_.subscribe(imu_topic_, 50, &OdomInformationNode::imuCallback, this);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_out_topic_, 10);

    ROS_INFO_STREAM("[odom_information] started. mode=" << mode_);
    ROS_INFO_STREAM("[odom_information] sub back : " << odom_back_topic_);


    if (mode_ == "dual_gps")
    {
      ROS_INFO_STREAM("[odom_information] sub front: " << odom_front_topic_);
    }
    ROS_INFO_STREAM("[odom_information] sub imu  : " << imu_topic_);
    ROS_INFO_STREAM("[odom_information] pub odom : " << odom_out_topic_);
    ROS_INFO_STREAM("[odom_information] publish TF: " << (publish_tf_ ? "true" : "false"));
    ROS_INFO_STREAM("[odom_information] speed source: "
                    << (competition_speed_enabled_ ? competition_topic_ : std::string("disabled"))
                    << " (priority), fallback=" << cur_speed_topic_);

    if (mode_ == "dual_gps")
    {
      ROS_INFO_STREAM("[odom_information] dual_gps params: min_baseline=" << min_baseline_
                      << ", max_baseline=" << max_baseline_
                      << ", yaw_lpf_alpha=" << yaw_lpf_alpha_
                      << ", yaw_offset=" << yaw_offset_
                      << ", print_yaw=" << (print_yaw_ ? "true" : "false")
                      << ", print_period=" << print_period_);
    }
    else if(mode_ =="gps_imu")
    {
      ROS_INFO_STREAM("[odom_information] gps_imu params: yaw_lpf_alpha=" << yaw_lpf_alpha_
                      << ", yaw_offset=" << yaw_offset_
                      << ", print_yaw=" << (print_yaw_ ? "true" : "false")
                      << ", print_period=" << print_period_);
    }
  }

private:
  // ---------- callbacks ----------
  void OdombackCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    last_back_ = *msg;
    have_back_ = true;
    new_back_ = true;
    publishIfReady();
  }


  void OdomfrontCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    last_front_ = *msg;
    have_front_ = true;
    new_front_ = true;
    publishIfReady();
  }


  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
  {
    tf2::Quaternion q(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w
    );

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    imu_yaw_ = wrapToPi(yaw);
    have_imu_ = true;

    // 중요: imu 콜백에서 publish하면 gps_imu 모드에서 /odom이 중복 발행됨
    // publishIfReady();  <-- 삭제!
  }


  void curSpeedCallback(const std_msgs::Float32::ConstPtr& msg)
  {
    cur_speed_raw_ = msg->data;
    cur_speed_recv_time_ = ros::Time::now();
    have_speed_ = true;
  }

  void competitionCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& msg)
  {
    // MORAI publishes the ego velocity vector in m/s. nav_msgs/Odometry
    // expresses twist in child_frame_id (base_link), so its magnitude is
    // published as the forward component rather than copying map-frame x/y.
    const double speed_mps = std::hypot(msg->velocity.x, msg->velocity.y) *
        competition_speed_scale_;
    if (!std::isfinite(speed_mps) || speed_mps < 0.0 ||
        speed_mps > competition_speed_max_mps_) {
      ROS_WARN_THROTTLE(1.0,
                        "[odom_information] ignoring invalid Competition speed %.3f m/s",
                        speed_mps);
      return;
    }
    competition_speed_mps_ = speed_mps;
    competition_speed_recv_time_ = ros::Time::now();
    have_competition_speed_ = true;
  }

  bool freshCompetitionSpeed(double& speed_mps) const
  {
    if (!competition_speed_enabled_ || !have_competition_speed_) return false;
    const double age = (ros::Time::now() - competition_speed_recv_time_).toSec();
    if (age < 0.0 || age > competition_speed_stale_sec_) return false;
    if (!std::isfinite(competition_speed_mps_) || competition_speed_mps_ < 0.0 ||
        competition_speed_mps_ > competition_speed_max_mps_) {
      return false;
    }
    speed_mps = competition_speed_mps_;
    return true;
  }

  bool freshCurSpeed(double& speed_mps) const
  {
    if (!have_speed_) return false;
    const double age = (ros::Time::now() - cur_speed_recv_time_).toSec();
    if (age < 0.0 || age > speed_stale_sec_) return false;

    double speed = cur_speed_raw_;
    if (cur_speed_is_kmh_) speed /= 3.6;
    speed *= speed_scale_;
    if (!std::isfinite(speed) || speed < 0.0 || speed > 30.0) return false;
    speed_mps = speed;
    return true;
  }

  void applyEgoSpeedToOdom(nav_msgs::Odometry& odom) const
  {
    double speed_mps = 0.0;
    if (!freshCompetitionSpeed(speed_mps) && !freshCurSpeed(speed_mps)) {
      // A missing speed source is explicitly visible as zero twist. In normal
      // MORAI use the fresh Competition feedback above is always selected.
      speed_mps = 0.0;
    }
    odom.twist.twist.linear.x = speed_mps;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
  }
  static double wrapToPi(double a)
  {
    while (a > M_PI)  a -= 2.0 * M_PI;  // M_PI = 3.14
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  // dual gps yaw: front-back baseline vector
  bool computeYawFromDualGps(double& yaw_out, double& baseline_out, double& dx_out, double& dy_out)
  {
    const double xb = last_back_.pose.pose.position.x;
    const double yb = last_back_.pose.pose.position.y;
    const double xf = last_front_.pose.pose.position.x;
    const double yf = last_front_.pose.pose.position.y;

    const double dx = xf - xb;
    const double dy = yf - yb;

    const double baseline = std::hypot(dx, dy);   // 벡터(dx, dy)사이의 길이

    dx_out = dx;
    dy_out = dy;
    baseline_out = baseline;

    // 방어코드: baseline이 너무 작으면 헤딩 불안정 / 너무 크면 GPS 튐 가능성
    if (baseline < min_baseline_ || baseline > max_baseline_)
      return false;

    double yaw_meas = std::atan2(dy, dx) + yaw_offset_;   // yaw_meas : yaw 계산 , atan2(dy, dx) : 차량이 바라보는 방향
    yaw_meas = wrapToPi(yaw_meas);

    // LPF(Low Pass Filter) (angle wrap 고려)
    if (!yaw_initialized_)
    {
      yaw_ = yaw_meas;
      yaw_initialized_ = true;
    }
    else
    // -> 최단 각도 차이 179(3.124rad) ~ -179(-3.124rad) 실제 2만 움직임 하지만 rad: -6.2348 이걸 보정
    {
      const double err = wrapToPi(yaw_meas - yaw_);
      // gps 노이즈 보정
      yaw_ = wrapToPi(yaw_ + yaw_lpf_alpha_ * err);
    }

    yaw_out = yaw_;
    return true;
  }

  // imu yaw
  bool computeYawFromImu(double& yaw_out)
  {
    if (!have_imu_) return false;

    double yaw_meas = wrapToPi(imu_yaw_ + yaw_offset_);

    if (!yaw_initialized_)
    {
      yaw_ = yaw_meas;
      yaw_initialized_ = true;
    }
    else
    {
      const double err = wrapToPi(yaw_meas - yaw_);
      yaw_ = wrapToPi(yaw_ + yaw_lpf_alpha_ * err);
    }

    yaw_out = yaw_;
    return true;
  }

  void publishIfReady()
  {
    if (mode_ == "dual_gps")
    {
      if (!have_back_ || !have_front_) return;

      // ✅ 핵심: back/front 둘 다 "새로운 데이터"가 들어왔을 때만 publish 1회
      if (!(new_back_ && new_front_)) return;

      // 같은 조합으로 이미 publish 했으면 방지 (안전장치)
      if (last_back_.header.seq == last_pub_back_seq_ &&
          last_front_.header.seq == last_pub_front_seq_) {
        new_back_ = false;
        new_front_ = false;
        return;
      }

      nav_msgs::Odometry odom = last_back_;

      // ✅ stamp는 now() 금지: 입력 센서 stamp 사용 (back/front 중 최신)
      odom.header.stamp = last_back_.header.stamp;
      if (last_front_.header.stamp > odom.header.stamp)
        odom.header.stamp = last_front_.header.stamp;

      odom.header.frame_id = parent_frame_;
      odom.child_frame_id  = child_frame_;

      double yaw = 0.0;
      double baseline = 0.0, dx = 0.0, dy = 0.0;
      const bool yaw_ok = computeYawFromDualGps(yaw, baseline, dx, dy);

      if (!yaw_ok)
      {
        if (!yaw_initialized_) return;
        yaw = yaw_;
      }

      if (print_yaw_)
      {
        ROS_INFO_THROTTLE(print_period_,
                          "[dual_gps] yaw=%.3f rad (%.1f deg) baseline=%.2f m ok=%d dx=%.2f dy=%.2f",
                          yaw, yaw * 180.0 / M_PI, baseline, yaw_ok ? 1 : 0, dx, dy);
      }

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);

      odom.pose.pose.orientation.x = q.x();
      odom.pose.pose.orientation.y = q.y();
      odom.pose.pose.orientation.z = q.z();
      odom.pose.pose.orientation.w = q.w();
      
      applyEgoSpeedToOdom(odom);


      geometry_msgs::TransformStamped t;
      // /odom_back can carry a simulator clock that differs from ROS wall
      // time.  Publish the TF with the active clock so RViz can transform
      // base_link-frame visualizations at their current display time.
      t.header.stamp = ros::Time::now();
      t.header.frame_id = parent_frame_;   // "map"
      t.child_frame_id  = child_frame_;    // "base_link"

      t.transform.translation.x = odom.pose.pose.position.x;
      t.transform.translation.y = odom.pose.pose.position.y;
      t.transform.translation.z = odom.pose.pose.position.z;

      t.transform.rotation.x = odom.pose.pose.orientation.x;
      t.transform.rotation.y = odom.pose.pose.orientation.y;
      t.transform.rotation.z = odom.pose.pose.orientation.z;
      t.transform.rotation.w = odom.pose.pose.orientation.w;

      if (publish_tf_) tf_br_.sendTransform(t);
      
      odom_pub_.publish(odom);

      // ✅ publish 완료 처리: 이번 조합을 기록하고, 새 데이터 플래그 리셋
      last_pub_back_seq_  = last_back_.header.seq;
      last_pub_front_seq_ = last_front_.header.seq;
      new_back_  = false;
      new_front_ = false;
      return;
    }

    // gps_imu
    if (mode_ == "gps_imu")
    {
      if (!have_back_ || !have_imu_) return;

      // ✅ gps_imu는 "back이 새로 들어온 경우"에만 publish (imu는 yaw 업데이트만 함)
      if (!new_back_) return;

      // 같은 back seq로 중복 publish 방지
      if (last_back_.header.seq == last_pub_back_seq_) {
        new_back_ = false;
        return;
      }

      nav_msgs::Odometry odom = last_back_;
      odom.header.stamp = last_back_.header.stamp;   // ✅ now() 금지
      odom.header.frame_id = parent_frame_;
      odom.child_frame_id  = child_frame_;

      double yaw = 0.0;
      const bool yaw_ok = computeYawFromImu(yaw);
      if (!yaw_ok)
      {
        if (!yaw_initialized_) return;
        yaw = yaw_;
      }

      if (print_yaw_)
      {
        ROS_INFO_THROTTLE(print_period_,
                          "[gps_imu] yaw=%.3f rad (%.1f deg)",
                          yaw, yaw * 180.0 / M_PI);
      }

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);

      odom.pose.pose.orientation.x = q.x();
      odom.pose.pose.orientation.y = q.y();
      odom.pose.pose.orientation.z = q.z();
      odom.pose.pose.orientation.w = q.w();


      applyEgoSpeedToOdom(odom);

      geometry_msgs::TransformStamped t;
      // /odom_back can carry a simulator clock that differs from ROS wall
      // time.  Publish the TF with the active clock so RViz can transform
      // base_link-frame visualizations at their current display time.
      t.header.stamp = ros::Time::now();
      t.header.frame_id = parent_frame_;   // "map"
      t.child_frame_id  = child_frame_;    // "base_link"

      t.transform.translation.x = odom.pose.pose.position.x;
      t.transform.translation.y = odom.pose.pose.position.y;
      t.transform.translation.z = odom.pose.pose.position.z;

      t.transform.rotation.x = odom.pose.pose.orientation.x;
      t.transform.rotation.y = odom.pose.pose.orientation.y;
      t.transform.rotation.z = odom.pose.pose.orientation.z;
      t.transform.rotation.w = odom.pose.pose.orientation.w;

      if (publish_tf_) tf_br_.sendTransform(t);

      odom_pub_.publish(odom);

      // publish 완료 처리
      last_pub_back_seq_ = last_back_.header.seq;
      new_back_ = false;
      return;
    }
  }


  // // ---------- publish ----------
  // void publishIfReady()
  // {
  //   if (mode_ == "dual_gps")
  //   {
  //     if (!have_back_ || !have_front_) return;

  //     nav_msgs::Odometry odom = last_back_;
  //     odom.header.stamp = ros::Time::now();
  //     odom.header.frame_id = parent_frame_;
  //     odom.child_frame_id = child_frame_;

  //     double yaw = 0.0;
  //     double baseline = 0.0, dx = 0.0, dy = 0.0;
  //     const bool yaw_ok = computeYawFromDualGps(yaw, baseline, dx, dy);

  //     // baseline invalid면 "이전 yaw"를 쓰되, 아직 초기화 안됐으면 publish 보류
  //     if (!yaw_ok)
  //     {
  //       if (!yaw_initialized_) return;
  //       yaw = yaw_;
  //     }

  //     if (print_yaw_)
  //     {
  //       ROS_INFO_THROTTLE(print_period_,
  //                         "[dual_gps] yaw=%.3f rad (%.1f deg) baseline=%.2f m ok=%d dx=%.2f dy=%.2f",
  //                         yaw, yaw * 180.0 / M_PI, baseline, yaw_ok ? 1 : 0, dx, dy);
  //     }

  //     tf2::Quaternion q;
  //     q.setRPY(0.0, 0.0, yaw);

  //     odom.pose.pose.orientation.x = q.x();
  //     odom.pose.pose.orientation.y = q.y();
  //     odom.pose.pose.orientation.z = q.z();
  //     odom.pose.pose.orientation.w = q.w();

  //     odom_pub_.publish(odom);
  //     return;
  //   }

  //   // gps_imu
  //   if (mode_ == "gps_imu")
  //   {
  //     if (!have_back_ || !have_imu_) return;

  //     nav_msgs::Odometry odom = last_back_;
  //     odom.header.stamp = ros::Time::now();
  //     odom.header.frame_id = parent_frame_;
  //     odom.child_frame_id = child_frame_;

  //     double yaw = 0.0;
  //     const bool yaw_ok = computeYawFromImu(yaw);
  //     if (!yaw_ok)
  //     {
  //       if (!yaw_initialized_) return;
  //       yaw = yaw_;
  //     }

  //     if (print_yaw_)
  //     {
  //       ROS_INFO_THROTTLE(print_period_,
  //                         "[gps_imu] yaw=%.3f rad (%.1f deg)",
  //                         yaw, yaw * 180.0 / M_PI);
  //     }

  //     tf2::Quaternion q;
  //     q.setRPY(0.0, 0.0, yaw);

  //     odom.pose.pose.orientation.x = q.x();
  //     odom.pose.pose.orientation.y = q.y();
  //     odom.pose.pose.orientation.z = q.z();
  //     odom.pose.pose.orientation.w = q.w();

  //     odom_pub_.publish(odom);
  //     return;
  //   }
  // }

private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber odom_back_sub_, odom_front_sub_, imu_sub_, cur_speed_sub_, competition_sub_;
  ros::Publisher odom_pub_;

  nav_msgs::Odometry last_back_, last_front_;
  bool have_back_, have_front_, have_imu_;

  std::string mode_;
  std::string odom_back_topic_, odom_front_topic_, odom_out_topic_, imu_topic_;
  std::string parent_frame_, child_frame_;
  std::string cur_speed_topic_, competition_topic_;


  // params
  double yaw_offset_;
  double yaw_lpf_alpha_;

  // dual_gps only
  double min_baseline_;
  double max_baseline_;

  // debug
  bool print_yaw_;
  double print_period_;

  // yaw state
  bool yaw_initialized_;
  double yaw_;
  double imu_yaw_;

  // publish gating (중복 publish 방지용)
  uint32_t last_pub_back_seq_  = 0;
  uint32_t last_pub_front_seq_ = 0;
  bool new_back_  = false;
  bool new_front_ = false;

  double cur_speed_raw_ = 0.0;   // 받은 값 그대로
  ros::Time cur_speed_recv_time_;
  bool have_speed_ = false;

  double competition_speed_mps_ = 0.0;
  ros::Time competition_speed_recv_time_;
  bool have_competition_speed_ = false;

  bool cur_speed_is_kmh_ = true;
  double speed_scale_ = 1.0;          // 필요시 보정
  double speed_stale_sec_ = 0.5;      // 0.5초 이상 old면 무시(선택)
  bool competition_speed_enabled_ = true;
  double competition_speed_scale_ = 1.0;
  double competition_speed_stale_sec_ = 0.5;
  double competition_speed_max_mps_ = 60.0;
  bool publish_tf_ = true;

  tf2_ros::TransformBroadcaster tf_br_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_information_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  OdomInformationNode node(nh, pnh);
  ros::spin();
  return 0;
}
