/*
 * File: line_node.cpp
 * Description: MORAI lane following node.
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <numeric>
#include <cmath>
#include <algorithm>
#include <limits>
#include <cv_bridge/cv_bridge.h>

class LineNode
{
public:
  LineNode() : pnh_("~")
  {
    loadParams();
    sub_ = nh_.subscribe(image_topic_, 1, &LineNode::imageCb, this);
    steering_pub_ = nh_.advertise<std_msgs::Float64>(steering_topic_, 10);
    valid_pub_ = nh_.advertise<std_msgs::Bool>(valid_topic_, 10);
    current_lane_pub_ = nh_.advertise<std_msgs::Int32>(current_lane_topic_, 1, true);
    lane_change_sub_ = nh_.subscribe("/lane_change_cmd", 1, &LineNode::laneChangeCallback, this);
    ego_sub_ = nh_.subscribe("/Competition_topic", 1, &LineNode::egoCb, this);
    publishCurrentLane();

    cv::namedWindow("Original", cv::WINDOW_NORMAL);
    cv::namedWindow("ROI", cv::WINDOW_NORMAL);
    cv::namedWindow("BEV", cv::WINDOW_NORMAL);
    cv::namedWindow("SlidingWindow", cv::WINDOW_NORMAL);
  }

  void spin()
  {
    ros::Rate rate(40);
    while (ros::ok())
    {
      ros::spinOnce();
      if (!frame_.empty())
        processOnce();
      if (cv::waitKey(1) == 'q')
        break;
      rate.sleep();
    }
    cv::destroyAllWindows();
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_;
  ros::Subscriber lane_change_sub_;
  ros::Subscriber ego_sub_;
  ros::Publisher steering_pub_;
  ros::Publisher valid_pub_;
  ros::Publisher current_lane_pub_;

  std::string image_topic_ = "/twinlitenet/lane_mask";
  std::string steering_topic_ = "/lane_perception/steering_angle";
  std::string valid_topic_ = "/lane_perception/valid";
  std::string current_lane_topic_ = "/current_lane";
  int target_width_ = 640;
  int target_height_ = 480;


  cv::Mat frame_;
  double last_lane_width_px_ = 440.0;
  double prev_left_x_ = -1.0;
  double prev_right_x_ = -1.0;
  bool have_prev_ = false;
  double lane_curve_abs_a_ = 0.0; // 2차항 |a| 기반 곡률 프록시

  double steer_k_straight_ = 0.08;
  double steer_k_curve_ = 0.20;
  double curve_a_low_ = 8e-5;
  double curve_a_high_ = 3e-4;
  double steer_limit_deg_ = 30.0;
  double lane_change_target_distance_ = 10.0;
  double lane_change_min_distance_ = 7.0;
  double bev_src_bl_x_ = 0.0;
  double bev_src_bl_y_ = 480.0;
  double bev_src_br_x_ = 640.0;
  double bev_src_br_y_ = 480.0;
  double bev_src_tl_x_ = 150.0;
  double bev_src_tl_y_ = 240.0;
  double bev_src_tr_x_ = 490.0;
  double bev_src_tr_y_ = 240.0;
  double bev_dst_bl_x_ = 100.0;
  double bev_dst_bl_y_ = 480.0;
  double bev_dst_br_x_ = 540.0;
  double bev_dst_br_y_ = 480.0;
  double bev_dst_tl_x_ = 0.0;
  double bev_dst_tl_y_ = 0.0;
  double bev_dst_tr_x_ = 640.0;
  double bev_dst_tr_y_ = 0.0;
  double roi_bottom_ratio_ = 1.0;

  enum DetectState
  {
    BOTH = 0,
    LEFT_ONLY = 1,
    RIGHT_ONLY = 2,
    NONE = 3
  };
  DetectState det_state_ = NONE;

  enum DriveState { KEEP_LANE, CHANGE_LEFT, CHANGE_RIGHT };
  enum LanePosition { LEFT_LANE = 1, RIGHT_LANE = 2 };
  DriveState drive_state_ = KEEP_LANE;
  LanePosition current_lane_ = RIGHT_LANE;
  ros::Time lane_change_last_update_;
  double lane_change_distance_traveled_ = 0.0;
  double lane_change_physical_distance_ = 0.0;
  double ego_speed_ = 0.0;

  void publishCurrentLane()
  {
      std_msgs::Int32 msg;
      msg.data = static_cast<int32_t>(current_lane_);
      current_lane_pub_.publish(msg);
      ROS_INFO("[CURRENT LANE] %s (%d)",
               current_lane_ == LEFT_LANE ? "LEFT" : "RIGHT", msg.data);
  }

  void laneChangeCallback(const std_msgs::Int32::ConstPtr& msg)
  {
      if (msg->data == -1 && drive_state_ != CHANGE_LEFT) {
          drive_state_ = CHANGE_LEFT;
          lane_change_last_update_ = ros::Time::now();
          lane_change_distance_traveled_ = 0.0;
          lane_change_physical_distance_ = 0.0;
          ROS_INFO("[LANE CHANGE] Starting Left Lane Change!");
      } else if (msg->data == 1 && drive_state_ != CHANGE_RIGHT) {
          drive_state_ = CHANGE_RIGHT;
          lane_change_last_update_ = ros::Time::now();
          lane_change_distance_traveled_ = 0.0;
          lane_change_physical_distance_ = 0.0;
          ROS_INFO("[LANE CHANGE] Starting Right Lane Change!");
      } else if (msg->data == 0 && drive_state_ != KEEP_LANE) {
          drive_state_ = KEEP_LANE;
      }
  }

  void egoCb(const morai_msgs::EgoVehicleStatus::ConstPtr &msg)
  {
      ego_speed_ = std::abs(msg->velocity.x);
  }

  void imageCb(const sensor_msgs::ImageConstPtr &msg)
  {
    try
    {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "mono8");
      setFrame(cv_ptr->image);
    }
    catch (const cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }

  void setFrame(const cv::Mat &frame)
  {
    if (frame.empty())
      return;

    if (frame.cols != target_width_ || frame.rows != target_height_)
      cv::resize(frame, frame_, cv::Size(target_width_, target_height_));
    else
      frame_ = frame.clone();
    cv::threshold(frame_, frame_, 127, 255, cv::THRESH_BINARY);
  }

  cv::Mat applyROI(const cv::Mat &bin)
  {
    int h = bin.rows, w = bin.cols;
    double roi_ratio = std::max(0.0, std::min(1.0, roi_bottom_ratio_));
    int y0 = static_cast<int>(h * (1.0 - roi_ratio));
    cv::Mat mask = cv::Mat::zeros(bin.size(), CV_8UC1);
    cv::rectangle(mask, cv::Rect(0, y0, w, h - y0), 255, cv::FILLED);
    cv::Mat out;
    cv::bitwise_and(bin, mask, out);
    return out;
  }

  void bevMatrices(int /*width*/, int /*height*/, cv::Mat &M, cv::Mat &Minv)
  {
    cv::Point2f src[4] = {
      {static_cast<float>(bev_src_bl_x_), static_cast<float>(bev_src_bl_y_)},
      {static_cast<float>(bev_src_br_x_), static_cast<float>(bev_src_br_y_)},
      {static_cast<float>(bev_src_tl_x_), static_cast<float>(bev_src_tl_y_)},
      {static_cast<float>(bev_src_tr_x_), static_cast<float>(bev_src_tr_y_)}
    };
    cv::Point2f dst[4] = {
      {static_cast<float>(bev_dst_bl_x_), static_cast<float>(bev_dst_bl_y_)},
      {static_cast<float>(bev_dst_br_x_), static_cast<float>(bev_dst_br_y_)},
      {static_cast<float>(bev_dst_tl_x_), static_cast<float>(bev_dst_tl_y_)},
      {static_cast<float>(bev_dst_tr_x_), static_cast<float>(bev_dst_tr_y_)}
    };
    M = cv::getPerspectiveTransform(src, dst);
    Minv = cv::getPerspectiveTransform(dst, src);
  }

  cv::Mat toBEV(const cv::Mat &img)
  {
    cv::Mat M, Minv, warped;
    bevMatrices(img.cols, img.rows, M, Minv);
    cv::warpPerspective(img, warped, M, img.size(), cv::INTER_NEAREST);
    return warped;
  }

  void loadParams()
  {
    pnh_.param<std::string>("image_topic", image_topic_, image_topic_);
    pnh_.param<std::string>("steering_topic", steering_topic_, steering_topic_);
    pnh_.param<std::string>("valid_topic", valid_topic_, valid_topic_);
    pnh_.param<std::string>("current_lane_topic", current_lane_topic_, current_lane_topic_);
    pnh_.param("target_width", target_width_, target_width_);
    pnh_.param("target_height", target_height_, target_height_);
    pnh_.param("steer_k_straight", steer_k_straight_, steer_k_straight_);
    pnh_.param("steer_k_curve", steer_k_curve_, steer_k_curve_);
    pnh_.param("curve_a_low", curve_a_low_, curve_a_low_);
    pnh_.param("curve_a_high", curve_a_high_, curve_a_high_);
    pnh_.param("steer_limit_deg", steer_limit_deg_, steer_limit_deg_);
    pnh_.param("lane_change_target_distance", lane_change_target_distance_, lane_change_target_distance_);
    pnh_.param("lane_change_min_distance", lane_change_min_distance_, lane_change_min_distance_);
    pnh_.param("bev_src_bl_x", bev_src_bl_x_, bev_src_bl_x_);
    pnh_.param("bev_src_bl_y", bev_src_bl_y_, bev_src_bl_y_);
    pnh_.param("bev_src_br_x", bev_src_br_x_, bev_src_br_x_);
    pnh_.param("bev_src_br_y", bev_src_br_y_, bev_src_br_y_);
    pnh_.param("bev_src_tl_x", bev_src_tl_x_, bev_src_tl_x_);
    pnh_.param("bev_src_tl_y", bev_src_tl_y_, bev_src_tl_y_);
    pnh_.param("bev_src_tr_x", bev_src_tr_x_, bev_src_tr_x_);
    pnh_.param("bev_src_tr_y", bev_src_tr_y_, bev_src_tr_y_);
    pnh_.param("bev_dst_bl_x", bev_dst_bl_x_, bev_dst_bl_x_);
    pnh_.param("bev_dst_bl_y", bev_dst_bl_y_, bev_dst_bl_y_);
    pnh_.param("bev_dst_br_x", bev_dst_br_x_, bev_dst_br_x_);
    pnh_.param("bev_dst_br_y", bev_dst_br_y_, bev_dst_br_y_);
    pnh_.param("bev_dst_tl_x", bev_dst_tl_x_, bev_dst_tl_x_);
    pnh_.param("bev_dst_tl_y", bev_dst_tl_y_, bev_dst_tl_y_);
    pnh_.param("bev_dst_tr_x", bev_dst_tr_x_, bev_dst_tr_x_);
    pnh_.param("bev_dst_tr_y", bev_dst_tr_y_, bev_dst_tr_y_);
    pnh_.param("roi_bottom_ratio", roi_bottom_ratio_, roi_bottom_ratio_);
  }

  static bool polyfit_quadratic(const std::vector<cv::Point> &pts, cv::Vec3d &abc)
  {
    if (pts.size() < 6)
      return false;
    const int N = (int)pts.size();
    cv::Mat A(N, 3, CV_64F), x(N, 1, CV_64F);
    for (int i = 0; i < N; ++i)
    {
      double y = pts[i].y;
      A.at<double>(i, 0) = y * y;
      A.at<double>(i, 1) = y;
      A.at<double>(i, 2) = 1.0;
      x.at<double>(i, 0) = pts[i].x;
    }
    cv::Mat sol;
    if (!cv::solve(A.t() * A, A.t() * x, sol, cv::DECOMP_SVD))
      return false;
    abc = cv::Vec3d(sol.at<double>(0, 0), sol.at<double>(1, 0), sol.at<double>(2, 0));
    return true;
  }

  double slidingCenterX(const cv::Mat &bev_mask, cv::Mat &vis)
  {
    const int H = bev_mask.rows, W = bev_mask.cols;

    // 차선 마스크는 BEV 결과를 그대로 사용
    cv::Mat lane_mask = bev_mask.clone();
    cv::cvtColor(lane_mask, vis, cv::COLOR_GRAY2BGR);

    const int nwindows = 20;
    const int margin = std::max(25, (int)(W * 0.04));
    const int minpix = std::max(20, (int)(W * H / 20000.0));
    const int window_height = H / nwindows;
    const int y_gate_min = (int)(H * 0);
    const int y_preview = (int)(H * 0.60);
    const int per_win_max_shift = std::max(15, (int)(W * 0.023));
    const double lane_min = W * 0.30, lane_max = W * 0.90;
    const int split_tol = std::max(6, (int)(W * 0.009));

    cv::Mat bottom = lane_mask(cv::Rect(0, (int)(H * 0.6), W, (int)(H * 0.2)));
    cv::Mat hist;
    cv::reduce(bottom, hist, 0, cv::REDUCE_SUM, CV_32S);

    int threshold = 15; // Minimum pixels to be considered a lane

    std::vector<int> peaks;
    for (int x = 0; x < W; ++x) {
        if (hist.at<int>(0, x) > threshold) {
            int max_val = hist.at<int>(0, x);
            int best_x = x;
            int scan_x = x;
            while (scan_x < W && hist.at<int>(0, scan_x) > threshold / 3) {
                if (hist.at<int>(0, scan_x) > max_val) {
                    max_val = hist.at<int>(0, scan_x);
                    best_x = scan_x;
                }
                scan_x++;
            }
            peaks.push_back(best_x);
            x = scan_x; // skip this blob
        }
    }

    int leftx = -1;
    int rightx = -1;

    if (drive_state_ == KEEP_LANE || !have_prev_) {
        // Find the lines bracketing the midpoint
        int midpoint = W / 2;
        int max_left = -1;
        int min_right = W + 1;
        for (int p : peaks) {
            if (p <= midpoint && p > max_left) max_left = p;
            if (p > midpoint && p < min_right) min_right = p;
        }
        if (max_left != -1) leftx = max_left;
        if (min_right != W + 1) rightx = min_right;
    } else {
        // We are changing lanes. Track the expected positions!
        int expected_l = prev_left_x_;
        int expected_r = prev_right_x_;
        
        int best_dist_l = 200; // Search radius
        for (int p : peaks) {
            if (std::abs(p - expected_l) < best_dist_l) {
                best_dist_l = std::abs(p - expected_l);
                leftx = p;
            }
        }
        
        int best_dist_r = 200;
        for (int p : peaks) {
            if (std::abs(p - expected_r) < best_dist_r) {
                best_dist_r = std::abs(p - expected_r);
                rightx = p;
            }
        }
    }

    // If one line is lost (e.g. out of camera view), reconstruct it using the other line
    if (leftx == -1 && rightx != -1) leftx = rightx - last_lane_width_px_;
    if (rightx == -1 && leftx != -1) rightx = leftx + last_lane_width_px_;
    if (leftx == -1 && rightx == -1) {
        leftx = have_prev_ ? prev_left_x_ : (int)(W * 0.2);
        rightx = have_prev_ ? prev_right_x_ : (int)(W * 0.8);
    }

    std::vector<cv::Point> nz;
    cv::findNonZero(lane_mask, nz);
    double x_split = 0.5 * (leftx + rightx);

    auto clamp_shift = [&](int prev_x, int new_x)
    {
      if (prev_x < 0)
        return new_x;
      int dx = new_x - prev_x;
      if (dx > per_win_max_shift)
        return prev_x + per_win_max_shift;
      if (dx < -per_win_max_shift)
        return prev_x - per_win_max_shift;
      return new_x;
    };

    std::vector<cv::Point> left_pts, right_pts;

    for (int win = 0; win < nwindows; ++win)
    {
      int y0 = H - (win + 1) * window_height;
      int y1 = H - win * window_height;
      int lx0 = std::max(0, leftx - margin), lx1 = std::min(W - 1, leftx + margin);
      int rx0 = std::max(0, rightx - margin), rx1 = std::min(W - 1, rightx + margin);

      cv::rectangle(vis, {lx0, y0, lx1 - lx0 + 1, y1 - y0}, {0, 255, 0}, 2);
      cv::rectangle(vis, {rx0, y0, rx1 - rx0 + 1, y1 - y0}, {0, 0, 255}, 2);

      std::vector<cv::Point> Lw, Rw;
      Lw.reserve(256);
      Rw.reserve(256);

      for (const auto &p : nz)
      {
        if (p.y < y0 || p.y >= y1)
          continue;
        if (p.y < y_gate_min)
          continue;
        if (p.x < x_split - split_tol && p.x >= lx0 && p.x <= lx1)
          Lw.push_back(p);
        if (p.x > x_split + split_tol && p.x >= rx0 && p.x <= rx1)
          Rw.push_back(p);
      }

      if ((int)Lw.size() > minpix)
      {
        int sx = 0;
        for (auto &q : Lw)
          sx += q.x;
        int cand = sx / (int)Lw.size();
        leftx = clamp_shift(leftx, cand);
      }
      if ((int)Rw.size() > minpix)
      {
        int sx = 0;
        for (auto &q : Rw)
          sx += q.x;
        int cand = sx / (int)Rw.size();
        rightx = clamp_shift(rightx, cand);
      }
      left_pts.insert(left_pts.end(), Lw.begin(), Lw.end());
      right_pts.insert(right_pts.end(), Rw.begin(), Rw.end());
    }

    cv::Vec3d aL, aR;
    bool fitL = polyfit_quadratic(left_pts, aL);
    bool fitR = polyfit_quadratic(right_pts, aR);

    auto eval_poly = [](const cv::Vec3d &c, double y)
    { return c[0] * y * y + c[1] * y + c[2]; };
    auto ynear_mean = [&](const std::vector<cv::Point> &pts, int y0, int dy) -> double
    {
      long acc = 0;
      int cnt = 0;
      for (auto &p : pts)
        if (std::abs(p.y - y0) <= dy)
        {
          acc += p.x;
          ++cnt;
        }
      return (cnt > 0) ? (acc / (double)cnt) : std::numeric_limits<double>::quiet_NaN();
    };

    double xL_obs = fitL ? eval_poly(aL, y_preview) : ynear_mean(left_pts, y_preview, window_height);
    double xR_obs = fitR ? eval_poly(aR, y_preview) : ynear_mean(right_pts, y_preview, window_height);

    bool haveL = std::isfinite(xL_obs);
    bool haveR = std::isfinite(xR_obs);

    if (haveL && haveR)
    {
      if (xL_obs >= xR_obs)
      {
        if (std::abs(xL_obs - x_split) <= std::abs(xR_obs - x_split))
          xR_obs = xL_obs + last_lane_width_px_;
        else
          xL_obs = xR_obs - last_lane_width_px_;
      }
      double w = xR_obs - xL_obs;
      if (w < lane_min)
        xR_obs = xL_obs + lane_min;
      else if (w > lane_max)
        xR_obs = xL_obs + lane_max;
    }

    double xL_est = std::numeric_limits<double>::quiet_NaN();
    double xR_est = std::numeric_limits<double>::quiet_NaN();

    if (haveL && haveR)
    {
      xL_est = xL_obs;
      xR_est = xR_obs;
      double w = xR_est - xL_est;
      if (w >= lane_min && w <= lane_max && drive_state_ == KEEP_LANE)
        last_lane_width_px_ = 0.9 * last_lane_width_px_ + 0.1 * w;
      det_state_ = BOTH;
    }
    else if (haveL && !haveR)
    {
      xL_est = xL_obs;
      xR_est = xL_obs + last_lane_width_px_;
      det_state_ = LEFT_ONLY;
    }
    else if (!haveL && haveR)
    {
      xR_est = xR_obs;
      xL_est = xR_obs - last_lane_width_px_;
      det_state_ = RIGHT_ONLY;
    }
    else
    {
      det_state_ = NONE;
      lane_curve_abs_a_ = 0.0;
      cv::line(vis, {W / 2, 0}, {W / 2, H}, {180, 180, 180}, 1);
      return std::numeric_limits<double>::quiet_NaN();
    }

    if (det_state_ != NONE && std::isfinite(xL_est) && std::isfinite(xR_est))
    {
      prev_left_x_ = xL_est;
      prev_right_x_ = xR_est;
      have_prev_ = true;
    }

    double xc_preview = 0.5 * (xL_est + xR_est);
    cv::circle(vis, {(int)std::round(xc_preview), y_preview}, 5, {0, 255, 255}, -1);

    const cv::Scalar COLOR_LEFT = cv::Scalar(255, 255, 0);  // Cyan (B,G,R)
    const cv::Scalar COLOR_RIGHT = cv::Scalar(255, 0, 255); // Magenta
    const int CURVE_THICKNESS = 8;                          // 6~10 추천

    if (haveL && fitL)
    {
      std::vector<cv::Point> p_draw;
      for (int y = 0; y < H; y += 10)
        p_draw.push_back({(int)eval_poly(aL, y), y});

      cv::polylines(vis, p_draw, false, COLOR_LEFT, CURVE_THICKNESS, cv::LINE_AA);
    }

    if (haveR && fitR)
    {
      std::vector<cv::Point> p_draw;
      for (int y = 0; y < H; y += 10)
        p_draw.push_back({(int)eval_poly(aR, y), y});

      cv::polylines(vis, p_draw, false, COLOR_RIGHT, CURVE_THICKNESS, cv::LINE_AA);
    }

    if (haveL && haveR && fitL && fitR)
      lane_curve_abs_a_ = std::abs(0.5 * (aL[0] + aR[0]));
    else if (haveL && fitL)
      lane_curve_abs_a_ = std::abs(aL[0]);
    else if (haveR && fitR)
      lane_curve_abs_a_ = std::abs(aR[0]);
    else
      lane_curve_abs_a_ = 0.0;

    double raw_target_center = 0.5 * (xL_est + xR_est);
    double target_center = raw_target_center;

    if (drive_state_ == CHANGE_LEFT || drive_state_ == CHANGE_RIGHT) {
        ros::Time now = ros::Time::now();
        double dt = (now - lane_change_last_update_).toSec();
        lane_change_last_update_ = now;
        lane_change_distance_traveled_ += std::max(ego_speed_, 5.0) * dt;
        lane_change_physical_distance_ += ego_speed_ * dt;

        double current_target_distance = lane_change_target_distance_;
        if (drive_state_ == CHANGE_RIGHT) {
            current_target_distance = std::max(lane_change_target_distance_, 8.0); // Smooth return
        }
        
        const double distance = std::max(current_target_distance, 1.0);
        double ratio = std::min(1.0, lane_change_distance_traveled_ / distance);
        
        double smooth_ratio;
        if (drive_state_ == CHANGE_LEFT) {
            // Dodging: Steep-start curve for immediate aggressive steering
            smooth_ratio = ratio * (2.0 - ratio);
        } else {
            // Returning: Gentle S-curve (smoothstep) to prevent wobble
            smooth_ratio = ratio * ratio * (3.0 - 2.0 * ratio);
        }

        if (drive_state_ == CHANGE_LEFT) {
            // Track old lane and shift target to the left
            target_center = raw_target_center - last_lane_width_px_ * smooth_ratio;

            if (lane_change_physical_distance_ > current_target_distance) {
                if (have_prev_) {
                    prev_left_x_ -= last_lane_width_px_;
                    prev_right_x_ -= last_lane_width_px_;
                }
                drive_state_ = KEEP_LANE;
                current_lane_ = LEFT_LANE;
                publishCurrentLane();
                ROS_INFO("[LANE CHANGE] Arrived at Left Lane! Mission Complete!");
            }
        } else if (drive_state_ == CHANGE_RIGHT) {
            // Track old lane and shift target to the right
            target_center = raw_target_center + last_lane_width_px_ * smooth_ratio;

            if (lane_change_physical_distance_ > current_target_distance) {
                if (have_prev_) {
                    prev_left_x_ += last_lane_width_px_;
                    prev_right_x_ += last_lane_width_px_;
                }
                drive_state_ = KEEP_LANE;
                current_lane_ = RIGHT_LANE;
                publishCurrentLane();
                ROS_INFO("[LANE CHANGE] Arrived at Right Lane! Mission Complete!");
            }
        }
    }

    return target_center;
  }

  double computeSteerRad(double lane_center_x, int img_w, double curve_abs_a) const
  {
    double t = (curve_abs_a - curve_a_low_) / std::max(1e-6, curve_a_high_ - curve_a_low_);
    t = std::max(0.0, std::min(1.0, t));
    double k_deg_per_px = steer_k_straight_ + (steer_k_curve_ - steer_k_straight_) * t;

    double offset_px = lane_center_x - (img_w / 2.0);
    double steer_deg = -k_deg_per_px * offset_px;
    steer_deg = std::max(-steer_limit_deg_, std::min(steer_limit_deg_, steer_deg));
    return steer_deg * M_PI / 180.0;
  }

  void processOnce()
  {
    loadParams();
    cv::Mat frame = frame_.clone();
    if (frame.empty())
      return;

    cv::imshow("Original", frame);
    cv::Mat masked = applyROI(frame);
    cv::imshow("ROI", masked);
    cv::Mat bev = toBEV(masked);
    cv::imshow("BEV", bev);
    cv::Mat vis;
    double lane_cx = slidingCenterX(bev, vis);

    double steering_rad = 0.0;

    cv::imshow("SlidingWindow", vis);

    if (det_state_ != NONE && std::isfinite(lane_cx))
      steering_rad = computeSteerRad(lane_cx, bev.cols, lane_curve_abs_a_);

    std_msgs::Float64 steering_msg;
    steering_msg.data = steering_rad;
    steering_pub_.publish(steering_msg);

    std_msgs::Bool valid_msg;
    valid_msg.data = det_state_ != NONE && std::isfinite(lane_cx);
    valid_pub_.publish(valid_msg);

    ROS_INFO_THROTTLE(0.5, "lane steering: %.2f deg, valid: %s",
                      steering_rad * 180.0 / M_PI,
                      valid_msg.data ? "true" : "false");
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sliding_window_node");
  LineNode node;
  node.spin();
  return 0;
}
