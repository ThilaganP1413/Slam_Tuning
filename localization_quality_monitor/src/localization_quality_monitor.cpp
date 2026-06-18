// localization_quality_monitor.cpp
//
// Monitors localization quality by scan-matching live LiDAR data against a
// precomputed Euclidean distance-transform of a static 2D occupancy grid.
// Publishes unified "Inlier Fraction", "RMSE", "Covariance Degradation",
// "TF Jump", and an overall 0-100 quality score as a std_msgs/String.

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Dense>

#include <algorithm>   // for std::min, std::max
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <deque>

using std::placeholders::_1;

class LocalizationQualityMonitor : public rclcpp::Node
{
public:
  LocalizationQualityMonitor()
  : Node("localization_quality_monitor")
  {
    inlier_threshold_ = this->declare_parameter<double>("inlier_threshold", 0.2);
    occupancy_threshold_ = this->declare_parameter<int>("occupancy_threshold", 50);
    max_scan_points_ =
      static_cast<size_t>(this->declare_parameter<int>("max_scan_points", 2400));
    map_topic_ = this->declare_parameter<std::string>("map_topic", "/map");
    scan_topic_ = this->declare_parameter<std::string>("scan_topic", "/scan");
    pose_topic_ = this->declare_parameter<std::string>("pose_topic", "/pose");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    ema_alpha_ = this->declare_parameter<double>("ema_alpha", 0.1);
    ema_beta_ = this->declare_parameter<double>("ema_beta", 0.0005);
    tf_jump_window_ = this->declare_parameter<double>("tf_jump_window", 5.0);

    // New thresholds for scoring
    max_rmse_ = this->declare_parameter<double>("max_rmse", 0.5);
    max_cov_degradation_ = this->declare_parameter<double>("max_cov_degradation", 5.0);
    max_trans_jump_rate_ = this->declare_parameter<double>("max_trans_jump_rate", 1.0);

    ensureCapacity(max_scan_points_);
    transform_matrix_.setIdentity();

    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.transient_local();
    map_qos.reliable();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      std::bind(&LocalizationQualityMonitor::mapCallback, this, _1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationQualityMonitor::scanCallback, this, _1));

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic_, rclcpp::QoS(10),
      std::bind(&LocalizationQualityMonitor::poseCallback, this, _1));

    // Unified publisher
    quality_pub_ = this->create_publisher<std_msgs::msg::String>("/localization_quality", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(
      this->get_logger(),
      "Localization Quality Monitor started. Waiting for '%s' (transient local)...",
      map_topic_.c_str());
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (map_ready_) {
      return;
    }

    map_width_ = static_cast<int>(msg->info.width);
    map_height_ = static_cast<int>(msg->info.height);
    map_resolution_ = static_cast<double>(msg->info.resolution);
    map_origin_x_ = msg->info.origin.position.x;
    map_origin_y_ = msg->info.origin.position.y;
    map_frame_ = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;

    if (map_width_ <= 0 || map_height_ <= 0 || map_resolution_ <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "Received an invalid /map message; ignoring it.");
      return;
    }

    cv::Mat binary(map_height_, map_width_, CV_8UC1);
    const size_t total_cells =
      static_cast<size_t>(map_width_) * static_cast<size_t>(map_height_);
    uint8_t * dst = binary.data;
    const int8_t * src = msg->data.data();
    for (size_t i = 0; i < total_cells; ++i) {
      dst[i] = (static_cast<int>(src[i]) >= occupancy_threshold_) ? 0 : 255;
    }

    cv::distanceTransform(binary, distance_transform_, cv::DIST_L2, 5);

    map_ready_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "Map processed (%dx%d @ %.4f m/px, frame '%s'). Distance transform ready.",
      map_width_, map_height_, map_resolution_, map_frame_.c_str());
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!map_ready_) {
      return;
    }

    // --- 1. Calculate Map -> Odom Jumps (N-Second Average Window) ---
    try {
      geometry_msgs::msg::TransformStamped mo_tf_msg =
        tf_buffer_->lookupTransform(map_frame_, odom_frame_, tf2::TimePointZero);

      const Eigen::Isometry3d mo_iso = tf2::transformToEigen(mo_tf_msg);
      const double curr_mo_yaw = std::atan2(mo_iso.rotation()(1, 0), mo_iso.rotation()(0, 0));
      const double curr_mo_x = mo_iso.translation().x();
      const double curr_mo_y = mo_iso.translation().y();

      rclcpp::Time now = this->get_clock()->now();

      if (has_prev_map_odom_) {
        double dx = curr_mo_x - prev_mo_x_;
        double dy = curr_mo_y - prev_mo_y_;
        double dyaw = curr_mo_yaw - prev_mo_yaw_;
        dyaw = std::atan2(std::sin(dyaw), std::cos(dyaw));

        // If transform physically moved, record the jump
        if (std::abs(dx) > 1e-6 || std::abs(dy) > 1e-6 || std::abs(dyaw) > 1e-6) {
          double trans_jump = std::sqrt(dx * dx + dy * dy);
          double rot_jump = std::abs(dyaw);

          jump_history_.push_back({now, trans_jump, rot_jump});

          prev_mo_x_ = curr_mo_x;
          prev_mo_y_ = curr_mo_y;
          prev_mo_yaw_ = curr_mo_yaw;
        }
      } else {
        prev_mo_x_ = curr_mo_x;
        prev_mo_y_ = curr_mo_y;
        prev_mo_yaw_ = curr_mo_yaw;
        has_prev_map_odom_ = true;
      }

      // Process the Sliding Window
      rclcpp::Time cutoff = now - rclcpp::Duration::from_seconds(tf_jump_window_);

      // Pop old jump records that fall outside the time window
      while (!jump_history_.empty() && jump_history_.front().stamp < cutoff) {
        jump_history_.pop_front();
      }

      // Sum all remaining jumps in the window
      double sum_trans = 0.0;
      double sum_rot = 0.0;
      for (const auto & record : jump_history_) {
        sum_trans += record.trans_jump;
        sum_rot += record.rot_jump;
      }

      // Calculate the average jump rate (meters/sec and rad/sec) over the N seconds
      current_trans_jump_rate_ = sum_trans / tf_jump_window_;
      current_rot_jump_rate_ = sum_rot / tf_jump_window_;

    } catch (const tf2::TransformException & ex) {
      // It's okay if odom isn't available yet
    }

    // --- 2. Calculate Scan Matching ---
    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_->lookupTransform(map_frame_, msg->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "TF lookup %s -> %s failed: %s", map_frame_.c_str(),
        msg->header.frame_id.c_str(), ex.what());
      return;
    }

    const Eigen::Isometry3d iso = tf2::transformToEigen(tf_msg);
    const double yaw = std::atan2(iso.rotation()(1, 0), iso.rotation()(0, 0));
    const float cos_yaw = static_cast<float>(std::cos(yaw));
    const float sin_yaw = static_cast<float>(std::sin(yaw));
    const float tx = static_cast<float>(iso.translation().x());
    const float ty = static_cast<float>(iso.translation().y());

    transform_matrix_(0, 0) = cos_yaw;
    transform_matrix_(0, 1) = -sin_yaw;
    transform_matrix_(0, 2) = tx;
    transform_matrix_(1, 0) = sin_yaw;
    transform_matrix_(1, 1) = cos_yaw;
    transform_matrix_(1, 2) = ty;

    updateAngleCache(msg);

    const size_t n = msg->ranges.size();
    const float range_min = msg->range_min;
    const float range_max = msg->range_max;

    size_t valid_count = 0;
    for (size_t i = 0; i < n; ++i) {
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < range_min || r > range_max) {
        continue;
      }
      scan_homogeneous_(0, valid_count) = r * range_cos_cache_[i];
      scan_homogeneous_(1, valid_count) = r * range_sin_cache_[i];
      scan_homogeneous_(2, valid_count) = 1.0f;
      ++valid_count;
    }

    if (valid_count == 0) {
      return;
    }

    transformed_points_.leftCols(valid_count).noalias() =
      transform_matrix_ * scan_homogeneous_.leftCols(valid_count);

    const float inv_res = static_cast<float>(1.0 / map_resolution_);
    const float origin_x = static_cast<float>(map_origin_x_);
    const float origin_y = static_cast<float>(map_origin_y_);

    size_t rays_with_lookup = 0;
    size_t inlier_count = 0;
    double sum_sq_dist = 0.0;

    for (size_t j = 0; j < valid_count; ++j) {
      const float u = (transformed_points_(0, j) - origin_x) * inv_res;
      const float v = (transformed_points_(1, j) - origin_y) * inv_res;

      float pixel_dist;
      if (!bilinearLookup(distance_transform_, u, v, pixel_dist)) {
        continue;
      }

      ++rays_with_lookup;
      const double d = static_cast<double>(pixel_dist) * map_resolution_;
      if (d <= inlier_threshold_) {
        ++inlier_count;
        sum_sq_dist += d * d;
      }
    }

    const double inlier_fraction = (rays_with_lookup > 0)
      ? static_cast<double>(inlier_count) / static_cast<double>(rays_with_lookup)
      : 0.0;
    const double rmse = (inlier_count > 0)
      ? std::sqrt(sum_sq_dist / static_cast<double>(inlier_count))
      : 0.0;

    // --- 3. Compute 0–100 scores for each metric ---
    // Inlier Score (already percentage)
    double inlier_score = inlier_fraction * 100.0;

    // RMSE Score
    double rmse_score = 0.0;
    if (max_rmse_ > 0.0) {
      rmse_score = 100.0 * (1.0 - (rmse / max_rmse_));
    } else {
      rmse_score = (rmse == 0.0) ? 100.0 : 0.0;
    }
    rmse_score = std::clamp(rmse_score, 0.0, 100.0);

    // Covariance Score
    double cov_score = 0.0;
    if (max_cov_degradation_ > 1.0) {
      cov_score = 100.0 * (1.0 - ((current_degradation_ratio_ - 1.0) / (max_cov_degradation_ - 1.0)));
    } else if (max_cov_degradation_ == 1.0) {
      cov_score = (current_degradation_ratio_ <= 1.0) ? 100.0 : 0.0;
    } else {
      cov_score = 100.0; // fallback
    }
    cov_score = std::clamp(cov_score, 0.0, 100.0);

    // Jump Score (using translational jump rate)
    double jump_score = 0.0;
    if (max_trans_jump_rate_ > 0.0) {
      jump_score = 100.0 * (1.0 - (current_trans_jump_rate_ / max_trans_jump_rate_));
    } else {
      jump_score = (current_trans_jump_rate_ == 0.0) ? 100.0 : 0.0;
    }
    jump_score = std::clamp(jump_score, 0.0, 100.0);

    // --- 4. Weighted average (weights: Jump 20%, Inlier 35%, RMSE 30%, Cov 15%) ---
    const double w_jump = 0.20;
    const double w_inlier = 0.35;
    const double w_rmse = 0.30;
    const double w_cov = 0.15;
    double base_weighted_score = w_jump * jump_score +
                                 w_inlier * inlier_score +
                                 w_rmse * rmse_score +
                                 w_cov * cov_score;

    // --- 5. Safety veto based on Jump Score ---
    double veto_multiplier = std::min(1.0, jump_score / 50.0);
    double final_quality_score = base_weighted_score * veto_multiplier;

    // --- 6. Publish Unified Message ---
    char buf[512];
    std::snprintf(buf, sizeof(buf),
      "Overall Quality: %.1f%% | Inlier Fraction: %.1f%% | RMSE: %.3fm | Cov Uncertainity: %.1f | TF Avg Jump: %.3fm/s, %.1fdeg/s",
      final_quality_score,
      inlier_fraction * 100.0,
      rmse,
      current_degradation_ratio_,
      current_trans_jump_rate_,
      current_rot_jump_rate_ * 180.0 / M_PI);

    std_msgs::msg::String quality_msg;
    quality_msg.data = buf;
    quality_pub_->publish(quality_msg);
  }

  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    const auto & cov = msg->pose.covariance;
    const double t_curr = cov[0] + cov[7] + cov[35];

    if (T_base_ < 0.0) {
      T_base_ = t_curr;
    } else if (t_curr < T_base_) {
      T_base_ = (ema_alpha_ * t_curr) + ((1.0 - ema_alpha_) * T_base_);
    } else {
      T_base_ = (ema_beta_ * t_curr) + ((1.0 - ema_beta_) * T_base_);
    }

    current_degradation_ratio_ = (T_base_ > 1e-12) ? (t_curr / T_base_) : 1.0;
  }

  void ensureCapacity(size_t required_points)
  {
    if (static_cast<Eigen::Index>(required_points) <= scan_homogeneous_.cols()) {
      return;
    }
    const size_t new_capacity = required_points + (required_points / 10);
    scan_homogeneous_.resize(3, new_capacity);
    transformed_points_.resize(3, new_capacity);
    range_cos_cache_.resize(new_capacity);
    range_sin_cache_.resize(new_capacity);
    max_scan_points_ = new_capacity;
    cached_num_points_ = 0;
  }

  void updateAngleCache(const sensor_msgs::msg::LaserScan::SharedPtr & msg)
  {
    const size_t n = msg->ranges.size();
    ensureCapacity(n);

    constexpr float kAngleEpsilon = 1e-5f;
    if (n == cached_num_points_ &&
      std::fabs(msg->angle_min - cached_angle_min_) < kAngleEpsilon &&
      std::fabs(msg->angle_increment - cached_angle_increment_) < kAngleEpsilon)
    {
      return;
    }

    for (size_t i = 0; i < n; ++i) {
      const float angle = msg->angle_min + static_cast<float>(i) * msg->angle_increment;
      range_cos_cache_[i] = std::cos(angle);
      range_sin_cache_[i] = std::sin(angle);
    }

    cached_num_points_ = n;
    cached_angle_min_ = msg->angle_min;
    cached_angle_increment_ = msg->angle_increment;
  }

  static inline bool bilinearLookup(const cv::Mat & dist_mat, float u, float v, float & out_dist)
  {
    if (u < 0.0f || v < 0.0f || u >= dist_mat.cols - 1 || v >= dist_mat.rows - 1) {
      return false;
    }
    const int u0 = static_cast<int>(u);
    const int v0 = static_cast<int>(v);
    const float du = u - static_cast<float>(u0);
    const float dv = v - static_cast<float>(v0);

    const float * row0 = dist_mat.ptr<float>(v0);
    const float * row1 = dist_mat.ptr<float>(v0 + 1);

    const float top = row0[u0] * (1.0f - du) + row0[u0 + 1] * du;
    const float bottom = row1[u0] * (1.0f - du) + row1[u0 + 1] * du;

    out_dist = top * (1.0f - dv) + bottom * dv;
    return true;
  }

  // Define a struct to hold historical jump data
  struct JumpRecord {
    rclcpp::Time stamp;
    double trans_jump;
    double rot_jump;
  };

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

  // Unified Publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr quality_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  cv::Mat distance_transform_;
  bool map_ready_{false};
  double map_resolution_{0.05};
  double map_origin_x_{0.0};
  double map_origin_y_{0.0};
  int map_width_{0};
  int map_height_{0};
  std::string map_frame_{"map"};

  double inlier_threshold_{0.2};
  int occupancy_threshold_{50};
  size_t max_scan_points_{2400};
  std::string map_topic_{"/map"};
  std::string scan_topic_{"/scan"};
  std::string pose_topic_{"/pose"};
  std::string odom_frame_{"odom"};
  double ema_alpha_{0.1};
  double ema_beta_{0.0005};
  double tf_jump_window_{5.0};

  // New scoring parameters
  double max_rmse_{0.5};
  double max_cov_degradation_{5.0};
  double max_trans_jump_rate_{1.0};

  // State for Covariance
  double T_base_{-1.0};
  double current_degradation_ratio_{1.0};

  // State for TF Jumps (Moving Average)
  bool has_prev_map_odom_{false};
  double prev_mo_x_{0.0};
  double prev_mo_y_{0.0};
  double prev_mo_yaw_{0.0};
  std::deque<JumpRecord> jump_history_;
  double current_trans_jump_rate_{0.0};
  double current_rot_jump_rate_{0.0};

  std::vector<float> range_cos_cache_;
  std::vector<float> range_sin_cache_;
  Eigen::Matrix3Xf scan_homogeneous_;
  Eigen::Matrix3Xf transformed_points_;
  Eigen::Matrix3f transform_matrix_;

  float cached_angle_min_{std::numeric_limits<float>::quiet_NaN()};
  float cached_angle_increment_{std::numeric_limits<float>::quiet_NaN()};
  size_t cached_num_points_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationQualityMonitor>());
  rclcpp::shutdown();
  return 0;
}