// localization_quality_monitor.cpp
//
// Monitors localization quality by scan-matching live LiDAR data against a
// precomputed Euclidean distance-transform of a static 2D occupancy grid.
// Publishes "Inlier Fraction" and "RMSE" as std_msgs/String at scan rate.
//
// Design notes (read before modifying):
//  * The map is processed exactly once. On receipt of the first /map message
//    we build a binary cv::Mat (occupied=0, free=255) and run
//    cv::distanceTransform(DIST_L2, mask=5) to get, for every map cell, the
//    sub-pixel-accurate Euclidean distance (in pixels) to the nearest
//    occupied cell. We then drop the /map subscription.
//  * The /scan callback never allocates heap memory in steady state. Eigen
//    matrices and std::vectors used as scratch space are sized once
//    (constructor + capacity-growth path) and reused every callback via
//    .leftCols(n) views.
//  * The map -> laser transform is fetched as the single *latest* available
//    transform (tf2::TimePointZero) rather than waiting on the scan's
//    timestamp, since this is a monitoring/diagnostic node, not a control
//    loop -- we'd rather get a slightly-stale answer immediately than block
//    the 20 Hz callback waiting for TF interpolation.
//  * The map->laser transform is reduced to a single 2D (yaw + xy) rigid
//    transform, consistent with the 2D occupancy-grid / planar-scan
//    assumption used throughout. If your robot has significant roll/pitch
//    at the laser, this is an approximation (the yaw component of the full
//    3D rotation is extracted via atan2, the standard projection used by
//    tf2::getYaw).
//  * Thread-safety: this node assumes the default single-threaded executor.
//    map_ready_/distance_transform_ are written once in mapCallback and read
//    repeatedly in scanCallback; with a multi-threaded executor you would
//    need to guard the handover with a mutex or atomic flag + memory fence.

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

// Included per the requested dependency list. The core pipeline below builds
// the cv::Mat directly from OccupancyGrid::data (no sensor_msgs::Image is
// ever involved), so cv_bridge is not exercised by the hot path. It's kept
// available/declared as a dependency in case you want to extend this node
// with a debug publisher that visualizes the distance-transform field as an
// Image topic.
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Dense>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <vector>

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

    // --- Pre-allocate every scratch buffer used by the hot (scan) path. ---
    // ensureCapacity() is only called again inside scanCallback() as a cheap
    // size check; it only *resizes* (i.e. allocates) if a scan ever exceeds
    // this capacity, which should not happen in steady state if
    // 'max_scan_points' is set sensibly for your LiDAR.
    ensureCapacity(max_scan_points_);
    transform_matrix_.setIdentity();

    // --- Map subscription: transient_local + reliable mimics a latched topic ---
    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.transient_local();
    map_qos.reliable();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      std::bind(&LocalizationQualityMonitor::mapCallback, this, _1));

    // --- Scan subscription: best-effort sensor data QoS, high frequency ---
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationQualityMonitor::scanCallback, this, _1));

    inlier_pub_ =
      this->create_publisher<std_msgs::msg::String>("/localization_inlier_fraction", 10);
    rmse_pub_ = this->create_publisher<std_msgs::msg::String>("/localization_rmse", 10);

    // --- TF2 listener (standard ROS 2 pattern: Buffer + TransformListener) ---
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(
      this->get_logger(),
      "Localization Quality Monitor started. Waiting for '%s' (transient local)...",
      map_topic_.c_str());
  }

private:
  // ---------------------------------------------------------------------
  // Map processing: runs exactly once, on the first /map message.
  // ---------------------------------------------------------------------
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (map_ready_) {
      return;  // Should not normally fire again since we unsubscribe below.
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

    // Binary mask: occupied cells = 0, free space = 255 (per spec, this is
    // exactly the polarity cv::distanceTransform wants: it reports, for
    // every non-zero pixel, the distance to the nearest zero pixel).
    // Unknown cells (-1) are treated as free; only cells whose occupancy
    // probability is >= occupancy_threshold_ count as obstacles.
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
      "Map processed (%dx%d @ %.4f m/px, frame '%s'). Distance transform ready; "
      "unsubscribing from '%s'.",
      map_width_, map_height_, map_resolution_, map_frame_.c_str(), map_topic_.c_str());

    // Run once, per spec: drop the subscription to save overhead.
    map_sub_.reset();
  }

  // ---------------------------------------------------------------------
  // Scan processing: runs at sensor rate (e.g. 20 Hz). No heap allocation
  // on this path in steady state.
  // ---------------------------------------------------------------------
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!map_ready_) {
      return;
    }

    // --- TF lookup: latest available map -> laser_frame transform ---
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

    // --- Build a 3x3 homogeneous 2D transform from the 3D isometry ---
    // (tf2_eigen gives us rotation()/translation() directly; we extract the
    // yaw component the standard way, which collapses to an exact pure-Z
    // rotation when the transform truly is planar.)
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
    // Row 2 is fixed at [0, 0, 1] and never touched after setIdentity().

    // --- Eigen vectorization: ranges+angles -> 3xN homogeneous matrix ---
    updateAngleCache(msg);  // Also grows buffers if this scan is unusually large.

    const size_t n = msg->ranges.size();
    const float range_min = msg->range_min;
    const float range_max = msg->range_max;

    size_t valid_count = 0;
    for (size_t i = 0; i < n; ++i) {
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < range_min || r > range_max) {
        continue;  // Filters out NaN/Inf as well as out-of-spec ranges.
      }
      scan_homogeneous_(0, valid_count) = r * range_cos_cache_[i];
      scan_homogeneous_(1, valid_count) = r * range_sin_cache_[i];
      scan_homogeneous_(2, valid_count) = 1.0f;
      ++valid_count;
    }

    if (valid_count == 0) {
      return;  // Nothing to score this cycle.
    }

    // --- Transformation: single Eigen matrix multiplication, whole scan ---
    transformed_points_.leftCols(valid_count).noalias() =
      transform_matrix_ * scan_homogeneous_.leftCols(valid_count);

    // --- O(1) per-point distance lookup with bilinear interpolation ---
    const float inv_res = static_cast<float>(1.0 / map_resolution_);
    const float origin_x = static_cast<float>(map_origin_x_);
    const float origin_y = static_cast<float>(map_origin_y_);

    size_t rays_with_lookup = 0;  // Rays for which a distance was computable.
    size_t inlier_count = 0;
    double sum_sq_dist = 0.0;

    for (size_t j = 0; j < valid_count; ++j) {
      const float u = (transformed_points_(0, j) - origin_x) * inv_res;
      const float v = (transformed_points_(1, j) - origin_y) * inv_res;

      float pixel_dist;
      if (!bilinearLookup(distance_transform_, u, v, pixel_dist)) {
        continue;  // Point fell outside the map; excluded from both totals.
      }

      ++rays_with_lookup;
      const double d = static_cast<double>(pixel_dist) * map_resolution_;
      if (d <= inlier_threshold_) {
        ++inlier_count;
        sum_sq_dist += d * d;
      }
    }

    // --- Metrics ---
    const double inlier_fraction = (rays_with_lookup > 0)
      ? static_cast<double>(inlier_count) / static_cast<double>(rays_with_lookup)
      : 0.0;
    const double rmse = (inlier_count > 0)
      ? std::sqrt(sum_sq_dist / static_cast<double>(inlier_count))
      : 0.0;

    // --- Publish (snprintf into stack buffers; avoids <sstream> overhead) ---
    char buf[64];
    std_msgs::msg::String inlier_msg;
    std::snprintf(buf, sizeof(buf), "Inlier Fraction: %.1f%%", inlier_fraction * 100.0);
    inlier_msg.data = buf;
    inlier_pub_->publish(inlier_msg);

    std_msgs::msg::String rmse_msg;
    std::snprintf(buf, sizeof(buf), "RMSE: %.3fm", rmse);
    rmse_msg.data = buf;
    rmse_pub_->publish(rmse_msg);
  }

  // ---------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------

  // Grows the pre-allocated scratch buffers if (and only if) a scan turns
  // out to be larger than what we've reserved. This should be a one-time
  // (ideally zero-time, if max_scan_points is set correctly) event, never a
  // per-callback occurrence.
  void ensureCapacity(size_t required_points)
  {
    if (static_cast<Eigen::Index>(required_points) <= scan_homogeneous_.cols()) {
      return;
    }
    RCLCPP_WARN(
      this->get_logger(),
      "Growing pre-allocated scan buffers from %ld to %zu columns "
      "(set the 'max_scan_points' parameter to your LiDAR's point count "
      "to avoid this reallocation).",
      static_cast<long>(scan_homogeneous_.cols()), required_points);
    scan_homogeneous_.resize(3, required_points);
    transformed_points_.resize(3, required_points);
    range_cos_cache_.resize(required_points);
    range_sin_cache_.resize(required_points);
    max_scan_points_ = required_points;
    cached_num_points_ = 0;  // Force the angle cache to rebuild below.
  }

  // Rebuilds the per-index cos/sin cache only when the scan's angular
  // geometry actually changes (almost never, for a fixed LiDAR), so that
  // std::cos/std::sin are not recomputed for every point on every callback.
  void updateAngleCache(const sensor_msgs::msg::LaserScan::SharedPtr & msg)
  {
    const size_t n = msg->ranges.size();
    ensureCapacity(n);

    if (n == cached_num_points_ &&
      msg->angle_min == cached_angle_min_ &&
      msg->angle_increment == cached_angle_increment_)
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

  // O(1), allocation-free bilinear lookup into the float distance-transform
  // matrix. Returns false (and leaves out_dist untouched) if (u, v) falls
  // outside the region where all four interpolation neighbors exist.
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

  // --- Subscriptions / Publishers ---
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr inlier_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rmse_pub_;

  // --- TF2 ---
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // --- Map data, set once in mapCallback() ---
  cv::Mat distance_transform_;  // CV_32F: pixel distance to nearest occupied cell.
  bool map_ready_{false};
  double map_resolution_{0.05};
  double map_origin_x_{0.0};
  double map_origin_y_{0.0};
  int map_width_{0};
  int map_height_{0};
  std::string map_frame_{"map"};

  // --- Parameters ---
  double inlier_threshold_{0.2};
  int occupancy_threshold_{50};
  size_t max_scan_points_{2400};
  std::string map_topic_{"/map"};
  std::string scan_topic_{"/scan"};

  // --- Pre-allocated hot-path scratch space (sized by ensureCapacity) ---
  std::vector<float> range_cos_cache_;
  std::vector<float> range_sin_cache_;
  Eigen::Matrix3Xf scan_homogeneous_;
  Eigen::Matrix3Xf transformed_points_;
  Eigen::Matrix3f transform_matrix_;

  // --- Angle-cache invalidation state ---
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