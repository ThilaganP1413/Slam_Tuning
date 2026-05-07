-- Cartographer Configuration: PURE SCAN MATCHING MODE
-- For environments with BROKEN/UNRELIABLE odometry
-- Relies entirely on laser scan matching

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,  -- CHANGED: Cartographer will provide odom
  publish_frame_projected_to_2d = false,
  use_odometry = false,  -- CRITICAL: Disable broken odometry!
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.5,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 0.,  -- Ignore odometry completely
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 16

-- ==============================================================================
-- LOCAL SLAM: Aggressive scan-to-scan matching
-- ==============================================================================

-- Accumulate more scans for robust matching
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2

-- Medium-sized submaps
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90

TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 100.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- ENABLE and STRENGTHEN real-time correlative scan matching
-- This is now our PRIMARY localization method
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.25
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(40.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.

-- CERES SCAN MATCHER: Now primary position estimator
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.  -- Trust scan matching heavily
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.  -- No odometry to weight against
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1.  -- No odometry to weight against
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 30
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 8

-- High-quality map building
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.7
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- Keep maximum scan detail for matching
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.015
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.4
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 300  -- High point count
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 50.

-- Permissive motion filter - process all scans
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)

-- ==============================================================================
-- GLOBAL SLAM: CRITICAL for scan-only SLAM
-- ==============================================================================

-- Optimize very frequently to correct drift
POSE_GRAPH.optimize_every_n_nodes = 45

-- AGGRESSIVE loop closure for repeated structures
POSE_GRAPH.constraint_builder.min_score = 0.48  -- Very permissive
POSE_GRAPH.constraint_builder.sampling_ratio = 0.15  -- Check many candidates
POSE_GRAPH.constraint_builder.max_constraint_distance = 25.  -- Look far
POSE_GRAPH.constraint_builder.log_matches = true

-- Very frequent global search
POSE_GRAPH.global_constraint_search_after_n_seconds = 3.

-- FAST CORRELATIVE SCAN MATCHER: Critical for loop detection
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 8
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 12.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(45.)

-- Constraint builder Ceres
POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 30.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 15.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 5.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 40
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 8

-- Strong loop closure corrections
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 2e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 3e5

-- ==============================================================================
-- POSE GRAPH OPTIMIZATION: Trust scan matching and loop closures
-- ==============================================================================

POSE_GRAPH.optimization_problem.huber_scale = 1e1

-- Local SLAM poses: Primary constraint
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e6
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e6

-- ODOMETRY: ZERO WEIGHT (disabled)
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0.
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0.

-- Fixed frame
POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1e1
POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 1e2

-- Solver
POSE_GRAPH.optimization_problem.ceres_solver_options.use_nonmonotonic_steps = false
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 100
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 7

-- Enable detailed logging
POSE_GRAPH.optimization_problem.log_solver_summary = true

-- Strong final optimization
POSE_GRAPH.matcher_translation_weight = 2e3
POSE_GRAPH.matcher_rotation_weight = 5e3
POSE_GRAPH.max_num_final_iterations = 500

return options