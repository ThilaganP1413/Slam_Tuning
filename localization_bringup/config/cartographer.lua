include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1, -- no of lidar
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.5,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 8
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 -- no of parts a full 360 degree scan is divided into

TRAJECTORY_BUILDER_2D.min_range = 0.5
TRAJECTORY_BUILDER_2D.max_range = 17.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0

-- LOCAL SLAM

TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 1.0
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.2

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90

TRAJECTORY_BUILDER_2D.use_imu_data = false

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.01
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(1.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 30.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 150
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 150

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 15
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 8

TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49

TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- GLOBAL SLAM

POSE_GRAPH.optimize_every_n_nodes = 100
POSE_GRAPH.constraint_builder.log_matches = false -- DEBUG LOGGING
POSE_GRAPH.log_residual_histograms = false -- DEBUG LOGGING

POSE_GRAPH.constraint_builder.max_constraint_distance = 20 -- Loop Closure Maximum Distance
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 15.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)

POSE_GRAPH.constraint_builder.sampling_ratio = 0.3

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 6

POSE_GRAPH.optimization_problem.huber_scale = 50
POSE_GRAPH.constraint_builder.min_score = 0.35

POSE_GRAPH.global_constraint_search_after_n_seconds = 30

POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 120
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 90

POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 100
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 100

POSE_GRAPH.matcher_translation_weight = 80
POSE_GRAPH.matcher_rotation_weight = 80
POSE_GRAPH.max_num_final_iterations = 1000

POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 100
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 100
POSE_GRAPH.optimization_problem.odometry_translation_weight = 150
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 150

return options