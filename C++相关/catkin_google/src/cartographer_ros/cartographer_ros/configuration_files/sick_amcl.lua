--
-- Created by IntelliJ IDEA.
-- User: ros
-- Date: 17-2-21
-- Time: 下午5:26
-- To change this template use File | Settings | File Templates.
--

include "map_builder.lua"

options = {
    map_builder = MAP_BUILDER,
    map_frame = "map",
    tracking_frame = "base_link",
    published_frame = "odom",
    odom_frame = "odom",
    provide_odom_frame = false,
    use_odometry = true,
    use_laser_scan = true,
    use_multi_echo_laser_scan = false,
    num_point_clouds = 0,
    lookup_transform_timeout_sec = 0.1,
    submap_publish_period_sec = 0.1,
    pose_publish_period_sec = 5e-3,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 7
MAP_BUILDER.sparse_pose_graph.optimization_problem.huber_scale = 5e2
MAP_BUILDER.sparse_pose_graph.optimize_every_n_scans = 40
MAP_BUILDER.sparse_pose_graph.constraint_builder.sampling_ratio = 0.03

TRAJECTORY_BUILDER_2D.laser_min_range = 0.1
TRAJECTORY_BUILDER_2D.laser_max_range = 50.
TRAJECTORY_BUILDER_2D.laser_missing_echo_ray_length = 5.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

SPARSE_POSE_GRAPH.constraint_builder.min_score = 0.3
SPARSE_POSE_GRAPH.constraint_builder.global_localization_min_score = 0.3
SPARSE_POSE_GRAPH.optimization_problem.huber_scale = 1e2

return options

