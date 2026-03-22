

# ROS2 Nav2 Implementation and Setup Guide

This document outlines the step-by-step methodology for configuring and running the ROS2 Navigation 2 (Nav2) stack on a custom robot testbed. Setting up autonomous navigation requires several coordinated components: providing a map of the environment, localizing the robot within that map, and finally generating/following paths while avoiding obstacles.

Below is the complete workflow, including codebase modifications, configuration parameters, and launch files required to bring up the full navigation system.

---

## 1. Installation & Package Initialization

**Methodology:**
Before we can write custom configurations, we need to ensure the core Nav2 packages are installed on our system. Once installed, we create a dedicated ROS2 package (`testbed_navigation`) to house our specific launch scripts, parameter files, and behavior trees. Separating navigation from the core robot bringup ensures modularity.

### Install Nav2 Binaries
First, we install the Nav2 stack and its bringup utilities for the ROS2 Humble distribution:

```bash
# Install core navigation tools and standard bringup files
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

### Create the Navigation Package
Next, we generate a new Python-based package and declare our dependencies:

```bash
# Create a new ament_python package with required ROS2 geometry and navigation dependencies
ros2 pkg create testbed_navigation \
    --build-type ament_python \
    --license Apache-2.0 \
    --dependencies rclpy nav2_msgs geometry_msgs nav_msgs tf2_ros
```

---

## 2. Workspace Build Configuration

**Methodology:**
In ROS2, non-source files (like `.yaml` configurations, `.launch.py` files, and `.yaml/.pgm` map files) must be explicitly installed to the package's `share` directory. If we skip this step, ROS2 won't be able to find our configurations at runtime.

### Update `setup.py` in `testbed_navigation`
Add the following lines to the `data_files` array to ensure our launch scripts and config files are copied over during the `colcon build` process:

```python
# Add these lines inside the data_files list in setup.py
(os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
(os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
```

### Update `CMakeLists.txt` in `testbed_bringup`
Since our map files are stored in the bringup package, we need to instruct CMake to install the `maps` directory:

```cmake
# Install the maps directory to the package's share folder
install(
  DIRECTORY
    maps
  DESTINATION
    share/${PROJECT_NAME}/
)
```

---

## 3. Map Server Configuration

**Methodology:**
Nav2 operates on a known map of the environment. The `nav2_map_server` reads a saved map (YAML + image) and publishes it to the `/map` topic via a ROS2 Lifecycle Node. A Lifecycle Manager is required to transition the map server from `unconfigured` to `active`.

### Create `map_loader.launch.py`

Create this file in the `testbed_navigation/launch` directory:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Allow passing of use_sim_time from terminal, defaults to true
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Resolve the absolute path to the map YAML file
    map_file = os.path.join(
        get_package_share_directory('testbed_bringup'),
        'maps',
        'testbed_world.yaml'
    )
    
    # Define the Map Server lifecycle node
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_file},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Define the Lifecycle Manager to handle the map server's state transitions
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},               # Automatically activate the managed nodes
            {'node_names': ['map_server']}     # List of nodes to manage
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        map_server,
        lifecycle_manager
    ])
```

### Testing Map Loading

Run the following commands in separate terminals to verify the map loads correctly in RViz:

```bash
# Terminal 1: Launch the robot bringup (robot state publisher, environment, etc.)
ros2 launch testbed_bringup testbed_full_bringup.launch.py

# Terminal 2: Launch the map loader
ros2 launch testbed_navigation map_loader.launch.py
```

**Result:**
![RVIZ IMAGE WITH MAP LOADED](assets/rviz_map_loaded.png)  
*Figure: Image showing the static `/map` topic visualized in RViz.*

---

## 4. Localization (AMCL) Configuration

**Methodology:**
To navigate, the robot must know where it is. AMCL (Adaptive Monte Carlo Localization) uses particle filters to match the robot's real-time LiDAR scans against the static map. We provide specific tuning parameters (like particle counts and update thresholds) to balance CPU load with localization accuracy.

### Set AMCL Parameters (`config/amcl_params.yaml`)

```yaml
amcl:
  ros__parameters:
    use_sim_time: True
    # Odometry motion model noise parameters (differential drive)
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    
    # Frames
    base_frame_id: "base_footprint"
    global_frame_id: "map"
    odom_frame_id: "odom"
    
    # Laser model configurations
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    
    # Particle filter tuning
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    pf_err: 0.05
    pf_z: 0.99
    
    # Update conditions and frame publishing
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2  # Minimum rotational movement to trigger update
    update_min_d: 0.25 # Minimum translational movement to trigger update
    
    # Sensor likelihood weights
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan
```

### Create `localization.launch.py`

Create this file in the `testbed_navigation/launch` directory:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Path to the AMCL parameters defined above
    amcl_config = os.path.join(
        get_package_share_directory('testbed_navigation'),
        'config',
        'amcl_params.yaml'
    )
    
    # AMCL node definition
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_config]
    )
    
    # Lifecycle Manager specific to localization
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['amcl']}
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),
        amcl_node,
        lifecycle_manager
    ])
```

### Testing Localization

```bash
# Terminal 1: Bringup
ros2 launch testbed_bringup testbed_full_bringup.launch.py

# Terminal 2: Map Loader
ros2 launch testbed_navigation map_loader.launch.py

# Terminal 3: Localization (AMCL)
ros2 launch testbed_navigation localization.launch.py
```

**Result:**
![RVIZ IMAGE WITH AMCL POSE](assets/rviz_amcl_pose.png)  
*Figure: Image showing the robot pose array (particle cloud) matching the robot footprint in RViz.*

---

## 5. Navigation Stack (Nav2) Configuration

**Methodology:**
The final layer consists of the core Nav2 components:
*   **Costmaps (Global & Local)**: Generates safe/unsafe zones based on LiDAR and the static map.
*   **Planners & Controllers**: Calculate the overarching route to the goal (Planner) and generate the exact motor velocities to follow that route while avoiding dynamic obstacles (Controller).
*   **Behavior Trees (BT Navigator)**: Orchestrates the high-level logic (e.g., plan path -> follow path -> if blocked, try recovery).

### Set Nav2 Parameters (`config/nav2_params.yaml`)

*Note: This file contains settings for the DWB Local Planner, costmaps, and behavior servers. Take note of the updated velocity limits and robot radius.*

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    wait_for_service_timeout: 1000
    # Required BT plugins for actions, conditions, and controls
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_smooth_path_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_assisted_teleop_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_drive_on_heading_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_globally_updated_goal_condition_bt_node
      - nav2_is_path_valid_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_truncate_path_local_action_bt_node
      - nav2_goal_updater_node_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_path_expiring_timer_condition
      - nav2_distance_traveled_condition_bt_node
      - nav2_single_trigger_bt_node
      - nav2_goal_updated_controller_bt_node
      - nav2_is_battery_low_condition_bt_node
      - nav2_navigate_through_poses_action_bt_node
      - nav2_navigate_to_pose_action_bt_node
      - nav2_remove_passed_goals_action_bt_node
      - nav2_planner_selector_bt_node
      - nav2_controller_selector_bt_node
      - nav2_goal_checker_selector_bt_node
      - nav2_controller_cancel_bt_node
      - nav2_path_longer_on_approach_bt_node
      - nav2_wait_cancel_bt_node
      - nav2_spin_cancel_bt_node
      - nav2_back_up_cancel_bt_node
      - nav2_assisted_teleop_cancel_bt_node
      - nav2_drive_on_heading_cancel_bt_node
      - nav2_is_battery_charging_condition_bt_node

bt_navigator_navigate_through_poses_rclcpp_node:
  ros__parameters:
    use_sim_time: True

bt_navigator_navigate_to_pose_rclcpp_node:
  ros__parameters:
    use_sim_time: True

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
    
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
      
    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.5     # ← UPDATED (was 0.26)
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5  # ← UPDATED (was 0.26)
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      vx_samples: 20
      vy_samples: 5
      vtheta_samples: 20
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25
      trans_stopped_velocity: 0.25
      short_circuit_trajectory_evaluation: True
      stateful: True
      critics:["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale: 24.0
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.3  # ← UPDATED (was 0.22)
      plugins:["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      always_send_full_costmap: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.3  # ← UPDATED (was 0.22)
      resolution: 0.05
      track_unknown_space: true
      plugins:["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True

map_server:
  ros__parameters:
    use_sim_time: True
    yaml_filename: ""

map_saver:
  ros__parameters:
    use_sim_time: True
    save_map_timeout: 5.0
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65
    map_subscribe_transient_local: True

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

smoother_server:
  ros__parameters:
    use_sim_time: True
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: True

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins:["spin", "backup", "drive_on_heading", "assisted_teleop", "wait"]
    spin:
      plugin: "nav2_behaviors/Spin"
    backup:
      plugin: "nav2_behaviors/BackUp"
    drive_on_heading:
      plugin: "nav2_behaviors/DriveOnHeading"
    wait:
      plugin: "nav2_behaviors/Wait"
    assisted_teleop:
      plugin: "nav2_behaviors/AssistedTeleop"
    global_frame: odom
    robot_base_frame: base_link
    transform_tolerance: 0.1
    use_sim_time: true
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2

robot_state_publisher:
  ros__parameters:
    use_sim_time: True

waypoint_follower:
  ros__parameters:
    use_sim_time: True
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: True
      waypoint_pause_duration: 200

velocity_smoother:
  ros__parameters:
    use_sim_time: True
    smoothing_frequency: 20.0
    scale_velocities: False
    feedback: "OPEN_LOOP"
    max_velocity: [0.5, 0.0, 1.0]     # ← UPDATED (was 0.26)
    min_velocity:[-0.5, 0.0, -1.0]   # ← UPDATED (was -0.26)
    max_accel: [2.5, 0.0, 3.2]
    max_decel: [-2.5, 0.0, -3.2]
    odom_topic: "odom"
    odom_duration: 0.1
    deadband_velocity:[0.0, 0.0, 0.0]
    velocity_timeout: 1.0
```

### Create `navigation.launch.py`

Create this file in the `testbed_navigation/launch` directory:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('testbed_navigation')
    params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    
    # Shared parameters for all navigation nodes (YAML + sim time override)
    nav_params = [params_file, {'use_sim_time': True}]
    
    # Nodes that must be managed by the lifecycle manager
    lifecycle_nodes =[
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother'
    ]
    
    return LaunchDescription([
        # Controller Server (Local Planner)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=nav_params,
            remappings=[('cmd_vel', 'cmd_vel_nav')] # Output to intermediate topic
        ),
        
        # Path Smoother
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=nav_params
        ),
        
        # Planner Server (Global Path)
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=nav_params
        ),
        
        # Behavior Server (Recoveries, spin, etc.)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=nav_params
        ),
        
        # BT Navigator (Manages the /navigate_to_pose action)
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=nav_params
        ),
        
        # Waypoint Follower
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=nav_params
        ),
        
        # Velocity Smoother (Smooths final cmd_vel to robot)
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=nav_params,
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),       # Listen to intermediate topic
                ('cmd_vel_smoothed', 'cmd_vel')   # Publish actual motor commands
            ]
        ),
        
        # Lifecycle Manager (Activates everything automatically in correct order)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'autostart': True},
                {'node_names': lifecycle_nodes}
            ]
        ),
    ])
```

### Final Full-System Testing

Launch all systems in isolated terminals to watch the logging outputs and verify successful activation:

```bash
# Terminal 1: Core Robot Bringup
ros2 launch testbed_bringup testbed_full_bringup.launch.py

# Terminal 2: Static Map Loader
ros2 launch testbed_navigation map_loader.launch.py

# Terminal 3: AMCL Localization
ros2 launch testbed_navigation localization.launch.py

# Terminal 4: Core Nav2 Stack
ros2 launch testbed_navigation navigation.launch.py
```

**Result:**
Watch the system in action.

<p align="center">
  <a href="https://youtu.be/TLiT2OXBVs4">
    <img src="https://img.youtube.com/vi/TLiT2OXBVs4/maxresdefault.jpg" width="700">
  </a>
</p>

<p align="center">
Click the image to watch the demo video.
</p>
*Video: A screen recording of the robot completing an autonomous point-to-point navigation task.*
```