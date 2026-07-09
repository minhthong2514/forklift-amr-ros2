import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

def generate_launch_description():
    # --- 1. PATH TO YOUR PACKAGE ---
    our_package = "car_main" # Update your package name here
    our_dir = get_package_share_directory(our_package)
    

    rviz_config_file = os.path.join(
        get_package_share_directory('car_main'), 
        'rviz', 'default_rviz.rviz'
    )

    # 2. CONFIGURATION DECLARATIONS
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')

    # List of nodes to be managed by the Lifecycle Manager
    lifecycle_nodes = [
        'controller_server', 'planner_server', 'behavior_server',
        'bt_navigator', 'waypoint_follower', 'smoother_server', 'velocity_smoother'
    ]
    
    # Remappings: It is NOT recommended to remap /tf if your system follows standard conventions
    # remappings = [] 
    remappings = [('odom', '/odometry/filtered')]
    
    # 3. PARAMETER OVERRIDES (RewrittenYaml)
    # This allows injecting LaunchConfiguration values directly into the YAML file
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_bt_xml_filename': default_bt_xml_filename,
        'autostart': autostart
    }
    
    configured_params = RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=True)
    
    crop_box_container = ComposableNodeContainer(
        name='crop_box_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='pcl_ros',
                plugin='pcl_ros::CropBox',      
                name='crop_box_filter_node',     
                parameters=[configured_params],  
                remappings=[
                    ('input', '/zed2/zed_node/point_cloud/cloud_registered'),
                    ('output', '/zed2/zed_node/point_cloud/cloud_registered_filtered')
                ]
            )
        ],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        # Set environment variable for buffered logging
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        # Default use_sim_time to false for real robot hardware execution
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(our_dir, "config", "nav2_params.yaml"),
            description='Full path to the ROS2 parameters file to use'),

        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml'),
            description='Full path to the behavior tree xml file to use'),

        # --- NAV2 NODE EXECUTION ---
        # Controller Server: Handles local path tracking and obstacle avoidance
        Node(package='nav2_controller', executable='controller_server', output='screen',
             parameters=[configured_params], remappings=remappings),

        # Planner Server: Handles global path searching from start to goal
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
             parameters=[configured_params], remappings=remappings),
        
        # Smoother Server: Refines paths to be more traversable for the robot
        Node(package='nav2_smoother', executable='smoother_server', name='smoother_server', output='screen',
             parameters=[configured_params], remappings=remappings),

        # Velocity Smoother: Dampens sudden changes in velocity commands
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother', output='screen',
             parameters=[configured_params], remappings=remappings),

        # Behavior Server: Manages recovery behaviors like spinning or reversing
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen',
             parameters=[configured_params], remappings=remappings),

        # BT Navigator: Uses Behavior Trees to orchestrate navigation tasks
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen',
             parameters=[configured_params], remappings=remappings),

        # Waypoint Follower: Coordinates moving through a series of goals
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower', output='screen',
             parameters=[configured_params], remappings=remappings),

        # Lifecycle Manager: Responsible for transitioning nodes through states (Configure, Activate, etc.)
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
             output='screen',
             parameters=[{'use_sim_time': use_sim_time},
                         {'autostart': autostart},
                         {'node_names': lifecycle_nodes}]),
        crop_box_container,
        rviz_node
    ])