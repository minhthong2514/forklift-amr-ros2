import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. PATH DEFINITIONS ---
    # Locate the share directories for all required packages
    car_main_dir = get_package_share_directory('car_main')
    lidar_pkg_dir = get_package_share_directory('sllidar_ros2') 
    slam_toolbox_dir = get_package_share_directory('slam_toolbox') 
    view_robot_model_dir = get_package_share_directory('view_robot_model') # Path to the robot model package
    # Define paths for configuration and description files
    slam_config_file = os.path.join(car_main_dir, 'config', 'slam_params.yaml')
    nav2_params_path = os.path.join(car_main_dir, 'config', 'nav2_params.yaml')

    # Use PathJoinSubstitution for flexible path management of the URDF file
    urdf_file = PathJoinSubstitution([view_robot_model_dir, 'urdf', 'robot_model.urdf']) 

    ekf_config_path = os.path.join(car_main_dir, 'config', 'ekf.yaml')
    # Load URDF content into the robot_description parameter using the 'cat' command
    robot_description = Command(['cat ', urdf_file])

    # --- 2. ROBOT MODEL NODES (Replacing old static TFs) ---
    # The robot_state_publisher reads URDF and publishes the robot's TF tree
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    # Optional: joint_state_publisher can be used to publish non-fixed joint states if needed
    # jsp_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     parameters=[{'use_sim_time': False}]
    # )

    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': False}]
    )

    # --- 3. OPERATIONAL NODES ---
    # Node to calculate and publish odometry from wheel encoders
    odom_node = Node(
        package='mecanum_odom',
        executable='mecanum_odom_node', 
        name='mecanum_odom',
        output='screen'
    )

    # Teleoperation node launched in a separate terminal for keyboard control
    teleop_node = Node(
        package='car_controller',
        executable='teleop_cmd_vel',
        name='teleop_keyboard',
        prefix='gnome-terminal --', 
        output='screen'
    )

    # Include the Lidar driver launch file with specific hardware configurations
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_pkg_dir, 'launch', 'view_sllidar_c1_launch.py') 
        ),
        launch_arguments={
            'frame_id': 'laser', 
            'serial_port': '/dev/ttyUSB0', 
            'inverted': 'false',
            'angle_compensate': 'true'
        }.items()
    )

    # Include SLAM Toolbox for real-time mapping and localization
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_config_file, 
            'use_sim_time': 'false'               
        }.items()
    )

    # Micro-ROS Agent to bridge communication between the MCU (ESP32/STM32) and ROS 2
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyUSB1', '-b', '921600'],
        output='screen'
    )
    
    velocity_smoother_node = Node(
    package='nav2_velocity_smoother',
    executable='velocity_smoother',
    name='velocity_smoother',
    output='screen',
    parameters=[nav2_params_path], # Sử dụng file có gia tốc 0.3 m/s2
    remappings=[
        ('input_cmd_vel', '/cmd_vel'),          # Nhận từ file Python của bạn
        ('output_cmd_vel', '/cmd_vel_smoothed') # Gửi ra ESP32 của bạn
    ]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapping',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['velocity_smoother'] # Tên node cần kích hoạt
        }]
    )   
    
    # --- 4. RETURN LAUNCH DESCRIPTION ---
    # Return the list of nodes and launch actions to be executed
    return LaunchDescription([
        micro_ros_agent,
        odom_node,
        ekf_filter_node,
        rsp_node,       # Added to handle TF transforms from URDF
        # jsp_node,     # Placeholder for joint states
        teleop_node,
        lidar_launch,
        slam_launch,
        velocity_smoother_node,
        lifecycle_manager_node
    ])