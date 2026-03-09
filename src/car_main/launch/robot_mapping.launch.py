import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. ĐỊNH NGHĨA ĐƯỜNG DẪN ---
    car_main_dir = get_package_share_directory('car_main')
    lidar_pkg_dir = get_package_share_directory('sllidar_ros2') 
    slam_toolbox_dir = get_package_share_directory('slam_toolbox') 
    view_robot_model_dir = get_package_share_directory('view_robot_model') # Đường dẫn tới package model
    
    slam_config_file = os.path.join(car_main_dir, 'config', 'slam_params.yaml')
    # Đường dẫn file URDF của bạn
    urdf_file = PathJoinSubstitution([view_robot_model_dir, 'urdf', 'robot_model.urdf']) 

    # Load URDF vào biến robot_description
    robot_description = Command(['cat ', urdf_file])

    # --- 2. CÁC NODE ROBOT MODEL (Thay thế static TF cũ) ---
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}]
    )

    # jsp_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     parameters=[{'use_sim_time': False}]
    # )

    # --- 3. CÁC NODE CẦN CHẠY ---
    odom_node = Node(
        package='mecanum_odom',
        executable='mecanum_odom_node', 
        name='mecanum_odom',
        output='screen'
    )

    teleop_node = Node(
        package='car_controller',
        executable='teleop_cmd_vel',
        name='teleop_keyboard',
        prefix='gnome-terminal --', 
        output='screen'
    )

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

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_config_file, 
            'use_sim_time': 'false'               
        }.items()
    )

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyUSB1', '-b', '115200'],
        output='screen'
    )
    
    # --- 4. TRẢ VỀ ---
    return LaunchDescription([
        micro_ros_agent,
        odom_node,
        rsp_node,       # Thêm vào
        # jsp_node,       # Thêm vào
        teleop_node,
        lidar_launch,
        slam_launch
    ])