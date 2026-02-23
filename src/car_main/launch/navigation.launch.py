import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. ĐỊNH NGHĨA ĐƯỜNG DẪN ---
    car_main_dir = get_package_share_directory('car_main')
    car_assets_dir = get_package_share_directory('car_assets') 
    lidar_pkg_dir = get_package_share_directory('sllidar_ros2') 
    nav2_bringup_dir = get_package_share_directory('nav2_bringup') 

    # File Config & Map
    nav2_params_file = os.path.join(car_main_dir, 'config', 'nav2_params.yaml')
    # Trỏ thẳng vào package car_assets theo cấu trúc của bạn
    map_file = os.path.join(car_assets_dir, 'maps', 'map.yaml') 

    # --- 2. CÁC NODE CẦN CHẠY ---

    # A. Micro-ROS Agent
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyUSB1', '-b', '115200'],
        output='screen'
    )

    # B. Odom Node
    odom_node = Node(
        package='mecanum_odom',
        executable='mecanum_odom_node', 
        name='mecanum_odom',
        output='screen'
    )

    # C. Static TF (base_link -> laser)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0.2825', '0', '0.531', '3.14159', '0', '0', 'base_link', 'laser'],
        output='screen'
    )

    # D. Lidar Launch
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

    # E. Nav2 Bringup (Định vị AMCL + Điều khiển Nav2)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'false',
            'params_file': nav2_params_file
        }.items()
    )
    
    return LaunchDescription([
        micro_ros_agent,
        odom_node,
        static_tf,
        lidar_launch,
        nav2_launch
    ])