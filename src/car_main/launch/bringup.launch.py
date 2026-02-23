import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. ĐỊNH NGHĨA ĐƯỜNG DẪN ---
    car_main_dir = get_package_share_directory('car_main')
    lidar_pkg_dir = get_package_share_directory('sllidar_ros2') 
    slam_toolbox_dir = get_package_share_directory('slam_toolbox') 

    # File Config
    slam_config_file = os.path.join(car_main_dir, 'config', 'slam_params.yaml')

    # --- 2. CÁC NODE CẦN CHẠY ---

    # A. Odom Node
    odom_node = Node(
        package='mecanum_odom',
        executable='mecanum_odom_node', 
        name='mecanum_odom',
        output='screen'
    )

    # B. Static TF (Base_link -> Laser)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0.2825', '0', '0.531', '3.14159', '0', '0', 'base_link', 'laser'],
        output='screen'
    )

    # C. Teleop Node (Cửa sổ riêng)
    teleop_node = Node(
        package='car_controller',
        executable='teleop_cmd_vel',
        name='teleop_keyboard',
        prefix='gnome-terminal --', 
        output='screen'
    )

    # D. Lidar Launch (Cổng USB0)
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

    # E. SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_config_file, 
            'use_sim_time': 'false'               
        }.items()
    )

    # F. Micro-ROS Agent (MỚI THÊM)
    # Kết nối với ESP32 qua Serial
    # Lưu ý: Kiểm tra cổng USB của ESP32 (thường là USB1 nếu Lidar đã lấy USB0)
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyUSB1', '-b', '115200'],
        output='screen'
    )
    
    # --- 3. TRẢ VỀ ---
    return LaunchDescription([
        micro_ros_agent,
        odom_node,
        static_tf,
        teleop_node,
        lidar_launch,
        slam_launch
    ])