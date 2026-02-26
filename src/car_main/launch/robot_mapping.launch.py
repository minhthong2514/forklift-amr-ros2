import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. ĐỊNH NGHĨA ĐƯỜNG DẪN ---
    # Sử dụng package của bạn để lấy file cấu hình
    car_main_dir = get_package_share_directory('car_main')
    lidar_pkg_dir = get_package_share_directory('sllidar_ros2') 
    slam_toolbox_dir = get_package_share_directory('slam_toolbox') 

    # File Config SLAM
    slam_config_file = os.path.join(car_main_dir, 'config', 'slam_params.yaml')

    # --- 2. CÁC NODE CẦN CHẠY ---

    # A. Odom Node: Tính toán Odom từ xung của ESP32
    # Lưu ý: Ở bước Mapping này, node này VẪN phát TF odom -> base_footprint
    odom_node = Node(
        package='mecanum_odom',
        executable='mecanum_odom_node', 
        name='mecanum_odom',
        output='screen'
    )

    # B1. Static TF: base_footprint -> base_link (MỚI THÊM)
    # Đây là "cái bóng" của robot trên mặt đất, giúp Nav2 tính toán chuẩn hơn
    tf_footprint_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        output='screen'
    )

    # B2. Static TF: base_link -> laser (Dựa trên thông số của Tu)
    # x=0.2825 (LiDAR phía trước), z=0.531 (Độ cao), yaw=3.14159 (Xoay 180 độ)
    tf_base_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0.2825', '0', '0.531', '3.14159', '0', '0', 'base_link', 'laser'],
        output='screen'
    )

    # C. Teleop Node: Mở cửa sổ terminal riêng để Tu lái xe quét map
    teleop_node = Node(
        package='car_controller',
        executable='teleop_cmd_vel',
        name='teleop_keyboard',
        prefix='gnome-terminal --', 
        output='screen'
    )

    # D. Lidar Launch: Khởi chạy cảm biến LiDAR
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

    # E. SLAM Toolbox: Chế độ quét bản đồ Online
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_config_file, 
            'use_sim_time': 'false'               
        }.items()
    )

    # F. Micro-ROS Agent: Kết nối với ESP32
    # Thường ESP32 sẽ là USB1 nếu LiDAR đã chiếm USB0
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
        tf_footprint_base,
        tf_base_laser,
        teleop_node,
        lidar_launch,
        slam_launch
    ])