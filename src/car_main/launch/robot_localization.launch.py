import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # 1. Khai báo các đường dẫn
    car_main_dir = get_package_share_directory('car_main')
    car_assets_dir = get_package_share_directory('car_assets')
    lidar_pkg_dir = get_package_share_directory('sllidar_ros2')
    
    amcl_param_path = os.path.join(car_main_dir, 'config', 'amcl.yaml')
    ekf_param_path = os.path.join(car_main_dir, 'config', 'ekf.yaml')
    default_map_path = os.path.join(car_assets_dir, 'maps', 'home_map.yaml')

    # 2. Khai báo các Argument
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')
    map_file = LaunchConfiguration('map', default=default_map_path)
    
    # 3. Ghi đè tham số map vào file amcl
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_file
    }
    
    configured_params = RewrittenYaml(
        source_file=amcl_param_path,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # --- CÁC NODE PHẦN CỨNG ---
    # Micro-ROS cho Motor
    micro_ros_agent = Node(
        package='micro_ros_agent', executable='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyUSB1', '-b', '115200']
    )

    # Node LiDAR (QUAN TRỌNG: Phải có LiDAR thì AMCL mới định vị được)
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

    # Node tính toán Odom từ xung
    odom_node = Node(
        package='mecanum_odom', executable='mecanum_odom_node',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- HỆ THỐNG TF TĨNH ---
    tf_footprint_base = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )
    
    tf_base_laser = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0.2825', '0', '0.531', '3.14159', '0', '0', 'base_link', 'laser']
    )

    # --- ĐỊNH VỊ & LỌC DỮ LIỆU ---
    # EKF Lọc Odom và phát TF odom -> base_footprint
    ekf_localization = Node(
        package="robot_localization", 
        executable="ekf_node",
        name="ekf_filter_node",
        parameters=[ekf_param_path, {'use_sim_time': use_sim_time}]
    )

    # Map Server cung cấp bản đồ
    map_server_node = Node(
        package="nav2_map_server", 
        executable="map_server", 
        name = "map_server",
        parameters=[{'yaml_filename': map_file}, {'use_sim_time': use_sim_time}]
    )

    # AMCL khớp Laser vào bản đồ và phát TF map -> odom
    nav2_amcl = Node(
        package="nav2_amcl", 
        executable="amcl",
        name="amcl",
        parameters=[configured_params]
    )

    # Quản lý vòng đời Nav2
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        parameters=[{"autostart": True}, 
                    {"node_names": ['map_server', 'amcl']}] # ...phải khớp với tên ở đây
    )

    return LaunchDescription([
        micro_ros_agent,
        lidar_launch,      
        odom_node,
        tf_footprint_base,
        tf_base_laser,
        ekf_localization,
        map_server_node,
        nav2_amcl,
        lifecycle_manager
    ])