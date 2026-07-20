import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution # Added Command & PathJoinSubstitution
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # --- 1. FILE PATHS (Using PathJoinSubstitution for synchronization) ---
    car_main_dir = get_package_share_directory('car_main')
    car_assets_dir = get_package_share_directory('car_assets')
    lidar_pkg_dir = get_package_share_directory('sllidar_ros2')
    view_robot_model_dir = get_package_share_directory('view_robot_model')
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')
    crop_box_dir = get_package_share_directory("pointcloud_crop_box")
    rosbridge_dir = get_package_share_directory('rosbridge_server')

    # Define the URDF file path identical to your Mapping configuration
    xacro_file_path = PathJoinSubstitution([view_robot_model_dir, 'urdf', 'robot_model.urdf.xacro'])
    urdf_file = Command(['xacro', ' ', xacro_file_path]) 

    amcl_param_path = os.path.join(car_main_dir, 'config', 'amcl.yaml')
    ekf_param_path = os.path.join(car_main_dir, 'config', 'ekf.yaml')
    default_map_path = os.path.join(car_assets_dir, 'maps', 'map_214.yaml')
    crop_box_path = os.path.join(crop_box_dir,"config","pointcloud_crop_box_params.yaml",)

    # --- 2. CONFIGURATION ARGUMENTS ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_file = LaunchConfiguration('map', default=default_map_path)
    
    # Load URDF using the Command tool, matching the Mapping file setup
    robot_description = Command(['cat ', urdf_file])
    
    # Reconfigure AMCL parameters (override map file)
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_file
    }
    
    configured_params = RewrittenYaml(
        source_file=amcl_param_path,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # --- 3. ROBOT STATE PUBLISHER (Replaces all static TFs) ---
    # Publishes the robot's coordinate frames based on the URDF model
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': urdf_file,
            'publish_frequency': 30.0
        }]
    )

    # --- 4. HARDWARE NODES ---
    # Bridge for communication with microcontrollers via Serial
    micro_ros_agent = Node(
        package='micro_ros_agent', 
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyUSB1', '-b', '921600'],
        output='screen'
    )


    # Launch the Lidar driver
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            # os.path.join(lidar_pkg_dir, 'launch', 'view_sllidar_c1_launch.py')
            os.path.join(lidar_pkg_dir, 'launch', 'sllidar_c1_launch.py')       # dont open rviz2
        ),
        launch_arguments={
            'frame_id': 'laser', 
            'serial_port': '/dev/ttyUSB0', 
            'inverted': 'false',
            'angle_compensate': 'true'
        }.items()
    )
    
    # ZED Camera Launch: Calls official camera sensors
    zed_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_dir, 'launch', 'zed_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': 'zed2',
            'camera_name': 'zed2',  
            'publish_tf': 'false',
            'publish_map_tf': 'false',
            'common_params_path': os.path.join(car_main_dir, 'config', 'common_stereo.yaml'),
            'config_path': os.path.join(car_main_dir, 'config', 'zed2.yaml')
        }.items()
    )

    # Odometry node for Mecanum wheel calculations
    odom_node = Node(
        package='mecanum_odom', 
        executable='mecanum_odom_node',
        name='mecanum_odom',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # --- 5. LOCALIZATION & DATA FILTERING ---
    # Extended Kalman Filter for merging odometry and IMU data
    ekf_localization = Node(
        package="robot_localization", 
        executable="ekf_node",
        name="ekf_filter_node",
        output='screen', 
        parameters=[ekf_param_path, {'use_sim_time': use_sim_time}]
    )

    # Map Server: Loads and serves the static map file
    map_server_node = Node(
        package="nav2_map_server", 
        executable="map_server", 
        name="map_server",
        output='screen',
        parameters=[{'yaml_filename': map_file}, {'use_sim_time': use_sim_time}]
    )

    # AMCL: Adaptive Monte Carlo Localization for global positioning on a map
    nav2_amcl = Node(
        package="nav2_amcl", 
        executable="amcl",
        name="amcl",
        output='screen',
        parameters=[configured_params],
        remappings=[('odom', '/odometry/filtered')] 
    )
    # Manages the lifecycle states (startup/shutdown) for map_server and amcl
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        parameters=[{"autostart": True}, 
                    {"node_names": ['map_server', 'amcl']}]
    )

    # Pointcloud Crop Box
    crop_box_pointcloud = Node(
        package="pointcloud_crop_box",
        executable="pointcloud_crop_box_node",
        name="pointcloud_crop_box",
        output="screen",
        parameters=[crop_box_path],
    )

    # ROSBRIDGE WEBSOCKET SERVER
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(rosbridge_dir, 'launch', 'rosbridge_websocket_launch.xml')
        ),
        # launch_arguments={'port': '9090'}.items() 
    )

    return LaunchDescription([
        micro_ros_agent,
        rsp_node,           # Robot Model node synchronized with Mapping
        lidar_launch,
        zed_camera_launch,      
        odom_node,
        ekf_localization,
        map_server_node,
        nav2_amcl,
        lifecycle_manager,
        crop_box_pointcloud,
        rosbridge_launch
    ])