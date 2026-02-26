import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # --- 1. ĐƯỜNG DẪN ĐẾN PACKAGE CỦA BẠN ---
    our_package = "car_main" # Sửa tên package của bạn ở đây
    our_dir = get_package_share_directory(our_package)
    
    # 2. KHAI BÁO CÁC CẤU HÌNH
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')

    # Danh sách các node cần quản lý vòng đời (Lifecycle)
    lifecycle_nodes = [
        'controller_server', 'planner_server', 'behavior_server',
        'bt_navigator', 'waypoint_follower', 'smoother_server', 'velocity_smoother'
    ]
    
    # KHÔNG NÊN Remap /tf nếu hệ thống của bạn đang chạy chuẩn
    remappings = [] 
    
    # 3. GHI ĐÈ THAM SỐ (RewrittenYaml)
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_bt_xml_filename': default_bt_xml_filename,
        'autostart': autostart
    }
    
    configured_params = RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=True)
    
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        # Đổi use_sim_time về false vì Tu chạy robot thật
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

        # --- KHỞI CHẠY CÁC NODE NAV2 ---
        Node(package='nav2_controller', executable='controller_server', output='screen',
             parameters=[configured_params], remappings=remappings),

        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
             parameters=[configured_params], remappings=remappings),
        
        Node(package='nav2_smoother', executable='smoother_server', name='smoother_server', output='screen',
             parameters=[configured_params], remappings=remappings),

        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother', output='screen',
             parameters=[configured_params], remappings=remappings),

        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen',
             parameters=[configured_params], remappings=remappings),

        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen',
             parameters=[configured_params], remappings=remappings),

        Node(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower', output='screen',
             parameters=[configured_params], remappings=remappings),

        # Quản lý trạng thái các node trên
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
             output='screen',
             parameters=[{'use_sim_time': use_sim_time},
                         {'autostart': autostart},
                         {'node_names': lifecycle_nodes}]),
    ])