import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():
    # Đường dẫn tới các package
    pkg_gazebo = get_package_share_directory('gazebo_sim')
    pkg_description = get_package_share_directory('view_robot_model')
    
    # File Xacro mô phỏng
    xacro_file = os.path.join(pkg_gazebo, 'urdf', 'car_sim.urdf.xacro')
    robot_description_content = Command(['xacro ', xacro_file])

    # SỬA LỖI MESH: Ép Gazebo tìm kiếm file mesh trong thư mục cài đặt của ROS 2
    # Ta lấy đường dẫn cha của thư mục share/view_robot_model
    gazebo_resource_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.path.dirname(pkg_description)
    )

    # Khởi động thế giới ảo trống của Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Node robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str),
            'use_sim_time': True
        }]
    )

    # Node Spawner để thả xe vào
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'mecanum_car',
            '-string', robot_description_content,
            '-z', '0.05' 
        ],
        output='screen'
    )

    # Cầu nối Bridge cho cmd_vel
    # Đoạn cấu hình mẫu trong file launch của anh
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist', 
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_resource_path, # Kích hoạt đường dẫn mesh trước
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge_node
    ])