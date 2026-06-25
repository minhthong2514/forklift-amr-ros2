from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node điều khiển chính cho việc lấy hàng và bỏ hàng
        Node(
            package='car_controller',      # Tên gói (Package) của bạn
            executable='test_nav2_only',   # Tên thực thi đã đăng ký trong setup.py
            name='mission_controller_node', # Tên node khi hiển thị trong ros2 node list
            output='screen',               # Để in log ra màn hình terminal
            parameters=[{'use_sim_time': False}]
        )
    ])