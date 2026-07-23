import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    home_dir = os.path.expanduser('~')
    web_dir = os.path.join(home_dir, 'Ung_dung', 'Code', 'Python', 'Forklift', 'Web_forklift')

    start_web_server = ExecuteProcess(
        cmd=['npm', 'start'], 
        cwd=web_dir,
        output='screen',
        shell=True
    )

    return LaunchDescription([
        start_web_server
    ])