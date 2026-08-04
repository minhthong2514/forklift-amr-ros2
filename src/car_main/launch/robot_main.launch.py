import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_car_main = get_package_share_directory('car_main')

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_car_main, 'launch', 'robot_localization.launch.py') 
        )
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_car_main, 'launch', 'robot_navigation.launch.py') 
        )
    )
    

    pickdrop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_car_main, 'launch', 'robot_pickdrop.launch.py') 
        )
    )

    web_server_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_car_main, 'launch', 'robot_web_server.launch.py') 
            )
        )
    
    return LaunchDescription([
        localization_launch,
        navigation_launch,
        pickdrop_launch,
        web_server_launch
    ])