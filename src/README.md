sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
(Tùy vào máy bạn nhận cổng nào, thường là 0 và 1).

Terminal 1
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

Terminal 2
ros2 run mecanum_odom mecanum_odom_node

ros2 run tf2_ros static_transform_publisher 0.2825 0.0 0.531 0 0 0 base_link laser

ros2 launch sllidar_ros2 view_sllidar_c1_launch.py

# Máy Tú
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/tu/carto_ws/src/car_main/config/slam_params.yaml use_sim_time:=false
# Máy Thông
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/media/minhthong/DATA/DO_AN_TOT_NGHIEP/FORKLIFT-AGV/ros/src/car_main/config/slam_params.yaml use_sim_time:=false
