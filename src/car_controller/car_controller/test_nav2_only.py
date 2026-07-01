#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import csv
import os

class Nav2AccuracyTestNode(Node):
    def __init__(self):
        super().__init__('nav2_accuracy_test_node')

        # Action Client gửi lệnh cho Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscriber lắng nghe vị trí thực tế từ AMCL
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )

        # Tọa độ khởi tạo thực tế của anh
        self.GOAL_POSE = {
            'x': 4.472296478097551, 'y': -0.8228745070170737, 
            'z': -0.6138053793350687, 'w': 0.7894573809265022
        }
        self.HOME_POSE = {
            'x': 0.0, 'y': 0.0, 
            'z': 0.0, 'w': 1.0
        }

        # Quản lý Trạng thái & Bộ đếm
        self.STATE_GO_TO_GOAL = 1  
        self.STATE_GO_TO_HOME = 2  
        self.current_state = self.STATE_GO_TO_GOAL
        
        self.loop_count = 0        
        self.max_loops = 10

        # Biến lưu vị trí thực tế hiện tại từ AMCL và thời gian bấm giờ
        self.current_x = 0.0
        self.current_y = 0.0
        self.start_time = None

        # Khởi tạo file CSV
        self.csv_filename = "nav2_accuracy_test_results.csv"
        self.init_csv_file()

        # Khai báo biến quản lý timer delay (tránh blocking node)
        self.delay_timer = None

        self.get_logger().info("=====================================================")
        self.get_logger().info(f"Node Test Độ Chính Xác Nav2 Sẵn Sàng. Kết quả lưu tại: {self.csv_filename}")
        self.get_logger().info("Đợi 3 giây để xuất phát...")
        self.get_logger().info("=====================================================")
        
        self.start_timer = self.create_timer(3.0, self.start_mission)

    def init_csv_file(self):
        """Khởi tạo file CSV và ghi dòng tiêu đề (Header)"""
        if not os.path.exists(self.csv_filename):
            with open(self.csv_filename, mode='w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    "Lần_Chạy", "X_Đích", "X_Thực_Tế", "Y_Đích", "Y_Thực_Tế", 
                    "Thời_Gian_Hoàn_Thành", "Trạng_Thái"
                ])

    def amcl_pose_callback(self, msg):
        """Cập nhật liên tục vị trí thực tế của robot từ hệ thống định vị"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def start_mission(self):
        self.start_timer.cancel() 
        self.execute_next_step()

    def execute_next_step(self):
        """Điều hướng luồng chạy dựa trên trạng thái hiện tại"""
        if self.loop_count >= self.max_loops:
            self.get_logger().info(f"=== ĐÃ HOÀN THÀNH ĐỦ {self.max_loops} LẦN TEST! Dừng Node ===")
            rclpy.shutdown()
            return

        if self.current_state == self.STATE_GO_TO_GOAL:
            self.get_logger().info(f"\n>>> [LẦN CHẠY {self.loop_count + 1}/10]: Tiến về GOAL POSE...")
            self.start_time = time.time() 
            self.send_nav_goal(self.GOAL_POSE)
            
        elif self.current_state == self.STATE_GO_TO_HOME:
            self.get_logger().info(">>> [PHỤC HỒI]: Tự động quay về HOME POSE...")
            self.send_nav_goal(self.HOME_POSE)

    def send_nav_goal(self, pose_dict):
        self.nav_client.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = pose_dict['x']
        goal_msg.pose.pose.position.y = pose_dict['y']
        goal_msg.pose.pose.orientation.z = pose_dict['z']
        goal_msg.pose.pose.orientation.w = pose_dict['w']

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.nav_goal_accepted_callback)

    def nav_goal_accepted_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 từ chối Goal! Thử lại sau 2 giây...")
            self.start_delay_timer(2.0, self.execute_next_step)
            return
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        status = future.result().status
        
        if self.current_state == self.STATE_GO_TO_GOAL:
            # XỬ LÝ LƯỢT ĐI (TEST)
            duration = round(time.time() - self.start_time, 2) if self.start_time else 0.0
            status_str = "SUCCEEDED" if status == GoalStatus.STATUS_SUCCEEDED else f"FAILED_{status}"
            
            # Ghi dữ liệu trực tiếp vào CSV
            with open(self.csv_filename, mode='a', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    self.loop_count + 1,
                    self.GOAL_POSE['x'], self.current_x,
                    self.GOAL_POSE['y'], self.current_y,
                    duration, status_str
                ])
                
            self.get_logger().info(f"-> Đã ghi dữ liệu Lần {self.loop_count + 1}. Thời gian di chuyển: {duration}s")
            self.get_logger().info("Nghỉ ổn định 2 giây tại Goal trước khi quay đầu...")
            
            # Đổi sang trạng thái về Home và tạo khoảng trễ ngắn an toàn cho robot dừng hẳn
            self.current_state = self.STATE_GO_TO_HOME
            self.start_delay_timer(2.0, self.execute_next_step)

        elif self.current_state == self.STATE_GO_TO_HOME:
            # XỬ LÝ LƯỢT VỀ (PHỤC HỒI)
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.loop_count += 1
                self.get_logger().info("-> Đã về Home thành công. Nghỉ 3 giây chuẩn bị lượt tiếp theo...")
                
                # Cập nhật trạng thái mới và tạo trễ an toàn bằng Timer thay vì time.sleep()
                self.current_state = self.STATE_GO_TO_GOAL
                self.start_delay_timer(3.0, self.execute_next_step)
            else:
                self.get_logger().warn("Lỗi khi quay về Home! Thử gửi lại lệnh sau 2 giây...")
                self.start_delay_timer(2.0, self.execute_next_step)

    def start_delay_timer(self, seconds, callback_func):
        """Hàm phụ trợ tạo khoảng trễ bất đồng bộ (Non-blocking)"""
        if self.delay_timer is not None:
            self.delay_timer.cancel()
        
        def timer_callback():
            self.delay_timer.cancel()
            callback_func()

        self.delay_timer = self.create_timer(seconds, timer_callback)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2AccuracyTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Đã dừng kịch bản bằng tay (Ctrl+C)")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()