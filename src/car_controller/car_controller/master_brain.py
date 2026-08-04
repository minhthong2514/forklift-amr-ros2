#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import Int8, String
from std_srvs.srv import Trigger
import time

class MasterBrain(Node):
    def __init__(self):
        super().__init__('master_brain')
        
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.docking_client = self.create_client(Trigger, 'start_docking')
        self.dropoff_client = self.create_client(Trigger, 'start_dropoff')
        
        self.web_sub = self.create_subscription(String, '/web/command', self.web_command_cb, 10)
        self.state_pub = self.create_publisher(Int8, '/master_brain_state', 10)

        self.HOME_POSE = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        # self.HOME_POSE = {'x': -1.6877453782511358, 'y': -1.0034006228696994, 'z': -0.06822652422328765, 'w': 0.9976698559104655}
                
        self.PICKUP_POSE = {'x': 4.472296478097551, 'y': -0.8228745070170737, 'z': -0.6138053793350687, 'w': 0.7894573809265022}
        self.DROPOFF_POSE = {'x': 2.25411928005759, 'y': 2.430015761398594, 'z': 0.7171808132907074, 'w': 0.6968871365204551}

        # Định nghĩa các trạng thái
        self.STATE_IDLE = -1          # Trạng thái chờ lệnh từ Web UI
        self.STATE_GO_TO_HOME = 0
        self.STATE_GO_TO_PICKUP = 1
        self.STATE_GO_TO_DROPOFF = 2
        self.STATE_RETURNING_HOME = 3 # Trạng thái quay về HOME sau khi thả hàng xong

        while not self.docking_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Đang chờ Docking Server...')

        # Khởi tạo ở trạng thái IDLE (Dừng chờ bấm RUN từ Web UI)
        self.current_state = self.STATE_IDLE
        self.get_logger().info("=== MASTER BRAIN KHỞI ĐỘNG: ĐANG Ở TRẠNG THÁI CHỜ (IDLE) ===")

    def web_command_cb(self, msg):
        cmd = msg.data.strip().upper()
        if cmd in ["RUN", "START"]:
            if self.current_state == self.STATE_IDLE:
                self.get_logger().info(">>> NHẬN LỆNH [RUN] TỪ WEB: Bắt đầu chu kỳ xuất phát từ HOME!")
                self.current_state = self.STATE_GO_TO_HOME
                self.send_nav_goal(self.HOME_POSE)
            else:
                self.get_logger().warn("Hệ thống đang trong chu kỳ chạy, bỏ qua lệnh RUN.")
        elif cmd == "STOP":
            self.get_logger().warn(">>> NHẬN LỆNH [STOP] TỪ WEB: Dừng hệ thống!")
            self.current_state = self.STATE_IDLE
            
    def send_nav_goal(self, pose_dict):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = pose_dict['x']
        goal_msg.pose.pose.position.y = pose_dict['y']
        goal_msg.pose.pose.orientation.z = pose_dict['z']
        goal_msg.pose.pose.orientation.w = pose_dict['w']
        
        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return
        future_result = goal_handle.get_result_async()
        future_result.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            
            # 1. Đến HOME ở đầu chu kỳ -> Nghỉ 3s rồi đi Pickup
            if self.current_state == self.STATE_GO_TO_HOME:
                self.get_logger().info(">>> NAV2: Đã ở vị trí HOME! Nghỉ 3 giây để ổn định...")
                time.sleep(3.0)
                
                # Phát tín hiệu số 1 báo hiệu bắt đầu tính giờ Nav2 sang Pickup
                msg = Int8()
                msg.data = 1
                self.state_pub.publish(msg)
                
                self.get_logger().info(">>> Hết 3 giây nghỉ. Xuất phát đi tới trạm LẤY HÀNG (Pickup)...")
                self.current_state = self.STATE_GO_TO_PICKUP
                self.send_nav_goal(self.PICKUP_POSE)
                
            # 2. Đến điểm Pickup -> Giao quyền cho Camera Docking
            elif self.current_state == self.STATE_GO_TO_PICKUP:
                self.get_logger().info(">>> NAV2: Đã đến điểm lấy hàng! Giao quyền cho Camera...")
                self.docking_client.call_async(Trigger.Request()).add_done_callback(self.docking_done_callback)
                
            # 3. Đến điểm Dropoff -> Kích hoạt thả hàng
            elif self.current_state == self.STATE_GO_TO_DROPOFF:
                self.get_logger().info(">>> NAV2: Đã đến trạm thả hàng! Kích hoạt quy trình thả...")
                self.dropoff_client.call_async(Trigger.Request()).add_done_callback(self.dropoff_done_callback)

            # 4. Đã chạy Nav2 về tới HOME sau khi hoàn tất thả hàng -> Chuyển về IDLE chờ lệnh tiếp
            elif self.current_state == self.STATE_RETURNING_HOME:
                self.get_logger().info("=== ĐÃ VỀ TỚI HOME AN TOÀN. HOÀN TẤT 1 CHU KỲ (CHUYỂN VỀ IDLE CHỜ RUN) ===")
                self.current_state = self.STATE_IDLE

    def docking_done_callback(self, future):
        if future.result().success:
            self.get_logger().info(">>> HOÀN TẤT LẤY HÀNG. Phát tín hiệu tính giờ Nav2 sang Dropoff...")
            
            msg = Int8()
            msg.data = 2
            self.state_pub.publish(msg)

            self.current_state = self.STATE_GO_TO_DROPOFF
            self.send_nav_goal(self.DROPOFF_POSE)

    def dropoff_done_callback(self, future):
        if future.result().success:
            self.get_logger().info("=== HOÀN TẤT THẢ HÀNG. Bắt đầu Nav2 quay về vị trí HOME... ===")
            self.current_state = self.STATE_RETURNING_HOME
            self.send_nav_goal(self.HOME_POSE)

def main(args=None):
    rclpy.init(args=args)
    node = MasterBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()