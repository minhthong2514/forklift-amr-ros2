#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus 
import time

class Nav2OnlyTestNode(Node):
    def __init__(self):
        super().__init__('nav2_only_test_node')

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.PICKUP_POSE = {
            'x': 2.2543599201220976, 'y': -0.43436129078981633, 
            'z': 0.7292691236459413, 'w': 0.6842269691386629 
        }

        self.DROPOFF_POSE = {
            'x': -6.906173430938212, 'y': 0.597566915831788, 
            'z': -0.7094737548603341, 'w': 0.7047318576340781
        }
        self.STATE_GO_TO_PICKUP = 1
        self.STATE_GO_TO_DROPOFF = 2
        self.current_state = self.STATE_GO_TO_PICKUP

        self.get_logger().info("Node Test Nav2 Đã Sẵn Sàng. Đợi 3 giây để xuất phát...")
        
        # SỬA CHỖ NÀY: Gán timer vào một biến để lát nữa gọi lệnh tắt
        self.start_timer = self.create_timer(3.0, self.start_mission)

    def start_mission(self):
        # SỬA CHỖ NÀY: Tắt timer ngay lập tức để nó không bao giờ lặp lại nữa!
        self.start_timer.cancel() 
        
        self.get_logger().info(">>> BẮT ĐẦU: Tiến về trạm Lấy hàng (Chỉ gửi 1 lần duy nhất!)")
        self.send_nav_goal(self.PICKUP_POSE)

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
            self.get_logger().error("Nav2 từ chối Goal!")
            return
        
        self.get_logger().info("Nav2 đã nhận lệnh. Bánh xe đang lăn...")
        result_future = goal_handle.get_result_async()
        
        # Chờ Nav2 phản hồi kết quả cuối cùng
        result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        status = future.result().status
        
        # TẠI ĐÂY LÀ CHỖ CHỐT HẠ: CHỈ KHI TRẠNG THÁI LÀ SUCCEEDED MỚI LÀM TIẾP
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(">>> NAV2 XÁC NHẬN: ĐÃ ĐẾN CHÍNH XÁC TỌA ĐỘ!")

            if self.current_state == self.STATE_GO_TO_PICKUP:
                self.get_logger().info("Bắt đầu quy trình LẤY HÀNG (đợi 5 giây)...")
                time.sleep(5.0) # Robot sẽ đứng im 5 giây ở đây
                
                self.current_state = self.STATE_GO_TO_DROPOFF
                self.get_logger().info(">>> Nâng xong! Gửi tọa độ trạm Bỏ hàng.")
                self.send_nav_goal(self.DROPOFF_POSE)
                
            elif self.current_state == self.STATE_GO_TO_DROPOFF:
                self.get_logger().info("Bắt đầu quy trình BỎ HÀNG (đợi 5 giây)...")
                time.sleep(5.0)

                self.current_state = self.STATE_GO_TO_PICKUP
                self.get_logger().info("=== HOÀN THÀNH 1 VÒNG === Gửi lại tọa độ Lấy hàng.")
                self.send_nav_goal(self.PICKUP_POSE)
        else:
            # Nếu xe bị kẹt, bị hủy lệnh, hoặc không đến được, nó sẽ in ra cảnh báo này
            self.get_logger().warn(f"CẢNH BÁO: Nav2 chưa đến được đích (Mã trạng thái: {status}). Hủy quy trình nâng hạ!")

def main(args=None):
    rclpy.init(args=args)
    node = Nav2OnlyTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Đã dừng kịch bản bằng tay (Ctrl+C)")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()