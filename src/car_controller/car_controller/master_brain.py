#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus 
from std_srvs.srv import Trigger

class MasterBrain(Node):
    def __init__(self):
        super().__init__('master_brain')

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Tạo kết nối tới 2 dịch vụ của Docking Server
        self.docking_client = self.create_client(Trigger, 'start_docking')
        self.dropoff_client = self.create_client(Trigger, 'start_dropoff')

        # self.PICKUP_POSE = {'x': 0.9898801273585425, 'y': 2.5434634660084776, 'z': 0.005471447589904, 'w': 0.9999850315186077}
        self.PICKUP_POSE = {'x': 4.472296478097551, 'y': -0.8228745070170737, 'z': -0.6138053793350687, 'w': 0.7894573809265022}
        self.DROPOFF_POSE = {'x': 2.25411928005759, 'y': 2.430015761398594, 'z': 0.7171808132907074, 'w': 0.6968871365204551}

        self.STATE_GO_TO_PICKUP = 1
        self.STATE_GO_TO_DROPOFF = 2
        
        # Đợi các Server khởi động
        while not self.docking_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Đang chờ Docking Server...')

        self.current_state = self.STATE_GO_TO_PICKUP
        self.get_logger().info("=== MASTER BRAIN KHỞI ĐỘNG ===")
        self.send_nav_goal(self.PICKUP_POSE)

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
        if not goal_handle.accepted: return
        future_result = goal_handle.get_result_async()
        future_result.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            if self.current_state == self.STATE_GO_TO_PICKUP:
                self.get_logger().info(">>> NAV2: Đã đến điểm lấy hàng! Giao quyền cho Camera...")
                req = Trigger.Request()
                future_dock = self.docking_client.call_async(req)
                future_dock.add_done_callback(self.docking_done_callback)
                
            elif self.current_state == self.STATE_GO_TO_DROPOFF:
                self.get_logger().info(">>> NAV2: Đã đến trạm thả hàng! Kích hoạt quy trình thả...")
                req = Trigger.Request()
                future_drop = self.dropoff_client.call_async(req)
                future_drop.add_done_callback(self.dropoff_done_callback)

    # --- CALLBACK KHI DOCKING SERVER BÁO CÁO "ĐÃ XONG" ---
    def docking_done_callback(self, future):
        res = future.result()
        if res.success:
            self.get_logger().info(">>> HOÀN TẤT LẤY HÀNG. Chuyển hướng sang trạm Dropoff...")
            self.current_state = self.STATE_GO_TO_DROPOFF
            self.send_nav_goal(self.DROPOFF_POSE)

    def dropoff_done_callback(self, future):
        res = future.result()
        if res.success:
            self.get_logger().info("=== HOÀN TẤT 1 VÒNG === Quay lại trạm Pickup.")
            self.current_state = self.STATE_GO_TO_PICKUP
            self.send_nav_goal(self.PICKUP_POSE)

def main(args=None):
    rclpy.init(args=args)
    node = MasterBrain()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()