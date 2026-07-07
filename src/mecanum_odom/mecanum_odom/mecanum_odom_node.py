#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from sensor_msgs.msg import JointState
import math

class MecanumOdom(Node):
    def __init__(self):
        super().__init__('mecanum_odom')

        # Thông số vật lý của Robot
        self.r = 0.05
        self.L = 0.3965             # Lx (0.205) + Ly (0.1915)
        self.steps = 800

        # Khởi tạo biến vị trí (x, y, theta)
        self.x = self.y = self.th = 0.0
        self.last_ticks = None
        
        # Khởi tạo Publisher cho Joint State (Xoay bánh xe trong RViz)
        self.joint_names = ['front_left_jt', 'front_right_jt', 'back_left_jt', 'back_right_jt']
        self.wheel_angles = [0.0, 0.0, 0.0, 0.0] 
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Publisher cho Odometry
        self.odom_pub = self.create_publisher(Odometry, 'mecanum/odom', 10)

        # Subscriber nhận raw encoder ticks từ ESP32
        # Logic tính toán được đưa thẳng vào đây, XOÁ BỎ HOÀN TOÀN TIMER ĐỘC LẬP
        self.create_subscription(
            Int32MultiArray,
            '/wheel_ticks',
            self.tick_cb,
            10
        )
        self.last_time = self.get_clock().now()

    def tick_cb(self, msg):
        cur_ticks = msg.data
        now = self.get_clock().now()

        # Khởi tạo last_ticks ở chu kỳ đầu tiên khi có dữ liệu
        if self.last_ticks is None:
            self.last_ticks = list(cur_ticks)
            self.last_time = now
            return

        # Tính toán khoảng thời gian sai lệch giữa 2 lần nhận Ticks
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        
        if dt <= 0.001:
            return

        d = []
        left_wheel_indices = [0, 2]
        for i in range(4):
            # Tính delta ticks
            delta = cur_ticks[i] - self.last_ticks[i]
            
            # Bộ lọc nhiễu nhảy vọt encoder
            if abs(delta) > 1000:
                delta = 0
                
            # Cập nhật góc quay bánh xe để hiển thị RViz
            angle_delta = (delta / self.steps) * 2 * math.pi
            if i in left_wheel_indices:
                self.wheel_angles[i] -= angle_delta     
            else:
                self.wheel_angles[i] += angle_delta

            # Tính quãng đường di chuyển thực tế của từng bánh (mét)
            d.append(delta / self.steps * 2 * math.pi * self.r)
            self.last_ticks[i] = cur_ticks[i]

        # Publish Joint States để xoay bánh xe mô phỏng
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = self.joint_names
        js.position = self.wheel_angles
        self.joint_pub.publish(js)

        # Động học Mecanum: Tính toán vận tốc robot trong hệ tọa độ robot (dx, dy, dtheta)
        dx_b = sum(d) * 0.25
        dy_b = (-d[0] + d[1] + d[2] - d[3]) * 0.25
        dth  = (-d[0] + d[1] - d[2] + d[3]) / (4.0 * self.L)

        # Tích phân chuyển động vào hệ tọa độ thế giới (Odom)
        self.x += dx_b * math.cos(self.th) - dy_b * math.sin(self.th)
        self.y += dx_b * math.sin(self.th) + dy_b * math.cos(self.th)
        self.th += dth 
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))

        # Chuyển đổi góc Yaw sang Quaternion
        q = self.to_quat(self.th)
        now_msg = now.to_msg()

        # Tạo và Publish gói tin Odometry chuẩn lên ROS2
        odom = Odometry()
        odom.header.stamp = now_msg
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q

        # Tính vận tốc (m/s và rad/s) dựa trên dt thực tế nhận từ ESP32
        odom.twist.twist.linear.x = dx_b / dt
        odom.twist.twist.linear.y = dy_b / dt
        odom.twist.twist.angular.z = dth / dt
        
        # Cấu hình Ma trận hiệp phương sai sai số (Covariance)
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0]  = 0.01
        odom.pose.covariance[7]  = 0.01
        odom.pose.covariance[35] = 0.01

        odom.twist.covariance = [0.0] * 36
        odom.twist.covariance[0]  = 0.01 
        odom.twist.covariance[7]  = 0.01  
        odom.twist.covariance[35] = 0.01
        
        self.odom_pub.publish(odom)

    def to_quat(self, yaw):
        return Quaternion(x=0.0, y=0.0, z=math.sin(yaw/2), w=math.cos(yaw/2))

def main():
    rclpy.init()
    rclpy.spin(MecanumOdom())
    rclpy.shutdown()

if __name__ == '__main__':
    main()