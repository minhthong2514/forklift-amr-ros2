#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import math

class MecanumOdom(Node):
    def __init__(self):
        super().__init__('mecanum_odom')

        self.r = 0.05
        self.L = 0.40
        self.steps = 800

        self.x = self.y = self.th = 0.0
        self.last_ticks = None
        self.create_subscription(
            Int32MultiArray,
            '/wheel_ticks',
            self.tick_cb,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)

        self.timer = self.create_timer(0.02, self.update)
        self.last_time = self.get_clock().now()

        self.cur_ticks = None

    def tick_cb(self, msg):
        self.cur_ticks = msg.data

    def update(self):
        if self.cur_ticks is None:
            return

        now = self.get_clock().now()

        if self.last_ticks is None:
            self.last_ticks = list(self.cur_ticks)
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        
        # Kiểm tra dt tối thiểu để tránh nhiễu tính toán
        if dt <= 0.001:
            return

        d = []
        for i in range(4):
            delta = self.cur_ticks[i] - self.last_ticks[i]
            
            # BỘ LỌC CHỐNG NHẢY: Ngăn Odom văng khi reset ESP32
            if abs(delta) > 1000:
                delta = 0
                
            d.append(delta / self.steps * 2 * math.pi * self.r)
            self.last_ticks[i] = self.cur_ticks[i]

        dx_b = sum(d) * 0.25
        dy_b = (-d[0] + d[1] + d[2] - d[3]) * 0.25
        dth  = (-d[0] + d[1] - d[2] + d[3]) / (4 * self.L)

        # Cập nhật Pose
        self.x += dx_b * math.cos(self.th) - dy_b * math.sin(self.th)
        self.y += dx_b * math.sin(self.th) + dy_b * math.cos(self.th)
        self.th += dth # Cộng dồn góc trực tiếp giúp mượt hơn
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))

        q = self.to_quat(self.th)
        
        # Lấy nhãn thời gian đồng bộ mới nhất ngay trước khi gửi
        now_msg = self.get_clock().now().to_msg()

        # Phát TF: odom -> base_footprint
        t = TransformStamped()
        t.header.stamp = now_msg
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf.sendTransform(t)

        # Phát Odom Topic
        odom = Odometry()
        odom.header.stamp = now_msg
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        
        # HIỆP BIẾN (COVARIANCE): Quan trọng để SLAM không làm trôi robot
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0]  = 0.01 # sai số x
        odom.pose.covariance[7]  = 0.01 # sai số y
        odom.pose.covariance[35] = 0.01 # sai số yaw
        
        self.odom_pub.publish(odom)

    def to_quat(self, yaw):
        return Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(yaw/2),
            w=math.cos(yaw/2)
        )

def main():
    rclpy.init()
    rclpy.spin(MecanumOdom())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
