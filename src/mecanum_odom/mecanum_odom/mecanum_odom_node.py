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
        if dt <= 0.0:
            return

        d = []
        for i in range(4):
            delta = self.cur_ticks[i] - self.last_ticks[i]
            d.append(delta / self.steps * 2 * math.pi * self.r)
            self.last_ticks[i] = self.cur_ticks[i]

        dx_b = sum(d) * 0.25
        dy_b = (-d[0] + d[1] + d[2] - d[3]) * 0.25
        dth  = (-d[0] + d[1] - d[2] + d[3]) / (4 * self.L)

        self.x += dx_b * math.cos(self.th) - dy_b * math.sin(self.th)
        self.y += dx_b * math.sin(self.th) + dy_b * math.cos(self.th)
        self.th = math.atan2(math.sin(self.th + dth), math.cos(self.th + dth))

        q = self.to_quat(self.th)

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = q
        self.tf.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
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
