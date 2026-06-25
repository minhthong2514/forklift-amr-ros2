#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, termios, tty, threading, time

msg = """
Teleop CMD_VEL (Keyboard)
---------------------------
Điều khiển:
   Q    W    E
   A    S    D

* Auto stop sau 0.5s nếu mất phím
* Reset hướng khi đổi phím
---------------------------
"""

class TeleopCmdVel(Node):
    def __init__(self):
        super().__init__('teleop_cmd_vel')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Target velocities
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0

        self.speed = 0.30
        self.turn  = 0.20

        self.last_key_time = time.time()
        self.running = True

        self.key_thread = threading.Thread(target=self.get_key_loop)
        self.key_thread.daemon = True
        self.key_thread.start()

        # 20 Hz publish
        self.timer = self.create_timer(0.05, self.publish_callback)
        print(msg)

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def get_key_loop(self):
        while self.running and rclpy.ok():
            key = self.get_key()
            self.last_key_time = time.time()

            if key == 'w':
                self.target_vx = self.speed
                self.target_vy = 0.0
                self.target_wz = 0.0
            elif key == 's':
                self.target_vx = -self.speed
                self.target_vy = 0.0
                self.target_wz = 0.0
            elif key == 'q':
                self.target_vx = 0.0
                self.target_vy = self.speed
                self.target_wz = 0.0
            elif key == 'e':
                self.target_vx = 0.0
                self.target_vy = -self.speed
                self.target_wz = 0.0
            elif key == 'a':
                self.target_vx = 0.0
                self.target_vy = 0.0
                self.target_wz = self.turn
            elif key == 'd':
                self.target_vx = 0.0
                self.target_vy = 0.0
                self.target_wz = -self.turn
            elif key == 'x' or key == ' ':
                self.stop()
            elif key == '\x03':  # CTRL+C
                self.running = False
                break

    def stop(self):
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0

    def publish_callback(self):
        # Safety timeout
        if time.time() - self.last_key_time > 0.5:
            self.stop()

        msg = Twist()
        msg.linear.x = self.target_vx
        msg.linear.y = self.target_vy
        msg.angular.z = self.target_wz
        self.pub.publish(msg)

    def destroy_node(self):
        # Gửi lệnh dừng trước khi chết
        stop_msg = Twist()
        self.pub.publish(stop_msg)

        self.running = False
        time.sleep(0.1)  # cho thread thoát sạch
        super().destroy_node()

def main():
    rclpy.init()
    node = TeleopCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()