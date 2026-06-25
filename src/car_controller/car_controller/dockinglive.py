#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int8, Bool
from std_srvs.srv import Trigger
from tf2_ros import TransformException, Buffer, TransformListener
from cv_bridge import CvBridge
import message_filters
import numpy as np
from ultralytics import YOLO
import time
import math
import tf2_geometry_msgs
from nav_msgs.msg import Odometry

class DockingServer(Node):
    def __init__(self):
        super().__init__('docking_server')
        
        # Chia nhóm luồng độc lập để tránh Deadlock
        self.cb_group_service = ReentrantCallbackGroup()
        self.cb_group_sensor = MutuallyExclusiveCallbackGroup()
        self.cb_group_timer = MutuallyExclusiveCallbackGroup() 

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.current_pose = None
        self.start_pose = None
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)

        # --- CÁC KÊNH GIAO TIẾP VỚI ESP32 & ĐỘNG CƠ ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lift_pub = self.create_publisher(Int8, '/lift_cmd', 10) 
        
        self.lift_done_flag = False
        self.lift_status_sub = self.create_subscription(
            Bool, '/lift_status', self.lift_status_cb, 10, callback_group=self.cb_group_service
        )

        self.srv_dock = self.create_service(Trigger, 'start_docking', self.docking_callback, callback_group=self.cb_group_service)
        self.srv_drop = self.create_service(Trigger, 'start_dropoff', self.dropoff_callback, callback_group=self.cb_group_service)

        self.L_tam_den_goc_cang = 0.315  
        self.khoang_cach_an_toan = 0.02  
        
        # Thiết lập quãng đường đi thẳng cố định cho trạm Dropoff
        self.fixed_dropoff_distance = 0.30 

        self.get_logger().info("Đang tải model YOLO TensorRT ngầm...")
        self.model = YOLO('/home/tai/Ung_dung/Code/Python/Camera_ZED/model/best (1).engine', task="obb")

        # Khởi tạo mặc định IDLE chờ lệnh từ Master Brain
        self.state = 'IDLE'
        
        self.action_done = False
        self.start_action_time = 0.0

        self.target_locked = False 
        self.locked_x = 0.0
        self.locked_y = 0.0
        self.locked_theta = 0.0

        self.fx = self.fy = self.cx = self.cy = None
        self.info_sub = self.create_subscription(CameraInfo, '/zed2/zed_node/rgb/color/rect/camera_info', self.cam_info_cb, 10, callback_group=self.cb_group_sensor)

        self.rgb_sub = message_filters.Subscriber(self, Image, '/zed2/zed_node/rgb/color/rect/image', callback_group=self.cb_group_sensor)
        self.depth_sub = message_filters.Subscriber(self, Image, '/zed2/zed_node/depth/depth_registered', callback_group=self.cb_group_sensor)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 10, 0.1)
        self.ts.registerCallback(self.camera_callback)

        self.timer = self.create_timer(0.1, self.blind_sequence_loop, callback_group=self.cb_group_timer)
        
        self.last_cam_log_time = time.time()
        self.get_logger().info("=== DOCKING SERVER SẴN SÀNG CHẠY TỰ ĐỘNG ===")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose.position

    def cam_info_cb(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]

    def docking_callback(self, request, response):
        self.get_logger().info(">>> DỊCH VỤ: Yêu cầu lấy hàng được kích hoạt.")
        self.target_locked = False  
        self.action_done = False
        self.state = 'ALIGN_THETA' 
        while not self.action_done:
            time.sleep(0.1) 
        response.success = True
        return response

    def dropoff_callback(self, request, response):
        self.get_logger().info(">>> DỊCH VỤ: Yêu cầu thả hàng được kích hoạt.")
        self.action_done = False
        self.state = 'DROP_FORWARD'
        self.start_action_time = time.time()
        while not self.action_done:
            time.sleep(0.1)
        response.success = True
        return response

    def lift_status_cb(self, msg):
        if msg.data == True:
            self.lift_done_flag = True

    def camera_callback(self, rgb_msg, depth_msg):
        # Camera chỉ hoạt động trong hai trạng thái căn chỉnh tư thế hình học này
        if self.state not in ['ALIGN_THETA', 'ALIGN_Y'] or self.fx is None:
            return 

        frame_bgr = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1") 

        results = self.model.predict(frame_bgr, conf=0.8, device=0, verbose=False)
        best_rx, best_ry, best_rz, best_r = None, None, None, None
        
        for result in results:
            if result.obb is not None:
                names = result.names
                pallet_list, pocket_list = [], []
                
                for i, box in enumerate(result.obb.xywhr):
                    x_c, y_c, w, h, r = box.tolist()
                    cls = int(result.obb.cls[i])    
                    label = names[cls]
                    center_x, center_y = int(x_c), int(y_c)
                    
                    if label.lower() == 'pallet':
                        pallet_list.append({'center': (center_x, center_y)})
                    else:
                        depth_values = []
                        for i_off in range(-2, 3):
                            for j_off in range(-2, 3):
                                cx_clip = np.clip(center_x + i_off, 0, frame_bgr.shape[1] - 1)
                                cy_clip = np.clip(center_y + j_off, 0, frame_bgr.shape[0] - 1)
                                val = depth_image[cy_clip, cx_clip]
                                if np.isfinite(val) and val > 0: 
                                    depth_values.append(float(val))

                        if depth_values:
                            cam_z = float(np.median(depth_values))
                            cos_goc = max(0.5, abs(math.cos(r))) 
                            cam_z -= 0.173 / cos_goc 

                            cam_x = (center_x - self.cx) * cam_z / self.fx
                            cam_y = (center_y - self.cy) * cam_z / self.fy
                            
                            robot_x_raw = cam_z    
                            robot_y_raw = -cam_x + 0.06 
                            robot_z_raw = -cam_y

                            pocket_list.append({
                                'center': (center_x, center_y), 
                                'robot_x': robot_x_raw, 
                                'robot_y': robot_y_raw, 
                                
                                'robot_z': robot_z_raw, 
                                'r': r,
                                'matched': False
                            })

                for pallet in pallet_list:
                    min_dist = float('inf')
                    matched_pocket = None
                    for pocket in pocket_list:
                        if not pocket['matched']:
                            dist = np.hypot(pallet['center'][0] - pocket['center'][0], pallet['center'][1] - pocket['center'][1])
                            if dist < 120:
                                min_dist = dist
                                matched_pocket = pocket
                    
                    if matched_pocket is not None:
                        matched_pocket['matched'] = True
                        if best_rx is None:
                            best_rx = matched_pocket['robot_x']
                            best_ry = matched_pocket['robot_y']
                            best_rz = matched_pocket['robot_z']
                            best_r = matched_pocket['r']

        if best_rx is not None:
            try:
                pt_cam = PointStamped()
                pt_cam.header.frame_id = 'zed2' 
                pt_cam.header.stamp = self.get_clock().now().to_msg()
                
                pt_cam.point.x = -best_ry  
                pt_cam.point.y = -best_rz  
                pt_cam.point.z = best_rx   
                
                pt_rob = self.tf_buffer.transform(pt_cam, 'base_link', timeout=rclpy.duration.Duration(seconds=0.1))
                # y_robot = pt_rob.point.y
                y_move = best_ry
                x_move = pt_rob.point.x - self.L_tam_den_goc_cang - self.khoang_cach_an_toan
                
                theta_deg = -best_r * (180.0 / math.pi)
                if theta_deg > 90: theta_deg -= 180
                elif theta_deg < -90: theta_deg += 180
                
                msg = Twist()
                
                # CHẶNG 1: CHỈNH ĐỘ LỆCH GÓC THETA (Tiêu chuẩn: <= 2.0 độ)
                if self.state == 'ALIGN_THETA':
                    if abs(theta_deg) > 2.0:
                        wz = 0.015 * theta_deg # Thêm dấu âm nếu hướng xoay cơ khí bị ngược
                        msg.angular.z = float(np.clip(wz, -0.2, 0.2))
                        self.cmd_pub.publish(msg)
                        if int(time.time() * 10) % 20 == 0:
                            self.get_logger().info(f"[ALIGN_THETA] Đang quay góc: Lệch {theta_deg:.1f} độ")
                    else:
                        self.cmd_pub.publish(Twist()) # Khóa phanh dừng xoay
                        self.get_logger().info(">>> CHỈNH THETA XONG. DỪNG ỔN ĐỊNH CƠ KHÍ...")
                        self.state = 'WAIT_THETA_STOP'
                        self.start_action_time = time.time()

                # CHẶNG 2: CHỈNH SAI LỆCH NGANG Y (Tiêu chuẩn: <= 1cm tương đương 0.01m)
                elif self.state == 'ALIGN_Y':
                    if abs(y_move) > 0.01: 
                        msg.linear.y = float(np.clip(0.6 * y_move, -0.12, 0.12))
                        self.cmd_pub.publish(msg)
                        if int(time.time() * 10) % 20 == 0:
                            self.get_logger().info(f"[ALIGN_Y] Đang trượt ngang: Lệch Y = {y_move:.3f} m")
                    else:
                        self.cmd_pub.publish(Twist()) # Khóa phanh dừng trượt ngang
                        
                        # Khóa cứng giá trị tĩnh tuyệt đối, ngắt xử lý ảnh từ frame sau
                        self.target_locked = True 
                        self.locked_x = x_move
                        self.locked_y = y_move
                        self.locked_theta = theta_deg
                        
                        self.get_logger().info("===========================================")
                        self.get_logger().info("==> TƯ THẾ HOÀN HẢO! KHÓA TỌA ĐỘ VÀ TẮT CAMERA:")
                        self.get_logger().info(f"  + X_tiến chốt = {self.locked_x:.3f} m")
                        self.get_logger().info(f"  + Y_lệch chốt = {self.locked_y:.3f} m")
                        self.get_logger().info(f"  + Theta chốt  = {self.locked_theta:.2f} độ")
                        self.get_logger().info("===========================================")
                        
                        self.state = 'WAIT_Y_STOP'
                        self.start_action_time = time.time() 
                        
            except TransformException as e:
                pass
        else:
            self.cmd_pub.publish(Twist()) # Đảm bảo an toàn: Mất dấu dấu lỗ pallet tự động đứng yên

    def blind_sequence_loop(self):
        msg = Twist()
        
        # Ngăn chặn spam log Terminal
        if not hasattr(self, 'last_log_time'): self.last_log_time = time.time()
        if time.time() - self.last_log_time >= 2.0:
            if self.state not in ['IDLE', 'ALIGN_THETA', 'ALIGN_Y']: 
                self.get_logger().info(f"[DEBUG] Trạng thái vòng lặp: {self.state} | Cờ ESP32 nâng xong: {self.lift_done_flag}")
            self.last_log_time = time.time()

        # Dừng ngắn sau khi xoay xong để ổn định quán tính cơ khí
        if self.state == 'WAIT_THETA_STOP':
            self.cmd_pub.publish(Twist()) 
            if time.time() - self.start_action_time > 1.0: # Dừng hẳn 1 giây
                self.get_logger().info(">>> Cơ khí ổn định. Chuyển sang trạng thái chỉnh ngang ALIGN_Y...")
                self.state = 'ALIGN_Y'

        # Dừng ngắn sau khi trượt Y xong trước khi tiến thẳng
        elif self.state == 'WAIT_Y_STOP':
            self.cmd_pub.publish(Twist())
            if time.time() - self.start_action_time > 1.0: # Dừng hẳn 1 giây
                self.get_logger().info(">>> Khung gầm ổn định tâm. Bắt đầu phóng mù INSERT_X...")
                self.state = 'INSERT_X'
                self.start_action_time = time.time()

        # Thực hiện hành trình tiến thẳng mù hoàn toàn không phụ thuộc camera
        elif self.state == 'INSERT_X':
            if self.start_pose is None and self.current_pose is not None:
                self.start_pose = self.current_pose # Chốt tọa độ ngay khi bắt đầu tiến
                
            if self.current_pose is not None:
                # Tính khoảng cách đã đi được so với điểm xuất phát
                dist_traveled = math.hypot(self.current_pose.x - self.start_pose.x, 
                                           self.current_pose.y - self.start_pose.y)
                
                # Nếu chưa tới đích thì tiến, nếu tới rồi thì phanh
                if dist_traveled < abs(self.locked_x) - 0.02: # -0.02 để dừng sớm tránh va đập
                    msg.linear.x = 0.05
                    self.cmd_pub.publish(msg)
                else:
                    self.cmd_pub.publish(Twist())
                    self.get_logger().info(">>> Đã tiến tới đích theo Odometry. Chờ phanh...")
                    self.state = 'WAIT_STOP_INSERT'
                    self.start_action_time = time.time()

        # Dừng ngắn chờ xe đứng yên hoàn toàn rồi mới kích nâng
        elif self.state == 'WAIT_STOP_INSERT':
            self.cmd_pub.publish(Twist()) # Ghì chặt phanh cơ khí
            if time.time() - self.start_action_time > 1.5: 
                self.get_logger().info(">>> CHỐT PHANH AN TOÀN! SPAM LỆNH NÂNG ĐỂ CHỐNG MẤT GÓI TIN...")
                self.lift_done_flag = False 
                
                # --- ĐƯA VÒNG LẶP TRỞ LẠI ĐỂ ĐẢM BẢO GIAO TIẾP ---
                lift_msg = Int8(data=1)
                for _ in range(3):
                    self.lift_pub.publish(lift_msg)
                    time.sleep(0.05) 
                # ------------------------------------------------
                
                self.state = 'LIFTING'
                self.start_action_time = time.time()

        elif self.state == 'LIFTING':
            self.cmd_pub.publish(Twist()) # Giữ phanh chết bánh xe
            
            # GIẢI PHÁP: Nếu có cờ ESP32 HOẶC chờ đủ 15 giây (tự ước lượng thời gian nâng cơ khí) thì cho đi tiếp
            if self.lift_done_flag == True or (time.time() - self.start_action_time > 15.0):
                if self.lift_done_flag:
                    self.get_logger().info(">>> Đã nhận phản hồi nâng xong từ phần cứng. Tiến hành lùi xe...")
                else:
                    self.get_logger().warn(">>> Mất kết nối cờ báo từ ESP32, ép buộc lùi xe theo thời gian...")
                
                self.state = 'RETRACT_PICKUP'
                self.start_action_time = time.time()

        elif self.state == 'RETRACT_PICKUP':
            if self.current_pose is not None:
                dist_traveled = math.hypot(self.current_pose.x - self.start_pose.x, 
                                           self.current_pose.y - self.start_pose.y)
                
                # Lùi cho đến khi dist_traveled quay về gần 0
                if dist_traveled > 0.02: 
                    msg.linear.x = -0.1
                    self.cmd_pub.publish(msg)
                else:
                    self.cmd_pub.publish(Twist())
                    self.get_logger().info(">>> Đã lùi về đúng điểm xuất phát theo Odometry.")
                    self.state = 'WAIT_RETRACT_STOP'
                    self.start_action_time = time.time()
                
        elif self.state == 'WAIT_RETRACT_STOP':
            self.cmd_pub.publish(Twist()) # Khóa phanh
            if time.time() - self.start_action_time > 3.0: # Nghỉ ngơi 3 giây
                self.get_logger().info(">>> QUY TRÌNH LẤY HÀNG DOCKING HOÀN THÀNH XUẤT SẮC!")
                self.state = 'IDLE'
                self.action_done = True

        # =============================================================
        # NHÁNH THẢ HÀNG CŨNG SỬA TƯƠNG TỰ
        # =============================================================
        elif self.state == 'DROP_FORWARD':
            if time.time() - self.start_action_time < 3.5: 
                msg.linear.x = 0.1
                self.cmd_pub.publish(msg)
            else:
                self.cmd_pub.publish(Twist())
                self.get_logger().info(">>> Đã tới bệ thả hàng. Chờ phanh ổn định...")
                self.state = 'WAIT_DROP_STOP'
                self.start_action_time = time.time()

        elif self.state == 'WAIT_DROP_STOP':
            self.cmd_pub.publish(Twist())
            if time.time() - self.start_action_time > 1.5:
                self.get_logger().info(">>> CHỐT PHANH! SPAM LỆNH HẠ SÁT SÀN...")
                self.lift_done_flag = False 
                
                # --- ĐƯA VÒNG LẶP TRỞ LẠI ---
                lift_msg = Int8(data=0)
                for _ in range(3):
                    self.lift_pub.publish(lift_msg)
                    time.sleep(0.05)
                # -----------------------------
                    
                self.state = 'DROPPING'
                self.start_action_time = time.time()
                
        elif self.state == 'DROPPING':
            self.cmd_pub.publish(Twist())
            if self.lift_done_flag == True:
                self.get_logger().info(">>> Hàng đã chạm đất an toàn. Rút càng nâng...")
                self.state = 'RETRACT_DROP'
                self.start_action_time = time.time()
            elif time.time() - self.start_action_time > 25.0:
                self.get_logger().error("HẾT THỜI GIAN CHỜ AN TOÀN: Thả hàng lỗi cơ khí!")
                self.state = 'IDLE'
                self.action_done = True 
                
        elif self.state == 'RETRACT_DROP':
            if time.time() - self.start_action_time < 4.0: 
                msg.linear.x = -0.1
                self.cmd_pub.publish(msg)
            else:
                self.cmd_pub.publish(Twist())
                self.get_logger().info(">>> Quy trình thả hàng và rút cơ cấu HOÀN TẤT hoàn toàn!")
                self.state = 'IDLE'
                self.action_done = True

def main(args=None):
    rclpy.init(args=args)
    node = DockingServer()
    # Nâng cấp số lượng luồng để không bị nghẽn (Deadlock) giữa Service và Timer
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': 
    main()