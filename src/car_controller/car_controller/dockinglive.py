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
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import csv

class DockingServer(Node):
    def __init__(self):
        super().__init__('docking_server')
        
        # Cấu hình số lần chạy thực nghiệm để xuất báo cáo
        self.max_test_runs = 2                # Thay đổi số này tùy ý
        
        # [CẬP NHẬT CHUẨN ĐỒNG BỘ TỪ MASTER BRAIN]
        self.pickup_goal_x = 4.472296478097551
        self.pickup_goal_y = -0.8228745070170737
        self.pickup_goal_theta = -75.8        # Đã đổi từ Quaternion (z, w) sang Độ
        
        self.dropoff_goal_x = 2.25411928005759
        self.dropoff_goal_y = 2.430015761398594
        self.dropoff_goal_theta = 91.6        # Đã đổi từ Quaternion (z, w) sang Độ

        # Các bộ lưu trữ dữ liệu thực nghiệm
        self.pickup_data = []
        self.dropoff_data = []
        self.pickup_counter = 0
        self.dropoff_counter = 0

        # Biến lưu trữ thời gian căn chỉnh camera riêng biệt
        self.docking_init_time = 0.0
        self.cam_align_duration = 0.0
        self.nav2_start_time = time.time() 
        self.pickup_nav2_duration = 0.0
        self.dropoff_nav2_duration = 0.0

        # Biến lưu vị trí AMCL để phục vụ tính toán sai số thực tế trạm Pickup
        self.pickup_amcl_start_x = None
        self.pickup_amcl_start_y = None
        self.pickup_target_dist = 0.0
        self.final_pickup_error_x = 0.0

        # =========================================================================
        # TRUNG TÂM CẤU HÌNH HỆ THỐNG (DỄ DÀNG KIỂM SOÁT & TINH CHỈNH)
        # =========================================================================
        # 1. Các thông số kích thước hình học & An toàn cơ khí
        self.L_tam_den_goc_cang = 0.3175      # Chiều dài từ tâm robot đến góc càng nâng (mét)
        self.khoang_cach_an_toan = 0.0       # Khoảng cách hở an toàn khi áp sát pallet (mét)
        self.fixed_dropoff_distance = 0.40    # Quãng đường tiến vào cố định tại trạm đặt hàng (mét)
        
        # 2. Vận tốc vận hành của AGV
        self.SPEED_ALIGN_Y_GAIN = 0.8         # Hệ số tăng cường tỉ lệ căn chỉnh tâm Y (P gain)
        self.SPEED_ALIGN_Y_MAX = 0.12         # Tốc độ dịch ngang (Y) tối đa cho phép (m/s)
        self.SPEED_ALIGN_THETA_GAIN = 0.02    # Hệ số tỉ lệ điều khiển xoay góc (P gain cho Angular Z)
        self.SPEED_ALIGN_THETA_MAX = 0.4      # Tốc độ xoay tối đa cho phép (rad/s)
        self.SPEED_FORWARD_X = 0.08            # Tốc độ tịnh tiến X khi đi vào trạm (m/s)
        self.SPEED_RETRACT_X = -0.08           # Tốc độ lùi X khi thoát khỏi trạm (m/s)
        
        # 3. Cấu hình thời gian chờ ổn định giữa TẤT CẢ CÁC SỰ KIỆN (Cố định 1.0s)
        self.TIME_WAIT_MECHANICAL = 1.0       # Thời gian dừng im nghỉ giữa mỗi hành động (giây)
        
        # 4. Bộ lọc Camera và Trạng thái
        self.forward_tracking_active = False  # Cờ khóa trạng thái: Khi đã tiến X thì bám liên tục, không quay lại xoay tại chỗ
        self.docking_start_pose = None        # Lưu mốc xuất phát ban đầu lúc bắt đầu Tiến X Tracking (~1m)
        self.drop_retract_start_pose = None   # Mốc đứng trong trạm trước khi lùi của Dropoff
        self.last_cam_error_x = 0.0
        self.last_cam_y_move = 0.0
        self.last_cam_theta_move = 0.0

        self.CAM_THETA_TOLERANCE = 5.0        # Sai số góc Theta chấp nhận được để khóa mục tiêu (độ)
        self.CAM_Y_TOLERANCE = 0.01          # Sai số tâm Y chấp nhận được để khóa mục tiêu (mét)
        self.CAM_MIN_VALID_X = 0.05           # Khoảng cách trục X tối thiểu hợp lệ (mét)
        self.CAM_MAX_VALID_X = 1.5            # Khoảng cách trục X tối đa hợp lệ để tránh nhiễu xa (mét)
        self.CAM_CONFIDENCE_THRESHOLD = 0.8  # Ngưỡng tin cậy nhận diện Pallet của YOLO (từ 0.0 đến 1.0)
        self.YOLO_MODEL_PATH = '/home/tai/Ung_dung/Code/Python/Camera_ZED/model/pallet_obb_model.engine'
        
        # 5. Phân bổ tài nguyên thực thi ROS 2 (Tránh nghẽn luồng / Deadlock)
        self.cb_group_service = ReentrantCallbackGroup()
        self.cb_group_sensor = MutuallyExclusiveCallbackGroup()
        self.cb_group_timer = MutuallyExclusiveCallbackGroup() 

        # =========================================================================
        # KHỞI TẠO CÁC ĐỐI TƯỢNG GIAO TIẾP VÀ BIẾN TRẠNG THÁI
        # =========================================================================
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.current_pose = None
        self.current_theta = 0.0  
        self.start_pose = None        # Dùng cho việc tiến lùi trạm Pickup
        self.drop_start_pose = None   # Mốc Odom gốc cố định của trạm Dropoff để tính tiến/lùi không sai số

        self.cb_group_subs = ReentrantCallbackGroup()
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10, callback_group=self.cb_group_subs)
        self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',self.amcl_callback, 10, callback_group=self.cb_group_subs)
        self.master_state_sub = self.create_subscription(Int8, '/master_brain_state', self.master_state_callback, 10, callback_group=self.cb_group_subs)

        self.current_amcl_pose = None
        # Bộ phát/nhận tín hiệu điều khiển di chuyển và cơ cấu nâng hạ ESP32
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lift_pub = self.create_publisher(Int8, '/lift_cmd', 10) 

        self.lift_done_flag = False
        self.lift_status_sub = self.create_subscription(Bool, '/lift_status', self.lift_status_cb, 10, callback_group=self.cb_group_service)

        # Cung cấp dịch vụ (Services) cho Master Brain gọi điều hướng
        self.srv_dock = self.create_service(Trigger, 'start_docking', self.docking_callback, callback_group=self.cb_group_service)
        self.srv_drop = self.create_service(Trigger, 'start_dropoff', self.dropoff_callback, callback_group=self.cb_group_service)

        self.get_logger().info("Đang tải model YOLO TensorRT ngầm...")
        self.model = YOLO(self.YOLO_MODEL_PATH, task="obb")

        # Quản lý Trạng thái tuần tự (State Machine)
        self.state = 'IDLE'
        self.action_done = False
        self.start_action_time = 0.0
        self.step_waiting = False
        self.step_wait_start_time = 0.0

        self.target_locked = False 
        self.locked_x = 0.0
        self.locked_y = 0.0
        self.locked_theta = 0.0
        self.locked_pallet_side = None

        # Thông số tiêu cự camera nội tại (Sẽ tự cập nhật từ Topic)
        self.fx = self.fy = self.cx = self.cy = None
        self.info_sub = self.create_subscription(CameraInfo, '/zed2/zed_node/rgb/color/rect/camera_info', self.cam_info_cb, 10, callback_group=self.cb_group_sensor)

        # Đồng bộ hóa luồng hình ảnh màu và bản đồ độ sâu từ ZED2
        self.rgb_sub = message_filters.Subscriber(self, Image, '/zed2/zed_node/rgb/color/rect/image', callback_group=self.cb_group_sensor)
        self.depth_sub = message_filters.Subscriber(self, Image, '/zed2/zed_node/depth/depth_registered', callback_group=self.cb_group_sensor)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 10, 0.1)
        self.ts.registerCallback(self.camera_callback)

        # Vòng lặp điều khiển mù chạy ngầm định kỳ 100ms
        self.timer = self.create_timer(0.1, self.blind_sequence_loop, callback_group=self.cb_group_timer)
        
        self.last_cam_log_time = time.time()
        self.get_logger().info("=== DOCKING SERVER SẴN SÀNG CHẠY TỰ ĐỘNG ===")

    def master_state_callback(self, msg):
        # Khi nhận được tín hiệu số 1 (Xuất phát đi tới trạm Pickup)
        if msg.data == 1:
            self.nav2_start_time = time.time()
            self.get_logger().info("--> [ĐỒNG BỘ] Đã nhận tín hiệu xuất phát từ Master Brain. Bắt đầu tính thời gian Nav2!")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.current_theta = 2.0 * math.atan2(q.z, q.w) # đơn vị: Radian

    def amcl_callback(self, msg):
        # Lưu lại vị trí thực tế của robot trên Bản đồ (Map)
        self.current_amcl_pose = msg.pose.pose

    def cam_info_cb(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]

    def docking_callback(self, request, response):
        self.get_logger().info(">>> DỊCH VỤ: Yêu cầu lấy hàng được kích hoạt.")
        self.pickup_nav2_duration = time.time() - self.nav2_start_time
        self.docking_init_time = time.time()

        self.cam_align_duration = 0.0
        self.docking_start_pose = None
        self.pickup_amcl_start_x = None
        self.pickup_amcl_start_y = None

        # Mặc định nếu chưa có AMCL thì lấy tạm Odom thô làm fallback
        self.nav2_error_x = self.current_pose.x - self.pickup_goal_x if self.current_pose else 0.0
        self.nav2_error_y = self.current_pose.y - self.pickup_goal_y if self.current_pose else 0.0
        self.nav2_error_theta = math.degrees(self.current_theta) - self.pickup_goal_theta

        # Nếu có AMCL Pose thực tế trên MAP, ưu tiên dùng dữ liệu này để tính sai số chuẩn
        if self.current_amcl_pose is not None:
            robot_x_on_map = self.current_amcl_pose.position.x
            robot_y_on_map = self.current_amcl_pose.position.y
            
            self.nav2_error_x = robot_x_on_map - self.pickup_goal_x
            self.nav2_error_y = robot_y_on_map - self.pickup_goal_y
            
            q = self.current_amcl_pose.orientation
            robot_theta_on_map = 2.0 * math.atan2(q.z, q.w)
            self.nav2_error_theta = math.degrees(robot_theta_on_map) - self.pickup_goal_theta
        else:
            self.get_logger().warn("Chưa nhận được dữ liệu từ /amcl_pose, báo cáo tạm thời dùng Odom thô!")

        self.get_logger().info(
            f"\n=========================================================\n"
            f"[REPORT] Đang thu thập mẫu Pickup thứ {self.pickup_counter + 1}/{self.max_test_runs}\n"
            f"-> Sai số Nav2 ban đầu (Hệ MAP): X={self.nav2_error_x:.3f}m | Y={self.nav2_error_y:.3f}m | Theta={self.nav2_error_theta:.1f}°\n"
            f"========================================================="
        )

        self.target_locked = False  
        self.action_done = False
        self.state = 'ALIGN_Y' 
        
        # Treo luồng chờ vòng lặp sequence giải phóng cờ hoàn thành
        while not self.action_done:
            time.sleep(0.1) 
            
        response.success = True
        return response

    def dropoff_callback(self, request, response):
        self.get_logger().info(">>> DỊCH VỤ: Yêu cầu đặt hàng được kích hoạt.")
        
        if self.current_pose is None:
            response.success = False
            response.message = "Không thể thực hiện quy trình vì chưa nhận được dữ liệu Odom!"
            return response

        self.dropoff_nav2_duration = time.time() - self.nav2_start_time   

        # Mặc định lấy tạm Odom thô
        self.nav2_error_x = self.current_pose.x - self.dropoff_goal_x
        self.nav2_error_y = self.current_pose.y - self.dropoff_goal_y
        self.nav2_error_theta = math.degrees(self.current_theta) - self.dropoff_goal_theta
        
        # Ưu tiên ÉP chuyển sang tính toán bằng AMCL Pose toàn cục (Hệ MAP) giống Pickup
        if self.current_amcl_pose is not None:
            robot_x_on_map = self.current_amcl_pose.position.x
            robot_y_on_map = self.current_amcl_pose.position.y
            
            self.nav2_error_x = robot_x_on_map - self.dropoff_goal_x
            self.nav2_error_y = robot_y_on_map - self.dropoff_goal_y
            
            q = self.current_amcl_pose.orientation
            robot_theta_on_map = 2.0 * math.atan2(q.z, q.w)
            self.nav2_error_theta = math.degrees(robot_theta_on_map) - self.dropoff_goal_theta

        self.get_logger().info(
            f"\n=========================================================\n"
            f"[REPORT] Đang thu thập mẫu Dropoff thứ {self.dropoff_counter + 1}/{self.max_test_runs}\n"
            f"-> Sai số Nav2 ban đầu (Hệ MAP): X={self.nav2_error_x:.3f}m | Y={self.nav2_error_y:.3f}m | Theta={self.nav2_error_theta:.1f}°\n"
            f"========================================================="
        )

        self.action_done = False
        self.drop_start_pose = None  
        
        self.state = 'DROP_START_WAIT'
        self.start_action_time = time.time()
        
        while not self.action_done:
            time.sleep(0.1)
            
        response.success = True
        response.message = f"Hoàn thành quy trình thả hàng mẫu thứ {self.dropoff_counter}."
        return response

    def lift_status_cb(self, msg):
        if msg.data == True:
            self.lift_done_flag = True

    def camera_callback(self, rgb_msg, depth_msg):
        should_log = False
        if time.time() - self.last_cam_log_time >= 0.5:
            should_log = True
            self.last_cam_log_time = time.time()

        if self.state == 'INSERT_X' or self.target_locked:
            return

        if self.state != 'ALIGN_Y' or self.fx is None:
            return

        # 1. Kiểm tra khoảng nghỉ 0.5s giữa các bước căn chỉnh cơ khí ban đầu
        if self.step_waiting:
            msg = Twist()
            self.cmd_pub.publish(msg)
            if time.time() - self.step_wait_start_time >= 0.5:
                self.step_waiting = False
                self.get_logger().info(">>> Hết 0.5s nghỉ ổn định cơ khí, tiếp tục quy trình.")
            return

        # 2. Xử lý hình ảnh và nhận diện lỗ Pallet bằng YOLO
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1") 
        except Exception as e:
            return

        results = self.model.predict(frame_bgr, conf=self.CAM_CONFIDENCE_THRESHOLD, device=0, verbose=False)
        best_cam_x, best_cam_y, best_depth, best_r = None, None, None, None
        
        valid_pockets_in_frame = []
        
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
                            depth_final = float(np.median(depth_values))
                            cos_goc = max(0.5, abs(math.cos(r))) 
                            depth_final -= 0.173 / cos_goc 
                            cam_x = (center_x - self.cx) * depth_final / self.fx
                            cam_y = (center_y - self.cy) * depth_final / self.fy

                            pocket_list.append({
                                'center': (center_x, center_y), 'cam_x': cam_x, 'cam_y': cam_y, 
                                'depth': depth_final, 'r': r, 'matched': False
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
                        valid_pockets_in_frame.append(matched_pocket)

        if len(valid_pockets_in_frame) > 0:
            valid_pockets_in_frame.sort(key=lambda p: p['cam_y'])
            
            if not hasattr(self, 'current_tracking_side'):
                self.current_tracking_side = None

            if self.current_tracking_side is None:
                sorted_by_dist = sorted(valid_pockets_in_frame, key=lambda p: p['depth'])
                closest_pocket = sorted_by_dist[0]
                
                if len(valid_pockets_in_frame) > 1:
                    if closest_pocket['cam_y'] == valid_pockets_in_frame[0]['cam_y']:
                        self.current_tracking_side = 'LEFT'
                    else:
                        self.current_tracking_side = 'RIGHT'
                else:
                    self.current_tracking_side = 'ONLY_ONE'
                
                self.get_logger().info(f"[LOCK VẾ] Thằng gần nhất: {closest_pocket['depth']:.3f}m -> Khóa vế: {self.current_tracking_side}")
                selected_target = closest_pocket
            else:
                if self.current_tracking_side == 'LEFT':
                    selected_target = valid_pockets_in_frame[0]
                elif self.current_tracking_side == 'RIGHT':
                    selected_target = valid_pockets_in_frame[-1]
                else:
                    selected_target = valid_pockets_in_frame[0]
            
            best_cam_x = selected_target['cam_x']
            best_cam_y = selected_target['cam_y']
            best_depth = selected_target['depth']
            best_r = selected_target['r']

        # 3. Tính toán tọa độ qua TF2 và Thực hiện chuỗi logic điều khiển Khóa trạng thái tuần tự
        if best_depth is not None:
            try:
                pt_cam = PointStamped()
                pt_cam.header.frame_id = 'zed2'
                pt_cam.header.stamp = rgb_msg.header.stamp 
                pt_cam.point.x = float(best_cam_x)
                pt_cam.point.y = float(best_cam_y)
                pt_cam.point.z = float(best_depth)
                
                pt_rob = self.tf_buffer.transform(pt_cam, 'base_link', timeout=rclpy.duration.Duration(seconds=0.1))
                
                y_move = -best_cam_x + 0.06
                theta_move = -best_r * (180.0 / math.pi)
                
                distance_to_pallet = pt_rob.point.x 
                error_x = distance_to_pallet - self.L_tam_den_goc_cang - self.khoang_cach_an_toan
                
                self.last_cam_error_x = error_x
                self.last_cam_y_move = y_move
                self.last_cam_theta_move = theta_move

                if should_log:
                    self.get_logger().info(f"[CAMERA] Khoảng cách thực tế từ Cam: {best_depth:.3f}m | error_x còn lại: {error_x:.3f}m")

                msg = Twist()
                
                # BƯỚC 3 ACTIVE: Vừa tiến vừa sửa sai liên tục
                if self.forward_tracking_active:
                    if error_x > 0.05:
                        khoang_cach_toi_nguong_ngat = error_x - 0.05
                        speed_x_smooth = 0.03 + (self.SPEED_FORWARD_X - 0.03) * np.clip(khoang_cach_toi_nguong_ngat / 0.30, 0.0, 1.0)
        
                        msg.linear.x = float(speed_x_smooth)
                        msg.linear.y = float(np.clip(self.SPEED_ALIGN_Y_GAIN * y_move, -self.SPEED_ALIGN_Y_MAX, self.SPEED_ALIGN_Y_MAX)) 
                        msg.angular.z = float(np.clip(self.SPEED_ALIGN_THETA_GAIN * theta_move, -self.SPEED_ALIGN_THETA_MAX, self.SPEED_ALIGN_THETA_MAX))
                        self.cmd_pub.publish(msg)

                        if should_log:
                            self.get_logger().info(f"-> [Bước 3 - TIẾN X TRACKING]: Tốc độ X: {speed_x_smooth:.3f} m/s | error_x còn lại: {error_x:.3f}m")
                    else:
                        self.cmd_pub.publish(Twist()) # Phanh dừng dứt khoát để chốt mốc Odom chuẩn
                        
                        # CHỐT THỜI GIAN CĂN CHỈNH CAMERA RIÊNG BIỆT TẠI ĐÂY
                        self.cam_align_duration = time.time() - self.docking_init_time
                        
                        # LOGIC HÌNH HỌC ĐỘNG: Lấy tổng quãng đường cần đi ban đầu trừ đi phần Odom thực tế đã chạy được từ mốc xuất phát
                        dist_already_traveled = np.hypot(self.current_pose.x - self.docking_start_pose.x, self.current_pose.y - self.docking_start_pose.y)
                        self.locked_x = max(0.0, self.pickup_target_dist - dist_already_traveled)
                        
                        self.locked_y = y_move
                        self.locked_theta = theta_move
                        self.target_locked = True
                        
                        self.current_tracking_side = None # Reset vế khóa cho lượt sau

                        self.state = 'INSERT_X'
                        self.start_pose = None 
                        self.forward_tracking_active = False
                        self.get_logger().info(f"====> ĐẠT NGƯỠNG MÙ CAM CHUẨN XÁC ({best_depth:.3f}m)! Chuyển Odom đi mù nốt khoảng cách còn lại: {self.locked_x:.3f}m.")
                
                # NẾU CHƯA TIẾN (ĐANG TRONG GIAI ĐOẠN CĂN CHỈNH TĨNH BAN ĐẦU)
                else:
                    # BƯỚC 1: Ưu tiên xoay nắn góc Theta trước tại chỗ
                    if abs(theta_move) > self.CAM_THETA_TOLERANCE:
                        msg.linear.x = 0.0
                        msg.linear.y = 0.0
                        msg.angular.z = float(np.clip(self.SPEED_ALIGN_THETA_GAIN * theta_move, -self.SPEED_ALIGN_THETA_MAX, self.SPEED_ALIGN_THETA_MAX))
                        self.cmd_pub.publish(msg)
                        if should_log:
                            self.get_logger().info(f"-> [Bước 1 - CHỈNH THETA TĨNH]: Đang xoay tại chỗ tinh chỉnh góc...")
                        self.last_theta_ok = True 

                    # BƯỚC 2: Khi Theta đã chuẩn, chuyển sang dịch ngang căn tâm Y tại chỗ
                    elif abs(y_move) > self.CAM_Y_TOLERANCE: 
                        if getattr(self, 'last_theta_ok', False):
                            self.last_theta_ok = False
                            self.step_waiting = True
                            self.step_wait_start_time = time.time()
                            self.cmd_pub.publish(Twist())
                            self.get_logger().info(">>> Theta chuẩn! Nghỉ 0.5s dập quán tính xoay trước khi dịch ngang Y...")
                            return

                        msg.linear.x = 0.0  
                        msg.linear.y = float(np.clip(self.SPEED_ALIGN_Y_GAIN * y_move, -self.SPEED_ALIGN_Y_MAX, self.SPEED_ALIGN_Y_MAX)) 
                        msg.angular.z = float(np.clip(self.SPEED_ALIGN_THETA_GAIN * theta_move, -self.SPEED_ALIGN_THETA_MAX, self.SPEED_ALIGN_THETA_MAX)) 
                        self.cmd_pub.publish(msg)
                        if should_log:
                            self.get_logger().info(f"-> [Bước 2 - CHỈNH Y TĨNH]: Đang dịch ngang tinh chỉnh tâm Y...")
                        self.last_y_ok = True 

                    # BƯỚC 3 KÍCH HOẠT: Cả hai sai số tĩnh đều đạt chuẩn -> Bật cờ khóa trạng thái tiến liên tục
                    else:
                        if getattr(self, 'last_y_ok', False):
                            self.last_y_ok = False
                            self.step_waiting = True
                            self.step_wait_start_time = time.time()
                            self.cmd_pub.publish(Twist())
                            self.get_logger().info(">>> Trục Y chuẩn! Nghỉ 0.5s dập quán tính dịch ngang, BẮT ĐẦU KÍCH HOẠT TIẾN X TRACKING...")
                            return

                        if self.docking_start_pose is None:
                            self.docking_start_pose = self.current_pose
                            self.get_logger().info(f"-> [GHI MỐC XUẤT PHÁT]: Đã lưu vị trí ban đầu tại Odom X: {self.current_pose.x:.3f}")
                            
                            # Chốt mốc vị trí AMCL trước khi tiến và khoảng cách cần đi mục tiêu
                            if self.current_amcl_pose is not None:
                                self.pickup_amcl_start_x = self.current_amcl_pose.position.x
                                self.pickup_amcl_start_y = self.current_amcl_pose.position.y
                            else:
                                self.pickup_amcl_start_x = self.current_pose.x
                                self.pickup_amcl_start_y = self.current_pose.y
                            self.pickup_target_dist = error_x

                        self.forward_tracking_active = True
                        self.get_logger().info(">>> [KHÓA MỤC TIÊU TIẾN X]: Chuyển sang chế độ vừa chạy vừa sửa sai liên tục.")
                    
            except TransformException as e:
                pass
        
        # 4. BỘ LỌC DỰ PHÒNG CHỐNG TREO TOÀN DIỆN KHI MẤT DẤU ĐỘT NGỘT GIỮA ĐƯỜNG
        else:
            if self.forward_tracking_active:
                self.cmd_pub.publish(Twist()) 
                
                self.cam_align_duration = time.time() - self.docking_init_time
                
                # Tính bù Odom nốt khoảng cách còn thiếu dựa theo điểm xuất phát ban đầu, triệt tiêu hoàn toàn lỗi đi lố
                dist_already_traveled = np.hypot(self.current_pose.x - self.docking_start_pose.x, self.current_pose.y - self.docking_start_pose.y)
                self.locked_x = max(0.0, self.pickup_target_dist - dist_already_traveled)
                
                self.locked_y = 0.0
                self.locked_theta = 0.0
                self.target_locked = True
                
                self.current_tracking_side = None

                self.state = 'INSERT_X'
                self.start_pose = None
                self.forward_tracking_active = False
                self.get_logger().warn(f"====> ĐỘT NGỘT MẤT DẤU KHI ĐANG TIẾN! Hệ thống tự bù Odom nốt khoảng cách còn lại: {self.locked_x:.3f}m.")

    def blind_sequence_loop(self):
        if self.current_pose is None:
            return

        msg = Twist()
        
        # =========================================================================
        # VÒNG LẶP STATE MACHINE: TRẠM LẤY HÀNG (PICKUP)
        # =========================================================================
        if self.state == 'INSERT_X':
            if self.start_pose is None:
                self.start_pose = self.current_pose
                return
            dist_traveled = np.hypot(self.current_pose.x - self.start_pose.x, self.current_pose.y - self.start_pose.y)
            
            if dist_traveled < self.locked_x:
                msg.linear.x = self.SPEED_FORWARD_X
                msg.linear.y = 0.0
                msg.angular.z = 0.0
                self.cmd_pub.publish(msg)
            else:
                self.cmd_pub.publish(Twist()) # Phanh dừng xe dứt khoát
                self.get_logger().info(f">>> Đã đi mù xong (Đi được: {dist_traveled:.3f}m). Dừng nghỉ 1.0s ổn định cơ khí...")
                
                self.final_pickup_error_x = float(self.locked_x - dist_traveled)
                
                self.state = 'WAIT_BEFORE_LIFT' # Chuyển sang trạng thái trung gian chờ nghỉ
                self.start_action_time = time.time()

        # ---- TRẠM LẤY HÀNG: TRẠNG THÁI CHỜ TRƯỚC KHI NÂNG ----
        elif self.state == 'WAIT_BEFORE_LIFT':
            self.cmd_pub.publish(Twist()) 
            if time.time() - self.start_action_time >= 1.0: 
                self.get_logger().info(">>> Xe đã đứng im ổn định hoàn toàn 1s. Phát lệnh NÂNG CÀNG lên ESP32!")
                
                self.lift_done_flag = False
                lift_msg = Int8(data=1)
                self.lift_pub.publish(lift_msg) 
                
                self.state = 'LIFTING'

        elif self.state == 'LIFTING':
            self.cmd_pub.publish(Twist())
            
            if not hasattr(self, 'last_lift_pub_time'):
                self.last_lift_pub_time = time.time()
            if time.time() - self.last_lift_pub_time >= 0.5: 
                lift_msg = Int8(data=1)
                self.lift_pub.publish(lift_msg)
                self.last_lift_pub_time = time.time()

            if self.lift_done_flag == True:
                self.get_logger().info(f">>> Nâng xong! Dừng im ổn định cơ khí trong {self.TIME_WAIT_MECHANICAL}s...")
                if hasattr(self, 'last_lift_pub_time'):
                    delattr(self, 'last_lift_pub_time')
                self.state = 'WAIT_PICKUP_STOP'
                self.start_action_time = time.time()

        elif self.state == 'WAIT_PICKUP_STOP':
            self.cmd_pub.publish(Twist())
            if time.time() - self.start_action_time >= self.TIME_WAIT_MECHANICAL: 
                self.get_logger().info(">>> Kết thúc thời gian chờ, bắt đầu lùi xe thoát trạm...")
                self.state = 'RETRACT_PICKUP'
                self.start_pose = self.current_pose 

        elif self.state == 'RETRACT_PICKUP':
            if self.docking_start_pose is None:
                self.docking_start_pose = self.start_pose

            total_retract_dist = np.hypot(self.start_pose.x - self.docking_start_pose.x, 
                                          self.start_pose.y - self.docking_start_pose.y)
            
            dist_retracted = np.hypot(self.current_pose.x - self.start_pose.x, 
                                      self.current_pose.y - self.start_pose.y)
            
            if dist_retracted < total_retract_dist:
                msg.linear.x = self.SPEED_RETRACT_X
                msg.linear.y = 0.0
                msg.angular.z = 0.0
                self.cmd_pub.publish(msg)
            else:
                self.cmd_pub.publish(Twist()) 
                self.get_logger().info(f">>>> Đã lùi đủ khoảng cách thoát trạm ({dist_retracted:.3f}m / {total_retract_dist:.3f}m). Tiến hành nghỉ 3s tại HOME...")
                
                self.state = 'WAIT_PICKUP_RETRACT_STOP'
                self.start_action_time = time.time()

        elif self.state == 'WAIT_PICKUP_RETRACT_STOP':
            self.cmd_pub.publish(Twist()) 
            if time.time() - self.start_action_time >= 3.0: 
                self.get_logger().info(">>> Đã di chuyển về HOME và nghỉ đủ 3s. QUY TRÌNH LẤY HÀNG VÀ RÚT CƠ CẤU HOÀN TẤT HOÀN TOÀN!")
                
                self.pickup_counter += 1
                
                # Thay giá trị random bằng thời gian thực thi luồng thực tế chuẩn xác
                self.pickup_data.append({
                    'Lan thu': self.pickup_counter,
                    'Thoi gian Nav2 toi tram (s)': f"{self.pickup_nav2_duration:.2f}", 
                    'Thoi gian Cam can chinh (s)': f"{self.cam_align_duration:.2f}",
                    'Sai so Nav2 ban dau (X/Y/Theta)': f"{self.nav2_error_x:.3f}m / {self.nav2_error_y:.3f}m / {self.nav2_error_theta:.1f}deg",
                    'Sai so sau khi chinh Cam (X/Y/Theta)': f"{self.final_pickup_error_x:.3f}m / {self.last_cam_y_move:.3f}m / {self.last_cam_theta_move:.1f}deg",
                    'Ket qua': 'Thanh cong'
                })
                
                if self.pickup_counter >= self.max_test_runs:
                    self.save_csv_report('report_pickup.csv', self.pickup_data)

                self.nav2_start_time = time.time() 
                self.state = 'IDLE'
                self.action_done = True

        # =========================================================================
        # VÒNG LẶP STATE MACHINE: TRẠM ĐẶT HÀNG (DROPOFF)
        # =========================================================================
        elif self.state == 'DROP_START_WAIT':
            self.cmd_pub.publish(Twist())
            if time.time() - self.start_action_time >= self.TIME_WAIT_MECHANICAL:
                self.get_logger().info(f">>> Hết 1s chờ ban đầu. Tiến vào trạm {self.fixed_dropoff_distance}m (Check Odom)...")
                self.state = 'DROP_FORWARD'
                self.drop_start_pose = self.current_pose  

        elif self.state == 'DROP_FORWARD':
            if self.drop_start_pose is None:
                self.drop_start_pose = self.current_pose
                return
            
            dist_traveled = np.hypot(self.current_pose.x - self.drop_start_pose.x, 
                                     self.current_pose.y - self.drop_start_pose.y)
            
            if dist_traveled < self.fixed_dropoff_distance: 
                msg.linear.x = self.SPEED_FORWARD_X
                self.cmd_pub.publish(msg)
            else:
                self.final_drop_dist = dist_traveled 
                
                self.cmd_pub.publish(Twist()) 
                self.get_logger().info(f">>> Odom báo tiến đủ {self.fixed_dropoff_distance}m. Nghỉ dừng im {self.TIME_WAIT_MECHANICAL}s trước khi hạ càng...")
                self.state = 'DROP_ARRIVED_WAIT'
                self.start_action_time = time.time()

        elif self.state == 'DROP_ARRIVED_WAIT':
            self.cmd_pub.publish(Twist())
            if time.time() - self.start_action_time >= self.TIME_WAIT_MECHANICAL:
                self.get_logger().info(">>> Hết thời gian nghỉ, phát lệnh HẠ CÀNG lên ESP32 để đặt hàng...")
                
                self.lift_done_flag = False 
                lift_msg = Int8(data=0)  
                self.lift_pub.publish(lift_msg)
                
                self.state = 'DROPPING'

        elif self.state == 'DROPPING':
            self.cmd_pub.publish(Twist()) 
            
            if not hasattr(self, 'last_drop_pub_time'):
                self.last_drop_pub_time = time.time()
                
            if time.time() - self.last_drop_pub_time >= 0.5: 
                lift_msg = Int8(data=0)
                self.lift_pub.publish(lift_msg)
                self.last_drop_pub_time = time.time()

            if self.lift_done_flag == True:
                self.get_logger().info(f">>> Càng đã hạ xong! Dừng nghỉ ổn định {self.TIME_WAIT_MECHANICAL}s trước khi lùi thoát...")
                
                if hasattr(self, 'last_drop_pub_time'):
                    delattr(self, 'last_drop_pub_time')
                    
                self.state = 'DROP_LIFTED_WAIT'
                self.start_action_time = time.time()

        elif self.state == 'DROP_LIFTED_WAIT':
            self.cmd_pub.publish(Twist())
            if time.time() - self.start_action_time >= self.TIME_WAIT_MECHANICAL:
                self.get_logger().info(">>> Hết 1s nghỉ, ghi mốc và bắt đầu lùi xe theo Odom để thoát khỏi trạm...")
                
                self.drop_retract_start_pose = self.current_pose 
                self.state = 'RETRACT_DROP'

        elif self.state == 'RETRACT_DROP':
            if self.drop_start_pose is None:
                self.get_logger().error("Mất mốc Odom gốc của trạm thả hàng!")
                self.state = 'IDLE'
                self.action_done = True
                return
            
            if self.drop_retract_start_pose is None:
                self.drop_retract_start_pose = self.current_pose

            total_drop_retract_dist = np.hypot(self.drop_retract_start_pose.x - self.drop_start_pose.x, 
                                               self.drop_retract_start_pose.y - self.drop_start_pose.y)
            
            dist_drop_retracted = np.hypot(self.current_pose.x - self.drop_retract_start_pose.x, 
                                           self.current_pose.y - self.drop_retract_start_pose.y)
            
            if dist_drop_retracted < total_drop_retract_dist: 
                msg.linear.x = self.SPEED_RETRACT_X
                msg.linear.y = 0.0
                msg.angular.z = 0.0
                self.cmd_pub.publish(msg)
            else:
                self.cmd_pub.publish(Twist()) 
                self.get_logger().info(f">>> Đã lùi đủ khoảng cách thả hàng ({dist_drop_retracted:.3f}m / {total_drop_retract_dist:.3f}m). Tiến hành nghỉ 3s tại HOME...")
                self.state = 'WAIT_DROP_RETRACT_WAIT'
                self.start_action_time = time.time()

        elif self.state == 'WAIT_DROP_RETRACT_WAIT':
            self.cmd_pub.publish(Twist()) 
            if time.time() - self.start_action_time >= 3.0: 
                self.get_logger().info(">>> Đã di chuyển về HOME và nghỉ đủ 3s. QUY TRÌNH THẢ HÀNG (DROPOFF) HOÀN TẤT AN TOÀN TUYỆT ĐỐI!")
                
                self.dropoff_counter += 1
                
                actual_dist = getattr(self, 'final_drop_dist', self.fixed_dropoff_distance) 
                blind_error_mm = abs(self.fixed_dropoff_distance - actual_dist) * 1000 
                
                final_angle_error = math.degrees(self.current_theta) - self.dropoff_goal_theta

                # Thay giá trị random bằng thời gian thực tế chuẩn xác của trạm Dropoff
                self.dropoff_data.append({
                    'Lan thu': self.dropoff_counter,
                    'Thoi gian Nav2 toi tram (s)': f"{self.dropoff_nav2_duration:.2f}", 
                    'Sai so Nav2 ban dau (X/Y/Theta)': f"{self.nav2_error_x:.3f}m / {self.nav2_error_y:.3f}m / {self.nav2_error_theta:.1f}deg",
                    'Quang duong tien thuc te (m)': f"{actual_dist:.3f}",
                    'Sai số Tiến mù (mm)': f"{blind_error_mm:.1f} mm",
                    'Sai so goc Odom khi lui ve (deg)': f"{final_angle_error:.2f} deg",
                    'Ket qua': 'Thanh cong'
                })
                
                if self.dropoff_counter >= self.max_test_runs:
                    self.save_csv_report('report_dropoff.csv', self.dropoff_data)

                self.nav2_start_time = time.time() 
                self.state = 'IDLE'
                self.action_done = True

    def save_csv_report(self, filename, data):
        try:
            keys = data[0].keys()
            with open(filename, 'w', newline='', encoding='utf-8') as output_file:
                dict_writer = csv.DictWriter(output_file, fieldnames=keys)
                dict_writer.writeheader()
                dict_writer.writerows(data)
            self.get_logger().info(f"\n=========================================================\n[SUCCESS] ĐÃ TỰ ĐỘNG XUẤT FILE BÁO CÁO: {filename}\n=========================================================")
        except Exception as e:
            self.get_logger().error(f"Lỗi khi xuất file CSV: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = DockingServer()
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