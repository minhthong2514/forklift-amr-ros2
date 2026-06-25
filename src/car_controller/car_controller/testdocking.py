#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import pyzed.sl as sl
import cv2
import numpy as np
from ultralytics import YOLO
import time
import math
import tf2_geometry_msgs

class LiveDockingTest(Node):
    def __init__(self):
        super().__init__('live_docking_test')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # THÔNG SỐ CƠ KHÍ
        self.L_tam_den_goc_cang = 0.315  
        self.khoang_cach_an_toan = 0.02  

        self.get_logger().info("Đang tải model YOLO...")
        self.model = YOLO('/home/tai/Ung_dung/Code/Python/Camera_ZED/model/best (1).pt', task="obb")

        self.get_logger().info("Đang mở Camera ZED...")
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL
        init_params.coordinate_units = sl.UNIT.METER
        
        if self.zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error("LỖI: Không thể mở Camera ZED!")
            return

        cam_info = self.zed.get_camera_information()
        intrin = cam_info.camera_configuration.calibration_parameters.right_cam
        self.fx, self.fy, self.cx, self.cy = intrin.fx, intrin.fy, intrin.cx, intrin.cy

        self.timer = self.create_timer(0.1, self.live_process_callback)
        self.prev_time = time.time()
        self.get_logger().info("=== HỆ THỐNG TEST LIVE ĐÃ SẴN SÀNG ===")

    def live_process_callback(self):
        runtime_params = sl.RuntimeParameters()
        image_zed = sl.Mat()
        depth_zed = sl.Mat()

        if self.zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            curr_time = time.time()
            fps = 1 / (curr_time - self.prev_time)
            self.prev_time = curr_time

            self.zed.retrieve_image(image_zed, sl.VIEW.RIGHT)
            self.zed.retrieve_measure(depth_zed, sl.MEASURE.DEPTH)
            frame_bgr = cv2.cvtColor(image_zed.get_data(), cv2.COLOR_BGRA2BGR)
            
            results = self.model.predict(frame_bgr, conf=0.8, device=0, verbose=False)

            best_x, best_y, best_z, best_th = None, None, None, None
            
            for result in results:
                if result.obb is not None:
                    names = result.names
                    pallet_list = []
                    pocket_list = []

                    for i, box in enumerate(result.obb.xywhr):
                        x_c, y_c, w, h, r = box.tolist()
                        cls = int(result.obb.cls[i])    
                        label = names[cls]
                        center_x, center_y = int(x_c), int(y_c)
                        points = result.obb.xyxyxyxy[i].cpu().numpy().astype(np.int32)

                        depth_values = []
                        for i_off in range(-2, 3):
                            for j_off in range(-2, 3):
                                check_x = np.clip(center_x + i_off, 0, frame_bgr.shape[1] - 1)
                                check_y = np.clip(center_y + j_off, 0, frame_bgr.shape[0] - 1)
                                err, val = depth_zed.get_value(check_x, check_y)
                                if np.isfinite(val) and val > 0:
                                    depth_values.append(val)

                        real_x, real_y, depth_final = 0.0, 0.0, 0.0
                        if depth_values:
                            depth_final = float(np.median(depth_values))
                            
                            # --- BÙ TRỪ QUANG HỌC LƯỢNG GIÁC ---
                            if label.lower() != 'pallet': 
                                cos_goc = max(0.5, abs(math.cos(r))) 
                                offset_xuyen_thau = 0.173 / cos_goc
                                depth_final = depth_final - offset_xuyen_thau
                            # -----------------------------------

                            real_x = (center_x - self.cx) * depth_final / self.fx
                            real_y = (center_y - self.cy) * depth_final / self.fy
                            real_x = real_x + 0.06

                        obj_data = {
                            'center': (center_x, center_y), 'points': points,
                            'real_x': real_x, 'real_y': real_y, 'depth': depth_final,
                            'r': r, 'matched': False
                        }
                        
                        if label.lower() == 'pallet':
                            pallet_list.append(obj_data)
                        else:
                            pocket_list.append(obj_data)

                    for pallet in pallet_list:
                        matched_pocket = None
                        min_dist = float('inf')

                        for pocket in pocket_list:
                            if not pocket['matched']:
                                dist = np.hypot(pallet['center'][0] - pocket['center'][0], pallet['center'][1] - pocket['center'][1])
                                if dist < 120:
                                    min_dist = dist
                                    matched_pocket = pocket

                        cv2.polylines(frame_bgr, [pallet['points']], isClosed=True, color=(0, 255, 0), thickness=2)
                        
                        if matched_pocket is not None:
                            matched_pocket['matched'] = True
                            cv2.polylines(frame_bgr, [matched_pocket['points']], isClosed=True, color=(255, 0, 255), thickness=2)

                            theta_deg = matched_pocket['r'] * (180 / np.pi)
                            if theta_deg > 90: theta_deg -= 180
                            elif theta_deg < -90: theta_deg += 180

                            if best_x is None:
                                best_x = matched_pocket['real_x']
                                best_y = matched_pocket['real_y']
                                best_z = matched_pocket['depth']
                                best_th = theta_deg

            cv2.putText(frame_bgr, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("Live Docking Test", frame_bgr)
            cv2.waitKey(1)

            if best_x is not None:
                self.transform_and_print(best_x, best_y, best_z, best_th)

    def transform_and_print(self, x_cam, y_cam, z_cam, theta):
        try:
            point_cam = PointStamped()
            point_cam.header.frame_id = 'zed2'
            point_cam.header.stamp = self.get_clock().now().to_msg()
            point_cam.point.x = float(x_cam)
            point_cam.point.y = float(y_cam)
            point_cam.point.z = float(z_cam)

            point_robot = self.tf_buffer.transform(point_cam, 'base_link', timeout=rclpy.duration.Duration(seconds=0.1))
            
            x_robot = point_robot.point.x
            y_robot = point_robot.point.y
            
            x_move = x_robot - self.L_tam_den_goc_cang - self.khoang_cach_an_toan
            
            print(f"| Lệch Ngang: {y_robot:+.3f}m | Góc Xoay: {theta:+.1f}° | Cần tiến: {x_move:+.3f}m |")

        except TransformException:
            pass 

def main(args=None):
    rclpy.init(args=args)
    node = LiveDockingTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.zed.close()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()