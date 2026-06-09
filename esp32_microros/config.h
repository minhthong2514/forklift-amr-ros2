#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <FastAccelStepper.h>
#include "DFRobot_BNO055.h"
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <rmw_microros/rmw_microros.h>

/* --- 1. SETTINGS & PINS --- */
#define LED_PIN 2
#define FL_STEP 18
#define FL_DIR 5
#define FR_STEP 27
#define FR_DIR 26
#define RL_STEP 17
#define RL_DIR 16
#define RR_STEP 33
#define RR_DIR 32

const float WHEEL_RADIUS = 0.05, L = 0.3965; // Lx (0.205) + Ly (0.1915)
const int STEPS_PER_REV = 800;
#define CMD_ALPHA 0.10
#define CMD_VEL_DEADZONE 0.005
const float mg_to_ms2 = 0.00980665;
const float deg_to_rad = PI / 180.0;

/* --- 2. KHAI BÁO EXTERN --- */
typedef DFRobot_BNO055_IIC BNO;
extern BNO bno;
extern FastAccelStepperEngine engine;
extern FastAccelStepper *motor_fl, *motor_fr, *motor_rl, *motor_rr;

extern rcl_node_t node;
extern rcl_subscription_t cmd_sub;
extern rcl_publisher_t pub_ticks, pub_imu;
extern rcl_timer_t pub_timer;
extern rclc_executor_t executor;
extern rclc_support_t support;
extern rcl_allocator_t allocator;

extern geometry_msgs__msg__Twist cmd_msg;
extern std_msgs__msg__Int32MultiArray ticks_msg;
extern sensor_msgs__msg__Imu imu_msg;

extern float vx_f, vy_f, wz_f;
extern float current_wheel_hz[4];
extern unsigned long last_cmd_time;

#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }

void error_loop();
void cmd_vel_cb(const void* msgin);
void publish_timer_cb(rcl_timer_t* timer, int64_t last_call_time);

/* --- 3. HÀM KHỞI TẠO --- */
inline void InitMotors() {
  engine.init();
  motor_fl = engine.stepperConnectToPin(FL_STEP);
  motor_fl->setDirectionPin(FL_DIR);
  motor_fr = engine.stepperConnectToPin(FR_STEP);
  motor_fr->setDirectionPin(FR_DIR);
  motor_rl = engine.stepperConnectToPin(RL_STEP);
  motor_rl->setDirectionPin(RL_DIR);
  motor_rr = engine.stepperConnectToPin(RR_STEP);
  motor_rr->setDirectionPin(RR_DIR);
  FastAccelStepper* m[4] = { motor_fl, motor_fr, motor_rl, motor_rr };
  for (int i = 0; i < 4; i++) {
    m[i]->setAcceleration(5000);  // Giảm gia tốc xuống 5000 để êm hơn
    m[i]->setAutoEnable(true);
  }
}

inline void InitSensors() {
  bno.reset();
  
  uint8_t retry = 0;
  while (bno.begin() != BNO::eStatusOK && retry < 10) { 
    delay(100); 
    retry++;
  }

  bno.setOprMode(BNO::eOprModeConfig);
  delay(100);

  BNO::sAxisAnalog_t sOffsetGyr;
  sOffsetGyr.x = 0; 
  sOffsetGyr.y = 0;
  sOffsetGyr.z = 0.04; // Giá trị bạn đã đo được để triệt tiêu Gz

  BNO::sAxisAnalog_t sOffsetAcc;
  sOffsetAcc.x = 0.01;
  sOffsetAcc.y = 0.01;
  sOffsetAcc.z = 0;

  bno.setAxisOffset(BNO::eAxisGyr, sOffsetGyr);
  bno.setAxisOffset(BNO::eAxisAcc, sOffsetAcc);

  bno.setOprMode(BNO::eOprModeImu); 
  
  delay(3000); 
  
  // Serial.println("IMU initialized with Hardware Offsets and 6-DOF mode.");
}

inline void InitMicroros() {
  set_microros_transports();
  delay(2000);
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_motor_node", "", &support));

  // Hàm động bộ thời gian với ROS2
  rmw_uros_sync_session(1000);

  RCCHECK(rclc_subscription_init_default(&cmd_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel_smoothed"));
  RCCHECK(rclc_publisher_init_default(&pub_ticks, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "/wheel_ticks"));
  RCCHECK(rclc_publisher_init_best_effort(&pub_imu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu_data"));

  static int32_t ticks_mem[4] = { 0, 0, 0, 0 };
  ticks_msg.data.capacity = 4;
  ticks_msg.data.size = 4;
  ticks_msg.data.data = ticks_mem;

  static char imu_name[] = "imu_link";
  imu_msg.header.frame_id.data = imu_name;
  imu_msg.header.frame_id.size = strlen(imu_name);
  imu_msg.header.frame_id.capacity = strlen(imu_name) + 1;

  // Cài đặt Timer 20ms (50Hz)
  RCCHECK(rclc_timer_init_default(&pub_timer, &support, 1000000000ULL / 100, publish_timer_cb)); //60hz
  // RCCHECK(rclc_timer_init_default(&pub_timer, &support, RCL_MS_TO_NS(20), publish_timer_cb));
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &pub_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &cmd_vel_cb, ON_NEW_DATA));
}
#endif
