#include <Arduino.h>
#include <FastAccelStepper.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32_multi_array.h>


/* ========== CMD_VEL FILTER ========== */
#define CMD_ALPHA 0.1
float vx_f = 0.0;
float vy_f = 0.0;
float wz_f = 0.0;

/* ========== PIN ========== */
#define FL_STEP 18
#define FL_DIR  5
#define FR_STEP 27
#define FR_DIR  26
#define RL_STEP 17
#define RL_DIR  16
#define RR_STEP 23
#define RR_DIR  22

/* ========== PARAM ========== */
const float WHEEL_RADIUS = 0.05;
const float L = 0.40;
const int STEPS_PER_REV = 800;

/* Deadzone chỉ cho điều khiển */
#define CMD_VEL_DEADZONE 0.01  

unsigned long last_cmd_time = 0; // Biến watchdog

/* ========== STEPPER ========== */
FastAccelStepperEngine engine;
FastAccelStepper *motor_fl, *motor_fr, *motor_rl, *motor_rr;

/* ========== micro-ROS ========== */
rcl_node_t node;
rcl_subscription_t cmd_sub;
rcl_publisher_t pub_ticks;
rcl_timer_t pub_timer;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

geometry_msgs__msg__Twist cmd_msg;
std_msgs__msg__Int32MultiArray ticks_msg;

/* ========== HELPER ========== */
float filter_deadzone(float v) {
  if (fabs(v) < CMD_VEL_DEADZONE) return 0.0;
  return v;
}

/* ========== MOTOR CONTROL ========== */
void set_mecanum_speed(float vx, float vy, float wz) {
  vx = filter_deadzone(vx);
  vy = filter_deadzone(vy);
  wz = filter_deadzone(wz);

  float w[4];
  w[0] = (vx - vy - L * wz) / WHEEL_RADIUS; // FL (rad/s)
  w[1] = (vx + vy + L * wz) / WHEEL_RADIUS; // FR
  w[2] = (vx + vy - L * wz) / WHEEL_RADIUS; // RL
  w[3] = (vx - vy + L * wz) / WHEEL_RADIUS; // RR

  FastAccelStepper* motors[4] = {motor_fl, motor_fr, motor_rl, motor_rr};

  for (int i = 0; i < 4; i++) {
    float step_hz = (w[i] / (2 * PI)) * STEPS_PER_REV;

    if (fabs(step_hz) < 1.0) { // Ngưỡng dừng cực nhỏ
      motors[i]->stopMove();
      motors[i]->setSpeedInHz(0); // Ép tần số về 0
    } else {
      motors[i]->setSpeedInHz(fabs(step_hz));
      if (step_hz > 0) motors[i]->runForward();
      else motors[i]->runBackward();
    }
  }
}

/* ========== ROS CALLBACK ========== */
void cmd_vel_cb(const void *msg) {
  last_cmd_time = millis(); // Cập nhật thời gian nhận lệnh
  const geometry_msgs__msg__Twist *tw = (const geometry_msgs__msg__Twist *)msg;

  // 1. NẾU NHẬN LỆNH DỪNG TUYỆT ĐỐI
  if (tw->linear.x == 0 && tw->linear.y == 0 && tw->angular.z == 0) {
    vx_f = 0.0; vy_f = 0.0; wz_f = 0.0;
    motor_fl->stopMove();
    motor_fr->stopMove();
    motor_rl->stopMove();
    motor_rr->stopMove();
    return; 
  }

  // 2. TĂNG ALPHA ĐỂ PHẢN ỨNG NHANH
  vx_f += CMD_ALPHA * (tw->linear.x  - vx_f);
  vy_f += CMD_ALPHA * (tw->linear.y  - vy_f);
  wz_f += CMD_ALPHA * (tw->angular.z - wz_f);

  set_mecanum_speed(vx_f, vy_f, wz_f);
}

/* ========== TIMER CALLBACK (50Hz) ========== */
void publish_steps(rcl_timer_t *, int64_t) {
  ticks_msg.data.data[0] = motor_fl->getCurrentPosition();
  ticks_msg.data.data[1] = motor_fr->getCurrentPosition();
  ticks_msg.data.data[2] = motor_rl->getCurrentPosition();
  ticks_msg.data.data[3] = motor_rr->getCurrentPosition();

  rcl_publish(&pub_ticks, &ticks_msg, NULL);
}

/* ========== SETUP ========== */
void setup() {
  last_cmd_time = millis();
  set_microros_transports();
  Serial.begin(115200);
  delay(2000);

  engine.init();

  motor_fl = engine.stepperConnectToPin(FL_STEP);
  motor_fr = engine.stepperConnectToPin(FR_STEP);
  motor_rl = engine.stepperConnectToPin(RL_STEP);
  motor_rr = engine.stepperConnectToPin(RR_STEP);

  motor_fl->setDirectionPin(FL_DIR);
  motor_fr->setDirectionPin(FR_DIR);
  motor_rl->setDirectionPin(RL_DIR);
  motor_rr->setDirectionPin(RR_DIR);

  motor_fl->setAcceleration(10000);
  motor_fr->setAcceleration(10000);
  motor_rl->setAcceleration(10000);
  motor_rr->setAcceleration(10000);

  motor_fl->setAutoEnable(true);
  motor_fr->setAutoEnable(true);
  motor_rl->setAutoEnable(true);
  motor_rr->setAutoEnable(true);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_motor_node", "", &support);

  rclc_subscription_init_default(
    &cmd_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"
  );

  rclc_publisher_init_default(
    &pub_ticks,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "/wheel_ticks"
  );

  /* CẤP PHÁT BỘ NHỚ – BẮT BUỘC */
  ticks_msg.data.capacity = 4;
  ticks_msg.data.size = 4;
  ticks_msg.data.data = (int32_t *)malloc(4 * sizeof(int32_t));

  rclc_timer_init_default(
    &pub_timer,
    &support,
    RCL_MS_TO_NS(20),   // 50Hz
    publish_steps
  );

  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_subscription(
    &executor, &cmd_sub, &cmd_msg, &cmd_vel_cb, ON_NEW_DATA
  );
  rclc_executor_add_timer(&executor, &pub_timer);
}

/* ========== LOOP ========== */
void loop() {
  if (millis() - last_cmd_time > 500) {
    vx_f = 0; vy_f = 0; wz_f = 0;
    motor_fl->stopMove();
    motor_fr->stopMove();
    motor_rl->stopMove();
    motor_rr->stopMove();
  }

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
