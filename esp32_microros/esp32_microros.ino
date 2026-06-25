#include "config.h"
#include "motor.h"
#include "sensor.h"
#include "lift.h"

/* --- ĐỊNH NGHĨA THỰC TẾ --- */
BNO bno(&Wire, 0x28);
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *motor_fl = NULL, *motor_fr = NULL, *motor_rl = NULL, *motor_rr = NULL;

rcl_node_t node;
rcl_subscription_t cmd_sub;
rcl_publisher_t pub_ticks, pub_imu;
rcl_timer_t pub_timer;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
geometry_msgs__msg__Twist cmd_msg;
std_msgs__msg__Int32MultiArray ticks_msg;
sensor_msgs__msg__Imu imu_msg;

rcl_subscription_t lift_sub;
std_msgs__msg__Int8 lift_msg;
rcl_publisher_t pub_lift_status;
std_msgs__msg__Bool lift_status_msg;

float vx_f = 0, vy_f = 0, wz_f = 0;
float current_wheel_hz[4] = { 0, 0, 0, 0 };
unsigned long last_cmd_time = 0;
double manual_ticks[4] = { 0, 0, 0, 0 };
unsigned long last_odom_update = 0;

void update_manual_odom() {
  unsigned long now = micros();
  if (last_odom_update == 0) {
    last_odom_update = now;
    return;
  }
  double dt = (double)(now - last_odom_update) / 1000000.0;
  last_odom_update = now;
  for (int i = 0; i < 4; i++) manual_ticks[i] += (double)current_wheel_hz[i] * dt;
}

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void cmd_vel_cb(const void *msgin) {
  const geometry_msgs__msg__Twist *tw = (const geometry_msgs__msg__Twist *)msgin;

  float target_vx = (fabs(tw->linear.x) < CMD_VEL_DEADZONE) ? 0 : tw->linear.x;
  float target_vy = (fabs(tw->linear.y) < CMD_VEL_DEADZONE) ? 0 : tw->linear.y;
  float target_wz = (fabs(tw->angular.z) < CMD_VEL_DEADZONE) ? 0 : tw->angular.z;

  last_cmd_time = millis();

  vx_f += CMD_ALPHA * (target_vx - vx_f);
  vy_f += CMD_ALPHA * (target_vy - vy_f);
  wz_f += CMD_ALPHA * (target_wz - wz_f);

  if (target_vx == 0 && target_vy == 0 && target_wz == 0 && fabs(vx_f) < 0.005 && fabs(vy_f) < 0.005 && fabs(wz_f) < 0.005) {
    vx_f = 0;
    vy_f = 0;
    wz_f = 0;
    stop_all_motors();
  } else {
    update_manual_odom();
    set_mecanum_speed(vx_f, vy_f, wz_f);
  }
}

void publish_timer_cb(rcl_timer_t *timer, int64_t last_call_time) {
  if (timer == NULL) return;

  // 1. Luôn cập nhật Odom nội bộ để đảm bảo độ chính xác
  update_manual_odom();

  static bool toggle_bit = true;
  // struct timespec ts;
  // clock_gettime(CLOCK_REALTIME, &ts);

  int64_t time_ns = rmw_uros_epoch_nanos();

  // 2. Chia nhịp gửi dữ liệu (Interleaving)
  // Việc này giúp Bus I2C không bị ép cùng lúc với tác vụ gửi Serial nặng
  if (toggle_bit) {
    // NHỊP A: Gửi Odom Ticks
    for (int i = 0; i < 4; i++) ticks_msg.data.data[i] = (int32_t)manual_ticks[i];
    RCSOFTCHECK(rcl_publish(&pub_ticks, &ticks_msg, NULL));
  } else {
    // NHỊP B: Đọc và Gửi IMU
    IMURawData raw = readSensor();
    if (!isnan(raw.qw)) {
      // imu_msg.header.stamp.sec = ts.tv_sec;
      // imu_msg.header.stamp.nanosec = ts.tv_nsec;
      imu_msg.header.stamp.sec = (int32_t)(time_ns / 1000000000);
      imu_msg.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000);

      imu_msg.orientation.w = raw.qw;
      imu_msg.orientation.x = raw.qx;
      imu_msg.orientation.y = raw.qy;
      imu_msg.orientation.z = raw.qz;
      imu_msg.angular_velocity.x = raw.gx;
      imu_msg.angular_velocity.y = raw.gy;
      imu_msg.angular_velocity.z = raw.gz;
      imu_msg.linear_acceleration.x = raw.ax;
      imu_msg.linear_acceleration.y = raw.ay;
      imu_msg.linear_acceleration.z = raw.az;

      RCSOFTCHECK(rcl_publish(&pub_imu, &imu_msg, NULL));
    }
  }
  toggle_bit = !toggle_bit;
}

//// Hàm này tự kích hoạt khi Jetson gửi số 1 (Nâng) hoặc 0 (Hạ)
void lift_cmd_callback(const void *msgin) {
  const std_msgs__msg__Int8 *msg = (const std_msgs__msg__Int8 *) msgin;
  control_lift(msg->data);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  // engine.init();
  pinMode(LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(LIFT_STEP, OUTPUT);
  pinMode(LIFT_DIR, OUTPUT);
  digitalWrite(LIFT_STEP, LOW);

  // micro-ROS vận hành trên cổng Serial 921600
  InitMicroros();
  homing();


  // Khởi tạo Bus I2C với các thông số an toàn
  Wire.begin();
  Wire.setClock(100000);
  Wire.setTimeOut(10);  // Quan trọng: Tránh treo chip khi nhiễu bus

  InitSensors();
  InitMotors();
  last_odom_update = micros();

}

void loop() {
  //  check_lift_status();
  // Watchdog dừng motor nếu mất kết nối lệnh (200ms)
  if (millis() - last_cmd_time > 200) {
    if (vx_f != 0 || vy_f != 0 || wz_f != 0) {
      vx_f = 0;
      vy_f = 0;
      wz_f = 0;
      stop_all_motors();
    }
  }

  update_manual_odom();

  // Spin executor với thời gian 10ms để cân bằng hiệu năng
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
