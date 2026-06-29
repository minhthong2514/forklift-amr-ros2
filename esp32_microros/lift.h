#ifndef LIFT_H
#define LIFT_H

#include "config.h"

extern rcl_publisher_t pub_lift_status;
extern std_msgs__msg__Bool lift_status_msg;
bool is_lifting_active = false;
int current_lift_state = 0; // 0: Đang ở dưới (0cm), 1: Đang ở trên (15cm)

extern FastAccelStepperEngine engine;
FastAccelStepper *motor_lift = NULL;

int CalculateSteps(float dist) {
  return (int)((PulsePerRound * abs(dist)) / DistPerRound);
}

int CalculateFrequency(int RPM) {
  return (abs(RPM) * PulsePerRound) / 60;
}

// ==============================================================
// HÀM HOMING ĐƯỢC THIẾT KẾ ĐỂ KHÔNG LÀM RỚT MẠNG MICRO-ROS
// ==============================================================
inline void homing(rclc_executor_t *executor) {
  if (!motor_lift) return;
  
  if (digitalRead(LIMIT_SWITCH) == 1) {
    current_lift_state = 0;
    motor_lift->setCurrentPosition(0); // Set gốc tọa độ 0
    return;
  }

  Serial.println("Bat dau Homing...");
  motor_lift->setSpeedInHz(CalculateFrequency(LIFT_SPEED));
  motor_lift->setAcceleration(5000); 
  motor_lift->runBackward(); // Cho động cơ chạy lùi liên tục

  // Vòng lặp chờ chạm công tắc
  while (digitalRead(LIMIT_SWITCH) != 1) {
    // QUAN TRỌNG NHẤT: Trong lúc chờ động cơ chạy, vẫn phải cho micro-ROS hoạt động
    // để Jetson không nghĩ rằng ESP32 đã chết!
    if (executor != NULL) {
      rclc_executor_spin_some(executor, RCL_MS_TO_NS(10));
    }
    delay(5); 
  }

  // Đã chạm công tắc -> Dừng khẩn cấp
  motor_lift->forceStopAndNewPosition(0); 
  current_lift_state = 0;
}

// ==============================================================
// CÁC HÀM ĐIỀU KHIỂN BẰNG THƯ VIỆN FASTACCELSTEPPER
// ==============================================================
void rotateForward(float dist) {
  int32_t steps = CalculateSteps(dist);
  motor_lift->setSpeedInHz(CalculateFrequency(LIFT_SPEED));
  motor_lift->moveTo(steps); // Chạy tới vị trí tuyệt đối
}

void rotateBackward(float dist) {
  motor_lift->setSpeedInHz(CalculateFrequency(LIFT_SPEED));
  motor_lift->moveTo(0); // Chạy lùi về đúng gốc 0 đã set lúc Homing
}

inline void control_lift(int cmd) {
  if (!motor_lift) return;
  if (motor_lift->isRunning()) return; // Đang chạy dở thì bỏ qua lệnh mới

  if (cmd == 1 && current_lift_state == 0) {
    rotateForward(LIFT_HEIGHT);
    is_lifting_active = true;
    current_lift_state = 1; 

  } else if (cmd == 0 && current_lift_state == 1) {
    rotateBackward(LIFT_HEIGHT);
    is_lifting_active = true;
    current_lift_state = 0; 
  }
}

// Báo cáo trạng thái về Jetson
void check_lift_status() {
  if (!motor_lift) return;

  bool currently_moving = motor_lift->isRunning();

  if (is_lifting_active && !currently_moving) {
    is_lifting_active = false;
    lift_status_msg.data = true;
    rcl_publish(&pub_lift_status, &lift_status_msg, NULL);
  }
}

#endif
