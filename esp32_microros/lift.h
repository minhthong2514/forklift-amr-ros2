#ifndef LIFT_H
#define LIFT_H

#include "config.h"

// Khai báo công cụ báo cáo cho ROS 2 (Sẽ khởi tạo ở file .ino)
extern rcl_publisher_t pub_lift_status;
extern std_msgs__msg__Bool lift_status_msg;
bool is_lifting_active = false;
int current_lift_state = 0;

//extern float CURRENT_POSITION;
// Hàm tính số xung dựa trên khoảng cách (cm)
int CalculateSteps(float dist) {
  return (int)((PulsePerRound * abs(dist)) / DistPerRound);
}

// Hàm tính tần số (Hz) từ tốc độ RPM
int CalculateFrequency(int RPM) {
  int frequency = (abs(RPM) * PulsePerRound) / 60;
  return frequency;
}

// Hàm tạo xung mềm theo tần số mong muốn
void generatePulses(int totalSteps, int frequency) {
  if (frequency <= 0) return;

  long T = 1000000 / frequency; // Chu kỳ xung (microseconds)
  long T_HIGH = 50;             // Thời gian mức cao cố định 50us
  long T_LOW = T - T_HIGH;      // Thời gian mức thấp

  if (T_LOW < 50) T_LOW = 50;   // Bảo vệ nếu tần số quá cao

  for (int i = 0; i < totalSteps; i++) {
    digitalWrite(LIFT_STEP, HIGH);
    delayMicroseconds(T_HIGH);
    digitalWrite(LIFT_STEP, LOW);
    delayMicroseconds(T_LOW);
  }
}

// Hàm quay thuận (Nâng lên) - Sử dụng LIFT_SPEED tổng
inline void rotateForward(float dist) {
  int steps = CalculateSteps(dist);
  digitalWrite(LIFT_DIR, HIGH);
  int freq = CalculateFrequency(LIFT_SPEED); // Lấy tốc độ tổng
  generatePulses(steps, freq);
}

// Hàm quay nghịch (Hạ xuống) - Sử dụng LIFT_SPEED tổng
inline void rotateBackward(float dist) {
  int steps = CalculateSteps(dist);
  digitalWrite(LIFT_DIR, LOW);
  int freq = CalculateFrequency(LIFT_SPEED); // Lấy tốc độ tổng
  generatePulses(steps, freq);
}

// Hàm tìm điểm gốc (Homing) - Sử dụng LIFT_SPEED tổng
inline void homing() {
  digitalWrite(LIFT_DIR, LOW);

  int freq = CalculateFrequency(LIFT_SPEED); // Lấy tốc độ tổng
  long T = 1000000 / freq;
  long T_HALF = T / 2;
  //
  //  Serial.print("Dang Homing voi toc do tong... Tan so: ");
  //  Serial.print(freq);
  //  Serial.println(" Hz");

  while (digitalRead(LIMIT_SWITCH) != 1) {
    digitalWrite(LIFT_STEP, HIGH);
    delayMicroseconds(T_HALF);
    digitalWrite(LIFT_STEP, LOW);
    delayMicroseconds(T_HALF);
  }

  //  CURRENT_POSITION = 0.0f;
  //  Serial.println("Homing HOAN THANH! Vi tri hien tai da reset ve 0.0 cm");
}

inline void control_lift(int cmd) {
  // Chỉ nâng khi có lệnh Nâng (1) VÀ càng đang ở dưới (0)
  if (cmd == 1 && current_lift_state == 0) {
    rotateForward(LIFT_HEIGHT);
    is_lifting_active = true;
    current_lift_state = 1; // Cập nhật trạng thái đã nâng

    // Chỉ hạ khi có lệnh Hạ (0) VÀ càng đang ở trên (1)
  } else if (cmd == 0 && current_lift_state == 1) {
    rotateBackward(LIFT_HEIGHT);
    is_lifting_active = true;
    current_lift_state = 0; // Cập nhật trạng thái đã hạ
  }
  else {
    // Nếu gửi lệnh trùng lặp, ESP32 sẽ bỏ qua không làm gì cả
    //    Serial.println("Bo qua lenh nang/ha vi trung lap trang thai!");
    return;
  }
}

inline void check_lift_status() {
  // Nếu cờ đang bật (motor đang được giao việc) NHƯNG motor đã ngừng quay (isRunning = false)
  // Tức là nó đã chạy đến đích thành công.
  if (is_lifting_active) {
    is_lifting_active = false; // Tắt cờ

    // Gửi tin nhắn True lên Topic /lift_status cho Jetson biết
    lift_status_msg.data = true;
    rcl_publish(&pub_lift_status, &lift_status_msg, NULL);
  }
}

#endif
