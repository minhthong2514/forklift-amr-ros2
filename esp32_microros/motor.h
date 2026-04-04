#ifndef MOTOR_H
#define MOTOR_H

#include "config.h"

/* CHỈ KHAI BÁO EXTERN Ở ĐÂY */
extern float vx_f, vy_f, wz_f;
extern unsigned long last_cmd_time;
extern float current_wheel_hz[4];
extern FastAccelStepper *motor_fl, *motor_fr, *motor_rl, *motor_rr;

inline void stop_all_motors() {
  FastAccelStepper* motors[4] = { motor_fl, motor_fr, motor_rl, motor_rr };
  for (int i = 0; i < 4; i++) {
    if (motors[i]) {
      motors[i]->stopMove();
      current_wheel_hz[i] = 0.0;
    }
  }
}

inline void set_mecanum_speed(float vx, float vy, float wz) {
  float w[4];
  w[0] = (vx - vy - L * wz) / WHEEL_RADIUS;  // FL
  w[1] = (vx + vy + L * wz) / WHEEL_RADIUS;  // FR
  w[2] = (vx + vy - L * wz) / WHEEL_RADIUS;  // RL
  w[3] = (vx - vy + L * wz) / WHEEL_RADIUS;  // RR

  FastAccelStepper* motors[4] = { motor_fl, motor_fr, motor_rl, motor_rr };
  for (int i = 0; i < 4; i++) {
    if (motors[i] == NULL) continue;
    float step_hz = (w[i] / (2.0 * PI)) * (float)STEPS_PER_REV;
    current_wheel_hz[i] = step_hz;
    float abs_hz = fabs(step_hz);

    if (abs_hz < 1.0) {
      motors[i]->stopMove();
      current_wheel_hz[i] = 0.0;
    } else {
      motors[i]->setSpeedInHz((uint32_t)abs_hz);
      if (step_hz > 0) motors[i]->runForward();
      else motors[i]->runBackward();
    }
  }
}
#endif