#ifndef SENSOR_H
#define SENSOR_H
#include "config.h"

struct IMURawData {
  float qw, qx, qy, qz; // Quaternion
  float gx, gy, gz;     // Angular velocity
  float ax, ay, az;     // Linear acceleration
};

inline IMURawData readSensor() {
  IMURawData data;
  
  // 1. Đọc Quaternion (Hướng)
  BNO::sQuaAnalog_t sQua = bno.getQua();
  data.qw = sQua.w; data.qx = sQua.x; data.qy = sQua.y; data.qz = sQua.z;

  // 2. Đọc Gyro (Vận tốc góc - đổi sang rad/s)
  BNO::sAxisAnalog_t sGyr = bno.getAxis(BNO::eAxisGyr);
  data.gx = sGyr.x * deg_to_rad;
  data.gy = sGyr.y * deg_to_rad;
  data.gz = sGyr.z * deg_to_rad;

  // 3. Đọc Linear Accel (Gia tốc - đổi sang m/s^2)
  BNO::sAxisAnalog_t sLia = bno.getAxis(BNO::eAxisLia);
  data.ax = sLia.x * mg_to_ms2;
  data.ay = sLia.y * mg_to_ms2;
  data.az = sLia.z * mg_to_ms2;

  return data;
}

#endif