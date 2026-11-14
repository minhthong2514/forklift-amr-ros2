# 02. ESP32 Micro-ROS Firmware & Low-Level Control

This document covers the real-time firmware architecture running on the **ESP32**, including FreeRTOS task scheduling, Micro-ROS serial communication, 4-wheel Mecanum kinematics, and signal conditioning for heavy-load stability.

---

## 1. Real-Time FreeRTOS Architecture

To ensure deterministic motor control loops without blocking serial communication with the Jetson Orin Nano, the firmware is partitioned into isolated FreeRTOS tasks pinned across the ESP32's dual cores:

* **Micro-ROS Communication Task (Core 0):** Manages the `rclc` executor over hardware UART at **$921600\text{ baud}$**, subscribing to smoothed velocity/lift commands (`/cmd_vel_smoothed`, `/lift_cmd`) and publishing raw quadrature encoder ticks (`/wheel_ticks`), IMU telemetry (`/imu_data`), and lift limit-switch states (`/lift_status`).
* **Closed-Loop PID & Kinematics Task (Core 1 — $50\text{ Hz}$ / $20\text{ ms}$):** Reads quadrature encoder ticks, computes inverse Mecanum kinematics, applies digital velocity smoothing, and updates PWM duty cycles for all four drive motors plus the vertical lift actuator.
* **Safety Watchdog:** Automatically zeroes motor PWM outputs if `/cmd_vel_smoothed` heartbeat packets are lost for more than $500\text{ ms}$.

---

## 2. Omnidirectional Mecanum Kinematics

Let $L_x$ be the half-wheelbase (distance from robot center to front/rear axles) and $L_y$ be the half-track width (distance from center to left/right wheels), with wheel radius $R$.

### Inverse Kinematics (Onboard ESP32: Body Velocity $\rightarrow$ Wheel Angular Velocities)
Given target body velocities $(V_x, V_y, \omega_z)$ from `/cmd_vel_smoothed`:

$$\begin{bmatrix} \omega_{FL} \\ \omega_{FR} \\ \omega_{RL} \\ \omega_{RR} \end{bmatrix} = \frac{1}{R} \begin{bmatrix} 1 & -1 & -(L_x + L_y) \\ 1 & 1 & (L_x + L_y) \\ 1 & 1 & -(L_x + L_y) \\ 1 & -1 & (L_x + L_y) \end{bmatrix} \begin{bmatrix} V_x \\ V_y \\ \omega_z \end{bmatrix}$$

### Forward Kinematics (Host Jetson Node: Wheel Ticks $\rightarrow$ Odometry Twist)
Raw encoder ticks (`/wheel_ticks`) are converted into wheel angular velocities $(\hat{\omega}_{FL}, \hat{\omega}_{FR}, \hat{\omega}_{RL}, \hat{\omega}_{RR})$ on the Jetson Orin Nano to compute body-frame odometry before fusion in the Extended Kalman Filter (EKF):

$$\begin{aligned}
V_x &= \frac{R}{4} (\hat{\omega}_{FL} + \hat{\omega}_{FR} + \hat{\omega}_{RL} + \hat{\omega}_{RR}) \\
V_y &= \frac{R}{4} (-\hat{\omega}_{FL} + \hat{\omega}_{FR} + \hat{\omega}_{RL} - \hat{\omega}_{RR}) \\
\omega_z &= \frac{R}{4(L_x + L_y)} (-\hat{\omega}_{FL} + \hat{\omega}_{FR} - \hat{\omega}_{RL} + \hat{\omega}_{RR})
\end{aligned}$$

---

## 3. Signal Conditioning: Digital Low-Pass Filter ($\alpha = 0.1$)

Before feeding target wheel velocities into the PID controllers, raw velocity setpoints pass through a first-order exponential moving average (Low-Pass Filter):

$$v_{\text{filtered}}[k] = \alpha \cdot v_{\text{target}}[k] + (1 - \alpha) \cdot v_{\text{filtered}}[k-1], \quad \alpha = 0.1$$

### Engineering Rationale & Trade-Off Analysis
* **Load Protection & Jerk Suppression:** Unlike lightweight differential-drive robots, a forklift carrying up to $19.34\text{ kg}$ suffers from high center-of-gravity inertia and Mecanum roller slip under sudden step inputs. Setting $\alpha = 0.1$ creates a smooth S-shaped acceleration ramp, eliminating mechanical backlash and preventing pallet shift.
* **Steady-State Velocity Preservation:** While $\alpha = 0.1$ intentionally trades off instantaneous step responsiveness ($\tau \approx 180\text{ ms}$ at $50\text{ Hz}$ loop rate), **it has zero attenuation on steady-state maximum velocity** ($V_{x,\max} = 0.35\text{ m/s}$). Once the transient ramp settles, $v_{\text{filtered}}$ converges identically to $v_{\text{target}}$.

---

## 4. Micro-ROS Topic Interface

| Topic Name | Message Type | Direction | QoS | Description |
| :--- | :--- | :--- | :--- | :--- |
| `/cmd_vel_smoothed` | `geometry_msgs/msg/Twist` | Subscribe | Reliable | Smoothed chassis velocity commands from Nav2 `velocity_smoother`. |
| `/lift_cmd` | `std_msgs/msg/Int8` | Subscribe | Reliable | Discrete lift actuator control (`1`: Up, `-1`: Down, `0`: Stop). |
| `/wheel_ticks` | `std_msgs/msg/Int32MultiArray` | Publish | Reliable | Raw 4-wheel quadrature encoder tick counts for host odometry computation. |
| `/imu_data` | `sensor_msgs/msg/Imu` | Publish | Best Effort | 6-axis IMU readings streamed with minimal latency for EKF fusion. |
| `/lift_status` | `std_msgs/msg/Bool` | Publish | Reliable | Fork carriage limit-switch / target position feedback flag. |

---

[< Previous: 01. Hardware Setup](01_hardware_setup.md) | [Next: 03. Nav2 Navigation & Tuning >](03_navigation_nav2.md)