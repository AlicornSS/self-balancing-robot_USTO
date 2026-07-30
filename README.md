# Two-Wheeled PID Self-Balancing Robot

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Microcontroller](https://img.shields.io/badge/MCU-ATmega328P%2FArduino-orange.svg)](https://www.microchip.com/)
[![Control](https://img.shields.io/badge/Control-Discrete%20PID-green.svg)]()
[![Sensors](https://img.shields.io/badge/IMU-MPU6050%20(6--DOF)-red.svg)](https://www.invensense.com/)

An inverted pendulum mobile robot that dynamically stabilizes itself in the upright position using closed-loop PID control, sensor fusion via a Complementary Filter, and motor deadzone compensation.

---

## 📌 Demo & Preview

### Physical Assembly

![Self-Balancing Robot Prototype](path_to_your_image.jpg)

> 📹 **Current Project Status:** Hardware assembly and sensor fusion routines are fully completed! The robot is currently undergoing fine-tuning of its PID parameters ($K_p, K_i, K_d$) to achieve optimal dynamic response. A video demonstration showcasing real-time balance recovery from external push disturbances will be uploaded once tuning is finalized.

```
+-------------------------------------------------------+
|                                                       |
|                   [ MPU6050 IMU ]                     |
|                          |                            |
|                    I2C (400kHz)                       |
|                          v                            |
|                 [ ATmega328P MCU ]                    |
|                          |                            |
|                 PWM + Direction Control               |
|                          v                            |
|                  [ L298N Driver ]                     |
|                     /         \                       |
|                    v           v                      |
|              [Motor Left]  [Motor Right]              |
|                                                       |
+-------------------------------------------------------+

```
## 📋 Table of Contents

- [System Overview & Architecture](#-system-overview--architecture)
- [Hardware Specifications & Wiring Pinout](#%EF%B8%8F-hardware-specifications--wiring-pinout)
- [Control Theory & Mathematics](#-control-theory--mathematics)
  - [1. Sensor Fusion (Complementary Filter)](#1-sensor-fusion-complementary-filter)
  - [2. Discrete PID Algorithm](#2-discrete-pid-algorithm)
  - [3. Deadzone Compensation](#3-deadzone-compensation)
- [Firmware Implementation](#-firmware-implementation)
- [PID Tuning Protocol](#-pid-tuning-protocol)
- [Installation & Getting Started](#-installation--getting-started)
- [Project Roadmap & Updates](#-project-roadmap--updates)
- [License](#-license)

---

## 🔬 System Overview & Architecture

The robot models a classic **inverted pendulum on a two-wheeled cart**, an underactuated, highly non-linear, and inherently unstable dynamic system. Without active control, the system's center of mass resides above its axis of rotation, causing gravity to generate an destabilizing torque that scales with tilt angle $\theta$:

$$\tau_{\text{gravity}} = m \cdot g \cdot L \cdot \sin(\theta)$$

To prevent the chassis from collapsing, the system implements high-frequency closed-loop feedback. The sensor suite measures inclination, predicts rotational velocity, and drives dual high-torque DC gear motors toward the direction of the fall to generate counteracting horizontal acceleration.

### High-Level System Mechanics

1. **State Acquisition (100 Hz Loop):** An MPU6050 6-DOF Inertial Measurement Unit (IMU) continuously samples 3-axis acceleration and angular rate data at a $10\text{ ms}$ update interval ($f_s = 100\text{ Hz}$).
2. **Signal Conditioning & Fusion:** Accelerometer noise (caused by motor vibration) and gyroscope drift are removed in real time using a Complementary Filter to compute an absolute, lag-free pitch angle $\theta$.
3. **Control Output Computation:** A discrete Proportional-Integral-Derivative (PID) algorithm evaluates the error $e(t) = \theta_{\text{target}} - \theta_{\text{current}}$, generating an actuation value $u(t)$ proportional to restoring force.
4. **Non-Linear Actuation:** The computed $u(t)$ is passed through a deadzone compensation module to bypass motor static friction ($V_{\text{threshold}}$) before driving the L298N H-Bridge PWM outputs.

```
                                  +-------------------+
                                  |   Target Pitch    |
                                  |   (Setpoint = 0°) |
                                  +---------+---------+
                                            |
                                            v  (+)
  +------------------+   Angle Error   +----+----+   Control Out u(t)   +---------------+
  |  MPU6050 IMU     |--------------->|    Σ    |---------------------->|  L298N Driver |
  | (Accel + Gyro)   |     (-)        +---------+                       +-------+-------+
  +--------+---------+                                                          |
           ^                                                                    v
           |                        Physical Feedback                   +---------------+
           +------------------------------------------------------------|   DC Motors   |
                                                                        +---------------+
```

## 🛠️ Hardware Specifications & Wiring Pinout

### Bill of Materials (BOM)

| Component | Description | Quantity | Interface |
| :--- | :--- | :---: | :--- |
| **Microcontroller** | ATmega328P (Arduino Uno / Nano) | 1 | $I^2C$, PWM, GPIO |
| **IMU Module** | MPU6050 (3-Axis Accelerometer + 3-Axis Gyro) | 1 | $I^2C$ (Address `0x68`) |
| **Motor Driver** | L298N Dual H-Bridge Module | 1 | 2x PWM (`ENA`, `ENB`), 4x GPIO (`IN1`..`IN4`) |
| **Actuators** | 12V DC Geared Motors with high-torque gearboxes | 2 | Differential Drive |
| **Power Source** | 2S/3S LiPo Battery or 18650 Li-ion cells | 1 | 7.4V – 11.1V Nominal |
| **Power Regulation** | LM2596 Buck Converter (if external 5V needed) | 1 | Step-Down Regulated |

### Pin Map

| MPU6050 | Arduino | Function |
| :--- | :--- | :--- |
| `VCC` | `5V` | System Logic Power |
| `GND` | `GND` | Common Ground |
| `SDA` | `A4` | $I^2C$ Data |
| `SCL` | `A5` | $I^2C$ Clock |
| `INT` | `D2` | Data Ready Interrupt |

| L298N Driver | Arduino | Function |
| :--- | :--- | :--- |
| `ENA` | `D5` (PWM) | Left Motor Speed |
| `IN1` | `D6` | Left Motor Direction A |
| `IN2` | `D7` | Left Motor Direction B |
| `IN3` | `D8` | Right Motor Direction A |
| `IN4` | `D9` | Right Motor Direction B |
| `ENB` | `D10` (PWM) | Right Motor Speed |

---

## 📐 Control Theory & Mathematics

### 1. Sensor Fusion (Complementary Filter)
Raw accelerometer data suffers from high-frequency vibrations and motor noise, while gyroscope readings suffer from low-frequency integration drift. A **Complementary Filter** combines both into a clean, low-latency pitch angle $\theta$:

$$\theta_{k} = \alpha \cdot (\theta_{k-1} + G_x \cdot \Delta t) + (1 - \alpha) \cdot \theta_{acc}$$

Where:
- $\alpha = 0.96$ (Weight given to integrated high-pass gyroscope data)
- $(1 - \alpha) = 0.04$ (Weight given to low-pass accelerometer data)
- $\theta_{acc} = \arctan2(A_y, A_z) \cdot \frac{180}{\pi}$

### 2. Discrete PID Algorithm
The error input to the system is $e(t) = \theta_{\text{target}} - \theta(t)$. In discrete form, the control command $u[k]$ is calculated as:

$$u[k] = K_p \cdot e[k] + K_i \cdot \sum_{i=0}^{k} e[i] \cdot \Delta t + K_d \cdot \frac{e[k] - e[k-1]}{\Delta t}$$

- **$K_p$ (Proportional):** Restores upright posture relative to magnitude of angle error.
- **$K_i$ (Integral):** Accumulates residual steady-state error (e.g., asymmetric weight distribution).
- **$K_d$ (Derivative):** Acts as a virtual damper resisting sudden velocity variations.

### 3. Deadzone Compensation
DC motors feature a physical threshold below which static friction prevents motion ($PWM \in [0, 40]$). To eliminate deadband lag near zero speed, an offset bias $PWM_{\text{min}}$ is applied dynamically:

$$PWM_{\text{applied}} = \begin{cases} u[k] + PWM_{\text{min}} & \text{if } u[k] > 0 \\ u[k] - PWM_{\text{min}} & \text{if } u[k] < 0 \\ 0 & \text{if } u[k] = 0 \end{cases}$$

---

## 💻 Firmware Implementation

```cpp
#include <Wire.h>

// PID Gain Parameters
float Kp = 28.0;
float Ki = 1.5;
float Kd = 0.8;

// System Constants
const float SAMPLE_TIME = 0.01; // 100 Hz Loop
const int MOTOR_DEADZONE = 35;

float targetAngle = 0.0;
float currentAngle = 0.0;
float errorSum = 0.0;
float lastError = 0.0;

unsigned long lastTime = 0;

void setup() {
    Wire.begin();
    Wire.setClock(400000); // Fast I2C mode
    initMPU6050();
    initMotors();
}

void loop() {
    unsigned long now = millis();
    if ((now - lastTime) >= (SAMPLE_TIME * 1000)) {
        lastTime = now;

        // 1. Read Sensor & Apply Filter
        currentAngle = getComplementaryAngle();

        // 2. Compute PID
        float error = targetAngle - currentAngle;
        errorSum += error * SAMPLE_TIME;
        
        // Anti-windup clamping
        errorSum = constrain(errorSum, -100, 100);

        float errorDerivative = (error - lastError) / SAMPLE_TIME;
        float output = (Kp * error) + (Ki * errorSum) + (Kd * errorDerivative);
        lastError = error;

        // 3. Drive Motors with Deadzone Correction
        driveMotors(output);
    }
}
