# Self-Balancing Robot using ESP32-WROOM-32 and MPU6050

This project implements a self-balancing two-wheeled robot using the ESP32-WROOM-32 development board and the MPU6050 accelerometer + gyroscope module. It uses a PID (Proportional-Integral-Derivative) control loop to estimate tilt and adjust motor speeds to maintain balance in real-time.

---

## Components Used

| Component              | Description                                                                 |
|------------------------|-----------------------------------------------------------------------------|
| ESP32-WROOM-32         | Microcontroller for sensor reading and motor control.                       |
| MPU6050                | 6-axis accelerometer and gyroscope used to detect the robot’s tilt angle.  |
| L298N Motor Driver     | Controls two DC motors based on PWM input and direction pins.               |
| DC Gear Motors (x2)    | Drives the robot forward or backward to maintain balance.                   |
| Wheels and Chassis     | Mechanical structure and mobility system.                                   |
| Battery Pack           | Powers the motors and ESP32.                                                |
| Jumper Wires           | For all electrical connections between modules.                             |

---

## How It Works

1. The MPU6050 provides accelerometer and gyroscope data.
2. A complementary filter fuses this data to estimate the current tilt angle.
3. A PID controller calculates how much correction is needed to reach the upright position.
4. Based on the correction, PWM signals are sent to the motor driver to adjust motor speed and direction.

---

## PID Control Logic

The PID algorithm continuously calculates an error value as the difference between a desired angle (`setPoint`) and the actual tilt:

- **Proportional (P):** Correction based on current error.
- **Integral (I):** Correction based on accumulated error over time.
- **Derivative (D):** Correction based on the rate of change of the error.

These are combined to form the output that controls motor speed and direction.

---

## Wiring Overview

| ESP32 Pin | Connected To (Motor Driver / Sensor) |
|-----------|---------------------------------------|
| D16       | IN1 (Motor A Input 1)                 |
| D17       | IN2 (Motor A Input 2)                 |
| D4        | ENA (Motor A Enable, for PWM)         |
| D18       | IN3 (Motor B Input 1)                 |
| D19       | IN4 (Motor B Input 2)                 |
| D5        | ENB (Motor B Enable, for PWM)         |
| D21       | SDA (MPU6050)                         |
| D22       | SCL (MPU6050)                         |
| 3.3V / GND| Power supply for MPU6050              |

> Make sure ENA and ENB pins on the motor driver are connected to **ESP32 pins D4 and D5**, respectively. These must receive PWM signals to control motor speed.

---

## Setup Instructions

1. Connect all components as per the wiring table.
2. Upload the Arduino code to the ESP32 using Arduino IDE or PlatformIO.
3. Open the Serial Monitor at 115200 baud to observe live sensor readings and PID output.
4. Place the robot upright and apply power. The robot should attempt to self-balance.

---

## Tuning & Notes

- The PID constants (`Kp`, `Ki`, `Kd`) can be adjusted to suit different builds or center of gravity.
- For better stability, keep the battery and heavy components low to reduce the center of mass.
- Use good quality wheels and ensure both motors rotate freely.
- Avoid placing the robot on uneven or slippery surfaces during testing.
