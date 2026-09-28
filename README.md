# Self-Balancing-Robot

A two-wheeled self-balancing robot designed and developed using an ESP32, MPU6050 IMU sensor, PID control algorithm, and stepper motors.

The system continuously measures the robot's tilt angle and uses a closed-loop PID controller to adjust the motor movement and maintain the robot in an upright position.

Overview

The main objective of this project is to design and implement a two-wheeled robot capable of maintaining its balance autonomously.

The MPU6050 is used to measure the robot's orientation. The measured angle is fed back to the PID controller, which calculates the required motor control output to compensate for the robot's tilt.

System Architecture

              MPU6050
                 │
                 │ I2C
                 ▼
        ┌─────────────────┐
        │  Microcontroller │
        │      ESP32       │
        └────────┬────────┘
                 │
                 │ Tilt Angle
                 ▼
          ┌───────────────┐
          │ PID Controller│
          └───────┬───────┘
                  │
            Motor Control
                  │
          ┌───────┴───────┐
          ▼               ▼
      A4988 Driver     A4988 Driver
          │               │
          ▼               ▼
      Stepper Motor   Stepper Motor
          │               │
          └───────┬───────┘
                  ▼
           Robot Movement
                  │
                  └──────────────► MPU6050
                                   Feedback

Hardware

ESP32 microcontroller

MPU6050 6-axis IMU sensor

2 × A4988 stepper motor drivers

2 × NEMA17 stepper motors

Two-wheel mechanical frame

Battery / power supply

Software & Technologies

C/C++

ESP32

Arduino IDE

PID Control

I2C Communication

Timer / Interrupt-based Motor Control

Stepper Motor Control

Control Algorithm

The robot uses a closed-loop PID controller to maintain its balance.

The MPU6050 provides the robot's current tilt angle. The controller calculates the error between the target angle and the measured angle:

Error = Target Angle - Measured Angle

The PID controller calculates the control output:

Output = Kp × Error
       + Ki × Integral(Error)
       + Kd × Derivative(Error)

The resulting output is used to determine the direction and stepping rate of the motors.

Feedback Control

The robot operates as a closed-loop control system:

       Target Angle
            │
            ▼
       ┌─────────┐
       │   PID   │
       └────┬────┘
            │
            ▼
       Motor Control
            │
            ▼
       Stepper Motors
            │
            ▼
          Robot
            │
            ▼
         MPU6050
            │
            │ Feedback
            └──────────────► PID

The feedback loop allows the robot to continuously detect changes in its orientation and generate corrective motor movement.

Project Features

Real-time tilt-angle measurement

MPU6050 sensor integration

Closed-loop PID control

Automatic self-balancing

Dual stepper motor control

A4988 motor driver control

I2C communication

Real-time feedback control

Adjustable PID parameters

Project Structure

Self-Balancing-Robot/
│
├── Car_Blance.ino
└── README.md

Getting Started

1. Hardware Setup

Connect the MPU6050 to the ESP32 through the I2C interface.

Connect the two A4988 motor drivers to the corresponding STEP, DIR, and ENABLE control pins.

Connect the stepper motors to the A4988 drivers and provide an appropriate power supply.

2. Upload the Firmware

Open:

Car_Blance.ino

using the Arduino IDE.

Select the correct ESP32 board and upload the firmware.

3. PID Tuning

The PID parameters need to be tuned according to the mechanical structure, motor characteristics, sensor configuration, and center of gravity of the robot.

The main parameters are:

Kp → Proportional gain
Ki → Integral gain
Kd → Derivative gain

Current Status

Feature

Status

ESP32 integration

✅ Implemented

MPU6050 integration

✅ Implemented

I2C communication

✅ Implemented

PID control

✅ Implemented

Stepper motor control

✅ Implemented

Closed-loop feedback

✅ Implemented

Self-balancing

🔧 Under development / Testing

Bluetooth control

⏳ Planned

Android application

⏳ Planned

Future Improvements

Further PID tuning for improved balancing stability

Improve sensor filtering and angle estimation

Implement encoder feedback

Add complementary or Kalman filtering

Implement Bluetooth remote control

Develop an Android control application

Add velocity and position control

Add battery voltage monitoring

Improve motor control performance

Optimize the real-time control loop

Author

Name: Hoang Tuan Kiet
University: Vietnam - Korea University of Information and Communication Technology
Major: Embedded Systems Engineering

This project was developed to explore embedded systems, robotics, feedback control, PID algorithms, IMU sensors, and stepper motor control.
