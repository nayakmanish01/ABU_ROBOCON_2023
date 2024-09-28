# ABU_ROBOCON_2023

This repository contains the source files and related components for the **ABU ROBOCON 2023** project by Sardar Vallabhbhai National Institute of Technology, Surat. The competition's theme is *"Casting Flowers Over Angkor Wat"*, and it involves designing and building two robots—the **Elephant Robot (ER)** and **Rabbit Robot (RR)**—to work together to achieve the game objectives.

## Project Overview

The competition involves two primary robots:
1. **Elephant Robot (ER):** Equipped with a holonomic drive, this robot performs tasks like picking, loading, and tossing rings.
2. **Rabbit Robot (RR):** Features a mecanum drive system designed for agility and performing similar tasks to the ER.

Both robots are semi-autonomous and designed according to the rulebook.

---

## Repository Structure

The repository is divided into several subfolders corresponding to different parts of the project. Below is an overview of the folder structure and the main components inside each:

### 1. **Actuators**
Contains all actuator-related implementations used for robot control.

- **`Stepper_motor_NEMA_23/`**: Stepper motor control for precise rotation.
- **`Sweep_servo/`**: Sweep servo control for mechanisms requiring angular motion.
- **`linear_actuator_dual_pos_control_esp32_blynk/`**: ESP32 and Blynk-based control for dual-position linear actuators.
- **`pneumatic3by2_PS4/`**: Control for pneumatic systems using a 3/2 valve, configured through a PS4 controller.

### 2. **Estimation**
Houses sensor data processing components for position, orientation, and speed estimation.

- **`IMU/`**: General inertial measurement unit (IMU) code.
- **`IMU_10DOF_Test/`**: Testing code for a 10 Degrees of Freedom IMU.
- **`IMU_6050/`**: Implementation of the IMU6050 sensor for orientation tracking.
- **`RPMEncoder/`**: Code to estimate rotations per minute (RPM) from encoder data.
- **`encoder_data/`**: Contains raw and processed encoder data.
- **`14_kalmanFilter.pdf`**: Documentation for Kalman filter implementation used in sensor fusion.

### 3. **PID**
Contains different versions of PID control algorithms for motor and robot control.

- **`PID_Version1/`**: Initial version of the PID controller.
- **`PID_Version2/`**: Second version with enhancements for precision.
- **`PID_Version3/`**: Final version with optimized tuning parameters.
- **`PID_control_one_motor/`**: Implementation of PID control for a single motor.

### 4. **ROBOCON_Stage2**
Project files related to the second stage of the ROBOCON competition.

- **`Elephant_bot_PID/`**: PID control specific to the Elephant Robot.
- **`Elephant_bot_without_pid_working/`**: Code for Elephant Robot without PID control.
- **`transmitter_esp_with_buttons_v1/`**: First version of ESP-based transmitter with button control.
- **`transmitter_esp/`**: Finalized version of ESP-based transmitter.

### 5. **TR_LOWER_CONTROL**
Contains files related to the lower control mechanisms for the TryRobot (TR).

- **`LowerControl-TR/`**: Codebase for the lower control systems of the robot.

### 6. **TR_UPPER_CONTROL**
Includes the upper control mechanisms for the TryRobot (TR).

- **`TryRobot-master/UpperControl-TR/`**: Codebase for the upper control systems of the robot.

### 7. **four_wheel_RC_without_PID**
Contains the code for a four-wheel RC-controlled robot without PID implementation.

---
