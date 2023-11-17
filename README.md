# ABU Robocon 2023

## Overview

This repository contains code for various components of a robotics project designed for ABU Robocon 2023. The project involves the implementation of a PID controller in Arduino for velocity control, communication between Arduino Due and Mega, code for controlling a 4-wheel omni drive using a PS4 controller, interfacing an IMU (Inertial Measurement Unit), position estimation using rotary encoders and IMU, and code for controlling pneumatics and servo motors.

## Contents

1. **PID Controller for Velocity Control:**
   - The `PID_Controller_Arduino` directory contains code for implementing a Proportional-Integral-Derivative (PID) controller on Arduino for precise velocity control.

2. **Communication between Arduino Due and Mega:**
   - The `Arduino_Due_Mega_Communication` directory includes code for establishing communication between Arduino Due and Mega, facilitating data exchange between the two microcontrollers.

3. **4-Wheel Omni Drive Control using PS4:**
   - In the `Omni_Drive_PS4_Control` directory, you'll find code for controlling a 4-wheel omni drive using a PS4 controller, allowing for intuitive and dynamic robot movement.

4. **IMU Interfacing:**
   - The `IMU_Interfacing` directory provides code for interfacing with an IMU, extracting data for orientation, acceleration, and angular velocity.

5. **Position Estimation with Rotary Encoders and IMU:**
   - In the `Position_Estimation` directory, there's code for estimating the robot's position by combining data from rotary encoders and the IMU.

6. **Pneumatics and Servo Motor Control:**
   - The `Pneumatics_Servo_Control` directory contains code for controlling pneumatics and servo motors, enabling the manipulation of physical actuators.

## Usage

1. **Clone the Repository:**
   - Clone this repository to your local machine using the following command:
     ```
     git clone https://github.com/your-username/ABU_ROBOCON_2023.git
     ```

2. **Navigate to the Desired Directory:**
   - Choose the specific directory based on the component or functionality you are interested in.
