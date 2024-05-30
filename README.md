# Autonomous Navigation for TurtleBot3 Using Custom PID Controller

## Project Overview
This project focuses on developing an autonomous navigation system for the TurtleBot3 robot. The system allows the robot to reach a user-specified location with precise yaw orientation using a custom-tuned PID controller. An innovative exponential decay factor was implemented to improve the robot's efficiency in managing large angular errors while maintaining speed, which significantly enhances path correction and overall navigation stability. The project demonstrates proficiency in Python, ROS (Robot Operating System), and real-time control systems.

## Features
- __Custom PID Controller:__ Designed and implemented a custom PID controller for linear and angular motion control.
- __Exponential Decay Factor:__ Introduced an innovative exponential decay factor to manage large angular errors efficiently while maintaining speed.
- __Real-Time Control:__ Utilized Python and ROS (Robot Operating System) for real-time control and responsiveness.
- __Robust Error Handling:__ Fine-tuned PID parameters to handle various scenarios and disturbances effectively.

## Technical Details
- __Languages and Tools:__ Python, ROS (Robot Operating System)
- __Maximum Linear Velocity:__ 0.22 m/s
- __Maximum Angular Velocity:__ 2.84 rad/s
- __PID Parameters:__
  - Linear: Kp = 1.0, Ki = 0.0, Kd = 0.1
  - Angular (Step 1): Kp1 = 0.3, Ki1 = 0.0, Kd1 = 0.1
  - Angular (Step 2): Kp2 = 1.0, Ki2 = 0.0, Kd2 = 0.1
  - Decay Factor: decay_k = 2
