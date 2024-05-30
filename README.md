# Autonomous Navigation for TurtleBot3 Using Custom PID Controller

## Project Overview
This project focuses on developing an autonomous navigation system for the TurtleBot3 robot. The system allows the robot to reach a user-specified location with precise yaw orientation using a custom-tuned PID controller. An innovative exponential decay factor was implemented to improve the robot's efficiency in managing large angular errors while maintaining speed, which significantly enhances path correction and overall navigation stability. The project demonstrates proficiency in Python, ROS (Robot Operating System), and real-time control systems.

## Features
- __Custom PID Controller:__ Designed and implemented a custom PID controller for linear and angular motion control.
- __Exponential Decay Factor:__ Introduced an innovative exponential decay factor to manage large angular errors efficiently while maintaining speed.
- __Real-Time Control:__ Utilized Python and ROS (Robot Operating System) for real-time control and responsiveness.
- __Robust Error Handling:__ Fine-tuned PID parameters to handle various scenarios and disturbances effectively.

## Technical Details
- __Languages and Tools:__ Python, ROS (Robot Operating System), Gazebo
- __Maximum Linear Velocity:__ 0.22 m/s
- __Maximum Angular Velocity:__ 2.84 rad/s
- __PID Parameters:__
  - Linear: __Kp = 1.0__, __Ki = 0.0__, __Kd = 0.1__
  - Angular (Step 1): __Kp = 0.3__, __Ki = 0.0__, __Kd = 0.1__
  - Angular (Step 2): __Kp = 1.0__, __Ki = 0.0__, __Kd = 0.1__
  - Decay Factor: __decay_k = 2__

## Visualization
__Location:__ x = __0.0__, y = __0.5__, yaw = __0.0__\
\
*i) Without the exponential decay factor (cannot reach the specified location)*

https://github.com/Shubh-57/Autonomous-Navigation-for-TurtleBot3/assets/138125561/c61bb660-a9e8-49cc-9a13-c0327365ccc0

*ii) With the exponential decay factor (can reach the specified location)*

https://github.com/Shubh-57/Autonomous-Navigation-for-TurtleBot3/assets/138125561/f362158e-1d0b-42a8-b09f-6249724fe540
