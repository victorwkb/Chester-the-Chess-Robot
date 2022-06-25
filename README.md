# Chester-the-Chess-Robot
This repository contains the source code for the main project of MCEN90028 Robotics System at the University of Melbourne.

## Overview
The main purpose of the project is to design "Chester", a serial manipulator, that is able to pick up and place chess pieces on the designated location of the chess board. The project was started from scratch with a provided tool kit consisting of 4 SCS15 servo motors, 2 SCS009 servo motors alongside an Arduino Mega board. This project was structured in 5 main sections with different sub-goals as listed below:  

Assignment 1: Derive the Forward Kinematics and Inverse Kinematics of the robotic arm to determine a suitable link length  
Assignment 2: Derive the Jacobian matrix to map joint velocities to end-effector velocity and validate that the maximum torque when lifting the load does not exceed the motor's torque limit  
Assignment 3: Construct a trajectory generation algorithm in task space and translate it into a set of joint space trajectories  
Assignment 4: Design CAD model prior to manufacturing the chess robot which would implement all previous tasks to perform specific chess moving tasks. Testing and validate performance throughout multiple iterations of improvement prior to demonstration to the tutors(judges)  
Assignment 5: Present the overall outcome by writing a Final Report emphasising on the process of the entire project and validation for the accuracy of the outcomes  

Tools used in this project are mainly MATLAB for coding (located in the src folder), Fusion360 for CAD modelling and laser cutting / manufacturing, and LaTeX for writing reports.
