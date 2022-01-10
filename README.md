# Crazyflie-RL-SAM-Work-Repo
Repository for holding code for crazyflie RL project

### crazyflie_mpc_mod:
This folder includes the ROS package that includes the modified crazyflie trajectory tracking python code, in which the service for take of and landing are included

### crazyflie_optitrack_integration
This folder includes the ROS package that includes all files needed to control crazyflie with optitrack, which can be launched with hover_optitrack.launch

### flow_rate_measuring
This folder includes the ROS package and arduino code for measuring flow rate with pitot tube, the instructions are listed below
*compile and upload arduino code to the arduino and connect pitot tube to corresponding pins (Analog pin to A0 port)
*launch roscore in a new command window and run the following command: rosrun rosserial_python serial_node.py <serial port used by the arduino>
*launch the only launch file in the launch folder for starting data recording
*run the following command to start each file recording: rosservice call /record <double tab to complete the message>
