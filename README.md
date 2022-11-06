# Card Signing Robot Simulation
### 41013 Industrial Robotics 
### Assignment 2 

![alt text](https://github.com/GeorgeSheslow/Robotics_A2/blob/documentation/Simulation_Image.png)

## Video Links

Promo Video: https://youtu.be/hQq1Lgs0Sq0

Final Video: https://www.youtube.com/watch?v=Qhk6cRGMptY

## Software Requirements:

1. MATLAB 2020b or newer
2. Peter Corke's Robotics ToolBox

## Hardware Requirements (optional):

1. Microcontroller (Arduino or Teensy)
2. Push Button 
3. IR Distance Sensor


## Main Demo Instructions

1. Open mainDemo.m script and if Hardware connected, find serial port and add it to the HardwareSafety class constructor
2. Run mainDemo.m


### Simulation and Features

The aim of this project was to create and simulate a robotic system that was capable of signing cards and stacking them. The simualtion uses a ABB IRB120 robot for card pick placing and the dobot magician for writing a text on the card. 

- Robot jogging via joints or end-effector
- Robots use Resolved Motion Rate Control (RMRC) for task trajectory
- Safety: 2 Action emergency stop with GUI Estop and hardware EStop and collision
- Safety: Hardware IR sensor if distance below threshold will pause the simulation 
- Collision Checking with point cloud cube
- Visual Servoing demonstration with 2 robots

### Real Dobot Drawing

Script RealDobotDrawing.m can be used with a real Dobot Magician to draw text on a piece of paper.

- Requires ROS MATLAB Toolbox 
- Dobot Magician Driver: https://github.com/gapaul/dobot_magician_driver/
- Instructions for setting up hardware/ROS to interface with MATLAB: https://github.com/gapaul/dobot_magician_driver/wiki

