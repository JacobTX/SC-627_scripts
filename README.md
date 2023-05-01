# SC-627_scripts

This contains scripts for different motion planninng and coordination tasks

**Assignment 1** - Motion planning using Bug algorithm

**Assignment 2** - Motion planning using Artificial potential field, Voronoi diagram and Trapezoidal cell decomposition

**Assignment 3** - Achieving synchronization or balanced consensus among multiple agents

The assignments has a **software simulation phase** in which the algorithm was implemented using **ROS** and tested in **Gazebo**, and a **hardware implementation phase** in which the algorithm would be tested in real-world scenarios with actual Turtlebots in the **ARMS** Lab, SysCon Department, IIT Bombay !

## Connecting laptop to Turtlebot hardware

Refer [link](https://github.com/JacobTX/SC-627_scripts/blob/main/Connect%20bot%20to%20roscore.pdf)

## Hardware challenges

1) LiPo batteries running out of charge, pain in the ass to remove the batteries and re-mount them after charging

2) Odom data getting screwed if the bot is moved/lifted manually from one place to another

3) LiDAR has been mounted in the opposite fashion, thus it takes some initial testing to determine which direction corresponds to the front. Also LiDAR data has 240 entries as opposed to 360 entries in simulation.
