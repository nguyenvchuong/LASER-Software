# LASER-Software

Software package for LASER (Legged-Agile-Smart-Efficient-Robot) Series

Details in /laikago_ros, software guide and user manual provided by Unitree in /documentation

## Notes on pybullet interface
Tested on Ubuntu 18.04 (and 20.04). Built ROS melodic with python3 target. To run the MPC interface, source ROS in 3 terminals, and in order:

  * launch aliengo robot (no vis, see GUI flag in launch file, but gazebo will not be updated)  
  `roslaunch laikago_gazebo aliengo.launch`
  * Run the MPC service:  
  `rosrun laikago_gazebo pyb_mpc`  
  * Activate virtualenv and run (from usc_learning/ros_interface)  
  `python run_mpc.py`

Another option to visualize the communication from ROS to pybullet:  

  `rosrun laikago_gazebo laikago_pyb_send_state`  
  `python example_vis_ros_state.py`


## RL functionalities

Currently depends on [Tensorflow 1.15.0](https://storage.googleapis.com/tensorflow/libtensorflow/libtensorflow-cpu-linux-x86_64-1.15.0.tar.gz) for loading networks trained in Pybullet with rllib. 

To check running or jumping: 
* `roslaunch laikago_gazebo a1.launch`
* `rosrun laikago_gazebo run_rl`
* `rosrun laikago_gazebo jump_rl`

---
## Dependencies
  * for Ubuntu 16.04 ROS Kinetic + Gazebo 8
  * for Ubuntu 18.04 ROS Melodic + Gazebo 9 Eigen3 3.3.6 (other versions can work)
  * for Ubuntu 20.04 ROS Noetic + Gazebo 10

Make sure these packages have been installed, changing distro name depending on ROS version:
```
sudo apt-get install ros-kinetic-controller-manager ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-joint-state-controller ros-kinetic-effort-controllers ros-kinetic-velocity-controllers ros-kinetic-position-controllers ros-kinetic-robot-controllers ros-kinetic-robot-state-publisher ros-kinetic-gazebo8-ros ros-kinetic-gazebo8-ros-control
```
```
sudo apt-get install ros-melodic-controller-manager ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-joint-state-controller ros-melodic-effort-controllers ros-melodic-velocity-controllers ros-melodic-position-controllers ros-melodic-robot-controllers ros-melodic-robot-state-publisher ros-melodic-gazebo9-ros ros-melodic-gazebo9-ros-control

```


## Code structure
  FSM.h connects low level and high level controllers and run them for the robot.

  Servo.cpp is the executable file that contains FSM_State object

## Build Instructions
  * source your ROS
  * build laikago_msgs package first
  `catkin_make --pkg laikago_msgs`
  * compile the packge 
  `catkin_make -DCMAKE_BUILD_TYPE=Release`
  * launch aliengo robot (if it can't find the launch file, source ROS workspace again)
  `roslaunch laikago_gazebo aliengo.launch`
  * or launch A1 robot
  `roslaunch laikago_gazebo a1.launch`
  * run executable file in a new terminal for locomotion if needed
  `rosrun laikago_gazebo laikago_servo`

To switch between robots, change `setQuadruped()` function in servo.cpp or anywhere using Quadruped object. Then adjust z-height for QP and MPC in FSM.cpp and ConvexMPCLocomotion.cpp

## A1 Jumping with RL

* `roslaunch laikago_gazebo a1.launch`

Make the robot jump with:

* `rosrun laikago_gazebo jump_rl`

The A1 robot will jump in ROS simulation.

## Current Status
  * RL trained with MLP hist https://github.com/ChuongCS/Reinforcement-Learning-for-Quadruped-Robots-Jumps/tree/mlp_hist
  * In this v2 version: observation space include reference, reward consider penalty for shorten jump


## Possible Problem during installation
* if the simulation doesn't run well and the time factor is quite low, change the real_time_update_rate smaller based on your computer performance (e.g 100) in /laikago_ros/laikago_gazebo/launch/world/* .world file
