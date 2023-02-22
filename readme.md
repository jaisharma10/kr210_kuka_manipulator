

# ROS2 Kuka Kr 210 Manipulator


To build a ROS2 Manipulator simulation that uses forward and inverse kinematics to generate motion paths. 

A KUKA KR210 robot arm is used. It has 6 degrees of freedom. For this project, Position, Effort and Joint Trajectory controllers are taken from the ros_conotrol package.

IKPY python library is used to calculate forward and inverse kinematics solutions with the help of user input. The inverse_kinematics script can process  4 command line arguments as inputs. The arguments are the end effector cartesian position [x, y, z] and the gripper status [o, c] for open and closed. 

&nbsp;

<p align="center">
  <img src = "media/kr210_rviz.png" height = "320" >
  <!-- <img src = "media/kr210_gazebo_2.png" height = "320" > -->
  <img src = "media/kuka_kr210_img.jpg" height = "320" >
</p>

<p align="center">
  <video src= "media/gazebo_IK.mp4" width = "180" >

</p>

 &nbsp;

# System Requirements

- Ubuntu 20.04
- ROS2 - Foxy Fitzroy
- Gazebo
- Rviz

 &nbsp;


# Build and Compile instructions

```
    cd "your_colcon_workspace"/src
    git clone https://github.com/jaisharma10/kuka_kr210_simulation
    cd ..
    colcon build
    source "your_colcon_workspace"/install/setup.bash
```
 &nbsp;



# General Launch Files

Launch File to import robot into Gazebo

```
  ros2 launch kr210_kuka_manipulator gazebo.launch.py 
```
Launch File to import robot into Rviz
```
  ros2 launch kr210_kuka_manipulator rviz.launch.py 
```

 &nbsp;

# Running Task Scripts

Start the complete Launch File

```
  ros2 launch kr210_kuka_manipulator control_kr210.launch.py 
```

&nbsp;

## Inverse Kinematics
The script will compute the trajectory to go to a Goal End Effector position. It will then enable the robot to follow 
the generated trajectory. The command needs 4 command line arguments as inputs. The arguments are the end effector cartesian position [x, y, z] and the gripper position [o, c]. Run the command in a new terminal

&nbsp;

```
  ros2 run kr210_kuka_manipulator inverse_kinematics 1.5 2.0 1.0 o
```
&nbsp;

## Square Motion Planning
The command below uses an action-client to enable the manipulator to follow a Trajectory that represents a planar SQUARE in 3D-Space. Call the following run command 

&nbsp;

```
  ros2 run kr210_kuka_manipulator sqaure_actionClient
```

&nbsp;

# Video Output

https://user-images.githubusercontent.com/87094729/220734806-bde3d30b-5433-4811-b34f-0f7699ea7512.mp4



# License

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)


# Support

For any questions, email me at jaisharm@umd.edu


