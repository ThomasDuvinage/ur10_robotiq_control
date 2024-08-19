# UR Tutorial

This tutorial explains how to use UR robot from scratch.


This tutorial has been written using Ubuntu 20.04 and ROS Noetic.

## Requirements 

Before going any deeper in the following documentation, please make sure the following packages are installed : 


* [ROS](http://wiki.ros.org/noetic/Installation/Ubuntu)


* [Universal robot driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver#universal_robots_ros_driver)

    - From source [prefered]
        ```
        [Optinal] Creation of a ur workspace
        mkdir -p ur_ws/src && cd ur_ws


        git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver
        git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git src/universal_robot


        sudo apt update -qq
        rosdep update
        rosdep install --from-paths src --ignore-src -y

        catkin_make

        source devel/setup.bash
        ```

    - Using ros package

        ```
        sudo apt install ros-${ROS_DISTRO}-ur-robot-driver 
        ```


## Initial robot setup

To use ur_robot_driver with a real robot you need to make sure that **externalcontrol-x.x.x.urcap** is installed. [install it](https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases)

When it's done, follow the necessary [instructions](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_cb3.md){:target="_blank"} to install it on the robot using the user interface.  

TODO Explain here how to setup the network parameter in ubuntu. Relating to the tutorial mentionned above. 

Once this step is done, you can move on to the calibration step : 

> Each UR robot is calibrated inside the factory giving exact forward and inverse kinematics. To also make use of this in ROS, you first have to extract the calibration information from the robot.

```roslaunch ur_calibration calibration_correction.launch \
  robot_ip:=<robot_ip> target_filename:="${HOME}/my_robot_calibration.yaml"
```

## Control your robot

* [UR ROS Controller](ur_controller/readme.md)
* [Gripper control](robotiq_2f_gripper_control/readme.md)
* [UR10 Moveit Visualisation](robotiq_2f_140_gripper_visualization/README.md)

To install all :

  ```
  git clone https://github.com/ThomasDuvinage/ur10_robotiq_control.git
  ```

and then compile using `catkin_make`

Don't forget to source the workspace.