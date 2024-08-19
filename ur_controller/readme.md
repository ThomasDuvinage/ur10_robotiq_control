# UR ROS Controller

This folder contains all the necessary tools (ros packages) to control ur robots.


## How is UR10 robot controlled 

TODO Explain here how the action lib server work and what is sended to the robot in order for it to move


## Run Demo

This demo will move the robot to given desired joint pose.

```
roslaunch ur_controller DemoTrajectoryController.launch
```

## Run Robot Controller

In order to control the robot from different ros packages you can use rosservice `MoveRobot`. 

First you need to launch the controller : 
```
 roslaunch ur_controller TrajectoryController.launch
```

Then using the `MoveRobot` service embedded inside `ur_controller` you can ask to move the robot following cartesian, joint or velocity command. 

Please refer to [ur_controller/src/rosservice_client_example.cpp](ur_controller/src/rosservice_client_example.cpp) as an example. 

You can try it using the following command. Note that before running the following command you must update the pose coordinates. 

```
rosrun ur_controller ur10_control_service_client_example
```

a python version of this example is avaible in the scripts folder.

## Run for LLM Experiment

* First run the bringup of the robot

```
roslaunch ur_robot_driver ur10_bringup.launch robot_ip:=192.168.1.3 kinematics_config:=${HOME}/ur10.yaml
```

* Run the LMM Controller

```
roslaunch ur_controller LLM_experiment.launch
```