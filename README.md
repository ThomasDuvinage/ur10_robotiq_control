# UR10_Robotiq_Control

This repository aims to control the ur10 robot with the robotiq 2F140.


# Install 

In src folder : 

```
git clone https://github.com/ThomasDuvinage/ur10_robotiq_control.git

```

and then compile using `catkin_make`

Don't forget to source the workspace.

## Move the robot using joint position

In simulation simulation : 

```
roslaunch ur10_gripper_control ur10_control_example.launch
roslaunch robotiq_2f_140_gripper_visualization robotiq_2f_140.launch 
```

Real robot : 

```
roslaunch ur_robot_driver ur10_bringup.launch robot_ip:=192.168.1.3 kinematics_config:=${HOME}/AIST_UR10_robot_calibration.yaml

```

## Test the gripper 

Connected the robot to the computer. 

Run :

```
roslaunch robotiq_2f_gripper_control robotiq_2f_gripper_control.launch
```

You can edit the robot_ip and the port use for the gripper.

In another terminal run : 

```
rostopic pub /gripperCmd std_msg/Int16 "data: 120"
```

The value can be set from 0 to 255. 

## Run rosservice example

In separated terminals : 

```
rosrun ur10_gripper_control ur10_control_service_example
```

```
rosrun ur10_gripper_control rosservice_client_example.py
or 
rosrun ur10_gripper_control ur10_control_service_client_example
```

Edit `scripts/rosservice_client_example.py` if using python or `src/rosservice_client_example.cpp` 

You need to call your pose generator within the node. 