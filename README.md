# UR10_Robotiq_Control

This repository aims to control the ur10 robot with the robotiq 2F140.


# Install 

In src folder : 

```
git clone https://github.com/ThomasDuvinage/ur10_robotiq_control.git

```

and then compile using `catkin_make`

Don't forget to source the workspace.

## Test the gripper 

Connected the robot to the computer. 

Run :

```
roslaunch robotiq_2f_gripper_control robotiq_2f_gripper_control.launch
```

You can edit the robot_ip and the port use for the gripper.

In another terminal run : 

```
rostopic pub /gripperCmd Int16 "data: 20"
```

The value can be set from 0 to 255. 