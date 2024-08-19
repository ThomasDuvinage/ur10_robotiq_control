# Robotiq gripper control 

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