# Estimation of friction with gripper mounted on UR10

## Step 1:
Make sure the object height and parameters are set up correctly

Power on the UR10 robot

program the robot,

create a or load the correct initilaization (external control)
```
Host ip 192.168.56.1
```
```
robot ip 192.168.56.3
```

## Step 2: 
Start robot driver
```
roslaunch ur_robot_driver ur10_bringup.launch robot_ip:=192.168.56.3
```
and visualization
```
roslaunch ur_robot_driver example_rviz.launch
```

press play on UR10 after the two above, to start the external control 

Make sure the robot cant hit anything and you are at safe distance with emergency button ready. 

Start 
```
rosrun gripper UR10_gripper.py
```

move joint angles to reset pose

name: 
  - elbow_joint = -2.371053997670309
  - shoulder_lift_joint = -1.67930776277651
  - shoulder_pan_joint =  -1.4216578642474573
  - wrist_1_joint = 0.9087690114974976
  - wrist_2_joint = 1.421658992767334
  - wrist_3_joint = -1.5707958380328577
 
If robot is far from these the manually move close to these before calling reset joints. 

```
rosservice call /UR10_gripper "{time: 0.0, mode: 'reset_joints', lin_dist: 0.0, rotation: 0.0, recording_name: ''}"
```

Pause the program on the UR10

Now mount the gripper and start it 

```
roslaunch gripper gripper.launch
```

Zero out the force readings

```
rosservice call /gripper_callibrate_ft
```

## Step 3:

Unpause the robot

Before we can go to the object loationm 
```
rosservice call /UR10_gripper "{time: 0.0, mode: 'release', lin_dist: 0.0, rotation: 0.0, recording_name: ''}"
```
Then got to object
```
rosservice call /UR10_gripper "{time: 0.0, mode: 'go_to_object', lin_dist: 0.0, rotation: 0.0, recording_name: ''}"
```

pause the robot and place object then unpause the robot

## Step 4:
Call the estimation with the correct naming 
```
rosservice call /UR10_gripper "{time: 0.0, mode: 'est_contact', lin_dist: 0.0, rotation: 0.0, recording_name: 'est_contact_wood_1'}"
```
Save the print out estimated friction. If needed rename the file generated in frictio_test folder. 

repeat step 4 10x
