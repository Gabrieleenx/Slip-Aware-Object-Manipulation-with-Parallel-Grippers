# Hinge mode test

## Step 1: 

Make sure the friction estimation has been done, 

With the robot unpaused and on. 

Without an object in the gripper

Go to hinge pose
Unpause the robot
```
rosservice call /UR10_gripper "{time: 0.0, mode: 'go_to_hinge_pose', lin_dist: 0.0, rotation: 0.0, recording_name: ''}" 
```

Pause robot 

Grasp object, make mark of start pos
```
rosservice call /UR10_gripper "{time: 0.0, mode: 'grasp', lin_dist: 0.0, rotation: 0.0, recording_name: ''}" 
```

Unpause the robot


```
rosservice call /UR10_gripper "{time: 0.0, mode: 'hinge_mode', lin_dist: 0.0, rotation: 0.0, recording_name: 'plastic_hinge_mode_1'}" 
```


pause robot 

measure 
