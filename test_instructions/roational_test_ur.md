# rotational slippage test

## Step 1:
Follow hold mode. 

## step 2: 

if object in then release

```
rosservice call /UR10_gripper "{time: 0.0, mode: 'release', lin_dist: 0.0, rotation: 0.0, recording_name: ''}" 
```

Place the object in gripper so that is can rotate 15 deg from vertical. 

```
rosservice call /UR10_gripper "{time: 0.0, mode: 'grasp', lin_dist: 0.0, rotation: 0.0, recording_name: ''}" 
```
For the 2 second
```
rosservice call /UR10_gripper "{time: 2.0, mode: 'rotational_slippage', lin_dist: 0.00, rotation: 45.0, recording_name: 'cardboard_rotation_t2_15_1'}" 
```
or 5 second
```
rosservice call /UR10_gripper "{time: 5.0, mode: 'rotational_slippage', lin_dist: 0.00, rotation: 45.0, recording_name: 'cardboard_rotation_t5_15_1'}" 
```

Measure final angle with phone

repeat 10x 

Then setup 60 degrees form vertical. 

For the 2 second
```
rosservice call /UR10_gripper "{time: 2.0, mode: 'rotational_slippage', lin_dist: 0.00, rotation: 60.0, recording_name: 'cardboard_rotation_t2_60_1'}" 
```
or 5 second
```
rosservice call /UR10_gripper "{time: 5.0, mode: 'rotational_slippage', lin_dist: 0.00, rotation: 60.0, recording_name: 'cardboard_rotation_t5_60_1'}" 
```
repeat 10x 
