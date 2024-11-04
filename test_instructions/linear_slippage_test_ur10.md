# Linear slippage test 

## Step 1: 
Follow Hold test, assume robot off. 

## step 2: 

if object in then release

```
rosservice call /UR10_gripper "{time: 0.0, mode: 'release', lin_dist: 0.0, rotation: 0.0, recording_name: ''}" 
```

Place the object in gripper at resonable height, should be able to slide 3 cm. Make mark with pen of start position and dont forget to meassure .

```
rosservice call /UR10_gripper "{time: 0.0, mode: 'grasp', lin_dist: 0.0, rotation: 0.0, recording_name: ''}" 
```
For the 2 second
```
rosservice call /UR10_gripper "{time: 2.0, mode: 'linear_slippage', lin_dist: 0.02, rotation: 0.0, recording_name: 'cardboard_linear_t2_2cm_1'}" 
```
or 5 second
```
rosservice call /UR10_gripper "{time: 5.0, mode: 'linear_slippage', lin_dist: 0.02, rotation: 0.0, recording_name: 'cardboard_linear_t5_2cm_1'}" 
```
Measure!!!!!!!
repeat 10x 

Then 4 cm slippage

For the 2 second
```
rosservice call /UR10_gripper "{time: 2.0, mode: 'linear_slippage', lin_dist: 0.04, rotation: 0.0, recording_name: 'cardboard_linear_t2_4cm_1'}" 
```
or 5 second
```
rosservice call /UR10_gripper "{time: 5.0, mode: 'linear_slippage', lin_dist: 0.04, rotation: 0.0, recording_name: 'cardboard_linear_t5_4cm_1'}" 
```

repeat 10x 
