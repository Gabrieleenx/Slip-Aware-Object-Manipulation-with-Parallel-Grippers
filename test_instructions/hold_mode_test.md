# Hold controller test 

## Step 1 follow friction estimation. 
This assumes gripper is open and is more goint to a location then picking up an object

```
rosservice call /UR10_gripper "{time: 0.0, mode: 'pick_up_object', lin_dist: 0.0, rotation: 0.0, recording_name: ''}" 
```

pause the robot controller, alternativly turn of the robot, no movement nedded for a while. Recomended to do friction est, (hold, linear and rotation (robot off)) and hinge test for all for each object before moving to the next object as we dont need to redo the friction estimation. 

Put object in gripper and call:
```
rosservice call /UR10_gripper "{time: 0.0, mode: 'grasp', lin_dist: 0.0, rotation: 0.0, recording_name: ''}" 
```
Then call 

```
rosservice call /UR10_gripper "{time: 60.0, mode: 'hold_mode', lin_dist: 0.0, rotation: 0.0, recording_name: plastic_hold_mode_1'}" 
```
touch the object and try to move it out of the gripper.

try gently pulling it, gently rotationg it, lightly tapping it, then tapping it progressvly harder.

