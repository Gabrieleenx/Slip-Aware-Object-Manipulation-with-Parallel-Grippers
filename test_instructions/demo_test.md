# Full demo

## make sure robt is reset, and ft is callibrated,

then call 

```
rosservice call /UR10_gripper "{time: 3.0, mode: 'demo', lin_dist: 0.03, rotation: 30.0, recording_name: 'demo_plastic_1'}" 

```
