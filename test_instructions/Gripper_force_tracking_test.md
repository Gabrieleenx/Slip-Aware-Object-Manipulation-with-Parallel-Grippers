
# Test procedure for gripper force tracking tuning and testing. 

## Step 1: 
Take of velocity sensors. The experiments have high frequency occilations, could potentially damage sensors. Force sensors are neccessary for the experiment. 

## Step 2:

Make sure the closed loop contronller only uses sensor one as feedback. (this will be changed back to the average of the two after this test), this is because the data analysis is simpler and it dosent affect the results. 

## Step 3: 

Plug in the gripper and FT sensors. (make sure the fingers are not at end stops) 

## Step 4:

Setup the devel
```
. ~/Documents/Catkin_workspace/catkin_yumi/devel/setup.bash
```

Launch the gripper:
```
roslaunch gripper gripper.launch
```

## Step 5 

```
rosservice call /gripper_callibrate_ft 
```

To open gripper:
```
rostopic pub /gripper_closed_loop gripper/Gripper_target "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
target: -5.0" 

```
To close gripper
```
rostopic pub /gripper_closed_loop gripper/Gripper_target "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
target: 5.0" 

```

## Step 6: experiments or tuning 

1: open gripper
2: object in gripper
3: close gripper

4: change recording name in gripper_control_test.py, i.e. ["Sponge_x", "Case_x", "Wood_x", "Cardboard_x", "Plastic_x"], where x is the number of the experiment, 10 per object

```
rosrun gripper gripper_control_test.py
```

The tuning can be updated with 
```
rosservice call /gripper_set_gamma "value: 0.0" 
```
```
rosservice call /gripper_set_ki "value: 0.0" 
```
```
rosservice call /gripper_set_kp "value: 0.0" 
```


## Step 7: plotting
You can plot a single file with the python script:
```
plot_gripper_force_control.py
```
just add the path to the file.

To plot and get metrics from the final step response: add all paths and then

```
step_response.py
```
For the bode plot
```
plot_bode.py
```
And to get a plot of the control signal 
```
plot_trajectory.py
```







