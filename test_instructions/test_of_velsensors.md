# Test of velociy sensors. 

## Step 1: 
Mount the sensor on the test rig. 

## Step 2:
Run the sensor velocity script.
```
rosrun gripper vel_sensors.py
```
Put the sensor againts on of the stops. 100 mm appart

## Step 3:

```
rosservice call /sensor_reset_pos {}
```

Look at data from msgs

```
rostopic echo /velocity_sensor_1
```
or
```
rostopic echo /velocity_sensor_2
```

## Step 5 Linear test
Move sensor to other side of the stop
- Note down the value
- reset pos
Move back
- Note down value
- reset pos

repeat 10x

## Step 5 Linear rejection test
Mode the surface so that one sensor is exposed to the hole half way
Move sensor to other side of the stop
- Note down the value
- reset pos
Move back
- Note down value
- reset pos

repeat 10x

## Step 5 cobined rotaiton and linear test
Put rotaiton pin to allow 180 degrees 
Move sensor to other side of the stop with rotaiton,
- Note down the value
- reset pos
Move back
- Note down value
- reset pos

repeat 10x


## Step 5 Rotation test
Put stops to limit linear movment, and only allow 180 deg
move to one direction
- Note down the value
- reset pos
Move back
- Note down value
- reset pos

repeat 10x

## Step 6:
Calculate mean and sd. 


