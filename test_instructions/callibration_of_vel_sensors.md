
# Calibratoin of velocity sensors

## Step 1: 
Mount the sensor on the test rig. 

## Step 2:
Run the sensor velocity script.
```
rosrun gripper vel_sensors.py
```
Put the sensor againts on of the stops. 100 mm appart


## Step 4:
```
rosrun gripper calibrate_sensors.py --sensor 1 --chip 3 --direction "y"
```
and follow instructions

do for each sensor and chip and direction. 

## Step 5:
Run

```
analyse_sensor_calibration.py
```
for each bag file.

Finetune the values and update the calibration in 
```
vel_sensors.py
```








































