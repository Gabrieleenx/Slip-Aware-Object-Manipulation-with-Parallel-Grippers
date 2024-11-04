#!/usr/bin/env python3

import rospy
import argparse
import time
import os
import subprocess
import numpy as np
from gripper.srv import SensorMode, SensorModeRequest 
sensor = 1
chip = 3
direction = 'y'
path_to_bag_folder = ".../gripper/sensor_test/calibration_bags" # put your own path here

def start_rosbag(topics, bag_name="recording", directory="."):
    """
    Start recording a rosbag with the specified topics and save it to the specified directory.
    
    :param topics: List of topics to record.
    :param bag_name: Name of the bag file.
    :param directory: Directory where the bag file will be saved.
    :return: The subprocess.Popen object for the rosbag recording process.
    """
    if not os.path.exists(directory):
        os.makedirs(directory)
    bag_path = os.path.join(directory, bag_name)
    command = ['rosbag', 'record', '-O', bag_path] + topics
    process = subprocess.Popen(command)
    return process

def stop_rosbag(process):
    """
    Stop the rosbag recording process.
    
    :param process: The subprocess.Popen object for the rosbag recording process.
    """
    process.terminate()
    process.wait()


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="ROS Node with Multiple Input Arguments")
    # Define the arguments
    
    parser.add_argument('--sensor', type=int, default=1, help="Which sensor")
    parser.add_argument('--chip', type=int, default=2, help="Which chip on the sensor")
    parser.add_argument('--direction', type=str, default=False, help="x or y direction")
    
    # Parse the arguments
    args = parser.parse_args()

    rospy.init_node('calibrate', anonymous=True)
    
    sensor = args.sensor
    chip = args.chip
    direction = args.direction
    if sensor == 1:
        topics_to_record = ["/velocity_sensor_1"]
    elif sensor == 2:
        topics_to_record = ["/velocity_sensor_2"]
    else:
        rospy.signal_shutdown("Not correct sensor nr")

    bag_directory = path_to_bag_folder 
    recording_name = "Sensor"+str(sensor)+"chip"+str(chip)+direction
    print("Move to stop on right side.")
    rospy.wait_for_service('/sensor_individual_mode')
    sensor_individual_mode = rospy.ServiceProxy('/sensor_individual_mode', SensorMode)
    request = sensor_individual_mode(sensor=sensor, chip=chip)

    # Grasp object
    print("Recording started.")
    bag_process = start_rosbag(topics_to_record, recording_name, bag_directory)
    time.sleep(4)
    print("Move to left side slowly (10s)")
    time.sleep(15)
    print("Move to right side slowly (10s)")
    time.sleep(15)
    
    print("Move to left side medium slowly (5s)")
    time.sleep(10)
    print("Move to right side medium slowly (5s)")
    time.sleep(10)
   
    print("Move to left side medium (3s)")
    time.sleep(8)
    print("Move to right side medium (3s)")
    time.sleep(8)

    print("Move to left side medium fast (2s)")
    time.sleep(7)
    print("Move to right side medium fast (2s)")
    time.sleep(7)

    print("Move to left side fast (1s)")
    time.sleep(6)
    print("Move to right side fast (1s)")
    time.sleep(6)

    print("Move to left side very fast (<1s)")
    time.sleep(5)
    print("Move to right side very fast (<1s)")
    time.sleep(5)


    stop_rosbag(bag_process)
    print("Recording stopped.")
    request = sensor_individual_mode(sensor=sensor, chip=0)
    rospy.spin()


