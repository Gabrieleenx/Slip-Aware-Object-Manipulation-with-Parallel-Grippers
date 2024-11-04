#!/usr/bin/env python3

import rospy
import subprocess
import os
import sys
import time
from gripper.msg import Gripper_target
import numpy as np

parent_dir = "add path"

recording_name = "cardboard_10"

square_wave_amplitude = [5, 10, 5, 15, 5, 20, 5, 25, 5, 25, 15, 25, 5]
square_wave_duration = 1.0

sinusoid_wave_amplitude_high = 15
sinusoid_wave_amplitude_low = 5
sinusoid_wave_frequency = [1, 2, 4, 8, 16, 32, 64]
 #[1, 2, 4, 8, 16, 32, 64, 128]
sinusoid_wave_duration = 5
frequency = 500 


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


def square_wave_gen(pub_target_force):
    rate = rospy.Rate(frequency)  
    time_offset = rospy.Time.now()
    idx = 0 
    while not rospy.is_shutdown():
        
        current_time = rospy.Time.now() - time_offset
        if current_time.to_sec() < (idx + 1)*square_wave_duration:
            force = square_wave_amplitude[idx]
            # publish force 
            msg = Gripper_target()
            msg.header.stamp = rospy.Time.now()
            msg.target = force
            pub_target_force.publish(msg)

        else:
            idx += 1
            if idx >= len(square_wave_amplitude):
                break

        # Sleep to maintain the loop rate
        rate.sleep()


def sinusoidal_wave_gen(pub_target_force):
    rate = rospy.Rate(frequency)  
    time_offset = rospy.Time.now()
    idx = 0 
    while not rospy.is_shutdown():
        
        current_time = rospy.Time.now() - time_offset
        t = current_time.to_sec()
        if t < (idx + 1)*sinusoid_wave_duration:
            force_hz = sinusoid_wave_frequency[idx]
            force = ((np.sin(2 * np.pi * force_hz * t) + 1)/2) * (sinusoid_wave_amplitude_high - sinusoid_wave_amplitude_low) + sinusoid_wave_amplitude_low

            # publish force 
            msg = Gripper_target()
            msg.header.stamp = rospy.Time.now()
            msg.target = force
            pub_target_force.publish(msg)

        else:
            idx += 1
            if idx >= len(sinusoid_wave_frequency):
                msg = Gripper_target()
                msg.header.stamp = rospy.Time.now()
                msg.target = 5.0
                pub_target_force.publish(msg)
                break

        # Sleep to maintain the loop rate
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node('my_node', anonymous=True)
    if len(sys.argv) < 2:
        rospy.loginfo("No arguments provided")
    else:
        for i, arg in enumerate(sys.argv):
            recording_name = arg
            rospy.loginfo(f"Argument {i}: {arg}")

    pub_target_force = rospy.Publisher('/gripper_closed_loop', Gripper_target, queue_size=1, tcp_nodelay=True)  

    topics_to_record = ["/gripper_closed_loop", "/gripper_control", "/gripper_monitor", "/netft_data_1_calibrated"]
    
    bag_directory = parent_dir + "/plotting/gripper_control_bags"
    # Grasp object
    time.sleep(0.1)
    msg = Gripper_target()
    msg.header.stamp = rospy.Time.now()
    msg.target = 5.0
    pub_target_force.publish(msg) 
    
    print("Recording started.")
    bag_process = start_rosbag(topics_to_record, recording_name, bag_directory)

    time.sleep(2)
    # Square wave control 
    square_wave_gen(pub_target_force)

    time.sleep(0.99)
    msg = Gripper_target()
    msg.header.stamp = rospy.Time.now()
    msg.target = 5.0
    pub_target_force.publish(msg)
    time.sleep(0.01)

    # Generate a viarity of different sinusoidal curves 
    sinusoidal_wave_gen(pub_target_force)
    # Record for 10 seconds

    msg = Gripper_target()
    msg.header.stamp = rospy.Time.now()
    msg.target = 5.0
    pub_target_force.publish(msg)
    time.sleep(1)
    stop_rosbag(bag_process)
    print("Recording stopped.")
    msg = Gripper_target()
    msg.header.stamp = rospy.Time.now()
    msg.target = 0.0
    pub_target_force.publish(msg)
    rospy.spin()


