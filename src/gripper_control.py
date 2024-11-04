#!/usr/bin/env python3
import serial, time
import rospy
import sys
import serial.tools.list_ports
from gripper.msg import Gripper_target, Gripper_monitor
import threading
import random
import os
import tf


sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import utils
import numpy as np


class Gripper(object):
    def __init__(self, baudrate, tf_frames) -> None:
        # mutex
        self.lock = threading.Lock()
        self.baudrate = baudrate
        self.motor_ready = True #False
        self.calibrated = False
        self.tf_frames = tf_frames

        self.port = self.find_microcontroller_port()
        self.gripper = serial.Serial(self.port, baudrate) # , parity="O"
        print(self.gripper)
        while not self.gripper.isOpen():
            time.sleep(0.1)
            print("Gripper:","Waiting for connection to be open")
        print("Gripper:","Connection is open")
        self.time_open = time.time()
        self.gripper.timeout = 2.5
        self.target = 0
        self.target_force = 0
        self.angle = 0 # rad
        self.position = 0 # in mm 
        self.velocity = 0 # in mm/s
        self.filterd_velocity = 0 # in mm/s

        self.pre_time = time.time()
        self.pos_est = 0
        self.vel_est = 0
        self.vel_int = 0

        self.counter = 0

        self.pre_position = 0
        self.pre_time = time.time()
        self.force_per_volt = 3.9166
        self.target_v = 0.0
        self.min_angle = 0 # rad
        self.max_angle = 0 # rad
        self.mm_per_rad = 8.6751614
        self.pub = rospy.Publisher('/gripper_monitor', Gripper_monitor, queue_size=5, tcp_nodelay=True)
        rospy.Subscriber("/gripper_control", Gripper_target, self.send_target_sub, tcp_nodelay=True)

        self.calibrate_position()

    def find_microcontroller_port(self):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            print('Hi', port.description, 'he', port.device)
            #if 'NanoESP32' in port.description:
            if 'USB JTAG/serial'  in port.description:
            #if 'Arduino Nano Every' in port.description:
                print("Microcontroller found at:", port.device)
                return port.device
        print("Gripper:","Microcontroller not found.")
        print("Gripper:","Exiting the script...")
        sys.exit()  # Exit the script  
        return None 

    def send_target_sub(self, data):
        self.lock.acquire()
        target_v = np.clip(-data.target/self.force_per_volt, -20, 20)
        self.target_v = round(target_v, 2)
        self.target_force = data.target
        self.lock.release()
        if self.calibrated:
            self.send_target()  

    def send_target(self):
        if self.motor_ready:
            msg = 'T' + str(self.target_v) + "\n"
            self.gripper.write(msg.encode())
            self.gripper.flush()
        else:
            print("Gripper:",'not ready')
            return
        
    def target_tracking_loop(self, pos, kp, ki):
        time_now = time.time()
        dt =  time_now - self.pre_time
        self.pre_time = time_now
        self.pos_est += self.vel_est*dt 
        pos_error = pos - self.pos_est
        self.vel_int += pos_error * ki * dt
        self.vel_est = pos_error * kp + self.vel_int
        return self.vel_est, self.vel_int

    
    def read_serial(self):
        try:
            data = self.gripper.readline()[:-2] # keep everyting but the new line 
        except Exception as error:
            print("Gripper:",'Something reading wrong', time.time() - self.time_open)
            print("Gripper:",error)
            ports = serial.tools.list_ports.comports()
            for port in ports:
                print(port.description, port.device)
            self.counter += 1
            if self.counter > 5:
                rospy.signal_shutdown("Exiting application")
            return
        if data: # if there is data and not just a blank line
            data_str = string = data.decode('utf-8')

            data_str_vals = data_str.split("\t") # remove b' and ' from byte array
            self.lock.acquire()
            try:
                #self.target = float(data_str_vals[0])
                self.angle = float(data_str_vals[0])
                self.angle_to_position(self.angle)
                #self.est_velocity()
                _,_ = self.target_tracking_loop(self.position, kp=40, ki=500)
                self.voltage_to_force(self.target)
                self.motor_ready = True
                self.publish()
                if self.calibrated:
                    #self.send_target()
                    pass
                    
            except Exception as error:
                print("Gripper:",error)
                print("Gripper:",data_str)
            self.lock.release()
        else:
            if self.gripper.isOpen():
                print("Gripper:","Serial port is open")
            else:
                print("Serial port is not open")
            print("Gripper:",'No data, time since opend ', time.time() - self.time_open)




    def publish(self):
        msg = Gripper_monitor()
        msg.header.stamp = rospy.Time.now()
        msg.angle = self.angle
        msg.target_voltage = self.target
        msg.target_force = self.target_force
        msg.position = self.position
        #msg.velocity = self.filterd_velocity
        msg.velocity =self.vel_est
        self.pub.publish(msg)

    def angle_to_position(self, angle):
        angle_temp = angle - self.min_angle
        self.position = angle_temp*self.mm_per_rad

    def voltage_to_force(self, voltage):
        self.target_force = voltage*self.force_per_volt

    def est_velocity(self):
        delta_pos = self.position - self.pre_position
        time_now = time.time()
        delta_time = time_now - self.pre_time
        self.velocity = delta_pos/delta_time
        alpha = 0.1
        self.filterd_velocity = alpha*self.velocity + (1- alpha)*self.filterd_velocity
        self.pre_time = time_now
        self.pre_position = self.position

    def calibrate_position(self):
 
        print("Gripper:","Calibrating")
        self.calibrated = False

        while not self.motor_ready and not rospy.is_shutdown():
            self.read_serial()
        self.pos_est = self.position

        # find closed position 
        alpha = 0.98    
        
        found_min = False
        roling_avg = 1
        prv_angle = self.angle
        self.target_v = -4.0
        self.send_target()
        while not found_min and not rospy.is_shutdown():
            self.read_serial()  
            e = abs(self.angle - prv_angle)  
            roling_avg = (1-alpha)*e + alpha*roling_avg 
            prv_angle = self.angle

            if roling_avg < 1e-3:
                found_min = True
                self.min_angle = self.angle
                print("Gripper:",'Min angle', self.min_angle)
        
        found_max = False
        roling_avg = 1
        prv_angle = self.angle
        
        self.target_v = 4.0
        self.send_target()
        while not found_max and not rospy.is_shutdown():
            self.read_serial()  
            e = abs(self.angle - prv_angle)  
            roling_avg = (1-alpha)*e + alpha*roling_avg 
            prv_angle = self.angle

            if roling_avg < 1e-3:
                found_max = True
                self.max_angle = self.angle
                print("Gripper:",'Max angle', self.max_angle)

        self.target_v = 0.0
        self.send_target()

        if rospy.is_shutdown():
            self.gripper.close()
            time.sleep(0.05)
            print("Gripper:",'Communication closed')

        print("Gripper:","Calibration done")  
        self.calibrated = True

    def main_loop(self):
        while not rospy.is_shutdown():
            self.read_serial()
            self.tf_frames.updateGripper(self.position)
        
        self.gripper.close()
        time.sleep(0.05)
        print("Gripper:",'Communication closed')


if __name__ == '__main__':
    rospy.init_node('gripper_control', anonymous=True)
    
    rot_1 = tf.transformations.quaternion_from_euler(0, 0, 47.5 * np.pi / 180, 'rxyz')
    ft_sensor_1 = utils.FramePose(position=np.array([0.0, 0.0, 0.0]), quaternion=rot_1)
    vel_sensor_1 = utils.FramePose(position=np.array([0.0, 0.0, 0.0078]), quaternion=np.array([0.0, 0.0, 0.0, 1.0]))
    
    rot_2 = tf.transformations.quaternion_from_euler(np.pi, 0, -132.5 * np.pi / 180, 'rxyz')
    ft_sensor_2 = utils.FramePose(position=np.array([0.0, 0.0, 0.0]), quaternion=rot_2)
    vel_sensor_2 = utils.FramePose(position=np.array([0.0, 0.0, 0.0078]), quaternion=np.array([0.0, 0.0, 0.0, 1.0]))
    
    middle = tf.transformations.quaternion_from_euler(np.pi/2, 0, 0, 'rxyz')
    middle_frame = utils.FramePose(position=np.array([0.0, 0.0, 0.173]), quaternion=middle)
    gripper_tf = utils.TfBroadcastFrames(ft_sensor_1, vel_sensor_1, ft_sensor_2, vel_sensor_2, middle_frame)
    
    gripper = Gripper(baudrate=115200, tf_frames=gripper_tf)
    
    gripper.main_loop()
