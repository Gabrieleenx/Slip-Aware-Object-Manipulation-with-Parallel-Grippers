#!/usr/bin/env python3
import serial, time
import rospy
import sys
import serial.tools.list_ports
import threading
import numpy as np
from gripper.msg import Sensor_vel
import threading
from gripper.srv import SensorMode
from std_srvs.srv import Empty, EmptyResponse


""" 
        cal_nr = {0,x: radious, 1,0: s_1_x_a, 1,1: s_1_x_b,
                  1,2: s_1_y_a, 1,3: s_1_y_b,
                  2,0: s_2_x_a, 2,1: s_2_x_b,
                  2,2: s_2_y_a, 2,3: s_2_y_b,
                  3,0: s_3_x_a, 3,1: s_3_x_b,
                  3,2: s_3_y_a, 3,3: s_3_y_b,}
"""

class Calbibration_data_1():
    def __init__(self) -> None:
        self.r = 24.3 # mm
        self.s1xa = 0.0 # ax +b
        self.s1xb = 0.8847 # ax +b
        self.s1ya = 0.00031 # ax +b
        self.s1yb = 0.9066 # ax +b
        self.s2xa = 0.0 # ax +b
        self.s2xb = 0.8852 # ax +b
        self.s2ya = 0.00029 # ax +b
        self.s2yb = 0.8831 # ax +b
        self.s3xa = 0.0 # ax +b
        self.s3xb = 0.9313 # ax +b
        self.s3ya = 0.00048 # ax +b
        self.s3yb = 0.9284 # ax +b

class Calbibration_data_2():
    def __init__(self) -> None:
        self.r = 24.3 # mm
        self.s1xa = 0.0 # ax +b
        self.s1xb = 0.7086 # ax +b
        self.s1ya = 0.00027 # ax +b
        self.s1yb = 0.7029 # ax +b
        self.s2xa = 0.0 # ax +b
        self.s2xb = 0.8573 # ax +b
        self.s2ya = 0.00029 # ax +b
        self.s2yb = 0.88 # ax +b
        self.s3xa = 0.0 # ax +b
        self.s3xb = 0.8861# ax +b
        self.s3ya = 0.0003 # ax +b
        self.s3yb = 0.9008 # ax +b

class Sensors(object):
    def __init__(self, baudrate) -> None:
        # mutex
        self.pre_time1 = time.time()
        self.pre_time2 = time.time()
        self.x1 = 0
        self.y1 = 0
        self.theta1 = 0
        self.x2 = 0
        self.y2 = 0
        self.theta2 = 0
        self.lock = threading.Lock()
        self.baudrate = baudrate
        self.time_open = time.time()
        self.sensor_1 , self.sensor_2 = self.find_microcontroller_port(baudrate)
        self.cal_1 = Calbibration_data_1()
        self.cal_2 = Calbibration_data_2()

        time.sleep(0.1)
        
        self.pub_sensor_1 = rospy.Publisher('/velocity_sensor_1', Sensor_vel, queue_size=5, tcp_nodelay=True)
        self.pub_sensor_2 = rospy.Publisher('/velocity_sensor_2', Sensor_vel, queue_size=5, tcp_nodelay=True)
        rospy.Service('sensor_individual_mode', SensorMode, self.sensor_mode)
        rospy.Service('sensor_reset_pos', Empty, self.reset_pos)

    def calibrate_sensor_1(self):
        print("Velocity sensor 1: updating parameters")
        msg ="k"+ "0" + "0" + str(self.cal_1.r)  + "\n"
        self.sensor_1.write(msg.encode())
        time.sleep(0.01)
        msg ="k"+ "1" + "0" + str(self.cal_1.s1xa)  + "\n"
        self.sensor_1.write(msg.encode())
        time.sleep(0.01)
        msg ="k"+ "1" + "1" + str(self.cal_1.s1xb)  + "\n"
        self.sensor_1.write(msg.encode())
        time.sleep(0.01)
        msg ="k"+ "1" + "2" + str(self.cal_1.s1ya)  + "\n"
        self.sensor_1.write(msg.encode())
        time.sleep(0.01)
        msg ="k"+ "1" + "3" + str(self.cal_1.s1yb)  + "\n"
        self.sensor_1.write(msg.encode())
        time.sleep(0.01)
        
        msg ="k"+ "2" + "0" + str(self.cal_1.s2xa)  + "\n"
        self.sensor_1.write(msg.encode())
        time.sleep(0.01)
        msg ="k"+ "2" + "1" + str(self.cal_1.s2xb)  + "\n"
        self.sensor_1.write(msg.encode())
        time.sleep(0.01)
        msg ="k"+ "2" + "2" + str(self.cal_1.s2ya)  + "\n"
        self.sensor_1.write(msg.encode())
        time.sleep(0.01)
        msg ="k"+ "2" + "3" + str(self.cal_1.s2yb)  + "\n"
        self.sensor_1.write(msg.encode())
        time.sleep(0.01)

        msg ="k"+ "3" + "0" + str(self.cal_1.s3xa)  + "\n"
        self.sensor_1.write(msg.encode())
        time.sleep(0.01)
        msg ="k"+ "3" + "1" + str(self.cal_1.s3xb)  + "\n"
        self.sensor_1.write(msg.encode())
        time.sleep(0.01)
        msg ="k"+ "3" + "2" + str(self.cal_1.s3ya)  + "\n"
        self.sensor_1.write(msg.encode())
        time.sleep(0.01)
        msg ="k"+ "3" + "3" + str(self.cal_1.s3yb)  + "\n"
        self.sensor_1.write(msg.encode())
        time.sleep(0.01)

    def calibrate_sensor_2(self):
        msg ="k"+ "0" + "0" + str(self.cal_2.r)  + "\n"
        self.sensor_2.write(msg.encode())
        time.sleep(0.01)
        msg ="k"+ "1" + "0" + str(self.cal_2.s1xa)  + "\n"
        self.sensor_2.write(msg.encode())
        time.sleep(0.01)
        msg ="k"+ "1" + "1" + str(self.cal_2.s1xb)  + "\n"
        self.sensor_2.write(msg.encode())
        time.sleep(0.01)
        msg ="k"+ "1" + "2" + str(self.cal_2.s1ya)  + "\n"
        self.sensor_2.write(msg.encode())
        time.sleep(0.01)
        msg ="k"+ "1" + "3" + str(self.cal_2.s1yb)  + "\n"
        self.sensor_2.write(msg.encode())
        time.sleep(0.01)
        
        msg ="k"+ "2" + "0" + str(self.cal_2.s2xa)  + "\n"
        self.sensor_2.write(msg.encode())
        time.sleep(0.01)
        msg ="k"+ "2" + "1" + str(self.cal_2.s2xb)  + "\n"
        self.sensor_2.write(msg.encode())
        time.sleep(0.01)
        msg ="k"+ "2" + "2" + str(self.cal_2.s2ya)  + "\n"
        self.sensor_2.write(msg.encode())
        time.sleep(0.01)
        msg ="k"+ "2" + "3" + str(self.cal_2.s2yb)  + "\n"
        self.sensor_2.write(msg.encode())
        time.sleep(0.01)

        msg ="k"+ "3" + "0" + str(self.cal_2.s3xa)  + "\n"
        self.sensor_2.write(msg.encode())
        time.sleep(0.01)
        msg ="k"+ "3" + "1" + str(self.cal_2.s3xb)  + "\n"
        self.sensor_2.write(msg.encode())
        time.sleep(0.01)
        msg ="k"+ "3" + "2" + str(self.cal_2.s3ya)  + "\n"
        self.sensor_2.write(msg.encode())
        time.sleep(0.01)
        msg ="k"+ "3" + "3" + str(self.cal_2.s3yb)  + "\n"
        self.sensor_2.write(msg.encode())
        time.sleep(0.01)


    def reset_pos(self, req):
        self.x1 = 0
        self.y1 = 0
        self.theta1 = 0
        self.x2 = 0
        self.y2 = 0
        self.theta2 = 0
        return EmptyResponse()
    
    def find_microcontroller_port(self,baudrate):
        sensor_1 = None
        sensor_2 = None
        ports = serial.tools.list_ports.comports()
        sensor_ports = []
        for port in ports:
            print(port.description)
            if 'CP2102N USB to UART Bridge Controller' in port.description:
                print("Microcontroller found at:", port.device)
                sensor_ports.append(port.device)
        if len(sensor_ports) == 0:
            print("Vel sensors: Microcontroller not found.")
            print("Vel sensors: Exiting the script...")
            sys.exit()  # Exit the script 
        time.sleep(0.1)
        # Handshake to figure out sensor 1 vs sensor 2. 
        print(sensor_ports)
        for sensor_port in sensor_ports:
            msg = "n"  + "\n"
            sensor=serial.Serial(sensor_port, baudrate)
            sensor.timeout = 1

            time.sleep(0.2)
            sensor.write(msg.encode())
            while True:
                msg_out = self.read_serial(sensor)
                if msg_out == "resynch":
                    print("Vel sensors:", msg_out)
                    break            
                
                if msg_out == "sensor_1":
                    print("Vel sensors:", "sensor 1 connected")
                    sensor_1 = sensor
                    break
                elif msg_out == "sensor_2":
                    print("Vel sensors:", "sensor 2 connected")
                    sensor_2 = sensor
                    break
        return sensor_1, sensor_2 
    
    def sensor_mode(self, sensor_msg):
        msg = "c"  + str(sensor_msg.chip) + "\n"

        if sensor_msg.sensor == 1:
            self.sensor_1.write(msg.encode())
            print("write to sensor 1:", msg)
        if sensor_msg.sensor == 2:
            self.sensor_2.write(msg.encode())
            print("write to sensor 2:", msg)
        return True
    
    def read_serial(self, sensor_port):
        data = sensor_port.readline()[:-2] # keep everyting but the new line 
        if data: # if there is data and not just a blank line
            try:
                data_str = string = data.decode('utf-8')
                data_str_vals = data_str.split("/t") # remove b' and ' from byte array
                key = str(data_str_vals[0])
                if key == "v":
                    vx = float(data_str_vals[1])/1e3 #mm to m
                    vy = float(data_str_vals[2])/1e3 #mm to m
                    omega = float(data_str_vals[3])
                    return vx, vy, omega
                elif key == "n":
                    return str(data_str_vals[1])
                else:
                    print("else ", key)
                    print(data_str_vals)
                
            except Exception as error:
                print(error)
        else:
            if sensor_port.isOpen():
                print("Vel sensors:", "Serial port is open")
            else:
                print("Vel sensors:", "Serial port is not open")
            print("Vel sensors:", 'No data, time since opend ', time.time() - self.time_open)

    def publish(self, pub, vx, vy, omega, x, y, theta):
        msg = Sensor_vel()
        msg.header.stamp = rospy.Time.now()
        msg.vx = vx
        msg.vy = vy
        msg.omega = omega
        msg.x = x
        msg.y = y
        msg.theta = theta
        pub.publish(msg)

    def sensor_1_loop(self):
        print("Velocity sensor 1")
        self.calibrate_sensor_1()
        while not rospy.is_shutdown():
            try:
                time_now = time.time()
                dt = time_now - self.pre_time1
                self.pre_time1 = time_now
                vx, vy, omega = self.read_serial(self.sensor_1)
                self.x1 += vx*dt
                #self.x1 += np.linalg.norm([vx*dt, vy*dt])
                self.y1 += vy*dt
                self.theta1 += omega*dt
                self.publish(self.pub_sensor_1, vx, vy, omega, self.x1, self.y1, self.theta1)
            except:
                pass

    def sensor_2_loop(self):
        print("Velocity sensor 2")
        self.calibrate_sensor_2()
    
        while not rospy.is_shutdown():
            try:
                time_now = time.time()
                dt = time_now - self.pre_time2
                self.pre_time2 = time_now
                vx, vy, omega = self.read_serial(self.sensor_2)
                self.x2 += vx*dt
                self.y2 += vy*dt
                self.theta2 += omega*dt
                self.publish(self.pub_sensor_2, vx, vy, omega, self.x2, self.y2, self.theta2)
            except:
                pass


    def main_loop(self):

        if self.sensor_1 != None:
            thread1 = threading.Thread(target=self.sensor_1_loop)
            thread1.start()

        if self.sensor_2 != None:
            thread2 = threading.Thread(target=self.sensor_2_loop)
            thread2.start()
        

        rospy.spin()
        
        if self.sensor_1 != None:
            thread1.join()
        if self.sensor_2 != None:
            thread2.join()

        print('Communication closed')


if __name__ == '__main__':
    rospy.init_node('vel_sensor', anonymous=True)
    print('Starting velocity sensors')

    sensor = Sensors(baudrate=115200)
    sensor.main_loop()
