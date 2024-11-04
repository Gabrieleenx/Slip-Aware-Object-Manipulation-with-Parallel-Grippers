#!/usr/bin/env python3
import rospy
import copy
import time
import numpy as np
from gripper.msg import Sensor_vel, Gripper_target, Target_slippage
from gripper.srv import FrictionParam
from geometry_msgs.msg import WrenchStamped
from gripper.srv import SlippMode

import tf

class Sensor(object):
    def __init__(self) -> None:
        # these should be in the orientation of the middle frame 
        self.fx = 0.0
        self.fy = 0.0
        self.tau = 0.0
        self.fn = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.mu_c = 0.0
        self.mu_s = 0.0
        self.mu_v = 0.0
        self.r = 0.0

class Sensors(object):
    def __init__(self) -> None:
        self.sensor_1 = Sensor()
        self.sensor_2 = Sensor()


class Trapzoidal(object):
    def __init__(self, t_end, p_end, t_c_ratio) -> None:
        self.t_start = 0.0
        self.p_start = 0.0
        self.t_end = t_end
        self.p_end = p_end
        self.t_c_ratio = t_c_ratio
        self.t_c = t_end*t_c_ratio
        self.v_c = self.p_end/(self.t_end - self.t_c)
        self.acc = self.v_c/self.t_c

    def get_point(self, t):
        if t < 0:
            return 0.0, 0.0
        elif t> self.t_end:
            return self.p_end, 0.0
        
        elif t < self.t_c:
            v = self.acc*t
            x = 0.5 * self.acc*t**2
        elif t > self.t_end - self.t_c:
            t_dec = t - self.t_end+ self.t_c
            v = self.v_c - self.acc*t_dec
            x = 0.5 * self.acc*self.t_c**2 + self.v_c*(t - self.t_c) -  0.5 * self.acc*t_dec**2
        else:
            v = self.v_c
            x = 0.5 * self.acc*self.t_c**2 + self.v_c*(t - self.t_c)
        return x, v


class HoldMode(object):
    def __init__(self, hold_margin, min_fn) -> None:
        self.hold_margin = hold_margin
        self.min_fn = min_fn
        self.f_tau_m = 1
        self.fd_send = 0

    def get_fn(self, sensors):
        t_m = abs(sensors.sensor_1.tau + sensors.sensor_2.tau)/2
        fx_m = abs(sensors.sensor_1.fx + sensors.sensor_2.fx)/2
        fy_m = abs(sensors.sensor_1.fy + sensors.sensor_2.fy)/2
        ft_m = np.linalg.norm([fx_m, fy_m])
        mu_c = abs(sensors.sensor_1.mu_c + sensors.sensor_2.mu_c)/2
        r = abs(sensors.sensor_1.r + sensors.sensor_2.r)/2
        r = np.max([r, 1e-5])
        gamma_ft = ((ft_m)+1e-5)/(np.linalg.norm([fx_m, fy_m, t_m/r])+1e-5)
        f_lin_min = ft_m/(mu_c*gamma_ft)

        gamma_tau = ((t_m/r)+1e-5)/(np.linalg.norm([fx_m, fy_m, t_m/r])+1e-5)
        f_tau = self.f_tau_m*t_m/(mu_c  * r * gamma_tau)


        vx = abs(sensors.sensor_1.vx + sensors.sensor_2.vx)/2
        vy = abs(sensors.sensor_1.vy + sensors.sensor_2.vy)/2
        omega =  abs(sensors.sensor_1.omega + sensors.sensor_2.omega)/2
        vs = np.linalg.norm([vx, vy, r*omega])
        f_d = self.hold_margin*np.max([f_lin_min, f_tau, self.min_fn]) + 100*vs
         
        if np.isnan(f_d):
            print('f_d', f_d, 'vs', vs, 'f_tau', f_tau, 'f_lin_min', f_lin_min, 'r', r)
        if self.fd_send < f_d:
            self.fd_send = f_d
        else:
            self.fd_send = 0.95*self.fd_send + (1-0.95)*f_d
        return self.fd_send


class RotationalSlippage(object):
    def __init__(self, min_fn) -> None:
        self.theta = 0.0
        self.t = 0.0
        self.kp = 2
        self.ki = 40
        self.kd = 0.05
        self.ki_floor = 0
        self.kp_cop = 0
        self.target_angle = 0.0
        self.t_end = 0.0
        self.pre_omega = 0.0
        self.min_fn = min_fn
        self.pub_target_traj = rospy.Publisher('/rotation_slippage_traj', Target_slippage, queue_size=1, tcp_nodelay=True) 
        self.f_slip = 0

    def set_goal(self, t, angle):
        # calculate trapezoid
        self.target_angle = angle 
        self.t_end = t
        self.trajectory = Trapzoidal(t_end=t, p_end=angle, t_c_ratio=0.2)
        self.theta = 0.0 
        self.t = 0.0
        self.e_theta_int = 0.0
        self.e_floor_int = 0.0
        self.pre_time = time.time()
        self.pre_omega = 0.0
        self.alpha = 0.95
        self.f_slip = 0

        

    def control(self, sensors):
        done = False
        # average sensors
        vx = abs(sensors.sensor_1.vx + sensors.sensor_2.vx)/2
        vy = abs(sensors.sensor_1.vy + sensors.sensor_2.vy)/2
        omega =  abs(sensors.sensor_1.omega + sensors.sensor_2.omega)/2
        t_m = abs(sensors.sensor_1.tau + sensors.sensor_2.tau)/2
        fx_m = abs(sensors.sensor_1.fx + sensors.sensor_2.fx)/2
        fy_m = abs(sensors.sensor_1.fy + sensors.sensor_2.fy)/2
        fn = abs(sensors.sensor_1.fn + sensors.sensor_2.fn)/2

        ft_m = np.linalg.norm([fx_m, fy_m])
        mu_c = abs(sensors.sensor_1.mu_c + sensors.sensor_2.mu_c)/2
        r = abs(sensors.sensor_1.r + sensors.sensor_2.r)/2
        vs = np.linalg.norm([vx, vy, r*omega])

        time_now = time.time()
        dt = time_now - self.pre_time
        self.pre_time = time_now        

        self.theta += omega * dt
        self.t += dt
        omega_acc = (omega - self.pre_omega)/(dt+1e-10)
        # get trajectory parameters
        theta_d, w_d = self.trajectory.get_point(self.t)

        msg = Target_slippage()
        msg.header.stamp = rospy.Time.now()
        msg.pos = theta_d
        msg.vel = w_d
        msg.pos_measured = self.theta
        msg.vel_measured = omega
        self.pub_target_traj.publish(msg)
        # calculate control omega
        w_c = w_d + self.kp * (theta_d - self.theta)
        #w_c = np.max([0.0, w_c])
        # calcualte desiered force
        #limit_surface_tau = (omega*r + 1e-3)/(vs + 1e-3)
        #f_tau = t_m/(mu_c  * r * limit_surface_tau) 
        self.e_theta_int += (w_c - omega) * dt
        #f_d = f_tau - self.ki * self.e_theta_int + 10*f_tau* (omega - w_c)
        gamma_tau = ((t_m/r)+1e-5)/(np.linalg.norm([fx_m, fy_m, t_m/r])+1e-5)

        if self.f_slip == 0:
            self.f_slip = (t_m/(mu_c*r*gamma_tau))

        if omega_acc < np.pi/8:
            self.f_slip = self.alpha * self.f_slip + (1-self.alpha)*(t_m/(mu_c*r*gamma_tau))
        f_d = self.f_slip - self.ki * self.e_theta_int + self.f_slip*self.kd * omega_acc + 2*self.f_slip*(omega - w_c)
        # calculate force floor
        cop_x = - vy/(omega + 1e-3)
        cop_y = vx/(omega + 1e-3)
        d_cop = np.linalg.norm([cop_x, cop_y])
        
        if vs == 0:
            e = 0
        else:
            e = np.max([0, d_cop - r])
        
        #self.e_floor_int += e*dt 

        f_lin_min = ft_m/mu_c

        f_min = f_lin_min + self.kp_cop * e


        f_d = np.max([f_d, f_min, self.min_fn])
        f_tau = self.f_slip
        print('th_d', theta_d, 'th', self.theta, 'wd', w_d, 'wc', w_c, 'f_tau', f_tau,'fd', f_d, 'fn', fn,  'fmin', f_min,'self.t', self.t, 'omega', omega)

        self.pre_omega = omega

        # chech if done
        if self.target_reached(self.t, omega=omega):
            done = True
        # return force 
    
        return f_d, done
    
    def target_reached(self, t, omega):
        if self.theta >= self.target_angle:
            print(self.theta*180/np.pi)
            return True
        elif t >= self.t_end:
            if omega == 0:
                print(self.theta*180/np.pi)
                return True
        return False


class RotationalHinge(object):
    def __init__(self, treashold, min_fn) -> None:
        self.treashold = treashold
        self.min_fn = min_fn

    def get_fn(self, sensors):
        vx = abs(sensors.sensor_1.vx + sensors.sensor_2.vx)/2
        vy = abs(sensors.sensor_1.vy + sensors.sensor_2.vy)/2

        vt = np.linalg.norm([vx, vy])
        fx_m = abs(sensors.sensor_1.fx + sensors.sensor_2.fx)/2
        fy_m = abs(sensors.sensor_1.fy + sensors.sensor_2.fy)/2
        ft_m = np.linalg.norm([fx_m, fy_m])
        mu_c = abs(sensors.sensor_1.mu_c + sensors.sensor_2.mu_c)/2
        r = abs(sensors.sensor_1.r + sensors.sensor_2.r)/2
        t_m = abs(sensors.sensor_1.tau + sensors.sensor_2.tau)/2

        f_lin_min = ft_m/(mu_c)
        f_min = self.treashold* f_lin_min + 100*vt
        f_min = np.max([self.min_fn, f_min])
        return f_min


class LinearSlippage(object):
    def __init__(self, min_fn) -> None:
        self.pos = 0.0
        self.t = 0.0
        self.kp = 2
        self.ki = 1000
        self.kd = 0.1
        self.ki_floor = 1
        self.target_pos = 0.0
        self.t_end = 0.0
        self.alpha = 0.95
        self.min_fn = min_fn
        self.f_slip = 0
        self.pre_v = 0
        self.pub_target_traj = rospy.Publisher('/linear_slippage_traj', Target_slippage, queue_size=1, tcp_nodelay=True) 


    def set_goal(self, t, pos):
        # calculate trapezoid
        self.target_pos = pos 
        self.t_end = t
        self.trajectory = Trapzoidal(t_end=t, p_end=pos, t_c_ratio=0.2)
        self.pos = 0.0 
        self.t = 0.0
        self.e_v_int = 0.0
        self.e_floor_int = 0.0
        self.pre_time = time.time()
        self.f_slip = 0
        self.pre_v = 0

    def control(self, sensors):
        done = False
        # average sensors
        vx = abs(sensors.sensor_1.vx + sensors.sensor_2.vx)/2
        vy = abs(sensors.sensor_1.vy + sensors.sensor_2.vy)/2
        v = np.linalg.norm([vx, vy])

        fx_m = abs(sensors.sensor_1.fx + sensors.sensor_2.fx)/2
        fy_m = abs(sensors.sensor_1.fy + sensors.sensor_2.fy)/2
        fn = abs(sensors.sensor_1.fn + sensors.sensor_2.fn)/2

        ft_m = np.linalg.norm([fx_m, fy_m])
        mu_c = abs(sensors.sensor_1.mu_c + sensors.sensor_2.mu_c)/2

        time_now = time.time()
        dt = time_now - self.pre_time
        self.pre_time = time_now        

        self.pos += v * dt
        self.t += dt

        # get trajectory parameters
        x_d, v_d = self.trajectory.get_point(self.t)
        msg = Target_slippage()
        msg.header.stamp = rospy.Time.now()
        msg.pos = x_d
        msg.vel = v_d
        msg.pos_measured = self.pos
        msg.vel_measured = v
        self.pub_target_traj.publish(msg)
        # calculate control omega
        v_c = v_d + self.kp * (x_d - self.pos)
        #v_c = np.max([0.0, v_c])
        # calcualte desiered force
        acc = (v - self.pre_v)/dt
        
        if self.f_slip == 0:
            self.f_slip = (ft_m/mu_c)

        if acc < 0.1:
            self.f_slip = self.alpha * self.f_slip + (1-self.alpha)*(ft_m/mu_c)

        self.e_v_int += (v_c - v) * dt
 
        self.pre_v = v
        # calculate force floor
        f_d = self.f_slip - self.ki * self.e_v_int + self.f_slip*self.kd * acc + 50*self.f_slip*(v - v_c)
        print('x_d', x_d, 'self.pos', self.pos, 'ft_m', ft_m, 'fd', f_d, 'fn', fn, 'self.t', self.t, 'vel_c', 50*self.f_slip*(v - v_c), 'acc_c', self.f_slip*self.kd * acc)
        f_d = np.max([f_d, self.min_fn])

        # chech if done
        if self.target_reached(self.t, v=v):
            done = True
        # return force 
        return f_d, done
    
    def target_reached(self, t, v):
        if self.pos >= self.target_pos:
            print(self.pos)
            return True
        elif t >= self.t_end:
            if v == 0:
                print(self.pos)
                return True
        return False



class ControlledSlippage(object):
    def __init__(self) -> None:
        self.slip_mode = 0
        self.hold_mode = HoldMode(hold_margin=2, min_fn=2)
        self.rot_slip = RotationalSlippage(min_fn=0.6)
        self.rot_hinge = RotationalHinge(treashold=1.4, min_fn=0.6)
        self.lin_slip = LinearSlippage(min_fn=0.6)
        self.sensors_transformed = Sensors()
        self.sensors = Sensors()
        self.tf_listener = tf.TransformListener()
        self.target_frame = "gipper_middle_frame"
        rospy.wait_for_service('/get_friction_params')

        self.get_friction_param = rospy.ServiceProxy('get_friction_params', FrictionParam)


        rospy.sleep(0.1)
        self.get_friction_param_func()
        rospy.sleep(0.1)

        self.tf_listener.waitForTransform(self.target_frame, "ft_sensor_1", rospy.Time(), rospy.Duration(50.0))
        (trans, rot) = self.tf_listener.lookupTransform(self.target_frame, "ft_sensor_1", rospy.Time(0))
        rotation_matrix = tf.transformations.quaternion_matrix(rot)
        self.rot_ft_1 = rotation_matrix[:3, :3]

        self.tf_listener.waitForTransform(self.target_frame, "ft_sensor_2", rospy.Time(), rospy.Duration(50.0))

        (trans, rot) = self.tf_listener.lookupTransform(self.target_frame, "ft_sensor_2", rospy.Time(0))
        rotation_matrix = tf.transformations.quaternion_matrix(rot)
        self.rot_ft_2 = rotation_matrix[:3, :3]
        self.tf_listener.waitForTransform(self.target_frame, "vel_sensor_1", rospy.Time(), rospy.Duration(50.0))

        (trans, rot) = self.tf_listener.lookupTransform(self.target_frame, "vel_sensor_1", rospy.Time(0))
        rotation_matrix = tf.transformations.quaternion_matrix(rot)
        self.rot_vel_1 = rotation_matrix[:3, :3]
        self.tf_listener.waitForTransform(self.target_frame, "vel_sensor_2", rospy.Time(), rospy.Duration(50.0))

        (trans, rot) = self.tf_listener.lookupTransform(self.target_frame, "vel_sensor_2", rospy.Time(0))
        rotation_matrix = tf.transformations.quaternion_matrix(rot)
        self.rot_vel_2 = rotation_matrix[:3, :3]

        self.pub_target_force = rospy.Publisher('/gripper_closed_loop', Gripper_target, queue_size=1, tcp_nodelay=True) 

        rospy.Subscriber("/netft_data_1_calibrated", WrenchStamped, self.callback_ft_1, tcp_nodelay=True)
        rospy.Subscriber("/netft_data_2_calibrated", WrenchStamped, self.callback_ft_2, tcp_nodelay=True)

        rospy.Subscriber("/velocity_sensor_2", Sensor_vel, self.callback_vel_2, tcp_nodelay=True)
        rospy.Subscriber("/velocity_sensor_1", Sensor_vel, self.callback_vel_1, tcp_nodelay=True)

        rospy.Service('slippage_control', SlippMode, self.service_callback_slippage)
        print("Slipp control ready")

    def track_object(self):
        pass

    def transform_to_middle_frame(self):
        # rotate forces and fn = abs(fz)
        self.sensors_transformed = copy.deepcopy(self.sensors)
        
        ft_1 = np.array([[self.sensors.sensor_1.fx],
                         [self.sensors.sensor_1.fy],
                         [self.sensors.sensor_1.fn]])
        ft_1_t = np.dot(self.rot_ft_1, ft_1)
        self.sensors_transformed.sensor_1.fx = ft_1_t[0, 0]
        self.sensors_transformed.sensor_1.fy = ft_1_t[1, 0]
        self.sensors_transformed.sensor_1.fn = abs(ft_1_t[2, 0])
        ft_2 = np.array([[self.sensors.sensor_2.fx],
                         [self.sensors.sensor_2.fy],
                         [self.sensors.sensor_2.fn]])
        ft_2_t = np.dot(self.rot_ft_2, ft_2)
        self.sensors_transformed.sensor_2.fx = ft_2_t[0, 0]
        self.sensors_transformed.sensor_2.fy = ft_2_t[1, 0]
        self.sensors_transformed.sensor_2.fn = abs(ft_2_t[2, 0])

        tau_1 = np.array([[0],
                          [0],
                          [self.sensors.sensor_1.tau]])
        tau_1_t = np.dot(self.rot_ft_1, tau_1)
        self.sensors_transformed.sensor_1.tau = tau_1_t[2, 0]

        tau_2 = np.array([[0],
                          [0],
                          [self.sensors.sensor_2.tau]])
        tau_2_t = np.dot(self.rot_ft_2, tau_2)
        self.sensors_transformed.sensor_2.tau = tau_2_t[2, 0]

        vel_1 = np.array([[self.sensors.sensor_1.vx],
                          [self.sensors.sensor_1.vy],
                          [0]])
        vel_1_t = np.dot(self.rot_vel_1, vel_1)
        self.sensors_transformed.sensor_1.vx = vel_1_t[0, 0]
        self.sensors_transformed.sensor_1.vy = vel_1_t[1, 0]

        vel_2 = np.array([[self.sensors.sensor_2.vx],
                          [self.sensors.sensor_2.vy],
                          [0]])
        vel_2_t = np.dot(self.rot_vel_2, vel_2)

        self.sensors_transformed.sensor_2.vx = vel_2_t[0, 0]
        self.sensors_transformed.sensor_2.vy = vel_2_t[1, 0]

        omega_1 = np.array([[0],
                          [0],
                          [self.sensors.sensor_1.omega]])
        omega_1_t = np.dot(self.rot_vel_1, omega_1)
        self.sensors_transformed.sensor_1.omega = omega_1_t[2, 0]

        omega_2 = np.array([[0],
                          [0],
                          [self.sensors.sensor_2.omega]])
        omega_2_t = np.dot(self.rot_vel_2, omega_2)
        self.sensors_transformed.sensor_2.omega = omega_2_t[2, 0]

    def get_friction_param_func(self):
        # service call to get friction and contace parameter. 
        
        resp = self.get_friction_param(True)
        self.sensors.sensor_1.mu_c = resp.mu_c_1
        self.sensors.sensor_1.mu_s = resp.mu_s_1
        self.sensors.sensor_1.mu_v = resp.mu_v_1
        self.sensors.sensor_1.r = resp.r_1

        self.sensors.sensor_2.mu_c = resp.mu_c_2
        self.sensors.sensor_2.mu_s = resp.mu_s_2
        self.sensors.sensor_2.mu_v = resp.mu_v_2
        self.sensors.sensor_2.r = resp.r_2
        """ #plastic tests 
        
        self.sensors.sensor_1.mu_c = 0.354889337791966
        self.sensors.sensor_1.mu_s = 1.15053795907255 * 0.354889337791966
        self.sensors.sensor_1.mu_v = 11.5114807909912
        self.sensors.sensor_1.r = 0.00768999681330613

        self.sensors.sensor_2.mu_c = 0.364793816303355
        self.sensors.sensor_2.mu_s = 1.18677084446741 * 0.364793816303355
        self.sensors.sensor_2.mu_v = 4.45369047263295
        self.sensors.sensor_2.r = 0.00575330798652555
        """
        
        """
        
         #wood tests 
        self.sensors.sensor_1.mu_c = 0.459095140321046
        self.sensors.sensor_1.mu_s = 1.00372556590224 * 0.459095140321046
        self.sensors.sensor_1.mu_v = -1.67413487735187
        self.sensors.sensor_1.r = 0.00714739738459398

        self.sensors.sensor_2.mu_c = 0.404417505794869
        self.sensors.sensor_2.mu_s = 1.11350780949767 * 0.404417505794869
        self.sensors.sensor_2.mu_v = -1.06819401498444
        self.sensors.sensor_2.r = 0.000190167232091308
        """
        """
        
        # case 
        self.sensors.sensor_1.mu_c = 0.538393248975308

        self.sensors.sensor_1.mu_s = 1.0 * 0.538393248975308
        self.sensors.sensor_1.mu_v = -0.363302269503817
        self.sensors.sensor_1.r = 0.00571535847041971

        self.sensors.sensor_2.mu_c = 0.467251222491261
        self.sensors.sensor_2.mu_s = 1.0 * 0.467251222491261
        self.sensors.sensor_2.mu_v = 1.41054439671736
        self.sensors.sensor_2.r = 0.00341122169683117
        """
        
        """
        # cardboard 
        self.sensors.sensor_1.mu_c = 0.513560285188774
        self.sensors.sensor_1.mu_s = 1.00831205123625 *  0.513560285188774
        self.sensors.sensor_1.mu_v =1.32327944414497
        self.sensors.sensor_1.r = 0.00484036627254104

        self.sensors.sensor_2.mu_c = 0.472721170931145
        self.sensors.sensor_2.mu_s = 1.06728488405522* 0.472721170931145
        self.sensors.sensor_2.mu_v = 1.0862840625844
        self.sensors.sensor_2.r = 0.0062955802002477
        """        

        #print(resp.mu_c_1,  resp.r_1)

    def service_callback_slippage(self, data):
        self.get_friction_param_func()

        # set goal 
        self.slip_mode = data.mode
        if data.mode == 1:
            angle = data.angle * np.pi / 180
            self.rot_slip.set_goal(t=data.t,
                                   angle=angle)
            self.slip_mode = 1

        if data.mode == 2:
            self.lin_slip.set_goal(t=data.t, pos=data.dist)
            self.slip_mode = 2

        return True
    
    def callback_vel_2(self, data):
        self.sensors.sensor_2.vx = data.vx
        self.sensors.sensor_2.vy = data.vy
        self.sensors.sensor_2.omega = data.omega
    
    def callback_vel_1(self, data):
        self.sensors.sensor_1.vx = data.vx
        self.sensors.sensor_1.vy = data.vy
        self.sensors.sensor_1.omega = data.omega
        # vel 1 msg is used as a trigger
        self.slippage_master()

    def callback_ft_1(self, data):
        self.sensors.sensor_1.fx = data.wrench.force.x
        self.sensors.sensor_1.fy = data.wrench.force.y
        self.sensors.sensor_1.fn = abs(data.wrench.force.z)
        self.sensors.sensor_1.tau = data.wrench.torque.z

    def callback_ft_2(self, data):
        self.sensors.sensor_2.fx = data.wrench.force.x
        self.sensors.sensor_2.fy = data.wrench.force.y
        self.sensors.sensor_2.fn = abs(data.wrench.force.z)
        self.sensors.sensor_2.tau = data.wrench.torque.z

    def slippage_master(self):

        # transfrom all sensors
        self.transform_to_middle_frame()

        # track object (not important, for now)

        done = False
        
        if self.slip_mode == 1:
            fn, done = self.rot_slip.control(sensors=self.sensors_transformed)
        elif self.slip_mode == 2:
            fn, done = self.lin_slip.control(sensors=self.sensors_transformed)
        elif self.slip_mode == 3:
            fn = self.rot_hinge.get_fn(sensors=self.sensors_transformed)
        elif self.slip_mode == 4:
            fn = self.hold_mode.get_fn(self.sensors_transformed)
        else:
            return
        
        if done:
            self.slip_mode = 4
            fn = self.hold_mode.get_fn(self.sensors_transformed)
        
        # publish closed loop target.

        msg = Gripper_target()
        msg.header.stamp = rospy.Time.now()
        msg.target = fn
        self.pub_target_force.publish(msg) 


if __name__ == '__main__':
    rospy.init_node('controlled_slippage_node', anonymous=True)
    controller = ControlledSlippage()
    rospy.spin()

