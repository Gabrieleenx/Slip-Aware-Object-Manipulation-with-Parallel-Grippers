#!/usr/bin/env python3
import rospy
import copy
import time
import numpy as np
from gripper.srv import FrictionMode, FrictionParam, FrictionParamResponse
from gripper.msg import Sensor_vel
from geometry_msgs.msg import WrenchStamped
from sklearn.linear_model import LinearRegression

class Sensors(object):
    def __init__(self, sensor_1, sensor_2) -> None:
        self.sensor_1 = sensor_1
        self.sensor_2 = sensor_2

class SensorData(object):
    def __init__(self) -> None:
        self.fx = 0.0
        self.fy = 0.0
        self.tau = 0.0
        self.fn = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.a_mu_s = 1.0 # mu_s = a_mu_s * mu_c
        self.mu_c = 1.0
        self.mu_v = 0.1
        self.r = 0.01
        self.u = 0.0        

def get_h(sensor):
    v = np.linalg.norm([sensor.vx, sensor.vy])
    r = sensor.r
    omega = abs(sensor.omega)
    v_s_norm = np.linalg.norm([r*omega, v])
    
    h_t = (v + 1e-8)/(v_s_norm + 1e-8)
    h_w = (r*omega + 1e-8)/(v_s_norm + 1e-8)
    return h_t, h_w


class ActiveFrictionEstimator(object):
    def __init__(self, v_min, omega_min) -> None:
        self.v_min = v_min
        self.omega_min = omega_min

    def update(self, sensor):
        mu_c = sensor.mu_c
        mu_v = sensor.mu_v
        v = np.linalg.norm([sensor.vx, sensor.vy])
        omega = abs(sensor.omega)

        if v < self.v_min or omega>self.omega_min:
            return mu_c, mu_v
        fn = sensor.fn
        R = np.array([[1]])
        h_t, h_w = get_h(sensor)
        r = sensor.r
        u = sensor.u
        H = np.array([[h_t, v]])
        H_T = np.transpose(H)
        R_inv = np.linalg.pinv(R)
        f_t = np.linalg.norm([sensor.fx, sensor.fy])
        z = np.array([[f_t]])
        est_f_t = (h_t*mu_c + mu_v*v)*fn
        z_est = np.array([[est_f_t]])
        print('ft', f_t, 'est ft', est_f_t, 'h_t*mu_c*fn', h_t*mu_c, 'mu_v*v*fn', mu_v*v*fn)
        res = z- z_est
        dx = np.linalg.pinv(H_T.dot(R_inv).dot(H)).dot(H_T).dot(R_inv).dot(res)
        new_mu_c = mu_c + dx[0,0]
        new_mu_v = mu_v + dx[1,0]
        return new_mu_c, new_mu_v

class LinearCoefficentEstimator(object):
    def __init__(self, vs, max_rot_vel, min_fn, nr) -> None:
        # treasholds 
        self.min_v = 2*vs # mm/s, 1e-3 m/s
        self.min_fn = min_fn
        self.max_rotation_vel = max_rot_vel # maximum rotaiton vel for when it is not considered linear motion
        self.mu_c = 1
        self.a_mu_s = 1  # mu_s = a_mu_s * mu_c
        self.mu_v = 1
        self.data_f = []
        self.data_v = []
        self.data_fs = []
        self.nr = nr

    def collect_data(self, sensor):
        # input one of the sensors. 
        if abs(sensor.omega) > self.max_rotation_vel:
            return
        if abs(sensor.fn) < self.min_fn:
            return
        
        v = np.linalg.norm([sensor.vx, sensor.vy])
        f_norm = np.linalg.norm([sensor.fx, sensor.fy])/abs(sensor.fn)
        if v < self.min_v:
            self.data_fs.append(f_norm)
        else:
            self.data_f.append(f_norm)
            self.data_v.append(v)

    def estimate(self):
        try:
            # esimate static frciton 
            mu_s = np.max(self.data_fs)
            # linear estimator 
            print('data_points:', len(self.data_f))
            vel_copy = np.array(self.data_v).reshape(-1, 1)
            self.model = LinearRegression().fit(vel_copy, np.array(self.data_f))
            
            self.mu_v = self.model.coef_[0] 
            self.mu_c = self.model.intercept_    
            self.a_mu_s = mu_s/self.mu_c
            self.a_mu_s = np.max([self.a_mu_s, 1])
            data = np.array([self.data_f,
                            self.data_v])
            #np.save('path_if_you_want_to_save'.npy', data)
            # reset data
            self.data_f = []
            self.data_v = []
            self.data_fs = []
            return copy.copy(self.mu_c), copy.copy(self.mu_v), copy.copy(self.a_mu_s)  
        except:
            print("Not enought data for estimation")
            return 1, 0, 1
            

class RotationCoefficentEstimator(object):
    def __init__(self, max_lin_vel, min_omega, min_fn, nr) -> None:
        self.max_lin_vel = max_lin_vel
        self.min_omega = min_omega
        self.min_fn = min_fn
        self.data_tau_norm = []
        self.data_omega = []
        self.nr = nr

    def collect_data(self, sensor):
        # input one of the sensors. 
        if abs(sensor.omega) < self.min_omega:
            return
        if abs(sensor.fn) < self.min_fn:
            return
        v = np.linalg.norm([sensor.vx, sensor.vy])
        if v > self.max_lin_vel:
            return
        tau_norm = abs(sensor.tau / sensor.fn)
        self.data_tau_norm.append(tau_norm)
        self.data_omega.append(abs(sensor.omega))
        
    def estimate(self, mu_c, mu_v):
        try:
            # linear estimator 
            print('data_points:', len(self.data_omega))
            data_omega_copy = np.array(self.data_omega).reshape(-1, 1)
            self.model = LinearRegression().fit(data_omega_copy, np.array(self.data_tau_norm))
            
            a = self.model.coef_[0] 
            b = self.model.intercept_    
            r = b/mu_c

            if mu_v == 0:
                u = r**2
            else:
                u = a/mu_v
                if abs(u) > 5 * r**2:
                    u = r**2

            data = np.array([self.data_tau_norm,
                            self.data_omega])
            #np.save('path_if_you_want_to_save'.npy', data)
            # reset data
            self.data_tau_norm = []
            self.data_omega = []
            return copy.copy(r), copy.copy(u)  
        except:
            print("Not enought data for estimation")
            return 0.01, 1



class FrictionCoeficientEstimator(object):
    def __init__(self) -> None:
        sensor_1 = SensorData()
        sensor_2 = SensorData()
        self.sensors =  Sensors(sensor_1=sensor_1, sensor_2=sensor_2)
        self.elapsed_time = time.time()
        self.assigned_time = 0
        self.mode = 0
        self.linear_coeff_estimator_1 = LinearCoefficentEstimator(vs=1e-3,
                                                                  max_rot_vel=0.1,
                                                                  min_fn=1,
                                                                  nr=1)
        self.linear_coeff_estimator_2 = LinearCoefficentEstimator(vs=1e-3,
                                                                  max_rot_vel=0.1,
                                                                  min_fn=1,
                                                                  nr=2)
        
        self.rot_coeff_est_1 = RotationCoefficentEstimator(max_lin_vel=5e-3,
                                                           min_omega=0.1,
                                                           min_fn=1,
                                                           nr=1)
        
        self.rot_coeff_est_2 = RotationCoefficentEstimator(max_lin_vel=5e-3,
                                                           min_omega=0.1,
                                                           min_fn=1,
                                                           nr=2)
        
        self.active_est_1 = ActiveFrictionEstimator(v_min=5, omega_min=0.3)
        self.active_est_2 = ActiveFrictionEstimator(v_min=5, omega_min=0.3)
        

        rospy.Subscriber("/netft_data_1_calibrated", WrenchStamped, self.callback_ft_1, tcp_nodelay=True)
        rospy.Subscriber("/netft_data_2_calibrated", WrenchStamped, self.callback_ft_2, tcp_nodelay=True)

        rospy.Subscriber("/velocity_sensor_1", Sensor_vel, self.callback_vel_1, tcp_nodelay=True)
        rospy.Subscriber("/velocity_sensor_2", Sensor_vel, self.callback_vel_2, tcp_nodelay=True)

        rospy.Service('friction_estimation_mode', FrictionMode, self.friction_mode)

        rospy.Service('get_friction_params', FrictionParam, self.get_friction_param)
        # TODO rosservice to save data to txt file 

    def callback_ft_1(self, data):
        self.sensors.sensor_1.fx = data.wrench.force.x
        self.sensors.sensor_1.fy = data.wrench.force.y
        self.sensors.sensor_1.fn = abs(data.wrench.force.z)
        self.sensors.sensor_1.tau = data.wrench.torque.z

    def get_friction_param(self, data):
        respons_srv = FrictionParamResponse()
        respons_srv.mu_c_1 = self.sensors.sensor_1.mu_c
        respons_srv.mu_s_1 = self.sensors.sensor_1.a_mu_s * self.sensors.sensor_1.mu_c
        respons_srv.mu_v_1 = self.sensors.sensor_1.mu_v
        respons_srv.r_1 = self.sensors.sensor_1.r

        respons_srv.mu_c_2 = self.sensors.sensor_2.mu_c
        respons_srv.mu_s_2 = self.sensors.sensor_2.a_mu_s * self.sensors.sensor_2.mu_c
        respons_srv.mu_v_2 = self.sensors.sensor_2.mu_v
        respons_srv.r_2 = self.sensors.sensor_2.r
        return respons_srv

    def callback_ft_2(self, data):
        self.sensors.sensor_2.fx = data.wrench.force.x
        self.sensors.sensor_2.fy = data.wrench.force.y
        self.sensors.sensor_2.fn = abs(data.wrench.force.z)
        self.sensors.sensor_2.tau = data.wrench.torque.z

    def callback_vel_1(self, data):
        self.sensors.sensor_1.vx = data.vx
        self.sensors.sensor_1.vy = data.vy
        self.sensors.sensor_1.omega = data.omega
        self.velocity_callback()

    def callback_vel_2(self, data):
        self.sensors.sensor_2.vx = data.vx
        self.sensors.sensor_2.vy = data.vy
        self.sensors.sensor_2.omega = data.omega

    def friction_mode(self, msg):
        # modes 0: non, 1: linear, 2: rotaion, 3: limit surface, 4 active
        # For all modes except 4 it will return to mode 0 automatically after the assigned time
        self.mode = msg.mode
        self.assigned_time = msg.time
        self.elapsed_time = time.time()
        return True

    def velocity_callback(self):
        
        # update velocity in sensors
        # deep copy sensors 
        sensor_copy = copy.deepcopy(self.sensors)
        # check mode of opteration 5 cases: Non, linear, roation, limit, active
        # Call the appropiate function  
        elapsed_time = time.time() - self.elapsed_time
        if self.mode == 4:
            # activally update friciton parameters 
            # publish friction paramaters and prediction 
            new_mu_c, new_mu_v = self.active_est_1.update(sensor=sensor_copy.sensor_1)
            print('mu_c 1', new_mu_c, 'mu_v 1 ', new_mu_v)
            self.sensors.sensor_1.mu_c = new_mu_c
            self.sensors.sensor_1.mu_v = new_mu_v

            new_mu_c, new_mu_v = self.active_est_2.update(sensor=sensor_copy.sensor_2)
            print('mu_c2 ', new_mu_c, 'mu_v2 ', new_mu_v)
            self.sensors.sensor_2.mu_c = new_mu_c
            self.sensors.sensor_2.mu_v = new_mu_v

        elif self.mode == 0:
            pass

        elif self.mode == 1:
            # linear friciton estimation
            self.linear_coeff_estimator_1.collect_data(sensor_copy.sensor_1)
            self.linear_coeff_estimator_2.collect_data(sensor_copy.sensor_2)
            if elapsed_time > self.assigned_time:
                # update self.sensors with the latest value
                mu_c, mu_v, a_mu_s = self.linear_coeff_estimator_1.estimate()
                self.sensors.sensor_1.mu_c = mu_c
                self.sensors.sensor_1.mu_v = mu_v
                self.sensors.sensor_1.a_mu_s = a_mu_s
                print('Sensor 1 mu_c: ', mu_c, ' mu_v: ', mu_v, ' a_mu_s', a_mu_s)
                mu_c, mu_v, a_mu_s = self.linear_coeff_estimator_2.estimate()
                self.sensors.sensor_2.mu_c = mu_c
                self.sensors.sensor_2.mu_v = mu_v
                self.sensors.sensor_2.a_mu_s = a_mu_s
                print('Sensor 2 mu_c: ', mu_c, ' mu_v: ', mu_v, ' a_mu_s', a_mu_s)
                self.mode = 0
        
        elif self.mode == 2:
            # update roational coefficients
            self.rot_coeff_est_1.collect_data(sensor_copy.sensor_1)
            self.rot_coeff_est_2.collect_data(sensor_copy.sensor_2)

            if elapsed_time > self.assigned_time:
                # update self.sensors with the latest value
                # update self.sensors with the latest value
                r, u = self.rot_coeff_est_1.estimate(self.sensors.sensor_1.mu_c, self.sensors.sensor_1.mu_v)
                self.sensors.sensor_1.r = r
                self.sensors.sensor_1.u = u
                print('Sensor 1 r: ', r, ' u: ', u)
                r, u = self.rot_coeff_est_2.estimate(self.sensors.sensor_2.mu_c, self.sensors.sensor_2.mu_v)
                self.sensors.sensor_2.r = r
                self.sensors.sensor_2.u = u
                print('Sensor 2 r: ', r, ' u: ', u)
                self.mode = 0

        elif self.mode == 3:
            # update limit surface            
            #self.limit_est_1.collect_data(sensor_copy.sensor_1)
            #self.limit_est_2.collect_data(sensor_copy.sensor_2)

            if elapsed_time > self.assigned_time:
                # update self.sensors with the latest value
                #self.limit_est_1.compute()
                #self.limit_est_2.compute()
                self.mode = 0


if __name__ == '__main__':
    rospy.init_node('friction_est_node', anonymous=True)
    estimator = FrictionCoeficientEstimator()
    rospy.spin()



