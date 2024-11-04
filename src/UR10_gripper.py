#!/usr/bin/env python3

import rospy
import numpy as np
import sys
from gripper.srv import UR10_gripper
parent_dir = "add path"

rospy.init_node('UR10_gripper', anonymous=True)

import UR10_utils
import tf

bag_directory = parent_dir +  "/UR_experiments/bags"

friction_vertical_move = -0.01 # meters
friction_rotational_move = 10 # degrees
hinge_pose_rot_start = 60
hinge_pose_rot_end = 120 
hold_height = 0.2
go_to_obj_pos = np.array([-0.1, -0.85, 0.23])


go_to_obj_rot = tf.transformations.quaternion_from_euler(np.pi/2, 0,np.pi/2, 'rxyz')
go_to_obj_rot_friction = tf.transformations.quaternion_from_euler(np.pi/2+friction_rotational_move*np.pi/180, 0,np.pi/2, 'rxyz')
go_to_rot_hinge_start = tf.transformations.quaternion_from_euler(np.pi/2+hinge_pose_rot_start*np.pi/180, 0,np.pi/2, 'rxyz')
go_to_rot_hinge_end = tf.transformations.quaternion_from_euler(np.pi/2+hinge_pose_rot_end*np.pi/180, 0,np.pi/2, 'rxyz')
reset_list = [-2.37105375925173, -1.6793082396136683, -1.4216583410846155, 0.9087687730789185, 1.4216585159301758, -1.5707958380328577]




# Grasp and ungrasp object
class SystemState(object):
    def __init__(self) -> None:
        self.holding_object = False
        self.at_object_pick_up = False
        self.at_hinge_pose = False
        self.joints_reset = False
        self.gripper_open = False
        self.friction_estimated = True

def grasp(data, state):
    UR10_utils.set_gripper_force(10)
   
    #if state.friction_estimated:
    
    
    rospy.sleep(2)
    UR10_utils.activate_hold_mode()
    state.holding_object = True
    state.gripper_open = False

def release(data, state):
    # call grasp function to release
    UR10_utils.unactivate_hold_mode()
    UR10_utils.set_gripper_force(-8)
    rospy.sleep(1)
    UR10_utils.calibrate_ft_sensors()
    state.holding_object = False
    state.gripper_open = True

def go_to_object(data, state):
    if state.gripper_open and state.joints_reset:
        time_ = UR10_utils.get_pose_time(position=go_to_obj_pos, oriention=go_to_obj_rot)
        UR10_utils.go_to_pose(position=go_to_obj_pos, oriention=go_to_obj_rot, time_=time_)
        # Go to pose
        state.at_object_pick_up = True 
        state.at_hinge_pose = False

def est_contact(data, state):
    state.friction_estimated = False
    if not state.holding_object:
        grasp(data, state)
    if state.at_object_pick_up:
        # grasp with 5 newton
        UR10_utils.unactivate_hold_mode()
        UR10_utils.set_gripper_force(5)
        go_to_obj_pos_ = go_to_obj_pos.copy()
        go_to_obj_pos_[2] += friction_vertical_move

        topics_to_record = ["/gripper_closed_loop", "/gripper_control", 
                            "/gripper_monitor", "/netft_data_1_calibrated", 
                            "/netft_data_2_calibrated", "velocity_sensor_1",
                            "velocity_sensor_2", "/tf"]
        if len(data.recording_name)>=3:
            bag_process = UR10_utils.start_rosbag(topics_to_record, data.recording_name, bag_directory)
        
        UR10_utils.set_friction_mode(time_=1, mode_=1)
        rospy.sleep(0.05)
        UR10_utils.go_to_pose(position=go_to_obj_pos_, oriention=go_to_obj_rot, time_=0.8)
        rospy.sleep(0.2)
        UR10_utils.set_friction_mode(time_=1, mode_=2)
        UR10_utils.go_to_pose(position=go_to_obj_pos_, oriention=go_to_obj_rot_friction, time_=0.5)
        UR10_utils.go_to_pose(position=go_to_obj_pos_, oriention=go_to_obj_rot, time_=0.5)
        if len(data.recording_name)>=3:
            UR10_utils.stop_rosbag(bag_process)
        state.friction_estimated = True
        release(data, state)
        # get estimated paramters and print them 
        s1, s2 = UR10_utils.call_get_friction_param()
        print(" ")
        print("mu_c_1", s1[0], "mu_s_1", s1[1], "mu_v_1", s1[2], "r_1", s1[3])
        print("mu_c_2", s2[0], "mu_s_2", s2[1], "mu_v_2", s2[2], "r_2", s2[3])
        # go to object
        UR10_utils.go_to_pose(position=go_to_obj_pos, oriention=go_to_obj_rot, time_=1)


def linear_slippage(data, state):
    if state.holding_object and state.friction_estimated:
        topics_to_record = ["/gripper_closed_loop", "/gripper_control", 
                            "/gripper_monitor", "/netft_data_1_calibrated", 
                            "/netft_data_2_calibrated", "velocity_sensor_1",
                            "velocity_sensor_2", "linear_slippage_traj"]
        if len(data.recording_name)>=3:
            bag_process = UR10_utils.start_rosbag(topics_to_record, data.recording_name, bag_directory)
        UR10_utils.activate_linear_slippage(data.time, data.lin_dist)
        rospy.sleep(data.time+1)
        if len(data.recording_name)>=3:
            UR10_utils.stop_rosbag(bag_process)

def hold_mode(data, state):
    if state.holding_object and state.friction_estimated:
        topics_to_record = ["/gripper_closed_loop", "/gripper_control", 
                            "/gripper_monitor", "/netft_data_1_calibrated", 
                            "/netft_data_2_calibrated", "velocity_sensor_1",
                            "velocity_sensor_2"]
        if len(data.recording_name)>=3:
            bag_process = UR10_utils.start_rosbag(topics_to_record, data.recording_name, bag_directory)
        UR10_utils.activate_hold_mode()
        rospy.sleep(data.time)
        if len(data.recording_name)>=3:
            UR10_utils.stop_rosbag(bag_process)
       

def pick_up_object(data, state):
    if state.friction_estimated and state.joints_reset:
        # go to pose
        hold_obj_pos = go_to_obj_pos.copy()
        hold_obj_pos[2] += hold_height
        time_ = UR10_utils.get_pose_time(position=hold_obj_pos, oriention=go_to_obj_rot)
        UR10_utils.go_to_pose(position=hold_obj_pos, oriention=go_to_obj_rot, time_=time_)
        print("pose sent pick up", hold_obj_pos)
        # Go to pose
        state.at_object_pick_up = True 
        state.at_hinge_pose = False

        state.at_hinge_pose = False
        state.at_object_pick_up = False
    else:
        print(state.friction_estimated, state.joints_reset)

def go_to_hinge_pose(data, state):
    # go to hinge pose
    if state.joints_reset:
        hold_obj_pos = go_to_obj_pos.copy()
        hold_obj_pos[2] += hold_height
        time_ = UR10_utils.get_pose_time(position=hold_obj_pos, oriention=go_to_rot_hinge_start)
        UR10_utils.go_to_pose(position=hold_obj_pos, oriention=go_to_rot_hinge_start, time_=time_)
        state.at_hinge_pose = True
        state.at_object_pick_up = False

def hinge_mode(data, state):
    if state.holding_object and state.at_hinge_pose and state.friction_estimated:
        hold_obj_pos = go_to_obj_pos.copy()
        hold_obj_pos[2] += hold_height
        topics_to_record = ["/gripper_closed_loop", "/gripper_control", 
                            "/gripper_monitor", "/netft_data_1_calibrated", 
                            "/netft_data_2_calibrated", "velocity_sensor_1",
                            "velocity_sensor_2", "/tf"]
        if len(data.recording_name)>=3:
            bag_process = UR10_utils.start_rosbag(topics_to_record, data.recording_name, bag_directory)
        
        UR10_utils.activate_hinge_mode()
        time_ = UR10_utils.get_pose_time(position=hold_obj_pos, oriention=go_to_rot_hinge_end)
        UR10_utils.go_to_pose(position=hold_obj_pos, oriention=go_to_rot_hinge_end, time_=time_)
        rospy.sleep(2)
        # end bag recording
        if len(data.recording_name)>=3:
            UR10_utils.stop_rosbag(bag_process)
        UR10_utils.activate_hold_mode()
        # go to hinge pose
        

def rotational_slippage(data, state):
    if state.holding_object and state.friction_estimated:
        topics_to_record =  ["/gripper_closed_loop", "/gripper_control", 
                            "/gripper_monitor", "/netft_data_1_calibrated", 
                            "/netft_data_2_calibrated", "velocity_sensor_1",
                            "velocity_sensor_2", "/rotation_slippage_traj"]
        if len(data.recording_name)>=3:
            bag_process = UR10_utils.start_rosbag(topics_to_record, data.recording_name, bag_directory)
        UR10_utils.activate_rotational_slippage(data.time, data.rotation)
        rospy.sleep(data.time+1)
        if len(data.recording_name)>=3:
            UR10_utils.stop_rosbag(bag_process)


def demo(data, state):
    if not state.at_object_pick_up:
        release(data, state)
        go_to_object(data, state)
    go_to_obj_pos_ = go_to_obj_pos.copy()
    go_to_obj_pos_[2] += friction_vertical_move
    hold_obj_pos = go_to_obj_pos.copy()
    hold_obj_pos[2] += hold_height
    if not state.holding_object and state.joints_reset and state.at_object_pick_up:
        topics_to_record =  ["/gripper_closed_loop", "/gripper_control", 
                            "/gripper_monitor", "/netft_data_1_calibrated", 
                            "/netft_data_2_calibrated", "velocity_sensor_1",
                            "velocity_sensor_2", "/tf"]
        # start bag recording
        if len(data.recording_name)>=3:
            bag_process = UR10_utils.start_rosbag(topics_to_record, data.recording_name, bag_directory)
        # grasp with 5 newton
        state.friction_estimated = False
        grasp(data=data, state=state)
        UR10_utils.unactivate_hold_mode()
        rospy.sleep(0.1)
        UR10_utils.set_gripper_force(5)
        rospy.sleep(0.1)
        # call est friction function
        UR10_utils.set_friction_mode(time_=1, mode_=1)
        rospy.sleep(0.05)
        UR10_utils.go_to_pose(position=go_to_obj_pos_, oriention=go_to_obj_rot, time_=0.8)
        rospy.sleep(0.2)
        UR10_utils.set_friction_mode(time_=1, mode_=2)
        UR10_utils.go_to_pose(position=go_to_obj_pos_, oriention=go_to_obj_rot_friction, time_=0.5)
        UR10_utils.go_to_pose(position=go_to_obj_pos_, oriention=go_to_obj_rot, time_=0.5)
        state.friction_estimated = True

        # activate hold mode
        grasp(data=data, state=state)
        pick_up_object(data, state)
        # call linear slippage 
        UR10_utils.activate_linear_slippage(data.time, data.lin_dist)
        rospy.sleep(data.time+1)
        # call hinge mode
        UR10_utils.activate_hinge_mode()
        time_ = UR10_utils.get_pose_time(position=hold_obj_pos, oriention=go_to_rot_hinge_end)
        UR10_utils.go_to_pose(position=hold_obj_pos, oriention=go_to_rot_hinge_end, time_=time_)
        UR10_utils.activate_hold_mode()
        # activate hold mode
        pick_up_object(data, state)
        # call rotational slippage
        UR10_utils.activate_rotational_slippage(data.time, data.rotation)
        rospy.sleep(data.time+1)
        if len(data.recording_name)>=3:
            UR10_utils.stop_rosbag(bag_process)

def reset_joints(data, state):
    UR10_utils.reset_pose(reset_list, 10)
    state.joints_reset = True
    state.at_object_pick_up = False

cases = {"reset_joints": reset_joints,
         "grasp": grasp,
         "release": release,
         "go_to_object": go_to_object,
         "est_contact": est_contact,
         "pick_up_object": pick_up_object,
         "linear_slippage": linear_slippage,
         "hinge_mode": hinge_mode,
         "hold_mode": hold_mode, 
         "go_to_hinge_pose": go_to_hinge_pose,
         "rotational_slippage": rotational_slippage,
         "demo": demo}


def default_case(data, state):
    print("The mode: ", data.mode, " is not valid.")
    print("Valid modes are: ", cases.keys())


class Decider(object):
    def __init__(self) -> None:
        self.system_state = SystemState()
    
    def service_callback(self, data):

        mode = data.mode

        cases.get(mode, default_case)(data, self.system_state)
        return True

class Data_(object):
    def __init__(self) -> None:
        self.time = 2
        self.lin_dist = 0.05
        self.rotation = 30    
        self.recording_name = ""


system_state = SystemState()


def service_callback(data):
    cases.get(data.mode, default_case)(data, system_state)
    return True

if __name__ == "__main__":
    

    UR10_service = rospy.Service('UR10_gripper', UR10_gripper, service_callback)
    """
    data = Data_()
    cases.get("reset_joints", default_case)(data, system_state)
    cases.get("release", default_case)(data, system_state)
    cases.get("go_to_object", default_case)(data, system_state)
    cases.get("est_contact", default_case)(data, system_state)
    cases.get("grasp", default_case)( data, system_state)
    cases.get("pick_up_object", default_case)( data, system_state)
    cases.get("linear_slippage", default_case)( data, system_state)
    cases.get("go_to_hinge_pose", default_case)( data, system_state)
    cases.get("hinge_mode", default_case)( data, system_state)
    cases.get("pick_up_object", default_case)( data, system_state)
    cases.get("rotational_slippage", default_case)( data, system_state)
    cases.get("release", default_case)( data, system_state)
    cases.get("demo", default_case)( data, system_state)
    """
    rospy.spin()





































