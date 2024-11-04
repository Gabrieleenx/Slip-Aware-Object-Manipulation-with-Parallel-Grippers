import sys
import numpy as np
import rospy
import tf
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
from controller_manager_msgs.srv import ListControllers, ListControllersRequest
from std_srvs.srv import Trigger
import geometry_msgs.msg as geometry_msgs
import os
import sys
import subprocess


from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)

from gripper.msg import Gripper_target
from gripper.srv import SlippMode, FrictionParam, FrictionMode

import tf2_ros
import tf2_geometry_msgs


tf_buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tf_buffer)

# If your robot description is created with a tf_prefix, those would have to be adapted
JOINT_NAMES = [
    "elbow_joint",
    "shoulder_lift_joint",
    "shoulder_pan_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# All of those controllers can be used to execute joint-based trajectories.
# The scaled versions should be preferred over the non-scaled versions.
JOINT_TRAJECTORY_CONTROLLERS = [
    "scaled_pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "pos_joint_traj_controller",
    "vel_joint_traj_controller",
    "forward_joint_traj_controller",
]

# All of those controllers can be used to execute Cartesian trajectories.
# The scaled versions should be preferred over the non-scaled versions.
CARTESIAN_TRAJECTORY_CONTROLLERS = [
    "pose_based_cartesian_traj_controller",
    "joint_based_cartesian_traj_controller",
    "forward_cartesian_traj_controller",
]

cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[0]
joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[0]
# We'll have to make sure that none of these controllers are running, as they will
# be conflicting with the joint trajectory controllers
CONFLICTING_CONTROLLERS = ["joint_group_vel_controller", "twist_controller"]

pub_target_force = rospy.Publisher('/gripper_closed_loop', Gripper_target, queue_size=1, tcp_nodelay=True)  
get_friction_param = rospy.ServiceProxy('get_friction_params', FrictionParam)
set_slip_mode = rospy.ServiceProxy('slippage_control', SlippMode)
load_srv = rospy.ServiceProxy("controller_manager/load_controller", LoadController)
list_srv = rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)
switch_srv = rospy.ServiceProxy("controller_manager/switch_controller", SwitchController)
set_friction_mode_ = rospy.ServiceProxy('friction_estimation_mode', FrictionMode)
calibrate_ft = rospy.ServiceProxy('/gripper_callibrate_ft', Trigger)


def set_friction_mode(time_, mode_):
    # 1 linear, 2 rotation
    try:    
        resp = set_friction_mode_(time=time_, mode=mode_)
    except:
        print("Friction estimation not active")

def set_gripper_force(force):
    msg = Gripper_target()
    msg.header.stamp = rospy.Time.now()
    msg.target = force
    pub_target_force.publish(msg) 


def activate_hold_mode():
    try:
        resp = set_slip_mode(t=0,angle=0,dist=0,mode=4)
    except:
        print("slip service not availabe")

def activate_hinge_mode():
    try:
        resp = set_slip_mode(t=0,angle=0,dist=0,mode=3)
    except:
        print("slip service not availabe")

def calibrate_ft_sensors():
    try:
        resp = calibrate_ft()
    except:
        print("ft service not availabe")

def activate_linear_slippage(time_, dist_):
    try:
        resp = set_slip_mode(t=time_,angle=0,dist=dist_,mode=2)
    except:
        print("slip service not availabe")


def activate_rotational_slippage(time_, angle_):
    try:
        resp = set_slip_mode(t=time_,angle=angle_,dist=0,mode=1)
    except:
        print("slip service not availabe")

def unactivate_hold_mode():
    try:
        resp = set_slip_mode(t=0,angle=0,dist=0,mode=0)
    except:
        print("slip service not availabe")

def switch_controller(target_controller):
    """Activates the desired controller and stops all others from the predefined list above"""
    other_controllers = (
        JOINT_TRAJECTORY_CONTROLLERS
        + CARTESIAN_TRAJECTORY_CONTROLLERS
        + CONFLICTING_CONTROLLERS
    )

    other_controllers.remove(target_controller)

    srv = ListControllersRequest()
    response = list_srv(srv)
    for controller in response.controller:
        if controller.name == target_controller and controller.state == "running":
            return

    srv = LoadControllerRequest()
    srv.name = target_controller
    load_srv(srv)

    srv = SwitchControllerRequest()
    srv.stop_controllers = other_controllers
    srv.start_controllers = [target_controller]
    srv.strictness = SwitchControllerRequest.BEST_EFFORT
    switch_srv(srv)


def go_to_pose(position, oriention, time_):
    switch_controller(cartesian_trajectory_controller)

    trajectory_client = actionlib.SimpleActionClient(
        "{}/follow_cartesian_trajectory".format(cartesian_trajectory_controller),
        FollowCartesianTrajectoryAction,
    )

    # Wait for action server to be ready
    timeout = rospy.Duration(5)
    if not trajectory_client.wait_for_server(timeout):
        rospy.logerr("Could not reach controller action server.")
        sys.exit(-1)
    goal = FollowCartesianTrajectoryGoal()
    pose_list = [
        geometry_msgs.Pose(
            geometry_msgs.Vector3(position[0], position[1], position[2]), 
            geometry_msgs.Quaternion(oriention[0], oriention[1], oriention[2], oriention[3])
        ),
    ]
    duration_list = [time_]
    for i, pose in enumerate(pose_list):
        point = CartesianTrajectoryPoint()
        point.pose = pose
        point.time_from_start = rospy.Duration(duration_list[i])
        goal.trajectory.points.append(point)

    trajectory_client.send_goal(goal)
    trajectory_client.wait_for_result()
    result = trajectory_client.get_result()
    rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))




def get_pose_time(position, oriention, speed=1):

    try:
        trans = tf_buffer.lookup_transform('base', 'tool0_controller', rospy.Time(0))
        pos = trans.transform.translation
        rot = trans.transform.rotation
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        print(e)
    pos = np.array([pos.x, pos.y, pos.z])
    rot = np.array([rot.x, rot.y, rot.z, rot.w])

    delta_pos = np.linalg.norm(position - pos)
    dot_product = np.dot(oriention, rot)
    dot_product = max(min(dot_product, 1.0), -1.0)
    angle_diff = 2 * np.arccos(abs(dot_product))
    combined_distance = 5*delta_pos + angle_diff
    time_ = combined_distance*4/speed
    time_ = np.max([time_, 1])
    return time_


def reset_pose(joint_pos_list, time_):
    """Creates a trajectory and sends it using the selected action server"""

    # make sure the correct controller is loaded and activated
    switch_controller(joint_trajectory_controller)
    trajectory_client = actionlib.SimpleActionClient(
        "{}/follow_joint_trajectory".format(joint_trajectory_controller),
        FollowJointTrajectoryAction,
    )

    # Wait for action server to be ready
    timeout = rospy.Duration(5)
    if not trajectory_client.wait_for_server(timeout):
        rospy.logerr("Could not reach controller action server.")
        sys.exit(-1)

    # Create and fill trajectory goal
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = JOINT_NAMES
    
    position_list = [joint_pos_list]
    duration_list = [time_]
    velocity_list = [[0, 0, 0, 0, 0, 0]]
    for i, position in enumerate(position_list):
        point = JointTrajectoryPoint()
        point.positions = position
        point.velocities = velocity_list[i]
        point.time_from_start = rospy.Duration(duration_list[i])
        goal.trajectory.points.append(point)

    rospy.loginfo("Executing trajectory using the {}".format(joint_trajectory_controller))

    trajectory_client.send_goal(goal)
    trajectory_client.wait_for_result()

    result = trajectory_client.get_result()
    rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))




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
    rospy.sleep(2)
    return process

def stop_rosbag(process):
    """
    Stop the rosbag recording process.
    
    :param process: The subprocess.Popen object for the rosbag recording process.
    """
    process.terminate()
    process.wait()


def call_get_friction_param():
        # service call to get friction and contace parameter. 
        try:
            resp = get_friction_param(True)
        except:
            print("Friction service not active")
            return [0, 0, 0,0], [0,0,0,0]
        mu_c_1 = resp.mu_c_1
        mu_s_1 = resp.mu_s_1
        mu_v_1 = resp.mu_v_1
        r_1 = resp.r_1

        mu_c_2 = resp.mu_c_2
        mu_s_2 = resp.mu_s_2
        mu_v_2 = resp.mu_v_2
        r_2 = resp.r_2
        return [mu_c_1, mu_s_1, mu_v_1, r_1], [mu_c_2, mu_s_2, mu_v_2, r_2]