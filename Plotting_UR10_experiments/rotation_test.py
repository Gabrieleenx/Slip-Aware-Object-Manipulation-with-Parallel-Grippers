# # instal bagpy
import rosbag
from scipy.spatial.transform import Rotation as R_
import seaborn as sns
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np 

from pathlib import Path
current_file_dir = Path(__file__).parent
parent_dir = str(current_file_dir.parent)

mpl.rcParams['font.family'] = 'Times New Roman'
mpl.rcParams['font.serif'] = ['Times New Roman'] + mpl.rcParams['font.serif']
mpl.rcParams["mathtext.fontset"] = 'cm'
mpl.rcParams['axes.xmargin'] = 0
mpl.rcParams['axes.formatter.limits'] = (-2, 3)
sns.set_theme("paper", "ticks", font_scale=1.0, rc={"lines.linewidth": 2})


def find_closest_within_tolerance(array, target, tolerance):
    # Compute the absolute difference between each element and the target
    differences = np.abs(array - target)
    
    # Find indices where the difference is within the tolerance
    within_tolerance_indices = np.where(differences <= tolerance)[0]
    
    if len(within_tolerance_indices) == 0:
        # If no indices are within the tolerance, return None or some flag
        return None
    
    # Find the index of the smallest difference within the tolerance
    closest_index = within_tolerance_indices[np.argmin(differences[within_tolerance_indices])]
    
    return closest_index

def avg_bags(data1, t1, data2, t2):
    """Synchronize messages from two bags based on their timestamps."""
    # Load messages from both bag

    synchronized_data = []
    synchronized_time = []

    tolerance = (t1[100]-t1[0])/100

    for i in range(len(data1)):
        idx = find_closest_within_tolerance(t2, t1[i], tolerance)
        if idx is not None:
            synchronized_data.append((data1[i] + data2[idx])/2)
            synchronized_time.append(t1[i])

    return np.array(synchronized_time), np.array(synchronized_data)


def read_rosbag_ft_data(bag_file):
    bag = rosbag.Bag(bag_file)
    timestamps_ft_1 = []
    data_ft_1_x = []
    data_ft_1_y = []
    r = R_.from_euler('XYZ', [0, 0, 47.5 * np.pi / 180])
    rot_ft_1 = r.as_matrix()
    for topic, msg, t in bag.read_messages(topics=["/netft_data_1_calibrated"]):
        # Assuming the message has a 'data' attribute and a header with a timestamp
        timestamps_ft_1.append(t.to_sec())
        fx = msg.wrench.force.x
        fy = msg.wrench.force.y
        ft_1 = np.array([[fx],
                         [fy],
                         [0]])
        ft_1_t = np.dot(rot_ft_1, ft_1)

        data_ft_1_x.append(ft_1_t[0,0])
        data_ft_1_y.append(ft_1_t[1,0])
    timestamps_ft_2 = []
    data_ft_2_x = []
    data_ft_2_y = []
    r = R_.from_euler('XYZ', [180, 0, -132.5 ], degrees=True)
    rot_ft_2 = r.as_matrix()
    for topic, msg, t in bag.read_messages(topics=["/netft_data_2_calibrated"]):
        # Assuming the message has a 'data' attribute and a header with a timestamp
        timestamps_ft_2.append(t.to_sec())
        fx = msg.wrench.force.x
        fy = msg.wrench.force.y
        ft_2 = np.array([[fx],
                         [fy],
                         [0]])
        ft_2_t = np.dot(rot_ft_2, ft_2)
        data_ft_2_x.append(ft_2_t[0,0])
        data_ft_2_y.append(ft_2_t[1,0])
    bag.close()


    time_, data_x = avg_bags(np.array(data_ft_1_x),
                            np.array(timestamps_ft_1), 
                            np.array(data_ft_2_x),
                            np.array(timestamps_ft_2))
    
    time_, data_y = avg_bags(np.array(data_ft_1_y),
                            np.array(timestamps_ft_1), 
                            np.array(data_ft_2_y),
                            np.array(timestamps_ft_2))
    
    data_ = np.linalg.norm([data_x, data_y], axis=0)
     
    return time_, 2*data_

def read_rosbag_fn_data(bag_file):
    bag = rosbag.Bag(bag_file)
    timestamps_fn_1 = []
    data_fn_1 = []
    for topic, msg, t in bag.read_messages(topics=["/netft_data_1_calibrated"]):
        # Assuming the message has a 'data' attribute and a header with a timestamp
        timestamps_fn_1.append(t.to_sec())
        fn = msg.wrench.force.z
        data_fn_1.append(fn)

    timestamps_fn_2 = []
    data_fn_2 = []
    for topic, msg, t in bag.read_messages(topics=["/netft_data_2_calibrated"]):
        # Assuming the message has a 'data' attribute and a header with a timestamp
        timestamps_fn_2.append(t.to_sec())
        fn = msg.wrench.force.z
        data_fn_2.append(fn)
    
    bag.close()

    time_, data_ = avg_bags(np.array(data_fn_1),
                            np.array(timestamps_fn_1), 
                            np.array(data_fn_2),
                            np.array(timestamps_fn_2))
    
    return time_, data_


def read_rosbag_fd_data(bag_file):
    bag = rosbag.Bag(bag_file)
    timestamps_fn_1 = []
    data_fn_1 = []
    for topic, msg, t in bag.read_messages(topics=["/gripper_closed_loop"]):
        # Assuming the message has a 'data' attribute and a header with a timestamp
        timestamps_fn_1.append(t.to_sec())
        fn = msg.target
        data_fn_1.append(fn)
    bag.close()
    
    return np.array(timestamps_fn_1), np.array(data_fn_1)


def read_rosbag_tau_data(bag_file):
    bag = rosbag.Bag(bag_file)
    timestamps_tau_1 = []
    data_tau_1 = []
    for topic, msg, t in bag.read_messages(topics=["/netft_data_1_calibrated"]):
        # Assuming the message has a 'data' attribute and a header with a timestamp
        timestamps_tau_1.append(t.to_sec())
        tau = msg.wrench.torque.z
        data_tau_1.append(tau)

    timestamps_tau_2 = []
    data_tau_2 = []
    for topic, msg, t in bag.read_messages(topics=["/netft_data_2_calibrated"]):
        # Assuming the message has a 'data' attribute and a header with a timestamp
        timestamps_tau_2.append(t.to_sec())
        tau = msg.wrench.torque.z
        data_tau_2.append(tau)
    bag.close()
    time_, data_ = avg_bags(np.array(data_tau_1),
                            np.array(timestamps_tau_1), 
                            -np.array(data_tau_2),
                            np.array(timestamps_tau_2))
    
    return time_, 2*data_

def read_rosbag_theta_data(bag_file):
    bag = rosbag.Bag(bag_file)
    timestamps_tau_1 = []
    data_tau_1 = []
    for topic, msg, t in bag.read_messages(topics=["velocity_sensor_1"]):
        # Assuming the message has a 'data' attribute and a header with a timestamp
        timestamps_tau_1.append(t.to_sec())
        data_tau_1.append(msg.omega)

    timestamps_tau_2 = []
    data_tau_2 = []
    for topic, msg, t in bag.read_messages(topics=["velocity_sensor_2"]):
        # Assuming the message has a 'data' attribute and a header with a timestamp
        timestamps_tau_2.append(t.to_sec())
        data_tau_2.append(msg.omega)

    bag.close()
    time_, data_ = avg_bags(np.array(data_tau_1),
                            np.array(timestamps_tau_1), 
                            -np.array(data_tau_2),
                            np.array(timestamps_tau_2))
    dist_ = []
    d = 0
    for i in range(len(data_)):
        if i == 0:
            d += data_[i] * (time_[1] - time_[0])
        else:
            d += data_[i] * (time_[i] - time_[i-1])
        dist_.append(d)
    return time_, np.abs(np.array(dist_)*180/np.pi)



def read_rosbag_lindist_data(bag_file):
    bag = rosbag.Bag(bag_file)
    timestamps_1 = []
    data_x_1 = []
    data_y_1 = []
    r = R_.from_euler('XYZ', [0, 0, 47.5 * np.pi / 180])
    rot_ft_1 = r.as_matrix()
    for topic, msg, t in bag.read_messages(topics=["velocity_sensor_1"]):
        # Assuming the message has a 'data' attribute and a header with a timestamp
        timestamps_1.append(t.to_sec())
        vx = msg.vx
        vy = msg.vy
        v_1 = np.array([[vx],
                        [vy],
                        [0]])
        vel_1 = np.dot(rot_ft_1, v_1)

        data_x_1.append(vel_1[0,0])
        data_y_1.append(vel_1[1,0])

    timestamps_2 = []
    data_x_2 = []
    data_y_2 = []
    r2 = R_.from_euler('XYZ', [180, 0, -132.5], degrees=True)
    rot_ft_2 = r2.as_matrix()
    for topic, msg, t in bag.read_messages(topics=["velocity_sensor_2"]):
        # Assuming the message has a 'data' attribute and a header with a timestamp
        timestamps_2.append(t.to_sec())
        vx = msg.vx
        vy = msg.vy
        v_2 = np.array([[vx],
                        [vy],
                        [0]])
        vel_2 = np.dot(rot_ft_2, v_2)

        data_x_2.append(vel_2[0,0])
        data_y_2.append(vel_2[1,0])
    bag.close()
    time_, data_x = avg_bags(np.array(data_x_1),
                            np.array(timestamps_1), 
                            np.array(data_x_2),
                            np.array(timestamps_2))
    
    time_, data_y = avg_bags(np.array(data_y_1),
                            np.array(timestamps_1), 
                            np.array(data_y_2),
                            np.array(timestamps_2))
    data_ = np.linalg.norm([data_x, data_y], axis=0)

    dist_ = []
    d = 0
    for i in range(len(data_)):
        if i == 0:
            d += data_[i] * (time_[1] - time_[0])
        else:
            d += data_[i] * (time_[i] - time_[i-1])
        dist_.append(d)
    return time_, 1000*np.array(dist_)


def read_rosbag_trajectory_data(bag_file):
    bag = rosbag.Bag(bag_file)
    timestamps = []
    data = []
    for topic, msg, t in bag.read_messages(topics=["/rotation_slippage_traj"]):
        # Assuming the message has a 'data' attribute and a header with a timestamp
        timestamps.append(t.to_sec())
        data.append(msg.pos)
    bag.close()
    
    return np.array(timestamps), 180/np.pi*np.array(data)


def read_rosbag_control_pos_data(bag_file):
    bag = rosbag.Bag(bag_file)
    timestamps = []
    data = []
    for topic, msg, t in bag.read_messages(topics=["linear_slippage_traj"]):
        # Assuming the message has a 'data' attribute and a header with a timestamp
        timestamps.append(t.to_sec())
        data.append(msg.pos_measured)
    bag.close()
    return np.array(timestamps), 1000*np.array(data)


if __name__ == "__main__":
    path_plastic = parent_dir + "/UR_experiments/bags/plastic_rotation_t5_15_1.bag"
    path_cardboard = parent_dir + "/UR_experiments/bags/cardboard_rotation_t5_15_1.bag"
    path_case = parent_dir + "/UR_experiments/bags/case_rotation_t5_15_1.bag"

    path_plastic2 = parent_dir + "/UR_experiments/bags/plastic_rotation_t2_60_1.bag"
    path_cardboard2 = parent_dir + "/UR_experiments/bags/cardboard_rotation_t2_60_2.bag"
    path_case2 = parent_dir + "/UR_experiments/bags/case_rotation_t2_60_1.bag"

    timestamps_traj, data_traj = read_rosbag_trajectory_data(path_plastic)
    time0 = timestamps_traj[0]-1
    timestamps_traj -= time0
    timestamps, data = read_rosbag_theta_data(path_plastic)
    timestamps -= time0


    timestamps_con, data_con = read_rosbag_control_pos_data(path_plastic)
    timestamps_con -= time0

    timestamps_traj_, data_traj_ = read_rosbag_trajectory_data(path_case)
    time0 = timestamps_traj_[0]-1

    timestamps_case, data_case = read_rosbag_theta_data(path_case)
    timestamps_case -= time0


    timestamps_traj_, data_traj_ = read_rosbag_trajectory_data(path_cardboard)
    time0 = timestamps_traj_[0]-1
    timestamps_cardboard, data_cardboard = read_rosbag_theta_data(path_cardboard)
    timestamps_cardboard -= time0
    timestamps_traj_-= time0
    f, (ax1, ax2) = plt.subplots(2, 1, figsize=(6,4))
    
    ax1.plot(timestamps, data, alpha=0.7, linewidth=1.5, label="Plastic")
    ax1.plot(timestamps_case, data_case, alpha=0.7, linewidth=1.5, label="Case")
    ax1.plot(timestamps_cardboard, data_cardboard, alpha=0.7, linewidth=1.5, label="Cardboard")
    ax1.plot(timestamps_traj, data_traj, alpha=1.0, linewidth=2, label="Trajectory")
    ax1.get_yaxis().set_label_coords(-0.07,0.2)
    ax1.set_ylabel('$|\hat{\\theta}|$ [deg]')
    #ax1.set_xlabel('Time [s]')
    ax1.legend()

    timestamps_traj, data_traj = read_rosbag_trajectory_data(path_plastic2)
    time0 = timestamps_traj[0]-1
    timestamps, data = read_rosbag_theta_data(path_plastic2)
    timestamps -= time0
    timestamps_traj -= time0


    timestamps_traj_, data_traj_ = read_rosbag_trajectory_data(path_case2)
    time0 = timestamps_traj_[0]-1
    timestamps_case, data_case = read_rosbag_theta_data(path_case2)
    timestamps_case -= time0
    
    timestamps_traj_, data_traj_ = read_rosbag_trajectory_data(path_cardboard2)
    time0 = timestamps_traj_[0]-1
    timestamps_cardboard, data_cardboard = read_rosbag_theta_data(path_cardboard2)
    timestamps_cardboard -=  time0

 
    ax2.plot(timestamps, data, alpha=0.7, linewidth=1.5, label="Plastic")
    ax2.plot(timestamps_case, data_case, alpha=0.7, linewidth=1.5, label="Case")
    ax2.plot(timestamps_cardboard, data_cardboard, alpha=0.7, linewidth=1.5, label="Cardboard")
    ax2.plot(timestamps_traj, data_traj, alpha=1.0, linewidth=2, label="Trajectory")
    ax2.get_yaxis().set_label_coords(-0.07,0.2)
    ax2.set_ylabel('$|\hat{\\theta}|$ [deg]')
    ax2.set_xlabel('Time [s]')
    ax2.legend()


    plt.tight_layout()
    plt.show()

    
    