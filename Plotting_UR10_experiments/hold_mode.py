# # instal bagpy
import rosbag
from scipy.spatial.transform import Rotation as R_
import seaborn as sns
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np 
from pathlib import Path

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




def read_rosbag_vel_data(bag_file):
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
    r = R_.from_euler('XYZ', [180, 0, -132.5 ], degrees=True)
    rot_ft_2 = r.as_matrix()
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
    for i in range(len(data_x)):
        if i == 0:
            d += data_x[i] * (timestamps_1[1] - timestamps_1[0])
        else:
            d += data_x[i] * (timestamps_1[i] - timestamps_1[i-1])
        dist_.append(d)
    return time_, np.array(dist_)


if __name__ == "__main__":
    current_file_dir = Path(__file__).parent
    parent_dir = str(current_file_dir.parent)
    path_ = parent_dir+"/UR_experiments/bags/plastic_hold_mode_2.bag"
    end_time = 55

    timestamps, data = read_rosbag_ft_data(path_)
    time0 = timestamps[0]
    timestamps -= time0
    end_idx_ft = find_closest_within_tolerance(timestamps, end_time, 0.1)

    timestamps_fn, data_fn = read_rosbag_fn_data(path_)
    timestamps_fn -= time0
    end_idx_fn = find_closest_within_tolerance(timestamps_fn, end_time, 0.1)


    timestamps_tau, data_tau = read_rosbag_tau_data(path_)
    timestamps_tau -= time0
    end_idx_tau = find_closest_within_tolerance(timestamps_tau, end_time, 0.1)

    #timestamps_fn_d, data_fn_d = read_rosbag_fd_data(path_)
    #timestamps_fn_d -= time0

    timestamps_vel, data_vel = read_rosbag_vel_data(path_)
    timestamps_vel -= time0
    end_idx_vel = find_closest_within_tolerance(timestamps_vel, end_time, 0.1)

    f, (ax1,ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(6,5))
    
    ax1.plot(timestamps_fn[0:end_idx_fn], -data_fn[0:end_idx_fn], alpha=0.9, linewidth=1.5)
    ax1.get_yaxis().set_label_coords(-0.07,0.2)
    ax1.set_ylabel('$f_n$ [N]')
    ax1.xaxis.set_ticklabels([])
    #ax1.get_xaxis().set_visible(False)
    ax1.grid(True)

    ax2.plot(timestamps[0:end_idx_ft], data[0:end_idx_ft], alpha=0.9, linewidth=1.5)
    ax2.get_yaxis().set_label_coords(-0.07,0.2)
    ax2.set_ylabel('$f_t$ [N]')
    ax2.grid(True)
    ax2.xaxis.set_ticklabels([])

    ax3.plot(timestamps_tau[0:end_idx_tau], data_tau[0:end_idx_tau], alpha=0.9, linewidth=1.5)
    ax3.get_yaxis().set_label_coords(-0.07,0.2)
    ax3.set_ylabel('$\\tau$ [Nm]')
    ax3.grid(True, which='both', axis='both')
    ax3.xaxis.set_ticklabels([])

    ax4.plot(timestamps_vel[0:end_idx_vel], 1000*data_vel[0:end_idx_vel], alpha=0.9, linewidth=1.5)
    ax4.get_yaxis().set_label_coords(-0.07,0.2)
    ax4.set_ylabel('$x$ [mm]')
    ax4.set_xlabel('Time [s]')
    ax4.grid(True)

    ax1.text(6, 12, '1', fontsize=12, color='black')
    ax1.text(11, 17, '2', fontsize=12, color='black')
    ax1.text(16, 23, '3', fontsize=12, color='black')
    ax1.text(24, 30, '4', fontsize=12, color='black')
    ax1.text(28, 30, '5', fontsize=12, color='black')
    ax1.text(32, 30, '6', fontsize=12, color='black')
    ax1.text(38, 30, '7', fontsize=12, color='black')
    ax1.text(42, 40, '8', fontsize=12, color='black')
    ax1.text(47, 30, '9', fontsize=12, color='black')
    ax1.text(48.2, 40, '10', fontsize=12, color='black')

    plt.tight_layout()
    plt.show()

    
    