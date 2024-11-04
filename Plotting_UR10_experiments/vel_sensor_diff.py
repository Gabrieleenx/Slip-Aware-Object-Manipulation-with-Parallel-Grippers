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


def read_rosbag_theta_1_data(bag_file):
    bag = rosbag.Bag(bag_file)
    timestamps_tau_1 = []
    data_tau_1 = []
    for topic, msg, t in bag.read_messages(topics=["velocity_sensor_1"]):
        # Assuming the message has a 'data' attribute and a header with a timestamp
        timestamps_tau_1.append(t.to_sec())
        data_tau_1.append(msg.omega)
    dist_ = []
    d = 0
    for i in range(len(data_tau_1)):
        if i == 0:
            d += data_tau_1[i] * (timestamps_tau_1[1] - timestamps_tau_1[0])
        else:
            d += data_tau_1[i] * (timestamps_tau_1[i] - timestamps_tau_1[i-1])
        dist_.append(d)
    return np.array(timestamps_tau_1), np.abs(np.array(dist_)*180/np.pi)


def read_rosbag_theta_2_data(bag_file):
    bag = rosbag.Bag(bag_file)
    timestamps_tau_1 = []
    data_tau_1 = []
    for topic, msg, t in bag.read_messages(topics=["velocity_sensor_2"]):
        # Assuming the message has a 'data' attribute and a header with a timestamp
        timestamps_tau_1.append(t.to_sec())
        data_tau_1.append(msg.omega)
    dist_ = []
    d = 0
    for i in range(len(data_tau_1)):
        if i == 0:
            d += data_tau_1[i] * (timestamps_tau_1[1] - timestamps_tau_1[0])
        else:
            d += data_tau_1[i] * (timestamps_tau_1[i] - timestamps_tau_1[i-1])
        dist_.append(d)
    return np.array(timestamps_tau_1), np.abs(np.array(dist_)*180/np.pi)



if __name__ == "__main__":
    path_ =  parent_dir + "/UR_experiments/bags/plastic_rotation_t2_60_5.bag"
    timestamps, data = read_rosbag_theta_1_data(path_)
    time0 = timestamps[0]
    timestamps -= time0
    timestamps2, data2 = read_rosbag_theta_2_data(path_)
    timestamps2 -= time0


    f, (ax1) = plt.subplots(1, 1, figsize=(6,4))
    
    ax1.plot(timestamps, data, alpha=0.7, linewidth=1.5, label="1")
    ax1.plot(timestamps2, data2, alpha=0.7, linewidth=1.5, label="2")
    ax1.legend()
    plt.tight_layout()
    plt.show()
