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



if __name__ == "__main__":
    path_base = parent_dir + "/UR_experiments/bags/plastic_rotation_t2_60_"
    path_list = []
    for i in range(10):
        #if i == 0:
        #    continue
        path_ = path_base + str(i+1) + ".bag"
        timestamps, data = read_rosbag_theta_data(path_)
        print(data[-1])
