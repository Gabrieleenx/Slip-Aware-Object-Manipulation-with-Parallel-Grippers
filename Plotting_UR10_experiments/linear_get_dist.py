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
    for topic, msg, t in bag.read_messages(topics=["linear_slippage_traj"]):
        # Assuming the message has a 'data' attribute and a header with a timestamp
        timestamps.append(t.to_sec())
        data.append(msg.pos_measured)
    bag.close()
    return np.array(timestamps), 1000*np.array(data)


if __name__ == "__main__":
    path_base = parent_dir + "/UR_experiments/bags/wood_linear_t5_4cm_"
    path_list = []
    for i in range(10):
        #if i == 8:
        #    continue
        path_ = path_base + str(i+1) + ".bag"
        timestamps, data = read_rosbag_lindist_data(path_)
        timestamps_, data_ = read_rosbag_trajectory_data(path_)
        print(data[-1])
