import rosbag

import seaborn as sns
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np 
from scipy.signal import correlate, resample

from pathlib import Path
current_file_dir = Path(__file__).parent
parent_dir = str(current_file_dir.parent)

mpl.rcParams['font.family'] = 'Times New Roman'
mpl.rcParams['font.serif'] = ['Times New Roman'] + mpl.rcParams['font.serif']
mpl.rcParams["mathtext.fontset"] = 'cm'
mpl.rcParams['axes.xmargin'] = 0
mpl.rcParams['axes.formatter.limits'] = (-2, 3)
sns.set_theme("paper", "ticks", font_scale=1.0, rc={"lines.linewidth": 2})

t_start = 0
t_end = 14
fn_start = 5
fn_end = 25
t_fn_start = 8
t_fn_end = 9
plot_index = 0
topic = '/netft_data_1_calibrated'  
topic_fd = '/gripper_closed_loop'  
path_to_map = parent_dir + "/plotting_gripper/gripper_control_bags/"
bag_disered_force = path_to_map + "wood_1.bag"

path_list_sponge = [path_to_map + "sponge_1.bag",
                    path_to_map + "sponge_2.bag",
                    path_to_map + "sponge_3.bag",
                    path_to_map + "sponge_4.bag",
                    path_to_map + "sponge_5.bag",
                    path_to_map + "sponge_6.bag",
                    path_to_map + "sponge_7.bag",
                    path_to_map + "sponge_8.bag",
                    path_to_map + "sponge_9.bag",
                    path_to_map + "sponge_10.bag"]

path_list_cardboard_box = [path_to_map + "cardboard_1.bag",
                           path_to_map + "cardboard_2.bag",
                           path_to_map + "cardboard_3.bag",
                           path_to_map + "cardboard_4.bag",
                           path_to_map + "cardboard_5.bag",
                           path_to_map + "cardboard_6.bag",
                           path_to_map + "cardboard_7.bag",
                           path_to_map + "cardboard_8.bag",
                           path_to_map + "cardboard_9.bag",
                           path_to_map + "cardboard_10.bag"]

path_list_plastic = [path_to_map + "plastic_1.bag",
                  path_to_map + "plastic_2.bag",
                  path_to_map + "plastic_3.bag",
                  path_to_map + "plastic_4.bag",
                  path_to_map + "plastic_5.bag",
                  path_to_map + "plastic_6.bag",
                  path_to_map + "plastic_7.bag",
                  path_to_map + "plastic_8.bag",
                  path_to_map + "plastic_9.bag",
                  path_to_map + "plastic_10.bag"]

path_list_case = [path_to_map + "case_1.bag",
                  path_to_map + "case_2.bag",
                  path_to_map + "case_3.bag",
                  path_to_map + "case_4.bag",
                  path_to_map + "case_5.bag",
                  path_to_map + "case_6.bag",
                  path_to_map + "case_7.bag",
                  path_to_map + "case_8.bag",
                  path_to_map + "case_9.bag",
                  path_to_map + "case_10.bag"]

path_list_wood = [path_to_map + "wood_1.bag",
                    path_to_map + "wood_2.bag",
                    path_to_map + "wood_3.bag",
                    path_to_map + "wood_4.bag",
                    path_to_map + "wood_5.bag",
                    path_to_map + "wood_6.bag",
                    path_to_map + "wood_7.bag",
                    path_to_map + "wood_8.bag",
                    path_to_map + "wood_9.bag",
                    path_to_map + "wood_10.bag"]

objects = {'Sponge':path_list_sponge,
           'Carboard':path_list_cardboard_box,
           'Case':path_list_case,
           'Plastic':path_list_plastic,
           'Wood':path_list_wood}

def read_rosbag_Fd_data(bag_file, topic):
    bag = rosbag.Bag(bag_file)
    timestamps = []
    data = []
    
    for topic, msg, t in bag.read_messages(topics=[topic]):
        # Assuming the message has a 'data' attribute and a header with a timestamp
        timestamps.append(t.to_sec())
        data.append(msg.target)
    
    bag.close()
    return np.array(timestamps), np.array(data)

def read_rosbag_Ft_data(bag_file, topic):
    bag = rosbag.Bag(bag_file)
    timestamps = []
    data = []
    
    for topic, msg, t in bag.read_messages(topics=[topic]):
        # Assuming the message has a 'data' attribute and a header with a timestamp
        timestamps.append(t.to_sec())
        data.append(-msg.wrench.force.z)
    
    bag.close()
    return np.array(timestamps), np.array(data)

def get_data_time(timestamps, data, start_time, end_time):
    absolute_differences = np.abs(timestamps - start_time)
    index_of_start = np.argmin(absolute_differences)

    absolute_differences = np.abs(timestamps - end_time)
    index_of_end = np.argmin(absolute_differences)

    return timestamps[index_of_start:index_of_end], data[index_of_start:index_of_end]

def find_time_start(timestapms, data):
    indices = np.where(data == 10)[0][0]
    return timestamps[indices] - 2

def find_rise_time(timestams, data):
    absolute_differences = np.abs(timestamps - t_fn_start)
    index_of_start = np.argmin(absolute_differences)

    absolute_differences = np.abs(timestamps - t_fn_end)
    index_of_end = np.argmin(absolute_differences)
    delta = fn_end - fn_start
    fn_low = fn_start + delta*0.1
    fn_high = fn_end - delta*0.1
    t_low = 0
    t_high = 0
    t_delay = 0
    for i in range(index_of_start, index_of_end):
        if data[i] > fn_low:
            t_low = timestamps[i]
            break

    for i in range(index_of_start, index_of_end):
        if data[i] > fn_start + delta/2:
            t_delay = timestamps[i]
            break

    for i in range(index_of_start, index_of_end):
        if data[i] > fn_high:
            t_high = timestamps[i]
            break
    return t_high - t_low, t_delay - t_fn_start

def calculate_settling_time(time, response, percentage=2):
    # Final steady-state value
    final_value = fn_end
    
    # Define the tolerance based on the specified percentage
    tolerance = (final_value- fn_start) * (percentage / 100)
    
    # Find the index where the response first stays within the tolerance range
    within_tolerance = np.abs(response - final_value) <= tolerance

    min_points = 200
    # Find the first segment where the response is within tolerance for the specified duration
    for i in range(len(within_tolerance) - min_points + 1):
        if np.all(within_tolerance[i:i + min_points]):
            settling_time = time[i]
            return settling_time - t_fn_start
    
    return None
    
    return settling_time


if __name__ == "__main__":
    f, ax = plt.subplots(1, 1, figsize=(6,3))

    timestamps, data = read_rosbag_Fd_data(bag_disered_force, topic_fd)
    time_0 = find_time_start(timestamps, data)
    print(time_0)
    timestamps += -time_0

    timestamps, data = get_data_time(timestamps, data, t_start, t_end)

    ax.plot(timestamps, data, alpha=0.9, linewidth=1.5, label="$f_d$")

    for key, value in objects.items():
        rise_time_list = []
        delay_time_list = []
        overshoot_list = []
        settle_time_list = []
        for i in range(len(value)):
            timestamps, data = read_rosbag_Fd_data(value[i], topic_fd)
            time_0 = find_time_start(timestamps, data)
            timestamps, data = read_rosbag_Ft_data(value[i], topic)
            timestamps += -time_0
            timestamps, data = get_data_time(timestamps, data, t_start, t_end)
            if i == plot_index:
                ax.plot(timestamps, data, alpha=0.7, linewidth=1.5, label=key)
            # get risetime, overshoot and settle time
            t, t_delay = find_rise_time(timestamps, data)
            t_settle = calculate_settling_time(timestamps, data)
            rise_time_list.append(t)
            delay_time_list.append(t_delay)
            overshoot_list.append(np.max(data)- fn_end)
            settle_time_list. append(t_settle)
        # get statistical risetime, overshoot and settle time and print 
        rise_mean = np.mean(rise_time_list)
        rise_std_dev = np.std(rise_time_list)
        t50_mean = np.mean(delay_time_list)
        t_50_std_dev = np.std(delay_time_list)
        Mp_mean = np.mean(overshoot_list)
        Mp_std_dev = np.std(overshoot_list)
        ts_mean = np.mean(settle_time_list)
        ts_std_dev = np.std(settle_time_list)
        print(key, "t_r mean", rise_mean, "SD", rise_std_dev, "t50", t50_mean, "SD", t_50_std_dev,
              "Mp", Mp_mean, "SD", Mp_std_dev, "ts", ts_mean, "SD", ts_std_dev)
    ax.set_xlabel('Time [s]', fontsize="12")
    ax.set_ylabel('$F_n$ [N]', fontsize="12")
    # Show the plot
    plt.legend()

    plt.tight_layout()
    plt.show()
            
            
 


    
















