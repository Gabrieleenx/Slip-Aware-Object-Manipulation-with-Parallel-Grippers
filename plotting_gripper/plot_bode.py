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

frequency = [1, 2,4,8,16,32,64]
sampling_rate = 500 # sensors are 1000 but resampled to 500 as controller is running 500

t_start = 0
t_end = 14
fn_start = 5
fn_end = 25
t_fn_start = 8
t_fn_end = 9
plot_index = 0

t_plot_start=14
t_plot_end = 49

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

def get_sinusoid(idx, time_stamp, data):
    time_ofsset = 14.0
    duration = 5
    margin = 0.1
    start_time = time_ofsset + duration*idx + margin
    end_time = time_ofsset + duration*(idx+1) - margin
    absolute_differences = np.abs(time_stamp - start_time)
    index_of_start = np.argmin(absolute_differences)

    absolute_differences = np.abs(time_stamp - end_time)
    index_of_end = np.argmin(absolute_differences)

    return time_stamp[index_of_start:index_of_end], data[index_of_start:index_of_end]


def phase_magnitude(input_signal, output_signal, sampling_rate, frequency):
    # Cross-correlation to find the time delay
    output_signal_resampled = resample(output_signal, len(input_signal))
    t = np.arange(0, len(input_signal)/sampling_rate, 1/sampling_rate)  

    # Remove the DC offset by subtracting the mean value
    input_signal_centered = input_signal - np.mean(input_signal)
    output_signal_centered = output_signal_resampled - np.mean(output_signal_resampled)
    
    correlation = correlate(output_signal_centered, input_signal_centered, mode='full')
    lags = np.arange(-len(input_signal_centered) + 1, len(input_signal_centered))
    peak_index = np.argmax(correlation)
    time_delay_calculated = lags[peak_index] / sampling_rate
    # Calculate the phase shift
    period = 1 / frequency
    phase_shift_degrees = 360 * (time_delay_calculated / period)
    phase_shift_radians = 2 * np.pi * (time_delay_calculated / period)
    normalized_angle = phase_shift_degrees % 360
    if normalized_angle < 0:
        normalized_angle += 360

    # Calculate the RMS values of the centered signals
    input_rms = np.sqrt(np.mean(input_signal_centered**2))
    output_rms = np.sqrt(np.mean(output_signal_centered**2))

    # Calculate the magnitude ratio in dB
    magnitude_ratio_dB = 20 * np.log10(output_rms / input_rms)
    # Plot the cross-correlation

    return -normalized_angle, -phase_shift_radians, magnitude_ratio_dB

def gen_bode_plot(input_signal, input_time_stamp, output_signal, output_time_stamp, sampling_rate, frequencies):
    phase_list = []
    magnitude_list = []

    for i in range(len(frequencies)):
        time_stamp_out, signal_out = get_sinusoid(i, output_time_stamp, output_signal)
        time_stamp_in, signal_in = get_sinusoid(i, input_time_stamp, input_signal)
        phase_shift_degrees, phase_shift_radians, magnitude_ratio_dB = phase_magnitude(signal_in, signal_out, sampling_rate, frequencies[i])
        phase_list.append(phase_shift_degrees)
        magnitude_list.append(magnitude_ratio_dB)
    return phase_list, magnitude_list

if __name__ == "__main__":
    f, (ax1,ax2) = plt.subplots(2, 1, figsize=(6,4))
    f2, ax3 = plt.subplots(1, 1, figsize=(6,3))
    timestamps, data = read_rosbag_Fd_data(bag_disered_force, topic_fd)
    timestamps -= timestamps[0]
    timestamps, data = get_data_time(timestamps, data, t_plot_start, t_plot_end)
    ax3.plot(timestamps, data, linewidth=1.5, label="$F_d$")

    for key, value in objects.items():
        phase_list = []
        magnitude_list = []

        for i in range(len(value)):
            timestamps, data = read_rosbag_Fd_data(value[i], topic_fd)
            time_0 = timestamps[0]
            timestamps_ft, data_ft = read_rosbag_Ft_data(value[i], topic)
            timestamps += -time_0
            timestamps_ft += -time_0
            phase, magnitude = gen_bode_plot(data, timestamps, data_ft, timestamps_ft, sampling_rate, frequency)
            magnitude_list.append(magnitude)
            phase_list.append(phase)
            if i == plot_index:
                timestamps, data = get_data_time(timestamps_ft, data_ft, t_plot_start, t_plot_end)
                ax3.plot(timestamps, data, alpha=0.5, linewidth=1.5, label=key)

        # get statistical risetime, overshoot and settle time and print 
        avg_magnitude = np.average(magnitude_list, axis=0)
        avg_phase = np.average(phase_list, axis=0)
        print(frequency, avg_magnitude)
        ax1.semilogx(frequency, avg_magnitude, label=key)
        ax2.semilogx(frequency, avg_phase, label=key)

    plt.ylabel('Phase (degrees)')

    ax2.set_xlabel('Frequency (Hz)', fontsize="12")
    plt.tight_layout()
    ax1.set_ylabel('Magnitude (dB)', fontsize="12")
    ax2.set_ylabel('Phase (degrees)', fontsize="12")
    ax3.set_xlabel('Time (s)', fontsize="12")
    ax3.set_ylabel('$f_n (N)$', fontsize="12")
    # Show the plot
    ax1.legend()
    ax2.legend()
    ax3.legend()

    plt.tight_layout()
    plt.show()
            
            
 


    
















