# # instal bagpy
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

path__ = parent_dir + "/plotting_gripper/gripper_control_bags/"
bag_file = path__ + 'wood_2.bag' 


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


def read_rosbag_vel_data(bag_file, topic):
    bag = rosbag.Bag(bag_file)
    timestamps = []
    data = []
    
    for topic, msg, t in bag.read_messages(topics=[topic]):
        # Assuming the message has a 'data' attribute and a header with a timestamp
        timestamps.append(t.to_sec())
        data.append(msg.velocity)
    
    bag.close()
    return np.array(timestamps), np.array(data)

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
    print(phase_list)
    return phase_list, magnitude_list

if __name__ == "__main__":
    topic = '/gripper_closed_loop'  # Replace with the topic you want to read from

    timestamps, data = read_rosbag_Fd_data(bag_file, topic)

    topic = '/netft_data_1_calibrated'  # Replace with the topic you want to read from
    timestamps_ft, data_ft = read_rosbag_Ft_data(bag_file, topic)

    topic = '/gripper_monitor'  # Replace with the topic you want to read from
    timestamps_vel, data_vel = read_rosbag_vel_data(bag_file, topic)


    time_0 = timestamps[0]

    timestamps += -time_0
    timestamps_ft += -time_0
    timestamps_vel += -time_0
    print('t' , timestamps_ft[1000]-timestamps_ft[0])
    
    plt.figure()
    plt.plot(timestamps, data)
    plt.plot(timestamps_ft, data_ft)
    #plt.plot(timestamps_vel, -data_vel)

    plt.xlabel('Time (s)')
    plt.ylabel('Data')
    plt.title('Data from ROS Bag')
    frequency = [1, 2,4,8,16, 32, 64]
    sampling_rate = 500 # sensors are 1000 but resampled to 500 as controller is running 500
    phase, magnitude = gen_bode_plot(data, timestamps, data_ft, timestamps_ft, sampling_rate, frequency)
    # Create a new figure
    plt.figure()
    # Create the magnitude plot
    plt.subplot(2, 1, 1)  # 2 rows, 1 column, first subplot
    plt.semilogx(frequency, magnitude)  # Bode magnitude plot uses semilog scale for x-axis
    plt.title('Bode Plot')
    plt.ylabel('Magnitude (dB)')
    plt.grid(True, which="both", ls="--")

    # Create the phase plot
    plt.subplot(2, 1, 2)  # 2 rows, 1 column, second subplot
    plt.semilogx(frequency, phase)  # Bode phase plot uses semilog scale for x-axis
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Phase (degrees)')
    plt.grid(True, which="both", ls="--")

    # Show the plot
    plt.tight_layout()
    plt.show()

    