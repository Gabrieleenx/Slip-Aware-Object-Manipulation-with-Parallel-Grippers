# # instal bagpy
import rosbag
from sklearn.linear_model import LinearRegression

from pathlib import Path
current_file_dir = Path(__file__).parent
parent_dir = str(current_file_dir.parent)

import seaborn as sns
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np 

sensor = 2
chip = 3
direction = 'y'
distance = 100 # m
Hz_sensor = 120
nr_of_stops = 13
mpl.rcParams['font.family'] = 'Times New Roman'
mpl.rcParams['font.serif'] = ['Times New Roman'] + mpl.rcParams['font.serif']
mpl.rcParams["mathtext.fontset"] = 'cm'
mpl.rcParams['axes.xmargin'] = 0
mpl.rcParams['axes.formatter.limits'] = (-2, 3)
sns.set_theme("paper", "ticks", font_scale=1.0, rc={"lines.linewidth": 2})
name = 'Sensor' + str(sensor)+ 'chip' + str(chip) + direction #+ 'new'
bag_file = parent_dir + '/sensor_test/calibration_bags/'+ name + '.bag'

def read_rosbag_data(bag_file, topic, data_entry):
    bag = rosbag.Bag(bag_file)
    timestamps = []
    data = []
    
    for topic, msg, t in bag.read_messages(topics=[topic]):
        # Assuming the message has a 'data' attribute and a header with a timestamp
        timestamps.append(t.to_sec())
        if data_entry == 'y':
            data.append(msg.y)
        elif data_entry == 'theta':
            data.append(msg.theta)
        elif data_entry == 'x':
            data.append(msg.x)
    bag.close()
    return np.array(timestamps), np.array(data)*1000

def fint_platoes(time_stamps, data, plato_length):
    groups = []
    times = []
    
    x0 = data[0]
    candidate = []
    for i in range(len(data)):
        x1 = data[i]
        if abs(x0 - x1)<1e-3 and len(candidate) <= 1.1*plato_length*Hz_sensor:
            candidate.append(x1)
        else:
            if len(candidate) > plato_length*Hz_sensor:
                can_avg = np.average(candidate)
                if len(groups) > 0:
                    print( abs(can_avg - groups[-1]) )
                    if abs(can_avg - groups[-1]) > distance/4:
                        groups.append(np.average(candidate))
                        times.append(time_stamps[i])
                else:
                    groups.append(np.average(candidate))
                    times.append(time_stamps[i])
            candidate = []
            x0 = x1

    return groups, times
        
            
def order_two_lists(list1, list2):
    # Pair the elements of the lists
    paired_lists = list(zip(list1, list2))
    
    # Sort the pairs based on the first list
    paired_lists.sort()
    
    # Unzip the pairs back into two lists
    sorted_list1, sorted_list2 = zip(*paired_lists)
    
    return list(sorted_list1), list(sorted_list2)

def analyse_data(time_stamps, data):
    avg_platoes, tim_platoes = fint_platoes(timestamps, data, 2)
    if len(avg_platoes) != nr_of_stops:
        print("found wrong nr of platoes", len(avg_platoes))
        print(avg_platoes)
    index = 0
    index_1 = None
    start_x = []
    end_x = []
    start_t = []
    end_t = []
    for i in range(len(data)):
        if time_stamps[i] > tim_platoes[index]:
            x = data[i]
            if abs(x - avg_platoes[index]) > distance/100 and index_1 == None:
                index_1 = i
            if abs(x - avg_platoes[index+1]) < distance/100:
                start_x.append(avg_platoes[index])
                start_t.append(time_stamps[index_1])
                end_x.append(avg_platoes[index+1])
                end_t.append(time_stamps[i])
                index += 1
                index_1 = None
                if index >= len(avg_platoes)-1:
                    break
    start_x = np.array(start_x)
    end_x = np.array(end_x)
    start_t = np.array(start_t)
    end_t = np.array(end_t)
    delta_x = end_x - start_x

    delta_t = (end_t - start_t)
    vel_m = delta_x/delta_t

    vel_t = []
    vel_mt = []

    for i in range(len(delta_x)):
        if delta_x[i] >= 0:

            v_t = distance/delta_t[i]
            vel_t.append(v_t)
            vel_mt.append(vel_m[i]/v_t)
        else:
            v_t = -distance/delta_t[i]
            vel_t.append(v_t)
            vel_mt.append(vel_m[i]/v_t)

    vel_m, vel_mt = order_two_lists(vel_m, vel_mt)
    return vel_mt, vel_m


def recalibrate_data(timestamps, data, a, b):
    pre_t = timestamps[0] - 1/Hz_sensor
    pre_data = data[0]
    data_new = []
    x = 0
    for i in range(len(data)):
        '''
        dt = timestamps[i] - pre_t
        pre_t = timestamps[i]
        dx = data * dt
        if dx >= 0:
            dx_new = dx + a_pos * data[i] *dt
        else:
        '''
        dt = timestamps[i] - pre_t
        pre_t = timestamps[i]
        v = (data[i] - pre_data)/dt
        pre_data = data[i]
        
        v_new =  v/(b + a*v) 

        x += v_new*dt
        data_new.append(x)

    return data_new



if __name__ == "__main__":
    if sensor == 1:
        topic = '/velocity_sensor_1'  # Replace with the topic you want to read from
    elif sensor == 2:
        topic = '/velocity_sensor_2'  # Replace with the topic you want to read from
    else:
        print("invalid sensor")

    timestamps, data = read_rosbag_data(bag_file, topic, direction)
    timestamps += - timestamps[0]

    
    avg_platoes, tim_platoes = fint_platoes(timestamps, data, 2)
    print(avg_platoes)
    data -= avg_platoes[0]
    vel_mt, vel_m = analyse_data(timestamps, data)

    vel_m_copy = np.array(vel_m).reshape(-1, 1)
    model = LinearRegression().fit(vel_m_copy, np.array(vel_mt))
    y_pred = model.predict(vel_m_copy)


    # Get the parameters
    manual_tuning = 0.88
    a = manual_tuning*model.coef_[0] # 
    b = model.intercept_
    print('For sensor:', sensor, ', chip: ', chip, ' and direction: ', direction)
    print('a:', a)
    print('b:', b)

    data_new = recalibrate_data(timestamps, data, a, b)
    
    if max(data[0:1300]) > 0.6*distance:    
        upper = np.ones(timestamps.shape)*distance
        lower = np.ones(timestamps.shape)*0
    else:
        upper = np.ones(timestamps.shape)*0
        lower = -np.ones(timestamps.shape)*distance

    f, (ax1, ax2) = plt.subplots(2, 1, figsize=(6,4))
    ax1.plot(timestamps, data, label='Measured')
    ax1.plot(timestamps, data_new, '--', label='After tuning')
    ax1.plot(timestamps, upper, alpha=0.5)
    ax1.plot(timestamps, lower, alpha=0.5)

    ax1.set_xlabel('Time [s]', fontsize="12")
    ax1.set_ylabel('Displacement [mm]', fontsize="12")
    ax1.legend(bbox_to_anchor=(1.02, 1), loc='upper left', borderaxespad=0., labelspacing = 0.1, fontsize="12")
    
    ax2.plot(np.array(vel_m)/1000, vel_mt, label='Measured')
    ax2.plot(np.array(vel_m)/1000, y_pred, '--', label='Fitted')

    ax2.set_xlabel('$v_m$ [m/s]', fontsize="12")
    ax2.set_ylabel('$v_m/v_{true}$', fontsize="12")
    ax2.legend(bbox_to_anchor=(1.02, 1), loc='upper left', borderaxespad=0., labelspacing = 0.1, fontsize="12")

    plt.tight_layout()
    plt.show()

    