#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from std_msgs.msg import Float64
from nav_msgs.msg import Path

import rospy

mpl.rcParams['toolbar'] = 'None'
plt.ion()

max_run_time = 0
flight_length = 0
coverage_rate = 0

time_list1 = np.array([])
time_list2 = np.array([])
time_list3 = np.array([])

coverage_rate_list = np.array([])
flight_length_list = np.array([])
run_time_list = np.array([])

start_time = []
trigger = False

def trigger_callback(msg:Path):
    global start_time, trigger
    if trigger == False:
        start_time = rospy.Time.now()
        trigger = True

def flight_length_callback(msg:Float64):
    global flight_length
    flight_length = msg.data

    
def coverage_rate_callback(msg:Float64):
    global coverage_rate_list, time_list1, coverage_rate
    if trigger:
        coverage_rate = msg.data * 100.0
        coverage_rate_list = np.append(coverage_rate_list, coverage_rate)
        time_list1 = np.append(time_list1, (rospy.Time.now() - start_time).to_sec())

def run_time_callback(msg:Float64):
    global run_time_list, time_list3, max_run_time
    if trigger:
        run_time_list = np.append(run_time_list, msg.data)
        time_list3 = np.append(time_list3, (rospy.Time.now() - start_time).to_sec())
        if msg.data > max_run_time:
            max_run_time = msg.data


def listener():
    global trigger, max_run_time, time_list1, time_list2, time_list3, run_time_list, explored_rate_list, flight_length_list, flight_length, run_time

    rospy.init_node('visulization_tool')

    rospy.Subscriber("/waypoint_generator/waypoints", Path, trigger_callback)
    rospy.Subscriber("/statistics/flight_length", Float64, flight_length_callback)
    rospy.Subscriber("/statistics/exp_time", Float64 ,run_time_callback)
    rospy.Subscriber("/statistics/coverage_rate", Float64 ,coverage_rate_callback)

    fig = plt.figure(figsize=(20, 12))
    plt.suptitle("Exploration Metrics\n", fontsize=18)
    plt.subplots_adjust(left=0.05, right=0.95, top=0.9, bottom=0.1)
    plt.tight_layout()
    

    fig1 = fig.add_subplot(411)
    plt.margins(x=0.0001, y=0.0001)
    fig1.set_ylabel("Volume (m$^2$)", fontsize=18)
    l1, = fig1.plot(time_list1, coverage_rate_list, color='r', label='Explored Volume')

    fig2 = fig.add_subplot(412)
    fig2.set_ylabel("Distance (m)", fontsize=18)
    l2, = fig2.plot(time_list2, flight_length_list, color='r', label='Traveling Distance')

    fig3 = fig.add_subplot(413)
    fig3.set_ylabel("Volume (m$^2$)", fontsize=18)
    l3, = fig3.plot(time_list2, flight_length_list, color='r', label='Coverage Rate')

    fig4 = fig.add_subplot(414)
    fig4.set_ylabel("Runtime (s)", fontsize=18)
    l4, = fig4.plot(time_list3, run_time_list, 'o', color='r', label='Algorithm Runtime')

    fig.canvas.draw()

    count = 0
    r = rospy.Rate(100)  # 100hz
    while not rospy.is_shutdown():
        r.sleep()
        if trigger:
            count = count + 1
            duration = (rospy.Time.now() - start_time).to_sec()
            if count % 10 == 0:

                
                flight_length_list = np.append(flight_length_list, flight_length)
                time_list2 = np.append(time_list2, duration)

            if count >= 100:
                count = 0
                l1.set_xdata(time_list1)
                l1.set_ydata(coverage_rate_list)

                l2.set_xdata(time_list2)
                l2.set_ydata(flight_length_list)

                l3.set_xdata(time_list2)
                l3.set_ydata(flight_length_list)

                l4.set_xdata(time_list3)
                l4.set_ydata(run_time_list)
           
                fig1.set_ylim(0,  coverage_rate + 10)
                fig1.set_xlim(0.2, duration + 10)
                fig1.tick_params(axis='both', which='major', labelsize=16)  
              
                fig2.set_ylim(0,  flight_length + 100)
                fig2.set_xlim(0.2, duration + 10)
                fig2.tick_params(axis='both', which='major', labelsize=16) 

                fig3.set_ylim(0,  flight_length + 100)
                fig3.set_xlim(0.5, duration + 10)
                fig3.tick_params(axis='both', which='major', labelsize=16) 

                fig4.set_ylim(0, max_run_time + 10)
                fig4.set_xlim(0, duration + 10)
                fig4.tick_params(axis='both', which='major', labelsize=16) 

                fig.canvas.draw()


if __name__ == '__main__':
    listener()
