#!/usr/bin/env python
import numpy as np
from numpy import float64
from pandas import array
import rospy
from sensor_msgs.msg import JointState 
import sys
import csv
import time

# pass as first argument the topic where we will read (ex. /locobot/pan_controller/command)
# and as second where to save the csv file (ex. /home/<username>/Desktop/) )
prev_data_wheel = 0
prev_data_arm = 0
data_array = []
DELTA = 0.00001 #Minimum change between values that will be logged
MIN_VAL = 0.0001 #Minimum change with respect to original value size (if I subtract 0.01e-5 to 0.01e-8 the difference will be higher 
                #than the delta but still needs to be ignored
TIME_STEP = 50 #Milliseconds between logging events
STARTING_TIME = rospy.get_rostime()

def checker(data, flag) :
    """Checker function. Check minimum change between time steps to discourage useless data 
        (minimum change between joints, errors from sensors within a DELTA value

    Args:
        data (list of float64 ): list of current position and velocities values
        flag ( Boolean ): if True compare data with prev_data_wheel, if False compare data with prev_data_arm 

    Returns:
        _type_: Boolean
    """
    
    global prev_data_wheel
    global prev_data_arm
    data = np.array(data)
    if flag :
        sub = np.abs(np.subtract(data, prev_data_wheel))
        prev_data_wheel = data
    else :
        sub = np.abs(np.subtract(data, prev_data_arm))
        prev_data_arm = data
    if sub.max() > DELTA and data[np.argmax(sub)] > MIN_VAL:
        #print('Values of subtraction:\n', sub, '\n')
        #print( data[np.argmax(sub)] ,'was more than ', MIN_VAL, '\n')
        return True
    return False

def data_received(data):
    """Topic callback function. Receives JointState and parses it into data_array while doing frequency
        checks and useful data checks with checker()

    Args:
        data (JointState msg): Describes current joint states
    """

    global data_array
    global prev_time

    log_time = int(time.time() * 1000) #get actual time in ms to for frequency checking

    if (log_time - prev_time) > TIME_STEP :#frequency check, lower TIME_STEP value for more frequent logging and viceversa
        prev_time = log_time
        rospy.get
        ros_time = rospy.get_rostime() - STARTING_TIME #get rostime for logging purposes
        rospy.loginfo("Read acquired: %s", data.name)
        if data.name == ['wheel_right_joint', 'wheel_left_joint']:
            if checker(data.position, True):
                data_array.append([data.name,ros_time.secs, data.position[0], data.position[1]])
        else:
            if checker( data.position + data.velocity, False):
                data_array.append([ros_time.secs, data.position[0], data.position[1], data.position[2],
                                    data.position[3], data.position[4], data.position[5], data.position[6],
                                        data.position[7], data.position[8], data.position[11], data.position[12], 
                                            data.velocity[0], data.velocity[1], data.velocity[2], data.velocity[3],
                                                data.velocity[4], data.velocity[5], data.velocity[6], data.velocity[7],
                                                    data.velocity[8], data.velocity[9], data.velocity[10]])

def listener():
    """Listener node that initializes callback function data_received and waits for shutdown commands while
    processing events.
    """

    global prev_time
    rospy.init_node('save_data_joint', anonymous = True)
    topic_name = sys.argv[1]
    prev_time = int(time.time() * 1000) #get actual time in ms to for frequency checking    
    rospy.Subscriber(topic_name, JointState, data_received)
    rospy.spin()

def writeFile() :
    """File writer, called by try/finally block in main. Writes data_array to file with I/O open operations
        in CSV format. Logs error in case of raised IOError or Logs info in case of I/O success.
    """

    wheel_cnt = 0
    arm_cnt = 0    
    try:
        with open(sys.argv[2] + 'data_saved_arm.csv', mode = 'a') as file_arm, open(sys.argv[2] + 'data_saved_wheel.csv', mode = 'a') as file_wheel:
            file_arm = csv.writer(file_arm, delimiter = ',', quotechar = '"', quoting = csv.QUOTE_MINIMAL)
            file_wheel = csv.writer(file_wheel, delimiter = ',', quotechar = '"', quoting = csv.QUOTE_MINIMAL)
            for data in data_array :
                if data[0] == ['wheel_right_joint', 'wheel_left_joint']:
                    file_wheel.writerow(data)     
                    wheel_cnt = wheel_cnt +1
                else :
                    file_arm.writerow(data)
                    arm_cnt = arm_cnt +1
    except IOError as e:
        rospy.logerr('Operation failed: %s at time %f', e.strerror, rospy.get_rostime())
    else :
        rospy.loginfo('Operation Success! Written %d lines to %s and %d to %s\n', arm_cnt, sys.argv[2] 
                                +'data_saved_arm.csv', wheel_cnt, sys.argv[2] +'data_saved_wheel.csv')


if __name__ == '__main__':
    try :
        listener()
    finally :
        writeFile()
