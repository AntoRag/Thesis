#!/usr/bin/env python
from numpy import float64
from pandas import array
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState 
import sys
import csv
# pass as first argument the topic where we will read (ex. /locobot/pan_controller/command)
# and as second where to save the csv file (ex. /home/<username>/Desktop/) )
previous_data_wheel = 0
previous_data_arm = 0

def data_received(data):
    global previous_data_wheel
    global previous_data_arm
    rospy.loginfo("Read acquired: %s", data.name)
    headerlist=[str(data.name)]
    if data.name == ['wheel_right_joint', 'wheel_left_joint']:
        with open(sys.argv[2] +'data_saved_wheel.csv', mode='a') as file:
            file = csv.writer(file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            time = rospy.get_rostime()
            if previous_data_wheel != data:
                file.writerow([time.secs,data.position[0],data.position[1]])
                previous_data_wheel = data
    else:
        with open(sys.argv[2] +'data_saved_arm.csv', mode='a') as file:
            file = csv.writer(file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            time = rospy.get_rostime()
            if previous_data_arm != data:
                file.writerow([time.secs,data.position[0],data.position[1],data.position[2],data.position[3],data.position[4],data.position[5],data.position[6],data.position[7],data.position[8],data.position[11],data.position[12], data.velocity[0],data.velocity[1],data.velocity[2],data.velocity[3],data.velocity[4],data.velocity[5],data.velocity[6],data.velocity[7],data.velocity[8],data.velocity[9],data.velocity[10]])
                previous_data_arm =data

def listener():
    rospy.init_node('save_data_joint', anonymous=True)
    topic_name = sys.argv[1]
    rospy.Subscriber(topic_name, JointState, data_received)
    rospy.spin()

if __name__ == '__main__':
    listener()