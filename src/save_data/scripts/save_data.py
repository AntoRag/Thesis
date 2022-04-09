#!/usr/bin/env python
from numpy import float64
import rospy
from std_msgs.msg import Float64
import sys
import csv
# pass as first argument the topic where we will read (ex. /locobot/pan_controller/command and as second where to save the csv file (ex. /home/<username>/Desktop/) )
def data_received(data):
    rospy.loginfo("Read acquired: %s", data.data)
    #print(data.data) #data.data debugging print float 64 to be saved
    with open(sys.argv[2] +'data_saved.csv', mode='a') as file:
        file = csv.writer(file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        time = rospy.get_rostime()
        file.writerow([str(time.secs), str(data.data)])

def listener():
    rospy.init_node('save_data', anonymous=True)
    topic_name = sys.argv[1]
    rospy.Subscriber(topic_name, Float64, data_received)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
      listener()