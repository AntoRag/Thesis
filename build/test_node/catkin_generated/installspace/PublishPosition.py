import sys
import rospy
from std_msgs.msg import Float64

def publisher():
    pub = rospy.Publisher('/locobot/pan_controller/command',Float64,queue_size=10)
    rospy.init_node('ArmController',anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        command = 1.0
        rospy.loginfo(command)
        pub.publish(command)
        rate.sleep()
if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass