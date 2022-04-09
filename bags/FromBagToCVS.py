from bagpy import bagreader
import sys
from std_msgs.msg import String

bagfile_name = String()
bagfile_name = sys.argv[1] #specify the path of the .bag file to be converted
b = bagreader(bagfile_name)
topic_name = String()
topic_name = sys.argv[2] #Specify the topic of the .bag file to be translated into .cvs
bmesg = b.message_by_topic(topic_name)
