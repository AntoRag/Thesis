import sys
import rospy
import rosbag
from std_msgs.msg import String
import pandas as pd

def rosbag_writer():
    rospy.init_node('save_data',anonymous=True)
    topic_name = String()
    topic_name.data = sys.argv[2]
    bag = rosbag.Bag(sys.argv[1], 'w') # (sys.argv[1]) Location and name of the .bag file to be created
    bag.write('Header',topic_name) #sys.argv[2] topic that im reading
    column_names = ['message', 'time']
    df = pd.DataFrame(columns=column_names)
    while not rospy.is_shutdown():
        print("sono fuori dal for")   
        for topic, msg, t in bag.read_messages(topics=[topic_name.data]):
            bag.write(topic, msg, t)
            df = df.append(
            {'m': msg,
             'T': t},
            ignore_index=True
            )
            print("sono qui")   
    bag.close()
    df.to_csv('/home/antonio/Desktop/out.csv')


if __name__ == '__main__':
    try:
        rosbag_writer()
    except rospy.ROSInterruptException:
        pass
