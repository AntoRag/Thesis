#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <iostream>
#include <string>


void test_callback(std_msgs::String data){
    ROS_INFO("test callback");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle node_handle;
    ros::Subscriber sub = node_handle.subscribe("/test", 1, test_callback);
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        ROS_INFO("test while");
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}