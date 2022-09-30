#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <iostream>
#include <string>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Ar_track_alvar
#include <tf/transform_datatypes.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

// Move base
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>





void id_callback(std_msgs::String id_request){


}

void artag_callback(ar_track_alvar_msgs::AlvarMarkers req){

}

void arm_status_callback(std_msgs::String arm_status){

}

void base_status_callback(std_msgs::String base_status){

}


int main(int argc, char **argv)
{
    
    putenv((char*) "ROS_NAMESPACE=locobot");
    ros::init(argc, argv, "communication_manager");
    ros::NodeHandle node_handle;
    ros::Subscriber sub_id_request = node_handle.subscribe("/locobot/frodo/id_request", 1, id_callback);
    ros::Subscriber sub_artag = node_handle.subscribe("/locobot/move_group/ar_pose_marker", 1, artag_callback);
    ros::Subscriber sub_status_arm = node_handle.subscribe("/locobot/frodo/arm_status", 1, arm_status_callback);
    ros::Subscriber sub_status_base = node_handle.subscribe("/locobot/frodo/base_status", 1, base_status_callback);

    ros::Publisher pub_grasp_pose_goal = node_handle.advertise<geometry_msgs::PoseStamped>("locobot/frodo/grasp_pose_goal", 1);
    ros::Publisher pub_pick_place = node_handle.advertise<std_msgs::String>("locobot/frodo/pick_or_place", 1);
    ros::Publisher pub_mobile_pose_goal = node_handle.advertise<geometry_msgs::PoseStamped>("locobot/frodo/mobile_pose_goal", 1);
    ros::Publisher pub_no_marker = node_handle.advertise<std_msgs::String>("locobot/frodo/no_marker", 1);

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}