#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
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


//frodo costants
#include "../include/elrond_macro.h"


//Auxiliary functions
#include "../include/auxiliaryFunctions.h"
ros::Publisher pub_status;
ros::Publisher goal_pub;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

std_msgs::Int64 base_status_msg;

bool wait;
bool success_navigation;

//Handles simple case of goal pose from Communication Manacer
void moveBaseCallback(geometry_msgs::PoseStamped pPose)
    {
    ROS_INFO("Inside movebasecallback");
    actionlib::SimpleClientGoalState rResult = actionlib::SimpleClientGoalState::ABORTED;
    move_base_msgs::MoveBaseGoal rGoal;
    fMultiplyQuaternion(rGoal,pPose);
    rGoal.target_pose.header.frame_id = "map";
    rGoal.target_pose.header.stamp = ros::Time::now();
    MoveBaseClient client("/locobot/move_base", true);
    while (!client.waitForServer(ros::Duration(5.0)))
        { 
            ROS_INFO("Waiting for the move_base action server to come up");
        }
    client.sendGoal(rGoal);
    client.waitForResult();
    
    // if (wait)
    //     {
    //     rResult = client.getState();
    //     }
    // else
    //     {
    //     ROS_ERROR("Action server not available!");
    //     ros::requestShutdown();
    //     }

    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
        ROS_INFO("Goal execution done!");
        base_status_msg.data = BASE_GOAL_OK;
        pub_status.publish(base_status_msg);
        }
    else {
        ROS_INFO("Goal not reached!");
        base_status_msg.data = BASE_GOAL_FAIL;
        pub_status.publish(base_status_msg);
        }
    base_status_msg.data = BASE_TO_GOAL;
    pub_status.publish(base_status_msg);
    }
int main(int argc, char** argv)
    {
    ROS_INFO("Inside movebasecallback");
    putenv((char*)"ROS_NAMESPACE=locobot");
    ros::init(argc, argv, "base_controller");
    ros::NodeHandle node_handle;
    ros::Subscriber sub_mobile_goal_pose = node_handle.subscribe("/frodo/mobile_pose_goal", 1, moveBaseCallback);
    pub_status = node_handle.advertise<std_msgs::Int64>("/locobot/frodo/base_status", 1);
    ros::spin();
    }