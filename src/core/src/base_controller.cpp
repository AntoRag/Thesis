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
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

// frodo costants
#include "../include/elrond_macro.h"
#include <move_base/move_base.h>
#include <move_base/MoveBaseConfig.h>
#include <global_planner/planner_core.h>
// Auxiliary functions
#include "../include/auxiliaryFunctions.h"

#include <tf2_ros/transform_listener.h>
#include <global_planner/astar.h>

#include <move_base/move_base.h>
#include <move_base/MoveBaseConfig.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Publisher pub_status;
ros::Publisher goal_pub;
ros::ServiceClient check_goal_feasable;

std_msgs::Int64 base_status_msg;
nav_msgs::GetPlan srv;
bool wait;
bool success_navigation;

// Handles simple case of goal pose from Communication Manacer
bool moveBaseGoalCalc(move_base_msgs::MoveBaseGoal& pGoal)
    {
    srv.request.goal = pGoal.target_pose;
    check_goal_feasable.call(srv);
    if (srv.response.plan.poses.size() != 0)
        {
        ROS_INFO("[CORE::BASE_CONTROLLER] ---- GOAL NOT OCCUPIED! Trying ");
        return true;
        }
    ROS_WARN("[CORE::BASE_CONTROLLER] ---- GOAL OCCUPIED! Trying new position");
    return false;
    }

void moveBaseCallback(geometry_msgs::PoseStamped pPose)
    {
    move_base_msgs::MoveBaseGoal rGoal;
    ROS_INFO("[CORE::BASE_CONTROLLER] ---- Inside movebasecallback");

    rGoal.target_pose.pose.position = pPose.pose.position;
    rGoal.target_pose.pose.orientation = pPose.pose.orientation;
    rGoal.target_pose.header.frame_id = pPose.header.frame_id;
    rGoal.target_pose.header.stamp = ros::Time::now();
    MoveBaseClient moveBaseClient("/locobot/move_base", true);
    if (moveBaseGoalCalc(rGoal))
        {
        while (!moveBaseClient.waitForServer(ros::Duration(5.0)))
            {
            ROS_INFO("[CORE::BASE_CONTROLLER] ---- Waiting for the move_base action server to come up");
            }
        base_status_msg.data = BASE_TO_GOAL;
        pub_status.publish(base_status_msg);
        moveBaseClient.sendGoal(rGoal);
        moveBaseClient.waitForResult();
        auto rState = moveBaseClient.getState();
        if (moveBaseClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
            ROS_INFO("[CORE::BASE_CONTROLLER] ---- Goal execution done!");
            base_status_msg.data = BASE_GOAL_OK;
            pub_status.publish(base_status_msg);
            }
        else
            {
            ROS_INFO("[CORE::BASE_CONTROLLER] ---- Goal not reached!");
            base_status_msg.data = BASE_GOAL_FAIL;
            pub_status.publish(base_status_msg);
            }
        }
    else
        {
        ROS_INFO("[CORE::BASE_CONTROLLER] ---- Goal not reached!");
        base_status_msg.data = BASE_GOAL_FAIL;
        pub_status.publish(base_status_msg);
        }
    ::sleep(5);
    base_status_msg.data = BASE_IDLE;
    pub_status.publish(base_status_msg);
    }

int main(int argc, char** argv)
    {
    putenv((char*)"ROS_NAMESPACE=locobot");
    ros::init(argc, argv, "base_controller");
    ros::NodeHandle node_handle;
    ros::Subscriber sub_mobile_goal_pose = node_handle.subscribe("/locobot/frodo/mobile_pose_goal", 1, moveBaseCallback);
    pub_status = node_handle.advertise<std_msgs::Int64>("/locobot/frodo/base_status", 1);
    check_goal_feasable = node_handle.serviceClient<nav_msgs::GetPlan>("/locobot/move_base/make_plan");

    ros::spin();
    }