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

#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>

//frodo costants
#include "../include/elrond_macro.h"
#include <move_base/move_base.h>

//Auxiliary functions
#include "../include/auxiliaryFunctions.h"






typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

MoveBaseClient moveBaseClient("/locobot/move_base", true);
ros::Publisher pub_status;
ros::Publisher goal_pub;

std_msgs::Int64 base_status_msg;

bool wait;
bool success_navigation;
nav_msgs::OccupancyGrid localMap;
nav_msgs::OccupancyGrid globalMap;



tf2_ros::Buffer tfbuf(ros::Duration(10));
costmap_2d::Costmap2DROS local_costmap("local_costmap", tfbuf);
costmap_2d::Costmap2DROS global_costmap("global_costmap", tfbuf);

bool fIsMapOccupied(nav_msgs::OccupancyGrid& pMap, geometry_msgs::PoseStamped& pPose)
    {
    unsigned int mx, my;
    float x = pPose.pose.position.x;
    float y = pPose.pose.position.y;
    local_costmap.getCostmap()->worldToMap(x, y, mx, my);
    unsigned char cost = local_costmap.getCostmap()->getCost(mx, my);
    if (cost < 20)
        {
        return false;
        }

    global_costmap.getCostmap()->worldToMap(x, y, mx, my);
    cost = global_costmap.getCostmap()->getCost(mx, my);
    if (cost < 20)
        {
        return false;
        }
    return true;

    }

// void localMapCallback(nav_msgs::OccupancyGrid rMap)
//     {
//     localMap = rMap;
//     }
// void  globalMapCallback(nav_msgs::OccupancyGrid rMap)
//     {
//     globalMap = rMap;
//     }
//Handles simple case of goal pose from Communication Manacer
bool moveBaseGoalCalc(move_base_msgs::MoveBaseGoal& pGoal, geometry_msgs::PoseStamped pPose)
    {
    uint retry = 0;
    float distance = 0.4;
    ROS_INFO("Inside movebasecallback");
    actionlib::SimpleClientGoalState rResult = actionlib::SimpleClientGoalState::ABORTED;

    fMultiplyQuaternion(pGoal, pPose, distance);
    while (fIsMapOccupied(localMap, pGoal.target_pose) || fIsMapOccupied(globalMap, pGoal.target_pose))
        {
        retry++;
        if (retry >= 2)
            {
            ROS_INFO("GOAL OCCUPIED! Max retry reached, FAIL!");
            return false;
            }

        ROS_INFO("GOAL OCCUPIED! Trying new position. Retry : %d", retry);
        distance += 0.1;
        fMultiplyQuaternion(pGoal, pPose, distance);
        }

    pGoal.target_pose.header.frame_id = "map";
    pGoal.target_pose.header.stamp = ros::Time::now();
    return true;
    }
void moveBaseCallback(geometry_msgs::PoseStamped pPose)
    {
    move_base_msgs::MoveBaseGoal rGoal;
    if (moveBaseGoalCalc(rGoal, pPose))
        {
        while (!moveBaseClient.waitForServer(ros::Duration(5.0)))
            {
            ROS_INFO("Waiting for the move_base action server to come up");
            }
        base_status_msg.data = BASE_TO_GOAL;
        pub_status.publish(base_status_msg);
        moveBaseClient.sendGoal(rGoal);
        moveBaseClient.waitForResult();
        auto rState = moveBaseClient.getState();

        if (moveBaseClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
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
        }
    else
        {
        ROS_INFO("Goal not reached!");
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
    ros::spin();
    }