#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <iostream>
#include <string>
// TF2
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Ar_track_alvar
#include <tf2/transform_datatypes.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

// Move base
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// NAV
#include <nav_msgs/Odometry.h>

// BOND
#include <bondcpp/bond.h>

// MACRO
#include "../include/elrond_macro.h"

// HELPER FUNCTIONS
#include "../include/auxiliaryFunctions.h"
#include <boost/circular_buffer.hpp>
bool pSearchingActive = false;
bool pFoundMarker = false;
//////////  VARIABLE INITIALIZED //////////

int ARM_STATUS = ARM_IDLE;
bool pApproaching = false;
std_msgs::Int64 arm_status;
int BASE_STATUS = BASE_IDLE;
int BASE_PREV_STATUS = BASE_IDLE;
int ID_REQUESTED = 100;
float distance_arm = 0.5;
float distance_base = 0.52;
uint retry_base = 0;
uint retry_arm = 0;
const std::string planning_frame_arm = "locobot/base_footprint";

geometry_msgs::Pose pMobileBasePosition;
geometry_msgs::PoseStamped grasp_pose_goal;
geometry_msgs::PoseStamped pre_grasp_pose_goal;
geometry_msgs::PoseStamped base_pose_goal;
geometry_msgs::PoseStamped MARKER_POSE_GOAL;
geometry_msgs::PoseStamped HOME_POSE_GOAL; // Da definire
geometry_msgs::PoseStamped PLACE_GRASP_GOAL;
geometry_msgs::PoseStamped DEPOT_POSE_GOAL;
geometry_msgs::PoseStamped SPOT1_POSE_GOAL;
geometry_msgs::PoseStamped SPOT2_POSE_GOAL;
geometry_msgs::PoseStamped INITIAL_ROBOT_POSE;
ar_track_alvar_msgs::AlvarMarkers markers_poses;
ar_track_alvar_msgs::AlvarMarkers markers_poses_arm;
std_msgs::Int64 pick_place;

// DATABASE FOR REQUEST AND AR TAG
boost::circular_buffer<int32_t> id_request_buffer(10);

// ---------- PUBLISHERS ------------------------
ros::Publisher pub_grasp_pose_goal;
ros::Publisher pub_pick_place;
ros::Publisher pub_mobile_pose_goal;
ros::Publisher pub_no_marker;
ros::Publisher pub_pre_grasp_pose_goal;

std::vector<geometry_msgs::PoseStamped> pSearchPoses;

bool fSearchFunction()
{
    ROS_INFO("[CORE::COMM_MANAGER] ---- ENTERED SEARCH FUNCTION");
    pSearchingActive = true;
    pFoundMarker = false;
    int rSpot = 0;
    int rCount = 0;
    geometry_msgs::PoseStamped TempPose;
   
    while (rSpot < pSearchPoses.size())
    {
        int i = 0;
        while (i < 12)
        {
            // rotate 30 degrees
            fTurn30deg(TempPose, pMobileBasePosition);
            pub_mobile_pose_goal.publish(TempPose);
            ROS_INFO("[CORE::COMM_MANAGER] ---- ROTATING 30 DEGREES ON THE SPOT");
            WaitOnVariableOfPair(pFoundMarker, true, BASE_STATUS, BASE_IDLE);
            ROS_INFO("[CORE::COMM_MANAGER] ---- DONE ROTATING");
            i++;
            if (pFoundMarker)
                break;
        }
        if (pFoundMarker)
            break;
        pub_mobile_pose_goal.publish(pSearchPoses.at(rSpot));
        ROS_INFO("[CORE::COMM_MANAGER] ---- MOVING TO NEXT SEARCH SPOT");
        WaitOnVariableOfPair(pFoundMarker, true, BASE_STATUS, BASE_IDLE);
        if (pFoundMarker)
            break;
        rSpot++;
    }
    return pFoundMarker;
}

void base_status_idle_switchHandler()
{
    // todo
    return;
}

void base_status_ToGoal_switchHandler()

{
    // todo
    return;
}

void base_status_GoalFail_switchHandler()
{
    // For now if I'm searching do not consider fail status
    if (pSearchingActive || pApproaching)
        return;
    if (retry_base > 2)
    {
        ROS_ERROR("[CORE::COMM_MANAGER] ---- GOAL OCCUPIED! Max retry reached, FAIL!");
        distance_base = 0.5;
        return;
    }
    ROS_WARN("[CORE::COMM_MANAGER] ---- GOAL OCCUPIED! Trying new position. Retry : %d", retry_arm);
    distance_base += 0.25;
    ros::spinOnce();
    ros::WallDuration(1).sleep();
    auto i = fFindIdInMarkers(markers_poses, ID_REQUESTED);
    MARKER_POSE_GOAL = markers_poses.markers[i].pose;
    fChangePosition(base_pose_goal, MARKER_POSE_GOAL.pose, distance_base);
    pub_mobile_pose_goal.publish(base_pose_goal);
    retry_base++;
    return;
}

void base_status_GoalOk_switchHandler()
{
    // For now if I'm searching I do not need to handle
    if (pSearchingActive || pApproaching){
        return;
    }
    retry_base = 0;
    WaitOnVariable(BASE_STATUS, BASE_IDLE);
    if (pick_place.data == PICK)
    {
        ROS_INFO("[CORE::COMM_MANAGER] ---- STARTING PICK...");
        auto time_now = ros::WallTime::now().toSec();
        while (ros::WallTime::now().toSec()- time_now<10){
            ros::spinOnce();
        }

        auto i = fFindIdInMarkers(markers_poses_arm, ID_REQUESTED);
        auto marker_pose_arm = markers_poses_arm.markers[i].pose;
        // fChangeOrientationArm(pre_grasp_pose_goal,marker_pose_arm.pose);
        fChangePositionArm(pre_grasp_pose_goal,marker_pose_arm.pose,0.1);
        pub_pre_grasp_pose_goal.publish(pre_grasp_pose_goal);
        ros::WallDuration(5).sleep();
        // fChangeOrientationArm(grasp_pose_goal,marker_pose_arm.pose);
        fChangePositionArm(grasp_pose_goal,marker_pose_arm.pose,0.03);
        pub_grasp_pose_goal.publish(grasp_pose_goal);
    }
    else
    {
        ROS_INFO("[CORE::COMM_MANAGER] ---- STARTING PLACE...");
        pub_grasp_pose_goal.publish(PLACE_GRASP_GOAL);
    }
}
