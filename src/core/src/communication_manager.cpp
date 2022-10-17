
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
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

// BOND
#include <bondcpp/bond.h>

// MACRO
#include "../include/elrond_macro.h"

// HELPER FUNCTIONS
#include "../include/auxiliaryFunctions.h"
#include <boost/circular_buffer.hpp>
//////////  VARIABLE INITIALIZED //////////

int64_t ARM_STATUS = ARM_IDLE; // ARM_STATUS = 0 is arm idle
std_msgs::Int64 arm_status;
int64_t BASE_STATUS = BASE_IDLE;
int64_t BASE_PREV_STATUS = BASE_IDLE;
int ID_REQUESTED = 100;
float distance_arm = 0.5;
float distance_base = 0.5;
uint retry_base = 0;
uint retry_arm = 0;
const std::string planning_frame_arm = "locobot/base_footprint";

geometry_msgs::PoseStamped grasp_pose_goal;
geometry_msgs::PoseStamped base_pose_goal;
geometry_msgs::PoseStamped MARKER_POSE_GOAL;
geometry_msgs::PoseStamped HOME_POSE_GOAL; // Da definire
geometry_msgs::PoseStamped PLACE_GRASP_GOAL;

ar_track_alvar_msgs::AlvarMarkers markers_poses;
std_msgs::Int64 pick_place;

// DATABASE FOR REQUEST AND AR TAG
boost::circular_buffer<int32_t> id_request_buffer(10);

// ---------- PUBLISHERS ------------------------
ros::Publisher pub_grasp_pose_goal;
ros::Publisher pub_pick_place;
ros::Publisher pub_mobile_pose_goal;
ros::Publisher pub_no_marker;

void id_callback(std_msgs::Int64 id_request)
{

    ROS_INFO("[CORE::COMM_MANAGER] ---- ENTERED ID CALLBACK");
    int i;
    ID_REQUESTED = id_request.data;
    id_request_buffer.push_back(ID_REQUESTED);

    ROS_INFO("[CORE::COMM_MANAGER] ---- ID REQUESTED: %d", ID_REQUESTED);
    i = fFindIdInMarkers(markers_poses, ID_REQUESTED);

    // FOUND
    if (i >= 0)
    {

        ROS_INFO("[CORE::COMM_MANAGER] ---- BASE STATUS: %d", BASE_STATUS);
        if (BASE_STATUS == BASE_IDLE)
        {
            pick_place.data = PICK;
            pub_pick_place.publish(pick_place);
            MARKER_POSE_GOAL = markers_poses.markers[id_request_buffer.front()].pose;
            fChangeOrientation(base_pose_goal,MARKER_POSE_GOAL);
            fChangePosition(base_pose_goal,MARKER_POSE_GOAL,distance_base);
            pub_mobile_pose_goal.publish(base_pose_goal);
            ROS_INFO("[CORE::COMM_MANAGER] ---- ID FOUND PUBLISHING BASE GOAL");
        }
        else
        {
            ROS_INFO("[CORE::COMM_MANAGER] ---- ROBOT RUNNING, TRY LATER");
        }
    }
    else // NOT FOUND
    {
        ROS_ERROR("[CORE::COMM_MANAGER] ---- ARTAG NOT FOUND!");
    }
}

void artag_callback(ar_track_alvar_msgs::AlvarMarkers req)
{
    markers_poses = req;
}

void arm_status_callback(std_msgs::Int64 arm_status)
{
    ROS_INFO("[CORE::COMM_MANAGER] ---- ENTERED ARM CALLBACK");
    ARM_STATUS = arm_status.data;
    uint retry = 0;
    switch (ARM_STATUS)
    {
    case ARM_SUCCESS:
        if (pick_place.data == PICK)
        {
            WaitOnVariable(BASE_STATUS, BASE_IDLE);
            pick_place.data = PLACE;
            ROS_INFO("[CORE::COMM_MANAGER] ---- Starting PLACE");
            pub_mobile_pose_goal.publish(HOME_POSE_GOAL);
            pub_pick_place.publish(pick_place);
            retry_arm = 0;
            distance_arm = 0.5;
        }
        else
        {
            id_request_buffer.pop_front();
            retry_arm = 0;
            distance_arm = 0.5;

        }
        break;
    case ARM_IDLE:
        ROS_INFO("[CORE::COMM_MANAGER] ---- ARM IDLE");
        break;
    case ARM_FAIL:
        if (pick_place.data == PICK){
            if (retry_arm > 2)
            {
                ROS_ERROR("[CORE::COMM_MANAGER] ---- Max retry reached, FAIL!");
                distance_arm = 0.5;
                break;
            }
            distance_arm += 0.15;
            ROS_INFO("[CORE::COMM_MANAGER] ---- ARM FAILED PICK, Repositioning...");
            WaitOnVariable(BASE_STATUS, BASE_IDLE);
            fChangePosition(base_pose_goal, MARKER_POSE_GOAL, distance_arm);
            pub_mobile_pose_goal.publish(base_pose_goal);
            retry_arm++;
        }
        else{
            if (retry_arm > 2)
            {
                ROS_ERROR("[CORE::COMM_MANAGER] ---- Max retry reached, FAIL!");
                distance_arm = 0.5;
                break;
            }
            distance_arm += 0.15;
            ROS_INFO("[CORE::COMM_MANAGER] ---- ARM FAILED PLACE, Repositioning...");
            WaitOnVariable(BASE_STATUS, BASE_IDLE);
            fChangePosition(base_pose_goal, HOME_POSE_GOAL, distance_arm);
            pub_mobile_pose_goal.publish(base_pose_goal);
            retry_arm++;
        }
        break;

    case ARM_RUNNING:
        ROS_INFO("[CORE::COMM_MANAGER] ---- ARM RUNNING");
        break;
    default:
        break;
    }
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
    
    if (retry_base > 2)
    {
        ROS_ERROR("[CORE::COMM_MANAGER] ---- GOAL OCCUPIED! Max retry reached, FAIL!");
        distance_base = 0.5;
        return;
    }
    ROS_WARN("[CORE::COMM_MANAGER] ---- GOAL OCCUPIED! Trying new position. Retry : %d", retry_arm);
    distance_base += 0.15;
    fChangePosition(base_pose_goal,MARKER_POSE_GOAL, distance_base);
    pub_mobile_pose_goal.publish(base_pose_goal);
    retry_base++;
    return;
}

void base_status_GoalOk_switchHandler()
{
    retry_base = 0;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf2_listener(tf_buffer);
    geometry_msgs::TransformStamped odom_to_footprint;

    odom_to_footprint = tf_buffer.lookupTransform("locobot/base_footprint", "locobot/odom", ros::Time(0), ros::Duration(1.0));
    WaitOnVariable(BASE_STATUS,BASE_IDLE);

    if (pick_place.data == PICK)
    {
        ROS_INFO("[CORE::COMM_MANAGER] ---- STARTING PICK...");
        tf2::doTransform(markers_poses.markers[id_request_buffer.front()].pose, grasp_pose_goal, odom_to_footprint);
        pub_grasp_pose_goal.publish(grasp_pose_goal);
    }
    else
    {
        ROS_INFO("[CORE::COMM_MANAGER] ---- STARTING PLACE...");
        //tf2::doTransform(PLACE_GRASP_GOAL,grasp_pose_goal,odom_to_footprint);
        pub_grasp_pose_goal.publish(PLACE_GRASP_GOAL);
    }
}

void base_status_callback(std_msgs::Int64 base_status)
{
    ROS_INFO("[CORE::COMM_MANAGER] ---- ENTERD BASE CALLBACK");
    BASE_STATUS = base_status.data;
    switch (BASE_STATUS)
    {
    case BASE_IDLE:
        ROS_INFO("[CORE::COMM_MANAGER] ---- BASE IDLE");
        base_status_idle_switchHandler();
        break;
    case BASE_TO_GOAL:
        ROS_INFO("[CORE::COMM_MANAGER] ---- BASE TO GOAL");
        base_status_ToGoal_switchHandler();
        break;
    case BASE_GOAL_OK:
        ROS_INFO("[CORE::COMM_MANAGER] ---- BASE ARRIVED AT GOAL");
        base_status_GoalOk_switchHandler();
        break;
    case BASE_GOAL_FAIL:
        ROS_ERROR("[CORE::COMM_MANAGER] ---- BASE FAILED");
        base_status_GoalFail_switchHandler();
        break;
    }
}

int main(int argc, char **argv)
{

    putenv((char *)"ROS_NAMESPACE=locobot");
    ros::init(argc, argv, "communication_manager");

    HOME_POSE_GOAL.header.frame_id = "map";
    HOME_POSE_GOAL.pose.position.x = 0;
    HOME_POSE_GOAL.pose.position.y = 0;
    HOME_POSE_GOAL.pose.position.z = 0;
    HOME_POSE_GOAL.pose.orientation.x = 0;
    HOME_POSE_GOAL.pose.orientation.y = 0;
    HOME_POSE_GOAL.pose.orientation.z = 0;
    HOME_POSE_GOAL.pose.orientation.w = 1;

    PLACE_GRASP_GOAL.header.frame_id = "map";
    PLACE_GRASP_GOAL.pose.position.x = 0.4;
    PLACE_GRASP_GOAL.pose.position.y = 0;
    PLACE_GRASP_GOAL.pose.position.z = 0.2;
    PLACE_GRASP_GOAL.pose.orientation.x = 0;
    PLACE_GRASP_GOAL.pose.orientation.y = 0;
    PLACE_GRASP_GOAL.pose.orientation.z = 0;
    PLACE_GRASP_GOAL.pose.orientation.w = 1;
    arm_status.data = ARM_IDLE; // Initialize arm as idle

    ros::NodeHandle node_handle;
    ros::Subscriber sub_id_request = node_handle.subscribe("/locobot/frodo/id_request", 1, id_callback);
    ros::Subscriber sub_artag = node_handle.subscribe("/locobot/move_group/ar_pose_marker", 1, artag_callback);
    ros::Subscriber sub_status_arm = node_handle.subscribe("/locobot/frodo/arm_status", 1, arm_status_callback);
    ros::Subscriber sub_status_base = node_handle.subscribe("/locobot/frodo/base_status", 1, base_status_callback);

    pub_grasp_pose_goal = node_handle.advertise<geometry_msgs::PoseStamped>("/locobot/frodo/grasp_pose_goal", 1);
    pub_pick_place = node_handle.advertise<std_msgs::Int64>("/locobot/frodo/pick_or_place", 1);
    pub_mobile_pose_goal = node_handle.advertise<geometry_msgs::PoseStamped>("/locobot/frodo/mobile_pose_goal", 1);
    pub_no_marker = node_handle.advertise<std_msgs::String>("/locobot/frodo/no_marker", 1);

    ros::spin();
    return 0;
}
