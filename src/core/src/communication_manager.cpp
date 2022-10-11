
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

const std::string planning_frame_arm = "locobot/base_footprint";

geometry_msgs::PoseStamped grasp_pose_goal;
geometry_msgs::PoseStamped base_pose_goal;
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

    ROS_INFO("Entered id_callback");
    int i;
    ID_REQUESTED = id_request.data;
    id_request_buffer.push_back(ID_REQUESTED);

    ROS_INFO("Id requested: %d", ID_REQUESTED);
    i = fFindIdInMarkers(markers_poses, ID_REQUESTED);
    // FOUND
    if (i >= 0)
    {

        ROS_INFO("Id BASE_STATUS: %d", BASE_STATUS);
        if (BASE_STATUS == BASE_IDLE)
        {
            fGetPoseFromMarker(base_pose_goal, markers_poses.markers[id_request_buffer.front()].pose);
            pick_place.data = PICK;
            pub_pick_place.publish(pick_place);
            pub_mobile_pose_goal.publish(base_pose_goal);

            ROS_INFO("Id finished publishing goal");
        }
        else
        {
            // TODO DO SOMETHING
        }
    }
    else // NOT FOUND
    {
        ROS_ERROR("IL TESSSSOROOOO");
    }
}

void artag_callback(ar_track_alvar_msgs::AlvarMarkers req)
{
    markers_poses = req;
}

void arm_status_callback(std_msgs::Int64 arm_status)
{
    ROS_INFO("Entered arm_callback");
    switch (arm_status.data)
    {
    case ARM_SUCCESS:
        if (pick_place.data == PICK)
        {
            if (BASE_STATUS == BASE_GOAL_OK)
            {
                pick_place.data = PLACE;
                ROS_INFO("Comincio il place");
                pub_mobile_pose_goal.publish(HOME_POSE_GOAL);
                pub_pick_place.publish(pick_place);
            }
        }
        break;
    case ARM_IDLE:
        id_request_buffer.pop_front();
        break;
    case ARM_FAIL:
        // Reposition the base TO-DO
        ROS_ERROR("ARM FAILED");
        break;
    case ARM_RUNNING:
        ROS_INFO("ARM CURRENTLY RUNNING");
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
    // todo
    return;
}

void base_status_GoalOk_switchHandler()
{

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf2_listener(tf_buffer);
    geometry_msgs::TransformStamped odom_to_footprint;
    geometry_msgs::PoseStamped new_grasp_pose_goal;

    odom_to_footprint = tf_buffer.lookupTransform("locobot/base_footprint", "locobot/odom", ros::Time(0), ros::Duration(1.0));

    auto idx = id_request_buffer.front();
    switch (ARM_STATUS)
    {
    case ARM_SUCCESS:
        ROS_INFO("Arm success after base status ok");
        grasp_pose_goal = PLACE_GRASP_GOAL;
        ros::Duration(5).sleep();
        pub_grasp_pose_goal.publish(grasp_pose_goal);
        break;
    case ARM_IDLE:
        ROS_INFO("Arm idle after base status ok");
        fGetPoseFromMarker(grasp_pose_goal, markers_poses.markers[idx].pose);
        tf2::doTransform(grasp_pose_goal, grasp_pose_goal, odom_to_footprint);
        ros::Duration(5).sleep();
        pub_grasp_pose_goal.publish(grasp_pose_goal);
        break;
    case ARM_FAIL:
        ROS_INFO("Arm fail after base status ok");
        pub_mobile_pose_goal.publish(HOME_POSE_GOAL); // repositioning to do
        break;
    default:
        break;
    }
}
void base_status_callback(std_msgs::Int64 base_status)
{
    ROS_INFO("Entered base_callback");
    // Need previous status to handle switching
    // BASE_PREV_STATUS = BASE_STATUS;
    BASE_STATUS = base_status.data;
    // if (BASE_STATUS == BASE_PREV_STATUS)
    //     return;
    switch (BASE_STATUS)
    {
    case BASE_IDLE:
        ROS_INFO("Base idle");
        base_status_idle_switchHandler();
        break;
    case BASE_TO_GOAL:
        ROS_INFO("Base to goal");
        base_status_ToGoal_switchHandler();
        break;
    case BASE_GOAL_OK:
        ROS_INFO("Base goal ok");
        base_status_GoalOk_switchHandler();
        break;
    case BASE_GOAL_FAIL:
        ROS_INFO("Base fail");
        base_status_GoalFail_switchHandler();
        break;
    }
}

int main(int argc, char **argv)
{

    putenv((char *)"ROS_NAMESPACE=locobot");
    ros::init(argc, argv, "communication_manager");
    HOME_POSE_GOAL.pose.position.x = 0;
    HOME_POSE_GOAL.pose.position.y = 0;
    HOME_POSE_GOAL.pose.position.z = 0;
    HOME_POSE_GOAL.pose.orientation.x = 0;
    HOME_POSE_GOAL.pose.orientation.y = 0;
    HOME_POSE_GOAL.pose.orientation.z = 0;
    HOME_POSE_GOAL.pose.orientation.w = 1;

    PLACE_GRASP_GOAL.header.frame_id = "locobot/base_footprint";
    PLACE_GRASP_GOAL.pose.position.x = 0.3;
    PLACE_GRASP_GOAL.pose.position.y = 0;
    PLACE_GRASP_GOAL.pose.position.z = 0.1;
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
