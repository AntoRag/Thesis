
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

// BOND
#include <bondcpp/bond.h>

//////////  VARIABLE INITIALIZED //////////

bool ARM_STATUS = 0; // ARM_STATUS = 0 is arm idle
int ID_REQUESTED = 0;
geometry_msgs::PoseStamped grasp_pose_goal;
geometry_msgs::PoseStamped base_pose_goal;
geometry_msgs::PoseStamped HOME_POSE_GOAL; // Da definire
ar_track_alvar_msgs::AlvarMarkers markers_poses;
std_msgs::String pick_place;

void id_callback(std_msgs::String id_request)
{
    ID_REQUESTED = stoi(id_request.data);
    ROS_INFO("Id requested: %d", ID_REQUESTED);
}

void artag_callback(ar_track_alvar_msgs::AlvarMarkers req)
{
    markers_poses = req;
}

void arm_status_callback(std_msgs::String arm_status)
{
    if (arm_status.data == "Idle")
    {
        ARM_STATUS = 0;
        ROS_INFO("Arm currently idle, can take commands");
    }
    else if (arm_status.data == "Running")
    {
        ARM_STATUS = 1;
        ROS_INFO("Arm currently running, cannot take commands");
    }
}

void base_status_callback(std_msgs::String base_status)
{
}

int main(int argc, char **argv)
{

    putenv((char *)"ROS_NAMESPACE=locobot");
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

    bond::Bond bond_pick_arm("/locobot/pick_arm", "PickArm");
    bond::Bond bond_place_arm("/locobot/place_arm", "PlaceArm");

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        // BASE MOVING TO TARGET

        // BASE SEARCHING FOR TARGET

        // ARM COMMUNICATION PART PICK;
        int i;
        for (i = 0; i < markers_poses.markers.size(); i++)
        {
            if (ID_REQUESTED == markers_poses.markers[i].id)
            {
                ROS_INFO("Lettura n%d OK, trovato id richiesto", i);

                if (ARM_STATUS == 0)
                {
                    grasp_pose_goal.pose.position.x = markers_poses.markers[i].pose.pose.position.x;
                    grasp_pose_goal.pose.position.y = markers_poses.markers[i].pose.pose.position.y;
                    grasp_pose_goal.pose.position.z = markers_poses.markers[i].pose.pose.position.z;
                    grasp_pose_goal.pose.orientation.x = markers_poses.markers[i].pose.pose.orientation.x;
                    grasp_pose_goal.pose.orientation.y = markers_poses.markers[i].pose.pose.orientation.y;
                    grasp_pose_goal.pose.orientation.z = markers_poses.markers[i].pose.pose.orientation.z;
                    grasp_pose_goal.pose.orientation.w = markers_poses.markers[i].pose.pose.orientation.w;

                    pick_place.data = "pick";

                    pub_grasp_pose_goal.publish(grasp_pose_goal);
                    ros::Duration(5).sleep(); // Wait 5 seconds that the topic receives the pick command
                    pub_pick_place.publish(pick_place);
                    bond_pick_arm.start();
                    if (!bond_pick_arm.waitUntilFormed(ros::Duration(10.0)))
                    {
                        ROS_ERROR("ERROR bond not formed!");
                        return false;
                    }
                    bond_pick_arm.waitUntilBroken(ros::Duration(-1.0));
                    ROS_INFO("Arm finished the pick routine");
                }
                else
                {
                    ROS_INFO("Arm currently running");
                }
            }
        }

        // BASE MOVING TO DEPOT

        // ARM COMMUNICATION PART PLACE
        if (ARM_STATUS == 0)
        {
            pick_place.data = "place";
            pub_grasp_pose_goal.publish(HOME_POSE_GOAL);
            ros::Duration(5).sleep(); // Wait 5 seconds that the topic receives the pick command
            pub_pick_place.publish(pick_place);

            bond_place_arm.start();
            if (!bond_place_arm.waitUntilFormed(ros::Duration(10.0)))
            {
                ROS_ERROR("ERROR bond not formed!");
            }
            bond_place_arm.waitUntilBroken(ros::Duration(-1.0));
            ROS_INFO("Arm finished the place routine");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}