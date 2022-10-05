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
geometry_msgs::PoseStamped GRASP_POSE_GOAL;

void id_callback(std_msgs::String id_request)
{
    ID_REQUESTED = stoi(id_request.data);
    ROS_INFO("Id requested: %d", ID_REQUESTED);
}

void artag_callback(ar_track_alvar_msgs::AlvarMarkers req)
{

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

        // ARM COMMUNICATION PART PICK
        if (ARM_STATUS == 0)
        {
            pub_grasp_pose_goal.publish(GRASP_POSE_GOAL);
            ros::Duration(5).sleep(); // Wait 5 seconds that the topic receives the pick command
            pub_pick_place.publish("pick");
            bond_pick_arm.start();
            if (!bond_pick_arm.waitUntilFormed(ros::Duration(10.0)))
            {
                ROS_ERROR("ERROR bond not formed!");
                return false;
            }
            bond_pick_arm.waitUntilBroken(ros::Duration(-1.0));
            ROS_INFO("Arm finished the pick routine");
        }





        // ARM COMMUNICATION PART PLACE
        if (ARM_STATUS == 0)
        {
            pub_grasp_pose_goal.publish(GRASP_POSE_GOAL);
            ros::Duration(5).sleep(); // Wait 5 seconds that the topic receives the pick command
            pub_pick_place.publish("place");

            bond_place_arm.start();
            if (!bond_place_arm.waitUntilFormed(ros::Duration(10.0)))
            {
                ROS_ERROR("ERROR bond not formed!");
                return false;
            }
            bond_place_arm.waitUntilBroken(ros::Duration(-1.0));
            ROS_INFO("Arm finished the place routine");
        }


        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}