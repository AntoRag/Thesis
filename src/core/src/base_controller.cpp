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

int id_request;
int status_bool = 0;
ros::Publisher goal_pub;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void artag_callback(ar_track_alvar_msgs::AlvarMarkers req)
{
    int i;
    geometry_msgs::PoseStamped pose_id;
    for (i = 0; i < req.markers.size(); i++)
    {
        if (id_request == req.markers[i].id)
        {
            if (status_bool == 0)
            {
                pose_id.pose.position.x = req.markers[i].pose.pose.position.x;
                pose_id.pose.position.y = req.markers[i].pose.pose.position.y;
                pose_id.pose.position.z = req.markers[i].pose.pose.position.z;
                pose_id.pose.orientation.x = req.markers[i].pose.pose.orientation.x;
                pose_id.pose.orientation.y = req.markers[i].pose.pose.orientation.y;
                pose_id.pose.orientation.z = req.markers[i].pose.pose.orientation.z;
                pose_id.pose.orientation.w = req.markers[i].pose.pose.orientation.w;
                ROS_INFO("Lettura n%d OK, trovato id richiesto", i);
                goal_pub.publish(pose_id);
            }
            else if (status_bool == 1)
            {
                ROS_INFO("Lettura n%d OK, trovato id richiesto ma il robot sta runnando", i);
            }
        }
    }
}

void request_callback(std_msgs::String id_req)
{
    id_request = stoi(id_req.data);
    ROS_INFO("Id requested %d", id_request);
}

void status_callback(std_msgs::String status_robot)
{
    if (status_robot.data == "idle")
    {
        ROS_INFO("Robot currently idle");
        status_bool = 0;
    }
    else if (status_robot.data == "running")
    {
        ROS_INFO("Robot currently running");
        status_bool = 1;
    }
    else
    {
        ROS_INFO("Non so cosa hai scritto");
    }
}
void moveBaseCallback()
int main(int argc, char **argv)
{
    
    putenv((char*) "ROS_NAMESPACE=locobot");
    ros::init(argc, argv, "base_controller");
    ros::NodeHandle node_handle;
    ros::Subscriber sub_mobile_goal_pose = node_handle.subscribe("/locobot/frodo/mobile_goal_pose", 1, artag_callback);
    ros::Subscriber sub_no_marker = node_handle.subscribe("/locobot/frodo/no_marker", 1, request_callback);
    //ros::Publisher pub_status = node_handle.publish("/locobot/robot_status", 1, status_callback);
    ros::Publisher pub_status = node_handle.advertise<std_msgs::String>("locobot/frodo/base_status", 1);
    

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}