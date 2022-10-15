#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <string>


#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

costmap_2d::Costmap2DROS* local_costmap;
costmap_2d::Costmap2DROS* global_costmap;
ros::Publisher pub_goal_occupied;


bool fIsMapOccupied(geometry_msgs::Pose pGoalPose)
    {

    unsigned int mx, my;
    float x = pGoalPose.position.x;
    float y = pGoalPose.position.y;
    local_costmap->getCostmap()->worldToMap(x, y, mx, my);
    unsigned char cost = local_costmap->getCostmap()->getCost(mx, my);
    if (cost < 20)
        {
        return false;
        }

    global_costmap->getCostmap()->worldToMap(x, y, mx, my);
    cost = global_costmap->getCostmap()->getCost(mx, my);
    if (cost < 20)
        {
        return false;
        }
    return true;

    }

void goal_request_callback(geometry_msgs::Pose pGoalPose)
    {
    std_msgs::Bool rAnswer;
    rAnswer.data = fIsMapOccupied(pGoalPose);
    }



int main(int argc, char** argv)
    {

    ros::init(argc, argv, "costmap_manager");
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf2_listener(tf_buffer);
    ros::NodeHandle node_handle;
    ros::Subscriber sub_goal_request = node_handle.subscribe("/locobot/frodo/goal_request", 1, goal_request_callback);

    pub_goal_occupied = node_handle.advertise<std_msgs::Bool>("/locobot/frodo/goal_occupied", 1);
    putenv((char*)"ROS_NAMESPACE=");
    local_costmap = new costmap_2d::Costmap2DROS("/locobot/move_base/local_costmap", tf_buffer);
    global_costmap = new costmap_2d::Costmap2DROS("/locobot/move_base/global_costmap", tf_buffer);
    putenv((char*)"ROS_NAMESPACE=locobot/costmap_manager");
    ros::spin();
    ROS_ERROR("Exiting prematurely");
    }
