#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
double x, y, z;
double roll, pitch, yaw;

void callback(ar_track_alvar_msgs::AlvarMarkers req)
{
  tf::Quaternion qinv;
  if (!req.markers.empty())
  {
    uint i;
    for (i = 0; i < req.markers.size(); i++)
    {
      // in this example we read only the first marker in position 0 of the vector req.markers
      // Get the ID of the ARTag
      int id;
      id = req.markers[i].id;
      // ROS_INFO("id=%d", id);
      //  Get the orientation of the ARTag and trasnform it in RPY
      tf::Quaternion q(req.markers[i].pose.pose.orientation.x, req.markers[i].pose.pose.orientation.y, req.markers[i].pose.pose.orientation.z, req.markers[i].pose.pose.orientation.w);
      qinv = q;
      tf::Matrix3x3 m(q);
      m.getRPY(roll, pitch, yaw);
      // ROS_INFO("roll = %1.3f, pitch = %1.3f, yaw=%1.2f ", roll, pitch, yaw);
      //  Get the position of the ARTag
      x = req.markers[i].pose.pose.position.x;
      y = req.markers[i].pose.pose.position.y;
      z = req.markers[i].pose.pose.position.z;
      ROS_INFO("ARTag id = %d, x = %1.3f, y = %1.3f, z = %1.3f, q_z = %1.3f, q_w= %1.3f", id, x, y, z, q[2], q[3]);
      // Get how many ARTags are in the scene
      // ROS_INFO("lunghezza vettore = %d",req.markers.size());
      // roll  --> rotate around vertical axis
      // pitch --> rotate around horizontal axis
      // yaw   --> rotate around depth axis
    }
    MoveBaseClient ac("/locobot/move_base", true);

    // wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal;
    // we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.z =  qinv[2];
    goal.target_pose.pose.orientation.w =  qinv[3];

    ROS_INFO("Sending goal x = %f, y = %f, q_z = %1.3f, q_w= %1.3f", x, y, qinv[2], qinv[3]);
    ac.sendGoal(goal);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("YESSSS, the base moved");
    else
      ROS_INFO(":( NOPE");
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_listener");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/locobot/move_group/ar_pose_marker", 1, callback);
  // tell the action client that we want to spin a thread by default
  ros::spin();

  return 0;
}