#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Ar_track_alvar
#include <tf/transform_datatypes.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

// MoveIt
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
// Definition
const double tau = 2 * M_PI;
ar_track_alvar_msgs::AlvarMarkers req;
uint flag = 0;

class Artag
{
public:
  float x[10];
  float y[10];
  float z[10];
  float w[10];
  int id[10];
  double R[10];
  double P[10];
  double Y[10];
  void callback(ar_track_alvar_msgs::AlvarMarkers req);
};

void Artag::callback(ar_track_alvar_msgs::AlvarMarkers req)
{

  uint i;
  for (i = 0; i < req.markers.size(); i++)
  {
    id[i] = req.markers[i].id;
    tf::Quaternion q(req.markers[i].pose.pose.orientation.x, req.markers[i].pose.pose.orientation.y, req.markers[i].pose.pose.orientation.z, req.markers[i].pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(R[i], P[i], Y[i]);
    //  Get the position of the ARTag
    x[i] = req.markers[i].pose.pose.position.x;
    y[i] = req.markers[i].pose.pose.position.y;
    z[i] = req.markers[i].pose.pose.position.z;
    w[i] = req.markers[i].pose.pose.orientation.w;
    ROS_INFO("Lettura OK x=%1.3f,x=%1.3f,x=%1.3f",x[i],y[i],z[i]);
    // Get how many ARTags are in the scene
    // ROS_INFO("lunghezza vettore = %d",req.markers.size());
    // roll  --> rotate around vertical axis
    // pitch --> rotate around horizontal axis
    // yaw   --> rotate around depth axis
  }
}

int main(int argc, char **argv)
{
  putenv("ROS_NAMESPACE=locobot");
  ros::init(argc, argv, "pose_listener");
  static const std::string PLANNING_GROUP_ARM = "interbotix_arm";
  static const std::string PLANNING_GROUP_GRIPPER = "interbotix_gripper";
  moveit::planning_interface::MoveGroupInterface::Options move_group_options_arm(PLANNING_GROUP_ARM, "locobot/robot_description");
  moveit::planning_interface::MoveGroupInterface::Options move_group_options_gripper(PLANNING_GROUP_GRIPPER, "locobot/robot_description");
  moveit::planning_interface::MoveGroupInterface move_group_interface_arm(move_group_options_arm);
  moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(move_group_options_gripper);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  ros::NodeHandle node_handle;
  ros::Rate loop_rate(1);
  Artag artag;
  ros::Subscriber sub = node_handle.subscribe("/locobot/move_group/ar_pose_marker", 1, &Artag::callback, &artag);
  
  while (ros::ok())
  {
    ros::spinOnce();
    if (flag == 0)
    {
      moveit_msgs::CollisionObject collision_object;

      // moveit_msgs::AttachedCollisionObject attached_object;

      // attached_object.link_name = "";
      /* The header must contain a valid TF frame*/
      collision_object.header.frame_id = move_group_interface_arm.getPlanningFrame();

      // attached_object.object.header.frame_id = "locobot/odom";
      /* The id of the object */
      collision_object.id = "box";
      ROS_INFO("Non sono ancora rotto");
      /* Pose given by the ARTag */
      geometry_msgs::Pose pose;
      pose.position.x = artag.x[0];
      pose.position.y = artag.y[0];
      pose.position.z = artag.z[0];
      pose.orientation.w = artag.w[0];
      // debug
      ROS_INFO("id = 0 trovato,x = %1.3f, y = %1.3f, z = %1.3f", pose.position.x, pose.position.y, pose.position.z);
      /* Until now we display a simple box in the position of the ARTag*/
      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[0] = 0.075;
      primitive.dimensions[1] = 0.075;
      primitive.dimensions[2] = 0.075;

      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(pose);
      collision_object.operation = collision_object.ADD;

      std::vector<moveit_msgs::CollisionObject> collision_objects;
      collision_objects.push_back(collision_object);
      planning_scene_interface.applyCollisionObjects(collision_objects);
      flag = 0;
    }
    loop_rate.sleep();
  }
  return 0;
}
