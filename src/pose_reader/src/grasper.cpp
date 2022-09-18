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

void callback(ar_track_alvar_msgs::AlvarMarkers req)
{
  static const std::string PLANNING_GROUP_ARM = "interbotix_arm";
  static const std::string PLANNING_GROUP_GRIPPER = "interbotix_gripper";
  moveit::planning_interface::MoveGroupInterface::Options move_group_options_arm(PLANNING_GROUP_ARM,"locobot/robot_description");
  moveit::planning_interface::MoveGroupInterface::Options move_group_options_gripper(PLANNING_GROUP_GRIPPER, "locobot/robot_description");
  moveit::planning_interface::MoveGroupInterface move_group_interface_arm(move_group_options_arm);
  moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(move_group_options_gripper);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  double x, y, z, w;
  double roll, pitch, yaw;
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
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    // ROS_INFO("roll = %1.3f, pitch = %1.3f, yaw=%1.2f ", roll, pitch, yaw);
    //  Get the position of the ARTag
    x = req.markers[i].pose.pose.position.x;
    y = req.markers[i].pose.pose.position.y;
    z = req.markers[i].pose.pose.position.z;
    ROS_INFO("ARTag id = %d, x = %1.3f, y = %1.3f, z = %1.3f", id, x, y, z);
    // Get how many ARTags are in the scene
    // ROS_INFO("lunghezza vettore = %d",req.markers.size());
    // roll  --> rotate around vertical axis
    // pitch --> rotate around horizontal axis
    // yaw   --> rotate around depth axis
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
    pose.position.x = req.markers[id].pose.pose.position.x;
    pose.position.y = req.markers[id].pose.pose.position.y;
    pose.position.z = req.markers[id].pose.pose.position.z;
    pose.orientation.w = req.markers[id].pose.pose.orientation.w;
    // debug
    ROS_INFO("id = %d trovato,x = %1.3f, y = %1.3f, z = %1.3f", id, pose.position.x, pose.position.y, pose.position.z);
    /* Until now we display a simple box in the position of the ARTag*/
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.75;
    primitive.dimensions[1] = 0.75;
    primitive.dimensions[2] = 0.75;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    planning_scene_interface.applyCollisionObjects(collision_objects);
  }
}

int main(int argc, char **argv)
{
  putenv("ROS_NAMESPACE=locobot");
  ros::init(argc, argv, "pose_listener");
  ros::NodeHandle node_handle;
  // ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("/locobot/planning_scene", 1);
  ros::Subscriber sub = node_handle.subscribe("/locobot/move_group/ar_pose_marker", 1, callback);
  ros::spin();
  return 0;
}
