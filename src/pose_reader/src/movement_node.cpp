#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <iostream>
#include <string>

// Move base
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
static const std::string PLANNING_GROUP_ARM = "interbotix_arm";
static const std::string PLANNING_GROUP_GRIPPER = "interbotix_gripper";

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>

ros::Publisher status_pub;
geometry_msgs::PoseStamped home_config;
geometry_msgs::PoseStamped q1;

void multiplyQuaternion(move_base_msgs::MoveBaseGoal &goal, geometry_msgs::PoseStamped &pose_goal)
{
  q1.pose.orientation.w = 0.5;
  q1.pose.orientation.x = 0.5;
  q1.pose.orientation.y = 0.5;
  q1.pose.orientation.z = -0.5;

  // First quaternion q1 (x1 y1 z1 r1)
  const float x1 = q1.pose.orientation.x;
  const float y1 = q1.pose.orientation.y;
  const float z1 = q1.pose.orientation.z;
  const float r1 = q1.pose.orientation.w;

  // Second quaternion q2 (x2 y2 z2 r2)
  const float x2 = pose_goal.pose.orientation.x;
  const float y2 = pose_goal.pose.orientation.y;
  const float z2 = pose_goal.pose.orientation.z;
  const float r2 = pose_goal.pose.orientation.w;

  // goal.target_pose.pose.orientation.x = x2 * r1 + r2 * x1 + y2 * z1 - z2 * y1; // x component
  // goal.target_pose.pose.orientation.y = r2 * y1 - x2 * z1 + y2 * r1 + z2 * x1; // y component
  goal.target_pose.pose.orientation.z = r2 * z1 + x2 * y1 - y2 * x1 + z2 * r1; // z component
  goal.target_pose.pose.orientation.w = r2 * r1 - x2 * x1 - y2 * y1 - z2 * z1; // r component
  pose_goal.pose.orientation.x = x2 * r1 + r2 * x1 + y2 * z1 - z2 * y1;        // x component
  pose_goal.pose.orientation.y = r2 * y1 - x2 * z1 + y2 * r1 + z2 * x1;        // y component
  pose_goal.pose.orientation.z = r2 * z1 + x2 * y1 - y2 * x1 + z2 * r1;        // z component
  pose_goal.pose.orientation.w = r2 * r1 - x2 * x1 - y2 * y1 - z2 * z1;        // r component
}

void openGripper(moveit::planning_interface::MoveGroupInterface &gripper_group)
{
  // // BEGIN_SUB_TUTORIAL open_gripper
  // /* Add both finger joints of panda robot. */
  // posture.joint_names.resize(2);
  // posture.joint_names[0] = "left_finger";
  // posture.joint_names[1] = "right_finger";
  // /* Set them as open, wide enough for the object to fit. */
  // posture.points.resize(1);
  // posture.points[0].positions.resize(2);
  // posture.points[0].positions[0] = 0.035;
  // posture.points[0].positions[1] = -0.035;
  // posture.points[0].time_from_start = ros::Duration(0.5);

  // // END_SUB_TUTORIAL
  // const robot_state::JointModelGroup* joint_model_group = gripper_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER);
  // moveit::core::RobotStatePtr current_state = gripper_group.getCurrentState();
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  ROS_INFO("Sono vivo1");
  // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  moveit::planning_interface::MoveGroupInterface::Plan plan1;
  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[0] = 0.035;  // meters
  gripper_group.setJointValueTarget(joint_group_positions);
  ROS_INFO("Sono vivo2");
  gripper_group.plan(plan1);
  ROS_INFO("Sono vivo3");
  gripper_group.execute(plan1);

}

// void closedGripper(moveit::planning_interface::MoveGroupInterface &gripper_group)
// {
//   // // BEGIN_SUB_TUTORIAL closed_gripper
//   // /* Add both finger joints of panda robot. */
//   // posture.joint_names.resize(2);
//   // posture.joint_names[0] = "left_finger";
//   // posture.joint_names[1] = "right_finger";
//   // /* Set them as closed. */
//   // posture.points.resize(2);
//   // posture.points[0].positions.resize(1);
//   // posture.points[0].positions[0] = 0.00;
//   // posture.points[0].positions[1] = 0.00;
//   // posture.points[0].time_from_start = ros::Duration(0.5);
//   // // END_SUB_TUTORIAL
// }

void pick(moveit::planning_interface::MoveGroupInterface &arm_group, moveit::planning_interface::MoveGroupInterface &gripper_group, geometry_msgs::PoseStamped &goal_pose)
{
  // // BEGIN_SUB_TUTORIAL pick1
  // // Create a vector of grasps to be attempted, currently only creating single grasp.
  // // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  // ROS_INFO("Enter picking pipeline");
  // std::vector<moveit_msgs::Grasp> grasps;
  // grasps.resize(1);

  // // Setting grasp pose
  // // ++++++++++++++++++++++
  // // This is the pose of panda_link8. |br|
  // // From panda_link8 to the palm of the eef the distance is 0.058, the cube starts 0.01 before 5.0 (half of the length
  // // of the cube). |br|
  // // Therefore, the position for panda_link8 = 5 - (length of cube/2 - distance b/w panda_link8 and palm of eef - some
  // // extra padding)
  // grasps[0].grasp_pose.header.frame_id = "locobot/base_footprint";

  // // grasps[0].grasp_pose.pose.orientation = goal_pose.pose.orientation;
  // // grasps[0].grasp_pose.pose.position.x = 0.3*goal_pose.pose.position.x;
  // // grasps[0].grasp_pose.pose.position.y = 0.3*goal_pose.pose.position.y;
  // // grasps[0].grasp_pose.pose.position.z = goal_pose.pose.position.z;
  // tf2::Quaternion orientation;
  // orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
  // grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  // grasps[0].grasp_pose.pose.position.x = 0.5;
  // grasps[0].grasp_pose.pose.position.y = 0;
  // grasps[0].grasp_pose.pose.position.z = 0.3;

  // // Setting pre-grasp approach
  // // ++++++++++++++++++++++++++
  // /* Defined with respect to frame_id */
  // grasps[0].pre_grasp_approach.direction.header.frame_id = "locobot/base_footprint";
  // /* Direction is set as positive x axis */
  // grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  // grasps[0].pre_grasp_approach.min_distance = 0.04;
  // grasps[0].pre_grasp_approach.desired_distance = 0.05;

  // // Setting post-grasp retreat
  // // ++++++++++++++++++++++++++
  // /* Defined with respect to frame_id */
  // grasps[0].post_grasp_retreat.direction.header.frame_id = "locobot/base_footprint";
  // /* Direction is set as positive z axis */
  // grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  // grasps[0].post_grasp_retreat.min_distance = 0.06;
  // grasps[0].post_grasp_retreat.desired_distance = 0.05;

  // // Setting posture of eef before grasp
  // // +++++++++++++++++++++++++++++++++++
  // ROS_INFO("OpenGripper");
  // openGripper(grasps[0].pre_grasp_posture);
  // // END_SUB_TUTORIAL

  // // BEGIN_SUB_TUTORIAL pick2
  // // Setting posture of eef during grasp
  // // +++++++++++++++++++++++++++++++++++
  // ROS_INFO("CloseGripper");
  // closedGripper(grasps[0].grasp_posture);
  // // END_SUB_TUTORIAL
  // // Call pick to pick up the object using the grasps given
  // ROS_INFO("Picking");
  // arm_group.pick("box", grasps);
  // // END_SUB_TUTORIAL

  ROS_INFO("Reference frame: %s", arm_group.getPlanningFrame().c_str());
  ROS_INFO( "End effector link: %s", arm_group.getEndEffectorLink().c_str());

  geometry_msgs::Pose pre_grasp_pose;
  // pre_grasp_pose.orientation.w = 1.0;
  // pre_grasp_pose.position.x = 0.3*goal_pose.pose.position.x-0.05;
  // pre_grasp_pose.position.y = 0.3*goal_pose.pose.position.y-0.05;
  // pre_grasp_pose.position.z = 0.3*goal_pose.pose.position.z;
  pre_grasp_pose.orientation.w = 1.0;
  pre_grasp_pose.position.x = 0.4;
  pre_grasp_pose.position.y = 0.0;
  pre_grasp_pose.position.z = 0.3;
  arm_group.setPoseTarget(pre_grasp_pose);
  ROS_INFO("Currently planning and moving");
  arm_group.move();
  ROS_INFO("Finished to move");
  
  //openGripper(gripper_group);

  //closedGripper(gripper_group);



  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  // arm_group.plan(my_plan);
  // //bool success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // //ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  // ROS_INFO("So qua2");
  // arm_group.execute(my_plan);
  // ROS_INFO("Finished to move");
}

// void place(moveit::planning_interface::MoveGroupInterface &arm_group, moveit::planning_interface::MoveGroupInterface &gripper_group)
// {
//   // BEGIN_SUB_TUTORIAL place
//   // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
//   // location in
//   // verbose mode." This is a known issue and we are working on fixing it. |br|
//   // Create a vector of placings to be attempted, currently only creating single place location.
//   std::vector<moveit_msgs::PlaceLocation> place_location;
//   place_location.resize(1);

//   // Setting place location pose
//   // +++++++++++++++++++++++++++
//   place_location[0].place_pose.header.frame_id = "locobot/base_footprint";
//   tf2::Quaternion orientation;
//   orientation.setRPY(0, 0, M_PI / 2);
//   place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

//   /* While placing it is the exact location of the center of the object. */
//   place_location[0].place_pose.pose.position.x = 0.1;
//   place_location[0].place_pose.pose.position.y = 0.1;
//   place_location[0].place_pose.pose.position.z = 0.05;

//   // Setting pre-place approach
//   // ++++++++++++++++++++++++++
//   /* Defined with respect to frame_id */
//   place_location[0].pre_place_approach.direction.header.frame_id = "locobot/base_footprint";
//   /* Direction is set as negative z axis */
//   place_location[0].pre_place_approach.direction.vector.z = -1.0;
//   place_location[0].pre_place_approach.min_distance = 0.05;
//   place_location[0].pre_place_approach.desired_distance = 0.06;

//   // Setting post-grasp retreat
//   // ++++++++++++++++++++++++++
//   /* Defined with respect to frame_id */
//   place_location[0].post_place_retreat.direction.header.frame_id = "locobot/base_footprint";
//   /* Direction is set as negative y axis */
//   place_location[0].post_place_retreat.direction.vector.y = -1.0;
//   place_location[0].post_place_retreat.min_distance = 0.05;
//   place_location[0].post_place_retreat.desired_distance = 0.06;

//   // Setting posture of eef after placing object
//   // +++++++++++++++++++++++++++++++++++++++++++
//   /* Similar to the pick case */
//   openGripper(place_location[0].post_place_posture);

//   // Call place to place the object using the place locations given.
//   arm_group.place("box", place_location);
//   // END_SUB_TUTORIAL
// }

void addCollisionObjects(geometry_msgs::PoseStamped &goal_pose, moveit::planning_interface::MoveGroupInterface &move_group_interface_arm)
{
  // BEGIN_SUB_TUTORIAL table1
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  moveit_msgs::CollisionObject collision_object;

  // Add the first table where the cube will originally be kept.
  collision_object.id = "box";
  collision_object.header.frame_id = "locobot/base_footprint";

  /* Define the primitive and its dimensions. */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.03; // dimension od medicine based on id
  primitive.dimensions[1] = 0.03;
  primitive.dimensions[2] = 0.03;

  /* Define the pose of the table. */
  geometry_msgs::Pose pose;
  // pose.position.x = goal_pose.pose.position.x;
  // pose.position.y = goal_pose.pose.position.y;
  // pose.position.z = goal_pose.pose.position.z;
  pose.position.x = 0.5;
  pose.position.y = 0;
  pose.position.z = 0.3;
  // END_SUB_TUTORIAL
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(pose);
  collision_object.operation = collision_object.ADD;
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  planning_scene_interface.applyCollisionObjects(collision_objects);
  ROS_INFO("Aggiunta scatola alla scena");
}

void pose_callback(geometry_msgs::PoseStamped pose_goal)
{
  // robot is occupied
  std_msgs::String current_status;
  current_status.data = "running";
  status_pub.publish(current_status);
  ROS_INFO("Robot running");
  // Manipulator moves
  moveit::planning_interface::MoveGroupInterface::Options move_group_options_arm(PLANNING_GROUP_ARM, "locobot/robot_description");
  moveit::planning_interface::MoveGroupInterface::Options move_group_options_gripper(PLANNING_GROUP_GRIPPER, "locobot/robot_description");
  moveit::planning_interface::MoveGroupInterface move_group_interface_arm(move_group_options_arm);
  moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(move_group_options_gripper);

  // pick and place routine
  // base moves
  MoveBaseClient ac("/locobot/move_base", true);
  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;
  tf::Quaternion q(pose_goal.pose.orientation.x, pose_goal.pose.orientation.y, pose_goal.pose.orientation.z, pose_goal.pose.orientation.w);

  multiplyQuaternion(goal, pose_goal);

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 0.7 * pose_goal.pose.position.x;
  goal.target_pose.pose.position.y = 0.7 * pose_goal.pose.position.y;
  ac.sendGoal(goal);
  ac.waitForResult();
  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The base moved correctly");
  else
    ROS_INFO(":( NOPE");

  ros::WallDuration(5.0).sleep();
  addCollisionObjects(pose_goal, move_group_interface_arm);
  ros::WallDuration(5.0).sleep();
  move_group_interface_arm.setPlanningTime(45.0);

  ros::WallDuration(1.0).sleep();

  pick(move_group_interface_arm, move_group_interface_gripper, pose_goal);
  ros::WallDuration(1.0).sleep();

  //place(move_group_interface_arm, move_group_interface_gripper);
  // robot is free to work
  current_status.data = "idle";
  status_pub.publish(current_status);
  ROS_INFO("Robot idle");
}

int main(int argc, char **argv)
{

  putenv("ROS_NAMESPACE=locobot");
  ros::init(argc, argv, "movement_node");
  ros::NodeHandle node_handle;
  ros::Subscriber sub_pose = node_handle.subscribe("/locobot/pose_goal", 1, pose_callback);
  status_pub = node_handle.advertise<std_msgs::String>("/locobot/robot_status", 1);
  static const std::string PLANNING_GROUP_ARM = "interbotix_arm";
  static const std::string PLANNING_GROUP_GRIPPER = "interbotix_gripper";
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}