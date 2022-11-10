
#include <geometry_msgs/Pose.h>
// MACRO STATUS FOR COMMUNICATION MANAGER AND BASE CONTROLLER

// BASE STATUS MACRO
#define BASE_TO_GOAL 0
#define BASE_GOAL_OK 1
#define BASE_GOAL_FAIL 2
#define BASE_IDLE 3

// ARM STATUS MACRO
#define ARM_FAIL 0
#define ARM_IDLE 1
#define ARM_RUNNING 2
#define ARM_SUCCESS 3
#define PICK 0
#define PLACE 1
// PICK AND PLACE MACRO

#define PICK 0
#define PLACE 1
#define NAN 2

// // DEFINE HOME POSITION BASE
// geometry_msgs::Pose HOME_POSE_GOAL( 
// 0,
// 0,
// 0,
// 0,
// 0,
// 0,
// 1);