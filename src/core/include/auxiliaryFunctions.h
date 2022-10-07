
#include <tf/transform_datatypes.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
int fFindIdInMarkers(ar_track_alvar_msgs::AlvarMarkers markers_poses, int32_t id_request)
{
    int i = 0;
    for (auto iter : markers_poses.markers)
    {
        if (id_request == iter.id)
            return i;
        i++;
    }
    return -1;
}



void fMultiplyQuaternion(move_base_msgs::MoveBaseGoal &base_pose_goal, geometry_msgs::PoseStamped &pose_goal)
{

    geometry_msgs::PoseStamped q1;

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
    base_pose_goal.target_pose.pose.orientation.z = r2 * z1 + x2 * y1 - y2 * x1 + z2 * r1; // z component
    base_pose_goal.target_pose.pose.orientation.w = r2 * r1 - x2 * x1 - y2 * y1 - z2 * z1; // r component
    // pose_goal.pose.orientation.x = x2 * r1 + r2 * x1 + y2 * z1 - z2 * y1;        // x component
    // pose_goal.pose.orientation.y = r2 * y1 - x2 * z1 + y2 * r1 + z2 * x1;        // y component
    // pose_goal.pose.orientation.z = r2 * z1 + x2 * y1 - y2 * x1 + z2 * r1;        // z component
    // pose_goal.pose.orientation.w = r2 * r1 - x2 * x1 - y2 * y1 - z2 * z1;        // r component
}