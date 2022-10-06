
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


void fGetPoseFromMarker(geometry_msgs::PoseStamped& grasp_pose_goal, geometry_msgs::PoseStamped marker_pose)
    {
    grasp_pose_goal.pose.position.x = marker_pose.pose.position.x;
    grasp_pose_goal.pose.position.y = marker_pose.pose.position.y;
    grasp_pose_goal.pose.position.z = marker_pose.pose.position.z;
    grasp_pose_goal.pose.orientation.x = marker_pose.pose.orientation.x;
    grasp_pose_goal.pose.orientation.y = marker_pose.pose.orientation.y;
    grasp_pose_goal.pose.orientation.z = marker_pose.pose.orientation.z;
    grasp_pose_goal.pose.orientation.w = marker_pose.pose.orientation.w;
    }
