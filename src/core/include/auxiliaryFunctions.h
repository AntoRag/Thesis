
#include <tf/transform_datatypes.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include <nav_msgs/OccupancyGrid.h>


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

void fTurn30deg(geometry_msgs::PoseStamped& base_pose_goal,geometry_msgs::Pose& pose_goal){
    geometry_msgs::PoseStamped q1;

    q1.pose.orientation.w = 0.9659258; //0.7071068;
    q1.pose.orientation.x = 0;
    q1.pose.orientation.y = 0;
    q1.pose.orientation.z = 0.258819; //0.7071068;
 
    // First quaternion q1 (x1 y1 z1 r1)
    const float x1 = q1.pose.orientation.x;
    const float y1 = q1.pose.orientation.y;
    const float z1 = q1.pose.orientation.z;
    const float r1 = q1.pose.orientation.w;

    // Second quaternion q2 (x2 y2 z2 r2)
    float x2 = pose_goal.orientation.x;
    float y2 = pose_goal.orientation.y;
    float z2 = pose_goal.orientation.z;
    float r2 = pose_goal.orientation.w;

    base_pose_goal.pose.position.x = pose_goal.position.x;
    base_pose_goal.pose.position.y = pose_goal.position.y;
    base_pose_goal.pose.orientation.z = r2 * z1 + x2 * y1 - y2 * x1 + z2 * r1; // z component
    base_pose_goal.pose.orientation.w = r2 * r1 - x2 * x1 - y2 * y1 - z2 * z1; // r component
    base_pose_goal.header.frame_id = "map";
    base_pose_goal.header.stamp = ros::Time::now();

}



void fChangeOrientation(geometry_msgs::PoseStamped& base_pose_goal, geometry_msgs::Pose& pose_goal) {
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
    float x2 = pose_goal.orientation.x;
    float y2 = pose_goal.orientation.y;
    float z2 = pose_goal.orientation.z;
    float r2 = pose_goal.orientation.w;

    base_pose_goal.pose.orientation.x = 0;
    base_pose_goal.pose.orientation.y = 0;
    base_pose_goal.pose.orientation.z = r2 * z1 + x2 * y1 - y2 * x1 + z2 * r1; // z component
    base_pose_goal.pose.orientation.w = r2 * r1 - x2 * x1 - y2 * y1 - z2 * z1; // r component
    base_pose_goal.header.frame_id = "map";
    base_pose_goal.header.stamp = ros::Time::now();
    }


void fChangePosition(geometry_msgs::PoseStamped& base_pose_goal, geometry_msgs::Pose& pose_goal, float distance)
    {
    geometry_msgs::PoseStamped new_pose;
    float x2 = pose_goal.orientation.x;
    float y2 = pose_goal.orientation.y;
    float z2 = pose_goal.orientation.z;
    float r2 = pose_goal.orientation.w;


    tf2::Quaternion q(x2, y2, z2, r2);
    tf2::Matrix3x3 m_FromMaptoMarker(q);
    tf2::Vector3 vector(0, 0, distance);
    tf2::Vector3 raw1, raw2, raw3;

    raw1 = m_FromMaptoMarker.getRow(0);
    raw2 = m_FromMaptoMarker.getRow(1);
    raw3 = m_FromMaptoMarker.getRow(2);

    new_pose.pose.position.x = raw1.getX() * vector.getX() + raw1.getY() * vector.getY() + raw1.getZ() * vector.getZ() + pose_goal.position.x;
    new_pose.pose.position.y = raw2.getX() * vector.getX() + raw2.getY() * vector.getY() + raw2.getZ() * vector.getZ() + pose_goal.position.y;
    new_pose.pose.position.z = raw3.getX() * vector.getX() + raw3.getY() * vector.getY() + raw3.getZ() * vector.getZ() + pose_goal.position.z;
    base_pose_goal.pose.position.x = new_pose.pose.position.x;
    base_pose_goal.pose.position.y = new_pose.pose.position.y;
    base_pose_goal.pose.position.z = 0;

    base_pose_goal.header.frame_id = "map";
    base_pose_goal.header.stamp = ros::Time::now();
    ROS_INFO("Marker pose: x=%1.3f y=%1.3f, Goal pose: x=%1.3f y=%1.3f", pose_goal.position.x, pose_goal.position.y, base_pose_goal.pose.position.x, base_pose_goal.pose.position.y);

    }
template<class T, class U>
void WaitOnVariableOfPair(T& pFirstVariable, T pFirstPredicate, U& pSecondVariable, U pSecondPredicate)
    {
    ros::Rate loop_rate(5);
    ros::Duration(0.3).sleep();
    ros::spinOnce();
    while ((pFirstVariable != pFirstPredicate) && (pSecondVariable != pSecondPredicate))
        {
        ros::spinOnce();
        loop_rate.sleep();
        }
    }
template<class T>
void WaitOnVariable(T& pVariable, T pPredicate)
    {
    ros::Rate loop_rate(10);
    while (pVariable != pPredicate)
        {
        ros::spinOnce();
        loop_rate.sleep();
        }
    }