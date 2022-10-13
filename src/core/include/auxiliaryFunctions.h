
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

void fGetPoseFromMarker(geometry_msgs::PoseStamped& grasp_pose_goal, geometry_msgs::PoseStamped  markers_pose)
    {
    ROS_INFO("fGetPoseFromMaker, calculating...");
    grasp_pose_goal = markers_pose;
    }

void fQuatProd(float vector_a[], float vector_b[], float vector_result[]) {

    vector_result[3] = vector_a[3] * vector_b[3] - vector_a[0] * vector_b[0] - vector_a[2] * vector_b[1] - vector_a[3] * vector_b[0];  // 1
    vector_result[0] = vector_a[3] * vector_b[0] + vector_a[0] * vector_b[3] + vector_a[2] * vector_b[2] - vector_a[3] * vector_b[1];  // i
    vector_result[1] = vector_a[3] * vector_b[1] - vector_a[0] * vector_b[2] + vector_a[2] * vector_b[3] + vector_a[3] * vector_b[0];  // j
    vector_result[2] = vector_a[3] * vector_b[2] + vector_a[0] * vector_b[1] - vector_a[2] * vector_b[0] + vector_a[3] * vector_b[3];  // k
}


void fMultiplyQuaternion(move_base_msgs::MoveBaseGoal& base_pose_goal, geometry_msgs::PoseStamped& pose_goal, float distance)
    {
    double roll, pitch, yaw;
    geometry_msgs::PoseStamped q1;
    geometry_msgs::PoseStamped q1_conj;
    geometry_msgs::PoseStamped Vector_subtract; // defined in Marker frame


    Vector_subtract.pose.position.x = 0;
    Vector_subtract.pose.position.y = 0;
    Vector_subtract.pose.position.z = 0;
    Vector_subtract.pose.orientation.x = 0;
    Vector_subtract.pose.orientation.y = 0;
    Vector_subtract.pose.orientation.z = 0;
    Vector_subtract.pose.orientation.w = 1;
    
    geometry_msgs::PoseStamped Vector_subtract;
    geometry_msgs::Quaternion poseQ = pose_goal.pose.orientation;
    geometry_msgs::Point poseP = pose_goal.pose.position;
    tf::Quaternion poseQuaternion(poseQ.x, poseQ.y, poseQ.z, poseQ.w);

    // float x = 2 * (poseQ.y * poseQ.w + poseQ.x * poseQ.z);
    // float y = 2 * (poseQ.z * poseQ.w - poseQ.x * poseQ.y);
    // float z = 1 - 2 * (poseQ.y * poseQ.y + poseQ.z * poseQ.z);
    poseQuaternion.normalize();
    base_pose_goal.target_pose.pose.position.x = poseP.x + distance * poseQuaternion.getX();
    base_pose_goal.target_pose.pose.position.y = poseP.y + distance * poseQuaternion.getY();
    //base_pose_goal.target_pose.pose.position.z = poseP.z + distance * poseQuaternion.getZ();


    //poseQuaternion = poseQuaternion * distance;
    // Vector_subtract.pose.position.x = -distance;
    // Vector_subtract.pose.position.y = 0;
    // Vector_subtract.pose.position.z = 0;
    // Vector_subtract.pose.orientation.x = 0;
    // Vector_subtract.pose.orientation.y = 0;
    // Vector_subtract.pose.orientation.z = 0;
    // Vector_subtract.pose.orientation.w = 1;

    // q1 quaternion for transformation from marker frame to base_footprint
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
    

    tf2::Quaternion q(x2,y2,z2,r2);
    tf2::Matrix3x3 m_FromMaptoMarker(q);
    tf2::Matrix3x3 m_FromMarkertoMap=m_FromMaptoMarker.inverse();    
    tf2::Vector3 vector(0,0,-0.5);
    tf2::Vector3 raw1,raw2,raw3,vector1;
    raw1 = m_FromMarkertoMap.getRow(0);
    raw2 = m_FromMarkertoMap.getRow(1);
    raw3 = m_FromMarkertoMap.getRow(2);
    
    Vector_subtract.pose.position.x = raw1.getX()*vector.getX()+raw1.getY()*vector.getY()+raw1.getZ()*vector.getZ();
    Vector_subtract.pose.position.y = raw2.getX()*vector.getX()+raw2.getY()*vector.getY()+raw2.getZ()*vector.getZ();
    Vector_subtract.pose.position.z = raw3.getX()*vector.getX()+raw3.getY()*vector.getY()+raw3.getZ()*vector.getZ();


    base_pose_goal.target_pose.pose.position.x = pose_goal.pose.position.x - Vector_subtract.pose.position.x ; 
    base_pose_goal.target_pose.pose.position.y = pose_goal.pose.position.y - Vector_subtract.pose.position.y ;
    base_pose_goal.target_pose.pose.orientation.z = r2 * z1 + x2 * y1 - y2 * x1 + z2 * r1; // z component
    base_pose_goal.target_pose.pose.orientation.w = r2 * r1 - x2 * x1 - y2 * y1 - z2 * z1; // r component

    ROS_INFO("Pose goal: x=%1.3f y=%1.3f, Goal pose: x=%1.3f y=%1.3f",pose_goal.pose.position.x,pose_goal.pose.position.y,base_pose_goal.target_pose.pose.position.x,base_pose_goal.target_pose.pose.position.y);

    }


bool fIsMapOccupied(nav_msgs::OccupancyGrid& pMap, geometry_msgs::PoseStamped& pPose)
    {
    nav_msgs::MapMetaData info = pMap.info;
    float x = pPose.pose.position.x;
    float y = pPose.pose.position.y;
    int indexX = floor(x / info.resolution);

    int indexY = floor(y / info.resolution);
    int indexY2 = indexY * info.width;
    int index = indexX + indexY2;
    if (pMap.data.at(index) > 20)
        return true;
    return false;
    }