
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

void fGetPoseFromMarker(geometry_msgs::PoseStamped& grasp_pose_goal, geometry_msgs::PoseStamped  markers_pose)
    {
    ROS_INFO("fGetPoseFromMaker, calculating...");
    grasp_pose_goal = markers_pose;
    }

void fcross_product(float vector_a[], float vector_b[], float temp[]) {
// a cross product b
   temp[0] = vector_a[1] * vector_b[2] - vector_a[2] * vector_b[1];
   temp[1] = -(vector_a[0] * vector_b[2] - vector_a[2] * vector_b[0]);
   temp[2] = vector_a[0] * vector_b[1] - vector_a[1] * vector_b[0];
}


void fMultiplyQuaternion(move_base_msgs::MoveBaseGoal& base_pose_goal, geometry_msgs::PoseStamped& pose_goal)
    {

    geometry_msgs::PoseStamped q1;
    geometry_msgs::PoseStamped q1_conj;
    geometry_msgs::PoseStamped Vector_subtract;



    Vector_subtract.pose.position.x = -0.5;
    Vector_subtract.pose.position.y = 0;
    Vector_subtract.pose.position.z = 0;
    Vector_subtract.pose.orientation.x = 0;
    Vector_subtract.pose.orientation.y = 0;
    Vector_subtract.pose.orientation.z = 0;
    Vector_subtract.pose.orientation.w = 1;

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
    // Conjugate second quaternion
    const float x2_conj = -pose_goal.pose.orientation.x;
    const float y2_conj = -pose_goal.pose.orientation.y;
    const float z2_conj = -pose_goal.pose.orientation.z;
    const float r2_conj = pose_goal.pose.orientation.w; 
    
    //Third quaternion
    float x3 = Vector_subtract.pose.position.x;
    float y3 = Vector_subtract.pose.position.y;
    float z3 = Vector_subtract.pose.position.z;
    
    float vecA[]={x2_conj,y2_conj,z2_conj};
    float vecB[]={x2,y2,z2};
    float vecC[]={x3,y3,z3};
    float temp[3];
    
    
    fcross_product(vecC,vecB,temp);
    fcross_product(vecA,temp,vecC);

    Vector_subtract.pose.position.x = vecC[1];
    Vector_subtract.pose.position.y = vecC[2];
    Vector_subtract.pose.position.z = vecC[3];


    base_pose_goal.target_pose.pose.position.x = base_pose_goal.target_pose.pose.position.x - Vector_subtract.pose.position.x;
    base_pose_goal.target_pose.pose.position.y = base_pose_goal.target_pose.pose.position.y - Vector_subtract.pose.position.y;
    // base_pose_goal.target_pose.pose.position.z = base_pose_goal.target_pose.pose.position.z - Vector_subtract.pose.position.z;

    base_pose_goal.target_pose.pose.orientation.z = r2 * z1 + x2 * y1 - y2 * x1 + z2 * r1; // z component
    base_pose_goal.target_pose.pose.orientation.w = r2 * r1 - x2 * x1 - y2 * y1 - z2 * z1; // r component
    // pose_goal.pose.orientation.x = x2 * r1 + r2 * x1 + y2 * z1 - z2 * y1;        // x component
    // pose_goal.pose.orientation.y = r2 * y1 - x2 * z1 + y2 * r1 + z2 * x1;        // y component
    // pose_goal.pose.orientation.z = r2 * z1 + x2 * y1 - y2 * x1 + z2 * r1;        // z component
    // pose_goal.pose.orientation.w = r2 * r1 - x2 * x1 - y2 * y1 - z2 * z1;        // r component
    }

