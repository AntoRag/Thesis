#!/usr/bin/env python

import sys
import copy
import os
os.environ['ROS_NAMESPACE'] = 'locobot'
# Must set `os.environ['ROS_NAMESPACE']` BEFORE importing `rospy`
from bondpy import bondpy
import rospy
import moveit_commander  
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


id = bondpy.generate_unique_id()

def add_box(target_pose,scene):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = "medicine"
    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene between the fingers:
    box_pose = PoseStamped()
    box_pose.header.frame_id = "locobot/base_footprint"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.z = target_pose.pose.position.z   # above the panda_hand frame
    box_pose.pose.position.x = target_pose.pose.position.x-0.03 
    box_pose.pose.position.y = target_pose.pose.position.y
    scene.add_box(box_name, box_pose, size=(0.06, 0.06, 0.06))
    rospy.sleep(5)


def gripper_status_callback(status):
    global gripper_status_bool
    if status.data == 'Finished':
        gripper_status_bool = 0
    elif status.data == 'Moving':
        gripper_status_bool = 1
        
    


def go_to_pose_goal(move_group,target_pose):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = Pose()
    pose_goal.orientation.w = 1
    pose_goal.position.x = 0.4*target_pose.pose.position.x
    pose_goal.position.y = 0.4*target_pose.pose.position.y
    pose_goal.position.z = target_pose.pose.position.z
    rospy.loginfo("Setting target pose")
    rospy.sleep(1)
    move_group.set_pose_target(pose_goal)
    rospy.loginfo("Moving the arm")
    ## Now, we call the planner to compute the plan and execute it.
    # `go()` returns a boolean indicating whether the planning and execution was successful.
    success = move_group.go(wait=True)
    if success:
        rospy.loginfo("Moved correctly")
    else :
        rospy.loginfo("Problem during motion")
    rospy.sleep(1)
    # Calling `stop()` ensures that there is no residual movement
    rospy.loginfo("Stop any residual motion")
    move_group.stop()
    rospy.sleep(1)
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets().
    rospy.loginfo("Clear pose target")
    move_group.clear_pose_targets()
    rospy.sleep(1)    
    ## END_SUB_TUTORIAL
    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = move_group.get_current_pose().pose
    return
    

def move_base_client(target):
    
    q1 = PoseStamped()
    q1.pose.orientation.w = 0.5
    q1.pose.orientation.x = 0.5
    q1.pose.orientation.y = 0.5
    q1.pose.orientation.z = -0.5
    # First quaternion q1 (x1 y1 z1 r1)
    x1 = q1.pose.orientation.x
    y1 = q1.pose.orientation.y
    z1 = q1.pose.orientation.z
    r1 = q1.pose.orientation.w

    # Second quaternion q2 (x2 y2 z2 r2)
    x2 = target.pose.orientation.x
    y2 = target.pose.orientation.y
    z2 = target.pose.orientation.z
    r2 = target.pose.orientation.w    
    
    client = actionlib.SimpleActionClient('/locobot/move_base',MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 0.6*target.pose.position.x
    goal.target_pose.pose.position.y = 0.6*target.pose.position.y
    goal.target_pose.pose.orientation.z = r2 * z1 + x2 * y1 - y2 * x1 + z2 * r1
    goal.target_pose.pose.orientation.w = r2 * r1 - x2 * x1 - y2 * y1 - z2 * z1
    
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        success_navigation = client.get_result()

    if success_navigation:
        rospy.loginfo("Goal execution done!")
    else:
        rospy.logerr("Error in navigation")




def movement_callback(pose_goal):
    global gripper_status_bool
    robot_status_pub = rospy.Publisher('/locobot/robot_status', String, queue_size=1)
    gripper_command = rospy.Publisher('/locobot/gripper_command', String, queue_size=1)
    
    current_status = "running"
    robot_status_pub.publish(current_status)
    print("Robot currently running")
    
    moveit_commander.roscpp_initialize(sys.argv)
    #robot = moveit_commander.RobotCommander(robot_description="/locobot/robot_description")
    scene = moveit_commander.PlanningSceneInterface()
    arm_name = "interbotix_arm"
    move_group_arm = moveit_commander.MoveGroupCommander(arm_name,robot_description="locobot/robot_description")
    bond = bondpy.Bond("/locobot/status_gripper", id)
    
    move_base_client(pose_goal)

    add_box(pose_goal,scene)
    pre_grasp_pose = pose_goal
    pre_grasp_pose.pose.position.x = pre_grasp_pose.pose.position.x - 0.05
    go_to_pose_goal(move_group_arm,pre_grasp_pose)
    
    gripper_command.publish("Open")
    bond.start()
    if not bond.wait_until_formed(rospy.Duration(1.0)):
        raise Exception('Bond could not be formed')
    bond.wait_until_broken()
    rospy.loginfo("Gripped Opened")
    
    go_to_pose_goal(move_group_arm,pose_goal)

    gripper_command.publish("Close")
    bond.start()
    if not bond.wait_until_formed(rospy.Duration(1.0)):
        raise Exception('Bond could not be formed')
    bond.wait_until_broken()
    rospy.loginfo("Gripped Closed")
    
    
    current_status = "idle"
    robot_status_pub.publish(current_status)




def listener():

    rospy.init_node('movement_node')
    rospy.Subscriber("/locobot/pose_goal", PoseStamped , movement_callback)
    rospy.Subscriber("/locobot/gripper_status", String , gripper_status_callback)
    rospy.spin()


if __name__ == '__main__':
    listener()