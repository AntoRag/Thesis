#!/usr/bin/env python

import sys
import copy
import rospy
import random
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
#
# from six.moves import input
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
from moveit_commander.conversions import pose_to_list
import numpy as np

joint_state_topic = ['joint_states:=/locobot/joint_states']
moveit_commander.roscpp_initialize(joint_state_topic)
rospy.init_node('moveit_python_interface')
robot = moveit_commander.RobotCommander('/locobot/robot_description')
group_name = "interbotix_arm"
group = moveit_commander.MoveGroupCommander(group_name,robot_description='/locobot/robot_description',ns='/locobot')
display_trajectory_publisher = rospy.Publisher("/locobot/move_group/display_planned_path",
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
print(group.get_current_joint_values())

## Getting Basic Information
## ^^^^^^^^^^^^^^^^^^^^^^^^^
# We can get the name of the reference frame for this robot:
planning_frame = group.get_planning_frame()
print("============ Reference frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
print("============ End effector: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Robot Groups: " + str(group_names))
print("")
print("============ Printing robot state")
print(robot.get_current_state())
print("")
def go_to_joint_state(joint_goal):
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    global group
    print("============ Printing Joint Goal: " + str(joint_goal))

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    plan = group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    current_joints = group.get_current_joint_values()
    return plan

def go_to_pose_goal(pose_goal):
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    global group
    print("============ Printing Pose Goal:\n" + str(pose_goal))
    group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    current_pose = group.get_current_pose().pose
    return plan

def main():
    print("============ Press `Enter` to begin the test by setting up the moveit_commander (press ctrl-d to exit) ...")
    input()

    pose1 = [   0.375291195719765,
                0.164125227476363,
                0.137546262683140,
                0,
                0,
                0]
    angles1 = [0.2, 0.8, 0.3, 2.5, 2, 2.57]
    pose2 = [   0.355309117038622,
                -0.272767318119011,
                0.413831169558669, 
                -0.505880892091619,  
                0.564605033327691,
                -0.454673623730534]
    angles2 = [-0.7386040584516571, 0.3753509366387695, -1.1433557625060344, 0, 0, 0]
    pose3 = [0.076440280064513,   
            0.431202835440426,   
            0.090938520928812, 
            0,   
            0,  
            0]
    angles3 = [1.6600778277779833, 1.0585292544065141, -1.0029056563377778, 0, 0, 0]
    pose4 = [-0.329682911463161,  
            -0.345931257202446,   
            0.170237835919068, 
            0,   
            0,   
            0]
    angles4 = [-2.191600726742686, 0.431530487593379, -0.3363234983674577, 0, 0, 0]
    pose5 = [0.1721,
            -0.3621,
            0.0653, 
            0,
            0,
            0]

    angles5 = [-1.4663662740970747, 1.0722627411112038, -0.09853363686700903, 0, 0, 0]

    pose_target = [pose1,pose2,pose3,pose4,pose5]
    joint_target=[angles1,angles2,angles3,angles4,angles5]
    i = 0
    while i<6:
        #plan = go_to_joint_state(joint_goal=joint_target[i])
        plan = go_to_pose_goal(pose_goal=pose_target[i])
        if plan:
            print("Successfully planned the path!")
        else:
            print("No feasible path found")
        i=i+1
    rospy.spin()

if __name__ == '__main__':
  main()

