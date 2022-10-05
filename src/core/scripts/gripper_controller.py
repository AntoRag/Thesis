#!/usr/bin/env python

import sys
import os
os.environ['ROS_NAMESPACE'] = 'locobot'
# Must set `os.environ['ROS_NAMESPACE']` BEFORE importing `rospy`
from bondpy import bondpy
import rospy
import moveit_commander  

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list



def gripper_callback(data):
    gripper_name = "interbotix_gripper"
    move_group_gripper = moveit_commander.MoveGroupCommander(gripper_name,robot_description="locobot/robot_description")
    
    bond_open = bondpy.Bond("/locobot/open_gripper","opengripper")
    bond_close = bondpy.Bond("/locobot/close_gripper","closegripper")
    if data.data == 'Open':
        bond_open.start()
        joint_goal = move_group_gripper.get_current_joint_values()
        print("joint status",joint_goal[0]," ",joint_goal[1])
        joint_goal[0] = 0.035
        joint_goal[1] = -0.035
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group_gripper.go(joint_goal, wait=True)
        rospy.loginfo("Gripper Opened")
        # Calling ``stop()`` ensures that there is no residual movement
        move_group_gripper.stop()
        ## END_SUB_TUTORIAL
        # For testing:
        bond_open.break_bond()
    elif data.data == 'Close':
        bond_close.start()
        joint_goal_close = move_group_gripper.get_current_joint_values()
        print("joint status",joint_goal_close[0]," ",joint_goal_close[1])
        joint_goal_close[0] = 0.015
        joint_goal_close[1] = -0.015
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group_gripper.go(joint_goal_close, wait=True)
        rospy.loginfo("Gripper Closed")
        # Calling ``stop()`` ensures that there is no residual movement
        move_group_gripper.stop()
        ## END_SUB_TUTORIAL
        # For testing:
        bond_close.break_bond()
    

def listener():

    rospy.init_node('gripper_controller')
    rospy.Subscriber("/locobot/frodo/gripper_command", String , gripper_callback)

    rospy.spin()


if __name__ == '__main__':
    listener()