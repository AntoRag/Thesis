#!/usr/bin/env python
import sys
import os
os.environ['ROS_NAMESPACE'] = 'locobot'
from bondpy import bondpy
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Int64
from std_msgs.msg import String
import moveit_commander


# //ARM STATUS MACRO
ARM_FAIL = 0
ARM_IDLE = 1
ARM_RUNNING = 2
ARM_SUCCESS = 3
PICK = 0
PLACE = 1

# //PICK AND PLACE MACR
PICK = 0
PLACE = 1

current_arm_status = Int64()
current_arm_status.data = ARM_IDLE


def add_box(target_pose, scene):

    box_name = "medicine"
    box_pose = PoseStamped()
    box_pose.header.frame_id = "locobot/base_footprint"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.z = target_pose.pose.position.z
    box_pose.pose.position.x = target_pose.pose.position.x+0.03
    box_pose.pose.position.y = target_pose.pose.position.y
    scene.add_box(box_name, box_pose, size=(0.04, 0.04, 0.06))
    rospy.sleep(5)
    return


def attach_box(robot, scene, move_group):
    box_name = 'medicine'
    eef_link = move_group.get_end_effector_link()
    grasping_group = 'interbotix_gripper'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    rospy.sleep(5)
    return


def detach_box(scene, move_group):
    box_name = 'medicine'
    eef_link = move_group.get_end_effector_link()
    scene.remove_attached_object(eef_link, name=box_name)
    rospy.sleep(5)
    return


def remove_box(scene):
    box_name = 'medicine'
    scene.remove_world_object(box_name)
    rospy.sleep(5)
    return


def go_to_pose_goal(move_group, target_pose):
    pose_goal = Pose()
    pose_goal.orientation.w = 1
    pose_goal.position.x = target_pose.pose.position.x
    pose_goal.position.y = target_pose.pose.position.y
    pose_goal.position.z = target_pose.pose.position.z
    rospy.loginfo("Setting target pose")
    rospy.sleep(1)
    move_group.set_pose_target(pose_goal)
    rospy.loginfo("Moving the arm")
    success = move_group.go(wait=True)
    if success:
        rospy.loginfo("Moved correctly")
    else:
        rospy.loginfo("Problem during motion")
    rospy.sleep(1)
    rospy.loginfo("Stop any residual motion")
    move_group.stop()
    rospy.sleep(1)
    rospy.loginfo("Clear pose target")
    move_group.clear_pose_targets()
    rospy.sleep(1)
    current_pose = move_group.get_current_pose().pose
    rospy.sleep(5)
    return


pick_place = Int64()

def GraspCallback(pose_goal):

    # define the topic used by the arm to communicate its status: running, idle or fail
    # define the topic used by the arm to communicate whenever close or open the gripper
    arm_status_pub = rospy.Publisher('locobot/frodo/arm_status', Int64, queue_size=1)
    gripper_command_pub = rospy.Publisher('locobot/frodo/gripper_command', String, queue_size=1)

    # setting the arm running to avoid other callbacks
    current_arm_status.data = ARM_RUNNING
    arm_status_pub.publish(current_arm_status)
    rospy.loginfo("Arm currently running")  # log when running

    # initialize the communication with the moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)
    arm_name = "interbotix_arm"  # define the planning interface for the arm
    move_group_arm = moveit_commander.MoveGroupCommander(arm_name, robot_description="locobot/robot_description")

    # get some parameters for the arm and the scene
    robot = moveit_commander.RobotCommander(robot_description="/locobot/robot_description")
    scene = moveit_commander.PlanningSceneInterface()

    # initialize the bond used for synchronizing
    # the opening and the close of the gripper
    bond_open = bondpy.Bond("/locobot/open_gripper", "opengripper")
    bond_close = bondpy.Bond("/locobot/close_gripper", "closegripper")
    bond_pick_arm = bondpy.Bond("/locobot/pick_arm", "PickArm")
    bond_place_arm = bondpy.Bond("/locobot/place_arm", "PlaceArm")

    if pick_place == PICK:

        bond_pick_arm.start()

        # first we add the box to be grasped to the planning scene
        add_box(pose_goal, scene)

        # then we proceed by approaching the object defining a pre_grasp_pose
        pre_grasp_pose = PoseStamped()
        pre_grasp_pose = pose_goal
        pre_grasp_pose.pose.position.x = pre_grasp_pose.pose.position.x - 0.05  # we arrive 5 cm far from the goal position
        # actuate the motion
        go_to_pose_goal(move_group_arm, pre_grasp_pose)

        # then we open the gripper
        gripper_command_pub.publish('Open')
        bond_open.start()
        if not bond_open.wait_until_formed(rospy.Duration(10.0)):
            current_arm_status.data = ARM_FAIL
            arm_status_pub.publish(current_arm_status)
            raise Exception('Bond could not be formed')
        bond_open.wait_until_broken()
        rospy.loginfo("Gripped Opened")

        # then we approach the object
        go_to_pose_goal(move_group_arm, pose_goal)

        # now we add the box to the end effector link
        attach_box(robot, scene, move_group_arm)

        # then we close the gripper
        gripper_command_pub.publish('Close')
        bond_close.start()
        if not bond_close.wait_until_formed(rospy.Duration(10.0)):
            current_arm_status.data = ARM_FAIL
            arm_status_pub.publish(current_arm_status)
            raise Exception('Bond could not be formed')
            
        bond_close.wait_until_broken()
        rospy.loginfo("Gripped Closed")

        # going into retraction pose
        retraction_pose = PoseStamped()
        retraction_pose = pose_goal
        retraction_pose.pose.position.x = retraction_pose.pose.position.x - 0.1  # we go 10 cm far from the goal position
        # actuate the motion
        go_to_pose_goal(move_group_arm, retraction_pose)

        current_arm_status.data = ARM_SUCCESS
        arm_status_pub.publish(current_arm_status)
        bond_pick_arm.break_bond()

    elif pick_place == PLACE:

        bond_place_arm.start()

        # We go into the place position
        place_pose = PoseStamped()
        # actuate the motion
        go_to_pose_goal(move_group_arm, place_pose)

        # then we open the gripper
        gripper_command_pub.publish('Open')
        bond_open.start()
        if not bond_open.wait_until_formed(rospy.Duration(10.0)):
            current_arm_status.data = ARM_FAIL
            arm_status_pub.publish(current_arm_status)
            raise Exception('Bond could not be formed')
        bond_open.wait_until_broken()
        rospy.loginfo("Gripped Opened")

        # now we add the box to the end effector link
        detach_box(scene, move_group_arm)

        # going into retraction pose
        retraction_pose = PoseStamped()
        retraction_pose = pose_goal
        retraction_pose.pose.position.x = retraction_pose.pose.position.x - 0.1  # we go 10 cm far from the goal position
        # actuate the motion
        go_to_pose_goal(move_group_arm, retraction_pose)

        # we remove the object from the scene
        remove_box(scene)

        # then we close the gripper
        gripper_command_pub.publish("Close")
        bond_close.start()
        if not bond_close.wait_until_formed(rospy.Duration(10.0)):
            current_arm_status.data = ARM_FAIL
            arm_status_pub.publish(current_arm_status)
            raise Exception('Bond could not be formed')
        bond_close.wait_until_broken()
        rospy.loginfo("Gripped Closed")

        current_arm_status.data = ARM_IDLE
        arm_status_pub.publish(current_arm_status)
        bond_place_arm.break_bond()

    else:
        rospy.ERROR("Error in giving command to pick or place")


def PickPlaceCallback(data):
    global pick_place
    pick_place = data.data
    return



def listener():

    rospy.init_node('arm_controller')
    rospy.Subscriber("/locobot/frodo/pick_or_place", Int64, PickPlaceCallback)
    rospy.Subscriber("/locobot/frodo/grasp_pose_goal",PoseStamped, GraspCallback)
    rospy.spin()


if __name__ == '__main__':
    listener()
