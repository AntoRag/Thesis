#!/usr/bin/env python
# ----------- IMPORTS ----------- #
import sys
import os
from tkinter import CURRENT
os.environ['ROS_NAMESPACE'] = 'locobot'
import moveit_commander
from std_msgs.msg import Int64
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import rospy
from bondpy import bondpy
from std_srvs.srv import Empty

# ----------- FUNCTONS ----------- #
def ObjectInScene(scene, box_name, box_is_attached, box_is_known):
    timeout = 10  # timeout in seconds before error
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
        # Test if the box is in attached objects
        attached_objects = scene.get_attached_objects([box_name])
        is_attached = len(attached_objects.keys()) > 0

        # Test if the box is in the scene.
        # Note that attaching the box will remove it from known_objects
        is_known = box_name in scene.get_known_object_names()

        # Test if we are in the expected state
        if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True

        # Sleep so that we give other threads time on the processor
        rospy.sleep(0.1)
        seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False

def add_box(target_pose, scene):

    box_name = "medicine"
    box_pose = PoseStamped()
    box_pose.header.frame_id = "locobot/base_footprint"
    box_pose.pose.orientation.x = target_pose.pose.orientation.x
    box_pose.pose.orientation.y = target_pose.pose.orientation.y
    box_pose.pose.orientation.z = target_pose.pose.orientation.z
    box_pose.pose.orientation.w = target_pose.pose.orientation.w
    box_pose.pose.position.z = target_pose.pose.position.z
    box_pose.pose.position.x = target_pose.pose.position.x
    box_pose.pose.position.y = target_pose.pose.position.y
    # scene.add_box(box_name, box_pose, size=(0.02, 0.065, 0.125))
    scene.add_box(box_name, box_pose, size=(0.02, 0.03, 0.125))
    success = ObjectInScene(scene, box_name, False, True)
    if not success:
        rospy.logerr("[CORE::ARM_CONTROLLER] ---- NOT ADDED ANY BOX")
        fArmFail()
        return False
    rospy.sleep(5)

def attach_box(scene):
    box_name = 'medicine'
    eef_link = 'locobot/ee_gripper_link'
    touch_links = ['locobot/ee_gripper_link', 'locobot/left_finger_link',
                   'locobot/right_finger_link', 'locobot/fingers_link']
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    success = ObjectInScene(scene, box_name, True, False)
    if not success:
        rospy.logerr("[CORE::ARM_CONTROLLER] ---- NOT ATTACHED ANY BOX")
        fArmFail()
        return False
    rospy.sleep(5)

def detach_box(scene):
    box_name = 'medicine'
    eef_link = 'locobot/ee_gripper_link'
    scene.remove_attached_object(eef_link, name=box_name)
    rospy.sleep(5)

def remove_box(scene):
    box_name = 'medicine'
    scene.remove_world_object(box_name)
    rospy.sleep(5)

def go_to_pose_goal(move_group, target_pose):
    pose_goal = Pose()
    pose_goal.orientation.x = target_pose.pose.orientation.x
    pose_goal.orientation.y = target_pose.pose.orientation.y
    pose_goal.orientation.z = target_pose.pose.orientation.z
    pose_goal.orientation.w = target_pose.pose.orientation.w
    pose_goal.position.x = target_pose.pose.position.x
    pose_goal.position.y = target_pose.pose.position.y
    pose_goal.position.z = target_pose.pose.position.z
    # rospy.loginfo("Setting target pose")
    move_group.set_pose_target(pose_goal)
    rospy.loginfo("[CORE::ARM_CONTROLLER] ---- MOVING THE ARM...")
    rospy.loginfo("[CORE::ARM_CONTROLLER] ---- TARGET POS: X=%1.3f, Y=%1.3f, Z=%1.3f",pose_goal.position.x,pose_goal.position.y,pose_goal.position.z)
    success = move_group.go(wait=True)
    move_group.stop()
    # rospy.loginfo("Clear pose target")
    move_group.clear_pose_targets()
    if success:
        rospy.loginfo("[CORE::ARM_CONTROLLER] ---- ARM MOVED CORRECTLY")
    else:
        rospy.loginfo("[CORE::ARM_CONTROLLER] ---- PROBLEM DURING MOTION")
        fArmFail()
    # rospy.loginfo("Stop any residual motion")
    # current_pose = move_group.get_current_pose().pose
    return success

def fArmSuccess():
    current_arm_status.data = ARM_SUCCESS
    arm_status_pub.publish(current_arm_status)
    rospy.sleep(5)
    current_arm_status.data = ARM_IDLE
    arm_status_pub.publish(current_arm_status)

def fArmFail():
    global arm_status_pub
    current_arm_status.data = ARM_FAIL
    arm_status_pub.publish(current_arm_status)
    rospy.sleep(5)
    current_arm_status.data = ARM_IDLE
    arm_status_pub.publish(current_arm_status)  

def goHome(move_group):
    joint_goal = move_group.get_current_joint_values()
    joint_goal = [0, -0.92, 1.5, -0.0145, -0.62, 0]
    move_group.go(joint_goal, wait=True)
    rospy.loginfo("[CORE::ARM_CONTROLLER] ---- HOME POSITION")
    move_group.stop()

def closeGripper(move_group_gripper):
    joint_goal_close = move_group_gripper.get_current_joint_values()
    print("joint status",joint_goal_close[0]," ",joint_goal_close[1])
    joint_goal_close[0] = 0.03
    joint_goal_close[1] = -0.03

    joint_goal_close[0] = 0.015
    joint_goal_close[1] = -0.015
    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group_gripper.go(joint_goal_close, wait=True)
    rospy.loginfo("[CORE::ARM_CONTROLLER] ---- GRIPPER CLOSED")

def openGripper(move_group_gripper):
    joint_goal = move_group_gripper.get_current_joint_values()
    print("joint status",joint_goal[0]," ",joint_goal[1])
    joint_goal[0] = 0.035
    joint_goal[1] = -0.035
    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group_gripper.go(joint_goal, wait=True)
    rospy.loginfo("[CORE::ARM_CONTROLLER] ---- GRIPPER OPENED")

# ----------- CONSTANTS ----------- #
# ARM STATUS MACRO
ARM_FAIL = 0
ARM_IDLE = 1
ARM_RUNNING = 2
ARM_SUCCESS = 3
# GRIPPER MACRO
GRIPPER_OPEN = 1
GRIPPER_CLOSE = 0
# PICK AND PLACE MACRO
PICK = 0
PLACE = 1
# GLOBAL VARIABLES
current_arm_status = Int64()
gripper_command = Int64()
pick_place = Int64()

# ----------- CALLBACKS NODE -----------#
def GraspCallback(pose_goal):
    global bond_open, bond_close, robot, move_group_arm, arm_status_pub, gripper_command_pub, scene, current_arm_status,service_octomap,move_group_gripper
    # setting the arm running to avoid other callbacks
    current_arm_status.data = ARM_RUNNING
    arm_status_pub.publish(current_arm_status)
    rospy.loginfo("[CORE::ARM_CONTROLLER] ---- ARM RUNNNING")  # log when running
    rospy.sleep(5)
    #return fArmSuccess()
    #return fArmFail()
    if pick_place == PICK:
        rospy.loginfo("[CORE::ARM_CONTROLLER] ---- PICK TASK...")
        service_octomap()
        # Wait for OctoMap update
        rospy.loginfo("[CORE::ARM_CONTROLLER] ---- WAITING FOR OCTOMAP")
        rospy.sleep(10)
        # then we proceed by approaching the object defining a pre_grasp_pose
        pre_grasp_pose = PoseStamped()
        pre_grasp_pose = pose_goal
        pre_grasp_pose.pose.position.x = pre_grasp_pose.pose.position.x - \
            0.1  # we arrive 10 cm far from the goal position
        # actuate the motion
        if (go_to_pose_goal(move_group_arm, pre_grasp_pose) == False):
            return

        # Wait for OctoMap update
        rospy.loginfo("[CORE::ARM_CONTROLLER] ---- WAITING FOR OCTOMAP")
        rospy.sleep(10)

        # then we open the gripper
        openGripper(move_group_gripper)
        # gripper_command.data = GRIPPER_OPEN
        # gripper_command_pub.publish(gripper_command)
        # bond_open.start()
        # if not bond_open.wait_until_formed(rospy.Duration(10.0)):
        #     fArmFail()
        #     raise Exception('Bond could not be formed')
        # bond_open.wait_until_broken()
        #rospy.loginfo("Gripped Opened")

        # first we add the box to be grasped to the planning scene
        pose_goal.pose.position.x = pose_goal.pose.position.x
        if (add_box(pose_goal, scene) == False):
            return

        # then we approach the object
        pose_goal.pose.position.x = pose_goal.pose.position.x
        if (go_to_pose_goal(move_group_arm, pose_goal) == False):
            return

        # now we add the box to the end effector link
        if (attach_box(scene) == False):
            return

        # then we close the gripper
        closeGripper(move_group_gripper)
        # gripper_command.data = GRIPPER_CLOSE
        # gripper_command_pub.publish(gripper_command)
        # bond_close.start()
        # if not bond_close.wait_until_formed(rospy.Duration(10.0)):
        #     fArmFail()
        #     raise Exception('Bond could not be formed')

        # bond_close.wait_until_broken()
        #rospy.loginfo("Gripped Closed")

        # going into retraction pose
        retraction_pose = PoseStamped()
        retraction_pose = pose_goal
        retraction_pose.pose.position.x = retraction_pose.pose.position.x - \
            0.1  # we go 10 cm far from the goal position
        # actuate the motion
        if (go_to_pose_goal(move_group_arm, retraction_pose) == False):
            return
        goHome(move_group_arm)
        fArmSuccess()

    elif pick_place == PLACE:
        rospy.loginfo("[CORE::ARM_CONTROLLER] ---- PLACE TASK...")
        service_octomap()
        # Wait for OctoMap update
        rospy.loginfo("[CORE::ARM_CONTROLLER] ---- WAITING FOR OCTOMAP")
        rospy.sleep(10)
        # We go into the place position
        # actuate the motion
        go_to_pose_goal(move_group_arm, pose_goal)

        # then we open the gripper
        openGripper(move_group_gripper)
        # gripper_command.data = GRIPPER_OPEN
        # gripper_command_pub.publish(gripper_command)
        # bond_open.start()
        # if not bond_open.wait_until_formed(rospy.Duration(10.0)):
        #     fArmFail()
        #     raise Exception('Bond could not be formed')
        # bond_open.wait_until_broken()
        #rospy.loginfo("Gripped Opened")

        # now we add the box to the end effector link
        detach_box(scene)

        # going into retraction pose
        goHome(move_group_arm)

        # we remove the object from the scene
        remove_box(scene)

        # then we close the gripper
        closeGripper(move_group_gripper)
        # gripper_command.data = GRIPPER_CLOSE
        # gripper_command_pub.publish(gripper_command)
        # bond_close.start()
        # if not bond_close.wait_until_formed(rospy.Duration(10.0)):
        #     fArmFail()
        #     raise Exception('Bond could not be formed')
        # bond_close.wait_until_broken()
        fArmSuccess()

    else:
        rospy.ERROR("[CORE::GRIPPER_CONTROLLER] ---- ERROR IN PICK/PLACE COMMAND")



def PickPlaceCallback(data):
    global pick_place
    pick_place = data.data
    return


def listener():
    global bond_close, bond_open, move_group_arm, robot, arm_status_pub, gripper_command_pub, scene,current_arm_status, service_octomap, move_group_gripper
     # initialize the communication with the moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm_controller')
    current_arm_status.data = ARM_IDLE
    rospy.Subscriber("/locobot/frodo/pick_or_place", Int64, PickPlaceCallback)
    rospy.Subscriber("/locobot/frodo/grasp_pose_goal",
                     PoseStamped, GraspCallback)
    rospy.sleep(5)
    gripper_name = "interbotix_gripper"
    move_group_gripper = moveit_commander.MoveGroupCommander(gripper_name,robot_description="/locobot/robot_description")
    rospy.sleep(5)
    arm_name = "interbotix_arm"  # define the planning interface for the arm
    move_group_arm = moveit_commander.MoveGroupCommander(
        arm_name, robot_description="/locobot/robot_description")

    move_group_arm.allow_replanning(True)
    move_group_arm.set_num_planning_attempts(10)
    tolerance = 0.1
    move_group_arm.set_goal_position_tolerance(tolerance)
    # get some parameters for the arm and the scene
    robot = moveit_commander.RobotCommander(
        robot_description="/locobot/robot_description")
    scene = moveit_commander.PlanningSceneInterface()

    # initialize the bond used for synchronizing
    # the opening and the close of the gripper
    bond_open = bondpy.Bond("/locobot/open_gripper", "opengripper")
    bond_close = bondpy.Bond("/locobot/close_gripper", "closegripper")
    # define the topic used by the arm to communicate its status: running, idle or fail
    arm_status_pub = rospy.Publisher(
        '/locobot/frodo/arm_status', Int64, queue_size=1)
    # define the topic used by the arm to communicate whenever close or open the gripper
    gripper_command_pub = rospy.Publisher(
        '/locobot/frodo/gripper_command', Int64, queue_size=1)
    rospy.wait_for_service('/locobot/clear_octomap')
    service_octomap = rospy.ServiceProxy('/locobot/clear_octomap', Empty)
    goHome(move_group_arm)
    closeGripper(move_group_gripper)
    rospy.spin()


if __name__ == '__main__':
    listener()

