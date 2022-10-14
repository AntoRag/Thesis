#!/usr/bin/env python
from numpy import True_
import moveit_commander
from std_msgs.msg import Int64
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import rospy
from bondpy import bondpy
import sys
import os
os.environ['ROS_NAMESPACE'] = 'locobot'


# //ARM STATUS MACRO
ARM_FAIL = 0
ARM_IDLE = 1
ARM_RUNNING = 2
ARM_SUCCESS = 3
PICK = 0
PLACE = 1
GRIPPER_OPEN = 1
GRIPPER_CLOSE = 0

# //PICK AND PLACE MACR
PICK = 0
PLACE = 1

current_arm_status = Int64()
gripper_command = Int64()
current_arm_status.data = ARM_IDLE
pick_place = Int64()


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
    box_pose.pose.orientation.x = 0.00001
    box_pose.pose.orientation.y = 0.00001
    box_pose.pose.orientation.z = 0.00001
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.z = target_pose.pose.position.z
    box_pose.pose.position.x = target_pose.pose.position.x + 0.02
    box_pose.pose.position.y = target_pose.pose.position.y
    scene.add_box(box_name, box_pose, size=(0.04, 0.04, 0.07))
    success = ObjectInScene(scene, box_name, False, True)
    if not success:
        rospy.logerr('Not added any box')
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
        rospy.logerr('Not added any box')
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
    pose_goal.orientation.x = 1e-6
    pose_goal.orientation.y = 1e-6
    pose_goal.orientation.z = 1e-6
    pose_goal.orientation.w = 1
    pose_goal.position.x = target_pose.pose.position.x
    pose_goal.position.y = target_pose.pose.position.y
    pose_goal.position.z = target_pose.pose.position.z
    rospy.loginfo("Setting target pose")
    move_group.set_pose_target(pose_goal)
    rospy.loginfo("Moving the arm")
    success = move_group.go(wait=True)
    if success:
        rospy.loginfo("Moved correctly")
    else:
        rospy.loginfo("Problem during motion")
        fArmFail()
    rospy.loginfo("Stop any residual motion")
    move_group.stop()

    rospy.loginfo("Clear pose target")
    move_group.clear_pose_targets()
    current_pose = move_group.get_current_pose().pose
    return success


def dummySuccess():
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



def GraspCallback(pose_goal):
    global bond_open, bond_close, robot, move_group_arm, arm_status_pub, gripper_command_pub, scene
    # setting the arm running to avoid other callbacks
    current_arm_status.data = ARM_RUNNING
    arm_status_pub.publish(current_arm_status)
    rospy.loginfo("Arm currently running")  # log when running
    return dummySuccess()
    if pick_place == PICK:

        # then we proceed by approaching the object defining a pre_grasp_pose
        pre_grasp_pose = PoseStamped()
        pre_grasp_pose = pose_goal
        pre_grasp_pose.pose.position.x = pre_grasp_pose.pose.position.x - \
            0.1  # we arrive 10 cm far from the goal position
        # actuate the motion
        if (go_to_pose_goal(move_group_arm, pre_grasp_pose) == False):
            return

        # Wait for OctoMap update
        rospy.loginfo("Waiting for Octomap update")
        rospy.sleep(10)

        # then we open the gripper
        gripper_command.data = GRIPPER_OPEN
        gripper_command_pub.publish(gripper_command)
        bond_open.start()
        if not bond_open.wait_until_formed(rospy.Duration(10.0)):
            current_arm_status.data = ARM_FAIL
            arm_status_pub.publish(current_arm_status)
            raise Exception('Bond could not be formed')
        bond_open.wait_until_broken()
        rospy.loginfo("Gripped Opened")

        # first we add the box to be grasped to the planning scene
        pose_goal.pose.position.x = pose_goal.pose.position.x + 0.08
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
        gripper_command.data = GRIPPER_CLOSE
        gripper_command_pub.publish(gripper_command)
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
        retraction_pose.pose.position.x = retraction_pose.pose.position.x - \
            0.1  # we go 10 cm far from the goal position
        # actuate the motion
        if (go_to_pose_goal(move_group_arm, retraction_pose) == False):
            return

        current_arm_status.data = ARM_SUCCESS
        arm_status_pub.publish(current_arm_status)

    elif pick_place == PLACE:

        # We go into the place position
        place_pose = PoseStamped()
        # actuate the motion
        go_to_pose_goal(move_group_arm, place_pose)

        # then we open the gripper
        gripper_command.data = GRIPPER_OPEN
        gripper_command_pub.publish(gripper_command)
        bond_open.start()
        if not bond_open.wait_until_formed(rospy.Duration(10.0)):
            current_arm_status.data = ARM_FAIL
            arm_status_pub.publish(current_arm_status)
            raise Exception('Bond could not be formed')
        bond_open.wait_until_broken()
        rospy.loginfo("Gripped Opened")

        # now we add the box to the end effector link
        detach_box(scene)

        # going into retraction pose
        retraction_pose = PoseStamped()
        retraction_pose = pose_goal
        retraction_pose.pose.position.x = retraction_pose.pose.position.x - \
            0.1  # we go 10 cm far from the goal position
        # actuate the motion
        go_to_pose_goal(move_group_arm, retraction_pose)

        # we remove the object from the scene
        remove_box(scene)

        # then we close the gripper
        gripper_command.data = GRIPPER_CLOSE
        gripper_command_pub.publish(gripper_command)
        bond_close.start()
        if not bond_close.wait_until_formed(rospy.Duration(10.0)):
            current_arm_status.data = ARM_FAIL
            arm_status_pub.publish(current_arm_status)
            raise Exception('Bond could not be formed')
        bond_close.wait_until_broken()
        rospy.loginfo("Gripped Closed")

        current_arm_status.data = ARM_SUCCESS
        arm_status_pub.publish(current_arm_status)
        rospy.sleep(5)
        current_arm_status.data = ARM_IDLE
        arm_status_pub.publish(current_arm_status)
    else:
        rospy.ERROR("Error in giving command to pick or place")


def PickPlaceCallback(data):
    global pick_place
    pick_place = data.data
    return


def listener():
    global bond_close, bond_open, move_group_arm, robot, arm_status_pub, gripper_command_pub, scene
    rospy.init_node('arm_controller')
    rospy.Subscriber("/locobot/frodo/pick_or_place", Int64, PickPlaceCallback)
    rospy.Subscriber("/locobot/frodo/grasp_pose_goal",
                     PoseStamped, GraspCallback)
    # initialize the communication with the moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.sleep(5)
    arm_name = "interbotix_arm"  # define the planning interface for the arm
    move_group_arm = moveit_commander.MoveGroupCommander(
        arm_name, robot_description="/locobot/robot_description")
    move_group_arm.allow_replanning(True)
    move_group_arm.set_num_planning_attempts(10)
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
    rospy.spin()


if __name__ == '__main__':
    listener()

