# Overview
The folder contains the software for the three nodes that composes the architecture of our solution. In particular here we have the "arm_controller" node, the "base_controller" node, and the "communication_node". 
The structure is composed of three nodes:

- Base controller: it is responsible for exchanging information with the communication manager about the working status of the hardware or the goal position's planning success or failure
- Arm controller: it is accountable for planning the motion of the servos to reach the desired pose. It also controls the opening or closure of the gripper by publishing to a specific topic. As for the base controller, the arm controller will also communicate the hardware working status to the communication manager
- Communication node: its task is to manage the ARTag present in the scene and the communication with base and arm. In particular, the node reads the IDs of the ARTags present in the scene and compares them with the ID request. When there is a match between a requested id and a marker present in the environment it will send the commands to start the pick/place routine. Along with these operations, it has a failure-handling mechanism

Essentially the goal of the communication node is to deal with a request for an id and manage the operations of the mobile base and the arm. This is achieved through the status topics and sending a pose goal to each component. For safety reasons, we decided to deal with each component individually, so the arm can run only if the base is idle and vice-versa.

## Item request handling
The first thing that has to be done when receiving a request for an id is to search if it is present in the robot's field of view. The software will listen to the ar_pose_marker topics which contain details such as pose and orientation characterizing each marker. To ensure that the arm and the mobile base have a correct reading at each time instant without having to perform any transformation online we set up two different ar_track_alvar nodes so that ones computes the positions for the mobile base (so with respect to "map" frame) and another one that estimates the position with respect to "locobot/base_footprint" which is the planning frame of MoveIt.
![How item requests are handled](/Media/idreq_alg.png "How item requests are handled")

## Search phase 
If an item is not found among all the available in the scene the robot enters a function that performs a series of predefined actions presented in the following Algorithm. The function has some parameters that can be tuned according to the application's necessities: we can tune the spots to be investigated and the degrees of rotation that the mobile base performs at each iteration.

![Sample of the search algorithm](/Media/search_algor.png "Sample of the search algorithm")


![Representation in the space of the mobile manipulator](/Media/search_phase.png "Representation in the space of the mobile manipulator")

# Arm controller
Now we can go in deep to analyze the role of the arm controller. It is constructed such that the node exchanges some information with the communication node. The topics used are the following: arm_status, pick_or_place, grasp_pose_goal, and pre_grasp_pose_goal. A general view of the communication structure is sketched in the following Figure.
![Arm controller software architecture](/Media/arm_comm.jpg "Arm controller software architecture")

## Description of the "arm_status" topic
The first topic that we will analyze is the arm_status topic. It is capable of communicating four status that characterizes the working flow of the arm:
- ARM FAIL: it is implemented to allow the replanning of the mobile base if the pick or place actions fail.
- ARM SUCCESS: this status is the one sent at the end of a pick/place action that is performed successfully. After 5 seconds the status is changed back to ARM IDLE.
- ARM IDLE: this is a necessary step to make the communication node aware that the arm completed its motion. Without this status published the base cannot move.
- ARM RUNNING: this message is exchanged only to make the user aware that the arm_controller is running.

## Description of the pose goal topics
These topics, grasp_pose_goal and pre_grasp_pose_goal, are constructed only to exchange the two pose goals, that the arm will exploit to perform its motion. The messages sent and received are of type geometry_msgs/PoseStamped, so they store information such as position, orientation, and frame of reference for which those values are computed.

## Description of the pick_or_place topic
The routine of movements to be performed changes depending if we are dealing with picking or placing. We construct the following topic to make the arm aware of the sequence of action to be executed. As we know, the options are PICK or PLACE. According to this subdivision, we split the routine into two parts depending on what has to be performed, as can be seen in the Figure below. The main difference between these two phases regards the update of the planning scene and the presence or not of some intermediate poses to approach the target position. Once the arm controller has received the goal pose and the action to be achieved, the routine starts and performs the desired moves taking into account the planning scene. Here we report the main steps that the manipulator performs in each situation:

![Flowchart for understanding in which order the actions are performed](/Media/Flowchart_pick_place.jpg "Flowchart for understanding in which order the actions are performed")

# Base controller