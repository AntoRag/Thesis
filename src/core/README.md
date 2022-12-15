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