# Thesis

## Introduction
Robotics nowadays is a science that is deeply involved in technological progress in many aspects. More and more robots are employed to carry out tasks that require high reliability and precision. The problem that we tried to address regards the construction of the software for a mobile manipulator that can work almost as a plug-and-play solution. Many of the applications presented in the thesis work in a specific environment with the supervision of a human operator; our goal is to create a solution that can work stand-alone following only a request for a product from the user. We construct an algorithm that can be easily modified and replicated for any robot with a similar hardware structure. In particular, the robot has first to map the environment, then, according to an identification system, locate the object of interest and grasp it without, in this phase, being aware of the object or how the shelves where the item is located are made. 
Each of these capabilities will be implemented via ROS packages, and communication with the algorithm will happen through topics and services. The application designed to accomplish our goal will be implemented as ROS nodes. The target environment for the application is a warehouse that has the necessity of picking stored items, medicines in our case, and placing them in a depot. This environment will have static obstacles, such as shelves, walls, and dynamic obstacles with varying degrees of predictability, from other mobile manipulators to humans.

## Research area 
By selecting ROS (Robot Operating System) as middleware, we can achieve hardware abstraction allowing for the generalized use of our software architecture on various platforms. For object manipulation, we chose to implement MoveIt as a planning framework. MoveIt is capable of being customized according to the necessities expressed by the user. For research purposes, we implemented different planning algorithms that are currently the state of the art in terms of tunability and reliability. In particular, for our application, we considered the following solutions:
- OMPL (Open Motion Planning Library)
- STOMP (Stochastic Trajectory Optimization for Motion Planning)
- CHOMP (Covariant Hamiltonian Optimization for Motion Planning)

In particular, from our tests, we understand that the fastest algorithm to solve the motion planning problem in a not-so-populated scenario is $RRT^{*}$, so we continued our tests, selecting it as the default planner.
The second problem addressed during this thesis work is the implementation of an obstacle avoidance algorithm that must be fully compatible with MoveIt and its planning tools. Our attention focused on Octomap, which is a pretty powerful tool combined with MoveIt. It guarantees, in fact, online computation of the grid map that characterizes the obstacles to be taken into account by the manipulator.
The third problem addressed with this work regards object recognition on the shelves. This is achieved through the implementation of a package for ROS called "ar_track_alvar". It relies on the recognition of ARTag markers so that we can identify and estimate the items poses with a certain precision.
Due to the constraints of the Locobot platform, we chose path-planning algorithms that favoured computational speed and detection algorithms that did not require the presence of a GPU. 
For autonomous navigation, the RTAB-Map SLAM algorithm is a state-of-the-art solution that we were able to use to map the office in which the tests were conducted.
For path planning, the chosen global and local planning algorithms are A* and Timed Elastic Band (TEB), respectively, due to their combined efficacy. The former favors speed while the latter favors accuracy. 
We achieved human detection with a random forest classifier for the LIDAR able to detect legs and a History of oriented gradients(HOG) detector on the RGB data from the RGB-D sensor. The depth information is used to direct the HOG detector to suitable regions of the image. Both classifiers inform the final detection and a Nearest Neighbour clustering algorithm that is able to track detections for moving persons in the robot's space. 

## Contribution

<p align="center">
    <img src="/Media/communication_structure.jpg" alt="Software structure mobile manipulator" style="height: 70%; width: 70%;"/>
</p>
The first thing we had to do was to modify the existing fingers of the Locobot WX250, which have a maximum opening width of 4 cm that is not suitable for grasping real medicines. To do this, we drew a CAD of the parts and then produced them thanks to a 3D printer.
The software architecture we developed, described in the Figure above, consists of a modular structure composed of three submodules: the communication node, the arm controller, and the base controller. The communication node is responsible for controlling the working logic of the robot and the workflow to perform a complete pick-and-place action.
The communication node is responsible for sending the pose to the arm and base controller. Then the arm and the base will communicate their status. They will act synchronously such that the action of a module can start only if the other module has successfully completed its task.
The base controller node handles movement requests and publishes its status to inform the communication manager node of the successful or failed operation.
We developed a fourth and smaller ROS node called "tracked\_people\_translator" for package interoperability. 

## Results

We obtained a working mobile manipulator that can handle a request for an item submitted through a topic and is able to perform pick and place actions. The software implemented for monitoring the robot's procedures is RViz, where we can also view the surrounding environment's map constructed by RTAB-Map. The robot is also able to navigate through complex spaces and avoid collisions with static obstacles recognizing humans and behaving differently to avoid a collision with them.

# Installation

## Prerequisites

### Install dependencies

The first thing needed in order to properly setup the code for working both in simulation and in the real rover is to clone the repository with all the submodules by giving the following command:

```bash
git clone --recurse-submodules https://github.com/AntoRag/thesis.git
```

Then it is necessary to install the required dependencies that are documented in the file **command.txt**. in particular we will execute in a new terminal:

```bash
#install the following packages
git clone https://github.com/ros-industrial ros_industrial_cmake_boilerplate.git
git clone https://github.com/ros-industrial/stomp.git
git clone https://github.com/ros-industrial/stomp_ros.git
#To install these it is necessary to put them in the catkin workspace and give "catkin build"

#install the following
# to have catkin build
sudo apt install python3-catkin-tools
#nlopt.hpp not found
sudo apt install libnlopt-cxx-dev
sudo apt install libnlopt-dev

#to install chomp planner
sudo apt install ros-noetic-moveit-planners-chomp
sudo apt install ros-noetic-moveit-planners-ompl-dbgsym

#to install necessary package for octomap
sudo apt install ros-noetic-octomap ros-noetic-octovis
sudo apt install ros-noetic-octomap-rviz-plugins
sudo apt install ros-noetic-octomap-ros

camera_self_filter TO_DO

#to install artag tracker
#in src/
git submodule add -b noetic-devel https://github.com/machinekoder/ar_track_alvar.git 
#then in thesis/
catkin build

#cannot find GlobalPlanner solved via:
sudo apt-get install ros-noetic-global-planner

#Failed to create tebLocalPlanner
sudo apt install ros-noetic-teb-local-planner

#Necessary for building
sudo apt install ros-noetic-rgbd-launch
sudo apt install ros-noetic-move-base

#failed to create global planner
sudo apt install ros-noetic-global-planner


sudo apt install ros-noetic-people-msgs
sudo apt install ros-noetic-rosparam-shortcuts
sudo apt install python3-catkin-tools libnlopt-cxx-dev libnlopt-dev ros-noetic-moveit-planners-chomp ros-noetic-moveit-planners-ompl-dbgsym ros-noetic-octomap ros-noetic-octovis ros-noetic-octomap-rviz-plugins ros-noetic-octomap-ros
```

## Build the workspace

Having installed all, we can move in the thesis folder and build the worskpace:

```bash
catkin build
 ```

### Modify .bashrc

Add the following lines to the .bahsrc file for the packages:

```bash
export ROS_PACKAGE_PATH=~/interbotix_ws/src:~/realsense_ws/src:~/apriltag_ws/src:$ROS_PACKAGE_PATH
 ```

To specify where the models are stored, in order to run the simulation, it is necessary to export the path of gazebo models in the following manner:

```bash
export ROS_PACKAGE_PATH=~/interbotix_ws/src:$ROS_PACKAGE_PATH
export GAZEBO_MODEL_PATH=~/:~/thesis/models:~/thesis artag:$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=~/:~/thesis/models:~/thesis/artag:$GAZEBO_RESOURCE_PATH
```

## Errors

If you face an error like **VWDictionary.cpp:741::addWordRef() Not found word 16308** simply give:

```bash
rm .ros/rtabmap.db
```
to clear the map created and suppress this error.