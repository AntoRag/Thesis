# Packages

## Repository structure

The folder contains the packages related to the construction of the software architecture for the solution developed for the mobile manipulator Locobot WX250. In particular, to construct our software we emplyed the following existing solutions:
- [Octomap](https://octomap.github.io/)
- [Moveit!](https://moveit.ros.org/)
- [Spencer]() 
- [ar_track_alvar](http://wiki.ros.org/ar_track_alvar)

Where the name of the folder is **"something"_custom** it means that they are taken from the original packages coming with the [Locobot WX250](https://www.trossenrobotics.com/docs/interbotix_xslocobots/index.html) developed by Trossen Robotics and then modified according to our needs. The software structure is depicted in the following figure:

<p align="center">
    <img src="/Media/communication_structure.jpg" alt="Software structure mobile manipulator" style="height: 70%; width: 70%;"/>
</p>

The core of the software is contained in the "core" folder where the software architecture is contained.

# How to launch the application

After explaining our application's software architecture, we performed different tests to determine this structure's limits and advantages. The tests we performed can be split into two types depending on the environment where the robot is executed: the ones performed in simulation through Gazebo and the ones where the deployment is in our laboratory where we positioned some fake medicines with ARTags to be picked up and placed.

## Simulated robot

The map implemented in Gazebo, which describes a possible warehouse environment, has some obstacles that the mobile base can avoid during path planning. The items to be manipulated are positioned on shelves, representing a possible situation we can face in reality. We can look at the world built in Gazebo in Figure below.

<p align="center">
    <img src="/Media/Gazebo_env.png" alt="Software structure mobile manipulator" style="height: 70%; width: 70%;"/>
</p>


### Commands and parameters
Then we can set other parameters like the followings:
| Parameter  | Description                                                                         |
| ---------- | ----------------------------------------------------------------------------------- |
| use_actual | if true we can control the real robot                                               |
| use_artag  | if true launches the node responsible for retreiving the position of ArTag in space |
| use_camera | if true launches the nodes for publishing the camera data                           |
| pipeline   | here we can select the motion planner to be exploited                               |

### Tests


## Real robot

### Commands and parameters

Then we can set other parameters like the followings:
| Parameter  | Description                                                                         |
| ---------- | ----------------------------------------------------------------------------------- |
| use_actual | if true we can control the real robot                                               |
| use_artag  | if true launches the node responsible for retreiving the position of ArTag in space |
| use_camera | if true launches the nodes for publishing the camera data                           |
| pipeline   | here we can select the motion planner to be exploited                               |

### Tests

#### Laboratory setup







