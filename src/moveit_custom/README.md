# Moveit package

This package is a modified version of the original package included with the Locobot WX250. It presents some changes and some additions, in particular it includes STOMP and CHOMP and presents the possibility to launch the obstacle detection with Octomap. The software architecture specifically for the manipulator is composed in the following way:
<p align="center">
    <img src="/Media/arm_comm.jpg" alt="Arm controller software structure" style="height: 70%; width: 70%;"/>
</p>

## Overview about MoveIt! planning framework

Currently by this package is possible to run the MoveIt framework where we can select between five different planning pipelines:

- OMPL
- STOMP
- OMPL + CHOMP
- OMPL + STOMP

## Octomap obstacle avoidance

The planner is aware of the environment thanks to the **Octmap** plugin available in the MoveIt planning framework. We set up the framework to work such that the poincloud is translated into colored voxels as we can see from the following image

<p align="center">
    <img src="/Media/retraction_real.png" alt="Example how what is possible to detect with Octomap" style="height: 40%; width: 40%;"/>
</p>





