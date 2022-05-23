# interbotix_locobot_moveit

This package is a modified version of the original package included with the Locobot wx250s. It presents some changes and some additions.

## Overview

Currently by this package is possible to run the MoveIt framework where we can select between five different planning pipelines:
- OMPL
- CHOMP
- STOMP
- OMPL + CHOMP
- OMPL + STOMP

The planner is aware of the environment thanks to the octmap updater. Currently it is capable of reading the topic where the point cloud is published and translate it in a series of cubic samples of the space. 