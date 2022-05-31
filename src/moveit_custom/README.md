# Moveit package

This package is a modified version of the original package included with the Locobot wx250s. It presents some changes and some additions.

## Overview

Currently by this package is possible to run the MoveIt framework where we can select between five different planning pipelines:

- OMPL
- STOMP
- OMPL + CHOMP
- OMPL + STOMP

The planner is aware of the environment thanks to the **Octmap** plugin available in the MoveIt planning framework. 

Another package is added "**ar_track_alavr**" which allows us to retrieve the position of an ArTag in the space thanks to the camera (Intel Realsense D435) so that we can identify the position in the shelf of an object.


![Example how what is possible to detect with Octomap](/media/octomap.png)


https://user-images.githubusercontent.com/94801013/171169179-fe046676-bd94-4ccd-87d2-d38759f99afc.mp4


