# Thesis

## Prerequisites

### Install dependencies

The first thing needed in order to properly setup the code for working both in simulation and in the real rover is to clone the repository with all the submodules by giving the following command:

```bash
git clone --recurse-submodules https://github.com/AntoRag/thesis.git
```

Then it is necessary to install the required dependencies that are documented in the file **command.txt**. in particular we will execute in a new terminal:

```bash
sudo apt install python3-catkin-tools libnlopt-cxx-dev libnlopt-dev ros-noetic-moveit-planners-chomp ros-noetic-moveit-planners-ompl-dbgsym ros-noetic-octomap ros-noetic-octovis ros-noetic-octomap-rviz-plugins ros-noetic-octomap-ros
```

### Build the workspace

Having installed all, we can move in the thesis folder and build the worskpace:

```bash
catkin build
 ```

### Modify .bashrc

Add the following lines to the .bahsrc file for the packages:

```bash
 export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/interbotix_ws/src
 export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/realsense_ws/src
 export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/apriltag_ws/src
 ```

To specify where the models are stored, in order to run the simulation, it is necessary to export the path of gazebo models in the following manner:

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<path_to_worlds>
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH: <path_to_worlds>
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/thesis/artag
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/thesis/artag
```

## Errors

If you face an error like **VWDictionary.cpp:741::addWordRef() Not found word 16308** simply give:

```bash
rm .ros/rtabmap.db
```
