# thesis

- First clone and then catkin_make workspace:

    ```bash
    git clone https://github.com/AntoRag/AntoRag.git
    cd Thesis
    catkin build
    ```

- save_data_topic script can be implemented to read messages published on a topic and store they in a .csv file that after can be easily imported in MATLAB. Until now it is possible to read Float64 messages published on a topic. To run it just type on the terminal

  ```bash
  rosrun save_data save_data_topic.py /topic/path /path/where/save/file
  ```

In this way we will obtain a file called data_saved.csv with time as first column and msg as second

- save_data_jointstate script can be used to specifically collect data from the topic /locobot/joint_states. In this case there will be produced 2 files .csv named: 'data_saved_wheel.csv' and 'data_saved_arm.csv'. In the first one we will save the joint value for the wheels in the second one the angles (in radians) at which the arm joints are.
  
  ```bash
  rosrun save_data save_data_jointstate.py /locobot/joint_states /path/where/save/file
  ```

- To convert a .bag file previously recorded in /bags there is a simple script called FromBagToCVS.py. To run type:

  ```bash
  python3 FromBagToCVS.py /path/to/.bag/file topic/to/be/converted/
  ```

- To run a simulation in Gazebo while using MoveIt as framework we can give as command:
  ```bash

  roslaunch nav_custom xslocobot_nav_sim_rtabmap.launch
  
  ```

  It will load as default OMPL library for motion planning. If we want to explicitly specify the motion planner we can give:
    ```bash
  
  roslaunch nav_custom xslocobot_nav_sim_rtabmap.launch pipeline:=ompl
  
  ```

  Currently is possible to select between two different motion planner: CHOMP and OMPL.
  (Currently, to make CHOMP working correclty, have to be selected "use cartesian path" and select a velocity and acceleration scaling factor equal to 0.6).
- In the folder Matlab there are two scripts useful to analyze the .csv files produced by "save_data_jointstate.py". "DirectKinematic.m" takes as input the angles in radians of the joints and gives in output the end effector position and RPY angles. Those results are then processed in "test.m" and simply plotted.
