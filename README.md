# Thesis

- First clone and then catkin_make workspace:

    ```bash
    git clone https://github.com/AntoRag/AntoRag.git
    cd Thesis
    catkin_make
    ```

- test_node implemented to publish on /locobot/<joint_name>/command to control the position of a joint while running the simulation on Gazebo. To run a simulation in Gazebo while visualizing in Rviz type in the terminal:

  ```bash
  # This will run the node for controlling singularly each joint while visualizing it on Rviz
  roslaunch interbotix_xslocobot_control xslocobot_control.launch use_sim:=true use_rviz:=true robot_model:=locobot_wx250s
  ```

  Then, in another terminal, we can type

  ```bash
  # This will run the simulation on Gazebo
  roslaunch interbotix_xslocobot_nav xslocobot_nav_sim.launch robot_model:=locobot_wx250s dof:=6 use_lidar:=true
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
