# Thesis
First clone and then catkin_make workspace
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
    roslaunch interbotix_xslocobot_gazebo xslocobot_gazebo.launch use_base:=true robot_model:=locobot_wx250s use_camera:=true use_lidar:=true use_sim:=true use_position_controllers:=true dof:=6 
  ```

- save_data node can be implemented to read messages published on a topic and store they in a .csv file that after can be easily imported in MATLAB. To run it just type on the terminal

  ```bash
    rosrun save_data save_data.py /topic/path /path/where/save/file
  ```

  in this way we will obtain a file called data_saved.csv with time as first column and msg as second
