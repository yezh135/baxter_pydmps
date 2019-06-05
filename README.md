# baxter_pydmps
my own code for testing dmp in baxter

# How to run
1. start terminator
2. ./baxter.sh sim (simulator)  ./baxter.sh (real robot)
3. start Baxter gazebo  simulator 
`roslaunch baxter_gazebo baxter_world.launch`
4. start joint server `rosrun baxter_interface joint_trajectory_action_server.py`
5. moveit `roslaunch baxter_moveit_config baxter_grippers.launch`
6. Enable robot `rosrun baxter_tools enable_robot.py -e`
7. `rosrun baxter_moveit_config joint_remapping.py`
8. Run DMP `rosrun baxter_pydmps demo_3.py`
