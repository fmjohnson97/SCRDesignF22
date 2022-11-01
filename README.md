# SCRDesignF22
The code for our Fall 2022 socially cognizant robotics design course trashbot project with the locobot.

# Instructions for launching ROS packages for the robot
```
roslaunch interbotix_xslocobot_nav xslocobot_nav.launch robot_model:=locobot_wx250s launch_driver:=false
```
This will launch the navigation stack, without running the robot control.
```
roslaunch interbotix_xslocobot_moveit xslocobot_moveit.launch robot:model:=locobot_wx250s
```
This will launch the moveit framework, which includes the controller for the arm and the base.
This will also use ros_control for controlling the gripper, which uses a position-based controller. Hence we can
use commands as in gripper_test.py to control the gripper to a desired position.
```
python3 ros2_to_ros_relay.py
```
this will rename ROS topics for driving the base.