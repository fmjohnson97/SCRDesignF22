# SCRDesignF22
The code for our Fall 2022 socially cognizant robotics design course trashbot project with the locobot.

# Instructions for launching ROS packages for the robot
```
./moveit_launch.sh
```
This will launch the moveit framework, which includes the controller for the arm and the base.
This will also use ros_control for controlling the gripper, which uses a position-based controller. Hence we can
use commands as in gripper_test.py to control the gripper to a desired position.
use commands in motion_planner.py to plan the arm.
```
./nav_stack_launch.sh
```
This will launch the navigation stack, without running the robot control.
```
python3 ros2_to_ros_relay.py
```
this will rename ROS topics for driving the base.