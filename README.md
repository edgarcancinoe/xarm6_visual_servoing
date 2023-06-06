# Xarm6 visual servoing
## Project Description
Python project for visual servoing of a 6-DOF XARM robotic arm. An outside camera identifies two QR codes, one with the word 'robot' encoded corresponding to the arm's gripper position, and other with the word 'target' encoding, used to identify the target object's position.

Relative position of these points of interest is determined and a trajectory is computed to make the arm reach the desired position.

## Dependencies
This projects runs in ROS melodic. The Xarm6 developer demo packages for ROS simulation were used (1), and MoveIt motion planning framework (2) must be installed in the workspace for it to run properly.


#### References
1. https://github.com/xArm-Developer/xarm_ros
2. http://docs.ros.org/en/melodic/api/moveit_tutorials/html/index.html
3. http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html

#### Keywords
computer vision, visual servoing, xarm, ros, gazebo, python

