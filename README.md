# xArm6 visual servoing
## Project Description
Python project for visual servoing of a 6-DOF robotic arm (xArm 6). An external camera is used to identify two QR codes, one with the word 'robot' encoded corresponding to the arm's gripper position, and other with the word 'target' encoded, used to identify the target object's position.

Relative position of these points of interest is determined and a trajectory is computed and excecuted to make the arm's gripper reach the desired position.

## Dependencies
This projects runs in ROS melodic. The xArm developer demo packages for ROS simulation were used (1), and MoveIt motion planning framework (2) must be installed in the workspace for it to run properly.

<img width="678" alt="Screenshot 2023-06-06 at 18 23 24" src="https://github.com/edgarcancinoe/xarm6_visual_servoing/assets/59784477/e7f99c60-d604-40b9-b9e3-9fe5bdd9b33a">

#### Video
https://youtu.be/yTxkO5lXrIg

#### References
1. https://github.com/xArm-Developer/xarm_ros
2. http://docs.ros.org/en/melodic/api/moveit_tutorials/html/index.html
3. http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html

#### Keywords
computer vision, visual servoing, xarm, ros, gazebo, python

