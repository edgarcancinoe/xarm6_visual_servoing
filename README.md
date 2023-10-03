# Visual servoing of robotic arm
## Project Description
Python project for visual servoing of a 6-DOF robotic arm (xArm 6). An external camera is used to identify two QR codes, one with the word 'robot' encoded corresponding to the arm's gripper position, and other with the word 'target' encoded, used to identify the target object's position.

Relative position of these points of interest is determined and a trajectory is computed and excecuted to make the arm's gripper reach the desired position.

## Dependencies
This projects runs in ROS melodic. The xArm developer demo packages for ROS simulation were used (1), and MoveIt motion planning framework (2,3) must be installed in the workspace for it to run properly.

## Gallery
<img width="480" alt="Screenshot 2023-06-06 at 18 24 12" src="https://github.com/edgarcancinoe/xarm6_visual_servoing/assets/59784477/b9707baa-741d-4d1d-a557-420fe10f1a29">
<br>
<img width="480" alt="Screenshot 2023-06-06 at 18 26 05" src="https://github.com/edgarcancinoe/xarm6_visual_servoing/assets/59784477/54cd2a18-03af-4af1-9a6b-201438e156a4">

## Watch it!


<img src="media/visualservoing.gif?raw=true"/>

[Watch on youtube](https://youtu.be/yTxkO5lXrIg)

---

#### References
1. [xArm ROS developer](https://github.com/xArm-Developer/xarm_ros)
2. [MoveIt Getting Started](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html)
3. [MoveIt Setup](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/index.html)

#### Keywords
computer vision, visual servoing, xarm, ros, gazebo, python

