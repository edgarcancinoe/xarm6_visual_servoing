#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, Point

# Variables and constants definition
target = Point()
current_position = Pose()
diferencia = Point()

# Workspace limits
z_low = 0.02
z_up = 0.5

x_max = 0.7
x_min = 0.1

y_max = 0.35
y_min = -0.35


# Origin point values for robots frame of reference
origin = Pose()
origin.position.x = 0.2
origin.position.y = 0
origin.position.z = 0.15

# Starting point coordinates x, y, z as pix, cm, pix
start = Pose()

ready = False
moving = False

# Useful function: target_callback
def target_callback(msg):
    global target, current_position, ready, diferencia, moving
    if not ready or moving:
        return
    movements = msg.position
    diferencia = movements
    # print("Current position: ", current_position.position)
    target.x = current_position.position.x - movements.x/100
    target.y = current_position.position.y - movements.y/100 + 0.01
    target.z = current_position.position.z - movements.z/100 + 0.18

    
# Initialize ROS and MoveIt
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('motion', anonymous=True)


## Instantiate a RobotCommander object. Provides information such as the robot's
## kinematic model and the robot's current joint states
robot = moveit_commander.RobotCommander()


# Define publishers/subscribers
position_publisher = rospy.Publisher('/current_position', Pose, queue_size=10)
rospy.Subscriber('/movement', Pose, target_callback)


## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
## to a planning group (group of joints).
group_name = "xarm6"
arm = moveit_commander.MoveGroupCommander(group_name)

def move_xarm6():
    global current_position, diferencia, moving

    print("Starting to listen for instructions")
    while not rospy.is_shutdown():

        # Get current position
        current_position = arm.get_current_pose().pose
        position_publisher.publish(current_position)

        current_x = round(current_position.position.x, 2)
        current_y = round(current_position.position.y, 2)
        current_z = round(current_position.position.z, 2)
        target_x = round(target.x, 2)
        target_y = round(target.y, 2)
        target_z = round(target.z, 2)


        # print("Current: ", current_x, current_y, current_z)
        # print("Diferencia: ", diferencia.x, diferencia.y, diferencia.z)

        if target_x != current_x or target_y != current_y or target_z != current_z:
            # Workspace limits
            if target_x > x_max or target_x < x_min or target_y < y_min or target_y > y_max or target_z > z_up or target_z < z_low:
                print("Goal is outsite workspace limits")
                print(target_x, target_y, target_z)
            else:
                # Moving to target position
                moving = True

                arm.set_pose_target([ target_x, target_y, target_z, 1, 0, 0, 0])
                plan = arm.plan()
                arm.execute(plan, wait=True) 
                
                print('Target position reached')
                
                moving = False
        else:
            print("Current state is equal to goal state")

        # Update robot instance
        robot = moveit_commander.RobotCommander()

    # Clean up
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        # Initial position
        print("Moving")

        arm.set_pose_target([origin.position.x, origin.position.y, origin.position.z, 1, 0, 0, 0])
        plan = arm.plan()
        arm.execute(plan, wait=True) 

        # At the beggining, target equals current_position
        target = arm.get_current_pose().pose.position
        
        ready = True
        move_xarm6()
    except rospy.ROSInterruptException:
        pass
