#!/usr/bin/env python3

import sys
import rospy
from pyzbar import pyzbar
from geometry_msgs.msg import Pose
from helper import *

# Init ros node
rospy.init_node('vision')

# Define publishers/subscribers
target_pub = rospy.Publisher('/target', Pose, queue_size=10)
robot_pub = rospy.Publisher('/robot', Pose, queue_size=10)
movement_pub = rospy.Publisher('/movement', Pose, queue_size=10)

# Define Target and Robot Pose objects
target = Pose()
robot = Pose()
movement = Pose()

# Configurar las coordenadas iniciales del punto en (0, 0)
target.position.x = 0
target.position.y = 0
target.position.z = 0


if __name__=='__main__':
    # Open Camera
    cap = cv2.VideoCapture(0)
    measurements = []

    try:
        while not rospy.is_shutdown():
            
            # Read a frame from the video stream
            ret, frame = cap.read()

            # Convert the frame to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Use pyzbar to detect QR codes in the frame
            qr_codes = pyzbar.decode(gray)

            # Draw qr boxes, undistort and get cooridnates
            points = process_qr(qr_codes, frame)

            # Flags
            found_target = False
            found_robot = False

            if 'target' in points:
                target.position.x = points['target'][0] # Pixeles
                target.position.y = points['target'][1] # cm
                target.position.z = points['target'][2] # Pixeles
                found_target = True
            if 'robot' in points:
                robot.position.x = points['robot'][0] # Pixeles
                robot.position.y = points['robot'][1] # cm
                robot.position.z = points['robot'][2] # Pixeles
                found_robot = True

            #######
            if 'target' in points and 'robot' in points:
                x_t = points['target'][0] # Pixeles
                y_t = points['target'][1] # Pixeles
                d_t = points['target'][2] # cm

                x_r = points['robot'][0] # Pixeles
                y_r = points['robot'][1] # Pixeles
                d_r = points['robot'][2] # cm

                # Normalize (convert to terms of sensor size) and convert to metric
                diff = np.array([(x_t - x_r)/10, (y_t - y_r)/10, d_t - d_r])
                
                measurements.append(diff)
                
                # If we have collected enough measurements, calculate the average
                if len(measurements) == 10:
                    # Convert the list of measurements to a NumPy array for easier manipulation
                    measurements_array = np.array(measurements)

                    # Calculate the average for each dimension
                    avg_diff = np.mean(measurements_array, axis=0)

                    # Set the average values to the movement position
                    movement.position.x = avg_diff[0]
                    movement.position.y = avg_diff[2]
                    movement.position.z = avg_diff[1]

                    print("Average Movement:\n", movement.position)

                    # Clear the measurements list for the next set of measurements
                    measurements = []

                    # Publish the movement
                    movement_pub.publish(movement)

                # movement.position.x = diff[0]
                # movement.position.y = diff[2]
                # movement.position.z = diff[1]

                # print(movement.position)
                # movement_pub.publish(movement)

            #######
            
            # Transmit robot/target detected cooridnates
            target_pub.publish(target) if found_target else None
            robot_pub.publish(robot) if found_robot else None

            # Show the undistorted frame with the detections
            cv2.imshow("QR Code Detection", frame)

            # Exit the loop if the 'q' key is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

    except rospy.ROSInterruptException:
        # Pasar al bloque siguiente
        pass