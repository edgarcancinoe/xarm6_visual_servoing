import cv2
import numpy as np
import math

# Constants

# Load the camera matrix and distortion coefficients
camera_matrix = np.array([[615.6994619, 0, 310.65921052],
                          [0, 614.09468385, 221.27468646],
                          [0, 0, 1]])

distortion_coeffs = np.array([1.03432284e-02, 6.15742130e-02, 5.88511152e-03, -2.73265332e-04, -1.26407881])

focal_length_x = camera_matrix[0, 0]
focal_length_y = camera_matrix[1, 1]

camera_height = 480

def process_qr(qr_codes, frame):
    # QR code size in cm
    qr_code_size_robot = 7.2
    qr_code_size_target = 7.2
    coordinates = dict()

    # Iterate over the detected QR codes
    for qr_code in qr_codes:       
        
        qr = str()     
        qr_code_size = float()

        # Extract the (x, y) coordinates of the QR code
        x = qr_code.rect.left
        y = qr_code.rect.top

        x = (2*x + qr_code.rect.width)/2
        y = (2*y + qr_code.rect.height)/2
        color = (255, 255, 0)

        label = ""
       
        qr = qr_code.data.decode('utf-8')
        if qr == 'target':
            color = (0,0, 255)
            label = "Target " + label
            qr_code_size = qr_code_size_target
            
        if qr == 'robot':
            color = (0,255, 0)
            label = "Robot " + label
            qr_code_size = qr_code_size_robot

        # Estimate the depth using the known QR code size and camera matrix
        depth = (camera_matrix[0][0] * qr_code_size) / qr_code.rect.width 

        l = qr_code_size * (x - camera_height/2) / qr_code.rect.height

        z = math.sqrt(depth**2 - l**2)

        
        label = label + "X: " + str(x) + " Y: " + str(y) + " Z: "
        label = label + str(round(z,2))
        
        # Save position
        coordinates[qr] = (x, y, depth)

        # Draw a rectangle around the QR code on the frame
        cv2.rectangle(frame, (qr_code.rect.left, qr_code.rect.top),
                    (qr_code.rect.left + qr_code.rect.width, qr_code.rect.top + qr_code.rect.height),
                    color, 2)

        # Display the coordinates and depth on the image
        cv2.putText(frame, label,
                    (qr_code.rect.left, qr_code.rect.top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    # Apply distortion correction to the frame
    frame = cv2.undistort(frame, camera_matrix, distortion_coeffs)
    
    # Return position dictionary
    return coordinates