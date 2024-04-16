#!/usr/bin/env python 
# <- This is the shebang line which tells the OS which interpreter to use
import cv2
import time
import csv

# Try to open the 0 index for the secondary camera
camera_index = 0

# Camera FPS
camFPS = 60

# Publish image frequency
pubFPS = 1

# Process image frequency
processFPS = 30

# Frame count
frameCount = 0

# Save FPS
saveFPS = 1
saveBool = 1        # Boolean to save video, 1 means to record video

# Compression Level
# resize the image by this amount
COMPRESS_CONST = 8.0

# Video object
secondaryVideoObj = None


# Function that checks a given image for an AR tag and returns corners if its found
def processImg(img):
    corners_A = [0, 0, 0, 0, 0, 0, 0, 0]
    corners_B = [0, 0, 0, 0, 0, 0, 0, 0]

    if img is None:
        out_str = "Camera Connection Lost %s" % time.time()
    else:          
        # Search for Aruco tag
        corners, ids, rejected = cv2.aruco.detectMarkers(img, finalDict)

        # Output the detected corners if detected
        if corners:
            k = 0
            for i in ids:
                # Format the corners as an array
                firstCorners = corners[k][0]
                topLeft = firstCorners[1]
                topRight = firstCorners[2]
                bottomRight = firstCorners[3]
                bottomLeft = firstCorners[0]
                outArr = [int(topLeft[0]), int(topLeft[1]), int(topRight[0]), int(topRight[1]), int(bottomRight[0]), int(bottomRight[1]), int(bottomLeft[0]), int(bottomLeft[1])]

                if i == 1:
                    corners_A = outArr
                else:
                    corners_B = outArr
                k = k + 1

            out_str = "AR Tag Detected %s" % time.time()
        else:
            out_str = "No AR Tag %s" % time.time()

    print(out_str)
    return corners_A, corners_B
    
# Highlight the detected markers
def aruco_display(corners, image):
    if corners.data[0]: # Are any aruco tags detected
        topLeft = (int(corners.data[0]), int(corners.data[1]))
        topRight = (int(corners.data[2]), int(corners.data[3]))
        bottomRight = (int(corners.data[4]), int(corners.data[5]))
        bottomLeft = (int(corners.data[6]), int(corners.data[7]))

        # Draw the lines for the AR tag detection
        cv2.line(image, topLeft, topRight, (0, 0, 255), 2)
        cv2.line(image, topRight, bottomRight, (0, 0, 255), 2)
        cv2.line(image, bottomRight, bottomLeft, (0, 0, 255), 2)
        cv2.line(image, bottomLeft, topLeft, (0, 0, 255), 2)
    return image

# Function for shutting down the node
def releaseObjects():
    global secondaryVideoObj
    secondaryVideoObj.release()
    print("Successfully closed the secondary video file.") 

    cv2.destroyAllWindows()         # Close everything and release the camera
    cap.release()
    print("End of program") # This will output to the terminal



# ArUco dictionary
ARUCO_DICT = {
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
}

# Get predefined dictionary for AR tag detection
aruco_type = "DICT_6X6_250"
finalDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])

####################################################################

################## Subscriber Definitions ###########################
#sub_img = rospy.Subscriber('webcam/image_raw', )
####################################################################

# Video file
if saveBool:
    size = (int(1920/COMPRESS_CONST), int(1080/COMPRESS_CONST)) 
    filename = "secondaryVideo%s.avi" % str(time.time())
    secondaryVideoObj = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*'XVID'), saveFPS, size)


# Search for camera, if not found open faux camera
camera_found = False
faux_camera = False
attempts = 0   

# Initialize the Camera and Savings
while not camera_found and not faux_camera:
    # Setup the GStreamer Pipeline
    #pipeline = f'nvarguscamerasrc sensor-id={camera_index} ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080, format=(string)NV12, framerate=(fraction)15/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink'
    pipeline = 'nvarguscamerasrc sensor-id=' + str(camera_index) + ' ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080, format=(string)NV12, framerate=(fraction)60/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink'
    #pipeline = 'nvarguscamerasrc sensor-id=' + str(camera_index) + ' ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080, format=(string)NV12, framerate=(fraction)60/1 ! capsfilter caps="video/x-raw, width=(int)1920, height=(int)1080, framerate=(fraction)30/1" ! autovideosink ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink'

    # Create a VideoCapture object with the GStreamer pipeline
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    # Check if the camera opened successfully
    if cap.isOpened():
        camera_found = True     # Camera is found
        print("Camera " + str(camera_index) + " Connected!")
        break            

    # If camera is not found, output an error message
    print("Camera not found. Trying again...")
    attempts += 1

    # Camera wasn't found after multiple attempts. 
    if attempts > 5:
        print("Camera not found after 10 attempts. Connecting to faux camera.")
        faux_camera = True
        camera_found = False

if faux_camera:
    pass


# Begin the main loop that consistently outputs AR tag corners when running
while True:
    if cap.isOpened():                          # Capture image while camera is opened
        # Get the current video feed frame
        ret, img = cap.read()
        frameCount += 1                         # Update the frame count

        # Publish to ROS
        if frameCount % (camFPS // pubFPS) == 0:
            # Compress image by resizing
            compressed_frame = cv2.resize(img, (int(1920/COMPRESS_CONST), int(1080/COMPRESS_CONST)))

            pass

        # Process Image
        if frameCount % (camFPS // processFPS) == 0:                
            # Output message with corners
            corners_msg_A, corners_msg_B = processImg(img)

        # Save image
        if saveBool:
            if frameCount % (camFPS // saveFPS) == 0: 
                # Resize the image
                compressed_frame = cv2.resize(img, (int(1920/COMPRESS_CONST), int(1080/COMPRESS_CONST)))

                # Save video
                secondaryVideoObj.write(compressed_frame)

                # Log info
                print("Saved Secondary frame")
        
        if frameCount >= 60:
            frameCount = 0  # Reset frame count

    # If the camera is connected through a faux camera in ROS
    elif faux_camera:
        # Wait for received image with the callback
        pass
    else:
        out_str = "Camera Connection Lost %s" % str(time.time())
        print(out_str)
        
    # Publish to the ROS node
    # if corners_msg_A.data[0]:
    #     pub_corners_A.publish(corners_msg_A)
    # if corners_msg_B.data[0]:
    #     pub_corners_B.publish(corners_msg_B)


secondaryVideoObj.release()
cv2.destroyAllWindows()         # Close everything and release the camera
cap.release()
rospy.loginfo("End of program") # This will output to the terminal
