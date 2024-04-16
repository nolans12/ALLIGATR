#!/usr/bin/env python 
# <- This is the shebang line which tells the OS which interpreter to use
import rospy
import cv2
import csv
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pickle
import numpy as np

# Try to open the 1 index for the primary camera
camera_index = 1

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
saveBool = 0        # Boolean to save video, 1 means to record video

# Undistort bool
undistBool = 0      # 1 to undistort

# Compression Level
# resize the image by this amount
COMPRESS_CONST = 8.0

# Video object
primaryVideoObj = None


# Image callback for received image
def callback_GAZEBO(data):
    # Used to convert between ROS and OpenCV images
    br = CvBridge()
    
    # Convert ROS Image message to OpenCV image
    img = br.imgmsg_to_cv2(data)
    corners_msg_A = Int32MultiArray()
    corners_msg_B = Int32MultiArray()

    # Run AR tag detection
    corners_msg_A.data, corners_msg_B.data = processImg(img)
    
    # Display image
    outImage = aruco_display(corners_msg_A, img)
    outImage = aruco_display(corners_msg_B, outImage)
    # Resize the image
    scale_percent = 50  # percent of original size
    width = int(outImage.shape[1] * scale_percent / 100)
    height = int(outImage.shape[0] * scale_percent / 100)
    dim = (width, height)
    resized = cv2.resize(outImage, dim, interpolation = cv2.INTER_AREA)
    cv2.imshow("camera", resized)    # Comment this line out for headless detection

    # Publish if they are detected
    if corners_msg_A.data[0]:
        pub_corners_A.publish(corners_msg_A)
    if corners_msg_B.data[0]:
        pub_corners_B.publish(corners_msg_B)
    
    cv2.waitKey(1)


# Function that checks a given image for an AR tag and returns corners if its found
def processImg(img):
    corners_A = [0, 0, 0, 0, 0, 0, 0, 0]
    corners_B = [0, 0, 0, 0, 0, 0, 0, 0]

    if img is None:
        out_str = "Camera Connection Lost %s" % rospy.get_time()
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

            out_str = "AR Tag Detected %s" % rospy.get_time()
        else:
            out_str = "No AR Tag %s" % rospy.get_time()

    rospy.loginfo(out_str)
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
    global primaryVideoObj
    if saveBool:
        primaryVideoObj.release()
        rospy.loginfo("Successfully closed the primary video file.") 

    cv2.destroyAllWindows()         # Close everything and release the camera
    cap.release()
    rospy.loginfo("End of program") # This will output to the terminal


if __name__ == '__main__': # <- Executable 
    # ArUco dictionary
    ARUCO_DICT = {
        "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    }

    # Get predefined dictionary for AR tag detection
    aruco_type = "DICT_6X6_250"
    finalDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])

    rospy.init_node("Primary_Detection_Node") # Initialize the ROS node

    rospy.loginfo("############# PRIMARY AR DETECTION NODE #################") # This will output to the terminal

    # This is how to initialize a publisher
    rospy.loginfo("Initializing ROS connection...")
    
    ################## Publisher Definitions ###########################
    pub_corners_A = rospy.Publisher('CV/Primary/AR_corners_A', Int32MultiArray, queue_size=1)     # RGV A
    pub_corners_B = rospy.Publisher('CV/Primary/AR_corners_B', Int32MultiArray, queue_size=1)     # RGV B
    pub_image = rospy.Publisher('CV/Primary_Video', Image, queue_size=1)

    ####################################################################

    ################## Subscriber Definitions ###########################
    #sub_img = rospy.Subscriber('webcam/image_raw', )
    ####################################################################

    rate = rospy.Rate(10) # 10hz

    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # Video file 
    if saveBool:
        # Add callback for shutdown
        rospy.on_shutdown(releaseObjects)
        size = (int(1920/COMPRESS_CONST), int(1080/COMPRESS_CONST)) 
        filename = "primaryVideo%s.avi" % str(rospy.get_time())
        primaryVideoObj = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*'XVID'), saveFPS, size)

    # Undistort
    if undistBool:

        camMatrix = np.array([[2.49100915e+03, 0, 4.83656938e+02],
              [0, 2.62434619e+03, 5.22873439e+02],
              [0, 0, 1]])

        distCoeff = np.array([-1.58920862e+00, -4.12086079e+00, -5.84693333e-03, 1.80546798e-01, 7.93548451e+01])  # k1, k2, p1, p2, k3

        # Specify the path to your pickle file
        # pickle_file_path = '~/ALLIGATR/HUB/ros_packages/gatr_computer_vision/src/WideMatrix.pkl'
        # with open(pickle_file_path, 'rb') as file:
        #     # Load data from the pickle file
        #     camMatrix = pickle.load(file)

        # pickle_file_path = '~/ALLIGATR/HUB/ros_packages/gatr_computer_vision/src/WideDist.pkl'
        # with open(pickle_file_path, 'rb') as file:
        #     # Load data from the pickle file
        #     distCoeff = pickle.load(file)

        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(camMatrix,distCoeff,(int(1920),int(1080)),1,(int(1920),int(1080)))


    # Now that ROS connection is established, begin searching for the camera
    rospy.loginfo("Establishing camera connection...")

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
            rospy.loginfo("Camera " + str(camera_index) + " Connected!")
            break            

        rospy.sleep(1.0) # Sleep for 1 second

        # If camera is not found, output an error message
        rospy.logwarn("Camera not found. Trying again...")
        attempts += 1

        # Camera wasn't found after multiple attempts. 
        if attempts > 5:
            rospy.logfatal("Camera not found after 10 attempts. Connecting to faux camera.")
            faux_camera = True
            camera_found = False

    if faux_camera:
        # Verify that there is a connection to the webcam/image_raw topic
        if camera_index == 0:
            sub_img = rospy.Subscriber('webcam/image_raw', Image, callback_GAZEBO)
        else:
            sub_img = rospy.Subscriber('webcam/image_raw2', Image, callback_GAZEBO)
        rospy.spin()


    # Begin the main loop that consistently outputs AR tag corners when running
    while not rospy.is_shutdown():
        # Output messages
        corners_msg_A = Int32MultiArray()
        corners_msg_B = Int32MultiArray()
        corners_msg_A.data = [0, 0, 0, 0, 0, 0, 0, 0]
        corners_msg_B.data = [0, 0, 0, 0, 0, 0, 0, 0]


        if cap.isOpened():                          # Capture image while camera is opened
            # Get the current video feed frame
            ret, img = cap.read()
            frameCount += 1                         # Update the frame count

            # Publish to ROS
            if frameCount % (camFPS // pubFPS) == 0:
                # Compress image by resizing
                compressed_frame = cv2.resize(img, (int(1920/COMPRESS_CONST), int(1080/COMPRESS_CONST)))

                # Convert to ros message
                ros_image = br.cv2_to_imgmsg(compressed_frame)

                # Publish image message to image topic
                pub_image.publish(ros_image)

            # Process Image
            if frameCount % (camFPS // processFPS) == 0:
                if undistBool:  #undistort
                    img = cv2.undistort(img, camMatrix, distCoeff, None, newcameramtx)    

                # Output message with corners
                corners_msg_A.data, corners_msg_B.data = processImg(img)

            # Save image
            if saveBool:
                if frameCount % (camFPS // saveFPS) == 0: 
                    # Compress image by resizing
                    compressed_frame = cv2.resize(img, (int(1920/COMPRESS_CONST), int(1080/COMPRESS_CONST)))

                    # Save video
                    primaryVideoObj.write(compressed_frame)
                    rospy.loginfo("Saved Primary frame")

            if frameCount >= 60:
                frameCount = 0  # Reset frame count

        # If the camera is connected through a faux camera in ROS
        elif faux_camera:
            # Wait for received image with the callback
            pass
        else:
            out_str = "Camera Connection Lost %s" % str(rospy.get_time())
            rospy.loginfo(out_str)
            
        # Publish to the ROS node
        if corners_msg_A.data[0]:
            pub_corners_A.publish(corners_msg_A)
        if corners_msg_B.data[0]:
            pub_corners_B.publish(corners_msg_B)

    primaryVideoObj.release()
    cv2.destroyAllWindows()         
    cap.release()
    rospy.loginfo("End of program") 
