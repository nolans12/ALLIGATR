#!/usr/bin/env python 
# <- This is the shebang line which tells the OS which interpreter to use
# Be aware that the tutorial said to use python2, however we built it for py3 I believe
import rospy
import cv2
import sys
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray


if __name__ == '__main__': # <- Executable 
    # ArUco dictionary
    ARUCO_DICT = {
        "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    }

    # Get predefined dictionary for AR tag detection
    aruco_type = "DICT_6X6_250"
    finalDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])

    rospy.init_node("ARtag_detection_node") # Initialize the ROS node

    rospy.loginfo("############# ARtag DETECTION NODE #################") # This will output to the terminal

    # This is how to initialize a publisher
    rospy.loginfo("Initializing ROS connection...")
    
    ################## Publisher Definitions ###########################
    pub_corners = rospy.Publisher('AR_corners', Int32MultiArray, queue_size=10)

    ####################################################################
    #rospy.init_node('blob_detection_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # Log the version of Python and OpenCV
    rospy.loginfo("Python Version: " + sys.version)
    rospy.loginfo("OpenCV Version: " + cv2.__version__)


    # Now that ROS connection is established, begin searching for the camera
    rospy.loginfo("Establishing camera connection...")

    camera_found = False
    faux_camera = False
    attempts = 0

    # Try to open the 0 index for the secondary camera
    camera_index = 1   

    while not camera_found:
        # Setup the GStreamer Pipeline
        #pipeline = f'nvarguscamerasrc sensor-id={camera_index} ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080, format=(string)NV12, framerate=(fraction)15/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink'
        pipeline = 'nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080, format=(string)NV12, framerate=(fraction)15/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink'


        # Create a VideoCapture object with the GStreamer pipeline
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        # Check if the camera opened successfully
        if cap.isOpened():
            camera_found = True     # Camera is found
            rospy.loginfo("Camera " + str(camera_index) + " Connected!")
            break            

        rospy.sleep(3.0) # Sleep for 1 second

        # If camera is not found, output an error message
        rospy.logwarn("Camera not found. Trying again...")
        camera_index += 1

        if camera_index > 10:
            rospy.logfatal("Camera not found after 10 attempts.")
            faux_camera = True
            camera_found = True


    # Begin the main loop that consistently outputs AR tag corners when running
    while not rospy.is_shutdown():

        if cap.isOpened():                      # Capture image while camera is opened
            # Get the current video feed frame
            ret, img = cap.read()
            
            # Search for Aruco tag
            corners, ids, rejected = cv2.aruco.detectMarkers(img, finalDict)
            corners_msg = Int32MultiArray()

            # Output the detected corners if detected
            if corners:
                out_str = "AR Tag Detected %s" % rospy.get_time()
                corners_msg.data = [1, 1, 1, 1, 1, 1, 1, 1]
            else:
                out_str = "No AR Tag %s" % rospy.get_time()
                corners_msg.data = [0, 0, 0, 0, 0, 0, 0, 0]

        else:
            out_str = "Camera Connection Lost %s" % rospy.get_time()
            
        # Publish to the ROS node
        rospy.loginfo(out_str)
        pub_corners.publish(corners_msg)
        rate.sleep()

    cv2.destroyAllWindows()         # Close everything and release the camera
    cap.release()
    rospy.loginfo("End of program") # This will output to the terminal
