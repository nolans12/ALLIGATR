#!/usr/bin/env python2 
# <- This is the shebang line which tells the OS which interpreter to use
# Be aware that the tutorial said to use python2, however we built it for py3 I believe
import rospy
import cv2
from std_msgs.msg import String


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
    pub = rospy.Publisher('AR_corners', String, queue_size=10)
    #rospy.init_node('blob_detection_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # Now that ROS connection is established, begin searching for the camera
    rospy.loginfo("Establishing camera connection...")

    camera_found = False
    faux_camera = False
    attempts = 0

    while not camera_found:

        # Implement code to check for camera connection here
        camera_index = 0    # Webcam

        # Create a VideoCapture object using the webcam
        cap = cv2.VideoCapture(camera_index)

        # Check if the camera opened successfully
        if cap.isOpened():
            camera_found = True     # Camera is found
            rospy.loginfo("Camera Connected")
            break            

        rospy.sleep(3.0) # Sleep for 1 second

        # If camera is not found, output an error message
        rospy.logwarn("Camera not found. Retrying...")
        attempts += 1

        if attempts > 5:
            rospy.logfatal("Camera not found after 5 attempts. Providing default centroid output...")
            faux_camera = True
            camera_found = True


    # Begin the main loop that consistently outputs AR tag corners when running
    while not rospy.is_shutdown():

        if cap.isOpened():                      # Capture image while camera is opened
            # Get the current video feed frame
            ret, img = cap.read()
            
            # Search for Aruco tag
            corners, ids, rejected = cv2.aruco.detectMarkers(img, finalDict)

            # Output the detected corners
            if corners:
                out_str = "AR Tag Detected %s" % rospy.get_time()
            else:
                out_str = "No AR Tag %s" % rospy.get_time()

        else:
            out_str = "Camera Connection Lost %s" % rospy.get_time()
            
        # Publish to the ROS node
        rospy.loginfo(out_str)
        pub.publish(out_str)
        rate.sleep()


    rospy.loginfo("End of program") # This will output to the terminal
