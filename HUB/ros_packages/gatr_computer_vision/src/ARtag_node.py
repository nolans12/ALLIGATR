#!/usr/bin/env python 
# <- This is the shebang line which tells the OS which interpreter to use
# Be aware that the tutorial said to use python2, however we built it for py3 I believe
import rospy
import cv2
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# Image callback for received image
def callback(data):
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
   
  # Convert ROS Image message to OpenCV image
  img = br.imgmsg_to_cv2(data)

  corners_msg = Int32MultiArray()

  # Run AR tag detection
  corners_msg.data = processImg(img)
   
  # Display image
  cv2.imshow("camera", img)

  pub_corners.publish(corners_msg)
   
  cv2.waitKey(1)

# Function that checks a given image for an AR tag and returns corners if its found
def processImg(img):
    if img is None:
        out_str = "Camera Connection Lost %s" % rospy.get_time()
        corners = [0, 0, 0, 0, 0, 0, 0, 0]
    else:            
        # Search for Aruco tag
        corners, ids, rejected = cv2.aruco.detectMarkers(img, finalDict)

        # Output the detected corners if detected
        if corners:
            # Format the corners as an array
            firstCorners = corners[0][0]
            topLeft = firstCorners[1]
            topRight = firstCorners[2]
            bottomRight = firstCorners[3]
            bottomLeft = firstCorners[0]
            testArr = [int(topLeft[0]), int(topLeft[1]), int(topRight[0]), int(topRight[1]), int(bottomRight[0]), int(bottomRight[1]), int(bottomLeft[0]), int(bottomLeft[1])]

            out_str = "AR Tag Detected %s" % rospy.get_time()
            corners = testArr
        else:
            out_str = "No AR Tag %s" % rospy.get_time()
            corners = [0, 0, 0, 0, 0, 0, 0, 0]

        rospy.loginfo(out_str)
        return corners
    
# Highlight the detected markers
def aruco_display(corners, image):
    if(len(corners) > 0): # Are any aruco tags detected
        
        for (markerCorner, markerID) in zip(corners, ids):
            corners = markerCorner.reshape((4,2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners  # Get the corners

            # Cast the data to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            # Draw the lines for the AR tag detection
            cv2.line(image, topLeft, topRight, (0, 0, 255), 2)
            cv2.line(image, topRight, bottomRight, (0, 0, 255), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 0, 255), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 0, 255), 2)

    return image


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
    pub_image = rospy.Publisher('Secondary_Video', Image, queue_size=10)

    ####################################################################

    ################## Subscriber Definitions ###########################
    #sub_img = rospy.Subscriber('webcam/image_raw', )
    ####################################################################

    rate = rospy.Rate(10) # 10hz

    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # Now that ROS connection is established, begin searching for the camera
    rospy.loginfo("Establishing camera connection...")

    # Search for camera, if not found open faux camera
    camera_found = False
    faux_camera = False
    attempts = 0

    # Try to open the 0 index for the secondary camera
    camera_index = 0   

    while not camera_found and not faux_camera:
        # Setup the GStreamer Pipeline
        #pipeline = f'nvarguscamerasrc sensor-id={camera_index} ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080, format=(string)NV12, framerate=(fraction)15/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink'
        pipeline = 'nvarguscamerasrc sensor-id=' + str(camera_index) + ' ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080, format=(string)NV12, framerate=(fraction)15/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink'

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
        sub_img = rospy.Subscriber('webcam/image_raw', Image, callback)
        rospy.spin()


    # Begin the main loop that consistently outputs AR tag corners when running
    while not rospy.is_shutdown():
        # Output message
        corners_msg = Int32MultiArray()

        if cap.isOpened():                          # Capture image while camera is opened
            # Get the current video feed frame
            ret, img = cap.read()

            # Publish image message to image topic
            pub_image.publish(br.cv2_to_imgmsg(img))

            # Output message with corners
            corners_msg.data = processImg(img)

        # If the camera is connected through a faux camera in ROS
        elif faux_camera:
            # Wait for received image with the callback
            pass

        else:
            out_str = "Camera Connection Lost %s" % rospy.get_time()
            corners_msg.data = [0, 0, 0, 0, 0, 0, 0, 0]
            
        # Publish to the ROS node
        pub_corners.publish(corners_msg)
        rate.sleep()

    cv2.destroyAllWindows()         # Close everything and release the camera
    cap.release()
    rospy.loginfo("End of program") # This will output to the terminal
