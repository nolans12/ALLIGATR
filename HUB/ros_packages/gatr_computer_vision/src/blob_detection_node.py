#!/usr/bin/env python 
# <- This is the shebang line which tells the OS which interpreter to use
# Be aware that the tutorial said to use python2, however we built it for py3 I believe
import rospy
import cv2
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
import numpy as np


# A function to fix HSV range
def fixHSVRange(h, s, v):
    # Normal H,S,V: (0-360,0-100%,0-100%)
    # OpenCV H,S,V: (0-180,0-255 ,0-255)
    return (180 * h / 360, 255 * s / 100, 255 * v / 100)

def detectBlob(im):
    # Make a copy of Image; find the HSV range; convert it to OpenCV
    # undrestandble range and make a mask from it
    frm=im.copy()
    frm = cv2.cvtColor(frm, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(frm, fixHSVRange(0, 0, 0), fixHSVRange(360, 4.53, 100))

    # Remove the noise
    noise=cv2.dilate(mask,np.ones((5,5)))
    noise=cv2.erode(mask,np.ones((5,5)))
    noise=cv2.medianBlur(mask,7)

    # Change image channels
    mask=cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
    noise=cv2.cvtColor(noise,cv2.COLOR_GRAY2BGR)
    cleanMask=~noise

    # Make a new mask without noise
    centerMask=cv2.cvtColor(cleanMask.copy(),cv2.COLOR_BGR2GRAY)
        
    # Image detector
    keypoints = detector.detect(centerMask)

    centroids_x = np.array([])
    centroids_y = np.array([])
    for keypoint in keypoints:
        centroids_x = np.append(centroids_x, keypoint.pt[0])
        centroids_y = np.append(centroids_y, keypoint.pt[1])

    centroids_x = centroids_x.astype(int)
    centroids_y = centroids_y.astype(int)

    return zip(centroids_x,centroids_y)


if __name__ == '__main__': # <- Executable 

    rospy.init_node("blob_detection_node") # Initialize the ROS node

    rospy.loginfo("############# BLOB DETECTION NODE #################") # This will output to the terminal

    # This is how to initialize a publisher
    rospy.loginfo("Initializing ROS connection...")
    pub = rospy.Publisher('Blob_Centroid', String, queue_size=10)
    #rospy.init_node('blob_detection_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    params.filterByColor = False

    # Change thresholds
    params.minThreshold = 0
    params.maxThreshold = 250

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 530
    params.maxArea = 100000

    # Filter by circularity
    params.filterByCircularity = True
    params.minCircularity = 0.1
    #params.maxCircularity = 0.8

    # Filter by inertia ratio
    params.filterByInertia = True
    params.maxInertiaRatio = 0.94

    # FIlter by convexity
    params.filterByConvexity = False
    params.minConvexity = 0.4

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)


    # Now that ROS connection is established, begin searching for the camera
    rospy.loginfo("Establishing camera connection...")

    camera_found = False
    faux_camera = False
    attempts = 0

    # Try to open the 0 index for the primary camera
    camera_index = 0   

    while not camera_found:
        # Setup the GStreamer Pipeline
        pipeline = f'nvarguscamerasrc sensor-id={camera_index} ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080, format=(string)NV12, framerate=(fraction)15/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink'

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

            # Call blob detection
            centroid = detectBlob(img)
            rosOut = Int32MultiArray()

            # Output the detected blob if detected
            if centroid:
                out_str = "Blob Detected %s" % rospy.get_time()
                rosOut.data = [1, 1, 1, 1, 1, 1, 1, 1]  # Detected
            else:
                out_str = "Blob Not Detected %s" % rospy.get_time()
                rosOut.data = [0, 0, 0, 0, 0, 0, 0, 0]  # Not Detected

        else:
            out_str = ("Camera Connection Lost %s" % rospy.get_time())
            
        # Publish to the ROS node
        rospy.loginfo(out_str)
        pub.publish(rosOut)
        rate.sleep()

    cv2.destroyAllWindows()         # Close everything and release the camera
    cap.release()
    rospy.loginfo("End of program") # This will output to the terminal
