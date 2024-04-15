#!/usr/bin/env python 
# <- This is the shebang line which tells the OS which interpreter to use
import rospy
import cv2
import csv
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

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

# Compression Level
# resize the image by this amount
COMPRESS_CONST = 4


# Video file
if saveBool:
    size = (int(1920/COMPRESS_CONST), int(1080/COMPRESS_CONST)) 
    filename = "primaryVideo%s.avi" % rospy.get_time()
    writeObj = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*'XVID'), saveFPS, size)


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


# Process the image, BLOB DETECTION
def processImg(img):
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

    # Not detected
    if not keypoints:
        return -1    # Return negative for no detection

    # Zip the detected centroid arrays
    centroids_x = np.array([])
    centroids_y = np.array([])
    for keypoint in keypoints:
        centroids_x = np.append(centroids_x, keypoint.pt[0])
        centroids_y = np.append(centroids_y, keypoint.pt[1])

    # X and Y centroid coordinates of the detected RGVs
    centroids_x = centroids_x.astype(int)
    centroids_y = centroids_y.astype(int)

    return zip(centroids_x,centroids_y)


if __name__ == '__main__': # <- Executable 
    
    rospy.init_node("Primary_Detection_Node") # Initialize the ROS node

    rospy.loginfo("############# PRIMARY BLOB DETECTION NODE #################") # This will output to the terminal

    # This is how to initialize a publisher
    rospy.loginfo("Initializing ROS connection...")
    
    ################## Publisher Definitions ###########################
    pub_centroid = rospy.Publisher('CV/Primary/centroid', Int32MultiArray, queue_size=1)     # RGV B
    pub_image = rospy.Publisher('CV/Primary_Video', Image, queue_size=1)

    ####################################################################

    ################## Subscriber Definitions ###########################
    #sub_img = rospy.Subscriber('webcam/image_raw', )
    ####################################################################

    rate = rospy.Rate(10) # 10hz

    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # Setup SimpleBlobDetector parameters.
    rospy.loginfo("Initializing Blob Detection Parameters...")
    params = cv2.SimpleBlobDetector_Params()

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Now that ROS connection is established, begin searching for the camera
    rospy.loginfo("Establishing camera connection...")

    # Search for camera, if not found open faux camera
    camera_found = False
    faux_camera = False
    attempts = 0   

    phase_max = 10 #Number of frames to smooth out the detection
    phase_smoother_counter_A = 2*phase_max
    phase_smoother_counter_B = 2*phase_max

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
                # Output message with corners
                corners_msg_A.data, corners_msg_B.data = processImg(img)

            # Save image
            if saveBool:
                if frameCount % (camFPS // saveFPS) == 0: 
                    # Compress image by resizing
                    compressed_frame = cv2.resize(img, (int(1920/COMPRESS_CONST), int(1080/COMPRESS_CONST)))

                    # Save video
                    writeObj.write(compressed_frame)

            if frameCount >= 60:
                frameCount = 0  # Reset frame count

        # If the camera is connected through a faux camera in ROS
        elif faux_camera:
            # Wait for received image with the callback
            pass
        else:
            out_str = "Camera Connection Lost %s" % rospy.get_time()
            

    writeObj.release()
    cv2.destroyAllWindows()         
    cap.release()
    rospy.loginfo("End of program") 
