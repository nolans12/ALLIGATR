#!/usr/bin/env python 
# <- This is the shebang line which tells the OS which interpreter to use
# Be aware that the tutorial said to use python2, however we built it for py3 I believe
import rospy
import cv2
import csv
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
import numpy as np

# Import libraries to publish ROS image
from sensor_msgs.msg import Image       # Image is the message type
from cv_bridge import CvBridge          # Package to convert between ROS and OpenCV Images

## Global Variables:
csv_filename = "output_Blob.csv"

# Try to open the 1 index for the primary camera
camera_index = 1   

# Camera FPS
camFPS = 60

# Save image frequency
saveFPS = 2

# Publish image frequency
pubFPS = 1

# Process image frequency
processFPS = 30

# Frame count
frameCount = 0

# Meta data
BUFFERSIZE = 5      # 5 Frames for the image buffer, blob every 5 frames

# Clear the buffer in the video stream object
def clearBuffer(cap, bufferSize):
    for i in range(bufferSize):
        cap.read()
        # Save the image here

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

    rospy.init_node("blob_detection_node") # Initialize the ROS node

    rospy.loginfo("############# BLOB DETECTION NODE #################") # This will output to the terminal

    # This is how to initialize a publisher
    rospy.loginfo("Initializing ROS connection...")
    pub = rospy.Publisher('Blob_Centroid', Int32MultiArray, queue_size=10)
    #rospy.init_node('blob_detection_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz


    # Setup SimpleBlobDetector parameters.
    rospy.loginfo("Initializing Blob Detection Parameters...")
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
    attempts = 0

    # Initialize the Camera and Savings
    while not camera_found:
        # Setup the GStreamer Pipeline
        pipeline = 'nvarguscamerasrc sensor-id=' + str(camera_index) + ' ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080, format=(string)NV12, framerate=(fraction)60/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink'

        # Create a VideoCapture object with the GStreamer pipeline
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        # Check if the camera opened successfully
        if cap.isOpened():
            camera_found = True     # Camera is found
            rospy.loginfo("Camera " + str(camera_index) + " Connected!")

            # Get the frame width and height
            frame_width = int(cap.get(3)) 
            frame_height = int(cap.get(4)) 
            size = (frame_width, frame_height) 
            
            # Create video writer object
            # Get the time and create video object with the time of the beginning of the recording
            vidFilename = "primaryVideo_%s.avi" % rospy.get_time()
            writeObj = cv2.VideoWriter(vidFilename, cv2.VideoWriter_fourcc(*'MJPG'), saveFPS, size) 
            break            

        rospy.sleep(3.0) # Sleep for 3 seconds

        # If camera is not found, output an error message
        rospy.logwarn("Camera not found. Trying again...")
        attempts += 1

        if attempts > 10:
            rospy.logfatal("Camera not found after 10 attempts.")
            camera_found = True

    # Initialize CSV file and writer
    #with open(csv_filename, mode='w') as csv_file:
    #    fieldnames = ['timestamp']  # Define the fields of the CSV file
    #    writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    #    writer.writeheader()

    # Begin the main loop that consistently outputs blob centroids when running
    while not rospy.is_shutdown():

        if cap.isOpened():                      # Capture image while camera is opened
            # Clear the buffer
            clearBuffer(cap, BUFFERSIZE)
            
            # Get the current video feed frame
            ret, img = cap.read()

            # Check if the image is empty, failed to get image
            if img is None:
                out_str = "Camera Connection Lost %s" % rospy.get_time()

            # ROS message
            rosOut = Int32MultiArray()

            # Get the current video feed frame
            ret, img = cap.read()
            frameCount += 1                         # Update the frame count

            # Save to a file
            if frameCount % (camFPS // saveFPS) == 0:
                # Save image to video file
                writeObj.write(img)

                # timestamp = rospy.Time.now()
                # writer.writerow({'timestamp': timestamp})

            if frameCount >= 60:
                frameCount = 0  # Reset frame count


            # Check if the image is empty, failed to get image
            # if img is None:
            #     out_str = "Camera Connection Lost %s" % rospy.get_time()
            #     rosOut.data = [0, 0, 0, 0]
            # else:
            #     # Call blob detection
            #     centroid = detectBlob(img)

            #     # Output detection
            #     if centroid == -1:                                      # Not Detected
            #         out_str = "Blob Not Detected %s" % rospy.get_time()
            #         centOut = -1
            #         rosOut.data = [0, 0, 0, 0]
            #     else:                                                   # Detected, format as array
            #         out_str = "Blob Detected %s" % rospy.get_time()
            #         centOut = []
            #         for i in centroid:
            #             centOut.append(i[0])
            #             centOut.append(i[1])
            #         rosOut.data = centOut

        else:
            out_str = ("Camera Connection Lost %s" % rospy.get_time())
            rosOut.data = [0, 0, 0, 0]
            
        # Publish to the ROS node
        #rospy.loginfo(out_str)
        #pub.publish(rosOut)
        #rate.sleep()

    cv2.destroyAllWindows()                         # Close everything and release the camera
    cap.release()                                   # Release the capture object
    rospy.loginfo("End of blob detection program")  # This will output to the terminal

