#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from datetime import datetime
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import csv
import cv2

#### Description: ####
# This node will log all of the data of the drone and where it localizes
# Folder format: data_YYYY_MM_DD_HH_MM_SS
# Files: rgvA_detections.csv, rgvB_detections.csv, drone_state_hist.csv

# Global file handles
drone_state_hist_file = None
drone_global_state_hist_file = None
rgvA_detections_file = None
rgvB_detections_file = None
video_timestamp_file = None
writeObj = None
rgvA_AR_file = None
rgvB_AR_file = None


# Global phase variable
PHASE = "STANDBY"
DRONE_COUNTER = 0
DRONE_COUNTER_G = 0
RGV_COUNTER = 0

# Save FPS for video stream
saveFPS = 1

# Create a new directory for the data
def create_directory(save_location):

    # Get the current date and time
    now = datetime.now()

    # Format the date and time
    formatted_now = now.strftime("%Y_%m_%d_%H_%M_%S")

    # Create the directory name
    data_dir_name = "data_" + formatted_now
    data_dir = save_location + "/" + data_dir_name

    # Create the directory
    os.system("mkdir " + data_dir)
    return data_dir

def check_file(file):
    if file:
        rospy.loginfo(file.name + " opened successfully")
    else:
        rospy.logfatal("Failed to open " + file.name)

# Pose callback
def pose_callback(data):
    # Writes the state of the drone to the drone_state_hist.csv file
    # data in the form of [X, Y, Z, Roll, Pitch, Yaw, phase, Time]

    # Get the pose data
    x = data.pose.position.x
    y = data.pose.position.y
    z = data.pose.position.z

    # Extract quaternion orientation from the message
    orientation_q = data.pose.orientation

    # Convert quaternion to Euler angles (roll, pitch, yaw)
    (roll, pitch, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    # Get the current time
    ros_now = rospy.get_time()  # This is the time in seconds since the start of the node

    # Write the data to the file
    global drone_state_hist_file, DRONE_COUNTER
    writer = csv.writer(drone_state_hist_file)
    writer.writerow([x, y, z, roll, pitch, yaw, PHASE, ros_now])

    # Increment the counter
    DRONE_COUNTER += 1

    # Pose callback
def pose_callback_global(data):
    # Writes the state of the drone to the drone_state_hist.csv file
    # data in the form of [X, Y, Z, Roll, Pitch, Yaw, phase, Time]

    # Get the pose data
    x = data.pose.position.x
    y = data.pose.position.y
    z = data.pose.position.z

    # Extract quaternion orientation from the message
    orientation_q = data.pose.orientation

    # Convert quaternion to Euler angles (roll, pitch, yaw)
    (roll, pitch, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    # Get the current time
    ros_now = rospy.get_time()  # This is the time in seconds since the start of the node

    # Write the data to the file
    global drone_global_state_hist_file, DRONE_COUNTER_G
    writer = csv.writer(drone_state_hist_file)
    writer.writerow([x, y, z, roll, pitch, yaw, PHASE, ros_now])

    # Increment the counter
    DRONE_COUNTER_G += 1
    
# rgv A callback
def callbackrgvA_primary(data):
    # Writes the data to the rgvA_detections.csv file
    # data in the form of [rgvX, rgvY, phase, Time]

    # Get the current time
    ros_now = rospy.get_time()  # This is the time in seconds since the start of the node

    global rgvA_detections_file, RGV_COUNTER
    writer = csv.writer(rgvA_detections_file)
    writer.writerow([data.data[0], data.data[1], PHASE, ros_now, "Primary"])

    # Increment the counter
    RGV_COUNTER += 1

# rgv A callback
def callbackrgvA_secondary(data):
    # Writes the data to the rgvA_detections.csv file
    # data in the form of [rgvX, rgvY, phase, Time]

    # Get the current time
    ros_now = rospy.get_time()  # This is the time in seconds since the start of the node

    global rgvA_detections_file, RGV_COUNTER
    writer = csv.writer(rgvA_detections_file)
    writer.writerow([data.data[0], data.data[1], PHASE, ros_now, "Secondary"])

    # Increment the counter
    RGV_COUNTER += 1

# rgv B callback
def callbackrgvB_primary(data):
    # Writes the data to the rgvB_detections.csv file
    # data in the form of [rgvX, rgvY, phase, Time]

    # Get the current time
    ros_now = rospy.get_time()  # This is the time in seconds since the start of the node

    global rgvB_detections_file, RGV_COUNTER
    writer = csv.writer(rgvB_detections_file)
    writer.writerow([data.data[0], data.data[1], PHASE, ros_now, "Primary"])

    # Increment the counter
    RGV_COUNTER += 1

# rgv B callback
def callbackrgvB_secondary(data):
    # Writes the data to the rgvB_detections.csv file
    # data in the form of [rgvX, rgvY, phase, Time]

    # Get the current time
    ros_now = rospy.get_time()  # This is the time in seconds since the start of the node

    global rgvB_detections_file, RGV_COUNTER
    writer = csv.writer(rgvB_detections_file)
    writer.writerow([data.data[0], data.data[1], PHASE, ros_now, "Secondary"])

    # Increment the counter
    RGV_COUNTER += 1

    # RGV A AR Callback
def callback_primary_AR_A(data):
    global rgvA_AR_file
    # Writes the data to the rgvA_detections.csv file
    # data in the form of [rgvX, rgvY, phase, Time]
    # Get the current time
    ros_now = rospy.get_time()  # This is the time in seconds since the start of the node

    writer = csv.writer(rgvA_AR_file)
    writer.writerow([data.data[0], data.data[1], data.data[2], data.data[3], data.data[4], data.data[5], data.data[6], data.data[7], ros_now, "Primary"])

# RGV A AR Callback
def callback_secondary_AR_A(data):
    global rgvA_AR_file
    # Writes the data to the rgvA_detections.csv file
    # data in the form of [rgvX, rgvY, phase, Time]
    # Get the current time
    ros_now = rospy.get_time()  # This is the time in seconds since the start of the node

    writer = csv.writer(rgvA_AR_file)
    writer.writerow([data.data[0], data.data[1], data.data[2], data.data[3], data.data[4], data.data[5], data.data[6], data.data[7], ros_now, "Secondary"])


def callback_primary_AR_B(data):
    global rgvB_AR_file
    # Writes the data to the rgvA_detections.csv file
    # data in the form of [rgvX, rgvY, phase, Time]
    # Get the current time
    ros_now = rospy.get_time()  # This is the time in seconds since the start of the node

    writer = csv.writer(rgvB_AR_file)
    writer.writerow([data.data[0], data.data[1], data.data[2], data.data[3], data.data[4], data.data[5], data.data[6], data.data[7], ros_now, "Primary"])

def callback_secondary_AR_B(data):
    global rgvB_AR_file
    # Writes the data to the rgvA_detections.csv file
    # data in the form of [rgvX, rgvY, phase, Time]
    # Get the current time
    ros_now = rospy.get_time()  # This is the time in seconds since the start of the node

    writer = csv.writer(rgvB_AR_file)
    writer.writerow([data.data[0], data.data[1], data.data[2], data.data[3], data.data[4], data.data[5], data.data[6], data.data[7], ros_now, "Secondary"])


def callbackphase(data):
    global PHASE
    PHASE = data.data

def callback_SecondaryVid(data):
    global writeObj, video_timestamp_file
    br = CvBridge()
    img = br.imgmsg_to_cv2(data)        # Convert ROS Image message to OpenCV image

    # Save image and time to a file
    ros_now = rospy.get_time()
    writeObj.write(img)
    writer = csv.writer(video_timestamp_file)
    writer.writerow([ros_now])



if __name__ == '__main__':

    # Initialize the node
    rospy.init_node('data_logger_node', anonymous=True)

    rospy.loginfo("### DATA LOGGER ###") # This will output to the terminal

    # Subscribers
    subState = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_callback)
    subState = rospy.Subscriber("/mavros/global_position/local", Odometry, pose_callback_global)
    subrgvA_p = rospy.Subscriber('CV/inert_coord_A', Float32MultiArray, callbackrgvA_primary)
    subrgvB_p = rospy.Subscriber('CV/inert_coord_B', Float32MultiArray, callbackrgvB_primary)
    subrgvA_s = rospy.Subscriber('CV/Secondary/inert_coord_A', Float32MultiArray, callbackrgvA_secondary)
    subrgvB_s = rospy.Subscriber('CV/Secondary/inert_coord_B', Float32MultiArray, callbackrgvB_secondary)
    subCorners_primary_A = rospy.Subscriber('CV/Primary/AR_corners_A', Int32MultiArray, callback_primary_AR_A)
    subCorners_primary_B = rospy.Subscriber('CV/Primary/AR_corners_B', Int32MultiArray, callback_primary_AR_B)
    subCorners_secondary_A = rospy.Subscriber('CV/Secondary/AR_corners_A', Int32MultiArray, callback_secondary_AR_A)
    subCorners_secondary_B = rospy.Subscriber('CV/Secondary/AR_corners_B', Int32MultiArray, callback_secondary_AR_B)
    subphase = rospy.Subscriber('MP/phase', String, callbackphase)
    #sub_img = rospy.Subscriber('CV/Secondary_Video', Image, callback_SecondaryVid)

    # Hyperparameters
    save_location = os.path.expanduser("~/ALLIGATR/HUB/data")

    # Create a new directory for the data
    data_dir = create_directory(save_location)
    rospy.loginfo("New data directory created at: " + data_dir)

    drone_state_hist_file = open(os.path.join(data_dir, "drone_state_hist.csv"), 'w')
    check_file(drone_state_hist_file)
    drone_global_state_hist_file = open(os.path.join(data_dir, "drone_global_state_hist.csv"), 'w')
    check_file(drone_global_state_hist_file)
    rgvA_detections_file = open(os.path.join(data_dir, "rgvA_detections.csv"), 'w')
    check_file(rgvA_detections_file)
    rgvB_detections_file = open(os.path.join(data_dir, "rgvB_detections.csv"), 'w')
    check_file(rgvB_detections_file)

    # AR Tag Detection Save Files
    rgvA_AR_file = open(os.path.join(data_dir, "rgvA_AR.csv"), 'w')
    check_file(rgvA_AR_file)
    rgvB_AR_file = open(os.path.join(data_dir, "rgvB_AR.csv"), 'w')
    check_file(rgvB_AR_file)

    # Video file
    # video_timestamp_file = open(os.path.join(data_dir, "video_timestamp_file.csv"), 'w')
    # check_file(video_timestamp_file)
    # size = (int(1920), int(1080)) 
    # writeObj = cv2.VideoWriter(os.path.join(data_dir, "secondaryVideo.avi"), cv2.VideoWriter_fourcc(*'MJPG'), saveFPS, size)

    # Write the headers to the files
    drone_writer = csv.writer(drone_state_hist_file)
    drone_writer.writerow(["X", "Y", "Z", "Roll", "Pitch", "Yaw", "Time"])

    rgvA_writer = csv.writer(rgvA_detections_file)
    rgvA_writer.writerow(["rgvX", "rgvY", "Time"])

    rgvB_writer = csv.writer(rgvB_detections_file)
    rgvB_writer.writerow(["rgvX", "rgvY", "Time"])

    rgvA_AR_writer = csv.writer(rgvA_AR_file)
    rgvA_AR_writer.writerow(["AR_A"])

    rgvB_AR_writer = csv.writer(rgvB_AR_file)
    rgvB_AR_writer.writerow(["AR_B"])

    # videoTime_writer = csv.writer(video_timestamp_file)
    # videoTime_writer.writerow(["Time"])

    # Set ros rate to 10 hz
    rate = rospy.Rate(1)

    # Wait until the phase is not standby
    # rospy.loginfo("Waiting for phase to change out of STANDBY...")
    # while PHASE == "STANDBY":
    #     rate.sleep()

    # rospy.loginfo("Phase changed to: " + PHASE + ". Data logger node started")

    # Main loop, just check for callbacks on the subscribers
    while not rospy.is_shutdown():

        # Output the number of data points logged
        rospy.loginfo("Drone data points logged: " + str(DRONE_COUNTER))
        rospy.loginfo("Drone global data points logged: " + str(DRONE_COUNTER_G))
        rospy.loginfo("RGV data points logged: " + str(RGV_COUNTER))

        flag = ''
        try:
            flag = str(raw_input("Enter 'q' to quit: "))
        except EOFError:
            pass

        if flag == 'q':
            # Close the CSV files
            drone_state_hist_file.close()
            # Check that csv was succesfully closed
            if drone_state_hist_file.closed:
                rospy.loginfo("drone_state_hist.csv closed successfully")
            else:
                rospy.logerr("Failed to close drone_state_hist.csv")


            rgvA_detections_file.close()
            # Check that csv was succesfully closed
            if rgvA_detections_file.closed:
                rospy.loginfo("rgvA_detections.csv closed successfully")
            else:
                rospy.logerr("Failed to close rgvA_detections.csv")


            rgvB_detections_file.close()
            # Check that csv was succesfully closed
            if rgvB_detections_file.closed:
                rospy.loginfo("rgvB_detections.csv closed successfully")
            else:
                rospy.logerr("Failed to close rgvB_detections.csv")

            rgvA_AR_file.close()
            # Check that csv was succesfully closed
            if rgvA_AR_file.closed:
                rospy.loginfo("rgvA_AR.csv closed successfully")
            else:
                rospy.logerr("Failed to close rgvA_AR.csv")

            rgvB_AR_file.close()
            # Check that csv was succesfully closed
            if rgvB_AR_file.closed:
                rospy.loginfo("rgvB_AR.csv closed successfully")
            else:
                rospy.logerr("Failed to close rgvB_AR.csv")

            rospy.loginfo("CSV files closed. Ending node...")
            
            # Stop spinning and end the node
            rospy.signal_shutdown("Quiting...")

        rate.sleep()

    rospy.spin()

    rospy.logfatal("Data logger node stopped")
        
