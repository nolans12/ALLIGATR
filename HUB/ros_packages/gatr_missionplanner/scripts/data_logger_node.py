#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from datetime import datetime
import os
import csv

#### Description: ####
# This node will log all of the data of the drone and where it localizes
# Folder format: data_YYYY_MM_DD_HH_MM_SS
# Files: rgvA_detections.csv, rgvB_detections.csv, drone_state_hist.csv


# Global file handles
drone_state_hist_file = None
rgvA_detections_file = None
rgvB_detections_file = None

# Global phase variable
PHASE = "STANDBY"

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
    global drone_state_hist_file
    writer = csv.writer(drone_state_hist_file)
    writer.writerow([x, y, z, roll, pitch, yaw, PHASE, ros_now])
    
# rgv A callback
def callbackrgvA(data):
    # Writes the data to the rgvA_detections.csv file
    # data in the form of [rgvX, rgvY, phase, Time]

    # Get the current time
    ros_now = rospy.get_time()  # This is the time in seconds since the start of the node

    global rgvA_detections_file
    writer = csv.writer(rgvA_detections_file)
    writer.writerow([data.data[0], data.data[1], PHASE, ros_now])

# rgv B callback
def callbackrgvB(data):
    # Writes the data to the rgvB_detections.csv file
    # data in the form of [rgvX, rgvY, phase, Time]

    # Get the current time
    ros_now = rospy.get_time()  # This is the time in seconds since the start of the node

    global rgvB_detections_file
    writer = csv.writer(rgvB_detections_file)
    writer.writerow([data.data[0], data.data[1], PHASE, ros_now])

def callbackphase(data):
    global PHASE
    PHASE = data.data

if __name__ == '__main__':

    # Initialize the node
    rospy.init_node('data_logger_node', anonymous=True)

    rospy.loginfo("### DATA LOGGER ###") # This will output to the terminal

    # Subscribers
    subState = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_callback)
    subrgvA = rospy.Subscriber('CV/inert_coord_A', Float32MultiArray, callbackrgvA)
    subrgvB = rospy.Subscriber('CV/inert_coord_B', Float32MultiArray, callbackrgvB)
    subphase = rospy.Subscriber('MP/phase', String, callbackphase)

    # Hyperparameters
    save_location = os.path.expanduser("~/ALLIGATR/HUB/data")

    # Create a new directory for the data
    data_dir = create_directory(save_location)
    rospy.loginfo("New data directory created at: " + data_dir)

    drone_state_hist_file = open(os.path.join(data_dir, "drone_state_hist.csv"), 'w')
    check_file(drone_state_hist_file)
    rgvA_detections_file = open(os.path.join(data_dir, "rgvA_detections.csv"), 'w')
    check_file(rgvA_detections_file)
    rgvB_detections_file = open(os.path.join(data_dir, "rgvB_detections.csv"), 'w')
    check_file(rgvB_detections_file)

    # Write the headers to the files
    drone_writer = csv.writer(drone_state_hist_file)
    drone_writer.writerow(["X", "Y", "Z", "Roll", "Pitch", "Yaw", "Time"])

    rgvA_writer = csv.writer(rgvA_detections_file)
    rgvA_writer.writerow(["rgvX", "rgvY", "Time"])

    rgvB_writer = csv.writer(rgvB_detections_file)
    rgvB_writer.writerow(["rgvX", "rgvY", "Time"])

    # Set ros rate to 10 hz
    rate = rospy.Rate(10)
    rospy.spin()

    # Main loop, just check for callbacks on the subscribers
    while not rospy.is_shutdown():

        rospy.sleep(0.1)

    rospy.logfatal("Data logger node stopped")
        
    # Close the CSV files
    drone_state_hist_file.close()
    rgvA_detections_file.close()
    rgvB_detections_file.close()
