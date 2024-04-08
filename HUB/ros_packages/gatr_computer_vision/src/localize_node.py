#!/usr/bin/env python 
# <- This is the shebang line which tells the OS which interpreter to use
# This node will pull AR tag estimates and the altitude of the drone and return the relative localization measurements in meters
import rospy
import numpy as np
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

# Global current state variables
AR_CORNERS_A = Int32MultiArray()
AR_CORNERS_A.data = [0, 0, 0, 0, 0, 0, 0, 0]

AR_CORNERS_B = Int32MultiArray()
AR_CORNERS_B.data = [0, 0, 0, 0, 0, 0, 0, 0]

# Global Drone State
global PITCH, ROLL, YAW, DRONEX, DRONEY, ALTITUDE, AR_LENGTH, XPIXELS, YPIXELS
PITCH = 0
ROLL = 0
YAW = 0
DRONEX = 0
DRONEY = 0
ALTITUDE = 9.144                           # Assume we localize at 30 ft (9.144 meters)

# Global AR Tag and Camera Data
#AR_LENGTH = 0.2496                          # 24.96 cm AR tag side length for the Gazebo AR tag
AR_LENGTH = 0.615                           # 61.5 cm AR tag side length for the large AR tag
XPIXELS = 1920
YPIXELS = 1080

# Inertially Localize given relative estimates
def inertLocalize(relX, relY):
    # Set the bearing
    p = YAW

    # Rotate the relative measurements into the ENU frame
    # Erel = np.cos(p) * relX + np.sin(p) * relY
    # Nrel = -1*np.sin(p) * relX + np.cos(p) * relY

    # Rotate into the ENU frame using the yaw angle DOUBLE CHECK THIS IS THE CORRECT YAW ANGLE MEASURED FROM EAST
    Erel = np.cos(p) * relY + np.sin(p) * relX
    Nrel = -1*np.cos(p) * relX + np.sin(p) * relY

    # Calculate the RGV inertial position in the local inertial frame frame
    XRGV = DRONEX + Erel
    YRGV = DRONEY + Nrel

    #rospy.loginfo("Drone_x: {}, Drone_y: {}, Drone_z: {}, RGV_x: {}, RGV_y: {}".format(DRONEX, DRONEY, ALTITUDE, XRGV, YRGV))
    return XRGV, YRGV

# Return the relative position of the RGV in the camera frame
def localize(ARCorners):
    # Define the center of the image
    cx = XPIXELS / 2
    cy = YPIXELS / 2

    # Get the centroid coordinates of the AR tag and the corners
    topLeft = (ARCorners.data[0], ARCorners.data[1])
    topRight = (ARCorners.data[2], ARCorners.data[3])
    bottomRight = (ARCorners.data[4], ARCorners.data[5])
    bottomLeft = (ARCorners.data[6], ARCorners.data[7])

    # Centroid
    xf = (topLeft[0] + bottomRight[0]) / 2
    yf = (topLeft[1] + bottomRight[1]) / 2

    # Length of each side of the AR Tag in pixels
    delL1 = np.sqrt((topLeft[0] - topRight[0])**2 + (topLeft[1] - topRight[1])**2)
    delL2 = np.sqrt((topRight[0] - bottomRight[0])**2 + (topRight[1] - bottomRight[1])**2)
    delL3 = np.sqrt((bottomRight[0] - bottomLeft[0])**2 + (bottomRight[1] - bottomLeft[1])**2)
    delL4 = np.sqrt((bottomLeft[0] - topLeft[0])**2 + (bottomLeft[1] - topLeft[1])**2)

    # Average length of the sides of the AR tag in pixels
    delL = (delL1 + delL2 + delL3 + delL4) / 4

    # Define AR Tag scaling factor
    s = AR_LENGTH / delL               # Meters per pixel

    # Relative camera coordinates in pixels
    yc = cy - yf
    xc = xf - cx

    # Calculate the angles using the distance to the RGV in the x and y axes and the height
    alpha = np.arctan(xc * s / ALTITUDE)
    beta = np.arctan(yc * s / ALTITUDE)

    # Relative coordinates in meters
    relX = ALTITUDE * np.tan(alpha + PITCH)
    relY = ALTITUDE * np.tan(beta + ROLL)
    # relX = ALTITUDE * np.tan(alpha)
    # relY = ALTITUDE * np.tan(beta)

    return relX, relY


# RGV A callback PRIMARY SENSOR
def callback_primary_AR_A(data):
    # Echo data to the ROS node
    outData = Float32MultiArray()
    AR_CORNERS_A = data               # Update AR Tag corner estimate

    relX, relY = localize(data)     # Get relative coordinates in meters
    RGVX_inert, RGVY_inert = inertLocalize(relX, relY)

    rospy.loginfo("RGV Inertial - RGVA_x: {}, RGVA_y: {}".format(RGVX_inert, RGVY_inert))
    rospy.loginfo("Drone_x: {}, Drone_y: {}, Drone_z: {}, Drone_Yaw".format(DRONEX, DRONEY, ALTITUDE, YAW))

    outData.data = [RGVX_inert, RGVY_inert]

    pubCoord_primary_A.publish(outData)       # Output estimates
    rate.sleep()

# RGV B callback PRIMARY SENSOR
def callback_primary_AR_B(data):
    # Echo data to the ROS node
    outData = Float32MultiArray()
    AR_CORNERS_B = data               # Update AR Tag corner estimate

    relX, relY = localize(data)     # Get relative coordinates in meters
    RGVX_inert, RGVY_inert = inertLocalize(relX, relY)

    rospy.loginfo("RGV Inertial - RGVB_x: {}, RGVB_y: {}".format(RGVX_inert, RGVY_inert))
    rospy.loginfo("Drone_x: {}, Drone_y: {}, Drone_z: {}, Drone_Yaw".format(DRONEX, DRONEY, ALTITUDE, YAW))

    outData.data = [RGVX_inert, RGVY_inert]

    pubCoord_primary_B.publish(outData)       # Output estimates
    rate.sleep()


# RGV A callback SECONDARY SENSOR
def callback_secondary_AR_A(data):
    # Echo data to the ROS node
    outData = Float32MultiArray()
    AR_CORNERS_A = data               # Update AR Tag corner estimate

    relX, relY = localize(data)     # Get relative coordinates in meters
    RGVX_inert, RGVY_inert = inertLocalize(relX, relY)

    # rospy.loginfo("RGV Inertial - RGVA_x: {}, RGVA_y: {}".format(RGVX_inert, RGVY_inert))
    # rospy.loginfo("Drone_x: {}, Drone_y: {}, Drone_z: {}, Drone_Yaw".format(DRONEX, DRONEY, ALTITUDE, YAW))
    rospy.loginfo("RGV A Detected! RGVA_x: {}, RGVA_y: {}".format(RGVX_inert, RGVY_inert))

    outData.data = [RGVX_inert, RGVY_inert]

    pubCoord_secondary_A.publish(outData)       # Output estimates
    rate.sleep()

# RGV B callback SECONDARY SENSOR
def callback_secondary_AR_B(data):
    # Echo data to the ROS node
    outData = Float32MultiArray()
    AR_CORNERS_B = data               # Update AR Tag corner estimate

    relX, relY = localize(data)     # Get relative coordinates in meters
    RGVX_inert, RGVY_inert = inertLocalize(relX, relY)

    # rospy.loginfo("RGV Inertial - RGVB_x: {}, RGVB_y: {}".format(RGVX_inert, RGVY_inert))
    # rospy.loginfo("Drone_x: {}, Drone_y: {}, Drone_z: {}, Drone_Yaw".format(DRONEX, DRONEY, ALTITUDE, YAW))
    rospy.loginfo("RGV A Detected! RGVA_x: {}, RGVA_y: {}".format(RGVX_inert, RGVY_inert))

    outData.data = [RGVX_inert, RGVY_inert]

    pubCoord_secondary_B.publish(outData)       # Output estimates
    rate.sleep()

# Subscribe to get position data of the drone relative to its instantiated inertial local frame, this is the ENU frame with the origin
# at the point of initialization
def pose_callback(data):
    global DRONEX, DRONEY, ALTITUDE, PITCH, ROLL, YAW

    # Get the pose data
    x = data.pose.position.x
    y = data.pose.position.y
    z = data.pose.position.z

    # Extract quaternion orientation from the message
    orientation_q = data.pose.orientation

    # Convert quaternion to Euler angles (roll, pitch, yaw)
    (ROLL, PITCH, YAW) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    # Set the global variables
    DRONEX = x
    DRONEY = y
    ALTITUDE = z

    # Output the x, y, z position
    rospy.loginfo("Position - x: {}, y: {}, z: {}, Yaw: {}".format(x, y, z, YAW))


if __name__ == '__main__': # <- Executable 

    rospy.init_node("Localize_Node") # Initialize the ROS node

    rospy.loginfo("############# LOCALIZATION NODE #################") # This will output to the terminal

    # This is how to initialize a publisher
    rospy.loginfo("Initializing ROS connection...")
    
    ################## Publisher Definitions ###########################
    pubCoord_primary_A = rospy.Publisher('CV/inert_coord_A', Float32MultiArray, queue_size=1)
    pubCoord_primary_B = rospy.Publisher('CV/inert_coord_B', Float32MultiArray, queue_size=1)
    pubCoord_secondary_A = rospy.Publisher('CV/Secondary/inert_coord_A', Float32MultiArray, queue_size=1)
    pubCoord_secondary_B = rospy.Publisher('CV/Secondary/inert_coord_B', Float32MultiArray, queue_size=1)

    ################## Subscriber Definitions ###########################
    subCorners_primary_A = rospy.Subscriber('CV/Primary/AR_corners_A', Int32MultiArray, callback_primary_AR_A)
    subCorners_primary_B = rospy.Subscriber('CV/Primary/AR_corners_B', Int32MultiArray, callback_primary_AR_B)
    subCorners_secondary_A = rospy.Subscriber('CV/Secondary/AR_corners_A', Int32MultiArray, callback_secondary_AR_A)
    subCorners_secondary_B = rospy.Subscriber('CV/Secondary/AR_corners_B', Int32MultiArray, callback_secondary_AR_B)
    subState = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_callback)

    ####################################################################
    rate = rospy.Rate(10) # 10hz
    rospy.sleep(1.0)
    
    # Begin the main loop that consistently outputs Localization estimates
    while not rospy.is_shutdown():
        # Spin so the script keeps looking for messages
        rospy.spin()

        
    rospy.loginfo("End of localization program") # This will output to the terminal
