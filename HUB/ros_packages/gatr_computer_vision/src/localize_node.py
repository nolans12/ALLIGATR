#!/usr/bin/env python 
# <- This is the shebang line which tells the OS which interpreter to use
# This node will pull AR tag estimates and the altitude of the drone and return the relative localization measurements in meters
import rospy
import numpy as np
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray

# Global current state variables
AR_CORNERS = Int32MultiArray()
AR_CORNERS.data = [0, 0, 0, 0, 0, 0, 0, 0]
BLOB_CENTROID = [0, 0]
THETA = 0
PHI = 0
ALTITUDE = 9.144                           # Assume we localize at 30 ft (9.144 meters)

# Global Data
#AR_LENGTH = 0.15875                     # AR Tag length in meters
AR_LENGTH = 0.2496                       # Gazebo sim value
XPIXELS = 1920
YPIXELS = 1080

# Localization function
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
    relX = ALTITUDE * np.tan(alpha + THETA)
    relY = ALTITUDE * np.tan(beta + PHI)

    return relX, relY

# Callback function that will execute whenever data is received
def callbackAR(data):
    # Echo data to the ROS node
    outData = Float32MultiArray()
    AR_CORNERS = data               # Update AR Tag corner estimate

    if(data.data[0] == 0):          # Condition for no AR tag detection
        outData.data = [0.0, 0.0]
        out_str = "No AR Data Received"
    else:
        relX, relY = localize(AR_CORNERS)     # Get relative coordinates in meters
        outData.data = [relX, relY]
        out_str = "AR Data Received"

    rospy.loginfo(out_str)
    pubCoord.publish(outData)       # Output estimates
    rate.sleep()


def callbackBlob(data):
    # Echo data to the ROS node
    out_str = "Blob Data Received"
    rospy.loginfo(out_str)
    pubCoord.publish(data)      # Echo data
    rate.sleep()


if __name__ == '__main__': # <- Executable 

    rospy.init_node("Localize_Node") # Initialize the ROS node

    rospy.loginfo("############# LOCALIZATION NODE #################") # This will output to the terminal

    # This is how to initialize a publisher
    rospy.loginfo("Initializing ROS connection...")
    
    ################## Publisher Definitions ###########################
    pubCoord = rospy.Publisher('rel_coord', Float32MultiArray, queue_size=10)

    ################## Subscriber Definitions ###########################
    subCorners = rospy.Subscriber('AR_corners', Int32MultiArray, callbackAR)
    #subBlob = rospy.Subscriber('Blob_Centroid', Int32MultiArray, callbackBlob)
    #subIMU = rospy.Subscriber('MAVROS/Something, Int32MultiArray, callbackIMU)

    ####################################################################
    rate = rospy.Rate(10) # 10hz
    rospy.sleep(1.0)
    

    # Begin the main loop that consistently outputs Localization estimates
    while not rospy.is_shutdown():
        # Spin so the script keeps looking for messages
        rospy.spin()
            


    rospy.loginfo("End of localization program") # This will output to the terminal
