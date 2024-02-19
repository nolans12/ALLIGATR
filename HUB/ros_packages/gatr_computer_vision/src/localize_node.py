#!/usr/bin/env python 
# <- This is the shebang line which tells the OS which interpreter to use
# This node will pull AR tag estimates and the altitude of the drone and return the relative localization measurements in meters
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray


# Localization function
def localize():
    pass

# Callback function that will execute whenever data is received
def callback(data):
    # Echo data to the ROS node
    out_str = "Data Received"
    rospy.loginfo(out_str)
    pubCoord.publish(data)      # Echo data
    rate.sleep()


if __name__ == '__main__': # <- Executable 

    rospy.init_node("Localize_Node") # Initialize the ROS node

    rospy.loginfo("############# LOCALIZATION NODE #################") # This will output to the terminal

    # This is how to initialize a publisher
    rospy.loginfo("Initializing ROS connection...")
    
    ################## Publisher Definitions ###########################
    pubCoord = rospy.Publisher('rel_coord', Int32MultiArray, queue_size=10)

    ################## Subscriber Definitions ###########################
    subCorners = rospy.Subscriber('AR_corners', Int32MultiArray, callback)
    #subBlob = rospy.Subscriber('Blob_Centroid', Int32MultiArray, callback)

    ####################################################################
    rate = rospy.Rate(10) # 10hz
    rospy.sleep(1.0)
    

    # Begin the main loop that consistently outputs Localization estimates
    while not rospy.is_shutdown():
        # Spin so the script keeps looking for messages
        rospy.spin()
            


    rospy.loginfo("End of localization program") # This will output to the terminal
