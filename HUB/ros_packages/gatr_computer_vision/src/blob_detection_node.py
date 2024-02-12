#!/usr/bin/env python2 
# <- This is the shebang line which tells the OS which interpreter to use
# Be aware that the tutorial said to use python2, however we built it for py3 I believe

import rospy
from std_msgs.msg import String


if __name__ == '__main__': # <- Executable 

    rospy.init_node("blob_detection_node") # Initialize the ROS node

    rospy.loginfo("############# BLOB DETECTION NODE #################") # This will output to the terminal

    # This is how to initialize a publisher
    rospy.loginfo("Initializing ROS connection...")
    pub = rospy.Publisher('blob_cenroid', String, queue_size=10)
    #rospy.init_node('blob_detection_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # Now that ROS connection is established, begin searching for the camera
    rospy.loginfo("Establishing camera connection...")

    camera_found = False
    faux_camera = False
    attempts = 0

    while not camera_found:

        # Implement code to check for camera connection here



        rospy.sleep(3.0) # Sleep for 1 second

        # If camera is not found, output an error message
        rospy.logwarn("Camera not found. Retrying...")
        attempts += 1

        if attempts > 5:
            rospy.logfatal("Camera not found after 5 attempts. Providing default centroid output...")
            faux_camera = True
            camera_found = True

    # Begin the main loop that consistently outputs a centroid index when running
    while not rospy.is_shutdown():

        #Quick override to send a default value if no camera was found
        if faux_camera:
            hello_str = "hello world %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            pub.publish(hello_str)
            rate.sleep()


    rospy.loginfo("End of program") # This will output to the terminal
