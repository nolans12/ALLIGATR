#!/usr/bin/env python2 
# <- This is the shebang line which tells the OS which interpreter to use
# Be aware that the tutorial said to use python2, however we built it for py3 I believe

import rospy

if __name__ == '__main__': # <- Executable 
    rospy.init_node("test_node") # Initialize the ROS node

    rospy.loginfo("Hello from test node") # This will output to the terminal
    rospy.logwarn("This is an error") # This will output to the terminal as a warning message font
    rospy.logerr("This is an error") # This will output to the terminal as an error message font

    rospy.sleep(1.0) # Sleep for 1 second

    rospy.loginfo("End of program") # This will output to the terminal
