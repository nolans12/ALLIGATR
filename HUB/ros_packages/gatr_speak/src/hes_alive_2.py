#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String
import subprocess

def phaseCallback(data):
    # Callback for the phase topic
    s = data.data
    rospy.loginfo('Saying: %s' % s)

    # Say the phase using espeak with the default voice
    subprocess.call(['espeak', s])

    # Wait for the sound to finish
    rospy.sleep(2)

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('speaker', anonymous=True)

    rospy.loginfo("### GATR GPT ###") # This will output to the terminal

    # Subscribers
    subState = rospy.Subscriber("/MP/phase", String, phaseCallback, queue_size=1)

    rospy.loginfo("Listening for messages...") # This will output to the terminal

    rospy.spin()

    rospy.loginfo("Shutting down...") # This will output to the terminal
