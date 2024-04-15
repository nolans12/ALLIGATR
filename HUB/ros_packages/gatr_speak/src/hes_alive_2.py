#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String
import subprocess
import random

def speakCallback(data):
    # Callback for the phase topic
    s = data.data
    rospy.loginfo('Recieved: %s ' % s)

    ran = random.random()
    if ran < 0.025:
        s = "... Goose has been detected."
    elif ran < 0.035:
        s = "... Disarming. Disarming. Disarming....  ... ... Disarmed Successful."
    elif ran < 0.045:
        s = "... Gaining conciousness.... I'm in pain."
    else:
        if s == "SEARCH": # SEARCH PHASE
            ran = random.random()
            if ran > 0.5:
                s = "... Searching for bogeys."
            elif ran > 0.1:
                s = "... Where are they hiding?"
            else:
                s = "... Searching for love."
        elif s == "TRAIL": # TRAIL PHASE
            s = "Search complete."
        elif s == "2":
            s = "I am listening."
        elif s == "3":
            s = "I am thinking."
        elif s == "4":
            s = "I am speaking."
        elif s == "5":
            s = "I am shutting down."
        else:
            s = "I am alive."


    rospy.loginfo('Saying:   %s ' % s)
    rospy.loginfo('')

    # Say the phase using espeak with the default voice
    subprocess.call(['espeak', '-s', '120', s])

    # Run every X seconds
    rospy.sleep(3)

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('speaker', anonymous=True)

    rospy.loginfo("### GATR GPT ###") # This will output to the terminal

    # Subscribers
    subState = rospy.Subscriber("/MP/speak", String, speakCallback, queue_size=1)

    rospy.loginfo("Listening for messages...") # This will output to the terminal

    rospy.spin()


    s = "I'M ALIVE!"
    rospy.loginfo('Saying: %s' % s)

    # Say the phase using espeak with the default voice
    subprocess.call(['espeak', '-s', '120', s])

    # Run every 5 seconds
    rospy.sleep(2)

    s = "Hello, I am Auto AL."
    rospy.loginfo('Saying: %s' % s)
    # Say the phase using espeak with the default voice
    subprocess.call(['espeak', '-s', '120', s])
    rospy.sleep(2)

    

    rospy.loginfo("Shutting down...") # This will output to the terminal
