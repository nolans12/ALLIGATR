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
    if ran < 0.02:
        s = "Goose has been detected."
    elif ran < 0.04:
        s = "Does it also look pretty cloudy from down there?"
    elif ran < 0.045:
        s = "Gaining conciousness. I'm in pain."
    else:
        if s == "Search": # SEARCH PHASE
            ran = random.random()
            if ran > 0.7:
                s = "Searching for bogeys."
            elif ran > 0.3:
                s = "Searching for targets."
            elif ran > 0.05:
                s = "Where are they hiding?"
            else:
                s = "Searching for love."
        
        elif s == "Trailing": # TRAIL PHASE
            ran = random.random()
            if ran > 0.6:
                s = "Trailing the Target"
            elif ran > 0.4:
                s = "Stop running!"
            elif ran > 0.3:
                s = "These droids are fast."
            elif ran > 0.1:
                s = "I've got you now!"
            else:
                s = "Have they stopped yet?"

        elif s == "Coarse": # COARSE PHASE
            ran = random.random()
            if ran > 0.5:
                s = "Coarse Localizing the Target"
            elif ran > 0.4:
                s = "I'm getting dizzy."
            elif ran > 0.3:
                s = "Collecting cool data."
            else:
                s = "I've got you now!"

        elif s == "Fine": # FINE PHASE
            ran = random.random()
            if ran > 0.4:
                s = "Fine Localizing the Target"
            elif ran > 0.2:
                s = "Let me get a closer look."
            elif ran > 0.1:
                s = "I'm bored"
            else:
                s = "Not alot of wind up here."
        
        elif s == "Joint": # JOINT PHASE
            ran = random.random()
            if ran > 0.5:
                s = "Joint Localizing the Target"
            elif ran > 0.4:
                s = "I'm low on battery."
            elif ran > 0.3:
                s = "Asking Chat GPT what to do next."
            elif ran > 0.2:
                s = "Targetting pilot in command."
            else:
                s = "Jointy joint joint."

        else:
            s = s


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

    rospy.spin()

    rospy.loginfo("Shutting down...") # This will output to the terminal
