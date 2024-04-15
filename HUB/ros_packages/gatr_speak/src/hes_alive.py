#!/usr/bin/env python

import sys
# Import after printing usage for speed.
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String
import rospy

global voice, volume

voice = 'voice_kal_diphone'
volume = 1.0

def phaseCallback(data):
    global voice, volume

    # Callback for the phase topic
    # s needs to be a char array
    s = data.data
    rospy.loginfo('Saying: %s' % s)

    # Say the phase
    soundhandle.say(s, voice, volume)

    # Wait for the sound to finish
    rospy.sleep(2)

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('speaker', anonymous=True)

    rospy.loginfo("### GATR GPT ###") # This will output to the terminal

    # Subscribers
    subState = rospy.Subscriber("/MP/phase", String, phaseCallback, queue_size=1)

    # Initialize the sound client
    rospy.loginfo("Initializing sound client...") # This will output to the terminal
    soundhandle = SoundClient() # Create a sound client object
    rospy.loginfo("Sound client initialized.") # This will output to the terminal

    rate = rospy.Rate(1)
    rospy.loginfo('Voice: %s' % voice)
    rospy.loginfo('Volume: %s' % volume)

    # Say the initial message
    soundhandle.say('I am alive.', voice, volume)
    rospy.sleep(1)

    rospy.spin()

    rospy.loginfo("Shutting down...") # This will output to the terminal