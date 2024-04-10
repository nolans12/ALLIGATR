#!/usr/bin/env python
import rospy
from mavros_msgs.srv import ParamSet
from mavros_msgs.msg import ParamValue
from std_msgs.msg import String
from mavros_msgs.msg import State

def set_parameter(param_id, param_value):
    """
    Set a parameter on the vehicle.
    :param param_id: The ID of the parameter
    :param param_value: The value to set the parameter to
    """
    rospy.wait_for_service('/mavros/param/set')
    try:
        param_set = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        param = ParamValue()
        #param.real = param_value  # Set this for real (float) parameters
        param.integer = param_value  # Uncomment and use this line for integer parameters
        response = param_set(param_id, param)
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def mavros_state_callback(data):
    """
    Callback function for MAVROS state messages.
    Checks if MAVROS is connected and sets a parameter.
    """
    if data.connected:
        rospy.loginfo("MAVROS is now connected. Setting parameter...")
        # Set your parameter and value here
        if set_parameter("MAV_0_RATE", 57600):  # Example parameter
            rospy.loginfo("Parameter set successfully.")
        else:
            rospy.logerr("Failed to set parameter.")
        # After setting parameter, you might want to shutdown the node
        rospy.signal_shutdown("Parameter set. Node is shutting down.")

def listener():
    """
    Initializes ROS node and subscribes to the /mavros/state topic.
    """
    rospy.init_node('set_mavros_param', anonymous=True)
    rospy.Subscriber("/mavros/state", State, mavros_state_callback)
    rospy.loginfo("Waiting for MAVROS connection...")
    rospy.spin()

if __name__ == '__main__':
    listener()
