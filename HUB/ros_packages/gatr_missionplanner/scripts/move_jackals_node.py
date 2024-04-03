#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import random
import os
import math

def spawn_model(model_name, model_xml):
    rospy.wait_for_service('gazebo/spawn_sdf_model')
    spawn_sdf_model = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    pose = Pose()

    # Set random x and y position
    pose.position.x = random.uniform(5, (150*0.3048 - 5))  # Change this value to set robot's initial x position
    pose.position.y = random.uniform(-5, -(150*0.3048 - 5))  # Change this value to set robot's initial y position
    
    # Set random yaw angle
    yaw = random.uniform(0, 360)
    quaternion = quaternion_from_euler(0, 0, yaw)
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]

    rospy.loginfo("Spawned at x = " + str(pose.position.x) + " y = " + str(pose.position.y) + " yaw = " + str(yaw))

    spawn_sdf_model(model_name, model_xml, "", pose, "world")

def move_robot(robot_name, speed):

    # Create a service client for the get_model_state and set_model_state services
    rospy.wait_for_service('/gazebo/get_model_state')
    rospy.wait_for_service('/gazebo/set_model_state')
    get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    # Get the current state of the robot
    resp = get_state(robot_name, "")
    current_state = resp.pose

    # If outside the bounds, change direction
    if current_state.position.x > (150*0.3048 - 5) or current_state.position.x < 5 or current_state.position.y > -5 or current_state.position.y < -(150*0.3048 - 5):
        speed = -speed

    # Get the orientation of the robot in quaternion
    quaternion = (
        current_state.orientation.x,
        current_state.orientation.y,
        current_state.orientation.z,
        current_state.orientation.w)

    # Convert the quaternion into Euler angles (roll, pitch, yaw)
    roll, pitch, yaw = euler_from_quaternion(quaternion)

    new_yaw = yaw
    if random.uniform(0, 1) > 0.9:
        new_yaw = yaw + random.uniform(-math.pi/4, math.pi/4)

    new_quaternion = quaternion_from_euler(roll, pitch, new_yaw)

    # Create a ModelState message and set its position to the current position plus 1.0
    state_msg = ModelState()
    state_msg.model_name = robot_name
    state_msg.pose.position.x = current_state.position.x + speed * math.cos(yaw + math.pi/2)
    state_msg.pose.position.y = current_state.position.y + speed * math.sin(yaw + math.pi/2)
    state_msg.pose.position.z = current_state.position.z
    state_msg.pose.orientation.x = new_quaternion[0]
    state_msg.pose.orientation.y = new_quaternion[1]
    state_msg.pose.orientation.z = new_quaternion[2]
    state_msg.pose.orientation.w = new_quaternion[3]

    # Call the set_model_state service with the new state
    resp = set_state(state_msg)

    return speed

if __name__ == '__main__':

    rospy.loginfo("### GAZEBO JACKAL CONTROLLER NODE ###") # This will output to the terminal

    # Spawn the jackal robot
    rospy.init_node('jackal_controller', anonymous=True)
    with open(os.path.expanduser('~/ALLIGATR/HUB/models/jackal/model.sdf'), 'r') as f:
        model_xml = f.read()
    rospy.loginfo("Spawning Jackal robot...")
    spawn_model('jackal', model_xml)
    with open(os.path.expanduser('~/ALLIGATR/HUB/models/jackal_2/model.sdf'), 'r') as f:
        model_xml = f.read()
    rospy.loginfo("Spawning Jackal_2 robot...")
    spawn_model('jackal_2', model_xml)

    # Jackal movement speed
    speed = 0.005
    speedA = speed
    speedB = speed

    start_time = rospy.get_time()

    # Move for 30 seconds, then stop for a minute
    while not rospy.is_shutdown():

        time = rospy.get_time() - start_time
        if time < 40:
            speedA = move_robot('jackal', speedA)
            speedB = move_robot('jackal_2', speedB)
        elif time < 90:
            continue
        else:
            start_time = rospy.get_time()

        rospy.sleep(0.05)