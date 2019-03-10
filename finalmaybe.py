#!/usr/bin/env python
"""
A simple script that translates desired gripper width to command for
JointGroupPositionController.
"""
import rospy
import math
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Float64
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel

pause = 2.0 #time between instructions, seconds

def set_joints(gripper,joints):
    for i in range(7):
	joint_publishers[i].publish(joints[i])

    msg.data = gripper
    gripper_publishers.publish(msg)

    rospy.sleep(pause)

def move(joints_start,joints_end):
    joints = joints_start
    msg.data = [0.06,0.06]
    gripper_publishers.publish(msg)

    #start abover
    joints[3] = joints_start[3]+0.5
    for i in range(7):
	joint_publishers[i].publish(joints[i])
    rospy.sleep(pause)

    #go down
    for i in range(7):
	joint_publishers[i].publish(joints_start[i])
    rospy.sleep(pause)
    
    #grip
    msg.data = [0.0,0.0]
    gripper_publishers.publish(msg)
    rospy.sleep(pause)

    #go up
    joints[3] = joints_start[3]+0.5
    for i in range(7):
	joint_publishers[i].publish(joints[i])
    rospy.sleep(pause)
    
    #swing and go above
    joints = joints_end
    joints[3] = joints_end[3]+0.5
    for i in range(7):
	joint_publishers[i].publish(joints[i])
    rospy.sleep(pause)

    #go down
    joints[3] = joints_end[3]
    for i in range(7):
	joint_publishers[i].publish(joints[i])
    rospy.sleep(pause)

    #let go
    msg.data = [0.06,0.06]
    gripper_publishers.publish(msg)
    rospy.sleep(pause)

    #go up
    joints[3] = joints_end[3]+0.05
    for i in range(7):
	joint_publishers[i].publish(joints[i])
    rospy.sleep(pause)


    
if __name__ == '__main__':
    rospy.init_node('joint_gripper')   

    #set initial joint angles and gripper position
    joints = [0, 1*math.pi/8, 0, 1*math.pi/8, 0, 6*math.pi/8, -math.pi/4]
    gripper = [0.0,0.0]

    #load in publishers for arm and gripper
    joint_publishers = [rospy.Publisher('/franka/joint{}_position_controller/command'.format(i), Float64, queue_size=1) for i in range(1, 8)]
    gripper_publishers = rospy.Publisher('/franka/gripper_position_controller/command', Float64MultiArray, queue_size=1)

    msg = Float64MultiArray()
    msg.layout.dim = [MultiArrayDimension('', 2, 1)]
    data = 0
    msg.data = gripper

    start = rospy.Time.now()

    #set starting pose
    rospy.loginfo("Setting initial pose.")
    set_joints(gripper,joints)
    rospy.sleep(pause*5)

    rospy.loginfo("Starting loop.")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
	joints_start = [0.1353202846978174, -0.7912125277208872, -1.341405083949207, -2.7274890168735983, -0.9210803293631155, 2.129154214642026, -0.9071725885544765]
	joints_end = [1.42468996084228, -1.7624944567307683, 1.4259742382795948, -2.2782770742975957, 1.578062949684358, 1.4223444325646437, 1.8091872114550764]
	move(joints_start,joints_end)


