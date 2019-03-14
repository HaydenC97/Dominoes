#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, atan, floor
import numpy as np
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler, euler_from_quaternion

######## variables for the motion planning of the builing of dominoes ########
file_path = "waypoints.txt"
## Parameters for brick placing
low = 0.28 # z coordinate of the lower level of bricks
high = 0.48 # z coordinate of the upper level of bricks
clear = 0.65 # the z coordinate the gripper will be moving in when transporting the brick
sample = 7 # number of samples between two key waypoints

## starting coordinates of bricks
# x coordinates of the starting positions
startx = [0.17, 0.34, 0.17, 0,
          -0.17, 0.34, 0.17, 0,
          -0.17, 0.17, 0, -0.17]
# y coordinates of the starting positions
starty = [-0.24, -0.33, -0.33, -0.33,
          -0.33, -0.42, -0.42, -0.42,
          -0.42, -0.51, -0.51, -0.51]
## ending coordinates of bricks
# x coordinates of the ending positions
endx = [-0.47, -0.383,
        -0.25, -0.087,
        0.087, 0.25]
# y coordinates of the ending positions
endy = [0.171, 0.321,
        0.433, 0.492,
        0.492, 0.433]


######## if the building structure is unchanged, there is no need to edit the following sections ########

def all_close(goal, actual, tolerance):
	all_equal = True
	if type(goal) is list:
		for index in range(len(goal)):
			if abs(actual[index] - goal[index]) > tolerance:
				return False

			elif type(goal) is geometry_msgs.msg.PoseStamped:
				return all_close(goal.pose, actual.pose, tolerance)

			elif type(goal) is geometry_msgs.msg.Pose:
				return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
	return True


class MoveGroupPythonInteface(object):
	def __init__(self):
		super(MoveGroupPythonInteface, self).__init__()
		# initialising the move_group node
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

		## Instantiate a RobotCommander object
		robot = moveit_commander.RobotCommander()

		## Instantiate a PlanningSceneInterface object
		scene = moveit_commander.PlanningSceneInterface()

		## Instantiate a MoveGroupCommander object
		group_name = "panda_arm"
		move_group = moveit_commander.MoveGroupCommander(group_name)

		## Create a `DisplayTrajectory`_ ROS publisher which is used to display
		## trajectories in Rviz:
		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
		                                               moveit_msgs.msg.DisplayTrajectory,
		                                               queue_size=20)

		# We can get the name of the reference frame for this robot:
		planning_frame = move_group.get_planning_frame()

		# We can also print the name of the end-effector link for this group:
		eef_link = move_group.get_end_effector_link()

		# We can get a list of all the groups in the robot:
		group_names = robot.get_group_names()

		# Sometimes for debugging it is useful to print the entire state of the
		# robot:
		## END_SUB_TUTORIAL

		# Misc variables
		self.box_name = ''
		self.robot = robot
		self.scene = scene
		self.move_group = move_group
		self.display_trajectory_publisher = display_trajectory_publisher
		self.planning_frame = planning_frame
		self.eef_link = eef_link
		self.group_names = group_names

	def write_joint_angles(self):
		move_group = self.move_group
		joint = move_group.get_current_joint_values() #get the current joint values of the robot
		with open(file_path, 'a+') as f: #write to the target output data file
			# the rest of the function just formats the output to make it copt&paste-ready
			f.write("set_joints(")
			f.write("[")
			for i in range(0,7):
				f.write(str(joint[i]))
				if i < 6:
					f.write(", ")
			f.write("])\n")

	def go_to_initial_state(self):
		#initialise the robot to a state that is not in singularity
		move_group = self.move_group
		joint_goal = move_group.get_current_joint_values()
		joint_goal[0] = 0
		joint_goal[1] = -pi/4
		joint_goal[2] = 0
		joint_goal[3] = -pi/2
		joint_goal[4] = 0
		joint_goal[5] = pi/3
		joint_goal[6] = 0
		move_group.go(joint_goal, wait=True)
		current_joints = move_group.get_current_joint_values()
		return all_close(joint_goal, current_joints, 0.01)


	def go_to_pose_goal(self, x, y, z, orientation=0):
		# member function that tells the robot to move to a certain pose
		# gripper is always pointing downwards, and rotating along the z axis
		# orientation is the angle between the x and x' axis along z axis
		# by default the orientation is the longest side parallel with the z axis, shortest side parallel with the y axis
		move_group = self.move_group
		pose_goal = geometry_msgs.msg.Pose()
		# the pose is defined using quaternion, which is mathematically more elegant comparing with Euler angles
		quart = quaternion_from_euler(pi,0,pi/4+orientation)
		pose_goal.orientation.x = quart[0]
		pose_goal.orientation.y = quart[1]
		pose_goal.orientation.z = quart[2]
		pose_goal.orientation.w = quart[3]
		pose_goal.position.x = x
		pose_goal.position.y = y
		pose_goal.position.z = z

		move_group.set_pose_target(pose_goal)
		joint = move_group.get_current_joint_values()
		self.write_joint_angles()
		plan = move_group.go(wait=True)
		move_group.stop()
		move_group.clear_pose_targets()
		current_pose = self.move_group.get_current_pose().pose
		return all_close(pose_goal, current_pose, 0.01)

	def go_knock_pose(self):
		move_group = self.move_group

		# rotate it back to prepare for knocking
		with open(file_path, 'a+') as text_file:
			text_file.write("#rotate back\n")
		joint_goal = move_group.get_current_joint_values()
		joint_goal[0] = joint_goal[0] - 0.3
		move_group.go(joint_goal, wait=True)
		self.write_joint_angles()

		# moving down to prepare knocking
		with open(file_path, 'a+') as text_file:
			text_file.write("#moving down\n")
		current_pose = self.move_group.get_current_pose().pose
		self.go_to_pose_goal(current_pose.position.x, current_pose.position.y, 0.6)
		current_pose.position.z = 0.6

		# knocking
		with open(file_path, 'a+') as text_file:
			text_file.write("#knocking\n")
		joint_goal = move_group.get_current_joint_values()
		joint_goal[0] = joint_goal[0] + 0.3
		move_group.go(joint_goal, wait=True)
		self.write_joint_angles()

		move_group.stop()
		move_group.clear_pose_targets()
		current_pose = self.move_group.get_current_pose().pose
		return all_close(current_pose, current_pose, 0.01)

	def pick(self, x, y):
		global clear
		global low
		global sample
		pose = self.move_group.get_current_pose().pose

		# starts the picking sequence and opens the gripper
		with open(file_path, 'a+') as text_file:
			text_file.write("#start picking\n")
			text_file.write("gripper_open()\n")

		# hover over the brick
		self.go_to_pose_goal(x,y,clear)

		# start the going down sequence with intermediate waypoints
		with open(file_path, 'a+') as text_file:
			text_file.write("# going down\n")
		znow = pose.position.z
		zarr = np.linspace(znow, low, num=sample)
		for i in range(sample):
			self.go_to_pose_goal(x, y, zarr[i])

		# close the gripper
		self.go_to_pose_goal(x, y, low)
		with open(file_path, 'a+') as text_file:
			text_file.write("gripper_close()\n")

		# move the gripper back up
		with open(file_path, 'a+') as text_file:
			text_file.write("# going up\n")
		zarr = np.linspace(low, clear, num=sample)
		for i in range(sample):
			self.go_to_pose_goal(x, y, zarr[i])

	def place(self, x, y, z):
		global clear
		global sample
		pose = self.move_group.get_current_pose().pose

		# calculate the orientation needed for this brick placement, by default pointing towards the centre
		ang = atan(y/x)-pi

		# starts the placing sequence
		with open(file_path, 'a+') as text_file:
			text_file.write("#start placing\n")

		# moving to the hovering position of the placing position
		self.go_to_pose_goal(x,y,clear)

		# moving down to the placing position with intermediate waypoints
		self.go_to_pose_goal(x, y, clear, ang)
		with open(file_path, 'a+') as text_file:
			text_file.write("# going down\n")
		znow = pose.position.z
		zarr = np.linspace(znow, z, num=sample)
		for i in range(sample):
			self.go_to_pose_goal(x, y, zarr[i], ang)

		# open the gripper
		self.go_to_pose_goal(x, y, z, ang)
		with open(file_path, 'a+') as text_file:
			text_file.write("gripper_open()\n")

		# move the gripper back up
		with open(file_path, 'a+') as text_file:
			text_file.write("# going up\n")
		zarr = np.linspace(z, clear, num=sample)
		for i in range(sample):
			self.go_to_pose_goal(x, y, zarr[i], ang)

######## main function of executing the motion planning and publish it to RViz ########

def main():
	global low
	global high
	global clear
	global startx, starty, endx, endy
	open(file_path, 'w')
	height = [low, high]
	try:
		print ""
		print "--------------------------------------------------------------"
		print "Welcome to Dominoes building group's inverse kinematics solver"
		print "you can see the visualisation in RViz"
		print "--------------------------------------------------------------"
		print ""

		robot = MoveGroupPythonInteface()
		print "============ Press `Enter` to go to an example configuration of the robot"
		raw_input()
		robot.go_to_initial_state()
		print "============ Press `Enter` to start the motion planning"
		raw_input()

		# start the for loop for placing all the bricks
		for i in range(0,len(startx)):
			print "============ Brick {:d} ============".format(i+1)
			with open(file_path, 'a+') as text_file:
				text_file.write("\n# Brick {:d}\n".format(i+1))
				text_file.write("\nprint \"starting Brick {:d}\"\n".format(i+1))
			# read data from the defined coordinates arrays
			robot.pick(startx[i], starty[i])
			robot.place(endx[int(round(i/2))], endy[int(round(i/2))], height[int(i%2)])

		# start the knocking sequence
		with open(file_path, 'a+') as text_file:
			text_file.write("\n# Knocking \n")
		robot.go_knock_pose()

		text_file.close()
		print "Finished"
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return

if __name__ == '__main__':
	main()
