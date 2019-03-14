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
from tf.transformations import euler_from_quaternion, quaternion_from_euler

## Parameters for brick placing
low = 0.28 # z coordinate of the lower level of bricks
high = 0.48 # z coordinate of the upper level of bricks
clear = 0.65 # the z coordinate the gripper will be moving in when transporting the brick
gripper_open = False # global coordinate that indicates whether the gripper is opened
sample = 5 # number of samples between two key waypoints
waypoints = []

## starting coordinates of bricks
startx = [0.17, 0.34, 0.17, 0,\
          -0.17, -0.34, 0.34, 0.17,\
          0, -0.17, -0.34, 0.17,\
          -0, 0.17]
starty = [-0.23, -0.32, -0.32, -0.32,\
          -0.32, -0.32, -0.41, -0.41,\
          -0.41, -0.41, -0.41, -0.5,\
          -0.5, -0.5]
## ending coordinates of bricks
endx = [-0.47, -0.383,\
        -0.25, -0.087,\
        0.087, 0.25,\
        0.383]
endy = [0.171, 0.321,\
        0.433, 0.492,\
        0.492, 0.433,\
        0.321]

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
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

    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
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
    global gripper_open
    move_group = self.move_group
    joint = move_group.get_current_joint_values()
    with open("waypoint.txt", 'a+') as f:
      f.write("set_joints(")
      if gripper_open == True:
        f.write("[0.06, 0.06], ")
      else:
        f.write("[0, 0], ")
      f.write("[")
      for i in range(0,7):
        f.write(str(joint[i]))
        if i < 6:
          f.write(", ")
      f.write("])\n")

  def go_to_joint_state(self):
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
    pose_goal = geometry_msgs.msg.Pose()
    quart = quaternion_from_euler(pi/2,0,pi/4)
    pose_goal.orientation.x = quart[0]
    pose_goal.orientation.y = quart[1]
    pose_goal.orientation.z = quart[2]
    pose_goal.orientation.w = quart[3]
    pose_goal.position.x = -0.5
    pose_goal.position.y = -0.3
    pose_goal.position.z = 3.7
    move_group.set_pose_target(pose_goal)
    joint = move_group.get_current_joint_values()
    self.write_joint_angles()
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def pick(self, x, y):
    global clear
    global low
    global gripper_open
    global sample
    global waypoints
    pose = self.move_group.get_current_pose().pose
    xnow = pose.position.x
    ynow = pose.position.y
    ang = 0
    xarr = np.linspace(xnow,x,num=sample)
    yarr = np.linspace(ynow,y,num=sample)
    with open("waypoint.txt", 'a+') as text_file:
      text_file.write("# open gripper\n")
      gripper_open = True

    '''for i in range(sample):
      self.go_to_pose_goal(xarr[i], yarr[i], clear)'''

    self.go_to_pose_goal(x,y,clear)
    with open("waypoint.txt", 'a+') as text_file:
      text_file.write("# going down\n")
    znow = pose.position.z
    zarr = np.linspace(znow, low, num=sample)
    for i in range(sample):
      self.go_to_pose_goal(x, y, zarr[i])

    with open("waypoint.txt", 'a+') as text_file:
      text_file.write("# close gripper\n")
    self.go_to_pose_goal(x, y, low)
    gripper_open = False

    with open("waypoint.txt", 'a+') as text_file:
      text_file.write("# going up\n\n")
    zarr = np.linspace(low, clear, num=sample)
    for i in range(sample):
      self.go_to_pose_goal(x, y, zarr[i])

  def place(self, x, y, z):
    global clear
    global gripper_open
    global sample
    global waypoints
    pose = self.move_group.get_current_pose().pose
    xnow = pose.position.x
    ynow = pose.position.y
    eulernow = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    ang = atan(y/x)-pi
    xarr = np.linspace(xnow,x,num=sample)
    yarr = np.linspace(ynow,y,num=sample)

    '''for i in range(sample):
      self.go_to_pose_goal(xarr[i], yarr[i], clear)'''


    self.go_to_pose_goal(x,y,clear)
    self.go_to_pose_goal(x, y, clear, ang)
    with open("waypoint.txt", 'a+') as text_file:
      text_file.write("# going down\n")
    znow = pose.position.z
    zarr = np.linspace(znow, z, num=sample)
    for i in range(sample):
      self.go_to_pose_goal(x, y, zarr[i], ang)

    with open("waypoint.txt", 'a+') as text_file:
      text_file.write("# open gripper\n")
    self.go_to_pose_goal(x, y, z, ang)
    gripper_open = True

    with open("waypoint.txt", 'a+') as text_file:
      text_file.write("# going up\n\n")
    zarr = np.linspace(z, clear, num=sample)
    for i in range(sample):
      self.go_to_pose_goal(x, y, zarr[i], ang)

def main():
  global low
  global high
  global clear
  global gripper_open
  global waypoints
  global startx, starty, endx, endy
  gripper_open = True
  open("waypoint.txt", 'w')
  height = [low, high]
  try:
    print ""
    print "----------------------------------------------------------"
    print "Welcome to the MoveIt! MoveGroup Python Interface Tutorial"
    print "----------------------------------------------------------"
    print "Press Ctrl-D to exit at any time"
    print ""
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
    raw_input()
    robot = MoveGroupPythonInteface()
    print robot.move_group.get_current_pose()
    print "============ Press `Enter` to execute a movement using a joint state goal ..."
    robot.go_to_joint_state()
    raw_input()

    for i in range(0,len(startx)):
      print "============ Brick {:d} ============".format(i+1)
      with open("waypoint.txt", 'a+') as text_file:
        text_file.write("\n# Brick {:d}\n".format(i+1))
      robot.pick(startx[i], starty[i])
      robot.place(endx[int(round(i/2))], endy[int(round(i/2))], height[int(i%2)])

    with open("waypoint.txt", 'a+') as text_file:
      text_file.write("\n# Knocking \n")


    text_file.close()
    print "Finished"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
