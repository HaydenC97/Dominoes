#!/usr/bin/env python

#this generates the start and end points of bricks in joint space

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, atan
import numpy as np
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler
## END_SUB_TUTORIAL

#text_file = open("start_end.txt", "w")

low = 0.27
high = 0.47
clear = 0.75
gripper_open = False
sample = 3
waypoints = []

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

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
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

  def go_to_joint_state(self):
    #initialise the robot state
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
    move_group = self.move_group
    pose_goal = geometry_msgs.msg.Pose()
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
    #self.write_joint_angles()
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


  def write_joint_angles(self):
    global gripper_open
    move_group = self.move_group
    joint = move_group.get_current_joint_values()
    with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as f:
      #set_joints([0.06, 0.06],[j1-7])
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

  def write_start(self):
    move_group = self.move_group
    joint = move_group.get_current_joint_values()
    with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as f:
      #set_joints([0.06, 0.06],[j1-7])
      f.write("joints_start = [")
      for i in range(0,7):
        f.write(str(joint[i]))
        if i < 6:
          f.write(", ")
      f.write("]\n")

  def write_end(self):
    move_group = self.move_group
    joint = move_group.get_current_joint_values()
    with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as f:
      #set_joints([0.06, 0.06],[j1-7])
      f.write("joints_end = [")
      for i in range(0,7):
        f.write(str(joint[i]))
        if i < 6:
          f.write(", ")
      f.write("]\n")


  def plan_cartesian_path(self, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group
    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.z += scale * 0.2  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

    ## END_SUB_TUTORIAL


  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL


  def execute_plan(self, plan):
    move_group = self.move_group
    move_group.execute(plan, wait=True)


  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    box_name = self.box_name
    scene = self.scene
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL

  def pick(self, x, y):
    global clear
    global low
    global gripper_open
    global sample
    global waypoints
    pose = self.move_group.get_current_pose().pose

    # with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
    #   text_file.write("# open gripper\n")
    #   gripper_open = True
    # for i in range(sample):
    #   self.go_to_pose_goal(xarr[i], yarr[i], clear)
    #self.go_to_pose_goal(x,y,clear)
      # wpose = self.move_group.get_current_pose().pose
      # waypoints.append(copy.deepcopy(wpose))
    # with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
    #   text_file.write("# going down\n")
    self.go_to_pose_goal(x, y, low)
    self.write_start()
      # wpose = self.move_group.get_current_pose().pose
      # waypoints.append(copy.deepcopy(wpose))

    # with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
    #   text_file.write("# close gripper\n")
    #self.go_to_pose_goal(x, y, low)
    # wpose = self.move_group.get_current_pose().pose
    # waypoints.append(copy.deepcopy(wpose))
    gripper_open = False

    # with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
    #   text_file.write("# going up\n\n")

    #self.go_to_pose_goal(x, y, clear)
      # wpose = self.move_group.get_current_pose().pose
      # waypoints.append(copy.deepcopy(wpose))

  def place(self, x, y, z):
    global clear
    global gripper_open
    global sample
    global waypoints
    pose = self.move_group.get_current_pose().pose
    ang = atan(y/x)-pi
    '''for i in range(sample):
      self.go_to_pose_goal(xarr[i], yarr[i], clear)'''
    #self.go_to_pose_goal(x,y,clear)
      # wpose = self.move_group.get_current_pose().pose
      # waypoints.append(copy.deepcopy(wpose))
    #self.go_to_pose_goal(x, y, clear, ang)
    # with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
    #   text_file.write("# going down\n")
    self.go_to_pose_goal(x, y, z, ang)
    self.write_end()

    # with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
    #   text_file.write("# open gripper\n")
    #self.go_to_pose_goal(x, y, z, ang)
    gripper_open = True

    # with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
    #   text_file.write("# going up\n\n")
    #self.go_to_pose_goal(x, y, clear, ang)
      # wpose = self.move_group.get_current_pose().pose
      # waypoints.append(copy.deepcopy(wpose))

# roll about x-axis is always pi/2
# about z axis is the orientation of the brick

#set_joints([0.06, 0.06],[j1-7])

def main():
  global low
  global high
  global clear
  global gripper_open
  global waypoints
  gripper_open = True
  open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'w')
  try:
    print ""
    print "----------------------------------------------------------"
    print "Welcome to the MoveIt! MoveGroup Python Interface Tutorial"
    print "----------------------------------------------------------"
    print "Press Ctrl-D to exit at any time"
    print ""
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
    raw_input()
    tutorial = MoveGroupPythonInteface()
    print tutorial.move_group.get_current_pose()
    #tutorial.text_file = open("start_end.txt", "w")
    print "============ Press `Enter` to execute a movement using a joint state goal ..."
    tutorial.go_to_joint_state()
    '''print "============ 1"
    with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
      text_file.write("\n# Brick 1\n")
    tutorial.pick(0.34, -0.32)
    tutorial.place(-0.5, 0, 0.27)
    
    print "============ 2"
    with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
      text_file.write("\n# Brick 2\n")
    tutorial.pick(0.17, -0.32)
    tutorial.place(-0.5, 0, high)'''

    print "============ 3"
    with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
      text_file.write("\n# Brick 3\n")
    tutorial.pick(0, -0.32)
    tutorial.place(-0.469846, 0.17101, low)
    with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
      text_file.write("\nmove(joints_start,joints_end)\n")

    print "============ 4"
    with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
      text_file.write("\n# Brick 4\n")
    tutorial.pick(-0.17, -0.32)
    tutorial.place(-0.469846, 0.17101, high)
    with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
      text_file.write("\nmove(joints_start,joints_end)\n")

    '''print "============ 5"
    with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
      text_file.write("\n# Brick 5\n")
    tutorial.pick(-0.34, -0.32)
    tutorial.place(-0.383022, 0.321394, low)'''

    print "============ 6"
    with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
      text_file.write("\n# Brick 6\n")
    tutorial.pick(0.34, -0.41)
    tutorial.place(-0.383022, 0.321394, high)
    with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
      text_file.write("\nmove(joints_start,joints_end)\n")

    print "============ 7"
    with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
      text_file.write("\n# Brick 7\n")
    tutorial.pick(0.17, -0.41)
    tutorial.place(-0.25, 0.433013, low)
    with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
      text_file.write("\nmove(joints_start,joints_end)\n")

    print "============ 8"
    with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
      text_file.write("\n# Brick 8\n")
    tutorial.pick(0, -0.41)
    tutorial.place(-0.25, 0.433013, high)
    with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
      text_file.write("\nmove(joints_start,joints_end)\n")

    print "============ 9"
    with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
      text_file.write("\n# Brick 9\n")
    tutorial.pick(-0.17, -0.41)
    tutorial.place(-0.086824, 0.492404, low)
    with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
      text_file.write("\nmove(joints_start,joints_end)\n")

    '''print "============ 10"
    with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
      text_file.write("\n# Brick 10\n")
    tutorial.pick(-0.34, -0.41)
    tutorial.place(-0.086824, 0.492404, high)
    
    print "============ 11"
    with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
      text_file.write("\n# Brick 11\n")
    tutorial.pick(0.34, -0.5)
    tutorial.place(0.086824, 0.492404, low)'''

    print "============ 12"
    with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
      text_file.write("\n# Brick 12\n")
    tutorial.pick(0.17, -0.5)
    tutorial.place(0.086824, 0.492404, high)
    with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
      text_file.write("\nmove(joints_start,joints_end)\n")

    print "============ 13"
    with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
      text_file.write("\n# Brick 13\n")
    tutorial.pick(0, -0.5)
    tutorial.place(0.25, 0.433013, low)
    with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
      text_file.write("\nmove(joints_start,joints_end)\n")

    print "============ 14"
    with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
      text_file.write("\n# Brick 14\n")
    tutorial.pick(-low, -0.5)
    tutorial.place(0.25, 0.433013, high)
    with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
      text_file.write("\nmove(joints_start,joints_end)\n")

    '''print "============ 15"
    with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
      text_file.write("\n# Brick 15\n")
    tutorial.pick(-0.34, -0.5)
    tutorial.place(0.383022, 0.321394, low)
    print "============ 16"
    with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
      text_file.write("\n# Brick 16\n")
    tutorial.pick(0, -0.59)
    tutorial.place(0.383022, 0.321394, high)'''


    print "knock"
    tutorial.go_knock_pose()
    with open("/home/user/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts/start_end.txt", 'a+') as text_file:
      text_file.write("\n# Knocking \n")

    # (plan, fraction) = tutorial.move_group.compute_cartesian_path(
    #                                    waypoints,   # waypoints to follow
    #                                    0.01,        # eef_step
    #                                    0.0)         # jump_threshold
    #
    # tutorial.execute_plan(plan)

    text_file.close()
    print "Finished"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()



