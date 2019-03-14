# Robot Dominoes
Dyson School of Design Engineering - Robitics I group project
Team members: Gordon Cheung, Hayden Cotgrove, Thomas Gonda, Ellie Peatman, Neel Le Penru, Haron Shams, Ruksana Shaukat-Jali.

A project using the Franka Panda robot arm to pick up bricks and arrange them in a line such that they can be knocked over like dominoes. Visualisation and simulations is acheived using Gazebo and any inverse kinematics by using MoveIt
The robot specifications and general information can be found here: https://www.franka.de/panda/
It's control interface can be found here: https://frankaemika.github.io/docs/
Lastly, moveit has a full tutorial on motion planning and controlling the Panda robot here: https://ros-planning.github.io/moveit_tutorials/

To use the scripts (they can be used in any catkin_ws as long as they are sourced):
  Download /launch/final.launch and put in the folder ~/catkin_ws/src/franka_gazebo/launch
  Download /launch/final2.world and put in the folder (may need to be created) ~/catkin_ws/src/franka_gazebo/worlds
  Download /joint_publishers/unified_publisher.py and put in the folder ~/catkin_ws/src/franka_gazebo/scripts
  Download /Inverse_Kinematics/python_interface.py and put in the folder ~/catkin_ws/src/franka_gazebo/scripts
  Download /franka_gazebo/robots/hand.xacro and put in the folder ~/catkin_ws/src/franka_gazebo/robots

More detailed instruction and script documentation can be found here: https://drive.google.com/file/d/1w-LwVvaylT83-L9LOh5srRP6lYMZMGxe/view?usp=sharing
The full development process of this project can be found here: https://drive.google.com/file/d/1GVXhP2Zk7qUlBTXRKviGo2JeZ8iw8Dau/view?usp=sharing
