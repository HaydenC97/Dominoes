%%DE3 Robotics Dominoes Group, 13th March 2019.
%Dyson School of Design Engineering, Imperial College London
%Adapted from code to DE3 Robotics Tutorial 2

%Setup:
close all
clear all
syms theta1 theta2 theta3 theta4 theta5 theta5 theta6 theta7 L_endef
%% Robot information
% DH table and joint angle ranges from https://frankaemika.github.io/docs/control_parameters.html

% DH table with values specified online substitiuted in.
%     ai alphai di thetai
DH = [0        0       0.333   theta1;... % Joint 1
      0        -pi/2   0       theta2;... % Joint 2
      0        pi/2    0.316   theta3;... % Joint 3
      0.0825   pi/2    0       theta4;... % Joint 4
      -0.0825  -pi/2   0.384   theta5;... % Joint 5
      0        pi/2    0       theta6;... % Joint 6
      0.088    pi/2    0       theta7;... % Joint 7
      0        0       0.117   0];   % Flange


%Joint angle ranges table:
AR = [-2.8973   2.8973;...  %Joint 1
      -1.7628   1.7628;...   %Joint 2
      -2.8973   2.8973;...  %Joint 3
      -3.0718   -0.0698;...   %Joint 4
      -2.8973   2.8973;...   %Joint 5
      -0.0175   3.7525;...   %Joint 6
      -2.8973   2.8973];     %Joint 7

LB = AR(:,1); %lower bound joint angle ranges
UB = AR(:,2); %upper bound joint angle ranges

%% Create the robot using the Robotics Toolbox
%%%%%%%%%%%%%  Robot Kinematics and Dynamics Model  %%%%%%%%%%%%%
robot = robotics.RigidBodyTree('DataFormat','row','MaxNumBodies',8);
% Add link1 with joint1
link1 = robotics.RigidBody('link1'); %create link
joint1 = robotics.Joint('joint1', 'revolute'); %create joint
setFixedTransform(joint1,trvec2tform([0 0 double(DH(1,3))])); %specify location of joint relative...
                                                              %to previous one (or base for Joint 1)...
                                                              %using information from the DH table.
joint1.JointAxis = [0 0 1]; %Joint rotation axis (using orientation of previous frame)
joint1.PositionLimits = [AR(1, 1), AR(1, 2)]; %specify joint angle limtits
link1.Joint = joint1; %specify joint created as that following current link 
addBody(robot, link1, 'base'); %add link and joint to robot
%repeat for each link
% Add link2 with joint2
link2 = robotics.RigidBody('link2');
joint2 = robotics.Joint('joint2','revolute');
setFixedTransform(joint2, trvec2tform([0,0,double(DH(2,3))]));
joint2.JointAxis = [0 1 0];
joint2.PositionLimits = [AR(2, 1), AR(2, 2)];
link2.Joint = joint2;
addBody(robot, link2, 'link1');
% Add link3 with joint3
link3 = robotics.RigidBody('link3');
joint3 = robotics.Joint('joint3','revolute');
setFixedTransform(joint3, trvec2tform([0,-double(DH(3,3)),0]));
joint3.JointAxis = [0 -1 0];
joint3.PositionLimits = [AR(3, 1), AR(3, 2)];
link3.Joint = joint3;
addBody(robot, link3, 'link2');
% Add link4 with joint4
link4 = robotics.RigidBody('link4');
joint4 = robotics.Joint('joint4','revolute');
setFixedTransform(joint4, trvec2tform([double(DH(4,1)),0,0]));
joint4.JointAxis = [0 -1 0];
joint4.PositionLimits = [AR(4, 1), AR(4, 2)];
joint4.HomePosition = -0.1;
link4.Joint = joint4;
addBody(robot, link4, 'link3');
% Add link5 with joint5
link5 = robotics.RigidBody('link5');
joint5 = robotics.Joint('joint5','revolute');
setFixedTransform(joint5, trvec2tform([double(DH(5,1)),double(DH(5,3)),0]));
joint5.JointAxis = [0 1 0];
joint5.PositionLimits = [AR(5, 1), AR(5, 2)];
link5.Joint = joint5;
addBody(robot, link5, 'link4');
% Add link6 with joint6
link6 = robotics.RigidBody('link6');
joint6 = robotics.Joint('joint6','revolute');
setFixedTransform(joint6, trvec2tform([double(DH(6,1)),0,double(DH(6,3))]));
joint6.JointAxis = [0 -1 0];
joint6.PositionLimits = [AR(6, 1), AR(6, 2)];
link6.Joint = joint6;
addBody(robot, link6, 'link5');
% Add link7 with joint7
link7 = robotics.RigidBody('link7');
joint7 = robotics.Joint('joint7','revolute');
setFixedTransform(joint7, trvec2tform([double(DH(7,1)),0,double(DH(7,3))]));
joint7.JointAxis = [0 -1 0];
joint7.PositionLimits = [AR(7, 1), AR(7, 2)];
link7.Joint = joint7;
addBody(robot, link7, 'link6');
% Add end effector
L_endef = 0.117;
endEffector = robotics.RigidBody('end effector');
lastjoint = robotics.Joint('fix1','fixed');
setFixedTransform(lastjoint, trvec2tform([0, 0, double(DH(8,3))]));
endEffector.Joint = lastjoint;
addBody(robot, endEffector, 'link7');

%Uncomment the below to show a model of the robot
% show the robot
% show(robot);
% view(2)
% ax = gca;
% ax.Projection = 'orthographic';
%return

%% Inverse Kinematics %%%%%%% 

dt = 0.4;
t = (0:dt:4)'; % Time
count = length(t);
% define the reference trajectory

%Specify start and end point in task space (Cartesian coordinates)
corner1 = [0 -0.5 0.6];
corner2 = [0 -0.5 0.17];

%Create a straight line path from start point to end point
line1 = [linspace(corner1(1),corner2(1),count)' linspace(corner1(2),corner2(2),count)' linspace(corner1(3),corner2(3),count)'];

q0 = homeConfiguration(robot); %set initial configuration of robot to that of the default home configuration defined in the robotics toolbox.
ndof = length(q0);
qs = zeros(count, ndof); %create structure to store joint angles.

ik = robotics.InverseKinematics('RigidBodyTree', robot); %create robotics toolbox inverse kinematics object
%weights = [0, 0, 0, 1, 1, 1]; % only consider orientation
weights = [1, 1, 1, 1, 1, 1]; %All 'weights' set to 1 to specify that x,y,z position and pitch, roll and yaw orientation of end effector must all be considered
qInitial = q0; % Use home configuration as the initial guess
%qInitial = [LB + 0.5*(UB-LB)].';
for i = 1:count
    % Solve for the configuration satisfying the desired end effector
    % position
    point = line1(i,:); %Coordinates of current point along line. 
    
    %Specifiy homogoneous transformation from base to tip for current
    %point, with rotation matrix portion also specified for certain
    %orientation (rotation matrix calculated separately)
    test_tform = [1 0 0 point(1);...
                  0 -1 0 point(2);...
                  0 0 -1 point(3);...
                  0 0 0 1];
    
    qSol = ik('end effector', test_tform, weights, qInitial); %perform inverse kinematics...
                                                              %to get joint position for...
                                                              %desired end effector pose...
                                                              %specified in homogenous transformation matrix above
    % Store the configuration
    qs(i,:) = qSol;
    % Start from prior solution
    qInitial = qSol;
end

qs %Print final joint angles of target pose in command window. 
%These were tested by being manually copied into a the Python script
%publishing joint angles to Gazebo.

%% Plot trajectory and animate robot
% figure;
% plot3(line1(:,1), line1(:,2), line1(:,3), 'b.'); % plot the reference trajectory
% 
% AnimateRobot(robot, qs)
% return

%% Function of animating robots in 2D  
% % Take robot model and a sequence of joint positions as input, animate with
% % a frequency of 15 Hz
function AnimateRobot(robot, jointPos)
% figure
% view in 2D
view(3) % 2D
ax = gca;
ax.Projection = 'orthographic';
axis equal
axis([0 1.1 0 1.1 0 1.1])
hold on
% 
framesPerSecond = 15;
r = robotics.Rate(framesPerSecond);
for i = 1:length(jointPos)
    tform = getTransform(robot,jointPos(i,:),'end effector','base');
    plot3(tform(1,4), tform(2,4), tform(3,4), 'ko')
    show(robot,jointPos(i,:),'PreservePlot',false);
    drawnow

    waitfor(r);
end
end