%DE3 Robotics Dominoes Group, 13th March 2019.
%Dyson School of Design Engineering, Imperial College London

%NB. A newer version of this code that includes the orientation as well as
%position is on the team's GitHub. However, this version the one is used by
%our the Python script mattest.py which calls successfully it.

%IMPORTANT: to be able to call this MatLab function from Python (using the
%mattest.py script) it is necessary to have this function open and to run 
%'matlab.engine.shareEngine in the command window below.


%The function takes as input desired starting joint angles and a target
%position in task space (Cartesian coordinates).
function set_joints = IKfunction(joint1, joint2, joint3, joint4, joint5, joint6, joint7, xpos,ypos,zpos)

%clear all;
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7

%for speed the Jacobian and homogoneous transformation matrix for the tip
%relative to the base were calculated earlier and are stored separately for
%use in this function. Refer to IKfunctionOLD to see the code to calculate
%each of these.
load('tip.mat', 'tip')
load('Jacobian.mat', 'J')

pos = tip(1:3, 4);

%Set initial joint angles
jointAngles = [joint1 joint2 joint3 joint4 joint5 joint6 joint7];
init_pos = vpa(subs(pos, [theta1, theta2, theta3, theta4, theta5, theta6, theta7], jointAngles));

point1 = init_pos;
point2 = [xpos; ypos; zpos]; %input desired position in Cartesian coordinates

%find distance along straight-line path from point1 to point2
diff = point2-point1;
dist = norm(diff);
dt = 0.01;
t = (0:dt:dist)/dist;
%set trajectory
targetpos = [point1(1)+diff(1)*t; point1(2)+diff(2)*t; point1(3)+diff(3)*t];
targetvel = ([targetpos(:, 2:end) targetpos(:, end)] - targetpos)/dt;

%Inverse kinematics using pseudo-inverse Jacobian for redunancy reduction
%to calculate required joint velocity and integrating this (i.e.
%multiplying by dt) for each loop to get the current joint angles (which
%are needed to recompute the Jacobian).
for i = 1:size(targetvel, 2)
    currentJ = vpa(subs(J, [theta1, theta2, theta3, theta4, theta5, theta6, theta7], jointAngles));
    pseudo_inverse = pinv(currentJ);
    jointVel = pseudo_inverse*targetvel(:, i);
    jointVel = transpose(jointVel);
    jointAngles = jointAngles+jointVel*dt;
end
set_joints = (double(jointAngles)); %converts the array of joint angles for the ...
                                    %...final target position (which is what we want)...
                                    %...to a double so that it can correctly be...
                                    %...interepreted in Python as an array...
                                    %...rather than a 'Matlab Obect'

%%------------- Additional code for saving joint angles and plotting -------------
%save('q3_joint_angle_profile.mat', 'jointAngles')

%plot trajectory in x-y plane
% figure
% hold on;
% plot3(traj_pos(1, 1:end-1), traj_pos(2 , 1:end-1), traj_pos(3 , 1:end-1));
% legend({'Position'})
% xlabel('X-position (m)')
% ylabel('Y-position (m)')
% zlabel('Z-position (m)')
% axis equal
