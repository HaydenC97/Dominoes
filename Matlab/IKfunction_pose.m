function set_joints = IKfunction_pose(joint1, joint2, joint3, joint4, joint5, joint6, joint7, xpos,ypos,zpos, xtheta, ytheta, ztheta)

%clear all;
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7

%load the presaved symbolic equations for calculating the pose of the tip and the jacobin
load('tip.mat', 'tip')
load('Jacobian_pose.mat', 'J')


%get the position and angle equations to create a pose vector
pos = tip([1:3], 4);
angles = [];
for i = 1:3
    angles = [angles; tip(i, i)]; %getting three diagonals of rotation matrix
end
angles = acos(angles); % diagonals are equal to cos of the actual angle
pose = [pos;angles];

%Set initial joint angles
jointAngles = [joint1 joint2 joint3 joint4 joint5 joint6 joint7];
init_pose = vpa(subs(pose, [theta1, theta2, theta3, theta4, theta5, theta6, theta7], jointAngles));

%calculates trajectory for straight line path between point1 pose and point2 pose
point1 = init_pose;
point2 = [0.4; 0.4; 0.4; 0; 0; 0];
diff = point2-point1;
dist = norm(diff);
dt = 0.01;
t = (0:dt:dist)/dist;
targetpose = [point1(1)+diff(1)*t; point1(2)+diff(2)*t; point1(3)+diff(3)*t; point1(4)+diff(4)*t; point1(5)+diff(5)*t; point1(6)+diff(6)*t]; %fast 2d circle trajectory
targetvel = ([targetpose(:, 2:end) targetpose(:, end)] - targetpose)/dt;

%Do forward and inverse kinematics to move along the trajectory
for i = 1:size(targetvel, 2)
    currentJ = vpa(subs(J, [theta1, theta2, theta3, theta4, theta5, theta6, theta7], jointAngles(end, :)));
    pseudo_inverse = pinv(currentJ);
    jointVel = pseudo_inverse*targetvel(:, i);
    jointVel = transpose(jointVel);
    jointAngles = jointAngles+jointVel*dt;
end
set_joints = (double(jointAngles)); %return joint angles of final pose

%plot trajectory in x-y-z
% figure
% hold on;
% plot3(traj_pos(1, 1:end-1), traj_pos(2 , 1:end-1), traj_pos(3 , 1:end-1));
% legend({'Position'})
% xlabel('X-position (m)')
% ylabel('Y-position (m)')
% zlabel('Z-position (m)')
% axis equal
