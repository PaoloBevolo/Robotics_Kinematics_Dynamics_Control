clc;
clear;
close all;

%% Initialize the workspace

radius=0.20;
n=200;
INI = transl(-0.25, 0.25,-0.5); %center of the part

%% Plot a sketch of the environment of the robot

mdl_puma560
p560.plot(qz);
hold on;
circle1 = circle([-0.25 0.25 -0.5], radius);
circle2 = circle([-0.25 0.25 -0.5], 0.5*radius);
plot3(circle1(1,:), circle1(2,:), circle1(3,:),'g','LineWidth',1);
patch(circle2(1,:), circle2(2,:), circle2(3,:),'r');

%% Finding the positions to locate the laser. Pay atention to the structure of Laser_Pose (4x4xn)

for i=1:n
Laser_Pose(:,:,i)= INI*trotx(-pi/2)*troty(2*pi*i/n)*transl(0, 0, -radius)
end

%% Using the Inverse kinematics to find the joints. Pay atention to the structure of Q (6x1xn)

Q= p560.ikine6s(Laser_Pose, 'run')

%% Ploting the result

p560.plot(Q)


