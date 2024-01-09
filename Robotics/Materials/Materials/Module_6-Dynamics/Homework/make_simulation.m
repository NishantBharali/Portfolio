close all
clear
clc

% create figure
figure
axis([-2, 2, -2, 2])
grid on
hold on

% save as a video file
v = VideoWriter('test.mp4', 'MPEG-4');
v.FrameRate = 100;
open(v);

% pick your system parameters
m1 = 1;
m2 = 1;
I1 = 0.1;
I2 = 0.1;
L1 = 1;
L2 = 1;
g = 9.81;
tau = [0;0];

% initial conditions
theta = [0; 0]; % joint position
thetadot = [0; 0];  % joint velocity
thetadotdot = [0; 0];   % joint acceleration

for idx = 1:1000
    
    % plot the robot
    % 1. get the position of each link
    p0 = [0; 0];
    p1 = % position of link 1 (location of joint 2)
    p2 = % position of link 2 (the end-effector)
    P = [p0, p1, p2];
    % 2. draw the robot and save the frame
    cla;
    plot(P(1,:), P(2,:), 'o-', 'color',[1, 0.5, 0],'linewidth',4)
    drawnow
    frame = getframe(gcf);
    writeVideo(v,frame);
    
    % integrate to update velocity and position
    deltaT = 0.01;
    thetadot = thetadot + deltaT * thetadotdot;
    theta = theta + deltaT * thetadot;
    
    % your code here
    thetadotdot = 
    
end

close(v);
close all
