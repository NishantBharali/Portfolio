close all
clear
clc

% create figure
figure
axis([-4, 4, -4, 4])
grid on
hold on

% save as a video file
v = VideoWriter('problem2_1.mp4', 'MPEG-4');
v.FrameRate = 100;
open(v);

% pick your system parameters
m1 = 1;
m2 = 1;
m3 = 1;
I3 = 0.1;
L = 1;
g = 9.81;
deltaT = 0.01;

% initial conditions
theta = [0; 0; 0];
thetadot = [0; 0; 0];
thetadotdot = [0; 0; 0];
time = 0;

% forward kinematics to end-effector
S1 = [0;0;0;1;0;0];
S2 = [0;0;0;0;1;0];
S3 = [0;0;1;0;-L;0];
M3 = makeT(eye(3), 2*L, 0, 0);

for idx = 1:1000

    % get desired position
    theta_d = [-2; 2; pi/4];
    thetadot_d = [0; 0; 0];
    T_d = fk(M3, [S1 S2 S3], theta_d);
    
    % plot the robot
    p0 = [0; 0];
    p1 = % position of end of link 1
    p2 = % position of end of link 2
    p3 = % position of end of link 3
    P = [p0, p1, p2, p3];
    cla;
    plot(P(1,:), P(2,:), 'o-', 'color',[1, 0.5, 0],'linewidth',4)
    % plot the desired position
    plot(T_d(1,4), T_d(2,4), 'ok', 'MarkerFaceColor','k')
    drawnow
    frame = getframe(gcf);
    writeVideo(v,frame);
    
    % mass matrix
    M = [m1 + m2 + m3, 0, -L*m3*sin(theta(3));
            0, m2 + m3, L*m3*cos(theta(3));
            -L*m3*sin(theta(3)), L*m3*cos(theta(3)), m3*L^2 + I3];
    % Coriolis matrix
    C = [0, 0, -L*thetadot(3)*m3*cos(theta(3));
        0, 0, -L*thetadot(3)*m3*sin(theta(3));
        0, 0,                 0];
    % gravity vector
    G = [0; g*m2 + g*m3; L*g*m3*cos(theta(3))];
    
    % choose your controller tau
    tau = 
    
    % integrate to update velocity and position
    thetadotdot = M \ (tau - C*thetadot - G);
    thetadot = thetadot + deltaT * thetadotdot;
    theta = theta + deltaT * thetadot;
    time = time + deltaT;
    
end

close(v);
close all