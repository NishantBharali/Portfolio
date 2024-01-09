close all
clear
clc

% create figure
figure
axis([-2, 2, -2, 2])
grid on
hold on

% pick your system parameters
m1 = 1;
m2 = 1;
I1 = 0.1;
I2 = 0.1;
L1 = 1;
L2 = 1;
g = 9.81;
deltaT = 0.01;

% initial conditions
theta = [0; 0];
thetadot = [0; 0];
thetadotdot = [0; 0];

% forward kinematics to end-effector
S1 = [0 0 1 0 0 0]';
S2 = [0 0 1 0 -L1 0]';
M2 = [eye(3), [L1+L2; 0; 0]; 0 0 0 1];

for time = linspace(0,10,10/deltaT)

    % get desired position
    theta_d = [pi/4; pi/4];
    thetadot_d = [0; 0];
    thetadotdot_d = [0; 0];
    T_d = fk(M2, [S1 S2], theta_d);
    
    % plot the robot
    if abs(mod(time, 0.1)) < .01
        p0 = [0; 0];
        p1 = L1 * [cos(theta(1)); sin(theta(1))];
        p2 = p1 + L2 * [cos(theta(1)+theta(2)); sin(theta(1)+theta(2))];
        P = [p0, p1, p2];
        cla;
        plot(P(1,:), P(2,:), 'o-', 'color',[1, 0.5, 0],'linewidth',4)
        plot(T_d(1,4), T_d(2,4), 'ok', 'MarkerFaceColor','k')
        drawnow
    end

    % mass matrix
    M = [m1*L1^2 + m2*(L1^2 + L2^2 + 2*L1*L2*cos(theta(2))) + I1 + I2,... 
        m2*(L2^2 + L1*L2*cos(theta(2))) + I2;...
        m2*(L2^2 + L1*L2*cos(theta(2))) + I2,...
        m2*L2^2 + I2];
    % Coriolis matrix
    C = [-m2*L1*L2*sin(theta(2))*thetadot(2),...
        -m2*L1*L2*sin(theta(2))*(thetadot(1)+ thetadot(2));...
        m2*L1*L2*sin(theta(2))*thetadot(1), 0];
    % gravity vector
    G = [g*(m1+m2)*L1*cos(theta(1))+g*m2*L2*cos(theta(1)+theta(2));...
        g*m2*L2*cos(theta(1)+theta(2))];
    
    % choose your controller tau
    Kd = eye(2)*1;
    Kp = eye(2)*1;
    tau = Kd*(thetadot_d - thetadot) + Kp*(theta_d - theta) + G;
    
    % integrate to update velocity and position
    thetadotdot = M \ (tau - C*thetadot - G);
    thetadot = thetadot + deltaT * thetadotdot;
    theta = theta + deltaT * thetadot;
    
end