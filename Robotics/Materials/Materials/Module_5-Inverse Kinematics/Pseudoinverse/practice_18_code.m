clear
clc

theta1 = pi/8;
theta2 = theta1;
theta3 = theta1;
theta4 = theta1;
L = 1;
theta = [theta1; theta2; theta3; theta4];

omega = [0;0;1];
q1 = [0;0;0];
q2 = [L;0;0];
q3 = [2*L;0;0];
q4 = [3*L;0;0];

S1 = [omega; -cross(omega, q1)];
S2 = [omega; -cross(omega, q2)];
S3 = [omega; -cross(omega, q3)];
S4 = [omega; -cross(omega, q4)];
S = [S1, S2, S3, S4];
M = [eye(3), [4*L;0;0]; 0 0 0 1];

Js = JacobianSpace(S, theta);
T = (fk(M, S, theta));
R = T(1:3, 1:3);
Jb = Adjoint(inv(T))*Js;
J = [R, zeros(3); zeros(3), R] * Jb;

% get forward kinematics at theta
T = (fk(M, S, theta))
% move the robot in the null space
delta_theta  = (eye(4) - pinv(J)*J)*[1;0;1;0]
theta_new = 0.1 * delta_theta + theta
% take the forward kinematics again
T1 = fk(M,S,theta_new)