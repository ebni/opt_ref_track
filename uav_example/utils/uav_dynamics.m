%% This file describes the dynamics used in the submitted ACC 2022 paper under review
function [sysc,sysd] = uav_dynamics(dt)
% Our model is a simple double-integrator with PD controller, to
% approximate the inertial characeteristics of the UAV.

% The model is the following:
% x_ddot = u_x
% y_ddot = u_y

% Here, u is commanded acceleration of the x/y coordinate. It is common
% for UAVs to command accelerations in the xy plane by using a PD
% controller, so the controls are:

% u_x = kp*( x_w - x ) - kv*x_dot
% u_y = kp*( y_w - y ) - kv*y_dot

% Here, (x_w,y_w) is the xy position of the waypoint to be tracked by the
% UAV.

% Assuming kp and kv are already tuned well, our planner chooses waypoints.
% In this sense, we can consider (x_w, y_w) to be the "controls" of the
% system.

% ddt(   x  ) = x_dot
% ddt(   y  ) = y_dot
% ddt( x_dot ) = -kp*x -kv*xdot + kp*x_w
% ddt( y_dot ) = -kp*y -kv*ydot + kp*y_w

% Constants
if nargin < 1
dt = 0.1;
end
kp = 4.5; % PD proportional gain
kv = 2.5; % PD derivative gain

% Continuous time dynamics
% State = (x, y, xdot, ydot)
% "Control" = (x_w, y_w)
A = [0   0   1   0   ;...
     0   0   0   1   ;...
     -kp 0  -kv  0   ;...
     0   -kp 0   -kv];
B = [zeros(2);...
    kp*eye(2)];
C = eye(4); % Full-State Observer Model
D = zeros(4,2);

sysc = ss(A,B,C,D);  % Continuous Time Model
sysd = c2d(sysc,dt); % Discrete Time Model
end


