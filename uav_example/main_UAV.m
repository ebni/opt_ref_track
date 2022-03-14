% =========== UAV Example ===========
% state x = [position x; position y; velocity x; velocity y]
% controls u = [acceleration x; acceleration y]
% references = [waypoint x; waypoint y]
% PID control law: ux = kp(rx - px) + kv(-vx) | same for x and y

addpath('./utils');
addpath('..');

%% Parameters
tau = 0.1;   % period of references
N = 10;      % number of intervals
t0 = 0;
x0 = [0; 1; 0; 0]; % initial sampled state

%% Dynamics 
[sysd,sysc] = uav_dynamics();

%% Trajectory to be followed
yTildex = @(t) sqrt(t);
yTildey = @(t) t-t+1;

%% Get optimal references (waypoints) in both x and y directions
[r_opt, y_opt, Q_final, lin_final] = opt_refs_uav(sysd, x0, t0, N, tau, yTildex, yTildey);

%% Plot desired trajectory, optimal references and resulting trajectory
close all;
figure(1); hold on; grid on;
plot(yTildex(y_opt.x.ts),yTildey(y_opt.y.ts));
plot(y_opt.x.ys,y_opt.y.ys);