% =========== UAV Example ===========
% state x = [position x; position y; velocity x; velocity y]
% controls u = [acceleration x; acceleration y]
% references = [waypoint x; waypoint y]
% PID control law: ux = kp(rx - px) + kv(-vx) | same for x and y

addpath('../uav_example/utils');
addpath('..');

%% Parameters
T = 2*pi;
tau = 0.1;   % period of references
N = round(T/tau);      % number of intervals
t0 = 0;
x0 = [0; 0; 1; 1]; % initial sampled state
beta = 0;   % Discount factor

%% Dynamics 
[sysc,sysd] = uav_dynamics();

%% Trajectory to be followed
% yTildex = @(t) chicane_x(t);
% yTildey = @(t) chicane_y(t);

yTildex = @(t) t;
yTildey = @(t) sin(t);

%% Get optimal references (waypoints) in both x and y directions
[r_opt, y_opt, Q_final, lin_final] = opt_refs_uav(sysd, x0, t0, N, tau, yTildex, yTildey, beta);

%% Plot desired trajectory, optimal references and resulting trajectory
%close all;
figure(1); clf; hold on; grid on;
plot(yTildex(y_opt.x.ts),yTildey(y_opt.y.ts),...
    'DisplayName','Reference Trajectory',...
    'LineWidth',2);
plot(y_opt.x.ys,y_opt.y.ys,...
   'DisplayName','Actual Trajectory',...
   'LineWidth',2);
scatter(r_opt.x,r_opt.y,'filled');
xlabel("X Position (m)"); ylabel("Y Position (m)");
legend('Location','southeast');

figure(2); clf; hold on;
subplot(211); hold on;
plot(y_opt.x.ts,yTildex(y_opt.x.ts));
plot(y_opt.x.ts,y_opt.x.ys);
ylabel('$x$','interpreter','latex')
legend('Reference','Actual');
% scatter(t,r(1,:));
% scatter(t,x(1,:));
hold off;

subplot(212); hold on;
plot(y_opt.y.ts,yTildey(y_opt.y.ts));
plot(y_opt.y.ts,y_opt.y.ys);
hold off;
ylabel('$y$','interpreter','latex')
legend('Reference','Actual');
% subplot(223); 
% plot(t,x(3,:));
% ylabel('$\dot x$','interpreter','latex')
% 
% subplot(224); 
% plot(t,x(4,:));
% ylabel('$\dot y$','interpreter','latex')
