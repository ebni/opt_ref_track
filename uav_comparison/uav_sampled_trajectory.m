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
dt  = tau;
t0 = 0;
x0 = [0; 0; 1; 1]; % initial sampled state

%% Dynamics 
[sysc,sysd] = uav_dynamics(dt);

%% Trajectory to be followed
%yTildex = @(t) chicane_x(t);
%yTildey = @(t) chicane_y(t);

yTildex = @(t) t;
yTildey = @(t) sin(t);

%% Get optimal references (waypoints) in both x and y directions
t = 0:dt:T;
n = length(t);
x = nan(size(sysd.a,1),n);
r = nan(size(sysd.b,2),n);

x(:,1) = x0;
for i=1:n-1
    k = t(i+1);
    r(:,i) = [yTildex(k);yTildey(k)];
    x(:,i+1) = sysd.a * x(:,i) + sysd.b * r(:,i);
    
% figure(1); clf;hold on; grid on;
% plot(r(1,:),r(2,:),...
%     'DisplayName','Reference Trajectory',...
%     'LineWidth',2);
% plot(x(1,:),x(2,:),...
%    'DisplayName','Actual Trajectory',...
%    'LineWidth',2);
% scatter(r(1,:),r(2,:),'filled','DisplayName','Reference waypoints');
% scatter(x(1,:),x(2,:),'filled','DisplayName','Reference waypoints');
% xlabel("X Position (m)"); ylabel("Y Position (m)");
% legend('Location','southeast');
end

%pause
    
%% Plot desired trajectory, optimal references and resulting trajectory
%close all;
figure(1); clf;hold on; grid on;
plot(yTildex(0:.01:T),yTildey(0:.01:T),...
    'DisplayName','Reference Trajectory',...
    'LineWidth',2);
plot(x(1,:),x(2,:),...
   'DisplayName','Actual Trajectory',...
   'LineWidth',2);
scatter(r(1,:),r(2,:),'filled','DisplayName','Reference waypoints');
%scatter(x(1,:),x(2,:),'filled','DisplayName','Actual positions');
xlabel("X Position (m)"); ylabel("Y Position (m)");
legend('Location','southeast');

figure(2); clf; hold on;
subplot(221); hold on;
plot(t,r(1,:));
plot(t,x(1,:));
ylabel('$x$','interpreter','latex')
legend('Reference','Actual');
% scatter(t,r(1,:));
% scatter(t,x(1,:));
hold off;

subplot(222); hold on;
plot(t,r(2,:));
plot(t,x(2,:));
hold off;
ylabel('$y$','interpreter','latex')
legend('Reference','Actual');
subplot(223); 
plot(t,x(3,:));
ylabel('$\dot x$','interpreter','latex')

subplot(224); 
plot(t,x(4,:));
ylabel('$\dot y$','interpreter','latex')

