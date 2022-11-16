% =========== UAV Example ===========
% state x = [position x; position y; velocity x; velocity y]
% controls u = [acceleration x; acceleration y]
% references = [waypoint x; waypoint y]
% PID control law: ux = kp(rx - px) + kv(-vx) | same for x and y

addpath('../uav_example/utils');
addpath('..');

%% Parameters
T = 5;
tau = 0.1;   % period of references
t0 = 0;
x0 = [0; 0; 0; 0]; % initial sampled state

%% Dynamics 
[sysc,sysd] = uav_dynamics(tau);

%% Trajectory to be followed
yTildex = @(t) chicane_x(t);
yTildey = @(t) chicane_y(t);

%% Get optimal references (waypoints) in both x and y directions
t = 0:tau:T;
n = length(t);
x = nan(size(sysd.a,1),n);
r = nan(size(sysd.b,2),n);

x(:,1) = x0;
for i=1:n-1
    k = t(i);
    r(:,i) = [yTildex(k);yTildey(k)];
    x(:,i+1) = sysd.a * x(:,i) + sysd.b * r(:,i);
end
    
    
%% Plot desired trajectory, optimal references and resulting trajectory
close all;
figure(1); hold on; grid on;
plot(r(1,:),r(2,:),...
    'DisplayName','Reference Trajectory',...
    'LineWidth',2);
plot(x(1,:),x(2,:),...
   'DisplayName','Actual Trajectory',...
   'LineWidth',2);
scatter(r(1,:),r(2,:),'filled','DisplayName','Reference waypoints');
xlabel("X Position (m)"); ylabel("Y Position (m)");
legend('Location','southeast');

figure(2); clf; hold on;
subplot(211); hold on;
plot(t,r(1,:));
plot(t,x(1,:));
hold off;

subplot(212); hold on;
plot(t,r(2,:));
plot(t,x(2,:));
hold off;

