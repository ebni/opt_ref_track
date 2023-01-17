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

dt  = 0.001;
t = 0:dt:T;  % fine-grain time

%% Dynamics 
[sysc,sysd] = uav_dynamics(tau);

%% Trajectory to be followed
yTildex = @(t) t;
yTildey = @(t) sin(t);

%% Get optimal references (waypoints) in both x and y directions
[r_opt, y_opt, Q_final, lin_final] = opt_refs_uav(sysc, x0, t0, N, tau, yTildex, yTildey, beta);

%% Sampled trajectory
n = length(t);
r_sampled = nan(size(sysc.b,2),n);
r_optim   = nan(size(sysc.b,2),n);

% Generate the piece-wise constant input
next = tau;
j = 1;
for i=1:n
    if t(i) >= next
        next = next + tau;
        j    = j+1;
    end
    r_sampled(:,i) = [yTildex(next);yTildey(next)];
    r_optim(:,i)   = [r_opt.x(j);   r_opt.y(j)];
end

x_sampled = lsim(sysc,r_sampled,t,x0);
x_optim   = lsim(sysc,r_optim,t,x0);


%% Plot desired trajectory, optimal references and resulting trajectory
%close all;
figure(1); clf; 
%subplot(121);
hold on; grid on;
title('Sampled reference');
plot(yTildex(t),yTildey(t),...
    'DisplayName','Reference Trajectory',...
    'LineWidth',3,'Color','b');
plot(x_sampled(:,1),x_sampled(:,2),...
   'DisplayName','Actual Trajectory',...
   'LineWidth',3,'Color','k');
scatter(r_sampled(1,:),r_sampled(2,:),'filled',...
    'DisplayName','Waypoints');
xlabel("X Position (m)"); ylabel("Y Position (m)");
legend('Location','northeast');
xlim([0,T+tau]);
ylim([-1.5,1.5]);
% adapting to size of figure
my_fig = gcf;
my_fig.PaperUnits='centimeters';
my_fig.PaperSize = [14 11];
print('waypoint_sampled.pdf','-dpdf')

figure(2); clf;
%subplot(122);
hold on; grid on;
title('Optimized waypoints');
plot(yTildex(t),yTildey(t),...
    'DisplayName','Reference Trajectory',...
    'LineWidth',3,'Color','b');
plot(x_optim(:,1),x_optim(:,2),...
   'DisplayName','Actual Trajectory',...
   'LineWidth',3,'Color','k');
scatter(r_optim(1,:),r_optim(2,:),'filled',...
    'DisplayName','Waypoints');
xlabel("X Position (m)"); ylabel("Y Position (m)");
legend('Location','northeast');
xlim([0,T+tau]);
ylim([-1.5,1.5]);
% adapting to size of figure
my_fig = gcf;
my_fig.PaperUnits='centimeters';
my_fig.PaperSize = [14 11];
print('waypoint_optimal.pdf','-dpdf')
