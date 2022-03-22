% =========== Receding Horizon UAV Example ===========
% Receding horizon control, resolves optimization problem at each time step
% and applies first optimal control input

% state x = [position x; position y; velocity x; velocity y]
% controls u = [acceleration x; acceleration y]
% references = [waypoint x; waypoint y]
% PID control law: ux = kp(rx - px) + kv(-vx) | same for x and y

addpath('./utils');
addpath('..');

%% Parameters
tau = 0.1;   % Solver sampling time
predN = 20;      % Prediction horizon length
simT = 5.0; % total simulation time
simdt = 0.1; % Simulation sampling time
t0 = 0; % initial time
x0 = [0; 0; 0; 0]; % initial sampled state

%% Dynamics 
[sysc,sysd] = uav_dynamics();

%% Trajectory to be followed
yTildex = @(t) sin(t.^2)+t;
yTildey = @(t) t+0.25;

%% Initialize Variables and Plot
close all;
figure(1); hold on; grid on; axis equal; title("Time: 0.0 s");
plt_xs = plot([0.0],[0.0],'LineWidth',2,'DisplayName','Actual Trajectory');
plt_yTilde = plot([0.0],[0.0],'LineWidth',2,'DisplayName','Reference Trajectory');
plt_future_xs = plot([0.0],[0.0],'LineWidth',2,'DisplayName','Future Trajectory');
legend('Location','eastoutside');

%% Simulate Receding Horizon Control
steps = int16((simT-t0)/simdt);
stepsPerWypt = int16(tau/simdt);
xs = zeros(size(x0,1),steps); rs = zeros(2,steps);
times = 0:simdt:simT;
xs(:,1) = x0;
r = x0(1:2,:); % Applied waypoint
for step = 1:steps
    % Get current time and state
    t = times(step);
    x = xs(:,step);
    if mod(step,stepsPerWypt)==0 % If time for a new waypoint
        % Find optimal references + resulting trajectory
        [r_opt, y_opt, Q_final, lin_final] = opt_refs_uav(sysd, x, t, predN, tau, yTildex, yTildey);
        r = [r_opt.x(1);r_opt.y(1)];
    end
    % Save reference and next state information
    [t_int,x_int] = ode45(@(t,x) EOM(t,sysc,x,r),[0 simdt],x);
    xs(:,step+1) = x_int(end,:)';
    rs(:,step) = r;
    % Update plot
    plt_xs.XData = xs(1,1:step); plt_xs.YData = xs(2,1:step);
    plt_yTilde.XData = yTildex(y_opt.x.ts); plt_yTilde.YData = yTildey(y_opt.x.ts);
    plt_future_xs.XData = y_opt.x.ys; plt_future_xs.YData = y_opt.y.ys;
    drawnow;
end

%% Plot trajectory with Receding Horizon Controller
% close all;
figure(2); hold on; grid on; axis equal; title("Full Trajectory");
plot(xs(1,:),xs(2,:),'LineWidth',2,'DisplayName','Actual Trajectory');
plot(yTildex(times(1:end-1)),yTildey(times(1:end-1)),'LineWidth',2,'DisplayName','Reference Trajectory');
legend('Location','eastoutside');

%% Plot SINGLE MPC solution, not entire trajectory. Used for debugging purposes
% Plot desired trajectory, optimal references and resulting trajectory
% close all;
% figure(1); hold on; grid on;
% plot(yTildex(y_opt.x.ts,0),yTildey(y_opt.y.ts,0),...
%     'DisplayName','Reference Trajectory',...
%     'LineWidth',2);
% plot(y_opt.x.ys,y_opt.y.ys,...
%     'DisplayName','Actual Trajectory',...
%     'LineWidth',2);
% % scatter(r_opt.x,r_opt.y);
% xlabel("X Position (m)"); ylabel("Y Position (m)");
% legend('Location','southeast');

%% Aux Functions
function sdot = EOM(t,sysc,x,r)
    sdot = zeros(4,1);
    sdot = sysc.A*x + sysc.B*r;
end