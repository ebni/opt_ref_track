% =========== Receding Horizon UAV Example ===========
% Receding horizon control, resolves optimization problem at each time step
% and applies first optimal control input

% state x = [position x; position y; velocity x; velocity y]
% controls u = [acceleration x; acceleration y]
% references = [waypoint x; waypoint y]
% PID control law: ux = kp(rx - px) + kv(-vx) | same for x and y

addpath('./utils');
addpath('..');

%% Flags
filename = "recedingHorizon";
makeGifFlag = 0;
makeVideoFlag = 1;
% scenario = 1; % Beta = 4 | Waypoint Freq = 10 Hz
% scenario = 2; % Beta = 4 | Waypoint Freq = 4 Hz
scenario = 3; % Beta = 4 | Waypoint Freq = 2 Hz

%% Parameters
T = 2; % Prediction Horizon time (s)
if scenario==1
    tau = 0.1;  % 10 Hz
elseif scenario==2
    tau = 0.25; % 4 Hz
elseif scenario==3
    tau = 0.5;  % 2 Hz
end
predN = T/tau;      % Prediction horizon length
simT = 8.0; % total simulation time
simdt = 0.02; % Simulation sampling time
t0 = 0; % initial time
x0 = [0; 0; 0; 0]; % initial sampled state

%% Dynamics 
[sysc,sysd] = uav_dynamics();

%% Trajectory to be followed
yTildex = @(t) 4*sin(t);
yTildey = @(t) 2*sin(2*t);

%% Initialize Variables and Plot
cmap = gray(4);
linesCmap = lines(4);
close all;
figure(1); hold on; grid on; xlim([-4.2,4.2]); ylim([-2.3,2.3]); axis equal; title(sprintf("$\\tau: %0.1f$ s $|$ Time: 0.0 s",tau),"Interpreter","latex"); 
% plt_yTilde = plot(yTildex(0:0.1:10),yTildey(0:0.1:10),'LineWidth',2,'DisplayName','Reference Trajectory','Color',[51 204 255]/255); % For figure
plt_yTilde = plot(yTildex(0:0.1:10),yTildey(0:0.1:10),'LineWidth',2,'DisplayName','Reference Trajectory','Color',[51 204 255]/255); % For video
plt_xs = plot([0.0],[0.0],'-','LineWidth',2,'DisplayName','Actual Trajectory','Color',cmap(scenario,:));
% plt_future_xs = plot([0.0],[0.0],'--','LineWidth',2.5,'DisplayName','Future Trajectory','Color',cmap(scenario,:)); % For figure
plt_future_xs = plot([0.0],[0.0],'--','LineWidth',2.5,'DisplayName','Future Trajectory','Color',linesCmap(2,:)); % For video
plt_rob = scatter(0.0,0.0,'ko','filled','SizeData',50,'DisplayName','Current Position','CData',cmap(scenario,:));
legend('Location','eastoutside');
if makeGifFlag
    
end
if makeVideoFlag
    writerObj = VideoWriter(filename+sprintf("%0.0f",1/tau)+"Hz","MPEG-4");
    writerObj.FrameRate = 1/simdt;
    open(writerObj);
end

%% Simulate Receding Horizon Control
steps = int16((simT-t0)/simdt);
stepsPerWypt = int16(tau/simdt);
xs = zeros(size(x0,1),steps); rs = zeros(2,steps);
times = 0:simdt:simT;
xs(:,1) = x0;
r = x0(1:2,:); % Applied waypoint
es = zeros(1,steps); % Error between y(t) and \tilde{y}(t)
for step = 1:steps
    % Get current time and state
    t = times(step);
    x = xs(:,step);
    if mod(step-1,stepsPerWypt)==0 % If time for a new waypoint
        % Find optimal references + resulting trajectory
        [r_opt, y_opt, Q_final, lin_final] = opt_refs_uav(sysd, x, t, predN, tau, yTildex, yTildey);
        r = [r_opt.x(1);r_opt.y(1)];
    end
    % Save reference and next state information
    [t_int,x_int] = ode45(@(t,x) EOM(t,sysc,x,r),[0 simdt],x);
    xs(:,step+1) = x_int(end,:)';
    rs(:,step) = r;
    % Get error between target trajectory and actual trajectory
    es(step) = norm(xs(1:2,step)-[yTildex(t);yTildey(t)]);
    % Update plot
    plt_rob.XData = xs(1,step); plt_rob.YData = xs(2,step);
    plt_xs.XData = xs(1,1:step); plt_xs.YData = xs(2,1:step);
%     plt_yTilde.XData = yTildex(y_opt.x.ts); plt_yTilde.YData = yTildey(y_opt.x.ts);
    plt_future_xs.XData = y_opt.x.ys; plt_future_xs.YData = y_opt.y.ys;
    title(sprintf("$\\tau: %0.1f$ s $|$  Time: %0.2f",tau,t));
    drawnow;
    if makeGifFlag
        frame = getframe(gcf); 
        im = frame2im(frame); 
        [imind,cm] = rgb2ind(im,256);
        if step==1
            imwrite(imind,cm,filename+".gif",'gif', 'Loopcount',inf); 
        else
            imwrite(imind,cm,filename+".gif",'gif','WriteMode','append','DelayTime',simdt);
        end
    end
    if makeVideoFlag
        frame = getframe(gcf);
        writeVideo(writerObj,frame);
    end
end
if makeVideoFlag
    close(writerObj);
end

%% Save data for plotting
save(sprintf('./data/scenario%d',scenario),'es','times');

%% Plot trajectory with Receding Horizon Controller
% close all;
figure(2); hold on; grid on; axis equal; title("Full Trajectory");
plot(xs(1,:),xs(2,:),'LineWidth',2,'DisplayName','Actual Trajectory');
plot(yTildex(times(1:end-1)),yTildey(times(1:end-1)),'LineWidth',2,'DisplayName','Reference Trajectory');

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