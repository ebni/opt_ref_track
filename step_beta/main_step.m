addpath("..");

%% Double integrator controller by PD controller
% eigenvalues of closed loop dynamics: DO NOT set them equal, otherwise
% different math applies
lam1=-1;
lam2=-2;

%% Sampling
tau = 0.1;   % period of references
N = 10;      % number of intervals
t0 = 0;
x0 = [0; 0]; % initial sampled state

%% Trajectory to be followed
% yTilde = @(t) sin(t).*exp(-2*t);
%yTilde = @(t) t.*t.*exp(-2*t);
%yTilde = @(t) t.*t;
yTilde = @(t) t-t+1;  % step response

close all;
figure;

%% Target trajectory
subplot(2,1,1);
timespan = linspace(t0,t0+N*tau,100);
plot(timespan,yTilde(timespan),'b','LineWidth',2);
hold on
ylabel('output y(t)');

%% Get optimal references and plot
[r_opt, Q_final, lin_final] = opt_refs_PDcontrol_DBLint(lam1,lam2,x0,t0,N,tau,yTilde, -10);
[output_x, output_y] = output_PDcontrol_DBLint(lam1,lam2,x0,t0,N,tau,r_opt);
subplot(2,1,1);
plot(output_x, output_y,'LineWidth',2,'Color',.2*[1 1 1]);
subplot(2,1,2);
hold on;
ylabel('reference r(t)');
tK = linspace(t0,t0+N*tau,N+1);
stairs(tK,[r_opt' r_opt(end)],'LineWidth',2,'Color',.2*[1 1 1]);   % refs

%% Get optimal references and plot
[r_opt, Q_final, lin_final] = opt_refs_PDcontrol_DBLint(lam1,lam2,x0,t0,N,tau,yTilde, 0);
[output_x, output_y] = output_PDcontrol_DBLint(lam1,lam2,x0,t0,N,tau,r_opt);
subplot(2,1,1);
plot(output_x, output_y,'LineWidth',2,'Color',.4*[1 1 1]);
subplot(2,1,2);
hold on;
tK = linspace(t0,t0+N*tau,N+1);
stairs(tK,[r_opt' r_opt(end)],'LineWidth',2,'Color',.4*[1 1 1]);   % refs

%% Get optimal references and plot
[r_opt, Q_final, lin_final] = opt_refs_PDcontrol_DBLint(lam1,lam2,x0,t0,N,tau,yTilde, 10);
[output_x, output_y] = output_PDcontrol_DBLint(lam1,lam2,x0,t0,N,tau,r_opt);
subplot(2,1,1);
plot(output_x, output_y,'LineWidth',2,'Color',.6*[1 1 1]);
subplot(2,1,2);
hold on;
tK = linspace(t0,t0+N*tau,N+1);
stairs(tK,[r_opt' r_opt(end)],'LineWidth',2,'Color',.6*[1 1 1]);   % refs


subplot(2,1,2);
legend('\beta =-10','\beta =0','\beta =10','Location','southeast')
%subplot(2,1,2);
%legend('\beta =-10','\beta =0','\beta =10','Location','southeast')
