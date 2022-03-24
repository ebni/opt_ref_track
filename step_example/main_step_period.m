addpath("..");

%% Double integrator controller by PD controller
% eigenvalues of closed loop dynamics: DO NOT set them equal, otherwise
% different math applies
lam1=-1;
lam2=-2;

%% Sampling
t0 = 0;
x0 = [0; 0]; % initial sampled state

%% Trajectory to be followed
yTilde = @(t) t-t+1;  % step response
yTilde = @(t) (.5*(1-cos(2*pi*t))).^8;  % follow cos

close all;
figure;

%% Target trajectory
tau = 0.1;   % period of references
N = 10;      % number of intervals
subplot(2,1,1);
timespan = linspace(t0,t0+N*tau,100);
plot(timespan,yTilde(timespan),'b','LineWidth',4);
hold on
ylabel('output y(t)');

%% Get optimal references and plot
tau = 0.2;   % period of references
N = 5;      % number of intervals
[r_opt, Q_final, lin_final] = opt_refs_PDcontrol_DBLint(lam1,lam2,x0,t0,N,tau,yTilde, 0);
[output_x, output_y] = output_PDcontrol_DBLint(lam1,lam2,x0,t0,N,tau,r_opt);
subplot(2,1,1);
plot(output_x, output_y,'LineWidth',2,'Color',.2*[1 1 1]);
subplot(2,1,2);
hold on;
ylabel('reference r(t)');
tK = linspace(t0,t0+N*tau,N+1);
stairs(tK,[r_opt' r_opt(end)],'LineWidth',2,'Color',.2*[1 1 1]);   % refs

%% Get optimal references and plot
tau = 0.1;   % period of references
N = 10;      % number of intervals
[r_opt, Q_final, lin_final] = opt_refs_PDcontrol_DBLint(lam1,lam2,x0,t0,N,tau,yTilde, 0);
[output_x, output_y] = output_PDcontrol_DBLint(lam1,lam2,x0,t0,N,tau,r_opt);
subplot(2,1,1);
plot(output_x, output_y,'LineWidth',2,'Color',.4*[1 1 1]);
subplot(2,1,2);
hold on;
tK = linspace(t0,t0+N*tau,N+1);
stairs(tK,[r_opt' r_opt(end)],'LineWidth',2,'Color',.4*[1 1 1]);   % refs

%% Get optimal references and plot
tau = 0.05;   % period of references
N = 20;      % number of intervals
[r_opt, Q_final, lin_final] = opt_refs_PDcontrol_DBLint(lam1,lam2,x0,t0,N,tau,yTilde, 0);
[output_x, output_y] = output_PDcontrol_DBLint(lam1,lam2,x0,t0,N,tau,r_opt);
subplot(2,1,1);
plot(output_x, output_y,'LineWidth',2,'Color',.6*[1 1 1]);
subplot(2,1,2);
hold on;
tK = linspace(t0,t0+N*tau,N+1);
stairs(tK,[r_opt' r_opt(end)],'LineWidth',2,'Color',.6*[1 1 1]);   % refs


subplot(2,1,2);
legend('\tau=0.2, N=5','\tau=0.1, N=10','\tau =0.05, N=20','Location','southeast')
