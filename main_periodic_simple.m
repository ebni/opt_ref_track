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
%yTilde = @(t) sin(t).*exp(-2*t);
%yTilde = @(t) t.*t.*exp(-2*t);
%yTilde = @(t) t.*t;
yTilde = @(t) t-t+1;  % step response

%% Get optimal references
[r_opt, Q_final, lin_final] = opt_refs_PDcontrol_DBLint(lam1,lam2,x0,t0,N,tau,yTilde);

%% Plotting
plot_PDcontrol_DBLint(lam1,lam2,x0,t0,N,tau,yTilde,r_opt);

%% Now passing as references the sampled trajectory
%r_sampled = yTilde(linspace(t0+tau,t0+N*tau,N))';
%plot_PDcontrol_DBLint(lam1,lam2,x0,t0,N,tau,yTilde,r_sampled);

%% Getting the most significant eigenvalues of Q_final
[H,D] = eig(Q_final);
LL = diag(D);
gap = 1e-4;  % eigenvalues smaller than gap*max(LL) are considered zero
sel = LL./max(LL) < gap;
H_null = H(:,sel);
H_good = H(:,not(sel));
r_star = H_good*inv(diag(LL(not(sel))))*H_good'*lin_final';
plot_PDcontrol_DBLint(lam1,lam2,x0,t0,N,tau,yTilde,r_star);

%% Now plotting r_star+a vector in the null space
plot_PDcontrol_DBLint(lam1,lam2,x0,t0,N,tau,yTilde,r_star+10*H_null*rand(sum(sel),1));

