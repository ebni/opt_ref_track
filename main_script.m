%% Double integrator controller by PD controller
% eigenvalues of closed loop dynamics: DO NOT set them equal, otherwise
% different math applies
lam1=-1;
lam2=-2;

% Closed loop matrices
A = [0 1; -lam1*lam2 lam1+lam2];
% A is diagonalized by A=H*diag[lam1 lam2])*invH
H = [1 1; lam1 lam2];
invH = 1/(lam2-lam1)*[lam2 -1; -lam1 1];
B = [0; lam1*lam2];
Phi = @(t) 1/(lam2-lam1)*[lam2*exp(lam1*t)-lam1*exp(lam2*t) exp(lam2*t)-exp(lam1*t);
	lam1*lam2*(exp(lam1*t)-exp(lam2*t)) lam2*exp(lam2*t)-lam1*exp(lam1*t)];
Gamma = @(t) lam1*lam2/(lam2-lam1)*[(exp(lam2*t)-1)/lam2-(exp(lam1*t)-1)/lam1;
	exp(lam2*t)-exp(lam1*t)];
C = [1 0];  % first state is output to be tracked

%% sizes
[n,m] = size(B);
% C and A should have sizes n\times n, and m\times n

% Sampling
N = 15;  % number of intervals
T = 3;   % time horizon
tK = linspace(0,T,N+1);  % periodic sampling
tauK = diff(tK);
kPrime = 1; % index of the 1st reference to be set
xPrime = [0; 0]; % initial state sampled at tK(kPrime)
beta = 1;

% trajectory to be followed
yTilde = @(t) sin(t).*exp(-2*t);
beta = 2; % discout factor of future costs
fprintf("Initial output gap: %f\n", C*xPrime-yTilde(tK(kPrime)));

%% Intermediate matrices: Xtilde matrices depend on the trajectory
Xtilde_P = zeros(1,n,N);
Xtilde_G = zeros(1,m,N);
Xtilde = zeros(1,m*N,N);
for k=1:N   % in the paper this k runs kPrime to kPrime+N-1
	Xtilde_P(:,:,k) = inner_beta_product(@(t) yTilde(t+tK(kPrime+k-1)), @(t) C*Phi(t),beta,tauK(kPrime+k-1));
	Xtilde_G(:,:,k) = inner_beta_product(@(t) yTilde(t+tK(kPrime+k-1)), @(t) C*Gamma(t),beta,tauK(kPrime+k-1));
	for j=1:k-1
		Xtilde(:,(j-1)*m+(1:m),k) = Xtilde_P(:,:,k)*...
			Phi(tK(kPrime+k-1)-tK(kPrime+j))*Gamma(tauK(kPrime+j-1));
	end
	Xtilde(:,(k-1)*m+(1:m),k) = Xtilde_G(:,:,k);
end

%% Intermediate matrices: M_PP, M_PG, M_GG, V, Q depends on the system
M_PP = zeros(n,n,N);
M_PG = zeros(n,m,N);
M_GG = zeros(m,m,N);
for k=1:N   % in the paper this k runs kPrime to kPrime+N-1
	M_PP(:,:,k) = inner_beta_product(@(t) C*Phi(t), @(t) C*Phi(t),beta,tauK(kPrime+k-1));
	M_PG(:,:,k) = inner_beta_product(@(t) C*Phi(t), @(t) C*Gamma(t),beta,tauK(kPrime+k-1));
	M_GG(:,:,k) = inner_beta_product(@(t) C*Gamma(t), @(t) C*Gamma(t),beta,tauK(kPrime+k-1));
end
V = zeros(n,m*N,N);
for k=1:N
	for j=1:k-1
		V(:,(j-1)*m+(1:m),k) = M_PP(:,:,k)*...
			Phi(tK(kPrime+k-1)-tK(kPrime+j))*Gamma(tauK(kPrime+j-1));
	end
	V(:,(k-1)*m+(1:m),k) = M_PG(:,:,k);
	V(:,:,k) = Phi(tK(kPrime+k-1)-tK(kPrime))'*V(:,:,k);
end
Q = zeros(m*N,m*N,N);
for k=1:N
	% upper left square block of indices less than k
	for i=1:k-1   % loop on block rows
		for j=1:k-1   % loop on block cols
			Q((i-1)*m+(1:m),(j-1)*m+(1:m),k) = ...
				Gamma(tauK(kPrime+i-1))'*Phi(tK(kPrime+k-1)-tK(kPrime+i))'*...
				M_PP(:,:,k)*...
				Phi(tK(kPrime+k-1)-tK(kPrime+j))*Gamma(tauK(kPrime+j-1));
		end
	end
	% upper k-th column
	for i=1:k-1
		Q((i-1)*m+(1:m),(k-1)*m+(1:m),k) = ...
			Gamma(tauK(kPrime+i-1))'*Phi(tK(kPrime+k-1)-tK(kPrime+i))'*M_PG(:,:,k);
	end
	% left k-th row
	for j=1:k-1
		Q((k-1)*m+(1:m),(j-1)*m+(1:m),k) = ...
			M_PG(:,:,k)'*Phi(tK(kPrime+k-1)-tK(kPrime+j))*Gamma(tauK(kPrime+j-1));
	end
	% element at (k,k) position
	Q((k-1)*m+(1:m),(k-1)*m+(1:m),k) = M_GG(:,:,k);
end

%% Forcing symmetry of Q to fix floating point arithmetic issues
for k=1:N
	Q_sym = 0.5*(Q(:,:,k)+Q(:,:,k)');
	Q_asym = 0.5*(Q(:,:,k)-Q(:,:,k)');
	if norm(Q_asym)/norm(Q_sym) > 1e-6
		fprintf("Possible issues when symmetrizing Q(:,:,%d)\n", k);
	end
	Q(:,:,k) = Q_sym;
end

%% Weighting everything by \alpha_k
Xtilde_final = zeros(1,m*N);
V_final = zeros(n,m*N);
Q_final = zeros(m*N,m*N);
for k=1:N
	alpha_k = exp(-beta*(tK(kPrime+k-1)-tK(kPrime)));
	Xtilde_final = Xtilde_final+alpha_k*Xtilde(:,:,k);
	V_final = V_final+alpha_k+V(:,:,k);
	Q_final = Q_final+alpha_k*Q(:,:,k);
end
% in the real world, we should check the null space of Q and drop the
% subspaces along with Q has very small eigenvalues. In the quick and
% dirt yworld, we just compute the solution

r_opt = inv(Q_final)*(Xtilde_final'-V_final'*xPrime);

%% Plotting target trajectory, state trajectory and references
timespan = linspace(0,T,100);
plot(timespan,yTilde(timespan));
hold on
stairs(tK(1:end-1),r_opt);