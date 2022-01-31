function [r_opt] = opt_refs_PDcontrol_DBLint(lam1, lam2, x0, t0, N, tau, yTilde)
%OPT_REFS_PDCONTROL_DBLINT Computes the optimal references of double int
%   This function computes the optimal references of a double integrator
%   controlled by a PD controller. Hence, the system is specifies by the
%   following parameters:
%      lam1, lam2: proportional and derivative gains, respectively. From
%         these values it follows that the matrix A is
%         A=[0 1; -lam1*lam2 lam1+lam2]
%      x0: sampled state at time t0
%      t0: absolute instant from which references are needed
%      N: number of references to be set
%      tau: interval from one reference switch to the next one
%      yTilde: output to be tracked

%% Matrices of the dynamics
if (abs(lam1-lam2)<10*eps)
	disp("WARNING: lam1 and lam2 too close. Numerical issues will pop up")
end
A = [0 1; -lam1*lam2 lam1+lam2];
B = [0; lam1*lam2];
C = [1 0];
Phi = @(t) 1/(lam2-lam1)*[lam2*exp(lam1*t)-lam1*exp(lam2*t) exp(lam2*t)-exp(lam1*t);
	lam1*lam2*(exp(lam1*t)-exp(lam2*t)) lam2*exp(lam2*t)-lam1*exp(lam1*t)];
Gamma = @(t) lam1*lam2/(lam2-lam1)*[(exp(lam2*t)-1)/lam2-(exp(lam1*t)-1)/lam1;
	exp(lam2*t)-exp(lam1*t)];
[n,m] = size(B);
if (n ~= 2)
	disp("WARNING: the state space of double integrator has dimension 2")
end
if (m ~= 1)
	disp("WARNING: the input/outputspace of double integrator has dimension 1")
end

%% Discout factor of future costs
beta = 0;

%% Instants and separations (in optimized versions to be simplified)
tK = linspace(0,N*tau,N+1);
tauK = diff(tK);

%% Intermediate matrices: Xtilde matrices depend on the trajectory
Xtilde_P = zeros(1,n,N);
Xtilde_G = zeros(1,m,N);
Xtilde = zeros(1,m*N,N);
for k=1:N   % in the paper this k runs kPrime to kPrime+N-1
	Xtilde_P(:,:,k) = inner_beta_product(@(t) yTilde(t+tK(k)), @(t) C*Phi(t),beta,tauK(k));
	Xtilde_G(:,:,k) = inner_beta_product(@(t) yTilde(t+tK(k)), @(t) C*Gamma(t),beta,tauK(k));
	for j=1:k-1
		Xtilde(:,(j-1)*m+(1:m),k) = Xtilde_P(:,:,k)*...
			Phi(tK(k)-tK(1+j))*Gamma(tauK(1+j-1));
	end
	Xtilde(:,(k-1)*m+(1:m),k) = Xtilde_G(:,:,k);
end

%% Intermediate matrices: M_PP, M_PG, M_GG, V, Q depends on the system
M_PP = zeros(n,n,N);
M_PG = zeros(n,m,N);
M_GG = zeros(m,m,N);
for k=1:N   % in the paper this k runs kPrime to kPrime+N-1
	M_PP(:,:,k) = inner_beta_product(@(t) C*Phi(t), @(t) C*Phi(t),beta,tauK(k));
	M_PG(:,:,k) = inner_beta_product(@(t) C*Phi(t), @(t) C*Gamma(t),beta,tauK(k));
	M_GG(:,:,k) = inner_beta_product(@(t) C*Gamma(t), @(t) C*Gamma(t),beta,tauK(k));
end
V = zeros(n,m*N,N);
for k=1:N
	for j=1:k-1
		V(:,(j-1)*m+(1:m),k) = M_PP(:,:,k)*...
			Phi(tK(k)-tK(1+j))*Gamma(tauK(1+j-1));
	end
	V(:,(k-1)*m+(1:m),k) = M_PG(:,:,k);
	V(:,:,k) = Phi(tK(k)-tK(1))'*V(:,:,k);
end
Q = zeros(m*N,m*N,N);
for k=1:N
	% upper left square block of indices less than k
	for i=1:k-1   % loop on block rows
		for j=1:k-1   % loop on block cols
			Q((i-1)*m+(1:m),(j-1)*m+(1:m),k) = ...
				Gamma(tauK(1+i-1))'*Phi(tK(k)-tK(1+i))'*...
				M_PP(:,:,k)*...
				Phi(tK(k)-tK(1+j))*Gamma(tauK(1+j-1));
		end
	end
	% upper k-th column
	for i=1:k-1
		Q((i-1)*m+(1:m),(k-1)*m+(1:m),k) = ...
			Gamma(tauK(1+i-1))'*Phi(tK(k)-tK(1+i))'*M_PG(:,:,k);
	end
	% left k-th row
	for j=1:k-1
		Q((k-1)*m+(1:m),(j-1)*m+(1:m),k) = ...
			M_PG(:,:,k)'*Phi(tK(k)-tK(1+j))*Gamma(tauK(1+j-1));
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
	alpha_k = exp(-beta*(tK(k)-tK(1)));
	Xtilde_final = Xtilde_final+alpha_k*Xtilde(:,:,k);
	V_final = V_final+alpha_k+V(:,:,k);
	Q_final = Q_final+alpha_k*Q(:,:,k);
end
% in the real world, we should check the null space of Q and drop the
% subspaces along with Q has very small eigenvalues. In the quick and
% dirt yworld, we just compute the solution

r_opt = inv(Q_final)*(Xtilde_final'-V_final'*x0);

