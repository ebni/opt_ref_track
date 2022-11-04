function [tK, Uvec, minCost] = optQuanta(uOpt, N, t_f)
%   Optimal Fast Numerical Quantization-based Sampling
% INPUTS
%   uOpt(t): optimal input to be quantized
%   N: number of sampling instants in [0, t_f]
%   t_f: final instant
% OUTPUTS
%   tK: tK(i), with i=1,...,N+1, is the (i-1)-th instant
%   Uvec: Uvec(:,k) is the optimal k-th input
%   minCost: quantization error at minimum

% Lipschitz constants: now numerical, if passed analytic, the better
Lmin = +inf;
Lmax = 0;
num_steps = 1000;   % large number
all_t = linspace(0,t_f,num_steps+1);
for k=1:num_steps
	curL = norm(uOpt(all_t(k+1))-uOpt(all_t(k)),2)/(all_t(k+1)-all_t(k));
	Lmax = max(Lmax,curL);
	Lmin = min(Lmin,curL);
end

%% Numerical search of the midpoint

% Equally spaced instants is the initial guess
tK = linspace(0,t_f, N+1);

Uvec = zeros(size(uOpt(0),1),N);

iter=1;
while (iter <= 100)     % STOP CONDITION (1): too many iterations

	% Computing averages
	for k=1:N
		Uvec(:,k) = integral(uOpt, tK(k), tK(k+1),"ArrayValued",true)/(tK(k+1)-tK(k));
	end
	% Serching for new tK(k+1)
	for k=1:N-1
		delta = Uvec(:,k+1)-Uvec(:,k);          % orthogonal to mid-plane
		b_mid = 0.5*(Uvec(:,k+1)'*Uvec(:,k+1)-Uvec(:,k)'*Uvec(:,k));

		if (dot(delta,uOpt(tK(k+1))) <= b_mid)
			tKprev = tK(k+1);
			tKnext = tK(k+2);
		else
			tKprev = tK(k);
			tKnext = tK(k+1);
		end

		% Now bisecting [tkprev,tKnext]
		%		while(0)
%		iter_inner = 0;
		while (tKnext-tKprev >= 1e-6)  % STOP CONDITION (2): interval small
%			Uprev = uOpt(tKprev);
%			Unext = uOpt(tKnext);
%			t_new = (b_mid-dot(delta,Uprev))/dot(delta,Unext-Uprev)*(tKnext-tKprev)+tKprev;
			t_new = (tKprev+tKnext)*0.5;
			if (dot(delta,uOpt(t_new)) < b_mid)
				tKprev = t_new;
			else
				tKnext = t_new;
			end
%			iter_inner = iter_inner+1;
%			if (iter_inner >= 100)
%				break;
%			end
		end
		tK(k+1) = (tKnext+tKprev)*0.5;
	end
	iter = iter+1;
end

% Computing averages
for k=1:N
	Uvec(:,k) = integral(uOpt, tK(k), tK(k+1),"ArrayValued",true)/(tK(k+1)-tK(k));
end

% Cost
minCost = 0;
for k=1:N
	y_error = @(t) uOpt(t)-Uvec(:,k);
	y_error2 = @(t) dot(y_error(t),y_error(t));
	newCost = integral(y_error2, tK(k), tK(k+1),"ArrayValued",true);
	minCost = minCost+newCost;
end
