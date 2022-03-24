function [output_x, output_y]= output_PDcontrol_DBLint(lam1, lam2, x0, t0, N, tau, r)
%PLOT_PDCONTROL_DBLINT Summary of this function goes here
%   Same as in OPT_REFS_PDCONTROL_DBLINT. In addition, the following input
%   parameter is also used
%      r: the sequence of applied references

n=2;
Phi = @(t) 1/(lam2-lam1)*[lam2*exp(lam1*t)-lam1*exp(lam2*t) exp(lam2*t)-exp(lam1*t);
	lam1*lam2*(exp(lam1*t)-exp(lam2*t)) lam2*exp(lam2*t)-lam1*exp(lam1*t)];
Gamma = @(t) lam1*lam2/(lam2-lam1)*[(exp(lam2*t)-1)/lam2-(exp(lam1*t)-1)/lam1;
	exp(lam2*t)-exp(lam1*t)];
C = [1 0];

%% Check of the input
if (size(r,2) ~= 1)
	error("Only accepting a column vector of references")
	return;
end
if (size(r,1) < N)
	error("Array of references is too short")
	return
end
if (size(r,1) > N)
	warning("Size of references larger than number of intervals: cutting after N")
end

%% Output trajectory
xK = zeros(n,N+1);  % sampled state
xK(:,1) = x0;
tK = linspace(t0,t0+N*tau,N+1);
timespan = [];
yspan = [];
for k=1:N
	xK(:,k+1) = Phi(tau)*xK(:,k)+Gamma(tau)*r(k);
	timespan_cur = linspace(tK(k),tK(k+1),ceil(100/N+2));
	yspan_cur = timespan_cur;    % init to something of appropriate size
	for i=1:length(timespan_cur)
		yspan_cur(i) = C*(Phi(timespan_cur(i)-timespan_cur(1))*xK(:,k)+Gamma(timespan_cur(i)-timespan_cur(1))*r(k));
	end
	timespan = [timespan timespan_cur];
	yspan = [yspan yspan_cur];
end
output_x = timespan;
output_y = yspan;

%plot(tK,C*xK,'ko'); % plotting sampled states

end

