function [ ts, ys ] = get_traj(lam1, lam2, x0, t0, N, tau, r)
    % Get resulting trajectory from a series of references r
    
    % Define propagating functions
    n=2;
    Phi = @(t) 1/(lam2-lam1)*[lam2*exp(lam1*t)-lam1*exp(lam2*t) exp(lam2*t)-exp(lam1*t);
        lam1*lam2*(exp(lam1*t)-exp(lam2*t)) lam2*exp(lam2*t)-lam1*exp(lam1*t)];
    Gamma = @(t) lam1*lam2/(lam2-lam1)*[(exp(lam2*t)-1)/lam2-(exp(lam1*t)-1)/lam1;
        exp(lam2*t)-exp(lam1*t)];
    C = [1 0];
    
    ts = []; ys = [];
    xK = zeros(n,N+1);  % sampled state
    xK(:,1) = x0;
    tK = linspace(t0,t0+N*tau,N+1);
    for k=1:N
        xK(:,k+1) = Phi(tau)*xK(:,k)+Gamma(tau)*r(k);
        timespan = linspace(tK(k),tK(k+1),ceil(100/N+2));
        yspan = timespan;    % init to something of appropriate size
        for i=1:length(timespan)
            yspan(i) = C*(Phi(timespan(i)-timespan(1))*xK(:,k)+Gamma(timespan(i)-timespan(1))*r(k));
        end
        ts = [ts timespan]; ys = [ys yspan];
    end

end