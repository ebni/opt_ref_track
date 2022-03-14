function [r_opt, y_opt, Q_final, lin_final] = opt_refs_uav(sysd, x0, t0, N, tau, yTildex, yTildey)
    % Finds optimal references for a uav to follow a given trajectory
    % yTilde in both x and y direction
    
    % Get eigenvalues of A matrix (for single dimension)
    e = eig(sysd.A); lam1 = e(1); lam2 = e(2);
    
    % Get optimal references in the x direction
    [r_opt.x, Q_final.x, lin_final.x] = opt_refs_PDcontrol_DBLint(lam1, lam2, [x0(1);x0(3)], t0, N, tau, yTildex);
    [r_opt.y, Q_final.y, lin_final.y] = opt_refs_PDcontrol_DBLint(lam1, lam2, [x0(2);x0(4)], t0, N, tau, yTildey);
    
    % Get resulting trajectory as well
    [y_opt.x.ts y_opt.x.ys] = get_traj(lam1, lam2, [x0(1);x0(3)], t0, N, tau, r_opt.x);
    [y_opt.y.ts y_opt.y.ys] = get_traj(lam1, lam2, [x0(2);x0(4)], t0, N, tau, r_opt.y);
end