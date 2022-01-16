function prod = inner_beta_product(f1, f2, beta, tau)
% INNER_BETA_PRODUCT Computes the inner product weighted by beta
%   f1, f2 are the two function handles
%   tau is the integration interval
prod = integral(@(t) exp(-beta*t)*f1(t)'*f2(t), 0, tau, 'ArrayValued',1);
end

