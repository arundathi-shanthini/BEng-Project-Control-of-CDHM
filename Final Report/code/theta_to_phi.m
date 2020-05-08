function [phi_p,phi_d] = fcn(theta)
N=9;
% Create theta
phi_p=theta(1:2:2*N-1);
for n=1:N
    phi_p(n)=(-1)^(n+1)*phi_p(n);
end
phi_d=theta(2:2:2*N);
