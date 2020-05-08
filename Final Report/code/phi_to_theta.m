function theta = fcn(phi_p,phi_d)
N=9;
% Create theta
theta=zeros(2*N,1);
for n=1:N
    theta(2*n-1:2*n)=[(-1)^(n+1)*phi_p(n);phi_d(n)];
end
