function theta = fcn(phi_p,phi_d)
N=9;
%Calculate theta
theta=zeros(2*N,1);
for n=1:N
    theta(2*n-1:2*n)=[(-1)^(n+1)*phi_p;phi_d];
end
