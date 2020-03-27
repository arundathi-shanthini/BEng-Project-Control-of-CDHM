function [phi_p,phi_d] = fcn(x_e)
N=9;
a=140/1000;
d=23/2/1000;
r=20.5/1000;
error_tol=1/1000;

theta_j=zeros(2*N,1);

for i=1:N
    if rem(i,2)==0
        DH_params=[DH_params;pi pi/2 0 0; pi 0 a 0];
    else
        DH_params=[DH_params;0 pi/2 0 0; 0 0 a 0];
    end
end

x_e_j=zeros(6,1);

%Numerical iteration to find solution
for j=0:10000
   delta_x_e=x_e-x_e_j;

   if (delta_l(1)<=error_tol) && (delta_l(2)<=error_tol) && (delta_l(3)<=error_tol)
       theta=theta_j;
       break;
   end
   
   A_wrt_base=eye(4);
   J_w=[];
   J_v=[];

   for n=1:2*N
        theta_i=DH_params(n,1);
        alpha_i=DH_params(n,2);
        a_i=DH_params(n,3);
        d_i=DH_params(n,4);
        A=[cos(theta_i) -sin(theta_i)*cos(alpha_i) sin(theta_i)*sin(alpha_i) a_i*cos(theta_i);...
           sin(theta_i)  cos(theta_i)*cos(alpha_i) -cos(theta_i)*sin(alpha_i) a_i*sin(theta_i);...
                      0              sin(alpha_i)   cos(alpha_i)              d_i;...
                      0              0              0                         1];
        A_wrt_base=A;
        J_w=[J_w A_wrt_base(1:3,3)];
        J_v=[J_v A_wrt_base(1:3,3)];
   end
   
   J=[J_v J_w];
   J_inv=pinv(J);

   delta_theta= J_inv*delta_x_e;
   theta_j=theta_j+delta_theta;

   if j==10000
       theta=NaN;
   end
end
if isnan(phi_p) && isnan(phi_d)
    phi_p=theta(1:2:2*N-1);
    for n=1:N
        phi_p(n)=(-1)^(n+1)*phi_p(n);
    end
    phi_d=theta(2:2:2*N);
end
end

