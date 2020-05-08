%% Neton-Euler Recursive Algorithm for Inverse Dynamics
function u  = fcn(theta, d_theta,dd_theta,m,a,d,r,g,F_ec,T_ec)
%% Parameters
N=9;
P_joint_to_joint=[a+2*d;0;0];
P_link_cm=[-(a/2+d);0;0];
z_0=[0;0;1];
x_0=[1;0;0];
% Create the D-H parameter table
DH_params=zeros(2*N,4);
for i=1:N
    if rem(i,2)==0
        DH_params(2*i-1:2*i,:)=[180+(theta(2*i-1)*180/pi) 90 0 0;...
                                180+(theta(2*i)*180/pi) 0 a 0];
    else
        DH_params(2*i-1:2*i,:)=[theta(2*i-1)*180/pi 90 0 0;...
                                theta(2*i)*180/pi 0 a 0];
    end
end
% Calculate transformation matrix n_A_n+1 (A_wrt_prev) and store in cell
% array
R = cell(2*N,1);
for n=1:2*N
    theta_i=DH_params(n,1);
    alpha_i=DH_params(n,2);
    R{n}=[cosd(theta_i) -sind(theta_i)*cosd(alpha_i) sind(theta_i)*sind(alpha_i);...
       sind(theta_i)  cosd(theta_i)*cosd(alpha_i) -cosd(theta_i)*sind(alpha_i);...
                  0              sind(alpha_i)   cosd(alpha_i) ];
end
% Calculate Inertia tensor
I = cell(N,1);
for i=1:N
    I{i}=[1/12*(m*(a)^2)+1/4*(m*r^2) 0 0;
                      0 1/12*(m*(a)^2)+1/4*(m*r^2) 0;
                      0 0 1/4*(m*r^2)];
end
%% Forward recursion
% Initialising the matrices
omega=zeros(3,N);
d_omega=zeros(3,N);
a_joint=zeros(3,N);
a_c=zeros(3,N);
% Initialising prev values
omega_prev=zeros(3,1);
d_omega_prev=zeros(3,1);
a_joint_prev=zeros(3,1);
for n=1:N
    omega(:,n)=(R{2*n-1}*R{2*n})'*(omega_prev + z_0*d_theta(2*n-1)) + ...
               (R{2*n})'*z_0*d_theta(2*n);
    d_omega(:,n)=(R{2*n-1}*R{2*n})'*(d_omega_prev + z_0*dd_theta(2*n-1)+...
                                     cross(omega(:,n),z_0*d_theta(2*n-1))) + ...
                 (R{2*n-1}*R{2*n})'*(cross((z_0*d_theta(2*n-1) + omega_prev),R{2*n-1}*z_0*d_theta(2*n))) + ...
                 (R{2*n})'*(z_0*dd_theta(2*n));
    if n==1
        a_joint(:,n)=[0;0;g];
    else
        a_joint(:,n)=(R{2*n-3}*R{2*n-2})'*a_joint_prev+ ...
                     cross(d_omega_prev,P_joint_to_joint)+...
                     cross(omega_prev,cross(omega_prev,P_joint_to_joint));
    end
    a_c(:,n)=(R{2*n-1}*R{2*n})'*a_joint(:,n)+...
             cross(d_omega(:,n),(P_joint_to_joint+P_link_cm))+...
             cross(omega(:,n),cross(omega(:,n),(P_joint_to_joint+P_link_cm)));
    % Update prev values for next iteration
    omega_prev=omega(:,n);
    d_omega_prev=d_omega(:,n);
    a_joint_prev=a_joint(:,n);
end

%% Backward recursion
% Initialising the matrices
F=zeros(3,N);
tau=zeros(3,N);
% Initialise next values
F_next=zeros(3,1);
tau_next=zeros(3,1);
for n=N:-1:1
   F(:,n) = F_next + m*a_c(:,n)-F_ec(:,n);
   tau(:,n) = tau_next - cross(F(:,n),(P_joint_to_joint+P_link_cm)) + ...
              cross(F_next,P_link_cm) + ...
              I{n}*d_omega(:,n)+ cross(omega(:,n),I{n}*omega(:,n))-T_ec(:,n);
   % Update next values for next iteration
   F_next=F(:,n);
   tau_next=tau(:,n);
end
%% Computing the output
u=vecnorm(tau)';
