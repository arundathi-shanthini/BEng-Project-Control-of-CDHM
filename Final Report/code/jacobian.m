function J = fcn(a,theta)
N=9;
% Calculate D-H parameter table
DH_params=zeros(2*N,4);
for i=1:N
    if rem(i,2)==0
        DH_params(2*i-1:2*i,:)=[180+(theta(2*i-1)*180/pi) 90 0 0; 180+(theta(2*i)*180/pi) 0 a 0];
    else
        DH_params(2*i-1:2*i,:)=[(theta(2*i-1)*180/pi) 90 0 0; (theta(2*i)*180/pi) 0 a 0];
    end
end

% Calculate transformation matrix n_A_n+1 (A_wrt_prev) and store in cell
% array
A = cell(2*N,1);
for n=1:2*N
    theta_i=DH_params(n,1);
    alpha_i=DH_params(n,2);
    r_i=DH_params(n,3);
    d_i=DH_params(n,4);
    A{n}=[cosd(theta_i) -sind(theta_i)*cosd(alpha_i) sind(theta_i)*sind(alpha_i) r_i*cosd(theta_i);...
       sind(theta_i)  cosd(theta_i)*cosd(alpha_i) -cosd(theta_i)*sind(alpha_i) r_i*sind(theta_i);...
                  0              sind(alpha_i)   cosd(alpha_i)              d_i;...
                  0              0              0                         1];
end

% Create a list of transformation matrices wrt base for all joints
A_till_prev_wrt_base=cell(2*N,1);
A_till_prev_wrt_base{1}=eye(4);
for n=1:2*N-1
    A_till_prev_wrt_base{n+1}=A_till_prev_wrt_base{n}*A{n};
end
A_2N=A_till_prev_wrt_base{2*N}*A{2*N};
% Create Jacobian
J=zeros(6,2*N);
for n=1:2*N
    J_v=cross((A_till_prev_wrt_base{n}(1:3,1:3)*[0;0;1]),(A_2N(1:3,4)-A_till_prev_wrt_base{n}(1:3,4)));
    J_w=A_till_prev_wrt_base{n}(1:3,1:3)*[0;0;1];
    J(:,n)=[J_v;J_w];
end
