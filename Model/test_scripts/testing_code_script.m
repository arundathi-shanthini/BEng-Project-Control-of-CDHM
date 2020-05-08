N=9;
theta=(1:18)';
phi_p=theta(1:2:2*N-1);
for n=1:N
    phi_p(n)=(-1)^(n+1)*phi_p(n);
end
phi_d=theta(2:2:2*N);
% N=9;
% a=140/1000;
% 
% % Calculate D-H parameter table
% DH_params=zeros(2*N,4);
% for i=1:N
%     if rem(i,2)==0
%         DH_params(2*i-1:2*i,:)=[180 90 0 0; 180 0 a 0];
%     else
%         DH_params(2*i-1:2*i,:)=[0 90 0 0; 0 0 a 0];
%     end
% end
% 
% A = cell(2*N,1);
% for n=1:2*N
%     theta_i=DH_params(n,1);
%     alpha_i=DH_params(n,2);
%     r_i=DH_params(n,3);
%     d_i=DH_params(n,4);
%     A{n}=[cosd(theta_i) -sind(theta_i)*cosd(alpha_i) sind(theta_i)*sind(alpha_i) r_i*cosd(theta_i);...
%        sind(theta_i)  cosd(theta_i)*cosd(alpha_i) -cosd(theta_i)*sind(alpha_i) r_i*sind(theta_i);...
%                   0              sind(alpha_i)   cosd(alpha_i)              d_i;...
%                   0              0              0                         1];
% end
% 
% % Create a list of transformation matrices wrt base for all joints
% A_wrt_base=cell(2*N,1);
% for n=1:2*N
%     A_base=eye(4);
%     for i=1:n
%         A_base=A_base*A{i};
%     end
%     A_wrt_base{n}=A_base;
% end
% % Create Jacobian
% J=zeros(6,2*N);
% for n=1:2*N
%     if n==1
%         J_v=cross((A_wrt_base{n}(1:3,1:3)*[0;0;1]),(A_wrt_base{n}(1:3,4)-[0;0;0]));
%     else
%         J_v=cross((A_wrt_base{n}(1:3,1:3)*[0;0;1]),(A_wrt_base{n}(1:3,4)-A_wrt_base{n-1}(1:3,4)));
%     end
%     J_w=A{n}(1:3,1:3)*[0;0;1];
%     J(:,n)=[J_v;J_w];
% end
% 
