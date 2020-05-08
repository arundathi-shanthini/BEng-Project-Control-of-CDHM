r_c_prev_frame=zeros(3,2*N);
r_c=zeros(3,2*N);
for i=1:N
    r_c_prev(:,2*i-1:2*i)=[a/2 0;...
                            0  0;...
                            0  0];
    r_c(:,2*i-1:2*i)=[-a/2 0;...
                            0  0;...
                            0  0];
end
% N=9;
% x_e_prev=zeros(6,1);
% x_e=zeros(6,1);
% x_e(1)=3;
% theta_prev=zeros(2*N,1);
% 
% error_tol=1/1000;
% 
% theta_j=zeros(2*N,1);
% 
% %Numerical iteration to find solution
% for j=0:10000
%    % Calculate D-H parameter table
%     DH_params=zeros(2*N,4);
%     for i=1:N
%         if rem(i,2)==0
%             DH_params(2*i-1:2*i,:)=[180+(theta_j(2*i-1)*180/pi) 90 0 0; 180+(theta_j(2*i)*180/pi) 0 a 0];
%         else
%             DH_params(2*i-1:2*i,:)=[(theta_j(2*i-1)*180/pi) 90 0 0; (theta_j(2*i)*180/pi) 0 a 0];
%         end
%     end
% 
%     % Calculate transformation matrix n_A_n+1 (A_wrt_prev) and store in cell
%     % array
%     A = cell(2*N,1);
%     for n=1:2*N
%         theta_i=DH_params(n,1);
%         alpha_i=DH_params(n,2);
%         r_i=DH_params(n,3);
%         d_i=DH_params(n,4);
%         A{n}=[cosd(theta_i) -sind(theta_i)*cosd(alpha_i) sind(theta_i)*sind(alpha_i) r_i*cosd(theta_i);...
%            sind(theta_i)  cosd(theta_i)*cosd(alpha_i) -cosd(theta_i)*sind(alpha_i) r_i*sind(theta_i);...
%                       0              sind(alpha_i)   cosd(alpha_i)              d_i;...
%                       0              0              0                         1];
%     end
% 
%     % Create a list of transformation matrices wrt base for all joints
%     A_wrt_base=cell(2*N,1);
%     for n=1:2*N
%         A_base=eye(4);
%         for i=1:n
%             A_base=A_base*A{i};
%         end
%         A_wrt_base{n}=A_base;
%     end
% 
%     % Create Jacobian
%     J=zeros(6,2*N);
%     for n=1:2*N
%         if n==1
%             J_v=cross((A_wrt_base{n}(1:3,1:3)*[0;0;1]),(A_wrt_base{n}(1:3,4)-[0;0;0]));
%         else
%             J_v=cross((A_wrt_base{n}(1:3,1:3)*[0;0;1]),(A_wrt_base{n}(1:3,4)-A_wrt_base{n-1}(1:3,4)));
%         end
%         J_w=A{n}(1:3,1:3)*[0;0;1];
%         J(:,n)=[J_v;J_w];
%     end
%     
%    del_theta=theta_j-theta_prev;
%    
%    x_e_dot=J*del_theta;
%    x_e_j=x_e_prev+x_e_dot;
%    x_e_j_error=x_e-x_e_j;
%    if norm(x_e_j_error) <= error_tol
%        theta = theta_j;
%        break
%    end
%    % Use error to estimate delta_theta
%    J_inv=pinv(J);
%    delta_theta_j= J_inv*x_e_j_error;
%    % Update theta_j
%    theta_j=theta_j+delta_theta_j;
%    
%    if j==10000
%        theta=ones(2*N,1)*NaN;
%    end
% end
% if all(isnan(theta(:)))
%     disp('Could not find solution!')
% end
