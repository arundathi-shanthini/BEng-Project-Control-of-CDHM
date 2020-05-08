function [phi_p,phi_d] = fcn(l)
N=9;
d=23/2/1000;
r=20.5/1000;
error_tol=1/1000;
phi_d=zeros(N,1);
phi_p=zeros(N,1);
% Numerical iteration to find solution
for n=1:N
    % Get delta
    if rem(n,2)==0
        delta=[(12*(round(((3*n)-1)/3)+1)+120*((3*n)-1)+90)*pi/180;...
               (12*(round(((3*n-1)-1)/3)+1)+120*((3*n-1)-1)+90)*pi/180;...
               (12*(round(((3*n-2)-1)/3)+1)+120*((3*n-2)-1)+90)*pi/180];
    else
        delta = [(12*(round(((3*n)-1)/3)+1)+120*((3*n)-1))*pi/180; ...
                (12*(round(((3*n-1)-1)/3)+1)+120*((3*n-1)-1))*pi/180; ...
                (12*(round(((3*n-2)-1)/3)+1)+120*((3*n-2)-1))*pi/180];
    end

   l_d=l(3*n-2:3*n);
   l_d=l_d/N;
   
   phi_j=[0;0];
   
   for j=0:500
       % Calculate l_j
       l_3n_0=get_length(phi_j(1),phi_j(2),delta(1));
       l_3n_1=get_length(phi_j(1),phi_j(2),delta(2));
       l_3n_2=get_length(phi_j(1),phi_j(2),delta(3));
       l_j=[l_3n_0;...
            l_3n_1;...
            l_3n_2];
       % Calculate error
       delta_l=l_d-l_j;
       % Check if the error is within tolerance
       if (delta_l(1)<=error_tol) && (delta_l(2)<=error_tol) && (delta_l(3)<=error_tol)
           phi_d(n)=phi_j(1);
           phi_p(n)=phi_j(2);
           break;
       end
       % Calculate the Jacobian         
       [J_1_1 J_1_2] = get_partial_diff(phi_j(1),phi_j(2),delta(1));
       [J_2_1 J_2_2] = get_partial_diff(phi_j(1),phi_j(2),delta(2));
       [J_3_1 J_3_2] = get_partial_diff(phi_j(1),phi_j(2),delta(3));
       J=[[J_1_1 J_1_2];...
          [J_2_1 J_2_2];...
          [J_3_1 J_3_2]];
       J_inv=pinv(J);
       %Calculate error
       delta_phi= J_inv*delta_l;
       % Use error to estimate delta_theta
       phi_j=phi_j+delta_phi;
       
       if j==10000
           phi_d(n)=NaN;
           phi_p(n)=NaN;
       end
   end
end
