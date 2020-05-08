function [F_ec,T_ec] = fcn(tau_m,theta,a,d,r,r_drum, l_link,eta,mu)
N=9;
F_ec=zeros(3,N);
T_ec=zeros(3,N);
F_cd=cell(N,3*N);
F_cp=cell(N,3*N);
for i=1:N
    for j=1:3*N
        F_cd{i,j}=[0;0;0];
        F_cp{i,j}=[0;0;0]; 
    end
end
r_d=cell(N,3*N);
r_p=cell(N,3*N);
T_p=cell(N,3*N);
T_d=cell(N,3*N);
P_link_cm=[-(a/2+d);0;0];
% Initialise prev values
Td_prev=tau_m*r_drum/eta;
% Calculate the Transformation matrices
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
A_wrt_base=cell(2*N,1);
A_wrt_base{1}=A{1};
for n=2:2*N
    A_wrt_base{n}=A_wrt_base{n-1}*A{n};
end

for i=1:N
    for j=1:3*N
        M=floor((j-1)/3)+1;
        % Calculate delta and r
        if rem(i,2)==0
            delta_p=((120/(N+1))*(M)+120*(j-1)+87);
            delta_d=((120/(N+1))*(M)+120*(j-1)-3);
            r_p_j=[d-l_link; r*cosd(delta_p);r*sind(delta_p); 1];
            r_d_j=[-d;r*cosd(delta_d);r*sind(delta_d); 1];
        else
            delta_p=((120/(N+1))*(M)+120*(j-1)-3);
            delta_d=((120/(N+1))*(M)+120*(j-1)+87);
            if i==1
                L_d_prev=[-d;r*cosd(delta_d);r*sind(delta_d); 1];
                e_d_prev=L_d_prev(1:3)/norm(L_d_prev(1:3));
                e_d_0=e_d_prev;
                r_d_prev=[-d;r*cosd(delta_d);r*sind(delta_d); 1];
            end
            r_p_j=[d-l_link; r*sind(delta_p);-r*cosd(delta_p); 1];
            r_d_j=[-d;r*sind(delta_d);-r*cosd(delta_d); 1];
        end
        r_p{i,j}=r_p_j(1:3);
        r_d{i,j}=r_d_j(1:3);
        % Calculate L
        if i==1
            L_p=(A{2*i}*r_p_j)-(r_d_prev);
        else
            L_p=(A{2*i}*r_p_j)-(A{2*i-2}*r_d_prev);
        end
        L_d=A{2*i}*(r_d_j-r_p_j);
        % Calculate e
        e_p=L_p(1:3)/norm(L_p(1:3));
        e_d=L_d(1:3)/norm(L_d(1:3));
        % Calculate phi
        phi_p=acos(dot(e_p,e_d));
        phi_d=acos(dot(e_d_prev,e_p));
        % Get Tp
        T_p_j=Td_prev/(exp(mu*phi_d));
        T_p{i,j}=T_p_j*e_p;
        % Get Td
        T_d_j=T_p_j/(exp(mu*phi_p));
        T_d{i,j}=T_d_j*e_d;
        % Update prev values
        e_d_prev=e_d;
        r_d_prev=r_d_j;
        Td_prev=T_d_j;
    end
end

% Calculate F_cp and F_cd
for n=1:N
    for k=1:3*N
        M=floor((k-1)/3)+1;
        if (3*n+1 <= k) && (k<=3*N) && n<=M
            F_cp{n,k}=T_d{n,k}-T_p{n,k};
            if n==1
                F_cd{n,k}=T_p{n,k}-((tau_m*r_drum/eta)*e_d_0);
            else
                F_cd{n,k}=T_p{n,k}-T_d{n-1,k};
            end
        end
        if (3*n-2 <= k) && (k <=3*n) && n==M
            F_cp{n,k}=-T_p{n,k};
        end
    end
end

% Calculate F_ec
for n=1:N
    F_ec_n=[0;0;0];
    A_2n=inv(A{2*n});
    R_2n=A_2n(1:3,1:3);
    for k=3*n+1:3*N
        F_ec_n=F_ec_n+((R_2n)*F_cd{n,k});
    end
    for k=3*n-2:3*N
        F_ec_n=F_ec_n+((R_2n)*F_cp{n,k});
    end
    F_ec(:,n)=F_ec_n;
end

% Calculate T_ec
for n=1:N
    T_ec_n=[0;0;0];
    A_2n=inv(A{2*n});
    R_2n=A_2n(1:3,1:3);
    for k=3*n+1:3*N
        T_ec_n=T_ec_n+cross((r_d{n,k}-P_link_cm),((R_2n)*F_cd{n,k}));
    end
    for k=3*n-2:3*N
        T_ec_n=T_ec_n+cross((r_p{n,k}-P_link_cm),((R_2n)*F_cp{n,k}));
    end
    T_ec(:,n)=T_ec_n;
end
