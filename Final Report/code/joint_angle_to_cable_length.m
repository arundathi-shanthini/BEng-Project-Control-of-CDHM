function l = fcn(phi_p,phi_d)
N=9;
d=23/2/1000;
r=20.5/1000;
l=zeros(3*N,1);
% Calculate transformation matrices
A=cell(N,1);
for n=1:N
    s_p=sin(phi_p(n));
    s_d=sin(phi_d(n));
    c_p=cos(phi_p(n));
    c_d=cos(phi_d(n));
    A{n}=[ c_p        0     s_p        s_p*d;...
           s_d*s_p   c_d   -s_d*c_p   -d*s_d*c_p ;...
           -c_d*s_p   s_d    c_p*c_d    d*(c_p*c_d+1);...
             0         0      0          1];
end
% Calculate the length of cable between joints
for k=1:3*N
    l_k = 0;
    % The link at which cable k terminates
    M = floor((k-1)/3)+1;
    for n=1:M
        % Calculate the angular position of the hole on distal and proximal
        % discs
        if rem(n,2)==0
            delta_p=((120/(N+1))*(M)+120*(k-1)+87);
            delta_d=((120/(N+1))*(M)+120*(k-1)-3);
        else
            delta_p=((120/(N+1))*(M)+120*(k-1)-3);
            delta_d=((120/(N+1))*(M)+120*(k-1)+87);
        end
        % Calculate hole position vector on distal and proximal discs
        c_delta_p=cosd(delta_p);
        s_delta_p=sind(delta_p);
        c_delta_d=cosd(delta_d);
        s_delta_d=sind(delta_d);
        H_p=[r*c_delta_p;r*s_delta_p;0;1];
        H_d=[r*c_delta_d;r*s_delta_d;0;1];
        % Get the length of cable between the proximal and distal disc at
        % joint n
        trans_H_p=A{n}*H_p;
        l_k_n=trans_H_p(1:3)-H_d(1:3);
        l_k = l_k + norm(l_k_n);
    end
    l(k)=l_k;
end
