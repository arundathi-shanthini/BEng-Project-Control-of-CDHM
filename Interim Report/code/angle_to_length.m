function l = fcn(phi_p,phi_d)
N=9;
d=23/2/1000;
r=20.5/1000;
l=zeros(3*N,N);
% Calculate the length of cable between joints
for n=1:N
    for k=1:3*N
        if rem(n,2)==0
            delta=(12*(round((k-1)/3)+1)+120*(k-1)+90)*pi/180;
        else
            delta=(12*(round((k-1)/3)+1)+120*(k-1))*pi/180;
        end
        
        s_p=sin(phi_p(n));
        s_d=sin(phi_d(n));
        c_p=cos(phi_p(n));
        c_d=cos(phi_d(n));
        c_delta=cos(delta);
        s_delta=sin(delta);
        A=[ c_p        0     s_p        s_p*d;...
            s_d*s_p   c_d   -s_d*c_p   -d*s_d*c_p ;...
           -c_d*s_p   s_d    c_p*c_d    d*(c_p*c_d+1);...
            0         0      0          1];
        H=[r*c_delta;r*s_delta;0;1];
        trans_H=A*H;
        l_k_n=trans_H(1:3)-H(1:3);
        l(k,n)=abs(sqrt((l_k_n(1))^2+(l_k_n(2))^2+(l_k_n(3))^2));
    end
end
