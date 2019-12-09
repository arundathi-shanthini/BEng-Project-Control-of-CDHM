clear all;
close all;
d=23/2;
phi_p=0;
phi_d=0;
n=1;
k=1;
r=20.5;
if rem(n,2)==0
    delta=(12*(round((k-1)/3)+1)+120*(k-1)+90)*pi/180;
else
    delta=(12*(round((k-1)/3)+1)+120*(k-1))*pi/180;
end
s_p=sin(phi_p);
s_d=sin(phi_d);
c_p=cos(phi_p);
c_d=cos(phi_d);
c_delta=cos(delta);
s_delta=sin(delta);
A=[ c_p        0     s_p        s_p*d;...
    s_d*s_p   c_d   -s_d*c_p   -d*s_d*c_p ;...
   -c_d*s_p   s_d    c_p*c_d    d*(c_p*c_d+1);...
    0         0      0          1];
H=[r*c_delta;r*s_delta;0;1];
trans_H=A*H;
l_k_n=trans_H(1:3)-H(1:3);
l_k_n=abs(sqrt((l_k_n(1))^2+(l_k_n(2))^2+(l_k_n(3))^2));
% l(k,n)=
