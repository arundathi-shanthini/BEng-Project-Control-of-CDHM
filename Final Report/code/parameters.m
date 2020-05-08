% This script is run as InitFcn by the Simulink model.
% clear all;

%% Physical parameters of CDHRM
% Number of links
N_links=10;
% Number of joints
N=N_links-1;
% Distance between 2 links
d=23/2000;
% Distance between 2 joints
a=140/1000;
% Radius of disc 
r=20.5/1000;
% Mass of the link
m=0.2;
% Length of the link
l_link=a-2*d;
%% Motor Characteristics : Maxon Motor 110063
% Transmission Efficiency
zeta = 0.8;
% Rotor Moment of Inertia
J_R = 0.864 * 10e-7;
% Load Moment of Inertia
J_L = 0.014 * 10e-7;
% Transmission efficiency
eta=0.39;
% Winch Transmission ratio
k_trans=0.36;
% Radius of the drum
r_drum=0.042;
% Pitch 
p=8/1000;
% Speed-torque Gradient
k_m=2560*100*pi/3;
% No load speed
n_0=9620*pi/30;
%% Physical constants
% acceleration due to gravity
g = 9.81;
% Coefficient of friction 
mu_cable=0.17;
% Steel to steel friction 
zeta_fr = 0.09;
