clear all
clc
close all

SIMTIME = 300;
m = 0.68;
g = 9.81;
L = 0.17;
Kt = 5.4e-5;
Kq = 1.1e-6;
reference_area = pi * 75e-3^2;
drag_coefficient = 0.47;
kdx = 0.5*drag_coefficient*reference_area; kdy = kdx; kdz = kdx;
Kd = [kdx;kdy;kdz];
drag_mat = [kdx 0 0; 0 kdy 0; 0 0 kdz];
inertia_xx = 0.007;
inertia_yy = 0.007;
inertia_zz = 0.012;
J = [inertia_xx; inertia_yy; inertia_zz]; % Body inertia
Jr = 8.54858e-06;
Gamma = [Kt Kt Kt Kt;
         0 -L*Kt 0 L*Kt;
         L*Kt 0 -L*Kt 0;
         Kq -Kq Kq -Kq];
Gamma_inv = inv(Gamma);
mot_speeds = [1 -1 1 -1]';
max_mot_speeds = ones(4,1).*[300;300;300;300].*mot_speeds;
max_torques = Gamma*(max_mot_speeds).^2;

%Linearize at hover 
% x y z u v r phi theta psi p q r
u0 = [m*g 0 0 0]'; x0 = zeros(12,1);
speeds0 = mot_speeds.*sign(Gamma_inv*u0).*sqrt(abs(Gamma_inv*u0));
[x0, u0, y0, ~] = trim('nonlin_quad_plant', x0, u0);
[A,B,C,D] = linmod('nonlin_quad_plant', x0, u0);
[Wn, ~] = damp(A);
sample_freq = 100; %should be at least twice than the fastest dynamics
sample_time = 1/sample_freq;
time = 0:sample_time:SIMTIME;

save("params.mat")