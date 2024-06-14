clear all
clc
close all

% Sim params
SIMTIME = 100;

% Env params
g = 9.81;

% Quad physical parameters
m = 1.4; % Mass [m]
L = 0.56; % Distance between center of mass and motor [m]
Kd = 1.3858e-6; % Drag torque coeffecient (kg-m^2)

kdx = 0.16481; % Translational drag force coeffecient (kg/s)
kdy = 0.31892; % Translational drag force coeffecient (kg/s)
kdz = 1.1e-6; % Translational drag force coeffecient (kg/s) = [kdx;kdy;kdz];
drag_mat = [kdx 0 0; 0 kdy 0; 0 0 kdz];

inertia_xx = 0.05; % Moment of inertia about X axis (kg-m^2)
inertia_yy = 0.05; % Moment of inertia about Y axis (kg-m^2)
inertia_zz = 0.24; % Moment of inertia about Z axis (kg-m^2)
J = [inertia_xx; inertia_yy; inertia_zz]; % Body inertia [kg-m^2]

% Motor parameters
Kt = 1.3328e-5; % Thrust force coeffecient (kg-m)
Jr = 0.044; % Moment of Intertia of the rotor (kg-m^2)
Gamma = [Kt Kt Kt Kt;
         0 -L*Kt 0 L*Kt;
         L*Kt 0 -L*Kt 0;
         Kd -Kd Kd -Kd];
Gamma_inv = inv(Gamma);
mot_speeds = [1 -1 1 -1]';
max_mot_speeds = 925; % motors upper limit (rad/s)
min_mot_speeds = 0; % motors lower limit (can't spin in reverse)

U_max = [4*Kt*max_mot_speeds^2; 
         Kt*L*max_mot_speeds^2;
         Kt*L*max_mot_speeds^2;
         2*Kd*max_mot_speeds^2];
U_min = [                    0;
         Kt*L*max_mot_speeds^2;
         Kt*L*max_mot_speeds^2;
         2*Kd*max_mot_speeds^2];
%%

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