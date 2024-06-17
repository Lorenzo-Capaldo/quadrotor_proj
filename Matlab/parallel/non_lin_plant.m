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
inertia_zz = 0.24; % Moment of inertia about
% Z axis (kg-m^2)
J = [inertia_xx; inertia_yy; inertia_zz]; % Body inertia [kg-m^2]

% Motor parameters
Kt = 1.3328e-5; % Thrust force coeffecient (kg-m)
Jr = 0.044; % Moment of Intertia of the rotor (kg-m^2)
Gamma = [Kt Kt Kt Kt;
         0 -L*Kt 0 L*Kt;
         L*Kt 0 -L*Kt 0;
         Kd -Kd Kd -Kd];
Gamma_inv = inv(Gamma);
max_mot_speed = 925; % motors upper limit (rad/s)
min_mot_speed = 0; % motors lower limit (can't spin in reverse)


%% Get commanded inputs
U1 = 0;
U2 = 0;
U3 = 0;
U4 = 0;

%% Get motor speeds
W = zeros(4,1);
W(1) = sqrt(U1/(4*Kt) + U3/(2*Kt*L) + U4/(4*Kd));
W(2) = sqrt(U1/(4*Kt) - U2/(2*Kt*L) - U4/(4*Kd));
W(3) = sqrt(U1/(4*Kt) - U3/(2*Kt*L) + U4/(4*Kd));
W(4) = sqrt(U1/(4*Kt) + U2/(2*Kt*L) - U4/(4*Kd));

for i = 1:4
    if W(i)>max_mot_speed
        W(i) = max_mot_speed;
    end
end

omega = W(1) - W(2) + W(3) - W(4);

%% UPDATE DYNAMICS
% Angle Rates
phi_dot = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
theta_dot = q*cos(phi) - r*sin(phi);
psi_dot = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);

% Angle Accelerations
p_dot = 1/Jx * (q*r*(Jy-Jz) - Jr*q*omega + U2);
q_dot = 1/Jy * (p*r*(Jz-Jx) - Jr*p*omega + U3);
r_dot = 1/Jz * (p*q*(Jx-Jy) + U4);

% Linear Acceleratios
x_ddot = 1/m * (-U1*(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))-Kdx*x_dot);
y_ddot = 1/m * (-U1*(cos(phi)*sin(theta)*sin(psi)-cos(psi)*sin(phi))-Kdy*y_dot);
z_ddot = -1/m*(U1*cos(phi)*cos(theta)-Kdz*z_dot)+g;

% Integrate Velocities
x_dot = x_ddot*Ts + x_dot;
y_dot = x_ddot*Ts + y_dot;
z_dot = x_ddot*Ts + z_dot;

% Integrate positions
x = x_dot*Ts + x;
y = y_dot*Ts + y;
z = z_dot*Ts + z;

% Integrate Orientation
phi = phi_dot*Ts + phi;
theta = theta_dot*Ts + theta;
psi = psi_dot*Ts + psi;

% Integrate Angvels
p = p_dot*Ts + p;
q = q_dot*Ts + q;
r = r_dot*Ts + r;