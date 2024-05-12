clear all
clc
close all

load("../../../params.mat")
%%
dPOSG = [0; 0; 1];
PSIdes = 0; 
%%
% Input limits
U1max = 4*Kt*max_mot_speeds(1)^2;
U1min = 0;
U2max = Kt*L*max_mot_speeds(1);
U2min = -U2max;
U3max = Kt*L*max_mot_speeds(1);
U3min = -U3max;
U4max = Kq*2*max_mot_speeds(1);
U4min = -U4max;
% Position Controller
X_KP = .35; X_KI = .25; X_KD = -.35;
Y_KP = .35; Y_KI = .25; Y_KD = -.35;
Z_KP = -20; Z_KI = -6; Z_KD = -10;
X_KI_lim = 10; Y_KI_lim = .25; Z_KI_lim = .25;
% Attitude Controller
phi_KP = 4.5; phi_KI = 0; phi_KD = 0;
theta_KP = 4.5; theta_KI = 0; theta_KD = 0;
psi_KP = 10; psi_KI = 0; psi_KD = 0;
phi_max = pi/4; theta_max = pi/4;
phi_KI_lim = 2*(2*pi/360); theta_KI_lim = 2*(2*pi/360); psi_KI_lim = 8*(2*pi/360);
% Angvel Controller
p_KP = 2.7; p_KI = 1; p_KD = -.01;
q_KP = 2.7; q_KI = 1; q_KD = -.01;
r_KP = 2.7; r_KI = 1; r_KD = -.01;
p_max = 50*(2*pi/360); q_max = 50*(2*pi/360); r_max = 50*(2*pi/360);
p_KI_lim = 10*(2*pi/360); q_KI_lim = 10*(2*pi/360); r_KI_lim = 10*(2*pi/360); 