clc; close all; clear
%% Define all constants here

a = [0, 0, 0];
T_max = 80;
m_0 = 1400; 
c = 2206; % T = -m_dot * c
w = [0, 0, 2.18 * 10^(-4)]; %Omega value
v_fd = [0,0,0]; % Final velocity
e_e = 0.0001; % epsilon, sliding mode control parameter
k_e = 0.006; % exponential reaching law parameter

% 1 + b + c = 10
b_x = 9; b_y = 8; b_z = 5;
c_x = 0; c_y = 1; c_z = 4;

h=0.05 ; % Time step

%% Landing Scenario 4
r_0 = [121.55, -5.32, 348.67];
r_fd = [0, 0, 250];
v_0 = [-1, -0.95, -1.35];

%%
R = norm(r_fd - r_0);