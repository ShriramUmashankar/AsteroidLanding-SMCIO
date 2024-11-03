clc; close all; clear
%% Define all constants here

a = [0, 0, 0];
T_max = 80;
m_0 = 1400; 
c = 2206; % T = -m_dot * c
w = [0, 0, 2.18 * 10^(-4)]; %Omega value

% 1 + b + c = 10
b_x = 9; b_y = 8; b_z = 5;
c_x = 0; c_y = 1; c_z = 4;

h=0.05 ; % Time step

%% Landing Scenario 1
r_0 = [1389.37, -60.78, -30.5];
r_fd = [1000, 0, 0];

v_0 = [-2, -1.25, -0.5];
v_fd = [0,0,0]; % Final velocity
t_f = 608.35;

r_wp = [1150, -75, -50];
v_wp = [-1, 0.65, 0.4];
t_wp = 450;


%%
R = norm(r_fd - r_0);