clc; close all; clear;

utilities
del_m = 0;
m = m_0 + del_m;
r = r_0;
v = v_0;

position = r_0;
velocity = v_0;
acceleration = a;
time = 0;
iter = 1;
mass = m;
thrust = 0;
controller_effort = 0;
sliding_var = 0;


while R > 0.08   
    %loop
    [g] = GetGravity(r,v,w);
    [e, a1, a2, a3, a4, K] = HeadingErrorDynamics(r_fd,r,v,g);
    [a0,se_dot] = SlidingVariable(e,a4,k_e,e_e);
    [r,V_l,V_u] = VelocityProfile(r, r_0,v ,v_0, r_fd, h, b_x, b_y, b_z, c_x, c_y, c_z);
    [x] = ConvexOptim(V_l, V_u, a0, a1, a2, a3, v, g, h, m, T_max, a);
    a = x;
    a_x = x(1); a_y = x(2); a_z = x(3);
    T = sqrt(a_x^2 + a_y^2 + a_z^2)*m;
    del_m = -T*h/c;
    m = m + del_m;
    v = v + (a + g)*h ;
    R = norm(r_fd - r);

    position = cat(1,position,r);
    velocity = cat(1,velocity,v);
    acceleration = cat(1,acceleration,a);
    time = cat(1,time,iter*h);
    iter = iter + 1;
    mass = cat(1,mass,m);
    thrust = cat(1,thrust,T);
    controller_effort = cat(1,controller_effort,controller_effort(end,1)+norm(a)*h);
    sliding_var = cat(1,sliding_var,e);

end    

save("accl.mat","acceleration")
save("posn.mat","position");
save("vel.mat","velocity");
save("controller_effort.mat","controller_effort");
save("thrust.mat","thrust");
save("time.mat","time");
save("mass.mat","mass");
save("sliding_var.mat","sliding_var");

fprintf('Final mass is %s .\n', mass(end,1));
fprintf('Change in mass is %s .\n', mass(1,1) - mass(end,1));
fprintf('controller effort is %s .\n',controller_effort(end,1));
fprintf('Final time is %s (s) .\n',time(end,1));
disp("---------------------------------------------------")
fprintf('Final position is rx = %s(m)\n', position(end,1));
fprintf('Final position is ry = %s(m)\n', position(end,2));
fprintf('Final position is rz = %s(m)\n', position(end,3));
disp("---------------------------------------------------")
fprintf('Final velocity is vx = %s(m)\n', velocity(end,1));
fprintf('Final velocity is vy = %s(m)\n', velocity(end,2));
fprintf('Final velocity is vz = %s(m)\n', velocity(end,3));
disp("---------------------------------------------------")
fprintf("Mod velocity is %s .\n",norm(velocity(end,:)));
disp("---------------------------------------------------")
disp("Simulation Done, Mat files saved")
