clc; close all; clear;

utilities

del_m = 0;
m = m_0 + del_m;

r = r_0;
[g] = GetGravity(r);
v = v_0;
[e, a1, a2, a3, a4, K] = HeadingErrorDynamics(r_fd,r,v,g);
[a0,se_dot] = SlidingVariable(e,a4,k_e,e_e);
[r,V_l,V_u] = VelocityProfile(r, r_0,v ,v_0, r_fd, h, b_x, b_y, b_z, c_x, c_y, c_z);
[x] = ConvexOptim(V_l, V_u, a0, a1, a2, a3, v, g, h, m, T_max, a);
