function [r,V_l,V_u] = VelocityProfile(r, r_0,v ,v_0, r_fd, h, b_x, b_y, b_z, c_x, c_y, c_z)


    v_x0 = v_0(1); v_y0 = v_0(2); v_z0 = v_0(3);
    v_x = v(1); v_y = v(2); v_z = v(3);

    r_x0 = r_0(1); r_y0 = r_0(2); r_z0 = r_0(3);
    r_x = r(1); r_y = r(2); r_z = r(3);
    r_xfd = r_fd(1); r_yfd = r_fd(2); r_zfd = r_fd(3);

    % Position in next time step
    r_xn = r_x + v_x*h;
    r_yn = r_y + v_y*h;
    r_zn = r_z + v_z*h;

    %Calculating zeta values
    zeta_x = (r_xn - r_xfd)/(r_x0 - r_xfd);
    zeta_y = (r_yn - r_yfd)/(r_y0 - r_yfd);
    zeta_z = (r_zn - r_zfd)/(r_z0 - r_zfd);
    
    %Max velocity bound for next time step
    V_xm = abs(v_x0)*log10(1 + b_x*zeta_x + c_x*(zeta_x)^2);
    V_ym = abs(v_y0)*log10(1 + b_y*zeta_y + c_y*(zeta_y)^2);
    V_zm = abs(v_z0)*log10(1 + b_z*zeta_z + c_z*(zeta_z)^2);

    V_m = [V_xm , V_ym, V_zm];
    V_l = [0 0 0];
    V_u = [0 0 0];
    for i = 1:3
        if sign(v_0(i)) == sign(r_fd(i) - r_0(i))
            if sign(v_0(i)) == 1
                V_l(i) = 0.8* V_m(i);
                V_u(i) = V_m(i);
            end

            if sign(v_0(i)) == -1
                V_l(i) = -1* V_m(i);
                V_u(i) = -0.8*V_m(i);
            end
        else
            V_l(i) = -1* V_m(i);
            V_u(i) = 1*V_m(i);
        end    
    end      
    r = [r_xn, r_yn, r_zn];
end

