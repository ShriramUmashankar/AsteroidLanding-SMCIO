function[g] = GetGravity(r,v,w)
   
    dist = norm(r);
    la = 1000;
    lb = 500;
    lc = 250;
    
    volume = (4/3)*pi*la*lb*lc ;
    density = 3000;
    GM = volume*density*6.67432e-11;

    C20 = -0.1125;
    C22 = 0.0375;
    phi = atan2(r(3)/lc,sqrt((r(2)/lb)^2 + (r(1)/la)^2));
    lambda = atan2(r(2)/lb,r(1)/la);
    
    a_r = (-GM/dist^2) - ((3*GM*la^2)/(dist^4))*((C20/2)*(3*sin(phi)*sin(phi) -1) + 3*C22*cos(phi)*cos(phi)*cos(2*lambda));
    a_phi = (GM*la^2/dist^4)*((3*C20*sin(phi)*cos(phi)) - 6*C22*sin(phi)*cos(phi)*cos(2*lambda));
    a_lambda = (GM*la^2/dist^4)*(-6*C22*cos(phi)*sin(2*lambda));
    
    T_Matrix = [cos(phi)*cos(lambda), -sin(phi)*cos(lambda), -sin(lambda) ;...
        cos(phi)*sin(lambda), - sin(phi)*sin(lambda), cos(lambda);...
        sin(phi), cos(phi), 0];


    a_geo = transpose(T_Matrix*[a_r ; a_phi ; a_lambda]);

    a_var = -2*(cross(w,v)) -cross(w,cross(w,r));
    g = a_geo + a_var; 
end


