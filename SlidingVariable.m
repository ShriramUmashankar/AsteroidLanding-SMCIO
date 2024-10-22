function [a0,se_dot] = SlidingVariable(e,a4,k_e,e_e)
    se = e;
    se_dot = -k_e*se - e_e*sign(se);
    a0 = -a4 + se_dot;
end

