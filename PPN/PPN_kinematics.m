function dydt = PP_kinematics( t,y )

global Vp 
global Vt
global N
global K
global Phi0

at = -30;

dydt = zeros(size(y));

R = y(1);

theta = y(2);
Vtheta = y(3);

Vr = y(4);

alphaP = y(5);
alphaT = y(6);

xt = y(7);
yt = y(8);
xp = y(9);
yp = y(10);

dydt(1) = Vt * cos(alphaT - theta) - Vp*cos(alphaP - theta);
dydt(2) = Vtheta/R;

dydt(3) = Vt*cos(alphaT - theta)*(at/Vt - dydt(2)) - Vp*cos(alphaP - theta)*(K*dydt(2));
dydt(4) = -Vt*sin(alphaT - theta)*(at/Vt - dydt(2)) + Vp*sin(alphaP - theta)*(K*dydt(2));

dydt(5) = N*dydt(2);

%AlphaT_dot = Ct * theta_dot
dydt(6) = at/Vt;

%% dynamics 
dydt(7) = Vt*cos(alphaT);
dydt(8) = Vt*sin(alphaT);
dydt(9) = Vp*cos(alphaP);
dydt(10) = Vp*sin(alphaP);

end

