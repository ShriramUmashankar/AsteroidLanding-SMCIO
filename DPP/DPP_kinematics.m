function dydt = DPP_kinematics( t,y )

global Vp; global Vt; global omega_T; global del; global Ct

dydt = zeros(size(y));

R = y(1);
theta = y(2);

alphaP = y(3);
alphaT = y(4);

Vtheta = y(6);
Vr = y(5);

xt = y(7);
yt = y(8);

xp = y(9);
yp = y(10);


dydt(1) = Vt*cos(alphaT - theta) - Vp*cos(del);
dydt(2) = (Vt * sin(alphaT - theta) - Vp*sin(del))/R;
dydt(3) = dydt(2);
dydt(4) = Ct*dydt(2);

dydt(5) = (dydt(2) - dydt(4))*(R*dydt(2) + Vp*sin(del));
dydt(6) = (-dydt(2) + dydt(4))*(dydt(1) + Vp*cos(del));

%% dynamics 
dydt(7) = Vt*cos(alphaT);
dydt(8) = Vt*sin(alphaT);
dydt(9) = Vp*cos(alphaP);
dydt(10) = Vp*sin(alphaP);

end
