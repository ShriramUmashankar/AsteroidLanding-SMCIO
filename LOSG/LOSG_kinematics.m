function dydt = LOSG_kinematics( t,y )

global Vp 
global Vt

Km = 10;

dydt = zeros(size(y));

R_t = y(1);
R_p = y(2);

theta_t = y(3);
theta_p = y(4);

Vtheta_t = y(5);
Vr_t = y(6);

Vtheta_p = y(7);
Vr_p = y(8);

alphaP = y(9);
alphaT = y(10);

xt = y(11);
yt = y(12);
xp = y(13);
yp = y(14);

dydt(1) = Vt * cos(alphaT - theta_t);
dydt(2) = Vp * cos(alphaP - theta_p);

dydt(3) = Vt*sin(alphaT - theta_t)/R_t ;
dydt(4) = Vp*sin(alphaP - theta_p)/R_p ;

at = 3*9.8;
dydt(5) = Vt*cos(alphaT - theta_t)*(at/Vt - dydt(3));
dydt(6) = -Vt*sin(alphaT - theta_t)*(at/Vt - dydt(3));

ap = Km*R_p*(theta_t - theta_p);
dydt(7) = Vp*cos(alphaP - theta_p)*(ap/Vp - dydt(4));
dydt(8) = -Vp*sin(alphaP - theta_p)*(ap/Vp - dydt(4));

dydt(9) = ap/Vp;
dydt(10) = at/Vt;

%% dynamics 
dydt(11) = Vt*cos(alphaT);
dydt(12) = Vt*sin(alphaT);
dydt(13) = Vp*cos(alphaP);
dydt(14) = Vp*sin(alphaP);

end

