function dydt = PP_kinematics( t,y )

global Vp 
global Vt
global omega_T

Ct = 1;

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

dydt(1) = Vt * cos(alphaT - theta) - Vp;
dydt(2) = Vtheta/R;
dydt(3) = -(1-Ct)*(Vr + Vp)*dydt(2);
dydt(4) = (1-Ct)*(Vtheta)*(dydt(2));

% dydt(4) = (dydt(2)-omega_T)*Vtheta;
% dydt(3) = -(Vr + Vp)*(dydt(2)-omega_T);

aP = Vp * dydt(2);
dydt(5) =aP/Vp;

%dydt(6) = omega_T; 
dydt(6) = Ct * dydt(2);

%% dynamics 
dydt(7) = Vt*cos(alphaT);
dydt(8) = Vt*sin(alphaT);
dydt(9) = Vp*cos(alphaP);
dydt(10) = Vp*sin(alphaP);

end

