close all
clear all
clc
global Vt Vp;

k = 5/3;
Vt = 300;
Vp = Vt*k;


R_T0 = sqrt(10000^2 + 1000^2);
R_P0 = 1;

theta_P0 = 0;
theta_T0 = atan2(1,10);

alphaP0 = theta_P0;
alphaT0 = theta_T0;

V_rp0 = Vp * cos(alphaP0 - theta_P0);
V_theta_p0 = Vp * sin(alphaP0 - theta_P0);

V_rt0 = Vt * cos(alphaT0 - theta_T0);
V_theta_t0 = Vp * sin(alphaT0 - theta_T0);

xp0 = 1;
yp0 = 0;
xt0 = R_T0*cos(theta_T0);
yt0 = R_T0*sin(theta_T0);


tstep = 0.01;
tend = 1000;
tspan =  0:tstep:tend;
options = odeset('Events', @(t, y) event_terminal(t, y));

[t,y] = ode45(@LOSG_kinematics,tspan,[R_T0 R_P0 theta_T0 theta_P0 V_theta_t0 V_rt0 V_theta_p0 V_rp0 alphaP0 alphaT0 xt0 yt0 xp0 yp0],options);

R_t = y(:,1);
R_p = y(:,2);
theta_t = y(:,3);
theta_p = y(:,4);
Vtheta_t = y(:,5);
Vr_t = y(:,6);
Vtheta_p = y(:,7);
Vr_p = y(:,8);
alphaP = y(:,9);
alphaT = y(:,10);
xt = y(:,11);
yt = y(:,12);
xp = y(:,13);
yp = y(:,14);
ap =Vp.*Vtheta_p./R_p;



%% ---------------------------------------------------------------------------
    figure(1)
    hold on
    plot(xp,yp,'Color',[0 0.1 0]);
    hold on;
    plot(xt,yt,'Color','r');
    title('Target and Pursuer in actual space');
    [R_min, R_minIndex] = min(R_t - R_p);
    disp(R_min);
    text(double(xp(R_minIndex)), double(yp(R_minIndex) - 400), sprintf('R_{miss} = %6.2f m, t_{miss} %3.2f s',R_min,t(R_minIndex)), 'FontSize',10);
    xlabel('X position (m)');
    ylabel('Y position (m)');
    legend('Pursuer''s trajectory','Target''s trajectory');
    text(double(1000), double(400), sprintf('V_{T} = %3.0f m/s,V_{P} = %3.0f m/s,',Vt,Vp), 'FontSize',10);
    grid on;
    axis normal;
    hold off

    figure(2);
    hold on
    plot1 = subplot(2,2,1);
    plot(plot1, t, R_t - R_p);
    title('Relative distance V/s Time');
    xlabel('Time (s)');
    ylabel('Relative distance (m)');
    grid on;
    axis normal;
    hold off
    
    hold on
    plot2 = subplot(2,2,2);
    plot(plot2, t, theta_p);
    title('\theta of pursuer V/s Time');
    ylabel('\theta(t) (radian)');
    xlabel('Time (s)');
    grid on;
    axis normal;
    hold off
    
    hold on
    plot3 = subplot(2,2,3);
    plot(plot3,t,Vr_t - Vr_p);
    title('V_R V/s Time');
    ylabel('V_R(t) (radian)');
    xlabel('Time (s)');
    grid on;
    axis normal;
    hold off
    
    hold on
    plot4 = subplot(2,2,4);
    plot(plot4,t,Vtheta_p);
    title('V_\theta of pursuer V/s Time');
    ylabel('V_\theta(t) (radian)');
    xlabel('Time (s)');
    grid on;
    axis normal;
    hold off

    figure(3);
    plot(t,ap);
    title('Lateral acceleration (a_P) V/s Time');
    ylabel('a_P(t) (m/s^2)');
    xlabel('Time (s)');
    grid on;
    axis normal;
    hold off
% end
