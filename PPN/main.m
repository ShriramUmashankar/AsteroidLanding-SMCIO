close all
clear all
clc
global Vt Vp N K Phi0;

k = 10/7;
Vt = 400;
Vp = Vt*k;

N = 4;
K = N - 1;

R0 = 3000;

theta0 = 30 *pi/180;
alphaP0 = 50 *pi/180;
alphaT0 = 0;
Phi0 = alphaP0 - N*theta0;

Vr0 = Vt * cos(alphaT0 - theta0) - Vp * cos(alphaP0 - theta0);
disp(Vr0);
Vtheta0 = Vt * sin(alphaT0 - theta0) - Vp * sin(alphaP0 - theta0);
disp(Vtheta0);
xp0 = 0;
yp0 = 0;
xt0 = R0*cos(theta0);
yt0 = R0*sin(theta0);


tstep = 0.01;
tend = 100;
tspan =  0:tstep:tend;
options = odeset('Events', @(t, y) event_terminal(t, y));

[t,y] = ode45(@PPN_kinematics,tspan,[R0 theta0 Vtheta0 Vr0 alphaP0 alphaT0 xt0 yt0 xp0 yp0],options);
R = y(:,1);
theta = y(:,2);
Vtheta = y(:,3);
Vr = y(:,4);
alphaP = y(:,5);
alphaT = y(:,6);
xt = y(:,7);
yt = y(:,8);
xp = y(:,9);
yp = y(:,10);
ap =Vp.*Vtheta./R;



%% ---------------------------------------------------------------------------
    figure(1)
    hold on
    plot(xp,yp,'Color',[0 0.1 0]);
    hold on;
    plot(xt,yt,'Color','r');
    title('Target and Pursuer in actual space');
    [R_min, R_minIndex] = min(R);
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
    plot(plot1, t, R);
    title('Relative distance V/s Time');
    xlabel('Time (s)');
    ylabel('Relative distance (m)');
    grid on;
    axis normal;
    hold off
    
    hold on
    plot2 = subplot(2,2,2);
    plot(plot2, t, theta);
    title('\theta V/s Time');
    ylabel('\theta(t) (radian)');
    xlabel('Time (s)');
    grid on;
    axis normal;
    hold off
    
    hold on
    plot3 = subplot(2,2,3);
    plot(plot3,t,Vr);
    title('V_R V/s Time');
    ylabel('V_R(t) (radian)');
    xlabel('Time (s)');
    grid on;
    axis normal;
    hold off
    
    hold on
    plot4 = subplot(2,2,4);
    plot(plot4,t,Vtheta);
    title('V_\theta V/s Time');
    ylabel('V_\theta(t) (radian)');
    xlabel('Time (s)');
    grid on;
    axis normal;
    hold off

    figure(3);
    hold on
    plot1 = subplot(1,2,1);
    plot(plot1,Vtheta,Vr);
    title('Trajectory in V_\theta V_R plane');
    xlabel('V_\theta (m/s)');
    ylabel('V_R (m/s)');
    %axis([-400 400 -400 400]);
    grid on;
    pbaspect([1 1 1])
    hold off
    %axis normal;
    
    hold on
    plot2 = subplot(1,2,2);
    plot(plot2,t,ap);
    title('Lateral acceleration (a_P) V/s Time');
    ylabel('a_P(t) (m/s^2)');
    xlabel('Time (s)');
    grid on;
    axis normal;
    hold off
% end
