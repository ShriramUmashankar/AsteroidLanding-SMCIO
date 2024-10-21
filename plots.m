clear; close all; clc;

load accl.mat
load posn.mat
load vel.mat
load time.mat
load controller_effort.mat
load mass.mat
load thrust.mat


subplot(3,2,1);
plot(time(2:end), acceleration(2:end,1),"LineWidth",1,"DisplayName",'a_x')
hold on
plot(time(2:end), acceleration(2:end,2),"LineWidth",1,"DisplayName",'a_y')
plot(time(2:end), acceleration(2:end,3),"LineWidth",1,"DisplayName",'a_z')
xlabel('Time')
xlim([0 time(end,1)]);
ylabel('Acceleration')
title('Acceleration vs Time')
legend()
grid("on")

subplot(3,2,2);
plot(time, velocity(:,1),"LineWidth",1,"DisplayName",'v_x')
hold on
plot(time, velocity(:,2),"LineWidth",1,"DisplayName",'v_y')
plot(time, velocity(:,3),"LineWidth",1,"DisplayName",'v_z')
xlabel('Time')
xlim([0 time(end,1)]);
ylabel('Velocity')
title('Velocity vs Time')
legend()
grid("on")

subplot(3,2,3);
plot(time, position(:,1),"LineWidth",1,"DisplayName",'x')
hold on
plot(time, position(:,2),"LineWidth",1,"DisplayName",'y')
plot(time, position(:,3),"LineWidth",1,"DisplayName",'z')
xlabel('Time')
xlim([0 time(end,1)]);
ylabel('Position')
title('Position vs Time')
legend()
grid("on")

subplot(3,2,4);
plot(time, mass(:,1),"LineWidth",1,"DisplayName",'mass')
xlabel('Time')
xlim([0 time(end,1)]);
ylabel('Mass')
title('Mass vs Time')
legend()
grid("on")

subplot(3,2,5);
plot(time(2:end), thrust(2:end,1),"LineWidth",1,"DisplayName",'Thrust')
xlabel('Time')
xlim([0 time(end,1)]);
ylabel('Thrust')
title('Thrust vs Time')
legend()
grid("on")

subplot(3,2,6);
plot(time, controller_effort(:,1),"LineWidth",1,"DisplayName",'effort')
xlabel('Time')
xlim([0 time(end,1)]);
ylabel('Effort')
title('Effort vs Time')
legend()
grid("on")
