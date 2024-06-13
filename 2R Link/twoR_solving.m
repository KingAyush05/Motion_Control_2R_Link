%% 2R Link
clc
close all
clear

%% Input Parameters:
g = 9.81; % (m/s^2)
l1 = 3; % (m)
l2 = 4; % (m)
m1 = 1; % (kg)
m2 = 2; % (kg)
a1 = 1.4; % (m)
a2 = 2.3; % (m)
i1 = 10; % (kgm^2)
i2 = 30; % (kgm^2)
torque1 = 0; % (Nm)
torque2 = 0; % (Nm)

%% Initial Conditions
theta1 = 0; % (rad)
theta2 = 0; % (rad)
theta1dot = 0; % (rad/sec)
theta2dot = 0; % (red/sec)

tspan = linspace(0, 10, 829); % Time span over which we seek the solution
y0 = [theta1; theta1dot; theta2; theta2dot; 0];
%% Setting Options and solving using ode45
opts = odeset('Mass',@(t,y) mass(t,y,l1,l2,m1,m2,a1,a2,i1,i2,torque1,torque2),'RelTol',1e-8,'AbsTol',1e-8);

[t,y] = ode45(@(t,y) ODE(t,y,g,l1,l2,m1,m2,a1,a2,i1,i2,torque1,torque2),tspan,y0,opts);

%% Plots
figure
subplot(2,2,1);
title('Theta vs Time');
hold on
plot(t,y(:,1));
plot(t,y(:,3));
xlabel('Time');
ylabel('Theta');
legend("Theta1","Theta2");
hold off 
%% Plot of motion

% figure
% subplot(1,3,1);
% title('Motion of a 2R linkage, Solved by ODE45');

% hold on
theta1 = y(:,1);
theta2 = y(:,3);
xvals = [l1*cos(theta1) l1*cos(theta1) + l2*cos(theta2)];
yvals = [l1*sin(theta1) l1*sin(theta1) + l2*sin(theta2)];
% plot(xvals(:,1),yvals(:,1));
% plot(xvals(:,2),yvals(:,2));
% legend("Link1","Link2");
 
% hold off
%% Total Energy of System
K = 0.5*(i1 + m1*a1*a1 + m2*l1*l1)*y(:,2).*y(:,2) + 0.5*(i2 + m2*a2*a2)*y(:,4).*y(:,4) + m2*l1*a2*y(:,2).*y(:,4).*cos(y(:,3)-y(:,1));
P = m1*g*a1*sin(y(:,1)) + m2*g*(l1*sin(y(:,1))+a2*sin(y(:,3)));
energy = K + P;

%% Work done by Torque
omega1 = y(:,2);
omega2 = y(:,4);

Power = torque1*y(:,2) + torque2*y(:,4);

W1 = cumtrapz(t,Power); % Work by integrating
W0 = y(:,5); %Work by ode45

subplot(2,2,3);
title("Work done and Energy");
hold on
plot(t,energy-energy(1),'r-');
plot(t,W0,'g--');
plot(t,W1,'b--');
legend("Energy of system","Work by ode45", "Work by integrating");
hold off 

%% Error in Work
subplot(2,2,4)
hold on
title('Error in Work');
plot(t,energy-energy(1)-y(:,5),'r-');
hold off

%% Animate motion

subplot(2,2,2);
hold on
axis equal;
axis([-(l1+l2+1), (l1+l2+1), -(l1+l2+1), (l1+l2+1)]);
grid on;
link1 = plot([0,xvals(1,1)], [0,yvals(1,1)], 'g', 'LineWidth',2);
link2 = plot([0,xvals(1,2)], [0,yvals(1,2)], 'r', 'LineWidth',2);

joint1 = plot(xvals(1,1), yvals(1,1), 'go', 'MarkerSize', 10);
joint2 = plot(xvals(1,2), yvals(1,2), 'ro', 'MarkerSize', 10);

for i = 2:length(t)
    set(link1, 'XData', [0 xvals(i,1)], 'YData', [0 yvals(i,1)]);
    set(link2, 'XData', [xvals(i,1) xvals(i,2)], 'YData', [yvals(i,1) yvals(i,2)]);
    set(joint1, 'XData', xvals(i,1), 'YData', yvals(i,1));
    set(joint2, 'XData', xvals(i,2), 'YData', yvals(i,2));
    drawnow;
end

%% Defining functions for ODE
function dydt = ODE(t,y,g,l1,l2,m1,m2,a1,a2,i1,i2,torque1,torque2)
% Equations of motion for the simple pendullum.
% Here y = [theta1 theta1dot theta2 theta2dot work];
T1 = torque1;
T2 = torque2;
dydt = [y(2)
        T1 + m2*l1*a2*y(4)*y(4)*sin(y(3)-y(1)) - g*cos(y(1))*(m1*a1 + m2*l1)
        y(4)
        T2 - m2*l1*a2*y(2)^2*sin(y(3)-y(1)) - m2*g*a2*cos(y(3))
        0];
end

%% Mass Matrix Function
function M = mass(t,y,l1,l2,m1,m2,a1,a2,i1,i2,torque1,torque2)
% Mass Matrix elements
T1 = torque1;
T2 = torque2;
M = zeros(5,5);
M(1,1) = 1;
M(2,2) = m1*a1^2 + m2*l1^2 + i1;
M(2,4) = m2*l1*a2*cos(y(3)-y(1));
M(3,3) = 1;
M(4,2) = m2*l1*a2*cos(y(3)-y(1));
M(4,4) = m2*a2^2 + i2;
M(5,1) = -T1;
M(5,3) = -T2;
M(5,5) = 1;
end









