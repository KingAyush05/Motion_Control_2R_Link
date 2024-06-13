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


%% Initial Conditions
theta1 = 0; % (rad)
theta2 = 0; % (rad)
theta1dot = 0; % (rad/sec)
theta2dot = 0; % (red/sec)

theta1d = pi/4;
theta1dotd = 0;
theta2d = pi/2;
theta2dotd = 0;

tspan = linspace(0, 10, 830); % Time span over which we seek the solution
y0 = [theta1; theta1dot; theta2; theta2dot; 0; 0; 0];
yd = [theta1d; theta1dotd; theta2d; theta2dotd];

%% PID Controller
Kp1 = 700;
Kd1 = 300;
Ki1 = 200;

Kp2 = 450;
Kd2 = 250;
Ki2 = 1;

%% Setting Options and solving using ode45
opts = odeset('Mass',@(t,y) mass(t,y,l1,l2,m1,m2,a1,a2,i1,i2),'RelTol',1e-8,'AbsTol',1e-8);

[t,y] = ode45(@(t,y) ODE(t,y,g,l1,l2,m1,m2,a1,a2,i1,i2,yd,Kp1,Kd1,Kp2,Kd2,Ki1,Ki2),tspan,y0,opts);

%% Torque graphs

% gravitytorque1 = m1*g*a1*cos(yd(1)) + m2*g*(a2*cos(yd(3)) + l1*cos(yd(1))); % Torque on Link1 due to Gravity
% gravitytorque2 = m2*g*a2*cos(yd(3)); % Torque on Link2 due to Gravity

intetheta1 = y(:,6); % Integration of Thetad1 - Theta1
intetheta2 = y(:,7); % Integration of Thetad2 - Theta2

torque1 = Kp1*(yd(1)-y(:,1)) + Kd1*(yd(2)-y(:,2)) + Ki1*intetheta1;
torque2 = Kp1*(yd(3)-y(:,3)) + Kd1*(yd(4)-y(:,4)) + Ki2*intetheta2;
figure
subplot(2,3,4);
hold on
title('Torque vs Time');
plot(t,torque1);
plot(t,torque2);
legend("Torque1","Torque2");
hold off

%% Plots of Theta
subplot(2,3,1);
title('Theta vs Time');
hold on
plot(t,y(:,1));
plot(t,y(:,3));
plot(t,(yd(1))*ones(length(t)));
plot(t,(yd(3))*ones(length(t)));

xlabel('Time');
ylabel('Theta');
legend("Theta1","Theta2","Theta1 desired","Theta2 desired");
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

Power = torque1.*y(:,2) + torque2.*y(:,4);

W1 = cumtrapz(t,Power); % Work by integrating
W0 = y(:,5); %Work by ode45

subplot(2,3,3);
title("Work done and Energy");
hold on
plot(t,energy-energy(1),'r-');
plot(t,W0,'g--');
plot(t,W1,'b--');
legend("Energy of system","Work by ode45", "Work by integrating");
hold off 

%% Error in Work
% subplot(2,3,4)
% hold on
% title('Error in Work');
% plot(t,energy-energy(1)-y(:,5),'r-');
% hold off

%% Error in Theta1 and Theta2
theta1_error = (y(end,1) - yd(1))*(180/pi);
theta2_error = (y(end,3) - yd(3))*(180/pi);

fprintf("Error in theta1 = %f degrees\n", theta1_error);
fprintf("Error in theta2 = %f degrees\n", theta2_error);

subplot(2,3,5)
hold on
title('Erorr in Theta1');
plot(t,yd(1) - y(:,1));
plot(t,0*ones(length(t)));
hold off

subplot(2,3,6)
hold on
title('Error in Theta2');
plot(t,yd(3) - y(:,3),t,yd(3));
plot(t,0*ones(length(t)));
hold off

%% Animate motion

subplot(2,3,2);
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
function dydt = ODE(t,y,g,l1,l2,m1,m2,a1,a2,i1,i2,yd,Kp1,Kd1,Kp2,Kd2,Ki1,Ki2)
% Equations of motion for the simple pendullum.
% Here y = [theta1 theta1dot theta2 theta2dot work intetheta1 intetheta2];
% yd are desired values 
% PD Controller


% gravityT1 = m1*g*a1*cos(yd(1)) + m2*g*a2*cos(yd(3));
% gravityT2 = m2*g*a2*cos(yd(3));

inttheta1 = y(6);
inttheta2 = y(7);

T1 = Kp1*(yd(1)-y(1)) + Kd1*(yd(2)-y(2)) + Ki1*inttheta1;
T2 = Kp2*(yd(3)-y(3)) + Kd2*(yd(4)-y(4)) + Ki2*inttheta2;

dydt = [y(2)
        T1 + m2*l1*a2*y(4)*y(4)*sin(y(3)-y(1)) - g*cos(y(1))*(m1*a1 + m2*l1)
        y(4)
        T2 - m2*l1*a2*y(2)^2*sin(y(3)-y(1)) - m2*g*a2*cos(y(3))
        T1*y(2) + T2*y(4)
        yd(1) - y(1)
        yd(3) - y(3)];
end

%% Mass Matrix Function
function M = mass(t,y,l1,l2,m1,m2,a1,a2,i1,i2)
% Mass Matrix elements
M = zeros(7,7);
M(1,1) = 1;
M(2,2) = m1*a1^2 + m2*l1^2 + i1;
M(2,4) = m2*l1*a2*cos(y(3)-y(1));
M(3,3) = 1;
M(4,2) = m2*l1*a2*cos(y(3)-y(1));
M(4,4) = m2*a2^2 + i2;
M(5,1) = 0;
M(5,3) = 0;
M(5,5) = 1;
M(6,6) = 1;
M(7,7) = 1;
end









