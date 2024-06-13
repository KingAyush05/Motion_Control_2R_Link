clc
close all
clear
%%
P = [5 4 2 3 0.5 0.5 0 0 9.81 1.76 1.99];
m1 = P(1);
m2 = P(2);
l1 = P(3);
l2 = P(4);
a1 = P(5);
a2 = P(6);
T1 = P(7);
T2 = P(8);
g = P(9);
I1 = P(10);
I2 = P(11);
y0 = [0; 0; 0; 0];
tspan = linspace(0,50,1000);
%%
opts = odeset('Mass',@(t,y) mass(t,y,P));
[t,y] = ode45(@(t,y) f(t,y,P), tspan, y0,opts);
%plot(t,y(:,1),'-o',t,y(:,3),'-.');
disp(length(t));
E = zeros(length(t));
figure
title('Motion of a 2R linkage, Solved by ODE45');

axis([0 22 0 25])
hold on
for j = 1:length(t)
   theta1 = y(j,1);
   time = t(j);
   theta2 = y(j,3);
   xvals = [l1*cos(theta1) l1*cos(theta1) + l2*cos(theta2)];
   yvals = [l1*sin(theta1) l1*sin(theta1) + l2*sin(theta2)];
   plot(xvals(1),yvals(1),"ro",xvals(2),yvals(2),"go");
end
hold off
figure
hold on
for i = 1: length(t)
E(i) = 0.5*(I1 +m1*a1*a1)*y(i,2)*y(i,2) + 0.5*I2*y(i,4)*y(i,4) +0.5*m2*(a2*a2*y(i,4)*y(i,4) +l1*l1*y(i,2)*y(i,2) + 2*l1*a2*y(i,2)*y(i,4)*cos(y(i,3)-y(i,1))) + m1*g*a1*sin(y(i,1)) + m2*g*(l1*sin(y(i,1)) + a2*sin(y(i,3))); 
plot(t(i),E(i),'ro');
end
hold off
%% 
function M = mass(t,y,P)
% Extract parameters
m1 = P(1);
m2 = P(2);
l1 = P(3);
l2 = P(4);
a1 = P(5);
a2 = P(6);
T1 = P(7);
T2 = P(8);
g = P(9);
I1 = P(10);
I2 = P(11);
% Mass matrix elements
M = zeros(4,4);
M(1,1) = 1;
M(2,2) = I1 +m1*a1*a1 + m2*l1*l1;
M(2,4) = m2*l1*a2*cos(y(1)-y(3));
M(3,3) = 1;
M(4,2) = m2*a2*l1*cos(y(3)-y(1));
M(4,4) = I2 + m2*a2*a2;
end
%% 
function dydt = f(t,y,P)
% Extract parameters
m1 = P(1);
m2 = P(2);
l1 = P(3);
l2 = P(4);
a1 = P(5);
a2 = P(6);
T1 = P(7);
T2 = P(8);
g = P(9);
I1 = P(10);
I2 = P(11);

% Equations to solve
dydt = [y(2)
        T1 -  m1*g*a1*cos(y(1)) - m2*g*l1*cos(y(1)) - m2*l1*a2*y(4)*y(4)*sin(y(1)-y(3))
        y(4)
        T2 - m2*g*a2*cos(y(3)) - m2*a2*l1*y(2)*y(2)*sin(y(3)-y(1))];
end


%A = 1;
%B = 2;
%tspan = [0 5];
%y0 = [0 0.01];
%[t,y] = ode45(@(t,y) odefn(t,y,A,B), tspan, y0);
%plot(t,y(:,1),'-o',t,y(:,2),'-.');
%function dydt = odefn(t,y,A,B)
%  dydt = zeros(2,1);
%  dydt(1) = y(2);
%  dydt(2) = (A/B)*t.*y(1);
%end
