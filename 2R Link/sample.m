clc
close all
clear
%%
m1 = 1;
m2 = 2;
l1 = 3;
l2 = 4;
a1 = 1.4;
a2 = 2.3;
T1 = 5;
T2 = 0;
g = 9.81;
I1 = 10;
I2 = 30;
y0 = [0; 0; 0; 0; 0];
P = [m1 m2 l1 l2 a1 a2 T1 T2 g I1 I2];
tspan = linspace(0, 10, 829);
%%
opts = odeset('RelTol',1e-8,'AbsTol',1e-8,'Mass',@(t,y) mass(t,y,P));
[t,y] = ode45(@(t,y) f(t,y,P), tspan, y0,opts);
figure
hold on
title('Theta vs time');
xlabel('time');
ylabel('theta');
plot(t,y(:,1),'-r');
plot(t,y(:,3),'-b');
legend('theta 1','theta 2')
hold off
%%  energy
E = zeros(length(t),1);
for i = 1: length(t)
    T = 0.5*(I1 + m1*a1*a1 + m2*l1*l1)*y(i,2)*y(i,2) + 0.5*(I2 + m2*a2*a2)*y(i,4)*y(i,4) + m2*l1*a2*y(i,2)*y(i,4)*cos(y(i,3)-y(i,1));
    V = m1*g*a1*sin(y(i,1)) + m2*g*(l1*sin(y(i,1))+a2*sin(y(i,3)));
    E(i) = T + V;
end
%% power and work
W = zeros(length(t),1);
Power = zeros(length(t),1);
W1 = 0;
for i = 1:length(t)
    W(i) = T1*y(i,1) + T2*y(i,3);
    Power(i) = T1*y(i,2) + T2*y(i,4);
    W1 = y(i,5) + W1;
end
W0 = cumtrapz(Power,t);
for i = length(t):2
    W0(i) = W(i) - W(i-1);
end

%%
figure
hold on
title('Energy vs time');
xlabel('time');
ylabel('energy');
plot(t,E-E(1),'r-','DisplayName','energy');
plot(t,W0,'g-','DisplayName','Work by trapz');
plot(t,y(:,5),'b--','DisplayName','work by ode45');
legend;
hold off
%% error
figure
hold on
title('error');
plot(t,E-E(1)-y(:,5),'r-');
%disp(W0 - W1)
hold off
%% animate
x1 = l1*cos(y(:,1));
y1 = l1*sin(y(:,1));
x2 = x1 + l2*cos(y(:,3));
y2 = y1 + l2*sin(y(:,3));
figure;
hold on;
axis equal;
axis([-(l1+l2+1), (l1+l2+1), -(l1+l2+1), (l1+l2+1)]);
grid on;
link1 = plot([0, x1(1)], [0, y1(1)], 'b', 'LineWidth', 2);
link2 = plot([x1(1), x2(1)], [y1(1), y2(1)], 'r', 'LineWidth', 2);

joint1 = plot(x1(1), y1(1), 'bo', 'MarkerSize', 10);
joint2 = plot(x2(1), y2(1), 'ro', 'MarkerSize', 10);

% Animation loop
for i = 2:length(t)
    set(link1, 'XData', [0 x1(i)], 'YData', [0 y1(i)]);
    set(link2, 'XData', [x1(i) x2(i)], 'YData', [y1(i) y2(i)]);
    set(joint1, 'XData', x1(i), 'YData', y1(i));
    set(joint2, 'XData', x2(i), 'YData', y2(i));
    drawnow;
end

%for i = 1:length(t)
 %   set(link1, 'XData', [0, x2(i)], 'YData', [0, y2(i)]);
  %  set(link2, 'XData', [x2(i), x3(i)], 'YData', [y2(i), y3(i)]);
   % set(link3, 'XData', [x3(i), x4(i)], 'YData', [y3(i), y4(i)]);
    %set(link4, 'XData', [x4(i), 0], 'YData', [y4(i), 0]);
    %pause(0.05);
%end
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
M = zeros(5,5);
M(1,1) = 1;
M(2,2) = I1 +m1*a1*a1 + m2*l1*l1;
M(2,4) = m2*l1*a2*cos(y(1)-y(3));
M(3,3) = 1;
M(4,2) = m2*a2*l1*cos(y(3)-y(1));
M(4,4) = I2 + m2*a2*a2;
M(5,1) = -T1;
M(5,3) = -T2;
M(5,5) = 1;
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
        T2 - m2*g*a2*cos(y(3)) - m2*a2*l1*y(2)*y(2)*sin(y(3)-y(1))
        0];
end