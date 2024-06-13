clc
close all
clear
%%
m1 = 1;
m2 = 1;
l1 = 1;
l2 = 1;
a1 = 0.5;
a2 = 0.5;
T1 = @(t)-90;
T2 = @(t)0;
g = 9.81;
I1 = 1;
I2 = 1;
theta10 =0;
theta20 = 0;
omega10 = 500;
omega20 = 0;
t0 = 0;
T0 = 0.5*(I1 + m1*a1*a1 + m2*l1*l1)*omega10*omega10 + 0.5*(I2 + m2*a2*a2)*omega20*omega20 + m2*l1*a2*omega10*omega20*cos(theta20-theta10);
V0 = m1*g*a1*sin(theta10) + m2*g*(l1*sin(theta10)+a2*sin(theta20));
E0 = T0 + V0;
Work0 = T1(t0)*theta10 + T2(t0)*theta20;
y0 = [theta10; omega10; theta20; omega20; E0];
P = [m1 m2 l1 l2 a1 a2 g I1 I2];
tspan = linspace(0, 10, 800);
%%
opts = odeset('RelTol',1e-9,'AbsTol',1e-9,'Mass',@(t,y) mass(t,y,P,T1,T2));
[t,y] = ode45(@(t,y) f(t,y,P,T1,T2), tspan, y0,opts);
figure
subplot(2,2,1)
hold on
title('Theta vs time');
xlabel('time');
ylabel('theta');
plot(t,mod(y(:,1),2*pi),'-b');
plot(t,mod(y(:,3),2*pi),'-r');
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
%W = zeros(length(t),1);
%Power = zeros(length(t),1);
%W1 = 0;
%for i = 1:length(t)
 %   W(i) = T1(t)*y(i,1) + T2(t)*y(i,3);
  %  Power(i) = T1*y(i,2) + T2*y(i,4);
   % W1 = y(i,5) + W1;
   %end
%W0 = cumtrapz(Power,t);
%for i = length(t):2
%    W0(i) = W(i) - W(i-1);
%end

%%
subplot(2,2,2)
hold on
title('Energy vs time');
xlabel('time');
ylabel('energy');
plot(t,E,'r-','DisplayName','energy');
%plot(t,W,'g-','DisplayName','Work by simple summation');
plot(t,y(:,5),'b--','DisplayName','work by ode45');
legend;
hold off
%% error
subplot(2,2,3)
hold on
title('error');
plot(t,(E-y(:,5))/E,'r-');
%disp(W0 - W1)
hold off
%% animate
x1 = l1*cos(y(:,1));
y1 = l1*sin(y(:,1));
x2 = x1 + l2*cos(y(:,3));
y2 = y1 + l2*sin(y(:,3));
subplot(2,2,4)
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
%% PID
Kp = 1;
Kd = 1;
Ki = 1;

%% 
function M = mass(t,y,P,T1,T2)
% Extract parameters
m1 = P(1);
m2 = P(2);
l1 = P(3);
l2 = P(4);
a1 = P(5);
a2 = P(6);
g = P(7);
I1 = P(8);
I2 = P(9);
% Mass matrix elements
M = zeros(5,5);
M(1,1) = 1;
M(2,2) = I1 +m1*a1*a1 + m2*l1*l1;
M(2,4) = m2*l1*a2*cos(y(1)-y(3));
M(3,3) = 1;
M(4,2) = m2*a2*l1*cos(y(3)-y(1));
M(4,4) = I2 + m2*a2*a2;
M(5,5) = 1;
end
%% 
function dydt = f(t,y,P,T1,T2)
% Extract parameters
m1 = P(1);
m2 = P(2);
l1 = P(3);
l2 = P(4);
a1 = P(5);
a2 = P(6);
g = P(7);
I1 = P(8);
I2 = P(9);

% Equations to solve
dydt = [y(2)
        T1(t) -  m1*g*a1*cos(y(1)) - m2*g*l1*cos(y(1)) - m2*l1*a2*y(4)*y(4)*sin(y(1)-y(3))
        y(4)
        T2(t) - m2*g*a2*cos(y(3)) - m2*a2*l1*y(2)*y(2)*sin(y(3)-y(1))
        T1(t)*y(2)+T2(t)*y(4)];
end