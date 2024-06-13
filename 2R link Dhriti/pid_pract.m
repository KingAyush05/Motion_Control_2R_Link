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
g = 9.81;
I1 = 1;
I2 = 1;
theta10 = 0;
theta20 = 0;
omega10 = 0;
omega20 = 0;
t0 = 0;
T0 = 0.5*(I1 + m1*a1*a1 + m2*l1*l1)*omega10*omega10 + 0.5*(I2 + m2*a2*a2)*omega20*omega20 + m2*l1*a2*omega10*omega20*cos(theta20-theta10);
V0 = m1*g*a1*sin(theta10) + m2*g*(l1*sin(theta10)+a2*sin(theta20));
E0 = T0 + V0;
yd = [pi/4  0 pi/4 0];
Kp1 = 100;
Ki1 = 1;
Kd1 = 100;
Kp2 = 100;
Ki2 = 1;
Kd2 = 10;
K = [Kp1 Ki1 Kd1 Kp2 Ki2 Kd2];
T10 = Kp1*(yd(1)-theta10) + Kd1*(yd(2)-omega10);
T20 = Kp2*(yd(3)-theta20) + Kd2*(yd(4)-omega20);
y0 = [theta10; omega10; theta20; omega20; E0; T10; T20];
P = [m1 m2 l1 l2 a1 a2 g I1 I2];
tspan = [0 10];
%%
opts = odeset('RelTol',1e-6,'AbsTol',1e-6,'Mass',@(t,y) mass(t,y,P,K));
[t,y] = ode45(@(t,y) f(t,y,P,yd,K), tspan, y0,opts);
figure
subplot(2,3,1)
hold on
title('Theta vs time');
xlabel('time');
ylabel('theta');
plot(t,cos(t),'--b','DisplayName','theta1d');
plot(t,y(:,1),'-b','DisplayName','theta1');
plot(t,-sin(t),'--r','DisplayName','theta2d');
plot(t,y(:,3),'-r','DisplayName','theta2');
legend;
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
subplot(2,3,2)
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
subplot(2,3,3)
hold on
title('error');
plot(t,(E-y(:,5))/E,'r-');
%disp(W0 - W1)
hold off
%%errors for desired trajectory
subplot(2,3,5)
hold on
title("error in theta1");
plot(t,cos(t)-y(:,1),'b-');
hold off
subplot(2,3,6)
hold on
title("error in theta2");
plot(t,-sin(t)-y(:,3),'r-')
hold off

%% animate
x1 = l1*cos(y(:,1));
y1 = l1*sin(y(:,1));
x2 = x1 + l2*cos(y(:,3));
y2 = y1 + l2*sin(y(:,3));
subplot(2,3,4)
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
%% 
function M = mass(t,y,P,K)
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
Kp1 = K(1);
Kd1 = K(3);
Kp2 = K(4);
Kd2 = K(6);
% Mass matrix elements
M = zeros(7,7);
M(1,1) = 1;
M(2,2) = I1 +m1*a1*a1 + m2*l1*l1;
M(2,4) = m2*l1*a2*cos(y(1)-y(3));
M(3,3) = 1;
M(4,2) = m2*a2*l1*cos(y(3)-y(1));
M(4,4) = I2 + m2*a2*a2;
M(5,5) = 1;
M(6,1) = Kp1;
M(6,2) = Kd1;
M(6,6) = 1;
M(7,3) = Kp2;
M(7,4) = Kd2;
M(7,7) = 1;
end 
%% 
function dydt = f(t,y,P,yd, K)
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
theta1d = cos(t);
omega1d = -sin(t);
alpha1d = -cos(t);
theta2d = yd(3);
omega2d = yd(4);
alpha2d = 0;
Kp1 = K(1);
Ki1 = K(2);
Kd1 = K(3);
Kp2 = K(4);
Ki2 = K(5);
Kd2 = K(6);
% Equations to solve
dydt = [y(2)
        y(6) -  m1*g*a1*cos(y(1)) - m2*g*l1*cos(y(1)) - m2*l1*a2*y(4)*y(4)*sin(y(1)-y(3))
        y(4)
        y(7) - m2*g*a2*cos(y(3)) - m2*a2*l1*y(2)*y(2)*sin(y(3)-y(1))
        y(6)*y(2)+y(7)*y(4)
        Kp1*omega1d + Ki1*(theta1d - y(1)) + Kd1*alpha1d 
        Kp2*omega2d + Ki2*(theta2d - y(3))+ Kd2*alpha2d];
end
%Ki2*(theta2d - y(3))