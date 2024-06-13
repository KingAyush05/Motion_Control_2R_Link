%% PID Practice
clc
close all
clear

%% Mass spring dampner system
s = tf('s');
P = 1/(s^2 + 10*s + 20);

% Proportional Control
Kp = 300;
C1 = pid(Kp);
T1 = feedback(C1*P,1);
t = 0:0.01:2;
step(T1,t)

% Proportional-Derivative Control
Kd = 10;
C2 = pid(Kp,0,Kd);
T2 = feedback(C2*P,1);
step(T2,t)

% Proportional-Integral Control
Kp = 30;
Ki = 70;
C3 =  pid(Kp,Ki);
T3 = feedback(C3*P,1);
step(T3,t);

% Proportional-Integral-Derivative Control
Kp = 350;
Ki = 300;
Kd = 50;
C4 = pid(Kp,Ki,Kd);
T4 = feedback(C4*P,1);
step(T4,t);



