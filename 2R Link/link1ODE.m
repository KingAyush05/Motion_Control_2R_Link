function xdot = link1ODE(t,x,g,l1,m1,a1,i1,torque1)
% Equations of motion for the simple pendullum.
% Here x = [Theta;Thetadot];
xdot = [x(2);-(g/L)*sin(x(1)) - (mu*g/L)*sign(x(2))];
end
