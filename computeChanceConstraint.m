function [c_ineq,c_eq] = computeChanceConstraint(u)

% get parameters for orbit
parameterFile();

A = zeros(6);
A(1:3,4:6) = eye(3);
A(4,1) = 3*n^2;A(4,5) = 2*n;
A(5,4) = -2*n;
A(6,3) = -n^2;
Phi = expm(A*dt);
B = [zeros(3,3);eye(3)];
xf = Phi*X0 + Phi*B*u*dt;% propagate dynamics linearly and add control

% propagate uncertainty linearly
Pf = Phi*P0*Phi';

% formulate chance constraint(s)
sd = sqrt(diag(Pf));

c_ineq(1) = xf(1)+3*sd(1) - d(1);
% c_ineq(2) = xf(2)+3*sd(2) - d(2);
% c_ineq(3) = xf(3)+3*sd(3) - d(3);
% c_ineq(4) = -xf(1)+3*sd(1) - d(1);
% c_ineq(5) = -xf(2)+3*sd(2) - d(2);
% c_ineq(6) = -xf(3)+3*sd(3) - d(3);

c_eq = [];
