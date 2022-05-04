function [Xdot] = hillEOM(t,X,n)
Xdot = zeros(size(X));

% velocity
Xdot(1:3) = X(4:6);

% acceleration
Xdot(4) = 2*n*X(5) + 3*n^2*X(1);
Xdot(5) = -2*n*X(4);
Xdot(6) = -n^2*X(3);