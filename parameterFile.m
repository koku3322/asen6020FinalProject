
t = 0:100;dt = t(end)-t(1);

rTarget = 6968.1363;%km
muEarth = 398600.5;rEarth = 6378;

% mean motion
n = sqrt(muEarth/rTarget^3);

% initial conditions
P0 = diag([.05,.01,.001,0,0,0].^2);
X0 = [0.01,0.01,0.01,0.001,0.01,0]';

% chance constraints' feasible regions
d = [.1;.1;.1];
