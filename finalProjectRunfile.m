clear; clc; clf;%close all
format shortg
rng default
%%

parameterFile();

options = odeset('RelTol',1e-8,'AbsTol',1e-8);
[~,Y] = ode113(@(t,X) hillEOM(t,X,n),t,X0(:)',options);

% linear propagation of state and covariance using STM
A = zeros(6);
A(1:3,4:6) = eye(3);
A(4,1) = 3*n^2;A(4,5) = 2*n;
A(5,4) = -2*n;
A(6,3) = -n^2;
B = [zeros(3,3);eye(3)];
Phi = expm(A*(t(end)-t(1)));
Xf = Phi*X0;
Pf = Phi*P0*Phi';

%% plot nominal trajectory
cla
plot3(0,0,0,'*k','MarkerSize',14,'DisplayName','Target Vehicle');
grid on, grid minor, hold on
plot3(Y(:,1),Y(:,2),Y(:,3),'r-.','LineWidth',2,'DisplayName','Relative Orbit')
plot3(X0(1),X0(2),X0(3),'db','MarkerFaceColor','b','DisplayName','Starting Location')
plot3(Xf(1),Xf(2),Xf(3),'sb','MarkerFaceColor','b','DisplayName','Final Location')
[xe,ye,ze] = ellipsoid(X0(1),X0(2),X0(3),3*sqrt(P0(1,1)),3*sqrt(P0(2,2)),3*sqrt(P0(3,3)),30);
surf(xe,ye,ze,'FaceColor','r','EdgeColor','none','FaceAlpha',.2,'DisplayName','Initial Uncertainty')
xlabel('x (km)'),ylabel('y (km)'),zlabel('z (km)')
[xe,ye,ze] = ellipsoid(Xf(1),Xf(2),Xf(3),3*sqrt(Pf(1,1)),3*sqrt(Pf(2,2)),3*sqrt(Pf(3,3)),30);
surf(xe,ye,ze,'FaceColor','r','EdgeColor','none','FaceAlpha',.5,'DisplayName','Final Uncertainty')
title('Nominal Chaser Orbit Around Target')
legend
view(0,90)
saveas(gcf,'nominalOrbit.png')
%% plot bounding box
a = -pi : pi/2 : pi;ph = pi/4;
x = [cos(a+ph); cos(a+ph)]/cos(ph)*d(1);
y = [sin(a+ph); sin(a+ph)]/sin(ph)*d(2);
z = [-ones(size(a)); ones(size(a))]*d(3);
surf(x, y, z,'FaceColor','g','FaceAlpha',.2,'DisplayName','Constraint')
p1 = plot(alphaShape([x(:),y(:),z(:)]),'FaceColor','g','FaceAlpha',.2,'EdgeColor','none');
p1.Annotation.LegendInformation.IconDisplayStyle = 'off';
axis equal
saveas(gcf,'nominalOrbitWithBoxConstraint.png')
% [x,y] = meshgrid(...
% linspace(-.1,1,2),...
% linspace(-.1,1,2));
% z = d(1)*ones(size(x, 1)); % Generate z data
% surf(1.2*z, 1.2*x, 1.2*y,'EdgeColor','k','FaceColor','r','FaceAlpha',.5,'DisplayName','Constraint');
% ylim([-.1 1.2])
% saveas(gcf,'nominalOrbitWith1DConstraint.png')
%% find optimal control to satisfy constraint
% setup problem
options  = optimoptions('fmincon');
options.Display = "iter";
options.Algorithm = 'sqp';
options.ConstraintTolerance = 1e-10;
options.FunctionTolerance = 1e-10;
problem.objective = @(u) norm(u);
problem.x0 = zeros(3,1);
problem.nonlcon = @computeChanceConstraint;
problem.solver = 'fmincon';
problem.options = options;
u_opt = fmincon(problem);

% plot adjusted trajectory
Xf_adj = Phi*X0+Phi*B*u_opt *dt ;
plot3(Xf_adj(1),Xf_adj(2),Xf_adj(3),'gs','MarkerFaceColor','g','DisplayName','Corrected Final State')
[xe,ye,ze] = ellipsoid(Xf_adj(1),Xf_adj(2),Xf_adj(3),3*sqrt(Pf(1,1)),3*sqrt(Pf(2,2)),3*sqrt(Pf(3,3)),30);
surf(xe,ye,ze,'FaceColor','b','EdgeColor','none','FaceAlpha',.2,'DisplayName','Final Uncertainty')

saveas(gcf,'adjustedState1D.png')