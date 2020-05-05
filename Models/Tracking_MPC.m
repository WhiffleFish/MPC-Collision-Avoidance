nx = 6; % [x Vx y Vy Phi Phidot]
ny = 1; % Need Output Fcn to determine which state values are measured
nu = 1;
nlobj = nlmpc(nx,ny,nu);

Ts = 0.1;
p = 61; % Run simulation for p time steps
c = 60; % Calculate and employ Control actions for c time steps
nlobj.Ts = Ts;
nlobj.PredictionHorizon = p;
nlobj.ControlHorizon = c;

nlobj.Model.StateFcn = "car";
nlobj.Model.OutputFcn = @(x,u) x(3); % Only need Ypos

nlobj.MV.Min = deg2rad(-30);
nlobj.MV.Max = deg2rad(30);
nlobj.MV.RateMax = deg2rad(100*Ts);

x0 = [0 20 0 0 0 0];
u0 = 0;

% Set Reference
yref = 3;

[~,~,info] = nlmpcmove(nlobj,x0,u0,yref);

subplot(3,1,1)
hold on
plot(info.Yopt)
plot(ones(length(info.Yopt),1)*yref, 'r--')
title('Lateral Position')
ylabel('Lat Position (m)')
xlabel('Control Step')
ylim([0,yref+0.5])
legend( 'Calculated Trajectory','Reference Trajectory','Location','southeast')

subplot(3,1,2)
plot(rad2deg(info.MVopt))
title('Steering Input')
ylabel('Steering Angle (deg)')
xlabel('Control Step')

subplot(3,1,3)
plot(rad2deg(info.Xopt(:,5)))
title('Heading')
ylabel('Heading (deg)')
xlabel('Control Step')

sgtitle('Lane Change Maneuver')