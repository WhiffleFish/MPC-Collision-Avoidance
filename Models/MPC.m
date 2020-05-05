nx = 6; % [x Vx y Vy Phi Phidot]
ny = 1; % Need Output Fcn to determine which state values are measured
nu = 1;
nlobj = nlmpc(nx,ny,nu);

% p = c = 100 for 5m/s (Ts = 0.2)
% p = c = 100 for 10m/s
% p = c = 67 for 15m/s
% p = c = 50 for 20m/s
Ts = 0.1;
p = 50; % Run simulation for p time steps
c = 50; % Calculate and employ Control actions for c time steps
nlobj.Ts = Ts;
nlobj.PredictionHorizon = p;
nlobj.ControlHorizon = c;

nlobj.Model.StateFcn = "car";
nlobj.Model.OutputFcn = @(x,u) x(3); % Only need Ypos

nlobj.MV.Min = deg2rad(-30);
nlobj.MV.Max = deg2rad(30);
nlobj.MV.RateMax = deg2rad(100*Ts);


Xobs = 25; % X Position of obstacle
Wobs = 2; % Obstacle width
Wroad = 4; % road width 
Lobs = 10; % Obstacle length

K_phi=0.1; % Maintain heading weight
nlobj.Optimization.CustomCostFcn = @(X,u,e,data) sum(u.^2) + K_phi*sum(X(:,5).^2);
nlobj.Optimization.ReplaceStandardCost = true;

nlobj.Optimization.CustomIneqConFcn = @(X,U,e,data) IneqConFunction(X,U,e,data);

x0 = [0 20 0 0 0 0];
u0 = 0;

% Set Reference
yref = 0.5; % not really necessary as yref not a part of obj fcn

[~,~,info] = nlmpcmove(nlobj,x0,u0,yref);