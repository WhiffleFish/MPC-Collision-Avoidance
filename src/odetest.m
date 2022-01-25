car = Car();
tspan = [0,10];
x0 = [0,10,0,0,0,0];
[t,X] = ode45(@(t,X) car.dX(X,sin(t)), tspan, x0)
plot(X(:,1), X(:,3))

%%
env = Env();
const = env.IneqConFcn(X,0,0,data,0,10);

all(const <= 0)

const <= 0

%%
L = length(env.obstacles);
P = size(X,1)-1;
con = -ones(L+2,P);
x = X(2:end,1);
y = X(2:end,3);
con(1,:) = y - env.Wroad;
con(2,:) = -y - env.Wroad;

%%
obs = env.obstacles(1);
xmask = (x >= obs.x) .* (x <= obs.x + obs.length);
con(3,:) = xmask .* (-y +(obs.width-env.Wroad));