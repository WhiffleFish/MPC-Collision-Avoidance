car = Car();
car.wt = 1.0;
env = Env();
env.obstacles = Obstacle(50, 6, 10, true);
ctrl = Controller(car, env, 50, 0.1);
%%
x0 = [0,20,0,0,0,0];
u0 = 0;
info = ctrl.move(x0,u0)

%%
ctrl.plot(info)