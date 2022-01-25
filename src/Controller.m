classdef Controller
    properties
        model
        mpc
        env
        yref
        Ky
        T
        Ts
    end
    methods
        function obj = Controller(model, env, T, Ts)
            env.Wcar = model.wt;
            obj.yref = 0.0;
            obj.Ky = 1.0;
            obj.env = env;
            obj.T = T;
            obj.Ts = Ts;

            nlobj = nlmpc(6,1,1);
            nlobj.Optimization.CustomCostFcn = @obj.cost;
            nlobj.Optimization.ReplaceStandardCost = true;
            nlobj.Ts = Ts;
            nlobj.PredictionHorizon = T;
            nlobj.ControlHorizon = T;
            nlobj.MV.Min = deg2rad(-30);
            nlobj.MV.Max = deg2rad(30);
%             nlobj.MV.RateMax = deg2rad(50*Ts);
            nlobj.Model.OutputFcn = @(X,u,params) X(3);

            obj.model = model;
            obj.mpc = nlobj;
            obj.mpc.Model.StateFcn = @model.dX;
            
            
            obj.mpc.Optimization.CustomIneqConFcn = @env.IneqConFcn;
            
        end
        
        function set_horizon(obj, T)
            obj.T = T;
            obj.mpc.PredictionHorizon = T;
            obj.mpc.ControlHorizon = T;
        end

        function set_model(obj, model)
            obj.model = model;
            obj.mpc.Model.StateFcn = @model.dX;
            obj.mpc.Model.OutputFcn = @(X,u,params) X(3);
        end
           
        function set_env(obj, env)
            obj.env = env;
            obj.mpc.Optimization.CustomEqConFcn = @env.IneqConFcn;
        end

        function info = move(obj, x0, u0)
            [~,~,info] = nlmpcmove(obj.mpc, x0, u0, obj.yref);
        end

        function c = cost(obj, X, u, ~, ~)
            Wroad = obj.env.Wroad;
            c = sum(u.^2) + obj.Ky*sum((X(:,3)/Wroad - obj.yref).^2);
        end
        
        function plot(obj, info)
            X = info.Xopt;
            u = rad2deg(info.MVopt);
            x = X(:,1);
            y = X(:,3);
            env = obj.env; %#ok<*PROPLC> 
            obstacles = env.obstacles;
            
           
            subplot(2,1,1)
            hold on
            
            for i = 1:length(obstacles)
                obs = obstacles(i);
                x1 = obs.x;
                x2 = obs.x + obs.length;
                if obs.bottom
                    y1 = -env.Wroad;
                    y2 = -env.Wroad + obs.width;
                else
                    y1 = env.Wroad;
                    y2 = env.Wroad - obs.width;
                end

                fill([x1, x2, x2, x1], [y1, y1, y2, y2], 'red')
            end
            plot(x,y,'LineWidth',2)
            plot(xlim, [obj.yref,obj.yref], 'r--')
            xlabel("X (m)")
            ylabel("Y (m)")
            title("Position")


            subplot(2,1,2)
            plot(info.Topt, u,'LineWidth',2)
            ylabel("Wheel deflection (deg)")
            xlabel("Time (s)")
            title("Control Input")
        end

    end
end