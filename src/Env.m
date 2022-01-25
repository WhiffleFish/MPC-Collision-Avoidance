classdef Env
    properties
        obstacles = [Obstacle()];
        Wroad = 4;
        Wcar = 0
    end

    methods
        function con = IneqConFcn(obj, X, ~, ~, ~, ~)
            L = length(obj.obstacles);
            P = size(X,1)-1;
            con = -ones(P, L+2);
            Wcar = obj.Wcar; %#ok<*PROPLC> 
            x = X(2:end,1);
            y = X(2:end,3);
            con(:,1) = y+Wcar - obj.Wroad;
            con(:,2) = -y+Wcar - obj.Wroad;
            
            obstacles = obj.obstacles;
            for i = 3:L+2
                obs = obstacles(i-2);
                xmask = (x >= obs.x) .* (x <= obs.x + obs.length);
                if obs.bottom
                    con(:,i) = xmask .* (-y+ Wcar +(obs.width-obj.Wroad));
                else
                    con(:,i) = xmask .* (y + Wcar -(obj.Wroad - obs.width));
                end
            end
            con = reshape(con,1,[]);
        end
    end
end