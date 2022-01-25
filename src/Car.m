classdef Car
    properties
        m       = 2050  % car mass (kg)
        sigma   = 0.7   % weight distribution (unitless)
        wt      = 1.63  % vehicle width (m)
        B12     = -10.5 % Pacejka forward tire coefficient
        B34     = -12.7 % Pacejka rear tire coefficient
        C12     = 0.5   % Pacejka forward tire coefficient
        C34     = 0.5   % Pacejka rear tire coefficient
        Jz      = 3344  % (kg*m^2)
        lf      = 1.43  % Distance from CM to forward wheels (m)
        lr      = 1.47  % Distance from CM to rear wheels(m)
        mu      = 1.0   % wheel friction coefficient
    end
    methods
        function dXdt = dX(obj, X, u)
            x = X(1);
            Vx = X(2);
            y = X(3);
            Vy = X(4);
            Phi = X(5);
            Phi_dot = X(6);
            Fb = 0;
            
            dXdt = zeros(6,1);
            [Fx1,Fy1] = obj.WheelForce(Vx, Vy, Phi_dot, u, Fb, 1);
            [Fx2,Fy2] = obj.WheelForce(Vx, Vy, Phi_dot, u, Fb, 2);
            [Fx3,Fy3] = obj.WheelForce(Vx, Vy, Phi_dot, u, Fb, 3);
            [Fx4,Fy4] = obj.WheelForce(Vx, Vy, Phi_dot, u, Fb, 4);

            sumFx = sum([Fx1,Fx2,Fx3,Fx4]);
            sumFy = sum([Fy1,Fy2,Fy3,Fy4]);

            dXdt(1) = Vx*cos(Phi) - Vy*sin(Phi); % Inertial X Vel
            dXdt(2) = Vy*Phi_dot + (1/obj.m)*sumFx; % Body-Fixed X Accel
            dXdt(3) = Vx*sin(Phi) + Vy*cos(Phi); % Inertial Y Vel
            dXdt(4) = -Vx*Phi_dot + (1/obj.m)*sumFy; % Body-Fixed Y Accel
            dXdt(5) = Phi_dot;
            dXdt(6) = (1/obj.Jz)*( ...
                obj.lf*(Fy1+Fy2) - ...
                obj.lr*(Fy3+Fy4) + ...
                0.5*obj.wt*(-Fx1+Fx2-Fx3+Fx4) ...
                );
        end

        function [Fxi,Fyi] = WheelForce(obj, Vx, Vy, Phi_dot, delta, Fb, wheel)

            if wheel == 1 || wheel == 2
                fx = obj.sigma*0.5*Fb;
                Fz = obj.sigma*0.5*obj.m*9.81; % Wheel weight distribution
                B = obj.B12;
                C = obj.C12;
            else
                fx = (1-obj.sigma)*0.5*Fb;
                Fz = (1-obj.sigma)*0.5*obj.m*9.81; % Wheel weight distribution
                B = obj.B34;
                C = obj.C34;
                delta = 0; % Rear Wheels do not steer
            end
            
            if wheel == 1
                alpha = ((Vy+obj.lf*Phi_dot)/(Vx-(obj.wt/2)*Phi_dot)) - delta;
            elseif wheel == 2
                alpha = ((Vy+obj.lf*Phi_dot)/(Vx+(obj.wt/2)*Phi_dot)) - delta;
            elseif wheel == 3
                alpha = (Vy-obj.lr*Phi_dot)/(Vx-(obj.wt/2)*Phi_dot);
            elseif wheel == 4
                alpha = (Vy-obj.lr*Phi_dot)/(Vx+(obj.wt/2)*Phi_dot);
            end
            
            fy = sqrt((obj.mu*Fz)^2 - fx^2) * sin(C*atan(B*alpha));
            
            Fxi = fx*cos(delta) - fy*sin(delta);
            Fyi = fx*sin(delta) + fy*cos(delta);
        end
    end
end