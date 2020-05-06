function dXdt = car(X,u)
    
    % Define Car Model Constants
    m = 2050; % kg
    sigma = 0.7;
    wt = 1.63; % m
    B12 = -10.5;
    B34 = -12.7;
    C12 = 0.5;
    C34 = 0.5;
    Jz = 3344; % kg*m^2
    lf = 1.43; % m
    lr = 1.47; % m
    mu = 1;
    
    % Define state variables individually
    x = X(1);
    Vx = X(2);
    y = X(3);
    Vy = X(4);
    Phi = X(5);
    Phi_dot = X(6);
    Fb = 0;
    
    % Initialize del_state vector
    dXdt = zeros(6,1);
    
    delta = u;
    
    function [Fxi,Fyi] = WheelForce(Vx,Vy,Phi_dot,delta,Fb, wheel)
        
        if wheel == 1 || wheel == 2
            fx = sigma*0.5*Fb;
            Fz = sigma*0.5*m*9.81; % Wheel weight distribution
            B = B12;
            C = C12;
        else
            fx = (1-sigma)*0.5*Fb;
            Fz = (1-sigma)*0.5*m*9.81; % Wheel weight distribution
            B = B34;
            C = C34;
            delta = 0; % Rear Wheels do not steer
        end
        
        if wheel == 1
            alpha = ((Vy+lf*Phi_dot)/(Vx-(wt/2)*Phi_dot)) - delta;
        elseif wheel == 2
            alpha = ((Vy+lf*Phi_dot)/(Vx+(wt/2)*Phi_dot)) - delta;
        elseif wheel == 3
            alpha = (Vy-lr*Phi_dot)/(Vx-(wt/2)*Phi_dot);
        elseif wheel == 4
            alpha = (Vy-lr*Phi_dot)/(Vx+(wt/2)*Phi_dot);
        end
        
        fy = sqrt((mu*Fz)^2 - fx^2) * sin(C*atan(B*alpha));
        
        Fxi = fx*cos(delta) - fy*sin(delta);
        Fyi = fx*sin(delta) + fy*cos(delta);
    end
    
    [Fx1,Fy1] = WheelForce(Vx,Vy,Phi_dot,delta,Fb,1);
    [Fx2,Fy2] = WheelForce(Vx,Vy,Phi_dot,delta,Fb,2);
    [Fx3,Fy3] = WheelForce(Vx,Vy,Phi_dot,delta,Fb,3);
    [Fx4,Fy4] = WheelForce(Vx,Vy,Phi_dot,delta,Fb,4);
    
    sumFx = sum([Fx1,Fx2,Fx3,Fx4]);
    sumFy = sum([Fy1,Fy2,Fy3,Fy4]);
    
    dXdt(1) = Vx*cos(Phi) - Vy*sin(Phi); % Inertial X Velocity
    dXdt(2) = Vy*Phi_dot + (1/m)*sumFx; % Body-Fixed X Acceleration
    dXdt(3) = Vx*sin(Phi) + Vy*cos(Phi); % Inertial Y Velocity
    dXdt(4) = -Vx*Phi_dot + (1/m)*sumFy; % Body-Fixed Y Acceleration
    dXdt(5) = Phi_dot;
    dXdt(6) = (1/Jz)*( lf*(Fy1+Fy2) - lr*(Fy3+Fy4) + 0.5*wt*(-Fx1+Fx2-Fx3+Fx4));
end
