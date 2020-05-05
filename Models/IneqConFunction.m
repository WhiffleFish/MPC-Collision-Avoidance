function cineq = IneqConFunction(X,~,~,~,~)
    % cineq = IneqConFunction(X,U,e,data)
    
    Xobs = 25; % X Position of obstacle
    Wobs = 2; % Obstacle width
    Wroad = 4; % road width 
    Lobs = 10; % Obstacle length
    
    x = X(:,1);
    y = X(:,3);
    Fx = Wobs*(heaviside(x-Xobs) -heaviside(x-(Xobs+Lobs)));
    
    
    cineq = [-y+Fx;
             y-Wroad];

end