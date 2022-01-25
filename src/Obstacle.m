classdef Obstacle
    properties
        x       % obstacle x position (m)
        width   % obstacle width / y span (m)
        length  % obstacle length / x span (m)
        bottom  % obstacle lies on bottom of road (bool)
    end
    methods
        function obj = Obstacle(x, width, length, bottom)
            if nargin == 0
                obj.x = 25;
                obj.width = 2;
                obj.length = 10;
                obj.bottom = true;
            else
                obj.x = x;
                obj.width = width;
                obj.length = length;
                obj.bottom = bottom;
            end
        end
    end
end