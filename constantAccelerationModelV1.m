classdef constantAccelerationModelV1 < handle
    properties
        x % State vector
        F % F matris
        G % G matris
        dt% Time interval
    end
    methods
        function obj = constantAccelerationModelV1(p0, v0, a0, dt)
            obj.dt = dt;
            obj.x = [p0; v0; a0];  % State with initial values
            obj.F = [1, dt, dt^2/2; % Discrete time A matris
                0, 1, dt;
                0, 0, 1];
            obj.G = [dt^3/6;       % Noise control input
                dt^2/2;
                dt];
        end
        function predict(obj, w)
            obj.x = obj.F*obj.x + obj.G*w;
        end
    end
end
