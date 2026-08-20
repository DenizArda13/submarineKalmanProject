classdef constantVelocityModelV1 < handle
    properties
        x  % State vector
        F  % F matris
        B  % B matris
        dt % Time interval
    end

    methods
        function obj = constantVelocityModelV1(p0, v0, dt)
            obj.x = [p0;       % State with initial values
                v0];
            obj.dt = dt;
            obj.F = [1, dt;    % Discrete time state matris
                0, 1];
            obj.B = [(dt^2)/2; % B mastris for control input (u)
                dt];
        end
        function predict(obj, u) % State updater
            obj.x = obj.F*obj.x + obj.B*u; % State equation with control input
        end
    end
end