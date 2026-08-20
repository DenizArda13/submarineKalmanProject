classdef caModelNED < handle
    properties
        x
        dt
        B
        F
        sigma_a
        Q
    end
    methods
        function obj = caModelNED (p0, v0, dt,  sigma_a)
            arguments
                p0 (3,1) double = zeros(3,1)
                v0 (3,1) double = zeros(3,1)
                dt (1,1) double {mustBePositive} = 0.1
                sigma_a (3,1) double {mustBeNonnegative} = 0.05*ones(3,1)
            end
            obj.dt = dt;
            obj.F = [eye(3), dt*eye(3);
                zeros(3), eye(3)];
            obj.B = [(dt^2/2)*eye(3);
                dt*eye(3)];
            obj.x = [p0;
                v0];
            obj.sigma_a = sigma_a;
            Sigma_a = diag(sigma_a.^2); % her elemanın ayrı karesi alınıyor ".^2" ile standart sapmanın karesini elde ediyoruz
            obj.Q = obj.B * Sigma_a*obj.B';

        end
        function predict(obj, u, w)
            arguments
                obj (1,1) caModelNED
                u (3,1) double = zeros(3,1)
                w (3,1) double = zeros(3,1)
            end
            obj.x = obj.F*obj.x +obj.B*(u+w);
        end
    end
end

