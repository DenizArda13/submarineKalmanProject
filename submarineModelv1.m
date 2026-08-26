classdef submarineModelv1 < handle
    properties
        x
        dt
    end
    methods
        function obj = submarineModelv1(x0, dt)
            obj.x = x0;
            obj.dt = dt;
        end

        function dx = derivatives(obj, u)
            psi = obj.x(4);
            theta = obj.x(5);
            V = obj.x(6);
            a = u(1);
            r = u(2);
            q = u(3);
            dxN = V*cos(theta)*cos(psi);
            dyE = V*cos(theta)*sin(psi);
            dzD = -V*sin(theta);
            dpsi = r;
            dtheta = q;
            dV = a;
            dx = [dxN;
                dyE;
                dzD;
                dpsi;
                dtheta;
                dV];
        end
        function predict(obj, u)
            obj.x = obj.x + obj.dt* obj.derivatives(u);
            obj.x(4) = atan2(sin(obj.x(4)), cos(obj.x(4)));
            obj.x(5) = atan2(sin(obj.x(5)), cos(obj.x(5)));
        end
    end
end


