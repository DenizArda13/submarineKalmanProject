classdef pidController < handle
    properties
        Kp
        Ki
        Kd
        dt
        I_k
        e_prev
    end
    methods
        function obj = pidController(Kp, Ki, Kd, dt)
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;
            obj.dt = dt;
            obj.e_prev = 0;
            obj.I_k = 0;
        end

        function u = controlSignal(obj, r, y)
            e_k = r-y;
            D_k = (e_k-obj.e_prev)/obj.dt;
            obj.I_k = obj.I_k + e_k*obj.dt;
            u = obj.Kp*e_k + obj.Ki*obj.I_k + obj.Kd*D_k;
            obj.e_prev = e_k;
        end
    end
end



