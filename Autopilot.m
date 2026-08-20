classdef Autopilot < handle
    properties
        ctrlV
        ctrlPsi
        ctrlTheta
        V_ref
    end

    methods
        function obj = Autopilot(dt, V_ref, pidV, pidPsi, pidTheta)
            arguments
                dt (1,1) double
                V_ref (1,1) double
                pidV (1,3) double = [10, 0, 0]       % [Kp, Ki, Kd]
                pidPsi (1,3) double = [10, 0, 0]     % [Kp, Ki, Kd]
                pidTheta (1,3) double = [10, 0, 0]   % [Kp, Ki, Kd]
            end

            obj.V_ref = V_ref;
            %% PID configurations %%
            obj.ctrlV = pidController(pidV(1), pidV(2), pidV(3), dt);
            obj.ctrlPsi = pidController(pidPsi(1), pidPsi(2), pidPsi(3), dt);
            obj.ctrlTheta = pidController(pidTheta(1), pidTheta(2), pidTheta(3), dt);
        end

        function [u, ref_signals] = getCommands(obj, p, V, psi, theta, wp)
            %% Guidance %%
            dN = wp(1) - p(1);
            dE = wp(2) - p(2);
            dZ = wp(3) - p(3);

            dNE = sqrt(dN^2 + dE^2);
            psi_ref = atan2(dE, dN);
            theta_ref = atan2(-dZ, dNE);
            z_ref = wp(3);

            %% Control %%
            a_cmd = obj.ctrlV.controlSignal(obj.V_ref, V);

            e_psi = atan2(sin(psi_ref-psi), cos(psi_ref-psi));
            r_cmd = obj.ctrlPsi.controlSignal(e_psi, 0);

            e_theta = atan2(sin(theta_ref-theta), cos(theta_ref-theta));
            q_cmd = obj.ctrlTheta.controlSignal(e_theta, 0);

            %% Outputs %%
            u = [a_cmd; r_cmd; q_cmd];
            ref_signals = [psi_ref; z_ref; theta_ref];
        end
    end
end