classdef Visualizer < handle
    properties
        X
        U
        REF
        IDX
        t
    end
    
    methods
        function obj = Visualizer(N, dt)
            obj.X = zeros(6, N);
            obj.U = zeros(3, N);
            obj.REF = zeros(3, N);
            obj.IDX = zeros(1, N);
            obj.t = (1:N) * dt;
        end
        
        function logStep(obj, k, x, u, ref, idx)
            obj.X(:,k) = x;
            obj.U(:,k) = u;
            obj.REF(:,k) = ref;
            obj.IDX(k) = idx;
        end
        
        function plotResults(obj, k_end, WP, V_ref)
            idx_range = 1:k_end-1;

            t_plot = obj.t(idx_range);
            X_plot = obj.X(:, idx_range);
            U_plot = obj.U(:, idx_range);
            REF_plot = obj.REF(:, idx_range);
            
            %% Figure 1 — Response
            figure('Name','Control Response');
            subplot(4,1,1);
            plot(t_plot, X_plot(6,:), 'LineWidth',1.2); grid on; hold on;
            yline(V_ref, 'r--');
            xlabel('t [s]'); ylabel('V [m/s]'); title('Speed');
            
            subplot(4,1,2);
            plot(t_plot, rad2deg(X_plot(4,:)), 'LineWidth',1.2); grid on; hold on;
            plot(t_plot, rad2deg(REF_plot(1,:)), 'r--');
            xlabel('t [s]'); ylabel('\psi [deg]'); title('Heading');
            
            subplot(4,1,3);
            plot(t_plot, X_plot(3,:), 'LineWidth',1.2); grid on; hold on;
            plot(t_plot, REF_plot(2,:), 'r--'); set(gca,'YDir','reverse');
            xlabel('t [s]'); ylabel('z_D [m]'); title('Depth');
            
            subplot(4,1,4);
            plot(t_plot, rad2deg(X_plot(5,:)), 'LineWidth',1.2); grid on; hold on;
            plot(t_plot, rad2deg(REF_plot(3,:)), 'r--');
            xlabel('t [s]'); ylabel('\theta [deg]'); title('Pitching Angle');

            %% Figure 2 — Control Signals
            figure('Name','Control Signals');
            subplot(3,1,1);
            plot(t_plot, U_plot(1,:), 'LineWidth',1.2); grid on; hold on;
            xlabel('t [s]'); ylabel('a_{cmd} [m/s^2]'); title('Acceleration Command');
            
            subplot(3,1,2);
            plot(t_plot, U_plot(2,:), 'LineWidth',1.2); grid on; hold on;
            xlabel('t [s]'); ylabel('r_{cmd} [rad/s]'); title('Heading Rate Command');
            
            subplot(3,1,3);
            plot(t_plot, U_plot(3,:), 'LineWidth',1.2); grid on; hold on;
            xlabel('t [s]'); ylabel('q_{cmd} [rad/s]'); title('Pitching Rate Command');
            
            %% Figure 3 — 3B Route
            figure('Name','Route');
            plot3(X_plot(2,:), X_plot(1,:), -X_plot(3,:), 'LineWidth',1.2); grid on; hold on;
            plot3(0,0,-10,'go','MarkerFaceColor','g');
            plot3(X_plot(2,end), X_plot(1,end), -X_plot(3,end), 'ro','MarkerFaceColor','r');
            plot3(WP(2,:), WP(1,:), -WP(3,:), 'ks','MarkerFaceColor','k','MarkerSize',6);
            xlabel('y_E [m]'); ylabel('x_N [m]'); zlabel('Height [m]');
            title('3B Route'); axis equal; view(45,25);
        end
    end
end