classdef Visualizer < handle
    properties
        X
        X_est
        U
        REF
        Z_gps
        P_diag    % YENİ: P matrisinin köşegenlerini (varyansları) tutacak matris
        IDX
        t
    end
    
    methods
        function obj = Visualizer(N, dt)
            obj.X = zeros(6, N);
            obj.X_est = zeros(6, N);
            obj.U = zeros(3, N);
            obj.REF = zeros(3, N);
            obj.Z_gps = NaN(3, N);
            obj.P_diag = zeros(6, N); % 6 boyutlu varyans kaydı
            obj.IDX = zeros(1, N);
            obj.t = (1:N) * dt;
        end
        
        % YENİ PARAMETRE: p_diag
        function logStep(obj, k, x, u, ref, idx, x_est, z_gps, p_diag)
            obj.X(:,k) = x;
            obj.U(:,k) = u;
            obj.REF(:,k) = ref;
            obj.IDX(k) = idx;
            obj.X_est(:,k) = x_est;
            obj.P_diag(:,k) = p_diag; % Varyansları kaydet
            
            if ~isnan(z_gps(1))
                obj.Z_gps(:,k) = z_gps;
            end
        end
        
function plotResults(obj, k_end, WP, V_ref)
            idx_range = 1:k_end-1;
            t_plot = obj.t(idx_range);
            X_plot = obj.X(:, idx_range);
            X_est_plot = obj.X_est(:, idx_range);
            U_plot = obj.U(:, idx_range);
            REF_plot = obj.REF(:, idx_range);
            
           %% Figure 1 — Response
            figure('Name','Control Response');
            
            % KF'den gelen Kartezyen hızları (v_x, v_y, v_z) çek
            v_x = X_est_plot(4,:);
            v_y = X_est_plot(5,:);
            v_z = X_est_plot(6,:);
            
            % Kartezyen hızlardan gerçek Toplam Hız (V) ve Euler Açılarını (psi, theta) hesapla
            V_est = sqrt(v_x.^2 + v_y.^2 + v_z.^2);
            psi_est = atan2(v_y, v_x);
            theta_est = atan2(-v_z, sqrt(v_x.^2 + v_y.^2));
            
            subplot(4,1,1);
            plot(t_plot, X_plot(6,:), 'b', 'LineWidth',1.2); hold on; grid on;
            plot(t_plot, V_est, 'm--', 'LineWidth', 1.2); % Artık gerçek V çizdiriliyor
            yline(V_ref, 'r--');
            xlabel('t [s]'); ylabel('V [m/s]'); title('Speed');
            legend('Gerçek', 'KF Tahmini', 'Referans', 'Location', 'best');
            
            subplot(4,1,2);
            plot(t_plot, rad2deg(X_plot(4,:)), 'b', 'LineWidth',1.2); hold on; grid on;
            plot(t_plot, rad2deg(psi_est), 'm--', 'LineWidth', 1.2); % Artık gerçek psi çizdiriliyor
            plot(t_plot, rad2deg(REF_plot(1,:)), 'r--');
            xlabel('t [s]'); ylabel('\psi [deg]'); title('Heading');
            
            subplot(4,1,3);
            plot(t_plot, X_plot(3,:), 'b', 'LineWidth',1.2); hold on; grid on;
            plot(t_plot, X_est_plot(3,:), 'm--', 'LineWidth', 1.2); % Derinlikte (z) uyumsuzluk yok
            plot(t_plot, REF_plot(2,:), 'r--'); set(gca,'YDir','reverse');
            xlabel('t [s]'); ylabel('z_D [m]'); title('Depth');
            
            subplot(4,1,4);
            plot(t_plot, rad2deg(X_plot(5,:)), 'b', 'LineWidth',1.2); hold on; grid on;
            plot(t_plot, rad2deg(theta_est), 'm--', 'LineWidth', 1.2); % Artık gerçek theta çizdiriliyor
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
            
            %% Figure 3 — 3B Route (GPS adımlarında noktalar)
            figure('Name','Route');
            plot3(X_plot(2,:), X_plot(1,:), -X_plot(3,:), 'b', 'LineWidth',1.2); hold on; grid on;
            
            valid_idx = ~isnan(obj.Z_gps(1, idx_range));
            plot3(X_est_plot(2, valid_idx), X_est_plot(1, valid_idx), -X_est_plot(3, valid_idx), 'm.', 'MarkerSize', 10);
            scatter3(obj.Z_gps(2,idx_range), obj.Z_gps(1,idx_range), -obj.Z_gps(3,idx_range), 20, 'yx', 'MarkerEdgeAlpha', 0.6);
            
            plot3(0,0,-10,'go','MarkerFaceColor','g');
            plot3(X_plot(2,end), X_plot(1,end), -X_plot(3,end), 'ro','MarkerFaceColor','r');
            plot3(WP(2,:), WP(1,:), -WP(3,:), 'ks','MarkerFaceColor','k','MarkerSize',6);
            xlabel('y_E [m]'); ylabel('x_N [m]'); zlabel('Height [m]');
            title('3B Route'); axis equal; view(45,25);
            legend('Gerçek Rota', 'KF Tahmini', 'GPS Ölçümleri', 'Başlangıç', 'Bitiş', 'Hedef Noktalar', 'Location', 'best');
            
            %% Figure 4 — Estimation Error & Uncertainty Bounds
            figure('Name','Uncertainty and Estimation Error');
            err = X_plot(1:3, :) - X_est_plot(1:3, :); 
            sigma = sqrt(obj.P_diag(1:3, idx_range)); 
            
            labels = {'x_N Error [m]', 'y_E Error [m]', 'z_D Error [m]'};
            for i = 1:3
                subplot(3,1,i);
                plot(t_plot, err(i,:), 'b', 'LineWidth', 1.2); hold on; grid on;
                plot(t_plot, 3*sigma(i,:), 'r--', 'LineWidth', 1.5);
                plot(t_plot, -3*sigma(i,:), 'r--', 'LineWidth', 1.5);
                ylabel(labels{i});
                if i == 1, title('Position Estimation Error and \pm3\sigma Bounds'); end
                if i == 3, xlabel('t [s]'); end
                legend('Gerçek Hata', '\pm3\sigma Sınırı', 'Location', 'best');
            end
            
            %% Figure 5 — Gaussian Error Distribution
            figure('Name','Gaussian Error Distribution');
            for i = 1:3
                subplot(1,3,i);
                histogram(err(i,:), 'Normalization', 'pdf', 'FaceColor', '#85C1E9'); hold on;
                
                mu = mean(err(i,:));
                sig = std(err(i,:));
                
                x_val = linspace(min(err(i,:)), max(err(i,:)), 100);
                y_val = (1 / (sig * sqrt(2*pi))) * exp(-0.5 * ((x_val - mu) ./ sig).^2);
                
                plot(x_val, y_val, 'r', 'LineWidth', 2);
                title(['Error in ' labels{i}(1:3)]);
                xlabel('Error [m]');
                if i == 1, ylabel('Probability Density'); end
                legend('Hata Dağılımı', 'Teorik Gauss Eğrisi', 'Location', 'best');
                grid on;
            end
        end
    end
end