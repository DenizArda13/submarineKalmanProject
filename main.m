%% Parameters %%
dt = 0.01;
T = 600;
N = T/dt;
V_ref = 1.5;
L = 60;
WP = (L/2) * [ -1  -1  1  1 -1  1  1 -1;
                1  -1 -1  1 -1 -1  1  1;
               -1  -1 -1 -1  1  1  1  1];
WP(3,:) = WP(3,:) + 40;
idx = 1; 
R_acc = 2; 
nWP = size(WP,2); 

%% Objects %%
sub = submarineModelv1([0;0;10;0;0;0], dt);
autopilot = Autopilot(dt, V_ref);
vis = Visualizer(N, dt);

%% Simulation Loop %%
for k = 1:N
    %% Measure %%
    V = sub.x(6);
    psi = sub.x(4);
    theta = sub.x(5);
    p = sub.x(1:3);
    
    %% Waypoint Management %%
    wp = WP(:,idx);
    if norm(wp - p) < R_acc
        if idx == nWP
            break
        end
        idx = idx + 1;
        wp = WP(:,idx);
    end    
    
    %% Autopilot & Step %%
    [u, ref_signals] = autopilot.getCommands(p, V, psi, theta, wp);
    
    sigma_u = [1; 1; 1]; % Noise 
    u_noisy = u +sigma_u.*rand(3,1);

    sub.predict(u_noisy);
    
    %% Log Data %%
    vis.logStep(k, sub.x, u, ref_signals, idx);
end

%% Graphs %%
vis.plotResults(k, WP, V_ref);