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

%% Kalman Filter and Sensor Parameters
sigma_gps = [1;
    1;
    1];

sigma_imu = [0.01;
    0.01;
    0.01];

H_sensor = [eye(3),zeros(3,3)];

Q_sigma_process = [0.1;
    0.1;
    0.1;
    0.1;
    0.1;
    0.1];


%% Objects %%
sub = submarineModelv1([0;0;10;0;0;0], dt);
est_sub = submarineModelv1([0;0;10;0;0;0], dt);

autopilot = Autopilot(dt, V_ref);
vis = Visualizer(N, dt);


sensor = sensorSimulator(H_sensor, sigma_gps, sigma_imu);
x0 = [sensor.getGPS(sub.x);zeros(3,1)];
kf = kalmanFilter(x0, sigma_gps, Q_sigma_process, dt);

%% Simulation Loop %%
for k = 1:N

    psi = sub.x(4);
    theta = sub.x(5);
    V = sub.x(6);
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

    %% Autopilot %%
    [u_clean, ref_signals] = autopilot.getCommands(p, V, psi, theta, wp);
    sub.predict(u_clean);

    %% Sensor Simulation and Kalman Filter Steps %%
    u_imu = sensor.getIMU(V, theta, psi, u_clean);
    kf.predict(u_imu);
    z_current = NaN(3,1);

    if mod(k, 100) == 0
        z_gps = sensor.getGPS(sub.x);
        kf.correction(z_gps);
        z_current = z_gps;
    end
    % En sona diag(kf.P) eklendi
    vis.logStep(k, sub.x, u_clean, ref_signals, idx, kf.x, z_current, diag(kf.P));
end
vis.plotResults(k, WP, V_ref);