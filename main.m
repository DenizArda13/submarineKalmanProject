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

%% EKF ve Sensör Parametreleri (YENİ) %%
% GPS Gözlem Matrisi (Sadece ilk 3 durumu: X, Y, Z ölçer)
H_gps = [eye(3), zeros(3,3)]; 

% Kovaryans Matrisleri (Örnek değerlerdir, sisteminize göre ayarlayabilirsiniz)
R_gps = diag([2^2, 2^2, 2^2]);          % GPS Hatası: 2m standart sapma
R_imu = diag([0.1^2, 0.05^2, 0.05^2]);  % IMU Hatası: İvme ve açısal hız standart sapmaları
Q = eye(6) * 0.01;                      % EKF Süreç Gürültüsü (Sistem modelindeki belirsizlik)
P0 = eye(6) * 100;                      % Başlangıç tahmin belirsizliği (Yüksek tutulur)

%% Objects %%
sub = submarineModelv1([0;0;10;0;0;0], dt);           % Gerçek Dünya
est_sub = submarineModelv1([0;0;10;0;0;0], dt);       % Kalman'ın Sanal Dünyası

autopilot = Autopilot(dt, V_ref);
vis = Visualizer(N, dt);

% Sensör ve EKF Objeleri (YENİ)
sensor = sensorSimulator(H_gps, R_gps, R_imu);
% est_sub objesinin içindeki durum vektörünü (x) ve zaman adımını (dt) gönderiyoruz
kf = kalmanFilter(est_sub.x, P0, Q, R_gps, H_gps, dt);

%% Simulation Loop %%
for k = 1:N
    %% Otopilot İçin Ölçüm (YENİ: Artık Gerçeği Değil, Kalman'ı Kullanıyoruz!) %%
%% Otopilot İçin Ölçüm (YENİ: Artık Gerçeği Değil, Kalman'ı Kullanıyoruz!) %%
    V_est     = kf.x_est(6);
    psi_est   = kf.x_est(4);
    theta_est = kf.x_est(5);
    p_est     = kf.x_est(1:3);
    
    %% Waypoint Management %%
    wp = WP(:,idx);
    if norm(wp - p_est) < R_acc  % (YENİ: Hedefe varışı da EKF tahminiyle kontrol ediyoruz)
        if idx == nWP
            break
        end
        idx = idx + 1;
        wp = WP(:,idx);
    end    
    
    %% Autopilot %%
    % Otopilot, EKF tahminlerine göre komut üretir
    [u_clean, ref_signals] = autopilot.getCommands(p_est, V_est, psi_est, theta_est, wp);
    
    %% Sensör Simülasyonu ve Gerçek Modelin İlerlemesi (YENİ) %%
    % 1. IMU gürültülü okuma yapar (Komuta gürültü biner)
    u_imu = sensor.getIMU(u_clean);
    
    % 2. Gerçek denizaltı fiziksel olarak u_imu (gürültülü komut) ile ilerler
    sub.predict(u_imu);
    
    % 3. GPS gerçek konumdan gürültülü okuma yapar
    z_gps = sensor.getGPS(sub.x);
    
    %% Extended Kalman Filter (EKF) Adımları (YENİ) %%
    % 1. IMU verisi ile kör uçuş (Tahmin)
    kf.predict(u_imu);
    
    % 2. GPS verisi ile doğrulama (Güncelleme)
    % Not: GPS genelde IMU'dan yavaştır. İsterseniz buraya "if mod(k, 100) == 0" 
    % gibi bir şart koyarak GPS'i saniyede 1 kez çalıştırabilirsiniz.
    kf.update(z_gps);
    
    %% Log Data %%
    % (İsteğe bağlı: Visualizer'ı hem sub.x hem de kf.model_est.x alacak şekilde 
    %  güncelleyerek gerçek vs. tahmin grafiklerini üst üste çizdirebilirsiniz)
    vis.logStep(k, sub.x, u_clean, ref_signals, idx);
end

%% Graphs %%
vis.plotResults(k, WP, V_ref);