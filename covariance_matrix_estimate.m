close all; clear; clc

T = readtable('IMU.csv');
idx = 1000:3000;                       % static segment (verify by plotting)

gyro = T{idx,1:3} / 1000 * pi/180;     % mdeg/s -> rad/s
acc  = T{idx,4:6} / 1000 * 9.80665;    % mg -> m/s^2
ts   = T{idx,8} * 1e-6;                % us -> s

% Sample rate (check: timestamps repeat in blocks, so use a robust estimate)
dt = (ts(end) - ts(1)) / (numel(ts)-1);
fprintf('Estimated dt = %.6f s (%.1f Hz)\n', dt, 1/dt);

% Statistics (consistent N-1 normalization)
a_hat = mean(acc,1);   Sa = cov(acc);
g_hat = mean(gyro,1);  Sg = cov(gyro);

fprintf('|a_hat| = %.3f m/s^2 (expect ~9.81)\n', norm(a_hat));
disp('Gyro bias (rad/s):');  disp(g_hat)
disp('Acc cov:');            disp(Sa)
disp('Gyro cov:');           disp(Sg)
disp('Acc corr:');           disp(corrcoef(acc))

% Derived quantities for the filter
roll0  = atan2(a_hat(2), a_hat(3));
pitch0 = atan2(-a_hat(1), hypot(a_hat(2), a_hat(3)));
R_acc  = Sa / norm(a_hat)^2;           % approx. tilt-angle variance (rad^2)
Q_ang  = Sg * dt^2;                    % angle noise variance per step

% Sanity plot
figure; subplot(2,1,1); plot(gyro); title('Gyro (rad/s)');
subplot(2,1,2); plot(acc); title('Acc (m/s^2)');
save('cov_matrices.mat', 'Sa', 'Sg');  