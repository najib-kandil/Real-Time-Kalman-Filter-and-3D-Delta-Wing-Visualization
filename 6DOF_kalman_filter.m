
clear all;
close all;
clc;

%% --- Arduino + MPU6050 Setup ---

a = arduino('COM7', 'nano3', 'Libraries', 'I2C');

fs = 100;
dt = 1/fs;

imu = mpu6050(a, 'SampleRate', fs, 'OutputFormat', 'matrix');

disp("Reading IMU values... Press Ctrl+C to stop.");

%% --- Kalman Matrices (UNCHANGED) ---

A = eye(6);
A = A + diag(dt * ones(1,3), 3);

B = [zeros(3); dt * eye(3)];

C = [1, 1, 1, 0, 0, 0];

D = zeros(1,3);

Sa = 2.27e-3 * eye(3);
Sg = 7.62e-7 * eye(3);

V1 = blkdiag(Sa, Sg);

Ca = [1 1 1];
V2 = Ca * Sa * Ca';

V12 = zeros(6,1);

[n,~] = size(A);

%% --- Initialize Predictor/Filter (UNCHANGED) ---

x_h(:,1) = zeros(n,1);
P{1} = 0.5 * eye(n);

%% --- Initialize Original Live Plot ---

t = 0;

t_hist = [];
y_hist = [];
yhat_hist = [];

maxPoints = 200;

figure('Name', 'Kalman Filter - Live Output');

h1 = plot(nan,nan,'b','LineWidth',1.5);
hold on;

h1_raw = plot(nan,nan,'r','LineWidth',1);

grid on;
xlabel('Time (s)');
ylabel('Output');
title('Kalman Filter Output');
legend('Filtered','Raw');

%% --- 3D Delta-Wing Visualization ---

fig3d = figure('Name', 'Delta Wing Drone - Live Attitude');

ax3d = axes('Parent', fig3d);

hold(ax3d, 'on');
grid(ax3d, 'on');
axis(ax3d, 'equal');

axis(ax3d, [-2 2 -2 2 -2 2]);

xlabel(ax3d, 'X - Forward');
ylabel(ax3d, 'Y - Right');
zlabel(ax3d, 'Z - Up');

title(ax3d, 'Real-Time Delta-Wing Attitude');

view(ax3d, 35, 25);

% Fixed world reference axes

quiver3(ax3d, 0,0,0, 1.5,0,0, ...
    'r', 'LineWidth',2, 'MaxHeadSize',0.3);

quiver3(ax3d, 0,0,0, 0,1.5,0, ...
    'g', 'LineWidth',2, 'MaxHeadSize',0.3);

quiver3(ax3d, 0,0,0, 0,0,1.5, ...
    'b', 'LineWidth',2, 'MaxHeadSize',0.3);

% Transform for rotating the complete aircraft

hDrone = hgtransform('Parent', ax3d);

% Delta-wing outline: nose points along +X

wingX = [1.4, -1.0, -0.75, -1.0, 1.4];
wingY = [0,    1.5,   0,   -1.5, 0];
wingZ = zeros(size(wingX));

% Main wing surface

patch('Parent', hDrone, ...
      'XData', wingX, ...
      'YData', wingY, ...
      'ZData', wingZ, ...
      'FaceColor', [0.1 0.55 0.85], ...
      'EdgeColor', [0.05 0.1 0.2], ...
      'LineWidth', 2);

% Center fuselage

patch('Parent', hDrone, ...
      'XData', [1.35 -0.9 -0.9 1.35], ...
      'YData', [-0.10 -0.10 0.10 0.10], ...
      'ZData', [0.035 0.035 0.035 0.035], ...
      'FaceColor', [0.85 0.85 0.85], ...
      'EdgeColor', 'k');

% Nose marker

plot3(ax3d, 1.4, 0, 0.06, 'ro', ...
      'MarkerFaceColor', 'r', ...
      'MarkerSize', 7);

% Attitude initialization (radians)

roll  = 0;
pitch = 0;
yaw   = 0;

% Complementary filter coefficient

alpha = 0.98;

hDrone.Matrix = eye(4);

%% --- Live Loop ---

k = 1;

while true

    t = t + dt;

    %% --- Read IMU ---

    [accel, gyro_] = read(imu);

    acc  = accel(end,:)' / 1000;
    gyro = gyro_(end,:)' / 1000;

    %% --- Measurement (UNCHANGED) ---

    y = C * x_h(:,k);

    % Measurement from the accelerometer

    y = acc(1) + acc(2) + acc(3);

    %% --- Dynamic Predictor K (UNCHANGED) ---

    y_h = C * x_h(:,k);

    e = y - y_h;

    K{k} = (A * P{k} * C' + V12) * ...
           inv(C * P{k} * C' + V2);

    x_h(:,k+1) = A * x_h(:,k) + B * gyro + K{k} * e;

    P{k+1} = A * P{k} * A' + V1 ...
           - K{k} * (C * P{k} * C' + V2) * K{k}';

    %% --- Dynamic Filter F (UNCHANGED) ---

    K0{k} = P{k} * C' * inv(C * P{k} * C' + V2);

    x_f(:,k) = x_h(:,k) + K0{k} * e;

    y_f = C * x_f(:,k);

    %% --- Estimate Drone Attitude ---

    % Accelerometer-based roll and pitch.
    % Assumes the sensor axes match the aircraft body axes
    % and the sensor is not undergoing strong linear motion.

    rollAcc = atan2(acc(2), acc(3));

    pitchAcc = atan2(-acc(1), ...
        sqrt(acc(2)^2 + acc(3)^2));

    % Gyroscope integration

    rollGyro  = roll  + gyro(1) * dt;
    pitchGyro = pitch + gyro(2) * dt;
    yawGyro   = yaw   + gyro(3) * dt;

    % Complementary filter

    roll  = alpha * rollGyro  + (1-alpha) * rollAcc;
    pitch = alpha * pitchGyro + (1-alpha) * pitchAcc;

    % No magnetometer correction is available for yaw

    yaw = yawGyro;

    %% --- Build 3D Rotation Matrix ---

    % Rotation around Z (yaw)

    Rz = [cos(yaw), -sin(yaw), 0;
          sin(yaw),  cos(yaw), 0;
          0,         0,        1];

    % Rotation around Y (pitch)

    Ry = [cos(pitch),  0, sin(pitch);
          0,           1, 0;
         -sin(pitch),  0, cos(pitch)];

    % Rotation around X (roll)

    Rx = [1, 0,          0;
          0, cos(roll), -sin(roll);
          0, sin(roll),  cos(roll)];

    % Combined Z-Y-X rotation

    R = Rz * Ry * Rx;

    % Homogeneous transformation matrix

    T = eye(4);

    T(1:3,1:3) = R;

    %% --- Update 3D Drone ---

    if isvalid(fig3d) && isvalid(hDrone)

        hDrone.Matrix = T;

        title(ax3d, sprintf( ...
            'Delta Wing | Roll: %.1f deg | Pitch: %.1f deg | Yaw: %.1f deg', ...
            rad2deg(roll), ...
            rad2deg(pitch), ...
            rad2deg(yaw)));

        drawnow limitrate;

    end

    %% --- Store Data for Original Plot ---

    if length(t_hist) >= maxPoints

        t_hist = [t_hist(2:end), t];
        y_hist = [y_hist(2:end), y];
        yhat_hist = [yhat_hist(2:end), y_f];

    else

        t_hist(end+1) = t;
        y_hist(end+1) = y;
        yhat_hist(end+1) = y_f;

    end

    %% --- Update Original Plot ---

    if isgraphics(h1) && isgraphics(h1_raw)

        set(h1, ...
            'XData', t_hist, ...
            'YData', yhat_hist);

        set(h1_raw, ...
            'XData', t_hist, ...
            'YData', y_hist);

        drawnow limitrate;

    end

    pause(dt);

    k = k + 1;

end
