%% 1. Simulation Parameters
fs = 100;               % Sampling frequency (Hz)
T = 10;                 % Total simulation time (s)
t = 0:1/fs:T-1/fs;      % Time vector
N = length(t);          % Number of samples

%% 2. Generate True Angle (Oscillation amplitude ±40 degrees)
% Superposition of multiple frequency components to ensure sufficient oscillation
true_angle = 40 * sin(0.5 * pi * t / T) + ...
             10 * sin(2 * pi * t / T) + ...
             5 * sin(5 * pi * t / T);

%% 3. Simulate Accelerometer Angle (with increased high-frequency noise)
% Base noise (low frequency component)
accel_noise_low = 5 * randn(1, N);
% Add high-frequency noise component (10x higher frequency content)
accel_noise_high = 8 * randn(1, N) .* sin(20 * pi * t);  % High-frequency noise
% Combine low and high frequency noise components
accel_noise = accel_noise_low + accel_noise_high;
% Add random spikes (simulating motion shocks)
spike_indices = datasample(1:N, 20, 'Replace', false);
accel_noise(spike_indices) = 15 * randn(1, 20);
accel_angle = true_angle + accel_noise;

%% 4. Simulate Gyroscope Angle (with drift)
% Gyroscope angular velocity (true angular velocity + drift)
gyro_rate = gradient(true_angle, t) + 0.5 * t;  % Angular velocity with time-dependent drift
% Integrate to get gyroscope angle (initial angle set to 0)
gyro_angle = zeros(1, N);
for i = 2:N
    gyro_angle(i) = gyro_angle(i-1) + gyro_rate(i) * (t(i)-t(i-1));
end

%% 5. Complementary Filter Fusion
alpha = 0.98;           % Filter coefficient balancing accelerometer (slow-varying) and gyroscope (fast-varying)
comp_angle = zeros(1, N);
comp_angle(1) = accel_angle(1);  % Initial value from accelerometer
for i = 2:N
    % Complementary filter formula
    comp_angle(i) = alpha * (comp_angle(i-1) + gyro_rate(i) * (t(i)-t(i-1))) + (1 - alpha) * accel_angle(i);
end

%% 6. Spectrum Analysis (extended frequency range)
f = (0:N-1)*(fs/N);     % Frequency vector
f = f(1:floor(N/2));    % Single-sided spectrum

% Compute spectra for each angle data
true_fft = abs(fft(true_angle));
true_fft = true_fft(1:floor(N/2))*2/N;

accel_fft = abs(fft(accel_angle));
accel_fft = accel_fft(1:floor(N/2))*2/N;

gyro_fft = abs(fft(gyro_angle));
gyro_fft = gyro_fft(1:floor(N/2))*2/N;

comp_fft = abs(fft(comp_angle));
comp_fft = comp_fft(1:floor(N/2))*2/N;

%% 7. Time-domain Plot
figure('Position', [100, 100, 1000, 600])
plot(t, true_angle, 'k-', 'LineWidth', 2, 'DisplayName', 'True Angle');
hold on;
plot(t, accel_angle, 'r--', 'LineWidth', 1, 'DisplayName', 'Accelerometer');
plot(t, gyro_angle, 'g--', 'LineWidth', 1, 'DisplayName', 'Gyroscope');
plot(t, comp_angle, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Complementary Filter');
xlabel('Time (s)');
ylabel('Angle (°)');
title('MPU6050 Angle Estimation Comparison (Time Domain)');
grid on;
ylim([-60, 60]);        % Set Y-axis range to better show ±40° oscillation
legend('True Angle', 'Accelerometer', 'Gyroscope', 'Complementary Filter', 'Location', 'best');

%% 8. Frequency-domain Plot (extended to 20Hz)
figure('Position', [100, 100, 1000, 600])
plot(f, true_fft, 'k-', 'LineWidth', 2, 'DisplayName', 'True Angle');
hold on;
plot(f, accel_fft, 'r--', 'LineWidth', 1, 'DisplayName', 'Accelerometer');
plot(f, gyro_fft, 'g--', 'LineWidth', 1, 'DisplayName', 'Gyroscope');
plot(f, comp_fft, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Complementary Filter');
xlabel('Frequency (Hz)');
ylabel('Amplitude');
title('MPU6050 Angle Estimation Spectrum Comparison (Frequency Domain)');
grid on;
xlim([0, 20]);          % Extended frequency range to 20Hz to show high-frequency components
legend('True Angle', 'Accelerometer', 'Gyroscope', 'Complementary Filter', 'Location', 'best');