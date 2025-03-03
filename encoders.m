time = positions2.Var1;      % Extract time column
position = positions2.Var2;  % Extract position column

% Compute velocity (numerical derivative)
velocity = diff(position) ./ diff(time);
time_vel = time(1:end-1); % Adjust time vector

% Sampling rate estimation
Ts = mean(diff(time));   % Approximate sampling time
Fs = 1 / Ts;             % Sampling frequency

% Design Butterworth Low-Pass Filter (10 Hz cutoff)
cutoff_freq = 10; % Hz
order = 4; % 4th-order filter for a smooth response
[b, a] = butter(order, cutoff_freq / (Fs / 2), 'low'); % Normalize cutoff freq

% Apply filter to velocity signal
velocity_filtered = filtfilt(b, a, velocity); % Zero-phase filtering

% Compute acceleration (second derivative of filtered velocity)
acceleration = diff(velocity_filtered) ./ diff(time_vel);
time_acc = time_vel(1:end-1); % Adjust time vector

% Compute FFT of filtered velocity
N = length(velocity_filtered); % Number of samples
fft_velocity = fft(velocity_filtered); % Compute FFT
frequencies = (0:N-1) * (Fs/N); % Frequency axis

% Keep only first half (positive frequencies)
half_N = floor(N/2);
fft_velocity_mag = abs(fft_velocity(1:half_N)); % Magnitude of FFT
frequencies = frequencies(1:half_N); % Keep positive frequencies

% Plot Position, Raw vs. Filtered Velocity, Acceleration, and FFT
figure;

% Position vs Time
subplot(4,1,1);
plot(time, position, 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Position');
title('Position vs. Time');
grid on;

% Velocity vs Time (Raw and Filtered)
subplot(4,1,2);
plot(time_vel, velocity, 'r', 'LineWidth', 1.2); hold on;
plot(time_vel, velocity_filtered, 'w', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Velocity');
title('Velocity vs. Time (Raw vs. Filtered)');
legend('Raw', 'Filtered (10 Hz Low-pass)');
grid on;

% Acceleration vs Time
subplot(4,1,3);
plot(time_acc, acceleration, 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Acceleration');
title('Acceleration vs. Time');
grid on;

% FFT of Filtered Velocity
subplot(4,1,4);
plot(frequencies, fft_velocity_mag, 'w', 'LineWidth', 1.5);
xlabel('Frequency (Hz)');
ylabel('Magnitude');
title('FFT of Filtered Velocity Signal');
grid on;
xlim([1 Fs/2]); % Limit x-axis to Nyquist frequency