function imuData = IMU_Processing()
    % Simulate IMU data with more noise and additional frequency components
    fs = 100; % Sampling frequency in Hz
    t = 0:1/fs:10; % 10 seconds of data
    noise = 0.5 * randn(size(t)); % Increased random noise
    tremorSignal = sin(2*pi*8*t); % Primary tremor signal at 8 Hz
    additionalSignal = sin(2*pi*15*t) + sin(2*pi*3*t); % Additional frequencies (3 Hz, 15 Hz)
    imuData = tremorSignal + noise + additionalSignal; % Combined signal
end