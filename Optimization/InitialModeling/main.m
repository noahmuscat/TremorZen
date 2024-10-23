% Modularized Main Script for TremorZen
% This script has been revised for proper parameter passing and scoping.

% Step 1: Sample IMU data (Replace with actual IMU data acquisition)
imuData = IMU_Processing(); % Add actual implementation for IMU data acquisition

% Step 2: Process IMU data using a band-pass filter
fs = 100; % Sampling frequency (ensure this aligns with your IMU sampling rate)
filteredSignal = IMU_Filter(imuData, fs);

% Step 3: Optimize PID parameters
[Kp_opt, Ki_opt, Kd_opt, costHistory] = optimizeParameters(filteredSignal);
disp(['Optimized PID Parameters: Kp: ', num2str(Kp_opt), ', Ki: ', num2str(Ki_opt), ', Kd: ', num2str(Kd_opt)]);

% Step 4: Generate control signal with optimized parameters
optimalControlSignal = PID_Control(filteredSignal, Kp_opt, Ki_opt, Kd_opt);

% Step 5: Visualization and Analysis
visualizeResults(imuData, filteredSignal, optimalControlSignal, costHistory, fs, Kp_opt, Ki_opt, Kd_opt);

% Functions ----------------------------------------------------------------------
% Function: visualizeResults
function visualizeResults(imuData, filteredSignal, optimalControlSignal, costHistory, fs, Kp_opt, Ki_opt, Kd_opt)
    figure;

    % Plot the raw IMU data and filtered signal
    subplot(4, 1, 1);
    plot((1:length(imuData))/fs, imuData, 'DisplayName', 'Raw IMU Data');
    hold on;
    plot((1:length(filteredSignal))/fs, filteredSignal, 'DisplayName', 'Filtered Signal (6-12 Hz)');
    title('Raw IMU Data and Filtered Signal');
    xlabel('Time (s)');
    ylabel('Amplitude');
    legend('show');
    hold off;

    % FFT Analysis: Plot the frequency content of the raw IMU data
    subplot(4, 1, 2);
    fft_plot(imuData, fs, 'Raw IMU Data FFT');

    % FFT Analysis: Plot the frequency content of the filtered signal
    subplot(4, 1, 3);
    fft_plot(filteredSignal, fs, 'Filtered Signal FFT');

    % Plot the optimized control signal
    subplot(4, 1, 4);
    plot((1:length(optimalControlSignal))/fs, optimalControlSignal);
    title('Optimized Control Signal');
    xlabel('Time (s)');
    ylabel('Control Signal Amplitude');

    % Plot the cost function history
    figure;
    plot(costHistory, '-o');
    title('Cost Function Value During Optimization');
    xlabel('Iteration');
    ylabel('Cost Function Value');

    % Additional Modeling and Analysis
    pidController = pid(Kp_opt, Ki_opt, Kd_opt);
    visualizeFurtherAnalysis(pidController, fs, Kp_opt, Ki_opt, Kd_opt);
end

% Function: visualizeFurtherAnalysis
function visualizeFurtherAnalysis(pidController, fs, Kp_opt, Ki_opt, Kd_opt)
    % Estimated Parameters for Mass-Spring-Damper System
    m = 0.5; % Mass (kg)
    b = 2.0; % Damping Coefficient (N·s/m)
    k = 20.0; % Spring Constant (N/m)

    % Define the Transfer Function for the Mass-Spring-Damper System
    s = tf('s');
    G = 1 / (m * s^2 + b * s + k);

    % Closed-loop system
    closedLoopSystem = feedback(pidController * G, 1);

    % Bode plot
    figure;
    bode(closedLoopSystem);
    title('Bode Plot of the Closed-Loop System');

    % Nyquist plot
    figure;
    nyquist(pidController * G);
    title('Nyquist Plot of the Open-Loop System');

    % Root Locus plot
    figure;
    rlocus(pidController * G);
    title('Root Locus of the Open-Loop System');

    % Step response
    figure;
    step(closedLoopSystem);
    title('Step Response of the Closed-Loop System');

    % Custom Simulation of Control Signal Accuracy and Device Stability
    simulationTime = 10; % Simulation time in seconds
    simulateDeviceResponse(closedLoopSystem, fs, simulationTime, Kp_opt, Ki_opt, Kd_opt);
end

% Function: simulateDeviceResponse
function simulateDeviceResponse(closedLoopSystem, fs, simulationTime, Kp_opt, Ki_opt, Kd_opt)
    % Use the parameters effectively within the function
    t = 0:1/fs:simulationTime;
    noise = 0.5 * randn(size(t)); % Random noise
    tremorSignal = sin(2*pi*8*t); % Example tremor signal at 8 Hz
    imuData = tremorSignal + noise;

    % Apply the band-pass filter
    filteredSignal = IMU_Filter(imuData, fs);

    % Generate optimized control signal
    optimalControlSignal = PID_Control(filteredSignal, Kp_opt, Ki_opt, Kd_opt);

    % Define the simulation of the device's response
    deviceResponse = lsim(closedLoopSystem, optimalControlSignal, t);

    % Plot the optimized control signal and device response
    figure;
    subplot(2, 1, 1);
    plot(t, optimalControlSignal);
    title('Optimized Control Signal');
    xlabel('Time (s)');
    ylabel('Control Signal Amplitude');

    subplot(2, 1, 2);
    plot(t, deviceResponse);
    title('Device Response to Control Signal');
    xlabel('Time (s)');
    ylabel('Device Response Amplitude');
end