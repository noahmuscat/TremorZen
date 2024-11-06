% Main script for TremorZen device focusing on tremor detection and electrical stimulation
% Incorporating synthetic data testing, metrics calculation, F1 score visualization, and figure saving

% Use synthetic data for initial testing
fs = 100; % Sampling frequency, Hz
t = 0:1/fs:30; % 30 seconds of data for more thorough testing
tremorFreq = 8; % Example tremor frequency, Hz
noiseLevel = 0.5; % Amplitude of noise

% Generate synthetic tremor signal with multiple frequencies
syntheticTremor = sin(2*pi*tremorFreq*t);
extraFrequencies = sin(2*pi*3*t) + sin(2*pi*15*t) + sin(2*pi*20*t);
noise = noiseLevel *randn(size(t));
allFreq = syntheticTremor + extraFrequencies;
syntheticData = allFreq + noise; 

% Step 1: Process synthetic data using a band-pass filter
filteredSyntheticData = IMU_Filter(syntheticData, fs);

% Step 2: Optimize Tremor Detection Parameters using filtered synthetic data
[threshold_opt, costHistory, f1Scores, thresholds] = optimizeDetectionParameters(filteredSyntheticData, fs);

% Display optimized threshold
disp(['Optimized Threshold: ', num2str(threshold_opt)]);

% Step 3: Generate and simulate control response with optimized parameters
[optimalControlSignal, stimulatedSignal] = simulateStimulus(filteredSyntheticData, threshold_opt, fs, t);

% Step 4: Validate tremor detection on synthetic data
groundTruth = abs(syntheticTremor) > 0.1; % Ground truth for synthetic tremor (tremor threshold=0.1)
detections = abs(filteredSyntheticData) > threshold_opt; % Tremor detections with optimized threshold

% Calculate performance metrics
TP = sum(detections == 1 & groundTruth == 1); % True Positives
FP = sum(detections == 1 & groundTruth == 0); % False Positives
TN = sum(detections == 0 & groundTruth == 0); % True Negatives
FN = sum(detections == 0 & groundTruth == 1); % False Negatives

accuracy = (TP + TN) / (TP + FP + TN + FN); % Accuracy
precision = TP / (TP + FP); % Precision
recall = TP / (TP + FN); % Recall
f1_score = 2 * (precision * recall) / (precision + recall); % F1 Score

% Display performance metrics
disp(['Accuracy: ', num2str(accuracy)]);
disp(['Precision: ', num2str(precision)]);
disp(['Recall: ', num2str(recall)]);
disp(['F1 Score: ', num2str(f1_score)]);

% Step 5: Plot and Save the results

%% Plot and save the raw and filtered synthetic data for comparison
figure;
subplot(2, 1, 1);
plot(t, syntheticData, 'DisplayName', 'Raw Synthetic Data');
hold on;
plot(t, syntheticTremor, 'DisplayName', 'Ground Truth Tremor (8 Hz)');
title('Raw Synthetic Data with Multiple Frequencies');
xlabel('Time (s)');
ylabel('Amplitude');
legend('show');
ylim([-5 5]);
grid on;
hold off;
%saveas(gcf, 'Raw_and_GroundTruth_SyntheticData.png'); % Save figure

subplot(2, 1, 2);
plot(t, filteredSyntheticData, 'DisplayName', 'Filtered Data (6-12 Hz)');
title('Filtered Synthetic Data');
xlabel('Time (s)');
ylabel('Amplitude');
legend('show');
ylim([-5 5]);
grid on;

% Set the desired resolution (dpi)
resolution = 300;
filename = 'asdf';

% Save the figure
print(gcf, filename, '-dpng', ['-r', num2str(resolution)]);
%saveas(gcf, 'Filtered_SyntheticData.png'); % Save figure

%% FFT Analysis: Compare the frequency content and save
figure;
subplot(2, 1, 1);
fft_plot(syntheticData, fs, 'Raw Synthetic Data FFT');
%saveas(gcf, 'Raw_SyntheticData_FFT.png'); % Save figure

subplot(2, 1, 2);
fft_plot(filteredSyntheticData, fs, 'Filtered Synthetic Data FFT');
%saveas(gcf, 'Filtered_SyntheticData_FFT.png'); % Save figure

% Plot and save the cost function history
figure;
plot(costHistory, '-o');
title('Cost Function Value During Optimization');
xlabel('Iteration');
ylabel('Cost Function Value');
grid on;
%saveas(gcf, 'Cost_Function_History.png'); % Save figure

% Plot and save the F1 score across different thresholds
figure;
plot(thresholds, f1Scores, '-o');
title('F1 Score vs. Threshold');
xlabel('Threshold');
ylabel('F1 Score');
grid on;
%saveas(gcf, 'F1_Score_vs_Threshold.png'); % Save figure

% Plot and save the optimized control signal and stimulated response
figure;
subplot(2, 1, 1);
plot((1:length(optimalControlSignal))/fs, optimalControlSignal, 'DisplayName', 'Control Signal');
title('Optimized Control Signal');
xlabel('Time (s)');
ylabel('Control Signal Amplitude');
legend('show');
grid on;

subplot(2, 1, 2);
plot((1:length(stimulatedSignal))/fs, stimulatedSignal, 'DisplayName', 'Stimulated Signal');
title('Electrical Stimulation Response');
xlabel('Time (s)');
ylabel('Electrical Stimulus Amplitude');
legend('show');
grid on;
%saveas(gcf, 'Optimized_Control_and_Stimulated_Signal.png'); % Save figure

% Plot and save the tremor detections
figure;
subplot(2, 1, 1);
plot(t, syntheticData, 'DisplayName', 'Synthetic Data');
hold on;
plot(t, filteredSyntheticData, 'DisplayName', 'Filtered Data');
title('Synthetic IMU Data');
xlabel('Time (s)');
ylabel('Amplitude');
legend('show');
grid on;
hold off;

subplot(2, 1, 2);
plot(t, detections, 'DisplayName', 'Detections');
title('Tremor Detections');
xlabel('Time (s)');
ylabel('Detection (0 or 1)');
legend('show');
grid on;
%saveas(gcf, 'Tremor_Detections.png'); % Save figure

%% Placeholder Functions

% Placeholder function for generating synthetic IMU data
function imuData = IMU_Processing()
    % Simulate IMU data with more noise and additional frequency components
    fs = 100; % Sampling frequency in Hz
    t = 0:1/fs:10; % 10 seconds of data
    noise = 0.5 * randn(size(t)); % Increased random noise
    tremorSignal = sin(2*pi*8*t); % Primary tremor signal at 8 Hz
    additionalSignal = sin(2*pi*15*t) + sin(2*pi*3*t); % Additional frequencies (3 Hz, 15 Hz)
    imuData = tremorSignal + noise + additionalSignal; % Combined signal
end

% Function for band-pass filter
function filtered = IMU_Filter(data, fs)
    % Band-pass filter between 6-12 Hz
    f_low = 6;
    f_high = 12;
    order = 4; % Increased filter order
    [b, a] = butter(order, [f_low, f_high] / (fs / 2), 'bandpass');
    filtered = filtfilt(b, a, data);
end

% Function for Optimization of Detection Parameters
function [threshold_opt, costHistory, f1Scores, thresholds] = optimizeDetectionParameters(signal, fs)
    % Define the range for threshold values to search
    thresholds = linspace(0.01, 1, 100);
    costHistory = zeros(size(thresholds));
    f1Scores = zeros(size(thresholds));
    
    % Generate ground truth for the synthetic signal
    groundTruth = abs(signal) > 0.1; % Use threshold=0.1 for synthetic ground truth
    
    % Iterate over each threshold and compute the cost and F1 score
    for i = 1:length(thresholds)
        threshold = thresholds(i);
        detections = abs(signal) > threshold;
        [f1, cost] = evaluatePerformance(groundTruth, detections);
        costHistory(i) = cost;
        f1Scores(i) = f1;
    end
    
    % Find the threshold that maximizes the F1 score
    [~, idx] = max(f1Scores);
    threshold_opt = thresholds(idx);
end

% Function to Evaluate Performance and Cost for Given Detections
function [f1_score, cost] = evaluatePerformance(groundTruth, detections)
    % Calculate performance metrics
    TP = sum(detections == 1 & groundTruth == 1);
    FP = sum(detections == 1 & groundTruth == 0);
    TN = sum(detections == 0 & groundTruth == 0);
    FN = sum(detections == 0 & groundTruth == 1);
    
    precision = TP / (TP + FP);
    recall = TP / (TP + FN);
    f1_score = 2 * (precision * recall) / (precision + recall);
    
    % Calculate cost as the negative of F1 score
    cost = -f1_score;
end

% Function to Simulate the Stimulus Based on Detection Parameters
function [controlSignal, stimulatedSignal] = simulateStimulus(signal, threshold, fs, t)
    % Generate binary control signal based on threshold detection
    controlSignal = abs(signal) > threshold;
    
    % Initialize stimulated signal
    stimulatedSignal = zeros(size(controlSignal));
    
    % Duration of stimulation in seconds
    stimulationDuration = 10;
    
    % Apply electrical stimulation for fixed duration after tremor detection
    stimulationSamples = round(stimulationDuration * fs);
    for i = 1:length(controlSignal)
        if controlSignal(i) == 1
            endIdx = min(i + stimulationSamples - 1, length(controlSignal));
            stimulatedSignal(i:endIdx) = 1;  % Apply stimulation
        end
    end
    
    % Modulate the control signal with the detected tremor signal
    stimulatedSignal = stimulatedSignal .* signal;
end

% Function to Plot FFT Analysis
function fft_plot(signal, fs, plotTitle)
    L = length(signal); % Length of signal
    Y = fft(signal); % Compute the FFT
    P2 = abs(Y/L); % Two-sided spectrum
    P1 = P2(1:floor(L/2)+1); % Single-sided spectrum
    P1(2:end-1) = 2*P1(2:end-1);
    f = fs*(0:(floor(L/2)))/L; % Frequency axis

    plot(f, P1);
    title(plotTitle);
    xlabel('Frequency (Hz)');
    ylabel('|Amplitude|');
    xlim([0 20]); % Focus on the frequency range of interest
    grid on;
end