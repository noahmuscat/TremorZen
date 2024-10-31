% Main Script
% Parameters
fs = 100; % Sampling frequency, Hz
t = 0:1/fs:30; % 30 seconds of data for more thorough testing
tremorFreq = 8; % Example tremor frequency, Hz
noiseLevel = 0.5; % Amplitude of noise

% Generate synthetic tremor signal with multiple frequencies
syntheticTremor = sin(2*pi*tremorFreq*t);
extraFrequencies = sin(2*pi*3*t) + sin(2*pi*15*t) + sin(2*pi*20*t);
noise = noiseLevel * randn(size(t));
allFreq = syntheticTremor + extraFrequencies;
syntheticData = allFreq + noise; 

% Apply band-pass filter
[filteredSyntheticData, b, a] = IMU_Filter(syntheticData, fs);

% Step 1: Frequency Response Plot
[freqResponse, freq] = freqz(b, a, 4096, fs);
figure;
plot(freq, 20*log10(abs(freqResponse)));
ylabel('Magnitude (dB)');
xlabel('Frequency (Hz)');
title('Frequency Response of Band-Pass Filter');
grid on;
xlim([0 50]); % Limit x-axis to focus on relevant frequencies

% Step 2: Time-Domain Signal Plots
figure;
subplot(2, 1, 1);
plot(t, syntheticData);
title('Original Synthetic Data');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

subplot(2, 1, 2);
plot(t, filteredSyntheticData);
title('Filtered Synthetic Data');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

% Step 3: Power Spectral Density (PSD) Comparison
[psd_orig, freq_orig] = pwelch(syntheticData, [], [], [], fs);
[psd_filtered, freq_filtered] = pwelch(filteredSyntheticData, [], [], [], fs);

figure;
plot(freq_orig, 10*log10(psd_orig));
hold on;
plot(freq_filtered, 10*log10(psd_filtered));
legend('Original Data', 'Filtered Data');
xlabel('Frequency (Hz)');
ylabel('Power/Frequency (dB/Hz)');
title('Power Spectral Density Comparison');
grid on;

% Step 4: Statistical Analysis (Passband Gain)
passbandMask = (freq >= 6) & (freq <= 12);
passbandGain = 20*log10(abs(freqResponse(passbandMask)));

meanGain = mean(passbandGain);
stdGain = std(passbandGain);

fprintf('Passband Gain: Mean = %.2f dB, Std = %.2f dB\n', meanGain, stdGain);

% Save figures as files if required
%saveas(gcf, 'psd_comparison.png');

% Function for band-pass filter
function [filtered, b, a] = IMU_Filter(data, fs)
    % Band-pass filter between 6-12 Hz
    f_low = 6;
    f_high = 12;
    order = 4; % Increased filter order
    [b, a] = butter(order, [f_low, f_high] / (fs / 2), 'bandpass');
    filtered = filtfilt(b, a, data);
    
    % Export coefficients
    fprintf('float b[] = {');
    fprintf('%g, ', b);
    fprintf('};\n');
    
    fprintf('float a[] = {');
    fprintf('%g, ', a);
    fprintf('};\n');
end