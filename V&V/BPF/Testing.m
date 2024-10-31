%% Parameters
fs = 100;
numTrials = 60;
t = 0:1/fs:30;
tremorFreq = 8;
noiseLevel = 0.5;

% Storage for statistics
meanGains = zeros(numTrials, 1);
stdevGains = zeros(numTrials, 1);

for trial = 1:numTrials
    % Generate synthetic tremor signal with multiple frequencies
    syntheticTremor = sin(2*pi*tremorFreq*t);
    extraFrequencies = sin(2*pi*3*t) + sin(2*pi*15*t) + sin(2*pi*20*t);
    noise = noiseLevel * randn(size(t));
    allFreq = syntheticTremor + extraFrequencies;
    syntheticData = allFreq + noise; 

    % Apply the band-pass filter
    [filteredSyntheticData, b, a] = IMU_Filter(syntheticData, fs);

    % Frequency response plot
    [freqResponse, freq] = freqz(b, a, 4096, fs);

    % Statistical Analysis for Passband Gain
    passbandMask = (freq >= 6) & (freq <= 12);
    passbandGain = 20*log10(abs(freqResponse(passbandMask)));
    
    meanGains(trial) = mean(passbandGain);
    stdevGains(trial) = std(passbandGain);
end

% Statistical Testing
% T-test for mean gain
[h_mean, p_mean, ci_mean, stats_mean] = ttest(meanGains, 0, 'Alpha', 0.05);

fprintf('T-test for mean gain: h = %d, p = %.2f\n', h_mean, p_mean);
fprintf('Mean of means = %.2f dB\n', mean(meanGains));
fprintf('CI = [%.2f, %.2f] dB\n', ci_mean(1), ci_mean(2));

% ANOVA for standard deviation across trials
[p_stdev, tbl, stats_stdev] = anova1(stdevGains);

fprintf('ANOVA for standard deviation: p = %.2f\n', p_stdev);

% Function for band-pass filter
function [filtered, b, a] = IMU_Filter(data, fs)
    % Band-pass filter between 6-12 Hz
    f_low = 6;
    f_high = 12;
    order = 4; % Increased filter order
    [b, a] = butter(order, [f_low, f_high] / (fs / 2), 'bandpass');
    filtered = filtfilt(b, a, data);
end