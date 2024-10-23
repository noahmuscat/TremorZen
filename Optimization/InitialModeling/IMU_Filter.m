function filteredSignal = IMU_Filter(imuData, fs)
    % Design a band-pass filter
    fpass = [6 12]; % Passband frequencies for tremor detection
    [b, a] = butter(2, fpass/(fs/2), 'bandpass');
    % Apply the filter to the IMU data
    filteredSignal = filtfilt(b, a, imuData);
end