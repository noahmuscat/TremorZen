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
end