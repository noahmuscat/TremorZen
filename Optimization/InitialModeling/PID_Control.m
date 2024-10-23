function controlSignal = PID_Control(filteredSignal, Kp, Ki, Kd)
    controlSignal = zeros(size(filteredSignal)); % Pre-allocate control signal array
    errorSum = 0;
    prevError = 0;
    alpha = 0.1; % Smoothing factor for derivative term (tune as needed)
    smoothedDerivative = 0;

    for i = 2:length(filteredSignal)
        error = -filteredSignal(i); % Goal is to cancel out the tremor
        errorSum = errorSum + error;
        derivative = (filteredSignal(i) - filteredSignal(i-1));
        smoothedDerivative = alpha * derivative + (1 - alpha) * smoothedDerivative;
        controlSignal(i) = Kp * error + Ki * errorSum + Kd * smoothedDerivative;
        prevError = error;
    end
end
