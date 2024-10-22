function [Kp_opt, Ki_opt, Kd_opt, costHistory] = optimizeParameters(filteredSignal)
    global costHistory;
    costHistory = [];

    % Define the objective function for optimization as a nested function
    function cost = objectiveFunction(params)
        Kp = params(1);
        Ki = params(2);
        Kd = params(3);
        
        % Generate control signal with current PID params
        controlSignal = PID_Control(filteredSignal, Kp, Ki, Kd);
        
        % The cost function to minimize
        cost = sum(controlSignal.^2); % Minimize the energy of the control signal

        % Track cost history for analysis
        costHistory = [costHistory; cost];
    end

    % Initial guess for PID parameters
    initialParams = [0.1, 0.01, 0.1];
    
    % Set optimization options with bounds on parameters
    options = optimoptions('fmincon', 'OutputFcn', @outfun, 'Display', 'iter');
    lb = [0, 0, 0]; % Lower bounds for Kp, Ki, Kd
    ub = [10, 10, 10]; % Upper bounds for Kp, Ki, Kd
    
    % Run optimization to minimize the objective function (cost)
    optimalParams = fmincon(@objectiveFunction, initialParams, [], [], [], [], lb, ub, [], options);

    % Extract optimal PID parameters
    Kp_opt = optimalParams(1);
    Ki_opt = optimalParams(2);
    Kd_opt = optimalParams(3);
    
    % Display the optimized PID parameters
    disp('Optimized PID Parameters:');
    fprintf('Kp: %.4f, Ki: %.4f, Kd: %.4f\n', Kp_opt, Ki_opt, Kd_opt);
end

% Output function to capture each iteration's cost function value
function stop = outfun(~, optimValues, ~)
    stop = false; % Required for `optimoptions`
    global costHistory;
    % Add cost value to history
    if isfield(optimValues, 'fval')
        costHistory = [costHistory; optimValues.fval];
    end
end