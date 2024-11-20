% Example data points
data = [9.2073
9.2359
9.2264]; % Replace x1, x2, x3 with your actual data points

% Define population mean for each test
mean_test1 = 12;
mean_test2 = 4;

% Perform first t-test (mean of data < 12)
[h1, p1, ci1, stats1] = ttest(data, mean_test1, 'Tail', 'left');

% Perform second t-test (mean of data > 4)
[h2, p2, ci2, stats2] = ttest(data, mean_test2, 'Tail', 'right');

% Display results
fprintf('Test 1 (mean < 12):\n');
fprintf('Hypothesis test result h1: %d\n', h1);
fprintf('p-value p1: %f\n', p1);
fprintf('Confidence interval ci1: [%f, %f]\n', ci1);
fprintf('Test statistic stats1.tstat: %f\n', stats1.tstat);
fprintf('Degrees of freedom stats1.df: %d\n', stats1.df);
fprintf('-----------------------------\n');

fprintf('Test 2 (mean > 4):\n');
fprintf('Hypothesis test result h2: %d\n', h2);
fprintf('p-value p2: %f\n', p2);
fprintf('Confidence interval ci2: [%f, %f]\n', ci2);
fprintf('Test statistic stats2.tstat: %f\n', stats2.tstat);
fprintf('Degrees of freedom stats2.df: %d\n', stats2.df);

% Interpret the results
if h1 == 1
    disp('The first test suggests that the mean of the data is less than 12.');
else
    disp('The first test does not suggest that the mean of the data is less than 12.');
end

if h2 == 1
    disp('The second test suggests that the mean of the data is greater than 4.');
else
    disp('The second test does not suggest that the mean of the data is greater than 4.');
end