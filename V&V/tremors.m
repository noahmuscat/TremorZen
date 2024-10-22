%% TremorZen Movement Quantification
% This code uses .csv files created using tremorActivity.py in order to
% analyze the movement of a tremor patient's hand

tremor_data = readtable("withtremor_actigraphy.csv");
notremor_data = readtable("withouttremor_actigraphy.csv");

% Filter out zero values
tremor_data_filtered = tremor_data.SelectedPixelDifference(tremor_data.SelectedPixelDifference ~= 0);
notremor_data_filtered = notremor_data.SelectedPixelDifference(notremor_data.SelectedPixelDifference ~= 0);

meanActivityTremor = mean(tremor_data_filtered);
meanActivityNoTremor = mean(notremor_data_filtered);

means = [meanActivityNoTremor, meanActivityTremor];

figure;
% bar graph for gross comparison
b1 = bar(means, 'BarWidth', 1);

title('Tremor vs. Non-Tremor Hand Movement');
xticks([1 2])
xticklabels({'No Tremor', 'Tremor'})
xlabel('Condition');
ylabel('Hand Movement');

% Plot histograms and Gaussian distributions
figure;

% Histogram for Tremor Data
hold on;

histogram(tremor_data_filtered, 'Normalization', 'pdf');
pd_tremor = fitdist(tremor_data_filtered, 'Normal');
x_vals_tremor = linspace(min(tremor_data_filtered), max(tremor_data_filtered), 100);
plot(x_vals_tremor, pdf(pd_tremor, x_vals_tremor), 'r', 'LineWidth', 2);

title('Histogram and Gaussian Fit - Tremor (Non-zero)');
xlabel('Selected Pixel Difference');
ylabel('Probability Density');
legend('Histogram', 'Gaussian Fit');
hold off;

% Histogram for Non-Tremor Data
figure;
hold on;

histogram(notremor_data_filtered, 'Normalization', 'pdf');
pd_notremor = fitdist(notremor_data_filtered, 'Normal');
x_vals_notremor = linspace(min(notremor_data_filtered), max(notremor_data_filtered), 100);

plot(x_vals_notremor, pdf(pd_notremor, x_vals_notremor), 'r', 'LineWidth', 2);
title('Histogram and Gaussian Fit - No Tremor (Non-zero)');
xlabel('Selected Pixel Difference');
ylabel('Probability Density');
legend('Histogram', 'Gaussian Fit');
hold off;