%% TremorZen Movement Quantification
% this code uses .csv files created using tremorActivity.py in order to
% analyze the movement of a tremor patient's hand

tremor_data = readtable("withtremor_actigraphy.csv");
notremor_data = readtable("withouttremor_actigraphy.csv");

meanActivityTremor = mean(tremor_data.SelectedPixelDifference);
meanActivityNoTremor = mean(notremor_data.SelectedPixelDifference);

x_vals = [1, 2];
means = [meanActivityNoTremor, meanActivityTremor];

figure;
b1 = bar(x_vals, means, 'BarWidth', 1);
title('Tremor vs. Non-Tremor Hand Movement');

