%% TremorZen Movement Quantification
% This code uses .csv files created using tremorActivity.py in order to
% analyze the movement of a tremor patient's hand

tremor_data = readtable("withtremor_actigraphy.csv");
notremor_data = readtable("withouttremor_actigraphy.csv");

meanActivityTremor = mean(tremor_data.SelectedPixelDifference);
meanActivityNoTremor = mean(notremor_data.SelectedPixelDifference);

means = [meanActivityNoTremor, meanActivityTremor];

figure;
b1 = bar(means, 'BarWidth', 1);
title('Tremor vs. Non-Tremor Hand Movement');
xticks([1 2])
xticklabels({'No Tremor', 'Tremor'})
xlabel('Condition');
ylabel('Hand Movement');

