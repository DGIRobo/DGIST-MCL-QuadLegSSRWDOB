clear all
clc
close all
system('./run_linux')

filename = './data/data.csv';

T = readtable(filename);
VariableNames = T.Properties.VariableNames;

Arr = table2array(T);
[m,n] = size(Arr);

figure(1);
subplot(2,2,1);
plot(Arr(:,1), Arr(:,2), 'Color', 'b', 'LineWidth', 3);
hold on;
plot(Arr(:,1), Arr(:,3), 'Color', 'm', 'LineWidth', 3);
xlim([0.2 1.2]);
xlabel('time (sec)');
ylabel('r-direction Ground Reaction Force (N)');
legend('real', 'FOB')

subplot(2,2,2);
plot(Arr(:,1), Arr(:,3)-Arr(:,2), 'Color', 'c', 'LineWidth', 3);
xlim([0.2 1.2]);
xlabel('time (sec)');
ylabel('r-direction Ground Reaction Force (N)');
legend('FOB error')

subplot(2,2,3);
plot(Arr(:,1), Arr(:,4), 'Color', 'b', 'LineWidth', 3);
hold on;
plot(Arr(:,1), Arr(:,5), 'Color', 'r', 'LineWidth', 3);
xlim([0.2 1.2]);
xlabel('time (sec)');
ylabel('r-direction (m)');
legend('reference', 'real')

subplot(2,2,4)
plot(Arr(:,1), Arr(:,6), 'Color', 'b', 'LineWidth', 3);
hold on;
plot(Arr(:,1), Arr(:,7), 'Color', 'r', 'LineWidth', 3);
xlim([0.2 1.2]);
xlabel('time (sec)');
ylabel('theta-direction (rad)');
legend('reference', 'real')