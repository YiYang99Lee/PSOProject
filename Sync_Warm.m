%{
clear all
clc
filename = 'Initiate Cold.xlsx';
sheet = 1;
xlRangex = 'D2:S227';
xlRangey = 'C2:C227';

x = xlsread(filename, sheet, xlRangex);
y = xlsread(filename, sheet, xlRangey);
z = exp(x+y) + randn(100,1)/100;
p = polyfitn([x,y],z,3);
%}

% Load your data
%clear all
%clc
%filename = 'Copy of Labelled Data.xlsx';
sheet = 1;
%xlRangex = 'D1211:AZ1436';
%xlRangey = 'C1211:C1436';
%xlRangetime = 'B1211:B1436';
xlRangex = 'C3:AO747';
xlRangey = 'B3:B747';
xlRangetime = 'B3:B747';
independent = xlsread('Sync Warm 1.xlsx',sheet, xlRangex);
dependent = xlsread('Sync Warm 1.xlsx',sheet, xlRangey);
timeline = 1:size(dependent);
%xlsread('Initiate Cold.xlsx',sheet, xlRangetime); 

% Define the degree of the polynomial you want to fit
degree = 1; % You can change this to the desired degree

% Fit the polynomial
coefficients = polyfitn(independent, dependent, degree);

% Evaluate the polynomial fit
yFit = polyvaln(coefficients, independent);

coef = coefficients.Coefficients;
varnum = size(coef,2);
coefficientPairs = cell(1, varnum);
for i = 1:varnum
    coefficientPairs{i} = sprintf('%.2f * x%d', coef(i), i);
end
coefficientString = strjoin(coefficientPairs, ' + ');
fprintf('y = %s\n', coefficientString);
curve=power;

RSS = sum(dependent-yFit).^2;
MeanError = (sum(dependent-yFit))/max_iterations;

% Plot the original data and the fitted curve
plot(timeline, dependent, '-r', timeline, yFit, '-b', timeline, power, '-g')
xlabel('Time');
ylabel('Y');
legend('Original Data', 'Fitted Curve', 'Optimized Curve');
