sheet = 1;
xlRangex = 'C3:U31';
xlRangey = 'B3:B31';
xlRangetime = 'B2:B227';
independent = xlsread('Firing Cold.xlsx',sheet, xlRangex);
dependent = xlsread('Firing Cold.xlsx',sheet, xlRangey);
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

