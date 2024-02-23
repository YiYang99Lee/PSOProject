%clear all
clc

% PSO parameters
num_particles = 3;
max_iterations = 745;
w = 0.35;
c1 = 1.5;
c2 = 1.5;
target_value = 19.95;
IdealIter=0;
timedelay=500;

% Search space bounds
lb = [-657.51	-355.99	22.03	-394.13	8.70	-251.05	-119.95	2.91	99.92	2,997.75	23.07	35.21	2.21	835.22	100.51	438.35	22.08	8.56	0.00	0.00	448.72	245.56	0.00	0.00	371.00	179.28	157.13	13.77	2.66	3.91	0.00	410.31	433.44	374.82	396.32	32.48	34.35	27.34	752.91];
ub = [-293.75	-76.48	50.66	-135.51	11.85	-12.96	33.74	3.26	100.08	3,002.50	25.36	40.62	5.38	905.93	100.52	439.73	49.39	11.52	85.00	67.50	488.18	445.22	90.00	50.00	488.36	248.17	212.74	54.79	2.98	8.86	56.25	411.53	435.01	376.38	398.44	32.75	44.02	45.50	753.30];
VarCount=numel(ub);

% Initialize particles
x = zeros(num_particles, VarCount);
v = zeros(num_particles, VarCount);
for i = 1:num_particles
    x(i, :) = [-657.51	-180.03	22.03	-281.05	8.70	-250.32	-51.72	3.02	99.99	2,998.00	24.35	38.80	2.40	848.00	100.51	439.65	22.08	8.56	0.00	0.00	469.94	245.56	30.00	0.00	371.00	179.28	157.17	13.77	2.98	3.98	0.00	411.53	434.87	376.38	398.30	32.70	34.35	45.38	753.30];%lb + (ub - lb) .* rand(1, 10);
    v(i, :) = -1 + 2 .* rand(1, VarCount);
end

% Initialize personal best positions and costs
pbest_x = x;
pbest_cost = zeros(num_particles, 1);
for i = 1:num_particles
    pbest_cost(i) = objective(x(i, :));
end

% Initialize global best position and cost
[gbest_cost, gbest_index] = max(pbest_cost);
gbest_x = pbest_x(gbest_index, :);

% Run PSO
for iter = 1:max_iterations
    % Update particle velocities and positions
    for i = 1:num_particles
        r1 = rand(1, VarCount);
        r2 = rand(1, VarCount);
        v(i, :) = w * v(i, :) + c1 .* r1 .* (pbest_x(i, :) - x(i, :)) + c2 .* r2 .* (gbest_x - x(i, :));
        x(i, :) = x(i, :) + v(i, :);
        
        % Enforce search space bounds
        x(i, :) = min(x(i, :), ub);
        x(i, :) = max(x(i, :), lb);
    end
    
    % Update personal best positions and costs
    for i = 1:num_particles
        cost = objective(x(i, :));
        if abs(cost - target_value) < abs(pbest_cost - target_value)%cost < pbest_cost(i)
            pbest_x(i, :) = x(i, :);
            pbest_cost(i) = cost;
        end
    end
    
    % Update global best position and cost
    [min_cost, min_index] = max(pbest_cost);
    if abs(min_cost - target_value) < abs(gbest_cost - target_value)
        gbest_cost = min_cost;
        ideal_reached=false;
        if abs(gbest_cost - target_value) <= 0.1 && ideal_reached==false
            IdealIter=iter;
            ideal_reached=true;
        end
        gbest_x = pbest_x(min_index, :);
    end
    
    % Print current iteration and best cost
    fprintf('Iteration %d: Best cost = %.4f\n', iter, gbest_cost);
    %onetime = true;
    %if gbest_cost == target_value && onetime
        %fprintf('\nIdeal Power Reached in second: %d\n',iter);
        %onetime = false;
    %end
    disp(gbest_x);
    power(iter) = gbest_cost;
    %pause(1);
end

% Print final results
fprintf('\nFinal results:\n');
fprintf('Best parameters = [%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f]\n', gbest_x);
fprintf('Best cost = %.4f\n', gbest_cost);
timesaved = iter-IdealIter;


% Plot the desired and actual power output over time using the best parameters
t_desired = [0 10 20 30 40 50 60 70 80 90 100];
P_desired = [0 0.2 0.3 0.4 0.6 0.8 1.0 0.9 0.8 0.6 0.4];
t_actual = linspace(0, 100, 11);
P_actual = zeros(size(t_actual));
P_actual(t_actual >= gbest_x(1) & t_actual < gbest_x(2)) = (t_actual(t_actual >= gbest_x(1) & t_actual < gbest_x(2)) - gbest_x(1)) / (gbest_x(2) - gbest_x(1));
P_actual(t_actual >= gbest_x(2) & t_actual < gbest_x(3)) = 1;
P_actual(t_actual >= gbest_x(3) & t_actual < gbest_x(4)) = 1 - (t_actual(t_actual >= gbest_x(3) & t_actual < gbest_x(4)) - gbest_x(3)) / (gbest_x(4) - gbest_x(3));
P_actual(t_actual >= gbest_x(4) & t_actual < gbest_x(5)) = 0.5 * (1 + cos(pi * (t_actual(t_actual >= gbest_x(4) & t_actual < gbest_x(5))) / (gbest_x(5) - gbest_x(4))));
%figure;
%plot(t_desired, P_desired, 'bo-', 'LineWidth', 2, 'MarkerSize', 8);
%hold on;
%plot(t_actual, P_actual, 'r-', 'LineWidth', 2);
%xlabel('Time (s)');
%ylabel('Power output');
%legend('Desired', 'Actual');
%title('Power Generator Startup Routine Optimization');

%//////////////////////////////////////////////////////////////////////////
%Timetable
% 
% % Define number of steps and time interval
% num_steps = 10; % number of steps in the start-up routine
% TimeIntervals = [5, 2, 3, 4, 5, 6, 7, 8, 9, 10]; % Time intervals for each step (in minutes)
% Distance = [2 4 6];
% 
% % Initialize start-up routine timetable
% timetable = zeros(num_steps, 3); % first column is turbine speed, second column is temperature
% 
% % Step 1: GT Initiate Start
% for i = 1:TimeIntervals(1)
%     timetable(i,:) = [tnh_min, temp_min,1]; % maintain turbine speed and temperature during purging
% end
% %timetable(1,:) = [tnh_min, temp_min,1];
% 
% % Step 2: Start purging
% purge_time=TimeIntervals(2);
% %purge_time = 11; % time for purging process (in minutes)
% for i = i+1:i+purge_time
%     timetable(i,:) = [tnh_min, temp_min,2]; % maintain turbine speed and temperature during purging
% end
% 
% % Step 3: Introduce fuel and ignite
% timetable(i+1,:) = [tnh_min, temp_min,3]; % maintain turbine speed and temperature for warm-up
% timetable(i+2,:) = [tnh_min, temp_min,3]; % maintain turbine speed and temperature for warm-up
% timetable(i+3,:) = [tnh_min, temp_min,3]; % maintain turbine speed and temperature for warm-up
% timetable(i+4,:) = [tnh_min, temp_min,3]; % maintain turbine speed and temperature for warm-up
% timetable(i+5,:) = [tnh_min, temp_min,3]; % maintain turbine speed and temperature for warm-up
% 
% TurbineSpeedMin = 300; % minimum turbine speed (rpm)
% TurbineSpeedMax = 3000; % maximum turbine speed (rpm)
% TemperatureMin = 200; % minimum temperature (Celsius)
% TemperatureMax = 800; % maximum temperature (Celsius)
% 
% % Define optimization problem
% problem = optimproblem;
% N = numel(TimeIntervals);
% startTime = optimvar('startTime', [1,N], 'LowerBound', 1, 'UpperBound', num_steps);
% duration = optimvar('duration', [1,N], 'LowerBound', 0, 'UpperBound', TimeIntervals(end));
% turbineSpeed = optimvar('turbineSpeed', [1,N], 'LowerBound', TurbineSpeedMin, 'UpperBound', TurbineSpeedMax);
% temperature = optimvar('temperature', [1,N], 'LowerBound', TemperatureMin, 'UpperBound', TemperatureMax);
% 
% % Set initial conditions
% initialDuration = 30; % initial duration for first interval
% initialSpeed = TurbineSpeedMin; % initial turbine speed
% initialTemperature = TemperatureMin; % initial temperature
% problem.Objective = sum(duration); % minimize overall duration
% 
% % Set constraints
% problem.Constraints.StartTimes = startTime(2:end) >= startTime(1:end-1) + duration(1:end-1); % enforce sequential ordering of intervals
% problem.Constraints.SpeedConstraints = turbineSpeed >= initialSpeed; % initial speed constraint
% problem.Constraints.TemperatureConstraints = temperature >= initialTemperature; % initial temperature constraint
% problem.Constraints.FinalConstraints = [turbineSpeed(end) == TurbineSpeedMax, temperature(end) == TemperatureMax]; % final turbine speed and temperature constraints
% 
% % Define model equations as constraints
% for i = 1:N
%     %problem.Constraints.SpeedConstraints(i) = duration(i)*turbineSpeed(i) == Distance(i)*time_interval(i); % distance equation
%     %problem.Constraints.TemperatureConstraints(i+1) = temperature(i+1) == temperature(i) + (duration(i)/60)*HeatInput(i); % temperature equation
% end
% 
% % Solve optimization problem
% [sol, fval] = solve(problem);
% %startTime = extractfield(sol, 'turbineSpeed');
% %duration = extractfield(sol, 'duration');
% startTime = getfield(sol,'turbineSpeed');
% duration = getfield(sol,'duration');
% 
% % Display results
% disp('Start Time (min)  Duration (min)  Turbine Speed (rpm)  Temperature (Celsius)');
% for i = 1:N
%     disp([num2str(startTime(i)) '  ' num2str(duration(i)) '  ' num2str(sol.turbineSpeed(i)) '  ' num2str(sol.temperature(i))]);
% %end

%disp(' ');
%disp(timetable)

% Find the position of each particle where the best cost is achieved
best_positions = x;

% Display the positions
disp('Particle positions where best cost is achieved:');
disp(best_positions);

%Define the objective function
function cost = objective(x)
    %Define the desired power output over time
    x1 = x(:,1);
    x2 = x(:,2);
    x3 = x(:,3);
    x4 = x(:,4);
    x5 = x(:,5);
    x6 = x(:,6);
    x7 = x(:,7);
    x8 = x(:,8);
    x9 = x(:,9);
    x10 = x(:,10);
    x11 = x(:,11);
    x12 = x(:,12);
    x13 = x(:,13);
    x14 = x(:,14);
    x15 = x(:,15);
    x16= x(:,16);
    x17 = x(:,17);
    x18 = x(:,18);
    x19 = x(:,19);
    x20 = x(:,20);
    x21 = x(:,21);
    x22 = x(:,22);
    x23 = x(:,23);
    x24 = x(:,24);
    x25 = x(:,25);
    x26= x(:,26);
    x27= x(:,27);
    x28= x(:,28);
    x29= x(:,29);
    x30= x(:,30);
    x31= x(:,31);
    x32= x(:,32);
    x33= x(:,33);
    x34= x(:,34);
    x35= x(:,35);
    x36= x(:,36);
    x37= x(:,37);
    x38= x(:,38);
    x39= x(:,39);

    %measured variable
    %comPort = 'COM3';  % Specify the correct COM port for your setup
    %baudRate = 9600;   % Adjust the baud rate based on your sensor's specifications

    % Open serial port
    %s = serial(comPort, 'BaudRate', baudRate);
    %fopen(s);

    %try
    % Read data from the sensor (adjust based on your sensor's communication protocol)
        %fprintf(s, 'GETDATA');  % Example command to request data from the sensor
        %sensorData = fscanf(s, '%f');

    
    %Define the power output over time using the given startup routine parameters
    % process_duration_fuel_supply_pressure = @( P) 0.6*P + 5; % Linear equation: y = 0.6x + 5
    % process_duration_fuel_supply_pressure = @(P_fuel) (5 - P_fuel) / 0.6; % Inverse linear equation: y = (5 - x) / 0.6
    % %process_duration_steam_temperature = @(T) 0.4*T + 2; % Linear equation: y = 0.4x + 2
    % process_duration_steam_temperature = @(T_steam) (2 - T_steam) / 0.4; % Inverse linear equation: y = (2 - x) / 0.4
    % %process_duration_inlet_pressure = @(P) 0.8*P + 3; % Linear equation: y = 0.8x + 3
    % process_duration_inlet_pressure = @(P) (3 - P) / 0.8; % Inverse linear equation: y = (3 - x) / 0.8
    % process_duration_outlet_pressure = @(P) 0.3*P + 1; % Linear equation: y = 0.3x + 1
    % %process_duration_turbine_speed = @(S) 0.5*S + 4; % Linear equation: y = 0.5x + 4
    % process_duration_turbine_speed = @(S) (4 - S) / 0.5; % Inverse linear equation: y = (4 - x) / 0.5
    total_process_duration=@(x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26,x27,x28,x29,x30,x31,x32,x33,x34,x35,x36,x37,x38,x39) -0.00 * x1 + 0.01 * x2 + -0.29 * x3 + 0.00 * x4 + 0.85 * x5 + -0.03 * x6 + 0.02 * x7 + -15.07 * x8 + -1.91 * x9 + -0.63 * x10 + 0.19 * x11 + 0.79 * x12 + 1.49 * x13 + 0.13 * x14 + -5.98 * x15 + 0.09 * x16 + -0.02 * x17 + 2.65 * x18 + -0.07 * x19 + 0.10 * x20 + -0.28 * x21 + -0.04 * x22 + -0.00 * x23 + -0.01 * x24 + -0.00 * x25 + -0.01 * x26 + 0.27 * x27 + 0.24 * x28 + 8.06 * x29 + -0.08 * x30 + -0.00 * x31 + -4.36 * x32 + 0.15 * x33 + 0.72 * x34 + -0.14 * x35 + 0.59 * x36 + -0.24 * x37 + -0.07 * x38 + 0.07 * x39 + 4258.93;%4043.93;
    total_process_duration2=@(P_fuel2,T_steam2,P_inlet2,P_outlet2,Speed2) (((P_outlet2*P_inlet2)/T_steam2)/(2*P_fuel2))*Speed2+60;


    %Define the weightage of time of each parameter in each parameter
    % w1 = 0.2;
    % w2 = 0.3;
    % w3 = 0.1;
    % w4 = 0.2;
    % w5 = 0.2;
    wt = 0.06;
    wt2 = 0.3;

    %Calculate the cost as the integral of the squared difference between the desired and actual power output over time
    % duration_fuel_supply_pressure = process_duration_fuel_supply_pressure(P_fuel);
    % duration_steam_temperature = process_duration_steam_temperature(T_steam);
    % duration_inlet_pressure = process_duration_inlet_pressure(P_inlet);
    % duration_outlet_pressure = process_duration_outlet_pressure(P_outlet);
    % duration_turbine_speed = process_duration_turbine_speed(Speed);
    duration_total=total_process_duration(x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26,x27,x28,x29,x30,x31,x32,x33,x34,x35,x36,x37,x38,x39);
    %duration_total2=total_process_duration2(P_fuel2,T_steam2,P_inlet2,P_outlet2,Speed2);

    % Calculate the weighted sum of process durations
    cost =   duration_total;%+wt2*duration_total2;
    %        w1 * duration_fuel_supply_pressure + ...
    %        w2 * duration_steam_temperature + ...
    %        w3 * duration_inlet_pressure + ...
    %        w4 * duration_outlet_pressure + ...
    %        w5 * duration_turbine_speed;
end
