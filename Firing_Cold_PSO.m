%clear all
clc

% PSO parameters
num_particles = 50;
max_iterations = 29;
w = 1;
c1 = 1.5;
c2 = 1.5;
target_value = -0.44;
IdealIter=0;

% Search space bounds
lb = [-749.43	-666.99	-470.77	-337.64	-327.18	13.20	396.25	-127.33	103.43	44.62	29.72	29.98	33.47	5.60	88.31	74.03	33.06	29.84	61.05];
ub = [-749.41	-663.67	-468.91	-337.60	-321.35	14.12	423.75	-125.17	103.54	45.42	29.78	30.03	33.53	9.24	88.51	74.41	33.09	30.16	63.60];
VarCount=numel(ub);

% Initialize particles
x = zeros(num_particles, VarCount);
v = zeros(num_particles, VarCount);
for i = 1:num_particles
    x(i, :) = [-749.43 -665.58 -469.70 -337.60 -323.90 14.12 423.75 -125.17 103.51 45.42 29.77 30.01 33.51 9.24 88.44 74.10 33.09 30.16 61.53];%lb + (ub - lb) .* rand(1, 10);
    v(i, :) = -1 + 2 .* rand(1, VarCount);
end

% Initialize personal best positions and costs
pbest_x = x;
pbest_cost = zeros(num_particles, 1);
for i = 1:num_particles
    pbest_cost(i) = objective(x(i, :));
end

% Initialize global best position and cost
[gbest_cost, gbest_index] = min(pbest_cost);
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
    [min_cost, min_index] = min(pbest_cost);
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

    total_process_duration=@(x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19) -26.20 * x1 + 0.00 * x2 + -0.39 * x3 + -5.69 * x4 + -0.16 * x5 + 0.29 * x6 + -0.03 * x7 + 0.18 * x8 + -6.42 * x9 + 1.65 * x10 + 5.06 * x11 + 4.50 * x12 + 1.10 * x13 + -0.06 * x14 + -0.78 * x15 + -0.37 * x16 + 3.37 * x17 + -0.29 * x18 + 0.19 * x19 + -21507.32;%-21505.42;
    total_process_duration2=@(P_fuel2,T_steam2,P_inlet2,P_outlet2,Speed2) (((P_outlet2*P_inlet2)/T_steam2)/(2*P_fuel2))*Speed2+60;


    wt = 0.06;
    wt2 = 0.3;

    duration_total=total_process_duration(x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19);

    % Calculate the weighted sum of process durations
    cost =   duration_total;%+wt2*duration_total2;
end