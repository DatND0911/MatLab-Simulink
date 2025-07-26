clc;
clear;

lb = [0, 0, 0];     % Lower bounds for [Kp, Ki, Kd]
ub = [2000, 2000, 1000]; % Upper bounds for [Kp, Ki, Kd]

options = optimoptions('ga', ...
    'Display', 'iter', ...
    'PopulationSize', 30, ...
    'MaxGenerations', 20);

[x, fval] = ga(@ObjectiveFunction, 3, [], [], [], [], lb, ub, [], options);

fprintf('Kp = %.3f, Ki = %.3f, Kd = %.3f\n', x(1), x(2), x(3));
fprintf('Minimum cost = %.4f\n', fval);
