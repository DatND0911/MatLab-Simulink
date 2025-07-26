function cost = simulate_quarter_car(pid_params)
    % Load system parameters
    run('params.m');

    % PID parameters
    Kp = pid_params(1);
    Ki = pid_params(2);
    Kd = pid_params(3);

    % Avoid improper transfer function
    if Ki == 0 && Kd ~= 0
        cost = 1e6;  % Penalize improper controller
        return;
    end

    s = tf('s');
    PID = Kp + Ki/s + Kd*s;  % PID as Transfer Function

    % State-space matrices for quarter car suspension
    A = [0          1             0             0;
        -k1/m1   -c1/m1       k1/m1        c1/m1;
         0          0             0             1;
        k1/m2    c1/m2   -(k1+k2)/m2   -((c1+c2)/m2)];

    B1 = [0; 0; 0; k2/m2];   % Road disturbance (W)
    B2 = [0; 1/m1; 0; -1/m2]; % Control input (U)

    C = [1 0 0 0];  % Output: sprung mass displacement
    D1 = 0; D2 = 0;

    sys = ss(A, [B1 B2], C, [D1 D2]);

    % Simulation settings
    t = 0:0.01:2;
    W = 0.1 * ones(size(t));           % 10 cm step road input
    e = -lsim(sys(:,1), W, t);         % error from road disturbance
    U = lsim(PID, e, t);               % control signal
    y = lsim(sys, [W' U], t);          % system response under control

    % Cost function: sum of squared displacement
    cost = sum(y.^2);  % smaller is better
end
