function J = ObjectiveFunction(x)
    % Giải mã bộ PID từ vector GA
    Kp = x(1);
    Ki = x(2);
    Kd = x(3);

    % Kiểm tra giá trị PID không hợp lệ
    if any([Kp Ki Kd] < 0) || any([Kp Ki Kd] > 1e3)
        J = 1e6;
        return;
    end

    % Thông số vật lý
    m1 = 2500; m2 = 320;
    k1 = 80000; k2 = 500000;
    c1 = 350; c2 = 15020;

    % Gán vào workspace cho mô phỏng Simulink
    assignin('base', 'm1', m1);
    assignin('base', 'm2', m2);
    assignin('base', 'k1', k1);
    assignin('base', 'k2', k2);
    assignin('base', 'c1', c1);
    assignin('base', 'c2', c2);
    assignin('base', 'Kp', Kp);
    assignin('base', 'Ki', Ki);
    assignin('base', 'Kd', Kd);

    try
        % Mô phỏng mô hình quarter_model.slx
        simOut = sim('quarter_model', 'ReturnWorkspaceOutputs', 'on');
        y = simOut.get('yout');

        % Lấy tín hiệu z2 (hoặc sửa theo tên bạn log)
        z2_struct = y.get('z2');
        z2_data = z2_struct.Values.Data;

        % Hàm mục tiêu: ISE (integral of squared error)
        J = sum(z2_data .^ 2);

    catch
        J = 1e6; 
    end

    % Debug
    fprintf("Kp=%.2f, Ki=%.2f, Kd=%.2f => J=%.4f\n", Kp, Ki, Kd, J);
end
