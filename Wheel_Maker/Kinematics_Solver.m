function [v_drive_out, theta_steer_out, omega_steer_out] = Kinematics_Solver(vx, vy, omega_z, current_theta, dt)
    % 运动学解算内核
    % 输入: 目标速度组合 (vx, vy, omega_z), 当前各轮偏转角 current_theta (1x4), 步长 dt
    % 输出: 各轮目标驱动线速度, 目标偏转角, 目标转向角速度
    
    Lx = Config_Params.wheel_base_x / 2;
    Ly = Config_Params.wheel_base_y / 2;
    
    % 四个轮子的几何位置矩阵 (右上1, 左上2, 左下3, 右下4)
    pos_matrix = [ Lx,  Ly;
                  -Lx,  Ly;
                  -Lx, -Ly;
                   Lx, -Ly];
               
    v_drive_out = zeros(1, 4);
    theta_steer_out = zeros(1, 4);
    omega_steer_out = zeros(1, 4);
    
    for i = 1:4
        % 1. 运动学逆解
        v_ix = vx - omega_z * pos_matrix(i, 2);
        v_iy = vy + omega_z * pos_matrix(i, 1);
        
        v_target = sqrt(v_ix^2 + v_iy^2);
        theta_target = atan2(v_iy, v_ix); % 范围 [-pi, pi]
        
        % 2. 最优转向逻辑 (限制 delta_theta 不超过 90 度)
        delta_theta = theta_target - current_theta(i);
        
        % 角度归一化到 [-pi, pi]
        delta_theta = wrapToPi(delta_theta); 
        
        if abs(delta_theta) > pi/2
            % 如果所需转角大于 90度，则反向旋转并翻转驱动轮方向
            theta_target = wrapToPi(theta_target - sign(delta_theta)*pi);
            v_target = -v_target; % 驱动电机速度反向
            delta_theta = wrapToPi(theta_target - current_theta(i));
        end
        
        % 3. 数据输出与舵向角速度计算
        v_drive_out(i) = v_target;
        theta_steer_out(i) = theta_target;
        omega_steer_out(i) = delta_theta / dt; % 瞬态舵向角速度 \omega_steer_i
    end
end