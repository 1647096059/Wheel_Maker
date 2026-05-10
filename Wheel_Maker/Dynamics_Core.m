function [Drive_Torque, Steer_Torque, Slip_Warning] = Dynamics_Core(ax, ay, alpha_steer)
    % Dynamics_Core: 动力学核心联合引擎
    
    %% 步骤一：材料建模与硬度转换
    s = Config_Params.hardness_shoreA;
    E_MPa = (0.0981 * (56 + 7.66 * s)) / (0.149 * (100 - s));
    E = E_MPa * 1e6; 
    
    %% 步骤二：载荷转移
    m = Config_Params.m_total;
    g = Config_Params.g;
    Fz_static = (m * g) / 4;
    delta_Fz_x = (m * ax * Config_Params.h_cog) / (2 * Config_Params.wheel_base_x);
    delta_Fz_y = (m * ay * Config_Params.h_cog) / (2 * Config_Params.wheel_base_y);
    
    Fz = [Fz_static - delta_Fz_x + delta_Fz_y, ...
          Fz_static - delta_Fz_x - delta_Fz_y, ...
          Fz_static + delta_Fz_x - delta_Fz_y, ...
          Fz_static + delta_Fz_x + delta_Fz_y];
    Fz(Fz < 0) = 0;
    
    %% 步骤三：合力需求与分配
    Fz_total = sum(Fz);
    if Fz_total == 0, Fz_total = eps; end
    Fx_total_req = m * ax;
    Fy_total_req = m * ay;
    
    Drive_Torque = zeros(1, 4);
    Steer_Torque = zeros(1, 4);
    Slip_Warning = zeros(1, 4);
    
    R = Config_Params.wheel_radius;
    w = Config_Params.wheel_width;
    mu = Config_Params.mu_ground;
    
    for i = 1:4
        if Fz(i) == 0, Slip_Warning(i) = 1; continue; end
        
        % 基础滚阻计算
        delta = Fz(i) / (E * w); 
        a = real(sqrt(2 * R * delta - delta^2)); 
        T_rolling_pure = Fz(i) * 0.05 * a; 
        
        % 驱动扭矩合成 (调用 Config 里的工程常数)
        Fx_i = Fx_total_req * (Fz(i) / Fz_total);
        T_drive_wheel = (T_rolling_pure * Config_Params.carpet_factor) + ...
                        Config_Params.T_mech_drive_wheel + abs(Fx_i) * R; 
        Drive_Torque(i) = (T_drive_wheel / Config_Params.i_drive) * Config_Params.redundancy_factor;
        
        % 转向扭矩合成 (调用 Config 里的工程常数)
        R_eq = sqrt(a^2 + (w/2)^2) / 1.5; 
        M_f = mu * Fz(i) * R_eq;
        T_steer_wheel = M_f + Config_Params.T_mech_steer_axis + ...
                        Config_Params.I_steer * abs(alpha_steer(i));
        Steer_Torque(i) = (T_steer_wheel / Config_Params.i_steer) * Config_Params.redundancy_factor;
        
        % 打滑校验
        if sqrt((Fx_total_req * (Fz(i) / Fz_total))^2 + (Fy_total_req * (Fz(i) / Fz_total))^2) > mu * Fz(i)
            Slip_Warning(i) = 1;
        end
    end
end