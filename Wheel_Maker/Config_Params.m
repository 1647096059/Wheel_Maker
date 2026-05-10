classdef Config_Params
    % Config_Params: 多机型底盘通用参数字典

    properties (Constant)
        %% --- [公共常数] ---
        g = 9.81; % 重力加速度 (m/s²)

        %% --- [转向舵轮 Swerve] ---
        % 机器人几何与质量参数
        swerve_m_total      = 25.0;    % 全车重量 (kg)
        swerve_wheel_base_x = 0.2700;  % 前后轴距 (m)
        swerve_wheel_base_y = 0.2700;  % 左右轮距 (m)
        swerve_wheel_radius = 0.0425;  % 轮子半径 (m)
        swerve_wheel_width  = 0.030;   % 轮子宽度 (m)
        swerve_h_cog        = 0.2;     % 重心高度 (m)
        swerve_I_steer      = 0.015;   % 转向机构转动惯量 (kg·m²)

        % 电机与传动参数
        swerve_i_drive       = 1.0;    % 驱动减速比
        swerve_i_steer       = 1.0;    % 转向减速比
        swerve_motor_max_rpm = 450;    % 驱动电机最高转速 (RPM)
        swerve_steer_max_rpm = 120;    % 转向电机最高转速 (RPM)

        % 运动性能目标
        swerve_target_max_a     = 3.0;    % 最大线加速度 (m/s²)
        swerve_target_max_alpha = 20.0;   % 最大转向角加速度 (rad/s²)

        % 工程常数与环境参数
        swerve_mu_ground          = 0.8;    % 基准摩擦系数
        swerve_carpet_factor      = 4.0;    % 地胶阻力放大系数
        swerve_roll_resistance    = 0.018;  % 滚动阻力系数
        swerve_eta_slip           = 0.9;    % 滑移效率
        swerve_hardness_shoreA    = 60;     % 轮胎邵氏硬度
        swerve_T_mech_drive       = 0.1;    % 驱动机械损耗扭矩 (N·m)
        swerve_T_mech_steer       = 0.1;    % 转向机械损耗扭矩 (N·m)
        swerve_redundancy         = 1.2;    % 安全冗余系数

        %% --- [双轮轮腿 Wheel-Leg, 预留] ---
        wl_m_total = 20.0;
    end

    methods (Static)
        function p = toStruct()
            % 把 Constant properties 转成普通 struct，方便函数统一访问。
            names = properties('Config_Params');
            p = struct();

            for k = 1:numel(names)
                name = names{k};
                p.(name) = Config_Params.(name);
            end
        end

        function p = mergeWithDefaults(p_in)
            % 用默认参数补全外部传入的部分参数结构体。
            p = Config_Params.toStruct();

            if nargin < 1 || isempty(p_in)
                return;
            end

            if isa(p_in, 'Config_Params')
                return;
            end

            if isstruct(p_in)
                names = fieldnames(p_in);
            else
                names = properties(p_in);
            end

            for k = 1:numel(names)
                name = names{k};
                try
                    p.(name) = p_in.(name);
                catch
                    % 忽略无法读取的字段
                end
            end
        end
    end
end