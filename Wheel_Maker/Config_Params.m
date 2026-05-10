classdef Config_Params
    % CONFIG_PARAMS 多机型底盘通用参数字典
    %
    % 描述:
    %   统一管理仿真平台中所有的机械几何、质量分布、环境阻力与电气边界参数。
    %   采用 Constant 属性确保全局单一数据源 (Single Source of Truth)，
    %   并通过静态方法实现动态参数的合并与注入。

    properties (Constant)
        %% =================== [公共物理常数] ===================
        g = 9.81; % 重力加速度 (m/s²)

        %% ================= [转向舵轮 Swerve 专属参数] =================
        % --- 1. 机器人本体几何与质量分布 ---
        swerve_m_total      = 25.0;    % 全车总质量 (kg)
        swerve_wheel_base_x = 0.2700;  % 纵向轴距 (m)，即前后轮中心距
        swerve_wheel_base_y = 0.2700;  % 横向轮距 (m)，即左右轮中心距
        swerve_wheel_radius = 0.0425;  % 轮子半径 R (m)
        swerve_wheel_width  = 0.030;   % 轮子接地宽度 w (m)
        swerve_h_cog        = 0.2;     % 整车质心高度 (m)，决定加减速时的载荷转移剧烈程度
        swerve_I_steer      = 0.015;   % 单个转向机构(含轮子)绕Z轴的转动惯量 (kg·m²)

        % --- 2. 电机与传动系统物理边界 ---
        swerve_i_drive       = 1.0;    % 驱动轴减速比 (电机端转速 / 轮端转速)
        swerve_i_steer       = 1.0;    % 转向轴减速比 (电机端转速 / 舵角转速)
        swerve_motor_max_rpm = 450;    % 驱动电机额定最高转速 (RPM)
        swerve_steer_max_rpm = 120;    % 转向电机额定最高转速 (RPM)，决定舵角响应延迟

        % --- 3. 运动学/动力学性能目标 ---
        swerve_target_max_a     = 3.0;    % 底盘期望达到的最大平移加速度 (m/s²)
        swerve_target_max_alpha = 20.0;   % 舵向期望达到的最大瞬态角加速度 (rad/s²)

        % --- 4. 环境与摩擦学经验常数 ---
        swerve_mu_ground          = 0.8;    % 基准地面摩擦系数 (基于 60HA 硬度标定)
        swerve_carpet_factor      = 4.0;    % 地胶阻力放大系数 (模拟地毯绒毛带来的额外滚动阻力)
        swerve_roll_resistance    = 0.018;  % 基础滚动阻力系数
        swerve_eta_slip           = 0.9;    % 轮胎传动滑移效率 (填补微观滑动造成的做功损耗)
        swerve_hardness_shoreA    = 60;     % 轮胎包胶邵氏硬度 (Shore A)，影响接触面积与动态摩擦力
        swerve_T_mech_drive       = 0.1;    % 驱动轴端机械摩擦静态损耗扭矩 (N·m)
        swerve_T_mech_steer       = 0.1;    % 转向轴端机械摩擦静态损耗扭矩 (N·m)
        swerve_redundancy         = 1.2;    % 选型安全冗余系数 (实际扭矩需求 = 理论需求 * 冗余系数)

        %% ================= [双轮轮腿 Wheel-Leg (预留)] =================
        wl_m_total = 20.0; % 轮腿底盘预设总质量 (kg)
    end

    methods (Static)
        function p = toStruct()
            % TOSTRUCT 将 Constant 属性转换为普通结构体
            % 用途: 使得外部解算函数可以动态修改参数(如配合 UI 仪表盘)
            names = properties('Config_Params');
            p = struct();

            for k = 1:numel(names)
                name = names{k};
                p.(name) = Config_Params.(name);
            end
        end

        function p = mergeWithDefaults(p_in)
            % MERGEWITHDEFAULTS 参数合并与补全
            % 用途: 当 UI 仅传入部分被修改的参数时，自动从字典中补全缺失的字段
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
                    % 忽略无法读取的非法字段
                end
            end
        end
    end
end