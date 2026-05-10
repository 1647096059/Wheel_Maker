function [Drive_Torque, Steer_Torque, Slip_Warning, debug] = Swerve_Dynamics_Core(ax, ay, alpha_steer, p)
% SWERVE_DYNAMICS_CORE 舵轮底盘动力学核心计算函数
%
% 描述:
%   基于给定的底盘加速度指令，解算由于载荷转移造成的各轮法向压力分布，
%   并结合轮胎的微观形变(赫兹接触)计算电机所需的驱动扭矩与转向扭矩，同时进行摩擦圆打滑校验。
%
% 坐标与物理定义:
%   X 向前为正，Y 向右为正，Z 向上为正。
%   ax: 机器人纵向真实加速度 (m/s²)。ax > 0 时向前加速，重心后移。
%   ay: 机器人横向真实加速度 (m/s²)。ay > 0 时向右加速，重心左移。
%
% 轮序分配:
%   1:FL(左前), 2:FR(右前), 3:RR(右后), 4:RL(左后)
%
% 返回值:
%   Drive_Torque - 1x4，各轮驱动电机所需轴端扭矩 (N·m)
%   Steer_Torque - 1x4，各轮转向电机所需轴端扭矩 (N·m)
%   Slip_Warning - 1x4，打滑标志位 (1 表示超出物理抓地极限)
%   debug        - 包含微观物理过程的详细结构体，供上层可视化调用

    %% 0. 参数解析与安全拦截
    if nargin < 4 || isempty(p)
        p = Config_Params.toStruct();
    else
        p = Config_Params.mergeWithDefaults(p);
    end

    % (输入合法性检查省略，逻辑保持不变)
    if ~isscalar(ax) || ~isscalar(ay), error('Swerve_Dynamics_Core:InputError', 'ax 和 ay 必须是标量。'); end
    if ~isfinite(ax) || ~isfinite(ay), error('Swerve_Dynamics_Core:InputError', 'ax 和 ay 必须是有限数值。'); end
    if numel(alpha_steer) ~= 4, error('Swerve_Dynamics_Core:InputError', 'alpha_steer 必须包含 4 个元素。'); end
    alpha_steer = reshape(alpha_steer, 1, 4);
    if any(~isfinite(alpha_steer)), error('Swerve_Dynamics_Core:InputError', 'alpha_steer 不能包含 NaN/Inf。'); end

    validatePositiveParam(p.swerve_m_total, 'swerve_m_total');
    validatePositiveParam(p.g, 'g');
    validatePositiveParam(p.swerve_wheel_base_x, 'swerve_wheel_base_x');
    validatePositiveParam(p.swerve_wheel_base_y, 'swerve_wheel_base_y');
    validatePositiveParam(p.swerve_wheel_radius, 'swerve_wheel_radius');
    validatePositiveParam(p.swerve_wheel_width, 'swerve_wheel_width');
    validatePositiveParam(p.swerve_i_drive, 'swerve_i_drive');
    validatePositiveParam(p.swerve_i_steer, 'swerve_i_steer');
    validatePositiveParam(p.swerve_eta_slip, 'swerve_eta_slip');
    validatePositiveParam(p.swerve_redundancy, 'swerve_redundancy');
    if p.swerve_h_cog < 0, error('Swerve_Dynamics_Core:ParamError', 'swerve_h_cog 不应为负数。'); end

    %% 1. 材料学建模与赫兹接触理论衰减
    s = p.swerve_hardness_shoreA;
    if s <= 0 || s >= 100
        error('Swerve_Dynamics_Core:ParamError', '邵氏硬度必须在 0 到 100 之间。');
    end

    % 使用 Gent 经验公式将邵氏硬度 (Shore A) 转换为杨氏模量 E (Pa)
    E = ((0.0981 * (56 + 7.66 * s)) / (0.149 * (100 - s))) * 1e6;

    % 获取基准状态下的杨氏模量 E_ref，作为衰减对照组
    s_ref = Config_Params.swerve_hardness_shoreA;
    E_ref = ((0.0981 * (56 + 7.66 * s_ref)) / (0.149 * (100 - s_ref))) * 1e6;

    % 基于赫兹接触理论的幂定律衰减公式：真实接触面积正比于 E^(-2/3)
    % 当轮胎变硬时，摩擦系数 mu 会发生非线性滑坡
    mu = p.swerve_mu_ground * (E_ref / E)^(2/3);
    mu = min(max(mu, 0.1), 1.5); % 物理阈值截断，防止数据爆炸

    %% 2. 宏观动力学：载荷转移 (Load Transfer) 计算
    m = p.swerve_m_total;
    g = p.g;
    Lx_full = p.swerve_wheel_base_x;
    Ly_full = p.swerve_wheel_base_y;
    h       = p.swerve_h_cog;

    % 静态均摊法向压力
    Fz_static = m * g / 4;

    % 计算由力矩平衡引起的载荷增量分布
    dFz_x = (m * ax * h) / (2 * Lx_full); % 纵向转移量
    dFz_y = (m * ay * h) / (2 * Ly_full); % 横向转移量

    % 分配至四轮 (依据叠加定理)
    Fz_unclipped = [
        Fz_static - dFz_x + dFz_y, ... % FL: 左前轮 (纵向减载，横向增载)
        Fz_static - dFz_x - dFz_y, ... % FR: 右前轮 (纵向减载，横向减载)
        Fz_static + dFz_x - dFz_y, ... % RR: 右后轮 (纵向增载，横向减载)
        Fz_static + dFz_x + dFz_y      % RL: 左后轮 (纵向增载，横向增载)
    ];

    % 剔除负压力 (模拟极限过弯或急刹时某轮"翘起"离地的物理现象)
    Fz = max(Fz_unclipped, 0);
    Fz_total = sum(Fz);
    if Fz_total <= eps
        Fz_total = eps;
    end

    %% 3. 整车合力需求计算
    Fx_total = m * ax;
    Fy_total = m * ay;
    R = p.swerve_wheel_radius;
    w = p.swerve_wheel_width;

    % 预分配内存，提升循环执行效率
    Drive_Torque = zeros(1, 4);
    Steer_Torque = zeros(1, 4);
    Slip_Warning = zeros(1, 4);
    Fx_i_all = zeros(1, 4);
    Fy_i_all = zeros(1, 4);
    F_total_i_all = zeros(1, 4);
    muFz_all = zeros(1, 4);
    grip_usage_all = zeros(1, 4);
    delta_all = zeros(1, 4);
    contact_half_len_all = zeros(1, 4);

    %% 4. 微观物理学解算 (逐轮独立计算)
    for i = 1:4
        % 处于离地状态的轮子直接失去抓地力并跳过计算
        if Fz(i) <= eps
            Slip_Warning(i) = 1;
            continue;
        end

        % 简化分配策略：按照此时刻各轮承担的法向载荷比例，均摊底盘的纵/横推力需求
        Fx_i = Fx_total * Fz(i) / Fz_total;
        Fy_i = Fy_total * Fz(i) / Fz_total;
        F_total_i = hypot(Fx_i, Fy_i);

        Fx_i_all(i) = Fx_i;
        Fy_i_all(i) = Fy_i;
        F_total_i_all(i) = F_total_i;

        % --- [核心] 计算轮胎压入深度(delta)与接触面半长(contact_half_len) ---
        delta = Fz(i) / (E * w);
        contact_expr = max(0, 2 * R * delta - delta^2);
        contact_half_len = sqrt(contact_expr);

        delta_all(i) = delta;
        contact_half_len_all(i) = contact_half_len;

        % --- [扭矩估算 1] 驱动扭矩计算 ---
        % 滚动阻力(T_roll)正比于接触面产生的形变迟滞损耗
        T_roll = Fz(i) * R * p.swerve_roll_resistance * p.swerve_carpet_factor;

        % 驱动总力矩 = 滚阻 + 静态机械损耗 + 惯性推力(经由滑移效率放大)
        T_drv_wheel = T_roll + ...
            p.swerve_T_mech_drive + ...
            (F_total_i * R) / p.swerve_eta_slip;

        Drive_Torque(i) = (T_drv_wheel / p.swerve_i_drive) * p.swerve_redundancy;

        % --- [扭矩估算 2] 转向扭矩计算 ---
        % 原地转向产生的摩擦阻力矩(M_f)，考虑接触面分布积分效应
        M_f = mu * Fz(i) * hypot(contact_half_len, w/2) / 1.5;

        % 转向总力矩 = 摩擦阻力矩 + 静态机械损耗 + 加速旋转产生的惯性矩
        T_str_wheel = M_f + ...
            p.swerve_T_mech_steer + ...
            p.swerve_I_steer * abs(alpha_steer(i));

        Steer_Torque(i) = (T_str_wheel / p.swerve_i_steer) * p.swerve_redundancy;

        % --- [状态监测] 摩擦圆打滑检验 ---
        muFz = mu * Fz(i);
        muFz_all(i) = muFz;

        if muFz > eps
            grip_usage_all(i) = F_total_i / muFz;
        else
            grip_usage_all(i) = inf;
        end

        % 如果所需抓地力超越了物理极限，触发红色预警
        if F_total_i > muFz
            Slip_Warning(i) = 1;
        end
    end

    %% 5. 组装调试字典 (Debug Info)
    debug = struct();
    debug.coord_convention = 'X forward positive, Y right positive, ax/ay are real body-frame accelerations';
    debug.wheel_order = '[FL, FR, RR, RL]';
    debug.mu = mu;
    debug.E = E;
    debug.Fz_static = Fz_static;
    debug.dFz_x_per_wheel = dFz_x;
    debug.dFz_y_per_wheel = dFz_y;
    debug.Fz_unclipped = Fz_unclipped;
    debug.Fz = Fz;
    debug.Fx_total = Fx_total;
    debug.Fy_total = Fy_total;
    debug.Fx_i = Fx_i_all;
    debug.Fy_i = Fy_i_all;
    debug.F_total_i = F_total_i_all;
    debug.muFz = muFz_all;
    debug.grip_usage = grip_usage_all;
    debug.delta = delta_all;
    debug.contact_half_len = contact_half_len_all;
end

% 本地辅助校验函数，防止传入脏数据
function validatePositiveParam(value, name)
    if ~isscalar(value) || ~isfinite(value) || value <= 0
        error('Swerve_Dynamics_Core:ParamError', '%s 必须是正的有限标量。', name);
    end
end