function [Drive_Torque, Steer_Torque, Slip_Warning, debug] = Swerve_Dynamics_Core(ax, ay, alpha_steer, p)
% SWERVE_DYNAMICS_CORE 舵轮底盘动力学核心计算函数
%
% 输入:
%   ax          - 机器人纵向加速度，向前为正，单位 m/s²
%   ay          - 机器人横向加速度，向右为正，单位 m/s²
%   alpha_steer - 4 个轮子的转向角加速度，1x4 向量，单位 rad/s²
%   p           - 参数结构体，可选
%
% 输出:
%   Drive_Torque - 4 个轮子的驱动电机轴端扭矩，单位 N·m
%   Steer_Torque - 4 个轮子的转向电机轴端扭矩，单位 N·m
%   Slip_Warning - 打滑警告，1 表示打滑，0 表示正常
%   debug        - 调试信息结构体
%
% 轮序:
%   [FL, FR, RR, RL]
%   左前、右前、右后、左后

    if nargin < 4 || isempty(p)
        p = Config_Params.toStruct();
    else
        p = Config_Params.mergeWithDefaults(p);
    end

    %% 输入检查
    if ~isscalar(ax) || ~isscalar(ay)
        error('Swerve_Dynamics_Core:InputError', ...
            'ax 和 ay 必须是标量。');
    end

    if numel(alpha_steer) ~= 4
        error('Swerve_Dynamics_Core:InputError', ...
            'alpha_steer 必须包含 4 个元素，对应 [FL, FR, RR, RL]。');
    end

    alpha_steer = reshape(alpha_steer, 1, 4);

    %% 1. 材料与摩擦特性建模
    s = p.swerve_hardness_shoreA;

    if s <= 0 || s >= 100
        error('Swerve_Dynamics_Core:ParamError', ...
            'swerve_hardness_shoreA 必须在 0 到 100 之间。');
    end

    E = ((0.0981 * (56 + 7.66 * s)) / (0.149 * (100 - s))) * 1e6;

    s_ref = Config_Params.swerve_hardness_shoreA;
    E_ref = ((0.0981 * (56 + 7.66 * s_ref)) / (0.149 * (100 - s_ref))) * 1e6;

    mu = p.swerve_mu_ground * (E_ref / E)^(2/3);
    mu = min(max(mu, 0.1), 1.5);

    %% 2. 载荷转移计算
    m = p.swerve_m_total;
    g = p.g;

    Fz_static = m * g / 4;

    % 约定：
    % ax > 0: 向前加速，前轮法向载荷增加
    % ay > 0: 向右加速，右轮法向载荷增加
    dFz_x = (m * ax * p.swerve_h_cog) / (2 * p.swerve_wheel_base_x);
    dFz_y = (m * ay * p.swerve_h_cog) / (2 * p.swerve_wheel_base_y);

    % 轮序 [FL, FR, RR, RL]
    Fz = [
        Fz_static + dFz_x - dFz_y, ...
        Fz_static + dFz_x + dFz_y, ...
        Fz_static - dFz_x + dFz_y, ...
        Fz_static - dFz_x - dFz_y
    ];

    Fz = max(Fz, 0);

    Fz_total = sum(Fz);
    if Fz_total <= eps
        Fz_total = eps;
    end

    %% 3. 总力计算
    Fx_total = m * ax;
    Fy_total = m * ay;

    R = p.swerve_wheel_radius;
    w = p.swerve_wheel_width;

    Drive_Torque = zeros(1, 4);
    Steer_Torque = zeros(1, 4);
    Slip_Warning = zeros(1, 4);

    Fx_i_all = zeros(1, 4);
    Fy_i_all = zeros(1, 4);
    F_total_i_all = zeros(1, 4);
    muFz_all = zeros(1, 4);
    grip_usage_all = zeros(1, 4);

    %% 4. 逐轮计算
    for i = 1:4
        if Fz(i) <= eps
            Slip_Warning(i) = 1;
            continue;
        end

        % 按法向力比例分配纵向力和横向力
        Fx_i = Fx_total * Fz(i) / Fz_total;
        Fy_i = Fy_total * Fz(i) / Fz_total;
        F_total_i = hypot(Fx_i, Fy_i);

        Fx_i_all(i) = Fx_i;
        Fy_i_all(i) = Fy_i;
        F_total_i_all(i) = F_total_i;

        % 接触形变估算
        delta = Fz(i) / (E * w);
        contact_expr = max(0, 2 * R * delta - delta^2);
        contact_half_len = sqrt(contact_expr);

        % 驱动扭矩
        T_roll = Fz(i) * R * p.swerve_roll_resistance * p.swerve_carpet_factor;
        T_drv_wheel = T_roll + p.swerve_T_mech_drive + ...
            (F_total_i * R) / p.swerve_eta_slip;

        Drive_Torque(i) = (T_drv_wheel / p.swerve_i_drive) * p.swerve_redundancy;

        % 转向扭矩
        M_f = mu * Fz(i) * hypot(contact_half_len, w/2) / 1.5;
        T_str_wheel = M_f + p.swerve_T_mech_steer + ...
            p.swerve_I_steer * abs(alpha_steer(i));

        Steer_Torque(i) = (T_str_wheel / p.swerve_i_steer) * p.swerve_redundancy;

        % 打滑判断
        muFz = mu * Fz(i);
        muFz_all(i) = muFz;

        if muFz > eps
            grip_usage_all(i) = F_total_i / muFz;
        else
            grip_usage_all(i) = inf;
        end

        if F_total_i > muFz
            Slip_Warning(i) = 1;
        end
    end

    %% 5. 调试信息
    debug = struct();
    debug.mu = mu;
    debug.E = E;
    debug.Fz = Fz;
    debug.Fx_i = Fx_i_all;
    debug.Fy_i = Fy_i_all;
    debug.F_total_i = F_total_i_all;
    debug.muFz = muFz_all;
    debug.grip_usage = grip_usage_all;
end