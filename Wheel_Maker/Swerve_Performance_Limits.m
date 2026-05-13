function limits = Swerve_Performance_Limits(p)
% SWERVE_PERFORMANCE_LIMITS 舵轮底盘理论运动学边界计算
%
% 描述:
%   基于电机额定转速、减速比以及车体几何尺寸，计算底盘在理想不打滑状态下
%   所能达到的物理天花板。常用于判断控制算法下发的指令是否超出了硬件能力。
%
% 输入:
%   p - 参数结构体
%
% 输出:
%   limits.max_drive_speed - 理论最大平移直线速度 (m/s)
%   limits.max_spin_rate   - 理论最大原地自转角速度 (rad/s)
%   limits.max_steer_rate  - 理论最大舵轮偏转角速度 (rad/s)

    if nargin < 1 || isempty(p)
        p = Config_Params.toStruct();
    else
        p = Config_Params.mergeWithDefaults(p);
    end

    R = p.swerve_wheel_radius / 1000;
    Lx = (p.swerve_wheel_base_x / 1000) / 2;
    Ly = (p.swerve_wheel_base_y / 1000) / 2;

    % 1. 最大直线速度 = 轮端最高转速 * 轮半径
    limits.max_drive_speed = ...
        (p.swerve_motor_max_rpm / p.swerve_i_drive) * ...
        (2*pi/60) * R;

    % 2. 最大自转角速度 = 最大直线速度 / 轮组中心到车体质心的距离
    limits.max_spin_rate = ...
        limits.max_drive_speed / hypot(Lx, Ly);

    % 3. 最大转向角速度 = 转向电机最高转速
    limits.max_steer_rate = ...
        (p.swerve_steer_max_rpm / p.swerve_i_steer) * ...
        (2*pi/60);
end