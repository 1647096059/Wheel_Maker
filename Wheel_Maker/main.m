% =========================================================================
% Whell_Maker四轮舵轮底盘性能评估主程序
% =========================================================================
clear; clc; close all;

fprintf('========== 舵轮底盘轮组设计选型与性能评估 ==========\n\n');

p = Config_Params.toStruct();

%% 1. 运动学正解：评估最高运动表现
R       = p.swerve_wheel_radius;
i_drive = p.swerve_i_drive;
Lx      = p.swerve_wheel_base_x / 2;
Ly      = p.swerve_wheel_base_y / 2;

max_wheel_v = (p.swerve_motor_max_rpm / i_drive) * (2*pi/60) * R;
max_spin_w  = max_wheel_v / hypot(Lx, Ly);

%% 2. 动力学逆解：评估扭矩需求

% A. 巡航工况，线加速度为 0，转向角加速度为 0
[Cruise_Drive_T, Cruise_Steer_T, ~] = ...
    Swerve_Dynamics_Core(0, 0, zeros(1, 4), p);

% B. 极限爆发工况：沿 45° 方向加速
a_limit = p.swerve_target_max_a;
ax_worst = a_limit * cos(pi/4);
ay_worst = a_limit * sin(pi/4);
alpha_vec = ones(1, 4) * p.swerve_target_max_alpha;

[Peak_Drive_T, Peak_Steer_T, Slip_Flags] = ...
    Swerve_Dynamics_Core(ax_worst, ay_worst, alpha_vec, p);

%% 3. 输出选型结论报告
fprintf('【1. 运动表现评估，基于物理边界】\n');
fprintf('  >> 理论最大直线速度 : %.2f m/s\n', max_wheel_v);
fprintf('  >> 理论最大自转速度 : %.2f rad/s，约 %.1f rpm\n\n', ...
    max_spin_w, max_spin_w * 60 / (2*pi));

fprintf('【2. 电机轴端扭矩需求，已折算减速比与 %.1fx 冗余】\n', p.swerve_redundancy);
fprintf('  >> 驱动电机 - 持续扭矩 : %.3f N·m\n', max(Cruise_Drive_T));
fprintf('  >> 驱动电机 - 峰值扭矩 : %.3f N·m，对应加速度 %.1f m/s²\n', ...
    max(Peak_Drive_T), a_limit);
fprintf('  >> 转向电机 - 持续扭矩 : %.3f N·m\n', max(Cruise_Steer_T));
fprintf('  >> 转向电机 - 峰值扭矩 : %.3f N·m，对应角加速度 %.1f rad/s²\n\n', ...
    max(Peak_Steer_T), p.swerve_target_max_alpha);

fprintf('【3. 抓地力安全评估】\n');
if any(Slip_Flags)
    fprintf('  [危险] 检测到车轮打滑！当前加速度指令超出了物理极限。\n');
    fprintf('  打滑轮编号 FL/FR/RR/RL: [%d %d %d %d]\n', Slip_Flags);
else
    fprintf('  [安全] 抓地力利用率正常，物理性能未达临界点。\n');
end

fprintf('========================================================\n');