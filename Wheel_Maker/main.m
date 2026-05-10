% =========================================================================
% RM 2026 四轮舵轮底盘性能评估主程序
% =========================================================================
clear; clc; close all;

fprintf('========== Wheel_Maker轮组设计选型与性能评估 ==========\n\n');

%% 1. 运动学正解：评估最高运动表现
R       = Config_Params.wheel_radius;
i_drive = Config_Params.i_drive;
Lx      = Config_Params.wheel_base_x / 2;
Ly      = Config_Params.wheel_base_y / 2;

% 计算单轮及底盘极限
max_wheel_v = (Config_Params.motor_max_rpm / i_drive) * (2*pi/60) * R;
max_spin_w  = max_wheel_v / sqrt(Lx^2 + Ly^2);

%% 2. 动力学逆解：评估扭矩需求
% A. 巡航工况 (a=0)
[Cruise_Drive_T, Cruise_Steer_T, ~] = Dynamics_Core(0, 0, [0,0,0,0]);

% B. 极限爆发工况 (最恶劣工况：沿对角线 45° 加速)
a_limit = Config_Params.target_max_a;
ax_worst = a_limit * cos(pi/4);
ay_worst = a_limit * sin(pi/4);
alpha_vec = ones(1,4) * Config_Params.target_max_alpha;

[Peak_Drive_T, Peak_Steer_T, Slip_Flags] = Dynamics_Core(ax_worst, ay_worst, alpha_vec);

%% 3. 输出选型结论报告
fprintf('【1. 运动表现表现 (基于物理边界)】\n');
fprintf('  >> 理论最大直线速度 : %.2f m/s\n', max_wheel_v);
fprintf('  >> 理论最大自转速度 : %.2f rad/s (约 %.1f rpm)\n\n', max_spin_w, max_spin_w * 60 / (2*pi));

fprintf('【2. 电机轴端扭矩需求 (已折算减速比与 %.1fx 冗余)】\n', Config_Params.redundancy_factor);
fprintf('  >> 驱动电机 - 持续扭矩 : %.3f N·m\n', max(Cruise_Drive_T));
fprintf('  >> 驱动电机 - 峰值扭矩 : %.3f N·m (对应加速度 %.1f m/s^2)\n', max(Peak_Drive_T), a_limit);
fprintf('  >> 转向电机 - 持续扭矩 : %.3f N·m\n', max(Cruise_Steer_T));
fprintf('  >> 转向电机 - 峰值扭矩 : %.3f N·m (对应角加速 %.1f rad/s^2)\n\n', max(Peak_Steer_T), Config_Params.target_max_alpha);

fprintf('【3. 抓地力安全评估】\n');
if any(Slip_Flags)
    fprintf('  [危险] 检测到车轮打滑！当前的加速度指令超出了物理极限。\n');
else
    fprintf('  [安全] 抓地力利用率正常，物理性能未达临界点。\n');
end
fprintf('========================================================\n');
