% =========================================================================
% Wheel_Maker 四轮舵轮底盘性能评估命令行主程序 (无 UI 版)
% 描述:
%   自动拉取默认配置，进行运动学极值、静态载荷、巡航稳态及极限爆发四个工况
%   的完整解算，并将结果规范化打印至控制台。适合用于自动化选型核算。
% =========================================================================
clear; clc; close all;

fprintf('========== RM2026 舵轮底盘轮组设计选型与性能评估 ==========\n\n');

%% 0. 依赖检查
assertDependency('Config_Params');
assertDependency('Swerve_Get_Preset');
assertDependency('Swerve_Performance_Limits');
assertDependency('Swerve_Kinematics_Solver');
assertDependency('Swerve_Dynamics_Core');
assertDependency('Swerve_Evaluate_Case');

%% 1. 载入默认预设
presetName = 'Swerve 四轮舵轮 - 默认';
p = Swerve_Get_Preset(presetName);

wheelNames = ["FL", "FR", "RR", "RL"];

fprintf('当前机器人预设: %s\n', presetName);
fprintf('轮序: [FL(左前), FR(右前), RR(右后), RL(左后)]\n');
fprintf('坐标约定: X 向前为正, Y 向右为正, omega_z 俯视逆时针为正\n');
fprintf('加速度定义: ax / ay 为机器人本体系下真实加速度\n\n');

%% 2. 理论性能边界计算
limits = Swerve_Performance_Limits(p);

fprintf('【1. 运动表现评估，基于物理理论边界】\n');
fprintf('  >> 理论最大直线速度 : %.3f m/s\n', limits.max_drive_speed);
fprintf('  >> 理论最大自转速度 : %.3f rad/s，约 %.1f rpm\n', ...
    limits.max_spin_rate, limits.max_spin_rate * 60 / (2*pi));
fprintf('  >> 理论最大转向角速度 : %.3f rad/s，约 %.1f rpm\n\n', ...
    limits.max_steer_rate, limits.max_steer_rate * 60 / (2*pi));

%% 3. 工况 A：静态承载工况 (全静止)
caseStatic = makeCase(0.0, 0.0, 0.0, 0.0, 0.0, 0.02, zeros(1, 4), zeros(1, 4));
outStatic = Swerve_Evaluate_Case(caseStatic, p);

%% 4. 工况 B：高速巡航工况 (无加速度，匀速前进)
cruiseVx = min(1.0, limits.max_drive_speed);
caseCruise = makeCase(cruiseVx, 0.0, 0.0, 0.0, 0.0, 0.02, zeros(1, 4), zeros(1, 4));
outCruise = Swerve_Evaluate_Case(caseCruise, p);

%% 5. 工况 C：极限爆发工况 (沿 45° 对角线满功率加速)
a_limit = p.swerve_target_max_a;
ax_worst = a_limit * cos(pi/4);
ay_worst = a_limit * sin(pi/4);
casePeak = makeCase(0.0, 0.0, 0.0, ax_worst, ay_worst, 0.02, zeros(1, 4), ones(1, 4) * p.swerve_target_max_alpha);
outPeak = Swerve_Evaluate_Case(casePeak, p);

%% 6. 生成并打印工程汇总报告
fprintf('【2. 静态载荷与基础阻力】\n');
fprintf('  >> 静态四轮法向载荷 Fz [FL FR RR RL] = [%.2f %.2f %.2f %.2f] N\n', outStatic.dyn_debug.Fz);
fprintf('  >> 静态驱动扭矩 [FL FR RR RL] = [%.4f %.4f %.4f %.4f] N·m\n', outStatic.drive_torque);
fprintf('  >> 静态转向扭矩 [FL FR RR RL] = [%.4f %.4f %.4f %.4f] N·m\n\n', outStatic.steer_torque);

fprintf('【3. 巡航稳态工况，vx = %.2f m/s】\n', cruiseVx);
fprintf('  >> 驱动电机持续扭矩峰值 : %.4f N·m\n', outCruise.max_drive_torque);
fprintf('  >> 转向电机持续扭矩峰值 : %.4f N·m\n', outCruise.max_steer_torque);
fprintf('  >> 最大抓地利用率 : %.3f\n', outCruise.max_grip_usage);
fprintf('  >> 最大轮胎下压形变 : %.5f mm\n\n', outCruise.max_tire_delta * 1000);

fprintf('【4. 极限爆发工况，45° 满载加速】\n');
fprintf('  >> 指令加速度 ax = %.3f m/s², ay = %.3f m/s²\n', ax_worst, ay_worst);
fprintf('  >> 驱动电机极值扭矩 : %.4f N·m\n', outPeak.max_drive_torque);
fprintf('  >> 转向电机极值扭矩 : %.4f N·m\n', outPeak.max_steer_torque);
fprintf('  >> 危险抓地利用率 : %.3f，出现在 %s\n', outPeak.max_grip_usage, wheelNames(outPeak.max_grip_wheel_idx));
fprintf('  >> 极值轮胎下压形变 : %.5f mm，出现在 %s\n\n', outPeak.max_tire_delta * 1000, wheelNames(outPeak.max_delta_wheel_idx));

fprintf('【5. 物理抓地力安全评估】\n');
if any(outPeak.slip_flag)
    fprintf('  [⛔ 危险] 检测到车轮打滑！当前加速度指令超出了轮胎物理极限。\n');
    fprintf('  打滑轮位标识 FL/FR/RR/RL: [%d %d %d %d]\n', outPeak.slip_flag);
else
    fprintf('  [✅ 安全] 当前极限工况下未检测到打滑行为。\n');
end

if any(outPeak.bottoming_flag)
    fprintf('  [⚠️  警告] 检测到轮胎包胶压缩触底！载荷过大，包胶已接近完全压缩。\n');
    fprintf('  触底轮位标识 FL/FR/RR/RL: [%d %d %d %d]\n', outPeak.bottoming_flag);
end
fprintf('\n');

fprintf('【6. 极限工况逐轮明细表】\n');
printWheelSummary(outPeak);
fprintf('========================================================\n');

% ---------------- 局部辅助函数 ----------------
function assertDependency(name)
    if exist(name, 'file') ~= 2 && exist(name, 'class') ~= 8
        error('main:MissingDependency', '缺少核心算法文件: %s。', name);
    end
end

function caseOut = makeCase(vx, vy, omega_z, ax, ay, dt, current_theta, alpha_steer)
    caseOut = struct('vx', vx, 'vy', vy, 'omega_z', omega_z, 'ax', ax, 'ay', ay, 'dt', dt, 'current_theta', current_theta);
    if nargin >= 8 && ~isempty(alpha_steer), caseOut.alpha_steer = alpha_steer; end
end

function printWheelSummary(out)
    names = out.wheel_names;
    fprintf('  Wheel | v_drive(m/s) | theta(deg) | driveT(Nm) | steerT(Nm) | Fz(N) | grip | delta(mm) | slip | bottom\n');
    fprintf('  ------|--------------|------------|------------|------------|-------|------|-----------|------|--------\n');
    for i = 1:4
        if out.slip_flag(i), slipText = 'SLIP'; else, slipText = 'OK'; end
        if out.bottoming_flag(i), bottomText = 'BOTTOM'; else, bottomText = 'OK'; end
        fprintf('  %5s | %12.4f | %10.2f | %10.4f | %10.4f | %5.1f | %4.2f | %9.5f | %4s | %s\n', ...
            names(i), out.v_drive(i), out.theta_steer(i) * 180/pi, out.drive_torque(i), ...
            out.steer_torque(i), out.dyn_debug.Fz(i), out.dyn_debug.grip_usage(i), ...
            out.dyn_debug.delta(i) * 1000, slipText, bottomText);
    end
    fprintf('\n');
end