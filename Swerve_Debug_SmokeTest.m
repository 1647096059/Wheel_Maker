function Swerve_Debug_SmokeTest()
% SWERVE_DEBUG_SMOKETEST 开发者环境全量冒烟测试
%
% 描述:
%   在提交任何代码更改前运行此脚本。它将按顺序对物理一致性、参数生成、
%   正解运动学、逆解动力学以及主报告流进行链式触发，以确保没有任何语法
%   错误或未声明的参数前缀。

    clc;
    fprintf('========== 开发者全环境冒烟测试 (Smoke Test) ==========\n');

    %% 阶段 0: 物理合规性校验
    fprintf('\n[0/4] 验证底层刚体力学特性...\n');
    Swerve_PhysicsConsistency_Test(false);

    %% 阶段 1: 内存参数字典装载
    fprintf('\n[1/4] 拉取内存全量配置 Config_Params...\n');
    p = Config_Params.toStruct();
    fprintf('      成功提取 %d 个环境/物理常量。\n', length(fieldnames(p)));

    %% 阶段 2: 运动学模块 (Kinematics) 接口注入测试
    fprintf('\n[2/4] 测试 Swerve_Kinematics_Solver 数据流转...\n');
    vx = 1.0; vy = 0.3; omega_z = 1.0; dt = 0.02;
    [v_drive, theta_steer, omega_steer, kin_dbg] = ...
        Swerve_Kinematics_Solver(vx, vy, omega_z, zeros(1, 4), dt, p);
    fprintf('      v_drive = [%.3f %.3f %.3f %.3f] m/s\n', v_drive);

    %% 阶段 3: 动力学模块 (Dynamics) 接口注入测试
    fprintf('\n[3/4] 测试 Swerve_Dynamics_Core 极限数据处理...\n');
    ax = 2.0; ay = 1.0; alpha_vec = ones(1, 4) * 10;
    [drive_T, steer_T, slip, dyn_dbg] = Swerve_Dynamics_Core(ax, ay, alpha_vec, p);
    fprintf('      drive_T = [%.4f %.4f %.4f %.4f] N·m\n', drive_T);

    %% 阶段 4: 集成报告流水线测试
    fprintf('\n[4/4] 触发无头报告链测试 (执行 main.m)...\n');
    evalc('main'); % 捕获并隐藏 main.m 的海量输出，仅确认无崩溃

    fprintf('\n========== 所有测试环境健康，代码可合规提交 ==========\n');
end