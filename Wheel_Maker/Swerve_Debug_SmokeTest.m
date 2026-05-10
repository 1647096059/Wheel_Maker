function Swerve_Debug_SmokeTest()
% SWERVE_DEBUG_SMOKETEST 最小冒烟测试
%
% 运行:
%   Swerve_Debug_SmokeTest

    clc;

    fprintf('========== Swerve Debug Smoke Test ==========\n');

    p = Config_Params.toStruct();

    fprintf('\n[1] 测试 Config_Params...\n');
    disp(p);

    fprintf('\n[2] 测试 Swerve_Kinematics_Solver...\n');
    vx = 1.0;
    vy = 0.3;
    omega_z = 1.0;
    current_theta = zeros(1, 4);
    dt = 0.02;

    [v_drive, theta_steer, omega_steer, kin_dbg] = ...
        Swerve_Kinematics_Solver(vx, vy, omega_z, current_theta, dt, p);

    fprintf('v_drive      = [%.3f %.3f %.3f %.3f] m/s\n', v_drive);
    fprintf('theta_steer  = [%.3f %.3f %.3f %.3f] rad\n', theta_steer);
    fprintf('omega_steer  = [%.3f %.3f %.3f %.3f] rad/s\n', omega_steer);
    fprintf('max_drive_v  = %.3f m/s\n', kin_dbg.max_drive_v);
    fprintf('max_steer_w  = %.3f rad/s\n', kin_dbg.max_steer_w);

    fprintf('\n[3] 测试 Swerve_Dynamics_Core...\n');
    ax = 2.0;
    ay = 1.0;
    alpha_vec = ones(1, 4) * 10;

    [drive_T, steer_T, slip, dyn_dbg] = ...
        Swerve_Dynamics_Core(ax, ay, alpha_vec, p);

    fprintf('drive_T      = [%.4f %.4f %.4f %.4f] N·m\n', drive_T);
    fprintf('steer_T      = [%.4f %.4f %.4f %.4f] N·m\n', steer_T);
    fprintf('slip         = [%d %d %d %d]\n', slip);
    fprintf('Fz           = [%.2f %.2f %.2f %.2f] N\n', dyn_dbg.Fz);
    fprintf('grip_usage   = [%.3f %.3f %.3f %.3f]\n', dyn_dbg.grip_usage);

    fprintf('\n[4] 测试 main.m...\n');
    main;

    fprintf('\n========== Smoke Test Passed ==========\n');
end