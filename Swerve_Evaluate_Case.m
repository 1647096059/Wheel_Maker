function out = Swerve_Evaluate_Case(caseIn, p)
% SWERVE_EVALUATE_CASE 舵轮底盘单工况全链路评估器
%
% 描述:
%   该函数是底层的最顶层封装。它接收一个明确的运动指令(期望速度+期望加速度)，
%   先调用运动学解算器计算目标舵角和轮速，然后调用动力学引擎核算扭矩和打滑风险，
%   最后将所有结果和中间调试变量打包为一个完整的结构体输出。
%
% 输入:
%   caseIn.vx            - 期望前向速度 (m/s)
%   caseIn.vy            - 期望右向速度 (m/s)
%   caseIn.omega_z       - 期望逆时针角速度 (rad/s)
%   caseIn.ax            - 期望前向真实加速度 (m/s²)
%   caseIn.ay            - 期望右向真实加速度 (m/s²)
%   caseIn.dt            - 控制/评估周期 (s)
%   caseIn.current_theta - 当前四个舵角反馈值 (1x4, rad)
%   caseIn.alpha_steer   - (可选) 强行指定的转向角加速度 (1x4, rad/s²)
%
% 输出:
%   out - 包含所有评估结论 (速度分配、扭矩核算、形变参数、打滑标志) 的综合字典

    if nargin < 2 || isempty(p)
        p = Config_Params.toStruct();
    else
        p = Config_Params.mergeWithDefaults(p);
    end

    % 自动补全缺失的工况输入参数，防止报错
    caseIn = completeCaseDefaults(caseIn);
    limits = Swerve_Performance_Limits(p);

    % --- 阶段一：调用运动学内核 (解算速度与角度) ---
    [vDrive, thetaSteer, omegaSteer, kinDbg] = ...
        Swerve_Kinematics_Solver( ...
            caseIn.vx, caseIn.vy, caseIn.omega_z, ...
            caseIn.current_theta, caseIn.dt, p);

    % 确定转向角加速度：若未指定，则通过微分计算，并施加物理饱和限制
    if isfield(caseIn, 'alpha_steer') && ~isempty(caseIn.alpha_steer)
        alphaSteer = reshape(caseIn.alpha_steer, 1, 4);
    else
        alphaSteer = abs(omegaSteer) ./ max(caseIn.dt, eps);
        alphaLimit = p.swerve_target_max_alpha;
        alphaSteer = min(alphaSteer, alphaLimit);
    end

    % --- 阶段二：调用动力学内核 (解算扭矩与受力形变) ---
    [driveT, steerT, slipFlag, dynDbg] = ...
        Swerve_Dynamics_Core(caseIn.ax, caseIn.ay, alphaSteer, p);

    % --- 阶段三：结果封装聚合 ---
    out = struct();
    out.case = caseIn;
    out.p = p;
    out.limits = limits;

    out.v_drive = vDrive;
    out.theta_steer = thetaSteer;
    out.omega_steer = omegaSteer;
    out.alpha_steer = alphaSteer;

    out.drive_torque = driveT;
    out.steer_torque = steerT;
    out.slip_flag = slipFlag;
    out.bottoming_flag = dynDbg.bottoming_warning;

    out.kin_debug = kinDbg;
    out.dyn_debug = dynDbg;

    % 提取极值，用于快速状态判断
    out.max_drive_torque = max(driveT);
    out.max_steer_torque = max(steerT);
    out.max_grip_usage = max(dynDbg.grip_usage);
    out.max_tire_delta = max(dynDbg.delta);
    out.max_contact_half_len = max(dynDbg.contact_half_len);

    [~, out.max_grip_wheel_idx] = max(dynDbg.grip_usage);
    [~, out.max_delta_wheel_idx] = max(dynDbg.delta);
    out.wheel_names = ["FL", "FR", "RR", "RL"];
end

% ---------------- 辅助函数区 ----------------
function caseOut = completeCaseDefaults(caseIn)
    if nargin < 1 || isempty(caseIn), caseIn = struct(); end
    caseOut = caseIn;
    caseOut = setDefault(caseOut, 'vx', 0.0);
    caseOut = setDefault(caseOut, 'vy', 0.0);
    caseOut = setDefault(caseOut, 'omega_z', 0.0);
    caseOut = setDefault(caseOut, 'ax', 0.0);
    caseOut = setDefault(caseOut, 'ay', 0.0);
    caseOut = setDefault(caseOut, 'dt', 0.02);
    caseOut = setDefault(caseOut, 'current_theta', zeros(1, 4));

    caseOut.current_theta = reshape(caseOut.current_theta, 1, 4);
    if ~isfinite(caseOut.dt) || caseOut.dt <= 0, caseOut.dt = 0.02; end
end

function s = setDefault(s, fieldName, defaultValue)
    if ~isfield(s, fieldName) || isempty(s.(fieldName))
        s.(fieldName) = defaultValue;
    end
end