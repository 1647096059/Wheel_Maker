function Swerve_UI_Dashboard()
% SWERVE_UI_DASHBOARD 四轮舵轮底盘交互式性能仪表盘
% 依赖：
%   Config_Params.m
%   Swerve_Kinematics_Solver.m
%   Swerve_Dynamics_Core.m
%
% 轮序：
%   [FL, FR, RR, RL]
%
% 坐标：
%   X 向前为正
%   Y 向右为正
%   omega_z 俯视逆时针为正

    clc;

    p = makePreset('Swerve 四轮舵轮 - RM2026 默认');

    wheelNames = ["FL", "FR", "RR", "RL"];
    state.theta = zeros(1, 4);

    %% ========================= 主窗口 =========================
    fig = uifigure( ...
        'Name', 'Swerve UI Dashboard', ...
        'Position', [40, 40, 1620, 900]);

    root = uigridlayout(fig, [1, 2]);
    root.ColumnWidth = {560, '1x'};
    root.RowHeight = {'1x'};
    root.Padding = [12, 12, 12, 12];
    root.ColumnSpacing = 12;

    %% ========================= 左侧区域 =========================
    leftGrid = uigridlayout(root, [2, 1]);
    leftGrid.Layout.Row = 1;
    leftGrid.Layout.Column = 1;
    leftGrid.RowHeight = {460, '1x'};
    leftGrid.Padding = [0, 0, 0, 0];
    leftGrid.RowSpacing = 12;

    %% ---------- 左侧第一栏：机器人类型与参数 ----------
    robotPanel = uipanel(leftGrid, ...
        'Title', '1. 机器人类型与本体参数');
    robotPanel.Layout.Row = 1;

    robotGrid = uigridlayout(robotPanel, [4, 1]);
    robotGrid.RowHeight = {34, '1x', 118, 34};
    robotGrid.Padding = [10, 10, 10, 10];
    robotGrid.RowSpacing = 8;

    robotTypeDropdown = uidropdown(robotGrid, ...
        'Items', { ...
            'Swerve 四轮舵轮 - RM2026 默认', ...
            'Swerve 四轮舵轮 - 轻量小车预设', ...
            'Swerve 四轮舵轮 - 重载底盘预设', ...
            'Wheel-Leg 双轮轮腿 - 预留' ...
        }, ...
        'Value', 'Swerve 四轮舵轮 - RM2026 默认', ...
        'ValueChangedFcn', @(~, ~) onRobotTypeChanged());
    robotTypeDropdown.Layout.Row = 1;

    robotParamTable = uitable(robotGrid);
    robotParamTable.Layout.Row = 2;
    robotParamTable.ColumnName = {'参数名', '数值', '单位', '说明'};
    robotParamTable.ColumnEditable = [false true false false];
    robotParamTable.ColumnWidth = {170, 82, 72, 210};
    robotParamTable.CellEditCallback = @(src, event) onParamEdited(src, event);
    robotParamTable.Data = makeRobotParamRows(p, robotTypeDropdown.Value);

    conditionPanel = uipanel(robotGrid, ...
        'Title', '当前测试工况');
    conditionPanel.Layout.Row = 3;

    conditionGrid = uigridlayout(conditionPanel, [2, 6]);
    conditionGrid.RowHeight = {32, 32};
    conditionGrid.ColumnWidth = {62, '1x', 62, '1x', 86, '1x'};
    conditionGrid.RowSpacing = 4;
    conditionGrid.ColumnSpacing = 6;
    conditionGrid.Padding = [8, 8, 8, 8];

    uilabel(conditionGrid, 'Text', 'vx m/s');
    efVx = uieditfield(conditionGrid, 'numeric', ...
        'Value', 1.0, ...
        'ValueChangedFcn', @(~, ~) updateAll());

    uilabel(conditionGrid, 'Text', 'vy m/s');
    efVy = uieditfield(conditionGrid, 'numeric', ...
        'Value', 0.0, ...
        'ValueChangedFcn', @(~, ~) updateAll());

    uilabel(conditionGrid, 'Text', 'omega');
    efOmega = uieditfield(conditionGrid, 'numeric', ...
        'Value', 0.0, ...
        'ValueChangedFcn', @(~, ~) updateAll());

    uilabel(conditionGrid, 'Text', 'dt s');
    efDt = uieditfield(conditionGrid, 'numeric', ...
        'Value', 0.02, ...
        'Limits', [0.001, 1.0], ...
        'ValueChangedFcn', @(~, ~) updateAll());

    uilabel(conditionGrid, 'Text', 'ax m/s²');
    efAx = uieditfield(conditionGrid, 'numeric', ...
        'Value', 1.0, ...
        'ValueChangedFcn', @(~, ~) updateAll());

    uilabel(conditionGrid, 'Text', 'ay m/s²');
    efAy = uieditfield(conditionGrid, 'numeric', ...
        'Value', 0.0, ...
        'ValueChangedFcn', @(~, ~) updateAll());

    buttonGrid = uigridlayout(robotGrid, [1, 2]);
    buttonGrid.Layout.Row = 4;
    buttonGrid.ColumnWidth = {'1x', '1x'};
    buttonGrid.Padding = [0, 0, 0, 0];
    buttonGrid.ColumnSpacing = 8;

    uibutton(buttonGrid, ...
        'Text', '重置舵角状态', ...
        'ButtonPushedFcn', @(~, ~) resetSteerState());

    uibutton(buttonGrid, ...
        'Text', '重新计算', ...
        'ButtonPushedFcn', @(~, ~) updateAll());

    %% ---------- 左侧第二栏：工程常数 ----------
    engineeringPanel = uipanel(leftGrid, ...
        'Title', '2. 工程常数 / 环境 / 电机传动');
    engineeringPanel.Layout.Row = 2;

    engineeringGrid = uigridlayout(engineeringPanel, [1, 1]);
    engineeringGrid.Padding = [10, 10, 10, 10];

    engineeringTable = uitable(engineeringGrid);
    engineeringTable.ColumnName = {'参数名', '数值', '单位', '说明'};
    engineeringTable.ColumnEditable = [false true false false];
    engineeringTable.ColumnWidth = {170, 82, 72, 210};
    engineeringTable.CellEditCallback = @(src, event) onParamEdited(src, event);
    engineeringTable.Data = makeEngineeringParamRows(p, robotTypeDropdown.Value);

    %% ========================= 右侧区域 =========================
    rightGrid = uigridlayout(root, [3, 1]);
    rightGrid.Layout.Row = 1;
    rightGrid.Layout.Column = 2;
    rightGrid.RowHeight = {210, '1x', 220};
    rightGrid.Padding = [0, 0, 0, 0];
    rightGrid.RowSpacing = 12;

    %% ---------- 结果报文：指标卡 ----------
    reportPanel = uipanel(rightGrid, ...
        'Title', '结果报文');
    reportPanel.Layout.Row = 1;

    reportGrid = uigridlayout(reportPanel, [4, 3]);
    reportGrid.Padding = [8, 8, 8, 8];
    reportGrid.RowSpacing = 6;
    reportGrid.ColumnSpacing = 8;
    reportGrid.RowHeight = {'1x', '1x', '1x', '1x'};
    reportGrid.ColumnWidth = {'1x', '1x', '1x'};

    reportCards = gobjects(12, 1);

    reportCards(1)  = makeMetricCard(reportGrid, '机器人类型', '-');
    reportCards(2)  = makeMetricCard(reportGrid, '速度指令', '-');
    reportCards(3)  = makeMetricCard(reportGrid, '加速度工况', '-');

    reportCards(4)  = makeMetricCard(reportGrid, '最大直线速度', '-');
    reportCards(5)  = makeMetricCard(reportGrid, '最大自转速度', '-');
    reportCards(6)  = makeMetricCard(reportGrid, '最大转向角速度', '-');

    reportCards(7)  = makeMetricCard(reportGrid, '峰值驱动扭矩', '-');
    reportCards(8)  = makeMetricCard(reportGrid, '峰值转向扭矩', '-');
    reportCards(9)  = makeMetricCard(reportGrid, '摩擦系数 / 弹性模量', '-');

    reportCards(10) = makeMetricCard(reportGrid, '最大轮胎下压', '-');
    reportCards(11) = makeMetricCard(reportGrid, '最大抓地利用率', '-');
    reportCards(12) = makeMetricCard(reportGrid, '打滑轮', '-');

    %% ---------- 图表区 ----------
    chartPanel = uipanel(rightGrid, ...
        'Title', '图表与轮胎形变示意');
    chartPanel.Layout.Row = 2;

    chartGrid = uigridlayout(chartPanel, [2, 2]);
    chartGrid.Padding = [8, 8, 8, 8];
    chartGrid.RowSpacing = 8;
    chartGrid.ColumnSpacing = 8;

    axTorque = uiaxes(chartGrid);
    axTorque.Layout.Row = 1;
    axTorque.Layout.Column = 1;

    axGrip = uiaxes(chartGrid);
    axGrip.Layout.Row = 1;
    axGrip.Layout.Column = 2;

    axFz = uiaxes(chartGrid);
    axFz.Layout.Row = 2;
    axFz.Layout.Column = 1;

    axDeform = uiaxes(chartGrid);
    axDeform.Layout.Row = 2;
    axDeform.Layout.Column = 2;

    %% ---------- 结果表 ----------
    tablePanel = uipanel(rightGrid, ...
        'Title', '逐轮结果表');
    tablePanel.Layout.Row = 3;

    tableGrid = uigridlayout(tablePanel, [1, 1]);
    tableGrid.Padding = [8, 8, 8, 8];

    resultTable = uitable(tableGrid);
    resultTable.ColumnSortable = true;

    updateAll();

    %% ============================================================
    %                         回调函数
    % ============================================================

    function onRobotTypeChanged()
        p = makePreset(robotTypeDropdown.Value);
        state.theta = zeros(1, 4);

        robotParamTable.Data = makeRobotParamRows(p, robotTypeDropdown.Value);
        engineeringTable.Data = makeEngineeringParamRows(p, robotTypeDropdown.Value);

        updateAll();
    end

    function onParamEdited(src, event)
        data = src.Data;

        row = event.Indices(1);
        fieldName = data{row, 1};
        newValue = event.NewData;

        if startsWith(string(fieldName), "说明") || startsWith(string(fieldName), "未接入")
            data{row, 2} = event.PreviousData;
            src.Data = data;
            return;
        end

        if ischar(newValue) || isstring(newValue)
            newValue = str2double(newValue);
        end

        if isempty(newValue) || ~isnumeric(newValue) || ~isfinite(newValue)
            data{row, 2} = event.PreviousData;
            src.Data = data;
            uialert(fig, '参数必须是有限数值。', '参数输入错误');
            return;
        end

        if newValue < 0 && ~isSignedAllowed(fieldName)
            data{row, 2} = event.PreviousData;
            src.Data = data;
            uialert(fig, sprintf('%s 不建议设置为负数。', fieldName), '参数输入错误');
            return;
        end

        p.(fieldName) = newValue;
        data{row, 2} = newValue;
        src.Data = data;

        updateAll();
    end

    function resetSteerState()
        state.theta = zeros(1, 4);
        updateAll();
    end

    function updateAll()
        choice = robotTypeDropdown.Value;

        if contains(choice, 'Wheel-Leg')
            showWheelLegPlaceholder();
            return;
        end

        try
            vx = efVx.Value;
            vy = efVy.Value;
            omega_z = efOmega.Value;
            dt = efDt.Value;

            axCmd = efAx.Value;
            ayCmd = efAy.Value;

            if dt <= 0 || ~isfinite(dt)
                dt = 0.02;
                efDt.Value = dt;
            end

            alphaVec = estimateSteerAngularAccel(vx, vy, omega_z, dt);

            [vDrive, thetaSteer, omegaSteer, kinDbg] = runKinematics( ...
                vx, vy, omega_z, state.theta, dt, p);

            state.theta = thetaSteer;

            [driveT, steerT, slipFlag, dynDbg] = runDynamics( ...
                axCmd, ayCmd, alphaVec, p);

            dynDbg = completeDynamicsDebug(dynDbg, p, axCmd, ayCmd);

            maxDriveV = getMaxDriveSpeed(p);
            maxSpinW = getMaxSpinSpeed(p);

            resultT = makeResultTable( ...
                wheelNames, vDrive, thetaSteer, omegaSteer, ...
                driveT, steerT, slipFlag, dynDbg);

            resultTable.Data = resultT;

            updateReport( ...
                choice, vx, vy, omega_z, axCmd, ayCmd, ...
                maxDriveV, maxSpinW, driveT, steerT, slipFlag, ...
                kinDbg, dynDbg);

            drawTorqueChart(axTorque, wheelNames, driveT, steerT);
            drawGripChart(axGrip, wheelNames, dynDbg.grip_usage);
            drawFzChart(axFz, wheelNames, dynDbg.Fz);
            drawDeformationChart(axDeform, wheelNames, p, dynDbg);

        catch ME
            showError(ME);
        end
    end

    %% ============================================================
    %                         计算函数
    % ============================================================

    function [vDrive, thetaSteer, omegaSteer, kinDbg] = runKinematics(vx, vy, omega_z, currentTheta, dt, pLocal)
        nout = nargout('Swerve_Kinematics_Solver');

        if nout >= 4
            [vDrive, thetaSteer, omegaSteer, kinDbg] = ...
                Swerve_Kinematics_Solver(vx, vy, omega_z, currentTheta, dt, pLocal);
        else
            [vDrive, thetaSteer, omegaSteer] = ...
                Swerve_Kinematics_Solver(vx, vy, omega_z, currentTheta, dt, pLocal);

            kinDbg = struct();
            kinDbg.max_drive_v = getMaxDriveSpeed(pLocal);
            kinDbg.max_steer_w = getMaxSteerOmega(pLocal);
        end

        if ~isstruct(kinDbg)
            kinDbg = struct();
            kinDbg.max_drive_v = getMaxDriveSpeed(pLocal);
            kinDbg.max_steer_w = getMaxSteerOmega(pLocal);
        end
    end

    function [driveT, steerT, slipFlag, dynDbg] = runDynamics(axCmd, ayCmd, alphaVec, pLocal)
        nout = nargout('Swerve_Dynamics_Core');

        if nout >= 4
            [driveT, steerT, slipFlag, dynDbg] = ...
                Swerve_Dynamics_Core(axCmd, ayCmd, alphaVec, pLocal);
        else
            [driveT, steerT, slipFlag] = ...
                Swerve_Dynamics_Core(axCmd, ayCmd, alphaVec, pLocal);

            dynDbg = localDynamicsDebug(pLocal, axCmd, ayCmd);
        end

        if ~isstruct(dynDbg)
            dynDbg = localDynamicsDebug(pLocal, axCmd, ayCmd);
        end
    end

    function alphaVec = estimateSteerAngularAccel(vx, vy, omega_z, dt)
        if dt <= 0 || ~isfinite(dt)
            dt = 0.02;
        end

        try
            [~, ~, omegaSteerNow] = Swerve_Kinematics_Solver( ...
                vx, vy, omega_z, state.theta, dt, p);

            alphaVec = abs(omegaSteerNow) / dt;
        catch
            alphaVec = ones(1, 4) * getField(p, 'swerve_target_max_alpha', 20.0);
        end

        alphaLimit = getField(p, 'swerve_target_max_alpha', 20.0);
        alphaVec = min(alphaVec, alphaLimit);
        alphaVec = reshape(alphaVec, 1, 4);
    end

    function dynDbg = completeDynamicsDebug(dynDbg, pLocal, axCmd, ayCmd)
        fallback = localDynamicsDebug(pLocal, axCmd, ayCmd);
        names = fieldnames(fallback);

        if isempty(dynDbg) || ~isstruct(dynDbg)
            dynDbg = fallback;
            return;
        end

        for i = 1:numel(names)
            name = names{i};
            if ~isfield(dynDbg, name)
                dynDbg.(name) = fallback.(name);
            end
        end
    end

    function dynDbg = localDynamicsDebug(pLocal, axCmd, ayCmd)
        m = getField(pLocal, 'swerve_m_total', 25.0);
        g = getField(pLocal, 'g', 9.81);

        LxFull = getField(pLocal, 'swerve_wheel_base_x', 0.27);
        LyFull = getField(pLocal, 'swerve_wheel_base_y', 0.27);
        hCog = getField(pLocal, 'swerve_h_cog', 0.2);

        R = getField(pLocal, 'swerve_wheel_radius', 0.0425);
        w = getField(pLocal, 'swerve_wheel_width', 0.03);

        [E, mu] = getElasticityAndMu(pLocal);

        FzStatic = m * g / 4;

        dFzX = m * axCmd * hCog / (2 * LxFull);
        dFzY = m * ayCmd * hCog / (2 * LyFull);

        Fz = [
            FzStatic + dFzX - dFzY, ...
            FzStatic + dFzX + dFzY, ...
            FzStatic - dFzX + dFzY, ...
            FzStatic - dFzX - dFzY ...
        ];

        Fz = max(Fz, 0);

        FzTotal = sum(Fz);
        if FzTotal <= eps
            FzTotal = eps;
        end

        FxTotal = m * axCmd;
        FyTotal = m * ayCmd;

        Fx_i = FxTotal .* Fz ./ FzTotal;
        Fy_i = FyTotal .* Fz ./ FzTotal;
        F_total_i = hypot(Fx_i, Fy_i);

        muFz = mu .* Fz;

        gripUsage = zeros(1, 4);
        for i = 1:4
            if muFz(i) > eps
                gripUsage(i) = F_total_i(i) / muFz(i);
            else
                gripUsage(i) = inf;
            end
        end

        delta = Fz ./ max(E * w, eps);
        contactHalfLen = sqrt(max(0, 2 * R .* delta - delta.^2));

        dynDbg = struct();
        dynDbg.E = E;
        dynDbg.mu = mu;
        dynDbg.Fz = Fz;
        dynDbg.Fx_i = Fx_i;
        dynDbg.Fy_i = Fy_i;
        dynDbg.F_total_i = F_total_i;
        dynDbg.muFz = muFz;
        dynDbg.grip_usage = gripUsage;
        dynDbg.delta = delta;
        dynDbg.contact_half_len = contactHalfLen;
    end

    function [E, mu] = getElasticityAndMu(pLocal)
        s = getField(pLocal, 'swerve_hardness_shoreA', 60);
        s = min(max(s, 1), 99);

        E = ((0.0981 * (56 + 7.66 * s)) / (0.149 * (100 - s))) * 1e6;

        sRef = 60;
        ERef = ((0.0981 * (56 + 7.66 * sRef)) / (0.149 * (100 - sRef))) * 1e6;

        muBase = getField(pLocal, 'swerve_mu_ground', 0.8);
        mu = muBase * (ERef / E)^(2/3);
        mu = min(max(mu, 0.1), 1.5);
    end

    %% ============================================================
    %                         UI 数据生成
    % ============================================================

    function rows = makeRobotParamRows(pLocal, choice)
        if contains(choice, 'Wheel-Leg')
            rows = {
                'wl_m_total', getField(pLocal, 'wl_m_total', 20.0), 'kg', '轮腿机器人总质量，当前仅预留';
                '说明_1', NaN, '-', 'Wheel-Leg 动力学模型尚未接入本 Dashboard';
            };
            return;
        end

        rows = {
            'swerve_m_total', getField(pLocal, 'swerve_m_total', 25.0), 'kg', '全车质量';
            'swerve_wheel_base_x', getField(pLocal, 'swerve_wheel_base_x', 0.27), 'm', '前后轴距';
            'swerve_wheel_base_y', getField(pLocal, 'swerve_wheel_base_y', 0.27), 'm', '左右轮距';
            'swerve_wheel_radius', getField(pLocal, 'swerve_wheel_radius', 0.0425), 'm', '轮胎半径';
            'swerve_wheel_width', getField(pLocal, 'swerve_wheel_width', 0.03), 'm', '轮胎宽度';
            'swerve_h_cog', getField(pLocal, 'swerve_h_cog', 0.2), 'm', '重心高度';
            'swerve_I_steer', getField(pLocal, 'swerve_I_steer', 0.015), 'kg·m²', '转向机构转动惯量';
            'swerve_target_max_a', getField(pLocal, 'swerve_target_max_a', 3.0), 'm/s²', '目标最大线加速度';
            'swerve_target_max_alpha', getField(pLocal, 'swerve_target_max_alpha', 20.0), 'rad/s²', '目标最大转向角加速度';
        };
    end

    function rows = makeEngineeringParamRows(pLocal, choice)
        if contains(choice, 'Wheel-Leg')
            rows = {
                '说明_2', NaN, '-', 'Wheel-Leg 工程常数待后续建模';
            };
            return;
        end

        rows = {
            'swerve_i_drive', getField(pLocal, 'swerve_i_drive', 1.0), '-', '驱动减速比';
            'swerve_i_steer', getField(pLocal, 'swerve_i_steer', 1.0), '-', '转向减速比';
            'swerve_motor_max_rpm', getField(pLocal, 'swerve_motor_max_rpm', 450), 'rpm', '驱动电机最高转速';
            'swerve_steer_max_rpm', getField(pLocal, 'swerve_steer_max_rpm', 120), 'rpm', '转向电机最高转速';
            'swerve_mu_ground', getField(pLocal, 'swerve_mu_ground', 0.8), '-', '基准摩擦系数';
            'swerve_carpet_factor', getField(pLocal, 'swerve_carpet_factor', 4.0), '-', '地胶阻力放大系数';
            'swerve_roll_resistance', getField(pLocal, 'swerve_roll_resistance', 0.018), '-', '滚动阻力系数';
            'swerve_eta_slip', getField(pLocal, 'swerve_eta_slip', 0.9), '-', '滑移效率';
            'swerve_hardness_shoreA', getField(pLocal, 'swerve_hardness_shoreA', 60), 'Shore A', '轮胎邵氏硬度';
            'swerve_T_mech_drive', getField(pLocal, 'swerve_T_mech_drive', 0.1), 'N·m', '驱动机械损耗扭矩';
            'swerve_T_mech_steer', getField(pLocal, 'swerve_T_mech_steer', 0.1), 'N·m', '转向机械损耗扭矩';
            'swerve_redundancy', getField(pLocal, 'swerve_redundancy', 1.2), '-', '安全冗余系数';
        };
    end

    function resultT = makeResultTable(names, vDrive, thetaSteer, omegaSteer, driveT, steerT, slipFlag, dynDbg)
        slipText = repmat("OK", 4, 1);
        slipIdx = find(logical(slipFlag(:)));

        if ~isempty(slipIdx)
            slipText(slipIdx) = "SLIP";
        end

        resultT = table( ...
            names(:), ...
            round(vDrive(:), 4), ...
            round(thetaSteer(:) * 180/pi, 2), ...
            round(omegaSteer(:), 4), ...
            round(driveT(:), 4), ...
            round(steerT(:), 4), ...
            round(dynDbg.Fz(:), 2), ...
            round(dynDbg.delta(:) * 1000, 4), ...
            round(dynDbg.contact_half_len(:) * 1000, 4), ...
            round(dynDbg.grip_usage(:), 3), ...
            slipText, ...
            'VariableNames', { ...
                'Wheel', ...
                'DriveSpeed_mps', ...
                'SteerAngle_deg', ...
                'SteerRate_radps', ...
                'DriveTorque_Nm', ...
                'SteerTorque_Nm', ...
                'Fz_N', ...
                'TireCompression_mm', ...
                'ContactHalfLen_mm', ...
                'GripUsage', ...
                'SlipState' ...
            });
    end

    function updateReport(choice, vx, vy, omega_z, axCmd, ayCmd, maxDriveV, maxSpinW, driveT, steerT, slipFlag, kinDbg, dynDbg)
    [maxDelta, maxDeltaIdx] = max(dynDbg.delta);
    [maxGrip, maxGripIdx] = max(dynDbg.grip_usage);

    slipIdx = find(logical(slipFlag(:)));

    if isempty(slipIdx)
        slipListText = '无';
    else
        slipListText = strjoin(cellstr(wheelNames(slipIdx)), ', ');
    end

    maxDeltaWheelName = char(wheelNames(maxDeltaIdx));
    maxGripWheelName = char(wheelNames(maxGripIdx));

    if isstruct(kinDbg) && isfield(kinDbg, 'max_steer_w')
        maxSteerW = kinDbg.max_steer_w;
    else
        maxSteerW = getMaxSteerOmega(p);
    end

    robotName = choice;
    robotName = erase(robotName, 'Swerve 四轮舵轮 - ');
    robotName = erase(robotName, 'Wheel-Leg 双轮轮腿 - ');

    setMetricCard(1,  char(robotName));
    setMetricCard(2,  sprintf('vx %.2f, vy %.2f, wz %.2f', vx, vy, omega_z));
    setMetricCard(3,  sprintf('ax %.2f, ay %.2f', axCmd, ayCmd));

    setMetricCard(4,  sprintf('%.3f m/s', maxDriveV));
    setMetricCard(5,  sprintf('%.3f rad/s', maxSpinW));
    setMetricCard(6,  sprintf('%.3f rad/s', maxSteerW));

    setMetricCard(7,  sprintf('%.4f N·m', max(driveT)));
    setMetricCard(8,  sprintf('%.4f N·m', max(steerT)));
    setMetricCard(9,  sprintf('mu %.3f / E %.2f MPa', dynDbg.mu, dynDbg.E / 1e6));

    setMetricCard(10, sprintf('%s %.4f mm', maxDeltaWheelName, maxDelta * 1000));
    setMetricCard(11, sprintf('%s %.3f', maxGripWheelName, maxGrip));
    setMetricCard(12, slipListText);
    end

    %% ============================================================
    %                         画图函数
    % ============================================================

    function drawTorqueChart(axHandle, names, driveT, steerT)
        cla(axHandle);

        x = 1:4;
        bar(axHandle, x, [driveT(:), steerT(:)], 'grouped');

        axHandle.XTick = x;
        axHandle.XTickLabel = cellstr(names);
        ylabel(axHandle, 'Torque / N·m');
        title(axHandle, '驱动 / 转向扭矩');
        legend(axHandle, {'Drive', 'Steer'}, 'Location', 'best');
        grid(axHandle, 'on');
    end

    function drawGripChart(axHandle, names, gripUsage)
        cla(axHandle);

        x = 1:4;
        bar(axHandle, x, gripUsage(:));

        hold(axHandle, 'on');
        plot(axHandle, [0.5, 4.5], [1, 1], '--', 'LineWidth', 1.2);
        hold(axHandle, 'off');

        axHandle.XTick = x;
        axHandle.XTickLabel = cellstr(names);
        ylabel(axHandle, 'F / \muF_z');
        title(axHandle, '抓地利用率，超过 1 代表打滑');

        yMax = max(gripUsage);
        if ~isfinite(yMax) || yMax <= 0
            yMax = 1;
        end

        ylim(axHandle, [0, max(1.2, yMax * 1.15)]);
        grid(axHandle, 'on');
    end

    function drawFzChart(axHandle, names, Fz)
        cla(axHandle);

        x = 1:4;
        bar(axHandle, x, Fz(:));

        axHandle.XTick = x;
        axHandle.XTickLabel = cellstr(names);
        ylabel(axHandle, 'Fz / N');
        title(axHandle, '四轮法向载荷');
        grid(axHandle, 'on');
    end

    function drawDeformationChart(axHandle, names, pLocal, dynDbg)
        cla(axHandle);

        R = getField(pLocal, 'swerve_wheel_radius', 0.0425);

        [deltaActual, idx] = max(dynDbg.delta);
        contactActual = dynDbg.contact_half_len(idx);

        visualScale = 25;
        deltaVisual = deltaActual * visualScale;
        deltaVisual = min(deltaVisual, 0.65 * R);

        contactVisual = sqrt(max(0, 2 * R * deltaVisual - deltaVisual^2));

        theta = linspace(0, 2*pi, 500);

        x0 = R * cos(theta);
        z0 = R + R * sin(theta);

        x1 = R * cos(theta);
        z1 = R - deltaVisual + R * sin(theta);
        z1(z1 < 0) = 0;

        plot(axHandle, x0 * 1000, z0 * 1000, '--', 'LineWidth', 1.0);
        hold(axHandle, 'on');

        plot(axHandle, x1 * 1000, z1 * 1000, 'LineWidth', 1.8);
        plot(axHandle, [-1.25 * R, 1.25 * R] * 1000, [0, 0], 'k-', 'LineWidth', 1.2);
        plot(axHandle, [-contactVisual, contactVisual] * 1000, [0, 0], 'LineWidth', 5);

        plot(axHandle, 0, (R - deltaVisual) * 1000, 'o', 'MarkerSize', 5);

        text(axHandle, -1.22 * R * 1000, 2.05 * R * 1000, ...
            sprintf([ ...
                '显示轮: %s\n', ...
                '实际下压: %.4f mm\n', ...
                '接触半长: %.4f mm\n', ...
                '示意放大: x%d' ...
            ], ...
            names(idx), ...
            deltaActual * 1000, ...
            contactActual * 1000, ...
            visualScale), ...
            'FontSize', 10);

        hold(axHandle, 'off');

        axis(axHandle, 'equal');
        xlim(axHandle, [-1.35 * R, 1.35 * R] * 1000);
        ylim(axHandle, [-0.12 * R, 2.35 * R] * 1000);
        xlabel(axHandle, '横向尺寸 / mm');
        ylabel(axHandle, '高度 / mm');
        title(axHandle, '轮胎下压形变示意，显示最大载荷轮');
        grid(axHandle, 'on');
    end

    %% ============================================================
    %                         辅助显示
    % ============================================================

   function cardLabel = makeMetricCard(parent, titleText, valueText)
    % 单 Label 版指标卡：
    % 避免 uipanel 内部两行布局被压缩，导致数值不可见。

    cardLabel = uilabel(parent, ...
        'Text', sprintf('%s\n%s', titleText, valueText), ...
        'FontSize', 13, ...
        'FontWeight', 'bold', ...
        'WordWrap', 'on', ...
        'HorizontalAlignment', 'left', ...
        'VerticalAlignment', 'center', ...
        'BackgroundColor', [0.96, 0.96, 0.96]);

    cardLabel.UserData = titleText;
   end

function setMetricCard(idx, valueText)
    titleText = reportCards(idx).UserData;

    if isstring(valueText)
        valueText = char(valueText);
    elseif isnumeric(valueText)
        valueText = num2str(valueText);
    end

    reportCards(idx).Text = sprintf('%s\n%s', titleText, valueText);
end
    
    function showWheelLegPlaceholder()
    for i = 1:numel(reportCards)
        setMetricCard(i, '-');
    end

    setMetricCard(1, 'Wheel-Leg 预留');
    setMetricCard(2, '模型尚未接入');
    setMetricCard(3, '需新增轮腿动力学');
    setMetricCard(4, 'WheelLeg_Kinematics');
    setMetricCard(5, 'WheelLeg_Dynamics');
    setMetricCard(6, '参数映射待定义');

    resultTable.Data = table();

    clearAxisWithMessage(axTorque, 'Wheel-Leg 暂未接入');
    clearAxisWithMessage(axGrip, 'Wheel-Leg 暂未接入');
    clearAxisWithMessage(axFz, 'Wheel-Leg 暂未接入');
    clearAxisWithMessage(axDeform, 'Wheel-Leg 暂未接入');
    end

    function clearAxisWithMessage(axHandle, msg)
        cla(axHandle);
        title(axHandle, msg);
        axis(axHandle, 'off');
        text(axHandle, 0.5, 0.5, msg, ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'middle', ...
            'FontSize', 14);
    end

    function showError(ME)
    for i = 1:numel(reportCards)
        setMetricCard(i, '-');
    end

    setMetricCard(1, '计算失败');
    setMetricCard(2, ME.message);

    resultTable.Data = table();

    clearAxisWithMessage(axTorque, '计算失败');
    clearAxisWithMessage(axGrip, '计算失败');
    clearAxisWithMessage(axFz, '计算失败');
    clearAxisWithMessage(axDeform, '计算失败');
    end

    %% ============================================================
    %                         参数与工具
    % ============================================================

    function pOut = makePreset(choice)
        pOut = loadDefaultConfig();

        if contains(choice, '轻量')
            pOut.swerve_m_total = 15.0;
            pOut.swerve_wheel_base_x = 0.230;
            pOut.swerve_wheel_base_y = 0.230;
            pOut.swerve_wheel_radius = 0.035;
            pOut.swerve_wheel_width = 0.026;
            pOut.swerve_h_cog = 0.16;
            pOut.swerve_motor_max_rpm = 600;
            pOut.swerve_steer_max_rpm = 150;
            pOut.swerve_target_max_a = 3.5;

        elseif contains(choice, '重载')
            pOut.swerve_m_total = 35.0;
            pOut.swerve_wheel_base_x = 0.320;
            pOut.swerve_wheel_base_y = 0.320;
            pOut.swerve_wheel_radius = 0.050;
            pOut.swerve_wheel_width = 0.040;
            pOut.swerve_h_cog = 0.24;
            pOut.swerve_motor_max_rpm = 400;
            pOut.swerve_steer_max_rpm = 100;
            pOut.swerve_target_max_a = 2.5;

        elseif contains(choice, 'Wheel-Leg')
            if ~isfield(pOut, 'wl_m_total')
                pOut.wl_m_total = 20.0;
            end
        end
    end

    function pDefault = loadDefaultConfig()
        pDefault = struct();

        try
            if exist('Config_Params', 'class') == 8
                names = properties('Config_Params');

                for k = 1:numel(names)
                    name = names{k};
                    pDefault.(name) = eval(['Config_Params.', name, ';']);
                end
            end
        catch
            % 若读取 Config_Params 失败，则使用下面的兜底默认参数。
        end

        fallback = struct();
        fallback.g = 9.81;

        fallback.swerve_m_total = 25.0;
        fallback.swerve_wheel_base_x = 0.2700;
        fallback.swerve_wheel_base_y = 0.2700;
        fallback.swerve_wheel_radius = 0.0425;
        fallback.swerve_wheel_width = 0.030;
        fallback.swerve_h_cog = 0.2;
        fallback.swerve_I_steer = 0.015;

        fallback.swerve_i_drive = 1.0;
        fallback.swerve_i_steer = 1.0;
        fallback.swerve_motor_max_rpm = 450;
        fallback.swerve_steer_max_rpm = 120;

        fallback.swerve_target_max_a = 3.0;
        fallback.swerve_target_max_alpha = 20.0;

        fallback.swerve_mu_ground = 0.8;
        fallback.swerve_carpet_factor = 4.0;
        fallback.swerve_roll_resistance = 0.018;
        fallback.swerve_eta_slip = 0.9;
        fallback.swerve_hardness_shoreA = 60;
        fallback.swerve_T_mech_drive = 0.1;
        fallback.swerve_T_mech_steer = 0.1;
        fallback.swerve_redundancy = 1.2;

        fallback.wl_m_total = 20.0;

        names = fieldnames(fallback);
        for k = 1:numel(names)
            name = names{k};
            if ~isfield(pDefault, name)
                pDefault.(name) = fallback.(name);
            end
        end
    end

    function val = getField(s, name, defaultVal)
        if isstruct(s) && isfield(s, name)
            val = s.(name);
        else
            val = defaultVal;
        end
    end

    function ok = isSignedAllowed(fieldName)
        signedFields = [
            "none"
        ];

        ok = any(strcmp(string(fieldName), signedFields));
    end

    function maxDriveV = getMaxDriveSpeed(pLocal)
        rpm = getField(pLocal, 'swerve_motor_max_rpm', 450);
        iDrive = getField(pLocal, 'swerve_i_drive', 1.0);
        R = getField(pLocal, 'swerve_wheel_radius', 0.0425);

        maxDriveV = rpm / iDrive * 2*pi/60 * R;
    end

    function maxSpinW = getMaxSpinSpeed(pLocal)
        maxDriveV = getMaxDriveSpeed(pLocal);

        Lx = getField(pLocal, 'swerve_wheel_base_x', 0.27) / 2;
        Ly = getField(pLocal, 'swerve_wheel_base_y', 0.27) / 2;

        maxSpinW = maxDriveV / hypot(Lx, Ly);
    end

    function maxSteerW = getMaxSteerOmega(pLocal)
        rpm = getField(pLocal, 'swerve_steer_max_rpm', 120);
        iSteer = getField(pLocal, 'swerve_i_steer', 1.0);

        maxSteerW = rpm / iSteer * 2*pi/60;
    end
end