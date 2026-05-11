function Swerve_UI_Dashboard()
% SWERVE_UI_DASHBOARD 四轮舵轮底盘交互式性能仪表盘

%
% 依赖文件：
%   Config_Params.m
%   Swerve_Get_Preset.m
%   Swerve_Performance_Limits.m
%   Swerve_Kinematics_Solver.m
%   Swerve_Dynamics_Core.m
%   Swerve_Evaluate_Case.m
%
% 坐标：
%   X 向前为正
%   Y 向右为正
%   omega_z 俯视逆时针为正
%
% 轮序：
%   [FL, FR, RR, RL]

    clc;

    %% 0. 依赖检查
    checkDependencies();

    %% 1. 初始状态
    presetNames = { ...
        'Swerve 四轮舵轮 - 默认', ...
        'Swerve 四轮舵轮 - 轻量小车预设', ...
        'Swerve 四轮舵轮 - 重载底盘预设', ...
        'Wheel-Leg 双轮轮腿 - 预留' ...
    };

    defaultPreset = presetNames{1};
    p = Swerve_Get_Preset(defaultPreset);

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
        'Items', presetNames, ...
        'Value', defaultPreset, ...
        'ValueChangedFcn', @(~, ~) onRobotTypeChanged());
    robotTypeDropdown.Layout.Row = 1;

    robotParamTable = uitable(robotGrid);
    robotParamTable.Layout.Row = 2;
    robotParamTable.ColumnName = {'参数', '数值', '单位'};
    robotParamTable.ColumnEditable = [false true false];
    robotParamTable.ColumnWidth = {300, 100, 90};
    robotParamTable.CellEditCallback = @(src, event) onParamEdited(src, event);

    [robotRows, robotFields] = makeRobotParamRows(p, robotTypeDropdown.Value);
    robotParamTable.Data = robotRows;
    robotParamTable.UserData = robotFields;

    conditionPanel = uipanel(robotGrid, ...
        'Title', '当前测试工况');
    conditionPanel.Layout.Row = 3;

    conditionGrid = uigridlayout(conditionPanel, [2, 6]);
    conditionGrid.RowHeight = {32, 32};
    conditionGrid.ColumnWidth = {78, '1x', 78, '1x', 92, '1x'};
    conditionGrid.RowSpacing = 4;
    conditionGrid.ColumnSpacing = 6;
    conditionGrid.Padding = [8, 8, 8, 8];

    uilabel(conditionGrid, 'Text', '前向速度');
    efVx = uieditfield(conditionGrid, 'numeric', ...
        'Value', 1.0, ...
        'ValueChangedFcn', @(~, ~) updateAll());

    uilabel(conditionGrid, 'Text', '右向速度');
    efVy = uieditfield(conditionGrid, 'numeric', ...
        'Value', 0.0, ...
        'ValueChangedFcn', @(~, ~) updateAll());

    uilabel(conditionGrid, 'Text', '角速度');
    efOmega = uieditfield(conditionGrid, 'numeric', ...
        'Value', 0.0, ...
        'ValueChangedFcn', @(~, ~) updateAll());

    uilabel(conditionGrid, 'Text', '控制周期');
    efDt = uieditfield(conditionGrid, 'numeric', ...
        'Value', 0.02, ...
        'Limits', [0.001, 1.0], ...
        'ValueChangedFcn', @(~, ~) updateAll());

    uilabel(conditionGrid, 'Text', '前向加速度');
    efAx = uieditfield(conditionGrid, 'numeric', ...
        'Value', 1.0, ...
        'ValueChangedFcn', @(~, ~) updateAll());

    uilabel(conditionGrid, 'Text', '右向加速度');
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
    engineeringTable.ColumnName = {'参数', '数值', '单位'};
    engineeringTable.ColumnEditable = [false true false];
    engineeringTable.ColumnWidth = {300, 100, 90};
    engineeringTable.CellEditCallback = @(src, event) onParamEdited(src, event);

    [engineeringRows, engineeringFields] = makeEngineeringParamRows(p, robotTypeDropdown.Value);
    engineeringTable.Data = engineeringRows;
    engineeringTable.UserData = engineeringFields;

    %% ========================= 右侧区域 =========================
    rightGrid = uigridlayout(root, [3, 1]);
    rightGrid.Layout.Row = 1;
    rightGrid.Layout.Column = 2;
    rightGrid.RowHeight = {260, '1x', 220};
    rightGrid.Padding = [0, 0, 0, 0];
    rightGrid.RowSpacing = 12;

    %% ---------- 结果报文 ----------
    reportPanel = uipanel(rightGrid, ...
        'Title', '结果报文');
    reportPanel.Layout.Row = 1;

    reportGrid = uigridlayout(reportPanel, [5, 3]);
    reportGrid.Padding = [8, 8, 8, 8];
    reportGrid.RowSpacing = 6;
    reportGrid.ColumnSpacing = 8;
    reportGrid.RowHeight = {'1x', '1x', '1x', '1x', '1x'};
    reportGrid.ColumnWidth = {'1x', '1x', '1x'};

    reportCards = gobjects(15, 1);

    reportCards(1)  = makeMetricCard(reportGrid, '机器人类型', '-');
    reportCards(2)  = makeMetricCard(reportGrid, '速度指令', '-');
    reportCards(3)  = makeMetricCard(reportGrid, '加速度工况', '-');

    reportCards(4)  = makeMetricCard(reportGrid, '最大直线速度', '-');
    reportCards(5)  = makeMetricCard(reportGrid, '最大自转速度', '-');
    reportCards(6)  = makeMetricCard(reportGrid, '最大转向角速度', '-');

    reportCards(7)  = makeMetricCard(reportGrid, '额定驱动扭矩', '-');
    reportCards(8)  = makeMetricCard(reportGrid, '额定转向扭矩', '-');
    reportCards(9)  = makeMetricCard(reportGrid, '额定工况定义', '-');

    reportCards(10) = makeMetricCard(reportGrid, '峰值驱动扭矩', '-');
    reportCards(11) = makeMetricCard(reportGrid, '峰值转向扭矩', '-');
    reportCards(12) = makeMetricCard(reportGrid, '摩擦系数 / 弹性模量', '-');

    reportCards(13) = makeMetricCard(reportGrid, '最大轮胎下压', '-');
    reportCards(14) = makeMetricCard(reportGrid, '最大抓地利用率', '-');
    reportCards(15) = makeMetricCard(reportGrid, '打滑轮', '-');

    %% ---------- 图表区 ----------
    chartPanel = uipanel(rightGrid, ...
        'Title', '图表与轮胎形变示意');
    chartPanel.Layout.Row = 2;

    % 新布局：
    % 左列：上=扭矩图，下=法向载荷图
    % 右列：轮胎下压形变示意图，纵向跨两行
    chartGrid = uigridlayout(chartPanel, [2, 2]);
    chartGrid.Padding = [8, 8, 8, 8];
    chartGrid.RowSpacing = 8;
    chartGrid.ColumnSpacing = 8;
    chartGrid.RowHeight = {'1x', '1x'};
    chartGrid.ColumnWidth = {'1x', '1x'};

    axTorque = uiaxes(chartGrid);
    axTorque.Layout.Row = 1;
    axTorque.Layout.Column = 1;

    axFz = uiaxes(chartGrid);
    axFz.Layout.Row = 2;
    axFz.Layout.Column = 1;

    axDeform = uiaxes(chartGrid);
    axDeform.Layout.Row = [1, 2];
    axDeform.Layout.Column = 2;

    %% ---------- 结果表 ----------
    tablePanel = uipanel(rightGrid, ...
        'Title', '逐轮结果表');
    tablePanel.Layout.Row = 3;

    tableGrid = uigridlayout(tablePanel, [1, 1]);
    tableGrid.Padding = [8, 8, 8, 8];

    resultTable = uitable(tableGrid);
    resultTable.ColumnSortable = true;
    resultTable.ColumnName = getResultTableColumnNames();
    resultTable.ColumnWidth = {58, 95, 85, 95, 115, 120, 120, 120, 120, 90, 115, 115, 95, 80, 80};
    resultTable.Data = makeEmptyResultTable();

    updateAll();

    %% =========================================================================
    %                               回调函数
    % =========================================================================

    function onRobotTypeChanged()
        p = Swerve_Get_Preset(robotTypeDropdown.Value);
        state.theta = zeros(1, 4);

        [robotRowsNew, robotFieldsNew] = makeRobotParamRows(p, robotTypeDropdown.Value);
        robotParamTable.Data = robotRowsNew;
        robotParamTable.UserData = robotFieldsNew;

        [engineeringRowsNew, engineeringFieldsNew] = makeEngineeringParamRows(p, robotTypeDropdown.Value);
        engineeringTable.Data = engineeringRowsNew;
        engineeringTable.UserData = engineeringFieldsNew;

        updateAll();
    end

    function onParamEdited(src, event)
        data = src.Data;

        row = event.Indices(1);
        fieldList = src.UserData;

        if row > numel(fieldList)
            data{row, 2} = event.PreviousData;
            src.Data = data;
            return;
        end

        fieldName = fieldList{row};

        if isempty(fieldName) || startsWith(string(fieldName), "说明") || startsWith(string(fieldName), "未接入")
            data{row, 2} = event.PreviousData;
            src.Data = data;
            return;
        end

        newValue = event.NewData;

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
            % 当前测试工况：用于当前扭矩、抓地利用率、载荷转移等
            caseIn = struct();
            caseIn.vx = efVx.Value;
            caseIn.vy = efVy.Value;
            caseIn.omega_z = efOmega.Value;
            caseIn.dt = efDt.Value;
            caseIn.ax = efAx.Value;
            caseIn.ay = efAy.Value;
            caseIn.current_theta = state.theta;

            out = Swerve_Evaluate_Case(caseIn, p);
            validateEvalOutput(out);

            state.theta = out.theta_steer;

            % 巡航 / 额定工况：
            % 保持当前速度指令，但令前向/右向加速度为 0，转向角加速度为 0。
            cruiseCase = makeCruiseCaseFromEval(out);
            cruiseOut = Swerve_Evaluate_Case(cruiseCase, p);
            validateEvalOutput(cruiseOut);

            resultTable.Data = makeResultTableFromEval(out, cruiseOut);
            resultTable.ColumnName = getResultTableColumnNames();

            updateReportFromEval(choice, out, cruiseOut);

            drawTorqueChart( ...
                axTorque, ...
                out.wheel_names, ...
                out.drive_torque, ...
                out.steer_torque, ...
                cruiseOut.drive_torque, ...
                cruiseOut.steer_torque);

            drawFzChart(axFz, out.wheel_names, out.dyn_debug.Fz);
            drawDeformationChart(axDeform, out.wheel_names, out.p, out.dyn_debug);

        catch ME
            showError(ME);
        end
    end

    %% =========================================================================
    %                               工况生成
    % =========================================================================

    function cruiseCase = makeCruiseCaseFromEval(out)
        cruiseCase = struct();

        cruiseCase.vx = out.case.vx;
        cruiseCase.vy = out.case.vy;
        cruiseCase.omega_z = out.case.omega_z;

        cruiseCase.ax = 0.0;
        cruiseCase.ay = 0.0;

        cruiseCase.dt = out.case.dt;
        cruiseCase.current_theta = out.theta_steer;

        % 额定 / 巡航转向扭矩定义：
        % 稳态巡航，不考虑正在加速打舵，因此 alpha_steer = 0。
        cruiseCase.alpha_steer = zeros(1, 4);
    end

    %% =========================================================================
    %                               UI 数据
    % =========================================================================

    function [rows, fields] = makeRobotParamRows(pLocal, choice)
        if contains(choice, 'Wheel-Leg')
            rows = {
                '轮腿机器人总质量', getField(pLocal, 'wl_m_total', 20.0), 'kg';
                'Wheel-Leg 动力学模型尚未接入本 Dashboard', NaN, '-';
            };

            fields = {
                'wl_m_total';
                '';
            };
            return;
        end

        rows = {
            '全车质量', getField(pLocal, 'swerve_m_total', 25.0), 'kg';
            '前后轴距', getField(pLocal, 'swerve_wheel_base_x', 270.0), 'mm';
            '左右轮距', getField(pLocal, 'swerve_wheel_base_y', 270.0), 'mm';
            '轮胎外半径', getField(pLocal, 'swerve_wheel_radius', 42.5), 'mm';
            '轮毂包胶厚度', getField(pLocal, 'swerve_rubber_thickness', 10.0), 'mm';
            '轮胎宽度', getField(pLocal, 'swerve_wheel_width', 30.0), 'mm';
            '重心高度', getField(pLocal, 'swerve_h_cog', 200.0), 'mm';
            '转向机构转动惯量', getField(pLocal, 'swerve_I_steer', 0.015), 'kg·m²';
            '目标最大线加速度', getField(pLocal, 'swerve_target_max_a', 3.0), 'm/s²';
            '目标最大转向角加速度', getField(pLocal, 'swerve_target_max_alpha', 20.0), 'rad/s²';
        };

        fields = {
            'swerve_m_total';
            'swerve_wheel_base_x';
            'swerve_wheel_base_y';
            'swerve_wheel_radius';
            'swerve_rubber_thickness';
            'swerve_wheel_width';
            'swerve_h_cog';
            'swerve_I_steer';
            'swerve_target_max_a';
            'swerve_target_max_alpha';
        };
    end

    function [rows, fields] = makeEngineeringParamRows(pLocal, choice)
        if contains(choice, 'Wheel-Leg')
            rows = {
                'Wheel-Leg 工程常数待后续建模', NaN, '-';
            };

            fields = {
                '';
            };
            return;
        end

        rows = {
            '驱动减速比', getField(pLocal, 'swerve_i_drive', 1.0), '-';
            '转向减速比', getField(pLocal, 'swerve_i_steer', 1.0), '-';
            '驱动电机最高转速', getField(pLocal, 'swerve_motor_max_rpm', 450), 'rpm';
            '转向电机最高转速', getField(pLocal, 'swerve_steer_max_rpm', 120), 'rpm';
            '基准摩擦系数', getField(pLocal, 'swerve_mu_ground', 0.8), '-';
            '地胶阻力放大系数', getField(pLocal, 'swerve_carpet_factor', 4.0), '-';
            '滚动阻力系数', getField(pLocal, 'swerve_roll_resistance', 0.018), '-';
            '滑移效率', getField(pLocal, 'swerve_eta_slip', 0.9), '-';
            '轮胎邵氏硬度', getField(pLocal, 'swerve_hardness_shoreA', 60), 'Shore A';
            '驱动机械损耗扭矩', getField(pLocal, 'swerve_T_mech_drive', 0.1), 'N·m';
            '转向机械损耗扭矩', getField(pLocal, 'swerve_T_mech_steer', 0.1), 'N·m';
            '安全冗余系数', getField(pLocal, 'swerve_redundancy', 1.2), '-';
        };

        fields = {
            'swerve_i_drive';
            'swerve_i_steer';
            'swerve_motor_max_rpm';
            'swerve_steer_max_rpm';
            'swerve_mu_ground';
            'swerve_carpet_factor';
            'swerve_roll_resistance';
            'swerve_eta_slip';
            'swerve_hardness_shoreA';
            'swerve_T_mech_drive';
            'swerve_T_mech_steer';
            'swerve_redundancy';
        };
    end

    function colNames = getResultTableColumnNames()
        colNames = { ...
            '轮位', ...
            '驱动速度 m/s', ...
            '舵角 deg', ...
            '舵速 rad/s', ...
            '舵角加速度 rad/s²', ...
            '额定驱动扭矩 N·m', ...
            '额定转向扭矩 N·m', ...
            '当前驱动扭矩 N·m', ...
            '当前转向扭矩 N·m', ...
            '法向载荷 N', ...
            '轮胎下压 mm', ...
            '接触半长 mm', ...
            '抓地利用率', ...
            '打滑状态', ...
            '触底状态' ...
        };
    end

    function names = getResultTableInternalNames()
        names = { ...
            'Wheel', ...
            'DriveSpeed_mps', ...
            'SteerAngle_deg', ...
            'SteerRate_radps', ...
            'AlphaSteer_radps2', ...
            'RatedDriveTorque_Nm', ...
            'RatedSteerTorque_Nm', ...
            'CurrentDriveTorque_Nm', ...
            'CurrentSteerTorque_Nm', ...
            'Fz_N', ...
            'TireCompression_mm', ...
            'ContactHalfLen_mm', ...
            'GripUsage', ...
            'SlipState', ...
            'BottomingState' ...
        };
    end

    function resultT = makeEmptyResultTable()
        internalNames = getResultTableInternalNames();
        variableTypes = [{'string'}, repmat({'double'}, 1, 12), {'string'}, {'string'}];

        resultT = table( ...
            'Size', [0, numel(internalNames)], ...
            'VariableTypes', variableTypes, ...
            'VariableNames', internalNames);
    end

    function resultT = makeResultTableFromEval(out, cruiseOut)
        slipText = repmat("正常", 4, 1);
        slipIdx = find(logical(out.slip_flag(:)));

        if ~isempty(slipIdx)
            slipText(slipIdx) = "打滑";
        end

        bottomText = repmat("正常", 4, 1);
        bottomIdx = find(logical(out.bottoming_flag(:)));

        if ~isempty(bottomIdx)
            bottomText(bottomIdx) = "触底";
        end

        resultT = table( ...
            string(out.wheel_names(:)), ...
            round(out.v_drive(:), 4), ...
            round(out.theta_steer(:) * 180/pi, 2), ...
            round(out.omega_steer(:), 4), ...
            round(out.alpha_steer(:), 4), ...
            round(cruiseOut.drive_torque(:), 4), ...
            round(cruiseOut.steer_torque(:), 4), ...
            round(out.drive_torque(:), 4), ...
            round(out.steer_torque(:), 4), ...
            round(out.dyn_debug.Fz(:), 2), ...
            round(out.dyn_debug.delta(:) * 1000, 5), ...
            round(out.dyn_debug.contact_half_len(:) * 1000, 5), ...
            round(out.dyn_debug.grip_usage(:), 3), ...
            slipText, ...
            bottomText, ...
            'VariableNames', getResultTableInternalNames());
    end

    function updateReportFromEval(choice, out, cruiseOut)
        slipIdx = find(logical(out.slip_flag(:)));

        if isempty(slipIdx)
            slipListText = '无';
        else
            slipListText = strjoin(cellstr(out.wheel_names(slipIdx)), ', ');
        end

        robotName = string(choice);
        robotName = erase(robotName, "Swerve 四轮舵轮 - ");
        robotName = erase(robotName, "Wheel-Leg 双轮轮腿 - ");

        maxDeltaWheelName = char(out.wheel_names(out.max_delta_wheel_idx));
        maxGripWheelName = char(out.wheel_names(out.max_grip_wheel_idx));

        setMetricCard(1, char(robotName));
        setMetricCard(2, sprintf('前向 %.2f m/s, 右向 %.2f m/s, 角速 %.2f rad/s', ...
            out.case.vx, out.case.vy, out.case.omega_z));
        setMetricCard(3, sprintf('前向 %.2f m/s², 右向 %.2f m/s²', ...
            out.case.ax, out.case.ay));

        setMetricCard(4, sprintf('%.3f m/s', out.limits.max_drive_speed));
        setMetricCard(5, sprintf('%.3f rad/s', out.limits.max_spin_rate));
        setMetricCard(6, sprintf('%.3f rad/s', out.limits.max_steer_rate));

        setMetricCard(7, sprintf('%.4f N·m', cruiseOut.max_drive_torque));
        setMetricCard(8, sprintf('%.4f N·m', cruiseOut.max_steer_torque));
        setMetricCard(9, '同速度，零加速度，稳态转向');

        setMetricCard(10, sprintf('%.4f N·m', out.max_drive_torque));
        setMetricCard(11, sprintf('%.4f N·m', out.max_steer_torque));
        setMetricCard(12, sprintf('mu %.3f / E %.2f MPa', ...
            out.dyn_debug.mu, out.dyn_debug.E / 1e6));

        setMetricCard(13, sprintf('%s %.5f mm', ...
            maxDeltaWheelName, out.max_tire_delta * 1000));
        setMetricCard(14, sprintf('%s %.3f', ...
            maxGripWheelName, out.max_grip_usage));
        setMetricCard(15, slipListText);
    end

    %% =========================================================================
    %                               画图函数
    % =========================================================================

    function drawTorqueChart(axHandle, names, driveT, steerT, ratedDriveT, ratedSteerT)
        cla(axHandle);

        x = 1:4;

        bar(axHandle, x, ...
            [ratedDriveT(:), ratedSteerT(:), driveT(:), steerT(:)], ...
            'grouped');

        axHandle.XTick = x;
        axHandle.XTickLabel = cellstr(names);
        ylabel(axHandle, '扭矩 / N·m');
        title(axHandle, '额定 / 当前工况驱动与转向扭矩');
        legend(axHandle, ...
            {'额定驱动', '额定转向', '当前驱动', '当前转向'}, ...
            'Location', 'best');
        grid(axHandle, 'on');
    end

    function drawFzChart(axHandle, names, Fz)
        cla(axHandle);

        x = 1:4;
        bar(axHandle, x, Fz(:));

        axHandle.XTick = x;
        axHandle.XTickLabel = cellstr(names);
        ylabel(axHandle, '法向载荷 / N');
        title(axHandle, '四轮法向载荷');
        grid(axHandle, 'on');
    end

    function drawDeformationChart(axHandle, names, pLocal, dynDbg)
        cla(axHandle);

        % pLocal 中长度参数单位为 mm，dynDbg.delta/contact_half_len 单位为 m（SI内部计算）
        R = pLocal.swerve_wheel_radius;          % mm
        tRubber = pLocal.swerve_rubber_thickness; % mm
        RHub = R - tRubber;                       % mm

        [deltaActual_m, idx] = max(dynDbg.delta);
        contactActual_m = dynDbg.contact_half_len(idx);

        % 转换为 mm 用于显示
        deltaActual = deltaActual_m * 1000;   % mm
        contactActual = contactActual_m * 1000; % mm

        % 仅用于显示的可视化放大，不改变实际结果。
        visualScale = 25;
        deltaVisual = deltaActual * visualScale;
        deltaVisual = min(deltaVisual, 0.65 * tRubber);

        contactVisual = sqrt(max(0, 2 * R * deltaVisual - deltaVisual^2));

        theta = linspace(0, 2*pi, 500);

        x0 = R * cos(theta);
        z0 = R + R * sin(theta);

        x1 = R * cos(theta);
        z1 = R - deltaVisual + R * sin(theta);
        z1(z1 < 0) = 0;

        plot(axHandle, x0, z0, '--', 'LineWidth', 1.0);
        hold(axHandle, 'on');

        hubTheta = linspace(0, 2*pi, 500);
        plot(axHandle, RHub * cos(hubTheta), (R - deltaVisual + RHub * sin(hubTheta)), ':', 'LineWidth', 1.2);
        plot(axHandle, x1, z1, 'LineWidth', 1.8);
        plot(axHandle, [-1.25 * R, 1.25 * R], [0, 0], 'k-', 'LineWidth', 1.2);
        plot(axHandle, [-contactVisual, contactVisual], [0, 0], 'LineWidth', 5);
        plot(axHandle, 0, (R - deltaVisual), 'o', 'MarkerSize', 5);

        text(axHandle, -1.22 * R, 2.05 * R, ...
            sprintf([ ...
                '显示轮: %s\n', ...
                '实际下压: %.5f mm\n', ...
                '包胶厚度: %.2f mm\n', ...
                '接触半长: %.5f mm\n', ...
                '示意放大: x%d' ...
            ], ...
            char(names(idx)), ...
            deltaActual, ...
            tRubber, ...
            contactActual, ...
            visualScale), ...
            'FontSize', 10);

        hold(axHandle, 'off');

        axis(axHandle, 'equal');
        xlim(axHandle, [-1.35 * R, 1.35 * R]);
        ylim(axHandle, [-0.12 * R, 2.35 * R]);
        xlabel(axHandle, '横向尺寸 / mm');
        ylabel(axHandle, '高度 / mm');
        title(axHandle, '轮胎下压形变示意，显示最大载荷轮');
        grid(axHandle, 'on');
    end

    %% =========================================================================
    %                               显示辅助
    % =========================================================================

    function cardLabel = makeMetricCard(parent, titleText, valueText)
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

        resultTable.Data = makeEmptyResultTable();
        resultTable.ColumnName = getResultTableColumnNames();

        clearAxisWithMessage(axTorque, 'Wheel-Leg 暂未接入');
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
        setMetricCard(2, stripHtmlTags(ME.message));

        resultTable.Data = makeEmptyResultTable();
        resultTable.ColumnName = getResultTableColumnNames();

        clearAxisWithMessage(axTorque, '计算失败');
        clearAxisWithMessage(axFz, '计算失败');
        clearAxisWithMessage(axDeform, '计算失败');
    end

    function outText = stripHtmlTags(inText)
        outText = regexprep(char(inText), '<[^>]*>', '');
    end

    %% =========================================================================
    %                               校验与工具
    % =========================================================================

    function checkDependencies()
        deps = { ...
            'Config_Params', ...
            'Swerve_Get_Preset', ...
            'Swerve_Performance_Limits', ...
            'Swerve_Kinematics_Solver', ...
            'Swerve_Dynamics_Core', ...
            'Swerve_Evaluate_Case' ...
        };

        for k = 1:numel(deps)
            name = deps{k};
            if exist(name, 'file') ~= 2 && exist(name, 'class') ~= 8
                error('Swerve_UI_Dashboard:MissingDependency', ...
                    '缺少依赖文件或类: %s。请确认它位于 MATLAB 当前路径。', name);
            end
        end
    end

    function validateEvalOutput(out)
        requiredTopFields = { ...
            'case', ...
            'p', ...
            'limits', ...
            'v_drive', ...
            'theta_steer', ...
            'omega_steer', ...
            'alpha_steer', ...
            'drive_torque', ...
            'steer_torque', ...
            'slip_flag', ...
            'dyn_debug', ...
            'wheel_names', ...
            'max_drive_torque', ...
            'max_steer_torque', ...
            'max_grip_usage', ...
            'max_tire_delta', ...
            'max_delta_wheel_idx', ...
            'max_grip_wheel_idx' ...
        };

        for k = 1:numel(requiredTopFields)
            name = requiredTopFields{k};
            if ~isfield(out, name)
                error('Swerve_UI_Dashboard:InvalidEvalOutput', ...
                    'Swerve_Evaluate_Case 输出缺少字段: %s。', name);
            end
        end

        requiredDynFields = { ...
            'mu', ...
            'E', ...
            'Fz', ...
            'grip_usage', ...
            'delta', ...
            'contact_half_len', ...
            'rubber_thickness', ...
            'R_hub' ...
        };

        for k = 1:numel(requiredDynFields)
            name = requiredDynFields{k};
            if ~isfield(out.dyn_debug, name)
                error('Swerve_UI_Dashboard:InvalidDynDebug', ...
                    'out.dyn_debug 缺少字段: %s。请确认 Swerve_Dynamics_Core 已更新。', name);
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
        signedFields = "none";

        ok = any(strcmp(string(fieldName), signedFields));
    end
end