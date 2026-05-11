function p = Swerve_Get_Preset(presetName)
% SWERVE_GET_PRESET 舵轮底盘典型工况预设生成器
%
% 描述:
%   为不同战术需求（如轻量化步兵、重装英雄、双轮平衡步兵等）快速生成一套初始化参数。
%   通过修改此处的预设，可以一键在 UI 仪表盘或主程序中对比不同方案的性能差异。
%
% 输入:
%   presetName - (string/char) 预设配置名称
%
% 输出:
%   p          - (struct) 包含完整物理与工程常数的参数结构体

    if nargin < 1 || isempty(presetName)
        presetName = 'Swerve 四轮舵轮 - 默认';
    end

    % 基于全局配置字典获取初始基准值
    p = Config_Params.toStruct();

    % [预设 1] 轻量化高机动配置 (模拟步兵)
    % 特点：轴距小、轮径小、整车极轻。追求极高的瞬态加速度，但对场地适应性稍弱。
    if contains(presetName, '轻量')
        p.swerve_m_total = 15.0;
        p.swerve_wheel_base_x = 230.0;
        p.swerve_wheel_base_y = 230.0;
        p.swerve_wheel_radius = 35.0;
        p.swerve_rubber_thickness = 8.0;
        p.swerve_wheel_width = 26.0;
        p.swerve_h_cog = 160.0;
        p.swerve_motor_max_rpm = 600;
        p.swerve_steer_max_rpm = 150;
        p.swerve_target_max_a = 3.5;

    % [预设 2] 重装高稳定配置 (模拟英雄或工程)
    % 特点：底盘宽大、轮宽增加、质心较高。加速度受限，但抗冲击和越障能力极强。
    elseif contains(presetName, '重载')
        p.swerve_m_total = 35.0;
        p.swerve_wheel_base_x = 320.0;
        p.swerve_wheel_base_y = 320.0;
        p.swerve_wheel_radius = 50.0;
        p.swerve_rubber_thickness = 12.0;
        p.swerve_wheel_width = 40.0;
        p.swerve_h_cog = 240.0;
        p.swerve_motor_max_rpm = 400;
        p.swerve_steer_max_rpm = 100;
        p.swerve_target_max_a = 2.5;

    % [预设 3] 未来轮腿底盘 (Wheel-Leg) 预留接口
    elseif contains(presetName, 'Wheel-Leg')
        if ~isfield(p, 'wl_m_total')
            p.wl_m_total = 20.0;
        end
    end
end