classdef Config_Params
    % Config_Params: 四轮舵轮机器人动力学全局参数字典
    
    properties (Constant)
        %% 1. 几何与质量参数 [单位: m, kg]
        wheel_base_x = 0.2700;    % 纵向轮距
        wheel_base_y = 0.2700;    % 横向轮距
        wheel_radius = 0.0425;    % 轮半径
        wheel_width  = 0.030;     % 轮宽
        h_cog        = 0.2;      % 重心高度
        m_total      = 25.0;      % 全车重量
        g            = 9.81;      % 重力加速度
        I_steer      = 0.015;     % 转向模块转动惯量
        
        %% 2. 传动与电机物理边界
        i_drive       = 1.0;     % 驱动减速比
        i_steer       = 1.0;      % 舵向传动比
        motor_max_rpm = 450;     % 驱动电机转子额定转速 (RPM)
        
        %% 3. 极限机动指标 
        % 舵轮为全向底盘，设定合成矢量加速度，代码会自动按最恶劣工况分解
        target_max_a     = 3.0;   % 预期最大平移加速度 (m/s^2)
        target_max_alpha = 20.0;  % 舵向最大瞬态角加速度 (rad/s^2)
        
        %% 4. 环境与工程经验常数 
        mu_ground          = 0.8;  % 地面摩擦系数
        carpet_factor      = 4.0;  % 地胶阻力放大系数 
        T_mech_drive_wheel = 0;  % 驱动轴端机械摩擦损耗 (N·m)
        T_mech_steer_axis  = 0;  % 转向轴端机械摩擦损耗 (N·m)
        
        %% 5. 材料与安全参数
        hardness_shoreA   = 60;    % 邵氏硬度
        redundancy_factor = 1;   % 安全冗余
    end
end