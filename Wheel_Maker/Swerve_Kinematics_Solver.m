function [v_drive_out, theta_steer_out, omega_steer_out, debug] = ...
    Swerve_Kinematics_Solver(vx, vy, omega_z, current_theta, dt, p)
% SWERVE_KINEMATICS_SOLVER 四轮舵轮运动学逆解求解器
%
% 描述:
%   将底盘全局期望速度 (vx, vy, omega_z) 逆解分配给四组独立舵轮，
%   内部集成了 "防绕线优选算法(Flip)" 以及真实的 "电机物理限幅(Saturation)" 延迟模拟。
%
% 输入:
%   vx, vy, omega_z - 底盘期望平移及自转速度
%   current_theta   - 1x4，底盘当前物理状态下的四轮偏航角
%   dt              - 控制解算周期 (s)，影响最大角速度截断
%
% 输出:
%   v_drive_out     - 1x4，解算出的轮端期望线速度 (m/s)
%   theta_steer_out - 1x4，解算出的最终偏转舵角 (rad)
%   omega_steer_out - 1x4，达到该目标角所需的转向角速度 (rad/s)

    if nargin < 6 || isempty(p)
        p = Config_Params.toStruct();
    else
        p = Config_Params.mergeWithDefaults(p);
    end

    if nargin < 5 || isempty(dt) || dt <= 0, dt = 0.02; end
    if nargin < 4 || isempty(current_theta), current_theta = zeros(1, 4); end
    if numel(current_theta) ~= 4
        error('Swerve_Kinematics_Solver:InputError', 'current_theta 必须包含 4 个元素。');
    end

    current_theta = reshape(current_theta, 1, 4);
    Lx = (p.swerve_wheel_base_x / 1000) / 2;
    Ly = (p.swerve_wheel_base_y / 1000) / 2;

    % 构建相对底盘质心的舵轮坐标矩阵 [x, y_right]
    % 遵守 1:FL, 2:FR, 3:RR, 4:RL 轮序
    pos_matrix = [
         Lx, -Ly;  % FL: 左前，y为负
         Lx,  Ly;  % FR: 右前，y为正
        -Lx,  Ly;  % RR: 右后，y为正
        -Lx, -Ly   % RL: 左后，y为负
    ];

    % 获取电机减速后的物理速度上限
    max_steer_w = (p.swerve_steer_max_rpm / p.swerve_i_steer) * (2*pi/60);
    max_drive_v = (p.swerve_motor_max_rpm / p.swerve_i_drive) * (2*pi/60) * (p.swerve_wheel_radius / 1000);

    v_drive_out = zeros(1, 4);
    theta_steer_out = zeros(1, 4);
    omega_steer_out = zeros(1, 4);

    raw_theta = zeros(1, 4);
    raw_speed = zeros(1, 4);
    optimized_flip = false(1, 4);

    %% 逐轮解算
    for i = 1:4
        x_i = pos_matrix(i, 1);
        y_i = pos_matrix(i, 2);

        % 1. 标准逆解运算
        % 当发生逆时针自转(omega_z > 0)时，利用叉乘特性叠加速度向量
        v_ix = vx + omega_z * y_i;
        v_iy = vy - omega_z * x_i;

        v_target = hypot(v_ix, v_iy);
        theta_target = atan2(v_iy, v_ix);

        raw_theta(i) = theta_target;
        raw_speed(i) = v_target;

        % 2. 最短路径优化算法 (Flip Logic)
        % 判断当前轮子目标角度与实际角度之差，若偏转 > 90度，
        % 则舵角仅需反向旋转并在轮端输出负速度，减少绕线与电机做功。
        delta_theta = localWrapToPi(theta_target - current_theta(i));

        if abs(delta_theta) > pi/2
            theta_target = localWrapToPi(theta_target - sign(delta_theta) * pi);
            v_target = -v_target; % 驱动轮反转补偿
            delta_theta = localWrapToPi(theta_target - current_theta(i));
            optimized_flip(i) = true;
        end

        % 3. 物理限幅约束 (Saturation)
        req_w = delta_theta / dt;

        % 如果单个控制周期内无法达到目标舵角(即转向迟滞)
        if abs(req_w) > max_steer_w
            % 限幅：只能以电机的最大物理角速度转动
            req_w = sign(req_w) * max_steer_w;
            actual_theta = localWrapToPi(current_theta(i) + req_w * dt);

            % 舵角跟不上时，将驱动线速度在当前真实舵角方向做投影降额，避免底盘路径跑偏
            angle_error = localWrapToPi(actual_theta - theta_target);
            v_target = v_target * cos(angle_error);

            theta_target = actual_theta;
        end

        % 驱动速度硬件天花板限幅
        if abs(v_target) > max_drive_v
            v_target = sign(v_target) * max_drive_v;
        end

        % 赋值输出
        v_drive_out(i) = v_target;
        theta_steer_out(i) = localWrapToPi(theta_target);
        omega_steer_out(i) = req_w;
    end

    %% 封装调试信息
    debug = struct();
    debug.pos_matrix = pos_matrix;
    debug.max_steer_w = max_steer_w;
    debug.max_drive_v = max_drive_v;
    debug.raw_theta = raw_theta;
    debug.raw_speed = raw_speed;
    debug.optimized_flip = optimized_flip;
end

% 角度归一化辅助函数 (将任意角映射至 [-pi, pi] 之间)
function theta = localWrapToPi(theta)
    theta = mod(theta + pi, 2*pi) - pi;
end