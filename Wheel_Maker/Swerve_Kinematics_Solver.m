function [v_drive_out, theta_steer_out, omega_steer_out, debug] = ...
    Swerve_Kinematics_Solver(vx, vy, omega_z, current_theta, dt, p)
% SWERVE_KINEMATICS_SOLVER 四轮舵轮运动学解算器
%
% 坐标约定:
%   X: 向前为正
%   Y: 向右为正
%   omega_z: 逆时针为正，俯视机器人
%
% 轮序:
%   [FL, FR, RR, RL]
%   左前、右前、右后、左后
%
% 输入:
%   vx            - 底盘前向速度 m/s
%   vy            - 底盘右向速度 m/s
%   omega_z       - 底盘角速度 rad/s，逆时针为正
%   current_theta - 当前四个舵轮角度，1x4，rad
%   dt            - 控制周期，s
%   p             - 参数结构体，可选
%
% 输出:
%   v_drive_out      - 四个轮子的驱动线速度 m/s
%   theta_steer_out  - 四个轮子的目标舵角 rad
%   omega_steer_out  - 四个轮子的转向角速度 rad/s
%   debug            - 调试信息

    if nargin < 6 || isempty(p)
        p = Config_Params.toStruct();
    else
        p = Config_Params.mergeWithDefaults(p);
    end

    if nargin < 5 || isempty(dt) || dt <= 0
        dt = 0.02;
    end

    if nargin < 4 || isempty(current_theta)
        current_theta = zeros(1, 4);
    end

    if numel(current_theta) ~= 4
        error('Swerve_Kinematics_Solver:InputError', ...
            'current_theta 必须包含 4 个元素，对应 [FL, FR, RR, RL]。');
    end

    current_theta = reshape(current_theta, 1, 4);

    Lx = p.swerve_wheel_base_x / 2;
    Ly = p.swerve_wheel_base_y / 2;

    % 位置矩阵，坐标为 [x, y_right]
    % FL: 左前，y 为负
    % FR: 右前，y 为正
    % RR: 右后，y 为正
    % RL: 左后，y 为负
    pos_matrix = [
         Lx, -Ly;
         Lx,  Ly;
        -Lx,  Ly;
        -Lx, -Ly
    ];

    max_steer_w = (p.swerve_steer_max_rpm / p.swerve_i_steer) * (2*pi/60);
    max_drive_v = (p.swerve_motor_max_rpm / p.swerve_i_drive) * ...
        (2*pi/60) * p.swerve_wheel_radius;

    v_drive_out = zeros(1, 4);
    theta_steer_out = zeros(1, 4);
    omega_steer_out = zeros(1, 4);

    raw_theta = zeros(1, 4);
    raw_speed = zeros(1, 4);
    optimized_flip = false(1, 4);

    for i = 1:4
        x_i = pos_matrix(i, 1);
        y_i = pos_matrix(i, 2);

        % 对于 X 前、Y 右、omega 逆时针为正：
        % 纯逆时针旋转时，前轮速度向左，即 vy 为负。
        v_ix = vx + omega_z * y_i;
        v_iy = vy - omega_z * x_i;

        v_target = hypot(v_ix, v_iy);
        theta_target = atan2(v_iy, v_ix);

        raw_theta(i) = theta_target;
        raw_speed(i) = v_target;

        % 舵轮最短路径优化：如果需要转超过 90°，反转轮速，舵角少转 180°
        delta_theta = localWrapToPi(theta_target - current_theta(i));

        if abs(delta_theta) > pi/2
            theta_target = localWrapToPi(theta_target - sign(delta_theta) * pi);
            v_target = -v_target;
            delta_theta = localWrapToPi(theta_target - current_theta(i));
            optimized_flip(i) = true;
        end

        % 转向角速度限制
        req_w = delta_theta / dt;

        if abs(req_w) > max_steer_w
            req_w = sign(req_w) * max_steer_w;
            actual_theta = localWrapToPi(current_theta(i) + req_w * dt);

            % 舵角跟不上时，驱动速度按角度误差投影降额
            angle_error = localWrapToPi(actual_theta - theta_target);
            v_target = v_target * cos(angle_error);

            theta_target = actual_theta;
        end

        % 驱动速度限制
        if abs(v_target) > max_drive_v
            v_target = sign(v_target) * max_drive_v;
        end

        v_drive_out(i) = v_target;
        theta_steer_out(i) = localWrapToPi(theta_target);
        omega_steer_out(i) = req_w;
    end

    debug = struct();
    debug.pos_matrix = pos_matrix;
    debug.max_steer_w = max_steer_w;
    debug.max_drive_v = max_drive_v;
    debug.raw_theta = raw_theta;
    debug.raw_speed = raw_speed;
    debug.optimized_flip = optimized_flip;
end

function theta = localWrapToPi(theta)
    theta = mod(theta + pi, 2*pi) - pi;
end