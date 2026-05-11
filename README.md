# Wheel_Maker 四轮舵轮底盘性能评估系统

## 项目简介

Wheel_Maker 是一个基于 MATLAB 的**四轮舵轮底盘性能评估与选型工具**，用于计算底盘运动学性能边界、动力学载荷分布、电机扭矩需求和轮胎打滑风险，为舵轮底盘硬件选型提供量化依据。

**核心功能**：运动学性能边界计算、逆运动学求解（防绕线优化）、动力学仿真（载荷转移、轮胎形变、扭矩计算、打滑判断）

**重要说明**：本工程是参数化仿真工具，不是实车控制代码，适用于设计早期的选型评估。

---

## 快速开始

### 运行方式

**命令行模式**：
```matlab
main  % 自动评估默认配置并输出报告
```

**图形界面模式**：
```matlab
Swerve_UI_Dashboard  % 启动交互式仪表盘
```

### 坐标系约定

- **X 轴**：车体前方为正
- **Y 轴**：车体右方为正
- **Z 轴**：向上为正
- **轮序**：1-FL（左前）、2-FR（右前）、3-RR（右后）、4-RL（左后）

---

## 核心计算原理

### 一、理论性能边界

**最大直线速度**：
```matlab
max_drive_speed = (motor_max_rpm / i_drive) * (2π / 60) * wheel_radius
```
电机最高转速经减速比折算到轮端线速度。

**最大自转角速度**：
```matlab
max_spin_rate = max_drive_speed / hypot(Lx, Ly)
```
轮组中心到质心的距离越大，自转角速度越小。

**最大转向角速度**：
```matlab
max_steer_rate = (steer_max_rpm / i_steer) * (2π / 60)
```
转向电机最高转速折算到舵角旋转速度。

---

### 二、运动学逆解

**速度分配**：
```
v_ix = vx + omega_z * y_i
v_iy = vy - omega_z * x_i
v_target = √(v_ix² + v_iy²)
theta_target = atan2(v_iy, v_ix)
```

**防绕线 Flip 优化**：
当目标舵角与当前舵角相差超过 90° 时，将目标舵角反向 180°，驱动速度取反，减少舵轮旋转角度。

**转向物理限幅**：
当转向电机角速度达到上限时，舵角只能部分到位，驱动速度按舵角误差的余弦投影降额，避免路径跑偏。

---

### 三、动力学核心

#### 3.1 轮胎材料模型

**邵氏硬度转杨氏模量**（Gent 公式）：
```matlab
E = ((0.0981 * (56 + 7.66 * s)) / (0.149 * (100 - s))) * 1e6  % Pa
```

**摩擦系数修正**（赫兹接触理论）：
```matlab
mu = mu_ground * (E_ref / E)^(2/3)
```
轮胎变硬时，接触面积减小，摩擦系数非线性下降。

#### 3.2 载荷转移模型

**静态法向载荷**：
```matlab
Fz_static = m * g / 4
```

**加速度引起的载荷转移**：
```matlab
dFz_x = (m * ax * h) / (2 * wheel_base_x)  % 纵向转移
dFz_y = (m * ay * h) / (2 * wheel_base_y)  % 横向转移
```

**四轮载荷分配**：
- FL（左前）：`Fz_static - dFz_x + dFz_y`
- FR（右前）：`Fz_static - dFz_x - dFz_y`
- RR（右后）：`Fz_static + dFz_x - dFz_y`
- RL（左后）：`Fz_static + dFz_x + dFz_y`

**物理解释**：向前加速时后轮增载、前轮减载；向右加速时左轮增载、右轮减载。

#### 3.3 轮胎形变

**下压形变**：
```matlab
delta_raw = Fz / (E * wheel_width)
delta = min(delta_raw, rubber_thickness)  % 不超过包胶厚度
```

**接触半长**：
```matlab
contact_half_len = √(2 * R_outer * delta - delta²)
```

**触底检测**：当 `delta_raw >= rubber_thickness * 0.95` 时触发警告。

#### 3.4 驱动扭矩

驱动扭矩 = 滚动阻力矩 + 机械损耗 + 牵引力扭矩

```matlab
T_roll = Fz * R_outer * roll_resistance * carpet_factor
T_traction = (F_total * R_outer) / eta_slip
T_drive_motor = (T_roll + T_mech_drive + T_traction) / i_drive * redundancy
```

#### 3.5 转向扭矩

转向扭矩 = 摩擦阻力矩 + 机械损耗 + 惯性扭矩

```matlab
M_f = mu * Fz * hypot(contact_half_len, wheel_width/2) / 1.5
T_inertia = I_steer * |alpha_steer|
T_steer_motor = (M_f + T_mech_steer + T_inertia) / i_steer * redundancy
```

#### 3.6 打滑判断

**抓地利用率**：
```matlab
grip_usage = F_total / (mu * Fz)
```

当 `grip_usage > 1.0` 时，所需抓地力超过轮胎摩擦极限，车轮打滑。

---

## 使用示例

### 评估自定义工况

```matlab
% 加载预设
p = Swerve_Get_Preset('Swerve 四轮舵轮 - 默认');

% 构造工况
caseIn.vx = 1.0;           % 前向速度 (m/s)
caseIn.vy = 0.2;           % 右向速度 (m/s)
caseIn.omega_z = 0.5;      % 自转角速度 (rad/s)
caseIn.ax = 2.0;           % 前向加速度 (m/s²)
caseIn.ay = 1.0;           % 右向加速度 (m/s²)
caseIn.dt = 0.02;          % 控制周期 (s)
caseIn.current_theta = zeros(1, 4);

% 评估
out = Swerve_Evaluate_Case(caseIn, p);

% 查看结果
disp(out.v_drive);         % 四轮驱动速度
disp(out.drive_torque);    % 四轮驱动扭矩
disp(out.steer_torque);    % 四轮转向扭矩
disp(out.dyn_debug.Fz);    % 四轮法向载荷
disp(out.slip_flag);       % 打滑标志
```

### 对比不同预设

```matlab
% 轻量配置（15kg，230mm轴距，3.5m/s²加速度）
p_light = Swerve_Get_Preset('Swerve 四轮舵轮 - 轻量');

% 重载配置（35kg，320mm轴距，2.5m/s²加速度）
p_heavy = Swerve_Get_Preset('Swerve 四轮舵轮 - 重载');

% 评估对比
out_light = Swerve_Evaluate_Case(caseIn, p_light);
out_heavy = Swerve_Evaluate_Case(caseIn, p_heavy);
```

### 自定义参数

```matlab
p = Swerve_Get_Preset('Swerve 四轮舵轮 - 默认');
p.swerve_m_total = 30.0;           % 修改质量
p.swerve_wheel_radius = 45.0;      % 修改轮半径
p.swerve_i_drive = 3.0;            % 修改减速比
p.swerve_hardness_shoreA = 70;     % 修改轮胎硬度
out = Swerve_Evaluate_Case(caseIn, p);
```

---

## 主要参数说明

| 参数 | 默认值 | 单位 | 含义 |
|------|--------|------|------|
| `swerve_m_total` | 25.0 | kg | 全车总质量 |
| `swerve_wheel_base_x` | 270.0 | mm | 轴距 |
| `swerve_wheel_base_y` | 270.0 | mm | 轮距 |
| `swerve_wheel_radius` | 42.5 | mm | 轮子外半径 |
| `swerve_rubber_thickness` | 10.0 | mm | 包胶厚度 |
| `swerve_h_cog` | 200.0 | mm | 质心高度 |
| `swerve_i_drive` | 1.0 | - | 驱动减速比 |
| `swerve_i_steer` | 1.0 | - | 转向减速比 |
| `swerve_motor_max_rpm` | 450 | rpm | 驱动电机最高转速 |
| `swerve_steer_max_rpm` | 120 | rpm | 转向电机最高转速 |
| `swerve_mu_ground` | 0.8 | - | 地面摩擦系数 |
| `swerve_hardness_shoreA` | 60 | Shore A | 轮胎邵氏硬度 |
| `swerve_redundancy` | 1.2 | - | 安全冗余系数 |

---

## 输出结果说明

| 字段 | 含义 |
|------|------|
| `v_drive` | 四轮驱动速度 (m/s) |
| `theta_steer` | 四轮舵角 (rad) |
| `drive_torque` | 四轮驱动扭矩 (N·m) |
| `steer_torque` | 四轮转向扭矩 (N·m) |
| `slip_flag` | 打滑标志 (0/1) |
| `bottoming_flag` | 包胶触底标志 (0/1) |
| `dyn_debug.Fz` | 四轮法向载荷 (N) |
| `dyn_debug.grip_usage` | 四轮抓地利用率 |
| `dyn_debug.delta` | 四轮轮胎下压 (m) |

---

## 注意事项

**模型假设**：
- 力分配按载荷比例，未实现最优分配
- 未考虑电机扭矩-转速曲线和热衰减
- 轮胎模型为工程近似，非完整 Pacejka 模型

**使用建议**：
- 适用于设计早期的硬件选型和方案对比
- 根据实际测试数据校准摩擦系数等参数
- 注意检查打滑标志和触底警告

---

## 测试验证

```matlab
Swerve_PhysicsConsistency_Test  % 物理一致性测试
Swerve_Debug_SmokeTest          % 冒烟测试
```

---

**最后更新**: 2026-05-11
