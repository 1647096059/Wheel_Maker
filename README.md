# Wheel Maker / Swerve Wheel Maker

四轮舵轮底盘性能评估、参数敏感性分析与选型辅助工具。

本项目基于 MATLAB 开发，面向 RoboMaster / 移动机器人四轮舵轮底盘的早期设计阶段，用于快速评估底盘几何、电机传动、轮胎参数、摩擦条件和测试工况对轮速、舵角、驱动扭矩、转向扭矩、法向载荷、抓地利用率与轮胎下压形变的影响。

当前版本的核心定位是：

> 用于四轮舵轮底盘早期方案比较、参数敏感性分析、设计讨论和教学展示的工程辅助工具。

当前版本不应直接作为最终电机选型、实车安全裕度认证或高保真车辆动力学仿真的唯一依据。

---

## 1. 项目特点

当前版本已经完成以下能力：

- 四轮舵轮底盘运动学解算。
- 四轮舵轮简化动力学与扭矩估算。
- 基于物理一致性修正后的载荷转移计算。
- 驱动电机与转向电机扭矩估算。
- 巡航额定扭矩与当前工况峰值扭矩对比。
- 抓地利用率与打滑风险判断。
- 轮胎下压形变与接触半长估算。
- 参数化机器人预设。
- 可交互 MATLAB Dashboard。
- 命令行评估报告。
- 物理一致性测试。
- 最小冒烟测试。
- Wheel-Leg 双轮轮腿模型预留入口。

---

## 2. 当前版本功能概览

### 2.1 Dashboard 可视化界面

`Swerve_UI_Dashboard.m` 是项目的主要交互入口。

当前 Dashboard 包含：

- 左侧机器人类型选择。
- 左侧机器人本体参数编辑。
- 左侧工程常数 / 环境 / 电机传动参数编辑。
- 当前测试工况输入。
- 右侧结果报文指标卡。
- 额定 / 当前工况驱动与转向扭矩图。
- 四轮法向载荷图。
- 放大的轮胎下压形变示意图。
- 中文逐轮结果表。

当前测试工况包含：

| 中文名称 | 内部变量 | 单位 | 说明 |
|---|---|---|---|
| 前向速度 | `vx` | m/s | 机器人本体系 X 方向速度 |
| 右向速度 | `vy` | m/s | 机器人本体系 Y 方向速度 |
| 角速度 | `omega_z` | rad/s | 俯视逆时针为正 |
| 控制周期 | `dt` | s | 用于舵角速度与转向角加速度估算 |
| 前向加速度 | `ax` | m/s² | 机器人本体系 X 方向真实加速度 |
| 右向加速度 | `ay` | m/s² | 机器人本体系 Y 方向真实加速度 |

---

### 2.2 命令行报告

`main.m` 提供命令行版本的性能评估报告。

它会输出：

- 当前机器人预设。
- 理论最大直线速度。
- 理论最大自转速度。
- 理论最大转向角速度。
- 静态载荷与基础阻力。
- 巡航工况扭矩。
- 极限 45° 爆发加速工况扭矩。
- 抓地力安全评估。
- 逐轮极限工况结果。

---

### 2.3 物理一致性测试

`Swerve_PhysicsConsistency_Test.m` 用于验证基础物理符号和几何关系。

当前测试包括：

1. 静态工况：四轮法向载荷相等。
2. 前向加速：后轮载荷大于前轮。
3. 前向刹车 / 反向加速：前轮载荷大于后轮。
4. 向右加速：左轮载荷大于右轮。
5. 向左加速：右轮载荷大于左轮。
6. 纯旋转运动学：四轮原始速度大小相等。

---

## 3. 坐标系与轮序约定

本项目统一采用如下坐标定义：

```text
X 轴：机器人前方为正
Y 轴：机器人右侧为正
Z 轴：机器人上方为正
omega_z：俯视机器人时逆时针为正
```

加速度定义：

```text
ax > 0：机器人向前加速，载荷向后轮转移
ax < 0：机器人向后加速 / 前向刹车，载荷向前轮转移

ay > 0：机器人向右加速，载荷向左轮转移
ay < 0：机器人向左加速，载荷向右轮转移
```

注意：

```text
ax / ay 表示机器人本体系下的真实加速度，不是达朗贝尔惯性等效加速度。
```

轮序统一为：

```text
[FL, FR, RR, RL]
```

对应关系：

| 编号 | 缩写 | 中文名称 |
|---|---|---|
| 1 | FL | 左前轮 |
| 2 | FR | 右前轮 |
| 3 | RR | 右后轮 |
| 4 | RL | 左后轮 |

所有四轮数组均默认遵循该顺序，包括轮速、舵角、舵角速度、驱动扭矩、转向扭矩、法向载荷、抓地利用率、轮胎下压形变和打滑标志。

---

## 4. 项目文件结构

推荐项目目录如下：

```text
Wheel_Maker/
├── Config_Params.m
├── Swerve_Get_Preset.m
├── Swerve_Performance_Limits.m
├── Swerve_Kinematics_Solver.m
├── Swerve_Dynamics_Core.m
├── Swerve_Evaluate_Case.m
├── Swerve_UI_Dashboard.m
├── Swerve_PhysicsConsistency_Test.m
├── Swerve_Debug_SmokeTest.m
├── main.m
└── README.md
```

各文件职责如下：

| 文件 | 作用 |
|---|---|
| `Config_Params.m` | 默认参数字典 |
| `Swerve_Get_Preset.m` | 机器人预设管理 |
| `Swerve_Performance_Limits.m` | 理论最大直线速度、自转速度、转向角速度计算 |
| `Swerve_Kinematics_Solver.m` | 四轮舵轮运动学解算 |
| `Swerve_Dynamics_Core.m` | 动力学、载荷、扭矩、轮胎形变与抓地计算 |
| `Swerve_Evaluate_Case.m` | 单工况统一评估接口 |
| `Swerve_UI_Dashboard.m` | MATLAB 图形化交互界面 |
| `Swerve_PhysicsConsistency_Test.m` | 物理一致性测试 |
| `Swerve_Debug_SmokeTest.m` | 冒烟测试 |
| `main.m` | 命令行评估报告 |

---

## 5. 架构设计

当前版本已经完成架构清理，推荐理解为以下分层：

```text
参数层
  ├── Config_Params.m
  └── Swerve_Get_Preset.m

核心计算层
  ├── Swerve_Performance_Limits.m
  ├── Swerve_Kinematics_Solver.m
  └── Swerve_Dynamics_Core.m

统一评估层
  └── Swerve_Evaluate_Case.m

展示与入口层
  ├── main.m
  └── Swerve_UI_Dashboard.m

测试层
  ├── Swerve_PhysicsConsistency_Test.m
  └── Swerve_Debug_SmokeTest.m
```

### 5.1 架构原则

当前版本遵循以下职责边界：

- `Swerve_UI_Dashboard.m` 只负责界面、输入读取、结果展示和画图。
- 物理计算统一收敛到 `Swerve_Dynamics_Core.m`。
- 单工况评估统一由 `Swerve_Evaluate_Case.m` 调度。
- 理论速度边界统一由 `Swerve_Performance_Limits.m` 计算。
- 机器人预设统一由 `Swerve_Get_Preset.m` 管理。
- `main.m` 和 Dashboard 共享同一套核心评估逻辑。

---

## 6. 快速开始

### 6.1 启动前准备

打开 MATLAB，并将当前工作目录切换到项目根目录。

建议先执行：

```matlab
clear classes; clear functions; clc;
```

如果你刚刚修改过 `Config_Params.m` 或其他 `classdef` / 函数文件，也建议先执行上述清理命令。

---

### 6.2 运行物理一致性测试

```matlab
Swerve_PhysicsConsistency_Test
```

如果正常，应看到类似输出：

```text
========== Swerve Physics Consistency Test ==========
[1] 静态工况测试：四轮法向载荷均等...
    PASS
[2] 前向加速测试：ax > 0 时后轮载荷应大于前轮...
    PASS
...
========== Physics Consistency Test Passed ==========
```

---

### 6.3 运行冒烟测试

```matlab
Swerve_Debug_SmokeTest
```

该测试会依次检查：

- 参数加载。
- 运动学求解。
- 动力学求解。
- `main.m` 报告输出。
- 物理一致性测试。

---

### 6.4 启动 Dashboard

```matlab
Swerve_UI_Dashboard
```

---

### 6.5 运行命令行报告

```matlab
main
```

---

## 7. 推荐运行顺序

第一次使用或修改核心代码后，推荐按如下顺序运行：

```matlab
clear classes; clear functions; clc;
Swerve_PhysicsConsistency_Test
Swerve_Debug_SmokeTest
main
Swerve_UI_Dashboard
```

---

## 8. 默认参数说明

默认参数保存在 `Config_Params.m` 中。

### 8.1 公共常数

| 参数 | 默认值 | 单位 | 说明 |
|---|---:|---|---|
| `g` | 9.81 | m/s² | 重力加速度 |

---

### 8.2 Swerve 机器人几何与质量参数

| 参数 | 默认值 | 单位 | 说明 |
|---|---:|---|---|
| `swerve_m_total` | 25.0 | kg | 整车质量 |
| `swerve_wheel_base_x` | 0.2700 | m | 前后轴距 |
| `swerve_wheel_base_y` | 0.2700 | m | 左右轮距 |
| `swerve_wheel_radius` | 0.0425 | m | 轮胎半径 |
| `swerve_wheel_width` | 0.030 | m | 轮胎宽度 |
| `swerve_h_cog` | 0.2 | m | 重心高度 |
| `swerve_I_steer` | 0.015 | kg·m² | 转向机构转动惯量 |

---

### 8.3 电机与传动参数

| 参数 | 默认值 | 单位 | 说明 |
|---|---:|---|---|
| `swerve_i_drive` | 1.0 | - | 驱动减速比 |
| `swerve_i_steer` | 1.0 | - | 转向减速比 |
| `swerve_motor_max_rpm` | 450 | rpm | 驱动电机最高转速 |
| `swerve_steer_max_rpm` | 120 | rpm | 转向电机最高转速 |

---

### 8.4 运动性能目标

| 参数 | 默认值 | 单位 | 说明 |
|---|---:|---|---|
| `swerve_target_max_a` | 3.0 | m/s² | 目标最大线加速度 |
| `swerve_target_max_alpha` | 20.0 | rad/s² | 目标最大转向角加速度 |

---

### 8.5 工程常数与环境参数

| 参数 | 默认值 | 单位 | 说明 |
|---|---:|---|---|
| `swerve_mu_ground` | 0.8 | - | 基准摩擦系数 |
| `swerve_carpet_factor` | 4.0 | - | 地胶阻力放大系数 |
| `swerve_roll_resistance` | 0.018 | - | 滚动阻力系数 |
| `swerve_eta_slip` | 0.9 | - | 滑移效率 |
| `swerve_hardness_shoreA` | 60 | Shore A | 轮胎邵氏硬度 |
| `swerve_T_mech_drive` | 0.1 | N·m | 驱动机械损耗扭矩 |
| `swerve_T_mech_steer` | 0.1 | N·m | 转向机械损耗扭矩 |
| `swerve_redundancy` | 1.2 | - | 安全冗余系数 |

---

### 8.6 Wheel-Leg 预留参数

| 参数 | 默认值 | 单位 | 说明 |
|---|---:|---|---|
| `wl_m_total` | 20.0 | kg | 双轮轮腿机器人质量，当前仅预留 |

当前 Wheel-Leg 入口只用于后续扩展，尚未接入实际运动学和动力学模型。

---

## 9. 机器人预设

机器人预设由 `Swerve_Get_Preset.m` 管理。

当前 Dashboard 中提供以下预设：

| 预设名称 | 说明 |
|---|---|
| `Swerve 四轮舵轮 - RM2026 默认` | 默认四轮舵轮底盘 |
| `Swerve 四轮舵轮 - 轻量小车预设` | 较轻质量、较小轴距、较高电机转速 |
| `Swerve 四轮舵轮 - 重载底盘预设` | 较大质量、较大轴距、较大轮径 |
| `Wheel-Leg 双轮轮腿 - 预留` | 轮腿模型预留入口 |

---

## 10. 核心函数说明

## 10.1 `Swerve_Performance_Limits`

```matlab
limits = Swerve_Performance_Limits(p)
```

用于计算理论性能边界。

输出字段：

| 字段 | 说明 |
|---|---|
| `limits.max_drive_speed` | 理论最大直线速度 |
| `limits.max_spin_rate` | 理论最大自转角速度 |
| `limits.max_steer_rate` | 理论最大转向角速度 |

---

## 10.2 `Swerve_Kinematics_Solver`

```matlab
[v_drive_out, theta_steer_out, omega_steer_out, debug] = ...
    Swerve_Kinematics_Solver(vx, vy, omega_z, current_theta, dt, p)
```

功能：

- 计算四个舵轮的驱动速度。
- 计算目标舵角。
- 计算舵角速度。
- 执行舵角最短路径优化。
- 执行转向角速度饱和。
- 执行驱动速度饱和。

输入：

| 参数 | 单位 | 说明 |
|---|---|---|
| `vx` | m/s | 前向速度 |
| `vy` | m/s | 右向速度 |
| `omega_z` | rad/s | 角速度 |
| `current_theta` | rad | 当前四轮舵角，1x4 |
| `dt` | s | 控制周期 |
| `p` | struct | 参数结构体 |

输出：

| 参数 | 单位 | 说明 |
|---|---|---|
| `v_drive_out` | m/s | 四轮驱动速度 |
| `theta_steer_out` | rad | 四轮目标舵角 |
| `omega_steer_out` | rad/s | 四轮舵角速度 |
| `debug` | struct | 运动学调试信息 |

---

## 10.3 `Swerve_Dynamics_Core`

```matlab
[Drive_Torque, Steer_Torque, Slip_Warning, debug] = ...
    Swerve_Dynamics_Core(ax, ay, alpha_steer, p)
```

功能：

- 根据当前加速度工况计算四轮法向载荷。
- 计算载荷转移。
- 计算轮胎等效弹性模量。
- 计算等效摩擦系数。
- 估算驱动扭矩。
- 估算转向扭矩。
- 计算抓地利用率。
- 判断打滑。
- 输出轮胎下压形变与接触半长。

输入：

| 参数 | 单位 | 说明 |
|---|---|---|
| `ax` | m/s² | 前向真实加速度 |
| `ay` | m/s² | 右向真实加速度 |
| `alpha_steer` | rad/s² | 四轮转向角加速度，1x4 |
| `p` | struct | 参数结构体 |

输出：

| 参数 | 说明 |
|---|---|
| `Drive_Torque` | 四轮驱动电机轴端扭矩 |
| `Steer_Torque` | 四轮转向电机轴端扭矩 |
| `Slip_Warning` | 四轮打滑标志 |
| `debug` | 动力学调试信息 |

`debug` 主要字段：

| 字段 | 说明 |
|---|---|
| `mu` | 等效摩擦系数 |
| `E` | 轮胎等效弹性模量 |
| `Fz_static` | 单轮静态法向载荷 |
| `dFz_x_per_wheel` | 单轮纵向载荷转移量 |
| `dFz_y_per_wheel` | 单轮横向载荷转移量 |
| `Fz_unclipped` | 未裁剪前法向载荷 |
| `Fz` | 裁剪后的法向载荷 |
| `Fx_i` | 各轮纵向力 |
| `Fy_i` | 各轮横向力 |
| `F_total_i` | 各轮合力 |
| `muFz` | 各轮最大摩擦力 |
| `grip_usage` | 抓地利用率 |
| `delta` | 轮胎下压形变 |
| `contact_half_len` | 接触半长 |

---

## 10.4 `Swerve_Evaluate_Case`

```matlab
out = Swerve_Evaluate_Case(caseIn, p)
```

这是当前项目的统一单工况评估接口。

输入结构体 `caseIn`：

| 字段 | 单位 | 说明 |
|---|---|---|
| `vx` | m/s | 前向速度 |
| `vy` | m/s | 右向速度 |
| `omega_z` | rad/s | 角速度 |
| `ax` | m/s² | 前向加速度 |
| `ay` | m/s² | 右向加速度 |
| `dt` | s | 控制周期 |
| `current_theta` | rad | 当前四轮舵角 |
| `alpha_steer` | rad/s² | 可选，转向角加速度 |

输出结构体 `out`：

| 字段 | 说明 |
|---|---|
| `case` | 当前工况输入 |
| `p` | 参数结构体 |
| `limits` | 理论性能边界 |
| `v_drive` | 四轮驱动速度 |
| `theta_steer` | 四轮目标舵角 |
| `omega_steer` | 四轮舵角速度 |
| `alpha_steer` | 四轮转向角加速度 |
| `drive_torque` | 四轮驱动扭矩 |
| `steer_torque` | 四轮转向扭矩 |
| `slip_flag` | 四轮打滑标志 |
| `kin_debug` | 运动学 debug |
| `dyn_debug` | 动力学 debug |
| `max_drive_torque` | 四轮最大驱动扭矩 |
| `max_steer_torque` | 四轮最大转向扭矩 |
| `max_grip_usage` | 最大抓地利用率 |
| `max_tire_delta` | 最大轮胎下压 |
| `max_contact_half_len` | 最大接触半长 |
| `max_grip_wheel_idx` | 最大抓地利用率对应轮编号 |
| `max_delta_wheel_idx` | 最大下压对应轮编号 |
| `wheel_names` | 轮名数组 `[FL, FR, RR, RL]` |

---

## 11. Dashboard 结果解释

### 11.1 结果报文

Dashboard 右上角结果报文包含：

| 指标 | 说明 |
|---|---|
| 机器人类型 | 当前选择的机器人预设 |
| 速度指令 | 当前前向速度、右向速度、角速度 |
| 加速度工况 | 当前前向加速度、右向加速度 |
| 最大直线速度 | 理论最大直线速度 |
| 最大自转速度 | 理论最大自转角速度 |
| 最大转向角速度 | 理论最大转向角速度 |
| 额定驱动扭矩 | 巡航工况下四轮最大驱动扭矩 |
| 额定转向扭矩 | 巡航工况下四轮最大转向扭矩 |
| 额定工况定义 | 同速度、零加速度、稳态转向 |
| 峰值驱动扭矩 | 当前测试工况下四轮最大驱动扭矩 |
| 峰值转向扭矩 | 当前测试工况下四轮最大转向扭矩 |
| 摩擦系数 / 弹性模量 | 当前等效摩擦系数与轮胎弹性模量 |
| 最大轮胎下压 | 四轮中最大轮胎下压形变 |
| 最大抓地利用率 | 四轮中最大抓地利用率 |
| 打滑轮 | 被判定为打滑的轮子 |

---

### 11.2 额定扭矩与当前工况扭矩

当前 Dashboard 同时显示：

```text
额定驱动扭矩
额定转向扭矩
当前驱动扭矩
当前转向扭矩
```

二者定义不同。

额定 / 巡航工况：

```text
保持当前速度指令
前向加速度 ax = 0
右向加速度 ay = 0
转向角加速度 alpha_steer = 0
```

当前工况：

```text
使用 Dashboard 中输入的 vx, vy, omega_z, ax, ay, dt
转向角加速度由当前舵角变化估算或由工况指定
```

因此：

- 额定扭矩更适合作为稳态巡航参考。
- 当前工况扭矩更适合作为瞬态机动或加速工况参考。
- 峰值扭矩通常会大于额定扭矩。

---

### 11.3 图表说明

当前 Dashboard 的图表区包含三个图：

| 图表 | 位置 | 说明 |
|---|---|---|
| 额定 / 当前工况驱动与转向扭矩 | 左上 | 对比四轮额定驱动、额定转向、当前驱动、当前转向扭矩 |
| 四轮法向载荷 | 左下 | 显示四个轮子的法向载荷 |
| 轮胎下压形变示意图 | 右侧整列 | 显示最大载荷轮的轮胎下压形变，经过可视化放大 |

---

### 11.4 逐轮结果表

逐轮结果表采用中文标题，包含：

| 中文列名 | 说明 |
|---|---|
| 轮位 | FL / FR / RR / RL |
| 驱动速度 m/s | 单轮驱动线速度 |
| 舵角 deg | 目标舵角 |
| 舵速 rad/s | 转向角速度 |
| 舵角加速度 rad/s² | 转向角加速度 |
| 额定驱动扭矩 N·m | 巡航工况驱动扭矩 |
| 额定转向扭矩 N·m | 巡航工况转向扭矩 |
| 当前驱动扭矩 N·m | 当前工况驱动扭矩 |
| 当前转向扭矩 N·m | 当前工况转向扭矩 |
| 法向载荷 N | 当前工况法向载荷 |
| 轮胎下压 mm | 当前工况轮胎下压形变 |
| 接触半长 mm | 当前工况接触半长 |
| 抓地利用率 | 当前工况抓地利用率 |
| 打滑状态 | 正常 / 打滑 |

---

## 12. 抓地利用率说明

抓地利用率定义为：

```text
GripUsage = F_total_i / (mu * Fz_i)
```

其中：

- `F_total_i` 为单轮所需合力。
- `mu` 为等效摩擦系数。
- `Fz_i` 为该轮法向载荷。

解释：

| 抓地利用率 | 含义 |
|---:|---|
| `< 0.5` | 抓地裕度较大 |
| `0.5 ~ 0.8` | 抓地利用明显 |
| `0.8 ~ 1.0` | 接近摩擦极限 |
| `> 1.0` | 简化模型下判定为打滑 |

---

## 13. 轮胎形变说明

当前轮胎下压形变采用简化材料模型估算：

```text
delta = Fz / (E * wheel_width)
```

接触半长采用简化几何关系估算。

注意：

- 该结果适合做趋势分析。
- 该结果不是高精度轮胎有限元分析。
- Dashboard 中的轮胎形变图为了便于观察，对形变量进行了可视化放大。
- 图中显示的实际下压数值仍为未放大的计算值。

---

## 14. 模型假设

当前版本是准静态估算模型，不是完整车辆动力学仿真器。

主要假设包括：

1. 机器人为刚体。
2. 地面水平。
3. 不考虑悬架动态。
4. 不考虑轮胎侧偏角动态。
5. 不考虑轮胎滑移率动态。
6. 不考虑电机转矩-转速曲线。
7. 不考虑电池电压限制。
8. 不考虑电流限制。
9. 不考虑电机热衰减。
10. 不考虑驱动轮转动惯量。
11. 不考虑完整的四轮力分配优化。
12. 不考虑底盘绕 z 轴角加速度对应的完整力矩平衡。
13. 纵向力和横向力当前按法向载荷比例分配。
14. Wheel-Leg 模型尚未接入。

---

## 15. 当前版本适用范围

适合用于：

- 四轮舵轮底盘早期设计。
- 轮径、质量、轴距、重心高度、减速比的敏感性分析。
- 驱动电机与转向电机扭矩量级预估。
- 设计方案横向比较。
- 队内机械 / 电控 / 算法沟通。
- MATLAB 教学与工程直觉训练。
- Dashboard 可视化演示。

不适合直接用于：

- 最终电机定型。
- 实车安全裕度认证。
- 高保真车辆动力学仿真。
- 轮胎高精度接触建模。
- 电机热设计。
- 电池与电调极限分析。
- 赛事级最终性能承诺。

---

## 16. 常见问题

### 16.1 修改参数后结果没有变化

建议重新运行：

```matlab
clear classes; clear functions; clc;
Swerve_UI_Dashboard
```

MATLAB 可能缓存了旧的函数或类定义。

---

### 16.2 Dashboard 报缺少依赖

确认以下文件均位于当前 MATLAB 路径下：

```text
Config_Params.m
Swerve_Get_Preset.m
Swerve_Performance_Limits.m
Swerve_Kinematics_Solver.m
Swerve_Dynamics_Core.m
Swerve_Evaluate_Case.m
Swerve_UI_Dashboard.m
```

---

### 16.3 文件名带有 `(1)` 或 `(2)` 后缀

MATLAB 函数文件名必须和函数名一致。

错误示例：

```text
Swerve_UI_Dashboard(2).m
Swerve_Dynamics_Core(1).m
```

应改为：

```text
Swerve_UI_Dashboard.m
Swerve_Dynamics_Core.m
```

---

### 16.4 选择 Wheel-Leg 后没有计算结果

这是正常现象。

当前 Wheel-Leg 仅为预留入口。后续需要新增：

```text
WheelLeg_Kinematics_Solver.m
WheelLeg_Dynamics_Core.m
Wheel-Leg 参数映射
Wheel-Leg 结果表与图表
```

---

### 16.5 为什么额定扭矩和峰值扭矩不同

额定扭矩使用巡航工况：

```text
同速度
零加速度
稳态转向
```

峰值扭矩使用当前测试工况：

```text
当前速度
当前加速度
当前转向动态
```

因此加速度越大、转向越剧烈、载荷转移越明显，峰值扭矩通常越高。

---

## 17. 测试说明

### 17.1 物理一致性测试

运行：

```matlab
Swerve_PhysicsConsistency_Test
```

该测试用于确认：

- 静态载荷是否均等。
- 前向加速是否向后轮转移载荷。
- 右向加速是否向左轮转移载荷。
- 纯旋转运动学是否几何一致。
- `delta` 和 `contact_half_len` 是否由动力学核心输出。

---

### 17.2 冒烟测试

运行：

```matlab
Swerve_Debug_SmokeTest
```

该测试用于确认：

- 参数能否正常加载。
- 运动学函数能否正常运行。
- 动力学函数能否正常运行。
- 主程序 `main.m` 能否正常输出。
- 基础测试链路是否完整。

---

## 18. 最小使用流程总结

```matlab
clear classes; clear functions; clc;
Swerve_PhysicsConsistency_Test
Swerve_Debug_SmokeTest
main
Swerve_UI_Dashboard
```

常用入口：

| 任务 | 命令 |
|---|---|
| 运行物理一致性测试 | `Swerve_PhysicsConsistency_Test` |
| 运行冒烟测试 | `Swerve_Debug_SmokeTest` |
| 生成命令行报告 | `main` |
| 打开可视化界面 | `Swerve_UI_Dashboard` |

---

## 19. 项目声明

本项目当前面向四轮舵轮底盘的早期设计和工程估算。

所有输出结果均依赖当前简化模型和输入参数。实际机器人性能还会受到电机、电调、电池、轮胎、地面、机械装配、控制器调参、结构刚度、热状态和实车动态响应等因素影响。

在进行最终设计定型前，应结合实车测试、传感器日志、电机电流数据和机械测试结果进行校准。
