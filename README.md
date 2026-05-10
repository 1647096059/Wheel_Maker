# Wheel Maker / Swerve Wheel Maker 使用文档

## 1. 项目简介

本项目是一个用于四轮舵轮底盘早期设计、参数估算和选型辅助的 MATLAB 工具。

当前版本主要支持：

- 四轮舵轮底盘，Swerve Drive。
- 机器人几何参数修改。
- 电机、传动、轮胎、摩擦、滚阻等工程常数修改。
- 底盘运动学解算。
- 简化动力学与扭矩估算。
- 抓地利用率和打滑风险判断。
- Dashboard 可视化展示。
- 最小冒烟测试。

当前版本也预留了 Wheel-Leg 双轮轮腿入口，但尚未接入轮腿动力学和运动学模型。

> 注意：本工具适合用于早期方案比较、参数敏感性分析、设计讨论和教学展示。当前结果不应直接作为最终电机选型、实车安全裕度认证或比赛级性能承诺的唯一依据。

---

## 2. 文件结构

推荐项目目录如下：

```text
Wheel_Maker/
├── Config_Params.m
├── main.m
├── Swerve_Kinematics_Solver.m
├── Swerve_Dynamics_Core.m
├── Swerve_UI_Dashboard.m
└── Swerve_Debug_SmokeTest.m
```

各文件功能如下：

| 文件 | 作用 |
|---|---|
| `Config_Params.m` | 全局参数字典，保存机器人几何参数、电机参数、工程常数和 Wheel-Leg 预留参数 |
| `main.m` | 命令行主程序，用于输出理论最大速度、最大自转速度、扭矩需求和打滑判断 |
| `Swerve_Kinematics_Solver.m` | 四轮舵轮运动学解算器 |
| `Swerve_Dynamics_Core.m` | 四轮舵轮简化动力学与扭矩估算核心 |
| `Swerve_UI_Dashboard.m` | 图形化交互式 Dashboard |
| `Swerve_Debug_SmokeTest.m` | 最小冒烟测试脚本 |

---

## 3. 文件命名要求

MATLAB 对函数文件名非常敏感。请确保文件名和主函数名完全一致。

正确文件名应该是：

```text
Config_Params.m
main.m
Swerve_Kinematics_Solver.m
Swerve_Dynamics_Core.m
Swerve_UI_Dashboard.m
Swerve_Debug_SmokeTest.m
```

不建议保留或运行下面这类文件名：

```text
Swerve_UI_Dashboard(1).m
Swerve_UI_Dashboard(2).m
Swerve_Dynamics_Core(1).m
Swerve_Kinematics_Solver(1).m
Config_Params(2).m
main(1).m
```

如果从浏览器或聊天工具下载文件后出现了 `(1)`、`(2)` 后缀，请手动重命名。

---

## 4. 坐标系与轮序约定

当前项目统一采用如下坐标约定：

```text
X 轴：向前为正
Y 轴：向右为正
Z 轴：向上为正
omega_z：俯视逆时针为正
```

轮序统一为：

```text
[FL, FR, RR, RL]
```

含义如下：

| 编号 | 缩写 | 中文 |
|---|---|---|
| 1 | FL | 左前轮 |
| 2 | FR | 右前轮 |
| 3 | RR | 右后轮 |
| 4 | RL | 左后轮 |

所有四维输出数组，例如轮速、舵角、扭矩、法向载荷、打滑标志，均默认遵循该轮序。

---

## 5. 快速运行

打开 MATLAB，将当前工作目录切换到项目目录，然后运行：

```matlab
clear classes; clear functions; clc;
Swerve_Debug_SmokeTest
```

如果 SmokeTest 正常通过，再运行 Dashboard：

```matlab
clear classes; clear functions; clc;
Swerve_UI_Dashboard
```

也可以单独运行命令行报告：

```matlab
clear classes; clear functions; clc;
main
```

---

## 6. 推荐运行顺序

第一次使用时建议按以下顺序：

```matlab
clear classes; clear functions; clc;
Swerve_Debug_SmokeTest
```

确认无报错后运行：

```matlab
Swerve_UI_Dashboard
```

如果修改过 `Config_Params.m` 或类定义，建议重新执行：

```matlab
clear classes; clear functions; clc;
```

这是因为 MATLAB 会缓存 `classdef` 类定义。修改 `Config_Params.m` 后如果不清除缓存，MATLAB 可能会继续使用旧参数。

---

## 7. Config_Params 参数说明

`Config_Params.m` 是项目的全局参数字典。

### 7.1 公共常数

| 参数 | 默认值 | 单位 | 说明 |
|---|---:|---|---|
| `g` | 9.81 | m/s² | 重力加速度 |

### 7.2 Swerve 几何与质量参数

| 参数 | 默认值 | 单位 | 说明 |
|---|---:|---|---|
| `swerve_m_total` | 25.0 | kg | 整车质量 |
| `swerve_wheel_base_x` | 0.2700 | m | 前后轴距 |
| `swerve_wheel_base_y` | 0.2700 | m | 左右轮距 |
| `swerve_wheel_radius` | 0.0425 | m | 轮子半径 |
| `swerve_wheel_width` | 0.030 | m | 轮子宽度 |
| `swerve_h_cog` | 0.2 | m | 重心高度 |
| `swerve_I_steer` | 0.015 | kg·m² | 转向机构转动惯量 |

### 7.3 电机与传动参数

| 参数 | 默认值 | 单位 | 说明 |
|---|---:|---|---|
| `swerve_i_drive` | 1.0 | - | 驱动减速比 |
| `swerve_i_steer` | 1.0 | - | 转向减速比 |
| `swerve_motor_max_rpm` | 450 | rpm | 驱动电机最高转速 |
| `swerve_steer_max_rpm` | 120 | rpm | 转向电机最高转速 |

### 7.4 运动性能目标

| 参数 | 默认值 | 单位 | 说明 |
|---|---:|---|---|
| `swerve_target_max_a` | 3.0 | m/s² | 目标最大线加速度 |
| `swerve_target_max_alpha` | 20.0 | rad/s² | 目标最大转向角加速度 |

### 7.5 工程常数与环境参数

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

### 7.6 Wheel-Leg 预留参数

| 参数 | 默认值 | 单位 | 说明 |
|---|---:|---|---|
| `wl_m_total` | 20.0 | kg | 轮腿机器人质量，当前仅预留 |

---

## 8. main.m 使用说明

`main.m` 是命令行版本的性能评估脚本。

运行：

```matlab
main
```

它会输出：

1. 理论最大直线速度。
2. 理论最大自转速度。
3. 巡航工况下的驱动电机持续扭矩。
4. 巡航工况下的转向电机持续扭矩。
5. 45° 极限爆发加速工况下的峰值驱动扭矩。
6. 45° 极限爆发加速工况下的峰值转向扭矩。
7. 是否检测到打滑风险。

典型输出格式类似：

```text
========== 舵轮底盘轮组设计选型与性能评估 ==========

【1. 运动表现评估，基于物理边界】
  >> 理论最大直线速度 : 2.00 m/s
  >> 理论最大自转速度 : 10.49 rad/s，约 100.2 rpm

【2. 电机轴端扭矩需求，已折算减速比与 1.2x 冗余】
  >> 驱动电机 - 持续扭矩 : ...
  >> 驱动电机 - 峰值扭矩 : ...
  >> 转向电机 - 持续扭矩 : ...
  >> 转向电机 - 峰值扭矩 : ...

【3. 抓地力安全评估】
  [安全] 抓地力利用率正常，物理性能未达临界点。
```

---

## 9. Dashboard 使用说明

运行：

```matlab
Swerve_UI_Dashboard
```

Dashboard 分为左侧输入区和右侧结果区。

---

### 9.1 左侧：机器人类型与本体参数

左侧第一栏为：

```text
1. 机器人类型与本体参数
```

包含：

- 机器人类型下拉框。
- 机器人几何和质量参数表。
- 当前测试工况输入框。
- 重置舵角状态按钮。
- 重新计算按钮。

当前机器人类型包括：

| 类型 | 说明 |
|---|---|
| `Swerve 四轮舵轮 - RM2026 默认` | 当前默认四轮舵轮底盘 |
| `Swerve 四轮舵轮 - 轻量小车预设` | 较轻质量、较小轴距、较高转速预设 |
| `Swerve 四轮舵轮 - 重载底盘预设` | 较大质量、较大轮径、较大轴距预设 |
| `Wheel-Leg 双轮轮腿 - 预留` | 轮腿模型预留入口，当前不执行计算 |

---

### 9.2 当前测试工况

当前测试工况包含：

| 输入 | 单位 | 说明 |
|---|---|---|
| `vx` | m/s | 底盘前向速度 |
| `vy` | m/s | 底盘右向速度 |
| `omega` | rad/s | 底盘角速度，逆时针为正 |
| `dt` | s | 控制周期 |
| `ax` | m/s² | 前向加速度 |
| `ay` | m/s² | 右向加速度 |

其中：

- `vx / vy / omega` 主要用于运动学解算。
- `ax / ay` 主要用于动力学估算。
- `dt` 用于舵角速度和转向角加速度估算。

---

### 9.3 左侧：工程常数 / 环境 / 电机传动

左侧第二栏为：

```text
2. 工程常数 / 环境 / 电机传动
```

这里可以修改：

- 驱动减速比。
- 转向减速比。
- 驱动电机最高转速。
- 转向电机最高转速。
- 摩擦系数。
- 滚动阻力系数。
- 地胶阻力放大系数。
- 滑移效率。
- 轮胎邵氏硬度。
- 机械损耗扭矩。
- 安全冗余系数。

修改表格中的数值后，Dashboard 会自动重新计算。

---

## 10. Dashboard 右侧结果说明

右侧分为三部分：

1. 结果报文。
2. 图表与轮胎形变示意。
3. 逐轮结果表。

---

### 10.1 结果报文

结果报文以指标卡形式展示，当前包含 12 个指标：

| 指标 | 说明 |
|---|---|
| 机器人类型 | 当前选择的机器人预设 |
| 速度指令 | 当前 `vx / vy / omega` |
| 加速度工况 | 当前 `ax / ay` |
| 最大直线速度 | 根据驱动电机最高转速、驱动减速比、轮径估算 |
| 最大自转速度 | 根据最大轮速和底盘几何尺寸估算 |
| 最大转向角速度 | 根据转向电机最高转速和转向减速比估算 |
| 峰值驱动扭矩 | 四轮中最大的驱动电机轴端扭矩 |
| 峰值转向扭矩 | 四轮中最大的转向电机轴端扭矩 |
| 摩擦系数 / 弹性模量 | 根据轮胎硬度修正后的等效摩擦系数和弹性模量 |
| 最大轮胎下压 | 四轮中估算下压形变最大的轮子 |
| 最大抓地利用率 | 四轮中最大 `F / μFz` |
| 打滑轮 | 被判定为打滑的轮子 |

---

### 10.2 图表与轮胎形变示意

图表区包含四个图：

| 图表 | 说明 |
|---|---|
| 驱动 / 转向扭矩 | 每个轮子的驱动扭矩和转向扭矩 |
| 抓地利用率 | 每个轮子的 `F / μFz`，超过 1 表示打滑风险 |
| 四轮法向载荷 | 每个轮子的 `Fz` |
| 轮胎下压形变示意 | 显示最大载荷轮的轮胎下压形变和接触区域 |

轮胎形变图是可视化示意图，并非高精度轮胎有限元模型。图中形变经过放大以便观察。

---

### 10.3 逐轮结果表

结果表按 `[FL, FR, RR, RL]` 顺序显示：

| 列名 | 说明 |
|---|---|
| `Wheel` | 轮子编号 |
| `DriveSpeed_mps` | 轮子驱动线速度，m/s |
| `SteerAngle_deg` | 目标舵角，deg |
| `SteerRate_radps` | 转向角速度，rad/s |
| `DriveTorque_Nm` | 驱动电机轴端扭矩，N·m |
| `SteerTorque_Nm` | 转向电机轴端扭矩，N·m |
| `Fz_N` | 法向载荷，N |
| `TireCompression_mm` | 轮胎下压形变，mm |
| `ContactHalfLen_mm` | 接触半长，mm |
| `GripUsage` | 抓地利用率 |
| `SlipState` | 打滑状态，`OK` 或 `SLIP` |

---

## 11. 运动学解算器说明

函数：

```matlab
[v_drive_out, theta_steer_out, omega_steer_out, debug] = ...
    Swerve_Kinematics_Solver(vx, vy, omega_z, current_theta, dt, p)
```

### 11.1 输入

| 参数 | 单位 | 说明 |
|---|---|---|
| `vx` | m/s | 底盘前向速度 |
| `vy` | m/s | 底盘右向速度 |
| `omega_z` | rad/s | 底盘角速度，逆时针为正 |
| `current_theta` | rad | 当前四个舵轮角度，1x4 |
| `dt` | s | 控制周期 |
| `p` | struct | 参数结构体 |

### 11.2 输出

| 输出 | 单位 | 说明 |
|---|---|---|
| `v_drive_out` | m/s | 四个轮子的驱动线速度 |
| `theta_steer_out` | rad | 四个轮子的目标舵角 |
| `omega_steer_out` | rad/s | 四个轮子的转向角速度 |
| `debug` | struct | 调试信息 |

### 11.3 当前支持的运动学特性

当前运动学解算器支持：

- 四轮舵轮速度分解。
- 目标舵角计算。
- 舵轮最短路径优化。
- 超过 90° 时反转轮速，减少舵角旋转。
- 转向角速度饱和。
- 驱动速度饱和。
- 输出最大驱动速度和最大转向角速度。

---

## 12. 动力学核心说明

函数：

```matlab
[Drive_Torque, Steer_Torque, Slip_Warning, debug] = ...
    Swerve_Dynamics_Core(ax, ay, alpha_steer, p)
```

### 12.1 输入

| 参数 | 单位 | 说明 |
|---|---|---|
| `ax` | m/s² | 机器人纵向加速度，向前为正 |
| `ay` | m/s² | 机器人横向加速度，向右为正 |
| `alpha_steer` | rad/s² | 四个轮子的转向角加速度，1x4 |
| `p` | struct | 参数结构体 |

### 12.2 输出

| 输出 | 单位 | 说明 |
|---|---|---|
| `Drive_Torque` | N·m | 四个轮子的驱动电机轴端扭矩 |
| `Steer_Torque` | N·m | 四个轮子的转向电机轴端扭矩 |
| `Slip_Warning` | - | 打滑标志，1 表示打滑，0 表示正常 |
| `debug` | struct | 调试信息 |

### 12.3 debug 信息

当前 `debug` 通常包含：

| 字段 | 说明 |
|---|---|
| `mu` | 等效摩擦系数 |
| `E` | 等效弹性模量 |
| `Fz` | 四轮法向载荷 |
| `Fx_i` | 分配到各轮的纵向力 |
| `Fy_i` | 分配到各轮的横向力 |
| `F_total_i` | 各轮合力 |
| `muFz` | 各轮最大摩擦力 |
| `grip_usage` | 抓地利用率 |

Dashboard 内部还会补全：

| 字段 | 说明 |
|---|---|
| `delta` | 轮胎下压形变 |
| `contact_half_len` | 接触半长 |

---

## 13. SmokeTest 使用说明

运行：

```matlab
Swerve_Debug_SmokeTest
```

SmokeTest 会依次执行：

1. 打印 `Config_Params` 参数。
2. 测试 `Swerve_Kinematics_Solver`。
3. 测试 `Swerve_Dynamics_Core`。
4. 运行 `main.m`。

如果运行成功，最后会显示：

```text
========== Smoke Test Passed ==========
```

SmokeTest 的作用是确认当前项目文件可以正常调用，主要用于排查函数名、文件名、参数接口和基础运行错误。

---

## 14. 常见使用场景

### 14.1 估算最大直线速度

调整以下参数：

- `swerve_motor_max_rpm`
- `swerve_i_drive`
- `swerve_wheel_radius`

查看右侧：

```text
最大直线速度
```

或者运行：

```matlab
main
```

---

### 14.2 估算最大自转速度

调整以下参数：

- `swerve_motor_max_rpm`
- `swerve_i_drive`
- `swerve_wheel_radius`
- `swerve_wheel_base_x`
- `swerve_wheel_base_y`

查看右侧：

```text
最大自转速度
```

---

### 14.3 比较轻量底盘和重载底盘

在 Dashboard 左上角机器人类型中选择：

```text
Swerve 四轮舵轮 - 轻量小车预设
```

或：

```text
Swerve 四轮舵轮 - 重载底盘预设
```

对比：

- 最大直线速度。
- 最大自转速度。
- 峰值驱动扭矩。
- 峰值转向扭矩。
- 法向载荷分布。
- 抓地利用率。

---

### 14.4 查看打滑风险

提高：

- `ax`
- `ay`

或者降低：

- `swerve_mu_ground`

观察：

- `最大抓地利用率`
- `打滑轮`
- `抓地利用率图`
- 结果表中的 `SlipState`

当 `GripUsage > 1` 时，代表当前简化模型下对应轮子有打滑风险。

---

### 14.5 观察轮胎下压形变

调整：

- `swerve_m_total`
- `swerve_h_cog`
- `swerve_wheel_width`
- `swerve_hardness_shoreA`
- `ax`
- `ay`

观察：

- `最大轮胎下压`
- `轮胎下压形变示意`
- 结果表中的 `TireCompression_mm`
- 结果表中的 `ContactHalfLen_mm`

---

## 15. 结果解释注意事项

### 15.1 最大直线速度

当前最大直线速度计算基于：

```text
驱动电机最高转速 / 驱动减速比 × 轮子周向速度
```

它没有考虑：

- 电机转矩-转速曲线。
- 电池电压限制。
- 电流限制。
- 地面阻力。
- 控制器限速。
- 轮胎打滑。

因此它是理论速度上限，不等于实车一定能跑到的速度。

---

### 15.2 峰值驱动扭矩

当前驱动扭矩包含：

- 滚动阻力。
- 机械损耗。
- 加速度对应的轮端合力。
- 滑移效率。
- 安全冗余系数。

但它没有完整建模：

- 电机转矩-转速曲线。
- 减速箱效率。
- 驱动轮转动惯量。
- 电机热衰减。
- 电流限制。
- 电池压降。
- 真实轮胎滑移率。

因此建议把结果视作初步量级估算。

---

### 15.3 峰值转向扭矩

当前转向扭矩包含：

- 接触斑摩擦阻力矩。
- 转向机械损耗。
- 转向机构惯量与角加速度。
- 安全冗余系数。

但真实转向扭矩还可能受到：

- 主销偏距。
- 舵轮模块轴承摩擦。
- 电缆拖拽。
- 轮胎材料非线性。
- 地面材质。
- 原地干磨状态。
- 舵角控制器动态响应。

因此最终转向电机选型必须通过实测校准。

---

### 15.4 抓地利用率

抓地利用率定义为：

```text
GripUsage = F_total_i / (mu * Fz_i)
```

解释：

| 数值 | 含义 |
|---:|---|
| `< 0.5` | 抓地裕度较大 |
| `0.5 ~ 0.8` | 抓地利用明显 |
| `0.8 ~ 1.0` | 接近摩擦极限 |
| `> 1.0` | 简化模型下判定为打滑 |

---

### 15.5 轮胎下压形变

轮胎下压形变来自简化的材料和接触模型，主要用于趋势分析和可视化展示。

不要将该结果理解为高精度轮胎有限元分析结果。

---

## 16. 模型假设与限制

当前模型主要是准静态估算模型，不是完整车辆动力学仿真器。

主要假设包括：

1. 机器人为刚体。
2. 四轮按法向载荷比例分配纵向力和横向力。
3. 地面水平。
4. 不考虑悬架动态。
5. 不考虑轮胎侧偏角。
6. 不考虑轮胎滑移率动态。
7. 不考虑电机转矩-转速曲线。
8. 不考虑电池电压和电流限制。
9. 不考虑驱动轮转动惯量。
10. 不考虑底盘绕 z 轴角加速度对应的完整力矩平衡。
11. 不考虑实际控制器闭环响应。
12. Wheel-Leg 模型尚未接入。

因此当前工具应定位为：

```text
早期工程估算工具 / 方案对比工具 / 参数敏感性分析工具 / 教学展示工具
```

不应定位为：

```text
最终电机选型认证工具 / 高保真车辆动力学仿真器 / 实车安全裕度证明工具
```

---

## 17. 已知重要问题

### 17.1 载荷转移符号需要进一步物理校核

当前代码约定中，`ax > 0` 时前轮法向载荷增加，`ay > 0` 时右轮法向载荷增加。

如果 `ax` 表示机器人向前加速，按照常规车辆动力学直觉，向前加速时载荷通常应向后轮转移。因此，后续需要用自由体图重新确认：

- `ax / ay` 是车体加速度还是等效惯性加速度。
- 载荷转移符号是否符合当前定义。
- Dashboard 中法向载荷和最大下压轮的显示是否符合预期。
- 打滑轮判断是否受符号影响。

这是当前版本最应该优先复核的问题。

---

### 17.2 Dashboard 中存在部分重复计算

Dashboard 中为了兼容不同版本的动力学核心，会在本地补全部分 debug 信息，例如：

- 轮胎下压形变。
- 接触半长。
- 抓地利用率。

后续建议将这些计算全部移动到 `Swerve_Dynamics_Core.m` 中，由动力学核心统一输出，Dashboard 只负责展示。

---

### 17.3 参数无法保存

当前 Dashboard 修改参数后，只在当前 MATLAB 会话中有效。关闭 Dashboard 后不会自动保存。

后续建议增加：

- 保存为 `.mat`
- 保存为 `.json`
- 从配置文件加载
- 导出当前报告

---

## 18. 常见问题排查

### 18.1 报错：找不到函数或变量

检查文件名是否正确。

错误示例：

```text
Swerve_UI_Dashboard(2).m
```

应该改为：

```text
Swerve_UI_Dashboard.m
```

---

### 18.2 修改 Config_Params 后没有生效

运行：

```matlab
clear classes; clear functions; clc;
```

然后重新启动 Dashboard：

```matlab
Swerve_UI_Dashboard
```

---

### 18.3 Dashboard 无法显示数值

确认 `makeMetricCard` 和 `setMetricCard` 是否使用当前版本。

当前推荐的指标卡逻辑是：

- `makeMetricCard` 创建单个 `uilabel`。
- `setMetricCard` 用标题和数值共同更新文本。
- 不再使用 `uipanel + 两个 uilabel` 的旧版指标卡结构。

---

### 18.4 Dashboard 运行但结果不更新

尝试点击：

```text
重新计算
```

如果仍然不更新，运行：

```matlab
clear classes; clear functions; clc;
Swerve_UI_Dashboard
```

---

### 18.5 Wheel-Leg 选择后没有计算结果

这是正常现象。

当前 Wheel-Leg 只是预留入口，尚未实现：

- `WheelLeg_Kinematics_Solver.m`
- `WheelLeg_Dynamics_Core.m`
- 轮腿参数映射
- 轮腿结果图表

---

## 21. 最小使用流程总结

第一次使用：

```matlab
clear classes; clear functions; clc;
Swerve_Debug_SmokeTest
```

打开 Dashboard：

```matlab
Swerve_UI_Dashboard
```

修改参数：

1. 选择机器人类型。
2. 修改机器人本体参数。
3. 修改工程常数。
4. 修改当前测试工况。
5. 查看右侧结果报文、图表和逐轮结果表。

命令行快速评估：

```matlab
main
```

清理缓存并重启：

```matlab
clear classes; clear functions; clc;
Swerve_UI_Dashboard
```
