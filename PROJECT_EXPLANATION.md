# Wheel_Maker 工程详细说明

## 1. 工程概述

`Wheel_Maker` 是一个基于 MATLAB 的四轮舵轮（Swerve Drive）底盘性能评估工程。该工程围绕 RM2026 舵轮底盘轮组设计选型展开，用于在给定机器人质量、几何尺寸、电机参数、轮胎参数和环境阻力条件下，完成以下分析：

- 四轮舵轮底盘理论运动学性能边界计算。
- 底盘速度指令到四个舵轮速度与舵角的逆运动学分配。
- 基于加速度指令的载荷转移、轮胎形变、抓地利用率和打滑风险评估。
- 驱动电机与转向电机所需扭矩估算。
- 命令行报告输出、交互式 MATLAB UI 仪表盘展示以及基础测试验证。

工程整体属于“参数化仿真 / 工程选型核算”工具，不是嵌入式实车控制代码。它更适合用于机器人设计早期的轮组、电机、减速比、轮胎硬度和车体尺寸选型评估。

---

## 2. 工程文件结构

当前文件夹中包含以下 MATLAB 文件：

| 文件 | 类型 | 作用 |
| --- | --- | --- |
| `main.m` | 命令行主程序 | 无 UI 的完整评估入口，自动运行默认工况并打印报告。 |
| `Config_Params.m` | 参数配置类 | 集中保存全局物理常数、舵轮底盘参数、电机传动参数和环境参数。 |
| `Swerve_Get_Preset.m` | 预设生成器 | 根据预设名称生成不同机器人配置，如默认、轻量、重载、Wheel-Leg 预留。 |
| `Swerve_Performance_Limits.m` | 理论性能边界计算 | 根据轮径、电机最高转速、减速比和底盘尺寸计算最大直线速度、自转速度和转向速度。 |
| `Swerve_Kinematics_Solver.m` | 舵轮运动学逆解 | 将底盘速度指令分配为四个舵轮的轮端线速度、目标舵角和转向角速度。 |
| `Swerve_Dynamics_Core.m` | 动力学核心 | 计算载荷转移、轮胎形变、驱动扭矩、转向扭矩、摩擦圆打滑风险。 |
| `Swerve_Evaluate_Case.m` | 单工况综合评估器 | 串联运动学、动力学和性能边界计算，输出完整评估结构体。 |
| `Swerve_UI_Dashboard.m` | 图形化仪表盘 | MATLAB App 风格交互 UI，可编辑参数并实时显示报告、图表和逐轮表格。 |
| `Swerve_PhysicsConsistency_Test.m` | 物理一致性测试 | 验证载荷转移方向、静态载荷均摊和纯自转运动学一致性。 |
| `Swerve_Debug_SmokeTest.m` | 冒烟测试 | 串联运行物理测试、参数装载、运动学、动力学和 `main.m`，用于开发检查。 |

---

## 3. 总体架构

工程可以分为五层：

```text
配置层
  Config_Params.m
  Swerve_Get_Preset.m

基础算法层
  Swerve_Performance_Limits.m
  Swerve_Kinematics_Solver.m
  Swerve_Dynamics_Core.m

集成评估层
  Swerve_Evaluate_Case.m

展示入口层
  main.m
  Swerve_UI_Dashboard.m

测试验证层
  Swerve_PhysicsConsistency_Test.m
  Swerve_Debug_SmokeTest.m
```

典型数据流如下：

```text
预设名称
  ↓
Swerve_Get_Preset
  ↓ 生成参数结构体 p
工况 caseIn + 参数 p
  ↓
Swerve_Evaluate_Case
  ├─ Swerve_Performance_Limits
  ├─ Swerve_Kinematics_Solver
  └─ Swerve_Dynamics_Core
  ↓
out 综合结果
  ↓
main.m 控制台报告 或 Swerve_UI_Dashboard 图形化展示
```

---

## 4. 坐标系与轮序约定

工程内部使用统一坐标约定：

- X 轴：车体前方为正。
- Y 轴：车体右方为正。
- Z 轴：向上为正。
- `omega_z`：俯视逆时针为正。
- `ax`：机器人本体系下真实前向加速度，`ax > 0` 表示向前加速。
- `ay`：机器人本体系下真实右向加速度，`ay > 0` 表示向右加速。

四个舵轮轮序固定为：

| 索引 | 缩写 | 含义 | 位置坐标 |
| --- | --- | --- | --- |
| 1 | FL | Front Left，左前轮 | \(+L_x, -L_y\) |
| 2 | FR | Front Right，右前轮 | \(+L_x, +L_y\) |
| 3 | RR | Rear Right，右后轮 | \(-L_x, +L_y\) |
| 4 | RL | Rear Left，左后轮 | \(-L_x, -L_y\) |

其中 \(L_x\) 和 \(L_y\) 是半轴距和半轮距。

---

## 5. 核心参数说明

参数统一由 `Config_Params.m` 管理，并通过 `Config_Params.toStruct()` 转换为普通结构体供其他函数使用。

### 5.1 机器人本体参数

| 参数 | 默认值 | 单位 | 含义 |
| --- | ---: | --- | --- |
| `swerve_m_total` | 25.0 | kg | 全车总质量。 |
| `swerve_wheel_base_x` | 270.0 | mm | 前后轮中心距。 |
| `swerve_wheel_base_y` | 270.0 | mm | 左右轮中心距。 |
| `swerve_wheel_radius` | 42.5 | mm | 轮子外半径，即含包胶后的实际滚动半径。 |
| `swerve_rubber_thickness` | 10.0 | mm | 轮毂包胶厚度，用于区分刚性轮毂和弹性包胶层。 |
| `swerve_wheel_width` | 30.0 | mm | 轮子接地宽度。 |
| `swerve_h_cog` | 200.0 | mm | 整车质心高度。 |
| `swerve_I_steer` | 0.015 | kg·m² | 单个转向机构绕 Z 轴转动惯量。 |

### 5.2 电机与传动参数

| 参数 | 默认值 | 单位 | 含义 |
| --- | ---: | --- | --- |
| `swerve_i_drive` | 1.0 | - | 驱动轴减速比，定义为电机端转速 / 轮端转速。 |
| `swerve_i_steer` | 1.0 | - | 转向轴减速比，定义为电机端转速 / 舵角转速。 |
| `swerve_motor_max_rpm` | 450 | rpm | 驱动电机额定最高转速。 |
| `swerve_steer_max_rpm` | 120 | rpm | 转向电机额定最高转速。 |

### 5.3 性能目标参数

| 参数 | 默认值 | 单位 | 含义 |
| --- | ---: | --- | --- |
| `swerve_target_max_a` | 3.0 | m/s² | 期望最大平移加速度。 |
| `swerve_target_max_alpha` | 20.0 | rad/s² | 期望最大转向角加速度。 |

### 5.4 环境与摩擦参数

| 参数 | 默认值 | 单位 | 含义 |
| --- | ---: | --- | --- |
| `swerve_mu_ground` | 0.8 | - | 基准地面摩擦系数。 |
| `swerve_carpet_factor` | 4.0 | - | 地胶 / 地毯阻力放大系数。 |
| `swerve_roll_resistance` | 0.018 | - | 基础滚动阻力系数。 |
| `swerve_eta_slip` | 0.9 | - | 轮胎传动滑移效率。 |
| `swerve_hardness_shoreA` | 60 | Shore A | 轮胎包胶邵氏硬度。 |
| `swerve_T_mech_drive` | 0.1 | N·m | 驱动轴端机械摩擦静态损耗扭矩。 |
| `swerve_T_mech_steer` | 0.1 | N·m | 转向轴端机械摩擦静态损耗扭矩。 |
| `swerve_redundancy` | 1.2 | - | 选型安全冗余系数。 |

---

## 6. 预设系统

`Swerve_Get_Preset.m` 提供不同机器人方案的参数覆盖逻辑。

### 6.1 默认预设

默认预设名称为：

```text
Swerve 四轮舵轮 - 默认
```

如果不传入预设名称，函数会自动使用默认参数，即 `Config_Params.m` 中的全局常量。

### 6.2 轻量小车预设

当预设名称中包含 `轻量` 时，系统会切换为轻量化高机动配置。特点包括：

- 质量降低到 15 kg。
- 轴距和轮距减小到 230 mm。
- 轮径减小。
- 包胶厚度设置为 8 mm。
- 驱动和转向最高转速提高。
- 目标最大加速度提高到 3.5 m/s²。

该配置适合模拟轻量化步兵或高机动小车。

### 6.3 重载底盘预设

当预设名称中包含 `重载` 时，系统会切换为重装高稳定配置。特点包括：

- 质量增加到 35 kg。
- 轴距和轮距增加到 320 mm。
- 轮径与轮宽增大。
- 包胶厚度设置为 12 mm。
- 质心高度提高。
- 目标最大加速度降低到 2.5 m/s²。

该配置适合模拟英雄、工程或较重载荷底盘。

### 6.4 Wheel-Leg 预留

当预设名称中包含 `Wheel-Leg` 时，仅保留轮腿质量字段 `wl_m_total`。当前工程尚未实现 Wheel-Leg 运动学和动力学模型，UI 中也会显示“模型尚未接入”的占位提示。

---

## 7. 运动学模块说明

运动学核心函数为 `Swerve_Kinematics_Solver.m`。

### 7.1 输入

```text
vx, vy, omega_z, current_theta, dt, p
```

含义如下：

- `vx`：期望前向速度，单位 m/s。
- `vy`：期望右向速度，单位 m/s。
- `omega_z`：期望自转角速度，单位 rad/s。
- `current_theta`：当前四个舵轮实际舵角，1x4 数组，单位 rad。
- `dt`：控制周期，单位 s。
- `p`：参数结构体。

### 7.2 舵轮速度分配

每个轮子的速度由底盘平移速度和绕质心旋转速度叠加得到：

```text
v_ix = vx + omega_z * y_i
v_iy = vy - omega_z * x_i
```

随后计算：

```text
v_target = hypot(v_ix, v_iy)
theta_target = atan2(v_iy, v_ix)
```

### 7.3 防绕线 Flip 优化

如果目标舵角与当前舵角差值超过 90°，函数会执行 Flip：

- 目标舵角反向转动 180° 的等效方向。
- 驱动速度取反。

这样可以让舵轮少转角度，降低转向机构负担，并减少绕线风险。

### 7.4 转向物理限幅

函数根据转向电机最高转速和减速比计算最大舵角角速度：

```text
max_steer_w = steer_max_rpm / i_steer * 2π / 60
```

如果一个控制周期内所需舵角速度超过上限，则舵角只能前进到物理可达位置。同时，驱动速度会按舵角误差的余弦进行投影降额，以减小底盘实际运动方向偏差。

### 7.5 驱动速度限幅

函数根据驱动电机最高转速、减速比和轮半径计算轮端最大线速度，并对 `v_target` 进行硬限幅。

### 7.6 输出

```text
v_drive_out, theta_steer_out, omega_steer_out, debug
```

其中：

- `v_drive_out`：四轮轮端线速度。
- `theta_steer_out`：四轮目标舵角。
- `omega_steer_out`：四轮转向角速度。
- `debug`：包含轮子坐标、原始舵角、原始速度、是否 Flip、速度上限等调试信息。

---

## 8. 动力学模块说明

动力学核心函数为 `Swerve_Dynamics_Core.m`。

### 8.1 输入

```text
ax, ay, alpha_steer, p
```

含义如下：

- `ax`：前向真实加速度。
- `ay`：右向真实加速度。
- `alpha_steer`：四个舵轮的转向角加速度，1x4 数组。
- `p`：参数结构体。

### 8.2 输入安全校验

函数会检查：

- `ax`、`ay` 必须是有限标量。
- `alpha_steer` 必须包含 4 个有限元素。
- 质量、重力、轴距、轮距、轮半径、轮宽、减速比、滑移效率、安全冗余等关键参数必须为正。
- 质心高度不能为负。
- 邵氏硬度必须位于 0 到 100 之间。

### 8.3 轮胎材料模型

函数使用 Gent 经验公式将邵氏硬度 Shore A 转换为杨氏模量 \(E\)：

```text
E = ((0.0981 * (56 + 7.66 * s)) / (0.149 * (100 - s))) * 1e6
```

随后基于赫兹接触思想，以默认硬度为参考，对摩擦系数进行非线性修正：

```text
mu = mu_ground * (E_ref / E)^(2/3)
```

并将 `mu` 限制在 0.1 到 1.5 之间，避免极端参数导致结果爆炸。

### 8.4 载荷转移模型

静态每轮法向载荷：

```text
Fz_static = m * g / 4
```

纵向加速度引起的单轮载荷转移量：

```text
dFz_x = m * ax * h / (2 * wheel_base_x)
```

横向加速度引起的单轮载荷转移量：

```text
dFz_y = m * ay * h / (2 * wheel_base_y)
```

四轮载荷分配为：

| 轮位 | 载荷表达式 |
| --- | --- |
| FL | `Fz_static - dFz_x + dFz_y` |
| FR | `Fz_static - dFz_x - dFz_y` |
| RR | `Fz_static + dFz_x - dFz_y` |
| RL | `Fz_static + dFz_x + dFz_y` |

物理解释：

- 向前加速时，重心后移，后轮增载，前轮减载。
- 刹车或向后加速时，前轮增载，后轮减载。
- 向右加速时，左轮增载，右轮减载。
- 向左加速时，右轮增载，左轮减载。

如果某轮计算得到负法向载荷，则截断为 0，用于模拟极限工况下车轮离地。

### 8.5 轮毂包胶厚度、轮胎形变与接触长度

工程现在显式区分轮子外半径、刚性轮毂半径和包胶厚度：

```text
R_outer = swerve_wheel_radius
rubber_thickness = swerve_rubber_thickness
R_hub = R_outer - rubber_thickness
```

其中 `swerve_wheel_radius` 仍表示含包胶后的实际滚动外半径，因此运动学速度边界和驱动扭矩力臂仍使用外半径。`swerve_rubber_thickness` 表示外层弹性包胶可被压缩的厚度。

轮胎下压形变近似计算：

```text
delta_raw = Fz / (E * wheel_width)
delta = min(delta_raw, rubber_thickness)
```

这意味着包胶层压缩量不会超过真实包胶厚度，避免旧模型在极端载荷下出现“压缩深度大于包胶层”的非物理结果。

接触半长：

```text
contact_half_len = sqrt(max(0, 2 * R_outer * delta - delta^2))
```

这部分用于估计轮胎接地变形，并进一步影响转向摩擦阻力矩。UI 轮胎形变示意图中也会同时画出外轮廓和刚性轮毂参考圆。

### 8.6 驱动扭矩估算

每个轮子的驱动扭矩由三部分组成：

1. 滚动阻力矩。
2. 机械静态损耗扭矩。
3. 加速所需牵引力经轮半径换算得到的扭矩。

最终再除以驱动减速比并乘以安全冗余系数。

### 8.7 转向扭矩估算

每个轮子的转向扭矩由三部分组成：

1. 原地转向引起的接触面摩擦阻力矩。
2. 转向轴机械静态损耗扭矩。
3. 转向机构角加速度引起的惯性扭矩。

最终再除以转向减速比并乘以安全冗余系数。

### 8.8 抓地利用率与打滑判断

每个轮子的摩擦极限：

```text
muFz = mu * Fz
```

抓地利用率：

```text
grip_usage = F_total_i / muFz
```

当 `F_total_i > muFz` 时，该轮被标记为打滑。

---

## 9. 单工况综合评估器

`Swerve_Evaluate_Case.m` 是工程中最重要的集成函数。它将一个具体测试工况完整转换为工程评估结果。

### 9.1 输入工况结构体

`caseIn` 可包含以下字段：

| 字段 | 含义 | 默认值 |
| --- | --- | --- |
| `vx` | 期望前向速度 | 0.0 |
| `vy` | 期望右向速度 | 0.0 |
| `omega_z` | 期望自转角速度 | 0.0 |
| `ax` | 期望前向真实加速度 | 0.0 |
| `ay` | 期望右向真实加速度 | 0.0 |
| `dt` | 控制 / 评估周期 | 0.02 |
| `current_theta` | 当前四轮舵角 | `[0 0 0 0]` |
| `alpha_steer` | 可选，指定转向角加速度 | 若缺失则自动估计 |

### 9.2 内部处理流程

1. 合并默认参数。
2. 补全缺失工况字段。
3. 调用 `Swerve_Performance_Limits` 计算理论边界。
4. 调用 `Swerve_Kinematics_Solver` 计算四轮速度、舵角和舵角速度。
5. 计算或读取 `alpha_steer`。
6. 调用 `Swerve_Dynamics_Core` 计算扭矩、载荷、形变和打滑。
7. 封装完整输出结构体。

### 9.3 输出结果结构体

输出 `out` 包含：

- 原始工况 `out.case`。
- 参数 `out.p`。
- 理论边界 `out.limits`。
- 四轮运动学结果：`v_drive`、`theta_steer`、`omega_steer`、`alpha_steer`。
- 四轮动力学结果：`drive_torque`、`steer_torque`、`slip_flag`。
- 调试结构体：`kin_debug`、`dyn_debug`。
- 快速极值：最大驱动扭矩、最大转向扭矩、最大抓地利用率、最大轮胎下压、对应轮位索引。
- 轮位名称：`["FL", "FR", "RR", "RL"]`。

---

## 10. 命令行主程序 `main.m`

`main.m` 是无 UI 的自动化评估入口。运行后会执行以下流程：

1. 清空 MATLAB 命令行和图窗。
2. 检查核心依赖文件是否存在。
3. 载入默认预设 `Swerve 四轮舵轮 - 默认`。
4. 打印轮序、坐标系和加速度定义。
5. 计算理论最大直线速度、最大自转速度和最大转向角速度。
6. 构造并评估三个代表性工况：
   - 静态承载工况。
   - 高速巡航工况。
   - 45° 对角线极限爆发加速工况。
7. 打印以下报告：
   - 理论运动表现。
   - 静态载荷与基础阻力。
   - 巡航稳态扭矩和抓地利用率。
   - 极限爆发工况扭矩、抓地利用率和轮胎形变。
   - 是否打滑。
   - 极限工况逐轮明细表。

适合用于快速命令行核算和批处理选型验证。

运行方式：

```matlab
main
```

---

## 11. 图形化仪表盘 `Swerve_UI_Dashboard.m`

`Swerve_UI_Dashboard.m` 提供交互式 MATLAB UI，用于可视化调参和实时评估。

运行方式：

```matlab
Swerve_UI_Dashboard
```

### 11.1 左侧区域

左侧分为两部分：

1. 机器人类型与本体参数。
   - 可选择默认、轻量、重载和 Wheel-Leg 预留预设。
   - 可编辑质量、轴距、轮距、轮半径、轮宽、质心高度、转向惯量、目标加速度等。
   - 可输入当前测试工况，包括速度、角速度、控制周期和加速度。
   - 提供“重置舵角状态”和“重新计算”按钮。

2. 工程常数 / 环境 / 电机传动。
   - 可编辑减速比、电机最高转速、摩擦系数、滚阻系数、轮胎硬度、机械损耗和安全冗余等。

### 11.2 右侧区域

右侧分为三部分：

1. 结果报文卡片。
   - 显示机器人类型、速度指令、加速度工况。
   - 显示最大直线速度、自转速度、转向角速度。
   - 显示额定扭矩、峰值扭矩、摩擦系数、杨氏模量、最大下压、最大抓地利用率和打滑轮。

2. 图表与轮胎形变示意。
   - 扭矩柱状图：额定 / 当前驱动与转向扭矩对比。
   - 法向载荷柱状图：四轮 Fz 分布。
   - 轮胎下压形变示意图：显示最大载荷轮的轮胎形变和接触半长。

3. 逐轮结果表。
   - 轮位。
   - 驱动速度。
   - 舵角。
   - 舵速。
   - 舵角加速度。
   - 额定与当前驱动 / 转向扭矩。
   - 法向载荷。
   - 轮胎下压。
   - 接触半长。
   - 抓地利用率。
   - 打滑状态。

---

## 12. 理论性能边界计算

`Swerve_Performance_Limits.m` 计算三个关键理论上限。

### 12.1 最大直线速度

```text
max_drive_speed = motor_max_rpm / i_drive * 2π / 60 * wheel_radius
```

含义：驱动电机最高转速折算到轮端角速度，再乘以轮半径得到最大线速度。

### 12.2 最大原地自转角速度

```text
max_spin_rate = max_drive_speed / hypot(Lx, Ly)
```

含义：原地自转时，轮组中心到质心的半径越大，同样轮端速度对应的自转角速度越小。

### 12.3 最大转向角速度

```text
max_steer_rate = steer_max_rpm / i_steer * 2π / 60
```

含义：转向电机最高转速折算到舵角旋转速度。

---

## 13. 测试与验证

### 13.1 物理一致性测试

运行：

```matlab
Swerve_PhysicsConsistency_Test
```

测试内容包括：

- 静态工况下四轮法向载荷均分。
- 前向加速时后轮增载。
- 刹车时前轮增载。
- 向右加速时左轮增载。
- 向左加速时右轮增载。
- 纯自转时四轮原始速度幅值一致。

### 13.2 冒烟测试

运行：

```matlab
Swerve_Debug_SmokeTest
```

测试内容包括：

1. 调用物理一致性测试。
2. 检查参数字典是否能正确转换为结构体。
3. 检查运动学模块输出。
4. 检查动力学模块输出。
5. 执行 `main.m`，确认主报告链路无崩溃。

---

## 14. 典型使用方式

### 14.1 快速命令行评估

在 MATLAB 当前路径切换到工程目录后运行：

```matlab
main
```

适合快速得到默认配置下的整车性能总结。

### 14.2 打开交互式仪表盘

```matlab
Swerve_UI_Dashboard
```

适合手动调整参数并查看实时结果。

### 14.3 单独评估自定义工况

```matlab
p = Swerve_Get_Preset('Swerve 四轮舵轮 - 默认');
caseIn = struct();
caseIn.vx = 1.0;
caseIn.vy = 0.2;
caseIn.omega_z = 0.5;
caseIn.ax = 2.0;
caseIn.ay = 1.0;
caseIn.dt = 0.02;
caseIn.current_theta = zeros(1, 4);
out = Swerve_Evaluate_Case(caseIn, p);
```

随后可以查看：

```matlab
out.v_drive
out.theta_steer
out.drive_torque
out.steer_torque
out.dyn_debug.Fz
out.dyn_debug.grip_usage
out.slip_flag
```

---

## 15. 当前工程的特点

### 15.1 优点

- 模块划分清晰，配置、运动学、动力学、评估、展示和测试相互分离。
- 参数集中管理，便于快速切换不同机器人配置。
- 运动学考虑了舵轮 Flip 优化和电机角速度限幅，比简单逆解更贴近真实舵轮系统。
- 动力学考虑了载荷转移、轮胎硬度、接触形变、摩擦极限和安全冗余。
- 同时提供命令行报告和交互式 UI 两种使用方式。
- 有物理一致性测试和冒烟测试，便于后续迭代时验证基本正确性。

### 15.2 需要注意的假设与简化

- 驱动力分配策略按各轮法向载荷比例分配，未实现真实控制器中的最优力分配或摩擦圆约束优化。
- 动力学模型未显式考虑电机扭矩-转速曲线、电池电压、电流限制和热衰减。
- 轮胎模型是工程近似模型，不是完整有限元或 Pacejka 轮胎模型。
- 转向扭矩估算使用接触面和摩擦阻力矩的简化表达。
- `Wheel-Leg` 相关内容目前只是预留接口，尚未真正接入轮腿模型。
- 工程目前没有包结构或命名空间，所有 `.m` 文件需要处于 MATLAB 路径下。

---