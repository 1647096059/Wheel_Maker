function Swerve_PhysicsConsistency_Test(verbose)
% SWERVE_PHYSICSCONSISTENCY_TEST 舵轮底盘物理规律一致性单元测试
%
% 描述:
%   验证动力学核心在各极端解算工况下，其载荷转移与运动方向是否严格符合刚体物理定律。
%   这构成了本算法库的基础信任链 (CI / TDD 测试基准)。
%
% 覆盖范围:
%   1. 静态重力均摊原则
%   2. 纵向加减速重心转移定律
%   3. 横向加减速重心转移定律
%   4. 纯自转运动学空间投影一致性

    if nargin < 1, verbose = true; end
    if verbose, clc; end

    fprintf('========== Swerve 物理一致性与边界校验测试 ==========\n');
    p = Config_Params.toStruct();

    testStaticLoad(p);
    testForwardAccelerationLoadTransfer(p);
    testBrakingLoadTransfer(p);
    testRightAccelerationLoadTransfer(p);
    testLeftAccelerationLoadTransfer(p);
    testPureRotationKinematics(p);

    fprintf('========== 校验通过 (All Physics Tests Passed) ==========\n');
end

% ---------------- 测试 1: 静态均摊 ----------------
function testStaticLoad(p)
    fprintf('\n[1] 验证静态重力均摊...');
    [~, ~, slip, dbg] = Swerve_Dynamics_Core(0, 0, zeros(1, 4), p);
    expected = ones(1, 4) * p.swerve_m_total * p.g / 4;
    assertCloseVec('静态 Fz', dbg.Fz, expected, 1e-8);
    assertCondition(all(slip == 0), '静态不应打滑');
    fprintf(' [PASS] Fz = [%.2f %.2f %.2f %.2f] N\n', dbg.Fz);
end

% ---------------- 测试 2: 加速重心后移 ----------------
function testForwardAccelerationLoadTransfer(p)
    fprintf('[2] 验证前向加速 (ax>0) 后轮增压规律...');
    ax = 1.0; ay = 0.0;
    [~, ~, ~, dbg] = Swerve_Dynamics_Core(ax, ay, zeros(1, 4), p);
    Fz = dbg.Fz;
    frontAvg = mean(Fz([1, 2])); rearAvg  = mean(Fz([3, 4]));
    assertCondition(rearAvg > frontAvg, '后轮平均载荷应大于前轮');
    assertCloseScalar('前轴左右平衡', Fz(1), Fz(2), 1e-8);
    fprintf(' [PASS] 前压=%.2f, 后压=%.2f\n', frontAvg, rearAvg);
end

% ---------------- 测试 3: 刹车重心前移 ----------------
function testBrakingLoadTransfer(p)
    fprintf('[3] 验证前向刹车 (ax<0) 前轮增压规律...');
    ax = -1.0; ay = 0.0;
    [~, ~, ~, dbg] = Swerve_Dynamics_Core(ax, ay, zeros(1, 4), p);
    Fz = dbg.Fz;
    frontAvg = mean(Fz([1, 2])); rearAvg  = mean(Fz([3, 4]));
    assertCondition(frontAvg > rearAvg, '前轮平均载荷应大于后轮');
    fprintf(' [PASS] 前压=%.2f, 后压=%.2f\n', frontAvg, rearAvg);
end

% ---------------- 测试 4/5: 横向侧倾规律 ----------------
function testRightAccelerationLoadTransfer(p)
    fprintf('[4] 验证向右加速 (ay>0) 左轮增压规律...');
    ax = 0.0; ay = 1.0;
    [~, ~, ~, dbg] = Swerve_Dynamics_Core(ax, ay, zeros(1, 4), p);
    leftAvg = mean(dbg.Fz([1, 4])); rightAvg = mean(dbg.Fz([2, 3]));
    assertCondition(leftAvg > rightAvg, '左轮侧载荷应大于右轮');
    fprintf(' [PASS] 左侧=%.2f, 右侧=%.2f\n', leftAvg, rightAvg);
end

function testLeftAccelerationLoadTransfer(p)
    fprintf('[5] 验证向左加速 (ay<0) 右轮增压规律...');
    ax = 0.0; ay = -1.0;
    [~, ~, ~, dbg] = Swerve_Dynamics_Core(ax, ay, zeros(1, 4), p);
    leftAvg = mean(dbg.Fz([1, 4])); rightAvg = mean(dbg.Fz([2, 3]));
    assertCondition(rightAvg > leftAvg, '右轮侧载荷应大于左轮');
    fprintf(' [PASS] 左侧=%.2f, 右侧=%.2f\n', leftAvg, rightAvg);
end

% ---------------- 测试 6: 纯自转投影验证 ----------------
function testPureRotationKinematics(p)
    fprintf('[6] 验证纯自转运动学空间投影等效性...');
    [~, ~, ~, kin_dbg] = Swerve_Kinematics_Solver(0, 0, 1.0, zeros(1, 4), 0.02, p);
    Lx = p.swerve_wheel_base_x / 2; Ly = p.swerve_wheel_base_y / 2;
    expected_speed = hypot(Lx, Ly);
    assertCloseVec('纯旋转 raw_speed', kin_dbg.raw_speed, expected_speed * ones(1, 4), 1e-8);
    fprintf(' [PASS] 四轮转速严格等效。\n');
end

% ---------------- 断言基类 ----------------
function assertCondition(condition, message)
    if ~condition, error('Swerve_PhysicsConsistency_Test:Failed', '%s', message); end
end

function assertCloseScalar(name, actual, expected, tol)
    if abs(actual - expected) > tol, error('TestFailed: %s 不一致', name); end
end

function assertCloseVec(name, actual, expected, tol)
    actual = actual(:); expected = expected(:);
    if max(abs(actual - expected)) > tol, error('TestFailed: %s 向量不一致', name); end
end

function assertCloseAngleVec(name, actual, expected, tol)
    err = max(abs(mod(actual - expected + pi, 2*pi) - pi));
    if err > tol, error('TestFailed: %s 角度偏差超限', name); end
end