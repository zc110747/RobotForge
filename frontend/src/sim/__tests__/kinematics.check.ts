/**
 * JS 侧运动学引擎的**交叉验证**：与 Python 侧逐点比对。
 *
 * 运行：
 * ```bash
 * node --experimental-strip-types frontend/src/sim/__tests__/kinematics.check.ts
 * ```
 * （`.ts` 只为让 Node 的类型剥离生效；内容全是 JS 兼容语法。）
 *
 * =============================================================================
 * 这个脚本在验什么
 * =============================================================================
 *
 * ```text
 * 夹具（Python 侧独立推导）       被测对象（JS 侧）
 *   fk.py::forward_kinematics  →   kinematics.js::forwardKinematics
 *   ik.py::solve               →   kinematics.js::solveIk
 * ```
 *
 * **判据全部来自夹具**，不来自 JS 侧的自我一致 ——
 * "JS 算的等于 JS 算的"是同义反复，什么也证明不了。
 *
 * =============================================================================
 * ⚠️ 容差为什么是 1e-15 而不是 1e-9
 * =============================================================================
 *
 * 两侧是**同一套公式**、同一个 IEEE754 double。差异只可能来自：
 * ```text
 * ① 运算顺序：JS 的 Math.hypot / Math.acos 与 Python 的 math.* 是不同实现，
 *    可能差 1-2 ulp（对 0.1 量级的数，1 ulp ≈ 1.4e-17）
 * ② 表达式重排：如 a*b+c 的求值顺序
 * ```
 * 实测最大偏差见脚本输出。若某天它涨到 1e-9 量级，说明**公式真的改了**，
 * 而不是"浮点噪声大了一点" —— 这正是把容差收紧的价值。
 *
 * ⚠️ 反例注射（脚本末尾）会故意破坏公式，断言必须变红。
 *    若注射后仍然全绿，说明判据没在量它声称要量的东西。
 */

import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, resolve } from "node:path";

/// <reference path="../../../../packages/mini_arm/kinematics/kinematics.d.ts" />
import {
  BASE_HEIGHT,
  SHOULDER_OFFSET,
  L1,
  L2,
  L_TOOL,
  L2_EFF,
  Z_BASE,
  JOINT_ORDER,
  YAW_LIMIT,
  SHOULDER_LIMIT,
  ELBOW_LIMIT,
  TOL_SINGULAR,
  forwardKinematics,
  solveIk,
  solveIkAll,
  reachLimits,
  IkError,
  UnreachableError,
  LimitViolationError,
  quatFromAxisAngle,
  quatIdentity,
} from "../../../../packages/mini_arm/kinematics/kinematics.js";
import type { Branch, Vec3, Quat, Pose } from "../../../../packages/mini_arm/kinematics/kinematics.js";

// =============================================================================
// 断言器（与 semantics.check.ts 同风格：每次新建 failures，不是模块级全局）
// =============================================================================

interface Failure {
  readonly check: string;
  readonly detail: string;
}

interface Checker {
  readonly failures: Failure[];
  ok(cond: boolean, check: string, detail: string): void;
  eq<T>(actual: T, expected: T, check: string): void;
  close(actual: number, expected: number, tol: number, check: string): void;
}

function newChecker(): Checker {
  const failures: Failure[] = [];
  return {
    failures,
    ok(cond: boolean, check: string, detail: string) {
      if (!cond) failures.push({ check, detail });
    },
    eq<T>(actual: T, expected: T, check: string) {
      if (actual !== expected) {
        failures.push({ check, detail: `期望 ${JSON.stringify(expected)}，实得 ${JSON.stringify(actual)}` });
      }
    },
    close(actual: number, expected: number, tol: number, check: string) {
      const diff = Math.abs(actual - expected);
      if (!(diff <= tol)) {
        failures.push({ check, detail: `期望 ${expected}，实得 ${actual}，差 ${diff.toExponential(3)} > ${tol}` });
      }
    },
  };
}

// =============================================================================
// 容差：**由实测分布定出来的**，不是猜的
// =============================================================================
//
// 两侧是同一套公式、同一个 IEEE754 double，差异只可能来自：
// ```text
// ① 库函数实现差异：JS 的 Math.hypot/acos 与 Python 的 math.* 是不同实现，
//    可能差 1-2 ulp（对 0.1 量级的数，1 ulp ≈ 1.4e-17）
// ② 表达式求值顺序
// ```
//
// ## 为什么 IK 的容差比 FK 大 100 倍（6.0e-15 vs 1.1e-16）
//
// 因为 IK 里有两处 `acos`，而 `acos` 在自变量趋近 ±1 时导数发散：
// ```text
// d(acos x)/dx = -1/√(1-x²)      x→1 时 →∞
// ```
// 输入上 1 ulp 的差异会被放大若干倍。实测最大 6.0e-15 rad ≈ 27 ulp，
// 取 1e-13 留约 16 倍余量 —— 足以吃掉库函数差异，又不足以放过
// "公式真的改了"（那种改动是 1e-3 量级起步）。
//
// ⚠️ 收紧容差的价值：若某天它涨到 1e-9，说明**公式真的改了**，
//    而不是"浮点噪声大了一点"。反过来，若这里需要 1e-6 才能过，
//    说明实现有实质分歧，不该用"放宽容差"掩盖。
const TOL_FK_POS = 1e-15;
const TOL_FK_QUAT = 1e-15;
const TOL_IK_JOINT = 1e-13;
const TOL_IK_POSERR = 1e-14;
const TOL_ROUNDTRIP = 1e-12;

/**
 * `2·acos(dot)` 的**数值分辨率下限**。
 *
 * ## 为什么需要这个常量
 *
 * `orientationError` 的返回值分布是**量化**的，不是连续的：
 * ```text
 * 实测（83 个 IK 用例，去重后 29 个不同取值）：
 *   0.0            37 个 —— 同分支解，姿态精确闭合
 *   2.98e-08       若干  —— ← "零"在 double 下的实际形状
 *   0.114 ~ 3.142  其余  —— d<0 的镜像分支，姿态真的差 180°
 * ```
 *
 * `dot` 在 1 附近的最小可分辨变化是一个 ulp（≈2.22e-16），
 * 而 `2·acos(1 − 1ulp) ≈ 4.2e-8`。⇒ **2.98e-8 不是"误差"，
 * 它就是"零"这个值在浮点下的表示**。
 *
 * 因此判据不能写成"|got − ref| < 常量"：
 * ```text
 * ref = 0      ⇒ 要求 |got| 也落在分辨率下限内（否则是无意义的比较）
 * ref = 0.114  ⇒ 要求普通容差
 * ```
 * 下面用 `compareOrientationError` 按量级分流，而不是挑一个折中的大容差
 * （那会把"真的差 1e-8"和"其实是零"混为一谈）。
 */
const ACOS_RESOLUTION = 6e-8;

// =============================================================================
// 姿态误差专用比较（见上方 ACOS_RESOLUTION 的说明）
// =============================================================================

/**
 * 比较两个 `orientationError`，按**量级分流**。
 *
 * ```text
 * ref ≤ 分辨率 ⇒ 两者都必须 ≤ 分辨率（这是在比"两个零"）
 * ref > 分辨率 ⇒ 普通容差（这是在比真实的姿态差）
 * ```
 * 混用会让"真的是零"与"差了一点点"无法区分 —— 而这两件事的
 * 排查方向完全不同（前者是浮点，后者可能是真 bug）。
 */
function compareOrientationError(got: number, ref: number, c: Checker, check: string): void {
  if (ref <= ACOS_RESOLUTION) {
    c.ok(
      got <= ACOS_RESOLUTION,
      check,
      `参考值 ${ref.toExponential(3)} 落在 acos 分辨率内（视作 0），` +
        `实得 ${got.toExponential(3)} 超出分辨率 ${ACOS_RESOLUTION}`
    );
    return;
  }
  c.close(got, ref, 1e-13, check);
}

const HERE = dirname(fileURLToPath(import.meta.url));
const FIXTURE_PATH = resolve(HERE, "fixtures", "kinematics.json");

// Pose / Quat / Vec3 直接复用包内 `.d.ts` 的声明（不在这里另写一份）：
// 另写一份的话，"夹具的 branch 是 string、引擎的 branch 是 Branch" 这类
// 形状分歧会被掩盖，而不是在 `tsc --noEmit` 时暴露。
interface Fixture {
  readonly constants: Record<string, number | string[]>;
  readonly fkCases: ReadonlyArray<{ name: string; q: Record<string, number>; analytic: Pose; generic: Pose }>;
  readonly fkContracts: ReadonlyArray<{
    name: string;
    q: Record<string, number>;
    analytic: Pose;
    reference: Pose;
  }>;
  readonly ikCases: ReadonlyArray<{
    name: string;
    target: Pose;
    branch: Branch;
    expect: { joints: Record<string, number>; branch: string; positionError: number; orientationError: number };
  }>;
  readonly ikErrors: ReadonlyArray<{
    name: string;
    target: Pose;
    branch: Branch;
    errorType: string;
    distance?: number;
    reachMin?: number;
    reachMax?: number;
  }>;
}

const fixture: Fixture = JSON.parse(readFileSync(FIXTURE_PATH, "utf-8"));

/** 用于对比的"模型"：只提供 axis（JS 引擎从模型读轴，长度用文件内常量）。 */
function modelFor(axes: Record<string, Vec3>): unknown {
  return {
    joints: Object.entries(axes).map(([id, axis]) => ({ id, axis: [...axis] })),
    sites: [],
  };
}

/** mini_arm 的轴（真值来自 MJCF，此处由夹具侧断言保证） */
const MINI_ARM_AXES: Record<string, Vec3> = {
  base_yaw: [0, 0, 1],
  shoulder: [0, 1, 0],
  elbow: [0, 1, 0],
};
const MODEL = modelFor(MINI_ARM_AXES);

// =============================================================================
// 检查 1：常量逐个相等
// =============================================================================

function runConstants(c: Checker): void {
  const expected: Array<[string, number]> = [
    ["BASE_HEIGHT", BASE_HEIGHT],
    ["SHOULDER_OFFSET", SHOULDER_OFFSET],
    ["L1", L1],
    ["L2", L2],
    ["L_TOOL", L_TOOL],
    ["L2_EFF", L2_EFF],
    ["Z_BASE", Z_BASE],
    ["YAW_LIMIT", YAW_LIMIT],
    ["SHOULDER_LIMIT", SHOULDER_LIMIT],
    ["ELBOW_LIMIT", ELBOW_LIMIT],
    ["TOL_SINGULAR", TOL_SINGULAR],
  ];
  for (const [name, value] of expected) {
    const ref = fixture.constants[name];
    c.ok(typeof ref === "number", `constants.${name} 存在于夹具`, `夹具缺少 ${name}`);
    if (typeof ref === "number") {
      // 常量是**逐位**复制的，容差取 0：若这里需要容差，说明有人手改了数值
      c.ok(value === ref, `constants.${name} 与 Python 逐位相等`, `JS=${value} Python=${ref}`);
    }
  }
  const orderRef = fixture.constants["JOINT_ORDER"];
  c.ok(
    Array.isArray(orderRef) && orderRef.join(",") === JOINT_ORDER.join(","),
    "constants.JOINT_ORDER 与 Python 一致",
    `JS=${JOINT_ORDER.join(",")} Python=${String(orderRef)}`
  );
}

// =============================================================================
// 检查 2：FK 与 Python 解析解逐点一致
// =============================================================================

/** 观测到的最大偏差（打印出来，让"容差够不够"有据可依） */
const observed = {
  fkVsPython: 0,
  fkVsCore: 0,
  ikJoints: 0,
  ikPosErr: 0,
};

function runFk(c: Checker): void {
  for (const kase of fixture.fkCases) {
    const got = forwardKinematics(MODEL, kase.q);
    const ref = kase.analytic;

    const dPos = Math.hypot(
      got.position[0] - ref.position[0],
      got.position[1] - ref.position[1],
      got.position[2] - ref.position[2]
    );
    observed.fkVsPython = Math.max(observed.fkVsPython, dPos);
    c.close(dPos, 0, TOL_FK_POS, `FK 位置 vs Python 解析解 [${kase.name}]`);

    // 姿态：四元数取 |dot| 比较（q 与 -q 是同一旋转）
    const dot = Math.abs(
      got.orientation[0] * ref.orientation[0] +
        got.orientation[1] * ref.orientation[1] +
        got.orientation[2] * ref.orientation[2] +
        got.orientation[3] * ref.orientation[3]
    );
    c.close(dot, 1, 1e-12, `FK 姿态 vs Python 解析解 [${kase.name}]`);

    // 与 +q 的分量差（本机构不产生符号翻转，直接分量比更严格）
    let maxComp = 0;
    for (let i = 0; i < 4; i++) {
      maxComp = Math.max(maxComp, Math.abs(got.orientation[i]! - ref.orientation[i]!));
    }
    c.close(maxComp, 0, TOL_FK_QUAT, `FK 姿态分量 vs Python [${kase.name}]`);

    // 顺带：与 Core 通用解也在容差内（证明 JS 与"独立路径"对得上）
    const dCore = Math.hypot(
      ref.position[0] - kase.generic.position[0],
      ref.position[1] - kase.generic.position[1],
      ref.position[2] - kase.generic.position[2]
    );
    observed.fkVsCore = Math.max(observed.fkVsCore, dCore);
    c.ok(dCore < 1e-12, `解析解与 Core 通用解一致 [${kase.name}]`, `差 ${dCore.toExponential(3)} m`);
  }
}

// =============================================================================
// 检查 3：FK 的两个刻意契约（缺关节按 0 / 多余键忽略）
// =============================================================================

function runFkContracts(c: Checker): void {
  for (const contract of fixture.fkContracts) {
    const got = forwardKinematics(MODEL, contract.q);
    const ref = contract.reference;

    let maxComp = 0;
    for (let i = 0; i < 3; i++) {
      maxComp = Math.max(maxComp, Math.abs(got.position[i]! - ref.position[i]!));
    }
    for (let i = 0; i < 4; i++) {
      maxComp = Math.max(maxComp, Math.abs(got.orientation[i]! - ref.orientation[i]!));
    }
    c.close(maxComp, 0, 1e-15, `FK 契约 [${contract.name}]`);

    // 且夹具自己也一致（防止夹具构造错了却"JS 通过"）
    let refSelf = 0;
    for (let i = 0; i < 3; i++) {
      refSelf = Math.max(refSelf, Math.abs(contract.analytic.position[i]! - ref.position[i]!));
    }
    c.close(refSelf, 0, 1e-15, `FK 契约的 Python 参考自洽 [${contract.name}]`);
  }
}

// =============================================================================
// 检查 4：IK 与 Python 逐点一致（含分支字符串**完全相等**）
// =============================================================================

function runIk(c: Checker): void {
  for (const kase of fixture.ikCases) {
    const got = solveIk(MODEL, kase.target, { branch: kase.branch });

    // ⚠️ 分支不许"差不多" —— 字符串必须完全相等。
    //    分支判据写反会让机械臂以**镜像姿态**够同一个点：
    //    位置仍然对，姿态与位形完全不同。用容差比位置会放过它。
    c.eq(got.branch, kase.expect.branch, `IK 分支字符串 [${kase.name}]`);

    for (const id of JOINT_ORDER) {
      const ref = kase.expect.joints[id]!;
      const diff = Math.abs(got.jointPositions[id]! - ref);
      observed.ikJoints = Math.max(observed.ikJoints, diff);
      c.close(diff, 0, TOL_IK_JOINT, `IK 关节角 ${id} [${kase.name}]`);
    }

    observed.ikPosErr = Math.max(observed.ikPosErr, Math.abs(got.positionError - kase.expect.positionError));
    c.close(
      got.positionError,
      kase.expect.positionError,
      TOL_IK_POSERR,
      `IK positionError [${kase.name}]`
    );

    if (kase.expect.orientationError !== null) {
      compareOrientationError(
        got.orientationError,
        kase.expect.orientationError,
        c,
        `IK orientationError [${kase.name}]`
      );
    }
  }
}

// =============================================================================
// 检查 5：IK 的错误路径（异常类型必须匹配）
// =============================================================================

function runIkErrors(c: Checker): void {
  for (const kase of fixture.ikErrors) {
    let thrown: unknown = null;
    try {
      solveIk(MODEL, kase.target, { branch: kase.branch });
    } catch (exc) {
      thrown = exc;
    }
    c.ok(thrown !== null, `IK 错误路径确实抛错 [${kase.name}]`, "没有抛错，但夹具说应该抛");

    if (thrown === null) continue;

    const name = (thrown as Error).name;
    c.eq(name, kase.errorType, `IK 错误类型 [${kase.name}]`);

    // 不可达时还要报告可达范围（UI 要用它提示用户）
    if (kase.errorType === "UnreachableError" && kase.distance !== undefined) {
      const err = thrown as UnreachableError;
      c.close(err.distance, kase.distance, 1e-15, `UnreachableError.distance [${kase.name}]`);
      c.close(err.reachMin, kase.reachMin ?? NaN, 1e-15, `UnreachableError.reachMin [${kase.name}]`);
      c.close(err.reachMax, kase.reachMax ?? NaN, 1e-15, `UnreachableError.reachMax [${kase.name}]`);
    }
  }
}

// =============================================================================
// 检查 6：IK 自己的不变量（不依赖夹具的部分）
// =============================================================================

function runIkInvariants(c: Checker): void {
  // ① 往返：FK 生成的目标，IK 解回去后 FK 应**精确**回到同一位置
  //    （解析解 ⇒ 应当是机器精度级，不是"容差内"）
  let worstRoundTrip = 0;
  for (const kase of fixture.fkCases) {
    // 跳过退化位形（两分支合并，往返仍成立但分支标签无意义）
    const t2 = kase.q["elbow"] ?? 0;
    if (Math.abs(t2) < 1e-3 || Math.abs(Math.abs(t2) - Math.PI) < 1e-3) continue;

    // 显式标 `Branch[]`：不标的话 TS 推成 `string[]`，
    // 会在 `{ branch }` 处报"string 不能赋给 Branch"，与语义无关。
    const bothBranches: readonly Branch[] = ["elbow_up", "elbow_down"];
    for (const branch of bothBranches) {
      let sol;
      try {
        sol = solveIk(MODEL, kase.analytic, { branch });
      } catch {
        continue; // 该分支超限，跳过（限位是模型的合法约束）
      }
      const back = forwardKinematics(MODEL, sol.jointPositions);
      const d = Math.hypot(
        back.position[0] - kase.analytic.position[0],
        back.position[1] - kase.analytic.position[1],
        back.position[2] - kase.analytic.position[2]
      );
      worstRoundTrip = Math.max(worstRoundTrip, d);
    }
  }
  // 解析解的往返应当是"精确闭合"（robot-package.md §6 的原话）
  c.ok(
    worstRoundTrip < TOL_ROUNDTRIP,
    "IK 往返位置精确闭合",
    `最差往返误差 ${worstRoundTrip.toExponential(3)} m ≥ ${TOL_ROUNDTRIP}`
  );

  // ② solveIkAll 在非退化位形上返回 2 个解，在退化位形上**去重成 1 个**
  const reachMin = Math.abs(L1 - L2_EFF);
  const reachMax = L1 + L2_EFF;
  const midR = 0.5 * (reachMin + reachMax);
  const bentTarget: Pose = {
    position: [midR, 0, Z_BASE],
    orientation: forwardKinematics(MODEL, { base_yaw: 0, shoulder: 0, elbow: 0 }).orientation,
  };
  const two = solveIkAll(MODEL, bentTarget);
  c.ok(two.length === 2, "solveIkAll 在一般位形返回 2 个解", `实得 ${two.length}`);

  // 完全伸展的目标（θ2 = 0）⇒ 两分支合并 ⇒ 去重成 1 个
  const fullExtend: Pose = forwardKinematics(MODEL, { base_yaw: 0, shoulder: 0, elbow: 0 });
  const one = solveIkAll(MODEL, fullExtend);
  c.ok(one.length === 1, "solveIkAll 在退化位形去重成 1 个解", `实得 ${one.length}`);
  if (one.length === 1) {
    const only = one[0]!;
    c.ok(only.branch.includes("degenerate"), "退化解被标注 degenerate", `branch = ${only.branch}`);
  }

  // ③ 可达范围与常量自洽
  const [lo, hi] = reachLimits();
  c.close(lo, Math.abs(L1 - L2_EFF), 0, "reachLimits 下界 = |L1-L2eff|");
  c.close(hi, L1 + L2_EFF, 0, "reachLimits 上界 = L1+L2eff");

  // ④ 未提供的关节按 0 处理（空字典 = 全零位形）
  const empty = forwardKinematics(MODEL, {});
  const zero = forwardKinematics(MODEL, { base_yaw: 0, shoulder: 0, elbow: 0 });
  let maxDiff = 0;
  for (let i = 0; i < 3; i++) maxDiff = Math.max(maxDiff, Math.abs(empty.position[i]! - zero.position[i]!));
  c.close(maxDiff, 0, 0, "FK 空字典 = 零位形（缺关节按 0）");

  // ⑤ 未知 branch ⇒ IkError（且不是 Unreachable/Limit 子类）
  let unknownErr: unknown = null;
  try {
    // 刻意传一个**不在联合类型里**的值：这里就是要验运行期的容错，
    // 用 `as unknown as Branch` 绕过编译期检查，否则这条路径测不到。
    solveIk(MODEL, bentTarget, { branch: "nope" as unknown as Branch });
  } catch (exc) {
    unknownErr = exc;
  }
  c.ok(unknownErr instanceof IkError, "未知 branch 抛 IkError", `实得 ${String(unknownErr)}`);
  c.ok(
    !(unknownErr instanceof UnreachableError) && !(unknownErr instanceof LimitViolationError),
    "未知 branch 不是 Unreachable/Limit 子类",
    "错误类型过于具体，会让调用方的 catch 分支误判"
  );

  // ⑥ 轴从模型读（不是硬编码）—— 传一个改过轴的模型，结果必须变
  const weirdModel = modelFor({ base_yaw: [0, 0, 1], shoulder: [1, 0, 0], elbow: [0, 1, 0] });
  const normalPose = forwardKinematics(MODEL, { base_yaw: 0.3, shoulder: 0.4, elbow: 0.5 });
  const weirdPose = forwardKinematics(weirdModel, { base_yaw: 0.3, shoulder: 0.4, elbow: 0.5 });
  let axisSensitivity = 0;
  for (let i = 0; i < 4; i++) {
    axisSensitivity = Math.max(
      axisSensitivity,
      Math.abs(normalPose.orientation[i]! - weirdPose.orientation[i]!)
    );
  }
  c.ok(
    axisSensitivity > 1e-6,
    "改模型里的 shoulder 轴 ⇒ FK 姿态跟着变（证明轴是读来的，不是写死的）",
    `姿态最大分量差仅 ${axisSensitivity.toExponential(3)}，轴可能被硬编码了`
  );

  // ⑦ 坐标系公理：X × Y = Z（右手系），以及 identity quaternion = [0,0,0,1]
  const idq = quatIdentity();
  c.ok(
    idq[0] === 0 && idq[1] === 0 && idq[2] === 0 && idq[3] === 1,
    "Identity Quaternion = [0,0,0,1]",
    `实得 [${idq.join(",")}]`
  );
  // 绕 +Z 转 90° 应把 +X 转到 +Y（右手系 X×Y=Z 的直接推论）
  const rz90 = quatFromAxisAngle([0, 0, 1], Math.PI / 2);
  const rotX = quatRotateLocal(rz90, [1, 0, 0]);
  c.close(rotX[0], 0, 1e-15, "Rz(90°)·(+X) 的 x 分量 = 0");
  c.close(rotX[1], 1, 1e-15, "Rz(90°)·(+X) 的 y 分量 = 1（右手系 ⇒ +X 转向 +Y）");
  c.close(rotX[2], 0, 1e-15, "Rz(90°)·(+X) 的 z 分量 = 0");

  // ⑧ 绕 +Y 转 θ>0 ⇒ +X 指向 -Z（fk.py 文档里的轴向语义）
  const ry30 = quatFromAxisAngle([0, 1, 0], 0.3);
  const rotX2 = quatRotateLocal(ry30, [1, 0, 0]);
  c.close(rotX2[2], -Math.sin(0.3), 1e-15, "Ry(θ)·(+X) 的 z 分量 = -sin θ（θ>0 向下偏）");
}

/** 本地转一下向量（避免为了一个断言再从模块导出一个函数） */
function quatRotateLocal(q: Quat, v: Vec3): Vec3 {
  const qv: Vec3 = [q[0], q[1], q[2]];
  const cross = (a: Vec3, b: Vec3): Vec3 => [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
  const t = cross(qv, v).map((x) => x * 2) as unknown as Vec3;
  const c1 = cross(qv, t);
  return [
    v[0] + t[0] * q[3] + c1[0],
    v[1] + t[1] * q[3] + c1[1],
    v[2] + t[2] * q[3] + c1[2],
  ];
}

// =============================================================================
// 自检：故意注入缺陷，断言必须变红
// =============================================================================
//
// 这一段证明"上面的判据真的在量它声称要量的东西"。
// 做法：用**篡改过的输入**喂给同一批判据，检查必须报错。
//
// ⚠️ 这些注入不改产品代码（不 monkey-patch 模块），而是构造"错误的期望"，
//    即模拟"JS 实现写错了"时判据的反应。

interface SelfTest {
  readonly name: string;
  readonly run: () => boolean; // true = 判据正确报错（期待）
}

const SELF_TESTS: SelfTest[] = [
  {
    // 模拟"配对符号写反"（ik.py 里那个静默错 0.15 m 的 bug）：
    // 把期望关节角取反，判据必须发现。
    //
    // ⚠️ 必须挑一个 shoulder **非零**的用例：若 shoulder ≈ 0，
    //    取反后还是 ≈0，注入等于没做，自检会"通过"而实际什么都没验。
    //    （这条是实测踩到的：第一版用 find() 拿到首个用例，恰好 shoulder=0。）
    name: "IK 配对符号写反 ⇒ 关节角判据变红",
    run: () => {
      const kase = fixture.ikCases.find(
        (k) => !k.expect.branch.includes("degenerate") && Math.abs(k.expect.joints["shoulder"]!) > 1e-3
      );
      if (!kase) return false; // 找不到可注入的用例 ⇒ 夹具退化，算自检失败
      const c = newChecker();
      const got = solveIk(MODEL, kase.target, { branch: kase.branch });
      // `tampered` 显式标成 `Record<string, number>`：原对象只有 `shoulder`
      // 一个键，不标的话 `tampered[id]` 会被推成 `number | undefined`。
      const tampered: Record<string, number> = {
        ...kase.expect.joints,
        shoulder: -kase.expect.joints["shoulder"]!,
      };
      for (const id of JOINT_ORDER) {
        c.close(Math.abs(got.jointPositions[id]! - tampered[id]!), 0, TOL_IK_JOINT, `tampered ${id}`);
      }
      return c.failures.length > 0;
    },
  },
  {
    // 模拟"分支判据用 θ2 的符号"⇒ branch 字符串会反，判据必须发现
    name: "IK 分支字符串判据能发现标签反了",
    run: () => {
      const kase = fixture.ikCases.find(
        (k) => !k.expect.branch.includes("[") // 非退化：标签必须严格是 up/down
      );
      if (!kase) return false;
      const c = newChecker();
      const got = solveIk(MODEL, kase.target, { branch: kase.branch });
      const flipped = kase.expect.branch === "elbow_up" ? "elbow_down" : "elbow_up";
      c.eq(got.branch, flipped as Branch, "tampered branch");
      return c.failures.length > 0;
    },
  },
  {
    // 模拟"常量被手改"⇒ 逐位相等判据必须发现。
    // 用 1 ulp 的最小扰动（nextafter），证明判据对**最细微**的改动也敏感。
    name: "常量被改 1 ulp ⇒ 逐位判据变红",
    run: () => {
      const c = newChecker();
      const ref = fixture.constants["L1"] as number;
      // 加一个可表示的最小增量（约 1.4e-17，远小于任何有意义的改动）
      const tampered = ref + Number.EPSILON * ref;
      c.ok(tampered === ref, "tampered constant", `常量相等（实得 ${tampered} vs ${ref}）`);
      return c.failures.length > 0;
    },
  },
  {
    // 模拟"姿态顺序写成 [w,x,y,z]"⇒ 分量判据必须发现
    name: "四元数顺序写错 ⇒ 姿态分量判据变红",
    run: () => {
      const kase = fixture.fkCases[0];
      if (!kase) return false;
      const q = kase.analytic.orientation;
      const reordered: Quat = [q[3]!, q[0]!, q[1]!, q[2]!]; // 误当成 wxyz
      const c = newChecker();
      let maxComp = 0;
      for (let i = 0; i < 4; i++) maxComp = Math.max(maxComp, Math.abs(reordered[i]! - q[i]!));
      c.close(maxComp, 0, 1e-15, "tampered quat order");
      return c.failures.length > 0;
    },
  },
  {
    // 模拟"solveIkAll 不去重"⇒ 退化位形的解数判据必须发现
    name: "退化位形不去重 ⇒ 解数判据变红",
    run: () => {
      const fullExtend = forwardKinematics(MODEL, { base_yaw: 0, shoulder: 0, elbow: 0 });
      const c = newChecker();
      // 断言"应该是 2 个"（错误的期望）—— 判据应当发现实得 1 个
      const got = solveIkAll(MODEL, fullExtend);
      c.ok(got.length === 2, "tampered dedup", `期望 2，实得 ${got.length}`);
      return c.failures.length > 0;
    },
  },
];

function runSelfTests(c: Checker): void {
  for (const st of SELF_TESTS) {
    const red = st.run();
    c.ok(red, `自检：${st.name}`, "注入缺陷后判据仍然全绿 ⇒ 判据没在量它声称量的东西");
  }
}

// =============================================================================
// 主流程
// =============================================================================

function main(): number {
  const c = newChecker();

  runConstants(c);
  runFk(c);
  runFkContracts(c);
  runIk(c);
  runIkErrors(c);
  runIkInvariants(c);

  // 自检单独用一个 checker（避免污染主检查的结果，反之亦然）
  const self = newChecker();
  runSelfTests(self);

  const totalCases =
    fixture.fkCases.length * 3 +
    fixture.fkContracts.length * 2 +
    fixture.ikCases.length * 6 +
    fixture.ikErrors.length * 2 +
    SELF_TESTS.length +
    20; // 不变量与公理的粗略计数

  console.log("");
  console.log("运动学交叉验证（JS ↔ Python）");
  console.log("─".repeat(62));
  console.log(`夹具        : ${FIXTURE_PATH}`);
  console.log(`FK 用例     : ${fixture.fkCases.length}`);
  console.log(`FK 契约     : ${fixture.fkContracts.length}`);
  console.log(`IK 用例     : ${fixture.ikCases.length}`);
  console.log(`IK 错误路径 : ${fixture.ikErrors.length}`);
  console.log("─".repeat(62));
  console.log("实测最大偏差（用于判断容差是否合理）：");
  console.log(`  FK vs Python 解析解 : ${observed.fkVsPython.toExponential(3)} m`);
  console.log(`  解析解 vs Core 通用 : ${observed.fkVsCore.toExponential(3)} m`);
  console.log(`  IK 关节角           : ${observed.ikJoints.toExponential(3)} rad`);
  console.log(`  IK positionError    : ${observed.ikPosErr.toExponential(3)} m`);
  console.log("─".repeat(62));

  const failures = [...c.failures];
  const selfFailures = [...self.failures];

  if (failures.length === 0 && selfFailures.length === 0) {
    console.log(`PASS  全部检查通过（约 ${totalCases} 条断言）`);
    console.log("      自检 5/5 注入均被捕获");
    return 0;
  }

  if (failures.length > 0) {
    console.log(`FAIL  ${failures.length} 条检查失败：`);
    for (const f of failures.slice(0, 25)) {
      console.log(`  ✗ ${f.check}`);
      console.log(`      ${f.detail}`);
    }
    if (failures.length > 25) console.log(`  ... 另有 ${failures.length - 25} 条`);
  }
  if (selfFailures.length > 0) {
    console.log(`FAIL  ${selfFailures.length} 条自检失败（判据本身有问题）：`);
    for (const f of selfFailures) {
      console.log(`  ✗ ${f.check}`);
      console.log(`      ${f.detail}`);
    }
  }
  return 1;
}

const isDirect =
  process.argv[1] !== undefined && resolve(process.argv[1]) === resolve(fileURLToPath(import.meta.url));
if (isDirect) {
  process.exit(main());
}

export { main, newChecker };
