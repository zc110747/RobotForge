/**
 * =============================================================================
 * kinematics.d.ts —— `kinematics.js` 的类型声明
 * -----------------------------------------------------------------------------
 * 为什么手写 `.d.ts` 而不是把实现改成 `.ts`：
 *
 * ```text
 * ① 包内已经有两份 Python（fk.py / ik.py），它们的执行方式是由
 *    `manifest.yaml` 的 `entry` 指到 `.py` 文件、由 Core 用
 *    `spec_from_file_location` 按**绝对路径**加载的。JS 侧如果写成 `.ts`，
 *    要么前端先编译一遍（多一个构建步骤、多一份产物），
 *    要么 Core / Node 都得学会剥离类型 —— 两者都在给"包"引入构建依赖。
 *
 * ② `.js` 是**零依赖可执行**的：`node kinematics.js` 就能跑，
 *    连 `--experimental-strip-types` 都不需要。包是"数据 + 算法"，
 *    不该要求消费者有 TypeScript。
 *
 * ③ 代价是消费方（frontend/）拿不到类型 ⇒ 用这份 `.d.ts` 补上。
 *    它**只描述形状，不含逻辑**，所以不可能与 `.js` 产生行为分歧；
 *    形状对不上的话，`kinematics.check.ts` 里的 749 条断言会立刻炸。
 * ```
 *
 * ⚠️ 这份声明**不是真值源**。真值源是 `kinematics.js` 本身 + Python 侧
 *    （`fk.py` / `ik.py`）与之的逐点比对结果（见 `docs/robot-package.md` §5.3）。
 *    改动 `.js` 的导出面时，这份文件必须同步 —— 但"忘了改"会被
 *    `tsc --noEmit` 或交叉验证脚本当场抓住，不会静默漂移。
 *
 * ⚠️ 消费方式（为什么是显式相对路径 + 显式 `.js` 扩展名）：
 *    `frontend/tsconfig.json` 的 `include` 只有 `["src"]`，包目录不在其中，
 *    所以这里**不能**靠"同目录自动发现"。
 *    调用方在 import 之前加一行：
 *    ```ts
 *    /// <reference path="../../../../packages/mini_arm/kinematics/kinematics.d.ts" />
 *    import { forwardKinematics } from ".../kinematics.js";
 *    ```
 *    三重斜杠引用是**编译期**的东西，不会进运行时、不影响 Vite 打包。
 */

/** 三维向量 `[x, y, z]`。 */
export type Vec3 = readonly [number, number, number];

/** 四元数 `[x, y, z, w]` —— RobotForge 约定，**不是** MuJoCo 的 `[w,x,y,z]`。 */
export type Quat = readonly [number, number, number, number];

/** 位姿：位置 + 朝向。 */
export interface Pose {
  readonly position: Vec3;
  readonly orientation: Quat;
}

/** IK 的两条分支。 */
export type Branch = "elbow_up" | "elbow_down";

/** 关节角（rad），按关节 id 索引。 */
export type JointPositions = Readonly<Record<string, number>>;

// -----------------------------------------------------------------------------
// 几何常量（与 Python 侧 `fk.py` 文件内常量逐位一致）
// -----------------------------------------------------------------------------

/** 基座高度（m）。 */
export declare const BASE_HEIGHT: number;
/** 肩关节沿 +Y 的偏置（m）。 */
export declare const SHOULDER_OFFSET: number;
/** 大臂长度（m）。 */
export declare const L1: number;
/** 小臂长度（m）。 */
export declare const L2: number;
/** 末端工具长度（m）。 */
export declare const L_TOOL: number;
/** 小臂 + 工具的等效长度 `L2 + L_TOOL`（m）。 */
export declare const L2_EFF: number;
/** 零位 TCP 的 z（m），`BASE_HEIGHT + SHOULDER_OFFSET`。 */
export declare const Z_BASE: number;

/** 关节顺序，固定为 `["base_yaw", "shoulder", "elbow"]`（已冻结）。 */
export declare const JOINT_ORDER: readonly string[];

/** 各关节限位（rad）。 */
export declare const YAW_LIMIT: number;
export declare const SHOULDER_LIMIT: number;
export declare const ELBOW_LIMIT: number;

/** 退化位形容差（rad）—— 由扫描出的空白带定容差，非拍脑袋。 */
export declare const TOL_SINGULAR: number;

// -----------------------------------------------------------------------------
// 四元数 / 向量原语
// -----------------------------------------------------------------------------

/** 单位四元数 `[0,0,0,1]`。 */
export declare function quatIdentity(): Quat;
/** 绕过原点的单位轴 `axis` 转 `angle`（rad）的四元数。 */
export declare function quatFromAxisAngle(axis: Vec3, angle: number): Quat;
/** 四元数乘法 `a ∘ b`（先 `b` 后 `a`）。 */
export declare function quatMul(a: Quat, b: Quat): Quat;
/** 归一化；模长过小 ⇒ 抛错（`what` 用于错误信息）。 */
export declare function quatNormalized(q: Quat, what?: string): Quat;
/** 用四元数旋转向量。 */
export declare function quatRotate(q: Quat, v: Vec3): Vec3;
/** 位姿复合：`a` 作用于 `b` 的父系。 */
export declare function poseCompose(a: Pose, b: Pose): Pose;
/** 单位位姿。 */
export declare function poseIdentity(): Pose;

// -----------------------------------------------------------------------------
// 模型读取（轴从模型读，不硬编码）
// -----------------------------------------------------------------------------

/** 取 `model.joints[jointId].axis`，缺失抛错。 */
export declare function axisOf(model: unknown, jointId: string): Vec3;

// -----------------------------------------------------------------------------
// FK
// -----------------------------------------------------------------------------

/**
 * 解析式正运动学。
 *
 * ⚠️ 几何量（`L1` / `L2_EFF` / `BASE_HEIGHT`…）是**文件内常量**，
 *    只从 `model` 取旋转轴 ⇒ **改 MJCF 几何时本函数输出不变**。
 *    验证"读对了 MJCF"必须用 Core 的 `backend/kinematics/fk.py`。
 *
 * @param model `RobotModel`（或含 `joints` 的等价对象）
 * @param jointPositions 关节角；**未提供的关节按 0 处理**（刻意契约）
 */
export declare function forwardKinematics(
  model: unknown,
  jointPositions?: JointPositions,
): Pose;

// -----------------------------------------------------------------------------
// IK
// -----------------------------------------------------------------------------

/** IK 相关错误基类。 */
export declare class IkError extends Error {}
/** 目标超出可达范围。 */
export declare class UnreachableError extends IkError {
  readonly distance: number;
  readonly reachMin: number;
  readonly reachMax: number;
}
/** 结果违反关节限位（`clamp` 为假时抛出）。 */
export declare class LimitViolationError extends IkError {
  readonly jointPositions: JointPositions;
}

/** 可达半径区间 `[|L1−L2_EFF|, L1+L2_EFF]`。 */
export declare function reachLimits(): readonly [number, number];

export interface SolveIkOptions {
  readonly branch?: Branch;
  readonly preferLimits?: boolean;
  readonly clamp?: boolean;
  readonly tol?: number;
}

export interface SolveIkResult {
  readonly jointPositions: JointPositions;
  readonly branch: Branch;
  readonly positionError: number;
  readonly clamped: boolean;
  readonly clampedJoints: readonly string[];
  /**
   * 姿态误差。
   *
   * ⚠️ 它是**量化**的：`2·acos(dot)` 在 `dot ≈ 1` 处的分辨率下限约 `4.2e-8 rad`
   *    （1 ulp 的 `dot` 误差）。所以 `2.98e-8` 就是"零"在浮点下的表示，
   *    判据必须按量级分流，不能直接与 `0` 比。
   */
  readonly orientationError: number;
}

/**
 * 单分支 IK。
 *
 * ⚠️ 四个不可凭直觉重写的语义细节（Python 侧都曾写错过）：
 * ```text
 * ① phi_target = atan2(-z_arm, r)                       ★ 负号
 * ② θ2>0 ⇒ θ1 = phi − alpha；θ2<0 ⇒ θ1 = phi + alpha    ★ 写反静默错 0.15 m
 * ③ 分支判据 = 肘侧偏的二维叉积，不是 θ2 的符号
 * ④ 退化判据 = 两个候选 θ2 实质相同，不是侧偏之差（±0 会算出相反符号）
 * ```
 */
export declare function solveIk(
  model: unknown,
  target: Pose,
  options?: SolveIkOptions,
): SolveIkResult;

/** 所有分支的解（含奇异位形去重）。 */
export declare function solveIkAll(
  model: unknown,
  target: Pose,
  options?: { readonly clamp?: boolean },
): readonly (SolveIkResult & { readonly degenerate?: boolean })[];
