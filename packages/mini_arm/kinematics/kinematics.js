/**
 * mini_arm 的**正/逆运动学** —— JS 侧实现（Robot Package 内的第三份实现）。
 *
 * =============================================================================
 * 这份文件为什么存在
 * =============================================================================
 *
 * RobotForge 的 FK 在 v0.1 里有**三份独立实现**，三者两两互为裁判：
 *
 * ```text
 * ① backend/kinematics/fk.py        通用链式：遍历 joint 链逐级相乘 Transform
 * ② packages/.../kinematics/fk.py   解析式：手推三角公式（几何常量写死）
 * ③ packages/.../kinematics/kinematics.js  ← 本文件，解析式的 JS 移植
 * ```
 *
 * ③ 的价值是让**前端预演**不必把 Python 的输出预先烧成表 ——
 * 预烧的表不是裁判，是一条会与真值漂移的缓存。本文件独立移植同一套三角公式，
 * 于是"前端算的位姿"与"后端算的位姿"可以在**同一个输入上逐点比对**。
 *
 * =============================================================================
 * ⚠️ 本文件**不能**用来证明"几何读对了 MJCF"
 * =============================================================================
 *
 * 解析解的几何长度（`L1` / `L2` / `L_TOOL` / `BASE_HEIGHT` / `SHOULDER_OFFSET`）
 * 是**文件内写死的常量** —— 这是 ② 的既有设计，③ 原样继承。
 *
 * 后果：**改 MJCF 的几何时，本文件的输出不变。**
 *
 * ```text
 * ✅ 本文件能证明：JS 侧与 Python 解析解数值一致（前端不会算出别的位姿）
 * ❌ 本文件不能证明：写死的常量与 MJCF 一致
 * ```
 *
 * 后者由 `packages/mini_arm/tests/`（常量 vs MJCF 逐项断言）与 Core 的
 * `forward_kinematics_generic`（真读模型几何）负责。
 * 要验证"FK 真的依赖几何"，**必须**用 Core 那条路径。
 *
 * =============================================================================
 * 坐标系与轴向（P0 契约）
 * =============================================================================
 *
 * `+X = 前` / `+Y = 左` / `+Z = 上`，右手系。
 * 肩、肘绕 **+Y** 旋转 ⇒ 连杆在 **XZ 平面**内运动；底座偏航绕 **+Z**。
 *
 * ```text
 *         |  cosθ   0   sinθ |
 * Ry(θ) = |   0     1    0   |      ⇒ 单位 +X → (cosθ, 0, -sinθ)
 *         | -sinθ   0   cosθ |         即 θ>0 时末端向 -Z（下）偏转
 * ```
 *
 * 四元数顺序全局统一为 **[x, y, z, w]**（**不是** Three.js 的默认顺序，
 * 也不是 MuJoCo 的 `[w,x,y,z]`）。转换只在各自的边界做一次。
 *
 * =============================================================================
 * ⚠️ 本文件里**没有**任何为 Three.js 做的坐标修改（§41）
 * =============================================================================
 *
 * 输入输出全部是 RobotForge 规范量。前端要显示时，
 * 唯一的转换点是 `frontend/src/viewer/coordinateAdapter.ts`。
 *
 * @module mini_arm/kinematics
 */

// =============================================================================
// 几何常量（m）—— 与 fk.py 逐值相同，由 tools/export_kinematics_fixture.py 比对
// =============================================================================

/** base → base_yaw 的高度 */
export const BASE_HEIGHT = 0.084;
/** base_yaw → shoulder 的高度 */
export const SHOULDER_OFFSET = 0.052;
/** shoulder → elbow（上臂长） */
export const L1 = 0.103;
/** elbow → ee_link */
export const L2 = 0.065;
/** ee_link → tcp */
export const L_TOOL = 0.032;

/**
 * 平面 2R 的**有效前臂长度**。
 *
 * `L2` 与 `L_TOOL` 刚性固连且同轴，故可合并为一个等效连杆 ——
 * 这也是本机构能写成 2R 的原因。
 */
export const L2_EFF = L2 + L_TOOL; // 0.097

/** 肩偏移与底座高度之和 = 平面 2R 解里要减掉的 Z 基准 */
export const Z_BASE = BASE_HEIGHT + SHOULDER_OFFSET; // 0.136

/** 关节顺序（= `RobotModel.mobile_joint_ids()` 的期望值 = MuJoCo qpos 顺序） */
export const JOINT_ORDER = Object.freeze(["base_yaw", "shoulder", "elbow"]);

// =============================================================================
// 关节限位（rad）—— 与 model/mini_arm.xml 的 range 一致，由包内测试断言
// =============================================================================
//
// ⚠️ 逆解必须在限位内求解，否则 solver 给出的角在 MuJoCo 里会被截断，
//    而截断后的实际位姿与目标不符 —— 表现为"Sim2Sim 对不上但 FK/IK 测试全绿"。
//
// 前端预演的限幅**不**用这三个常量，而是从 `RobotModel.joints[].limits` 读
// （见 `frontend/src/sim/predictor.ts` 的说明：限位是模型声明的事实，
// 不该在前端硬编码成第二份真值）。此处保留它们是为了让 IK 的
// `preferLimits` 语义与 Python 侧完全一致。

/** 底座偏航限位 */
export const YAW_LIMIT = Math.PI;
/** 肩限位 */
export const SHOULDER_LIMIT = Math.PI / 2;
/** 肘限位 */
export const ELBOW_LIMIT = (3 * Math.PI) / 4;

/** 目标是否超出可达半径的判定容差（m） */
export const TOL_REACH = 1e-9;

/**
 * **奇异位形**的判定容差（rad）。
 *
 * 语义：`|θ2| < TOL_SINGULAR` 或 `|π − |θ2|| < TOL_SINGULAR` 时，
 * "肘上/肘下"两种构型**数学上重合**，"分支"这个概念不再有意义 ⇒
 * 解被标注 `degenerate`，`solveIkAll` 也会把两个分支去重成一个。
 *
 * ## 这个值是怎么定出来的（不是拍脑袋）
 *
 * 在 mini_arm 的工作空间上扫描 `|θ2|` 的分布（约 10⁶ 个位形），
 * 结果是**双峰且中间完全空白**：
 *
 * ```text
 * == 0 的取值数: 1                              （完全伸展，唯一值 0.0）
 * (0, 1°) 之间的取值: 1.479e-06°, 2.561e-06°     （只有浮点噪声）
 * 最小非零真值:      1.0°                        （关节角网格的下一个刻度）
 * ```
 *
 * ⇒ `(2.6e-6°, 1.0°)` 是一段**没有任何物理意义**的空白带。
 *   1e-6 rad ≈ 5.7e-5° 位于空白带中央，两侧各留约 20 倍余量。
 *
 * 这是"用**观测到的**自然间隔定容差"，而不是"用 1e-12 这类漂亮数字"。
 */
export const TOL_SINGULAR = 1e-6;

// =============================================================================
// 基础几何原语（对应 backend/model/types.py 的 Vector3 / Quaternion / Transform）
// =============================================================================
//
// 这些是**契约对象的最小 JS 镜像**，只实现运动学需要的部分。
// 不引入 three.js —— 本文件必须能被 Node 直接跑（交叉验证脚本要用）。

/** @typedef {readonly [number, number, number]} Vec3 */
/** @typedef {readonly [number, number, number, number]} Quat  `[x,y,z,w]` */
/** @typedef {{ position: Vec3, orientation: Quat }} Pose */

/** @returns {Vec3} */
export function vec3(x, y, z) {
  return Object.freeze([x, y, z]);
}

/** 单位四元数 `[0,0,0,1]`（P0 验收项之一）。 @returns {Quat} */
export function quatIdentity() {
  return Object.freeze([0, 0, 0, 1]);
}

/** 向量加。 @returns {Vec3} */
function vAdd(a, b) {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

/** 向量缩放。 @returns {Vec3} */
function vScale(a, s) {
  return [a[0] * s, a[1] * s, a[2] * s];
}

/** 叉积（右手系判据 `X × Y = Z` 就用它验证）。 @returns {Vec3} */
function vCross(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

/** 模长。 */
function vNorm(a) {
  return Math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

/**
 * 归一化。零向量 ⇒ 抛错。
 *
 * 零向量**不能**归一化，而"悄悄返回零向量"的后果是关节轴变成 `[0,0,0]`
 * ⇒ FK 里绕零轴旋转 = 无旋转 ⇒ IK 永远解不到目标，
 * 而错误信息会是"IK 不收敛"。与 Python 侧 `Vector3.normalized` 同语义。
 *
 * @returns {Vec3}
 */
function vNormalized(a, where) {
  const n = vNorm(a);
  if (n < 1e-12) {
    throw new Error(`${where} 是零向量，无法归一化（关节轴不能为零向量）`);
  }
  return [a[0] / n, a[1] / n, a[2] / n];
}

/**
 * 绕**已归一化**的 axis 旋转 angle（右手定则），返回四元数 `[x,y,z,w]`。
 *
 * 轴未归一化时自动归一化 —— 与 Python 的 `Quaternion.from_axis_angle` 一致
 * （那边刻意宽松：调用方常有归一化轴，重复归一化只要几个 flops，
 * 但若这里要求严格，就会鼓励调用方在别处手工归一化，那更容易漏）。
 *
 * @param {Vec3} axis
 * @param {number} angleRad
 * @returns {Quat}
 */
export function quatFromAxisAngle(axis, angleRad) {
  if (!Number.isFinite(angleRad)) {
    throw new Error(`Quaternion.from_axis_angle(angle) 必须是有限数（当前 = ${angleRad}）`);
  }
  const a = vNormalized(axis, "Quaternion.from_axis_angle(axis)");
  const half = angleRad / 2;
  const s = Math.sin(half);
  return [a[0] * s, a[1] * s, a[2] * s, Math.cos(half)];
}

/**
 * Hamilton 积。`mul(q1, q2)` 表示"先 q2 后 q1"的复合旋转。
 *
 * ⚠️ 顺序不可反 —— 见 `forwardKinematics` 里 `q_yaw ∘ q_pitch` 的说明。
 *
 * @param {Quat} a
 * @param {Quat} b
 * @returns {Quat}
 */
export function quatMul(a, b) {
  const [ax, ay, az, aw] = a;
  const [bx, by, bz, bw] = b;
  return [
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
    aw * bw - ax * bx - ay * by - az * bz,
  ];
}

/** 四元数模长。 */
function quatNorm(q) {
  return Math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

/**
 * 四元数归一化。零四元数 ⇒ 抛错。 @returns {Quat}
 */
export function quatNormalized(q, where) {
  const n = quatNorm(q);
  if (n < 1e-12) {
    throw new Error(`${where} 是零四元数，无法归一化`);
  }
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}

/**
 * 用四元数旋转向量（`q v q*`，展开成无临时四元数的形式）。
 *
 * 展开形式比 `q*v*q⁻¹` 少两次四元数乘法，且不引入中间归一化误差。
 * 与 Python 的 `Quaternion.rotate` 逐运算对应。
 *
 * @param {Quat} q
 * @param {Vec3} v
 * @returns {Vec3}
 */
export function quatRotate(q, v) {
  const qv = [q[0], q[1], q[2]];
  const t = vScale(vCross(qv, v), 2.0);
  return vAdd(vAdd(v, vScale(t, q[3])), vCross(qv, t));
}

/**
 * `a ∘ b`：先在 b 的坐标系里定位，再经 a 变换到 a 的父系。
 *
 * 这是 FK 链式相乘的基本操作：`T_world_child = T_world_parent ∘ T_parent_child`。
 *
 * @param {Pose} a
 * @param {Pose} b
 * @returns {Pose}
 */
export function poseCompose(a, b) {
  return {
    position: vAdd(a.position, quatRotate(a.orientation, b.position)),
    orientation: quatMul(a.orientation, b.orientation),
  };
}

/** 恒等位姿。 @returns {Pose} */
export function poseIdentity() {
  return { position: [0, 0, 0], orientation: quatIdentity() };
}

// =============================================================================
// 从模型取轴 / 位置（**不硬编码**）
// =============================================================================
//
// ⚠️ 这是本文件与"把公式全写死"的关键区别：几何**长度**必须与 MJCF 同步
//    （由测试保证），但**轴的方向**直接从传入的 model 读 —— 因为轴错是
//    最难发现的错误（不会让结果变 NaN，只会让机械臂朝错误方向运动）。

/**
 * 从 RobotModel DTO 上取某个关节的轴。
 *
 * @param {any} model  `RobotModel.to_dict()` 的形状
 * @param {string} jointId
 * @returns {Vec3}
 */
function axisOf(model, jointId) {
  const joint = findJoint(model, jointId);
  return joint.axis;
}

/** 按 id 找关节；找不到抛错（**不**静默回退到某个默认轴）。 */
function findJoint(model, jointId) {
  const joints = model?.joints;
  if (!Array.isArray(joints)) {
    throw new Error("model.joints 必须是数组（传入的是 RobotModel.to_dict() 吗？）");
  }
  const joint = joints.find((j) => j.id === jointId);
  if (!joint) {
    throw new Error(`模型里没有关节 ${jointId}；现有 = ${joints.map((j) => j.id).join(", ")}`);
  }
  return joint;
}

/**
 * tcp site 相对其父 link 的偏移（从模型读，不硬编码）。
 *
 * @param {any} model
 * @param {string} [siteId]
 * @returns {Vec3}
 */
export function tcpOffset(model, siteId = "tcp") {
  const sites = model?.sites;
  if (!Array.isArray(sites)) {
    throw new Error("model.sites 必须是数组");
  }
  const site = sites.find((s) => s.id === siteId);
  if (!site) {
    throw new Error(`模型里没有 site ${siteId}`);
  }
  return site.transform.position;
}

// =============================================================================
// 解析式 **正运动学** —— 移植自 packages/mini_arm/kinematics/fk.py
// =============================================================================

/**
 * 正解：关节角 → **TCP** 位姿（世界系 / base 系，二者同向）。
 *
 * ## 输入
 *
 * `jointPositions`：`{joint_id: value}`，单位由关节类型决定
 * （revolute → rad，prismatic → m）。
 *
 * **未提供的关节按 0 处理** —— 这是刻意的：0 是零位，
 * 而"缺一个关节就报错"会让 UI 在初始化时（还没拿到全部状态）无法渲染。
 * （与 Core 的同名契约一致，由测试钉住。）
 *
 * ## 输出
 *
 * `{position: [x,y,z], orientation: [x,y,z,w]}`：
 * position 为 TCP 在 base 系中的位置（m），orientation 为 ee_link 的姿态。
 *
 * ⚠️ **orientation 不含工具偏移的旋转**：tcp 相对 ee_link 只有平移
 * （`L_TOOL` 沿 +X），没有旋转。因此 ee_link 的姿态 = TCP 的姿态。
 * 如果将来 tcp 带旋转，此处必须补上 —— 包内测试会在那时失败，
 * 从而提醒改这里。
 *
 * ## 实现方式（为什么分成两段）
 *
 * ```text
 * ① 平面 2R（肩 + 肘）在 XZ 平面内算出 r, z
 * ② 底座偏航 φ 把 (r, 0, z) 绕 Z 旋到 (r·cosφ, r·sinφ, z)
 * ```
 *
 * 这比"逐级乘 4 个矩阵"更短，也更容易核对 —— 而它的正确性由
 * 与 Core 通用引擎的逐点比对来互证。
 *
 * @param {any} model  RobotModel DTO（用于取轴；几何用本文件的常量）
 * @param {Record<string, number>} jointPositions
 * @returns {Pose}
 */
export function forwardKinematics(model, jointPositions) {
  const angleYaw = jointPositions["base_yaw"] ?? 0.0;
  const theta1 = jointPositions["shoulder"] ?? 0.0;
  const theta2 = jointPositions["elbow"] ?? 0.0;

  // ---- ① 平面 2R：肩 θ1、肘 θ2（都绕 +Y）----
  // 上臂：从 shoulder 原点沿 +X 走 L1，经 Ry(θ1) 后
  //   x += L1·cos(θ1),  z -= L1·sin(θ1)
  // 前臂：从肘沿 +X 走 (L2 + L_TOOL)，经 Ry(θ1+θ2) 后同理
  const rArm = L1 * Math.cos(theta1) + L2_EFF * Math.cos(theta1 + theta2);
  const zArm = -(L1 * Math.sin(theta1) + L2_EFF * Math.sin(theta1 + theta2));

  // ---- 抬高到 base 系：加上底座高度与肩偏移 ----
  // shoulder 原点在 base_yaw 旋转坐标系里位于 (0,0,SHOULDER_OFFSET)，
  // 而绕 +Z 旋转不改变 Z 分量 ⇒ z 可以直接相加。
  const r = rArm;
  const z = BASE_HEIGHT + SHOULDER_OFFSET + zArm;

  // ---- ② 底座偏航：把平面半径 r 绕 Z 转到 (x, y) ----
  const x = r * Math.cos(angleYaw);
  const y = r * Math.sin(angleYaw);

  // ---- 姿态 ----
  // ee_link 的朝向 = Ry(θ1) · Ry(θ2) = Ry(θ1+θ2)（同轴旋转可加）
  // 再经底座 Rz(φ)：q = Rz(φ) ∘ Ry(θ1+θ2)（先俯仰，再随底座偏航）
  const qRoll = quatIdentity(); // 本机构无 roll 自由度，显式写出以对齐 RPY 语义
  const qPitch = quatFromAxisAngle(axisOf(model, "shoulder"), theta1 + theta2);
  const qYaw = quatFromAxisAngle(axisOf(model, "base_yaw"), angleYaw);
  const orientation = quatNormalized(quatMul(quatMul(qYaw, qPitch), qRoll), "FK orientation");

  return { position: [x, y, z], orientation };
}

// =============================================================================
// 解析式 **逆运动学** —— 移植自 packages/mini_arm/kinematics/ik.py
// =============================================================================

/**
 * 逆解失败的基类。**不用返回值表示失败** —— 返回 null 会被漏检。
 */
export class IkError extends Error {
  constructor(message) {
    super(message);
    this.name = "IkError";
  }
}

/** 目标在可达工作空间之外。 */
export class UnreachableError extends IkError {
  /**
   * @param {string} message
   * @param {{distance: number, reachMin: number, reachMax: number}} info
   */
  constructor(message, info) {
    super(message);
    this.name = "UnreachableError";
    this.distance = info.distance;
    this.reachMin = info.reachMin;
    this.reachMax = info.reachMax;
  }
}

/** 解存在但超出关节限位（且未允许 clamp）。 */
export class LimitViolationError extends IkError {
  constructor(message) {
    super(message);
    this.name = "LimitViolationError";
  }
}

/**
 * 可达半径区间（**水平半径 r**，不含底座高度）。
 *
 * ```text
 * r_max = L1 + L2_EFF          完全伸展
 * r_min = |L1 − L2_EFF|        完全折叠
 * ```
 *
 * 注意这是"平面半径"，完整工作空间是一个**球壳的一部分**
 * （加上 Z 方向后是绕 Z 轴的旋转体）。UI 显示工作空间时应该显示这个旋转体，
 * 而不是一个球 —— 显示成球会让用户以为 z 可以任意取。
 *
 * @returns {[number, number]}  `[rMin, rMax]`
 */
export function reachLimits() {
  return [Math.abs(L1 - L2_EFF), L1 + L2_EFF];
}

/**
 * 逆解：目标 TCP 位姿 → 关节角（单分支）。
 *
 * ## 参数
 *
 * - `branch`：`'elbow_up'` / `'elbow_down'`。机构通常固定一个装配构型，
 *   因此默认值比"随机选一个"更有意义。
 * - `preferLimits`：解出后按限位筛选。若两解都超限 ⇒ 抛 `LimitViolationError`
 *   （除非 `clamp === true`）。
 * - `clamp`：允许把超限解夹到限位。**默认 false** ——
 *   **禁止**把不可达目标悄悄夹紧到最近可达点：那会让用户以为"IK 解到了"，
 *   而实际末端停在别处。
 *
 * ## 返回
 *
 * 解对象，含关节角、分支、位置误差、触限信息。
 *
 * ## 与 FK 的对称性
 *
 * 本函数只求出**关节角**（Joint Space），**不**做任何执行器/舵机映射。
 * `Joint → Actuator Mapping → Servo` 是未来 `Actuator` 层的职责。
 * 把舵机角混进 IK 会让换舵机就要改运动学。
 *
 * @param {any} model  RobotModel DTO（用于取轴，供回代 FK 自检）
 * @param {Pose} target
 * @param {{branch?: string, preferLimits?: boolean, clamp?: boolean, tol?: number}} [options]
 * @returns {{jointPositions: Record<string, number>, branch: string, positionError: number, clamped: boolean, clampedJoints: string[], orientationError: number}}
 */
export function solveIk(model, target, options = {}) {
  const {
    branch = "elbow_up",
    preferLimits = true,
    clamp = false,
    tol = 1e-6,
  } = options;

  // ---- ① 底座偏航：由 (x, y) 唯一确定 ----
  // atan2 而非 atan(y/x)：atan 丢失象限信息，且 x=0 时除零。
  // 这是 IK 里最常见的一个 bug —— 它只在第二/三象限暴露。
  const [x, y, z] = target.position;
  const angleYaw = Math.atan2(y, x);

  // ---- ② 平面半径与高度 ----
  const r = Math.hypot(x, y);
  const zArm = z - Z_BASE; // 减去底座与肩偏移，回到"肩为原点"的平面坐标系

  // 平面内到肘的距离（2D 距离，含 zArm）
  const d = Math.hypot(r, zArm);
  const [rMin, rMax] = reachLimits();

  // 容差放宽一点：目标若正好在边界上，浮点误差会把它推到外面
  if (d > rMax + tol) {
    throw new UnreachableError(
      `目标距离肩关节 ${d.toFixed(6)} m 超出可达范围 ` +
        `[${rMin.toFixed(6)}, ${rMax.toFixed(6)}] m；` +
        `目标位置 = (${x.toFixed(6)}, ${y.toFixed(6)}, ${z.toFixed(6)})，` +
        `底座平面半径 r = ${r.toFixed(6)} m`,
      { distance: d, reachMin: rMin, reachMax: rMax }
    );
  }
  if (d < rMin - tol) {
    throw new UnreachableError(
      `目标距离肩关节 ${d.toFixed(6)} m 小于最小可达半径 ${rMin.toFixed(6)} m` +
        `（肘部完全折叠的死区球内）；无法到达`,
      { distance: d, reachMin: rMin, reachMax: rMax }
    );
  }

  // ---- ③ 平面 2R：余弦定理求肘角 ----
  // cos(θ2) 由两连杆三角形确定： d² = L1² + L2e² − 2·L1·L2e·cos(π − θ2)
  // 展开后：cos(θ2) = (d² − L1² − L2e²) / (2·L1·L2e)
  let cosTheta2 = (d * d - L1 * L1 - L2_EFF * L2_EFF) / (2.0 * L1 * L2_EFF);
  // 夹紧到 [-1, 1]：边界目标的浮点误差会让它略超出，而 acos 会得到 NaN。
  // 这个夹紧**不改变解的正确性**（超出部分正是上面已判定的不可达），
  // 因此不需要报告为 clamped。
  cosTheta2 = Math.max(-1.0, Math.min(1.0, cosTheta2));
  const absTheta2 = Math.acos(cosTheta2);

  // ---- ④ 肩角：目标方向角 + 三角形内角 ----
  // ⚠️ 注意负号：绕 +Y 转 θ>0 时 +X 轴指向 -Z，故平面角 = -atan2(zArm, r)
  const phiTarget = Math.atan2(-zArm, r);
  // 余弦定理求 L1 与 d 的夹角
  let cosAlpha = (L1 * L1 + d * d - L2_EFF * L2_EFF) / (2.0 * L1 * d);
  cosAlpha = Math.max(-1.0, Math.min(1.0, cosAlpha));
  const alpha = Math.acos(cosAlpha);

  // ---- ★ 分支的判据是"肘在肩-手连线的哪一侧"，不是 θ2 的符号 ----
  //
  // "肘在上方"是一个**几何**概念：肘相对**肩→手连线**的侧向偏移。
  // 而 θ2 的符号只说明"前臂相对上臂折向哪边"，二者的对应关系**取决于
  // L1 与 L2eff 谁更长** —— 本机构 L2eff(0.097) < L1(0.103) 时
  // θ2<0 恰好对应肘**向下**，与直觉相反。
  //
  // 如果按 θ2 的符号命名分支，那么"elbow_up"这个**接口语义**就会随
  // 连杆长度漂移 —— 改一根连杆的长度，前端选中的分支就悄悄反了，
  // 而表现只是"机械臂以镜像姿态去够同一个点"，极难发现。
  //
  // ⇒ 所以：**先算出两个候选 θ2，再用几何量（肘的侧偏）判定谁是 up**。

  /**
   * 由 θ2 与目标位置唯一确定 θ1。
   *
   * ## ★ 配对规则的符号（Python 侧改过一次，是最难发现的一个 bug）
   *
   * ```text
   * θ2 > 0  ⇒  θ1 = phiTarget − alpha
   * θ2 < 0  ⇒  θ1 = phiTarget + alpha
   * ```
   *
   * 第一版写成了 `t1 = phi + alpha * (θ2 > 0 ? 1 : -1)` —— **正好写反**。
   * 后果不是崩溃，而是**灾难性的静默错误**：两个分支算出的 θ1 都把 TCP
   * 放到**完全错误的位置**（误差 ~0.15 m，几乎等于整条臂长），
   * 但这个错误位置**仍然在关节限位内**，所以没被限位检查拦下；
   * 最终由"回代 FK 自检"兜住 ⇒ 报"IK 失败"，而那是个**假失败**。
   *
   * ⇒ 这就是为什么 `solveIk` 必须**回代 FK 自检并报告 positionError**。
   */
  const theta1For = (theta2Candidate) =>
    theta2Candidate < 0 ? phiTarget + alpha : phiTarget - alpha;

  /**
   * 给定 θ2，算出该分支下肘相对'肩→手连线'的**有符号侧偏**。
   *
   * 正值 = 肘在连线的 +Z 侧（即"上方"，因为 +Z 是上）。
   * 判据用二维叉积：`handDir × elbowVec` 的 z 分量符号。
   */
  const sideOffset = (theta2Candidate) => {
    const t1 = theta1For(theta2Candidate);
    // 肘相对肩的位置（平面）
    const ex = L1 * Math.cos(t1);
    const ez = -L1 * Math.sin(t1);
    // 肩→手 的方向（= 目标方向）
    const hx = Math.cos(phiTarget);
    const hz = -Math.sin(phiTarget);
    // 二维叉积 z 分量：handDir × elbowVec 的符号 = 肘在连线哪一侧
    return hx * ez - hz * ex;
  };

  const candidatesT2 = [absTheta2, -absTheta2];
  // 分别算出两候选的侧偏，侧偏大的那个是 elbow_up。
  //
  // ⚠️ 必须复现 Python `max(offsets, key=...)` 的**并列取首个**语义：
  //    在完全伸展（θ2 = 0）时两候选侧偏可能相等，此时 Python 返回
  //    先出现的 `absTheta2`（+0）。用 `>` 严格比较（而不是 `>=`）即可 ——
  //    并列时保留先到的那个。写成 `>=` 会取到后一个，与 Python 分歧。
  const offsets = candidatesT2.map((t2) => ({ t2, off: sideOffset(t2) }));
  let best = offsets[0];
  let worst = offsets[0];
  for (const entry of offsets) {
    if (entry.off > best.off) best = entry;
    if (entry.off < worst.off) worst = entry;
  }
  const elbowUpT2 = best.t2;
  const elbowDownT2 = worst.t2;

  // ★ 退化判据必须看**解本身**，而不是侧偏之差（Python 侧改过一次）
  //
  // 第一版用 `|offsetUp − offsetDown| < eps` 判定退化，结果是错的：
  // 在完全伸展位形（θ2 = 0）下，两个候选 θ2 都是 0，**解完全相同**，
  // 但 `sideOffset` 对 ±0 会算出符号相反的侧偏（+0 与 -0 进入
  // `theta2 < 0` 判断时落到不同分支），于是 offsets 差得很大 ⇒
  // 判为非退化 ⇒ 给"同一个解"贴上 up / down 两个互相矛盾的标签。
  //
  // ⇒ 正确判据：**两个候选 θ2 是否实质相同**（= 肘几乎伸直或几乎对折）。
  const degenerate =
    Math.abs(absTheta2) < TOL_SINGULAR || Math.abs(absTheta2 - Math.PI) < TOL_SINGULAR;

  let theta2;
  if (branch === "elbow_up") {
    theta2 = elbowUpT2;
  } else if (branch === "elbow_down") {
    theta2 = elbowDownT2;
  } else {
    throw new IkError(`未知的 branch = ${JSON.stringify(branch)}；允许 'elbow_up' / 'elbow_down'`);
  }

  const theta1 = theta1For(theta2);

  let raw = { base_yaw: angleYaw, shoulder: theta1, elbow: theta2 };

  // 记录实际分支（退化时诚实标注，而不是假装两个分支分开了）
  let actualBranch = branch;
  if (degenerate) {
    actualBranch = `${branch} [degenerate]`;
  }

  // ---- ⑤ 限位处理 ----
  const limits = {
    base_yaw: YAW_LIMIT,
    shoulder: SHOULDER_LIMIT,
    elbow: ELBOW_LIMIT,
  };
  // base_yaw 的 atan2 结果天然落在 (-π, π]，与 YAW_LIMIT=π 一致
  const clampedJoints = Object.keys(raw).filter(
    (k) => Math.abs(raw[k]) > limits[k] + 1e-9
  );

  if (clampedJoints.length > 0 && preferLimits && !clamp) {
    const detail = clampedJoints
      .map((k) => `${k}=${raw[k] >= 0 ? "+" : ""}${raw[k].toFixed(6)} (限位 ±${limits[k].toFixed(6)})`)
      .join(", ");
    throw new LimitViolationError(
      `目标 ${branch} 解超出关节限位：${detail}。` +
        `该目标位置可达但**姿态不可达**（受关节限位约束）。` +
        `如需强制夹紧请传 clamp=true（结果会带 clamped 标记）`
    );
  }

  if (clamp) {
    const clamped = {};
    for (const k of Object.keys(raw)) {
      clamped[k] = Math.max(-limits[k], Math.min(limits[k], raw[k]));
    }
    raw = clamped;
  }

  // ---- ⑥ 自检：把解回代 FK，报告实际误差 ----
  const check = forwardKinematics(model, raw);
  const posErr = vNorm([
    check.position[0] - target.position[0],
    check.position[1] - target.position[1],
    check.position[2] - target.position[2],
  ]);
  const oriErr = orientationError(check.orientation, target.orientation);

  return {
    jointPositions: raw,
    branch: actualBranch,
    positionError: posErr,
    clamped: clamp ? clampedJoints.length > 0 : false,
    clampedJoints: clamp ? clampedJoints : [],
    orientationError: oriErr,
  };
}

/**
 * 返回**两个**可行解（elbow_up / elbow_down），按误差升序。
 *
 * 超出限位或不可达的分支被**跳过**（记入 warnings）。
 * 若两个分支都不可行 ⇒ 抛 `IkError`。
 *
 * ## 为什么要返回"全部解"而不只是最优解
 *
 * 因为"最优"依赖于选择准则，而准则属于**应用层**：
 * - 靠近奇异位形时该选"离当前位形最近"的解（避免跳变）；
 * - 避障时该选"不撞"的解；
 * - 演示时该选"看起来最自然"的解。
 *
 * 这些都要求上层能看到候选集。IK 层擅自选一个，就会让上层无法实现这些准则。
 *
 * @param {any} model
 * @param {Pose} target
 * @param {{clamp?: boolean}} [options]
 * @returns {Array<object>}
 */
export function solveIkAll(model, target, options = {}) {
  const { clamp = false } = options;

  /** @type {Array<object>} */
  const candidates = [];
  /** @type {string[]} */
  const warnings = [];
  /** @type {Set<string>} */
  const seen = new Set();

  for (const br of ["elbow_up", "elbow_down"]) {
    let sol;
    try {
      sol = solveIk(model, target, { branch: br, clamp });
    } catch (exc) {
      if (exc instanceof UnreachableError) {
        // 不可达 ⇒ 两个分支都不可达，直接抛出（不是"换一个分支试试"）
        throw exc;
      }
      if (exc instanceof LimitViolationError) {
        warnings.push(`${br}: ${exc.message}`);
        continue;
      }
      throw exc;
    }
    // 只保留误差在容差内的（防止"夹紧后误差巨大"仍被当作可行解）
    if (sol.positionError > 1e-3) {
      warnings.push(`${br}: 位置误差 ${sol.positionError.toFixed(6)} m 过大`);
      continue;
    }
    // ★ 去重：奇异位形（完全伸展 / 完全折叠）下两分支解**相同**，
    //   返回两份一样的解会让 UI 显示"2 个解"而用户看到两个一模一样的角度。
    //   按关节角量化后去重，保留先出现的（elbow_up）。
    const key = JOINT_ORDER.map((id) => sol.jointPositions[id].toFixed(12)).join("|");
    if (seen.has(key)) {
      warnings.push(`${br}: 与另一分支同解（奇异位形）⇒ 去重`);
      continue;
    }
    seen.add(key);
    candidates.push(sol);
  }

  if (candidates.length === 0) {
    throw new IkError("两个分支都不可行：\n  " + warnings.join("\n  "));
  }

  candidates.sort((a, b) => a.positionError - b.positionError);
  return candidates;
}

/**
 * 两个姿态之间的角度差（rad）。
 *
 * 用 `|dot|` 再 acos：四元数 q 与 −q 表示同一旋转，故取绝对值。
 * 不取绝对值会让"同一个姿态"算出 180° 的误差 —— 一个经典陷阱。
 */
export function orientationError(a, b) {
  let dot = Math.abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
  dot = Math.max(-1.0, Math.min(1.0, dot));
  return 2.0 * Math.acos(dot);
}
