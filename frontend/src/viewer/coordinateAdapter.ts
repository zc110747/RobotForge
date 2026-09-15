/**
 * =============================================================================
 * coordinateAdapter.ts —— Three.js 坐标适配层
 * -----------------------------------------------------------------------------
 * 本文件是 RobotForge 里**唯一**允许做 Three.js 坐标转换的地方。
 * 契约来源：`docs/coordinate-system.md` §4.2「Three.js 的 Z-up 处理（关键决策）」
 *
 * ```text
 * RobotModel（Z-up，永不改）
 *       ↓
 * Three.js Renderer Adapter        ← 本文件
 *       ↓  scene.rotation.x = -Math.PI / 2   （整场景旋转，而非逐 body 旋转）
 * Three.js（Y-up 表面）
 * ```
 *
 * ## 三条不可违反的规则
 *
 * 1. **RobotModel 的语义永不修改。** 本文件只做"读"：读 position、
 *    读 orientation、读 axis，然后转换。绝不回写、绝不改字段含义。
 * 2. **FK / IK 里不得为了 Three.js 改坐标**（提示词 §41）。
 *    FK/IK 在 `packages/` 与 `backend/` 里，根本不该 import 任何 three 相关物。
 * 3. **整场景旋转，不逐节点旋转。** 理由见 §4.2：
 *    一次变换不可能漏；逐节点漏一个的表现是"看起来还行"。
 *
 * ## 为什么整场景旋转是对的（而不是"偷懒"）
 *
 * 整场景旋转保证了：Three.js 里读出的局部关节角与 RobotModel **完全同源**。
 * 于是"同一状态在同一语义下"这个断言才有意义 —— 也就是
 * `semantics.check.ts` 里做的 `RobotModel ↔ ViewModel` 逐字段比对。
 * 若逐节点转换，两边各有一套矩阵乘法，比对就退化成"数值碰巧接近"。
 *
 * ## 代价（明确接受）
 *
 * Three.js 世界坐标 = `(x, z, -y)`，与 RobotForge 的 `(x, y, z)` 不同。
 * 这是**允许且预期**的：它属于 "UI Boundary"。
 *
 * ⚠️ 因此任何"把 Three.js 世界坐标当成机器人坐标"的代码都是 bug。
 *    需要机器人坐标时，用本文件的 `fromThreeWorld()` 换回来。
 */

import { Quaternion, Vector3 } from "three";

/** RobotForge 采用的是 Z-up 右手系（+X 前 / +Y 左 / +Z 上）。 */
export const ROBOT_UP_AXIS = "z" as const;

/**
 * 整场景旋转角：把 Z-up 的机器人放进 Y-up 的 Three.js 世界。
 *
 * `rotation.x = -PI/2` 的效果：机器人的 +Z 被转到 Three.js 的 +Y，
 * 机器人的 +Y 被转到 Three.js 的 -Z。
 *
 * 推导（不要凭记忆，这里写清）：
 * 绕 X 轴旋转 θ 的矩阵是
 *   [1    0       0   ]
 *   [0  cosθ   -sinθ  ]
 *   [0  sinθ    cosθ  ]
 * 取 θ = -π/2 ⇒ cos = 0, sin = -1，得
 *   [1  0   0]
 *   [0  0   1]
 *   [0 -1   0]
 * 于是 (x,y,z) → (x, z, -y)。
 * 验证：+Z up 轴 (0,0,1) → (0, 1, 0) = Three.js 的 +Y ✅
 *       +Y 左轴 (0,1,0) → (0, 0, -1) = Three.js 里朝向屏幕外 ✅
 */
export const SCENE_ROTATION_X = -Math.PI / 2;

/**
 * 给 Three.js 场景对象施加整场景旋转。
 *
 * ★ 这是本模块**唯一**推荐被渲染器调用的入口。
 * 直接写 `scene.rotation.x = -Math.PI/2` 也等价，但那样这个常量就有两份，
 * 而两份常量的漂移是静默的。
 *
 * 参数用结构化类型而不是 `THREE.Scene`：
 * 让本函数可被单元测试用最小桩对象调用，不必起 WebGL 上下文。
 */
export function applySceneRotation(scene: { rotation: { x: number } }): void {
  scene.rotation.x = SCENE_ROTATION_X;
}

// ---------------------------------------------------------------------------
// 位置转换
// ---------------------------------------------------------------------------

/**
 * RobotForge 坐标 → Three.js **世界**坐标。
 *
 *   (x, y, z)_robot  →  (x, z, -y)_three
 *
 * ⚠️ 只在"需要手工摆放一个不在场景树里的对象"时使用。
 *    场景树里的对象**不需要**调用它 —— 整场景旋转已经代劳了。
 *    误用会让对象被转两次（表现：某个标记飞到奇怪的地方）。
 */
export function toThreeWorld(v: {
  x: number;
  y: number;
  z: number;
}): { x: number; y: number; z: number } {
  return { x: v.x, y: v.z, z: -v.y };
}

/** Three.js 世界坐标 → RobotForge 坐标（`toThreeWorld` 的逆）。 */
export function fromThreeWorld(v: {
  x: number;
  y: number;
  z: number;
}): { x: number; y: number; z: number } {
  // 逆变换：y_robot = -z_three, z_robot = y_three
  return { x: v.x, y: -v.z, z: v.y };
}

/** 便捷版：RobotForge `[x,y,z]` 数组 → Three.js `Vector3`（世界）。 */
export function positionToThree(p: readonly [number, number, number]): Vector3 {
  return new Vector3(p[0], p[2], -p[1]);
}

// ---------------------------------------------------------------------------
// 姿态转换
// ---------------------------------------------------------------------------

/**
 * RobotForge 四元数 `[x,y,z,w]` → Three.js `Quaternion`。
 *
 * ★ **恒等映射**：Three.js 的 `Quaternion` 内部就是 `x,y,z,w` 顺序
 *   （见 `docs/coordinate-system.md` §5.2 的表格），与 RobotForge 天然一致。
 *   所以这里不需要重排分量 —— 只需要换容器。
 *
 * ⚠️ 关键：整场景旋转已经承担了 Z-up→Y-up 的变换，
 *    所以**关节的局部四元数不需要再转一次**。
 *    这里若再乘一个 `-π/2` 旋转，关节就会转两次。
 *    这是本文件最容易犯的错，因此语义测试专门钉住"恒等"。
 */
export function quaternionToThree(
  q: readonly [number, number, number, number]
): Quaternion {
  return new Quaternion(q[0], q[1], q[2], q[3]);
}

/** RobotForge 的 `Transform`（position + orientation）→ Three.js 局部变换。 */
export function transformToThree(t: {
  position: readonly [number, number, number];
  orientation: readonly [number, number, number, number];
}): { position: Vector3; quaternion: Quaternion } {
  // ★ 注意：这里是**局部**变换，用原始 position（不套 toThreeWorld）。
  // 因为父节点已经在整场景旋转的坐标系里，局部量保持 RobotForge 语义，
  // 由场景根的一次旋转统一带到 Three.js 世界。
  return {
    position: new Vector3(t.position[0], t.position[1], t.position[2]),
    quaternion: quaternionToThree(t.orientation),
  };
}

// ---------------------------------------------------------------------------
// 关节轴
// ---------------------------------------------------------------------------

/**
 * 关节轴向量 → Three.js 局部轴。
 *
 * ★ 返回**局部**轴（不做 `toThreeWorld`）。理由同上：
 *   轴向量表达的是"在父坐标系里绕哪个方向转"，
 *   父坐标系本身已经由整场景旋转带到了 Three.js。
 *   对轴再做一次世界转换会让旋转方向错 90°，而视觉表现是
 *   "关节在转，但转的方向诡异" —— 典型的不易察觉错误。
 */
export function axisToThree(
  axis: readonly [number, number, number]
): Vector3 {
  return new Vector3(axis[0], axis[1], axis[2]).normalize();
}

// ---------------------------------------------------------------------------
// 公理（供测试直接断言，别在测试里重写一遍数学）
// ---------------------------------------------------------------------------

/**
 * 本适配层的三条公理。测试直接 import 这些值，
 * **不要**在测试文件里重新推导 —— 那样测的是测试自己写的数学，不是本模块。
 */
export const AXIOMS = {
  /** 场景旋转落在 Z-up → Y-up 上 */
  sceneRotationX: SCENE_ROTATION_X,
  /** 期望的世界坐标映射：RobotForge (x,y,z) → Three.js (x,z,-y) */
  upAxisMapsTo: { x: 0, y: 1, z: 0 },
  /** 四元数分量顺序（与 Three.js 一致，因此是恒等映射） */
  quaternionOrder: ["x", "y", "z", "w"] as const,
  /** 恒等四元数的字面值 */
  identityQuaternion: [0, 0, 0, 1] as const,
  /** Three.js 是 Y-up */
  threeUpAxis: "y" as const,
} as const;
