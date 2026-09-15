/**
 * =============================================================================
 * viewModel.ts —— RobotModel → 渲染视图模型（**纯函数，零 Three.js 依赖**）
 * -----------------------------------------------------------------------------
 * 本文件是 §59 验收里 "Joint hierarchy 正确 / Joint axis 正确 /
 * End Effector 正确 / Coordinate Frame 正确" 的**唯一实现处**。
 *
 * ## 为什么要把"推导"从渲染里抽出来
 *
 * 若直接在 React 组件里递归建树、在 JSX 里读 `joint.axis`，
 * 那么"层级对不对"就只能靠**看**。而"看起来对"是本项目反复吃过亏的判据
 * （见 skill `robotforge-phase-gate` 第 ④ 节的假检查清单）。
 *
 * 抽成纯函数后，`semantics.check.ts` 可以在 Node 里：
 *   ① 拿真实后端 JSON，跑本文件；
 *   ② 与 Python 侧**独立**推导的结果逐字段比对；
 *   ③ 注入一个错误轴/错误父子关系，断言**比对会失败**。
 *
 * 第 ③ 步是关键：没有它，这个测试可能是"永远绿"的。
 *
 * ## 零 Three.js 依赖是硬要求
 *
 * 本文件 `import` 里**不允许**出现 `three`。理由：
 * 语义一致性要与后端比对，而后端没有 three；若两边都经过 Three.js 的
 * 数学库，一致性就退化成"Three.js 自己和自己一致"，失去意义。
 *
 * ## ★ 渲染树是 link 与 joint **交替**的
 *
 * 这是本文件最容易搞错、且搞错后"看起来还行"的地方。真实拓扑是：
 *
 * ```text
 * world
 *  └─ base                      （link，根）
 *      └─ base_yaw              （joint，挂在 parent_link=base 上）
 *          └─ shoulder_link     （link，child_link=shoulder_link）
 *              └─ shoulder      （joint）
 *                  └─ upper_arm （link）
 *                      └─ elbow
 *                          └─ forearm_link
 *                              └─ ee_link_fixed
 *                                  └─ ee_link
 * ```
 *
 * 依据：
 *  - link 的**父节点** = 它的 `parent_joint`（根 link 为 `world`）
 *  - joint 的**父节点** = 它的 `parent_link`
 *
 * ⚠️ 常见错误是"把 link 直接挂到 link 下"（把 `shoulder_link`
 *    挂到 `base` 下面）。在只有一个转动关节的例子里这也能画出东西，
 *    但关节的原点变换就无处安放 ⇒ 上臂的位置会偏掉一个关节原点。
 */

import type {
  EndEffectorDTO,
  JointDTO,
  LinkDTO,
  RobotModelDTO,
  TransformDTO,
  Vec3,
} from "./model.ts";

// ---------------------------------------------------------------------------
// 节点类型
// ---------------------------------------------------------------------------

export type RenderNodeKind = "world" | "link" | "joint";

/** 一个渲染节点的**语义**描述（不含任何几何/材质决策）。 */
export interface RenderNode {
  /** 唯一 id。link 用 `link:<id>`，joint 用 `joint:<id>`，避免同名冲突。 */
  readonly key: string;
  /** 原始 id（不带前缀），便于与后端 JSON 比对。 */
  readonly id: string;
  readonly kind: RenderNodeKind;
  /** 父节点 key；world 为 `null` */
  readonly parentKey: string | null;
  /** 子节点 key（顺序 = 数据顺序） */
  readonly childKeys: readonly string[];
  /**
   * 本节点相对**父节点**的局部变换。
   * - world：恒等
   * - link：其父是 joint ⇒ 恒等（link 的位姿由父 joint 决定）
   * - joint：其 `origin`（相对 parent_link）
   */
  readonly localTransform: TransformDTO;
  /** 仅 link 有：显示几何（来自 `link.collision`，v0.1 用碰撞体显示）。 */
  readonly geometries: readonly GeometryRefLike[];
  /** 仅 joint 有：原始轴（未经坐标转换）。 */
  readonly axis: Vec3 | null;
  /** 仅 joint 有：是否可动（`fixed` 为 false）。 */
  readonly isMovable: boolean;
}

export interface GeometryRefLike {
  readonly type: string;
  readonly size: readonly number[];
  readonly transform: TransformDTO;
  readonly rgba: readonly number[];
  readonly asset: string | null;
}

/** 关节的语义描述（与 `RenderNode` 里 joint 节点互为视图，便于测试）。 */
export interface JointVisual {
  readonly id: string;
  readonly type: string;
  readonly parentLink: string;
  readonly childLink: string;
  readonly origin: TransformDTO;
  /** 旋转轴，**已单位化**，未做任何坐标转换 */
  readonly axis: Vec3;
  readonly limits: { min: number; max: number } | null;
  /** `fixed` 关节无自由度，不应画轴指示器。 */
  readonly isMovable: boolean;
}

/** 末端执行器标记。 */
export interface EndEffectorVisual {
  readonly id: string;
  readonly name: string;
  /** 所在 link（由 `frame.parent` 解出）；`"world"` 表示直接挂在世界系 */
  readonly linkId: string;
  /** 相对所在 link 的变换 */
  readonly transform: TransformDTO;
  readonly siteId: string;
}

/** 坐标架标记。 */
export interface FrameVisual {
  readonly id: string;
  readonly name: string;
  /** link id 或 `"world"` */
  readonly parentId: string;
  readonly transform: TransformDTO;
  readonly isWorld: boolean;
}

export interface RobotViewModel {
  readonly robotId: string;
  /** 节点表：key → 节点 */
  readonly nodes: readonly RenderNode[];
  readonly joints: readonly JointVisual[];
  readonly endEffectors: readonly EndEffectorVisual[];
  readonly frames: readonly FrameVisual[];
  /** 可动关节 id（渲染时按状态设角度的目标） */
  readonly movableJointIds: readonly string[];
  /** 根 link id（可能多个 —— problems 里会报） */
  readonly rootLinkIds: readonly string[];
}

export interface BuildResult {
  readonly viewModel: RobotViewModel;
  readonly problems: readonly string[];
}

// ---------------------------------------------------------------------------
// 常量
// ---------------------------------------------------------------------------

export const IDENTITY_TRANSFORM: TransformDTO = {
  position: [0, 0, 0],
  orientation: [0, 0, 0, 1],
};

export const WORLD_KEY = "world";
export const linkKey = (id: string): string => `link:${id}`;
export const jointKey = (id: string): string => `joint:${id}`;

// ---------------------------------------------------------------------------
// 构建
// ---------------------------------------------------------------------------

/**
 * 从 RobotModel JSON 构建渲染视图模型。
 *
 * **不抛异常**：结构异常时把问题收集进 `problems` 并尽量返回可用结果。
 * 理由与 validator 相同 —— 一个"用来诊断问题"的函数如果自己抛异常，
 * 就完全失去了诊断价值。UI 应该能显示"你的模型这里有问题"。
 */
export function buildViewModel(model: RobotModelDTO): BuildResult {
  const problems: string[] = [];

  const linkById = new Map<string, LinkDTO>();
  for (const l of model.links) {
    if (linkById.has(l.id)) problems.push(`link id 重复：${l.id}`);
    linkById.set(l.id, l);
  }

  const jointById = new Map<string, JointDTO>();
  for (const j of model.joints) {
    if (jointById.has(j.id)) problems.push(`joint id 重复：${j.id}`);
    jointById.set(j.id, j);
  }

  // --- 1. 引用完整性 -------------------------------------------------------
  for (const j of model.joints) {
    if (!linkById.has(j.parent_link)) {
      problems.push(`joint ${j.id} 的 parent_link ${j.parent_link} 不存在`);
    }
    if (!linkById.has(j.child_link)) {
      problems.push(`joint ${j.id} 的 child_link ${j.child_link} 不存在`);
    }
    const child = linkById.get(j.child_link);
    if (child && child.parent_joint !== j.id) {
      problems.push(
        `joint ${j.id} 声明 child_link=${j.child_link}，` +
          `但该 link 的 parent_joint=${child.parent_joint} —— 双向引用不一致`
      );
    }
  }

  // --- 2. 找根 link --------------------------------------------------------
  const rootLinkIds = model.links
    .filter((l) => l.parent_joint === null)
    .map((l) => l.id);
  if (rootLinkIds.length === 0) {
    problems.push("找不到根 link（所有 link 都有 parent_joint）—— 树可能是环");
  }
  if (rootLinkIds.length > 1) {
    problems.push(
      `找到 ${rootLinkIds.length} 个根 link：${rootLinkIds.join(", ")} —— 契约要求恰好一个`
    );
  }

  // --- 3. 建节点（world + link + joint 交替）-------------------------------
  const nodes: RenderNode[] = [
    {
      key: WORLD_KEY,
      id: WORLD_KEY,
      kind: "world",
      parentKey: null,
      childKeys: [],
      localTransform: IDENTITY_TRANSFORM,
      geometries: [],
      axis: null,
      isMovable: false,
    },
  ];

  // 3a. link 节点：父 = 它的 parent_joint（joint 节点）；根 link 父 = world
  for (const link of model.links) {
    nodes.push({
      key: linkKey(link.id),
      id: link.id,
      kind: "link",
      parentKey: link.parent_joint === null ? WORLD_KEY : jointKey(link.parent_joint),
      childKeys: [],
      // link 相对其父 joint 的变换是恒等 —— link 的位姿完全由父 joint 承担
      localTransform: IDENTITY_TRANSFORM,
      geometries: link.collision.map((g) => ({
        type: g.type,
        size: g.size,
        transform: g.transform,
        rgba: g.rgba,
        asset: g.asset,
      })),
      axis: null,
      isMovable: false,
    });
  }

  // 3b. joint 节点：父 = 它的 parent_link
  for (const j of model.joints) {
    nodes.push({
      key: jointKey(j.id),
      id: j.id,
      kind: "joint",
      parentKey: linkKey(j.parent_link),
      childKeys: [],
      // ★ joint 的局部变换就是 origin —— 这是整棵树里唯一"有位移"的量
      localTransform: j.origin,
      geometries: [],
      axis: normalize(j.axis),
      isMovable: j.type !== "fixed",
    });
  }

  // 3c. 填 childKeys（一次扫描，父先于子由 depthFirstOrder 保证）
  const byKey = new Map(nodes.map((n) => [n.key, n]));
  const childKeysOf = new Map<string, string[]>();
  for (const n of nodes) {
    if (n.parentKey === null) continue;
    if (!byKey.has(n.parentKey)) {
      problems.push(`节点 ${n.key} 的父 ${n.parentKey} 不存在`);
      continue;
    }
    const arr = childKeysOf.get(n.parentKey) ?? [];
    arr.push(n.key);
    childKeysOf.set(n.parentKey, arr);
  }
  const finalized: RenderNode[] = nodes.map((n) => ({
    ...n,
    childKeys: childKeysOf.get(n.key) ?? [],
  }));

  // --- 4. 环检测（在交替树上做）-------------------------------------------
  {
    const state = new Map<string, 0 | 1 | 2>(); // 0=未访问 1=在栈 2=完成
    const dfs = (key: string): void => {
      const s = state.get(key) ?? 0;
      if (s === 1) {
        problems.push(`渲染树存在环，经过 ${key}`);
        return;
      }
      if (s === 2) return;
      state.set(key, 1);
      const node = finalized.find((n) => n.key === key);
      for (const c of node?.childKeys ?? []) dfs(c);
      state.set(key, 2);
    };
    dfs(WORLD_KEY);
    for (const n of finalized) {
      if ((state.get(n.key) ?? 0) === 0) {
        problems.push(`节点 ${n.key} 从 world 不可达（孤立子图）`);
        dfs(n.key); // 继续扫，把所有孤立子图都报出来
      }
    }
  }

  // --- 5. 关节视图 ---------------------------------------------------------
  const joints: JointVisual[] = model.joints.map((j) => ({
    id: j.id,
    type: j.type,
    parentLink: j.parent_link,
    childLink: j.child_link,
    origin: j.origin,
    axis: normalize(j.axis),
    limits:
      j.limits && j.limits.position_min !== null && j.limits.position_max !== null
        ? { min: j.limits.position_min, max: j.limits.position_max }
        : null,
    isMovable: j.type !== "fixed",
  }));

  // --- 6. 末端执行器 -------------------------------------------------------
  const frameById = new Map(model.frames.map((f) => [f.id, f]));
  const siteById = new Map(model.sites.map((s) => [s.id, s]));
  const endEffectors: EndEffectorVisual[] = [];

  for (const ee of model.end_effectors as readonly EndEffectorDTO[]) {
    const frame = frameById.get(ee.frame);
    if (!frame) {
      problems.push(`end_effector ${ee.id} 引用了不存在的 frame ${ee.frame}`);
      continue;
    }
    if (!siteById.has(ee.site)) {
      problems.push(`end_effector ${ee.id} 引用了不存在的 site ${ee.site}`);
    }
    if (frame.parent !== WORLD_KEY && !linkById.has(frame.parent)) {
      problems.push(
        `end_effector ${ee.id} 的 frame ${ee.frame} 挂在未知 link ${frame.parent} 上`
      );
      continue;
    }
    endEffectors.push({
      id: ee.id,
      name: ee.name,
      linkId: frame.parent,
      // ★ 用 frame.transform（frame-local）。
      //   当前模型里与 site.transform 数值相同，但语义不同：
      //   frame 是"这个 EE 定义在哪"，site 是"这个具名点在哪"。
      //   v0.1 的 EndEffector 契约以 frame 为主导（见 robot-model.md §3.7）。
      transform: frame.transform,
      siteId: ee.site,
    });
  }

  // --- 7. 坐标架 -----------------------------------------------------------
  const frames: FrameVisual[] = model.frames.map((f) => ({
    id: f.id,
    name: f.name,
    parentId: f.parent,
    transform: f.transform,
    isWorld: f.parent === WORLD_KEY,
  }));

  // --- 8. 自检：child_joints 与推导是否一致 --------------------------------
  for (const link of model.links) {
    const derived = model.joints
      .filter((j) => j.parent_link === link.id)
      .map((j) => j.id)
      .sort();
    const declared = [...link.child_joints].sort();
    if (declared.join(",") !== derived.join(",")) {
      problems.push(
        `link ${link.id} 的 child_joints=${JSON.stringify(declared)} ` +
          `与按 joints[].parent_link 推导的 ${JSON.stringify(derived)} 不一致`
      );
    }
  }

  return {
    viewModel: {
      robotId: model.metadata.id,
      nodes: finalized,
      joints,
      endEffectors,
      frames,
      movableJointIds: joints.filter((j) => j.isMovable).map((j) => j.id),
      rootLinkIds,
    },
    problems,
  };
}

// ---------------------------------------------------------------------------
// 遍历 / 小工具（供渲染器与测试使用）
// ---------------------------------------------------------------------------

/** 单位化；零向量原样返回（由 validator 负责报错，这里不抛）。 */
export function normalize(v: Vec3): Vec3 {
  const n = Math.hypot(v[0], v[1], v[2]);
  if (n === 0) return v;
  return [v[0] / n, v[1] / n, v[2] / n];
}

/** **深度优先**顺序（父总在子之前）—— 渲染器按它顺序建 Object3D。 */
export function depthFirstOrder(vm: RobotViewModel): readonly string[] {
  const byKey = new Map(vm.nodes.map((n) => [n.key, n]));
  const order: string[] = [];
  const seen = new Set<string>();
  const visit = (key: string, guard: number): void => {
    if (guard > 10000 || seen.has(key)) return;
    seen.add(key);
    order.push(key);
    for (const c of byKey.get(key)?.childKeys ?? []) visit(c, guard + 1);
  };
  for (const n of vm.nodes) {
    if (n.parentKey === null) visit(n.key, 0);
  }
  // 兜底：不可达节点也列出（problems 已记录具体原因）
  for (const n of vm.nodes) if (!seen.has(n.key)) visit(n.key, 0);
  return order;
}

/** 从 world 到某节点的**关节链**（按世界→末端的顺序）。 */
export function jointChainTo(
  vm: RobotViewModel,
  key: string
): readonly string[] {
  const byKey = new Map(vm.nodes.map((n) => [n.key, n]));
  const chain: string[] = [];
  let cur = byKey.get(key);
  let guard = 0;
  while (cur && cur.parentKey !== null && guard++ < 10000) {
    if (cur.kind === "joint") chain.unshift(cur.id);
    cur = byKey.get(cur.parentKey);
  }
  return chain;
}

/** link id → 它在树中的 key（渲染器常用）。 */
export function linkNodeOf(vm: RobotViewModel, linkId: string): RenderNode | undefined {
  return vm.nodes.find((n) => n.kind === "link" && n.id === linkId);
}

/** joint id → 它在树中的 key。 */
export function jointNodeOf(vm: RobotViewModel, jointId: string): RenderNode | undefined {
  return vm.nodes.find((n) => n.kind === "joint" && n.id === jointId);
}
