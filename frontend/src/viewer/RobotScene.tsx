/**
 * =============================================================================
 * RobotScene.tsx —— RobotModel → Three.js 场景
 * -----------------------------------------------------------------------------
 * 本组件是 §59 交付链的第 3 环：
 *
 * ```text
 * RobotModel  (来自后端 /api/robots/<id>/model)
 *       ↓   buildViewModel()        —— 纯函数，零 Three.js
 * RobotViewModel
 *       ↓   <RobotScene/>            —— 本文件
 * Three.js
 * ```
 *
 * ## 本文件**只做**陈述式的三件事
 *
 *   1. 按 `vm.nodes` 的父子关系建 `Object3D` 树（父子由 viewModel 决定，本文件不推导）
 *   2. 给 link 节点挂几何（几何数据来自模型，本文件不硬编码形状）
 *   3. 给 joint 节点挂轴指示器（轴来自模型，本文件不硬编码方向）
 *
 * ## 本文件**不做**的三件事（各自有唯一归属处）
 *
 *   - ❌ 不做坐标转换 → `coordinateAdapter.ts`
 *   - ❌ 不做层级推导 → `viewModel.ts`
 *   - ❌ 不按机器人型号分支 → 提示词 §69 规则 2
 *
 * ## 关于颜色
 *
 * v0.1 的 `GeometryRef.rgba` 全是 `[0.5,0.5,0.5,1]`（占位灰）。
 * 本文件**照用**它，不做"按链接名上色"这类美化 ——
 * 那会引入一份模型外的真值，且按名字上色必然要读 link id 做判断，
 * 离"型号分支"只有一步。
 *
 * ## 关于节点标识：只用 `name`，**禁止 `data-*`**
 *
 * ⚠️ 本文件曾经在 `<group>` 上同时挂 `data-kind` / `data-id`，
 *    理由是"静态渲染（`renderToStaticMarkup`）下它们会原样保留成字符串"。
 *    **这个理由只在 DOM 渲染器下成立；在真实浏览器里它会让整个场景崩掉。**
 *
 * 根因（已实测定位到 r3f 源码）：
 *
 * ```text
 * @react-three/fiber 的 diffProps 里有一条：
 *     if (key.includes("-")) entries2 = key.split("-");
 * 于是 `data-kind` 被拆成 ["data", "kind"]，
 * 接着 applyProps 执行：
 *     targetProp = keys.reduce((acc, k) => acc[k], instance)
 *     // → instance.data  →  undefined
 *     // → undefined["kind"]  →  TypeError: Cannot read properties of
 *     //                        undefined (reading 'kind')
 * ```
 *
 * 症状极具误导性：报错栈里全是 `chunk-*.js` 与 `<group>`，
 * **完全看不到本文件**；且它发生在 React 渲染期，
 * `ErrorBoundary` 捕获后整棵树重挂载 → 页面 **全黑**
 * （`#root` 的 innerHTML 为空、`canvas`/`.panel` 都查不到）。
 * 于是所有基于 DOM 查询的断言一起失败，看起来像"后端没起"或"前端挂了"。
 *
 * ⇒ 结论：**Three.js fiber 元素上的属性名不得含 `-`**。
 *   需要携带元数据时用 `name`（`Object3D.name`，真实可用）
 *   或 `userData`（对象，真实可用）。两者在静态渲染下会退化，
 *   但静态渲染本来就不是本组件的运行环境，**以浏览器里的正确性为准**。
 *
 * ## 其余两条约定仍然成立
 *
 *   - ⚠️ 任何**对象**属性在静态渲染里都会变成 `[object Object]`，
 *     所以 position / quaternion 一律传**数字数组**（见 GeomMesh 注释）
 *   - ⚠️ 同理，`<meshStandardMaterial/>` 等小写元素名在
 *     `renderToStaticMarkup` 下会被当成未知 **HTML** 元素并报
 *     "incorrect casing" —— 这是"用 DOM 渲染器渲染非 DOM 树"的固有副作用，
 *     **不是缺陷**。在真实 fiber 场景里小写名是正确且必需的。
 */

import { useMemo, useEffect } from "react";
import type { ReactNode } from "react";
import * as THREE from "three";

import {
  axisToThree,
  transformToThree,
} from "./coordinateAdapter.ts";
import type { GeometryRefLike, RenderNode, RobotViewModel } from "./viewModel.ts";

// ---------------------------------------------------------------------------
// 几何构造
// ---------------------------------------------------------------------------

/**
 * `GeometryRef` → Three.js `BufferGeometry`。
 *
 * ⚠️ `size` 的语义**按 type 不同**（与 MJCF 一致）：
 *   | type     | size 含义                    |
 *   |----------|------------------------------|
 *   | cylinder | [半径, 半长]（沿 Z 轴）      |
 *   | capsule  | [半径, 半长]（沿 Z 轴）      |
 *   | sphere   | [半径]                       |
 *   | box      | [x 半长, y 半长, z 半长]     |
 *   | plane    | [x 半长, y 半长]             |
 *
 * 记错这个表会得到一个"尺寸明显不对"的机器人 —— 属于**显眼**的错误，
 * 所以它比"坐标悄悄偏了"良性。但仍然在这里写清楚，避免重试成本。
 */
function makeGeometry(g: GeometryRefLike): THREE.BufferGeometry | null {
  const [a = 0, b = 0, c = 0] = g.size;
  switch (g.type) {
    case "box":
      // size 是**半长**（MJCF 约定）
      return new THREE.BoxGeometry(a * 2, b * 2, c * 2);
    case "sphere":
      return new THREE.SphereGeometry(a, 24, 16);
    case "cylinder":
      // CylinderGeometry(顶半径, 底半径, 高)；MJCF 的 cylinder 沿 Z 轴、高 = 2*半长
      return new THREE.CylinderGeometry(a, a, b * 2, 24);
    case "capsule":
      // CapsuleGeometry(半径, 圆柱段长度, ...)；MJCF 的 capsule 半长含两端半球
      // ⇒ 圆柱段长度 = 2*(半长 - 半径)，下限 0（半径大于半长时退化成球）
      return new THREE.CapsuleGeometry(a, Math.max(0, b * 2 - a * 2), 8, 16);
    case "ellipsoid":
      return new THREE.SphereGeometry(1, 24, 16);
    case "plane":
      return new THREE.PlaneGeometry(a * 2, b * 2);
    case "mesh":
      // v0.1 不加载外部网格资源（asset 为 null）。退回一个小盒子占位，
      // 这样"有个东西在这"是可见的，而不是静默消失。
      return new THREE.BoxGeometry(0.01, 0.01, 0.01);
    default:
      return null;
  }
}

/**
 * 单个几何的网格（含它自己的局部变换）。
 *
 * ★ 位置/朝向用**数字数组**而不是 `THREE.Vector3` / `THREE.Quaternion`。
 *   fiber 两者都接受，但数组有个额外好处：它会被序列化成
 *   `position="0.055,0,0"`，于是"位置到底对不对"在**渲染产物里可见**，
 *   而不只存在于 JS 对象里。这让自动化检查能直接在产物上验证数值。
 *   （`Vector3` 是对象，`renderToStaticMarkup` 会把它整个丢掉。）
 */
function GeomMesh({
  g,
  ghost = false,
}: {
  g: GeometryRefLike;
  ghost?: boolean | undefined;
}): ReactNode {
  const geometry = useMemo(() => makeGeometry(g), [g]);
  const { position, quaternion } = useMemo(
    () => transformToThree(g.transform),
    [g.transform]
  );
  const color = useMemo(
    () => new THREE.Color(g.rgba[0] ?? 0.5, g.rgba[1] ?? 0.5, g.rgba[2] ?? 0.5),
    [g.rgba]
  );
  const posArr: [number, number, number] = [position.x, position.y, position.z];
  const quatArr: [number, number, number, number] = [
    quaternion.x,
    quaternion.y,
    quaternion.z,
    quaternion.w,
  ];

  useEffect(() => () => geometry?.dispose(), [geometry]);

  if (!geometry) {
    // ✗ 未知类型：不画，但要能被看见（渲染一个醒目的线框盒）
    return (
      <mesh name="geom-unknown" position={posArr}>
        <boxGeometry args={[0.02, 0.02, 0.02]} />
        <meshBasicMaterial color="#ff00ff" wireframe />
      </mesh>
    );
  }

  return (
    <mesh
      name={`geom:${g.type}`}
      geometry={geometry}
      position={posArr}
      quaternion={quatArr}
    >
      {ghost ? (
        // 幽灵臂：线框 + 低不透明度。
        // 用**线框**而不是纯半透明实心：实心半透明与权威臂重叠时，
        // 两者会互相遮挡，反而看不出"偏差有多大"；
        // 线框能让人同时看见两套几何的轮廓。
        <meshBasicMaterial
          color="#39c5ff"
          wireframe
          transparent
          opacity={0.35}
          depthWrite={false}
        />
      ) : (
        <meshStandardMaterial color={color} metalness={0.2} roughness={0.6} />
      )}
    </mesh>
  );
}

// ---------------------------------------------------------------------------
// 轴指示器
// ---------------------------------------------------------------------------

/**
 * 关节轴指示器：在关节原点画一根沿 `axis` 的细线 + 一个锥头。
 *
 * ★ 轴必须来自模型（`joint.axis`），不允许按关节名硬编码。
 *   这是 §59 的 "Joint axis 正确" 验收项。
 *
 * ⚠️ 轴是**局部**的（`axisToThree` 不做世界转换）。理由见
 *   `coordinateAdapter.ts::axisToThree` —— 父坐标系已由整场景旋转带入 Three.js。
 */
function AxisIndicator({
  axis,
  length = 0.05,
}: {
  axis: readonly [number, number, number];
  length?: number;
}): ReactNode {
  // 把 +Z（Three.js 的 CylinderGeometry 轴向）转到目标轴
  const { dir, quaternion } = useMemo(() => {
    const d = axisToThree(axis);
    const q = new THREE.Quaternion().setFromUnitVectors(
      new THREE.Vector3(0, 0, 1),
      d
    );
    return { dir: d, quaternion: q };
  }, [axis]);

  const color = useMemo(() => {
    // 按轴分量上色只是**视觉辅助**（方便区分 X/Y/Z 轴），
    // 不参与任何计算 —— 计算用的永远是 `dir`。
    const [x, y, z] = [Math.abs(dir.x), Math.abs(dir.y), Math.abs(dir.z)];
    if (z >= x && z >= y) return new THREE.Color(0x2266ff); // 蓝 = Z
    if (y >= x) return new THREE.Color(0x22bb44); // 绿 = Y
    return new THREE.Color(0xff3333); // 红 = X
  }, [dir]);

  return (
    // ★ 给轴指示器一个可辨识的 name。
    //   没有它时，"这个 joint 下有几个子 group"就无法区分
    //   "轴指示器" 与 "子 link/joint" —— 检查只能靠数个数，那是脆弱的判据
    //   （本文件曾经就因此把一个正确的实现判成失败）。
    <group name={`axis:${axis.join(",")}`} quaternion={quaternion}>
      <mesh position={[0, 0, length / 2]}>
        <cylinderGeometry args={[0.0018, 0.0018, length, 8]} />
        <meshBasicMaterial color={color} />
      </mesh>
      <mesh position={[0, 0, length]}>
        <coneGeometry args={[0.0045, 0.012, 12]} />
        <meshBasicMaterial color={color} />
      </mesh>
    </group>
  );
}

// ---------------------------------------------------------------------------
// 单节点递归
// ---------------------------------------------------------------------------

function NodeView({
  vm,
  node,
  eeByLink,
  showAxes,
  jointPositions,
  ghost,
}: {
  vm: RobotViewModel;
  node: RenderNode;
  eeByLink: ReadonlyMap<string, readonly string[]>;
  showAxes: boolean;
  /** 关节角（rad）；缺省 = 全 0（零位形）。**权威值与预演值共用本参数**。 */
  jointPositions?: Readonly<Record<string, number>> | undefined;
  /** 幽灵臂：半透明 + 不画轴/EE（它只是一层"预测"的视觉提示）。 */
  ghost?: boolean | undefined;
}): ReactNode {
  // link 相对父 joint 恒等（位移由 joint 的 origin 承担）。
  // ★ 用数字数组而非 Vector3 —— 让数值在渲染产物里可见（见 GeomMesh 注释）。
  const { posArr, quatArr } = useMemo(() => {
    if (node.kind === "link" && node.parentKey !== null) {
      return {
        posArr: [0, 0, 0] as [number, number, number],
        quatArr: [0, 0, 0, 1] as [number, number, number, number],
      };
    }
    const { position, quaternion } = transformToThree(node.localTransform);
    return {
      posArr: [position.x, position.y, position.z] as [number, number, number],
      quatArr: [quaternion.x, quaternion.y, quaternion.z, quaternion.w] as [
        number,
        number,
        number,
        number,
      ],
    };
  }, [node]);

  // ★ 关节自由度：**在 joint 节点自身坐标系里**额外绕 `axis` 转 `q`。
  //
  //   为什么不能并进 `node.localTransform`：那个 transform 是关节的
  //   **安装位姿**（MJCF 的 `<joint pos=... axis=...>` 之于父 link），
  //   而自由度是**安装之后**绕自身轴的转动。两者相乘的顺序是
  //   `origin ∘ Rot(axis, q)`，Left-multiply 顺序不可反 ——
  //   反了会让整条链在非零位形下系统性偏移（且看起来"差不多对"）。
  //
  //   这与 Core `backend/kinematics/fk.py` 完全一致：那里也是
  //   `joint.origin` 再乘 `Quaternion.from_axis_angle(joint.axis, value)`。
  //
  //   `axisToThree` 只做**局部**轴换算（父坐标系已由整场景旋转带入），
  //   理由见 coordinateAdapter.ts。
  const jointQuat = useMemo<[number, number, number, number]>(() => {
    if (node.kind !== "joint" || node.axis === null || !node.isMovable) {
      return [0, 0, 0, 1];
    }
    const q = jointPositions?.[node.id] ?? 0;
    if (q === 0) return [0, 0, 0, 1];
    const axis = axisToThree(node.axis);
    const rot = new THREE.Quaternion().setFromAxisAngle(axis, q);
    return [rot.x, rot.y, rot.z, rot.w];
  }, [node, jointPositions]);

  const childNodes = node.childKeys
    .map((k) => vm.nodes.find((n) => n.key === k))
    .filter((n): n is RenderNode => n !== undefined);

  // ★ 末端执行器标记**必须挂在它所在 link 的坐标系里**，
  //   否则它就是个静止的装饰品：机器人一动、EE 标记不跟着动。
  //   这是"看起来还行"的典型错误 —— 静态截图完全正常。
  const eeIdsHere = node.kind === "link" ? (eeByLink.get(node.id) ?? []) : [];

  // ★ 自由度施加在一个**内层 group** 上，而不是并进上面那个 group。
  //
  //   并进去是可行的（四元数左乘即可），但会让 `name={node.key}` 那个
  //   group 的 `quaternion` **不再等于** `localTransform` 的朝向。
  //   而 `render.check.ts` 的判据正是"节点 quaternion 应等于
  //   adapter 换算出来的值"—— 那是一条验证 §41 没有偷偷改坐标的断言。
  //   并进去会把它变成一条恒真的断言（拿被验对象自己算出来的值判它自己）。
  //
  //   分层之后：外层 = 模型的静态安装位姿（可被上述断言独立检查），
  //             内层 = 运行期的自由度。两者各自可观测。
  // ★ 自由度施加在一个**内层 group** 上，而不是并进上面那个 group。
  //
  //   并进去是可行的（四元数左乘即可），但会让 `name={node.key}` 那个
  //   group 的 `quaternion` **不再等于** `localTransform` 的朝向。
  //   而 `render.check.ts` 的判据正是"节点 quaternion 应等于
  //   adapter 换算出来的值"—— 那是一条验证 §41 没有偷偷改坐标的断言。
  //   并进去会把它变成一条恒真的断言（拿被验对象自己算出来的值判它自己）。
  //
  //   分层之后：外层 = 模型的静态安装位姿（可被上述断言独立检查），
  //             内层 = 运行期的自由度。两者各自可观测。
  const isJoint = node.kind === "joint";

  const children = (
    <>
      {/* link：画它的几何 */}
      {node.kind === "link" &&
        node.geometries.map((g, i) => (
          <GeomMesh key={`${node.key}-g${i}`} g={g} ghost={ghost} />
        ))}

      {/* 该 link 上的末端执行器（幽灵臂不画 —— 它是预测，不该有"权威 EE 标记"的观感） */}
      {!ghost &&
        eeIdsHere.map((eeId) => (
          <EndEffectorMarker key={`ee-${eeId}`} vm={vm} eeId={eeId} />
        ))}

      {/* joint：画轴指示器（fixed 关节不画 —— 它没有自由度，画了会误导） */}
      {!ghost && showAxes && node.kind === "joint" && node.axis !== null && node.isMovable && (
        <AxisIndicator axis={node.axis} />
      )}

      {childNodes.map((c) => (
        <NodeView
          key={c.key}
          vm={vm}
          node={c}
          eeByLink={eeByLink}
          showAxes={showAxes}
          jointPositions={jointPositions}
          ghost={ghost}
        />
      ))}
    </>
  );

  return (
    <group name={node.key} position={posArr} quaternion={quatArr}>
      {isJoint ? (
        <group name={`dof:${node.id}`} quaternion={jointQuat}>
          {children}
        </group>
      ) : (
        children
      )}
    </group>
  );
}

// ---------------------------------------------------------------------------
// 世界坐标架 + 末端执行器标记
// ---------------------------------------------------------------------------

/**
 * 一个放大的三轴坐标架（红 X / 绿 Y / 蓝 Z）。
 *
 * 用于 "Coordinate Frame 正确" 验收：世界系原点应该在地面,
 * 且 +Z 朝上、+X 朝前、+Y 朝左。**能不能一眼看出来**是它的价值。
 */
export function FrameAxes({
  scale = 0.1,
  origin,
}: {
  scale?: number;
  origin?: [number, number, number];
}): ReactNode {
  return (
    <axesHelper args={[scale]} position={origin ?? [0, 0, 0]} />
  );
}

/**
 * 末端执行器标记：一个半透明球 + 坐标架。
 *
 * ★ 位置来自 `endEffectors[].transform`（即 `frame.transform`），
 *   不来自任何硬编码 —— 换台机器人自动正确。
 */
function EndEffectorMarker({
  vm,
  eeId,
}: {
  vm: RobotViewModel;
  eeId: string;
}): ReactNode {
  const ee = vm.endEffectors.find((e) => e.id === eeId);
  const posArr = useMemo<[number, number, number]>(() => {
    if (!ee) return [0, 0, 0];
    const { position } = transformToThree(ee.transform);
    return [position.x, position.y, position.z];
  }, [ee]);
  const quatArr = useMemo<[number, number, number, number]>(() => {
    if (!ee) return [0, 0, 0, 1];
    const { quaternion } = transformToThree(ee.transform);
    return [quaternion.x, quaternion.y, quaternion.z, quaternion.w];
  }, [ee]);
  if (!ee) return null;
  return (
    <group name={`ee:${ee.id}`} position={posArr} quaternion={quatArr}>
      <mesh name="ee-sphere">
        <sphereGeometry args={[0.008, 16, 12]} />
        <meshBasicMaterial color={0xffaa00} transparent opacity={0.85} />
      </mesh>
      <axesHelper args={[0.04]} />
    </group>
  );
}

// ---------------------------------------------------------------------------
// 主场景
// ---------------------------------------------------------------------------

export interface RobotSceneProps {
  vm: RobotViewModel;
  /** 是否显示关节轴指示器 */
  showAxes?: boolean;
  /** 是否显示末端执行器标记 */
  showEndEffector?: boolean;
  /** 是否显示世界坐标架 */
  showWorldFrame?: boolean;
  /**
   * 关节角（rad）。缺省 = 全 0（零位形）。
   *
   * 本参数只做**陈述式**应用：谁把值放进来由调用方决定。
   * ```text
   * 权威姿态 —— App 把 robot_state.joints 传进来（§49 唯一闭环）
   * 预演姿态 —— App 把 predictor 的 ghost 值传给 <GhostArm/>
   * ```
   */
  jointPositions?: Readonly<Record<string, number>>;
}

/**
 * 幽灵臂 —— 前端本地预演的可视化（§49 允许的形态）。
 *
 * ## 它为什么必须是一个**独立**的组件
 *
 * 因为它与权威树是**两棵 Object3D 树**。若把预演结果混进权威树的同一个
 * `jointPositions`，那么"屏幕上看到的是预测还是真值"就无从区分 ——
 * 而这正是 §49 要求被禁止的状态（预测会悄悄变成权威姿态的来源）。
 *
 * 排他性由 `App` 保证：
 * ```text
 * <RobotScene jointPositions={state.joints} />        ← 权威（永远画）
 * {ghostOn && <GhostArm vm={vm} jointPositions={pred} />}   ← 预演（可关）
 * ```
 * 两者视觉上可区分（幽灵是蓝色线框），所以**永远能看出当前偏差有多大**。
 */
export function GhostArm({
  vm,
  jointPositions,
}: {
  vm: RobotViewModel;
  jointPositions: Readonly<Record<string, number>>;
}): ReactNode {
  const roots = useMemo(() => vm.nodes.filter((n) => n.parentKey === null), [vm]);
  return (
    <group name="ghost-arm">
      {roots.map((r) => (
        <NodeView
          key={r.key}
          vm={vm}
          node={r}
          eeByLink={new Map()}
          showAxes={false}
          jointPositions={jointPositions}
          ghost
        />
      ))}
    </group>
  );
}

export function RobotScene({
  vm,
  showAxes = true,
  showEndEffector = true,
  showWorldFrame = true,
  jointPositions,
}: RobotSceneProps): ReactNode {
  const roots = useMemo(
    () => vm.nodes.filter((n) => n.parentKey === null),
    [vm]
  );

  // link id → 挂在该 link 上的 EE id 列表
  const eeByLink = useMemo(() => {
    const m = new Map<string, string[]>();
    for (const ee of vm.endEffectors) {
      const arr = m.get(ee.linkId) ?? [];
      arr.push(ee.id);
      m.set(ee.linkId, arr);
    }
    return m;
  }, [vm]);

  return (
    <group name="robot-root">
      {roots.map((r) => (
        <NodeView
          key={r.key}
          vm={vm}
          node={r}
          eeByLink={showEndEffector ? eeByLink : new Map()}
          showAxes={showAxes}
          jointPositions={jointPositions}
        />
      ))}

      {showWorldFrame && <FrameAxes scale={0.12} />}
    </group>
  );
}

export { AxisIndicator, EndEffectorMarker, makeGeometry };
