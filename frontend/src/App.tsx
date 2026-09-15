/**
 * =============================================================================
 * App.tsx —— 应用外壳
 * -----------------------------------------------------------------------------
 * 职责：拉模型 → 建视图模型 → 交给 <RobotScene/>；并提供相机控制与诊断面板。
 *
 * ## §59 验收项在此的落点
 *
 * | 验收项                     | 落点                                        |
 * |----------------------------|---------------------------------------------|
 * | 浏览器显示机器人           | <RobotScene/> + 默认选中第一个机器人        |
 * | Camera Orbit / Zoom / Pan  | <OrbitControls/>（三个能力由它一次给全）    |
 * | Joint hierarchy 正确       | viewModel.ts（本文件只消费，不推导）        |
 * | Joint axis 正确            | RobotScene 的 <AxisIndicator/>              |
 * | End Effector 正确          | RobotScene 的 <EndEffectorMarker/>          |
 * | Coordinate Frame 正确      | <WorldFrame/> 地面网格 + 坐标架             |
 * | RobotModel 与 Renderer 一致 | semantics.check.ts（121 项检查）            |
 *
 * ## ★ capability 驱动，不按型号分支（提示词 §69 规则 2）
 *
 * 面板显示与否全部由 `capabilities` 决定：
 *   `capabilities.fk` → FK 面板，`capabilities.ik` → IK 面板
 * 本文件**不允许**出现机器人 id 字面量比较。
 */

import { useCallback, useEffect, useMemo, useRef, useState, useSyncExternalStore } from "react";
import type { ReactNode } from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls } from "@react-three/drei";

import { ApiError, fetchRobotModel, fetchRobots } from "./api.ts";
import type { RobotModelResponse, RobotSummaryDTO } from "./viewer/model.ts";
import { buildViewModel } from "./viewer/viewModel.ts";
import type { RobotViewModel } from "./viewer/viewModel.ts";
import { GhostArm, RobotScene } from "./viewer/RobotScene.tsx";
import { SCENE_ROTATION_X } from "./viewer/coordinateAdapter.ts";
import { createWsClient } from "./ws.ts";
import type { WsClient } from "./ws.ts";
import { JointPanel } from "./JointPanel.tsx";
import { useGhostPrediction } from "./sim/useGhostPrediction.ts";

// ---------------------------------------------------------------------------
// 常量
// ---------------------------------------------------------------------------

/**
 * 稳定的空关节列表。
 *
 * ⚠️ 不能写 `vm?.movableJointIds ?? []` —— 每次渲染都是新数组，
 *    会让 `useGhostPrediction` 里以内容为 key 的 effect 反复重跑
 *    （进而反复重建预演初值 ⇒ 幽灵臂永远归零）。
 */
const EMPTY_IDS: readonly string[] = Object.freeze([]);

// ---------------------------------------------------------------------------
// 诊断面板
// ---------------------------------------------------------------------------

function DiagnosticsPanel({
  vm,
  problems,
}: {
  vm: RobotViewModel | null;
  problems: readonly string[];
}): ReactNode {
  if (!vm) return null;
  return (
    <div className="panel">
      <h3>模型诊断</h3>
      <table>
        <tbody>
          <tr>
            <td>robot</td>
            <td><code>{vm.robotId}</code></td>
          </tr>
          <tr>
            <td>nodes</td>
            <td>{vm.nodes.length}（world + links + joints）</td>
          </tr>
          <tr>
            <td>links</td>
            <td>{vm.nodes.filter((n) => n.kind === "link").length}</td>
          </tr>
          <tr>
            <td>joints</td>
            <td>
              {vm.joints.length}（可动 {vm.movableJointIds.length}）
            </td>
          </tr>
          <tr>
            <td>可动关节</td>
            <td>
              <code>{vm.movableJointIds.join(", ")}</code>
            </td>
          </tr>
          <tr>
            <td>末端执行器</td>
            <td>
              {vm.endEffectors.length === 0
                ? "（无）"
                : vm.endEffectors
                    .map((e) => `${e.id} @ ${e.linkId}`)
                    .join(", ")}
            </td>
          </tr>
          <tr>
            <td>坐标架</td>
            <td>
              <code>{vm.frames.map((f) => f.id).join(", ")}</code>
            </td>
          </tr>
        </tbody>
      </table>

      <h3>关节轴（来自模型，未变换）</h3>
      <table>
        <thead>
          <tr>
            <th>joint</th>
            <th>type</th>
            <th>axis [x,y,z]</th>
            <th>limits (rad)</th>
          </tr>
        </thead>
        <tbody>
          {vm.joints.map((j) => (
            <tr key={j.id}>
              <td><code>{j.id}</code></td>
              <td>{j.type}</td>
              <td>
                <code>[{j.axis.map((c) => c.toFixed(3)).join(", ")}]</code>
              </td>
              <td>
                {j.limits
                  ? `${j.limits.min.toFixed(4)} … ${j.limits.max.toFixed(4)}`
                  : "—"}
              </td>
            </tr>
          ))}
        </tbody>
      </table>

      <h3>坐标适配</h3>
      <p>
        整场景旋转 <code>rotation.x = {SCENE_ROTATION_X.toFixed(6)}</code>（−π/2）。
        RobotModel 保持 Z-up 语义不变；Three.js 世界坐标 = (x, z, −y)。
      </p>

      {problems.length > 0 && (
        <>
          <h3 className="bad">构建问题（{problems.length}）</h3>
          <ul>
            {problems.map((p) => (
              <li key={p}>{p}</li>
            ))}
          </ul>
        </>
      )}
    </div>
  );
}

function CapabilityPanel({ caps }: { caps: RobotSummaryDTO["capabilities"] }): ReactNode {
  // ★ 面板按 capability 显示，不按机器人 id
  const items: [string, boolean][] = [
    ["Simulation", caps.simulation],
    ["FK", caps.fk],
    ["IK", caps.ik],
    ["Actuator Control", caps.actuator_control],
    ["End Effector", caps.end_effector],
  ];
  return (
    <div className="panel">
      <h3>能力（capability 驱动）</h3>
      <ul className="caps">
        {items.map(([name, on]) => (
          <li key={name} className={on ? "on" : "off"}>
            {on ? "✓" : "✗"} {name}
          </li>
        ))}
      </ul>
      <p className="hint">
        关节控制面板由 <code>actuator_control</code> 开启；
        FK/IK 面板由 <code>fk</code> / <code>ik</code> 开启。
        这是 capability 驱动的（不是按型号判断 —— §69 规则 2）。
      </p>
    </div>
  );
}

/**
 * 连接状态。
 *
 * ★ 它显示的是"§49 闭环通不通"这一**唯一**事实。
 *   前端没有第二条通往机器人姿态的路 —— 所以这个灯灭了，
 *   界面上的机器人就**不应该**动。把这条做成一等公民的 UI 元素，
 *   而不是藏在控制台里。
 */
function ConnectionPanel({
  state,
  connectCount,
  lastError,
  robotCount,
}: {
  state: "connecting" | "open" | "closed";
  connectCount: number;
  lastError: string | null;
  robotCount: number;
}): ReactNode {
  return (
    <div className="panel">
      <h3>链路（§49 唯一闭环）</h3>
      <table>
        <tbody>
          <tr>
            <td>WebSocket</td>
            {/*
              ★ 刻意**不**用颜色区分连接状态。
                本工程的 UI 约定是"无装饰色"（见 USER.md 的 UI 偏好），
                而"连接中… / 已连接 / 未连接"这三个词本身已经说清楚了。
                仅在**出错**时用既有的 `.bad`（那是既有的告警约定，
                不是新增的装饰色）。
            */}
            <td>{state === "open" ? "已连接" : state === "connecting" ? "连接中…" : "未连接"}</td>
          </tr>
          <tr>
            <td>建连次数</td>
            <td>{connectCount}</td>
          </tr>
          <tr>
            <td>订阅机器人</td>
            <td>{robotCount}</td>
          </tr>
        </tbody>
      </table>
      <p className="hint">
        命令路径 <code>Frontend → WS → Runtime → Backend → MuJoCo</code>，
        状态原路返回。前端**不**直接改 Three.js。
      </p>
      {lastError !== null && (
        <p className="bad">
          <code>{lastError}</code>
        </p>
      )}
    </div>
  );
}

// ---------------------------------------------------------------------------
// 场景（含坐标适配）
// ---------------------------------------------------------------------------

function SceneRoot({
  vm,
  showAxes,
  showEe,
  authoritative,
  ghost,
}: {
  vm: RobotViewModel;
  showAxes: boolean;
  showEe: boolean;
  /** `robot_state.joints` —— 权威姿态（§49 唯一闭环的产物）。 */
  authoritative: Readonly<Record<string, number>> | null;
  /** 本地预演的关节角；`null` = 不画幽灵臂。 */
  ghost: Readonly<Record<string, number>> | null;
}): ReactNode {
  // ★ 整场景旋转：这是 `coordinateAdapter` 的唯一出口。
  //   施加在一个包住整个机器人的 <group> 上，**不**逐节点转换
  //   （理由见 docs/coordinate-system.md §4.2 与 adapter 的模块注释）。
  //
  //   ⚠️ 不要在 <Canvas> 上设 `rotation` —— Canvas 接受的是相机/渲染参数，
  //      不是场景变换。旋转必须落在一个 group（或 scene）对象上。
  //
  //   ⚠️⚠️ 幽灵臂必须在**同一个** adapter-root 内。
  //       它表达的是同一条链路的两个时刻（后端已到 vs 本地预演），
  //       若给它套第二个 adapter-root，两棵树的坐标基准就可能漂开 ——
  //       而漂移会表现为"幽灵臂与实际臂慢慢错位"，
  //       看起来像预测跑飞了，实际是两套变换。
  return (
    <group rotation={[SCENE_ROTATION_X, 0, 0]} name="threejs-adapter-root">
      <RobotScene
        vm={vm}
        showAxes={showAxes}
        showEndEffector={showEe}
        jointPositions={authoritative ?? undefined}
      />
      {ghost !== null && <GhostArm vm={vm} jointPositions={ghost} />}
    </group>
  );
}

// ---------------------------------------------------------------------------
// App
// ---------------------------------------------------------------------------

export default function App(): ReactNode {
  const [robots, setRobots] = useState<RobotSummaryDTO[]>([]);
  const [selected, setSelected] = useState<string | null>(null);
  const [payload, setPayload] = useState<RobotModelResponse | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [loading, setLoading] = useState(true);
  const [showAxes, setShowAxes] = useState(true);
  const [showEe, setShowEe] = useState(true);
  const [showPanel, setShowPanel] = useState(true);
  const [showGhost, setShowGhost] = useState(true);
  const [showJoints, setShowJoints] = useState(true);

  // -------------------------------------------------------------------------
  // §49 闭环：一个 WS 客户端，绑在整个 App 的生命周期上
  //
  // ★ 用 `useRef` 而不是 `useState` 持有它：
  //   客户端是个**有身份的对象**（它自己管重连、退避、订阅），
  //   放进 state 会让"新建一个客户端"与"重新渲染"耦合，
  //   每渲染一次就换掉连接。
  // -------------------------------------------------------------------------
  const wsRef = useRef<WsClient | null>(null);
  if (wsRef.current === null) {
    wsRef.current = createWsClient(null);
  }
  const ws = wsRef.current;

  // 换机器人 ⇒ 换订阅目标（影响帧过滤与命令里的 robot 字段）
  useEffect(() => {
    ws.setRobot(selected);
  }, [ws, selected]);

  // 建连 / 断开
  useEffect(() => {
    ws.connect();
    return () => ws.dispose();
  }, [ws]);

  // ★ 订阅客户端状态。
  //
  //   用 `useSyncExternalStore` 而不是 `useEffect + setState`：
  //   后者会在"订阅之前就已到达的帧"上丢一次更新（先连上、后挂监听），
  //   表现为首帧不显示。`useSyncExternalStore` 的 contract 恰好是
  //   "拿快照 + 订阅"两步，React 保证两者之间不会漏事件。
  //
  //   ⚠️⚠️ **只能有一个订阅**，快照必须是**一个引用稳定的对象**。
  //
  //   早先这里写了四个独立的 `useSyncExternalStore`（state / lastState /
  //   lastError / connectCount 各一个）。语法合法、单测也过，但它是错的：
  //   四个订阅器分别收敛，`notify()` 一次通知全部，而 React 对"快照没变"
  //   的那个会跳过重渲染 —— 字段之间的一致性只能靠运气。
  //   实测症状：**连接状态显示"未连接"，但面板/滑块全都正常渲染**。
  //   根因在 `frontend/src/ws.ts` 的 `getSnapshot()`（快照在 `notify()` 里
  //   统一重建），这里只需消费它。
  //
  //   `subscribe` 必须是稳定引用（`useCallback` 只依赖 `ws`），
  //   否则 React 会每次渲染都退订再订阅，白白丢掉中间的通知。
  const subscribeWs = useCallback((cb: () => void) => ws.subscribe(cb), [ws]);
  const getWsSnapshot = useCallback(() => ws.getSnapshot(), [ws]);
  const snap = useSyncExternalStore(subscribeWs, getWsSnapshot);

  const connectionState = snap.state;
  const lastFrame = snap.lastState;
  const lastErrorFrame = snap.lastError;
  const connectCount = snap.connectCount;

  // 状态帧只对**当前选中机器人**生效。
  // （后端可能同时推进多个机器人，别的机器人的帧不能画到我这儿。）
  const authoritative = useMemo(
    () =>
      lastFrame !== null && lastFrame.robot === selected ? lastFrame.joints : null,
    [lastFrame, selected]
  );

  // 拉机器人列表
  useEffect(() => {
    const ac = new AbortController();
    fetchRobots(ac.signal)
      .then((r) => {
        setRobots([...r.robots]);
        if (r.robots.length > 0 && selected === null) {
          setSelected(r.robots[0]!.id);
        }
        setLoading(false);
      })
      .catch((e: unknown) => {
        if (ac.signal.aborted) return;
        setError(e instanceof ApiError ? e.message : String(e));
        setLoading(false);
      });
    return () => ac.abort();
    // selected 刻意不入依赖：这条只在挂载时跑一次
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // 拉选中机器人的模型
  useEffect(() => {
    if (selected === null) return;
    const ac = new AbortController();
    setPayload(null);
    fetchRobotModel(selected, ac.signal)
      .then(setPayload)
      .catch((e: unknown) => {
        if (ac.signal.aborted) return;
        setError(e instanceof ApiError ? e.message : String(e));
      });
    return () => ac.abort();
  }, [selected]);

  const built = useMemo(
    () => (payload ? buildViewModel(payload.model) : null),
    [payload]
  );

  const vm = built?.viewModel ?? null;

  // -------------------------------------------------------------------------
  // 本地预演（ghost）
  //
  // ★ 它**不是**权威姿态的来源。它的作用是让拖动滑块时界面立刻有反应，
  //   而真正的姿态始终来自 `robot_state` 回帧（§49）。
  //   两条并存 —— 权威树画 actual，幽灵臂画 predicted。
  // -------------------------------------------------------------------------
  const ghost = useGhostPrediction({
    model: payload?.model ?? null,
    jointIds: vm?.movableJointIds ?? EMPTY_IDS,
    authoritative,
    enabled: showGhost,
  });

  // ★ 下发命令。
  //
  //   同时喂给**预演**与**WebSocket**：前者让界面立刻跟手，
  //   后者才是真正让机器人动的那一步。两者都做，顺序无所谓
  //   （预演只影响画什么，不影响真机）。
  //
  //   ⚠️ 这里**不做限幅** —— 限幅是后端职责（`clamp_targets_to_limits`）。
  //      前端也夹一次会让"命令超限"这条信息丢失（用户看到滑块停在边界，
  //      却不知道是自己拖超了还是模型只有这个范围）。
  //      预演内部会夹，但那是为了画出**合理的**预演姿态，不是为了改命令。
  const onCommand = useCallback(
    (targets: Readonly<Record<string, number>>): boolean => {
      ghost.pushCommand(targets);
      return ws.sendCommand(targets);
    },
    [ghost, ws]
  );

  const summary = robots.find((r) => r.id === selected) ?? null;

  if (loading) {
    return <div className="center">正在加载模型…</div>;
  }

  if (error !== null) {
    return (
      <div className="center error">
        <h2>无法加载</h2>
        <pre>{error}</pre>
        <p className="hint">
          后端是否已启动？在仓库根目录执行：
          <br />
          <code>.venv/Scripts/python.exe -m uvicorn backend.api.app:create_app --factory --port 8000</code>
        </p>
      </div>
    );
  }

  return (
    <div className="app">
      <aside className={showPanel ? "sidebar" : "sidebar collapsed"}>
        <h1>RobotForge <span className="ver">v0.1</span></h1>
        <p className="subtitle">
          RobotModel → Renderer Adapter → Three.js
        </p>

        <div className="panel">
          <h3>机器人</h3>
          <ul className="robots">
            {robots.map((r) => (
              <li
                key={r.id}
                className={r.id === selected ? "active" : ""}
                onClick={() => setSelected(r.id)}
              >
                {r.name} <code>{r.id}</code>
              </li>
            ))}
          </ul>
        </div>

        {summary && <CapabilityPanel caps={summary.capabilities} />}

        <ConnectionPanel
          state={connectionState}
          connectCount={connectCount}
          lastError={lastErrorFrame?.message ?? null}
          robotCount={robots.length}
        />

        {summary && vm && showJoints && (
          <JointPanel
            joints={vm.joints}
            actual={authoritative}
            status={lastFrame?.status ?? null}
            capabilities={summary.capabilities}
            connection={connectionState}
            onCommand={onCommand}
            lastError={lastErrorFrame?.message ?? null}
            predicted={ghost.prediction?.jointPositions ?? null}
          />
        )}

        <DiagnosticsPanel vm={built?.viewModel ?? null} problems={built?.problems ?? []} />

        {payload && !payload.validation.ok && (
          <div className="panel">
            <h3 className="bad">校验问题（{payload.validation.issues.length}）</h3>
            <ul>
              {payload.validation.issues.map((i, k) => (
                <li key={`${i.code}-${k}`}>
                  <code>{i.code}</code> @ {i.where}: {i.message}
                </li>
              ))}
            </ul>
          </div>
        )}
      </aside>

      <main className="viewport">
        <div className="toolbar">
          <button onClick={() => setShowPanel((v) => !v)}>
            {showPanel ? "隐藏面板" : "显示面板"}
          </button>
          <label>
            <input
              type="checkbox"
              checked={showAxes}
              onChange={(e) => setShowAxes(e.target.checked)}
            />
            关节轴
          </label>
          <label>
            <input
              type="checkbox"
              checked={showEe}
              onChange={(e) => setShowEe(e.target.checked)}
            />
            末端执行器
          </label>
          <label>
            <input
              type="checkbox"
              checked={showJoints}
              onChange={(e) => setShowJoints(e.target.checked)}
            />
            关节控制
          </label>
          <label>
            <input
              type="checkbox"
              checked={showGhost}
              onChange={(e) => setShowGhost(e.target.checked)}
            />
            本地预演
          </label>
          <span className="hint">左键拖拽=旋转 · 滚轮=缩放 · 右键拖拽=平移</span>
        </div>

        {built && built.viewModel.nodes.length > 1 ? (
          <Canvas
            camera={{ position: [0.35, 0.28, 0.35], fov: 50, near: 0.005, far: 50 }}
            shadows={false}
          >
            <color attach="background" args={["#0f1419"]} />
            <hemisphereLight args={["#cfd8dc", "#1b2126", 1.1]} />
            <directionalLight position={[2, 3, 2]} intensity={1.6} />
            <directionalLight position={[-2, 1, -2]} intensity={0.5} />

            <SceneRoot
              vm={built.viewModel}
              showAxes={showAxes}
              showEe={showEe}
              authoritative={authoritative}
              ghost={showGhost ? (ghost.prediction?.jointPositions ?? null) : null}
            />

            {/* 地面网格：让"哪边是上"一眼可见（Coordinate Frame 验收的一部分） */}
            <gridHelper args={[1, 20, "#37474f", "#232b31"]} />

            {/* Camera Orbit / Zoom / Pan 三项由 OrbitControls 一次给全 */}
            <OrbitControls
              enableDamping
              dampingFactor={0.08}
              minDistance={0.08}
              maxDistance={4}
              target={[0, 0.08, 0]}
            />
          </Canvas>
        ) : (
          <div className="center">模型没有可渲染的节点</div>
        )}
      </main>
    </div>
  );
}
