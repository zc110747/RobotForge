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

import { useEffect, useMemo, useState } from "react";
import type { ReactNode } from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls } from "@react-three/drei";

import { ApiError, fetchRobotModel, fetchRobots } from "./api.ts";
import type { RobotModelResponse, RobotSummaryDTO } from "./viewer/model.ts";
import { buildViewModel } from "./viewer/viewModel.ts";
import type { RobotViewModel } from "./viewer/viewModel.ts";
import { RobotScene } from "./viewer/RobotScene.tsx";
import { SCENE_ROTATION_X } from "./viewer/coordinateAdapter.ts";

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
        Phase 4 将按这些开关显示命令面板 —— 此处先声明能力已在链路中。
      </p>
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
}: {
  vm: RobotViewModel;
  showAxes: boolean;
  showEe: boolean;
}): ReactNode {
  // ★ 整场景旋转：这是 `coordinateAdapter` 的唯一出口。
  //   施加在一个包住整个机器人的 <group> 上，**不**逐节点转换
  //   （理由见 docs/coordinate-system.md §4.2 与 adapter 的模块注释）。
  //
  //   ⚠️ 不要在 <Canvas> 上设 `rotation` —— Canvas 接受的是相机/渲染参数，
  //      不是场景变换。旋转必须落在一个 group（或 scene）对象上。
  return (
    <group rotation={[SCENE_ROTATION_X, 0, 0]} name="threejs-adapter-root">
      <RobotScene vm={vm} showAxes={showAxes} showEndEffector={showEe} />
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

            <SceneRoot vm={built.viewModel} showAxes={showAxes} showEe={showEe} />

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
