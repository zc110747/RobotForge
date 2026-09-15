/**
 * =============================================================================
 * ws.ts —— 后端 WebSocket 客户端（§52）
 * -----------------------------------------------------------------------------
 * §49 的**唯一**闭环里，这一层是前端的入口与出口：
 *
 * ```text
 * Frontend → WebSocket → RobotCommand → RobotRuntime → MuJoCoBackend
 *         → MuJoCo → RobotState → WebSocket → Frontend
 * ```
 *
 * ## 本文件刻意不做的事
 *
 * ```text
 * ❌ 不解释关节角怎么变成 Three.js 的旋转   → 那是 coordinateAdapter 的事（§41）
 * ❌ 不给控制指令做"看起来更顺"的预处理      → 命令原样发；限幅/限速是后端的事
 * ❌ 不把收到的 state 直接当权威画           → 由调用方（App）决定画在哪
 * ```
 *
 * 第二条尤其重要：前端若在发送前先夹一次，后端**也**会夹，
 * 于是"前端夹过了"变成一条谁都没在验的路径。而一旦它和
 * `clamp_targets_to_limits` 的语义有偏差（比如一边夹关节 range、
 * 一边夹 actuator ctrlrange），屏幕上看到的限位就不是后端认的那个。
 * ⇒ 前端**只**发用户给的目标值。
 *
 * ## 两处必须复现的后端行为
 *
 * ```text
 * ① §52 的"两个都收"：入站帧 type 可以是 `joint_command` 或 `robot_command`。
 *    发出去的时候用哪个？用 `robot_command` —— 它是 §52 "至少支持"清单里的
 *    规范名，而 `joint_command` 只出现在示例里。后端两个都收（见
 *    backend/api/websocket.py 的 INBOUND_TYPES），所以选规范名不会失配。
 *
 * ② 命令帧**不产生即时响应**（后端 `_handle_raw` 对命令返回 None）。
 *    状态由随后的一帧 `robot_state` 给出。这不是"后端忘了回"，
 *    而是刻意的：为每条命令回一帧"OK"会创造出一种没人在等的响应，
 *    多客户端时让"这条 state 是哪条命令的结果"变得不可判定。
 *    ⇒ 本客户端**不**实现"等 ACK"的语义；它只做"发命令"与"收状态"两件事。
 * ```
 *
 * ## 为什么手动做指数退避重连（而不是用库）
 *
 * 依赖为零是这个项目在前端侧的一贯做法（见 `tools/e2e_browser_check.mjs`
 * 的"零依赖 CDP"）。重连在浏览器里只是 `close` 事件 + `setTimeout`，
 * 引一个库反而多一层需要被验证的行为。**但退避的上限必须存在**：
 * 无限增长的间隔会让"后端起来了但前端还在等 30 秒"表现得像没连上。
 *
 * ## 断线时**不清空**最后一帧状态
 *
 * 这是有意的（与 `RobotState.stale_like` 同一条理由）：
 * 一次瞬时断开若把机器人画面清回原点，那是**信息丢失**而不是安全。
 * 调用方应当用 `status` 字段（`disconnected`）去显示提示，
 * 而不是让模型消失。
 */

import type { JointDTO } from "./viewer/model.ts";

// ---------------------------------------------------------------------------
// 线上类型（与 backend/api/websocket.py + runtime/state.py 逐字段对应）
// ---------------------------------------------------------------------------

/**
 * `robot_state` 帧。
 *
 * ⚠️ 字段名用的是**线上名**（`joints` / `velocities` / `end_effector`），
 *    不是 Python 侧的内部名（`joint_positions` / `joint_velocities` /
 *    `end_effector_pose`）。见 `RobotState.to_dict()`。
 */
export interface RobotStateFrame {
  readonly type: "robot_state";
  readonly robot: string;
  /** 关节 id → 位置（rad）。线上叫 `joints`。 */
  readonly joints: Readonly<Record<string, number>>;
  readonly velocities: Readonly<Record<string, number>>;
  /** `Transform.to_dict()` 或 `null`（`null` ≠ 单位位姿，见 runtime/state.py）。 */
  readonly end_effector: {
    readonly position: readonly [number, number, number];
    readonly orientation: readonly [number, number, number, number];
  } | null;
  /** `idle` / `running` / `reset` / `error`（后端 `STATUSES`，已收严）。 */
  readonly status: string;
  readonly timestamp: number;
}

export interface ErrorFrame {
  readonly type: "error";
  readonly code: string;
  readonly message: string;
  readonly [k: string]: unknown;
}

export interface SimulationStateFrame {
  readonly type: "simulation_state";
  readonly running: boolean;
  readonly robot_count: number;
  readonly backends: readonly {
    readonly robot: string;
    readonly backend: string;
    readonly is_simulation: boolean;
    readonly status: string;
  }[];
}

/** 后端不认识或还没处理的帧类型：保留原始对象，不打成 `any`。 */
export interface UnknownFrame {
  readonly type: string;
  readonly [k: string]: unknown;
}

export type InboundFrame =
  | RobotStateFrame
  | ErrorFrame
  | SimulationStateFrame
  | UnknownFrame;

/** 连接状态。`disconnected` **不是** `RobotState.status` 的取值 —— 那一层是后端概念。 */
export type ConnectionState = "connecting" | "open" | "closed";

export interface WsClientOptions {
  /** WebSocket 路径。默认 `/ws`（由 vite 代理到后端，见 vite.config.ts）。 */
  readonly path?: string;
  /** 首次重连延迟（ms）。之后按 `backoffFactor` 指数增长。 */
  readonly reconnectDelayMs?: number;
  /** 退避上限（ms）。**必须存在** —— 见文件头。 */
  readonly maxReconnectDelayMs?: number;
  readonly backoffFactor?: number;
  /** 供测试注入。默认用全局 `WebSocket`。 */
  readonly socketFactory?: (url: string) => WebSocketLike;
  /** 供测试注入（重连计时）。默认 `setTimeout`。 */
  readonly timer?: (fn: () => void, ms: number) => unknown;
}

/** 本客户端用到的 `WebSocket` 子集 —— 便于在 Node 里喂养一个假 socket。 */
export interface WebSocketLike {
  readyState: number;
  send(data: string): void;
  close(code?: number, reason?: string): void;
  addEventListener(type: string, listener: (ev: unknown) => void): void;
  removeEventListener?(type: string, listener: (ev: unknown) => void): void;
}

const WS_OPEN = 1;

const DEFAULT_PATH = "/ws";
const DEFAULT_RECONNECT_DELAY = 500;
const DEFAULT_MAX_RECONNECT_DELAY = 8000;
const DEFAULT_BACKOFF_FACTOR = 2;

// ---------------------------------------------------------------------------
// 命令（出站）
// ---------------------------------------------------------------------------

/**
 * 构造一条 `robot_command` 帧。
 *
 * `joints` 是**线上字段名**（§52 示例）。后端也接受内部名 `joint_targets`，
 * 但发出去只用一个名字 —— 两个都发会让"哪个生效"取决于后端的取值顺序。
 *
 * ⚠️ 这里**不**夹紧、**不**补全未提及的关节：
 * ```text
 * 不夹紧 —— 限幅是后端的职责（clamp_targets_to_limits）。前端夹一次会让
 *           "前端夹过了"成为一条没人在验的路径，且一旦与后端语义有偏差
 *           （关节 range vs actuator ctrlrange），屏幕上的限位就不是后端认的。
 * 不补全 —— 后端对未提及的关节按"保持原位"处理（MockBackend 实测）；
 *           前端补一批 0 会把"我只想动一个关节"变成"把其余的都回零"。
 * ```
 */
export function buildRobotCommand(
  robot: string,
  jointTargets: Readonly<Record<string, number>>,
  timestamp = 0
): { type: "robot_command"; robot: string; joints: Record<string, number>; timestamp: number } {
  return {
    type: "robot_command",
    robot,
    joints: { ...jointTargets },
    timestamp,
  };
}

/**
 * 从模型里挑出**可以发命令**的关节 id。
 *
 * 判据全部来自模型，不写死名字（§69 规则 2）：
 * ```text
 * type === "revolute" | "prismatic"  —— fixed 不能动，free 在 v0.1 没有 UI
 * ```
 * `fixed` 必须显式排除：Core FK 里它是恒等变换，把它列进滑块会得到一个
 * 拖了没反应的控件，而那是"看起来坏了"的观感。
 */
export function commandableJoints(joints: readonly JointDTO[]): readonly JointDTO[] {
  return joints.filter((j) => j.type === "revolute" || j.type === "prismatic");
}

// ---------------------------------------------------------------------------
// 客户端
// ---------------------------------------------------------------------------

export interface WsClient {
  readonly state: ConnectionState;
  /** 最近一次收到的状态帧；**断线后不清空**（见文件头）。 */
  readonly lastState: RobotStateFrame | null;
  /** 最近一次错误帧。 */
  readonly lastError: ErrorFrame | null;
  /** 已成功建连次数（用于区分"首次连上"与"重连成功"）。 */
  readonly connectCount: number;
  sendCommand(jointTargets: Readonly<Record<string, number>>): boolean;
  /** 主动关闭（**不再**自动重连）。 */
  dispose(): void;
  /** 订阅任一变更（帧到达 / 连接状态变化）。返回取消订阅函数。 */
  subscribe(fn: () => void): () => void;
  /** 开始连接。刻意不自动执行 —— 何时开连由调用方决定。 */
  connect(): void;
  /** 切换目标机器人：影响后续帧的过滤与命令的 `robot` 字段。 */
  setRobot(next: string | null): void;
}

/**
 * 建一个 WS 客户端。**不立即连接** —— 调用方在 `useEffect` 里显式 `connect()`，
 * 这样"何时开始连"是调用方的决定，便于测试时控制时序。
 */
export function createWsClient(robot: string | null, options: WsClientOptions = {}): WsClient {
  const path = options.path ?? DEFAULT_PATH;
  const baseDelay = options.reconnectDelayMs ?? DEFAULT_RECONNECT_DELAY;
  const maxDelay = options.maxReconnectDelayMs ?? DEFAULT_MAX_RECONNECT_DELAY;
  const factor = options.backoffFactor ?? DEFAULT_BACKOFF_FACTOR;
  const makeSocket =
    options.socketFactory ??
    ((url: string) => new WebSocket(url) as unknown as WebSocketLike);
  const timer = options.timer ?? ((fn, ms) => setTimeout(fn, ms));

  let socket: WebSocketLike | null = null;
  let state: ConnectionState = "closed";
  let lastState: RobotStateFrame | null = null;
  let lastError: ErrorFrame | null = null;
  let connectCount = 0;
  let delay = baseDelay;
  let disposed = false;
  let robotId = robot;
  const listeners = new Set<() => void>();

  function notify(): void {
    for (const fn of listeners) fn();
  }

  function url(): string {
    // 用页面 origin，不硬编码 host/port —— 与 api.ts 的相对路径同一理由。
    // ws/wss 由页面协议推导（https 页面必须用 wss，否则浏览器直接拒绝）。
    const loc = globalThis.location;
    if (!loc) return `ws://127.0.0.1${path}`;
    const proto = loc.protocol === "https:" ? "wss:" : "ws:";
    return `${proto}//${loc.host}${path}`;
  }

  function setState(next: ConnectionState): void {
    if (state === next) return;
    state = next;
    notify();
  }

  function handleFrame(raw: unknown): void {
    let frame: InboundFrame;
    try {
      // ⚠️ 用**严格** JSON.parse：它会拒绝 NaN/Infinity（RFC 8259），
      //    而后端也用 parse_constant 拒绝了 —— 两侧必须一致，
      //    否则"服务端接受、前端解析失败"是一个只在真机上出现的故障。
      frame = JSON.parse(String(raw)) as InboundFrame;
    } catch {
      // 帧不是 JSON：不动 lastState（保留最后已知的好数据）
      lastError = { type: "error", code: "bad_frame", message: "收到无法解析的帧" };
      notify();
      return;
    }

    // 只关心**属于本机器人**的状态帧。多机器人时若不过滤，
    // 另一台的状态会覆盖这一台 —— 画面会跳到别的机器人位形上。
    if (frame.type === "robot_state") {
      const st = frame as RobotStateFrame;
      if (robotId !== null && st.robot !== robotId) return;
      lastState = st;
      lastError = null;
      notify();
      return;
    }

    if (frame.type === "error") {
      lastError = frame as ErrorFrame;
      notify();
      return;
    }

    // simulation_state / robot_info：本层不解释，交给需要的人。
    notify();
  }

  function connect(): void {
    if (disposed) return;
    if (socket !== null) return; // 已有一根在连/已连
    setState("connecting");

    let ws: WebSocketLike;
    try {
      ws = makeSocket(url());
    } catch {
      scheduleReconnect();
      return;
    }
    socket = ws;

    ws.addEventListener("open", () => {
      connectCount += 1;
      delay = baseDelay; // 连上就复位退避 —— 否则一次抖动会让下次重连等很久
      setState("open");
      notify();
    });

    ws.addEventListener("message", (ev) => {
      const data = (ev as { data?: unknown })?.data;
      if (data !== undefined) handleFrame(data);
    });

    ws.addEventListener("error", () => {
      // 不在这里 setState("closed")：`error` 之后浏览器**总会**再给
      // 一个 `close`。在这里改状态会让"重连被调两次"（error 一次、
      // close 一次），而第二次 connect 会被 `socket !== null` 挡掉 ——
      // 表现为"断线后再也连不上"。⇒ 只在 close 里处理。
    });

    ws.addEventListener("close", () => {
      socket = null;
      setState("closed");
      scheduleReconnect();
    });
  }

  function scheduleReconnect(): void {
    if (disposed) return;
    const wait = delay;
    delay = Math.min(delay * factor, maxDelay);
    timer(() => {
      if (!disposed) connect();
    }, wait);
  }

  return {
    get state() {
      return state;
    },
    get lastState() {
      return lastState;
    },
    get lastError() {
      return lastError;
    },
    get connectCount() {
      return connectCount;
    },
    connect,
    sendCommand(jointTargets) {
      if (socket === null || socket.readyState !== WS_OPEN) return false;
      if (robotId === null) return false;
      socket.send(JSON.stringify(buildRobotCommand(robotId, jointTargets)));
      return true;
    },
    dispose() {
      disposed = true;
      const s = socket;
      socket = null;
      setState("closed");
      if (s) {
        try {
          s.close(1000, "client dispose");
        } catch {
          /* 已关闭 */
        }
      }
    },
    subscribe(fn) {
      listeners.add(fn);
      return () => {
        listeners.delete(fn);
      };
    },
    /** 机器人切换：只影响**后续**帧的过滤与命令的 robot 字段。 */
    setRobot(next: string | null) {
      robotId = next;
    },
  };
}
