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

/**
 * 客户端状态的**不可变快照**，供 `useSyncExternalStore` 用。
 *
 * ## 为什么必须由客户端自己缓存
 *
 * `useSyncExternalStore` 用 `Object.is` 比较前后两次 `getSnapshot()` 的返回值，
 * 决定要不要重渲染。若 `getSnapshot` 每次现造一个对象，它会**每次都认为变了**
 * ⇒ 无限重渲染（React 会直接抛 "The result of getSnapshot should be cached"）。
 *
 * 若换成"每个字段各用一个 useSyncExternalStore"，返回的是 primitives，
 * 比较没问题 —— 但那样就有 N 个订阅器，而 `notify()` 是**一次**通知所有
 * 订阅器的；React 在同一个 tick 里逐个收敛，遇到"快照没变"的那个会跳过
 * 重渲染。**字段之间的一致性就只能靠运气**（实测踩到：连接状态更新了，
 * 但取帧的那个订阅器没跟着重渲染，界面显示"未连接"）。
 *
 * ⇒ 唯一的正确做法：快照是**一个**对象，且在 `notify()` 里**重建一次**。
 *   这样"有变化"这件事有唯一真值源，四个字段永远同进同出。
 */
export interface WsSnapshot {
  readonly state: ConnectionState;
  readonly lastState: RobotStateFrame | null;
  readonly lastError: ErrorFrame | null;
  readonly connectCount: number;
  /** 目标机器人（命令的 `robot` 字段 / 帧过滤依据）。 */
  readonly robot: string | null;
}

export interface WsClient {
  readonly state: ConnectionState;
  /** 最近一次收到的状态帧；**断线后不清空**（见文件头）。 */
  readonly lastState: RobotStateFrame | null;
  /** 最近一次错误帧。 */
  readonly lastError: ErrorFrame | null;
  /** 已成功建连次数（用于区分"首次连上"与"重连成功"）。 */
  readonly connectCount: number;
  /**
   * 取当前快照。**引用稳定**：同一状态连续调用返回同一对象。
   *
   * 它专为 `useSyncExternalStore` 准备 —— 见 `WsSnapshot` 的说明。
   */
  getSnapshot(): WsSnapshot;
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
  /**
   * 连接**代际号**：每次 `connect()` 自增，用来给这一轮的所有回调发身份证。
   *
   * 为什么需要它（而不是只看 `disposed`）：
   * `disposed` 是个布尔，只能回答"**现在**这一轮是不是被主动关了"。
   * 它回答不了"**这个回调**属于哪一轮"。而 React StrictMode 的
   * `mount → cleanup → mount` 会让旧轮次的回调（旧 socket 的 close、
   * 旧定时器的重连）在**新一轮已经开着**的时候才被触发 —— 此时
   * `disposed === false`，只判布尔的守卫会放行，于是：
   * - 旧 socket 的 close 会把刚建好的 `socket` 置 `null`（误杀新连接）；
   * - 旧定时器会再调一次 `connect()`，造出"两根并存的 socket"。
   * 两条都会表现成"连不上"。代际号让这些回调直接自废。
   */
  let generation = 0;
  const listeners = new Set<() => void>();

  /**
   * 缓存的快照。**只在 `notify()` 里重建**，这样"有变化"这件事
   * 有唯一的真值源 —— 见 `WsSnapshot` 的说明。
   *
   * 初始值手动构造一次（`notify()` 还没跑过，但 `getSnapshot()` 必须可调）。
   */
  let snapshot: WsSnapshot = {
    state,
    lastState: null,
    lastError: null,
    connectCount: 0,
    robot: robotId,
  };

  function notify(): void {
    // ★ 先重建快照，**再**通知订阅者。
    //   反过来的话，订阅者在回调里（或 React 收敛时）读到的还是旧快照 ——
    //   表现成"通知到了但值没变"，React 会跳过重渲染，界面停在旧状态。
    snapshot = {
      state,
      lastState,
      lastError,
      connectCount,
      robot: robotId,
    };
    for (const fn of listeners) fn();
  }

  function getSnapshot(): WsSnapshot {
    return snapshot;
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

  /**
   * 开始连接。刻意不自动执行 —— 何时开连由调用方决定。
   *
   * ## ★ `connect()` 会**解除** `dispose()` 的状态
   *
   * 这一条是被 React StrictMode 逼出来的，且不写清楚就会被后人"优化"掉：
   *
   * ```text
   * <React.StrictMode> 在开发模式下把 effect 跑成交错的两轮：
   *   mount  → effect 跑（connect）
   *   unmount→ cleanup 跑（dispose）
   *   mount  → effect 再跑（connect）
   * ```
   *
   * 若 `dispose()` 把 `disposed` 永久置真、而 `connect()` 在开头
   * `if (disposed) return`，那么第三次调用（第二次 mount）会被挡掉 ——
   * **整个页面永远连不上**。
   *
   * 症状与"服务器拒绝连接"一模一样（面板显示"未连接"），
   * 但浏览器控制台里那句真话是：
   * ```text
   * WebSocket connection to 'ws://…/ws' failed:
   *   WebSocket is closed before the connection is established.
   * ```
   * —— 即"我们自己把它关了"。而 `disposed` 是个**内部布尔**，
   * 从外面（面板、状态、错误帧）完全看不出来。
   * 实测：在页面里手动 `new WebSocket(...)` 是**成功**的，
   * 这恰好排除了"地址/代理/后端"三个方向的怀疑。
   *
   * ⇒ `disposed` 的含义收窄为"**当前**这一轮连接已被主动关闭"，
   *   而不是"这个客户端对象作废了"。要作废一个客户端，
   *   由调用方把它换掉（`useRef` 里换新对象）即可。
   */
  function connect(): void {
    // ★ 清除"已主动关闭"标记（见上）。
    disposed = false;
    if (socket !== null) return; // 已有一根在连/已连
    // ⚠️ 这里**不能**复位 `delay`。
    //
    //   退避的意义是"连续失败时越来越慢"；若每次尝试都从 baseDelay 重来，
    //   就退化成"断网后每 100ms 硬撞一次" —— 既打满 CPU 又刷满日志。
    //   复位只发生在**真的连上了**（见 `open` 事件里的 `delay = baseDelay`）。
    //   自检 `runBackoff` 就是靠这条抓到的（期望 100,200,400,400,400，
    //   实得 100,100,100,100,100）。
    setState("connecting");

    // ★ 代际号：这一轮连接尝试的身份证。
    //
    //   为什么需要它（而不是只看 `disposed`）：
    //   `dispose()` 之后可能**又** `connect()`（React StrictMode 的
    //   mount→cleanup→mount，见上面的长注释）。此时旧的那些回调
    //   （旧 socket 的 close、旧 timer 的重连）必须**失效**，
    //   否则会出现"两次 mount ⇒ 两根并存的 socket"，而 `socket !== null`
    //   这条守卫只挡得住其中一半。
    //
    //   `disposed` 只能回答"现在是关的还是开的"，回答不了
    //   "这个回调属于哪一轮"。代际号才能。
    const gen = ++generation;

    let ws: WebSocketLike;
    try {
      ws = makeSocket(url());
    } catch {
      scheduleReconnect(gen);
      return;
    }
    socket = ws;

    ws.addEventListener("open", () => {
      if (gen !== generation) return; // 属于上一轮的 socket，丢弃
      connectCount += 1;
      delay = baseDelay; // 连上就复位退避 —— 否则一次抖动会让下次重连等很久
      setState("open");
      notify();
    });

    ws.addEventListener("message", (ev) => {
      if (gen !== generation) return;
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
      if (gen !== generation) return; // 旧轮次的 close，不排重连
      socket = null;
      setState("closed");
      scheduleReconnect(gen);
    });
  }

  /**
   * 排一次重连。
   *
   * @param gen 发起这次排期的**代际号**。定时器到期时代际若不匹配，
   *            说明期间发生过 `dispose()` 或新一轮 `connect()` ⇒ 放弃这次重连。
   *            只判 `disposed` 是不够的（见 `connect` 里的说明）。
   */
  function scheduleReconnect(gen: number): void {
    if (disposed) return;
    const wait = delay;
    delay = Math.min(delay * factor, maxDelay);
    timer(() => {
      // 两道守卫都要：
      // - `disposed` 挡"这期间被主动关了"；
      // - `gen === generation` 挡"这期间已经开了新一轮"（旧定时器不得再开一个）。
      if (!disposed && gen === generation) connect();
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
      // ★ 递增代际：让所有**在飞**的回调（旧 socket 的 open/message/close、
      //   旧定时器的重连）立即失效。只置 `disposed` 不够 —— 若随后又
      //   `connect()`（StrictMode 的第二次 mount），`disposed` 会被复位成
      //   false，那些旧回调就"复活"了。
      generation += 1;
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
      if (robotId === next) return;
      robotId = next;
      // 要 notify：`snapshot.robot` 也是 UI 会读的字段
      // （链路面板显示"订阅机器人"）。不通知的话快照里的 robot 会陈旧，
      // 而陈旧的那个值恰好被用来判断"这帧是不是本机器人的"。
      notify();
    },
    getSnapshot,
  };
}
