/**
 * ws.ts 的自检。
 *
 * 运行：
 * ```bash
 * node --experimental-strip-types frontend/src/ws.check.ts
 * ```
 *
 * ## 为什么这个模块需要自检
 *
 * 它的行为**不能靠肉眼看页面**验证：
 * ```text
 * ① 重连退避：画面只会显示"已连接"，看不到"退避到第几档"；
 * ② 帧过滤：多机器人时收到别人的 state，画面会跳到别的位形 ——
 *    那看起来像"模型错了"，而不像"过滤漏了"；
 * ③ error 事件 vs close 事件：浏览器**总是**先 error 再 close。
 *    在 error 里就重连会让重连被调两次，第二次被 `socket !== null`
 *    挡掉 ⇒ 表现为"断线后再也连不上"，而这是**间歇性**的。
 * ```
 *
 * ⇒ 用一个假 socket 精确控制"什么时候 open / message / error / close"，
 *   把上面三条变成可断言的。**这不需要真实后端。**
 */

import { createWsClient, buildRobotCommand, commandableJoints } from "./ws.ts";
import type { WebSocketLike } from "./ws.ts";
import type { JointDTO } from "./viewer/model.ts";

// =============================================================================
// 断言器（与其它 check 脚本同风格）
// =============================================================================

interface Failure {
  readonly check: string;
  readonly detail: string;
}

interface Checker {
  readonly failures: Failure[];
  ok(cond: boolean, check: string, detail?: string): void;
  eq<T>(actual: T, expected: T, check: string): void;
  close(actual: number, expected: number, tol: number, check: string): void;
}

function newChecker(): Checker {
  const failures: Failure[] = [];
  return {
    failures,
    // `detail` 可选：只有一段"为什么"值得写时才写，避免每条都塞占位文本
    ok(cond, check, detail = "") {
      if (!cond) failures.push({ check, detail });
    },
    eq(actual, expected, check) {
      if (actual !== expected) {
        failures.push({ check, detail: `期望 ${JSON.stringify(expected)}，实得 ${JSON.stringify(actual)}` });
      }
    },
    close(actual, expected, tol, check) {
      const diff = Math.abs(actual - expected);
      if (!(diff <= tol)) {
        failures.push({ check, detail: `期望 ${expected}，实得 ${actual}，差 ${diff}` });
      }
    },
  };
}

// =============================================================================
// 假 socket + 假时钟
// =============================================================================

class FakeSocket implements WebSocketLike {
  readyState = 0;
  readonly sent: string[] = [];
  closed = false;
  readonly url: string;
  private readonly handlers = new Map<string, Array<(ev: unknown) => void>>();

  // ⚠️ 不能写成 `constructor(readonly url: string)`（TS 的**参数属性**）：
  //    `node --experimental-strip-types` 只做类型**剥离**，不做代码生成，
  //    而参数属性需要生成 `this.url = url`。Node 会直接报
  //    `ERR_UNSUPPORTED_TYPESCRIPT_SYNTAX`。手写赋值即可。
  constructor(url: string) {
    this.url = url;
  }

  send(data: string): void {
    if (this.readyState !== 1) throw new Error("send on non-open socket");
    this.sent.push(data);
  }

  close(): void {
    this.closed = true;
    this.readyState = 3;
  }

  addEventListener(type: string, listener: (ev: unknown) => void): void {
    const arr = this.handlers.get(type) ?? [];
    arr.push(listener);
    this.handlers.set(type, arr);
  }

  emit(type: string, ev: unknown = {}): void {
    if (type === "open") this.readyState = 1;
    if (type === "close") this.readyState = 3;
    for (const fn of this.handlers.get(type) ?? []) fn(ev);
  }
}

/** 假时钟：手动推进，断言退避序列。**不用真实 sleep**（否则测试会变慢且不确定）。 */
function fakeClock() {
  const pending: Array<{ fn: () => void; ms: number }> = [];
  return {
    timer: (fn: () => void, ms: number) => {
      pending.push({ fn, ms });
      return pending.length - 1;
    },
    /** 取出并执行**最早**一个待执行项，返回它的延迟。 */
    tick(): number {
      const item = pending.shift();
      if (!item) return -1;
      item.fn();
      return item.ms;
    },
    get size(): number {
      return pending.length;
    },
  };
}

// =============================================================================
// 检查 1：命令帧的构造（不夹紧、不补全）
// =============================================================================

function runBuildCommand(c: Checker): void {
  const frame = buildRobotCommand("mini_arm", { shoulder: 0.4 }, 1.5);
  c.eq(frame.type, "robot_command", "命令帧 type = robot_command（§52 规范名）");
  c.eq(frame.robot, "mini_arm", "命令帧带 robot 字段");
  c.close(frame.joints["shoulder"] ?? NaN, 0.4, 0, "命令帧保留目标值");
  c.eq(Object.keys(frame.joints).length, 1, "只发被提及的关节（不补全其余关节）");

  // ★ 前端**不**夹紧：超限值必须原样发出去，让后端去夹。
  //   若这里夹了，"前端夹过了"就成了一条没人在验的路径。
  const over = buildRobotCommand("mini_arm", { shoulder: 99.0 });
  c.close(over.joints["shoulder"] ?? NaN, 99.0, 0, "超限目标原样发出（限幅是后端的职责）");

  // ★ 拷贝而非引用：调用方后续改自己的对象不该影响已发出的帧
  const src: Record<string, number> = { elbow: 0.2 };
  const f2 = buildRobotCommand("mini_arm", src);
  src["elbow"] = 9.9;
  c.close(f2.joints["elbow"] ?? NaN, 0.2, 0, "命令帧是一次快照（改源对象不影响已构造的帧）");
}

// =============================================================================
// 检查 2：可命令关节的挑选（fixed 必须排除）
// =============================================================================

function runCommandableJoints(c: Checker): void {
  const mk = (id: string, type: string): JointDTO => ({
    id,
    name: id,
    type,
    parent_link: "l0",
    child_link: "l1",
    origin: { position: [0, 0, 0], orientation: [0, 0, 0, 1] },
    axis: [0, 0, 1],
    limits: null,
  });
  const joints = [
    mk("base_yaw", "revolute"),
    mk("slider", "prismatic"),
    mk("welded", "fixed"),
    mk("ball", "free"),
  ];
  const got = commandableJoints(joints).map((j) => j.id);
  c.eq(got.join(","), "base_yaw,slider", "只挑 revolute + prismatic（fixed/free 排除）");
  // 反向断言：`fixed` 若混进来，"拖了没反应"会被当成 bug
  c.ok(!got.includes("welded"), "fixed 关节不出现在滑块里", `实得 ${got.join(",")}`);
}

// =============================================================================
// 检查 3：连接 → 发命令 → 收状态
// =============================================================================

function runRoundTrip(c: Checker): void {
  const clock = fakeClock();
  const sockets: FakeSocket[] = [];
  const client = createWsClient("mini_arm", {
    socketFactory: (url) => {
      const s = new FakeSocket(url);
      sockets.push(s);
      return s;
    },
    timer: clock.timer,
  });

  c.eq(client.state, "closed", "建了但没 connect ⇒ 状态是 closed（不自动开连）");
  client.connect();
  c.eq(client.state, "connecting", "connect() 后是 connecting");
  c.eq(sockets.length, 1, "只建了一根 socket");

  const s0 = sockets[0]!;
  c.ok(s0.url.startsWith("ws://") || s0.url.startsWith("wss:"), "URL 用 ws/wss 协议", s0.url);
  c.ok(s0.url.endsWith("/ws"), "URL 指向 /ws 路径", s0.url);

  s0.emit("open");
  c.eq(client.state, "open", "open 事件后是 open");
  c.eq(client.connectCount, 1, "connectCount = 1");

  // ★ 未 open 时发命令必须**失败且不抛**（否则一次误点会炸掉整个 UI）
  const s1 = sockets[0]!;
  c.ok(client.sendCommand({ shoulder: 0.1 }) === true, "open 后 sendCommand 返回 true");
  c.eq(s1.sent.length, 1, "确实发出去了一帧");

  const parsed = JSON.parse(s1.sent[0]!) as { type: string; robot: string; joints: Record<string, number> };
  c.eq(parsed.type, "robot_command", "线上 type 是 robot_command");
  c.eq(parsed.robot, "mini_arm", "线上带 robot");

  // 收一帧状态
  s0.emit("message", {
    data: JSON.stringify({
      type: "robot_state",
      robot: "mini_arm",
      joints: { base_yaw: 0, shoulder: 0.3, elbow: 0.1 },
      velocities: { base_yaw: 0, shoulder: 0, elbow: 0 },
      end_effector: null,
      status: "running",
      timestamp: 0.02,
    }),
  });
  c.ok(client.lastState !== null, "收到了 lastState");
  c.close(client.lastState?.joints["shoulder"] ?? NaN, 0.3, 0, "lastState 的关节角正确");
  c.eq(client.lastState?.status, "running", "lastState.status 透传");

  client.dispose();
  c.eq(client.state, "closed", "dispose 后是 closed");
  c.eq(clock.size, 0, "dispose 后**不再**排下一次重连（否则页面卸载后还在重连）");
}

// =============================================================================
// 检查 3b：getSnapshot —— React 订阅契约（引用稳定性 + 同进同出）
// =============================================================================

/**
 * ## 这一节为什么必须存在
 *
 * `getSnapshot()` 是 `useSyncExternalStore` 的契约点，它有**两条**硬要求，
 * 而违反任一条**都不会抛错**：
 *
 * ```text
 * ① 引用稳定 —— 状态没变时必须返回**同一个对象**。
 *    每调用一次就现造一个 ⇒ React 认为"每次都变了" ⇒ 无限重渲染。
 *
 * ② 变化后必须**整个换掉** —— 不能只换其中几个字段。
 *    若四个字段各用一个 useSyncExternalStore，React 会对"快照没变"的
 *    那个跳过重渲染，字段之间的一致性就只能靠运气。
 *    实测症状：连接状态显示"未连接"，但面板/滑块全都正常渲染。
 * ```
 *
 * ⇒ 判据落在两条上：**同一状态连续取 = 同一引用**、
 *   **一次 notify = 快照整体换新且四个字段同进同出**。
 */
function runSnapshot(c: Checker): void {
  const clock = fakeClock();
  const sockets: FakeSocket[] = [];
  const client = createWsClient("mini_arm", {
    socketFactory: (url) => {
      const s = new FakeSocket(url);
      sockets.push(s);
      return s;
    },
    timer: clock.timer,
  });

  // ① 未连接时连续取两次 —— 必须是同一个对象
  const s0 = client.getSnapshot();
  const s0b = client.getSnapshot();
  c.ok(s0 === s0b, "同一状态下连续 getSnapshot 返回**同一引用**（否则 React 无限重渲染）",
       `两次取到 ${s0 === s0b ? "同一对象" : "不同对象"}`);
  c.eq(s0.state, "closed", "初始快照 state = closed");
  c.eq(s0.lastState, null, "初始快照 lastState = null");
  c.eq(s0.robot, "mini_arm", "初始快照带 robot（命令的 robot 字段来源）");

  // ② connect 是状态变化 ⇒ 必须换新快照
  client.connect();
  const s1 = client.getSnapshot();
  c.ok(s1 !== s0, "连接状态变化 ⇒ 快照换新", "引用没变 ⇒ 订阅者收不到变化");
  c.eq(s1.state, "connecting", "新快照的 state = connecting");
  c.ok(client.getSnapshot() === s1, "变化后连续取仍是同一引用", "又现造了新对象");

  // ③ open + 收帧：state / lastState / connectCount **三个字段一起**更新
  //
  //    ★ 这是这条判据的核心：它们必须在**同一个**快照里是新的。
  //      若拆成三个独立的 useSyncExternalStore，就可能出现
  //      "state 更新了但 lastState 还是旧的" —— 恰好是那个实测症状。
  sockets[0]!.emit("open");
  sockets[0]!.emit("message", {
    data: JSON.stringify({
      type: "robot_state",
      robot: "mini_arm",
      joints: { shoulder: 0.42 },
      velocities: { shoulder: 0 },
      end_effector: null,
      status: "running",
      timestamp: 0.02,
    }),
  });

  const s2 = client.getSnapshot();
  c.ok(s2 !== s1, "收到帧 ⇒ 快照换新");
  c.eq(s2.state, "open", "同一快照里 state 已更新为 open");
  c.eq(s2.connectCount, 1, "同一快照里 connectCount 已更新为 1");
  c.close(s2.lastState?.joints["shoulder"] ?? NaN, 0.42, 0, "同一快照里 lastState 已更新");
  c.eq(
    [s2.state, s2.connectCount, s2.lastState !== null].join(","),
    "open,1,true",
    "★ 三个字段**同进同出**（不是分三次各更新一个）"
  );

  // ④ 切换机器人也要换快照（否则快照里的 robot 陈旧，
  //    而它恰好被用来判断"这帧是不是本机器人的"）
  const s3 = client.getSnapshot();
  client.setRobot("other_arm");
  const s4 = client.getSnapshot();
  c.ok(s4 !== s3, "setRobot ⇒ 快照换新", "robot 字段陈旧会导致帧过滤用错机器人");
  c.eq(s4.robot, "other_arm", "新快照的 robot 已更新");

  // ⑤ 同一个机器人再设一次 ⇒ 不该换快照（值是幂等的）
  const s5 = client.getSnapshot();
  client.setRobot("other_arm");
  c.ok(client.getSnapshot() === s5, "setRobot 到**同一个**值 ⇒ 不换快照（幂等）",
       "每次调用都换 ⇒ 浪费重渲染");

  client.dispose();
}

// =============================================================================
// 检查 4：未 open 时发命令返回 false
// =============================================================================

function runSendBeforeOpen(c: Checker): void {
  const clock = fakeClock();
  const sockets: FakeSocket[] = [];
  const client = createWsClient("mini_arm", {
    socketFactory: (url) => {
      const s = new FakeSocket(url);
      sockets.push(s);
      return s;
    },
    timer: clock.timer,
  });
  c.ok(client.sendCommand({ shoulder: 1 }) === false, "没 connect 时 sendCommand 返回 false（不抛）");
  client.connect();
  c.ok(client.sendCommand({ shoulder: 1 }) === false, "connecting 时 sendCommand 返回 false");
  c.eq(sockets[0]!.sent.length, 0, "确实没发出去");
  client.dispose();
}

// =============================================================================
// 检查 5：重连退避 —— 指数增长 + 上限
// =============================================================================

function runBackoff(c: Checker): void {
  const clock = fakeClock();
  const sockets: FakeSocket[] = [];
  const client = createWsClient("mini_arm", {
    socketFactory: (url) => {
      const s = new FakeSocket(url);
      sockets.push(s);
      return s;
    },
    timer: clock.timer,
    reconnectDelayMs: 100,
    maxReconnectDelayMs: 400,
    backoffFactor: 2,
  });

  client.connect();
  const delays: number[] = [];
  // 连续断开 5 次：100 → 200 → 400 → 400（封顶）
  for (let i = 0; i < 5; i++) {
    sockets[sockets.length - 1]!.emit("close");
    const d = clock.tick();
    if (d >= 0) delays.push(d);
  }
  c.eq(delays.join(","), "100,200,400,400,400", "退避序列 100/200/400/400/400（封顶后不再增长）");

  // ★ 上限**必须**存在：无限增长的间隔会让"后端起来了但前端还在等"
  //   表现得像没连上。这条断言就是钉住它。
  c.ok(Math.max(...delays) <= 400, "退避不超过 maxReconnectDelayMs", `实测最大 ${Math.max(...delays)}`);

  // 连上后复位：下一次断开的延迟必须回到起点
  sockets[sockets.length - 1]!.emit("open");
  sockets[sockets.length - 1]!.emit("close");
  c.eq(clock.tick(), 100, "连上后退避复位（下次断线从基础延迟重来）");

  client.dispose();
}

// =============================================================================
// 检查 6：error 事件**不得**触发重连（必须在 close 里处理）
// =============================================================================

function runErrorEvent(c: Checker): void {
  const clock = fakeClock();
  const sockets: FakeSocket[] = [];
  const client = createWsClient("mini_arm", {
    socketFactory: (url) => {
      const s = new FakeSocket(url);
      sockets.push(s);
      return s;
    },
    timer: clock.timer,
  });
  client.connect();
  sockets[0]!.emit("open");

  sockets[0]!.emit("error");
  c.eq(sockets.length, 1, "error 事件不新建 socket");
  c.eq(clock.size, 0, "error 事件不排重连（只在 close 里排）");
  c.eq(client.state, "open", "error 事件不改连接状态（close 还没来）");

  // 浏览器随后给 close —— 这一次才该重连，且**只有一次**
  sockets[0]!.emit("close");
  c.eq(clock.size, 1, "close 排了**恰好一次**重连（不是两次）");
  clock.tick();
  c.eq(sockets.length, 2, "重连建了第二根 socket");

  client.dispose();
}

// =============================================================================
// 检查 7：多机器人时过滤掉别人的 state
// =============================================================================

function runFrameFiltering(c: Checker): void {
  const clock = fakeClock();
  const sockets: FakeSocket[] = [];
  const client = createWsClient("mini_arm", {
    socketFactory: (url) => {
      const s = new FakeSocket(url);
      sockets.push(s);
      return s;
    },
    timer: clock.timer,
  });
  client.connect();
  sockets[0]!.emit("open");

  sockets[0]!.emit("message", {
    data: JSON.stringify({
      type: "robot_state",
      robot: "other_robot",
      joints: { shoulder: 9.9 },
      velocities: {},
      end_effector: null,
      status: "running",
      timestamp: 1,
    }),
  });
  c.ok(client.lastState === null, "别人的 robot_state 被丢弃（不过滤会让画面跳到别的位形）");

  sockets[0]!.emit("message", {
    data: JSON.stringify({
      type: "robot_state",
      robot: "mini_arm",
      joints: { shoulder: 0.2 },
      velocities: {},
      end_effector: null,
      status: "running",
      timestamp: 2,
    }),
  });
  c.close(client.lastState?.joints["shoulder"] ?? NaN, 0.2, 0, "自己的 robot_state 被采纳");

  // 非 JSON 帧：不得污染 lastState
  sockets[0]!.emit("message", { data: "{ 这不是 JSON" });
  c.close(client.lastState?.joints["shoulder"] ?? NaN, 0.2, 0, "坏帧不清空 lastState（保留最后已知好数据）");
  c.ok(client.lastError !== null, "坏帧记入 lastError");
  c.eq(client.lastError?.code, "bad_frame", "坏帧的 code = bad_frame");

  // 好帧应清掉 lastError
  sockets[0]!.emit("message", {
    data: JSON.stringify({
      type: "robot_state",
      robot: "mini_arm",
      joints: { shoulder: 0.5 },
      velocities: {},
      end_effector: null,
      status: "running",
      timestamp: 3,
    }),
  });
  c.ok(client.lastError === null, "收到好帧后 lastError 被清空");

  client.dispose();
}

// =============================================================================
// 检查 8：断线**不清空** lastState
// =============================================================================

function runStaleOnDisconnect(c: Checker): void {
  const clock = fakeClock();
  const sockets: FakeSocket[] = [];
  const client = createWsClient("mini_arm", {
    socketFactory: (url) => {
      const s = new FakeSocket(url);
      sockets.push(s);
      return s;
    },
    timer: clock.timer,
  });
  client.connect();
  sockets[0]!.emit("open");
  sockets[0]!.emit("message", {
    data: JSON.stringify({
      type: "robot_state",
      robot: "mini_arm",
      joints: { shoulder: 0.77 },
      velocities: {},
      end_effector: null,
      status: "running",
      timestamp: 1,
    }),
  });
  sockets[0]!.emit("close");

  c.eq(client.state, "closed", "断线后状态是 closed");
  c.close(
    client.lastState?.joints["shoulder"] ?? NaN,
    0.77,
    0,
    "断线后仍保留最后一帧（一次瞬时断开不该把机器人清回原点）"
  );
  client.dispose();
}

// =============================================================================
// 检查 9：dispose 之后不再重连
// =============================================================================

function runDisposeStopsReconnect(c: Checker): void {
  const clock = fakeClock();
  const sockets: FakeSocket[] = [];
  const client = createWsClient("mini_arm", {
    socketFactory: (url) => {
      const s = new FakeSocket(url);
      sockets.push(s);
      return s;
    },
    timer: clock.timer,
  });
  client.connect();
  sockets[0]!.emit("close"); // 排了一次重连
  client.dispose(); // 然后卸载
  // ⚠️ 这里**不能**断言 `clock.size === 0`：定时器是在 dispose **之前**
  //    被排上的，dispose 没有（也不该）去取消一个已经登记的 timer ——
  //    它做的是让回调里的 `connect()` 变成 no-op。断言必须落在
  //    **可观测的后果**上（没有新建 socket），而不是"队列空了"。
  clock.tick(); // 执行那个已登记的回调
  c.eq(sockets.length, 1, "dispose 后已登记的重连回调不新建 socket（disposed 守卫生效）");
  c.ok(client.sendCommand({ shoulder: 1 }) === false, "dispose 后 sendCommand 返回 false");
}

// =============================================================================
// 自检：故意注入缺陷，断言必须变红
// =============================================================================

interface SelfTest {
  readonly name: string;
  readonly run: () => boolean;
}

const SELF_TESTS: SelfTest[] = [
  {
    // 模拟"退避上限被去掉"⇒ 退避序列判据必须发现
    name: "退避无上限 ⇒ 序列判据变红",
    run: () => {
      const clock = fakeClock();
      const sockets: FakeSocket[] = [];
      const client = createWsClient("mini_arm", {
        socketFactory: (url) => {
          const s = new FakeSocket(url);
          sockets.push(s);
          return s;
        },
        timer: clock.timer,
        reconnectDelayMs: 100,
        // 故意给一个**很大**的上限（模拟"忘了设上限"）
        maxReconnectDelayMs: 1_000_000,
        backoffFactor: 2,
      });
      client.connect();
      const delays: number[] = [];
      for (let i = 0; i < 5; i++) {
        sockets[sockets.length - 1]!.emit("close");
        const d = clock.tick();
        if (d >= 0) delays.push(d);
      }
      const c = newChecker();
      c.eq(delays.join(","), "100,200,400,400,400", "tampered backoff");
      client.dispose();
      return c.failures.length > 0;
    },
  },
  {
    // 模拟"别人机器人的 state 不过滤"⇒ 过滤判据必须发现
    name: "不过滤别人的 state ⇒ 过滤判据变红",
    run: () => {
      const clock = fakeClock();
      const sockets: FakeSocket[] = [];
      const client = createWsClient("mini_arm", {
        socketFactory: (url) => {
          const s = new FakeSocket(url);
          sockets.push(s);
          return s;
        },
        timer: clock.timer,
      });
      client.connect();
      sockets[0]!.emit("open");
      sockets[0]!.emit("message", {
        data: JSON.stringify({
          type: "robot_state",
          robot: "other_robot",
          joints: { shoulder: 9.9 },
          velocities: {},
          end_effector: null,
          status: "running",
          timestamp: 1,
        }),
      });
      const c = newChecker();
      // 篡改期望：断言"采纳了别人的值"（错误期望）—— 判据应发现实得 null
      c.close(client.lastState?.joints["shoulder"] ?? NaN, 9.9, 0, "tampered filter");
      client.dispose();
      return c.failures.length > 0;
    },
  },
  {
    // 模拟"命令被前端夹紧了"⇒ 原样发出判据必须发现
    name: "前端擅自夹紧命令 ⇒ 原样发出判据变红",
    run: () => {
      const c = newChecker();
      // 模拟一个"会夹紧"的实现
      const tampered = buildRobotCommand("mini_arm", { shoulder: Math.min(99.0, 1.5708) });
      c.close(tampered.joints["shoulder"] ?? NaN, 99.0, 0, "tampered clamp");
      return c.failures.length > 0;
    },
  },
  {
    // 模拟"fixed 关节混进可命令列表"⇒ 挑选判据必须发现
    name: "fixed 混进滑块 ⇒ 关节挑选判据变红",
    run: () => {
      const c = newChecker();
      const only = commandableJoints([
        { id: "welded", name: "welded", type: "fixed", parent_link: "a", child_link: "b",
          origin: { position: [0, 0, 0], orientation: [0, 0, 0, 1] }, axis: [0, 0, 1], limits: null },
      ]).map((j) => j.id);
      // 篡改期望：断言"包含 welded"（错误期望）
      c.ok(only.includes("welded"), "tampered fixed filter");
      return c.failures.length > 0;
    },
  },
  {
    // 模拟"dispose 后仍排重连"⇒ 不再重连判据必须发现
    name: "dispose 后仍重连 ⇒ 不重连判据变红",
    run: () => {
      const clock = fakeClock();
      const sockets: FakeSocket[] = [];
      const client = createWsClient("mini_arm", {
        socketFactory: (url) => {
          const s = new FakeSocket(url);
          sockets.push(s);
          return s;
        },
        timer: clock.timer,
      });
      client.connect();
      sockets[0]!.emit("close");
      // **不** dispose（模拟"卸载时忘了 dispose"）
      const c = newChecker();
      // 篡改期望：断言"队列里没有待执行的重连"（错误期望）。
      // 没 dispose 时 close 已经排了一次 ⇒ 实得 1 ⇒ 判据必须变红。
      c.eq(clock.size, 0, "tampered dispose");
      client.dispose();
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
  runBuildCommand(c);
  runCommandableJoints(c);
  runRoundTrip(c);
  runSnapshot(c);
  runSendBeforeOpen(c);
  runBackoff(c);
  runErrorEvent(c);
  runFrameFiltering(c);
  runStaleOnDisconnect(c);
  runDisposeStopsReconnect(c);

  const self = newChecker();
  runSelfTests(self);

  console.log("");
  console.log("WebSocket 客户端自检（ws.ts）");
  console.log("─".repeat(62));
  console.log("被测：命令构造 / 关节挑选 / 连接 / 退避 / 帧过滤 / 断线保留 / 卸载");
  console.log("      getSnapshot 契约（引用稳定 + 字段同进同出）");
  console.log("假体：FakeSocket + 假时钟（无真实网络、无真实 sleep）");
  console.log("─".repeat(62));

  const failures = [...c.failures];
  const selfFailures = [...self.failures];

  if (failures.length === 0 && selfFailures.length === 0) {
    console.log("PASS  全部检查通过");
    console.log(`      自检 ${SELF_TESTS.length}/${SELF_TESTS.length} 注入均被捕获`);
    return 0;
  }
  if (failures.length > 0) {
    console.log(`FAIL  ${failures.length} 条检查失败：`);
    for (const f of failures) {
      console.log(`  ✗ ${f.check}`);
      console.log(`      ${f.detail}`);
    }
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
  process.argv[1] !== undefined &&
  process.argv[1].replace(/\\/g, "/").endsWith("ws.check.ts");
if (isDirect) {
  process.exit(main());
}

export { main, newChecker };
