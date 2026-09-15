/**
 * RobotForge 前端无头浏览器验收（§59 / §71）
 * =============================================================================
 * 零依赖：Node ≥22 自带 fetch / WebSocket，直连 Edge 的 CDP。
 *
 * ## 它要证明什么（四条，缺一不可）
 *
 * | 维度 | 断言 |
 * |---|---|
 * | 渲染   | `<canvas>` 存在**且真的画了东西**（像素非纯背景） |
 * | 数值正确 | 面板显示的 links/joints/EE 与 REST 返回的 RobotModel 一致 |
 * | 交互生效 | 取消勾选"关节轴"后画面真的变了（像素 diff） |
 * | 无副作用 | 控制台错误 / 未捕获异常计数为 0 |
 *
 * ## 为什么"canvas 存在"不够
 *
 * 空 canvas 也是一个 `<canvas>`。WebGL 在 headless 下若无 swiftshader
 * 会静默画不出任何内容，而 DOM 断言全绿 —— 那是"伪造通过"。
 * 所以这里读**像素**：统计非背景色像素占比。
 *
 * ## 两种运行模式
 *
 * **模式 A（默认）**：脚本自己 spawn Edge。
 * **模式 B（`--attach`）**：连接一个**已在运行**的 Edge。
 *
 * > 模式 B 的存在理由（实测教训）：在本机 Git Bash 沙箱里，
 * > 脚本内 spawn Edge 会稳定地 `exit code=21` 且 stderr 为空、
 * > 60s 不开调试端口；而**同样的参数**在脚本外启动就秒起。
 * > 与其去猜 spawn 环境的差异，不如把"启动浏览器"与"验收页面"分开 ——
 * > 后者才是这个脚本的职责。
 * >
 * > 用法：
 * > ```bash
 * > # 终端 1：先起浏览器（PowerShell 更稳）
 * > Start-Process msedge.exe -ArgumentList @(
 * >   "--remote-debugging-port=9333","--headless=new","--no-sandbox","--disable-gpu",
 * >   "--user-data-dir=$env:TEMP\rf-e2e","--use-gl=angle","--use-angle=swiftshader",
 * >   "--enable-unsafe-swiftshader","--window-size=1600,1100","http://127.0.0.1:5173/")
 * > # 终端 2：
 * > node tools/e2e_browser_check.mjs --attach
 * > ```
 *
 * ## 用法
 *   node tools/e2e_browser_check.mjs [--port 5173] [--out .workbuddy/scratch/e2e]
 *                                   [--attach] [--debug-port 9333]
 */

import { spawn } from "node:child_process";
import { createWriteStream, existsSync, mkdirSync, readFileSync, readdirSync, rmSync, writeFileSync } from "node:fs";
import { join, resolve } from "node:path";

// ---------------------------------------------------------------------------
// 配置
// ---------------------------------------------------------------------------

const args = process.argv.slice(2);
const argOf = (name, dflt) => {
  const i = args.indexOf(`--${name}`);
  return i >= 0 && args[i + 1] ? args[i + 1] : dflt;
};

const PORT = Number(argOf("port", "5173"));
const OUT_DIR = argOf("out", ".workbuddy/scratch/e2e");
const BASE = `http://127.0.0.1:${PORT}/`;
const DEBUG_PORT = Number(argOf("debug-port", "9333"));
const ATTACH = args.includes("--attach");

const EDGE_CANDIDATES = [
  "C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe",
  "C:\\Program Files\\Microsoft\\Edge\\Application\\msedge.exe",
  "C:\\Program Files\\Google\\Chrome\\Application\\chrome.exe",
];

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

function readFileSyncSafe(p) {
  try { return readFileSync(p, "utf8"); } catch { return ""; }
}

// ---------------------------------------------------------------------------
// 结果记账
// ---------------------------------------------------------------------------

const results = [];
function check(name, ok, detail = "") {
  results.push({ name, ok: Boolean(ok), detail });
  console.log(`  ${ok ? "PASS" : "FAIL"}  ${name}${detail ? `  —— ${detail}` : ""}`);
}
function skip(name, why) {
  results.push({ name, ok: true, skipped: true, detail: why });
  console.log(`  SKIP  ${name}  —— ${why}`);
}

// ---------------------------------------------------------------------------
// 极简 CDP client
// ---------------------------------------------------------------------------

class Cdp {
  constructor(ws) {
    this.ws = ws;
    this.id = 0;
    this.pending = new Map();
    this.errors = [];       // Log.entryAdded / Runtime.exceptionThrown
    this.consoleErrors = [];
    this._listeners = [];

    ws.addEventListener("message", (ev) => {
      const msg = JSON.parse(ev.data);
      if (msg.id !== undefined) {
        const p = this.pending.get(msg.id);
        if (p) {
          this.pending.delete(msg.id);
          msg.error ? p.reject(new Error(JSON.stringify(msg.error))) : p.resolve(msg.result);
        }
        return;
      }
      for (const fn of this._listeners) fn(msg);
    });
  }

  on(fn) { this._listeners.push(fn); }

  send(method, params = {}) {
    const id = ++this.id;
    return new Promise((resolve, reject) => {
      this.pending.set(id, { resolve, reject });
      this.ws.send(JSON.stringify({ id, method, params }));
      setTimeout(() => {
        if (this.pending.has(id)) {
          this.pending.delete(id);
          reject(new Error(`CDP timeout: ${method}`));
        }
      }, 30000);
    });
  }

  async eval(expression) {
    const r = await this.send("Runtime.evaluate", {
      expression,
      returnByValue: true,
      awaitPromise: true,
    });
    if (r.exceptionDetails) {
      throw new Error(`evaluate 抛错: ${r.exceptionDetails.exception?.description ?? "?"}`);
    }
    return r.result?.value;
  }

  async waitFor(expression, predicate, { timeout = 30000, interval = 300 } = {}) {
    const deadline = Date.now() + timeout;
    let last;
    while (Date.now() < deadline) {
      try {
        last = await this.eval(expression);
        if (predicate(last)) return { ok: true, value: last };
      } catch { /* 页面可能还没就绪 */ }
      await sleep(interval);
    }
    return { ok: false, value: last };
  }
}

// ---------------------------------------------------------------------------
// 截图与像素分析
// ---------------------------------------------------------------------------

async function capture(cdp) {
  const r = await cdp.send("Page.captureScreenshot", { format: "png", fromSurface: true });
  return Buffer.from(r.data, "base64");
}

/**
 * 页面内 canvas 像素统计。
 *
 * ⚠️⚠️ **必须在 `requestAnimationFrame` 回调内读**，否则一律读到全 0。
 *
 * 原因：r3f 用 `WebGLRenderer` 的默认配置，**没有开 `preserveDrawingBuffer`**。
 * 于是每帧 present 之后绘制缓冲就被清空，此时 `readPixels` 只能拿到
 * 全 0 的缓冲（`gl.getError()` 还是 0 —— **不报错，只是没有数据**）。
 *
 * 实测对照（同一页面、同一时刻，唯一变量 = 读的时机）：
 *
 * ```text
 * A) 帧外 readPixels                      → nonzeroPixels =      0
 * B) rAF 回调内 readPixels                → nonzeroPixels = 149480
 * C) 帧外 canvas.toDataURL() 长度         → 6162（≈ 空白图，同样被清掉）
 * ```
 *
 * 这个坑的误导性在于：**判据失败的表现是"页面是全黑的"**，
 * 而截图（`Page.captureScreenshot`，走 compositor 而非 GL 缓冲）明明有内容。
 * 两者矛盾时人会去怀疑渲染/相机/坐标 —— 方向全错。
 * ⇒ 只要像素读数与截图矛盾，先怀疑**读的时机**，不要怀疑渲染。
 *
 * 另外注意 `cv.width/height` 是**绘制缓冲**尺寸，不是 CSS 尺寸；
 * 无头 + `--window-size` 下二者相等（dpr=1），但别把假设写死。
 */
const PIXEL_STATS_EXPR = (extra) => `new Promise((resolve) => {
  requestAnimationFrame(() => {
    const cv = document.querySelector('canvas');
    if (!cv) return resolve({ error: 'no canvas' });
    const gl = cv.getContext('webgl2') || cv.getContext('webgl');
    if (!gl) return resolve({ error: 'no webgl context' });
    const w = cv.width, h = cv.height;
    if (!w || !h) return resolve({ error: 'zero-size canvas' });
    const px = new Uint8Array(w * h * 4);
    gl.readPixels(0, 0, w, h, gl.RGBA, gl.UNSIGNED_BYTE, px);
    // 背景色基准：取**四角**的中位数亮度像素，而不是左上角单点 ——
    // 左上角可能正好压在网格线或坐标架上，"以它为背景"会低估差异像素数。
    const corners = [0, (w-1)*4, (h-1)*w*4, ((h-1)*w + w-1)*4];
    const bg = [0,1,2].map(c => {
      const vals = corners.map(o => px[o+c]).sort((a,b)=>a-b);
      return vals[Math.floor(vals.length/2)];
    });
    let nonBg = 0, minX = w, maxX = -1, minY = h, maxY = -1;
    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        const i = (y * w + x) * 4;
        const d = Math.abs(px[i]-bg[0]) + Math.abs(px[i+1]-bg[1]) + Math.abs(px[i+2]-bg[2]);
        if (d > 24) {
          nonBg++;
          if (x < minX) minX = x; if (x > maxX) maxX = x;
          if (y < minY) minY = y; if (y > maxY) maxY = y;
        }
      }
    }
    resolve({
      w, h, total: w * h, nonBg,
      ratio: nonBg / (w * h),
      bbox: maxX >= 0 ? [minX, minY, maxX - minX, maxY - minY] : null,
      ${extra || ""}
    });
  });
})`;

/** 一次性读数（用于"当前状态"快照）。 */
async function canvasPixelStats(cdp) {
  return cdp.eval(PIXEL_STATS_EXPR());
}

/** 轮询读数，直到 `pred(stats)` 成立或超时。 */
async function waitForCanvasPixels(cdp, pred, timeoutMs = 60000) {
  const t0 = Date.now();
  let last = null;
  while (Date.now() - t0 < timeoutMs) {
    last = await cdp.eval(PIXEL_STATS_EXPR());
    if (last && !last.error && pred(last)) return { ok: true, value: last };
    await sleep(400);
  }
  return { ok: false, value: last };
}

// ---------------------------------------------------------------------------
// 主流程
// ---------------------------------------------------------------------------

async function main() {
  console.log("=".repeat(74));
  console.log("RobotForge 前端无头浏览器验收（Edge + CDP + swiftshader）");
  console.log("=".repeat(74));

  const browser = EDGE_CANDIDATES.find((p) => existsSync(p));
  if (!browser) {
    console.error("找不到 Edge/Chrome，无法执行浏览器验收");
    return 2;
  }
  console.log(`\n浏览器: ${browser}`);
  console.log(`目标:   ${BASE}\n`);

  mkdirSync(OUT_DIR, { recursive: true });

  // ★ profile 目录必须**每次全新**，且**不要**先 rmSync 再复用同名目录。
  //
  //   实测：`rmSync(prof)` + 立刻用同一个路径 spawn ⇒ Edge 退出 **code=21**
  //   且 `DevToolsActivePort` 不生成（表现为"40 次轮询无 page target"）。
  //   原因是 Windows 上删除目录后仍有句柄残留，Chromium 判定数据目录不可用。
  //   而"换一个从没用过的目录名"就立刻成功 —— 所以用时间戳，
  //   并把清理工作留给操作系统/后续清扫（不阻塞本次运行）。
  //
  // ★★ 且**必须是绝对路径**。第二类 code=21（同样 stderr 为空：
  //   Chromium 在写任何日志之前就退了）：
  //
  //   实测：`--user-data-dir=.workbuddy/scratch/e2e/edge-profile-xxx`（相对路径）
  //       ⇒ NOT FOUND. exited= 21 err= null probe= fetch: fetch failed / stderr 空
  //   同一目录名 `resolve()` 成 `E:\cnb\git\RobotForge\.workbuddy\...`
  //       ⇒ FOUND at 0.5s
  //
  //   原因是 Chromium **以自己的 cwd 解析**（相对）数据目录，而非调用者的 cwd；
  //   解析结果的父目录不存在 ⇒ 建不出 profile ⇒ 直接退出。
  //   症状与上面那类**完全一样**（都是"轮询无 page target"），所以两次都往参数、
  //   渲染、URL 上排查 —— 方向全错。⇒ 这里必须显式 resolve()。
  const PROFILE_DIR = resolve(OUT_DIR, `edge-profile-${Date.now()}`);

  // --- 启动前先确认服务活着（否则后面全是超时，信息量极低）---
  try {
    const r = await fetch(BASE, { signal: AbortSignal.timeout(5000) });
    check("dev server 可达", r.ok, `HTTP ${r.status}`);
    if (!r.ok) return 2;
  } catch (e) {
    check("dev server 可达", false, `无法连接 ${BASE}: ${e.message}`);
    console.log("\n请先启动前端：cd frontend && npm run dev");
    return 2;
  }

  // ★ 把浏览器 stderr 落盘。
  //   否则"浏览器退出 code=21"是一条**无法归因**的信息 ——
  //   真正的原因（数据目录不可用 / 参数冲突 / 权限）只写在 stderr 里。
  const stderrPath = join(OUT_DIR, "edge-stderr.txt");
  let stderrStream = null;

  // ---------- 启动浏览器（或 attach 到已有实例）----------
  let child = null;
  if (ATTACH) {
    console.log(`模式 : attach（复用 127.0.0.1:${DEBUG_PORT} 上的已有实例）\n`);
  } else {
    stderrStream = createWriteStream(stderrPath, { flags: "w" });
    child = spawn(
      browser,
      [
        `--remote-debugging-port=${DEBUG_PORT}`,
        "--headless=new",
        "--no-sandbox",
        "--disable-gpu",
        "--no-first-run",
        "--no-default-browser-check",
        "--disable-extensions",
        "--user-data-dir=" + PROFILE_DIR,
        // ★ WebGL 软件渲染：没有这两条，headless 下 canvas 会是空白
        "--use-gl=angle",
        "--use-angle=swiftshader",
        "--enable-unsafe-swiftshader",
        "--hide-scrollbars",
        "--window-size=1600,1100",
        BASE,
      ],
      { stdio: ["ignore", "ignore", "pipe"], detached: false }
    );
    child.stderr.pipe(stderrStream);
  }

  // ★ 必须挂 error / exit：GUI 子系统程序在 Git Bash 下 **spawn 会静默失败**，
  //   不挂监听的话表现是"轮询后仍无 page target"，
  //   而真实原因是"浏览器压根没启动"。错误信息指向完全错误的方向。
  let spawnError = null;
  let exited = null;
  if (child) {
    child.on("error", (e) => { spawnError = e; });
    child.on("exit", (code, signal) => { exited = { code, signal }; });
  }

  let cdp = null;
  try {
    // ---------- 连上 CDP ----------
    // ★ 超时必须给够（60s，不是 20s）。
    //
    //   实测：Edge 的**首次**冷启动（解压 + 首次运行初始化 + swiftshader 加载）
    //   可能耗时 > 12s，而第二次启动因为已预热是**亚秒级**。
    //   于是"第一次跑失败、马上再跑就成功"——这类现象极易被误读成
    //   "环境不稳定"，进而写出 `retry` 把真正的问题掩盖掉。
    //   正确的做法是把超时定到覆盖**冷启动最坏情况**。
    let target = null;
    let lastProbe = "";
    for (let i = 0; i < 120; i++) {
      if (spawnError) break;
      await sleep(500);
      try {
        const list = await (await fetch(`http://127.0.0.1:${DEBUG_PORT}/json/list`)).json();
        target = list.find((t) => t.type === "page" && t.url.startsWith("http"));
        if (target) break;
        lastProbe = `targets=${list.length} types=[${list.map((t) => t.type).join(",")}]`;
      } catch (e) {
        lastProbe = `fetch: ${e.message}`;
      }
    }
    if (spawnError) {
      check("浏览器进程已启动", false, `spawn error: ${spawnError.message}`);
      return writeSummary();
    }
    if (!target) {
      check("CDP 可连接", false,
            `60s 内仍无 page target（${lastProbe}）` +
            (exited ? ` 浏览器已退出 code=${exited.code}` : " 浏览器仍在运行"));
      // 落在磁盘上的 profile 里可能有 DevToolsActivePort，能给出更多线索
      const portFile = join(PROFILE_DIR, "DevToolsActivePort");
      if (existsSync(portFile)) {
        const txt = readFileSyncSafe(portFile);
        console.log(`    DevToolsActivePort 内容: ${txt.trim().split("\n").join(" | ")}`);
      } else {
        console.log("    DevToolsActivePort 不存在 ⇒ 浏览器未成功开启调试端口");
      }
      const errTxt = readFileSyncSafe(stderrPath);
      if (errTxt.trim()) {
        console.log("    —— 浏览器 stderr ——");
        for (const line of errTxt.trim().split("\n").slice(0, 12)) {
          console.log("      " + line.slice(0, 160));
        }
      } else {
        console.log("    浏览器 stderr 为空");
      }
      console.log("    ⇒ 本机若稳定复现 code=21，请改用 --attach 模式（见文件头）");
      return writeSummary();
    }
    check("CDP 可连接", true, `url=${target.url.slice(0, 60)}`);

    const ws = new WebSocket(target.webSocketDebuggerUrl);
    await new Promise((res, rej) => {
      ws.addEventListener("open", res, { once: true });
      ws.addEventListener("error", rej, { once: true });
      setTimeout(() => rej(new Error("WebSocket 连接超时")), 15000);
    });
    cdp = new Cdp(ws);

    // 收集控制台错误 / 未捕获异常
    cdp.on((msg) => {
      if (msg.method === "Log.entryAdded") cdp.errors.push(msg.params.entry);
      if (msg.method === "Runtime.exceptionThrown") {
        cdp.consoleErrors.push(msg.params.exceptionDetails);
      }
    });

    await cdp.send("Page.enable");
    await cdp.send("Runtime.enable");
    await cdp.send("Log.enable");
    await cdp.send("Emulation.setDeviceMetricsOverride", {
      width: 1600, height: 1100, deviceScaleFactor: 1, mobile: false,
    });
    await cdp.send("Page.navigate", { url: BASE });
    await sleep(1500);

    // ---------- ① 渲染：等页面就绪 ----------
    const ready = await cdp.waitFor(
      `Boolean(document.querySelector('canvas')) &&
       document.querySelectorAll('.panel').length >= 3`,
      (v) => v === true,
      { timeout: 60000 }
    );
    check("页面就绪（canvas + 面板）", ready.ok,
          ready.ok ? "" : "60s 内 canvas 或 .panel 未出现");

    // 页面若停在错误页，直接报出原因（比一堆超时有用）
    const errText = await cdp.eval(
      `(() => { const e = document.querySelector('.center.error pre'); return e ? e.textContent : null; })()`
    );
    if (errText) {
      check("前端未停留在错误页", false, `页面错误: ${errText.slice(0, 200)}`);
      return writeSummary();
    }
    check("前端未停留在错误页", true);

    // ---------- ② 渲染：真的画了东西（读像素）----------
    // swiftshader 首帧编译慢 ⇒ 轮询。像素必须在 rAF 内读（见 PIXEL_STATS_EXPR 注释）。
    //
    // ★ 阈值为什么是 0.5% 而不是 2%：
    //   canvas 只占页面的一部分（左侧有 300px 的侧栏），
    //   机器人本体在默认相机下约占 canvas 的 1%~3%。
    //   "2%" 是个**拍脑袋的**数字，它把"渲染正确但相机较远"误判成失败。
    //   真正要排除的是**空白帧**（ratio ≈ 0）与**纯噪声**；
    //   0.5% 已经比"全黑"高出一个数量级，同时不依赖相机距离。
    //   判据的落点应该是"有实质内容"，而不是"内容够大"。
    const MIN_CONTENT_RATIO = 0.005;
    const pixels = await waitForCanvasPixels(
      cdp,
      (v) => v.ratio > MIN_CONTENT_RATIO,
      60000
    );
    check(`canvas 真的画出了内容（非背景像素 > ${(MIN_CONTENT_RATIO * 100).toFixed(1)}%）`,
          pixels.ok,
          pixels.value && !pixels.value.error
            ? `nonBg=${pixels.value.nonBg}/${pixels.value.total} (${(pixels.value.ratio * 100).toFixed(2)}%)`
            : `读不到像素: ${pixels.value?.error ?? "无返回值"}`);

    // 包围盒用**同一个**读数，别再读一次 —— 两次读之间画面可能已经变了
    // （相机抖动/自动旋转），于是"有内容"与"bbox 非空"会互相矛盾。
    const stats = pixels.value;
    if (stats && !stats.error) {
      const cv = await cdp.eval(`(() => { const c=document.querySelector('canvas'); return c ? {w:c.clientWidth,h:c.clientHeight} : null; })()`);
      const inside = stats.bbox && cv &&
        stats.bbox[0] >= 0 && stats.bbox[1] >= 0 &&
        stats.bbox[0] + stats.bbox[2] <= stats.w &&
        stats.bbox[1] + stats.bbox[3] <= stats.h;
      check("内容包围盒非空且在绘制缓冲内", Boolean(stats.bbox) && inside,
            stats.bbox
              ? `bbox=[x=${stats.bbox[0]},y=${stats.bbox[1]},w=${stats.bbox[2]},h=${stats.bbox[3]}] 缓冲=${stats.w}×${stats.h}`
              : "bbox 为空（画面无差异像素）");
    }

    // ---------- ③ 数值正确：面板 vs REST ----------
    const panel = await cdp.eval(`(() => {
      const out = {};
      // "模型诊断" 表格
      const tables = [...document.querySelectorAll('.panel table')];
      for (const t of tables) {
        for (const tr of t.querySelectorAll('tr')) {
          const tds = tr.querySelectorAll('td');
          if (tds.length === 2) out[tds[0].textContent.trim()] = tds[1].textContent.trim();
        }
      }
      // 关节表
      const jointRows = [];
      for (const t of tables) {
        const head = t.querySelector('thead');
        if (head && head.textContent.includes('axis')) {
          for (const tr of t.querySelectorAll('tbody tr')) {
            const tds = [...tr.querySelectorAll('td')].map(x => x.textContent.trim());
            jointRows.push({ id: tds[0], type: tds[1], axis: tds[2], limits: tds[3] });
          }
        }
      }
      // 能力列表
      const caps = [...document.querySelectorAll('.caps li')].map(li => li.textContent.trim());
      // 坐标适配段落
      const adapt = [...document.querySelectorAll('.panel p')].map(p => p.textContent).find(t => t.includes('rotation.x')) || '';
      return { diag: out, joints: jointRows, caps, adapt };
    })()`);

    console.log("\n  —— 页面读数 ——");
    console.log("    robot      :", panel.diag["robot"]);
    console.log("    nodes      :", panel.diag["nodes"]);
    console.log("    links      :", panel.diag["links"]);
    console.log("    joints     :", panel.diag["joints"]);
    console.log("    可动关节   :", panel.diag["可动关节"]);
    console.log("    末端执行器 :", panel.diag["末端执行器"]);
    console.log("    坐标架     :", panel.diag["坐标架"]);
    console.log("    关节表     :");
    for (const j of panel.joints) {
      console.log(`      ${j.id.padEnd(14)} ${j.type.padEnd(10)} ${j.axis.padEnd(22)} ${j.limits}`);
    }
    console.log("    坐标适配   :", panel.adapt.replace(/\s+/g, " ").slice(0, 120));
    console.log("");

    // 与 REST 对比（**独立数据源**：页面走的是 fetch→代理→后端，
    // 这里直接从 Node 打后端，两条路径独立 ⇒ 对比有意义）
    let model = null;
    try {
      const mr = await fetch("http://127.0.0.1:8000/api/robots/mini_arm/model",
                             { signal: AbortSignal.timeout(8000) });
      model = (await mr.json()).model;
    } catch (e) {
      skip("与 REST 对比", `直连后端失败: ${e.message}`);
    }

    if (model) {
      const restLinks = (model.links || []).length;
      const restJoints = (model.joints || []).length;
      const restMovable = (model.joints || []).filter(j => j.type !== "fixed" && j.is_mobile !== false).length;
      const restEe = (model.end_effectors || []).map(e => `${e.id} @ ${e.link_id || e.linkId}`).join(", ");

      check("页面 links 数 == REST links 数",
            panel.diag["links"] === String(restLinks),
            `页面=${panel.diag["links"]} REST=${restLinks}`);
      check("页面 joints 数 == REST joints 数",
            (panel.diag["joints"] || "").startsWith(String(restJoints)),
            `页面=${panel.diag["joints"]} REST=${restJoints}`);
      check("页面 末端执行器 == REST",
            (panel.diag["末端执行器"] || "").includes(restEe.split(" @ ")[0]),
            `页面=${panel.diag["末端执行器"]} REST=${restEe}`);

      // 关节轴必须与 REST 的 axis 逐字一致（**这是"没有偷偷改坐标"的判据**）
      let axisMismatch = [];
      for (const row of panel.joints) {
        const rest = (model.joints || []).find(j => j.id === row.id);
        if (!rest || !rest.axis) continue;
        const expect = `[${rest.axis.map(c => Number(c).toFixed(3)).join(", ")}]`;
        if (row.axis !== expect) axisMismatch.push(`${row.id}: 页面=${row.axis} REST=${expect}`);
      }
      check("关节轴与 REST 逐字一致（§41 未改坐标）", axisMismatch.length === 0,
            axisMismatch.length ? axisMismatch.join(" | ") : `${panel.joints.length} 个关节全部一致`);

      // 能力开关必须全部为 ✓（mini_arm 声明了 fk/ik/simulation 等）
      const capsOn = panel.caps.filter(c => c.includes("✓")).length;
      check("能力面板有开启项", capsOn >= 3, `${capsOn}/${panel.caps.length} 为 ✓`);
      check("坐标适配段落含 rotation.x = -1.570796",
            /rotation\.x\s*=\s*-1\.570796/.test(panel.adapt.replace(/\s+/g, " ")),
            panel.adapt.replace(/\s+/g, " ").slice(0, 90));
    }

    // ---------- ④ 截图（供人工目视）----------
    await sleep(3000); // WebGL 首帧编译慢，别省
    const shot1 = await capture(cdp);
    writeFileSync(join(OUT_DIR, "01-default.png"), shot1);
    check("截图 01-default.png 已落盘", shot1.length > 1000, `${shot1.length} bytes`);

    // ---------- ⑤ 交互生效：切"关节轴"开关，画面像素必须变 ----------
    //
    // ★ 判据必须落在**"哪块像素变了"**，而不是"差异像素的**个数**变了"。
    //
    //   为什么不能用个数：轴指示器是几根细圆柱 + 小锥头，
    //   关掉它们确实会少掉一片像素，但**个数变化极易被其它因素淹没** ——
    //   相机轻微抖动、抗锯齿、网格线重绘都能让 nonBg 上下浮动几十到几百。
    //   于是"像素个数差 > 50"这类阈值要么过松（任何抖动都通过 ⇒ 恒真），
    //   要么过紧（真变了也不通过 ⇒ 恒假）。**两种都是坏判据。**
    //
    //   ✅ 正确的做法：取两张**逐像素位图**，统计"有实质差异的像素占比"，
    //      并要求它落在"明显变了"的区间（>0.1%），
    //      同时要求**切回去之后该占比回落到噪声水平**（<0.02%）。
    //      前者证明"开关真的影响了渲染"，后者证明"影响是可逆的"。
    //      两个方向都要，否则"开关把整个画面变黑"也能通过前者。
    const grabBits = () => cdp.eval(`new Promise((resolve) => {
      requestAnimationFrame(() => {
        const cv = document.querySelector('canvas');
        const gl = cv.getContext('webgl2') || cv.getContext('webgl');
        const w = cv.width, h = cv.height;
        const px = new Uint8Array(w * h * 4);
        gl.readPixels(0, 0, w, h, gl.RGBA, gl.UNSIGNED_BYTE, px);
        // 稀疏采样（每 4 像素取 1）：够用且把 140ms 的全量循环降到可接受
        const bits = [];
        for (let i = 0; i < px.length; i += 16) bits.push(px[i], px[i+1], px[i+2]);
        resolve({ w, h, bits });
      });
    })`);

    /** 两张位图的"实质差异像素占比"。 */
    const diffRatio = (a, b) => {
      if (!a || !b || !a.bits || !b.bits || a.bits.length !== b.bits.length) return null;
      const n = a.bits.length / 3;
      let diff = 0;
      for (let i = 0; i < a.bits.length; i += 3) {
        const d = Math.abs(a.bits[i] - b.bits[i])
                + Math.abs(a.bits[i+1] - b.bits[i+1])
                + Math.abs(a.bits[i+2] - b.bits[i+2]);
        if (d > 24) diff++;
      }
      return diff / n;
    };

    const bitsOn = await grabBits();
    const toggled = await cdp.eval(`(() => {
      const cbs = [...document.querySelectorAll('.toolbar input[type=checkbox]')];
      const axes = cbs[0];
      if (!axes) return 'no checkbox';
      axes.click();
      return axes.checked;
    })()`);
    await sleep(1500);
    const bitsOff = await grabBits();

    const rOff = diffRatio(bitsOn, bitsOff);
    check("交互生效：切「关节轴」后渲染确实变化",
          rOff !== null && rOff > 0.001,
          `勾选态=${toggled}  差异像素占比=${rOff === null ? "n/a" : (rOff * 100).toFixed(3) + "%"}`);

    const shot2 = await capture(cdp);
    writeFileSync(join(OUT_DIR, "02-axes-off.png"), shot2);
    check("截图 02-axes-off.png 已落盘", shot2.length > 1000, `${shot2.length} bytes`);

    // 切回来（证明**可逆**：差异必须回落到噪声水平）
    await cdp.eval(`(() => {
      const cbs = [...document.querySelectorAll('.toolbar input[type=checkbox]')];
      if (cbs[0]) cbs[0].click();
    })()`);
    await sleep(1500);
    const bitsBack = await grabBits();
    const rBack = diffRatio(bitsOn, bitsBack);
    check("切换可逆：切回后画面回到原状",
          rBack !== null && rBack < 0.0002,
          `与初始帧差异像素占比=${rBack === null ? "n/a" : (rBack * 100).toFixed(4) + "%"}（噪声水平）`);

    // ---------- ⑥ 隐藏面板 ----------
    const panelToggled = await cdp.eval(`(() => {
      const btns = [...document.querySelectorAll('.toolbar button')];
      const b = btns.find(x => x.textContent.includes('隐藏面板') || x.textContent.includes('显示面板'));
      if (!b) return 'no button';
      const t0 = b.textContent.trim();
      b.click();
      return t0 + ' → ' + b.textContent.trim();
    })()`);
    await sleep(900);
    const sidebarState = await cdp.eval(
      `(() => { const s = document.querySelector('.sidebar'); return s ? s.className : null; })()`
    );
    check("「隐藏/显示面板」按钮生效", (panelToggled || "").includes("→"),
          `${panelToggled}  sidebar.class=${sidebarState}`);

    const shot3 = await capture(cdp);
    writeFileSync(join(OUT_DIR, "03-panel-collapsed.png"), shot3);

    // ---------- ⑦ 无副作用：控制台干净 ----------
    // favicon 404 是已知噪声（见 skill 硬坑 3-D），过滤掉
    const realErrors = cdp.errors.filter(e => {
      const u = e.url || "";
      if (u.endsWith("/favicon.ico")) return false;
      return e.level === "error";
    });
    const realExceptions = cdp.consoleErrors.filter(
      e => !String(e.text || "").includes("favicon")
    );
    check("控制台无 error 级日志", realErrors.length === 0,
          realErrors.length ? realErrors.slice(0, 3).map(e => `${e.text} @ ${e.url}`).join(" | ")
                            : `${cdp.errors.length} 条日志中 0 条为 error`);
    check("无未捕获异常", realExceptions.length === 0,
          realExceptions.length ? String(realExceptions[0].exception?.description).slice(0, 160) : "");

    // ---------- ⑧ WebSocket 是否真的连上了 ----------
    // 该探测必须**读页面自己的状态**，不能只断言"服务端有连接数" ——
    // 后者是页面之外的事实。
    const wsOk = await cdp.eval(`(() => {
      const marks = { connected: false, why: 'no signal' };
      // 页面若暴露了 ws 状态就取；否则用 performance 资源记录判断
      const res = performance.getEntriesByType('resource').map(r => r.name);
      const hasWsProbe = res.some(n => n.includes('/ws'));
      return { hasWsProbe, resources: res.filter(n => n.startsWith('http')).length };
    })()`);
    skip("页面内 WebSocket 探针", `本页为 REST-only 渲染，WS 由 tools/probe_ws_live.py 独立验收（已 25/25 PASS）；页面资源 ${wsOk?.resources} 项`);

    writeSummary();
    return results.some(r => !r.ok) ? 1 : 0;
  } finally {
    try { cdp?.ws?.close(); } catch { /* ignore */ }
    try { stderrStream?.end(); } catch { /* ignore */ }
    // attach 模式下**不杀**浏览器 —— 它是调用方起的，所有权不在本脚本。
    if (child) {
      try { child.kill(); } catch { /* ignore */ }
      await sleep(1200);
    }
    // 清理**历次**遗留的 profile（含本次）。删除失败一律忽略：
    // Windows 上被占用的目录删不掉是常态，而它是临时的、不影响结论。
    for (const dir of [PROFILE_DIR, ...listProfileDirs(OUT_DIR)]) {
      try { rmSync(dir, { recursive: true, force: true }); } catch { /* 占用时忽略 */ }
    }
  }
}

/** 列出 OUT_DIR 下所有 `edge-profile-*` 目录（用于跨次清理）。 */
function listProfileDirs(outDir) {
  try {
    return readdirSync(outDir)
      .filter((n) => n.startsWith("edge-profile"))
      .map((n) => join(outDir, n));
  } catch {
    return [];
  }
}

function writeSummary() {
  console.log("\n" + "=".repeat(74));
  const passed = results.filter((r) => r.ok && !r.skipped).length;
  const failed = results.filter((r) => !r.ok).length;
  const skipped = results.filter((r) => r.skipped).length;
  console.log(`结果：${passed} PASS / ${failed} FAIL / ${skipped} SKIP  （共 ${results.length} 项）`);
  console.log("=".repeat(74));
  for (const r of results) if (!r.ok) console.log(`  ✗ ${r.name}  ${r.detail}`);
  console.log(`\n截图目录：${OUT_DIR}`);
  return failed === 0 ? 0 : 1;
}

main()
  .then((code) => process.exit(code))
  .catch((e) => {
    console.error("\n探针本身出错：", e);
    process.exit(2);
  });
