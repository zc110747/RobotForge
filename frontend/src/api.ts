/**
 * =============================================================================
 * api.ts —— 后端 REST 客户端
 * -----------------------------------------------------------------------------
 * ★ 所有请求都用**相对路径**（`/api/...`）。
 *   端口由 Vite 代理处理（见 `vite.config.ts`），所以本文件里
 *   **不允许**出现 `http://localhost:8000` 这类硬编码 —— 那是把环境
 *   相关常量混进业务代码，换端口就要改全栈。
 *
 * ★ 响应体**不整形**：直接把后端的 RobotModel JSON 交给 viewModel。
 *   一旦这里做字段映射，就产生了第二份语义（见 `backend/api/app.py` 的注释）。
 */

import type { RobotListResponse, RobotModelResponse } from "./viewer/model.ts";

export const API_BASE = "/api";

export class ApiError extends Error {
  constructor(
    message: string,
    readonly status: number | null
  ) {
    super(message);
    this.name = "ApiError";
  }
}

async function getJson<T>(path: string, signal?: AbortSignal): Promise<T> {
  let resp: Response;
  try {
    // ⚠️ `exactOptionalPropertyTypes: true` 下不能传 `{ signal: undefined }`：
    //    RequestInit.signal 的类型是 `AbortSignal | null`（不含 undefined）。
    //    所以这里显式构造 init 对象，不要把 undefined 塞进属性。
    const init: RequestInit = signal === undefined ? {} : { signal };
    resp = await fetch(`${API_BASE}${path}`, init);
  } catch (e) {
    // 网络层失败（后端没起 / 代理没配）—— 与 HTTP 错误分开报，
    // 因为修复动作完全不同："去启动后端" vs "去看后端日志"
    throw new ApiError(
      `无法连接后端（${API_BASE}${path}）：${e instanceof Error ? e.message : String(e)}`,
      null
    );
  }
  if (!resp.ok) {
    let detail = `${resp.status} ${resp.statusText}`;
    try {
      const body = (await resp.json()) as { detail?: unknown };
      if (body.detail !== undefined) detail = String(body.detail);
    } catch {
      /* 响应体不是 JSON：保留状态码文本 */
    }
    throw new ApiError(detail, resp.status);
  }
  return (await resp.json()) as T;
}

export function fetchRobots(signal?: AbortSignal): Promise<RobotListResponse> {
  return getJson<RobotListResponse>("/robots", signal);
}

export function fetchRobotModel(
  robotId: string,
  signal?: AbortSignal
): Promise<RobotModelResponse> {
  return getJson<RobotModelResponse>(
    `/robots/${encodeURIComponent(robotId)}/model`,
    signal
  );
}
