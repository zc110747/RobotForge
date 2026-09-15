/**
 * =============================================================================
 * jsdom 的最小类型声明
 * -----------------------------------------------------------------------------
 * `jointPanel.check.ts` 用 jsdom 给"真 React 运行时"那节造一个 DOM 环境
 * （见该文件 `runRealReRender` 的注释：必须先把 DOM 装成全局，**再** import
 * `react-dom`，否则它会把"没有 DOM"这一探测结果缓存下来，事件系统走无 DOM
 * 分支 —— 症状是"派发了 input 事件但 onChange 一次都不触发"）。
 *
 * ## 为什么手写而不是装 `@types/jsdom`
 *
 * 我们**只用到 `new JSDOM(html).window`** 这一件事，其余全部通过
 * `as unknown as Window & typeof globalThis` 转到 DOM 标准类型上。
 * 装一整个 `@types/jsdom` 会把它的传递依赖（`@types/node` 版本等）
 * 拉进 `tsc --noEmit` 的检查面，而本项目其余部分**不依赖** jsdom 类型。
 *
 * ⇒ 这里只声明**用到的那个形状**，并且该模块只被 `.check.ts`（不进 bundle）
 *   引用。`vite build` 的输入图里没有它。
 *
 * ## ⚠️ 只描述形状，不含逻辑
 *
 * 声明若与真实 jsdom 对不上，`jointPanel.check.ts` 会**在运行期**炸
 * （不是静默通过）—— 因为它真的 `new` 了一个 jsdom。
 */
declare module "jsdom" {
  /** jsdom 构造出的一个"窗口"，形状按 DOM 标准用。 */
  export interface JSDOMWindow {
    readonly document: Document;
  }

  export interface JSDOMOptions {
    readonly url?: string;
    readonly pretendToBeVisual?: boolean;
  }

  export class JSDOM {
    constructor(html?: string, options?: JSDOMOptions);
    readonly window: JSDOMWindow;
  }
}
