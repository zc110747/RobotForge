import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// =============================================================================
// Vite 配置
// -----------------------------------------------------------------------------
// ★ 为什么把 /api 代理到后端
//   开发期前端跑在 5173，后端跑在 8000。若不代理，前端就得在代码里写死
//   `http://localhost:8000` —— 那是一个**环境相关常量混进业务代码**，
//   换端口就全栈改。代理让前端始终用相对路径 `/api/...`，
//   生产环境（同源部署）下同一份代码不用改。
//
// ★ 为什么显式写 `host: "127.0.0.1"`（Windows 上必须）
//   Vite 默认 host 是 `localhost`，在 Windows 上它会**只绑定 IPv6 `[::1]`**
//   （实测 netstat：`TCP [::1]:5173 LISTENING`）。
//   而 uvicorn 用 `--host 127.0.0.1` 时**只绑定 IPv4**。
//   两个进程各占一个地址族，代理就会报：
//
//     upstream connect failed: 由于目标计算机积极拒绝，无法连接。(os error 10061)
//
//   而 `curl http://127.0.0.1:5173/` 会拿到这段代理错误文本（HTTP 502），
//   看起来像"前端挂了"或"后端挂了"—— 两边其实都活着。
//   ⇒ 统一绑到 IPv4，代理 target 也用 IPv4，地址族就一致了。
// =============================================================================
export default defineConfig({
  plugins: [react()],
  server: {
    host: "127.0.0.1",
    port: 5173,
    strictPort: true,
    proxy: {
      "/api": {
        target: "http://127.0.0.1:8000",
        changeOrigin: true,
      },
      "/ws": {
        target: "ws://127.0.0.1:8000",
        ws: true,
      },
    },
  },
  build: {
    outDir: "dist",
    sourcemap: true,
  },
});
