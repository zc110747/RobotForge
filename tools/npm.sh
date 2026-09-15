#!/usr/bin/env bash
# =============================================================================
# npm 包装器 —— 绕过 MSYS 环境下的 npm shim
# -----------------------------------------------------------------------------
# 为什么需要它（三处独立的坑，少修一个都跑不起来）：
#
# ① `npm` 这个 shell 脚本会 `uname` 判断平台，MSYS 报 "Linux"，
#    于是它走 `type wslpath` → 调用 **wsl.exe** 去跑 npm。
#    本机 wsl.exe 在沙箱黑名单里 ⇒ 命令被拦，表现为"npm 直接失败"。
#    ⇒ 必须绕过 shim，直接跑 `node_modules/npm/bin/npm-cli.js`。
#
# ② 用 MSYS 风格路径 `/c/Users/...` 作为 node 的**脚本参数**会被路径翻译
#    搞成 `D:\c\Users\...`（当前盘符 + 路径）⇒ MODULE_NOT_FOUND。
#    ⇒ 脚本路径必须是原生 Windows 形式 `C:\...`。
#
# ③ `timeout` 在本环境会命中 System32 的 TIMEOUT.EXE，不认 GNU 参数。
#    ⇒ 本脚本不包 timeout（由调用方决定是否 run_in_background）。
#
# 用法：
#   bash tools/npm.sh --version
#   bash tools/npm.sh install
#   bash tools/npm.sh --prefix frontend run build
# =============================================================================
set -u

NODE_ROOT='C:\Users\lx176\.workbuddy\binaries\node\versions\22.22.2-3'
NODE_EXE="/c/Users/lx176/.workbuddy/binaries/node/versions/22.22.2-3/node.exe"
NPM_CLI="$NODE_ROOT\\node_modules\\npm\\bin\\npm-cli.js"

if [ ! -x "$NODE_EXE" ]; then
  echo "找不到 node 可执行文件：$NODE_EXE" >&2
  exit 127
fi

exec "$NODE_EXE" "$NPM_CLI" "$@"
