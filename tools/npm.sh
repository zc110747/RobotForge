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

# -----------------------------------------------------------------------------
# ④ node 安装位置**不能硬编码用户名**。
#    原实现写死 `C:\Users\lx176\...`，换机器/换账户后本脚本 100% 失效，
#    而症状是 "找不到 node 可执行文件"，看起来像"没装 node"。
#    ⇒ 改为按优先级探测，支持环境变量覆盖：
#       $ROBOTFORGE_NODE_ROOT  >  managed binaries（当前用户）
#
# ⑤ `versions/current` **不是目录，是一个 9 字节的普通文件**（内容是版本名）。
#    所以用 `ls | sort` 挑"最新版本目录"时，它会被当成候选，
#    再 `-f current/node.exe` 判假 → 循环里 `break` 落空。别用管道挑版本。
#    ⇒ 改成显式 for 遍历，并且**必须** `[ -d "$d" ]` 才接受（见 resolve_node_root）。
# -----------------------------------------------------------------------------
resolve_node_root() {
  if [ -n "${ROBOTFORGE_NODE_ROOT:-}" ]; then
    printf '%s\n' "$ROBOTFORGE_NODE_ROOT"
    return 0
  fi
  # managed 运行时目录：遍历子目录（**只认真正的目录**，见坑 ⑤）
  local base="$HOME/.workbuddy/binaries/node/versions"
  local d
  if [ -d "$base" ]; then
    # 先优先 22.x（本项目验证过的版本），再退化为任意版本
    for d in "$base"/22.*; do
      if [ -f "$d/node.exe" ]; then printf '%s\n' "$d"; return 0; fi
    done
    for d in "$base"/*; do
      if [ -d "$d" ] && [ -f "$d/node.exe" ]; then printf '%s\n' "$d"; return 0; fi
    done
  fi
  return 1
}

NODE_ROOT="$(resolve_node_root)" || {
  echo "找不到 node 安装目录。请设置 ROBOTFORGE_NODE_ROOT 指向含 node.exe 的目录。" >&2
  exit 127
}

# MSYS 风格路径（供本脚本 -x 检查）；原生 Windows 形式（供 node 当参数用，见坑 ②）
NODE_EXE_MSYS="$NODE_ROOT/node.exe"
# `pwd -W` 给出的是 `C:/Users/...`（正斜杠），统一换成反斜杠后再拼，
# 避免出现 `C:/a/b\node_modules\...` 这种混合分隔符。
NODE_ROOT_WIN="$(cd "$NODE_ROOT" && pwd -W 2>/dev/null | tr '/' '\\')"
[ -n "$NODE_ROOT_WIN" ] || NODE_ROOT_WIN="$NODE_ROOT"
NPM_CLI="$NODE_ROOT_WIN\\node_modules\\npm\\bin\\npm-cli.js"

if [ ! -x "$NODE_EXE_MSYS" ]; then
  echo "找不到 node 可执行文件：$NODE_EXE_MSYS" >&2
  exit 127
fi

if [ ! -f "$(cygpath -u "$NPM_CLI" 2>/dev/null || printf '%s' "$NPM_CLI")" ]; then
  echo "找不到 npm-cli.js：$NPM_CLI" >&2
  exit 127
fi

exec "$NODE_EXE_MSYS" "$NPM_CLI" "$@"
