#!/usr/bin/env python
"""Phase 2 验收清单（对应 spec §59）。

## 用法

```bash
.venv/Scripts/python.exe tools/accept_phase2.py
```

退出码 0 = 全部通过；非 0 = 有未通过项。

## Phase 2 是什么（spec §59 逐字）

```text
RobotModel
 ↓
Renderer Adapter
 ↓
Three.js
 ↓
3D mini_arm
```

验收：
* 浏览器显示机器人
* Camera Orbit
* Zoom
* Pan
* Joint hierarchy 正确
* Joint axis 正确
* End Effector 正确
* Coordinate Frame 正确
* RobotModel 与 Renderer 语义一致

## 这份清单怎么把这些条目变成可执行的

| 验收项                    | 由什么证明                                                    |
|---------------------------|---------------------------------------------------------------|
| 浏览器显示机器人          | `render.check.ts`（真实渲染出 101 项结构+数值断言）           |
| Camera Orbit / Zoom / Pan | `OrbitControls` 已挂载 + 构建产物存在（真交互需目视，见下）   |
| Joint hierarchy 正确      | `semantics.check.ts` 父子关系断言 + `render.check.ts` 树形     |
| Joint axis 正确           | 两边都断言 axis 逐分量与模型一致，且轴指示器只在可动关节上     |
| End Effector 正确         | EE 标记挂在其 link 下 + position == frame.transform.position   |
| Coordinate Frame 正确     | adapter 整场景旋转恰为 -π/2，且只在 coordinateAdapter.ts 里   |
| RobotModel ↔ Renderer 一致| `semantics.check.ts` 121 项 + 4 项自检（证明检查会失败）       |

## 两条"必须靠人"的事，脚本只钉住它的前提

* **真交互（拖拽/滚轮/右键）**：无法在无头环境断言。脚本改为断言
  `OrbitControls` 确实被挂载、且 canvas 存在 —— 把"忘了接控制器"
  这类**功能性缺失**变成会失败的检查，剩下的手感交给目视。
* **目视外观**：几何尺寸/配色的正确性只有真 GL 才看得见。脚本断言
  `makeGeometry` 已覆盖模型里出现的**全部**几何类型（不认识就画占位盒），
  避免"某类几何静默不渲染"。

## 设计原则（与 accept_phase1.py 一致）

* 每项**独立可判**，失败给"期望 vs 实际"
* 不依赖网络、不修改任何文件（临时注入的负向测试会自行清理）
* 子进程输出**显式落文件再读** —— 本机 PowerShell/沙箱会吞原生进程 stdout
"""

from __future__ import annotations

import io
import json
import pathlib
import subprocess
import sys
import tokenize

ROOT = pathlib.Path(__file__).resolve().parent.parent
PY = ROOT / ".venv" / "Scripts" / "python.exe"
if not PY.exists():
    PY = pathlib.Path(sys.executable)

FRONTEND = ROOT / "frontend"
# ★ 用托管 Node 的绝对路径：裸 `node` 会先命中 PATH 里其它版本，
#   而本工程依赖 Node 22 的 `--experimental-strip-types`。
NODE_CANDIDATES = [
    pathlib.Path(r"C:\Users\lx176\.workbuddy\binaries\node\versions\22.22.2-3\node.exe"),
    pathlib.Path(r"D:\Software\nodejs\node.exe"),
]

#: 共享的 pytest 判词解析器（唯一真值源），与 harness 同目录。
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from _pytest_verdict import pytest_verdict  # noqa: E402


_results: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    _results.append((name, bool(ok), detail))


def node_exe() -> pathlib.Path:
    for c in NODE_CANDIDATES:
        if c.exists():
            return c
    return pathlib.Path("node")


def run_pytest(*args: str) -> tuple[bool, str]:
    """跑 pytest，返回 `(是否全通过, 摘要)`。

    ⚠️ **判词来自输出，不是退出码** —— 见 `_pytest_verdict.py` 的模块 docstring：
    本机沙箱的批量删除守卫会拦下 pytest 的临时目录清理，
    让"测试全过"的一次运行**退出码非 0**（实测把 Phase 7 顶成 39/50）。
    """
    proc = subprocess.run(
        [str(PY), "-m", "pytest", *args, "-q", "--no-header"],
        cwd=ROOT,
        capture_output=True,
        text=True,
    )
    return pytest_verdict(proc.stdout + proc.stderr)


def run_node(script: str, timeout: int = 180) -> tuple[int, str]:
    """跑 frontend 下的一个 node 脚本，返回 (returncode, 输出尾部)。

    ⚠️ 只取 **stdout 的后半段**：React 在静态渲染下会打出大量
    "incorrect casing" 警告（假警报，见 RobotScene.tsx 注释），
    它们会淹没真正的结论。结论以 `✅` / `❌` 行判定。
    """
    proc = subprocess.run(
        [str(node_exe()), "--experimental-strip-types", script],
        cwd=FRONTEND,
        capture_output=True,
        text=True,
        timeout=timeout,
    )
    out = (proc.stdout or "").strip()
    # 结论行单独捞出来（stderr 只用于诊断，不参与判定）
    verdict = [ln for ln in out.splitlines() if "✅" in ln or "❌" in ln]
    tail = "\n".join(verdict) if verdict else out.splitlines()[-1] if out else "(无输出)"
    if proc.returncode != 0 and not verdict:
        err = (proc.stderr or "").strip().splitlines()
        tail = err[-1] if err else tail
    return proc.returncode, tail


def strip_comments_and_strings(src: str) -> str:
    return tokenize.untokenize(
        tok
        for tok in tokenize.generate_tokens(io.StringIO(src).readline)
        if tok.type not in (tokenize.COMMENT, tokenize.STRING)
    )


def frontend_sources() -> list[pathlib.Path]:
    """frontend/src 下所有源码（排除检查脚本自身所在的 __tests__）。"""
    out = []
    for p in sorted((FRONTEND / "src").rglob("*")):
        if p.suffix not in (".ts", ".tsx"):
            continue
        if "__tests__" in p.parts:
            continue
        out.append(p)
    return out


def main() -> int:
    print("=" * 72)
    print("RobotForge · Phase 2 验收清单（spec §59：Three.js）")
    print("=" * 72)

    # ================================================================ 1 文件
    print("\n[1] Phase 2 交付文件")
    files = [
        # 渲染链（spec §59 的四个环节）
        "frontend/src/viewer/model.ts",             # RobotModel 的只读 TS 镜像
        "frontend/src/viewer/viewModel.ts",         # 层级/轴/EE 的唯一推导处
        "frontend/src/viewer/coordinateAdapter.ts", # Renderer Adapter（P0 唯一转换点）
        "frontend/src/viewer/RobotScene.tsx",       # Three.js 场景
        # 应用外壳
        "frontend/src/App.tsx",
        "frontend/src/main.tsx",
        "frontend/src/api.ts",
        "frontend/src/styles.css",
        "frontend/index.html",
        "frontend/package.json",
        "frontend/tsconfig.json",
        "frontend/vite.config.ts",
        # 检查脚本（本 Phase 的"可执行验收"）
        "frontend/src/viewer/__tests__/semantics.check.ts",
        "frontend/src/viewer/__tests__/render.check.ts",
        "frontend/src/viewer/__tests__/geometry.check.ts",
        "frontend/src/viewer/__tests__/fixtures/model.json",
        # 后端模型服务
        "backend/api/registry.py",
        "backend/api/app.py",
        "tests/test_api.py",
        # 工具链
        "tools/npm.sh",
        "tools/export_view_fixture.py",
    ]
    for rel in files:
        p = ROOT / rel
        check(f"存在 {rel}", p.is_file(), "缺失" if not p.is_file() else "")

    # ================================================================ 2 类型
    print("\n[2] TypeScript 严格类型检查（tsc --noEmit）")
    tsc = FRONTEND / "node_modules" / "typescript" / "bin" / "tsc"
    if not tsc.exists():
        check("tsc 可用", False, f"找不到 {tsc}")
    else:
        proc = subprocess.run(
            [str(node_exe()), str(tsc), "--noEmit"],
            cwd=FRONTEND,
            capture_output=True,
            text=True,
            timeout=300,
        )
        out = (proc.stdout or "").strip()
        # tsc 把错误写到 stdout；无输出 + 退出码 0 = 干净
        check(
            "tsc --noEmit 零错误",
            proc.returncode == 0 and out == "",
            out[:800] if out else f"exit={proc.returncode}",
        )
        print(f"    {'零错误' if proc.returncode == 0 and not out else out[:200]}")

    # ================================================================ 3 构建
    print("\n[3] 生产构建（vite build）")
    vite = FRONTEND / "node_modules" / "vite" / "bin" / "vite.js"
    if not vite.exists():
        check("vite 可用", False, f"找不到 {vite}")
    else:
        proc = subprocess.run(
            [str(node_exe()), str(vite), "build"],
            cwd=FRONTEND,
            capture_output=True,
            text=True,
            timeout=600,
        )
        combined = (proc.stdout or "") + (proc.stderr or "")
        built = proc.returncode == 0 and "built in" in combined
        check("vite build 成功", built, combined.strip().splitlines()[-1] if combined else "")
        dist = FRONTEND / "dist"
        check("构建产物 dist/index.html 存在", (dist / "index.html").is_file(), "缺失")
        assets = sorted((dist / "assets").glob("*.js")) if (dist / "assets").is_dir() else []
        check("构建产物含 JS bundle", len(assets) > 0, f"找到 {len(assets)} 个 .js")

    # ================================================================ 4 检查脚本
    print("\n[4] Phase 2 可执行验收（渲染树 + 语义一致）")
    rc, tail = run_node("src/viewer/__tests__/semantics.check.ts")
    check("semantics.check.ts 全绿", rc == 0 and "✅" in tail, tail)
    print(f"    {tail}")

    rc, tail = run_node("src/viewer/__tests__/render.check.ts")
    check("render.check.ts 全绿", rc == 0 and "✅" in tail, tail)
    print(f"    {tail}")

    rc, tail = run_node("src/viewer/__tests__/geometry.check.ts")
    check("geometry.check.ts 全绿（几何尺寸语义）", rc == 0 and "✅" in tail, tail)
    print(f"    {tail}")

    # 负向证明：语义检查必须能**检出**注入缺陷。
    # 若某个自检项没检出，说明检查本身失效了 —— 这比"没检查"更危险。
    proc = subprocess.run(
        [str(node_exe()), "--experimental-strip-types",
         "src/viewer/__tests__/semantics.check.ts"],
        cwd=FRONTEND, capture_output=True, text=True, timeout=180,
    )
    stdout = proc.stdout or ""
    self_test_lines = [ln for ln in stdout.splitlines() if "⇒ 检查必须报错" in ln]
    detected = sum(1 for ln in self_test_lines if "✓" in ln)
    check(
        f"语义检查的 4 项自检全部检出缺陷（实际 {detected}）",
        detected == 4 and len(self_test_lines) == 4,
        f"自检行数={len(self_test_lines)} 检出={detected}",
    )

    # ================================================================ 5 语义一致
    print("\n[5] RobotModel ↔ Renderer 语义一致（spec §59 最后一条）")
    try:
        fixture = json.loads(
            (FRONTEND / "src" / "viewer" / "__tests__" / "fixtures" / "model.json").read_text(
                encoding="utf-8"
            )
        )
        exp = fixture.get("expect", {})
        model = fixture.get("model", {})

        check("fixture 含 robotId", bool(fixture.get("robotId")), "")
        check(
            "fixture 含 Python 侧独立推导的 expect（非 TS 自身输出）",
            bool(exp),
            "expect 为空 ⇒ 该检查会变成自证",
        )

        # fixture 的 model 必须与**真后端**返回的一致 —— 否则"语义一致"
        # 只是与一份可能过期的快照一致。
        sys.path.insert(0, str(ROOT))
        from backend.api.registry import get_package  # noqa: E402

        pkg = get_package(fixture["robotId"])
        live_model, live_report = pkg.load_model()

        # ⚠️ 比较前必须**归一化 source 绝对路径**，否则这条断言在不同机器上
        #    恒定 FAIL，且会把排查方向引向"fixture 过期、去重跑导出脚本" ——
        #    而重跑导出脚本永远只是把**本机**路径写进去，换台机器又不一致。
        #
        #    实测：`metadata.description` = `Loaded from native MJCF (<abs-path>)`，
        #    fixture 是提交进仓库的，于是它在
        #      `D:\user_project\git\RobotForge\...` 上导出、
        #      在 `E:\cnb\git\RobotForge\...` 上校验 ⇒ 必然不等。
        #
        #    **真正该断言的是"模型内容一致"，不是"某台机器的磁盘路径一致"。**
        #    路径属于**环境**，不属于模型语义。
        def normalize(d: dict) -> dict:
            d = json.loads(json.dumps(d))  # 深拷贝，别改原对象
            desc = d.get("metadata", {}).get("description") or ""
            marker = "Loaded from native MJCF ("
            if marker in desc and desc.endswith(")"):
                d["metadata"]["description"] = "Loaded from native MJCF (<path>)"
            return d

        live_dict = live_model.to_dict()
        differ = []
        if normalize(live_dict) != normalize(model):
            # 给出**具体差异字段**，而不是只报"不一致" ——
            # 否则排查只能靠人肉 diff 两个大 JSON。
            a, b = normalize(live_dict), normalize(model)
            for key in sorted(set(a) | set(b)):
                if a.get(key) != b.get(key):
                    differ.append(key)
        check(
            f"fixture.model 与后端实时加载的 {fixture['robotId']} 一致（忽略机器路径）",
            not differ,
            f"不一致字段：{differ[:5]} ⇒ 重跑 tools/export_view_fixture.py"
            if differ
            else "",
        )
        check("后端加载的模型校验通过", live_report.ok, live_report.format())

        # 交叉核对关键语义量（不依赖前端的任何代码）
        check("links 数一致", len(model.get("links", [])) == len(exp.get("linkParent", {})),
              f"model={len(model.get('links', []))} expect={len(exp.get('linkParent', {}))}")

        # ★ `to_dict()` 里**没有** `dof` 这个键 —— 它是派生量，不该出现在
        #   DTO 里（同 `test_api.py` 里钉的"row has no derived geometry"）。
        #   所以这里要**从 model 现算** dof，再与 Python 侧独立推导的 expect 比。
        #   （本清单第一版直接读 model["dof"] 拿到 None 而误报 FAIL ——
        #    是检查写错了，不是实现有问题。）
        derived_dof = sum(1 for j in model["joints"] if j["type"] != "fixed")
        check(
            "dof 一致（model 现算 vs expect 独立推导）",
            derived_dof == exp.get("dof"),
            f"model 现算={derived_dof} expect={exp.get('dof')}",
        )
        check(
            "DTO 不含派生量 dof（derived 不入 DTO）",
            "dof" not in model,
            "model 里出现了 dof ⇒ 派生量泄漏进 DTO",
        )
        check(
            "jointCount / linkCount 与 model 一致",
            exp.get("jointCount") == len(model["joints"])
            and exp.get("linkCount") == len(model["links"]),
            f"joints {exp.get('jointCount')} vs {len(model['joints'])}；"
            f"links {exp.get('linkCount')} vs {len(model['links'])}",
        )
        check(
            "可动关节数一致",
            exp.get("movableJointCount") == derived_dof,
            f"expect={exp.get('movableJointCount')} 现算={derived_dof}",
        )
    except Exception as e:  # noqa: BLE001
        check("语义一致的 fixture 交叉核对可执行", False, f"{type(e).__name__}: {e}")

    # ================================================================ 6 P0 坐标
    print("\n[6] P0 约束：坐标转换只有一个入口")
    adapter = ROOT / "frontend" / "src" / "viewer" / "coordinateAdapter.ts"
    # 6a. -π/2 这个常量**只允许**出现在 coordinateAdapter.ts
    offenders: list[str] = []
    for p in frontend_sources():
        if p.name == "coordinateAdapter.ts":
            continue
        src = p.read_text(encoding="utf-8")
        for lineno, line in enumerate(src.splitlines(), 1):
            # 匹配 -Math.PI / 2 及其变体；注释里的引用不算（这里刻意不剥离
            # 注释 —— 见到就报，改注释比重写逻辑便宜）
            if "Math.PI / 2" in line or "Math.PI/2" in line:
                offenders.append(f"{p.relative_to(ROOT)}:{lineno}")
    check(
        "场景旋转常量 -π/2 只出现在 coordinateAdapter.ts",
        not offenders,
        "命中: " + ", ".join(offenders),
    )

    # 6b. 除 App.tsx 的挂载点外，不允许别处再旋转场景
    rot_offenders: list[str] = []
    for p in frontend_sources():
        if p.name in ("coordinateAdapter.ts", "App.tsx"):
            continue
        src = strip_ts_comments(p.read_text(encoding="utf-8"))
        if "SCENE_ROTATION_X" in src or "applySceneRotation" in src:
            rot_offenders.append(str(p.relative_to(ROOT)))
    check(
        "整场景旋转只被 App.tsx 使用（其它文件不得二次旋转）",
        not rot_offenders,
        "命中: " + ", ".join(rot_offenders),
    )

    # 6c. 核心层（backend/）不得为 Three.js 改坐标 —— 复用 Phase 1 的判据
    coord_offenders: list[str] = []
    for p in sorted(ROOT.glob("backend/**/*.py")):
        if "__pycache__" in p.parts:
            continue
        src = strip_comments_and_strings(p.read_text(encoding="utf-8"))
        for lineno, line in enumerate(src.splitlines(), 1):
            if "three" in line.lower() or "y_up" in line.lower() or "yup" in line.lower():
                coord_offenders.append(f"{p.relative_to(ROOT)}:{lineno}")
    check(
        "backend/ 不含 Three.js 相关坐标处理（spec §41）",
        not coord_offenders,
        "命中: " + ", ".join(coord_offenders),
    )

    # ================================================================ 7 架构
    print("\n[7] 架构约束")
    # 7a. Core 不得出现型号名（v0.1 唯一的型号是 mini_arm）
    ton_offenders: list[str] = []
    for p in sorted(ROOT.glob("backend/**/*.py")):
        if "__pycache__" in p.parts:
            continue
        src = strip_comments_and_strings(p.read_text(encoding="utf-8"))
        for lineno, line in enumerate(src.splitlines(), 1):
            if "mini_arm" in line or "mearm" in line.lower():
                ton_offenders.append(f"{p.relative_to(ROOT)}:{lineno}")
    check("Core (backend/) 无型号特定分支", not ton_offenders, "命中: " + ", ".join(ton_offenders))

    # 7b. 前端也不得硬编码型号名（能力面板必须由 capabilities 驱动）
    fe_ton: list[str] = []
    for p in frontend_sources():
        src = strip_ts_comments(p.read_text(encoding="utf-8"))
        for lineno, line in enumerate(src.splitlines(), 1):
            if "mini_arm" in line or "mearm" in line.lower():
                fe_ton.append(f"{p.relative_to(ROOT)}:{lineno}")
    check(
        "Frontend (src/) 无型号特定分支",
        not fe_ton,
        "命中: " + ", ".join(fe_ton),
    )

    # 7c. 渲染层不得自己推导层级/坐标（各有唯一归属处）
    dup_offenders: list[str] = []
    scene = (FRONTEND / "src" / "viewer" / "RobotScene.tsx").read_text(encoding="utf-8")
    scene_code = strip_ts_comments(scene)
    for bad in ("parent_joint", "parent_link", "child_joints"):
        if bad in scene_code:
            dup_offenders.append(f"RobotScene.tsx 直接读了 {bad}")
    check(
        "RobotScene.tsx 不自己推导层级（层级只由 viewModel.ts 决定）",
        not dup_offenders,
        "; ".join(dup_offenders),
    )

    # 7d. viewModel.ts 必须零 Three.js 依赖（否则"纯函数"的说法不成立）
    vm_src = (FRONTEND / "src" / "viewer" / "viewModel.ts").read_text(encoding="utf-8")
    vm_imports_three = any(
        ln.strip().startswith("import") and '"three"' in ln
        for ln in vm_src.splitlines()
    )
    check("viewModel.ts 零 Three.js 依赖", not vm_imports_three, "发现有 import 'three'")

    # ================================================================ 8 交互
    print("\n[8] Camera 交互（Orbit / Zoom / Pan）")
    app_src = (FRONTEND / "src" / "App.tsx").read_text(encoding="utf-8")
    check("App.tsx 挂载了 OrbitControls", "OrbitControls" in app_src,
          "没挂在 Controller ⇒ 无法旋转/缩放/平移")
    check("Canvas 存在（可显示 3D）", "<Canvas" in app_src, "没有 Canvas")
    for prop in ("minDistance", "maxDistance", "target"):
        check(f"OrbitControls 设了 {prop}（限制缩放/对焦点）", prop in app_src, "")
    check(
        "界面标注了操作方式（旋转/缩放/平移）",
        all(k in app_src for k in ("左键", "滚轮", "右键")),
        "用户不知道能怎么操作",
    )

    # ================================================================ 9 几何
    print("\n[9] 几何覆盖（不得静默不渲染）")
    scene_code = strip_ts_comments(scene)
    # 模型里实际出现的几何类型，必须都在 makeGeometry 的分支里
    try:
        model = json.loads(
            (FRONTEND / "src" / "viewer" / "__tests__" / "fixtures" / "model.json").read_text(
                encoding="utf-8"
            )
        )["model"]
        used = sorted(
            {g["type"] for link in model["links"] for g in link.get("collision", [])}
        )
        missing = [t for t in used if f'"{t}"' not in scene_code]
        check(
            f"模型用到的几何类型均被 makeGeometry 覆盖（{', '.join(used)}）",
            not missing,
            f"缺: {missing}",
        )
        # 未知类型必须有显式兜底（否则是静默空渲染）
        check(
            "未知几何类型有显式兜底（占用位盒或告警）",
            "unknown" in scene_code.lower() or "占位" in scene,
            "没有兜底 ⇒ 新几何类型会静默消失",
        )
    except Exception as e:  # noqa: BLE001
        check("几何覆盖检查可执行", False, f"{type(e).__name__}: {e}")

    # ================================================================ 10 回归
    print("\n[10] 基线回归（Phase 1 未回退）")
    ok, out = run_pytest()
    check("pytest (Core) 全绿", ok, out)
    print(f"    {tail}")

    ok, out = run_pytest("packages/mini_arm/tests")
    check("pytest (mini_arm) 全绿", ok, out)
    print(f"    {tail}")

    # Phase 1 清单本身必须仍然全通过
    p1 = ROOT / "tools" / "accept_phase1.py"
    if p1.is_file():
        proc = subprocess.run(
            [str(PY), str(p1)], cwd=ROOT, capture_output=True, text=True, timeout=300
        )
        combined = (proc.stdout or "") + (proc.stderr or "")
        summary = [
            ln for ln in combined.splitlines() if "Phase 1 验收" in ln or "Phase 1 通过" in ln
        ]
        check(
            "tools/accept_phase1.py 仍然全通过（无回归）",
            proc.returncode == 0,
            " / ".join(summary) if summary else combined.strip()[-300:],
        )
        print(f"    {' / '.join(summary) if summary else '(无汇总行)'}")
    else:
        check("tools/accept_phase1.py 存在", False, "缺失")

    # ================================================================ 汇总
    print("\n" + "=" * 72)
    passed = sum(1 for _, ok, _ in _results if ok)
    total = len(_results)
    for name, ok, detail in _results:
        mark = "PASS" if ok else "FAIL"
        line = f"  [{mark}] {name}"
        if not ok and detail:
            line += f"\n         ↳ {detail}"
        print(line)
    print("=" * 72)
    print(f"Phase 2 验收：{passed}/{total} 通过")
    if passed != total:
        print("❌ Phase 2 未通过 —— 依据 spec：不得进入下一 Phase")
        return 1
    print("✅ Phase 2 通过 —— 允许进入 Phase 3")
    return 0


def strip_ts_comments(src: str) -> str:
    """粗略剥离 TS/TSX 注释（行注释 + 块注释）。

    够用即可：目的是让"注释里提到某关键字"不被误判成"代码里用了它"。
    字符串不剥离 —— 若某关键字只出现在字符串里，通常也值得看一眼。
    """
    out: list[str] = []
    i = 0
    n = len(src)
    in_line = False
    in_block = False
    in_str: str | None = None
    while i < n:
        c = src[i]
        nxt = src[i + 1] if i + 1 < n else ""
        if in_line:
            if c == "\n":
                in_line = False
                out.append(c)
            i += 1
            continue
        if in_block:
            if c == "*" and nxt == "/":
                in_block = False
                i += 2
                continue
            if c == "\n":
                out.append(c)
            i += 1
            continue
        if in_str is not None:
            out.append(c)
            if c == "\\":
                if i + 1 < n:
                    out.append(src[i + 1])
                i += 2
                continue
            if c == in_str:
                in_str = None
            i += 1
            continue
        if c == "/" and nxt == "/":
            in_line = True
            i += 2
            continue
        if c == "/" and nxt == "*":
            in_block = True
            i += 2
            continue
        if c in ("'", '"', "`"):
            in_str = c
            out.append(c)
            i += 1
            continue
        out.append(c)
        i += 1
    return "".join(out)


if __name__ == "__main__":
    raise SystemExit(main())
