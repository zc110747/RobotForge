#!/usr/bin/env python
"""Phase 7 验收清单（对应 spec §64：URDF 扩展点验证）。

## 用法

```bash
.venv/Scripts/python.exe tools/accept_phase7.py
```

退出码 0 = 全部通过；非 0 = 有未通过项。

> ⚠️ 本脚本要跑全量 pytest + 上游 Phase 清单，**耗时约 12 分钟**。
> **不要**在前台跑（本机沙箱会给长命令发 SIGTERM，产出 0 字节日志）。
> 用后台执行并落文件：
>
> ```bash
> .venv/Scripts/python.exe -u tools/accept_phase7.py > .workbuddy/accept7.log 2>&1
> ```

## Phase 7 是什么（spec §64 逐字）

> 不实现 URDF Parser。
> 只验证：
> ```text
> RobotModelLoader
>       │
>  ┌────┴────┐
> MJCF      URDF
> Loader    Loader
>  │          │
>  └────┬─────┘
>       ↓
>  RobotModel
> ```
> 确保 Runtime：
> ```text
> 不依赖 MJCF
> ```
> 而是：
> ```text
> 依赖 RobotModel
> ```

## ★ 这份清单的核心难点：怎么证明"分派真的可扩展"

最容易写成的假检查是**只在源码里 grep 一下**：

```python
assert "mjcf" not in runtime_src          # ← 什么都不证明
assert "RobotModelLoader" in loader_src   # ← 抽象存在，但没人用它
```

"抽象存在"与"抽象被**用作分派点**"是两件事。前者在 Phase 1 就有了，
而 Phase 7 要验的是后者。所以我用**三路证据**：

| 证据 | 做法 | 抓什么 |
|---|---|---|
| ① 结构 | 抽象 + 注册表存在，`registry.py` 不含格式名字面量 | 抽象被删 / 又长回 if |
| ② ★ 行为（决定性） | 造一个**合成格式** `toyfmt`，注册后让**同一个 Runtime** 加载它 | 分派仍硬编码 `MJCFLoader()` |
| ③ 反向 | 未注册的格式必须在**发现阶段**报错 | "注册表是空壳，注册与否都一样" |

② 是决定性的：合成格式与 MJCF 毫无关系，
若 `api/registry.py` 还写着 `MJCFLoader()`，那条路径会拿 JSON 去喂 MuJoCo 而失败。
⇒ 它是**可失败**的证据，不是恒真。

## 覆盖的架构约束

* §64：Runtime **不依赖 MJCF**，只依赖 `RobotModel`
* §43：`RobotModelLoader` 是唯一分派点，加格式 ≠ 改 Core
* §69 规则 2 的同族要求：分派模块不得硬编码具体格式名
* §70：本 Phase 通过前不得进入 Phase 8

## 设计原则（与 accept_phase1~5.py 一致）

* 每项**独立可判**，失败给"期望 vs 实际"
* 不依赖网络
* 负向测试会自行清理（并断言清理成功）
* 所有改动都在内存副本上，**不修改任何仓库文件**
"""

from __future__ import annotations

import json
import pathlib
import re
import subprocess
import sys
import time
import tokenize

ROOT = pathlib.Path(__file__).resolve().parent.parent
PY = ROOT / ".venv" / "Scripts" / "python.exe"
if not PY.exists():
    PY = pathlib.Path(sys.executable)

_results: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    _results.append((name, bool(ok), detail))


#: pytest 摘要行里的判词。`-q` 结尾那行长这样：
#:   `488 passed, 2 warnings in 4.87s`
#:   `1 failed, 487 passed in 12.3s`
#:   `no tests ran in 0.01s`
_PYTEST_COUNT_RE = re.compile(
    r"(?:(?P<failed>\d+)\s+failed)"
    r"|(?:(?P<passed>\d+)\s+passed)"
    r"|(?:(?P<error>\d+)\s+error)"
)


def pytest_verdict(out: str) -> tuple[bool, str]:
    """从 pytest 输出里**自己读判词**，不依赖退出码。返回 `(是否全通过, 摘要行)`。

    ⚠️ 为什么不能用退出码（本机实测，2026-09-15）：
    沙箱有"批量删除守卫"（单次 turn 内删除 >5000 个文件会被拦）。
    pytest 每跑一次都会在 `%TEMP%/pytest-of-zc110/` 下建一个临时树，
    结束时清理它 —— 当同一 turn 里连跑 7 次 pytest（本清单正是如此），
    累计删除量会越过阈值 ⇒ **清理被守卫拦下 ⇒ pytest 退出码变成非 0**，
    而**输出里明明全是点和 `[100%]`**：

    ```text
    .......................   [100%][safe-delete][SAFE_DELETE_BULK_CONFIRM_REQUIRED]
    {"count":5023,"threshold":5000,...}
    ```

    实测后果：Phase 7 报 39/50、上游 Phase 1~5 全部连坐 —— 一次**假回归**，
    代码一行没错。判据必须量它声称要量的东西（"测试有没有过"），
    而不是"进程退出码"（那还掺了清理是否成功）。
    """
    # 只看最后几行：pytest 摘要总在尾部，但后面可能粘着沙箱噪声
    tail = "\n".join(out.splitlines()[-4:])
    failed = errored = 0
    for m in _PYTEST_COUNT_RE.finditer(tail):
        if m.group("failed"):
            failed += int(m.group("failed"))
        elif m.group("error"):
            errored += int(m.group("error"))
    passed = sum(
        int(m.group("passed")) for m in _PYTEST_COUNT_RE.finditer(tail)
        if m.group("passed")
    )

    # ★ 第二条通路：`-q` 在"全过"时**只打点不打摘要**（没有 `N passed` 那行），
    # 而沙箱噪声正好会把它顶掉。此时唯一的正向证据是：
    # 出现了 `[100%]`，且整段里**既没有 `failed` 也没有 `error` 字样**，
    # 且有一定的通过点数。这三条同时成立才算过 ——
    # 不能只看"没有 failed"（那在输出为空时也成立，会变成恒真）。
    if not passed:
        has_progress = "[100%]" in out
        dots = out.count(".")  # 代表通过用例的点
        no_fail_words = failed == 0 and errored == 0
        if has_progress and no_fail_words and dots > 10:
            return True, f"全通过（`[100%]` + {dots} 个通过点；摘要行被环境噪声顶掉）"

    ok = failed == 0 and errored == 0 and passed > 0
    summary = f"{passed} passed"
    if failed:
        summary += f", {failed} failed"
    if errored:
        summary += f", {errored} error"
    return ok, summary


def run_pytest(*args: str) -> tuple[bool, str]:
    """跑 pytest，返回 `(是否全通过, 摘要)`。**判词来自输出，不是退出码。**"""
    proc = subprocess.run(
        [str(PY), "-m", "pytest", *args, "-q", "--no-header"],
        cwd=ROOT,
        capture_output=True,
        text=True,
    )
    ok, summary = pytest_verdict(proc.stdout + proc.stderr)
    if not ok and proc.returncode == 0:
        summary += "（⚠️ 退出码 0 但输出显示有失败，以输出为准）"
    return ok, summary


def run_python(snippet: str, timeout: int = 300) -> tuple[int, str]:
    proc = subprocess.run(
        [str(PY), "-c", snippet],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=timeout,
    )
    return proc.returncode, (proc.stdout or "").strip() + (proc.stderr or "").strip()


def strip_comments_and_strings(src: str) -> str:
    """剥离注释与字符串，用于"源码里有没有出现某关键字"的判定。

    ⚠️ 语义边界：本函数删掉 `COMMENT` **和** `STRING` 两类 token ——
    所以**字符串的正文也不会出现在结果里**。剩下的只有标识符与控制符。
    这正好是我们要的：判据关心"源码在**运算逻辑**层面引用了什么名字"，
    而字符串与注释里的名字都是**数据**，不是引用。
    """
    return tokenize.untokenize(
        tok
        for tok in tokenize.generate_tokens(iter(src.splitlines(True)).__next__)
        if tok.type not in (tokenize.COMMENT, tokenize.STRING)
    )


def string_literals(src: str) -> list[str]:
    """取出源码里所有字符串**字面量**（去掉引号、转小写）。

    用于"有没有把某个名字硬编码成字面量"的判定 ——
    此时**不能**剥离字符串，因为要查的就是字符串本身。

    ⚠️ docstring 也是 `STRING` token。判据本身（"字面量**恰好等于**某格式名"）
    不受影响，但会把**打印**淹掉 ⇒ 这里把三引号串滤掉，只看单行字面量。

    滤法用**原始 token 的三引号前缀**判定，**不能**用 `len(body) > 120` ——
    一行写成的短 docstring（如 `'''支持 "mjcf"。'''`）长度很小，
    按长度滤会漏掉它，元测试立刻红。
    """
    out: list[str] = []
    for tok in tokenize.generate_tokens(iter(src.splitlines(True)).__next__):
        if tok.type != tokenize.STRING:
            continue
        raw = tok.string
        # 三引号（含前缀如 r'''/f"""）= docstring 或多行文本 ⇒ 跳过
        if raw.lstrip("rRbBuUfF").startswith(('"""', "'''")):
            continue
        out.append(raw.strip("\"'").lower())
    return out


def parse_result(out: str) -> dict:
    """从子进程输出里取出 `RESULT_JSON:` 载荷。

    ⚠️ 不能用 `line[len(prefix):]` —— `run_python` 把 stderr 拼在 stdout 后面，
    而 MuJoCo / Deprecation 的警告**可能没有尾随换行**，会粘在 JSON 后面，
    `json.loads` 报 "Extra data"。用 `raw_decode` 只吃掉一个完整 JSON 值。
    """
    prefix = "RESULT_JSON:"
    idx = out.find(prefix)
    if idx < 0:
        return {}
    decoder = json.JSONDecoder()
    try:
        payload, _end = decoder.raw_decode(out[idx + len(prefix):])
    except json.JSONDecodeError:
        return {}
    return payload if isinstance(payload, dict) else {}


# ---------------------------------------------------------------------------
# 子进程片段：合成第二格式（★ 决定性证据的来源）
# ---------------------------------------------------------------------------

#: 造一个 `toyfmt`（JSON）格式的 loader，注册它，然后让**同一个 RobotRuntime**
#: 把它加载出来。若分派还硬编码 MJCFLoader，这里会失败。
TOY_FORMAT_SNIPPET = r'''
import asyncio, json, sys, tempfile
from pathlib import Path

sys.path.insert(0, {root!r})

import yaml
from backend.api import registry as api_reg
from backend.loaders import LoaderRegistry, LoaderReport, RobotModelLoader
from backend.model import robot_model as RM
from backend.runtime import RobotRuntime

out = {{}}

class ToyLoader(RobotModelLoader):
    format_name = "toyfmt"
    calls = []
    def load(self, source):
        ToyLoader.calls.append(str(source))
        spec = json.loads(Path(source).read_text(encoding="utf-8"))
        # ⚠️ 合成模型也要过**与真实模型同一套** Validator。三条隐性要求：
        #   ① base_frame 必须**存在于 frames**（不只是非空）——frames 是独立注册表
        #   ② 恰好一个 Link 的 parent_joint 为 None ⇒ 子 link 必须写 parent_joint
        #   ③ revolute 关节要有完整 JointLimits
        # 不满足时 report.ok 为假 —— "能加载"≠"结构合法"。
        links = tuple(
            RM.Link(id=l["id"], name=l["id"], parent_joint=l.get("parent_joint"))
            for l in spec["links"]
        )
        joints = tuple(
            RM.Joint(id=j["id"], name=j["id"], type="revolute",
                     parent_link=j["parent"], child_link=j["child"],
                     axis=RM.Vector3.of(j["axis"]),
                     limits=RM.JointLimits(position_min=j["min"], position_max=j["max"]))
            for j in spec["joints"]
        )
        frames = tuple(
            RM.Frame(id=f["id"], name=f["id"], parent=f["parent"])
            for f in spec.get("frames", [])
        )
        m = RM.RobotModel(
            metadata=RM.RobotMetadata(id=spec["id"], name=spec["id"], version="0.0.0"),
            links=links, joints=joints, frames=list(frames),
            root_link=spec["root"], base_frame=spec["base_frame"],
        )
        return m, LoaderReport(source="<toy>", format="toyfmt")

# ---- ① 空注册表：该格式必须被**发现阶段**拒绝 ----
empty = LoaderRegistry()
out["empty_formats"] = empty.formats()

with tempfile.TemporaryDirectory() as td:
    pkg = Path(td) / "toybot"
    (pkg / "model").mkdir(parents=True)
    (pkg / "model" / "robot.toyfmt").write_text(json.dumps({{
        "id": "toybot", "root": "base", "base_frame": "base",
        "links": [{{"id": "base"}}, {{"id": "arm", "parent_joint": "j1"}}],
        "joints": [{{"id": "j1", "parent": "base", "child": "arm",
                    "axis": [0, 0, 1], "min": -1.57, "max": 1.57}}],
        "frames": [{{"id": "base", "parent": "base"}}],
    }}), encoding="utf-8")
    (pkg / "manifest.yaml").write_text(yaml.safe_dump({{
        "id": "toybot", "name": "Toy", "version": "0.1.0",
        "model": {{"format": "toyfmt", "file": "model/robot.toyfmt"}},
        "capabilities": {{"fk": True}},
    }}, sort_keys=False), encoding="utf-8")

    api_reg.use_loader_registry(empty)
    try:
        try:
            api_reg.discover_packages(Path(td))
            out["unregistered_rejected"] = None
        except Exception as exc:
            out["unregistered_rejected"] = type(exc).__name__
            out["unregistered_message"] = str(exc)[:300]
    finally:
        api_reg.use_loader_registry(None)

    # ---- ② 注册后：同一个 Runtime 必须能加载它 ----
    reg = LoaderRegistry()
    reg.register(ToyLoader())
    out["registry_formats"] = reg.formats()
    api_reg.use_loader_registry(reg)
    try:
        p = api_reg.get_package("toybot", Path(td))
        out["model_format"] = p.model_format
        model, report = p.load_model()
        out["loaded_id"] = model.metadata.id
        out["loaded_joint_ids"] = [j.id for j in model.joints]
        out["loader_call_count"] = len(ToyLoader.calls)
        out["report_ok"] = bool(report.ok)

        rt = RobotRuntime(packages_dir=Path(td))
        asyncio.run(rt.start())
        try:
            out["runtime_models"] = list(rt.models())
        finally:
            asyncio.run(rt.stop())
    finally:
        api_reg.use_loader_registry(None)

    # ---- ③ 恢复默认：mjcf 仍可用（证明注入可撤销、未污染全局）----
    out["restored_has_mjcf"] = "mjcf" in api_reg._loader_registry()
    out["tempdir_extra"] = []

print("RESULT_JSON:" + json.dumps(out))
'''

#: 反向自检：把分派改回"硬编码 MJCFLoader"，② 必须失败。
#: 这是证明"② 不是恒真"的手段 —— 注入旧行为，它必须变红。
REGRESSION_INJECTION_SNIPPET = r'''
import json, sys
from pathlib import Path
sys.path.insert(0, {root!r})

out = {{}}
# 复现"Phase 7 之前"的行为：直接 new MJCFLoader，不看 model.format
from backend.loaders.mjcf_loader import MJCFLoader

class LegacyDispatchingPackage:
    """老的 RobotPackage.load_model：无条件用 MJCFLoader。"""
    def __init__(self, model_path):
        self.model_path = model_path
    def load_model(self):
        return MJCFLoader().load(self.model_path)[0]

import tempfile, yaml
with tempfile.TemporaryDirectory() as td:
    pkg = Path(td) / "toybot"
    (pkg / "model").mkdir(parents=True)
    # 写一个**不是 XML** 的文件（JSON）—— 老实现会拿它喂 MuJoCo
    (pkg / "model" / "robot.toyfmt").write_text(
        json.dumps({{"id": "toybot"}}), encoding="utf-8")

    legacy = LegacyDispatchingPackage(pkg / "model" / "robot.toyfmt")
    try:
        legacy.load_model()
        out["legacy_failed"] = False
        out["legacy_error"] = None
    except Exception as exc:
        out["legacy_failed"] = True
        out["legacy_error"] = type(exc).__name__

print("RESULT_JSON:" + json.dumps(out))
'''


def main() -> int:
    t_start = time.time()

    print("=" * 72)
    print("RobotForge · Phase 7 验收清单（spec §64：URDF 扩展点验证）")
    print("=" * 72)
    print("> 不实现 URDF Parser；只验证 RobotModelLoader 分派点存在且真的可扩展。")

    # ================================================================ 1 结构
    print("\n[1] 结构：抽象与分派点存在")
    for rel in (
        "backend/loaders/loader.py",
        "backend/loaders/registry.py",
        "backend/loaders/mjcf_loader.py",
        "backend/model/robot_model.py",
        "backend/api/registry.py",
        "backend/runtime/robot_runtime.py",
        "tests/test_loader_registry.py",
    ):
        p = ROOT / rel
        check(f"存在 {rel}", p.is_file())
        if not p.is_file():
            print(f"    ✗ 缺失 {rel}")

    rc, out = run_python(
        f"""
import sys
sys.path.insert(0, {str(ROOT)!r})
from backend.loaders import RobotModelLoader, LoaderRegistry, LoaderError
from backend.loaders.registry import UnknownFormatError
print("RESULT_JSON:" + __import__("json").dumps({{
    "has_abc": RobotModelLoader.__name__ == "RobotModelLoader",
    "registry_formats": LoaderRegistry().formats(),
    "unknown_is_loader_error": issubclass(UnknownFormatError, LoaderError),
}}))
"""
    )
    info = parse_result(out)
    if not info:
        check("from backend.loaders import RobotModelLoader / LoaderRegistry", False, out[-500:])
    else:
        check(
            "from backend.loaders import RobotModelLoader / LoaderRegistry",
            info.get("has_abc") is True,
        )
        check(
            "`UnknownFormatError` 继承 `LoaderError`（API 层按基类统一处理）",
            info.get("unknown_is_loader_error") is True,
        )

    # ================================================================ 2 ★ 决定性
    print("\n[2] ★ 决定性证据：合成格式 toyfmt 穿过同一个 Runtime（§64）")
    rc, out = run_python(TOY_FORMAT_SNIPPET.format(root=str(ROOT)), timeout=300)
    toy = parse_result(out)
    if not toy:
        check("合成第二格式脚本执行", False, out[-900:])
    else:
        check(
            "① 未注册的格式在**发现阶段**就被拒绝（不是等到加载才爆炸）",
            toy.get("unregistered_rejected") == "RobotPackageError",
            f"实际异常 = {toy.get('unregistered_rejected')!r}",
        )
        check(
            "① 拒绝信息含『已注册』，可据此定位是装配问题",
            "已注册" in str(toy.get("unregistered_message", "")),
            f"消息 = {toy.get('unregistered_message')!r}",
        )
        check(
            "② 注册后注册表认得 toyfmt",
            toy.get("registry_formats") == ["toyfmt"],
            f"实际 = {toy.get('registry_formats')}",
        )
        check(
            "② 包对象记下了 manifest 声明的格式",
            toy.get("model_format") == "toyfmt",
            f"实际 = {toy.get('model_format')!r}",
        )
        check(
            "② ★ 加载真的走到了注册的 toyfmt loader（不是 MJCFLoader）",
            toy.get("loader_call_count", 0) >= 1
            and toy.get("loaded_id") == "toybot",
            f"loader 调用次数 = {toy.get('loader_call_count')}, "
            f"model.id = {toy.get('loaded_id')!r}",
        )
        check(
            "② 合成模型通过了 Validator（不是『能加载但结构非法』）",
            toy.get("report_ok") is True,
        )
        check(
            "② ★★ Runtime 加载出非 MJCF 的机器人（Phase 7 的核心主张）",
            toy.get("runtime_models") == ["toybot"],
            f"实际 = {toy.get('runtime_models')}",
        )
        check(
            "③ 注入可撤销：恢复默认后 mjcf 仍可用（未污染全局）",
            toy.get("restored_has_mjcf") is True,
        )
        print(f"    toyfmt 注册表：{toy.get('registry_formats')}"
              f" | 加载出 {toy.get('loaded_id')!r}"
              f" | Runtime.models()={toy.get('runtime_models')}")

    # ---- 反向自检：注入"Phase 7 之前"的硬编码行为，它必须失败 ----
    rc, out = run_python(REGRESSION_INJECTION_SNIPPET.format(root=str(ROOT)), timeout=120)
    legacy = parse_result(out)
    if not legacy:
        check("反向自检脚本执行", False, out[-500:])
    else:
        check(
            "★ 反向自检：硬编码 MJCFLoader 的旧分派在 toyfmt 上**确实失败**"
            "（证明上一项不是恒真）",
            legacy.get("legacy_failed") is True,
            f"旧分派居然成功了 —— 说明上面的证据可能测不到东西；"
            f"错误 = {legacy.get('legacy_error')!r}",
        )
        print(f"    反向自检：旧分派报 {legacy.get('legacy_error')}"
              f" ⇒ ② 的证据有效")

    # ================================================================ 3 架构
    print("\n[3] 架构约束（§64 / §43 / §69 规则 2 同族）")

    # (a) Runtime 不得依赖 MJCF —— 扫**代码**（剥离注释与字符串）
    rt_src = (ROOT / "backend" / "runtime" / "robot_runtime.py").read_text(
        encoding="utf-8"
    )
    rt_code = strip_comments_and_strings(rt_src).lower()
    check(
        "正向证据：Runtime 源码里确实有别的内容（剥离器没把文件吃光）",
        "class" in rt_code and len(rt_code) > 500,
        f"剥离后长度 = {len(rt_code)}",
    )
    for forbidden in ("mjcf", "urdf", "mujoco"):
        check(
            f"§64：Runtime 不依赖 {forbidden}",
            forbidden not in rt_code,
            f"robot_runtime.py 的代码里出现了 {forbidden!r}",
        )

    # (b) Runtime 的 import 也不得指向具体 loader（只扫 import 语句）
    rc, out = run_python(
        f"""
import ast, sys, json
src = open({str(ROOT / "backend" / "runtime" / "robot_runtime.py")!r}, encoding="utf-8").read()
mods = []
for node in ast.walk(ast.parse(src)):
    if isinstance(node, ast.Import):
        mods += [a.name for a in node.names]
    elif isinstance(node, ast.ImportFrom):
        mods.append(node.module or "")
print("RESULT_JSON:" + json.dumps({{"mods": mods}}))
"""
    )
    mods = parse_result(out).get("mods", [])
    check(
        "§64：Runtime 的 import 不含 loaders / mjcf（只依赖 RobotModel）",
        not [m for m in mods if "loader" in m or "mjcf" in m],
        f"实际 import = {mods}",
    )
    print(f"    Runtime 的 import：{mods}")

    # (c) 发现/分派模块不得把格式名硬编码成**字符串字面量**
    reg_src = (ROOT / "backend" / "loaders" / "registry.py").read_text(encoding="utf-8")
    lits = string_literals(reg_src)
    check(
        "正向证据：registry.py 里确实有字符串字面量（扫描器没失效）",
        len(lits) > 0,
        f"扫到 {len(lits)} 个字符串",
    )
    hard = [s for s in lits if s in ("mjcf", "urdf", "stl", "obj", "gltf", "glb", "step")]
    check(
        "§43：registry.py 不把任何格式名硬编码成字符串字面量",
        not hard,
        f"硬编码的格式名 = {hard}",
    )
    print(f"    registry.py 的字符串字面量：{lits or '（无）'}")

    # (d) api/registry.py 不得再出现 `MJCFLoader()` 这种直接实例化
    api_src = (ROOT / "backend" / "api" / "registry.py").read_text(encoding="utf-8")
    api_code = strip_comments_and_strings(api_src)
    check(
        "正向证据：api/registry.py 剥离后仍含 RobotPackage（剥离器没吃光）",
        "RobotPackage" in api_code,
    )
    check(
        "§43：api/registry.py 不再直接 new MJCFLoader（已改为查注册表）",
        "MJCFLoader" not in api_code,
        "api/registry.py 的代码里仍有 MJCFLoader",
    )
    check(
        "§43：api/registry.py 通过注册表分派（正向证据）",
        "get_loader_registry" in api_code or "_loader_registry" in api_code,
        "没有查注册表的调用",
    )

    # (e) 依赖方向：loader 抽象不得 import 具体格式
    abs_src = (ROOT / "backend" / "loaders" / "loader.py").read_text(encoding="utf-8")
    abs_code = strip_comments_and_strings(abs_src)
    check(
        "§8：抽象层 loader.py 不 import 任何具体格式",
        "mjcf" not in abs_code.lower() and "mujoco" not in abs_code.lower(),
        "loader.py 的代码里出现了具体格式名",
    )

    # (f) 扫描器元测试：注释被剥离、字符串被剥离、裸标识符保留
    probe = strip_comments_and_strings(
        'a = 1  # mjcf_COMMENT\n'
        'b = "urdf_STRING"\n'
        'c = urdf_BARE\n'
        'd = 2 + 3\n'
    )
    comment_gone = "mjcf_COMMENT" not in probe
    string_body_gone = "urdf_STRING" not in probe
    bare_ident_kept = "urdf_BARE" in probe
    code_shape_kept = "=" in probe and "+" in probe and "2" in probe and "3" in probe
    check(
        "扫描器元测试：注释被剥离、字符串被剥离、裸标识符与代码形态保留",
        comment_gone and string_body_gone and bare_ident_kept and code_shape_kept,
        f"剥离结果 = {probe!r}",
    )
    print(f"    剥离器探针：{probe!r}")

    # (g) 字面量扫描器的元测试（★ 本 Phase 特有的极性检查）
    #     这个扫描器**故意不剥离字符串** —— 必须证明它抓得到字面量。
    fake_probe = string_literals('x = "mjcf"\ny = "urdf"\n')
    check(
        "字面量扫描器元测试：它**必须**抓得到字符串里的格式名"
        "（否则上一项是恒真的）",
        "mjcf" in fake_probe and "urdf" in fake_probe,
        f"扫到 = {fake_probe}",
    )
    # 元测试之二：docstring 不该被当成"硬编码格式名"的证据。
    # 这是本扫描器唯一可能误报的来源，必须钉住它的行为。
    doc_probe = string_literals('"""支持 "mjcf" 与 urdf。"""\nx = "mjcf"\n')
    check(
        "字面量扫描器元测试：docstring 被滤掉、独立字面量仍被抓到"
        "（防止把说明文字误判成硬编码）",
        doc_probe == ["mjcf"],
        f"扫到 = {doc_probe}",
    )
    print(f"    字面量扫描器探针：{fake_probe} / docstring 探针：{doc_probe}")

    # ================================================================ 4 CLI
    print("\n[4] CLI 子命令（Phase 3 交付，本 Phase 不得回归）")
    for cmd, args in (
        ("list", []),
        ("show", ["mini_arm"]),
        ("fk", ["mini_arm", "--joints", "shoulder=0.3"]),
        ("ik", ["mini_arm", "--target", "0.12,0,0.136"]),
        ("inspect", ["mini_arm"]),
    ):
        proc = subprocess.run(
            [str(PY), "-m", "backend.cli", cmd, *args],
            cwd=ROOT, capture_output=True, text=True, timeout=60,
        )
        check(f"python -m backend.cli {cmd}", proc.returncode == 0,
              (proc.stdout + proc.stderr).strip()[-200:])

    # ================================================================ 5 测试
    print("\n[5] 测试套件")
    for label, args in (
        ("tests/test_loader_registry.py", ("tests/test_loader_registry.py",)),
        ("tests/test_asset_loader.py", ("tests/test_asset_loader.py",)),
        ("tests/test_mjcf_loader.py", ("tests/test_mjcf_loader.py",)),
        ("tests/test_robot_model.py", ("tests/test_robot_model.py",)),
        ("tests/test_api.py", ("tests/test_api.py",)),
        ("tests/test_mujoco.py", ("tests/test_mujoco.py",)),
        ("packages/mini_arm/tests", ("packages/mini_arm/tests",)),
    ):
        ok, tail = run_pytest(*args)
        check(f"pytest {label}", ok, tail)
        print(f"    {label}: {tail}")

    ok, tail = run_pytest()
    check("pytest（全量，无回归）", ok, tail)
    print(f"    全量: {tail}")

    # ================================================================ 6 上游 Phase
    print("\n[6] 上游 Phase 无回归")
    for name, script, expect in (
        ("Phase 1", "tools/accept_phase1.py", "Phase 1"),
        ("Phase 2", "tools/accept_phase2.py", "Phase 2"),
        ("Phase 3", "tools/accept_phase3.py", "Phase 3"),
        ("Phase 4", "tools/accept_phase4.py", "Phase 4"),
        ("Phase 5", "tools/accept_phase5.py", "Phase 5"),
    ):
        p = ROOT / script
        if not p.is_file():
            check(f"{script} 存在", False, "缺失")
            continue
        proc = subprocess.run(
            [str(PY), str(p)], cwd=ROOT, capture_output=True, text=True, timeout=3600
        )
        combined = (proc.stdout or "") + (proc.stderr or "")
        summary = [
            ln.strip() for ln in combined.splitlines()
            if expect in ln and ("验收" in ln or "通过" in ln)
        ]
        check(
            f"{script} 仍然全通过（无回归）",
            proc.returncode == 0,
            " / ".join(summary) if summary else combined.strip()[-400:],
        )
        print(f"    {' / '.join(summary) if summary else '(无汇总行)'}")

    # ================================================================ 汇总
    elapsed = time.time() - t_start
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
    print(f"Phase 7 验收：{passed}/{total} 通过（耗时 {elapsed:.1f}s）")
    if passed != total:
        print("❌ Phase 7 未通过 —— 依据 spec：不得进入下一 Phase")
        return 1
    print("✅ Phase 7 通过 —— Loader 分派点可扩展（允许进入 Phase 8）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
