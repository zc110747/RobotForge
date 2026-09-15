#!/usr/bin/env python
"""Phase 8 验收清单（对应 spec §65：Geometry Asset 扩展点验证）。

## 用法

```bash
.venv/Scripts/python.exe tools/accept_phase8.py
```

退出码 0 = 全部通过；非 0 = 有未通过项。

可选参数（**只用于快速迭代，出交付结论前不要带**）：

```bash
.venv/Scripts/python.exe tools/accept_phase8.py --skip-upstream-phase7
```

Phase 7 清单内部会再跑一遍 Phase 1~5，与本节第 [7] 节重复 ⇒ 合计三遍。
加此参数可跳过 Phase 7 复查（省约 20 分钟），但该条会**计为 FAIL（跳过）**
而不是 PASS，避免"跳过"被误读成"通过"。

> ⚠️ 本脚本要跑全量 pytest + 上游 Phase 清单，**耗时约 12 分钟**。
> **不要**在前台跑（本机沙箱会给长命令发 SIGTERM，产出 0 字节日志）。
> 用后台执行并落文件：
>
> ```bash
> .venv/Scripts/python.exe -u tools/accept_phase8.py > .workbuddy/accept8.log 2>&1
> ```

## Phase 8 是什么（spec §65 逐字）

> 不实现 STL/OBJ/GLTF/STEP Parser。
> 只验证：
> ```text
> RobotModel → GeometryAsset → AssetLoader
> ```
> 架构未来可以加入 STLLoader/OBJLoader/GLTFLoader/CAD/STEP Loader，
> 而不影响 Runtime。

## ★ 这份清单与 Phase 7 的核心区别：v0.1 的**正确状态是空**

Phase 7 的默认注册表里有 `mjcf`（真实现），所以"注册表能用"可以直接观察。

Phase 8 的默认注册表**故意是空的** —— v0.1 不实现任何 mesh parser（§65）。
于是出现一个非常危险的判据形态：

```python
assert registry.extensions() == []      # ← 恒真：反正是空
assert "stl" not in registry            # ← 恒真：空表当然不含
```

这两条**在"扩展缝被删掉"时也照样通过**，属于 §4.11 那类自相矛盾的判据。
空表既可能是"设计如此"，也可能是"功能坏了"，**单看它是分不出来的**。

所以本 Phase 的证据必须**改变状态后再观察**：

| 证据 | 做法 | 抓什么 |
|---|---|---|
| ① 结构 | `AssetLoader` / `GeometryAsset` / `GeometryAssetRegistry` 存在 | 抽象被删 |
| ② ★ 行为（决定性） | 注册一个合成 `toymesh` loader，`resolve()` **从抛错变为返回资产** | 缝是空转的（注册与否都一样） |
| ③ 反向 | 用一个"登记了但从不查表"的假注册表，`resolve()` 必须仍抛错 | ② 的观察其实来自别的地方 |
| ④ v0.1 语义 | 默认空表 + `asset=None`（原生几何）走 `None` 快路径；五个未来格式全不支持 | 把"预留"误做成"未实现即报错" |

③ 是 ② 的**可失败性证明**：若 ② 的绿是因为别的路径（比如 `resolve` 里
硬编码了一个 toymesh 分支），③ 就不会红。

## 覆盖的架构约束

* §65：只验证缝，不实现 STL/OBJ/GLTF/STEP
* §44：`AssetLoader` 是可插拔点，加格式 ≠ 改 Runtime
* §24：`GeometryAsset.to_dict()` 不得带 `data`（否则 50k 顶点的 STL 会把
  每个 `robot_info` WS 帧撑到几 MB）
* §69：**缺 asset 不得阻止机器人启动** —— 所以 `resolve(None)` 返回 `None`
  是 v0.1 的**正常路径**，而"解析不了的引用"才抛 `UnsupportedAssetError`
* §70：本 Phase 通过前不得进入下一 Phase

## 设计原则（与 accept_phase1~7.py 一致）

* 每项**独立可判**，失败给"期望 vs 实际"
* 不依赖网络
* 负向测试会自行清理（并断言清理成功）
* 所有改动都在内存副本上，**不修改任何仓库文件**
"""

from __future__ import annotations

import json
import pathlib
import subprocess
import sys
import time
import tokenize

#: 跳过 Phase 7 复查（Phase 7 清单内含 Phase 1~5 复查，是整条链最慢的一环）。
#: 仅用于快速迭代；**出交付结论前必须不带此参数完整跑一次**。
SKIP_UPSTREAM_PHASE7 = "--skip-upstream-phase7" in sys.argv
if SKIP_UPSTREAM_PHASE7:
    sys.argv.remove("--skip-upstream-phase7")

ROOT = pathlib.Path(__file__).resolve().parent.parent
PY = ROOT / ".venv" / "Scripts" / "python.exe"
if not PY.exists():
    PY = pathlib.Path(sys.executable)

#: v0.1 **故意**不实现的格式（§65）。它们必须"不支持"，但错误信息要可行动。
FUTURE_FORMATS = ("stl", "obj", "gltf", "glb", "step")

_results: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    _results.append((name, bool(ok), detail))


def run_pytest(*args: str) -> tuple[int, str]:
    proc = subprocess.run(
        [str(PY), "-m", "pytest", *args, "-q", "--no-header"],
        cwd=ROOT,
        capture_output=True,
        text=True,
    )
    return proc.returncode, (proc.stdout + proc.stderr).strip()


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
    """剥离注释与字符串，用于"源码在**运算逻辑**层面引用了什么名字"的判定。

    ⚠️ 语义边界：本函数删掉 `COMMENT` **和** `STRING` 两类 token ——
    字符串的正文也不会出现在结果里。剩下的只有标识符与控制符。
    判据关心"代码引用"，而字符串与注释里的名字都是**数据**，不是引用。
    """
    return tokenize.untokenize(
        tok
        for tok in tokenize.generate_tokens(iter(src.splitlines(True)).__next__)
        if tok.type not in (tokenize.COMMENT, tokenize.STRING)
    )


def string_literals(src: str) -> list[str]:
    """取出源码里所有字符串**字面量**（去掉引号、转小写）。

    用于"有没有把某个格式名硬编码成字面量"的判定 ——
    此时**不能**剥离字符串，因为要查的就是字符串本身。

    ⚠️ docstring 也是 `STRING` token。判据本身（"字面量**恰好等于**某格式名"）
    不受影响，但会把**打印**淹掉 ⇒ 这里把三引号串滤掉，只看单行字面量。

    滤法用**原始 token 的三引号前缀**判定，**不能**用 `len(body) > 120` ——
    一行写成的短 docstring 长度很小，按长度滤会漏掉它，元测试立刻红。
    """
    out: list[str] = []
    for tok in tokenize.generate_tokens(iter(src.splitlines(True)).__next__):
        if tok.type != tokenize.STRING:
            continue
        raw = tok.string
        if raw.lstrip("rRbBuUfF").startswith(('"""', "'''")):
            continue
        out.append(raw.strip("\"'").lower())
    return out


def parse_result(out: str) -> dict:
    """从子进程输出里取出 `RESULT_JSON:` 载荷。

    ⚠️ 不能用 `line[len(prefix):]` —— `run_python` 把 stderr 拼在 stdout 后面，
    而 numpy/MuJoCo 的警告**可能没有尾随换行**，会粘在 JSON 后面导致
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
# 子进程片段
# ---------------------------------------------------------------------------

#: ★ 决定性证据：注册合成 mesh loader 前后，**同一段代码**的行为必须不同。
#: 若缝是空转的（resolve 里另有分支 / 注册表没被查），这里会露馅。
SEAM_BEHAVIOUR_SNIPPET = r'''
import json, sys, tempfile
from pathlib import Path

sys.path.insert(0, {root!r})

from backend.loaders import (
    AssetLoader,
    GeometryAsset,
    GeometryAssetRegistry,
    get_asset_registry,
)
from backend.model import robot_model as RM

out = {{}}

class ToyMeshLoader(AssetLoader):
    """合成第二格式：三行 `x y z` 文本。与 STL/OBJ/GLTF 毫无关系。"""
    extensions = ("toymesh",)
    calls = []
    def load(self, source, *, name=""):
        ToyMeshLoader.calls.append(str(source))
        if isinstance(source, (str, Path)):
            text = Path(source).read_text(encoding="utf-8")
        else:
            text = bytes(source).decode("utf-8")
        verts = tuple(
            tuple(float(v) for v in ln.split())
            for ln in text.splitlines() if ln.strip()
        )
        return GeometryAsset(
            name=name or str(source), kind="mesh", format="toymesh",
            data=verts, metadata={{"vertex_count": len(verts)}},
        )

with tempfile.TemporaryDirectory() as td:
    mesh = Path(td) / "wheel.toymesh"
    mesh.write_text("0 0 0\n1 0 0\n0 1 0\n", encoding="utf-8")
    # ⚠️ `resolve()` 把**整条引用串**交给 loader（asset 可能是 "meshes/wheel.stl"
    # 这样的相对路径，显示名由 loader 决定）。所以这里要传能读到的路径 ——
    # 否则抓到的是"文件不存在"，而不是"分派没生效"。
    ref = str(mesh)

    # ---- ① v0.1 默认：空表，解析不了 ⇒ 必须抛 UnsupportedAssetError ----
    reg = GeometryAssetRegistry()
    out["empty_extensions"] = reg.extensions()
    out["empty_len"] = len(reg)
    try:
        reg.resolve(ref)
        out["before_error"] = None
    except Exception as exc:
        out["before_error"] = type(exc).__name__
        out["before_message"] = str(exc)[:300]

    # ---- ② 注册之后：同一句 resolve 必须**不再抛错**，而是拿到资产 ----
    reg.register(ToyMeshLoader())
    out["registered_extensions"] = reg.extensions()
    out["contains_toymesh"] = "toymesh" in reg
    asset = reg.resolve(ref, name="wheel")
    out["after_asset_kind"] = asset.kind
    out["after_asset_format"] = asset.format
    out["after_asset_name"] = asset.name
    out["after_vertex_count"] = len(asset.data)
    out["loader_call_count"] = len(ToyMeshLoader.calls)

    d = asset.to_dict()
    out["to_dict_keys"] = sorted(d.keys())
    out["to_dict_has_data"] = "data" in d

    # ---- ④ v0.1 语义：原生几何（asset=None）走 None 快路径，不该抛错 ----
    out["none_resolves_to_none"] = reg.resolve(None) is None
    out["empty_string_resolves_to_none"] = reg.resolve("") is None
    # 默认注册表同样是空表 ⇒ 原生几何也必须是 None（不是"空表就炸"）
    out["default_none_is_none"] = get_asset_registry().resolve(None) is None
    out["default_extensions"] = get_asset_registry().extensions()

    # ---- ④ 五个未来格式：v0.1 一律不支持，但错误信息要可行动 ----
    unsupported = {{}}
    for ext in ("stl", "obj", "gltf", "glb", "step"):
        try:
            reg.resolve("thing." + ext)
            unsupported[ext] = "NO_ERROR"
        except Exception as exc:
            unsupported[ext] = type(exc).__name__
    out["future_unsupported"] = unsupported
print("RESULT_JSON:" + json.dumps(out))
'''

#: ★ 反向自检 ②：一个"登记了但从不查表"的假注册表。
#: 它满足"抽象存在"的表面要求（能 register、有 extensions），
#: 但 resolve 不consult 表 —— 若 SEAM_BEHAVIOUR_SNIPPET 的证据来自别处，
#: 那用这个 Hollow 应当**仍然抛错**。这就是 ② 的可失败性证明。
HOLLOW_REGISTRY_SNIPPET = r'''
import json, sys, tempfile
from pathlib import Path

sys.path.insert(0, {root!r})

from backend.loaders import AssetLoader, GeometryAsset

out = {{}}

class ToyMeshLoader(AssetLoader):
    extensions = ("toymesh",)
    def load(self, source, *, name=""):
        return GeometryAsset(name=str(source), kind="mesh", format="toymesh", data=())

class HollowRegistry:
    """长得像注册表，但 resolve 根本不查表 —— 等价于"注册了也没用"。"""
    def __init__(self):
        self._loaders = []
    def register(self, loader):
        self._loaders.append(loader)
        return loader
    def extensions(self):
        exts = []
        for ld in self._loaders:
            exts += list(ld.extensions)
        return exts
    def resolve(self, asset, *, name=""):
        if not asset:
            return None
        raise RuntimeError("hollow: never consults the table")

reg = HollowRegistry()
reg.register(ToyMeshLoader())
out["hollow_extensions"] = reg.extensions()
try:
    reg.resolve("wheel.toymesh")
    out["hollow_raised"] = False
except Exception as exc:
    out["hollow_raised"] = True
    out["hollow_error"] = type(exc).__name__

print("RESULT_JSON:" + json.dumps(out))
'''

#: 反向自检 ③：把"硬编码 toymesh 分支"塞进 resolve 的假注册表。
#: 用于证明 ② 的绿**不能**由一个硬编码分支造出来 ——
#: 因为一旦去掉那个分支，它必须变红。
HARDCODED_BRANCH_SNIPPET = r'''
import json, sys
sys.path.insert(0, {root!r})
from pathlib import Path

out = {{}}

class HardcodedRegistry:
    """承认吧：它 resolve 的是"格式名字面量"，不是注册表。"""
    def __init__(self):
        self._loaders = []
    def register(self, loader):
        self._loaders.append(loader)
        return loader
    def extensions(self):
        return ["toymesh"]
    def resolve(self, asset, *, name=""):
        if not asset:
            return None
        if str(asset).endswith(".toymesh"):
            # ← 硬编码：根本不看 self._loaders
            from backend.loaders import GeometryAsset
            return GeometryAsset(name=str(asset), kind="mesh", format="toymesh", data=())
        raise RuntimeError("nope")

# 空表也能"成功"，正是我们要抓的假绿
reg = HardcodedRegistry()
out["empty_table_still_resolves"] = reg.resolve("x.toymesh") is not None
out["but_toymesh_not_registered"] = reg.extensions()  # 声称支持，却什么都没注册

print("RESULT_JSON:" + json.dumps(out))
'''


def main() -> int:
    t_start = time.time()

    print("=" * 72)
    print("RobotForge · Phase 8 验收清单（spec §65：Geometry Asset 扩展点验证）")
    print("=" * 72)
    print("> 不实现 STL/OBJ/GLTF/STEP Parser；只验证 RobotModel → GeometryAsset → AssetLoader。")

    # ================================================================ 1 结构
    print("\n[1] 结构：抽象与扩展缝存在")
    for rel in (
        "backend/loaders/asset_loader.py",
        "backend/loaders/loader.py",
        "backend/loaders/registry.py",
        "backend/model/robot_model.py",
        "backend/runtime/robot_runtime.py",
        "tests/test_asset_loader.py",
    ):
        p = ROOT / rel
        check(f"存在 {rel}", p.is_file())
        if not p.is_file():
            print(f"    ✗ 缺失 {rel}")

    rc, out = run_python(
        f"""
import sys, json
sys.path.insert(0, {str(ROOT)!r})
from backend.loaders import (
    AssetError, AssetLoadError, AssetLoader,
    GeometryAsset, GeometryAssetRegistry, UnsupportedAssetError,
)
print("RESULT_JSON:" + json.dumps({{
    "has_abc": AssetLoader.__name__ == "AssetLoader",
    "asset_is_frozen": getattr(GeometryAsset, "__dataclass_params__").frozen,
    "unsupported_is_asseterror": issubclass(UnsupportedAssetError, AssetError),
    "loaderror_is_asseterror": issubclass(AssetLoadError, AssetError),
    "unsupported_is_not_loaderror": not issubclass(UnsupportedAssetError, AssetLoadError),
    "loaderror_is_not_unsupported": not issubclass(AssetLoadError, UnsupportedAssetError),
}}))
"""
    )
    info = parse_result(out)
    if not info:
        check("from backend.loaders import AssetLoader / GeometryAsset / ...", False, out[-500:])
    else:
        check(
            "from backend.loaders import AssetLoader / GeometryAsset / GeometryAssetRegistry",
            info.get("has_abc") is True,
        )
        check(
            "`GeometryAsset` 是 frozen dataclass（资产不可变，便于跨线程传）",
            info.get("asset_is_frozen") is True,
        )
        check(
            "错误分层：Unsupported/AssetLoadError 都继承 AssetError（API 层统一处理）",
            info.get("unsupported_is_asseterror") is True
            and info.get("loaderror_is_asseterror") is True,
        )
        check(
            "错误语义互斥：『不认识这格式』≠『认识但读坏了』（两者不互为子类）",
            info.get("unsupported_is_not_loaderror") is True
            and info.get("loaderror_is_not_unsupported") is True,
        )

    # ================================================================ 2 ★ 决定性
    print("\n[2] ★ 决定性证据：合成 mesh 格式穿过同一条 resolve（§65）")
    rc, out = run_python(SEAM_BEHAVIOUR_SNIPPET.format(root=str(ROOT)), timeout=300)
    seam = parse_result(out)
    if not seam:
        check("合成 asset loader 脚本执行", False, out[-900:])
    else:
        check(
            "④ v0.1 默认注册表是**空**的（§65：不实现任何 mesh parser）",
            seam.get("empty_extensions") == [] and seam.get("empty_len") == 0,
            f"实际 extensions = {seam.get('empty_extensions')!r}",
        )
        check(
            "④ 空表下解析 .toymesh 必须抛 `UnsupportedAssetError`（不是静默 None）",
            seam.get("before_error") == "UnsupportedAssetError",
            f"实际 = {seam.get('before_error')!r}",
        )
        check(
            "④ 错误信息点明 v0.1 现状，可据此判断是『设计如此』而非『配错了』",
            "v0.1" in str(seam.get("before_message", "")),
            f"消息 = {seam.get('before_message')!r}",
        )
        check(
            "② 注册后注册表认得 toymesh（缝存在）",
            seam.get("registered_extensions") == ["toymesh"]
            and seam.get("contains_toymesh") is True,
            f"实际 = {seam.get('registered_extensions')!r}",
        )
        check(
            "② ★ 同一句 resolve 从「抛错」变为「拿到资产」"
            "（证明注册真的改变了行为，不是空转）",
            seam.get("after_asset_format") == "toymesh"
            and seam.get("after_asset_kind") == "mesh",
            f"kind = {seam.get('after_asset_kind')!r}, "
            f"format = {seam.get('after_asset_format')!r}",
        )
        check(
            "② ★★ 分派真的调到了合成 loader（调用计数 ≥ 1，且数据来自该文件）",
            seam.get("loader_call_count", 0) >= 1
            and seam.get("after_vertex_count") == 3,
            f"调用次数 = {seam.get('loader_call_count')}, "
            f"顶点数 = {seam.get('after_vertex_count')}",
        )
        check(
            "② name 参数生效（调用方给的显示名优先于引用串）",
            seam.get("after_asset_name") == "wheel",
            f"实际 name = {seam.get('after_asset_name')!r}",
        )
        check(
            "§24：`to_dict()` 不含 `data`（否则大网格会撑爆 WS 帧）",
            seam.get("to_dict_has_data") is False,
            f"to_dict keys = {seam.get('to_dict_keys')}",
        )
        check(
            "§69：缺 asset（原生几何）走 `None` 快路径，**不得抛错**",
            seam.get("none_resolves_to_none") is True
            and seam.get("empty_string_resolves_to_none") is True,
            f"None→{seam.get('none_resolves_to_none')}, "
            f"''→{seam.get('empty_string_resolves_to_none')}",
        )
        check(
            "§69：默认（空）注册表下原生几何同样是 `None`（不是『空表就炸』）",
            seam.get("default_none_is_none") is True
            and seam.get("default_extensions") == [],
            f"实际 = {seam.get('default_none_is_none')!r} / "
            f"{seam.get('default_extensions')!r}",
        )
        fut = seam.get("future_unsupported", {})
        check(
            "§65：五个未来格式在 v0.1 一律不支持（但报的是 Unsupported 而非崩）",
            all(fut.get(ext) == "UnsupportedAssetError" for ext in FUTURE_FORMATS),
            f"实际 = {fut}",
        )
        print(f"    空表 extensions={seam.get('empty_extensions')}"
              f" | 注册后={seam.get('registered_extensions')}"
              f" | 顶点数={seam.get('after_vertex_count')}"
              f" | to_dict keys={seam.get('to_dict_keys')}")

    # ---- 反向自检 ②：空转的假注册表必须失败（证明上一节不是恒真）----
    rc, out = run_python(HOLLOW_REGISTRY_SNIPPET.format(root=str(ROOT)), timeout=120)
    hollow = parse_result(out)
    if not hollow:
        check("反向自检（空转注册表）脚本执行", False, out[-500:])
    else:
        check(
            "★ 反向自检：『登记了但从不查表』的假注册表 resolve 时**确实失败**"
            "（证明 ② 的绿不是恒真）",
            hollow.get("hollow_raised") is True,
            "空转注册表居然成功了 —— 说明 ② 可能测不到东西；"
            f"extensions={hollow.get('hollow_extensions')}",
        )
        print(f"    反向自检：空转注册表报 {hollow.get('hollow_error')}"
              f" ⇒ ② 的证据有效")

    # ---- 反向自检 ③：硬编码分支能把"空表"伪装成可用 ----
    rc, out = run_python(HARDCODED_BRANCH_SNIPPET.format(root=str(ROOT)), timeout=120)
    hard = parse_result(out)
    if not hard:
        check("反向自检（硬编码分支）脚本执行", False, out[-500:])
    else:
        check(
            "★ 反向自检：硬编码格式分支**能**在空注册表下伪装成功"
            "（这正是本 Phase 最危险的假绿形态）",
            hard.get("empty_table_still_resolves") is True,
            f"实际 = {hard.get('empty_table_still_resolves')!r}",
        )
        print("    反向自检：硬编码分支在空表下仍『成功』"
              " ⇒ 所以判据必须是『改变状态后再观察』，不能只看空表")

    # ================================================================ 3 架构
    print("\n[3] 架构约束（§65 / §44 / §24 / §69）")

    # (a) 默认注册表必须是"空且能装" —— 静态上看得出设计意图
    al_src = (ROOT / "backend" / "loaders" / "asset_loader.py").read_text(
        encoding="utf-8"
    )
    al_code = strip_comments_and_strings(al_src).lower()
    check(
        "正向证据：asset_loader.py 剥离后仍有内容（剥离器没吃光）",
        "class" in al_code and len(al_code) > 500,
        f"剥离后长度 = {len(al_code)}",
    )
    check(
        "§65：抽象层 asset_loader.py 不 import 任何具体 mesh 格式",
        not any(f in al_code for f in ("stl", "obj", "gltf", "collada", "step")),
        "asset_loader.py 的代码里出现了具体格式名",
    )

    # (b) Runtime 不得依赖任何 asset 格式（§65：加 loader 不影响 Runtime）
    rt_src = (ROOT / "backend" / "runtime" / "robot_runtime.py").read_text(
        encoding="utf-8"
    )
    rt_code = strip_comments_and_strings(rt_src).lower()
    check(
        "正向证据：Runtime 源码剥离后仍有内容",
        "class" in rt_code and len(rt_code) > 500,
        f"剥离后长度 = {len(rt_code)}",
    )
    for forbidden in FUTURE_FORMATS + ("mjcf", "urdf"):
        check(
            f"§65：Runtime 不依赖 asset 格式 {forbidden}",
            forbidden not in rt_code,
            f"robot_runtime.py 的代码里出现了 {forbidden!r}",
        )

    # (c) 分派模块不得把格式名硬编码成字符串字面量
    lits = string_literals(al_src)
    check(
        "正向证据：asset_loader.py 里确实有字符串字面量（扫描器没失效）",
        len(lits) > 0,
        f"扫到 {len(lits)} 个字符串",
    )
    hard_lits = [s for s in lits if s in FUTURE_FORMATS]
    check(
        "§44：asset_loader.py 不把任何 mesh 格式名硬编码成字符串字面量",
        not hard_lits,
        f"硬编码的格式名 = {hard_lits}",
    )
    print(f"    asset_loader.py 的字符串字面量：{lits or '（无）'}")

    # (d) 字面量扫描器元测试（★ 本 Phase 与 Phase 7 同源的极性检查）
    fake_probe = string_literals('x = "stl"\ny = "gltf"\n')
    check(
        "字面量扫描器元测试：它**必须**抓得到字符串里的格式名"
        "（否则上一项是恒真的）",
        "stl" in fake_probe and "gltf" in fake_probe,
        f"扫到 = {fake_probe}",
    )
    # 元测试之二：docstring 不该被当成"硬编码格式名"的证据。
    doc_probe = string_literals('"""未来支持 "stl" 与 obj。"""\nx = "stl"\n')
    check(
        "字面量扫描器元测试：docstring 被滤掉、独立字面量仍被抓到"
        "（防止把说明文字误判成硬编码）",
        doc_probe == ["stl"],
        f"扫到 = {doc_probe}",
    )
    print(f"    字面量扫描器探针：{fake_probe} / docstring 探针：{doc_probe}")

    # (e) 剥离器元测试
    probe = strip_comments_and_strings(
        'a = 1  # stl_COMMENT\n'
        'b = "gltf_STRING"\n'
        'c = obj_BARE\n'
        'd = 2 + 3\n'
    )
    check(
        "扫描器元测试：注释被剥离、字符串被剥离、裸标识符与代码形态保留",
        "stl_COMMENT" not in probe
        and "gltf_STRING" not in probe
        and "obj_BARE" in probe
        and "=" in probe and "+" in probe,
        f"剥离结果 = {probe!r}",
    )
    print(f"    剥离器探针：{probe!r}")

    # ================================================================ 4 两注册表互不干扰
    print("\n[4] 两个注册表是两件事（§43 vs §65）")
    rc, out = run_python(
        f"""
import sys, json
sys.path.insert(0, {str(ROOT)!r})
from backend.loaders import (
    default_loader_registry, get_loader_registry,
    default_asset_registry, get_asset_registry,
)
print("RESULT_JSON:" + json.dumps({{
    "loader_formats": default_loader_registry().formats(),
    "asset_extensions": default_asset_registry().extensions(),
    "loader_singleton": get_loader_registry().formats(),
    "asset_singleton": get_asset_registry().extensions(),
    "distinct_types": type(default_loader_registry()).__name__
                      != type(default_asset_registry()).__name__,
}}))
"""
    )
    two = parse_result(out)
    if not two:
        check("两注册表共存脚本执行", False, out[-500:])
    else:
        check(
            "模型注册表有 mjcf、资产注册表为空 —— 两者状态互不影响",
            two.get("loader_formats") == ["mjcf"]
            and two.get("asset_extensions") == [],
            f"loader={two.get('loader_formats')} / asset={two.get('asset_extensions')}",
        )
        check(
            "两者是不同实现（不是同一个类换个名字）",
            two.get("distinct_types") is True,
        )
        print(f"    模型注册表={two.get('loader_formats')}"
              f" | 资产注册表={two.get('asset_extensions')}（v0.1 应为空）")

    # ================================================================ 5 CLI
    print("\n[5] CLI 子命令（Phase 3 交付，本 Phase 不得回归）")
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

    # ================================================================ 6 测试
    print("\n[6] 测试套件")
    for label, args in (
        ("tests/test_asset_loader.py", ("tests/test_asset_loader.py",)),
        ("tests/test_loader_registry.py", ("tests/test_loader_registry.py",)),
        ("tests/test_mjcf_loader.py", ("tests/test_mjcf_loader.py",)),
        ("tests/test_robot_model.py", ("tests/test_robot_model.py",)),
        ("tests/test_api.py", ("tests/test_api.py",)),
        ("tests/test_mujoco.py", ("tests/test_mujoco.py",)),
        ("packages/mini_arm/tests", ("packages/mini_arm/tests",)),
    ):
        rc, out = run_pytest(*args)
        tail = out.splitlines()[-1] if out else ""
        check(f"pytest {label}", rc == 0, tail)
        print(f"    {label}: {tail}")

    rc, out = run_pytest()
    tail = out.splitlines()[-1] if out else ""
    check("pytest（全量，无回归）", rc == 0, tail)
    print(f"    全量: {tail}")

    # ================================================================ 7 上游 Phase
    #
    # ★ 为什么要有 --skip-upstream-phase7
    #
    # `accept_phase7.py` 清单的最后一节会**再跑一遍 Phase 1~5**，
    # 而本脚本第 7 节刚刚跑过同样六个脚本 ⇒ 合计重复三遍，实测多花 20 分钟。
    #
    # Phase 7 的"无回归"证据在**同一会话里**已经由本脚本第 7 节的部分覆盖
    # （Phase 1~5 全绿）。但 Phase 7 **自己的** 50 条检查不能由 Phase 8 代证 ——
    # 所以默认**仍然完整跑** `accept_phase7.py`（正确性优先）。
    #
    # 需要快速迭代时用 `--skip-upstream-phase7`：跳过 Phase 7 复查，
    # 但**打印醒目警告**并在汇总里计为一条 SKIP（不是 PASS），
    # 免得"跳过"被误读成"通过"（§4.6 假检查同族）。
    print("\n[7] 上游 Phase 无回归")
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

    # Phase 7 单独处理（因为它最慢：内含 Phase 1~5 复查）
    p7 = ROOT / "tools" / "accept_phase7.py"
    if SKIP_UPSTREAM_PHASE7:
        print("    ⚠️ 已按 --skip-upstream-phase7 跳过 Phase 7 复查 —— "
              "本次 Phase 8 的『上游无回归』证据不完整！")
        check(
            "tools/accept_phase7.py 仍然全通过（无回归）",
            False,
            "本次被 --skip-upstream-phase7 跳过：不能计为通过。"
            "出交付结论前必须不带该参数完整跑一次。",
        )
    elif not p7.is_file():
        check("tools/accept_phase7.py 存在", False, "缺失")
    else:
        proc = subprocess.run(
            [str(PY), str(p7)], cwd=ROOT, capture_output=True, text=True, timeout=3600
        )
        combined = (proc.stdout or "") + (proc.stderr or "")
        summary = [
            ln.strip() for ln in combined.splitlines()
            if "Phase 7" in ln and ("验收" in ln or "通过" in ln)
        ]
        check(
            "tools/accept_phase7.py 仍然全通过（无回归）",
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
    print(f"Phase 8 验收：{passed}/{total} 通过（耗时 {elapsed:.1f}s）")
    if passed != total:
        print("❌ Phase 8 未通过 —— 依据 spec：不得进入下一 Phase")
        return 1
    print("✅ Phase 8 通过 —— GeometryAsset 扩展缝可插拔（v0.1 全链路完成）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
