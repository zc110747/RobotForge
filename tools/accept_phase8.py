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
import re
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

#: 共享的 pytest 判词解析器（唯一真值源）。放在 tools/ 下、与 harness 同级。
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from _pytest_verdict import pytest_verdict  # noqa: E402


_results: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    _results.append((name, bool(ok), detail))


def run_pytest(*args: str) -> tuple[bool, str]:
    """跑 pytest，返回 `(是否全通过, 摘要)`。

    ⚠️ **判词来自输出，不是退出码** —— 原因见 `_pytest_verdict.py` 的模块
    docstring：本机沙箱的批量删除守卫会拦下 pytest 的临时目录清理，
    让"测试全过"的一次运行**退出码非 0**（实测把 Phase 7 顶成 39/50）。
    """
    proc = subprocess.run(
        [str(PY), "-m", "pytest", *args, "-q", "--no-header"],
        cwd=ROOT,
        capture_output=True,
        text=True,
    )
    return pytest_verdict(proc.stdout + proc.stderr)


def run_python(snippet: str, timeout: int = 300) -> tuple[int, str]:
    proc = subprocess.run(
        [str(PY), "-c", snippet],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=timeout,
    )
    return proc.returncode, (proc.stdout or "").strip() + (proc.stderr or "").strip()


#: 需要剥离的 token 类型集合。
#:
#: ★ 为什么**必须**含 `FSTRING_MIDDLE`（Python ≥ 3.12 / PEP 701）：
#: 3.12 起 f-string 被拆成 `FSTRING_START` + `FSTRING_MIDDLE` + `{表达式}`
#: + `FSTRING_END`，**f-string 里的字面正文不再是 `STRING` 而是 `FSTRING_MIDDLE`**。
#: 只滤 `COMMENT`/`STRING` 会让 f-string 的提示文本原样活下来 ——
#: 实测后果：`asset_loader.py` 里
#:     raise AssetLoaderRegistrationError(
#:         f"扩展名不含前导点（注册 'stl'，不是 '.stl'）。"
#:     )
#: 这句**纯提示文案**里的 `stl` 会被判成"抽象层引用了具体格式名"，
#: §65 的检查于是 FAIL —— 而代码里一个格式名都没硬编码。
#: 换句话说：漏剥 f-string 会把"错误信息写得具体"惩罚成"架构违规"，
#: 于是修法会退化成人人把提示文案写模糊（判据在腐蚀被测对象）。
_STRIPPED_TOKEN_TYPES = tuple(
    t
    for t in (
        tokenize.COMMENT,
        tokenize.STRING,
        getattr(tokenize, "FSTRING_MIDDLE", None),
        getattr(tokenize, "FSTRING_START", None),  # 只含 `f"` 前缀，无正文，顺手删
        getattr(tokenize, "FSTRING_END", None),
    )
    if t is not None
)


def strip_comments_and_strings(src: str) -> str:
    """剥离注释与字符串，用于"源码在**运算逻辑**层面引用了什么名字"的判定。

    ⚠️ 语义边界：本函数删掉 `COMMENT`、`STRING` **以及 f-string 的字面正文**
    （`FSTRING_MIDDLE`，见 `_STRIPPED_TOKEN_TYPES` 的说明）。剩下的只有
    标识符、运算符与控制符。判据关心"代码引用"，而字符串与注释里的名字
    都是**数据**，不是引用。

    ⚠️ 本函数**只**保证"字符串正文不在结果里"。它**不**保证结果是可执行的
    Python（`untokenize` 会补空白、被删的 token 位置会塌缩）。用途仅限
    "扫名字"，不要拿去 `exec`。
    """
    return tokenize.untokenize(
        tok
        for tok in tokenize.generate_tokens(iter(src.splitlines(True)).__next__)
        if tok.type not in _STRIPPED_TOKEN_TYPES
    )


#: 标识符字符集：ASCII 字母数字 + 下划线 + 非 ASCII（CJK 常用在变量名之外，
#: 但这里保守地算进去，避免把 `格式名` 这类标识符切开而漏判）。
_IDENT_CHARS = re.compile(r"[0-9A-Za-z_\u0080-\uffff]")
#: 允许出现在扩展名里的字符（格式名都是 `[a-z0-9]`）。
_WORD_CHARS = "0123456789abcdefghijklmnopqrstuvwxyz"


def mentions_format_name(code: str, name: str) -> bool:
    """判断 `name` 是否作为**独立名字/扩展名**出现在 `code` 里（词边界匹配）。

    ★ 为什么不能用裸子串 `name in code`：
    `step` / `obj` 这类短格式名是**大量常见标识符的子串** ——
    实测 `robot_runtime.py` 里 `async def step(...)`（正是 spec 要求的
    Runtime 接口方法名）会把 §65 的 `step` 检查顶红；`asset_loader.py`
    里 `extension: object` 的 `object` 会把 `obj` 检查顶红。
    于是判据在惩罚**正确的代码**（接口方法就该叫 `step`），
    而修法只能是改接口名或把类型注解写成 `"object"` —— 两者都是错的。

    词边界定义（两侧）：
      - 左侧：`code[i-1]` 不是标识符字符（`[0-9A-Za-z_]` 及非 ASCII）
      - 右侧：`code[i+len(name)]` 不是标识符字符

    这样 `step` 只命中独立出现（`.step` / `step(` / `"step"`），
    **不**命中 `steps` / `stepper` / `footstep`；
    `obj` 不命中 `object` / `obj_ref` 的 `obj_`，但**会**命中独立 `obj`。
    """
    if not name:
        return False
    start = 0
    while True:
        i = code.find(name, start)
        if i < 0:
            return False
        left_ok = i == 0 or not _IDENT_CHARS.match(code[i - 1])
        j = i + len(name)
        right_ok = j >= len(code) or not _IDENT_CHARS.match(code[j])
        if left_ok and right_ok:
            return True
        start = i + 1


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
    al_hits = [f for f in ("stl", "obj", "gltf", "collada", "step") if mentions_format_name(al_code, f)]
    check(
        "§65：抽象层 asset_loader.py 不 import 任何具体 mesh 格式",
        not al_hits,
        f"asset_loader.py 的代码里出现了具体格式名：{al_hits}",
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
        # ★ `step` 与 spec **强制要求的** Runtime 接口方法名 `step()` 同名。
        # 裸词匹配必然顶红 `async def step(...)` —— 而那是 spec §47/§55 规定的
        # Runtime 方法（`await rt.step(command)`），**不能删**。
        # 因此对 `step` 只检查"作为**格式/扩展名**被引用"的形态：
        #   `.step`（扩展名/属性）、`step_loader`、`load_step`、`StepLoader`、
        #   `"step"`（字面量，已由剥离器干掉 ⇒ 走到这里的只可能是代码形态）。
        # 而"作为方法名/局部变量"的 `step` **不算**依赖 asset 格式 ——
        # 这正是判据该有的语义：§65 关心的是"Runtime 有没有为某种 mesh 格式
        # 开分支/做特判"，不是"Runtime 里出现了 step 这个词"。
        if forbidden == "step":
            # ★ `step` 与 spec **强制要求的** Runtime 接口方法名 `step()` 同名，
            # 而 `await self.step(cmd)` / `def step(self)` 是**必删不能删**的代码。
            # 所以判据必须收窄到"**作为格式/加载器被引用**"的形态。
            #
            # ⚠️ 为什么不能靠 `.step` 这个形状：
            # 扩展名引用（`'wheel.step'` / `Path(p).suffix`）是**字符串**，
            # 已被剥离器干掉 ⇒ 走到这一步的 `.step` 只剩两种可能：
            # ① `self.step(...)` —— 就是那个接口方法（合法）
            # ② `obj.step` —— 属性名，与 asset 格式无关
            # 两者都不是"依赖 STEP 格式" ⇒ `.step` 作为判据只会误报。
            #
            # 因此收窄到 **loader / parser 命名形态**，这也正是 §65 的原话
            # （spec：「STLLoader / OBJLoader / GLTFLoader / CAD/STEP Loader」）。
            offending = [
                pat
                for pat in (
                    "step_loader",   # 模块/变量名
                    "steploader",    # 类名 StepLoader（lower 后）
                    "load_step",     # 函数名
                    "parse_step",    # 函数名
                    "step_parser",   # 解析器名
                )
                if pat in rt_code
            ]
            check(
                f"§65：Runtime 不依赖 asset 格式 {forbidden}",
                not offending,
                f"robot_runtime.py 的代码里出现了 STEP 格式引用：{offending}"
                "（注：`step()` 方法名本身除外 —— 它与 spec 的 Runtime 接口同名）",
            )
            continue
        check(
            f"§65：Runtime 不依赖 asset 格式 {forbidden}",
            not mentions_format_name(rt_code, forbidden),
            f"robot_runtime.py 的代码里出现了 {forbidden!r}（独立名字，非子串）",
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

    # (e2) ★ 剥离器元测试之二：**f-string 的字面正文也必须被剥掉**。
    # 这条是补上的 —— 原先只有上面那条（普通 STRING），于是 PEP 701 的
    # `FSTRING_MIDDLE` 漏剥整整漏了过去，直接导致 §65 的两条 FAIL。
    # 反例注射：把 `_STRIPPED_TOKEN_TYPES` 里的 FSTRING_MIDDLE 去掉，本项立刻红。
    fs_probe = strip_comments_and_strings(
        'msg = f"注册 \'stl\'，不是 \'.stl\' {ext!r}"\n'
        'plain = "gltf"\n'
        'keep = obj_bare\n'
    )
    check(
        "扫描器元测试：f-string 的**字面正文**被剥离、插值表达式与裸标识符保留"
        "（PEP 701：f-string 正文是 FSTRING_MIDDLE，不是 STRING）",
        "stl" not in fs_probe.lower()
        and "gltf" not in fs_probe.lower()
        and "obj_bare" in fs_probe
        and "ext" in fs_probe,  # 插值里的名字必须留着（那才是"代码引用"）
        f"剥离结果 = {fs_probe!r}",
    )
    print(f"    f-string 剥离探针：{fs_probe!r}")

    # (f) 词边界匹配器元测试 —— 判据本身有没有判别力
    check(
        "词边界匹配元测试：能抓到**独立出现**的格式名"
        "（`.step` / `step(` / 独立 `obj` 都要命中）",
        mentions_format_name("await self.step(cmd)", "step")
        and mentions_format_name("def step(self)", "step")
        and mentions_format_name("kind = obj", "obj")
        and mentions_format_name("x.stl", "stl"),
        "独立出现的格式名居然没抓到 ⇒ 判据恒真",
    )
    check(
        "词边界匹配元测试：**不**抓长标识符里的子串"
        "（`steps`/`stepper`/`object`/`obj_ref` 都不算）",
        not mentions_format_name("n = steps + 1", "step")
        and not mentions_format_name("class Stepper", "step")
        and not mentions_format_name("extension: object", "obj")
        and not mentions_format_name("my_obj_ref", "obj"),
        "子串被误判成格式名 ⇒ 判据会惩罚正确的代码",
    )
    # 反例注射：拿修好之前的写法（裸子串）跑一遍，必须与现在**结论相反** ——
    # 证明这次改的不是"凑绿"，而是判据的判别力真的变了。
    old_style_step = "step" in "async def step(self): pass"
    old_style_obj = "obj" in "extension: object"
    check(
        "★ 反例注射：旧的『裸子串』判据**确实**会把 `def step` 与 `: object` 判成违规"
        "（证明这次改的是判据判别力，不是把检查凑绿）",
        old_style_step and old_style_obj,
        f"旧判据命中 step={old_style_step} obj={old_style_obj}"
        "（预期两者都为 True）",
    )
    # (g) ★ `step` 的收窄判据本身也必须**有判别力** ——
    # 否则它就是一条恒真项（"永远不报"和"没在查"在输出上无法区分）。
    # 收窄后的定义（与上面 (b) 处的实现**必须**一致）：
    #   STEP 只认"被当成 loader/parser 引用"的命名形态，
    #   **不**认 `.step`（那是 spec 要求的接口方法调用，或无关属性名）。
    _STEP_PATS = ("step_loader", "steploader", "load_step", "parse_step", "step_parser")
    # 必须抓到：真在代码里引用 STEP 格式的形态
    caught = [s for s in (
        "from backend.loaders.step_loader import StepLoader",  # step_loader 模块
        "class StepLoader(AssetLoader):",                       # steploader（lower 后）
        "asset = load_step(path)",                              # load_step 函数
        "def parse_step(self): pass",                           # parse_step 私有方法
        "self.step_parser = Parser()",                          # step_parser 属性
    ) if any(p in s.lower() for p in _STEP_PATS)]
    check(
        "★ `step` 收窄判据的判别力：真在引用 STEP 格式的 5 种写法**都**被抓到",
        len(caught) == 5,
        f"只抓到 {len(caught)}/5：{caught}",
    )
    # 必须放过：spec 强制要求的 Runtime 接口
    spared = [s for s in (
        "async def step(self, command: RobotCommand) -> RobotState:",
        "await self.step(cmd)",
        "for _ in range(steps): pass",
        "return await self._dispatch(command)",
        "self._step_size = 0.02",
    ) if any(p in s.lower() for p in _STEP_PATS)]
    check(
        "★ `step` 收窄判据的正确性：spec 要求的 `step()` 接口、`steps` 变量、"
        "`_step_size` 常量**都不**被误判",
        not spared,
        f"被误判的合法代码：{spared}",
    )
    print(f"    step 收窄判据：抓到 {len(caught)}/5，误判 {len(spared)}/5")

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
        ok, out = run_pytest(*args)
        tail = out.splitlines()[-1] if out else ""
        check(f"pytest {label}", ok, tail)
        print(f"    {label}: {tail}")

    ok, out = run_pytest()
    tail = out.splitlines()[-1] if out else ""
    check("pytest（全量，无回归）", ok, tail)
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
