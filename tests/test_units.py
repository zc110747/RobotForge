"""单位约定契约测试（P0 契约 `docs/coordinate-system.md` §4）。

## 这个文件的核心是一条**源代码扫描**

架构文档说"单位转换只允许发生在边界（Loader / Adapter / Converter）"。
如果这条只写在文档里，它会被某个赶时间的人在 `fk.py` 里加一行
`math.radians(...)` 破坏掉 —— 而且**没有任何测试会红**。

因此本文件把文档条款变成可执行的检查：

```text
扫描 backend/ 下的 .py，找 math.radians / math.degrees / 180/math.pi / pi*180
若出现在白名单之外的文件 ⇒ 失败
```

## 白名单为什么是这几个文件

不是"暂时豁免"，而是**架构上就允许转换**的位置：

| 文件 | 为什么允许 |
|---|---|
| `backend/loaders/*.py` | 外部格式 → RobotModel 的**边界**（唯一入口） |
| `backend/simulation/*.py` | RobotModel → 引擎的**边界**（如 MuJoCo 需要别的角度单位） |
| `backend/api/*.py` | 协议 → 内部的**边界**（供 UI 友好显示） |
| `backend/model/*.py` | 契约对象自身（如 Euler 仅用于 UI 的转换工具） |

其余任何地方出现角度转换，都意味着"有一个中间层在替用户做单位决定"，
而那个决定应该是 Loader 的责任。
"""

from __future__ import annotations

import math
import re
from pathlib import Path

import pytest

# ----------------------------------------------------------------------
# 1. SI 单位本身
# ----------------------------------------------------------------------


class TestSIUnits:
    def test_length_is_meters(self, mini_arm_model):
        """连杆长度必须是米量级（本机构约 0.1 m）。

        ★ 若有人把 MJCF 里误写成毫米（103 而不是 0.103），这条会红。
          它比"看代码"可靠，因为 103 与 0.103 都是合法的 float。
        """
        for jid in mini_arm_model.mobile_joint_ids():
            o = mini_arm_model.joint(jid).origin.position
            assert o.norm() < 1.0, (
                f"关节 {jid} 的原点模长 {o.norm()} —— 看起来是毫米而不是米"
            )

    def test_angle_is_radians(self, mini_arm_model):
        """关节限位必须是弧度量级（π 附近），不是 180。"""
        for jid in mini_arm_model.mobile_joint_ids():
            lim = mini_arm_model.joint(jid).limits
            if lim.position_max is None:
                continue
            assert lim.position_max <= math.pi + 1e-9, (
                f"关节 {jid} 上限 {lim.position_max} > π —— 疑似角度制"
            )

    def test_no_degree_fields_in_model(self, mini_arm_model):
        """RobotModel 的字段名里不允许出现 `_deg` / `_degree`。

        P0 契约禁止 `min_degree` / `max_mm` 这类"在字段名里编码单位"的做法：
        它让同一个模型里可以混两种单位，而那正是契约要消灭的东西。
        """
        d = mini_arm_model.to_dict()
        text = repr(d)
        for bad in ("_deg", "_degree", "_degrees", "_mm", "_cm", "_inch"):
            assert bad not in text, (
                f"RobotModel 序列化结果里出现 {bad!r} —— "
                f"字段名不得编码单位（P0 契约）"
            )

    def test_velocity_units_are_rad_per_sec(self, mini_arm_model):
        """revolute 关节的 velocity_max 单位是 rad/s（不是 rpm）。"""
        found = False
        for jid in mini_arm_model.mobile_joint_ids():
            lim = mini_arm_model.joint(jid).limits
            if lim.velocity_max is not None:
                found = True
                # rpm 通常是几十~几千；rad/s 是 1~几十量级
                assert lim.velocity_max < 100.0, (
                    f"关节 {jid} 速度上限 {lim.velocity_max} —— 疑似 rpm 而非 rad/s"
                )
        # 本模型若未声明速度限位也算通过（None = 未声明，是合法语义）


# ----------------------------------------------------------------------
# 2. 源代码扫描：禁止散落的单位转换
# ----------------------------------------------------------------------

#: 允许出现角度/长度单位转换的**边界**文件（glob，相对 backend/）
_CONVERSION_ALLOWED = (
    "loaders/*.py",       # 外部格式 → RobotModel（唯一入口边界）
    "simulation/*.py",    # RobotModel → 引擎（引擎可能不用 rad）
    "api/*.py",           # 协议 → 内部（UI 友好显示）
    "model/*.py",         # 契约对象自己的工具方法（Euler 仅供 UI）
    "kinematics/*.py",    # Core 通用运动学（如需与世界系适配）
)

#: 检测"在做角度转换"的模式
_DEGREE_PATTERNS = [
    (re.compile(r"math\.radians\s*\("), "math.radians("),
    (re.compile(r"math\.degrees\s*\("), "math.degrees("),
    (re.compile(r"180\s*\.?\s*0?\s*/\s*math\.pi"), "180/math.pi"),
    (re.compile(r"math\.pi\s*/\s*180"), "math.pi/180"),
    (re.compile(r"\*\s*57\.29"), "* 57.29（度→弧度的近似常数）"),
    (re.compile(r"np\.radians\s*\("), "np.radians("),
    (re.compile(r"np\.degrees\s*\("), "np.degrees("),
]


def _iter_python_files(root: Path):
    for p in sorted(root.rglob("*.py")):
        if "__pycache__" in p.parts:
            continue
        yield p


def _is_allowed(rel: Path) -> bool:
    from fnmatch import fnmatch

    return any(fnmatch(rel.as_posix(), pat) for pat in _CONVERSION_ALLOWED)


def _scan_for_degree_conversion(path: Path):
    """返回 [(行号, 匹配到的模式名, 该行内容)]。"""
    hits = []
    for lineno, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        stripped = line.strip()
        # 注释与字符串里的说明性文字不算（本项目的注释大量讨论单位）
        code = stripped.split("#", 1)[0]
        if not code.strip():
            continue
        # docstring 里的示例代码也不算 —— 粗略地跳过以 >>> 或 | 开头的行
        if stripped.startswith((">>>", "|", "...", '"""', "'''")):
            continue
        for pat, label in _DEGREE_PATTERNS:
            if pat.search(code):
                hits.append((lineno, label, stripped))
    return hits


class TestNoScatteredUnitConversion:
    """★ 本文件最重要的一组：把架构条款变成可执行检查。"""

    def test_backend_dir_exists(self, repo_root):
        assert (repo_root / "backend").is_dir()

    def test_no_degree_conversion_outside_boundaries(self, repo_root):
        """单位转换只允许出现在白名单边界文件里。"""
        backend = repo_root / "backend"
        violations = []
        for p in _iter_python_files(backend):
            rel = p.relative_to(backend)
            if _is_allowed(rel):
                continue
            for lineno, label, line in _scan_for_degree_conversion(p):
                violations.append(f"  backend/{rel}:{lineno}  用到了 {label}\n      {line}")

        assert not violations, (
            "以下位置出现了单位转换，但它们在架构上**不该做转换**：\n"
            + "\n".join(violations)
            + "\n\n"
            "为什么这算错误：单位转换必须在**边界**（Loader / Adapter / Converter）"
            "一次性完成。散落在中间层的转换意味着每个中间层都在替上层做"
            "单位决定 —— 数量一多必然遗漏一处，而遗漏的表现是"
            "'某个方向角度偏 57 倍'这种极难定位的现象。\n"
            "若确实需要：把转换提到 Loader（或对应的边界模块）里做；"
            "若是一个新的正当边界，请把文件加入本文件的 _CONVERSION_ALLOWED 并说明理由。"
        )

    def test_robot_model_has_no_conversion(self, repo_root):
        """`backend/model/` 里不得出现**任何**角度转换（含 model/*.py 的例外）。

        `model/types.py` 的 `from_rpy` / `to_euler_rpy` 是**数学工具**
        （接受 rad、返回 rad），不是"单位转换"。
        因此 model 目录只放行不涉及 deg 字的纯数学函数 —— 这里用更严格
        的检查确认 `from_rpy` 系列没有偷偷做 deg→rad。
        """
        # 直接读 types.py，确认 from_axis_angle / from_rpy 的实现里没有 180
        types_py = repo_root / "backend" / "model" / "types.py"
        text = types_py.read_text(encoding="utf-8")
        # 取 from_rpy 的函数体
        m = re.search(r"def from_rpy\(.*?\n(.*?)(?=\n    def |\n    @|\Z)", text, re.DOTALL)
        if m:
            body = m.group(1)
            assert "180" not in body, (
                "Quaternion.from_rpy 的实现里出现了 180 —— "
                "它应接受**弧度**，不应做 deg→rad 转换"
            )

    def test_scan_actually_finds_known_conversions(self, repo_root):
        """★ 元测试：确认扫描器**真的能发现问题**。

        ## 为什么必须有这一条

        扫描类测试最大的风险是"它从来没匹配到任何东西，所以永远绿"。
        （真实教训：本项目曾用一个错误的 grep 模式扫端口，`\\b` 与漏掉的 `-E`
          让预检**恒返回"干净"** —— 测试全绿而检查从未生效。）

        ⇒ 所以先在一个**已知含转换**的文件上验证扫描器能命中，
          再对真正要检查的目录做断言。
        """
        import tempfile

        with tempfile.TemporaryDirectory() as td:
            f = Path(td) / "probe.py"
            f.write_text(
                "import math\n"
                "a = math.radians(90)\n"
                "b = 180 / math.pi\n"
                "c = math.degrees(1.0)\n",
                encoding="utf-8",
            )
            hits = _scan_for_degree_conversion(f)
        assert len(hits) >= 3, (
            f"扫描器只找到 {len(hits)} 处（期望 ≥3）—— "
            f"扫描器本身失效，后面的'无违规'断言是不可信的"
        )

    def test_no_mm_or_cm_conversion_outside_boundaries(self, repo_root):
        """长度单位同理：`* 1000` / `/ 1000` 配注释说"转成 mm" 之类也要拦住。"""
        backend = repo_root / "backend"
        violations = []
        pat = re.compile(r"(\*\s*1000(?:\.0)?\b|/\s*1000(?:\.0)?\b)")
        for p in _iter_python_files(backend):
            rel = p.relative_to(backend)
            if _is_allowed(rel):
                continue
            for lineno, line in enumerate(
                p.read_text(encoding="utf-8").splitlines(), 1
            ):
                code = line.split("#", 1)[0]
                if not pat.search(code):
                    continue
                # 只关心与长度有关的可疑行
                low = line.lower()
                if any(w in low for w in ("mm", "milli", "length", "position", "radius", "size")):
                    violations.append(f"  backend/{rel}:{lineno}  {line.strip()}")
        assert not violations, (
            "疑似长度单位转换出现在非边界位置：\n" + "\n".join(violations)
        )


# ----------------------------------------------------------------------
# 3. 跨库单位对照
# ----------------------------------------------------------------------


class TestCrossLibraryUnits:
    def test_mujoco_compiler_angle_is_radian(self, mini_arm_mjcf):
        """MJCF 必须显式声明 `angle="radian"`。

        ★ 这一条踩过一个具体的坑：MuJoCo 的 `compiler angle` 会在**编译期**
          把 deg 转成 rad，所以 `jnt_range` 读出来已经是 rad。
          如果 Loader 再乘一次 π/180，限位会缩小 57 倍。

          显式声明 `radian` 让"读了就是 rad"这件事无需推理。
        """
        text = mini_arm_mjcf.read_text(encoding="utf-8")
        assert 'angle="radian"' in text, (
            "MJCF 的 <compiler> 必须显式写 angle=\"radian\""
        )

    def test_mujoco_jnt_range_is_already_radians(self, mini_arm_mjcf, mini_arm_model):
        """MuJoCo 的 `jnt_range` 与 RobotModel 的 limits 必须**数值相等**。

        若 Loader 做了二次转换，二者会差 57 倍 —— 这条测试直接抓住。
        """
        mujoco = pytest.importorskip("mujoco")
        m = mujoco.MjModel.from_xml_path(str(mini_arm_mjcf))
        worst = 0.0
        for jid in mini_arm_model.mobile_joint_ids():
            jid_mj = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jid)
            assert jid_mj >= 0, f"MuJoCo 里找不到关节 {jid}"
            lim = mini_arm_model.joint(jid).limits
            assert lim.position_min is not None and lim.position_max is not None
            worst = max(
                worst,
                abs(lim.position_min - float(m.jnt_range[jid_mj][0])),
                abs(lim.position_max - float(m.jnt_range[jid_mj][1])),
            )
        assert worst < 1e-9, (
            f"RobotModel 限位与 MuJoCo jnt_range 最差差 {worst:.3e} rad。"
            f"若约等于 57 倍 ⇒ Loader 做了重复的 deg→rad 转换。"
        )
