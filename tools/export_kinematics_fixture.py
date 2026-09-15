"""导出**运动学真值夹具**：Python 侧独立推导 → JSON，供 JS 侧比对。

## 为什么需要这个文件

`packages/mini_arm/kinematics/kinematics.js` 是 FK/IK 的**第三份实现**。
"它和 Python 侧一致"这件事不能靠人工读代码确认 —— 那是一份会随
时间失效的断言。本文件把它变成**机器判据**：

```text
Python 侧（本文件）                       JS 侧（kinematics.js）
  fk.py::forward_kinematics   ──真值──▶   forwardKinematics()
  ik.py::solve                ──真值──▶   solveIk()
  backend/kinematics/fk.py    ──真值──▶   （第三方裁判，见下）
```

## 三方各自的角色（不可混淆）

```text
① Core 通用链式  真读 model 的几何，逐级 compose
② 包内 Python 解析  几何常量写死，三角公式
③ 包内 JS 解析      ← 本次比对对象，与 ② 同公式
```

- **②↔③** 应当**逐位一致**（同一套公式、IEEE754 double，容差 ~1e-15）
- **①↔②** 应当一致到链式相乘的累计舍入（容差 ~1e-12），
  这条**不是**本次重点：它已被 `packages/mini_arm/tests/` 钉住。
  这里带上它，是为了让 JS 侧能顺带知道自己与**独立路径**也对得上。

## ⚠️ 这个夹具**不能**证明"JS 读对了 MJCF"

解析解的几何长度是文件内写死的常量（② 的既有设计，③ 继承）。
改 MJCF 的几何时 ②③ 的输出**都不变**。几何正确性由
`packages/mini_arm/tests/test_kinematics_facts.py`（常量 vs MJCF）
与 Core 那条路径负责。

## 采样点为什么不是纯随机

纯随机会**几乎永远**采不到退化位形（θ2 = 0 测度为零），
而退化正是分支判据最容易写错的地方。所以显式包含：

```text
- 完全伸展  θ2 = 0        （两分支合并）
- 完全折叠  θ2 = ±π       （死区球边界）
- 限位边界  θ1 = ±π/2, θ2 = ±3π/4
- 底座 yaw  ∈ {0, ±π/2, π}（第二/三象限，atan2 陷阱）
- 随机点    覆盖一般情形
```

## 输出

`frontend/src/sim/__tests__/fixtures/kinematics.json`

```json
{
  "constants": {"BASE_HEIGHT": 0.084, ...},
  "fkCases": [{"name": "...", "q": {...}, "analytic": {...}, "generic": {...}}],
  "ikCases": [{"name": "...", "target": {...}, "branch": "...",
               "expect": {"joints": {...}, "branch": "...", "positionError": ...}}],
  "ikErrors": [{"name": "...", "target": {...}, "errorType": "UnreachableError"}]
}
```
"""

from __future__ import annotations

import importlib.util
import json
import math
import random
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from backend.model.types import Quaternion, Transform, Vector3  # noqa: E402

PKG_KINEMATICS = ROOT / "packages" / "mini_arm" / "kinematics"
FIXTURE = ROOT / "frontend" / "src" / "sim" / "__tests__" / "fixtures" / "kinematics.json"

_MODULES: dict[str, Any] = {}


def _load_by_path(name: str, path: Path):
    """按文件路径加载包内模块（`packages/` 刻意不是 Python package）。

    ⚠ `sys.modules[name] = mod` **不能省**：Python 3.13 的 `@dataclass`
      会通过 `sys.modules[cls.__module__]` 反查模块来解析字符串注解。
    """
    if name in _MODULES:
        return _MODULES[name]
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"无法为 {path} 建立 import spec")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    _MODULES[name] = mod
    return mod


def _load_model():
    from backend.loaders.mjcf_loader import MJCFLoader

    model_path = ROOT / "packages" / "mini_arm" / "model" / "mini_arm.xml"
    loader = MJCFLoader(robot_id="mini_arm")
    model, _report = loader.load(model_path)
    return model


# ---------------------------------------------------------------------------
# 采样：显式构造 + 随机
# ---------------------------------------------------------------------------

#: 退化与边界位形 —— 这些点纯随机会永远采不到，必须显式列出
NAMED_CONFIGS: list[tuple[str, dict[str, float]]] = [
    ("zero_pose", {"base_yaw": 0.0, "shoulder": 0.0, "elbow": 0.0}),
    # 完全伸展：θ1=θ2=0 时 r 最大；θ2=0 ⇒ 两分支合并（退化）
    ("fully_extended", {"base_yaw": 0.0, "shoulder": 0.0, "elbow": 0.0}),
    ("fully_extended_yaw90", {"base_yaw": math.pi / 2, "shoulder": 0.0, "elbow": 0.0}),
    # 完全折叠：θ2 = ±π ⇒ 肘对折（退化边界）
    ("folded_elbow_pos", {"base_yaw": 0.0, "shoulder": 0.0, "elbow": math.pi}),
    ("folded_elbow_neg", {"base_yaw": 0.0, "shoulder": 0.0, "elbow": -math.pi}),
    # 限位边界（与 MJCF range 一致）
    ("shoulder_at_limit", {"base_yaw": 0.0, "shoulder": math.pi / 2, "elbow": 0.0}),
    ("shoulder_at_neg_limit", {"base_yaw": 0.0, "shoulder": -math.pi / 2, "elbow": 0.0}),
    ("elbow_at_limit", {"base_yaw": 0.0, "shoulder": 0.0, "elbow": 3 * math.pi / 4}),
    ("elbow_at_neg_limit", {"base_yaw": 0.0, "shoulder": 0.0, "elbow": -3 * math.pi / 4}),
    # 底座偏航：第二/三象限（atan2 陷阱只在 x<0 时暴露）
    ("yaw_90", {"base_yaw": math.pi / 2, "shoulder": 0.3, "elbow": -0.4}),
    ("yaw_180", {"base_yaw": math.pi, "shoulder": 0.3, "elbow": -0.4}),
    ("yaw_neg_90", {"base_yaw": -math.pi / 2, "shoulder": 0.3, "elbow": -0.4}),
    ("yaw_neg_135", {"base_yaw": -3 * math.pi / 4, "shoulder": 0.5, "elbow": 0.6}),
    ("yaw_135", {"base_yaw": 3 * math.pi / 4, "shoulder": -0.5, "elbow": -0.6}),
    # 一般位形（两分支都非退化）
    ("typical_up", {"base_yaw": 0.4, "shoulder": 0.5, "elbow": -0.8}),
    ("typical_down", {"base_yaw": 0.4, "shoulder": 1.1, "elbow": 0.9}),
]

#: 随机采样个数（固定种子 ⇒ 夹具可复现）
RANDOM_CASES = 40
RANDOM_SEED = 20260915


def _random_configs(rng: random.Random) -> list[tuple[str, dict[str, float]]]:
    out: list[tuple[str, dict[str, float]]] = []
    for i in range(RANDOM_CASES):
        out.append(
            (
                f"random_{i:02d}",
                {
                    "base_yaw": rng.uniform(-math.pi, math.pi),
                    "shoulder": rng.uniform(-math.pi / 2, math.pi / 2),
                    "elbow": rng.uniform(-3 * math.pi / 4, 3 * math.pi / 4),
                },
            )
        )
    return out


def _pose_to_dict(t: Transform) -> dict[str, list[float]]:
    return {"position": t.position.to_list(), "orientation": t.orientation.to_list()}


def _all_configs() -> list[tuple[str, dict[str, float]]]:
    rng = random.Random(RANDOM_SEED)
    return NAMED_CONFIGS + _random_configs(rng)


# ---------------------------------------------------------------------------
# 导出
# ---------------------------------------------------------------------------


def build_fixture() -> dict[str, Any]:
    fk_mod = _load_by_path("mini_arm_pkg_fk_export", PKG_KINEMATICS / "fk.py")
    ik_mod = _load_by_path("mini_arm_pkg_ik_export", PKG_KINEMATICS / "ik.py")
    model = _load_model()

    from backend.kinematics.fk import forward_kinematics as core_fk

    # ---- 常量 ----
    constants = {
        "BASE_HEIGHT": fk_mod.BASE_HEIGHT,
        "SHOULDER_OFFSET": fk_mod.SHOULDER_OFFSET,
        "L1": fk_mod.L1,
        "L2": fk_mod.L2,
        "L_TOOL": fk_mod.L_TOOL,
        "L2_EFF": ik_mod.L2_EFF,
        "Z_BASE": ik_mod.Z_BASE,
        "JOINT_ORDER": list(fk_mod.JOINT_ORDER),
        "YAW_LIMIT": ik_mod.YAW_LIMIT,
        "SHOULDER_LIMIT": ik_mod.SHOULDER_LIMIT,
        "ELBOW_LIMIT": ik_mod.ELBOW_LIMIT,
        "TOL_SINGULAR": ik_mod.TOL_SINGULAR,
    }

    # ---- FK 用例：解析解 + Core 通用解 两条路径 ----
    fk_cases: list[dict[str, Any]] = []
    for name, q in _all_configs():
        analytic = fk_mod.forward_kinematics(model, q)
        generic = core_fk(model, q)
        fk_cases.append(
            {
                "name": name,
                "q": dict(q),
                "analytic": _pose_to_dict(analytic),
                "generic": _pose_to_dict(generic),
            }
        )

    # ---- FK 契约：缺关节按 0、多余键忽略 ----
    fk_contracts: list[dict[str, Any]] = []
    # ① 只给 shoulder，其余按 0
    partial_a = fk_mod.forward_kinematics(model, {"shoulder": 0.7})
    partial_a_ref = fk_mod.forward_kinematics(
        model, {"base_yaw": 0.0, "shoulder": 0.7, "elbow": 0.0}
    )
    fk_contracts.append(
        {
            "name": "missing_joint_defaults_to_zero",
            "q": {"shoulder": 0.7},
            "analytic": _pose_to_dict(partial_a),
            "reference": _pose_to_dict(partial_a_ref),
        }
    )
    # ② 空字典 = 全零
    empty = fk_mod.forward_kinematics(model, {})
    empty_ref = fk_mod.forward_kinematics(
        model, {"base_yaw": 0.0, "shoulder": 0.0, "elbow": 0.0}
    )
    fk_contracts.append(
        {
            "name": "empty_dict_equals_zero_pose",
            "q": {},
            "analytic": _pose_to_dict(empty),
            "reference": _pose_to_dict(empty_ref),
        }
    )
    # ③ 多余键被忽略（模拟"上一个模型的位姿字典"）
    extra = fk_mod.forward_kinematics(
        model, {"base_yaw": 0.2, "shoulder": 0.3, "elbow": 0.4, "ghost_joint": 99.0}
    )
    extra_ref = fk_mod.forward_kinematics(model, {"base_yaw": 0.2, "shoulder": 0.3, "elbow": 0.4})
    fk_contracts.append(
        {
            "name": "extra_joint_id_ignored",
            "q": {"base_yaw": 0.2, "shoulder": 0.3, "elbow": 0.4, "ghost_joint": 99.0},
            "analytic": _pose_to_dict(extra),
            "reference": _pose_to_dict(extra_ref),
        }
    )

    # ---- IK 用例：对可达位形求逆，两种分支 ----
    ik_cases: list[dict[str, Any]] = []
    ik_errors: list[dict[str, Any]] = []

    # 用一组"一定能解到"的目标：先由 FK 生成位形，再让 IK 解回去（往返）
    rng = random.Random(RANDOM_SEED + 1)
    ik_targets: list[tuple[str, Transform]] = []
    for name, q in _all_configs():
        # 只取两分支非退化的位形作为往返目标
        t2 = q["elbow"]
        if abs(abs(t2)) < 1e-3 or abs(abs(t2) - math.pi) < 1e-3:
            continue
        ik_targets.append((f"roundtrip_{name}", fk_mod.forward_kinematics(model, q)))
    # 补几个明确的解析目标
    reach_min, reach_max = ik_mod.reach_limits()
    mid_r = 0.5 * (reach_min + reach_max)
    for label, yaw in (("yaw0", 0.0), ("yaw90", math.pi / 2), ("yaw180", math.pi), ("yawN90", -math.pi / 2)):
        pos = Vector3(mid_r * math.cos(yaw), mid_r * math.sin(yaw), ik_mod.Z_BASE)
        ik_targets.append(
            (
                f"analytic_target_{label}",
                Transform(pos, fk_mod.forward_kinematics(
                    model, {"base_yaw": yaw, "shoulder": 0.0, "elbow": 0.0}
                ).orientation),
            )
        )

    for name, target in ik_targets:
        for branch in ("elbow_up", "elbow_down"):
            try:
                sol = ik_mod.solve(model, target, branch=branch)
            except (ik_mod.UnreachableError, ik_mod.LimitViolationError) as exc:
                ik_errors.append(
                    {
                        "name": f"{name}__{branch}",
                        "target": _pose_to_dict(target),
                        "branch": branch,
                        "errorType": type(exc).__name__,
                    }
                )
                continue
            ik_cases.append(
                {
                    "name": f"{name}__{branch}",
                    "target": _pose_to_dict(target),
                    "branch": branch,
                    "expect": {
                        "joints": dict(sol.joint_positions),
                        "branch": sol.branch,
                        "positionError": sol.position_error,
                        "orientationError": sol.orientation_error,
                    },
                }
            )

    # ---- IK 错误路径：不可达（太远 / 太近）----
    too_far = Transform(Vector3(reach_max + 0.05, 0.0, ik_mod.Z_BASE), Quaternion.identity())
    too_near = Transform(Vector3(max(reach_min - 0.02, 1e-4), 0.0, ik_mod.Z_BASE), Quaternion.identity())
    for label, tgt in (("unreachable_far", too_far), ("unreachable_near", too_near)):
        try:
            ik_mod.solve(model, tgt)
            raise AssertionError(f"夹具构造错误：{label} 竟然解出来了（说明判据选错了目标）")
        except ik_mod.UnreachableError as exc:
            ik_errors.append(
                {
                    "name": label,
                    "target": _pose_to_dict(tgt),
                    "branch": "elbow_up",
                    "errorType": type(exc).__name__,
                    "distance": exc.distance,
                    "reachMin": exc.reach_min,
                    "reachMax": exc.reach_max,
                }
            )

    # ---- 未知 branch ⇒ IkError ----
    reach_mid = Transform(Vector3(mid_r, 0.0, ik_mod.Z_BASE), Quaternion.identity())
    try:
        ik_mod.solve(model, reach_mid, branch="no_such_branch")
        raise AssertionError("夹具构造错误：未知 branch 竟然没抛错")
    except ik_mod.IkError as exc:
        if isinstance(exc, (ik_mod.UnreachableError, ik_mod.LimitViolationError)):
            raise AssertionError(f"夹具构造错误：应抛 IkError，实得 {type(exc).__name__}") from exc
        ik_errors.append(
            {
                "name": "unknown_branch",
                "target": _pose_to_dict(reach_mid),
                "branch": "no_such_branch",
                "errorType": "IkError",
            }
        )

    return {
        "constants": constants,
        "fkCases": fk_cases,
        "fkContracts": fk_contracts,
        "ikCases": ik_cases,
        "ikErrors": ik_errors,
    }


def main() -> int:
    fixture = build_fixture()
    FIXTURE.parent.mkdir(parents=True, exist_ok=True)
    FIXTURE.write_text(
        json.dumps(fixture, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    print(f"写入 {FIXTURE.relative_to(ROOT)}")
    print(f"  FK 用例    : {len(fixture['fkCases'])}（含解析解 + Core 通用解）")
    print(f"  FK 契约    : {len(fixture['fkContracts'])}")
    print(f"  IK 用例    : {len(fixture['ikCases'])}")
    print(f"  IK 错误路径: {len(fixture['ikErrors'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
