#!/usr/bin/env python
"""Phase 1 验收清单（对应 spec §58）。

## 用法

```bash
.venv/Scripts/python.exe tools/accept_phase1.py
```

退出码 0 = 全部通过；非 0 = 有未通过项。

## 为什么要有这个脚本

`pytest` 只回答"测试是否全绿"，不回答"这个 Phase 是否**完整**"。
两者不同：一个 Phase 的验收包含"某文件是否存在""某接口是否冻结"
"某约定是否被跨模块遵守"这类**结构事实**，它们不属于任何单元测试。

把清单脚本化的好处：
- 它本身是**可执行的规格说明** —— 读它就知道 Phase 1 要求什么；
- 未通过项直接打印原因，不需要人去翻文档对照；
- Phase 2 结束时可以只看"Phase 1 是否仍然通过"来确认没有回归。

## 设计原则

每一项都做到**独立可判**：失败时给出"期望 vs 实际"，而不是只说 fail。
不依赖网络、不依赖硬件、不修改任何文件。
"""

from __future__ import annotations

import ast
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

_results: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    _results.append((name, bool(ok), detail))


def run_pytest(*args: str) -> tuple[int, str]:
    """跑 pytest，返回 (失败数, 输出尾部)。"""
    proc = subprocess.run(
        [str(PY), "-m", "pytest", *args, "-q", "--no-header"],
        cwd=ROOT,
        capture_output=True,
        text=True,
    )
    out = (proc.stdout + proc.stderr).strip()
    return proc.returncode, out


def syntax_clean(path: pathlib.Path) -> tuple[bool, str]:
    """AST 能解析 = 语法正确。比 import 更轻（不执行副作用）。"""
    try:
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    except SyntaxError as e:
        return False, f"{e.msg} @ line {e.lineno}"
    return True, ""


def strip_comments_and_strings(src: str) -> str:
    return tokenize.untokenize(
        tok
        for tok in tokenize.generate_tokens(io.StringIO(src).readline)
        if tok.type not in (tokenize.COMMENT, tokenize.STRING)
    )


def main() -> int:
    print("=" * 72)
    print("RobotForge · Phase 1 验收清单（spec §58）")
    print("=" * 72)

    # ---------------------------------------------------------------- 骨架
    print("\n[1] 工程骨架")
    for rel in (
        "backend/model/robot_model.py",
        "backend/model/types.py",
        "backend/model/validator.py",
        "backend/loaders/loader.py",   # Loader ABC（RobotModelLoader）
        "backend/loaders/mjcf_loader.py",
        "packages/mini_arm/manifest.yaml",
        "packages/mini_arm/model/mini_arm.xml",
        "packages/mini_arm/kinematics/fk.py",
        "packages/mini_arm/kinematics/ik.py",
    ):
        p = ROOT / rel
        check(f"存在 {rel}", p.is_file(), "缺失" if not p.is_file() else "")

    # ---------------------------------------------------------------- 规格文件
    print("\n[2] spec 要求的测试文件")
    required = [
        "tests/test_manifest.py",
        "tests/test_mjcf_loader.py",
        "tests/test_robot_model.py",
        "tests/test_coordinate.py",
        "tests/test_units.py",
        "tests/test_fk.py",
        "tests/test_ik.py",
        # spec 未列但 Phase 1 必须有（validator 是加载链的一环）
        "tests/test_validator.py",
    ]
    for rel in required:
        p = ROOT / rel
        n = 0
        if p.is_file():
            n = p.read_text(encoding="utf-8").count("    def test_")
        check(f"{rel} 存在且含测试", n > 0, f"{n} 个 test_ 方法" if p.is_file() else "缺失")

    # ---------------------------------------------------------------- 语法
    print("\n[3] 源码语法")
    bad = []
    for p in sorted(ROOT.glob("backend/**/*.py")) + sorted(
        ROOT.glob("packages/**/*.py")
    ):
        if ".venv" in p.parts or "__pycache__" in p.parts:
            continue
        ok, why = syntax_clean(p)
        if not ok:
            bad.append(f"{p.relative_to(ROOT)}: {why}")
    check("全部 .py 可解析", not bad, "\n    ".join(bad))

    # ---------------------------------------------------------------- 测试
    print("\n[4] 测试套件")
    rc, out = run_pytest()
    tail = out.splitlines()[-1] if out else "(无输出)"
    check("pytest (Core) 全绿", rc == 0, tail)
    print(f"    {tail}")

    rc2, out2 = run_pytest("packages/mini_arm/tests")
    tail2 = out2.splitlines()[-1] if out2 else "(无输出)"
    check("pytest (mini_arm) 全绿", rc2 == 0, tail2)
    print(f"    {tail2}")

    rc3, out3 = run_pytest("-m", "slow")
    check("slow marker 可用", "deselected" in out3 or rc3 == 0, out3.splitlines()[-1] if out3 else "")

    # ---------------------------------------------------------------- 契约
    print("\n[5] P0 契约（冻结项）")
    sys.path.insert(0, str(ROOT))
    try:
        from backend.loaders.mjcf_loader import MJCFLoader
        from backend.model.robot_model import (
            ROBOTFORGE_COORDINATE,
            ROBOTFORGE_UNITS,
            RobotModel,
        )
        from backend.model.validator import validate_robot_model
        from backend.model.types import Quaternion, Transform, Vector3

        # 坐标系
        check(
            "坐标系冻结为 +X前/+Y左/+Z上",
            ROBOTFORGE_COORDINATE
            == {
                "convention": "robotforge",
                "handedness": "right",
                "forward_axis": "x",
                "left_axis": "y",
                "up_axis": "z",
            },
            f"实际 = {ROBOTFORGE_COORDINATE}",
        )

        # 单位
        check(
            "单位冻结为 SI (m/rad/s)",
            ROBOTFORGE_UNITS["length"] == "m"
            and ROBOTFORGE_UNITS["angle"] == "rad"
            and ROBOTFORGE_UNITS["time"] == "s",
            f"实际 = {ROBOTFORGE_UNITS}",
        )

        # 右手系：X × Y = Z
        ux, uy = Vector3.unit_x(), Vector3.unit_y()
        cross = ux.cross(uy)
        check(
            "X × Y = Z（右手系）",
            cross.approx_eq(Vector3.unit_z(), tol=1e-15),
            f"X×Y = {cross}",
        )

        # 四元数约定 [x, y, z, w]
        q = Quaternion.identity()
        check(
            "四元数约定 [x,y,z,w]（identity = w=1）",
            (q.x, q.y, q.z, q.w) == (0.0, 0.0, 0.0, 1.0),
            f"实际 = {q}",
        )

        # Transform 组合要转 child 平移
        a = Transform(Vector3.zero(), Quaternion.from_axis_angle(Vector3.unit_z(), 1.5707963267948966))
        b = Transform(Vector3(1.0, 0.0, 0.0), Quaternion.identity())
        composed = a.compose(b)
        check(
            "compose 会旋转 child 的平移",
            composed.position.approx_eq(Vector3(0.0, 1.0, 0.0), tol=1e-9),
            f"a∘b 的平移 = {composed.position}（期望 (0,1,0)）",
        )

        # ------------------------------------------------------------ 数据
        print("\n[6] mini_arm 数据链")
        loaded = MJCFLoader().load(ROOT / "packages" / "mini_arm" / "model" / "mini_arm.xml")
        model: RobotModel = loaded[0] if isinstance(loaded, tuple) else loaded
        report = validate_robot_model(model)
        check("mini_arm 零 error 零 warning", report.ok and not report.warnings,
              report.format() if not report.ok else f"{len(report.warnings)} warning")
        check(
            "mini_arm 有 end_effector",
            model.default_end_effector() is not None,
            "无默认末端",
        )
        check(
            "mini_arm dof == 3",
            model.dof() == 3,
            f"实际 dof = {model.dof()}",
        )

        # FK 零位 TCP 应在 x = L1 + L2 + L_TOOL = 0.200
        sys.path.insert(0, str(ROOT / "packages" / "mini_arm" / "kinematics"))
        import importlib.util

        spec = importlib.util.spec_from_file_location(
            "_acc_fk", ROOT / "packages" / "mini_arm" / "kinematics" / "fk.py"
        )
        fk = importlib.util.module_from_spec(spec)
        sys.modules["_acc_fk"] = fk
        spec.loader.exec_module(fk)
        t = fk.forward_kinematics(model, {"base_yaw": 0.0, "shoulder": 0.0, "elbow": 0.0})
        check(
            "FK 零位返回 TCP (x=0.200)",
            abs(t.position.x - 0.200) < 1e-9,
            f"实际 x = {t.position.x}（0.168 说明返回法兰）",
        )

        # ------------------------------------------------------------ 架构
        print("\n[7] 架构约束")
        offenders = []
        for p in sorted(ROOT.glob("backend/**/*.py")):
            if "__pycache__" in p.parts:
                continue
            src = strip_comments_and_strings(p.read_text(encoding="utf-8"))
            for lineno, line in enumerate(src.splitlines(), 1):
                if "mini_arm" in line or "mearm" in line.lower():
                    offenders.append(f"{p.relative_to(ROOT)}:{lineno}")
        check(
            "Core (backend/) 无型号特定分支",
            not offenders,
            "命中: " + ", ".join(offenders),
        )

        # RobotModel 冻结
        try:
            model.root_link = "hacked"  # type: ignore[misc]
            frozen = False
        except Exception:
            frozen = True
        check("RobotModel 不可变（frozen）", frozen, "能改 root_link ⇒ 未冻结")

    except Exception as e:  # noqa: BLE001
        check("P0 契约检查可执行", False, f"{type(e).__name__}: {e}")

    # ---------------------------------------------------------------- 汇总
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
    print(f"Phase 1 验收：{passed}/{total} 通过")
    if passed != total:
        print("❌ Phase 1 未通过 —— 依据 spec：不得进入下一 Phase")
        return 1
    print("✅ Phase 1 通过 —— 允许进入 Phase 2")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
