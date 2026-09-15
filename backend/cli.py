"""`python -m backend.cli` —— RobotForge 的命令行入口。

## 它为什么属于 Core，而不是某个包

CLI 是**架构纪律的执行器**，不是便利工具。提示词 §69 规则 2 说 Core 里
不准出现 `if robot.id == "mini_arm"`；而"这条纪律有没有被破坏"这件事，
靠人读代码是查不出来的 —— 必须有一个东西能**跑起来**并拒绝违规。

本文件的 `fk` / `ik` 子命令因此有一个硬约束：

> **实现从 `manifest.yaml` 的 `kinematics.*.entry` 动态加载，
>   本文件里不出现任何机器人名、任何包路径。**

一个反例说明为什么：如果这里写

    from packages.mini_arm.kinematics.ik import solve   # ❌

那么"给 RobotForge 加第二台机器人"就必须改 Core —— 这正是规则 2 禁止的。
正确写法是读 manifest 拿到 `entry: kinematics/ik.py`，按包目录拼出绝对路径，
用 `importlib` 加载。于是加机器人 = 加目录，`cli.py` 一行不动。

这也是 `manifest.kinematics.ik.type` 这个字段的**唯一消费者**：
`type: package` ⇒ 加载包内 `entry`；`type: engine` ⇒ 用 Core 的通用实现
（v0.1 还没有 `engine`，所以现在只有一条分支，但它已经是**数据驱动**的，
不是写死的）。

## 子命令

```text
python -m backend.cli list                          列出所有机器人包
python -m backend.cli show   <id>                   显示推导事实（dof/link/joint…）
python -m backend.cli fk     <id> --joints a=0,b=0   正解：关节角 → TCP 位姿
python -m backend.cli ik     <id> --target x,y,z     逆解：TCP 位置 → 关节角
python -m backend.cli inspect <id>                   模型自检（校验 + 几何清单）
```

`show` 与 `inspect` 的区别是刻意的：`show` 回答"这台机器人是什么"（身份 +
推导出的结构事实），`inspect` 回答"这份模型有没有问题"（Validator 报告 +
被丢弃的几何）。混在一个命令里会让"看一眼规格"和"排查模型"用同一段输出，
而两者的读者和关注点不同。

## 退出码约定

| 码 | 含义 |
|---|---|
| 0 | 成功 |
| 1 | 参数/包不存在等**用法**错误 |
| 2 | **模型或运动学**失败（校验不过、不可达、超限） |

区分 1 与 2 是为了让 CI 能分辨"命令敲错了"和"模型真的坏了"。
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path
from types import ModuleType
from typing import Any, Sequence

from .api.registry import (
    RobotPackage,
    RobotPackageError,
    discover_packages,
    get_package,
)
from .loaders.loader import LoaderError
from .model.robot_model import RobotModel
from .model.types import Transform, Vector3
from .model.validator import validate_robot_model

#: 用法错误（参数、包不存在）
EXIT_USAGE = 1
#: 模型 / 运动学失败（校验不过、不可达、超限）
EXIT_MODEL = 2


# ---------------------------------------------------------------------------
# 动态加载包内运动学实现
# ---------------------------------------------------------------------------


class KinematicsNotAvailable(Exception):
    """manifest 里没有声明该能力，或声明的 entry 无法加载。"""


def _load_package_module(pkg: RobotPackage, entry: str, tag: str) -> ModuleType:
    """按 `manifest.kinematics.<tag>.entry` 加载包内模块。

    ## 为什么必须按**路径**加载，而不是 `import packages.mini_arm...`

    因为 `packages/` 目录**刻意没有 `__init__.py`**（包不是 Python 包，
    是数据 + 可选实现的目录）。所以 `packages.mini_arm.kinematics.ik`
    不是一个可导入的模块名。这反过来是好事：它证明`机器人包 ≠ Python 包`，
    一个用 Rust 写 IK 的包将来也能放进同一套目录结构。

    ## 模块注册进 sys.modules 是必需的，不是保险

    `@dataclass` 在 Python 3.13 下需要一个真实模块项来做反向查找
    （它要读 `sys.modules[cls.__module__].__dict__` 来解析字符串注解）。
    不注册的话，包内的 `IkSolution` 会在实例化时抛
    `AttributeError: ... has no attribute '__dict__'`。
    这一点在 `ik.py` 的 `_load_sibling_fk()` 里已经踩过一次（见其注释），
    这里沿用同一策略。

    `tag` 参与模块名是为了让 fk 与 ik 各自独立注册、互不覆盖。
    """
    module_path = pkg.directory / entry
    if not module_path.is_file():
        raise KinematicsNotAvailable(
            f"{pkg.manifest_path} 声明 kinematics.{tag}.entry = {entry!r}，"
            f"但 {module_path} 不存在"
        )

    mod_name = f"robotforge_pkg_{pkg.id}_{tag}"
    spec = importlib.util.spec_from_file_location(mod_name, module_path)
    if spec is None or spec.loader is None:  # pragma: no cover - 路径已确认存在
        raise KinematicsNotAvailable(f"无法为 {module_path} 建立 import spec")

    module = importlib.util.module_from_spec(spec)
    # 先注册再执行：模块内的 dataclass / 相对重导出需要能反查到它自己
    sys.modules[mod_name] = module
    try:
        spec.loader.exec_module(module)
    except Exception as exc:  # noqa: BLE001 - 真正的错误信息在 exc 里，要原样上抛
        del sys.modules[mod_name]
        raise KinematicsNotAvailable(
            f"加载 {pkg.id} 的 kinematics.{tag}（{module_path}）失败：{exc}"
        ) from exc
    return module


def _kinematics_entry(pkg: RobotPackage, tag: str) -> str | None:
    """从 manifest 读 `kinematics.<tag>.entry`；未声明 ⇒ `None`。

    刻意**不**给默认值（比如兜底 `kinematics/fk.py`）：
    兜底会让"manifest 忘了声明"表现成"一切正常"，而实际上用了
    一个猜出来的路径。声明式配置的价值就在于"没声明"必须是一个可见的状态。
    """
    kin = pkg.raw_manifest.get("kinematics")
    if not isinstance(kin, dict):
        return None
    section = kin.get(tag)
    if not isinstance(section, dict):
        return None
    entry = section.get("entry")
    return str(entry) if entry else None


def _resolve_capability(pkg: RobotPackage, tag: str) -> tuple[str, ModuleType]:
    """返回 `(type, module)`；未声明或不支持 ⇒ `KinematicsNotAvailable`。

    `type: engine` 是预留给 Core 通用实现的分支。v0.1 只有 `package`，
    但这里仍然**读 manifest 而不是假设** —— 否则将来加 `engine`
    就得回来改这个函数，而不是只改 manifest。
    """
    kin = pkg.raw_manifest.get("kinematics")
    section = kin.get(tag) if isinstance(kin, dict) else None
    if not isinstance(section, dict):
        raise KinematicsNotAvailable(
            f"{pkg.id} 的 manifest 未声明 kinematics.{tag}（该机器人不提供此能力）"
        )

    kind = str(section.get("type", ""))
    if kind == "package":
        entry = _kinematics_entry(pkg, tag)
        if entry is None:
            raise KinematicsNotAvailable(
                f"{pkg.id} 的 kinematics.{tag}.type = 'package' 但没有 entry"
            )
        return kind, _load_package_module(pkg, entry, tag)
    if kind == "engine":
        # v0.1 未实现：Core 通用数值 IK 属于后续阶段。
        # 显式抛错而不是静默回退到包内实现 —— 静默回退会让
        # "manifest 说要走引擎" 与 "实际走了包" 长期不一致。
        raise KinematicsNotAvailable(
            f"{pkg.id} 的 kinematics.{tag}.type = 'engine'，但 v0.1 尚未提供 Core 通用实现"
        )
    raise KinematicsNotAvailable(
        f"{pkg.id} 的 kinematics.{tag}.type = {kind!r}，只支持 'package' / 'engine'"
    )


# ---------------------------------------------------------------------------
# 参数解析工具
# ---------------------------------------------------------------------------


def _parse_joint_pairs(text: str) -> dict[str, float]:
    """`"base_yaw=0,shoulder=0.3"` → `{"base_yaw": 0.0, "shoulder": 0.3}`。

    空串 ⇒ 空 dict（调用方负责按"未给 = 0"处理）。
    """
    result: dict[str, float] = {}
    for chunk in text.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        if "=" not in chunk:
            raise ValueError(f"关节项 {chunk!r} 缺少 '='（期望 joint=value）")
        name, _, raw = chunk.partition("=")
        name = name.strip()
        if not name:
            raise ValueError(f"关节项 {chunk!r} 的名字为空")
        try:
            result[name] = float(raw)
        except ValueError as exc:
            raise ValueError(f"关节 {name!r} 的值 {raw!r} 不是数字") from exc
    return result


def _parse_target(text: str) -> Transform:
    """`"0.12,0,0.136"` → `Transform`（位置 + 单位姿态）。

    ## 为什么姿态固定为单位四元数

    因为 CLI 的 IK 是**位置级**查询：mini_arm 的解析解只读 (x, y, z)，
    姿态由机构自身的构型决定，不是一个自由参数。
    强行让用户传姿态会给一个假的自由度 —— 传了也不会改变结果
    （这正是 `TestEngineAgainstModel` 里那条
    "orientation_error ≡ 2·acos(|w|)" 注释所指的性质）。

    允许 3 或 6/7 个分量：`x,y,z` 或 `x,y,z,qx,qy,qz,qw`。
    多给的分量会被真的用上（写进 Transform），这样将来换成
    通用数值 IK 时，CLI 的调用形式不需要变。
    """
    parts = [p.strip() for p in text.split(",") if p.strip()]
    if len(parts) == 3:
        values = [float(p) for p in parts]
        return Transform.from_parts(values, [0.0, 0.0, 0.0, 1.0])
    if len(parts) == 7:
        values = [float(p) for p in parts]
        return Transform.from_parts(values[:3], values[3:])
    raise ValueError(
        f"目标 {text!r} 应含 3 个分量（x,y,z）或 7 个（x,y,z,qx,qy,qz,qw），"
        f"实际 {len(parts)} 个"
    )


def _fmt_vec(values: Sequence[float], digits: int = 6) -> str:
    return "[" + ", ".join(f"{v:.{digits}f}" for v in values) + "]"


def _fmt_quat(values: Sequence[float], digits: int = 6) -> str:
    return "[" + ", ".join(f"{v:+.{digits}f}" for v in values) + "]"


# ---------------------------------------------------------------------------
# 子命令
# ---------------------------------------------------------------------------


def _load_or_fail(robot_id: str) -> RobotPackage:
    """取包；不存在 ⇒ 打印可用列表并退出 1。"""
    try:
        return get_package(robot_id)
    except KeyError as exc:
        print(f"错误：{exc.args[0]}", file=sys.stderr)
        raise SystemExit(EXIT_USAGE) from exc
    except RobotPackageError as exc:
        print(f"错误：机器人包损坏 —— {exc}", file=sys.stderr)
        raise SystemExit(EXIT_MODEL) from exc


def _model_or_fail(pkg: RobotPackage) -> RobotModel:
    """加载模型；失败 ⇒ 退出 2。**不**在此处做校验拦截。

    校验结果由调用方决定要不要拦 —— `show` 想带着 issues 显示
    （好让人看到问题），`fk`/`ik` 则必须先确认模型可用。
    """
    try:
        model, _ = pkg.load_model()
    except LoaderError as exc:
        print(f"错误：{pkg.id} 的模型无法加载 —— {exc}", file=sys.stderr)
        raise SystemExit(EXIT_MODEL) from exc
    return model


def cmd_list(args: argparse.Namespace) -> int:
    try:
        packages = discover_packages()
    except RobotPackageError as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return EXIT_MODEL

    if args.json:
        payload = [p.to_dict() for p in packages]
        print(json.dumps(payload, indent=2, ensure_ascii=False))
        return 0

    if not packages:
        print("（packages/ 下没有发现任何机器人包）")
        return 0

    print(f"发现 {len(packages)} 个机器人包：\n")
    for pkg in packages:
        caps = [k for k, v in sorted(pkg.capabilities.items()) if v]
        print(f"  {pkg.id}  v{pkg.version}  —  {pkg.name}")
        if pkg.description:
            print(f"      {pkg.description}")
        print(f"      能力: {', '.join(caps) if caps else '（无）'}")
        print(f"      目录: {pkg.directory}")
        print()
    return 0


def cmd_show(args: argparse.Namespace) -> int:
    """显示**推导事实** —— 这些值刻意不写在 manifest 里（避免第二份真值）。"""
    pkg = _load_or_fail(args.robot)
    model, report = pkg.load_model()

    mobile = model.mobile_joint_ids()
    # ★ 身份（id/name/version）从 **manifest** 读，不读 model.metadata。
    #   Loader 刻意写 version="0.0.0-from-mjcf" 并注释"真实版本由 manifest 覆盖" ——
    #   MJCF 里没有版本号这个概念，从它猜一个就是第二份真值。
    #   而结构事实（links/joints/dof）反过来必须从 model 推导。
    #   二者各取其源，正是 registry.py 说的"manifest 是包身份的唯一真值源"。
    data: dict[str, Any] = {
        "id": pkg.id,
        "name": pkg.name,
        "version": pkg.version,
        "description": pkg.description,
        "package_dir": str(pkg.directory),
        "model_path": str(pkg.model_path),
        "format": "mjcf",
        "coordinate": model.coordinate.to_dict(),
        "units": model.units.to_dict(),
        "root_link": model.root_link,
        "base_frame": model.base_frame,
        # ---- 推导事实（manifest 里刻意不写）----
        "derived": {
            "dof": model.dof(),
            "link_count": len(model.links),
            "joint_count": len(model.joints),
            "mobile_joints": mobile,
            "fixed_joints": [j.id for j in model.joints if j.type == "fixed"],
            "actuator_count": len(model.actuators),
            "frame_count": len(model.frames),
            "site_count": len(model.sites),
            "end_effector_ids": model.end_effector_ids(),
            "joint_axes": {j.id: j.axis.to_list() if j.axis else None for j in model.joints},
            "joint_origins": {j.id: j.origin.position.to_list() for j in model.joints},
        },
        "kinematics": {
            tag: {
                "type": (pkg.raw_manifest.get("kinematics", {}) or {})
                .get(tag, {})
                .get("type")
                if isinstance((pkg.raw_manifest.get("kinematics", {}) or {}).get(tag), dict)
                else None,
                "entry": _kinematics_entry(pkg, tag),
                "loadable": _entry_loadable(pkg, tag),
            }
            for tag in ("fk", "ik")
        },
        "capabilities": model.capabilities.to_dict(),
        "validation": {"ok": report.ok, "issues": [i.to_dict() for i in report.issues]},
    }

    if args.json:
        print(json.dumps(data, indent=2, ensure_ascii=False))
        return 0

    d = data["derived"]
    print(f"{pkg.id}  v{pkg.version}  —  {pkg.name}")
    if pkg.description:
        print(f"  {pkg.description}")
    print()
    print(f"  模型        : {pkg.model_path.relative_to(pkg.directory.parent.parent)}")
    print(f"  坐标契约    : {model.coordinate.convention} · "
          f"+{model.coordinate.forward_axis}=前 / +{model.coordinate.left_axis}=左 / "
          f"+{model.coordinate.up_axis}=上 · {model.coordinate.handedness}手系")
    print(f"  单位        : {model.units.length} · {model.units.angle} · {model.units.torque}")
    print(f"  根 link     : {model.root_link}      基准系: {model.base_frame}")
    print()
    print(f"  --- 推导事实（manifest 刻意不存，防止第二份真值）---")
    print(f"  自由度      : dof = {d['dof']}   活动关节 = {', '.join(d['mobile_joints']) or '（无）'}")
    print(f"  结构        : {d['link_count']} links / {d['joint_count']} joints "
          f"（固定 {len(d['fixed_joints'])}）/ {d['actuator_count']} actuators")
    print(f"  参考系      : {d['frame_count']} frames / {d['site_count']} sites / "
          f"{len(d['end_effector_ids'])} ee")
    print()
    print(f"  --- 关节表 ---")
    for j in model.joints:
        axis = _fmt_vec(j.axis.to_list(), 3) if j.axis else "—"
        origin = _fmt_vec(j.origin.position.to_list())
        lim = ""
        if j.limits is not None and j.limits.position_min is not None:
            lim = f"  限位 [{j.limits.position_min:+.4f}, {j.limits.position_max:+.4f}]"
        else:
            # 无限位 = 关节可自由旋转。显式打出来而不是留空 ——
            # 留空会让人以为"忘了填"，而无限位是合法且有意义的状态。
            lim = "  限位 无（自由旋转）"
        print(f"  {j.id:<16} {j.type:<10} 轴 {axis:<14} 原点 {origin}{lim}")
    print()
    print(f"  --- 运动学实现（来自 manifest，非硬编码）---")
    for tag, info in data["kinematics"].items():
        if info["type"] is None:
            print(f"  {tag.upper():<3}: （未声明）")
        else:
            mark = "✅" if info["loadable"] else "❌"
            print(f"  {tag.upper():<3}: type={info['type']}  entry={info['entry']}  可加载={mark}")
    print()
    caps = [k for k, v in sorted(model.capabilities.to_dict().items()) if v]
    print(f"  能力        : {', '.join(caps) or '（无）'}")
    ok = "✅ 通过" if report.ok else f"❌ {len(report.issues)} 个问题"
    print(f"  模型校验    : {ok}")
    return 0


def _entry_loadable(pkg: RobotPackage, tag: str) -> bool:
    """entry 文件是否存在（**不执行**它 —— `show` 不该有副作用）。

    只查存在性：真的 import 一遍会让 `show` 在包内实现有语法错时失败，
    而 `show` 的职责是"报告 manifest 声明了什么"。
    """
    entry = _kinematics_entry(pkg, tag)
    if entry is None:
        return False
    return (pkg.directory / entry).is_file()


def cmd_fk(args: argparse.Namespace) -> int:
    """正解：关节角 → TCP 位姿。实现从 manifest 动态加载。"""
    pkg = _load_or_fail(args.robot)
    try:
        kind, module = _resolve_capability(pkg, "fk")
    except KinematicsNotAvailable as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return EXIT_USAGE

    model = _model_or_fail(pkg)
    report = validate_robot_model(model)
    if not report.ok and not args.force:
        print(f"错误：{pkg.id} 的模型未通过校验，拒绝求正解：", file=sys.stderr)
        for issue in report.issues:
            print(f"  - [{issue.severity}] {issue.code}: {issue.message}", file=sys.stderr)
        print("（加 --force 可忽略校验继续）", file=sys.stderr)
        return EXIT_MODEL

    try:
        joints = _parse_joint_pairs(args.joints)
    except ValueError as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return EXIT_USAGE

    unknown = sorted(set(joints) - set(model.joint_ids()))
    if unknown:
        # 提示但不拦：Core FK 的契约是"多余的 joint_id 被忽略"。
        # 这里多打一行是 CLI 的增值（人敲错了要能看见），
        # 不改变引擎语义。
        print(f"提示：以下 joint_id 不在模型里，已忽略：{unknown}", file=sys.stderr)

    fn = getattr(module, "forward_kinematics", None)
    if fn is None:
        print(
            f"错误：{pkg.id} 的 fk 模块（type={kind}）没有 forward_kinematics()",
            file=sys.stderr,
        )
        return EXIT_MODEL

    pose = fn(model, joints)
    data = {
        "robot": pkg.id,
        "joints": joints,
        "tcp": pose.to_dict(),
        "position": pose.position.to_list(),
        "orientation": pose.orientation.to_list(),
    }
    if args.json:
        print(json.dumps(data, indent=2, ensure_ascii=False))
        return 0

    print(f"{pkg.id} · 正解（FK）")
    given = ", ".join(f"{k}={v:+.4f}" for k, v in sorted(joints.items())) or "（全零位）"
    print(f"  关节角 : {given}")
    print(f"  TCP 位置: {_fmt_vec(pose.position.to_list())}  m")
    print(f"  TCP 姿态: {_fmt_quat(pose.orientation.to_list())}  (x,y,z,w)")
    return 0


def cmd_ik(args: argparse.Namespace) -> int:
    """逆解：TCP 位置 → 关节角。实现从 manifest 动态加载。"""
    pkg = _load_or_fail(args.robot)
    try:
        kind, module = _resolve_capability(pkg, "ik")
    except KinematicsNotAvailable as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return EXIT_USAGE

    model = _model_or_fail(pkg)

    try:
        target = _parse_target(args.target)
    except ValueError as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return EXIT_USAGE

    fn = getattr(module, "solve", None)
    if fn is None:
        print(
            f"错误：{pkg.id} 的 ik 模块（type={kind}）没有 solve()",
            file=sys.stderr,
        )
        return EXIT_MODEL

    kwargs: dict[str, Any] = {"branch": args.branch, "clamp": args.clamp}
    if hasattr(module, "TOL_REACH"):
        kwargs["tol"] = float(module.TOL_REACH)

    try:
        solution = fn(model, target, **kwargs)
    except Exception as exc:  # noqa: BLE001 - IkError 家族在包内，Core 不该 import 它
        # ★ 刻意不 `from packages...ik import IkError`：
        #   Core 依赖包内异常 = 反向依赖。这里按**名字**兜住，
        #   并把原始类型名打出来（诊断需要它）。
        name = type(exc).__name__
        if name in {"UnreachableError", "LimitViolationError", "IkError"}:
            print(f"错误：无解 —— {name}: {exc}", file=sys.stderr)
            return EXIT_MODEL
        raise

    data = {
        "robot": pkg.id,
        "target": target.to_dict(),
        "solution": solution.to_dict(),
    }
    if args.json:
        print(json.dumps(data, indent=2, ensure_ascii=False))
        return 0

    print(f"{pkg.id} · 逆解（IK）")
    print(f"  目标位置: {_fmt_vec(target.position.to_list())}  m")
    print(f"  目标姿态: {_fmt_quat(target.orientation.to_list())}  (x,y,z,w)")
    print()
    print(f"  分支    : {solution.branch}")
    print(f"  关节角  :")
    for name, value in sorted(solution.joint_positions.items()):
        print(f"      {name:<16} = {value:+.6f} rad  ({value * 180.0 / 3.141592653589793:+.3f}°)")
    print(f"  位置误差: {solution.position_error:.3e} m")
    print(f"  姿态误差: {solution.orientation_error:.3e} rad（位置级解析解，姿态由构型决定）")
    if solution.clamped:
        print(f"  ⚠️ 已夹限位: {', '.join(solution.clamped_joints)}")
    return 0


def cmd_inspect(args: argparse.Namespace) -> int:
    """模型自检：Validator 报告 + 几何清单 + 加载报告。"""
    pkg = _load_or_fail(args.robot)
    try:
        model, report = pkg.load_model()
    except LoaderError as exc:
        print(f"错误：{pkg.id} 的模型无法加载 —— {exc}", file=sys.stderr)
        return EXIT_MODEL

    vis = [g for l in model.links for g in l.visual]
    col = [g for l in model.links for g in l.collision]
    # `GeometryRef.asset` 在 v0.1 恒为 None（只有原生 geom，不加载 mesh）。
    # 统计"非 None"的数量，而不是假装有个 assets 表 —— 后者在 v0.1 不存在。
    asset_refs = [g for g in (*vis, *col) if g.asset is not None]
    by_type: dict[str, int] = {}
    for g in (*vis, *col):
        by_type[g.type] = by_type.get(g.type, 0) + 1

    data = {
        "robot": pkg.id,
        "model_path": str(pkg.model_path),
        "validation": {"ok": report.ok, "issues": [i.to_dict() for i in report.issues]},
        "counts": {
            "links": len(model.links),
            "joints": len(model.joints),
            "visual_geoms": len(vis),
            "collision_geoms": len(col),
            "geoms_by_type": by_type,
            "asset_refs": len(asset_refs),
            "inertials": sum(1 for l in model.links if l.inertial is not None),
            "actuators": len(model.actuators),
        },
        "joint_order": model.mobile_joint_ids(),
        "ee": [e.to_dict() for e in model.end_effectors],
    }
    if args.json:
        print(json.dumps(data, indent=2, ensure_ascii=False))
        return 0

    c = data["counts"]
    print(f"{pkg.id} · 模型自检")
    print(f"  文件      : {pkg.model_path}")
    types = ", ".join(f"{k}×{v}" for k, v in sorted(by_type.items())) or "（无）"
    print(f"  几何      : visual {c['visual_geoms']} / collision {c['collision_geoms']}"
          f"  （{types}）")
    print(f"  asset 引用: {c['asset_refs']}   ← v0.1 不加载 mesh，故恒为 0")
    print(f"  惯量      : {c['inertials']}/{c['links']} links 带 inertial")
    print(f"  关节顺序  : {', '.join(data['joint_order'])}   ← 必须与 MuJoCo qpos 一致")
    for e in data["ee"]:
        print(f"  末端      : {e['id']}  frame={e['frame']}  site={e.get('site')}")
    print()
    if report.ok:
        print("  校验      : ✅ 通过（0 issues）")
    else:
        print(f"  校验      : ❌ {len(report.issues)} 个问题")
        for issue in report.issues:
            print(f"      [{issue.severity}] {issue.code}: {issue.message}")
    return 0 if report.ok else EXIT_MODEL


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m backend.cli",
        description="RobotForge CLI —— 机器人包的查看与运动学查询",
    )
    sub = parser.add_subparsers(dest="command", metavar="<command>")

    p_list = sub.add_parser("list", help="列出所有机器人包")
    p_list.add_argument("--json", action="store_true", help="以 JSON 输出")
    p_list.set_defaults(func=cmd_list)

    p_show = sub.add_parser("show", help="显示某台机器人的身份与推导事实")
    p_show.add_argument("robot", metavar="<id>", help="机器人 id（如 mini_arm）")
    p_show.add_argument("--json", action="store_true", help="以 JSON 输出")
    p_show.set_defaults(func=cmd_show)

    p_fk = sub.add_parser("fk", help="正解：关节角 → TCP 位姿")
    p_fk.add_argument("robot", metavar="<id>", help="机器人 id")
    p_fk.add_argument(
        "--joints",
        default="",
        metavar="a=0,b=0.3",
        help="关节角，逗号分隔；未给的关节按 0 处理",
    )
    p_fk.add_argument("--json", action="store_true", help="以 JSON 输出")
    p_fk.add_argument(
        "--force",
        action="store_true",
        help="即使模型未通过校验也继续（默认拒绝）",
    )
    p_fk.set_defaults(func=cmd_fk)

    p_ik = sub.add_parser("ik", help="逆解：TCP 位置 → 关节角")
    p_ik.add_argument("robot", metavar="<id>", help="机器人 id")
    p_ik.add_argument(
        "--target",
        required=True,
        metavar="x,y,z",
        help="目标 TCP 位置（米）；也可给 7 个分量 x,y,z,qx,qy,qz,qw",
    )
    p_ik.add_argument(
        "--branch",
        default="elbow_up",
        choices=["elbow_up", "elbow_down"],
        help="肘部构型（默认 elbow_up）",
    )
    p_ik.add_argument(
        "--clamp",
        action="store_true",
        help="允许把超限解夹到限位（默认拒绝超限解）",
    )
    p_ik.add_argument("--json", action="store_true", help="以 JSON 输出")
    p_ik.set_defaults(func=cmd_ik)

    p_inspect = sub.add_parser("inspect", help="模型自检（校验 + 几何清单）")
    p_inspect.add_argument("robot", metavar="<id>", help="机器人 id")
    p_inspect.add_argument("--json", action="store_true", help="以 JSON 输出")
    p_inspect.set_defaults(func=cmd_inspect)

    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not getattr(args, "func", None):
        # 无子命令时打印帮助并退出 1（而不是崩一个 AttributeError）
        parser.print_help()
        return EXIT_USAGE
    return int(args.func(args))


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
