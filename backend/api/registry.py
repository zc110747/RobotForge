"""机器人包注册表 —— 从文件系统发现可用机器人。

## 为什么必须"发现"而不是"登记"

如果这里写一张 `{"mini_arm": "packages/mini_arm"}` 的表，
那么"加一台机器人"就变成了**改 Core 代码**（提示词 §69 规则 2 的变体）。
发现式注册让新机器人 = 新目录 + manifest.yaml，Core 一行不动。

## 唯一的真值源

`packages/<id>/manifest.yaml` 是**包身份**的唯一真值源：
id / name / version / capabilities / model.file 全从它读。
本模块**不**从目录名推断 id —— 目录名与 `metadata.id` 不一致时，
以 manifest 为准并报 `RobotPackageError`，因为静默取其一就是第二份真值。
"""

from __future__ import annotations

import dataclasses
from pathlib import Path
from typing import Any, Iterator

import yaml

from ..loaders.loader import LoaderError
from ..loaders.registry import (
    UnknownFormatError,
    get_loader_registry,
    default_loader_registry,
)
from ..model.robot_model import RobotModel
from ..model.validator import ValidationReport, validate_robot_model


def _loader_registry():
    """当前用于分派的 Loader 注册表。

    抽成函数（而非模块级常量）有两个原因：
    ① 测试可以替换它来注入一个**合成的第二个格式**，从而证明分派真的可扩展
       （而不是"只有 MJCF 能过"这种结构性通过的假象）；
    ② §69"没有 mujoco 也要能起来"—— 注册表是懒装配的，
       模块级常量会在 import 本模块时就 import mujoco。
    """
    return _REGISTRY[0]


#: 可替换的分派表（单元素列表 = 可变盒，测试可原地替换）
_REGISTRY = [get_loader_registry()]


def use_loader_registry(registry) -> None:
    """替换分派表（测试/嵌入方用）。传 `None` 恢复默认。"""
    _REGISTRY[0] = registry if registry is not None else get_loader_registry()


#: 仓库根 = <root>/backend/api/registry.py 往上两级
ROOT = Path(__file__).resolve().parent.parent.parent

#: 机器人包所在目录（提示词 §2：`packages/<robot>`）
PACKAGES_DIR = ROOT / "packages"


class RobotPackageError(Exception):
    """机器人包**结构**层面的问题（manifest 缺失/字段非法/目录与 id 不符）。

    刻意与 `LoaderError`（模型**内容**层面的问题）分开：
    两者修复动作不同 —— 前者改 manifest/目录，后者改 MJCF。
    混在一起会让调用方无法区分"包没装好"和"模型写得不对"。
    """


@dataclasses.dataclass(frozen=True)
class RobotPackage:
    """一个已发现但**尚未加载模型**的机器人包。

    `load_model()` 是显式的 —— 发现阶段不编译 MJCF。
    理由：编译 MuJoCo 模型约几十毫秒，列表页不需要付这个代价；
    更重要的是**发现不该因为某个包的模型坏了而整体失败**。
    """

    id: str
    name: str
    version: str
    description: str
    directory: Path
    manifest_path: Path
    model_path: Path
    #: manifest 声明的模型格式（Loader 分派的键，提示词 §43）
    model_format: str
    capabilities: dict[str, bool]
    raw_manifest: dict[str, Any]

    def load_model(self) -> tuple[RobotModel, ValidationReport]:
        """加载 → 校验，返回 `(model, report)`。

        ★ ★ 分派点：按 `model_format` 从注册表取 Loader，**不**直接 new `MJCFLoader`。
        这是 Phase 7 的核心 —— 本方法里没有任何格式名，
        加 URDF 支持不需要改这一行。

        ★ 校验**不**在这里 raise：`report.ok is False` 时仍返回 model，
        让调用方（API 层）决定是 500 还是带着 issues 返回 200。
        在 v0.1 的 API 里是**后者** —— 前端需要看到问题列表才能显示诊断。
        """
        loader = _loader_registry().get(self.model_format)
        model = loader.load(self.model_path)[0]
        report = validate_robot_model(model)
        return model, report

    def to_dict(self) -> dict[str, Any]:
        """**不含**模型几何的轻量描述（列表页用）。

        ⚠️ 这里**不**放 dof / link_count —— 它们必须从 model 推导。
        在列表里塞一份推导值 = 第二份真值（manifest 里那段注释警告过同一件事）。
        """
        return {
            "id": self.id,
            "name": self.name,
            "version": self.version,
            "description": self.description,
            "capabilities": dict(self.capabilities),
        }


# ---------------------------------------------------------------------------
# 发现
# ---------------------------------------------------------------------------

_REQUIRED_FIELDS = ("id", "name", "model", "capabilities")


def _read_manifest(path: Path) -> dict[str, Any]:
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:  # pragma: no cover - 文件系统异常
        raise RobotPackageError(f"无法读取 {path}：{exc}") from exc
    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise RobotPackageError(f"{path} 不是合法 YAML：{exc}") from exc
    if not isinstance(data, dict):
        raise RobotPackageError(f"{path} 的顶层必须是映射，实际是 {type(data).__name__}")
    return data


def _validate_manifest(data: dict[str, Any], manifest_path: Path) -> None:
    missing = [f for f in _REQUIRED_FIELDS if f not in data]
    if missing:
        raise RobotPackageError(
            f"{manifest_path} 缺少必需字段：{missing}（必需字段 = {list(_REQUIRED_FIELDS)}）"
        )
    model = data["model"]
    if not isinstance(model, dict) or "file" not in model:
        raise RobotPackageError(
            f"{manifest_path} 的 `model` 必须是含 `file` 的映射，实际是 {model!r}"
        )
    fmt = model.get("format")
    if not isinstance(fmt, str) or not fmt:
        raise RobotPackageError(
            f"{manifest_path} 的 `model.format` 必须是非空字符串，实际是 {fmt!r}。"
            f"它是 Loader 分派的键（提示词 §43）。"
        )
    # ★ Phase 7：**不再**在这里硬编码 "mjcf"。
    # 判据从"是不是 mjcf"改成"注册表里有没有处理这个格式的 Loader" ——
    # 于是"加一个 URDF Loader"= 注册一个 loader，本文件一行不改。
    # 这里只查**已注册**，不查文件存在性：格式支持是装配期事实，
    # 与某个包的具体路径无关（路径检查在 _discover 里做）。
    if fmt.lower() not in _loader_registry():
        raise RobotPackageError(
            f"{manifest_path} 的 model.format = {fmt!r}，"
            f"但没有任何已注册的 Loader 能处理它。"
            f"当前已注册：{_loader_registry().formats() or '（空）'}。"
            f"新增格式 = 写一个 RobotModelLoader 子类并注册（提示词 §43）。"
        )
    caps = data["capabilities"]
    if not isinstance(caps, dict) or not caps:
        raise RobotPackageError(f"{manifest_path} 的 `capabilities` 必须是非空映射")


def discover_packages(packages_dir: Path | None = None) -> list[RobotPackage]:
    """扫描 `packages/*/manifest.yaml`，返回**按 id 排序**的包列表。

    排序是刻意的：文件系统返回顺序不稳定，而不排序会让
    `GET /api/robots` 的响应顺序在不同机器上不同 ⇒ 前端测试无法稳定断言。

    单个包损坏 ⇒ 抛 `RobotPackageError`。**不**跳过：
    静默跳过坏包会让"机器人消失了"表现成"它从来不存在"，
    而这两种情况需要完全不同的处理。
    """
    root = Path(packages_dir) if packages_dir is not None else PACKAGES_DIR
    if not root.is_dir():
        return []

    found: list[RobotPackage] = []
    seen: dict[str, Path] = {}

    # 只扫一层：packages/<id>/manifest.yaml（不做递归，避免把第三方
    # 子模块里的 manifest 也当成机器人包）
    for manifest_path in sorted(root.glob("*/manifest.yaml")):
        data = _read_manifest(manifest_path)
        _validate_manifest(data, manifest_path)

        pkg_id = str(data["id"])
        directory = manifest_path.parent

        # 目录名 vs manifest.id 不一致 ⇒ 报错，不静默取其一
        if directory.name != pkg_id:
            raise RobotPackageError(
                f"{manifest_path} 声明 id = {pkg_id!r}，但所在目录名为 "
                f"{directory.name!r}。二者必须一致 —— 否则 `packages/<id>/` 这个"
                f"路径契约（提示词 §2）就失效了。"
            )
        if pkg_id in seen:
            raise RobotPackageError(
                f"机器人 id {pkg_id!r} 重复：{seen[pkg_id]} 与 {manifest_path}"
            )
        seen[pkg_id] = manifest_path

        model_path = directory / str(data["model"]["file"])
        if not model_path.is_file():
            raise RobotPackageError(
                f"{manifest_path} 指向的模型文件不存在：{model_path}"
            )

        # 格式已由 `_validate_manifest` 校验过（非空 + 注册表认得），
        # 这里只取归一化后的值存进包对象 —— 分派键必须与注册表键同形。
        model_format = str(data["model"]["format"]).lower()

        found.append(
            RobotPackage(
                id=pkg_id,
                name=str(data["name"]),
                version=str(data.get("version", "0.0.0")),
                description=str(data.get("description", "")),
                directory=directory,
                manifest_path=manifest_path,
                model_path=model_path,
                model_format=model_format,
                capabilities={str(k): bool(v) for k, v in data["capabilities"].items()},
                raw_manifest=data,
            )
        )

    found.sort(key=lambda p: p.id)
    return found


def get_package(robot_id: str, packages_dir: Path | None = None) -> RobotPackage:
    """按 id 取单个包；不存在抛 `KeyError`（供 API 层转 404）。"""
    for pkg in discover_packages(packages_dir):
        if pkg.id == robot_id:
            return pkg
    available = [p.id for p in discover_packages(packages_dir)]
    raise KeyError(f"没有机器人 {robot_id!r}；可用：{available}")


def iter_packages(packages_dir: Path | None = None) -> Iterator[RobotPackage]:
    yield from discover_packages(packages_dir)


__all__ = [
    "RobotPackage",
    "RobotPackageError",
    "PACKAGES_DIR",
    "discover_packages",
    "get_package",
    "iter_packages",
]
