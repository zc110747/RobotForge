"""`AssetLoader` —— `GeometryRef.asset` 引用 → 可加载几何资源的抽象（提示词 §44 / §65）。

## 它在架构中的位置

```text
    RobotModel
        │
   GeometryRef.asset = "wheel.stl"      ← 只是**一个字符串引用**
        │
   AssetLoader（本模块，抽象）
        │
  ┌─────┼──────┬───────┬────────┐
 STL   OBJ   GLTF   GLB      STEP        ← 全部是 **future**，v0.1 一个都不实现
```

提示词 §65 的原文要求是「**不实现** STL / OBJ / GLTF / STEP Parser」，
「只验证 `RobotModel → GeometryAsset → AssetLoader`」这条缝的存在与形状。

## 为什么 v0.1 要先把抽象写出来（而不是"等要用时再说"）

因为"扩展点是否真的存在"是一个**可以被证伪的架构事实**，而"以后再加"
无法验证。§65 要的正是这个可验证性：今天我们就能写一个合成 loader，
证明"加 STL 支持"不需要动 `RobotModel`、不需要动 `RobotRuntime`。

这与 §43 的 `RobotModelLoader` 是同一种做法 —— 本项目已两次证明
"抽象 + 注册表"能让扩展变成"加一个模块"，而不是"改 Core 的 if"。

## 与 `RobotModelLoader` 的关键区别（不要合并它们）

| | `RobotModelLoader` | `AssetLoader` |
|---|---|---|
| 输入 | 机器人**描述文件** | 单个**几何资源** |
| 输出 | `RobotModel`（整个模型） | 网格/几何数据（一个 link 的一部分） |
| 时机 | 启动/加载期，有且仅有一次 | **按需**，可能在前端渲染时逐个拉取 |
| 缺失后果 | 整个机器人不可用 | 该 link 退回占位几何，其余照常 |

最后一条是最重要的：**asset 缺失不该让机器人起不来**。
这正是 §44 把几何只存"引用"的目的 —— 让大 mesh 与模型定义解耦。

## v0.1 的行为

`GeometryRef.asset` 恒为 `None`（原生 geom 不需要外部资源）。
所以 `GeometryAssetRegistry` 在 v0.1 里是**空注册表也完全正常**：
`resolve(None)` 返回 `None`，而 `resolve("foo.stl")` 抛
`UnsupportedAssetError`（带"v0.1 不实现任何 parser"的说明）。
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path
from typing import Any

__all__ = [
    "AssetError",
    "AssetLoadError",
    "AssetLoader",
    "AssetLoaderRegistrationError",
    "GeometryAsset",
    "GeometryAssetRegistry",
    "UnsupportedAssetError",
    "default_asset_registry",
    "get_asset_registry",
    "register_asset_loader",
]


class AssetError(RuntimeError):
    """几何资源相关的错误基类。"""


class UnsupportedAssetError(AssetError):
    """没有 loader 能处理这个资源（含 v0.1 "一个 parser 都没实现"的情况）。

    与 `AssetLoadError` 分开：前者是"我不知道怎么读"，后者是"我读的时候坏了"。
    对用户来说前者意味着"升个版本 / 换个格式"，后者意味着"文件很可能损坏"。
    """


class AssetLoadError(AssetError):
    """有 loader 认领了这个资源，但读取/解析失败。"""


class AssetLoaderRegistrationError(RuntimeError):
    """注册表使用错误（重复注册 / 非法扩展名列表 / 非法 loader）。"""


@dataclass(frozen=True)
class GeometryAsset:
    """一个已加载的几何资源。

    ★ 刻意**不**规定 `data` 的类型。

    `RobotModel` 必须能便宜地序列化（§24 的注释解释过：5 万顶点的 STL
    会让每条 `robot_info` 变成几 MB）。所以本类型只承载"loader 产出了什么"
    以及"它的形态是什么"，好让**调用方**按 `kind` 决定怎么用：

    - `kind="mesh"` → `data` 是顶点/索引（给前端上传 GPU）
    - `kind="uri"`  → `data` 是一个可下载的 URL/路径（大文件走网络，不进内存）
    - `kind="native"` → `data` 是参数化描述（box/sphere/cylinder 的尺寸）

    把 `data` 定死成 `bytes` 会强迫"URL 引用"也塞进内存，
    而定死成 `str` 会让"已解码的网格"无处安放。留成 `Any` 是对的，
    代价是调用方要判 `kind` —— 这个判据本来就是必要的。
    """

    #: 资源标识（通常等于 `GeometryRef.asset`）
    name: str
    #: 资源的**形态**：`mesh` / `uri` / `native`
    kind: str
    #: 产生它的 loader 的扩展名（`stl` / `obj` / ...）
    format: str
    #: 载荷（语义由 `kind` 决定，见类文档）
    data: Any = None
    #: 可选元信息（三角面数、包围盒、来源 URL 等）
    metadata: dict[str, Any] | None = None

    def to_dict(self) -> dict[str, Any]:
        """轻量描述 —— **不含** `data`。

        与 `GeometryRef` 同样的理由：这个 dict 可能被塞进 WS 帧。
        需要 `data` 的调用方拿对象本身，不需要的拿 dict。
        """
        return {
            "name": self.name,
            "kind": self.kind,
            "format": self.format,
            "metadata": dict(self.metadata) if self.metadata else None,
        }


class AssetLoader(ABC):
    """几何资源 loader 的抽象基类。

    只需实现 `extensions` + `load`。与 `RobotModelLoader` 一样保持极窄 ——
    抽象层不该知道任何具体格式（本模块里不出现 `stl`/`obj`/`gltf` 字面量）。
    """

    #: 认领的扩展名（小写、不含点），如 `("stl", "STL")` 应写成 `("stl",)`
    extensions: tuple[str, ...] = ()

    @abstractmethod
    def load(self, source: Any, *, name: str = "") -> GeometryAsset:
        """加载资源。失败抛 `AssetLoadError`（**不返回 None**）。"""
        raise NotImplementedError

    def can_load(self, source: Any) -> bool:
        """是否认得这个 source（默认按扩展名）。"""
        if isinstance(source, Path):
            source = str(source)
        if not isinstance(source, str):
            return False
        lowered = source.lower()
        return any(lowered.endswith(f".{e}") for e in self.extensions)

    def __repr__(self) -> str:  # pragma: no cover - 调试便利
        return f"{type(self).__name__}(ext={list(self.extensions)})"


class GeometryAssetRegistry:
    """扩展名 → `AssetLoader` 的映射（与 `LoaderRegistry` 同构，但按扩展名索引）。"""

    def __init__(self) -> None:
        self._by_ext: dict[str, AssetLoader] = {}

    def register(self, loader: AssetLoader) -> AssetLoader:
        exts = getattr(loader, "extensions", ())
        if not isinstance(exts, (tuple, list)) or not exts:
            raise AssetLoaderRegistrationError(
                f"{type(loader).__name__}.extensions 必须是非空元组，实际是 {exts!r}。"
                f"扩展名是分派的键，不能为空。"
            )
        normalized: list[str] = []
        for e in exts:
            if not isinstance(e, str) or not e:
                raise AssetLoaderRegistrationError(
                    f"{type(loader).__name__}.extensions 含非字符串项：{e!r}"
                )
            if e != e.lower():
                raise AssetLoaderRegistrationError(
                    f"{type(loader).__name__}.extensions 含大写 {e!r} —— "
                    f"必须全小写，否则与 `GeometryRef.asset` 的小写比对不一致，"
                    f"表现为'明明注册了却说不支持'。"
                )
            if e.startswith("."):
                raise AssetLoaderRegistrationError(
                    f"{type(loader).__name__}.extensions 含带点的 {e!r} —— "
                    f"扩展名不含前导点（注册 'stl'，不是 '.stl'）。"
                )
            normalized.append(e)

        # 先全量校验再写入：半写入会让注册表处于"一半新一半旧"的状态，
        # 而调用方看到的是"某些格式能用某些不能用"，极难定位。
        for e in normalized:
            if e in self._by_ext:
                existing = type(self._by_ext[e]).__name__
                raise AssetLoaderRegistrationError(
                    f"扩展名 {e!r} 已被 {existing} 注册，"
                    f"{type(loader).__name__} 不能重复占用。"
                    f"当前已注册：{self.extensions()}"
                )
        for e in normalized:
            self._by_ext[e] = loader
        return loader

    def unregister(self, extension: str) -> None:
        self._by_ext.pop(str(extension).lower().lstrip("."), None)

    # -- 查询 ---------------------------------------------------------------

    def get(self, extension: str) -> AssetLoader:
        """按扩展名取 loader；没有则抛 `UnsupportedAssetError`。

        `extension` 收 `.stl` 与 `stl` 两种写法（内部统一去点、转小写）——
        调用方常直接传 `Path(...).suffix`（带点），而注册时用的是不带点的形式。
        强制其中一种会让另一处必须记得转换，那是 bug 温床。
        """
        key = str(extension).lower().lstrip(".")
        try:
            return self._by_ext[key]
        except KeyError:
            known = self.extensions()
            hint = (
                str(known)
                if known
                else "（空 —— v0.1 不实现任何 mesh parser，这是正常的）"
            )
            raise UnsupportedAssetError(
                f"没有注册处理扩展名 {key!r} 的 AssetLoader。"
                f"当前已注册：{hint}。"
            ) from None

    def resolve(self, asset: str | None, *, name: str = "") -> GeometryAsset | None:
        """把 `GeometryRef.asset` 解析成 `GeometryAsset`。

        ★ `asset is None` 或空串 ⇒ 返回 `None`，**不抛异常**。

        这不是"宽容",而是 v0.1 的**正常路径**：原生 geom（box/cylinder/…）
        本来就没有外部资源。让 `None` 抛异常会逼每个调用方写 try/except，
        而真正该报错的是"有引用但没人能加载"——那是下面那条路径。
        """
        if asset is None or not str(asset).strip():
            return None
        ref = str(asset)
        # ⚠️ 不能用 `Path(ref).suffix` 一种写法：`Path(".stl").suffix` 是 **空串**
        # （pathlib 把前导点当"隐藏文件名"而不是扩展名分隔符）。
        # 所以先取 suffix，为空时退回"按最后一个点手动切"，
        # 再退回"整个字符串就是一个扩展名"。
        ext = Path(ref).suffix
        if not ext and "." in ref:
            ext = "." + ref.rsplit(".", 1)[1]
        if not ext and not ref.startswith("."):
            ext = "." + ref  # 调用方直接给了扩展名
        loader = self.get(ext)
        return loader.load(ref, name=name or ref)

    def __contains__(self, extension: object) -> bool:
        return str(extension).lower().lstrip(".") in self._by_ext

    def __len__(self) -> int:
        return len(self._by_ext)

    def extensions(self) -> list[str]:
        """已注册的扩展名（排序，便于稳定断言）。"""
        return sorted(self._by_ext)

    def __repr__(self) -> str:  # pragma: no cover - 调试便利
        return f"GeometryAssetRegistry({self.extensions()})"


# ---------------------------------------------------------------------------
# 进程级默认注册表
# ---------------------------------------------------------------------------

_DEFAULT: GeometryAssetRegistry | None = None


def get_asset_registry() -> GeometryAssetRegistry:
    """取进程级默认注册表。

    ## v0.1 它是**空的**，且这是**正确**状态

    提示词 §65 明确"不实现 STL/OBJ/GLTF/STEP Parser"。
    所以这个注册表在 v0.1 里 `len() == 0` 是完全预期的 ——
    验收脚本必须接受"空"，但**不能**接受"空得没道理"
    （即：必须证明它**能**接受注册，否则"空"就退化成"这个类是个壳"）。
    """
    global _DEFAULT
    if _DEFAULT is None:
        _DEFAULT = GeometryAssetRegistry()
    return _DEFAULT


def default_asset_registry() -> GeometryAssetRegistry:
    """构造一个空的注册表（测试用，隔离全局状态）。"""
    return GeometryAssetRegistry()


def register_asset_loader(loader: AssetLoader) -> AssetLoader:
    """把 loader 注册进进程级默认注册表（便捷函数）。"""
    return get_asset_registry().register(loader)
