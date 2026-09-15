"""Loader 注册表 —— `model.format` → `RobotModelLoader` 的**分派点**（提示词 §43 / §64）。

## 为什么需要这个模块

提示词 §43 画的是这样一张图：

```text
        RobotModelLoader
              │
       ┌──────┴──────┐
    MJCF           URDF
    Loader         Loader
       │             │
       └──────┬──────┘
              ▼
          RobotModel
```

`RobotModelLoader` 抽象（`loader.py`）早就写好了，但**没有人用它来做分派** ——
`api/registry.py` 里直接写 `MJCFLoader()`，并且用
`if fmt != "mjcf": raise` 把别的格式**显式拒掉**。

那意味着"加一个 URDF Loader"的代价是**改 Core 的分派代码**，
而不是"加一个模块"。§69 规则 2 的精神（Core 不得为具体型号/格式长出分支）
在这里被违反了 —— 只不过长的是"格式分支"而不是"型号分支"。

本模块把那个 `if` 换成一张**注册表**：

```text
manifest.model.format ──► LoaderRegistry ──► RobotModelLoader
                               │
                       register("urdf", URDFLoader())
```

## 三条设计约束（都为了"将来加格式不用改这里"）

### 1. 本模块**不认识任何具体格式**

文件里不出现 `"mjcf"` / `"urdf"` 字面量。具体格式的注册发生在
**各 Loader 模块自己**（`mjcf_loader.py` 末尾调用 `register_loader`），
或用 `default_loader_registry()` 集中装配。
这样"删掉 MJCF 支持"= 删掉那个文件，不是来这个文件里挖 if。

### 2. 注册**不覆盖**已存在的格式名

同一个格式名注册两次 ⇒ 抛 `LoaderRegistrationError`。
理由同 `registry.py` 的"id 重复要报错"：静默后者覆盖前者会让
"我注册没生效"表现成"加载出来的模型长得不对"，排查方向完全错。

### 3. 空注册表是**合法**状态，但查询会给出可操作错误

`v0.1` 之外的场景（比如只想做几何校验、不需要加载模型）不该被迫注册 Loader。
但 `get("urdf")` 找不到时必须说清"当前注册了哪些"，
否则用户看到的是 `KeyError: 'urdf'` —— 一个不含任何修复线索的错误。
"""

from __future__ import annotations

from .loader import LoaderError, RobotModelLoader

__all__ = [
    "LoaderRegistrationError",
    "LoaderRegistry",
    "UnknownFormatError",
    "default_loader_registry",
    "get_loader_registry",
    "register_loader",
]


class LoaderRegistrationError(RuntimeError):
    """注册表使用错误（重复注册 / 注册了带非法 format_name 的 loader）。

    刻意**不是** `LoaderError`：后者表示"模型内容坏了"，
    本异常表示"装配代码写错了"。两者的修复动作与责任人都不同。
    """


class UnknownFormatError(LoaderError):
    """`model.format` 在注册表里找不到对应的 loader。

    继承 `LoaderError` 是刻意的：对**调用方**（API 层）来说，
    "这个格式我不支持"和"这个格式的文件解析失败"是同一类事 ——
    都是 500 / 带 issues 返回，而不是 404。但它是**独立类型**，
    以便测试能精确断言"是因为没有 loader，而不是解析失败"。
    """


class LoaderRegistry:
    """格式名 → `RobotModelLoader` 实例的映射。

    实例而非类：Loader 可以携带配置（路径前缀、严格模式开关等），
    注册一个**已配置好的**实例比注册类再要求调用方自己 new 更省事。
    """

    def __init__(self) -> None:
        self._loaders: dict[str, RobotModelLoader] = {}

    # -- 注册 ---------------------------------------------------------------

    def register(self, loader: RobotModelLoader) -> RobotModelLoader:
        """注册一个 loader，返回它本身（便于当作装饰器用）。

        格式名从 `loader.format_name` 读，**不从调用方传参** ——
        传参会让"注册名"与"loader 自称的名字"存在不一致的可能，
        而分派时用的是注册名。单一真值源原则。
        """
        name = getattr(loader, "format_name", "")
        if not isinstance(name, str) or not name:
            raise LoaderRegistrationError(
                f"{type(loader).__name__} 的 format_name 必须是非空字符串，"
                f"实际是 {name!r}。格式名是分派的键，不能为空。"
            )
        if name != name.lower():
            raise LoaderRegistrationError(
                f"{type(loader).__name__}.format_name = {name!r} 必须全小写 —— "
                f"manifest 里的 model.format 按小写比对，大小写不一致会表现为"
                f"'明明注册了却找不到'。"
            )
        if name in self._loaders:
            existing = type(self._loaders[name]).__name__
            raise LoaderRegistrationError(
                f"格式 {name!r} 已被 {existing} 注册，"
                f"{type(loader).__name__} 不能重复占用。"
                f"当前已注册：{sorted(self._loaders)}"
            )
        self._loaders[name] = loader
        return loader

    def unregister(self, format_name: str) -> None:
        """移除一个格式（测试用；也让"临时关掉某格式"成为可能）。"""
        self._loaders.pop(format_name, None)

    # -- 查询 ---------------------------------------------------------------

    def get(self, format_name: str) -> RobotModelLoader:
        """按格式名取 loader；没有则抛 `UnknownFormatError`（带可操作信息）。"""
        key = str(format_name).lower()
        try:
            return self._loaders[key]
        except KeyError:
            known = sorted(self._loaders)
            raise UnknownFormatError(
                f"没有注册处理格式 {format_name!r} 的 Loader。"
                f"当前已注册：{known or '（空）'}。"
                f"新增格式 = 写一个 RobotModelLoader 子类（设好 format_name）"
                f"并在装配处 register_loader(它)。"
            ) from None

    def __contains__(self, format_name: object) -> bool:
        return str(format_name).lower() in self._loaders

    def __len__(self) -> int:
        return len(self._loaders)

    def formats(self) -> list[str]:
        """已注册的格式名（排序，便于稳定断言与展示）。"""
        return sorted(self._loaders)

    def __repr__(self) -> str:  # pragma: no cover - 调试便利
        return f"LoaderRegistry({self.formats()})"


# ---------------------------------------------------------------------------
# 进程级默认注册表
# ---------------------------------------------------------------------------

_DEFAULT: LoaderRegistry | None = None


def get_loader_registry() -> LoaderRegistry:
    """取进程级默认注册表（首次调用时装配内置格式）。

    ## 为什么是"懒装配"而不是模块级常量

    模块级 `_DEFAULT = LoaderRegistry(); _DEFAULT.register(MJCFLoader())`
    会让 **import 本模块** 就付出 import mujoco 的代价，
    而 §69 的架构要求是"没有 mujoco 也要能起来"（前端预览 / 纯几何校验）。
    懒装配把这个代价推迟到"真的要加载模型"时。
    """
    global _DEFAULT
    if _DEFAULT is None:
        _DEFAULT = LoaderRegistry()
        _register_builtin(_DEFAULT)
    return _DEFAULT


def default_loader_registry() -> LoaderRegistry:
    """构造一个**新的**、已装配内置格式的注册表（测试用，隔离全局状态）。"""
    reg = LoaderRegistry()
    _register_builtin(reg)
    return reg


def _register_builtin(reg: LoaderRegistry) -> None:
    """装配内置格式。

    这是**唯一**允许出现 `"mjcf"` 的地方 —— 且它是通过
    `MJCFLoader.format_name` 间接得到的，不硬编码字符串。
    """
    # 局部 import：避免 loaders 包 import 本模块时形成环，
    # 也避免"只想用注册表抽象"的调用方被迫 import mujoco。
    from .mjcf_loader import MJCFLoader

    reg.register(MJCFLoader())


def register_loader(loader: RobotModelLoader) -> RobotModelLoader:
    """把 loader 注册进进程级默认注册表（便捷函数）。"""
    return get_loader_registry().register(loader)
