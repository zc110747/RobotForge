"""模型加载层：把各种格式的机器人描述转成 `RobotModel`（提示词 §8 / §43 / §64）。

## 分层

```text
loader.py           RobotModelLoader（抽象）+ LoaderReport + LoaderError
registry.py         LoaderRegistry：格式名 → Loader 实例（**分派点**）
mjcf_loader.py      MJCFLoader   ← v0.1 唯一的具体实现
（未来）urdf_loader.py  URDFLoader ← 只需 register_loader(URDFLoader())
```

## 依赖纪律

- `loader.py`（抽象）**不** import 任何具体格式 —— 抽象层不该知道 MJCF 存在。
- `registry.py` 只在 `_register_builtin` 里**局部** import `MJCFLoader`，
  既能装配又不会让"只想用注册表抽象"的调用方被迫 import mujoco。
- `RobotRuntime` / `api/registry.py` 只认 `RobotModel` 与格式名，
  **不** import 具体 Loader（Phase 7 的验收对象）。
"""

from .asset_loader import (
    AssetError,
    AssetLoadError,
    AssetLoader,
    AssetLoaderRegistrationError,
    GeometryAsset,
    GeometryAssetRegistry,
    UnsupportedAssetError,
    default_asset_registry,
    get_asset_registry,
    register_asset_loader,
)
from .loader import LoaderError, LoaderReport, RobotModelLoader
from .registry import (
    LoaderRegistrationError,
    LoaderRegistry,
    UnknownFormatError,
    default_loader_registry,
    get_loader_registry,
    register_loader,
)

__all__ = [
    "AssetError",
    "AssetLoadError",
    "AssetLoader",
    "AssetLoaderRegistrationError",
    "GeometryAsset",
    "GeometryAssetRegistry",
    "LoaderError",
    "LoaderRegistrationError",
    "LoaderRegistry",
    "LoaderReport",
    "RobotModelLoader",
    "UnknownFormatError",
    "UnsupportedAssetError",
    "default_asset_registry",
    "default_loader_registry",
    "get_asset_registry",
    "get_loader_registry",
    "register_asset_loader",
    "register_loader",
]
