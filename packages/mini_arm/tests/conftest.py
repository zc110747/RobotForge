"""mini_arm 包内测试的共享 fixture。

## 为什么 `packages/` 里也要有自己的 conftest

`packages/` **刻意不是** Python package（没有 `__init__.py`）。
这是契约：Robot Package 是与语言无关的**数据包**，不应该因为
"Python 想 import 它"而获得一个 Python 包的形态。

代价：包内的模块（`kinematics/fk.py`、`kinematics/ik.py`）不能用
相对/包内 import 互相引用，测试也不能 `import packages.mini_arm.kinematics.fk`。
⇒ 用 `importlib.util.spec_from_file_location` 按**路径**加载，
   并注册进 `sys.modules`（Python 3.13 的 `dataclass` 需要反向查模块）。

这套加载器被写在这里而不是每个测试文件里重复一遍 —— 因为"注册 sys.modules"
这一步很容易漏，而漏掉的后果是 `@dataclass` 报一个与 dataclass 毫无关系的错。
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

PKG_DIR = Path(__file__).resolve().parent.parent
KINEMATICS_DIR = PKG_DIR / "kinematics"
MODEL_PATH = PKG_DIR / "model" / "mini_arm.xml"

#: 加载过的模块缓存（同一进程内不重复执行模块顶层代码）
_LOADED: dict[str, object] = {}


def _load_by_path(name: str, path: Path):
    """按文件路径加载一个模块，并注册进 sys.modules。

    ⚠ `sys.modules[name] = mod` **不能省**：
      Python 3.13 的 `@dataclass` 在处理 `from __future__ import annotations`
      时会通过 `sys.modules[cls.__module__]` 反查模块来解析字符串注解。
      未注册时抛 `AttributeError: 'NoneType' object has no attribute ...`，
      报错信息与被导入的代码完全无关，极难定位。
    """
    if name in _LOADED:
        return _LOADED[name]
    if not path.is_file():
        raise FileNotFoundError(f"包内模块不存在：{path}")
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"无法为 {path} 建立 import spec")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    _LOADED[name] = mod
    return mod


def mini_arm_fk():
    """mini_arm 的解析 FK 模块。"""
    return _load_by_path("mini_arm_pkg_fk", KINEMATICS_DIR / "fk.py")


def mini_arm_ik():
    """mini_arm 的 IK 模块（它会自己去加载 sibling fk）。"""
    return _load_by_path("mini_arm_pkg_ik", KINEMATICS_DIR / "ik.py")


@pytest.fixture(scope="session")
def mini_arm_model():
    """mini_arm 的 RobotModel（由 MJCF 经 Loader 得到）。

    用 session 作用域：加载一次即可，且可以尽早暴露"Loader 坏了"这件事 ——
    它会让包内**所有**测试一起失败，而不是零散地失败几个。
    """
    from backend.loaders.mjcf_loader import MJCFLoader

    if not MODEL_PATH.is_file():
        # 不 skip：模型文件不存在是真实的结构性破坏，不是"环境不满足"
        pytest.fail(f"mini_arm 模型不存在：{MODEL_PATH}")

    loader = MJCFLoader(robot_id="mini_arm")
    model, _report = loader.load(MODEL_PATH)
    return model


@pytest.fixture(scope="session")
def fk():
    """mini_arm 的解析 FK 模块（经 fixture 暴露，避免相对 import）。"""
    return mini_arm_fk()


@pytest.fixture(scope="session")
def ik():
    """mini_arm 的 IK 模块（经 fixture 暴露，避免相对 import）。"""
    return mini_arm_ik()


# ----------------------------------------------------------------------
# 复用顶层 tests/conftest.py 的 fixture
# ----------------------------------------------------------------------
#
# ## 为什么这里要显式转一层，而不是直接靠 pytest 的 conftest 发现机制
#
# pytest 只会自动加载**从 rootdir 到测试文件所在目录这条路径上**的
# conftest.py。`packages/mini_arm/tests/` 不在 `tests/` 下面，
# 所以 `tests/conftest.py` 里的 fixture（`mini_arm_mjcf` 等）**不会**
# 自动对本目录可见 —— 表现是 `fixture 'mini_arm_mjcf' not found`。
#
# 两种修法：
#   ① 把 `packages/mini_arm/tests/conftest.py` 写成 `pytest_plugins` 引用
#      —— 但 `pytest_plugins` 在非 rootdir 的 conftest 里已弃用并会报错。
#   ② 显式再声明一层同名 fixture 转发（本文件的做法）。
#
# 选 ② 的原因：它让"包内测试依赖了顶层 fixture"这件事**在文件里可见**，
# 而不是靠隐式的 conftest 搜索。同时包内测试仍然可以独立运行
# （`pytest packages/mini_arm/tests` 不需要 `tests/` 存在）。

from pathlib import Path  # noqa: E402  （已在文件顶部导入，此处仅作可读性锚点）


@pytest.fixture(scope="session")
def mini_arm_mjcf() -> Path:
    """mini_arm 的 MJCF 路径（转发自顶层 conftest 的同名 fixture）。"""
    if not MODEL_PATH.is_file():
        pytest.fail(f"mini_arm 模型不存在：{MODEL_PATH}")
    return MODEL_PATH
