"""RobotForge 测试的公共夹具。

## 设计原则：**真值只有一份**

测试里的期望值（关节顺序、连杆长度、限位）如果与生产代码各写一份，
那么"改了一处忘了另一处"就会让测试**通过**而系统是错的。
因此本文件提供的夹具全部从**唯一真值源**推导：

```text
packages/mini_arm/model/mini_arm.xml   ← MJCF（几何/关节/限位的真值）
packages/mini_arm/manifest.yaml        ← 包身份 / 能力 / 指针
```

测试里**允许**出现的硬编码期望值只有：**"这份真值应该长什么样"** ——
即断言 MJCF 内容的那些数（如 L1 = 0.103）。那是**验证真值本身**，
与生产代码不构成第二份真值。
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

#: 仓库根（本文件在 <root>/tests/ 下）
ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

PACKAGES_DIR = ROOT / "packages"
MINI_ARM_DIR = PACKAGES_DIR / "mini_arm"
MINI_ARM_MJCF = MINI_ARM_DIR / "model" / "mini_arm.xml"
MINI_ARM_MANIFEST = MINI_ARM_DIR / "manifest.yaml"


@pytest.fixture(scope="session")
def repo_root() -> Path:
    return ROOT


@pytest.fixture(scope="session")
def mini_arm_mjcf() -> Path:
    """mini_arm 的 MJCF 路径（不存在 ⇒ 直接失败，不跳过）。

    刻意**不**用 `pytest.skip`：文件缺失是**真实的架构破损**
    （包被搬走或改名），跳过会让它表现成"这台机器人没测试"。
    """
    assert MINI_ARM_MJCF.is_file(), f"mini_arm 的 MJCF 缺失：{MINI_ARM_MJCF}"
    return MINI_ARM_MJCF


@pytest.fixture(scope="session")
def mini_arm_manifest() -> Path:
    assert MINI_ARM_MANIFEST.is_file(), f"mini_arm 的 manifest 缺失：{MINI_ARM_MANIFEST}"
    return MINI_ARM_MANIFEST


@pytest.fixture(scope="session")
def mini_arm_model(mini_arm_mjcf: Path):
    """加载好的 `RobotModel`（session 级，避免每个测试都编译一次 MJCF）。

    注意：这里用**裸 Loader**（不带 manifest 的 metadata 覆盖），
    因为测试关注的是"MJCF → RobotModel"这条转换。
    manifest → metadata 的注入由 `test_runtime.py` 覆盖。
    """
    from backend.loaders.mjcf_loader import MJCFLoader

    model, report = MJCFLoader(robot_id="mini_arm").load(mini_arm_mjcf)
    return model


@pytest.fixture(scope="session")
def mini_arm_loader_report(mini_arm_mjcf: Path):
    from backend.loaders.mjcf_loader import MJCFLoader

    _model, report = MJCFLoader(robot_id="mini_arm").load(mini_arm_mjcf)
    return report


@pytest.fixture(scope="session")
def mj_model(mini_arm_mjcf: Path):
    """原生 `MjModel`（供"RobotModel ↔ MuJoCo 一致性"测试用）。

    它是**对照系**，不是契约 —— 测试用它来验证我们的转换没读错。
    """
    import mujoco

    return mujoco.MjModel.from_xml_path(str(mini_arm_mjcf))


@pytest.fixture
def mini_arm_pkg():
    """mini_arm 的包内运动学模块（fk / ik）。

    通过 importlib 按**路径**加载 —— 因为 `packages/` 不是 Python 包
    （它的 `__init__.py` 刻意不存在：包是**数据 + 算法**的容器，
    不是可 import 的库。让它可 import 会鼓励跨包硬引用，
    而"加一台机器人"应该只需加目录）。
    """
    import importlib.util

    def _load(module_name: str, rel: str):
        path = MINI_ARM_DIR / rel
        assert path.is_file(), f"包内模块缺失：{path}"
        spec = importlib.util.spec_from_file_location(module_name, path)
        assert spec and spec.loader
        mod = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = mod
        spec.loader.exec_module(mod)
        return mod

    fk = _load("mini_arm_fk", "kinematics/fk.py")
    ik = _load("mini_arm_ik", "kinematics/ik.py")
    return type("MiniArmPkg", (), {"fk": fk, "ik": ik, "dir": MINI_ARM_DIR})()
