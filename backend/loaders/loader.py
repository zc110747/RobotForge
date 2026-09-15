"""`RobotModelLoader` —— 机器人描述 → `RobotModel` 的抽象接口（P0 契约 §8）。

## 它在架构中的位置

```text
              RobotModelLoader            ← 本模块（抽象）
                     │
            ┌────────┴────────┐
            │                 │
        MJCFLoader        URDFLoader
         v0.1 ✅            Future ❌
            │                 │
            └────────┬────────┘
                     ▼
                RobotModel
```

## Runtime 只能依赖 RobotModel，**不**依赖 MJCF

这是 Phase 7 要验证的东西（提示词 §64）：

```text
RobotRuntime 里出现 "mjcf" 这个词 ⇒ 架构已破
```

因此本模块**不**import mujoco —— 抽象层不该知道任何具体格式。
`MJCFLoader` 才有权 import mujoco（它是那个格式的适配器）。

## 为什么 `source` 是 `Any` 而不是 `Path`

因为不同 Loader 的 `source` 语义不同：

| Loader | `source` |
|---|---|
| `MJCFLoader` | MJCF 文件路径 / XML 字符串 / `pathlib.Path` |
| `URDFLoader`（future） | URDF 文件路径 / 包 URL |

把它收窄成 `Path` 会强迫未来的 loader 也走文件系统 ——
而"从内存字符串加载"对测试与前端预览很有用。
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from ..model.robot_model import RobotModel


@dataclass
class LoaderReport:
    """加载过程的**报告**（不是错误日志，是"我做了什么"的记录）。

    ## 为什么加载要返回报告

    因为"加载成功"这件事本身**信息量很低**。真正需要知道的是：

    - 我读了哪个文件？（路径可能被 manifest 拼错但恰好命中另一个文件）
    - 我做了坐标转换吗？（若做了，说明"模型与平台不同向"，值得看一眼）
    - 我做了单位转换吗？（同上）
    - 有多少几何被丢弃了？（mesh 在 v0.1 不支持 ⇒ 丢弃是正常的，
      但"丢弃了 40 个 mesh"说明这个模型在 v0.1 里会显示成空壳）
    - 模型声明的能力与结构一致吗？（由 Validator 交叉检查）

    把这些写进 report，让"加载"从黑盒变成可审计的过程。
    它同时是 Phase 1 验收的**证据来源**（提示词 §58 列了十几项验收）。
    """

    #: 实际读取的源（绝对路径 / "<memory>"）
    source: str = ""
    #: 格式名（"mjcf"）
    format: str = ""

    #: 是否做了坐标系转换（转换矩阵不是单位阵）
    coordinate_converted: bool = False
    #: 转换说明（如 "Y-up → Z-up"）
    coordinate_transform: str = "none"
    #: 是否做了单位转换（MJCF 用 SI；若上游是 mm/deg 则需要）
    unit_converted: bool = False

    #: 被**跳过**的元素的分类计数（键 → 数量）
    #: 典型：{"geom:mesh": 2, "geom:plane": 1}
    skipped: dict[str, int] = field(default_factory=dict)
    #: 读取过程中的非致命提醒
    notes: list[str] = field(default_factory=list)

    #: 从 MJCF 直接读到的事实（用于与 manifest 声明对账）
    facts: dict[str, Any] = field(default_factory=dict)

    def skip(self, kind: str, count: int = 1) -> None:
        self.skipped[kind] = self.skipped.get(kind, 0) + count

    def note(self, message: str) -> None:
        self.notes.append(message)

    def summary(self) -> str:
        parts = [f"{self.format} ← {self.source}"]
        parts.append(f"coord={'converted' if self.coordinate_converted else 'none'}")
        parts.append(f"units={'converted' if self.unit_converted else 'si'}")
        if self.skipped:
            parts.append("skipped=" + ",".join(f"{k}×{v}" for k, v in sorted(self.skipped.items())))
        return " · ".join(parts)


class LoaderError(RuntimeError):
    """机器人描述文件无法转成 RobotModel。"""


class RobotModelLoader(ABC):
    """所有模型格式加载器的抽象基类（P0 契约 §8）。

    只有两个成员：`format_name` 与 `load`。
    刻意**不**在抽象层放 `load_manifest` / `validate` —— 那些是
    `RobotRuntime` 的职责（Package 发现 + 校验编排），不是 Loader 的。
    Loader 的职责单一到一句话：

    > **把一个格式的描述文件，转成一个 `RobotModel`。**
    """

    #: 格式标识（小写），用于 manifest 的 `model.format` 对账
    format_name: str = ""

    @abstractmethod
    def load(self, source: Any) -> tuple[RobotModel, LoaderReport]:
        """加载并把结果转成 `RobotModel`。

        返回 `(model, report)`。失败抛 `LoaderError`（**不返回 None**）。
        """
        raise NotImplementedError

    def can_load(self, source: Any) -> bool:
        """该 loader 是否认得这个 source（用于自动分派）。

        默认实现：按扩展名判断。子类可覆盖。
        """
        if isinstance(source, Path):
            source = str(source)
        if not isinstance(source, str):
            return False
        if source.lstrip().startswith("<"):
            # XML 字符串
            return True
        return source.lower().endswith(f".{self.format_name}")


__all__ = [
    "LoaderError",
    "LoaderReport",
    "RobotModelLoader",
]
