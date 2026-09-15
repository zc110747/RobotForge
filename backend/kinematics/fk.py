"""Core 通用**正运动学**（FK）：遍历 Link-Joint 链逐级相乘 `Transform`。

## 归属：这是 Core，不是任何型号的算法

```text
Joint (dict[str, float])
  ↓
forward_kinematics(model, joint_positions)
  ↓
Pose (Transform)          ← TCP 在 base 系中的位姿
```

本文件的算法对**任何 Tree 型 `RobotModel`** 都成立 —— 它不含任何型号名、
任何连杆长度常量、任何关节名。判据见 `tests/test_fk.py::TestEngineGenerality`
（源码扫型号字面量）与 `tests/test_kinematics_core.py`（用**合成的非 mini_arm
模型**验证它真的通用）。

## 核心算法

```text
T_world(root_link) = base_frame.transform   （若 base_frame 挂在 root_link 上）
T_world(child)     = T_world(parent) ∘ joint.origin ∘ Rot(joint.axis, q)
TCP                = T_world(ee_link) ∘ (site 或 frame).transform
```

左乘顺序不能反：`joint.origin` 是**父 link 系**里的偏移，`Rot` 是**关节自身**的
转动。写成 `Rot ∘ origin` 会让关节绕"错误位置的点"转 —— 后果是机器人看着还在动，
但整体绕着一个偏移了的枢轴画弧，属于最难靠肉眼发现的错误之一。

## 为什么"逐级相乘"是对的而不是近似

URDF / MJCF / DH 参数等**所有**序列式表示，其语义就是"每个关节在父坐标系里
先平移到 origin、再绕 axis 旋转"。因此本实现不是某一种约定的近似，
而是这些表示共同的形式语义本身。

## 两个刻意的行为（不要"顺手修掉"）

1. **未提供的关节按 0 处理**，不报错。
   理由：0 是零位；而"缺一个关节就报错"会让 UI 在初始化时（还没拿到全部
   状态）无法渲染第一帧。此契约由 `test_fk.py` 钉住。

2. **多余的 joint_id 被忽略**，不报错。
   理由：上层可能持有"上一个模型"的位姿字典（换机器人时）。报错会让
   换机器人变成必须清空状态的脆弱流程。此契约同样由测试钉住。

## 返回值是 **TCP**，不是法兰

`ee_link` 的位姿与 TCP 差一个工具偏移（mini_arm 是 32 mm 沿 +X）。
返回法兰会让 IK 的往返测试**通过**而真机差 32 mm —— 因为 FK 与 IK
两边都错同一个量，误差互相抵消。
"""

from __future__ import annotations

from ..model.robot_model import RobotModel
from ..model.types import Quaternion, Transform, Vector3


def _child_joints_by_parent(model: RobotModel) -> dict[str, list]:
    """建索引：`parent_link_id → [joint, ...]`。

    只建索引、不做遍历 —— 于是它可被 FK 与 `link_transforms` 共用，
    避免"两条链各自建索引"（那会让两者有漂移的机会）。
    """
    children: dict[str, list] = {}
    for j in model.joints:
        children.setdefault(j.parent_link, []).append(j)
    return children


def _root_world_transform(model: RobotModel) -> Transform:
    """root link 在世界系里的初始位姿。

    契约（`docs/robot-model.md` §3.7 Root Contract）：
    `base_frame` 通常挂在 `root_link` 上，此时它就是 root 的位姿；
    否则（frame 的 parent 不是 root_link）退化为恒等 —— 那种模型意味着
    "base 与 world 同向"，是合法但少见的情形。
    """
    root_frame = model.frame(model.base_frame)
    if root_frame.parent == model.root_link:
        return root_frame.transform
    return Transform.identity()


def _joint_rotation(model: RobotModel, joint, value: float) -> Transform:
    """单个关节在其 origin 之后产生的变换（相对它自己的关节系）。

    ## 三种关节类型，三种语义（`docs/robot-model.md` §3.4 / §3.1）

    | type | 该变换 | `value` 单位 |
    |---|---|---|
    | `fixed` | 恒等（**不参与运动学**） | 无意义 |
    | `revolute` | 绕 `axis` 旋转 `value` | rad |
    | `prismatic` | 沿 `axis` 平移 `value` | m |

    ## `fixed` 为什么必须在这里显式排除

    契约要求 `fixed` 关节也有 `axis` 字段（保持结构统一），但**其值
    不参与运动学**。若这里不判断，一个笔误的键（比如把 `elbow` 写成
    `ee_link_fixed`）会让固定关节跟着转，而模型本身仍然"合法" ——
    错误一路传到仿真。此契约由
    `tests/test_kinematics_core.py::TestAllFixedChain` 钉住。

    ## `prismatic` 为什么必须与 `revolute` 分开（曾经的 bug）

    本函数最初对两者一视同仁（都按 `from_axis_angle` 处理）。这对
    `revolute` 正确，但 `prismatic` 的语义是**沿轴平移**，
    按旋转处理会得到一个物理上完全不动的错位姿：

    ```text
    正确的 prismatic：TCP 沿 +Z 移动 0.25 m ⇒ (0, 0, 0.25)，朝向不变
    错误的（按旋转）：绕 +Z 转 0.25 rad ⇒ 位置不动，朝向转了 14°
    ```

    危险之处在于它**不报错、不为零、看起来像正常数字**，只有主动构造
    prismatic 模型才能发现。`docs/robot-model.md` §3.4 把 prismatic 列为
    受支持的类型、§3.1 明确其单位为 m，而
    `MOBILE_JOINT_TYPES` 也认它有自由度（`dof` 会数它、qpos 会发给它）
    ⇒ 因此"按旋转处理"是**违反契约**，而不是"尚未支持"。

    ⚠️ 未知的 `type` 一律抛错而不是回退：回退会让拼错的类型名
       （比如 `"rotation"`）静默退化成某种行为，而那正是最难查的一类错误。
       模型必须先过 Validator 才可能到这里，所以这里抛错说明
       模型没走 Validator（契约 §3.3 保证 `type` 是枚举之一）。
    """
    if joint.type == "fixed":
        return Transform.identity()
    if joint.type == "revolute":
        return Transform(Vector3.zero(), Quaternion.from_axis_angle(joint.axis, value))
    if joint.type == "prismatic":
        # 沿轴平移 value 米；朝向不变
        return Transform(joint.axis * value, Quaternion.identity())
    raise ValueError(
        f"关节 {joint.id!r} 的类型 {joint.type!r} 不属于运动学支持的类型。"
        f"合法值见 docs/robot-model.md §3.4："
        f"fixed / revolute / prismatic。"
        f"（若模型已过 Validator，则说明它绕过了校验直接构造）"
    )


def _ee_link_id(model: RobotModel) -> str:
    """末端执行器所属的 link id。

    优先取 `end_effector` 绑定链的末端（site 的 parent link）；
    退化到"树的叶子 link"（没有任何 child joint 的 link）。
    """
    ee = model.default_end_effector()
    if ee is not None and ee.site:
        return model.site(ee.site).parent
    # 叶子 link：没有 child_joints 的那个
    for link in model.links:
        if not link.child_joints:
            return link.id
    raise ValueError("模型里找不到叶子 link（不可能：树必有叶子）")


def _assert_tree(model: RobotModel) -> None:
    """确认模型是**树**（而非含环或断链的图）。

    ## 为什么必须显式查

    下面的遍历是 `stack.pop()` 式的 DFS。若模型里有环，它会**静默地**
    反复覆盖同一批 link 的位姿（`while stack` 因为不断入栈而不终止，
    或终止于某个任意状态），最终返回一个"看起来是正常数字"的结果。

    那比崩溃糟得多：一个位姿数字永远有限、永远能画出来、也永远不会
    触发任何断言 —— 但它是错的。所以宁可在这里抛。

    ## 判据

    树的两个等价条件同时成立：
      ① `len(joints) == len(links) - 1`（每条边对应一个非根 link）
      ② 从 root 出发能到达全部 link（连通性）

    只查①不够：8 个 link、7 个 joint 也可能是"一个 5-link 树 + 一个
    3-link 的环"。只查②也不够：一个有环的图也能从 root 到达全部节点。
    """
    n_links, n_joints = len(model.links), len(model.joints)
    if n_joints != n_links - 1:
        raise ValueError(
            f"模型不是树：{n_links} 个 link 应有 {n_links - 1} 个 joint，"
            f"实际 {n_joints} 个 ⇒ 含环或断链。"
            f"运动学遍历要求 Tree 型模型（见 forward_kinematics 的文档）"
        )

    children = _child_joints_by_parent(model)
    seen: set[str] = {model.root_link}
    stack = [model.root_link]
    while stack:
        link_id = stack.pop()
        for j in children.get(link_id, []):
            if j.child_link not in seen:
                seen.add(j.child_link)
                stack.append(j.child_link)
    if len(seen) != n_links:
        missing = sorted({l.id for l in model.links} - seen)
        raise ValueError(
            f"模型不是树：从 root_link {model.root_link!r} 出发只能到达 "
            f"{len(seen)}/{n_links} 个 link，不可达 {missing} ⇒ 图不连通"
        )


def link_transforms(
    model: RobotModel, joint_positions: dict[str, float]
) -> dict[str, Transform]:
    """所有 link 在世界系中的位姿（前端渲染需要逐段的位姿）。

    与 `forward_kinematics` 走**同一条链**，只是返回全部中间结果。
    因此 `link_transforms(...)[ee_link] ∘ site.transform` 必然等于
    `forward_kinematics(...)` —— 这条一致性由测试断言（若两者漂移，
    屏幕上画的机器人会和 FK/IK 算的位置对不上，而两边测试各自都绿）。

    返回的字典**包含 root_link**（其位姿即 `_root_world_transform`）。
    """
    _assert_tree(model)
    children = _child_joints_by_parent(model)

    out: dict[str, Transform] = {model.root_link: _root_world_transform(model)}
    stack = [model.root_link]
    while stack:
        link_id = stack.pop()
        t_parent = out[link_id]
        for j in children.get(link_id, []):
            value = joint_positions.get(j.id, 0.0)
            out[j.child_link] = (
                t_parent.compose(j.origin).compose(_joint_rotation(model, j, value))
            )
            stack.append(j.child_link)
    return out


def forward_kinematics(
    model: RobotModel, joint_positions: dict[str, float]
) -> Transform:
    """正解：`{joint_id: value}` → **TCP** 位姿。

    ## 输入

    `joint_positions`：`{joint_id: value}`，单位由关节类型决定
    （revolute → rad，prismatic → m，见 P0 契约 §3.1）。
    未提供的关节按 0 处理；多余的键被忽略（理由见模块文档末尾）。

    ## 输出

    `Transform`：`position` 为 TCP 在 base 系中的位置（m），
    `orientation` 为**末端 link 的姿态**（四元数 `[x,y,z,w]`）。

    ⚠️ 若 site 相对其 link 带**旋转**，这里会正确包含它（走的是
    `compose`，不是"只取平移"）。mini_arm 的 tcp site 只有平移，
    因此该路径在 v0.1 不被覆盖 —— 但它不是特例代码，是通用路径。

    ## 异常

    - 模型不是树 ⇒ `ValueError`（见 `_assert_tree`）
    - 模型没有 `end_effector` ⇒ `ValueError`
    - ee 绑定的 link 不在链上 ⇒ `ValueError`
    """
    t_world = link_transforms(model, joint_positions)

    ee = model.default_end_effector()
    if ee is None:
        raise ValueError("模型没有 end_effector，无法定义 TCP")

    ee_link_id = _ee_link_id(model)
    if ee_link_id not in t_world:
        raise ValueError(f"end effector 的 link {ee_link_id!r} 不在运动学链上")

    # TCP = ee_link 的世界位姿 ∘ 末端偏移
    # site 优先（它带完整 transform）；无 site 则用 frame（契约 §3.7）
    if ee.site:
        return t_world[ee_link_id].compose(model.site(ee.site).transform)
    return t_world[ee_link_id].compose(model.frame(ee.frame).transform)


__all__ = ["forward_kinematics", "link_transforms"]
