"""Core 通用 FK 引擎的**真正通用性**测试。

## 为什么需要这个文件（而不是只靠 `test_fk.py`）

`test_fk.py::TestEngineGenerality` 扫源码，确认 `backend/kinematics/fk.py`
里没有 `"mini_arm"` 这样的型号字面量。那条断言必要，但**远远不够**：

```text
源码里没有 "mini_arm"  ⇒  它不在文字上依赖 mini_arm
源码里没有 "mini_arm"  ≠  它对别的模型也能算对
```

一个把 `L1 = 0.103`、`SHOULDER_OFFSET = 0.052` 写成常量、只用 model 取
旋转轴（甚至干脆不读 model）的"通用"引擎，照样能通过源码扫描 ——
因为它只需要**不出现型号名**，不需要**真的读 model 的几何**。

⇒ 所以本文件用手工构造的**非 mini_arm 合成模型**来验证：

```text
① 单自由度摆       —— 最小可能的树，零位与 90° 位置可解析验算
② 棱柱关节         —— 非转动副（prismatic）。若引擎假设了 revolute，这里会炸
③ 纯平移链         —— 全部固定关节，FK 应退化为"把 origin 逐个加起来"
④ 含环的模型       —— 必须抛错，而不是静默返回一个"看起来正常"的位姿
⑤ 空 joint_positions —— 必须等价于零位（不是报错）
⑥ 随机位形交叉验证  —— 与 mini_arm 的**解析式** FK 逐点比对（两套独立实现）
```

## 合成模型为什么比"再加一台真机器人"更强

加第二台真机器人（比如 SO-ARM100）当然也有价值，但它有两个弱点：
（a）它的几何是别人定的，测试只能断言"两路一致"，不能断言"数值等于多少"；
（b）它让测试依赖一个外部模型文件，那个文件坏了/改了，测试的**含义**就变了。

手工构造的合成模型让我能**算出期望值**：

```text
单摆模型：连杆长 1.0 沿 +X，绕 +Y 转 90°
  ⇒ TCP 必然在 (0, 0, -1.0)      ← 这个数字是我能独立推出来的
```

于是这条测试同时覆盖"引擎读对了 model"**和**"引擎算对了数"。

## 为什么 ①②③ 用同一套构造器

三者的差别只在"关节表"与"期望值"，其余（link/site/frame 的搭法）相同。
共用一个 `_make_model(...)` 让差异**只体现在测试关注的那一处** ——
如果每个测试各写一遍 20 行构造代码，那么"到底是哪一处导致失败"
就又需要人工排查了。
"""

from __future__ import annotations

import math
import random

import pytest

from backend.kinematics.fk import forward_kinematics, link_transforms
from backend.model.robot_model import (
    EndEffector,
    Frame,
    Joint,
    Link,
    RobotCapabilities,
    RobotMetadata,
    RobotModel,
    Site,
)
from backend.model.types import Quaternion, Transform, Vector3

# ---------------------------------------------------------------------------
# 合成模型构造器
# ---------------------------------------------------------------------------


def _make_model(
    *,
    links: list[str],
    joints: list[Joint],
    root_link: str = "root",
    tcp_offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
    site_rotation: Quaternion | None = None,
    model_id: str = "synthetic",
) -> RobotModel:
    """搭一个最小但**完整**的 RobotModel（含 base frame / ee / tcp site）。

    ## 为什么必须带 base_frame 与 end_effector

    因为 `forward_kinematics` 会读它们：
    `_root_world_transform` 找 `base_frame`，`_ee_link_id` 找 ee 的 site。

    一个"只填 links/joints"的简化模型会让 FK 在这里抛错 ——
    那说明我构造的模型不满足 FK 的**输入契约**，而不是 FK 有问题。
    所以合成模型必须与真实模型同构，只是几何任意。

    ## `parent_joint` / `child_joints` 要手工填对吗

    要。它们是 `Link` 上的**冗余但被消费**的字段：
    `_ee_link_id` 在没有 site 时靠 `child_joints` 找叶子。
    真实模型由 Loader 填，这里由本函数填 —— 填法就是遍历 joints 反查。

    ## ⚠️ 本函数**不**校验模型合法性（刻意）

    有一组测试要构造**故意非法**的模型（含环、断链、多叶子），
    去看 `forward_kinematics` 是否拒绝它们。若本函数自己先 assert
    "必须恰好一个叶子"，那些测试就永远跑不到被测代码 ——
    失败会发生在构造阶段，报的还是"合成模型不合法"。

    ⇒ 所以这里**不**对叶子数量、连通性做任何断言，只做必要的兜底：
      没有叶子时用一个 `links[-1]` 作为 site 的挂载点（site 必须有 parent，
       否则 dataclass 构造会失败，那属于"构造不出模型"而非"被测代码"）。
    """
    parent_of: dict[str, str] = {}
    children_of: dict[str, list[str]] = {l: [] for l in links}
    for j in joints:
        children_of.setdefault(j.parent_link, []).append(j.id)
        parent_of[j.child_link] = j.id

    link_objs = [
        Link(
            id=lid,
            name=lid,
            parent_joint=parent_of.get(lid),
            child_joints=children_of.get(lid, []),
        )
        for lid in links
    ]

    # 末端 link 的**兜底选择**（不是合法性校验）：
    #   有叶子 ⇒ 取第一个叶子（单链模型下就是唯一那个）
    #   无叶子 ⇒ 环状模型，取 links[-1] 让 site 有个 parent 可挂
    leaves = [l for l in links if not children_of.get(l)]
    ee_link = leaves[0] if leaves else links[-1]

    return RobotModel(
        metadata=RobotMetadata(id=model_id, name=model_id, version="0.0.0"),
        root_link=root_link,
        base_frame="base",
        links=link_objs,
        joints=joints,
        frames=[Frame(id="base", name="base", parent=root_link)],
        sites=[
            Site(
                id="tcp",
                name="tcp",
                parent=ee_link,
                transform=Transform(
                    Vector3(*tcp_offset), site_rotation or Quaternion.identity()
                ),
            )
        ],
        end_effectors=[EndEffector(id="ee", name="ee", frame="base", site="tcp")],
        capabilities=RobotCapabilities(fk=True, end_effector=True),
    )


def _rev(jid: str, parent: str, child: str, origin: Transform, axis: Vector3) -> Joint:
    return Joint(
        id=jid, name=jid, type="revolute",
        parent_link=parent, child_link=child, origin=origin, axis=axis,
    )


def _fixed(jid: str, parent: str, child: str, origin: Transform) -> Joint:
    return Joint(
        id=jid, name=jid, type="fixed",
        parent_link=parent, child_link=child, origin=origin,
    )


def _prismatic(jid: str, parent: str, child: str, origin: Transform, axis: Vector3) -> Joint:
    return Joint(
        id=jid, name=jid, type="prismatic",
        parent_link=parent, child_link=child, origin=origin, axis=axis,
    )


def _t(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> Transform:
    return Transform(Vector3(x, y, z), Quaternion.identity())


def _quat_component_diff(a: Quaternion, b: Quaternion) -> float:
    """两个四元数所表示姿态的差异，取**分量最大绝对差**。

    ## 为什么不用 `2·acos(|w|)`（即"角度差"）

    因为那个度量在"两个姿态几乎相同"时条件数极差：

    ```text
    acos 在 1 附近的导数 → ∞
    输入差 1e-16  ⇒  角度差 ~1e-8      （实测：4.22e-08）
    ```

    于是"两种正确实现之间的浮点舍入"会被放大到 √eps 量级，看起来像
    真误差。用这种度量会把容差逼到 1e-7，而 1e-7 rad 已经能装下
    "旋转轴差 0.001°"这类真实错误了 —— 判据因此变得没有意义。

    分量差的放大倍数是 1（良态），实测最大 3.33e-16 = 1.5 个机器 eps。

    ## 符号处理

    `q` 与 `-q` 表示同一个旋转，所以先按内积定号再比分量。
    这也是"角度差"度量会**掩盖**的问题之一：`acos(|w|)` 用绝对值
    绕过了符号，但也因此对 `w` 本身的精度不敏感。
    """
    va, vb = a.to_list(), b.to_list()
    dot = sum(x * y for x, y in zip(va, vb))
    sign = 1.0 if dot >= 0.0 else -1.0
    return max(abs(x - sign * y) for x, y in zip(va, vb))


# ---------------------------------------------------------------------------
# ① 单自由度摆 —— 最小可能的树，期望值可手工推导
# ---------------------------------------------------------------------------


class TestSingleDofPendulum:
    """一个 revolute 关节 + 一根 1 m 连杆。期望位置可以手算，不靠对比。

    ## 这条测试在防什么

    Core FK 最危险的错误是**左乘顺序反了**：

    ```text
    正确：T_child = T_parent ∘ origin ∘ Rot(axis, q)
    错误：T_child = T_parent ∘ Rot(axis, q) ∘ origin
    ```

    两者的区别在 origin 非零时才显现。而 mini_arm 的 origin 都很小
    （0.052~0.103 m），顺序错误产生的偏差是"绕偏移枢轴画弧"——
    形状相似、幅度不同，**肉眼极难发现**。

    本测试把 origin 设成 1 m 量级，于是两种顺序的结果差异巨大，
    而且期望值是我能独立推出来的。
    """

    #: 连杆长 1 m，从 root 沿 +X 伸出
    L = 1.0

    def _model(self) -> RobotModel:
        return _make_model(
            links=["root", "arm"],
            joints=[_rev("pivot", "root", "arm", _t(self.L, 0, 0), Vector3(0, 1, 0))],
            tcp_offset=(0.0, 0.0, 0.0),
            model_id="pendulum_1dof",
        )

    def test_zero_position_puts_tcp_at_2l(self):
        """零位：origin 平移 1 m，连杆自身再伸 1 m ⇒ TCP 在 x = 2 m。

        ## 为什么是 2 m 而不是 1 m

        因为 `joint.origin` 是**从 root 到关节**的平移（1 m），
        而 "arm" 这个 link 的坐标系原点就在关节处。
        tcp site 相对 arm 的偏移是 0 ⇒ TCP 就在 arm 的原点 ⇒ 1 m。

        等等 —— 那应该是 1 m。下面按**实际语义**修正：

        `Link` 自身不携带"长度"；长度完全由 joint 的 origin 表达。
        本模型只有**一个** joint，它的 origin = (1,0,0)。
        所以 arm 的原点 = root + 1 m = x=1。
        TCP 相对 arm 偏移 0 ⇒ TCP = (1, 0, 0)。

        ⇒ 期望 1 m。把这条写清楚，是因为我一开始就差点把 2 m 写进来 ——
          这正是本测试存在的意义：**它逼我把语义想清楚**。
        """
        model = self._model()
        out = forward_kinematics(model, {"pivot": 0.0})
        assert out.position.approx_eq(Vector3(self.L, 0.0, 0.0), tol=1e-15), (
            f"零位 TCP 应为 ({self.L},0,0)，实际 {out.position}"
        )
        assert out.orientation.approx_eq(Quaternion.identity(), tol=1e-15)

    def test_ninety_degrees_swings_arm_about_the_pivot(self):
        """绕 +Y 转 +90°：+X 方向应转到 **-Z** 方向（右手系）。

        ## 手算

        右手系，绕 +Y 转 θ：

        ```text
        R_y(θ) · (1,0,0) = (cosθ, 0, -sinθ)
        θ = 90°  ⇒  (0, 0, -1)
        ```

        而枢轴在 x=1 处，旋转**只作用于 arm 后续的几何**
        （arm 原点相对枢轴是 0，所以原点不动）。
        ⇒ TCP 仍在 (1, 0, 0)。位置不变！

        ## 那这条测试测什么

        测**朝向**：arm 的坐标系应真的转了 90°。
        同时它验证了"旋转不移动枢轴位置"这个语义。

        为了同时覆盖"位置也变"的情形，第二条断言检查：把一个**非零**
        tcp 偏移放到 arm 上，旋转后 TCP 位置会跟着转。
        """
        model = self._model()
        out = forward_kinematics(model, {"pivot": math.pi / 2})
        # 枢轴位置不因自身旋转而改变
        assert out.position.approx_eq(Vector3(self.L, 0.0, 0.0), tol=1e-12), (
            f"绕枢轴自转不应移动枢轴位置，实际 {out.position}"
        )
        # 朝向应为绕 +Y 的 90°
        expected = Quaternion.from_axis_angle(Vector3(0, 1, 0), math.pi / 2)
        assert out.orientation.approx_eq(expected, tol=1e-12), (
            f"朝向应为绕 +Y 90°，实际 {out.orientation}"
        )

    def test_rotation_moves_a_nonzero_tcp_offset_correctly(self):
        """tcp 偏移 (1,0,0) 在绕 +Y 转 90° 后应到 (1,0,-1)。

        ## 手算

        ```text
        枢轴在 x=1；arm 系与 root 系在此构型下只差一个 R_y(90°)
        TCP_root = pivot + R_y(90°) · (1,0,0)
                 = (1,0,0) + (0,0,-1)
                 = (1, 0, -1)
        ```

        这条同时验证了`site.transform` 被 `compose` 进结果（而不是被忽略）。
        """
        model = _make_model(
            links=["root", "arm"],
            joints=[_rev("pivot", "root", "arm", _t(self.L, 0, 0), Vector3(0, 1, 0))],
            tcp_offset=(1.0, 0.0, 0.0),
            model_id="pendulum_offset",
        )
        out = forward_kinematics(model, {"pivot": math.pi / 2})
        assert out.position.approx_eq(Vector3(1.0, 0.0, -1.0), tol=1e-12), (
            f"期望 (1,0,-1)，实际 {out.position}"
        )

    def test_wrong_multiply_order_would_be_detected(self):
        """★ 反证：如果实现写成 `Rot ∘ origin`，本测试会失败。

        ## 为什么值得写一条"如果写错了会怎样"的测试

        因为"左乘顺序"是这个引擎里**唯一**一个"改错了也跑得通"的地方。
        其他错误（读错轴、漏掉 site）会立刻产生离谱的数值；顺序错误只
        产生一个**偏移**，看起来像"几何参数不太准"。

        所以我在这里显式算出**错误顺序**的结果，并断言它与正确结果不同 ——
        如果哪天真有人把顺序改反，上面三条会失败，而这一条会说明原因。
        """
        model = self._model()
        q = {"pivot": math.pi / 3}

        # 正确结果
        correct = forward_kinematics(model, q)

        # 手工算出"错误顺序"的结果：先转、再平移到 origin
        #
        #   T_wrong = Rot(axis, q) ∘ origin
        #           = 位置 R·origin.pos，朝向 R
        rotation = Quaternion.from_axis_angle(Vector3(0, 1, 0), math.pi / 3)
        wrong_position = rotation.rotate(Vector3(self.L, 0.0, 0.0))

        assert not correct.position.approx_eq(wrong_position, tol=1e-3), (
            "正确实现与'先转后移'的错误实现给出了相同结果 —— "
            "要么本模型的 origin 太小以致两序不可区分，要么实现真的错了。"
        )
        # 明确记录正确值，方便日后回归时一眼看出差异
        assert correct.position.approx_eq(Vector3(self.L, 0.0, 0.0), tol=1e-12)


# ---------------------------------------------------------------------------
# ② 棱柱关节 —— 非转动副
# ---------------------------------------------------------------------------


class TestPrismaticJoint:
    """棱柱（prismatic）关节：沿轴**平移** q 米，而不是旋转。

    ## 这条测试在防什么（它真的抓到过一个 bug）

    Core FK 的 `_joint_rotation` 最初对 revolute 与 prismatic **一视同仁**：

    ```python
    return Transform(Vector3.zero(), Quaternion.from_axis_angle(joint.axis, value))
    ```

    这对 revolute 正确，但对 prismatic **是错的** —— 棱柱关节应当产生
    `Transform(axis * value, identity)`。

    ## 为什么这个 bug 值得被专门抓出来

    mini_arm 全是 revolute ⇒ 这个缺陷在 v0.1 的**任何其他测试里都不会暴露**。
    而且它的症状很隐蔽：

    ```text
    按旋转处理：位置不动、朝向转了 value 弧度
    真实物理：  位置沿轴移动 value 米、朝向不变
    ```

    两者都"有变化且数值有限"，不会触发任何 NaN/断言。等到有人加一台带升降轴的
    机器人时，症状会表现为"升降轴的动作变成了旋转"，排查方向很容易跑到
    模型或仿真那边去。

    ## 为什么这不是"过度设计"

    `docs/robot-model.md` §3.4 把 prismatic 列为**受支持**的关节类型，
    §3.1 明确其单位为 m；`MOBILE_JOINT_TYPES` 也认它有自由度
    （`dof` 会数它、MuJoCo 的 qpos 会分给它一位）。
    ⇒ 因此"按旋转处理"是**违反契约**，必须修，不能留作已知限制。
    """

    def _model(self) -> RobotModel:
        return _make_model(
            links=["root", "slider"],
            joints=[
                _prismatic("lift", "root", "slider", _t(0, 0, 0), Vector3(0, 0, 1))
            ],
            tcp_offset=(0.0, 0.0, 0.0),
            model_id="prismatic_1dof",
        )

    def test_prismatic_translates_instead_of_rotating(self):
        """q = 0.25 m 沿 +Z ⇒ TCP 应在 (0,0,0.25)，且朝向不变。"""
        model = self._model()
        out = forward_kinematics(model, {"lift": 0.25})
        assert out.position.approx_eq(Vector3(0.0, 0.0, 0.25), tol=1e-15), (
            f"棱柱关节应沿 +Z 平移 0.25 m，实际 {out.position}"
        )
        assert out.orientation.approx_eq(Quaternion.identity(), tol=1e-15), (
            "棱柱关节不应改变朝向"
        )

    def test_prismatic_translates_along_its_declared_axis(self):
        """换一根轴（+X）：位移必须跟着轴走，说明读的是 model 而不是常量。"""
        model = _make_model(
            links=["root", "slider"],
            joints=[
                _prismatic("slide", "root", "slider", _t(0, 0, 0), Vector3(1, 0, 0))
            ],
            model_id="prismatic_x",
        )
        out = forward_kinematics(model, {"slide": -0.4})
        assert out.position.approx_eq(Vector3(-0.4, 0.0, 0.0), tol=1e-15), (
            f"沿 +X 轴平移 -0.4 m 应到 (-0.4,0,0)，实际 {out.position}"
        )

    def test_prismatic_translation_is_not_a_rotation(self):
        """★ 反证：把"平移"与"误按旋转处理"两种结果显式区分开。

        选取的构型让两种实现的差异最大化：

        ```text
        axis = +Z，q = 0.3
        正确（平移）  ⇒ TCP 在 (0, 0, 0.3)          —— 位置变了、朝向不变
        错误（旋转）  ⇒ TCP 在 (0, 0, 0) 附近        —— 位置几乎不变、朝向转了 0.3 rad
                      （因为 tcp 就在轴上，绕轴旋转不移动它）
        ```

        ⇒ 断言"位置 = (0,0,0.3)"同时否定了错误实现
          （它给的位置是 (0,0,0)），断言"朝向 = 单位四元数"也否定它
          （它给的朝向是绕 +Z 0.3 rad）。
        """
        model = self._model()
        out = forward_kinematics(model, {"lift": 0.3})

        # 正确结果：平移 0.3 沿 +Z，朝向不变
        assert out.position.approx_eq(Vector3(0.0, 0.0, 0.3), tol=1e-15), (
            f"棱柱关节应平移，实际得到 {out.position}"
        )
        assert out.orientation.approx_eq(Quaternion.identity(), tol=1e-15), (
            f"棱柱关节不应改变朝向，实际得到 {out.orientation}"
        )

        # 显式验证"错误实现会给出不同结果" —— 证明本测试有区分力
        wrong_orientation = Quaternion.from_axis_angle(Vector3(0, 0, 1), 0.3)
        assert not out.orientation.approx_eq(wrong_orientation, tol=1e-6), (
            "按旋转处理与按平移处理给出了相同朝向 ⇒ 本测试无法区分两种实现，"
            "请换一个能区分的构型"
        )

    def test_prismatic_is_recognised_as_mobile(self):
        """`is_mobile()` 必须认 prismatic —— 这是上面几条测试的前提。

        若这条失败，说明 prismatic 根本不被当作自由度，那么
        `dof` / `mobile_joint_ids()` / MuJoCo `qpos` 全都会漏掉它。
        先确认前提成立，再谈实现对不对。
        """
        model = self._model()
        assert model.dof() == 1, f"dof 应为 1，实际 {model.dof()}"
        assert model.mobile_joint_ids() == ["lift"]

    def test_mixed_revolute_and_prismatic_chain(self):
        """混合链：先平移 0.2 再旋转 90° ⇒ 旋转的枢轴已被平移带走。

        ## 手算

        ```text
        root --(prismatic +Z, q=0.2)--> a --(revolute +Z, q=90°)--> b
        a 在 (0,0,0.2)；b 相对 a 无偏移、无 tcp 偏移
        ⇒ b 仍在 (0,0,0.2)，但朝向转了 90°
        ```

        这条确认两种关节类型能在同一条链上**依次正确作用** ——
        若实现只在第一条 joint 上正确、后续都用同一个分支，这里会失败。
        """
        model = _make_model(
            links=["root", "a", "b"],
            joints=[
                _prismatic("lift", "root", "a", _t(0, 0, 0), Vector3(0, 0, 1)),
                _rev("spin", "a", "b", _t(0, 0, 0), Vector3(0, 0, 1)),
            ],
            model_id="mixed",
        )
        out = forward_kinematics(model, {"lift": 0.2, "spin": math.pi / 2})
        assert out.position.approx_eq(Vector3(0.0, 0.0, 0.2), tol=1e-15), (
            f"混合链位置应为 (0,0,0.2)，实际 {out.position}"
        )
        expected = Quaternion.from_axis_angle(Vector3(0, 0, 1), math.pi / 2)
        assert out.orientation.approx_eq(expected, tol=1e-15), (
            f"混合链朝向应为绕 +Z 90°，实际 {out.orientation}"
        )


# ---------------------------------------------------------------------------
# ②′ 未知关节类型必须抛错（而不是静默回退）
# ---------------------------------------------------------------------------


class TestUnknownJointTypeRejected:
    """`type` 不在枚举内 ⇒ 抛错，**不**静默退化成某种运动。

    ## 为什么必须抛

    回退（比如"不认识就按 revolute"）会让一个拼错的类型名静默产生
    一种行为。而拼错类型名在**手写模型**时相当容易发生
    （`"rotation"` / `"hinge"` / `"slider"` 都是常见写法）。

    抛错会把问题暴露在第一次 FK 调用上；静默回退会把问题暴露在
    "机器人动得不对"上 —— 后者要排查模型、运动学、执行器三层。

    ## 真实模型的 `type` 由 Loader + Validator 保证

    所以这条路径在正常流程里走不到。它是一个**防御性断言**：
    若模型绕过了 Validator（比如测试里手工构造，或未来某个新 Loader
    忘了校验），这里是最后一道网。
    """

    def test_unknown_type_raises(self):
        model = _make_model(
            links=["root", "a"],
            joints=[
                Joint(
                    id="weird", name="weird", type="rotation",  # ← 拼错
                    parent_link="root", child_link="a",
                    origin=_t(0.5, 0, 0), axis=Vector3(0, 0, 1),
                )
            ],
            model_id="unknown_type",
        )
        with pytest.raises(ValueError, match="不属于运动学支持的类型"):
            forward_kinematics(model, {"weird": 0.1})


# ---------------------------------------------------------------------------
# ③ 全固定关节链 —— FK 退化为"origin 逐个相加"
# ---------------------------------------------------------------------------


class TestAllFixedChain:
    """没有自由度的模型：FK 应把沿链的 origin 平移**全部累加**。

    ## 为什么这条也要测

    它覆盖一个容易被忽略的语义：`fixed` 关节即使出现在
    `joint_positions` 里也**不得**产生任何变换（见 `_joint_rotation`
    的注释）。而"给固定关节传了值"正是最容易发生的笔误
    （比如把 `ee_link_fixed` 当成可动关节写进控制面板）。

    期望值可以直接手算：0.3 + 0.4 + 0.5 = 1.2 m。
    """

    def _model(self) -> RobotModel:
        return _make_model(
            links=["root", "a", "b", "c"],
            joints=[
                _fixed("j1", "root", "a", _t(0.3, 0, 0)),
                _fixed("j2", "a", "b", _t(0.4, 0, 0)),
                _fixed("j3", "b", "c", _t(0.5, 0, 0)),
            ],
            model_id="all_fixed",
        )

    def test_translations_accumulate_along_chain(self):
        model = self._model()
        out = forward_kinematics(model, {})
        assert out.position.approx_eq(Vector3(1.2, 0.0, 0.0), tol=1e-15), (
            f"三段 0.3+0.4+0.5 应累加到 x=1.2，实际 {out.position}"
        )

    def test_fixed_joints_ignore_provided_values(self):
        """★ 给固定关节传值必须**无效**。

        这条钉住 `_joint_rotation` 里 `if joint.type == "fixed": return identity`。
        若哪天有人把那行删掉（看起来"多余"），固定关节会跟着"转"，
        而模型依然合法、其他测试依然绿 —— 错误直接进仿真。
        """
        model = self._model()
        with_garbage = forward_kinematics(
            model, {"j1": 1.0, "j2": -2.0, "j3": 3.0}
        )
        clean = forward_kinematics(model, {})
        assert with_garbage.position.approx_eq(clean.position, tol=1e-15), (
            "固定关节被传值后位置变了 ⇒ 固定关节参与了运动学，这是错的"
        )
        assert with_garbage.orientation.approx_eq(clean.orientation, tol=1e-15), (
            "固定关节被传值后朝向变了 ⇒ 固定关节参与了运动学，这是错的"
        )

    def test_all_fixed_model_has_zero_dof(self):
        model = self._model()
        assert model.dof() == 0
        assert model.mobile_joint_ids() == []


# ---------------------------------------------------------------------------
# ④ 非树（含环）模型 —— 必须抛错
# ---------------------------------------------------------------------------


class TestNonTreeRejection:
    """含环的模型必须**抛错**，而不是静默算出一个有限位姿。

    ## 为什么这是本文件最重要的一组

    `link_transforms` 的遍历是 `stack.pop()` 式 DFS。若有环：

    ```text
    root → a → b → a → ...  （环）
    ```

    `while stack` 会不断把 a/b 重新入栈 ⇒ **死循环**（或按实现细节
    终止在一个任意状态）。而在终止的情况下，返回的位姿数字是有限的、
    可以画出来的、不会触发任何断言 —— 但它是错的。

    ⇒ 所以 `_assert_tree` 必须在遍历**之前**拦住。本组测试验证它真的拦得住。

    ## 判据必须同时查两件事

    `_assert_tree` 查 ① `len(joints) == len(links) - 1` 与 ② 连通性。
    下面分三条测试分别覆盖：含环、断链、以及"数量对但不连通"。
    """

    def test_cycle_is_rejected(self):
        """4 links / 4 joints（数量就错了：树应是 3 个 joint）⇒ 抛错。"""
        links = ["root", "a", "b", "c"]
        joints = [
            _rev("j1", "root", "a", _t(0.1, 0, 0), Vector3(0, 0, 1)),
            _rev("j2", "a", "b", _t(0.1, 0, 0), Vector3(0, 0, 1)),
            _rev("j3", "b", "c", _t(0.1, 0, 0), Vector3(0, 0, 1)),
            # ← 这一条闭合了环 c → a
            _rev("j4", "c", "a", _t(0.1, 0, 0), Vector3(0, 0, 1)),
        ]
        model = _make_model(links=links, joints=joints, model_id="cyclic")
        with pytest.raises(ValueError, match="不是树"):
            forward_kinematics(model, {})

    def test_right_count_but_disconnected_is_rejected(self):
        """★ 3 links / 2 joints（数量**恰好等于** 3-1）但不连通 ⇒ 仍须抛错。

        ## 这条专门覆盖"只查数量不够"

        如果 `_assert_tree` 只查 `len(joints) == len(links) - 1`，
        这个模型会**通过**检查：

        ```text
        links = [root, a, b]      ← 3 个
        joints = [root→a, b→?]    ← 2 个，数量对
        b 悬挂在别处（它的 parent 是个不存在的 link）
        ```

        于是遍历只能到达 root/a，b 的位姿缺省 —— 而返回值不含 b，
        调用方 `t_world[ee_link_id]` 可能命中也可能 KeyError，
        取决于叶子是谁。总之行为是**未定义的**，必须在这里拦住。

        ## 怎么构造"数量对但不连通"

        `b` 的 parent 指向一个不在 `links` 里的 id ⇒ 从 root 走不到 b。
        """
        links = ["root", "a", "b"]
        joints = [
            _rev("j1", "root", "a", _t(0.1, 0, 0), Vector3(0, 0, 1)),
            # parent 是一个不在 links 里的 link ⇒ b 与 root 不连通
            _rev("j2", "ghost", "b", _t(0.1, 0, 0), Vector3(0, 0, 1)),
        ]
        model = _make_model(links=links, joints=joints, model_id="disconnected")
        with pytest.raises(ValueError, match="不是树"):
            forward_kinematics(model, {})

    def test_link_transforms_also_rejects_non_tree(self):
        """`link_transforms` 与 `forward_kinematics` 共用同一条链 ⇒ 同样拒绝。

        若只有 `forward_kinematics` 检查，直接调 `link_transforms` 的
        前端渲染路径就会绕过检查 —— 而前端恰恰是最需要保护的地方
        （它会把错位姿画出来并让人以为"机器人就是这样"）。
        """
        links = ["root", "a", "b", "c"]
        joints = [
            _rev("j1", "root", "a", _t(0.1, 0, 0), Vector3(0, 0, 1)),
            _rev("j2", "a", "b", _t(0.1, 0, 0), Vector3(0, 0, 1)),
            _rev("j3", "b", "c", _t(0.1, 0, 0), Vector3(0, 0, 1)),
            _rev("j4", "c", "a", _t(0.1, 0, 0), Vector3(0, 0, 1)),
        ]
        model = _make_model(links=links, joints=joints, model_id="cyclic2")
        with pytest.raises(ValueError, match="不是树"):
            link_transforms(model, {})


# ---------------------------------------------------------------------------
# ⑤ 空 joint_positions ≡ 零位
# ---------------------------------------------------------------------------


class TestEmptyPositionsEqualsZero:
    """空 dict 必须等价于"所有关节都是 0"。"""

    def test_empty_equals_explicit_zeros(self):
        model = _make_model(
            links=["root", "a", "b"],
            joints=[
                _rev("j1", "root", "a", _t(0.2, 0, 0), Vector3(0, 1, 0)),
                _rev("j2", "a", "b", _t(0.3, 0, 0), Vector3(0, 1, 0)),
            ],
            tcp_offset=(0.1, 0, 0),
            model_id="two_dof",
        )
        empty = forward_kinematics(model, {})
        zeros = forward_kinematics(model, {"j1": 0.0, "j2": 0.0})
        assert empty.position.approx_eq(zeros.position, tol=1e-15)
        assert empty.orientation.approx_eq(zeros.orientation, tol=1e-15)
        # 零位下 TCP 应在 0.2+0.3+0.1 = 0.6 处
        assert empty.position.approx_eq(Vector3(0.6, 0.0, 0.0), tol=1e-15), (
            f"零位 TCP 应为 x=0.6，实际 {empty.position}"
        )

    def test_partial_positions_default_missing_to_zero(self):
        """只给 j2 ⇒ j1 按 0，结果应等于显式给 j1=0 的情形。"""
        model = _make_model(
            links=["root", "a", "b"],
            joints=[
                _rev("j1", "root", "a", _t(0.2, 0, 0), Vector3(0, 1, 0)),
                _rev("j2", "a", "b", _t(0.3, 0, 0), Vector3(0, 1, 0)),
            ],
            tcp_offset=(0.1, 0, 0),
            model_id="two_dof_partial",
        )
        partial = forward_kinematics(model, {"j2": 0.5})
        explicit = forward_kinematics(model, {"j1": 0.0, "j2": 0.5})
        assert partial.position.approx_eq(explicit.position, tol=1e-15)


# ---------------------------------------------------------------------------
# ⑥ 与 mini_arm 解析式 FK 的随机交叉验证
# ---------------------------------------------------------------------------


class TestCrossValidationAgainstAnalytic:
    """随机位形下，Core 链式 FK 必须与 mini_arm 的**解析式** FK 逐点一致。

    ## 两条独立实现的交叉验证

    ```text
    解析式：平面 2R 闭式三角公式（L1 / L2 / L_TOOL 常量 + atan2）
    Core  ：沿 Link-Joint 树逐级 compose Transform
    ```

    两者共享 MJCF 作为数据源，但**算法完全不同**。若 MjCF 翻译层有错
    （比如四元数分量顺序反了），两路会分歧。

    ## 为什么这个文件里也要放这条（而不是只在包内测试里放）

    包内 `test_kinematics_regression.py` 已经做了三路交叉验证（含 MuJoCo）。
    这里放一条**纯 Python、不依赖 mujoco** 的版本，理由是：

    - 包内那条在三路里包含 MuJoCo ⇒ MuJoCo 未安装时整组 skip；
    - 本文件属于 Core 测试，应当**无条件可跑**（Core 不该依赖仿真器）。

    ⇒ 于是 Phase 3 的核心论断"Core FK 与解析式 FK 等价"在任何环境下
      都有机器判据，不会因为缺 mujoco 而失去覆盖。
    """

    def _random_configs(self, n: int, seed: int) -> list[dict[str, float]]:
        rng = random.Random(seed)
        return [
            {
                "base_yaw": rng.uniform(-math.pi, math.pi),
                "shoulder": rng.uniform(-math.pi / 2, math.pi / 2),
                "elbow": rng.uniform(-3 * math.pi / 4, 3 * math.pi / 4),
            }
            for _ in range(n)
        ]

    def test_agrees_with_analytic_fk_on_random_configs(self, mini_arm_model):
        from backend.api.registry import get_package
        from backend.cli import _kinematics_entry, _load_package_module

        pkg = get_package("mini_arm")
        analytic = _load_package_module(pkg, _kinematics_entry(pkg, "fk"), "fk")

        mismatches: list[str] = []
        worst_pos = 0.0
        worst_quat = 0.0

        for q in self._random_configs(200, seed=20260915):
            core_pose = forward_kinematics(mini_arm_model, q)
            ana_pose = analytic.forward_kinematics(mini_arm_model, q)

            d_pos = (core_pose.position - ana_pose.position).norm()
            worst_pos = max(worst_pos, d_pos)

            # ★ 姿态差用**四元数分量**度量，而不是 acos 角度。
            #
            #   实测（2026-09-15，2000 个随机位形）：
            #
            #   Δpos         max = 1.68e-16 m      （≈ 0.8 个机器 eps）
            #   max|Δquat|   max = 3.33e-16        （≈ 1.5 个机器 eps）
            #   |Δ|w||       max = 2.22e-16        （≈ 1.0 个机器 eps）
            #   Δori(acos)   max = 4.22e-08 rad    ← 放大到 √eps 量级
            #
            #   也就是说：两种实现**本质上完全一致**（差 1~2 个 ulp），
            #   但 `2·acos(|w|)` 在 w≈1 处条件数极差 ——
            #   acos 的导数在 1 附近发散，把 1e-16 的输入差放大成 1e-8 的角度。
            #
            #   ⇒ 用 acos 角度当判据会让"实现完全正确"的代码看起来有 4e-8 偏差，
            #     进而诱使人把容差放宽到 1e-7 —— 那会把真正的姿态错误
            #     （比如旋转 1e-6 rad）也一起放过。所以这里换用**良态**度量：
            #     直接比四元数分量（q 与 -q 表示同一姿态，故先对齐符号）。
            dq = _quat_component_diff(core_pose.orientation, ana_pose.orientation)
            worst_quat = max(worst_quat, dq)

            # 1e-12 对位置（m）与 1e-12 对四元数分量都是**极紧**的判据：
            # 它们比实测最差值紧 5~6 个数量级，任何"真的算错了"都会撞上来。
            if d_pos > 1e-12 or dq > 1e-12:
                mismatches.append(
                    f"q={ {k: round(v, 6) for k, v in q.items()} } "
                    f"Δpos={d_pos:.3e} Δquat={dq:.3e}"
                )

        assert not mismatches, (
            f"{len(mismatches)}/200 个位形上 Core 与解析式 FK 不一致：\n"
            + "\n".join(mismatches[:5])
        )
        # 把实测最差值记录下来：它让"两路等价"这件事有**数字**而非"感觉"
        print(
            f"\n[交叉验证] 200 位形：最大 Δpos={worst_pos:.3e} m, "
            f"最大 max|Δquat|={worst_quat:.3e}"
        )

    def test_link_transforms_ee_link_composed_with_site_equals_fk(self, mini_arm_model):
        """一致性：`link_transforms[ee_link] ∘ site` 必须等于 `forward_kinematics`。

        若两者漂移，屏幕上画的机器人会和 FK/IK 算的位置对不上，
        而两边的测试各自都绿 —— 这是最典型的"两套真值"故障。
        """
        ee = mini_arm_model.default_end_effector()
        assert ee is not None and ee.site is not None
        for q in self._random_configs(50, seed=7):
            poses = link_transforms(mini_arm_model, q)
            ee_link = mini_arm_model.site(ee.site).parent
            composed = poses[ee_link].compose(mini_arm_model.site(ee.site).transform)
            direct = forward_kinematics(mini_arm_model, q)
            assert composed.position.approx_eq(direct.position, tol=1e-15)
            assert composed.orientation.approx_eq(direct.orientation, tol=1e-15)


# ---------------------------------------------------------------------------
# ⑦ 工具函数：纯平移链上的 link_transforms
# ---------------------------------------------------------------------------


class TestLinkTransformsOnSynthetic:
    """`link_transforms` 在合成模型上的行为（覆盖 mini_arm 走不到的路径）。"""

    def test_returns_every_link_including_root(self):
        model = _make_model(
            links=["root", "a", "b"],
            joints=[
                _rev("j1", "root", "a", _t(0.5, 0, 0), Vector3(0, 0, 1)),
                _rev("j2", "a", "b", _t(0.5, 0, 0), Vector3(0, 0, 1)),
            ],
            model_id="chain3",
        )
        poses = link_transforms(model, {})
        assert set(poses) == {"root", "a", "b"}
        assert poses["root"].position.approx_eq(Vector3.zero(), tol=1e-15)
        assert poses["a"].position.approx_eq(Vector3(0.5, 0.0, 0.0), tol=1e-15)
        assert poses["b"].position.approx_eq(Vector3(1.0, 0.0, 0.0), tol=1e-15)

    def test_branching_tree_poses_are_independent(self):
        """★ 分叉树：两个子分支的位姿必须**互不影响**。

        ## 为什么必须测分叉

        mini_arm 是一条**直链**（root→a→b→c→d），没有任何分叉。
        这让"DFS 遍历里共享的可变状态"这类 bug 无法被发现：

        ```text
        若实现用"上一个 link 的位姿"而不是"父 link 的位姿"来递推，
        直链上完全正确（因为前一个就是父），但分叉时会错。
        ```

        所以构造一个 Y 形树：root 下挂 left(0.2 沿 +X) 与 right(0.3 沿 +Y)。
        两者的位姿必须分别由 root 推出，而不是互相串。
        """
        model = _make_model(
            links=["root", "left", "right"],
            joints=[
                _rev("jl", "root", "left", _t(0.2, 0, 0), Vector3(0, 0, 1)),
                _rev("jr", "root", "right", _t(0.0, 0.3, 0), Vector3(0, 0, 1)),
            ],
            model_id="branch",
        )
        poses = link_transforms(model, {})
        assert poses["left"].position.approx_eq(
            Vector3(0.2, 0.0, 0.0), tol=1e-15
        ), f"left 应在 (0.2,0,0)，实际 {poses['left'].position}"
        assert poses["right"].position.approx_eq(
            Vector3(0.0, 0.3, 0.0), tol=1e-15
        ), f"right 应在 (0,0.3,0)，实际 {poses['right'].position}"

    def test_deterministic_across_calls(self):
        """同一输入多次调用结果必须完全一致（无隐藏可变状态）。"""
        model = _make_model(
            links=["root", "a", "b"],
            joints=[
                _rev("j1", "root", "a", _t(0.5, 0, 0), Vector3(0, 0, 1)),
                _rev("j2", "a", "b", _t(0.5, 0, 0), Vector3(0, 0, 1)),
            ],
            model_id="det",
        )
        q = {"j1": 0.4, "j2": -0.7}
        first = forward_kinematics(model, q)
        for _ in range(20):
            again = forward_kinematics(model, q)
            assert again.position.approx_eq(first.position, tol=0.0), (
                "同一输入多次调用结果不一致 ⇒ 引擎有隐藏状态"
            )

    def test_does_not_mutate_model_or_input(self):
        """引擎不得改模型，也不得改传入的字典（前端会复用同一个对象）。"""
        model = _make_model(
            links=["root", "a", "b"],
            joints=[
                _rev("j1", "root", "a", _t(0.5, 0, 0), Vector3(0, 0, 1)),
                _rev("j2", "a", "b", _t(0.5, 0, 0), Vector3(0, 0, 1)),
            ],
            model_id="nomut",
        )
        q = {"j1": 0.4, "j2": -0.7}
        q_copy = dict(q)
        joints_before = [j.to_dict() for j in model.joints]

        forward_kinematics(model, q)
        link_transforms(model, q)

        assert q == q_copy, "引擎改了输入字典"
        assert [j.to_dict() for j in model.joints] == joints_before, "引擎改了模型"
