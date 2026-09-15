"""坐标约定契约测试（P0 契约 `docs/coordinate-system.md` 的机器判据）。

## 坐标约定为什么是 P0

因为它错了以后**不会报错**。四个经典静默错误：

```text
① 单位搞错   rad vs deg  → 57× 的角度误差，机械臂"能动但位置离谱"
② 轴序搞错   XYZ vs XZY  → 运动镜像，看起来像"控制方向反了"
③ 四元数序    [x,y,z,w] vs [w,x,y,z] → 部分位形下完全正确，另一些完全错
④ 手性搞错   right vs left → 旋转方向整体反了
```

四者都不产生 NaN、不抛异常、不违反任何类型契约。唯一能抓住它们的
就是**显式断言坐标约定本身**（本文件）与**跨库对照测试**（loader 测试）。
"""

from __future__ import annotations

import math

import pytest

from backend.model.types import Quaternion, Transform, Vector3

# ----------------------------------------------------------------------
# 1. 右手系与轴叉积恒等式
# ----------------------------------------------------------------------


class TestHandedness:
    def test_x_cross_y_equals_z(self):
        """右手系的定义性质：`X × Y = Z`。

        这一条是整套坐标约定的**根**。如果它是错的，那么所有基于
        "角度正方向 = 绕轴右手定则"的推理全部失效。
        """
        cross = Vector3.unit_x().cross(Vector3.unit_y())
        assert cross.approx_eq(Vector3.unit_z(), tol=1e-15), (
            f"X × Y = {cross}，应为 (0,0,1) ⇒ 这不是右手系"
        )

    def test_all_cyclic_permutations(self):
        """右手系的循环性：X×Y=Z, Y×Z=X, Z×X=Y。"""
        pairs = [
            (Vector3.unit_x(), Vector3.unit_y(), Vector3.unit_z()),
            (Vector3.unit_y(), Vector3.unit_z(), Vector3.unit_x()),
            (Vector3.unit_z(), Vector3.unit_x(), Vector3.unit_y()),
        ]
        for a, b, expect in pairs:
            assert a.cross(b).approx_eq(expect, tol=1e-15)

    def test_anticommutative(self):
        """`a × b = -(b × a)` —— 手性的又一种表述。"""
        a, b = Vector3(1, 2, 3), Vector3(4, 5, 6)
        assert a.cross(b).approx_eq(b.cross(a) * -1.0, tol=1e-15)

    def test_axis_semantics_documented(self):
        """把"+X 前 / +Y 左 / +Z 上"这组语义固定下来。

        ⚠ 这条测试的价值不在于"测出了什么"，而在于它**存在** ——
          它让"想把 +Y 改成右"的人必须先改一条有名字的测试，
          从而不能被顺手改掉。
        """
        # 一个"前方偏左上方"的向量应该是 (+, +, +)
        v = Vector3(1.0, 1.0, 1.0)
        assert v.x > 0 and v.y > 0 and v.z > 0
        # 从原点沿 +X 走，然后绕 +Z 转 +90°（右手定则：逆时针俯视）
        # 右手定则下，绕 +Z 转 +90° 把 +X 转到 +Y
        q = Quaternion.from_axis_angle(Vector3.unit_z(), math.pi / 2)
        rotated = q.rotate(Vector3.unit_x())
        assert rotated.approx_eq(Vector3.unit_y(), tol=1e-15), (
            f"绕 +Z 转 +90° 应把 +X 变为 +Y（左手系则会变为 -Y），实际 {rotated}"
        )


# ----------------------------------------------------------------------
# 2. 四元数
# ----------------------------------------------------------------------


class TestQuaternion:
    def test_identity_is_0001(self):
        """单位四元数在 RobotForge 契约里是 `[0,0,0,1]`（`w` 在**最后**）。"""
        q = Quaternion.identity()
        assert q.to_list() == [0.0, 0.0, 0.0, 1.0], (
            f"单位四元数应为 [0,0,0,1]，实际 {q.to_list()}"
        )

    def test_field_order_is_xyzw(self):
        """构造函数的分量顺序必须是 `(x, y, z, w)`。

        ★ 这是与库边界交互时**最容易出错**的一处：
          MuJoCo 内部是 `[w,x,y,z]`，Three.js 是 `[x,y,z,w]`。
        """
        q = Quaternion(0.1, 0.2, 0.3, 0.4)
        assert (q.x, q.y, q.z, q.w) == (0.1, 0.2, 0.3, 0.4)
        assert q.to_list() == [0.1, 0.2, 0.3, 0.4]

    def test_mujoco_wxyz_conversion(self):
        """`to_numpy_wxyz()` 必须真的把 `w` 提到最前面。"""
        q = Quaternion(0.1, 0.2, 0.3, 0.4)
        arr = q.to_numpy_wxyz()
        assert list(arr) == [0.4, 0.1, 0.2, 0.3], (
            f"MuJoCo 顺序应为 [w,x,y,z]=[0.4,0.1,0.2,0.3]，实际 {list(arr)}"
        )

    def test_90deg_about_z(self):
        """绕 +Z 转 90°：四元数应为 `(0, 0, sin45°, cos45°)`。"""
        q = Quaternion.from_axis_angle(Vector3.unit_z(), math.pi / 2)
        s = math.sin(math.pi / 4)
        c = math.cos(math.pi / 4)
        assert abs(q.x) < 1e-15 and abs(q.y) < 1e-15
        assert abs(q.z - s) < 1e-15
        assert abs(q.w - c) < 1e-15

    def test_sign_equivalence(self):
        """`q` 与 `-q` 是同一个旋转 ⇒ `approx_eq` 必须接受这一点。

        ★ 这是"经典陷阱"：直接比对分量会让同一个姿态报 180° 误差。
        """
        q = Quaternion.from_axis_angle(Vector3.unit_z(), math.pi / 3)
        neg = Quaternion(-q.x, -q.y, -q.z, -q.w)
        assert q.approx_eq(neg), (
            "q 与 -q 表示同一旋转，approx_eq 必须返回 True"
        )

    def test_rotation_composition(self):
        """两次 45° 旋转合成 90°。"""
        a = Quaternion.from_axis_angle(Vector3.unit_z(), math.pi / 4)
        b = Quaternion.from_axis_angle(Vector3.unit_z(), math.pi / 4)
        c = a * b
        expect = Quaternion.from_axis_angle(Vector3.unit_z(), math.pi / 2)
        assert c.approx_eq(expect), f"{c} ≠ {expect}"

    def test_composition_order_is_apply_right_first(self):
        """`a * b` 表示"先施加 b，再施加 a"。

        ★ 顺序错了不会报错，只会让组合姿态错 —— 而"先绕自身轴转"
          与"先绕世界轴转"在只有两个关节时往往看起来都对。
        """
        # a: 绕 Z 90°；b: 绕 X 90°
        a = Quaternion.from_axis_angle(Vector3.unit_z(), math.pi / 2)
        b = Quaternion.from_axis_angle(Vector3.unit_x(), math.pi / 2)
        combined = a * b
        # 先施加 b 再施加 a == 对 (b 作用后) 的结果再施加 a
        v = Vector3.unit_y()
        stepwise = a.rotate(b.rotate(v))
        direct = combined.rotate(v)
        assert stepwise.approx_eq(direct, tol=1e-14), (
            f"a*b 的语义应为'先 b 后 a'：{stepwise} ≠ {direct}"
        )

    def test_quaternion_rejects_nan(self):
        """NaN 必须在**构造时**就被拒绝。

        ★ 一个 NaN 关节角会默默污染整条 FK 链，最后表现为"IK 不收敛" ——
          排查方向完全跑偏。在入口拦住是唯一有效的防线。
        """
        with pytest.raises(ValueError):
            Quaternion(0.0, 0.0, 0.0, float("nan"))
        with pytest.raises(ValueError):
            Quaternion(float("inf"), 0.0, 0.0, 1.0)

    def test_vector3_rejects_nan(self):
        with pytest.raises(ValueError):
            Vector3(0.0, float("nan"), 0.0)
        with pytest.raises(ValueError):
            Vector3(float("-inf"), 0.0, 0.0)

    def test_euler_convention_is_rpy(self):
        """Euler 只用于 UI/调试，约定 Roll=X / Pitch=Y / Yaw=Z。

        本机构只有 Y（pitch）与 Z（yaw），所以 R/P/Y 里应有 R=0。
        """
        q = Quaternion.from_axis_angle(Vector3.unit_y(), math.pi / 6)
        roll, pitch, yaw = q.to_euler_rpy()
        assert abs(roll) < 1e-12, f"绕 Y 旋转的 roll 应为 0，实际 {roll}"
        assert abs(pitch - math.pi / 6) < 1e-12, f"pitch 应为 30°，实际 {math.degrees(pitch)}°"
        assert abs(yaw) < 1e-12, f"绕 Y 旋转的 yaw 应为 0，实际 {yaw}"


# ----------------------------------------------------------------------
# 3. Transform
# ----------------------------------------------------------------------


class TestTransform:
    def test_compose_semantics(self):
        """`a.compose(b)` = 先施加 b，再施加 a（与四元数一致）。"""
        a = Transform(Vector3(1, 0, 0), Quaternion.identity())
        b = Transform(Vector3(0, 1, 0), Quaternion.identity())
        c = a.compose(b)
        assert c.position.approx_eq(Vector3(1, 1, 0), tol=1e-15)

    def test_compose_applies_rotation_to_child_translation(self):
        """组合时子变换的平移必须被父旋转**旋转**，而不是直接相加。

        ★ 这是踩坑重点：写成 `a.position + b.position` 会在
          父旋转为单位阵时**恰好正确**，于是一半测试通过、另一半神秘失败。
        """
        a = Transform(Vector3(0, 0, 0), Quaternion.from_axis_angle(Vector3.unit_z(), math.pi / 2))
        b = Transform(Vector3(1, 0, 0), Quaternion.identity())
        c = a.compose(b)
        # 父绕 Z 转 90° ⇒ 子的 +X 平移变成 +Y
        assert c.position.approx_eq(Vector3(0, 1, 0), tol=1e-14), (
            f"{c.position}；子平移未被父旋转变换 ⇒ compose 实现错了"
        )

    def test_inverse_round_trip(self):
        for _ in range(20):
            q = Quaternion.from_axis_angle(
                Vector3(1, 2, 3).normalized(), 0.7
            )
            t = Transform(Vector3(0.1, -0.2, 0.3), q)
            ident = t.compose(t.inverse())
            assert ident.position.norm() < 1e-14
            assert ident.orientation.approx_eq(Quaternion.identity())

    def test_identity_is_neutral(self):
        t = Transform(Vector3(0.3, 0.2, 0.1), Quaternion.from_axis_angle(Vector3.unit_x(), 1.0))
        ident = Transform.identity()
        assert t.compose(ident).position.approx_eq(t.position, tol=1e-15)
        assert ident.compose(t).position.approx_eq(t.position, tol=1e-15)


# ----------------------------------------------------------------------
# 4. 数值契约
# ----------------------------------------------------------------------


class TestNumericContract:
    def test_tolerances_are_defined_and_ordered(self):
        """两档容差的存在本身就是契约（见 tests/__init__ 说明）。"""
        from backend.model.types import TOL_EXACT, TOL_EXTERNAL

        assert TOL_EXACT == 1e-9
        assert TOL_EXTERNAL == 1e-6
        assert TOL_EXACT < TOL_EXTERNAL, (
            "数学精确量（1e-9）必须比外部读入量（1e-6）更严格"
        )

    def test_vector3_equality_does_not_use_numpy_semantics(self):
        """`Vector3` 的 `==` 必须返回 `bool`，不是数组。

        ★ 如果内部存 numpy 数组，`v1 == v2` 会返回 elementwise 数组，
          而 `assert v1 == v2` 会以 "truth value of an array is ambiguous"
          崩掉 —— 或者更糟，在 `if` 里变成一个不报错的错误分支。
          这是**不用 numpy 存契约对象**的直接理由之一。
        """
        a = Vector3(1, 2, 3)
        b = Vector3(1, 2, 3)
        assert (a == b) is True
        assert isinstance(a == b, bool)

    def test_vector3_is_frozen(self):
        """契约对象必须不可变。

        ★ 不可变不是风格偏好，而是**契约的执行机制**：
          `model.runtime_state = ...` 会抛 FrozenInstanceError，
          于是"Runtime 不得往 RobotModel 里塞运行时状态"这条约束
          由运行时强制，而不是靠 code review。
        """
        import dataclasses

        v = Vector3(1, 2, 3)
        with pytest.raises(dataclasses.FrozenInstanceError):
            v.x = 5.0

    def test_transform_json_serializable(self):
        """契约对象必须能直接 `json.dumps`（供 WebSocket 传出去）。

        ★ 存 numpy float32 时 `json.dumps` 会抛 TypeError ——
          这是"计算用 numpy、存储用 python float"这条边界的由来。
        """
        import json

        t = Transform(Vector3(0.1, 0.2, 0.3), Quaternion(0, 0, 0, 1))
        dumped = json.dumps(t.to_dict())
        assert "0.1" in dumped
        loaded = json.loads(dumped)
        assert loaded["position"] == [0.1, 0.2, 0.3]

    def test_no_numpy_types_leak_into_contract(self):
        """`to_dict()` 里的数值必须是**纯 Python float**，不是 numpy 标量。"""
        import json

        t = Transform(Vector3(0.1, 0.2, 0.3), Quaternion(0, 0, 0, 1))
        for v in json.loads(json.dumps(t.to_dict()))["position"]:
            assert type(v) is float, f"期望 float，实际 {type(v)}"
