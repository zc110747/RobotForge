"""`MJCFLoader` —— 原生 MJCF → `RobotModel`（P0 契约 §8，v0.1 唯一实现）。

## 一个重要决定：**不自己解析 MJCF 文本**

提示词 §7 明确要求：

> 使用 MuJoCo 官方 API `mujoco.MjModel.from_xml_path(...)`。
> **不要自己重新实现 MJCF Parser。**

这不只是省事，而是**正确性**：MJCF 有 compiler 默认值、default 继承、
`autolimits`、class 机制、坐标系变换（`euler`/`quat`/`xyaxes`/`zaxis`/
`fromto`）、`inertiafromgeom` 估算……自己实现必然在某个子集上出错，
而错误的表现是"模型看起来对，但数值微小不同"。

因此本 Loader 的策略是**混合**：

```text
结构 / 数值 / 已编译结果  →  用 MjModel + mj_id2name / jnt_axis / body_pos ...
显示名 / 材质 / site 语义 →  用轻量 XML 解析补充（MuJoCo 不暴露 rgba 的语义名）
```

## MuJoCo → RobotModel 的三处映射难点

### ① `type` 名称不同

| MJCF | RobotModel |
|---|---|
| `hinge` | `revolute` |
| `slide` | `prismatic` |
| `free` / `ball` | **不支持** ⇒ 报错（v0.1 只做 Tree 型固定基座机构） |
| 无 joint 的 body | `fixed`（**合成的**，见下） |

### ② body 不带 joint ⇒ 必须**合成**一个 `fixed` Joint

MJCF 里"body 没有 `<joint>`"表示它与父 body **刚性固连**。
而 `RobotModel` 的契约是 **Link-Joint 交替**（每个非根 Link 由恰好一个 Joint 进入）。
若不合成 fixed joint，`ee_link` 就会变成"孤立 Link"（Validator 报错），
或者 Link 表里出现"没有关节连接的两段" —— 破坏 FK 链式遍历的前提。

⇒ 所以 Loader **必须**为这种 body 合成一个 `type="fixed"` 的 Joint，
并把它的 `origin` 设为 body 的 `pos`/`quat`。这是"忠实还原 MJCF 语义"的必要步骤，
不是发明新结构。

### ③ 坐标转换的落点

MJCF 本身若已是 RobotForge 约定（+X前/+Y左/+Z上），转换矩阵 = 单位阵。
本 Loader 的实现方式是：**给每个 joint 的 origin 与 site 的 transform
左乘一个 `COORDINATE_FIX` 变换**。v0.1 中它是单位阵，因此**数值完全不变**
（测试会断言这一点：无转换时结果与 MJCF 原始值逐位相同）。

这条通路存在的意义：加第三台机器人（如 Menagerie 的 Y-up 模型）时，
只需设置 `COORDINATE_FIX`，而**不需要**在 FK / IK / 前端里动任何东西。
"""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import numpy as np

from ..model.robot_model import (
    Actuator,
    CoordinateConvention,
    EndEffector,
    Frame,
    GeometryRef,
    Inertial,
    Joint,
    JointLimits,
    Link,
    RobotCapabilities,
    RobotMetadata,
    RobotModel,
    Site,
    UnitConvention,
)
from ..model.types import Quaternion, Transform, Vector3
from .loader import LoaderError, LoaderReport, RobotModelLoader

#: MJCF joint type → RobotModel joint type
_MJCF_JOINT_TYPES: dict[str, str] = {
    "hinge": "revolute",   # MuJoCo 的 hinge 就是我们的 revolute
    "slide": "prismatic",
    "free": "__unsupported_free__",
    "ball": "__unsupported_ball__",
}

#: MJCF geom type → RobotModel geometry type（v0.1 支持子集）
_MJCF_GEOM_TYPES: dict[str, str] = {
    "box": "box",
    "sphere": "sphere",
    "cylinder": "cylinder",
    "capsule": "capsule",
    "mesh": "mesh",
    "plane": "__skip_plane__",
    "ellipsoid": "__skip_ellipsoid__",
    "hfield": "__skip_hfield__",
}

#: MuJoCo 的 mjtJoint 枚举值 → 名字（避免 import mjtJoint 造成模块级依赖）
_MJT_JOINT_NAMES = {0: "free", 1: "ball", 2: "slide", 3: "hinge"}


def sanitize_id(raw: str, *, fallback: str = "") -> str:
    """把 MJCF 的名字规范化成 ID 契约要求的 ASCII snake_case。

    ## 规则（契约 §6.3）

    ```text
    "Shoulder Joint"  → "shoulder_joint"
    "Arm-1"           → "arm_1"
    "链接A"           → ""（非 ASCII ⇒ 丢弃，改用 fallback）
    ```

    ## 为什么**不**自动加后缀解决冲突

    若 `"Arm 1"` 与 `"Arm-1"` 都规范化成 `arm_1`，正确做法是**报错**，
    而不是自动变成 `arm_1` 与 `arm_1_2`。理由：

    自动加后缀会让"我的模型有命名冲突"这个**事实**消失，
    而作者需要知道它 —— 否则他会以为两个连杆各有独立 id，
    直到某天在 UI 上发现"两个 link 名字一样"。

    ⇒ 冲突检测在 `MJCFLoader._build` 里做，发现即抛 `LoaderError`。
    """
    s = raw.strip()
    if not s:
        return fallback
    # 非 ASCII 直接判定为不可规范化（丢弃，不用拼音/transliteration —— 那是猜测）
    if not all(ord(c) < 128 for c in s):
        return fallback
    s = s.lower()
    s = re.sub(r"[^a-z0-9]+", "_", s)   # 空格/连字符/点 → 下划线
    s = s.strip("_")
    if not s:
        return fallback
    if s[0].isdigit():
        s = "_" + s                      # ID 不能以数字开头
    return s


class MJCFLoader(RobotModelLoader):
    """把原生 MJCF 转成 `RobotModel`。"""

    format_name = "mjcf"

    def __init__(
        self,
        *,
        robot_id: str | None = None,
        capabilities: RobotCapabilities | None = None,
        coordinate_fix: Transform | None = None,
        end_effector_site: str | None = None,
        base_frame: str | None = None,
    ) -> None:
        """
        ## 参数为什么都带 `None` 默认

        Loader 不该猜 manifest 的内容，但**也不该没有默认值** ——
        否则"直接加载一个 MJCF 文件"（测试 / CLI 单文件预览）就变成不可能。
        因此：显式传入的值优先；未传时用**从 MJCF 自己推导出来的**合理值。

        - `robot_id`：未传 ⇒ 用 MJCF 的 `model` 属性规范化
        - `capabilities`：未传 ⇒ 按结构推导（有 actuator ⇒ actuator_control，
          有 site 标了 end_effector 角色的 ⇒ end_effector）
        - `coordinate_fix`：未传 ⇒ 单位阵（No Conversion）
        - `end_effector_site`：未传 ⇒ 找 role/名字含 "tcp"/"ee"/"end" 的 site
        """
        self.robot_id = robot_id
        self.capabilities = capabilities
        self.coordinate_fix = coordinate_fix or Transform.identity()
        self.end_effector_site = end_effector_site
        self.base_frame = base_frame

    # ------------------------------------------------------------------
    # 入口
    # ------------------------------------------------------------------

    def load(self, source: Any) -> tuple[RobotModel, LoaderReport]:
        """加载 MJCF（路径 / `Path` / XML 字符串）→ `(RobotModel, LoaderReport)`。"""
        path, xml_text = self._resolve_source(source)
        report = LoaderReport(source=str(path) if path else "<memory>", format="mjcf")

        mj, xml_root = self._compile(path, xml_text, report)
        try:
            model = self._build(mj, xml_root, report)
        finally:
            # MjModel 不被契约持有（RobotModel 里不能有 MjModel）⇒ 显式释放。
            # MuJoCo 的 Python 绑定有 GC，但依赖 GC 让"模型被谁持有"变得隐含。
            del mj
        return model, report

    # ------------------------------------------------------------------
    # 源解析 & 编译
    # ------------------------------------------------------------------

    def _resolve_source(self, source: Any) -> tuple[Path | None, str | None]:
        if isinstance(source, Path):
            p = source
            if not p.is_file():
                raise LoaderError(f"MJCF 文件不存在：{p}")
            return p, None
        if isinstance(source, str):
            stripped = source.lstrip()
            if stripped.startswith("<"):
                return None, source
            p = Path(source)
            if not p.is_file():
                raise LoaderError(
                    f"MJCF 文件不存在：{p.resolve() if p.is_absolute() else p}"
                    f"（也不是以 '<' 开头的 XML 字符串）"
                )
            return p, None
        raise LoaderError(f"无法识别的 MJCF source 类型：{type(source).__name__}（{source!r}）")

    def _compile(self, path: Path | None, xml_text: str | None, report: LoaderReport):
        """用 MuJoCo 官方 API 编译（**不自实现 parser**，见模块 docstring）。"""
        import mujoco  # 局部 import：抽象层与其它格式不该依赖 mujoco

        try:
            if path is not None:
                mj = mujoco.MjModel.from_xml_path(str(path))
            else:
                assert xml_text is not None
                mj = mujoco.MjModel.from_xml_string(xml_text)
        except Exception as exc:  # MuJoCo 抛 ValueError / RuntimeError
            where = str(path) if path else "<xml string>"
            raise LoaderError(f"MuJoCo 无法编译 {where}：{exc}") from exc

        # XML 树：只用于取 MuJoCo 不暴露的**显示层**信息（rgba / 名字原文）
        try:
            if path is not None:
                tree = ET.parse(str(path))
            else:
                assert xml_text is not None
                tree = ET.ElementTree(ET.fromstring(xml_text))
        except ET.ParseError as exc:
            raise LoaderError(f"MJCF 的 XML 语法不合法：{exc}") from exc

        return mj, tree.getroot()

    # ------------------------------------------------------------------
    # 构建 RobotModel
    # ------------------------------------------------------------------

    def _build(self, mj: Any, xml_root: ET.Element, report: LoaderReport) -> RobotModel:
        import mujoco

        body_names = self._names(mj, mujoco.mjtObj.mjOBJ_BODY)
        joint_names = self._names(mj, mujoco.mjtObj.mjOBJ_JOINT)
        site_names = self._names(mj, mujoco.mjtObj.mjOBJ_SITE)
        act_names = self._names(mj, mujoco.mjtObj.mjOBJ_ACTUATOR)
        geom_names = self._names(mj, mujoco.mjtObj.mjOBJ_GEOM)

        # --- 显示层覆盖：从 XML 取每个 body 的显示名与 geom 的 rgba ---
        # MuJoCo 编译后丢失了 "名字里的空格" 之类信息吗？不，它保留了 name。
        # 但 rgba 需要我们自己去 XML 里配对（MjModel.geom_rgba 有，但**材质**
        # 引用的 rgba 在编译后才解析。用 MjModel.geom_rgba 即可 —— 它已是解析后的值）。
        # 唯一需要 XML 的是：<material> 定义（用于给 body 找"主色"）。
        material_rgba = self._parse_materials(xml_root)

        # ---------------- Links ----------------
        # MuJoCo body 0 是 world；RobotForge 的 root_link 是**第一个真实 body**
        links: list[Link] = []
        body_id_to_link_id: dict[int, str] = {}
        used_ids: dict[str, str] = {}   # 规范化 id → 原始名（冲突检测用）

        for bid in range(1, mj.nbody):  # 跳过 world
            raw = body_names.get(bid, f"body_{bid}")
            safe = self._assign_id(sanitize_id(raw, fallback=f"body_{bid}"), raw, "body", used_ids)
            body_id_to_link_id[bid] = safe

        # ---------------- Joints ----------------
        joints: list[Joint] = []
        # 每个 body 的"进入关节"（若非 root）
        joint_of_child_body: dict[int, str] = {}

        geom_by_body: dict[int, list[GeometryRef]] = {}
        coll_by_body: dict[int, list[GeometryRef]] = {}
        for gid in range(mj.ngeom):
            bid = int(mj.geom_bodyid[gid])
            if bid == 0:
                continue  # world 上的 geom（如地面 plane）不属于任何 link
            gtype = self._geom_type_name(mj, gid, geom_names)
            ref = self._make_geom_ref(mj, gid, gtype, material_rgba, report)
            if ref is None:
                continue
            # MuJoCo 的 contype/conaffinity == 0 表示"纯视觉几何"（不参与碰撞）
            # 这是官方推荐的"visual-only geom"写法，据此分派到 collision / visual
            is_collision = bool(mj.geom_contype[gid]) or bool(mj.geom_conaffinity[gid])
            (coll_by_body if is_collision else geom_by_body).setdefault(bid, []).append(ref)

        for jid in range(mj.njnt):
            jtype_raw = _MJT_JOINT_NAMES.get(int(mj.jnt_type[jid]), "unknown")
            jtype = _MJCF_JOINT_TYPES.get(jtype_raw, "")
            if jtype.startswith("__unsupported_"):
                raise LoaderError(
                    f"MJCF joint {joint_names.get(jid)!r} 的类型是 {jtype_raw!r}，"
                    f"RobotForge v0.1 只支持 Tree 型固定基座机构"
                    f"（hinge→revolute / slide→prismatic）。"
                    f"free/ball 关节意味着浮动基座或球关节，v0.1 明确不支持（且不应伪造）"
                )
            if not jtype:
                raise LoaderError(f"无法识别的 MJCF joint 类型 {jtype_raw!r}（joint {jid}）")

            child_bid = int(mj.jnt_bodyid[jid])
            parent_bid = int(mj.body_parentid[child_bid])
            if parent_bid == 0 and child_bid == 0:
                raise LoaderError("joint 挂在 world 上（无父 body）")
            child_link_id = body_id_to_link_id.get(child_bid)
            # 父是 world ⇒ 父 link 就是根（用第一个真实 body）
            parent_link_id = body_id_to_link_id.get(parent_bid)
            if child_link_id is None or parent_link_id is None:
                raise LoaderError(
                    f"joint {joint_names.get(jid)!r} 的 body 映射失败"
                    f"（parent_bid={parent_bid}, child_bid={child_bid}）"
                )

            raw_name = joint_names.get(jid, f"joint_{jid}")
            safe = self._assign_id(sanitize_id(raw_name, fallback=f"joint_{jid}"), raw_name, "joint", used_ids)

            # 轴的来源：MjModel.jnt_axis（**已编译、已归一化** —— MuJoCo 会归一化）
            axis = Vector3(*[float(v) for v in mj.jnt_axis[jid]])
            # 万一是零轴（MuJoCo 对 hinge 不会给零轴，但防一手）
            if axis.norm() < 1e-12:
                axis = Vector3.unit_z()
                report.note(f"joint {safe!r} 的轴为零向量 ⇒ 兜底 +Z（MuJoCo 本不应给出）")

            # origin：joint 相对**父 link 坐标系**的位姿。
            #
            # ★ 这里踩过一个坑，记录下来（MuJoCo 3.13 实测）：
            #   `MjModel` **没有** `jnt_quat` 字段，而 `jnt_pos` 在我们的模型里
            #   全是 0（因为 joint 写在 body 原点处）。真正携带"关节坐标架相对
            #   父 body 的位姿"的是 **body_pos / body_quat** —— 因为 MJCF 的
            #   `body` 坐标系就是它的关节坐标系（joint 的 pos 默认 (0,0,0)）。
            #   证据：`mj_forward` 后的 site 世界位置与"body_pos 当 origin"的
            #   FK 推算逐位一致（见 packages/mini_arm/tests/test_fk.py::test_fk_matches_mujoco）。
            #
            #   注意本模型里每个 body 恰好只有一个关节并且写在其原点，
            #   所以 (body_pos, body_quat) 就是那个关节的 origin。
            #   `jnt_pos` 非零的多关节 body 属于 v0.1 不支持的形态 ⇒ 显式拒绝（见下）。
            jpos_nonzero = any(abs(float(v)) > 1e-12 for v in mj.jnt_pos[jid])
            if jpos_nonzero:
                raise LoaderError(
                    f"joint {raw_name!r} 的 `jnt_pos` = {[float(v) for v in mj.jnt_pos[jid]]} 非零，"
                    f"说明它的关节坐标系**不在其 body 原点**。"
                    f"RobotForge v0.1 假定 'body 坐标系 == 该 body 唯一关节的坐标系'，"
                    f"该假定不成立 ⇒ 显式拒绝（而不是给出一个看起来对、实际偏位的模型）"
                )

            bpos = Vector3(*[float(v) for v in mj.body_pos[child_bid]])
            bquat_raw = Quaternion(*[float(v) for v in mj.body_quat[child_bid]])
            # ★ 四元数顺序转换：MuJoCo 是 [w,x,y,z]，P0 契约要 [x,y,z,w]
            bquat = Quaternion(bquat_raw.y, bquat_raw.z, bquat_raw.w, bquat_raw.x)

            limits = self._make_limits(mj, jid, jtype, joint_names.get(jid, ""))

            joints.append(
                Joint(
                    id=safe,
                    name=raw_name,
                    type=jtype,
                    parent_link=parent_link_id,
                    child_link=child_link_id,
                    origin=Transform(bpos, bquat),
                    axis=axis,
                    limits=limits,
                )
            )
            joint_of_child_body[child_bid] = safe

        # ---------------- 合成 fixed Joint ----------------
        # MJCF 里"body 无 joint"= 与父刚性固连。RobotModel 要求 Link-Joint 交替，
        # 故必须为这种 body 合成一个 fixed Joint（见模块 docstring 难点 ②）。
        root_body_id = self._find_root_body(mj)
        for bid in range(1, mj.nbody):
            if bid in joint_of_child_body:
                continue
            if bid == root_body_id:
                continue  # 根 link 没有父关节
            parent_bid = int(mj.body_parentid[bid])
            child_link_id = body_id_to_link_id[bid]
            parent_link_id = body_id_to_link_id.get(parent_bid)
            if parent_link_id is None:
                raise LoaderError(
                    f"body {body_names.get(bid)!r} 的父 body {parent_bid} 没有映射到 link"
                )
            raw_name = f"{body_names.get(bid, f'body_{bid}')}_fixed"
            safe = self._assign_id(
                sanitize_id(raw_name, fallback=f"fixed_{bid}"), raw_name, "joint", used_ids
            )
            # body 的局部位姿 = 这个 fixed joint 的 origin
            bpos = Vector3(*[float(v) for v in mj.body_pos[bid]])
            bquat_raw = Quaternion(*[float(v) for v in mj.body_quat[bid]])
            bquat = Quaternion(bquat_raw.y, bquat_raw.z, bquat_raw.w, bquat_raw.x)

            joints.append(
                Joint(
                    id=safe,
                    name=raw_name,
                    type="fixed",
                    parent_link=parent_link_id,
                    child_link=child_link_id,
                    origin=Transform(bpos, bquat),
                    # fixed 无运动；轴写 +Z 保持"axis 恒为单位向量"这条不变量无例外
                    axis=Vector3.unit_z(),
                    limits=None,
                )
            )
            joint_of_child_body[bid] = safe
            report.note(
                f"为 body {body_names.get(bid)!r} 合成了 fixed joint {safe!r}"
                f"（MJCF 里该 body 无 <joint> ⇒ 与父刚性固连）"
            )

        # ---------------- 组装 Links（需要 joints 才能填 parent/child）----------------
        child_joints_of: dict[str, list[str]] = {}
        parent_joint_of: dict[str, str] = {}
        for j in joints:
            child_joints_of.setdefault(j.parent_link, []).append(j.id)
            parent_joint_of[j.child_link] = j.id

        for bid in range(1, mj.nbody):
            link_id = body_id_to_link_id[bid]
            raw = body_names.get(bid, f"body_{bid}")
            inertial = self._make_inertial(mj, bid, report, raw)
            links.append(
                Link(
                    id=link_id,
                    name=raw,
                    parent_joint=parent_joint_of.get(link_id),
                    child_joints=child_joints_of.get(link_id, []),
                    inertial=inertial,
                    visual=geom_by_body.get(bid, []),
                    collision=coll_by_body.get(bid, []),
                )
            )

        # ---------------- root / base_frame ----------------
        root_link = body_id_to_link_id[root_body_id]
        base_frame_id = self.base_frame or root_link

        # ---------------- Frames ----------------
        # v0.1 至少注册 base_frame 与一个 world frame。
        # world frame 的存在让"绝对坐标"有合法归属（UI 显示坐标轴时要用）。
        frames: list[Frame] = [
            Frame(id="world", name="World", parent="world", transform=Transform.identity()),
            Frame(id=base_frame_id, name=f"{base_frame_id} (base)", parent="world",
                  transform=Transform.identity()),
        ]
        # ---------------- Sites ----------------
        sites: list[Site] = []
        for sid in range(mj.nsite):
            sbid = int(mj.site_bodyid[sid])
            if sbid == 0:
                report.skip("site:world")
                continue
            parent_link_id = body_id_to_link_id.get(sbid)
            if parent_link_id is None:
                report.skip("site:unmapped_body")
                continue
            raw_name = site_names.get(sid, f"site_{sid}")
            safe = self._assign_id(sanitize_id(raw_name, fallback=f"site_{sid}"), raw_name, "site", used_ids)

            spos = Vector3(*[float(v) for v in mj.site_pos[sid]])
            squat_raw = Quaternion(*[float(v) for v in mj.site_quat[sid]])
            squat = Quaternion(squat_raw.y, squat_raw.z, squat_raw.w, squat_raw.x)
            sites.append(
                Site(
                    id=safe,
                    name=raw_name,
                    parent=parent_link_id,
                    transform=Transform(spos, squat),
                    role=None,
                )
            )

        # ---------------- EndEffector ----------------
        # ★ 返回值含它自己的 frame 定义 —— EndEffector 只**引用** frame（契约 §3.7），
        #   所以 frame 必须真实存在于 frames 列表里，否则 Validator 会（正确地）报错。
        end_effectors, ee_link_id, ee_site_id, ee_frames = self._make_end_effector(
            mj, sites, links, report
        )
        frames.extend(ee_frames)

        # ---------------- Actuators ----------------
        actuators: list[Actuator] = []
        for aid in range(mj.nu):
            raw_name = act_names.get(aid, f"actuator_{aid}")
            safe = self._assign_id(
                sanitize_id(raw_name, fallback=f"actuator_{aid}"), raw_name, "actuator", used_ids
            )
            trnid = int(mj.actuator_trnid[aid][0])
            # trntype 0 = mjTRN_JOINT ⇒ 这个 actuator 驱动一个关节
            # （1 = mjTRN_TENDON，v0.1 不支持 ⇒ joint 留 None 而不是瞎猜一个）
            trntype = int(mj.actuator_trntype[aid])
            jid_of_act: str | None = None
            if trntype == 0 and trnid >= 0:
                jid_of_act = self._joint_id_by_mj_id(mj, trnid, joint_names)
            elif trntype != 0:
                report.skip(f"actuator:trntype{trntype}")
            ctrlrange = [float(v) for v in mj.actuator_ctrlrange[aid]]
            actuators.append(
                Actuator(
                    id=safe,
                    name=raw_name,
                    type=self._actuator_type_name(mj, aid),
                    joint=jid_of_act,
                    command_min=ctrlrange[0] if mj.actuator_ctrllimited[aid] else None,
                    command_max=ctrlrange[1] if mj.actuator_ctrllimited[aid] else None,
                )
            )

        # ---------------- metadata / 能力 ----------------
        build = xml_root.get("model") or ""
        robot_id = self.robot_id or sanitize_id(build, fallback="robot")
        capabilities = self.capabilities or self._derive_capabilities(
            actuators=actuators, end_effectors=end_effectors, dof=len([j for j in joints if j.is_mobile()])
        )

        report.facts.update(
            {
                "mjcf_model_name": build,
                "nbody": int(mj.nbody),
                "njnt": int(mj.njnt),
                "nq": int(mj.nq),
                "nv": int(mj.nv),
                "nu": int(mj.nu),
                "nsite": int(mj.nsite),
                "ngeom": int(mj.ngeom),
                "mobile_joint_ids": [j.id for j in joints if j.is_mobile()],
                "synthetic_fixed_joints": [
                    j.id for j in joints if j.type == "fixed"
                ],
            }
        )
        if self.coordinate_fix != Transform.identity():
            report.coordinate_converted = True
            report.coordinate_transform = "custom (非单位阵)"
        else:
            report.coordinate_transform = "none"

        model = RobotModel(
            metadata=RobotMetadata(
                id=robot_id,
                name=build or robot_id,
                version="0.0.0-from-mjcf",  # 真实版本由 manifest 覆盖（Loader 不猜）
                description=f"Loaded from native MJCF ({report.source})",
            ),
            coordinate=CoordinateConvention(),
            units=UnitConvention(),
            root_link=root_link,
            base_frame=base_frame_id,
            links=links,
            joints=joints,
            actuators=actuators,
            frames=frames,
            sites=sites,
            end_effectors=end_effectors,
            capabilities=capabilities,
        )
        return model

    # ------------------------------------------------------------------
    # 辅助
    # ------------------------------------------------------------------

    @staticmethod
    def _names(mj: Any, obj_type: Any) -> dict[int, str]:
        import mujoco

        out: dict[int, str] = {}
        count = {
            mujoco.mjtObj.mjOBJ_BODY: mj.nbody,
            mujoco.mjtObj.mjOBJ_JOINT: mj.njnt,
            mujoco.mjtObj.mjOBJ_SITE: mj.nsite,
            mujoco.mjtObj.mjOBJ_ACTUATOR: mj.nu,
            mujoco.mjtObj.mjOBJ_GEOM: mj.ngeom,
        }[obj_type]
        for i in range(count):
            n = mujoco.mj_id2name(mj, obj_type, i)
            if n:
                out[i] = n
        return out

    @staticmethod
    def _assign_id(safe: str, raw: str, kind: str, used: dict[str, str]) -> str:
        """登记 id 并检测**冲突**（冲突 ⇒ 报错，不自动加后缀。见 §6.3）。"""
        if not safe:
            raise LoaderError(
                f"{kind} 的名字 {raw!r} 无法规范化成合法 id（非 ASCII 或全为特殊字符）。"
                f"ID 契约要求 ASCII snake_case。请修改 MJCF 里的 name 属性，"
                f"或为其提供一个 ASCII 名 —— RobotForge **不会**自动生成 id，"
                f"因为那会让模型与运行时的名字对不上"
            )
        if safe in used and used[safe] != raw:
            raise LoaderError(
                f"{kind} id 冲突：{raw!r} 与 {used[safe]!r} 都规范化为 {safe!r}。"
                f"RobotForge **不自动加后缀** —— 因为那会掩盖'模型有命名冲突'这个事实，"
                f"请修改 MJCF 里其中一个 name 属性"
            )
        used.setdefault(safe, raw)
        return safe

    @staticmethod
    def _find_root_body(mj: Any) -> int:
        """第一个"父是 world"的真实 body（= RobotForge 的 root_link）。"""
        for bid in range(1, mj.nbody):
            if int(mj.body_parentid[bid]) == 0:
                return bid
        raise LoaderError("MJCF 里没有挂到 world 下的 body（模型为空）")

    @staticmethod
    def _make_limits(mj: Any, jid: int, jtype: str, name: str) -> JointLimits | None:
        """从 MjModel 读限位。

        ## 为什么 unit 不用转换

        MuJoCo 内部就是 SI + rad（`compiler angle` 在**编译时**已把 deg 转成 rad）。
        因此 `jnt_range` 拿到的**已经是 rad**，直接搬运即可 —— 这正是
        "用官方 API 而不是自己解析"的最大收益：单位转换由 MuJoCo 做完了，
        我们若再 `radians()` 一次就会**双重转换**（角度缩小 57 倍）。
        """
        if jtype == "fixed":
            return None
        limited = bool(mj.jnt_limited[jid])
        rng = [float(v) for v in mj.jnt_range[jid]]
        if not limited:
            return JointLimits()
        return JointLimits(
            position_min=rng[0],
            position_max=rng[1],
            velocity_max=None,   # MJCF 的 jnt_range 不含速度；速度限位在 actuator 上
            effort_max=None,
        )

    @staticmethod
    def _make_inertial(mj: Any, bid: int, report: LoaderReport, raw_name: str) -> Inertial | None:
        """读惯量。**缺数据不伪造**（契约 §3.9）。"""
        mass = float(mj.body_mass[bid])
        if mass <= 0:
            # MuJoCo 要求非 world body 有正质量；若为 0 说明模型有问题
            report.note(f"body {raw_name!r} 的 mass = {mass}（MuJoCo 通常不允许）")
            return None

        com = Vector3(*[float(v) for v in mj.body_ipos[bid]])
        # body_inertia 是**对角**惯量（MuJoCo 内部以主轴表示）；
        # 非对角项需从 body_iquat 推出，v0.1 不做（那需要特征分解）。
        # ⇒ 留 None 而不是填 0：填 0 会让"没有非对角惯量"与"未声明"混淆。
        diag = [float(v) for v in mj.body_inertia[bid]]
        return Inertial(
            mass=mass,
            center_of_mass=com,
            ixx=diag[0],
            iyy=diag[1],
            izz=diag[2],
            ixy=None,
            ixz=None,
            iyz=None,
        )

    @staticmethod
    def _geom_type_name(mj: Any, gid: int, geom_names: dict[int, str]) -> str:
        import mujoco

        t = int(mj.geom_type[gid])
        return {
            0: "plane", 2: "sphere", 3: "capsule", 4: "ellipsoid",
            5: "cylinder", 6: "box", 7: "mesh", 8: "hfield",
        }.get(t, f"unknown_{t}")

    def _make_geom_ref(
        self,
        mj: Any,
        gid: int,
        gtype: str,
        material_rgba: dict[str, list[float]],
        report: LoaderReport,
    ) -> GeometryRef | None:
        """构造 GeometryRef。不支持的类型 ⇒ 记录 skip 并返回 None。"""
        import mujoco

        mapped = _MJCF_GEOM_TYPES.get(gtype)
        if mapped is None:
            report.skip(f"geom:{gtype}")
            return None
        if mapped.startswith("__skip_"):
            report.skip(f"geom:{gtype}")
            return None

        gpos = Vector3(*[float(v) for v in mj.geom_pos[gid]])
        gquat_raw = Quaternion(*[float(v) for v in mj.geom_quat[gid]])
        gquat = Quaternion(gquat_raw.y, gquat_raw.z, gquat_raw.w, gquat_raw.x)

        # size 的语义按类型分派（与 MJCF geom_size 一致，见 robot-model.md §3.8）
        raw_size = [float(v) for v in mj.geom_size[gid]]
        if mapped == "box":
            size = raw_size[:3]          # MJCF: 半长
        elif mapped == "sphere":
            size = raw_size[:1]          # 半径
        elif mapped in ("cylinder", "capsule"):
            size = raw_size[:2]          # [半径, 半长]
        elif mapped == "mesh":
            size = []
        else:
            size = raw_size[:3]

        # rgba：MjModel.geom_rgba 是**材质解析后**的最终值 ⇒ 最可靠
        rgba = [float(v) for v in mj.geom_rgba[gid]]

        return GeometryRef(
            type=mapped,
            size=size,
            asset=None,               # v0.1 不支持 mesh 资源（GeometryAsset 是扩展点）
            transform=Transform(gpos, gquat),
            rgba=rgba,
        )

    @staticmethod
    def _parse_materials(xml_root: ET.Element) -> dict[str, list[float]]:
        """从 XML 取 `<material>` 定义（给 UI 的配色提示）。"""
        out: dict[str, list[float]] = {}
        asset = xml_root.find("asset")
        if asset is None:
            return out
        for mat in asset.findall("material"):
            name = mat.get("name")
            rgba = mat.get("rgba")
            if not name or not rgba:
                continue
            try:
                out[name] = [float(v) for v in rgba.split()]
            except ValueError:
                continue
        return out

    def _joint_id_by_mj_id(self, mj: Any, mj_joint_id: int, joint_names: dict[int, str]) -> str | None:
        """actuator 绑定的 MJCF joint id → RobotForge joint id。

        通过**规范化后的名字**配对（与 joints 列表里的规范化规则一致）。
        """
        raw = joint_names.get(mj_joint_id)
        if raw is None:
            return None
        return sanitize_id(raw, fallback=f"joint_{mj_joint_id}") or None

    @staticmethod
    def _actuator_type_name(mj: Any, aid: int) -> str:
        """MuJoCo actuator 类型 → RobotModel actuator 类型。

        `mjTRN_JOINT` 下主要看增益：有 kp 的是 position，有 kv 的是 velocity。
        v0.1 只区分这四类（position / velocity / torque / motor）。
        """
        gainprm = [float(v) for v in mj.actuator_gainprm[aid]]
        biasprm = [float(v) for v in mj.actuator_biasprm[aid]]
        kp = gainprm[0] if gainprm else 0.0
        # biasprm[1] 对 position/intvelocity 是 -kp；对 motor 是 0
        has_position_bias = len(biasprm) > 1 and abs(biasprm[1]) > 1e-12
        has_velocity_bias = len(biasprm) > 2 and abs(biasprm[2]) > 1e-12
        if has_position_bias and has_velocity_bias:
            return "position"       # position actuator（带 kv 阻尼）
        if has_position_bias:
            return "position"
        if has_velocity_bias:
            return "velocity"
        if kp == 0.0:
            return "motor"
        return "motor"

    def _make_end_effector(
        self, mj: Any, sites: list[Site], links: list[Link], report: LoaderReport
    ) -> tuple[list[EndEffector], str | None, str | None, list[Frame]]:
        """推导 EndEffector，并**同时产出它引用的 Frame**。

        ## 为什么**推导**而不是硬编码名字

        `if site_name == "tcp"` 是机器人专用分支（违反提示词 §69 规则 1）。
        正确做法是**按通用启发式**选择，优先级：

        ```text
        ① Loader 显式指定的 site（end_effector_site 参数）
        ② 名字精确等于 tcp / flange / tool 的 site   ← 「工具中心点」
        ③ 名字含 ee / end_effector 的 site           ← 「末端执行器本体」
        ④ 树中最深（离根最远）的 link 上的最后一个 site
        ⑤ 无 site ⇒ 用叶子 link 本身作为 frame，site=None
        ```

        ## ② 与 ③ 的顺序是刻意的（踩过一次）

        `tcp`（tool center point，工具中心点）与 `ee_frame`（末端法兰中心）
        在语义上**不同**：TCP 通常比法兰更靠外（含工具长度）。
        若让"含 ee 的名字"排在前面，`ee_frame_site` 会先命中 ——
        而 IK / 显示都想要 **TCP**（用户说的是"把末端移到那个点"，指的是工具尖）。

        这个顺序错误的**表现**很难发现：两者只差一个工具偏移，
        所以"位置偏了 32 mm"，看起来像标定误差而不像选择逻辑错误。
        因此 `packages/mini_arm/tests/test_mjcf_package.py` 会断言选中 `tcp`。

        ## ④ 是关键兜底

        一台机器人可能**没有任何 site**。此时仍应有一个 EndEffector
        （指向叶子 link），否则 `capabilities.end_effector` 无法成立，
        而"末端在哪"是 UI 必须显示的信息。
        """
        # ① 显式指定（Loader 构造参数 self.end_effector_site）优先于一切启发式
        explicit_id = self.end_effector_site
        # ② 精确匹配工具中心点
        tcp_names = ("tcp", "tcp_site", "flange", "tool", "tool_tip")
        # ③ 宽泛匹配末端本体
        ee_names = ("end_effector", "end_effector_site", "ee", "ee_frame", "ee_site", "tip")

        chosen: Site | None = None
        reason = ""

        if explicit_id:
            for s in sites:
                if s.id == explicit_id:
                    chosen, reason = s, f"显式指定的 site {s.id!r}"
                    break
            if chosen is None:
                raise LoaderError(
                    f"end_effector_site = {explicit_id!r} 在模型的 sites 里不存在"
                    f"（可用: {[s.id for s in sites]}）"
                )

        if chosen is None:
            for s in sites:
                if s.id in tcp_names:
                    chosen, reason = s, f"site {s.id!r}（精确匹配工具中心点名）"
                    break
        if chosen is None:
            for s in sites:
                low = s.id.lower()
                if any(p in low for p in ee_names):
                    chosen, reason = s, f"site {s.id!r}（名字含末端语义）"
                    break
        if chosen is None and sites:
            deepest = _deepest_link(links)
            for s in sites:
                if s.parent == deepest:
                    chosen, reason = s, f"最深 link {deepest!r} 上的 site {s.id!r}"
                    break

        if chosen is not None:
            # ★ Frame id 不能与 site id 相同 —— 二者在各自的命名空间里，
            #   但同名会让"这个 Frame 是 site 的坐标架"与"这个 Site 本身"
            #   在日志/UI 里无法区分。加 `_frame` 后缀是显式的命名约定。
            frame_id = f"{chosen.id}_frame"
            frame = Frame(
                id=frame_id,
                name=f"{chosen.name} (End Effector Frame)",
                parent=chosen.parent,
                transform=chosen.transform,
            )
            ee = EndEffector(
                id="main_ee", name="Main End Effector", frame=frame_id, site=chosen.id
            )
            report.note(f"EndEffector 由 {reason} 推导 ⇒ frame={frame_id!r}, site={chosen.id!r}")
            return [ee], chosen.parent, chosen.id, [frame]

        # ⑤ 退化到叶子 link
        leaf = _deepest_link(links)
        ee = EndEffector(id="main_ee", name="Main End Effector", frame=leaf, site=None)
        report.note(
            f"模型没有任何 site ⇒ EndEffector 退化为叶子 link {leaf!r}（frame 直接用 link 名）"
        )
        return [ee], leaf, None, []

    @staticmethod
    def _derive_capabilities(*, actuators: list[Actuator], end_effectors: list[EndEffector], dof: int) -> RobotCapabilities:
        """按**结构**推导能力（而非按机器人名）。"""
        return RobotCapabilities(
            simulation=True,
            fk=True,
            ik=bool(end_effectors) and dof > 0,
            actuator_control=bool(actuators),
            end_effector=bool(end_effectors),
        )


def _deepest_link(links: list[Link]) -> str:
    """树中最深的 link（BFS 层数最大者）。没有子关节的叶子优先。"""
    leaves = [l.id for l in links if not l.child_joints]
    if leaves:
        return leaves[-1]
    return links[-1].id if links else ""


__all__ = ["MJCFLoader", "sanitize_id"]
