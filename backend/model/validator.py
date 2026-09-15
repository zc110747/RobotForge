"""`RobotModelValidator` —— 契约的可执行判据（契约 §9）。

## 它守的是什么

`RobotModel` 是一份**约定**，而约定如果只有文档，就会漂移。
本模块把每一条约定变成**会失败的断言**：

```text
Identity  → id 唯一、格式合法
References→ 每个引用的 id 都真实存在
Tree      → 恰好一个根、无孤立、无循环、无多父
Joint     → axis 归一化、limits 合法、类型合法
Transform → quaternion 归一化、无 NaN
Coordinate→ 与 P0 契约一致（right-handed / x-forward）
```

## 两条纪律

1. **宁可报错，不要兜底**。发现问题就记录到 report，
   由调用方决定 `error` 是否致命（`raise_if_invalid`）。
   但**绝不**自动修复 —— 自动修复会让"模型有问题"这个事实消失。

2. **`warn` 只用于"确实值得人看一眼、但不影响正确性"的事**。
   把错的告警留着比没有告警更糟：一旦有一条已知无害的告警，
   所有人就会忽略整个告警通道。

## 为什么单独一个模块（而不是写在 RobotModel 里）

`RobotModel` 是 frozen 契约对象，不该带校验逻辑（那会让"构造一个模型"和
"校验一个模型"耦合）。而且 Loader 之外的地方（如前端收到的 robot_info）
也需要能独立校验。
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Any

from .robot_model import (
    ACTUATOR_TYPES,
    GEOMETRY_TYPES,
    JOINT_TYPES,
    ROBOTFORGE_COORDINATE,
    ROBOTFORGE_UNITS,
    RobotModel,
)
from .types import TOL_EXTERNAL, Quaternion, Transform, Vector3

#: ID 契约（契约 §6.1）：ASCII snake_case，仅 `[a-z0-9_]`，不以数字开头。
#:
#: 为什么**不许**以数字开头：`_` 开头会给 Python/TS 的可读性带来噪音，
#: 而数字开头（如 `1_link`）在 JS 里做对象 key 尚可，但在 Python 里
#: 无法作为标识符，会让调试时的 `locals()` 快照难以阅读。
ID_PATTERN = re.compile(r"^[a-z_][a-z0-9_]*$")

#: 允许的坐标系轴字符
_AXIS_CHARS = ("x", "y", "z")


@dataclass(frozen=True)
class Issue:
    """一条校验发现。"""

    level: str  # 'error' | 'warn'
    code: str
    where: str
    message: str

    def line(self) -> str:
        icon = "✘" if self.level == "error" else "!"
        return f"  {icon} [{self.code}] {self.where}：{self.message}"


@dataclass
class ValidationReport:
    """校验结果。`ok` 仅取决于是否有 `error`。"""

    robot_id: str = "<unknown>"
    issues: list[Issue] = field(default_factory=list)

    # ---- 记录 ----

    def error(self, code: str, where: str, message: str) -> None:
        self.issues.append(Issue("error", code, where, message))

    def warn(self, code: str, where: str, message: str) -> None:
        self.issues.append(Issue("warn", code, where, message))

    # ---- 查询 ----

    @property
    def errors(self) -> list[Issue]:
        return [i for i in self.issues if i.level == "error"]

    @property
    def warnings(self) -> list[Issue]:
        return [i for i in self.issues if i.level == "warn"]

    @property
    def ok(self) -> bool:
        return not self.errors

    def raise_if_invalid(self) -> "ValidationReport":
        """有 error ⇒ 抛 `RobotModelError`（带全部错误行，便于一次看全）。"""
        if not self.ok:
            detail = "\n".join(i.line() for i in self.errors)
            raise RobotModelError(
                f"RobotModel {self.robot_id!r} 未通过校验（{len(self.errors)} 个错误）：\n{detail}"
            )
        return self

    def format(self) -> str:
        if not self.issues:
            return f"RobotModel {self.robot_id!r} 校验通过（0 error / 0 warn）"
        head = f"RobotModel {self.robot_id!r}: {len(self.errors)} error / {len(self.warnings)} warn"
        return "\n".join([head] + [i.line() for i in self.issues])


class RobotModelError(ValueError):
    """`RobotModel` 违反契约。"""


# ---------------------------------------------------------------------------
# 校验主入口
# ---------------------------------------------------------------------------


def validate_robot_model(model: RobotModel) -> ValidationReport:
    """全面校验一个 `RobotModel`（契约 §9）。

    刻意**不做**短路返回：即使发现 error 也继续检查其余项。
    理由是"一次看全所有问题"远比"修一个跑一次"高效，
    而且多个问题常有关联（如"少了一个 joint"往往同时导致孤立 link + 引用失败）。
    """
    report = ValidationReport(robot_id=model.metadata.id)

    _check_metadata(model, report)
    _check_coordinate(model, report)
    _check_units(model, report)
    _check_identity_uniqueness(model, report)
    _check_id_format(model, report)
    _check_references(model, report)
    _check_tree(model, report)
    _check_joints(model, report)
    _check_actuators(model, report)
    _check_frames(model, report)
    _check_sites(model, report)
    _check_end_effectors(model, report)
    _check_transforms(model, report)
    _check_geometry(model, report)
    _check_capabilities_cross(model, report)

    return report


# ---------------------------------------------------------------------------
# 各检查项
# ---------------------------------------------------------------------------


def _check_metadata(model: RobotModel, report: ValidationReport) -> None:
    md = model.metadata
    for attr in ("id", "name", "version"):
        if not isinstance(getattr(md, attr), str) or not getattr(md, attr).strip():
            report.error("metadata.empty", f"metadata.{attr}", "必须是非空字符串")
    # 机器人 id 本身也要满足 ID 契约（它出现在 WS 消息的 "robot" 字段里）
    if md.id and not ID_PATTERN.match(md.id):
        report.error(
            "metadata.id_format",
            "metadata.id",
            f"{md.id!r} 不符合 ID 契约（ASCII snake_case，正则 {ID_PATTERN.pattern}）",
        )


def _check_coordinate(model: RobotModel, report: ValidationReport) -> None:
    """坐标系必须与 P0 契约**逐字段**一致（契约 §9.1 Coordinate）。"""
    c = model.coordinate
    got = c.to_dict()
    for key, want in ROBOTFORGE_COORDINATE.items():
        if got.get(key) != want:
            report.error(
                "coordinate.mismatch",
                f"coordinate.{key}",
                f"必须是 {want!r}（P0 契约冻结），当前 = {got.get(key)!r}。"
                f"坐标系转换只允许在 Loader/Adapter 里完成",
            )

    # 三个轴必须是 x/y/z 的一个排列（防止 forward=left=x 这类矛盾声明）
    axes = (c.forward_axis, c.left_axis, c.up_axis)
    if sorted(axes) != sorted(_AXIS_CHARS):
        report.error(
            "coordinate.axes_not_permutation",
            "coordinate",
            f"forward/left/up = {axes} 不是 x/y/z 的一个排列（三轴必须互不相同）",
        )


def _check_units(model: RobotModel, report: ValidationReport) -> None:
    u = model.units
    got = u.to_dict()
    for key, want in ROBOTFORGE_UNITS.items():
        if got.get(key) != want:
            report.error(
                "units.mismatch",
                f"units.{key}",
                f"必须是 {want!r}（P0 契约冻结 SI），当前 = {got.get(key)!r}。"
                f"单位转换只允许在 Loader/Adapter/UI/Protocol 边界",
            )


def _check_identity_uniqueness(model: RobotModel, report: ValidationReport) -> None:
    """各类 id 在**各自类型内**唯一（契约 §9.1 Identity）。

    刻意**不**要求跨类型全局唯一：`base` 同时作为 link 名与 frame 名是**合法且常见**的
    （URDF/MJCF 里 link 自带同名坐标系）。要求全局唯一会强迫模型作者
    把 base frame 改名成 `base_frame`，那是我们强加的无谓约束。
    """
    groups: list[tuple[str, list[tuple[str, str]]]] = [
        ("link", [(i.id, i.name) for i in model.links]),
        ("joint", [(i.id, i.name) for i in model.joints]),
        ("actuator", [(i.id, i.name) for i in model.actuators]),
        ("frame", [(i.id, i.name) for i in model.frames]),
        ("site", [(i.id, i.name) for i in model.sites]),
        ("end_effector", [(i.id, i.name) for i in model.end_effectors]),
    ]
    for kind, items in groups:
        seen: dict[str, int] = {}
        for ident, _name in items:
            seen[ident] = seen.get(ident, 0) + 1
        for ident, count in seen.items():
            if count > 1:
                report.error(
                    "id.duplicate",
                    f"{kind}[{ident}]",
                    f"{kind} id {ident!r} 出现 {count} 次（id 必须唯一）",
                )


def _check_id_format(model: RobotModel, report: ValidationReport) -> None:
    """ID 契约：ASCII snake_case（契约 §6.1）。

    这里只查**格式**；规范化（把 `Arm 1` 变成 `arm_1`）是 Loader 的职责。
    到 Validator 这一步还出现非法字符，说明 Loader 漏了规范化。
    """
    groups: list[tuple[str, list[str]]] = [
        ("link", model.link_ids()),
        ("joint", model.joint_ids()),
        ("actuator", model.actuator_ids()),
        ("frame", model.frame_ids()),
        ("site", model.site_ids()),
        ("end_effector", model.end_effector_ids()),
    ]
    for kind, ids in groups:
        for ident in ids:
            if not ID_PATTERN.match(ident):
                report.error(
                    "id.format",
                    f"{kind}[{ident}]",
                    f"{ident!r} 不符合 ID 契约（ASCII snake_case，正则 {ID_PATTERN.pattern}）；"
                    f"规范化应由 Loader 完成",
                )


def _check_references(model: RobotModel, report: ValidationReport) -> None:
    """所有引用的 id 必须真实存在（契约 §9.1 References）。"""
    link_ids = set(model.link_ids())
    joint_ids = set(model.joint_ids())
    frame_ids = set(model.frame_ids())
    site_ids = set(model.site_ids())

    for j in model.joints:
        if j.parent_link not in link_ids:
            report.error(
                "ref.joint.parent_link",
                f"joint[{j.id}]",
                f"parent_link = {j.parent_link!r} 不存在于 links",
            )
        if j.child_link not in link_ids:
            report.error(
                "ref.joint.child_link",
                f"joint[{j.id}]",
                f"child_link = {j.child_link!r} 不存在于 links",
            )
        if j.mimic is not None and j.mimic.joint not in joint_ids:
            report.error(
                "ref.joint.mimic",
                f"joint[{j.id}]",
                f"mimic.joint = {j.mimic.joint!r} 不存在于 joints",
            )

    for link in model.links:
        if link.parent_joint is not None and link.parent_joint not in joint_ids:
            report.error(
                "ref.link.parent_joint",
                f"link[{link.id}]",
                f"parent_joint = {link.parent_joint!r} 不存在于 joints",
            )
        for cj in link.child_joints:
            if cj not in joint_ids:
                report.error(
                    "ref.link.child_joint",
                    f"link[{link.id}]",
                    f"child_joints 里的 {cj!r} 不存在于 joints",
                )

    for a in model.actuators:
        if a.joint is not None and a.joint not in joint_ids:
            report.error(
                "ref.actuator.joint",
                f"actuator[{a.id}]",
                f"joint = {a.joint!r} 不存在于 joints",
            )

    # frame.parent ∈ links ∪ frames（契约 §3.7）
    for f in model.frames:
        if f.parent not in link_ids and f.parent not in frame_ids:
            report.error(
                "ref.frame.parent",
                f"frame[{f.id}]",
                f"parent = {f.parent!r} 既不在 links 也不在 frames",
            )

    for s in model.sites:
        if s.parent not in link_ids:
            report.error(
                "ref.site.parent",
                f"site[{s.id}]",
                f"parent = {s.parent!r} 不存在于 links",
            )

    for ee in model.end_effectors:
        if ee.frame not in frame_ids:
            report.error(
                "ref.end_effector.frame",
                f"end_effector[{ee.id}]",
                f"frame = {ee.frame!r} 不存在于 frames",
            )
        if ee.site is not None and ee.site not in site_ids:
            report.error(
                "ref.end_effector.site",
                f"end_effector[{ee.id}]",
                f"site = {ee.site!r} 不存在于 sites",
            )


def _check_tree(model: RobotModel, report: ValidationReport) -> None:
    """Link-Joint Tree 契约（契约 §7）。"""
    link_ids = set(model.link_ids())
    joint_ids = set(model.joint_ids())

    # ① root_link 存在
    if not model.root_link:
        report.error("tree.root_link_empty", "root_link", "必须声明 root_link")
    elif model.root_link not in link_ids:
        report.error(
            "tree.root_link_missing",
            "root_link",
            f"root_link = {model.root_link!r} 不存在于 links（可用: {sorted(link_ids)}）",
        )

    # ② 恰好一个 Link 的 parent_joint is None
    roots = [link.id for link in model.links if link.is_root()]
    if len(roots) > 1:
        report.error(
            "tree.multiple_roots",
            "links",
            f"有 {len(roots)} 个 Link 的 parent_joint 为 None（{roots}）；"
            f"v0.1 是单棵树，不支持森林",
        )
    if model.root_link and roots and model.root_link not in roots:
        # ⚠️ 不能用 model.link(...)：root_link 可能**根本不存在**（上面 ① 已报
        # tree.root_link_missing）。那种情况下这里会抛 KeyError 而不是报 issue ——
        # 而 validator 的契约是"**任何**输入都返回报告，绝不抛异常"：
        # 它存在的意义就是把错误变成可读的报告，它自己崩掉就完全失去了意义。
        target = next((l for l in model.links if l.id == model.root_link), None)
        if target is not None:
            report.error(
                "tree.root_link_not_root",
                "root_link",
                f"root_link = {model.root_link!r} 但它的 parent_joint 不为 None"
                f"（parent_joint = {target.parent_joint!r}）",
            )

    # ③ 多父检查：每个非根 Link 恰好被一个 Joint 当作 child_link
    child_refs: dict[str, list[str]] = {}
    for j in model.joints:
        child_refs.setdefault(j.child_link, []).append(j.id)
    for link_id, jids in child_refs.items():
        if len(jids) > 1:
            report.error(
                "tree.multi_parent",
                f"link[{link_id}]",
                f"被多个 Joint 作为 child_link：{jids}（v0.1 禁止多父）",
            )

    # ④ 孤立 Link：既不是 root，也没有任何 Joint 指向它
    root_set = set(roots)
    for link in model.links:
        if link.id in root_set:
            continue
        if link.id not in child_refs:
            report.error(
                "tree.orphan_link",
                f"link[{link.id}]",
                "不属于任何 Joint（既非 root 也无 Joint 以它为 child）⇒ 孤立 Link",
            )

    # ⑤ 循环检测：从 root 做 DFS，走到已访问节点即环
    _check_no_cycle(model, report, link_ids, joint_ids)

    # ⑥ base_frame ∈ frames
    if not model.base_frame:
        report.error("tree.base_frame_empty", "base_frame", "必须声明 base_frame")
    elif not model.has_frame(model.base_frame):
        report.error(
            "tree.base_frame_missing",
            "base_frame",
            f"base_frame = {model.base_frame!r} 不存在于 frames（可用: {model.frame_ids()}）",
        )


def _check_no_cycle(
    model: RobotModel,
    report: ValidationReport,
    link_ids: set[str],
    joint_ids: set[str],
) -> None:
    """用"沿 parent 链上溯"的方式检测环。

    为什么用上溯而不是 DFS：环的**唯一表现**是某条 parent 链走不到根。
    上溯实现更短，且错误信息能直接给出"环上的节点序列"
    （DFS 只能告诉你"访问过一个已访问节点"，定位要多一步）。
    """
    if not model.root_link or model.root_link not in link_ids:
        return  # 根不存在，上溯无意义（已在上面的检查里报错）

    for start in model.links:
        path = [start.id]
        cur = start
        # 步数上限 = link 数，超过必有环（避免死循环）
        for _ in range(len(model.links) + 1):
            if cur.parent_joint is None:
                break  # 到根了，无环
            if cur.parent_joint not in joint_ids:
                break  # 引用错误已单独报过
            parent_joint = model.joint(cur.parent_joint)
            if parent_joint.parent_link not in link_ids:
                break
            nxt = model.link(parent_joint.parent_link)
            if nxt.id in path:
                report.error(
                    "tree.cycle",
                    f"link[{start.id}]",
                    f"从 {start.id!r} 上溯出现循环：{' → '.join(path + [nxt.id])}",
                )
                break
            path.append(nxt.id)
            cur = nxt


def _check_joints(model: RobotModel, report: ValidationReport) -> None:
    """Joint 契约：类型、axis 归一化、limits 合法性（契约 §9.1 Joint）。"""
    for j in model.joints:
        if j.type not in JOINT_TYPES:
            report.error(
                "joint.type",
                f"joint[{j.id}]",
                f"type = {j.type!r} 不合法；v0.1 允许 {' / '.join(JOINT_TYPES)}",
            )
            continue

        # axis 归一化（对所有类型都查 —— 让"axis 一定是单位向量"这条不变量无例外）
        if not j.axis.is_normalized(TOL_EXTERNAL):
            report.error(
                "joint.axis_not_normalized",
                f"joint[{j.id}]",
                f"axis = {j.axis} 的模 = {j.axis.norm():.9f}，必须 ≈ 1",
            )

        if j.type in ("revolute", "prismatic") and j.axis.norm() < 1e-6:
            report.error(
                "joint.axis_zero",
                f"joint[{j.id}]",
                "可动关节的 axis 不能是零向量（FK 里等价于无旋转）",
            )

        if j.limits is not None:
            lim = j.limits
            if lim.position_min is not None and lim.position_max is not None:
                if lim.position_min > lim.position_max:
                    report.error(
                        "joint.limits_inverted",
                        f"joint[{j.id}]",
                        f"position_min ({lim.position_min}) > position_max ({lim.position_max})",
                    )
            if lim.velocity_max is not None and lim.velocity_max <= 0:
                report.error(
                    "joint.limits_velocity",
                    f"joint[{j.id}]",
                    f"velocity_max = {lim.velocity_max} 必须 > 0",
                )
            if lim.effort_max is not None and lim.effort_max <= 0:
                report.error(
                    "joint.limits_effort",
                    f"joint[{j.id}]",
                    f"effort_max = {lim.effort_max} 必须 > 0",
                )

        # 可动关节缺少限位 ⇒ warn（不是 error）
        # 理由：MJCF 允许 unlimited joint（如连续旋转的云台），因此它不违法；
        # 但真实机械臂的关节几乎总有物理限位，缺它通常是建模疏漏 ⇒ 值得人看一眼。
        if j.is_mobile() and (j.limits is None or not j.limits.has_position_bounds()):
            report.warn(
                "joint.no_position_limit",
                f"joint[{j.id}]",
                "可动关节未声明完整位置限位 ⇒ IK 解空间可能无界（若非刻意，请补上）",
            )

        # mimic 在 v0.1 不实现 ⇒ warn。**不忽略**它，因为忽略会让
        # "模型有耦合但 FK 不知道"变成 MuJoCo 与 FK 结果不一致的静默 bug。
        if j.mimic is not None:
            report.warn(
                "joint.mimic_not_implemented",
                f"joint[{j.id}]",
                f"声明了 mimic（跟随 {j.mimic.joint!r}）；v0.1 的 FK/IK **不执行**耦合，"
                f"MuJoCo 会执行 ⇒ 两者结果会不同",
            )


def _check_actuators(model: RobotModel, report: ValidationReport) -> None:
    for a in model.actuators:
        if a.type not in ACTUATOR_TYPES:
            report.error(
                "actuator.type",
                f"actuator[{a.id}]",
                f"type = {a.type!r} 不合法；允许 {' / '.join(ACTUATOR_TYPES)}",
            )
        if a.command_min is not None and a.command_max is not None:
            if a.command_min > a.command_max:
                report.error(
                    "actuator.command_range_inverted",
                    f"actuator[{a.id}]",
                    f"command_min ({a.command_min}) > command_max ({a.command_max})",
                )


def _check_frames(model: RobotModel, report: ValidationReport) -> None:
    """frame 之间不能有循环（base → tool → base）。

    ## `parent == self.id` 是**合法的根 frame 标记**

    `world` frame 是宇宙的根，它没有父。契约允许两种写法表达"这是根"：
    ① `parent` 指向自己（`world → world`）；
    ② `parent` 指向一个不存在的名字（会触发 `ref.frame.parent` 错误 ⇒ 不可用）。

    因此这里对 `parent == id` **显式豁免** —— 否则每个模型都会被迫给 world
    编造一个假父。这是一个真实的踩坑记录：第一版校验器把 `world → world`
    判成循环，于是**任何**模型都过不了校验。
    """
    frame_ids = set(model.frame_ids())
    for f in model.frames:
        if f.parent == f.id:
            continue  # 根 frame 的自引用，合法
        path = [f.id]
        cur = f
        for _ in range(len(model.frames) + 1):
            if cur.parent not in frame_ids:
                break  # 父是 link 或引用错误（已单独报过）
            if cur.parent == cur.id:
                break  # 上溯到根 frame
            nxt = model.frame(cur.parent)
            if nxt.id in path:
                report.error(
                    "frame.cycle",
                    f"frame[{f.id}]",
                    f"frame parent 链出现循环：{' → '.join(path + [nxt.id])}",
                )
                break
            path.append(nxt.id)
            cur = nxt


def _check_sites(model: RobotModel, report: ValidationReport) -> None:
    """site 的 role 不做白名单校验。

    理由：`role` 是**自由文本语义标记**（如 `end_effector` / `camera` / `tcp`）。
    把它限定成枚举会让"加一个 sensor site"需要改契约 —— 而它并不影响正确性。
    `end_effector` 的绑定通过 `EndEffector.site` 的**引用**完成，那才有校验。
    """


def _check_end_effectors(model: RobotModel, report: ValidationReport) -> None:
    for ee in model.end_effectors:
        if ee.frame == "":
            report.error("end_effector.frame_empty", f"end_effector[{ee.id}]", "frame 不能为空")
        # site 为空时用 frame 的位姿 ⇒ 合法，不报错


def _check_transforms(model: RobotModel, report: ValidationReport) -> None:
    """transform 契约：四元数归一化 + 无 NaN（契约 §9.1 Transform）。"""
    def check_transform(t: Transform | None, where: str) -> None:
        if t is None:
            return
        if not t.orientation.is_normalized(1e-6):
            report.error(
                "transform.quaternion_not_normalized",
                where,
                f"quaternion = {t.orientation} 的模 = {t.orientation.norm():.9f}，必须 ≈ 1",
            )
        # NaN 在 types.py 的构造期已被拒（Vector3/Quaternion 都会抛），
        # 这里再查一次是为了捕获"绕过构造函数的拼接"（如 dataclass replace 误用）
        for label, vec in (("position", t.position),):
            if not vec.is_finite():
                report.error("transform.nan", where, f"{label} 含非有限值")

    for j in model.joints:
        check_transform(j.origin, f"joint[{j.id}].origin")
    for f in model.frames:
        check_transform(f.transform, f"frame[{f.id}].transform")
    for s in model.sites:
        check_transform(s.transform, f"site[{s.id}].transform")
    for link in model.links:
        for idx, g in enumerate(link.visual):
            check_transform(g.transform, f"link[{link.id}].visual[{idx}]")
        for idx, g in enumerate(link.collision):
            check_transform(g.transform, f"link[{link.id}].collision[{idx}]")


def _check_geometry(model: RobotModel, report: ValidationReport) -> None:
    """几何与惯量（契约 §3.8 / §3.9）。"""
    for link in model.links:
        for tag, geoms in (("visual", link.visual), ("collision", link.collision)):
            for idx, g in enumerate(geoms):
                where = f"link[{link.id}].{tag}[{idx}]"
                if g.type not in GEOMETRY_TYPES:
                    report.error(
                        "geometry.type",
                        where,
                        f"type = {g.type!r} 不合法；v0.1 允许 {' / '.join(GEOMETRY_TYPES)}",
                    )
                    continue
                expected = {
                    "box": 3,
                    "sphere": 1,
                    "cylinder": 2,
                    "capsule": 2,
                    # mesh 的 size 不承载语义（用 asset 引用），允许为空
                    "mesh": None,
                }[g.type]
                if expected is not None and len(g.size) != expected:
                    report.error(
                        "geometry.size",
                        where,
                        f"type={g.type} 需要 {expected} 个 size 分量，当前 = {g.size}",
                    )
                for v in g.size:
                    if v <= 0:
                        report.error(
                            "geometry.size_nonpositive",
                            where,
                            f"size 分量必须 > 0（当前 = {g.size}）；"
                            f"0 或负数几何在渲染与碰撞里都无意义",
                        )
                if g.type == "mesh" and not g.asset:
                    report.error(
                        "geometry.mesh_no_asset",
                        where,
                        "mesh 类型必须声明 asset",
                    )
                if g.rgba is not None and len(g.rgba) not in (3, 4):
                    report.error(
                        "geometry.rgba",
                        where,
                        f"rgba 必须是 3 或 4 个分量（当前 = {g.rgba}）",
                    )

        if link.inertial is not None:
            if link.inertial.mass < 0:
                report.error(
                    "inertial.mass",
                    f"link[{link.id}].inertial",
                    f"mass = {link.inertial.mass} 不能为负",
                )
            if link.inertial.mass == 0:
                report.warn(
                    "inertial.zero_mass",
                    f"link[{link.id}].inertial",
                    "mass = 0 ⇒ 在 MuJoCo 里该刚体无动力学意义（通常应为极小正数或省略 inertial）",
                )


def _check_capabilities_cross(model: RobotModel, report: ValidationReport) -> None:
    """capability 的**声明**与模型的**结构事实**交叉检查（契约 §3.10）。

    capability 是声明，不是保证。这里只查"声明与自己矛盾"的情况：

    - `ik: true` 但无 end_effector ⇒ IK 的目标点无处可定义
    - `end_effector: true` 但无 end_effector ⇒ 声明与结构不符
    - `actuator_control: true` 但无 actuator ⇒ 没有可控对象
    - `fk: true` 但 dof == 0 ⇒ 没有自由度，FK 恒为单位位姿

    真正的"能不能解"由 FK/IK 的实际测试决定，不是这里判定的。
    """
    caps = model.capabilities
    if caps.ik and not model.end_effectors:
        report.warn(
            "capabilities.ik_without_ee",
            "capabilities.ik",
            "声明 ik=true 但模型没有 end_effector ⇒ 逆解的目标位姿没有明确的语义落点",
        )
    if caps.end_effector and not model.end_effectors:
        report.error(
            "capabilities.ee_mismatch",
            "capabilities.end_effector",
            "声明 end_effector=true 但 end_effectors 为空（声明与结构不符）",
        )
    if caps.actuator_control and not model.actuators:
        report.error(
            "capabilities.actuator_mismatch",
            "capabilities.actuator_control",
            "声明 actuator_control=true 但 actuators 为空（声明与结构不符）",
        )
    if caps.fk and model.dof() == 0:
        report.warn(
            "capabilities.fk_no_dof",
            "capabilities.fk",
            "声明 fk=true 但模型没有可动关节（dof=0）⇒ FK 恒为单位位姿",
        )


# ---------------------------------------------------------------------------
# 便捷函数
# ---------------------------------------------------------------------------


def assert_valid(model: RobotModel) -> RobotModel:
    """校验并返回模型（链式用）。失败抛 `RobotModelError`。"""
    validate_robot_model(model).raise_if_invalid()
    return model


def summarize(report: ValidationReport) -> dict[str, Any]:
    """给 API/前端用的结构化摘要。"""
    return {
        "robot": report.robot_id,
        "ok": report.ok,
        "error_count": len(report.errors),
        "warn_count": len(report.warnings),
        "issues": [
            {"level": i.level, "code": i.code, "where": i.where, "message": i.message}
            for i in report.issues
        ],
    }


__all__ = [
    "Issue",
    "RobotModelError",
    "ValidationReport",
    "assert_valid",
    "summarize",
    "validate_robot_model",
    "ID_PATTERN",
    "Quaternion",
    "Transform",
    "Vector3",
]
