"""把 RobotModel + **Python 侧独立推导的渲染事实**导出为 JSON。

## 为什么需要"独立推导"

§59 的最后一条验收是 "RobotModel 与 Renderer 语义一致"。
要验证它与"渲染器"一致，必须有两份**由不同代码算出**的事实来比对：

```text
Python 侧（本文件）                  TS 侧（viewModel.ts）
  model.to_dict()          ←真值→     buildViewModel()
  自己按树递归推导层级                  自己按树递归推导层级
  自己数可动关节                        自己数可动关节
  ────────────────────────────  比对  ────────────────────────────
```

如果 TS 侧只是把 Python 的输出原样打印，测试永远绿而什么都没验证。
所以本文件**不**调用 `viewModel.ts`，而是用 Python 重新算一遍 ——
两套独立实现给出同一结论，才叫"语义一致"。

## 输出

写入 `frontend/src/viewer/__tests__/fixtures/model.json`：

```json
{
  "model": {...},                 // RobotModel.to_dict() 原样
  "expect": {
     "linkCount": 5,
     "jointCount": 4,
     "movableJointCount": 3,
     "hierarchy": [{"id":"base","parent":"world","viaJoint":null}, ...],
     "axes": {"base_yaw":[0,0,1], ...},
     "endEffectors": [{"id":"main_ee","link":"ee_link","position":[0.032,0,0]}],
     "frames": [{"id":"world","parent":"world"}, ...],
     "movableJointIds": ["base_yaw","shoulder","elbow"]
  }
}
```
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from backend.api.registry import get_package  # noqa: E402
from backend.model.robot_model import RobotModel  # noqa: E402

FIXTURE = ROOT / "frontend" / "src" / "viewer" / "__tests__" / "fixtures" / "model.json"


def derive_expectations(model: RobotModel) -> dict[str, Any]:
    """**独立**推导渲染事实（不复用 viewModel.ts 的任何逻辑）。

    刻意用最朴素的方式：字典、显式循环。
    越"聪明"的写法越可能与 TS 侧共享同一个思维错误。

    ## 渲染树 = link 与 joint **交替**

    ```text
    world
     └─ base(link) ─ base_yaw(joint) ─ shoulder_link(link) ─ ...
    ```
    - link 的父 = 它的 `parent_joint`（根 link 的父 = world）
    - joint 的父 = 它的 `parent_link`
    """
    # --- 节点表（key → parentKey），两套独立推导 ---------------------------------
    # link 节点
    link_parent: dict[str, str] = {}
    for link in model.links:
        link_parent[link.id] = (
            "world" if link.parent_joint is None else "joint:" + link.parent_joint
        )
    # joint 节点
    joint_parent: dict[str, str] = {}
    for j in model.joints:
        joint_parent[j.id] = "link:" + j.parent_link

    # --- 深度优先顺序（父先于子）------------------------------------------------
    # 自己实现，不用图库
    children: dict[str, list[str]] = {}

    def add_child(parent_key: str, child_key: str) -> None:
        children.setdefault(parent_key, []).append(child_key)

    for link in model.links:
        add_child(link_parent[link.id], "link:" + link.id)
    for j in model.joints:
        add_child(joint_parent[j.id], "joint:" + j.id)

    order: list[str] = []

    def visit(key: str, depth: int) -> None:
        if depth > 1000 or key in order:
            return
        order.append(key)
        for c in children.get(key, []):
            visit(c, depth + 1)

    visit("world", 0)

    # --- 关节 --------------------------------------------------------------
    axes = {j.id: [float(x) for x in j.axis.to_list()] for j in model.joints}
    movable = [j.id for j in model.joints if j.type != "fixed"]
    joint_origins = {
        j.id: [float(x) for x in j.origin.position.to_list()] for j in model.joints
    }

    # --- 末端执行器 ---------------------------------------------------------
    frame_by_id = {f.id: f for f in model.frames}
    ees: list[dict[str, Any]] = []
    for ee in model.end_effectors:
        frame = frame_by_id.get(ee.frame)
        if frame is None:
            continue
        ees.append(
            {
                "id": ee.id,
                "link": frame.parent,
                # frame.transform.position（frame-local）
                "position": [float(x) for x in frame.transform.position.to_list()],
                "site": ee.site,
            }
        )

    # --- 坐标架 ------------------------------------------------------------
    frames = [{"id": f.id, "parent": f.parent} for f in model.frames]

    # --- 几何（每个 link 的碰撞体，v0.1 用于显示）--------------------------
    geometry = {
        l.id: [
            {
                "type": g.type,
                "size": [float(x) for x in g.size],
                "position": [float(x) for x in g.transform.position.to_list()],
            }
            for g in l.collision
        ]
        for l in model.links
    }

    return {
        "linkCount": len(model.links),
        "jointCount": len(model.joints),
        "movableJointCount": len(movable),
        "movableJointIds": movable,
        "linkParent": link_parent,
        "jointParent": joint_parent,
        "depthFirstOrder": order,
        "axes": axes,
        "jointOrigins": joint_origins,
        "endEffectors": ees,
        "frames": frames,
        "geometry": geometry,
        "rootLink": model.root_link,
        "baseFrame": model.base_frame,
        "dof": model.dof(),
    }


def build_fixture(robot_id: str = "mini_arm") -> dict[str, Any]:
    pkg = get_package(robot_id)
    model = pkg.load_model()[0]
    return {"robotId": robot_id, "model": model.to_dict(), "expect": derive_expectations(model)}


def main() -> int:
    payload = build_fixture()
    FIXTURE.parent.mkdir(parents=True, exist_ok=True)
    FIXTURE.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    exp = payload["expect"]
    print(f"已写入 {FIXTURE}")
    print(
        f"  links={exp['linkCount']} joints={exp['jointCount']} "
        f"movable={exp['movableJointCount']} dof={exp['dof']}"
    )
    print(f"  可动关节: {exp['movableJointIds']}")
    print(f"  深度优先: {exp['depthFirstOrder']}")
    print(f"  末端执行器: {[e['id'] + '@' + e['link'] for e in exp['endEffectors']]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
