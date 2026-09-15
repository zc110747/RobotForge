# RobotForge

RobotForge 是一个面向机器人开发的模型构建与生成平台，旨在将机械臂的 CAD/STEP 等结构数据转换为标准化的机器人模型，并自动生成仿真、运动学、执行器映射及能力描述等工程资源。

> **v0.1 的定位**：建立一个基于**原生 MJCF** Robot Package、
> **统一 RobotModel**、**统一坐标系**和**统一单位**的 Web 机器人
> 数字孪生与 Sim2Sim 平台。

---

## 1. 核心链路

```text
packages/<robot>/manifest.yaml
        │
        ▼
   MJCF Loader          （§8，Core）
        │
        ▼
   RobotModel           （§9-§27，唯一规范内部表示）
        │
   ┌────┴────┐
   ▼         ▼
  FK        IK          （§53 / §54；FK 在 Core，IK 在包内）
   └────┬────┘
        ▼
  RobotRuntime          （§46）
        │
   RobotCommand         （§28，Desired）
        ▼
   RobotBackend         （§47；v0.1 = MockBackend，Phase 5 = MuJoCoBackend）
        │
   RobotState           （§29，Actual）
        ▼
   WebSocket            （§52）
        ▼
  TypeScript / Three.js （§51）
```

## 2. 目录

```text
backend/
├── api/           REST（registry / app）+ WebSocket 协议（websocket.py）
├── cli.py         python -m backend.cli {list,show,fk,ik,inspect}
├── kinematics/    Core 通用链式 FK（fk.py）
├── loaders/       loader.py（抽象）+ registry.py（分派点）+ mjcf_loader.py
│                  + asset_loader.py（Geometry Asset 扩展缝，§65）
├── model/         RobotModel + Validator（§9-§36）
├── runtime/       RobotCommand / RobotState / RobotBackend / RobotRuntime（§46-§47）
├── simulation/    SimulationBackend 抽象 + MuJoCoBackend（§60-§63）
└── transport/     （§48，v0.1 只预留）

packages/mini_arm/
├── manifest.yaml        身份 / 能力 / 指针（唯一真值源）
├── model/mini_arm.xml   原生 MJCF
├── kinematics/          包内 fk.py（解析闭式）与 ik.py（2R 解析解）
└── tests/               包内构造事实

frontend/src/
├── viewer/              Three.js 渲染 + coordinateAdapter.ts（唯一坐标转换点）
└── ...
docs/
├── 1.protometer.md      项目 spec（76 节）
├── robot-model.md       RobotModel 契约
├── coordinate-system.md 坐标系与单位契约
└── runtime.md           Command / State / Backend / WebSocket 契约
tools/                   accept_phaseN.py 验收清单
tests/                   自动化测试（§67）
```

## 3. 快速开始

```bash
# 环境（独立 venv，不动全局）
python -m venv .venv
.venv/Scripts/python.exe -m pip install -r requirements.txt

# 列出机器人
.venv/Scripts/python.exe -m backend.cli list

# 查看模型摘要 / 完整契约
.venv/Scripts/python.exe -m backend.cli show mini_arm
.venv/Scripts/python.exe -m backend.cli inspect mini_arm

# 正解 / 逆解
.venv/Scripts/python.exe -m backend.cli fk mini_arm --joints shoulder=0.3
.venv/Scripts/python.exe -m backend.cli ik mini_arm --target 0.12,0,0.136

# 服务端（REST + WebSocket）
.venv/Scripts/python.exe -m uvicorn backend.api.app:create_app --factory --reload
```

## 4. 验收

每个 Phase 有独立的可执行验收清单。**当前 Phase 未通过：不得进入下一 Phase**（§70）。

```bash
.venv/Scripts/python.exe tools/accept_phase1.py   # MJCF → RobotModel
.venv/Scripts/python.exe tools/accept_phase2.py   # Three.js 渲染
.venv/Scripts/python.exe tools/accept_phase3.py   # FK / IK 往返
.venv/Scripts/python.exe tools/accept_phase4.py   # Runtime + WebSocket
.venv/Scripts/python.exe tools/accept_phase5.py   # MuJoCo Sim2Sim
.venv/Scripts/python.exe tools/accept_phase7.py   # URDF 扩展点（不实现 Parser）
.venv/Scripts/python.exe tools/accept_phase8.py   # Geometry Asset 扩展点
```

> ⚠️ Phase 3 及之后的清单会跑全量 pytest + 上游 Phase，耗时数分钟。
> 在本机沙箱下**必须后台执行并落文件**（前台会被 SIGTERM，产出 0 字节日志）：
>
> ```bash
> .venv/Scripts/python.exe -u tools/accept_phase8.py > accept8.log 2>&1
> ```

### 当前基线（2026-09-15，Phase 8 收口）

```text
pytest -q                        → 488 passed
pytest packages/mini_arm/tests   → 34 passed
tools/accept_phase1.py           → 32/32 ✅
tools/accept_phase2.py           → 56/56 ✅
tools/accept_phase3.py           → 47/47 ✅
tools/accept_phase4.py           → 65/65 ✅
tools/accept_phase5.py           → 67/67 ✅
tools/accept_phase7.py           → 50/50 ✅
tools/accept_phase8.py           → 96/96 ✅
```

| Phase | 内容 | 状态 |
|---|---|---|
| 0 | 项目初始化 | ✅ |
| 1 | MJCF + RobotModel | ✅ 32/32 |
| 2 | Three.js | ✅ 56/56 |
| 3 | FK / IK | ✅ 47/47 |
| 4 | Runtime + WebSocket | ✅ 65/65 |
| 5 | MuJoCo Sim2Sim | ✅ 67/67 |
| 6 | （spec 无 Phase 6） | — |
| 7 | URDF 扩展点验证 | ✅ 50/50 |
| 8 | Geometry Asset 扩展点验证 | ✅ 96/96 |

### 扩展点：怎么加一个新格式（Phase 7 / 8 的用途）

```python
# ① 新**模型**格式（§43 / §64）：写一个 RobotModelLoader 子类并注册
from backend.loaders import RobotModelLoader, register_loader

class URDFLoader(RobotModelLoader):
    format_name = "urdf"
    def load(self, source): ...          # 返回 (RobotModel, LoaderReport)

register_loader(URDFLoader())            # 之后 manifest 写 format: urdf 即可
```

Runtime **一行不用改** —— 它只认 `RobotModel`（Phase 7 验收以此为准）。

```python
# ② 新**几何资源**格式（§44 / §65）：写一个 AssetLoader 子类并注册
from backend.loaders import AssetLoader, register_asset_loader

class STLLoader(AssetLoader):
    extensions = ("stl",)
    def load(self, source, *, name=""): ...   # 返回 GeometryAsset

register_asset_loader(STLLoader())
```

v0.1 的默认资产注册表**故意是空的**（不实现任何 mesh parser）。
原生几何（box/cylinder/…）的 `asset` 为 `None`，`resolve()` 走 `None` 快路径，
**不会抛错** —— 缺 asset 不得阻止机器人启动（§69）。

## 5. 三条不可违反的规则

1. **RobotModel 是唯一规范内部表示。** Runtime 里出现 `MjModel` / `Object3D` /
   XML 元素 ⇒ 架构已破（§9）。
2. **Core 与 Frontend 不得出现机器人型号分支**（§69 规则 1/2）。
   加一台机器人 = 加一个目录 + `manifest.yaml`，Core 一行不动。
   前端据 `RobotCapabilities` 动态决定显示哪些功能。
3. **不得为 Three.js 在 FK / IK 里修改坐标**（§41）。
   唯一转换点是 `frontend/src/viewer/coordinateAdapter.ts`。

## 6. 已知限制（v0.1）

* 只支持**原生 MJCF**（§7）；URDF **只验证扩展点存在**，不实现 Parser（Phase 7）。
* Geometry Asset **只验证扩展缝存在**，不实现 STL/OBJ/GLTF/STEP Parser（Phase 8）。
  默认资产注册表为空是**设计如此**，不是功能缺失。
* 只实现 Sim2Sim；Sim2Real 只预留接口（§63）。
* `Transport`（Serial / CAN / USB / TCP）只预留抽象，**不实现**（§48）。
* IK 是**包内**实现：闭式解依赖机构，通用 IK 会退化为数值迭代
  ⇒ 引入容差 ⇒ 削弱"精确往返"验收（见 `docs/runtime.md` §1 与 Phase 3 结论）。
* `packages/` 下**刻意没有 `__init__.py`**：机器人包是数据 + 算法的容器，
  不是可 import 的 Python 包。包内模块按**绝对路径**加载。

## 7. 关于"扩展点验证"的判据（Phase 7 / 8 的方法论）

"抽象存在"与"抽象被**用作分派点**"是两件不同的事。`grep` 只能证明前者，
而前者在 Phase 1 就已满足 —— 所以单靠源码扫描的验收是**自证式**的。

本项目的做法是**改变状态后再观察**：

1. **注入一个合成实现**（`toyfmt` / `toymesh`，与 MJCF/STL 毫无关系），
   注册后让**同一段调用代码**产生**不同的可见结果**（调用计数 +1、抛错变为返回值）。
2. **反向自检**：复现"改之前"的硬编码行为，断言它**必须失败**。
   若旧代码也照样通过，说明第 1 条测的不是分派点。
3. **扫描器元测试**：任何"源码里不得出现 X"的判据，都要先证明扫描器
   **抓得到** X —— 否则"没找到"与"扫描器坏了"无法区分。

Phase 7 的 `registry.py` 与 Phase 8 的 `asset_loader.py` 里，
格式名**只以字符串字面量禁止**为判据（不能用"剥离注释与字符串"那一套 ——
在扩展点场景里，字符串**就是**要查的目标；而 `from .mjcf_loader import` 里的
`mjcf` 是**模块名**，不是硬编码格式名）。
