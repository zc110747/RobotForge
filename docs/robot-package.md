# RobotForge Robot Package (v0.1)

> **状态：v0.1 已冻结**
>
> 本文件定义 Robot Package 的**目录约定、清单契约、以及动态加载规则**。
> 改这里等于改"怎么加一个机器人"，请连同 `tests/test_manifest.py` 一起改。

---

## 1. 一个 Robot Package 是什么

**一个目录**，自包含地描述一台机器人。它可以被复制、被版本控制、被独立测试，
而不需要改动 Core 的任何一行代码。

```text
packages/<robot_id>/
├── manifest.yaml            身份 / 能力 / 指针 —— **唯一真值源**
├── model/
│   └── <robot>.xml          原生 MJCF
├── kinematics/
│   ├── fk.py                解析闭式 FK（机构特定）
│   └── ik.py                解析闭式 IK（机构特定）
└── tests/
    └── test_*.py            这台机器人的**构造事实**
```

---

## 2. 为什么 `packages/` 下没有 `__init__.py`

**这是刻意的，别加回来。**

```text
packages/
├── mini_arm/          ← 没有 __init__.py
├── (future_arm)/      ← 没有 __init__.py
```

理由：加了 `__init__.py` 之后 `packages` 就成了一个 Python 包，
于是 `from packages.mini_arm.kinematics import ik` 会**看起来**能工作。
一旦有人这么写，就产生了**跨机器人包的隐式依赖**：
`mini_arm` 的实现引用了 `future_arm` 里的东西，而后者完全不知道自己被依赖。

这类 bug 的表现是"删掉 A 之后 B 坏了"，而 B 与 A 毫无业务关系。

**正确做法**：包与包之间只能通过 Core 的契约（`RobotModel`）交流，
不通过 Python 的 import 机制。

---

## 3. 包内模块怎么被加载

因为不能 `import packages.mini_arm.kinematics.ik`，Core 用
**绝对文件路径**动态加载（`backend/api/registry.py` 与 `backend/cli.py`）：

```python
spec = importlib.util.spec_from_file_location(module_name, abs_path)
mod = importlib.util.module_from_spec(spec)
sys.modules[module_name] = mod          # ← 这一行**必须**有
spec.loader.exec_module(mod)
```

> ⚠️ **`sys.modules[name] = mod` 不是可选的。**
> Python 3.13 的 `@dataclass` 在生成 `__eq__` / `__repr__` 时会**反查模块**
> （通过 `sys.modules[cls.__module__]` 取命名空间）。
> 不注册的话会报 `AttributeError: 'NoneType' object has no attribute '__dict__'`
> 或 `_DataclassParams` 相关错误，而错误信息完全不提"模块没注册"。

---

## 4. `manifest.yaml` 契约（§37）

`manifest.yaml` 是**唯一真值源** —— 身份、能力、指针都在这里，
Core 的代码里**不得**出现机器人型号名（§69 规则 2）。

### 4.1 完整示例

```yaml
robotforge:
  manifest_version: "1"
  id: mini_arm                    # ASCII / snake_case / 全局唯一（§30）
  name: "Mini Arm"                # 展示名，可与 id 不同（§31）
  description: "2R 平面机械臂 + 旋转基座，教学用最小可验证模型"
  version: "0.1.0"

coordinate:
  convention: robotforge          # 固定值（§38）
  handedness: right
  forward_axis: x                 # +X = Forward
  left_axis: y                    # +Y = Left
  up_axis: z                      # +Z = Up

units:
  length: m
  angle: rad
  time: s

model:
  format: mjcf                    # ← 决定用哪个 Loader（§8）
  path: model/mini_arm.xml        # 相对 manifest 所在目录

kinematics:
  fk:
    type: core                    # FK 由 Core 提供（§53）
  ik:
    type: package                 # IK 留包内（§54）—— 见 docs/architecture.md §6
    entry: kinematics/ik.py:solve_ik

capabilities:
  fk: true
  ik: true
  simulation: true
  collision: false                # ← 从未声明过 collision，这是**有意**的
  assets: false
```

### 4.2 字段语义

| 字段 | 必填 | 说明 |
|---|---|---|
| `robotforge.manifest_version` | ✅ | 清单格式版本，当前 `"1"` |
| `robotforge.id` | ✅ | ASCII / snake_case / 唯一；**跨版本稳定** |
| `robotforge.name` | ✅ | 展示名；ID 与 Name 分离（§31） |
| `model.format` | ✅ | Loader 的分派键，必须与某 Loader 的 `format_name` 一致 |
| `model.path` | ✅ | **相对 manifest 目录**的路径，不是相对 CWD |
| `kinematics.<tag>.type` | ✅ | `core` \| `package` |
| `kinematics.<tag>.entry` | `type=package` 时必需 | `相对路径:函数名` |
| `capabilities.*` | ✅ | 声明这台机器人**支持什么** |

### 4.3 两个容易搞错的地方

**① `model.path` 是相对 manifest 目录，不是相对 CWD。**
理由：以 CWD 为基准的话，`pytest` 从仓库根跑与从 `packages/` 跑会得到
不同结果 —— 这是最难查的一类环境相关问题。

**② `capabilities.collision: false` 不是"暂时没实现"，而是事实陈述。**
mini_arm 的 MJCF 里有 8 对 `<contact><exclude>` 排除自碰撞，
因为它的视觉几何是**为好看选的**、从未做过干涉检查。
声明 `collision: true` 会让人以为"仿真里的接触是可信的"。
详见 `docs/simulation.md`。

---

## 5. 包内 `kinematics/` 的边界

```text
fk.py            ✅ 包内可以放，但必须**重导出** Core 的实现
ik.py            ✅ 包内实现（闭式解依赖机构）
kinematics.js    ✅ 包内实现（闭式解的 **JS 移植**，供前端预演）—— 见 §5.3
```

### 5.1 为什么 `fk.py` 是"解析闭式解 + Core 重导出"两份

```python
# packages/mini_arm/kinematics/fk.py

BASE_HEIGHT     = 0.084
SHOULDER_OFFSET = 0.052
L1, L2, L_TOOL  = 0.103, 0.065, 0.032

def forward_kinematics(model, positions):
    """解析闭式解 —— 几何用**文件内写死的常量**，只从 model 取旋转轴。"""
    ...

# Core 的重导出（**同一个函数对象**，有 `is` 断言守着）
from backend.kinematics.fk import forward_kinematics as forward_kinematics_generic
```

**为什么包内还需要解析解？** 它是一份**独立实现的裁判**。
Core 的 FK 是通用链乘；包内的解析解是手推的三角公式。
两者在同一个输入上给出相同结果，才说明 Core 的链乘没写错。
如果包内直接调 Core，那就变成自证。

> ⚠️ **代价（必须记住）**：解析解的几何是**写死的常量**，
> 所以**改 model 的几何时，解析 FK 的输出不变**。
> 做几何扰动自检时必须用 Core 的 `forward_kinematics_generic`，
> 否则"扰动没生效"会被误读成"FK 不依赖几何"。

### 5.2 `entry` 的动态加载

```yaml
kinematics:
  ik:
    type: package
    entry: kinematics/ik.py:solve_ik
```

Core 按这个指针定位文件并取函数。**不得**在 Core 里硬编码
`packages/mini_arm/kinematics/ik.py` —— 那样"加一个机器人"就必须改 Core。

### 5.3 JS 侧 `kinematics.js` 的边界（v0.1 后期新增）

**背景**：前端需要"本地预演"（拖动滑块时即时预测位形，不等后端往返）。
预演要算 FK/IK，于是需要一份**能在浏览器里跑**的实现。

**为什么放在包内而不是前端**

```text
✅ 包内  几何真值、公式、限位都在包内 → 前端 import 它，不产生第二份真值
❌ 前端  把 L1/L2/公式抄进 frontend/src/ → 两份会漂移，且新增机器人要改前端
```

**三份实现的分工（判据见 §5.3.2）**

```text
① backend/kinematics/fk.py          通用链式：遍历 joint 链逐级 compose（真读模型几何）
② packages/.../kinematics/fk.py     解析式：手推三角，几何常量写死
③ packages/.../kinematics/kinematics.js   解析式的 JS 移植
```

①↔② 互证已被 `packages/mini_arm/tests/` 钉住；
②↔③ 由 `tools/export_kinematics_fixture.py` + `frontend/src/sim/__tests__/kinematics.check.ts`
逐点比对（实测最大偏差见该脚本输出）。

#### 5.3.1 ⚠️ JS 侧**不能**用来证明"几何读对了 MJCF"

③ 与 ② 一样，几何长度是**文件内写死的常量**。因此：

```text
✅ ③ 能证明：前端算出的位姿与后端一致（预演不会显示别的位姿）
❌ ③ 不能证明：写死的常量与 MJCF 一致
```

几何正确性由 `packages/mini_arm/tests/test_kinematics_facts.py`
（常量 vs MJCF 逐项断言）与 Core 那条路径负责。
**要验证"FK 真的依赖几何"，必须用 Core 的 `forward_kinematics_generic`。**

#### 5.3.2 为什么是三份而不是两份

多一份就多一处要维护，**只有当它能当裁判时才值得**：

```text
①↔②  证明 Core 的链乘没写错
②↔③  证明前端预演不会算出与后端不同的位姿
①↔③  （间接）三方一致比两方一致更难被"共同的错误假设"骗过
```

判据的关键在于 ③ 必须是**独立移植公式**，而不能是
"把 Python 的输出预先烧成表" —— 烧好的表不是裁判，
是一条会与真值静默漂移的缓存。

#### 5.3.3 为什么是 `.js` 而不是 `.ts`

包是**与语言无关的数据包**，不该假设消费方用 TypeScript。

```text
.js + JSDoc 类型注释  →  浏览器 / Node / Vite / 任何打包器都能直接吃
.ts                  →  强制消费方有 TS 工具链，且包内多一个构建步骤
```

实测（v0.1 环境）：Vite 5 dev / `tsc --noEmit` / `vite build` 三条路径
都能直接 import 这个文件，**不需要** `server.fs.allow`，也不需要改 `vite.config.ts`。

#### 5.3.4 `manifest.kinematics.js` 段**不带** `type`

```yaml
kinematics:
  js:
    entry: kinematics/kinematics.js   # ← 没有 type
```

`type` 是 **Core 加载分派**的语义（`core` / `package`）。
JS 侧由前端自己的打包器加载，不经过 Core。套用同一套语义会造成误解
（例如让人以为 Core 也能把它当 package entry 动态 import）。

没有 `type` ⇒ `backend.cli show` 会正确地跳过它（渲染逻辑只在 `type` 存在时打印）。


---

## 6. 包内 `tests/` 放什么

放这台机器人的**构造事实**，不是通用逻辑测试：

```text
test_mini_arm.py
├── 关节顺序（qpos 顺序 = mobile_joint_ids() 的顺序）
├── 连杆长度与实测值的对应
├── 限位范围（与 MJCF 里的 range 一致）
├── 零位 TCP 位置 = (0.200, 0, 0.136)
└── IK 往返精度（分类判据，不是 Δ < TOL）
```

> ⚠️ **这些测试必须被 `pytest -q` 跑到。**
> `pytest.ini` 的 `testpaths` 里显式列了 `packages`。
> 不列的话 `pytest -q` 会显示全绿，而包内测试**从未执行** ——
> 那是"伪造通过"，比没有测试更糟。

---

## 7. 加一个新机器人（checklist）

```text
□ 建目录 packages/<id>/（**不要** __init__.py）
□ 写 manifest.yaml（id / name / coordinate / units / model / kinematics / capabilities）
□ 放 MJCF 到 model/
□ （可选）写 kinematics/ik.py，并在 manifest 里用 type: package 指向它
□ 写 tests/ —— 至少覆盖关节顺序、限位、零位位姿
□ 跑 .venv/Scripts/python.exe -m backend.cli list 确认被发现
□ 跑 .venv/Scripts/python.exe -m backend.cli inspect <id> 确认契约完整
□ 跑 .venv/Scripts/python.exe -m pytest -q 确认包内测试真的被执行
```

**验证"Core 里没有我的型号名"**（§69 规则 2）：

```bash
grep -rn "<id>" backend/ --include=*.py
```
应该**只**命中注释与字符串（文档在解释"不能硬编码"）。
`accept_phase2.py` 里有对应的 tokenize 断言。

---

## 8. 已知限制（v0.1）

| 项 | 状态 |
|---|---|
| 目录名与 `manifest.id` 必须一致 | 由 `load_model()` 校验，不一致报错 |
| `model.format` 只支持 `mjcf` | 资产注册表默认为空；URDF 只预留扩展点 |
| 一个包只能有一台机器人 | 多机器人（如双臂）需拆成两个包 |
| `manifest.yaml` 无 JSON Schema 文件 | 校验逻辑在 `tests/test_manifest.py` |
