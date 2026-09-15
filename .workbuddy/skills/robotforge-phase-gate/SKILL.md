---
name: robotforge-phase-gate
description: RobotForge / ArmPilot 项目的 Phase 推进与基线回归方法论。在推进 Phase、写新模块测试、判断"能否进入下一 Phase"、或需要给一个子系统补契约测试时使用。涵盖 Phase Gate 硬门槛、三层测试分工、反例注射与元测试模式、以及"假检查"陷阱清单。触发词：RobotForge、ArmPilot、MeArm-3D、Phase 推进、基线回归、验收清单、契约测试、反例注射、元测试、Phase Gate。
agent_created: true
---

# RobotForge / ArmPilot Phase Gate 与基线回归

## 0. 铁律

> **当前 Phase 未通过：不得进入下一 Phase。**

这条不是口号。它意味着每推进一个 Phase，必须先跑验收清单，**32/32 全绿**
才允许动下一个 Phase 的代码。跳过它会在 Phase 3~4 付出 10 倍代价 ——
因为那时错误已经扩散到运动学、仿真、前端三层，无法定位是"哪一层引入的"。

## 1. 一次性跑完全部检查

```bash
cd D:/user_project/git/RobotForge

# ① 验收清单（结构 + 契约 + 架构约束 + 测试套件）
.venv/Scripts/python.exe tools/accept_phase1.py    # 32/32
.venv/Scripts/python.exe tools/accept_phase2.py    # 56/56
.venv/Scripts/python.exe tools/accept_phase3.py    # 47/47
.venv/Scripts/python.exe tools/accept_phase4.py    # 65/65
.venv/Scripts/python.exe tools/accept_phase5.py    # 67/67（含前四者，耗时约 10 min）

# ② Core 测试
.venv/Scripts/python.exe -m pytest -q                    # 期望 432 passed

# ③ 包内测试（型号目录）
.venv/Scripts/python.exe -m pytest packages/mini_arm/tests -q   # 期望 34 passed

# ④ slow 标记（30000 采样批量测试）
.venv/Scripts/python.exe -m pytest -q -m slow
```

⚠️ **验收脚本必须后台跑**：沙箱会给长命令发 SIGTERM，
前台跑会得到 0 字节日志 + SIGTERM（看着像崩溃，其实只是被杀）。
Phase 4 清单因为内含 Phase 1~3 复查，**耗时约 6 分钟**；
Phase 5 内含 Phase 1~4 复查，**耗时约 10 分钟**，尤其要后台跑。

```text
.venv/Scripts/python.exe tools/accept_phase4.py > .workbuddy/scratch/p4.log 2>&1   ✗ SIGTERM
Bash 工具 run_in_background=true（配 -u 且重定向到文件）                            ✓
```

**判据是数字**，不是"看起来没问题"。每次改动后核对：

```text
pytest -q                        → 432 passed
pytest packages/mini_arm/tests   → 34 passed
accept_phase1.py                 → 32/32
accept_phase2.py                 → 56/56
accept_phase3.py                 → 47/47
accept_phase4.py                 → 65/65
accept_phase5.py                 → 67/67
```

数字变了（而不只是"仍然全绿"）说明测试被增删，必须解释原因。

## 2. 三层测试分工（不要混）

```text
tests/                             Core 契约：接口长什么样、报什么错
  test_coordinate.py               坐标系/四元数/Transform 数值契约
  test_units.py                    单位边界 + 禁止散落角度换算
  test_manifest.py                 Robot Package 清单
  test_mjcf_loader.py              Loader 抽象 + 翻译保真 + 拒绝非法输入
  test_robot_model.py              Canonical IR + frozen 契约 + Link-Joint 树
  test_validator.py                校验规则逐条反例 + 元测试
  test_fk.py                       FK 接口契约（任何符合契约的引擎都应满足）
  test_ik.py                       IK 接口契约（分支/失败分类/限位/clamp）

packages/<robot>/tests/            型号数值正确性：这个型号算得准不准
  test_kinematics_facts.py         常量/轴/限位/可达域/分支语义
  test_kinematics_regression.py    三路交叉验证 + 往返精度

tools/accept_phaseN.py             Phase 验收：结构事实 + 契约冻结 + 架构约束
```

**关键区别**：`tests/test_fk.py` 的断言必须写成"**任何**符合契约的 FK 引擎
都应满足"的形式，这样 Phase 3 把引擎从型号目录搬进 `backend/kinematics/`
时只需换 import。写成"mini_arm 的 FK 算出来是 0.2"就绑死了。

## 3. 反例注射 + 元测试（本项目最有价值的模式）

### 问题

校验类模块（validator / loader / 任何"错误→报告"的转换器）有四个失效模式，
**全都表现为"测试全绿"**：

```text
① 漏报  规则没实现          → 坏模型被放行，问题推迟到仿真里才爆
② 误报  规则太严            → 合法模型被拒，"功能好好的却启动不了"
③ 虚绿  规则写了但从未被触发 → 测试写了，测的是恒真条件
④ 崩溃  规则本身抛异常       → 调用方拿到无关错误，真实诊断从未生成
```

### 解法：声明式反例表 + 三个元测试

```python
def _all_counterexamples(model) -> dict[str, RobotModel]:
    """键 = 该反例应当触发的规则码。"""
    out = {}
    def add(code, mm):
        assert code not in out
        out[code] = mm
    add("tree.cycle", with_joints(joint("base_yaw", parent_link="forearm_link")))
    ...
    return out
```

三个元测试（缺一不可）：

```python
def test_every_error_rule_has_a_counterexample(self, model):
    # ast 抽源码里的规则码，与反例表键集合做差集 → 漏报检测
def test_counterexample_table_has_no_typos(self, model):
    # 反例表的码必须真实存在 → 拼错的码 = 永不失败的检查
def test_every_counterexample_actually_fires(self, model):
    # 逐条跑，断言目标码真的出现 → 注射错地方 = 假覆盖
def test_validator_never_raises_on_any_counterexample(self, model):
    # ④ 崩溃检测，见下节
```

用 `ast` 抽规则码，**不要用正则** —— 正则会把注释/文档字符串里的示例也
算进来。本项目在这上面吃过两次亏。

### ④ 崩溃检测的写法

```python
for code, broken in cases.items():
    try:
        validate_robot_model(broken)
    except Exception as e:      # 就是要抓全部
        raised.append(f"{code}: {type(e).__name__}: {e}")
assert not raised, "validator 在损坏输入上抛了异常（它应该返回报告）"
```

**为什么必须单独测这条**：实测发现 `_check_tree` 里

```python
model.link(model.root_link).parent_joint   # root_link 可能是 "nope" → KeyError
```

上面已经报了 `tree.root_link_missing`，但这个 `if` 没排除该情况，继续去
查一个不存在的 id。修法是先安全查找：

```python
target = next((l for l in model.links if l.id == model.root_link), None)
if target is not None:
    report.error("tree.root_link_not_root", ...)
```

## 4. "假检查"陷阱清单（都是实测踩过的）

### 4.1 漏括号 ⇒ 断言恒真

```python
assert not j.is_mobile        # ✗ is_mobile 是**方法**，bound method 恒真
assert j.is_mobile() is False # ✓ 项目约定：用 is False / is True 严格断言
```

同类：`Link.is_root()`、`Link.parent_joint is None`。

### 4.2 测试自己写坏了判据（世界投影 vs 本体平面）

诊断"肘上/肘下"时用**世界 XZ 投影的叉积**会错 —— `base_yaw ≠ 0` 时臂的
平面绕 Z 转了，投影出来是另一个量。

```python
# ✗ 与 yaw 有关
(tcp - sh)[[0, 2]] × (el - sh)[[0, 2]]
# ✓ 唯一 yaw 无关的判据：肘的世界 Z vs 肩→TCP 弦上最近点
chord = tcp_world - shoulder_world
t = (elbow_world - shoulder_world).dot(chord) / chord.dot(chord)
return elbow_world.z - (shoulder_world + chord * t).z
```

**教训**：症状伪装成"MJCF 有几何缺陷"，实际是自己的诊断脚本错了。
修完必须补一条 `test_branch_semantics_is_yaw_invariant` 回归测试。

### 4.3 零角处轴不可观测 ⇒ 改轴不改输出

`_q_revolute(axis, 0.0)` 对**任何** axis 都返回单位四元数。所以：

```text
base_yaw 轴 = +X / +Y / +Z  →  φ=0 时 orientation 完全相同
shoulder 轴 = +Y / -Y / +Z  →  直接改四元数轴，可观测
```

用 `base_yaw` 的轴去测"引擎是否读 model"会得到一个**恒真的假检查**。

### 4.4 目标朝向为 identity ⇒ orientation_error 与轴无关

`_orientation_error` 用 `abs(dot)` 处理双重覆盖，而 `dot(identity, q) = q.w`：

```text
target.orientation = identity() 时，ori_err ≡ 2·acos(|q.w|)  （与轴无关）
```

⇒ 验证"改轴影响朝向"必须**同时**避开 4.3 和 4.4：改 `shoulder` 的轴 +
目标朝向取一个**真实构型**的朝向（实测 0.000 → 1.341 rad）。

### 4.5 扫描式测试可能从未匹配到东西

源码扫描测试（"Core 不含型号字面量"、"单位换算只在边界层"）最大风险是
"它从来没匹配上任何东西，所以永远绿"。必须加元测试：

```python
def test_scan_actually_finds_known_violations(tmp_path):
    """写一个含已知违规的临时文件，断言扫描器能抓到 ≥3 处。"""
```

同时：扫描前必须**剥离注释与字符串**（用 `tokenize.generate_tokens`，
不要用正则 —— 处理不了三引号与 f-string）：

```python
stripped = tokenize.untokenize(
    tok for tok in tokenize.generate_tokens(io.StringIO(src).readline)
    if tok.type not in (tokenize.COMMENT, tokenize.STRING)
)
```

否则文档字符串里写"因此本模块不 import mujoco"、或教学反例
`{robot.id === 'mini_arm' && <IkPanel />}` 都会被误报。

### 4.6 静默通过的错误断言形式

```python
assert abs(a.y) < 1e-12 and abs(a.z) < 1e-12   # ✗ 假设过强
```
`shoulder` 原点是 `(0,0,SHOULDER_OFFSET)`，`elbow` 是 `(L1,0,0)` ——
每个都要按自己的语义断言，不能统一假设"关节原点在原点"。

### 4.7 "扰动自检"写成空转（Phase 3 连踩三次）

验收脚本里常有这条自检：*"故意改坏一个东西，确认断言会红"* —— 它防的是
"整份清单恒真"。但它本身极易写成恒真。**Phase 3 为一个扰动踩了三次：**

```python
# ❌ 尝试一：用 configs[0] + 改 shoulder 轴 (0,1,0)→(0,-1,0)
#    实测恒为 0。原因：绕 −Y 转 +θ ≡ 绕 +Y 转 −θ —— **规范等价**，
#    轴取反根本不是扰动。换构型也没用（换到哪个都还是 0）。

# ❌ 尝试二：改用平移 model.joint.origin（+X 50 mm），仍恒为 0！
#    原因完全不同，而且更重要：
#    包内 fk.forward_kinematics 是**解析闭式解**，读的是文件里写死的
#    几何常量，**根本不读 model.joint.origin**。
#    ⇒ "改模型里的一条 origin 却验不出差别"是**预期行为**，不是 bug。
#    ⇒ 但用解析 FK 做扰动自检就是无效的。

# ✅ 尝试三：用 **Core 链式 FK** 做扰动 + 平移 joint.origin
```

**通用规则**：写"扰动自检"前先问两个问题：

```text
① 我这个"扰动"在语义上真的是扰动吗？
   反例：轴取反（规范等价）、零角处改轴（不可观测，见 §4.3）
② 我扰动的量，被测函数真的读它吗？
   反例：解析 FK 不读 model.origin；IK 不读 model.axis（见 §9）
```

两个问题的答案都必须**实测**（看扰动前后输出差异 > 0），不能靠推理。

### 4.8 诊断脚本自己的控制流 bug 会伪装成引擎失败

Phase 3 有 44 个位形被自写诊断分类为 "fail"，实际引擎**完全正确**
（`dp=0, dq=0, ori_err=0`）。根因是诊断脚本里 `except` 分支的 `break`
吞掉后续比较 —— 引擎没错，是**测量工具**错了。

**教训**：出现"大量位形失败"时，先抽 1 个样本手工复算，
再怀疑批量逻辑。当一个自写分类器给出"部分失败"时，
**先验证分类器本身**（用一条已知正确/已知错误的样本喂它）。

### 4.9 验收判据不要用"容差"，用"分类"

Phase 3 的决定性设计：**先由几何量预测该位形属于哪一类，再断言实测吻合。**

```text
✅ 好的判据                              ❌ 坏的判据
"d<0 的位形，ΔR.w 必须 ≈0"              "Δquat < 1e-9"（过不了，d<0 就是 π）
"每类都必须能被 ①②③ 解释"              "Δquat < 2.0"（什么都测不到）
"不允许有『其它』类"                    "误差在可接受范围内"（无判据）
```

**为什么分类优于容差**：容差的阈值必须覆盖最坏情况（此处是 π），
于是"1° 的系统性错误"被放过。分类则要求**每个位形都能被机制解释** ——
将来若有人改坏坐标约定，大量位形会掉进"其它"桶，验收立刻变红，
**即使位置误差仍在 1e-12 内**。这是"防容差掩盖系统性错误"的唯一可靠做法。

### 4.10 ★ 判据要写"不变量"，不要写"我期望的终值"

**Phase 4 连踩三坑，引擎全对、判据全错。** 凡是一条路径上有**串联约束**
（限幅 → 限速 → 目标），判据就必须落在**各自独立可观测的点**上，
不能只看最终状态。

```text
① 限幅 + 限速串联
   命令 99.0，1 步后状态 = 0.35（限速的步长），**不是** position_max
   ❌ 断言 state == upper        ⇒ 失败（引擎正确）
   ✅ 断言 backend.clamped[j] == upper（夹紧后的**目标**）
      + 连发 N 步后 state 收敛到 upper

② 并发 Lock
   两个并发 step 是**串行**的，第二个从第一个的终点出发
   ⇒ 两次读数的**起点不同**，结果本来就不该相同
   ❌ 断言 s1 == s2 或 s1/s2 都在 ±max_step 内
   ✅ 断言"单步位移 ≤ max_step"（增量有界）+ 两者都动过

③ "未提及的关节保持原位"
   限速下 shoulder 停在**中途**（0.35）是**正确**行为
   ❌ 断言 shoulder == 0.4 与 elbow == 0.4（两个都到位）
   ✅ 断言 shoulder 逐位不变（== 发 elbow 命令前的值）且 elbow 收敛
```

⇒ **一句话：判据要描述"什么不该变"（invariant），
而不是"我预期变成什么"（expected final value）。**

与 4.9（分类优于容差）同源：两者都是**把"是否出错"从数值比较里拿出来，
变成一个可独立验证的结构性陈述**。区别只是 4.9 面向"等价类"，
4.10 面向"时间演化中的不变量"。

**附带的一条**：子进程结果的 JSON 解析要用 `json.JSONDecoder().raw_decode()`，
**不要** `line[len("RESULT_JSON:"):]` —— `run_python` 会把 stderr 拼在 stdout 后，
而无尾随换行的 `DeprecationWarning`（FastAPI/Starlette 在本机会打）
会**粘在 JSON 后面**，`json.loads` 报 "Extra data"。
这个坑是 Phase 4 的 WS 片段首跑就中招的。

### 4.11 ★ 判据与"被验对象的语义"自相矛盾 ⇒ 恒假（Phase 5 踩了两个）

4.7/4.10 讲的是"扰动没效果"与"看错了观测点"。**这一条更隐蔽**：
判据本身写的是一个**不可能为真**的命题，于是它**永远 FAIL**（或永远 PASS），
而失败信息看起来完全像"被测代码有 bug"。

```text
① `tempdir_cleaned = not Path(td).exists()`  ← 写在 `with TemporaryDirectory()`
   **块内部**。而该目录的删除发生在 with 块**退出时**。
   ⇒ 此处 `exists()` 必然为 True ⇒ 断言必然 FAIL。
   ⇒ 与 Runtime/Backend 的行为**完全无关**，是判据选错了时机。
   ✅ 改为断言"目录里没有多出预期之外的文件"（真的能反映"留残渣"这件事）。

② `assert "mearm" in stripped_src`
   stripped = untokenize(tok for tok in toks
                         if tok.type not in (COMMENT, STRING))
   ↑ 判据点要求"字符串正文仍在"，但剥离器的定义就是**把 STRING token 删掉**。
   ⇒ 正文随 token 一起消失 ⇒ 恒假。**判据与函数语义直接矛盾**。
   ✅ 判据必须同时包含"字符串被删"与"裸标识符保留 + 代码形态保留"，
      后者才证明剥离器没有退化成 `return ""`。
```

**通用规则**：写完一条断言，先问
**"这个表达式在什么情况下会是假？"** —— 若答不出（或答案与业务无关），
它就是恒真/恒假。再用 §3 的**反例注射**实测一遍：故意破坏被测对象，
断言**必须**变红。Phase 5 的两条就是这样被自己的元测试抓出来的
（`抹平版 → bare_kept=False`、`空转版 → comment_gone=False`，两者都正确变红）。

**Phase 5 还踩了同一族的另外两个变体**（都是"判据与输入约定不符"）：

```text
③ 期望集合漏了**前缀**
   `TemporaryDirectory()` 的根是 td，包目录 two_dof_toy 在其下
   ⇒ 相对路径是 `two_dof_toy/manifest.yaml`，
     不是 `manifest.yaml`
   ⇒ 我写的 expected 漏了前缀 ⇒ **我们自己写的四个文件全被当成"残渣"**
   ⇒ 恒 FAIL，但失败信息显示"多出的文件 = 我自己写的东西"，
     看起来像 Runtime 在乱写文件。
   ✅ 期望集合必须与 `relative_to(哪个根)` 严格对应。

④ 两个函数**入参序号不同**，却用同一个变量喂
   `mj_quat_to_xyzw(q_wxyz)`  入参是 MuJoCo 序 [w,x,y,z]
   `xyzw_to_mj_quat(q_xyzw)`  入参是 Core   序 [x,y,z,w]
   ⇒ 我拿 [w,x,y,z] 的值去喂 rev，得到 [0.3,0.5,0.1,0.2]，
     与我断言的 [0.5,0.1,0.2,0.3] 不符 ⇒ 判 FAIL。
   **与产品代码无关**：函数各自完全正确，是我的断言喂错了序。
   ✅ 断言里对每个函数用**它自己的**输入约定构造期望值：
      `fwd = mj_quat_to_xyzw(WXYZ)`  期望 `XYZW`
      `rev = xyzw_to_mj_quat(XYZW)`  期望 `WXYZ`
   ※ 这也说明"非对称值"有多重要：若用对称值，③④ 这类错误
     会互相抵消而不被发现。
```

⇒ **这一族的根因是一条**：判据里的**期望值**必须与
**被测函数的输入/输出约定**逐字对齐。写期望值时不要"凭记忆"，
而是**明文写出约定**（`# 入参是 [w,x,y,z]`）再推期望值。

### 4.12 扫描"import 关系"时别扫**文本**（§69 规则 2 类检查）

```python
# ❌ 检查"runtime 不许依赖 simulation"
hits = [f for f in runtime_files if "simulation" in read(f)]
# ⇒ 恒有 hits：`RobotBackend.is_simulation` 是 §47 规定的**字段名**，
#   backend.py 里必然出现 ⇒ 这条检查**永远红**，与依赖方向无关。
```

判据声称在测"依赖方向"，实际测的是"某个子串是否出现"。
✅ 只扫 **import 语句**（`import X` / `from X import`）：

```python
for node in ast.walk(ast.parse(src)):
    if isinstance(node, ast.Import):
        mods += [a.name for a in node.names]
    elif isinstance(node, ast.ImportFrom):
        mods.append(node.module or "")
```

同类陷阱见 §4.5：扫描前要 `tokenize` 剥离注释与字符串，
**但剥离后要记得**：字符串里的名字也**不是**引用（§4.11 ②）。

### 4.13 `str.format` 片段里的花括号必须双写

验收脚本里的"子片段"（`PHYSICS_SNIPPET` / `WS_SNIPPET` 等）普遍用
`SNIPPET.format(root=...)` 注入路径。片段里**一切字面花括号**必须写成
`{{` `}}`，否则：

```text
片段里写   expected = {"manifest.yaml", "model"}
运行时     KeyError: '\n            "manifest'
```

这个错误**不在片段里报，而在主脚本的 `.format()` 调用处报**，
看起来像"片段内容被当成 key 解析"，第一次遇到容易误判成字典语法问题。
✅ 加一条自动扫描（本次已用）：

```python
# 列出每个 format 片段剩余的"单花括号"内容，应当只剩真正的占位符
re.findall(r'(?<!\{)\{(?!\{)([^{}]*)\}', body)   # → 只应得到 ['root']
```

### 4.14 ★ 被测引擎"派生的量"可能落后主状态一个子步

**Phase 5 最有价值的一条。** MuJoCo 的 `mj_step` 是这样工作的：

```text
mj_step(m, d):
    ① 用**积分前**的 qpos 计算派生量（site_xpos / xpos / …）
    ② 再积分 qpos 与 qvel
```

⇒ `mj_step` 返回之后，`d.site_xpos` 描述的是**上一步**的构型，
而 `d.qpos` 已经是新的。实测偏差 **5.57e-08 m**（56 纳米）——
对刚体运动学来说这是**巨大**的误差，但**在孤立查看时完全看不出来**：
位置看着对、姿态看着对，只有拿它跟同一时刻的 `qpos` 独立算一遍才会暴露。

后果：`get_state()` 会返回一个**自相矛盾的帧** ——
关节角是 t 时刻的，末端位姿是 t−dt 时刻的。对 IK / 抓取是致命的。

```python
# ✅ 修法：推进后补一次纯运动学前向传递（不做积分、不改状态）
mujoco.mj_step(self._mj, self._data)
mujoco.mj_forward(self._mj, self._data)   # ← 刷新所有派生量到当前 qpos
```

**怎么发现的**：锚④要求"MuJoCo 的 TCP 位置"与"Core FK"互证，
阈值定在机器精度（`1e-9`）。我先怀疑几何漂移、单精度、`mju_mat2Quat`，
逐个实测排除（`mju_mat2Quat` 对解析矩阵误差 **0.0**；`mj_forward` 后
位置差 **0.0**、四元数差 **1.1e-16**），才定位到 `mj_step` 的时序。

⇒ **教训**：把"两个独立来源必须一致"的阈值定到**机器精度**，
不只是"更严格"——它是**唯一**能发现"派生量时序错位"的手段。
定成 `1e-6` 就会放过这 56 纳米。

### 4.15 物理量的容差里要**命名**残余误差的来源

Phase 5 的"命令 0.4 → 稳态 0.40238711（残差 2.4e-3）"**不是 bug**，
而是两部分的叠加：

```text
① 重力下垂      位置伺服 kp=60 有限 ⇒ 稳态必然有比例误差
② 上一子步残余   settle 后残余速度极小但非零
```

⇒ 物理量用 `PHYSICS_TOL = 5e-3`（覆盖上面两项，且**能**抓住真正的 bug，
如 Phase 5 抓到的自碰撞把关节卡在 0.9367 而非 1.5708 —— 差 0.63 rad，
远超容差）；纯运动学断言（FK 互证、四元数换算）用 `EXACT_TOL = 1e-12`。

**与 4.9（分类优于容差）同源**：容差不是"放宽标准"，
而是**把已知的物理残余与未知的错误分开**。
写容差时必须在注释里写明"这个容差在承担什么"。

### 4.16 ★★ "抽象存在" ≠ "抽象被用作分派点"（Phase 7 / 8，最重要的一条）

这是**扩展点类**验收的专有陷阱，且它的假绿**极难自查**：

```python
assert "RobotModelLoader" in loader_src    # ← 抽象存在
assert "mjcf" not in runtime_src           # ← Runtime 不依赖格式
assert registry.formats() == []            # ← 空表（Phase 8 场景）
```

这三条**全都通过**，而 Phase 7/8 要验的东西**一条都没验**。

`RobotModelLoader` 抽象**从 Phase 1 就有了**，但 `api/registry.py` 里
一直写的是 `MJCFLoader()` + `if fmt != "mjcf": raise` —— 也就是说：
**抽象存在了六个 Phase，却从没被用作分派点。**
Phase 7 的真正交付物不是"写抽象"，而是"**把一个已有的、没人用的抽象
改造成真正的分派点**"。这件事只能**行为上**证明。

⇒ **判据必须是"改变状态后再观察"**：

| 步骤 | 做法 | 为什么它能失败 |
|---|---|---|
| 注入合成实现 | 造 `toyfmt`/`toymesh`（与 MJCF/STL 毫无关系），注册它 | 若分派仍硬编码 `MJCFLoader()`，它拿 JSON 去喂 MuJoCo ⇒ **炸** |
| 观察可见差异 | 调用计数 +1、`resolve()` 从**抛错**变为**返回资产** | 若注册表是空转的，注册前后**没有差异** ⇒ 判据红 |
| 反向自检 | 复现"改之前"的硬编码行为，断言它**必须失败** | 若旧代码也通过 ⇒ 说明上面的绿来自别处 |

**Phase 8 尤其危险**：v0.1 的默认资产注册表**故意是空的**（§65 不实现任何
mesh parser）。于是：

```python
assert registry.extensions() == []    # ← 恒真：反正是空
assert "stl" not in registry          # ← 恒真：空表当然不含
```

**"空表"既可能是"设计如此"，也可能是"功能坏了"，单看它分不出来。**
⇒ 必须注册一个 `toymesh` 之后再观察。并且要用一个
"登记了但从不查表"的假注册表（`HollowRegistry`）反向证明这条观察**可失败**。

**另一个更阴的形态**（Phase 8 已钉住为反例）：硬编码分支能在**空表**下
伪装成功 ——

```python
def resolve(self, asset, *, name=""):
    if str(asset).endswith(".toymesh"):     # ← 根本不看 self._loaders
        return GeometryAsset(...)
```

它满足"空表 + 能 resolve"的所有表面判据。所以**不能只看空表**。

### 4.17 扫"字符串字面量"时，**故意不能**剥离字符串

§4.12 的规则是"别扫文本，扫 import"。但在**扩展点**场景里恰好相反：

```python
# 要验的判据：分派模块不得把格式名硬编码成字面量
lits = string_literals(registry_src)      # tokenize.STRING
hard = [s for s in lits if s in ("mjcf","urdf","stl",...)]
assert not hard
```

⚠️ 三个必须同时处理的细节：

1. **不能**用 `strip_comments_and_strings` —— 那会把要查的字符串**也删掉**
   ⇒ 判据恒真。（这是 §4.12 的**极性反转**：那里字符串是噪声，这里是目标。）
2. **子串匹配会误报**：`from .mjcf_loader import MJCFLoader` 合法地含有
   `mjcf` —— 但它是**模块名**，不是硬编码格式名。
   ⇒ 判据必须是"字面量**恰好等于**格式名"，不能是 `"mjcf" in src`。
   （曾据此误报，见 §4.12 同族。）
3. **扫描器必须元测试**：`assert not hard` 在**扫描器坏掉**时也通过
   ⇒ 配一条 `string_literals('x = "mjcf"\ny = "urdf"\n')` 必须抓到两个。
   "没找到"与"扫描器坏了"无法区分，这是 §4.6 的同族。

**补充**：docstring 也是 `STRING` token。判据（恰好相等）不受影响，
但**打印**会被一大段说明文字淹掉 ⇒ 过滤 `"\n" in body or len(body) > 120`，
并配一条 docstring 元测试（`'"""支持 "mjcf"。"""\nx = "mjcf"\n'` → `["mjcf"]`）。

### 4.18 合成测试模型的等价约束（别让"能加载"冒充"结构合法"）

写合成 loader（`toyfmt`）时，产出的 `RobotModel` 要过**与真实模型同一套**
Validator，否则验收会变成"能加载但结构非法"，而 `check` 若只断言
"加载成功"就漏掉了。Phase 7 实际踩到三条：

```text
tree.base_frame_empty   RobotModel.base_frame 默认 "" ⇒ 必须显式给
tree.multiple_roots     只有 root_link 的 parent_joint 是 None；
                        另一个 link **必须真的挂到 joint 上**（不是并列）
joint.no_position_limit revolute 关节要给完整 JointLimits(position_min/max)
```

⇒ 合成模型也要断言 `report.ok is True`，不能只断言 `model is not None`。



这是最容易写出错误测试的地方。

```text
边界 1  位置可达     r ≤ r_max = L1 + L2_EFF = 0.103 + 0.097 = 0.200 m
                    超出 ⇒ UnreachableError（带 distance / reach_max）
边界 2  姿态可达     |elbow| ≤ 135°
                    超出 ⇒ "位置可达但姿态不可达"

边界 2 更紧：|elbow| = 135° ⟺ r_lim = sqrt(L1² + L2_eff² + 2·L1·L2_eff·cos 135°)
                                 = 0.076737164 m
即 r ∈ (r_min, r_lim) 的目标位置可达但姿态不可达。
```

⇒ 随手写 `(0.05, 0.15)`（r≈0.052）会撞边界 2 抛 `IkError`，**不是 bug**。
正确做法是让测试**现算**边界，而不是硬编码数字：

```python
l1 = abs(model.joint("elbow").origin.position.x)
l2 = abs(model.joint("ee_link_fixed").origin.position.x)
l_tool = abs(model.site("tcp").transform.position.x)
r_lim = math.sqrt(l1**2 + (l2+l_tool)**2 + 2*l1*(l2+l_tool)*math.cos(theta_lim))
```

### `solve` vs `solve_all` 的刻意不对称

```text
solve(..., clamp=True)   "我知道目标不可精确达到，给我最近的可行解"
                         → 返回解 + clamped 标记 + **诚实的残差**（实测 74 mm）
solve_all(...)           "给我所有**能精确命中**目标的解"
                         → 做不到就不出现在结果里（阈值 1e-3）
```

不是不一致，是**责任划分**。把 clamp 解塞进 `solve_all` 会让调用方遍历时
误以为每个解都精确命中 —— 最糟的静默错误。

## 6. 2R 往返不是完美可逆（真实性质，不是 bug）

> ⚠️ **本节在 Phase 3 被重写过两次。** 旧版结论是"镜像构型 ⇒ 关节角不精确返回"，
> 方向对但判据错。最终判据见 §6.3 —— 它是**分类**而非容差。

### 6.1 现象：朴素往返断言会失败

```python
q → FK → IK → q2;  assert q2 == q       # 对 mini_arm 有 ~48% 位形失败
pose → IK → Joint → FK → pose2;  assert pose2 == pose   # 也有 27/120 失败
```

失败原因**不是 bug**，是一条可精确陈述的机构定律。

### 6.2 定律：以"TCP 沿基座轴线的有符号投影"分类

定义 `d(θ1,θ2) = L1·cos θ1 + L2eff·cos(θ1+θ2)`（`L2eff = L2 + L_TOOL`）。

```text
① d > 0  TCP 在基座轴前方
     ⇒ IK 返回同分支解，位置与姿态**都精确闭合**
② d < 0  TCP 在基座轴**后方**（臂反折越过基座轴）
     ⇒ atan2(TCP) 必然 = base_yaw + π
       IK 返回"从反侧够到同一物理点"的等价位形：
       位置精确（7e-17 m），姿态差一个**精确的 180° 旋转**
       该解的 elbow 与源**逐位相同**，只是 yaw 差 π、shoulder 被重解
③ d = 0  elbow ≡ 0（肘全伸直，退化）
     ⇒ 两分支合并，IK 正确标注 [degenerate]，位姿仍精确闭合
```

**双向验证（全网格 11³）——交叉项全为 0，故 d 的符号是充要判据：**

```text
d>0 且姿态一致 : 979    d>0 且 |ΔR.w|≈0 :   0
d<0 且 |ΔR.w|≈0: 286    d<0 且姿态一致  :   0
elbow≡0 退化   :  44    无法解释         :   0
```

### 6.3 ✅ 正确判据：看相对旋转 ΔR 的 w 分量

**关键细节（踩了三次才定下来）**：`d<0` 时的 180° 旋转，其**转轴随位形变化**：

```text
ΔR = R_ik · R_src⁻¹  ⇒ w 恒为 0（转角恒为 180°），
但轴是 [0.021,0.016,0.9997] / [0.101,0.451,-0.887] / …
```

⇒ **不存在任何"左乘/右乘某个固定旋转就能补偿掉"的写法**。三次尝试的实测残差：

```text
直接比                  : 1.325   ← 太严，27 个位形"失败"
左乘 Rz(π)·src          : 0.970   ← 猜想"解=源绕基座Z转π" ❌
右乘 src·Rz(π)          : 1.198   ← ❌
共轭 Rz(-π)·src         : 0.970   ← ❌
```

所以断言必须写成：

```python
dR = R_ik * R_src.conjugate()
if abs(dR.w - 1.0) <= 1e-9:   # 同姿态    → 要求 Δquat ≤ 1e-12
elif abs(dR.w)     <= 1e-9:   # 精确 180° → 合法的"反侧等价位形"
else:                          # 中间态    → **判失败**（不许存在）
```

**为什么这条比"容差"锋利**：它把"差 180°"与"差 179°"区分开。
任何 `Δquat < TOL` 的写法都做不到 —— 要过就得把 TOL 设到 π 以上，
于是 1° 的系统性错误也会被放过。**本项目 Phase 3 的核心教训。**

### 6.4 位置与姿态必须分开返回，不能用一个 min 打包

第一版把 `Rz(π)` 也算进了**位置**，于是 `d<0` 组的"补偿后位置差"变成
绕 Z 转 π 后的距离（最坏 0.89 m），而 `min()` 会挑错配对 ⇒ 报出
**Δpos=0.89 m 的假失败**。正确做法：**位置永远直接比**（实测 1e-15），
姿态才做 π 补偿 —— 且二者要**分开返回**。

### 6.5 `solve_all` 的镜像率（旧版结论仍然有效）

```text
shoulder 与 elbow **符号相反** ⇒ 2925/2925 精确往返，0 失败
shoulder 与 elbow **符号相同** ⇒ 1927 可逆 / 1148 选中镜像（≈0.19）
```

⇒ 用 `solve_all` 取最优解时，位置总是精确，**姿态有 ~19% 概率是 180° 等价形**。
这解释了"不指定分支时 27/120 姿态差 π"—— 是 2R 双解，不是 IK 坏了。

## 7. 契约发现：先探针，再断言

写测试时**不要猜 API 名**。本项目猜错过至少 12 处：

```text
JointLimits.lower/upper          → 实际 position_min / position_max
Link.geometries                  → 实际 link.visual + link.collision
RobotModel.id                    → 实际 model.metadata.id
Transform.pos / .translation     → 实际 .position
Vector3.length                   → 实际 .norm()
RM.Coordinate / RM.Units         → 实际 CoordinateConvention / UnitConvention
ROBOTFORGE_UNITS.length          → 是 **dict**，不是 dataclass 实例
Joint.is_mobile / Link.is_root   → 是**方法**，必须带 ()
loader.load() 返回 Model       → 实际返回 tuple（model, report）
```

**方法**：动手前跑一个 5 行探针把真实签名/字段打出来：

```python
.venv/Scripts/python.exe -c "
import sys, inspect; sys.path.insert(0,'.')
from backend.model import robot_model as RM
for n in ['Joint','Link','Site','Frame','EndEffector']:
    print(n, [f.name for f in __import__('dataclasses').fields(getattr(RM, n))])
"
```

## 8. Python 3.13 `@dataclass` 按路径加载的坑

`packages/` 故意**没有** `__init__.py`，所以按文件路径加载模块时：

```python
def _load_by_path(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod          # ⚠ 不能省 —— @dataclass 需要反向查模块
    spec.loader.exec_module(mod)
    return mod
```

另外两条：

- **不要用相对 import**（`from .conftest import ...`）—— `packages/` 没有
  `__init__.py`，会报 `attempted relative import with no known parent package`。
  改用 pytest fixture 暴露。
- **`packages/mini_arm/tests/` 不在 rootdir 的 conftest 链上**，所以
  `mini_arm_mjcf` 这类 fixture 要在包内 conftest 里**重新声明**（转发）。
  不要用 `pytest_plugins`（非 rootdir conftest 里已废弃且会报错）。

## 9. mini_arm 几何与 FK 速查（实测值）

```text
BASE_HEIGHT      = 0.084   base → base_yaw
SHOULDER_OFFSET  = 0.052   base_yaw → shoulder      （肩的世界 z = 0.136）
L1               = 0.103   shoulder → elbow
L2               = 0.065   elbow → ee_link
L_TOOL           = 0.032   ee_link → tcp
L2_EFF           = 0.097   = L2 + L_TOOL
r_max = 0.200   r_min = 0.006   r_lim(姿态) = 0.076737164

关节：base_yaw 轴 +Z ±π ｜ shoulder 轴 +Y ±π/2 ｜ elbow 轴 +Y ±3π/4 ｜ ee_link 无 joint
零位 TCP = (0.200, 0, 0.136)   ← 若得到 0.168 说明返回的是法兰（ee_link）
```

### `fk.py` 有两个函数，位置语义**不同**（极易误判）

```text
packages/mini_arm/kinematics/fk.py
  forward_kinematics(model, q)          解析式：L1/L2/L_TOOL 是**常量**，
                                        只从 model 取旋转轴 + 算四元数
  forward_kinematics_generic(model, q)  通用链式 —— Phase 3 后是
                                        **Core 的重导出**（同一对象）

backend/kinematics/fk.py（Core，Phase 3 新增）
  forward_kinematics(model, q)          通用链式：几何**全部**来自 model
  link_transforms(model, q)             同一条链，返回全部中间位姿
```

**⚠️ Phase 3 后 `forward_kinematics_generic is core.forward_kinematics`
（同一对象，不是同名副本）** —— 由 `test_fk.py` 断言。

**这条差异有一个实际后果**（§4.7 尝试二的根因）：

```text
解析 FK 不读 model.joint.origin
⇒ 改 model 的几何（如肘原点×2）时它**输出不变**、
  IK 的 position_error **不变**
⇒ 不是自检是假的，而是常量几何 + 解析式 FK 的组合结果
⇒ 要验证"自检真的用了传入的 model"，判据必须落在**朝向**上
  （改 shoulder 的轴 → orientation_error 从 0.000 变 1.341）
⇒ 做扰动自检必须用 **Core**，不能用解析 FK
```

### v0.1 的 IK 是"型号算法"不是"通用引擎"（**Phase 3 后仍如此，是设计决定**）

实测：`ik.py` 中 `'axis'` 出现 **0** 次、`model.<属性>` 出现 **0** 次；
改 shoulder 的 axis 后 `solve()` 返回**完全相同**的关节角。几何与限位全部
来自模块常量（从 `fk.py` 复用，与 MJCF 一致）。

Phase 3 评审后的结论：**这是经论证的边界，不是待办缺口。**

```text
FK 可通用：沿 Link-Joint 链乘 Transform，对任何 Tree 都成立、无型号字面量
IK 不可通用：闭式解依赖机构（2R 余弦定理、atan2 偏航、肘侧判别）
            "通用 IK" 只能退化成数值迭代 ⇒ 引入容差 ⇒
            削弱 v0.1"精确往返"的验收标准
```

⇒ `manifest.yaml` 用 `kinematics.ik.type: package` 显式编码这个边界；
CLI 按 `type` 分派（`package` 加载 entry；`engine` 报"v0.1 尚未提供"）。
测试从"会失败的钉住测试"改写为
`test_package_ik_is_a_2r_analytic_solver_not_a_generic_engine`
+ `test_manifest_declares_ik_as_a_package_implementation`。

## 10. 新子系统测试的标准步骤

```text
① 探针  跑 5 行脚本把真实 API（字段名/签名/返回值形状）打出来
② 基线  先写"正确输入必须通过/必须零问题"—— 挡住误报
③ 契约  写接口断言（形状、报错类型、诊断信息、确定性、不改输入）
④ 反例  逐条规则注射坏模型（如果是对外契约），注册成声明式反例表
⑤ 元测  cover 差集 + 拼写检查 + 真的触发 + 不抛异常
⑥ 假检查 每个"改 X 应影响 Y"的断言，先问"这个 X 真的可观测吗"
⑦ 通用性 源码扫描（tokenize 剥离注释）+ 扫描器元测试
⑧ 全跑  更新 tools/accept_phaseN.py，跑三路数字确认
⑨ 记录  日志写进 .workbuddy/memory/YYYY-MM-DD.md
```

**每次测试失败都要归类**：是"我的假设错了"还是"代码真的有 bug"。
前者改测试并**把真实契约写进注释**（下一个人会再猜一次）；后者改代码并
补一条钉住该 bug 的回归测试。本项目至今的失败**绝大多数属于前者** ——
所以第 ① 步的探针不是可选项。

## 11. Phase 现状（截至 2026-09-15）

```text
Phase 0  ✅ 骨架 + venv + pytest 可跑
Phase 1  ✅ 32/32；Core 测试全绿
Phase 2  ✅ 56/56；Three.js 渲染链路 + 三套前端可执行检查
Phase 3  ✅ 47/47；293 (Core) + 34 (mini_arm) 全绿
             forward_kinematics 提升进 backend/kinematics/fk.py
             + backend/cli.py（list/show/fk/ik/inspect，均支持 --json）
             + tests/test_kinematics_core.py（24 项，合成非 mini_arm 模型）
Phase 4  ✅ 65/65；375 (Core) + 34 (mini_arm) 全绿
             backend/runtime/{command,state,backend,robot_runtime}.py
             backend/api/websocket.py + app.py（lifespan 管理 Runtime，WS/REST 共用实例）
             tests/test_runtime.py（52）+ tests/test_websocket.py（30）
             docs/runtime.md
Phase 5  ✅ 67/67；432 (Core) + 34 (mini_arm) 全绿
             backend/simulation/{simulation_backend,mujoco_backend}.py
             tests/test_mujoco.py（57）
             MuJoCo 3.13.0；mini_arm 的 MJCF 补 armature + contact/exclude
Phase 6  —（spec 无此 Phase，§64 直接接 Phase 7）
Phase 7  ✅ 84/84；488 (Core) + 34 (mini_arm) 全绿
             backend/loaders/registry.py（LoaderRegistry + get/default_loader_registry）
             backend/api/registry.py 的 load_model() 改为**查注册表分派**
               （原为 MJCFLoader() + if fmt != "mjcf": raise）
             tests/test_loader_registry.py（23）
             验收要点见 §4.16 / §4.17 / §4.18
Phase 8  ✅ 96/96；488 (Core) + 34 (mini_arm) 全绿
             backend/loaders/asset_loader.py
               （AssetLoader / GeometryAsset / GeometryAssetRegistry / 错误分层）
             tests/test_asset_loader.py（33）
             ★ v0.1 默认（模型）注册表 = [mjcf]；（资产）注册表 = []（**设计如此**）
```

### Phase 7 / 8 的关键决定（重要，别改回去）

```text
✅ 注册名取自 loader.format_name，**不**由调用方传参
   ⇒ 传参会让「注册名」与「loader 自称的名字」可能不一致，而分派用注册名

✅ 重复格式名 ⇒ 抛 LoaderRegistrationError（不是静默覆盖）
   ⇒ 静默覆盖会让「我注册没生效」表现成「加载出来的模型长得不对」

✅ UnknownFormatError **继承** LoaderError
   ⇒ API 层按基类统一处理（都是 500，不是 404）；
     但它是独立类型，测试才能精确断言「是因为没有 loader，不是解析失败」

✅ LoaderRegistrationError **不**继承 LoaderError
   ⇒ 「装配代码写错了」vs「模型内容坏了」，责任人与修复动作不同

✅ 两个注册表**故意**是两个类、两套生命周期：
     LoaderRegistry       整个模型、加载期一次、格式名索引
     GeometryAssetRegistry 单个几何资源、按需多次、**扩展名**索引
   ⇒ 缺 asset 不得阻止机器人启动（§69）

✅ GeometryAsset.data 是 Any + kind 判别符（mesh/uri/native）
   ⇒ 钉成 bytes 会把 URL 引用逼进内存；钉成 str 让已解码网格无处安放

✅ GeometryAsset.to_dict() **不含** data（§24）
   ⇒ 否则一个 50k 顶点的 STL 会让每个 robot_info WS 帧变成几 MB

✅ resolve(None) / resolve("") → None，**不抛异常**
   ⇒ v0.1 的**正常路径**（原生 geom 没有外部资源）。
     抛异常会逼每个调用方写 try/except；真正该报错的是「有引用但没人能加载」

✅ 模型注册表是**懒装配**（module-level 常量会在 import 时就拖进 mujoco，
   违反 §69「没有 mujoco 也要能起来」）
```

### Phase 5 的关键决定（重要，别改回去）

```text
✅ 仿真层拆两个文件：simulation_backend.py（引擎无关：关节序、限幅、拼 State）
   + mujoco_backend.py（引擎特定：积分、接触、执行器动力学）
   ⇒ §69 规则 2「Core/Frontend 无型号分支」的落地点：换引擎只动后者

✅ 关节序**只有一个定义**：model.mobile_joint_ids()
   命令展开、状态拼装、与引擎 qposadr 的对齐全部引用它
   ⇒ 并在 start() 里断言 qpos_addr == sorted(qpos_addr)，顺序错就立刻报错

✅ get_state() 的末端位姿用 **Core FK** 算，**不读** MuJoCo 的 site_xpos
   ⇒ 理由见 §4.14 与下面的「锚④」：让两者成为**互相独立的裁判**

✅ 限幅夹的是**关节 range**，不是 actuator ctrlrange
   ⇒ MuJoCo 对超 ctrlrange 的 ctrl 是**静默丢弃（保留旧值）**，
     按 ctrlrange 夹会让「命令超限」表现成「机器人完全不动」且无任何报错

✅ Runtime / WS / 路由**一行未改**就换上了 MuJoCoBackend
   ⇒ 靠 create_app(backend_factory=mujoco_backend_factory) 注入
```

**Phase 5 怎么防"物理是假的"** —— MockBackend 是线性插值器，
朴素的"命令 0.5，几步后 0.5"断言它也能过。所以用
**五个锚 + 两个反向自检**，且每个都必须**真的可能失败**：

```text
锚① 重力/惯量真的在作用   零位形下令肩关节下沉，且 ≠ 该构型平衡点
锚② 关节耦合（结构性判据） 只命令 shoulder，elbow 被惯性**带动**
                          ⇒ MockBackend 结构上做不到（它逐关节独立插值）
锚③ 速度真非零、真衰减    运动中 max|qvel| > 1e-3，稳态 < 1e-3
锚④ MuJoCo ↔ Core FK 互证 两个独立来源，阈值定在机器精度（见 §4.14）
锚⑤ 换一台机器人          合成 2-DOF 模型（pan/lift）驱动**同一个** Backend
                          ⇒ 关节数与关节名都不是硬编码

自检A mock vs mujoco 的单步状态**必须不同**（实测差 0.235）
      ⇒ 证明锚②③ 不是空转
自检B gravity = "0 0 0" 时下垂必须消失
      ⇒ 证明锚① 的效力来自重力，不是巧合
```

**实测数字**（写"通过"时引用的就是这个）：

```text
settle 后 shoulder 下垂      0.002571
命令 0.5 → 1 步 0.1166 → 60 步 0.50216 → 稳态残差 2.4e-3
tcp_position_gap（定位前）   5.5e-08  ← 就是 §4.14 的时序错位
tcp_position_gap（修复后）   5.1e-14
五个极值目标：sh +90° → 1.57079633 ｜ el −135° → −2.35625677
              yaw π → 3.14159265   （全部 ncon = 0，精确到位）
```

### Phase 4 的关键决定（重要，别改回去）

```text
✅ RobotCommand（Desired）与 RobotState（Actual）是两个类型，且必须**真的**分离
✅ Backend 由**注入的工厂**产生（backend_factory），否则 §69 规则 10 作废
✅ MockBackend **故意**有状态 + 限速 + 限幅 ⇒ 让"命令失败"这个失败模式
   在 Phase 4 就被演练（原样回显会让闭环验收"结构性通过、信息上零")
✅ Runtime 绑在 FastAPI **lifespan** 上（async start + 失败发生在启动期）
✅ WS 帧里只能有物理量（§49 禁止服务端发渲染数据）
```

**验收怎么防"假闭环"**：三道锚 + 一项反向自检 ——
① `command_count` 真的增长（命令穿过 Runtime 到达 Backend）
② 限速下 State ≠ Command（回显式实现会被抓）
③ **Core FK** 当独立裁判重算末端位姿（防 Backend 读包内写死常量）
自检：去掉限速后 State 就等于 Command ⇒ 证明锚②的有效性来自限速而非巧合。

**Phase 5 的接入点已落地**：`mujoco_backend_factory(model)` 传给
`create_app(backend_factory=...)` —— Runtime / WS / 路由**一行不改**。
唯一要特别小心的是 §40/§35 的 **MuJoCo 四元数边界换算
`[x,y,z,w] ↔ [w,x,y,z]` 必须集中在一处**：

```python
# backend/simulation/mujoco_backend.py —— 全项目**唯一**允许重排四元数的地方
def mj_quat_to_xyzw(q_wxyz):  w, x, y, z = q_wxyz[0], q_wxyz[1], q_wxyz[2], q_wxyz[3]
                              return [x, y, z, w]
def xyzw_to_mj_quat(q_xyzw):  return [q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]]
```

两个方向都要留（即使 v0.1 只用到单向）——**单向换算正是当初出 bug 的原因**。
验收脚本用一条正则扫描全 `backend/` 断言该重排**只出现于这一个文件**。

**边界换算的判据要用非对称值**（`[w,x,y,z] = [0.5, 0.1, 0.2, 0.3]`）：
用对称值或 `w=0` 的值，写错成"取后三位"也能过；非对称值下四个分量
全错位都无处躲。再加上"往返"和"一对互为逆函数"两条自检。

### Phase 3 的边界决定（重要，别改回去）

```text
✅ FK 提升为 Core 引擎   backend/kinematics/fk.py（通用链式相乘）
✅ IK **不**提升，留在包内  packages/mini_arm/kinematics/ik.py
```

**理由**：只有 FK 是可通用的（沿 Link-Joint 链乘 Transform，对任何 Tree
模型都成立、不含型号字面量）；IK 的闭式解**依赖机构**（平面 2R 余弦定理、
atan2 求偏航、肘侧判别）。通用 IK 只能退化成数值迭代 ⇒ 引入容差 ⇒
会削弱 v0.1 的"精确往返"验收标准。`manifest.yaml` 已用
`kinematics.ik.type: package` 把这个边界编码进去。

⇒ **因此 `tests/test_ik.py` 里那条"IK 不读轴"的钉住测试被重写为**
`test_package_ik_is_a_2r_analytic_solver_not_a_generic_engine`
（从"待办缺口"改成"经论证的设计边界"，双向守卫），
并**新增** `test_manifest_declares_ik_as_a_package_implementation`。

### Phase 2 遗留

- **未用真浏览器截图**（`agent-browser` 未装，需 ~500MB Chromium）。
  因此"浏览器显示机器人"是**间接证明**（元素树 + 几何构造），
  **目视外观与人眼手感未经确认**。起 `vite` 后应肉眼过一遍。
- `docs/architecture.md`、`docs/robot-package.md` 未补
- `pyproject.toml` 未建

## 12. 前端（Three.js / TS）Phase 的检查设计

> 前端部分的完整坑清单见 [references/frontend-checks.md](references/frontend-checks.md)。
> 这里只留决策要点。

### 三条能力边界，别混

`renderToStaticMarkup` **不是浏览器**。它走 React DOM、没有 fiber reconciler，
所以它能证明的与不能证明的必须分清：

| 想验的东西 | 该用什么 | 不能用什么 |
|---|---|---|
| 层级/父子/属性**结构** | `renderToStaticMarkup` + 自写 markup 解析 | 调组件函数（hooks 会抛） |
| 几何**尺寸/轴向** | 直接调 `makeGeometry` + 读 `boundingBox` | 静态渲染（看不见 args） |
| 坐标适配**数学** | 纯函数断言（axiom 导出后直接 assert） | 截图 |
| 真交互 / 目视外观 | 只能人眼 | 任何无头方案 |

### 症候 → 根因 速查

```text
所有"找得到吗"断言全红，但独立探针显示函数输出正确
  → 你把组件当普通函数调了（useMemo 抛错 → 静默降级成空节点）
  → 用 renderToStaticMarkup 真渲染

断言"元素属性值不对"，而 JS 对象里明明是对的
  → 该属性是个**对象**（Vector3/Quaternion/userData），序列化时被丢掉
  → 传数字数组 / 字符串；把"值可序列化"本身变成一条会失败的检查

把正确实现判成错的（最危险）
  → 你用"数量/存在性"当判据（子 group 个数、"有没有子节点"）
  → 改为按**语义名**识别（name 前缀 axis: / geom-unknown）
  → 并且：兜底可能换了标签类型（<group> → <mesh>），查找要按 name 而非 tag
```

### 铁律：检查的"期望值"必须有**独立来源**

前端检查最容易退化成自证。两条独立来源的用法：

1. **Python 侧独立重推**：`tools/export_view_fixture.py` 用最朴素的 Python
   重推一遍层级/EE/几何位置，导出成 fixture 的 `expect`。
   TS 侧只与 `expect` 比，**绝不与 TS 自己的输出比**。
2. **契约表来自模型数值**：`geometry.check.ts` 的期望外接盒由 **MJCF size 语义**
   从模型数字推出，不读 `makeGeometry` 的实现。否则就是把实现抄一遍。

再加一条：**每个检查脚本都要有"注入缺陷自检"**（`SELF_TESTS`），
断言"注入这个缺陷后检查**必须**报错"。没有它，坏掉的检查比没有检查更危险。

## 13. Windows/MSYS 沙箱：起服务与临时文件

```text
nohup node x.js &                      ✗ 进程只活到本次工具调用结束
Bash 工具 run_in_background=true       ✓ 长驻（起 vite / uvicorn 用这个）

cp x /tmp/x                            ✗ Permission denied
cp x .workbuddy/scratch/x              ✓ 临时文件放工作区内，用完即清

grep -oE '...' file                    ✗ `-u系统找不到指定的文件`（MSYS 引号/参数错乱）
Greet 工具 / 先落文件再读               ✓

curl -o /dev/null -w ... && curl ...   ✗ Exit 23 会中断整条链
逐条跑 / 把输出落文件再读               ✓

npm --version                          ✗ npm shim 读 uname 走 wslpath → 沙箱拦 wsl.exe
node.exe '<C:\...>\npm-cli.js'         ✓ 且脚本参数必须**原生 Windows 路径**
                                       （MSYS 会把 /c/... 改写成 D:\c\...）
bash tools/x.sh                        ✗ bash → System32\bash.exe（WSL 启动器）
/usr/bin/bash tools/x.sh               ✓

timeout 60 ...                         ✗ 命中 System32\TIMEOUT.EXE（不认 GNU 参数）
```

### 静态渲染的两个"假警报"——**不要照着改**

`renderToStaticMarkup` 下必然出现，且**不是缺陷**：

```text
"<meshStandardMaterial /> is using incorrect casing"   ← 真 fiber 里小写名正确且必需
"React does not recognize the `userData` prop"          ← 同上
```

照着改成 PascalCase 会把浏览器里的渲染改坏。
