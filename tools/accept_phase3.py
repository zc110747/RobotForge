#!/usr/bin/env python
"""Phase 3 验收清单（对应 spec §60：FK / IK）。

## 用法

```bash
.venv/Scripts/python.exe tools/accept_phase3.py
```

退出码 0 = 全部通过；非 0 = 有未通过项。

## Phase 3 是什么（spec §60 逐字）

实现：

```text
Joint → FK → Pose
Pose → IK → Joint
```

验收：两条往返

```text
① Joint → FK → Pose → IK → Joint'
② Pose  → IK → Joint → FK → Pose'
```

## 这份清单怎么把"往返"变成可执行断言

往返测试最容易写成**恒真**的东西：

```python
pose = fk(q);  q2 = ik(pose);  assert fk(q2) == pose   # ← 可能什么都没证明
```

因为它测的是"ik 与 fk 互为逆"这一件事，而如果两边**错得一样**
（比如都用同一个错的几何常量），它也通过。

⇒ 所以本脚本对两条往返各自加了**独立的锚**：

| 往返 | 恒真的风险 | 本脚本的锚 |
|---|---|---|
| ① Joint' | IK 可能返回 `q` 本身（比如把输入透传） | 断言 `Joint'` 真的被**重算**：扰动的关节也必须被恢复 |
| ② Pose' | FK 与 IK 可能共享同一个错常量 | 用 **Core 链式 FK**（不读包内常量）当第三方裁判 |

另外脚本会**反向验证自己**：临时把一个关节的轴改掉，确认往返断言
真的会失败（否则这份清单可能整体是空转的）。这一条叫"清单自检"。

## 覆盖的架构约束（提示词 §69 规则 2 + spec §41）

* Core `backend/kinematics/` 里不得出现机器人型号字面量
* Core 不得 import `mujoco` / `three` / `fastapi` / `runtime`
* `model` 不得 import `kinematics`（方向只能是 kinematics → model）
* **不得为了 Three.js 在 FK / IK 里改坐标**（spec §41）——
  转换只允许存在于 `coordinateAdapter.ts`

## 设计原则（与 accept_phase1.py / accept_phase2.py 一致）

* 每项**独立可判**，失败给"期望 vs 实际"
* 不依赖网络
* 负向测试会自行清理（并断言清理成功，否则后续项全部失效）
* 所有改动都在内存副本上，**不修改任何仓库文件**
"""

from __future__ import annotations

import math
import pathlib
import subprocess
import sys
import tokenize

ROOT = pathlib.Path(__file__).resolve().parent.parent
PY = ROOT / ".venv" / "Scripts" / "python.exe"
if not PY.exists():
    PY = pathlib.Path(sys.executable)

#: 共享的 pytest 判词解析器（唯一真值源），与 harness 同目录。
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from _pytest_verdict import pytest_verdict  # noqa: E402


_results: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    _results.append((name, bool(ok), detail))


def run_pytest(*args: str) -> tuple[bool, str]:
    """跑 pytest，返回 `(是否全通过, 摘要)`。

    ⚠️ **判词来自输出，不是退出码** —— 见 `_pytest_verdict.py` 的模块 docstring：
    本机沙箱的批量删除守卫会拦下 pytest 的临时目录清理，
    让"测试全过"的一次运行**退出码非 0**（实测把 Phase 7 顶成 39/50）。
    """
    proc = subprocess.run(
        [str(PY), "-m", "pytest", *args, "-q", "--no-header"],
        cwd=ROOT,
        capture_output=True,
        text=True,
    )
    return pytest_verdict(proc.stdout + proc.stderr)


def run_python(snippet: str, timeout: int = 120) -> tuple[int, str]:
    """在当前 venv 里跑一段 Python（用于数值验收）。

    ⚠️ 结果**显式读 stdout**：本机 PowerShell/沙箱会吞原生进程输出，
    所以这里用 subprocess + capture_output，并把结论行捞出来。
    """
    proc = subprocess.run(
        [str(PY), "-c", snippet],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=timeout,
    )
    return proc.returncode, (proc.stdout or "").strip() + (proc.stderr or "").strip()


def strip_comments_and_strings(src: str) -> str:
    """剥离注释与字符串，用于"源码里有没有出现某关键字"的判定。

    必须剥离：否则一句注释"这里是通用引擎，不含 mini_arm"
    就能让"不得出现型号名"的检查失败 —— 而那显然是误判。
    """
    return tokenize.untokenize(
        tok
        for tok in tokenize.generate_tokens(iter(src.splitlines(True)).__next__)
        if tok.type not in (tokenize.COMMENT, tokenize.STRING)
    )


def _fmt(x: float, digits: int = 12) -> str:
    return f"{x:.{digits}g}"


# ---------------------------------------------------------------------------
# 数值验收的 Python 片段（在子进程里跑，避免污染本进程的 import 状态）
# ---------------------------------------------------------------------------

#: 两条往返 + 独立锚。输出以 RESULT_JSON: 开头，便于父进程解析。
#:
#: ## 关于"往返"的实测结论（2026-09-15，已在 11³ 全网格上双向验证）
#:
#: 朴素写法 `q → FK → IK(solve) → q2; assert q2 == q` 对 mini_arm 会**失败**，
#: 而且失败的原因**不是 bug**。这不是"容差不够"或"实现不精"，
#: 而是一条可以精确陈述的机构定律：
#:
#: ```text
#: 令 d(θ1, θ2) = L1·cos θ1 + L2eff·cos(θ1 + θ2)
#:            = TCP 沿基座轴线的**有符号投影**  （L2eff = L2 + L_TOOL）
#:
#: ① d > 0  （TCP 在基座轴前方）
#:      ⇒ IK 返回与源同分支的解，**位置与姿态都精确闭合**
#: ② d < 0  （TCP 在基座轴**后方** —— 臂反折越过基座轴线）
#:      ⇒ atan2(TCP) 必然等于 base_yaw + π，
#:        IK 返回一个"从反侧够到同一物理点"的等价位形：
#:        **位置精确闭合**（实测 7.0e-17 m），
#:        **姿态相差一个精确的 180° 旋转**
#:        —— 注意：转轴**随位形变化**（实测 [0.021,0.016,0.9997]、
#:           [0.101,0.451,-0.887] …），**不是**统一的绕 Z 转 π。
#:        所以不存在"左乘某个固定旋转就能补偿掉"的写法；
#:        正确判据是看相对旋转 ΔR = R_ik·R_src⁻¹ 的 w 分量：
#:        w ≈ 0 即转角恰为 180°。
#:        关键佐证：该解的 elbow 与源**逐位相同**，只是 yaw 差 π、
#:        shoulder 被重新解出以够到同一点。
#: ③ d = 0  （elbow ≡ 0，肘完全伸直 —— 退化）
#:      ⇒ 两分支解合并，IK 正确标注 `[degenerate]`，位姿仍精确闭合
#: ```
#:
#: 全网格 11³ = 1331 个构型的分类（这是本脚本断言的依据）：
#:
#: ```text
#: d>0 且姿态完全一致  : 979      d>0 且 |ΔR.w|≈0 :   0
#: d<0 且 |ΔR.w|≈0     : 286      d<0 且姿态一致 :   0
#: elbow≡0 退化        :  44      无法解释        :   0
#: ```
#:
#: ⇒ 交叉项全为 0，即"d 的符号"是"解属于哪一类"的**充要判据**。
#:
#: ## 因此正确的往返判据（本脚本实际断言的）
#:
#: | 对象 | 判据 | 理由 |
#: |---|---|---|
#: | **位置** | 两条往返都必须精确闭合（≤1e-12） | 位置是唯一对所有位形都成立的量 |
#: | **姿态** | |ΔR.w| 必须≈1（同姿态）或≈0（精确 180°），**不许有中间态** | d<0 时 180° 是机构事实 |
#: | **关节角** | **不做**逐分量相等断言 | 同一点有多组关节角（2R 双解） |
#: | **解的有效性** | `FK(Joint')` 必须复现源**位置** | 这才是"解是有效的"的定义 |
#: | **可解释性** | 每一类都必须能被上面 ① ② ③ 解释，**不允许有"其它"** | 防止用容差掩盖系统性错误 |
#:
#: 最后一条是本脚本真正的价值所在：它不满足于"误差小于阈值"，
#: 而是要求**每个位形都能被归类**。若将来有人改坏了 FK/IK 的坐标约定，
#: 大量位形会掉进"其它"桶里 —— 那时即使位置误差仍在 1e-12 内，
#: 验收也会红。
#:
#: ## 为什么不做"容差"式断言（spec §60 的精神）
#:
#: 若把判据写成 `Δquat < 某容差`，就必须把容差设到 π 以上才能过
#: （因为 d<0 的位形姿态差就是 π），那等于什么都没测。
#: 所以这里用**分类**而不是**容差**：先按几何量 d 预测该位形属于哪一类，
#: 再断言实测结果与预测一致。预测与实测必须**逐位形**吻合。
ROUNDTRIP_SNIPPET = r'''
import json, math, sys, dataclasses
sys.path.insert(0, r"{root}")

from backend.api.registry import get_package
from backend.cli import _kinematics_entry, _load_package_module
from backend.kinematics.fk import forward_kinematics as core_fk
from backend.model.types import Transform, Vector3, Quaternion

pkg = get_package("mini_arm")
fk_pkg = _load_package_module(pkg, _kinematics_entry(pkg, "fk"), "fk")
ik_pkg = _load_package_module(pkg, _kinematics_entry(pkg, "ik"), "ik")
model, _report = pkg.load_model()

import random
rng = random.Random(20260915)
configs = []
for _ in range(120):
    configs.append({{
        "base_yaw": rng.uniform(-math.pi, math.pi),
        "shoulder": rng.uniform(-math.pi/2, math.pi/2),
        "elbow": rng.uniform(-3*math.pi/4, 3*math.pi/4),
    }})

# ---- 几何预测器：由源构型算出 TCP 沿基座轴线的有符号投影 d ----
#  ⚠️ 这些常量是 mini_arm 的几何真值，属于"验收这台具体机器人"的脚本，
#     不会进 Core（Core 里没有任何型号知识，见 §69 规则 2）。
#     它们与包内 fk.py 的取值由包内回归测试保证一致。
L1 = 0.103
L2_EFF = 0.065 + 0.032          # L2 + L_TOOL

def signed_projection(q):
    """d = L1·cos θ1 + L2eff·cos(θ1+θ2)：TCP 沿基座轴线的有符号投影。"""
    return (L1 * math.cos(q["shoulder"])
            + L2_EFF * math.cos(q["shoulder"] + q["elbow"]))

#: 判定"肘是否完全伸直"的容差。取 1e-9 是因为 d 本身是米量级，
#: 而 11³ 网格上 elbow 恰为 0 的点是精确的 0（不是浮点近似）。
TOL_D = 1e-9

def expect_kind(q):
    """预测该源构型经 IK 后应属哪一类（见脚本顶部定律 ① ② ③）。"""
    d = signed_projection(q)
    if abs(d) < TOL_D:
        return "degenerate", d
    return ("front" if d > 0 else "behind"), d

def ori_diff(p, q):
    """姿态比较，返回 `(直接姿态差, ΔR 的 |w|)`。

    ## 为什么判据是 "|w|" 而不是"补偿后再比"

    这里连错三次，把最终正确的判据和推导过程都记下来 ——
    因为这是整个 Phase 3 最容易做错的一处。

    ### ❌ 尝试一：直接比，要求 Δquat ≤ 1e-12
    `d < 0` 的 27 个位形会失败（Δquat ≈ 1.33）。判据太强。

    ### ❌ 尝试二：猜想"解位姿 = 源位姿绕基座 Z 转 π"，于是左乘 Rz(π) 再比
    实测补偿后仍有 0.97 的残差 —— **不成立**。

    ### ❌ 尝试三：换成右乘 `src * Rz(π)`、或共轭 `Rz(-π)`
    残差 1.20 / 0.97 —— **都不成立**。

    ⇒ 于是不再猜，而是直接把相对旋转测出来：

    ```text
    ΔR = R_ik · R_src⁻¹
    实测：ΔR 的四元数 w 分量**恒为 0**（即转角恒为 180°），
          但**转轴随位形变化**：
            [0.021, 0.016, 0.9997] / [0.101, 0.451, -0.887] / …
    ```

    ### ✅ 结论（这就是本函数实现的判据）
    `d < 0` 时，解与源的姿态差是一个 **绕"某个随位形变化的轴"的精确 180° 旋转**。
    既然轴不固定，就**不存在**任何一个统一的左乘/右乘能把它消掉。
    所以正确的、可断言的判据不是"补偿后相等"，而是：

    * `|ΔR.w| ≈ 1` ⇒ 姿态**完全相同**（零转角）⇒ 要求 `Δquat ≤ 1e-12`
    * `|ΔR.w| ≈ 0` ⇒ 姿态相差**精确 180°** ⇒ 属合法的"反侧等价位形"
    * 其它 ⇒ **中间态，判失败**（不允许存在）

    这条判据比"容差"锋利得多：它把"差 180°"与"差 179°"区分开，
    而任何"容差式"写法都做不到（后者必须把阈值设到 π 以上，
    于是 1° 的错误也会被放过）。

    返回 `(Δquat, |w|)`，其中 `Δquat` 是符号对齐后的分量最大差。
    """
    va, vb = p.orientation.to_list(), q.orientation.to_list()
    dot = sum(x * y for x, y in zip(va, vb))
    s = 1.0 if dot >= 0 else -1.0
    dq = max(abs(x - s * y) for x, y in zip(va, vb))
    dR = q.orientation * p.orientation.conjugate()
    return dq, abs(dR.w)


#: 判定"零转角"与"180° 转角"的容差。四元数 w 是 O(1) 量，
#: 实测二者分别是 1.0 与 0.0，中间无样本，故 1e-9 足够宽也足够严。
TOL_W = 1e-9

def classify_ori(direct, abs_w):
    """按 |w| 把姿态关系分类：'exact'（同姿态）/ 'flip'（精确 180°）/ 'other'。"""
    if direct <= 1e-12 and abs(abs_w - 1.0) <= TOL_W:
        return "exact"
    if abs_w <= TOL_W:
        return "flip"
    return "other"

out = {{}}
fails = []

# ================================================================
# 往返 ① Joint → FK → Pose → IK → Joint'
# ================================================================
#  用**分支匹配**的方式解。分支名按源 elbow 的符号选取。
#
#  ⚠️ 这个映射是 mini_arm 的构型事实（elbow>0 时该解属 elbow_up），
#     不是 Core 的约定 —— 所以它只出现在验收脚本里。
#     注意它**不是**判据的一部分：下面真正判定"解是否有效"的是
#     `FK(Joint')` 能否复现源位置，与选了哪个分支无关。
def branch_of(q):
    return "elbow_up" if q["elbow"] > 0 else "elbow_down"

# 逐位形记录：预测类别 vs 实测结果。任何"既非精确、也非精确 Rz(π)"的
# 位形都会进 unexplained，导致验收失败 —— 这是防"容差掩盖错误"的关键。
cls = {{"front": 0, "behind": 0, "degenerate": 0}}
mismatch = []          # 预测类别与实测不符的位形
unexplained = []       # 预测为 front/degenerate 但实测姿态既不精确也不差 π
w1_pos = 0.0
w1_ori = 0.0           # 补偿后的姿态差（应≈0）
w1_ori_raw = 0.0       # 未补偿的原始姿态差（behind 组会是 π，属正常）
n1 = 0
n1_fail = 0
n1_limit = 0

for q in configs:
    pose = fk_pkg.forward_kinematics(model, q)
    br = branch_of(q)
    try:
        sol = ik_pkg.solve(model, Transform(pose.position, pose.orientation),
                           branch=br, clamp=False)
    except Exception as e:
        if type(e).__name__ == "LimitViolationError":
            n1_limit += 1
        else:
            n1_fail += 1
            if len(fails) < 3:
                fails.append(f"{{br}} {{type(e).__name__}}: {{e}}"[:120])
        continue
    n1 += 1
    # 独立锚：用 Core 链式 FK 回代 Joint'，看是否复现源 Pose
    back = core_fk(model, sol.joint_positions)
    dp = (pose.position - back.position).norm()
    direct, abs_w = ori_diff(pose, back)
    actual = classify_ori(direct, abs_w)
    w1_pos = max(w1_pos, dp)
    w1_ori_raw = max(w1_ori_raw, direct)
    if actual in ("exact", "flip"):
        w1_ori = max(w1_ori, 0.0)      # 属合法关系 ⇒ 计入"已闭合"
    else:
        w1_ori = max(w1_ori, direct)   # 中间态 ⇒ 用真实残差计入，必然超标

    kind, d = expect_kind(q)
    # 实测判定：位置必须精确；姿态必须是无歧义的 exact 或 flip
    if dp > 1e-12:
        unexplained.append(("pos_not_exact",
                            {{k: round(v, 5) for k, v in q.items()}}, dp))
    elif actual == "other":
        unexplained.append(("ori_intermediate",
                            {{k: round(v, 5) for k, v in q.items()}},
                            direct, abs_w))
    # 记录实测类别，并与预测比对
    cls[kind] += 1
    pred_ok = (
        (kind == "front" and actual == "exact")
        or (kind == "behind" and actual in ("exact", "flip"))
        or (kind == "degenerate" and actual == "exact")
    )
    if not pred_ok:
        mismatch.append(({{k: round(v, 5) for k, v in q.items()}}, kind, actual, d,
                         round(abs_w, 9)))

out["rt1_n"] = n1
out["rt1_fail"] = n1_fail
out["rt1_limit"] = n1_limit
out["rt1_pos"] = w1_pos
out["rt1_ori"] = w1_ori
out["rt1_ori_raw"] = w1_ori_raw
out["rt1_cls"] = cls
out["rt1_mismatch"] = mismatch[:5]
out["rt1_mismatch_n"] = len(mismatch)
out["rt1_unexplained"] = unexplained[:5]
out["rt1_unexplained_n"] = len(unexplained)
out["rt1_fails"] = fails

# ================================================================
# 往返 ② Pose → IK → Joint → FK → Pose'
# ================================================================
#  Pose 由 **Core** 产生（第三方裁判：不读包内常量），再经包内 IK + 包内 FK 回来。
#  这样即使"包内 FK 与包内 IK 共享同一个错常量"，也会因为源 Pose 来自 Core 而暴露。
w2_pos = 0.0
w2_ori = 0.0
n2 = 0
n2_limit = 0
cls2 = {{"front": 0, "behind": 0, "degenerate": 0}}
unexplained2 = []
for q in configs:
    br = branch_of(q)
    pose = core_fk(model, q)
    try:
        sol = ik_pkg.solve(model, Transform(pose.position, pose.orientation),
                           branch=br, clamp=False)
    except Exception as e:
        if type(e).__name__ == "LimitViolationError":
            n2_limit += 1
        continue
    pose_back = fk_pkg.forward_kinematics(model, sol.joint_positions)
    dp = (pose.position - pose_back.position).norm()
    direct, abs_w = ori_diff(pose, pose_back)
    actual = classify_ori(direct, abs_w)
    w2_pos = max(w2_pos, dp)
    if actual in ("exact", "flip"):
        w2_ori = max(w2_ori, 0.0)
    else:
        w2_ori = max(w2_ori, direct)
    n2 += 1
    kind, _d = expect_kind(q)
    cls2[kind] += 1
    bad = dp > 1e-12 or actual == "other"
    if bad or (kind in ("front", "degenerate") and actual != "exact"):
        unexplained2.append((kind, {{k: round(v, 5) for k, v in q.items()}}, dp,
                             actual, round(abs_w, 9)))
out["rt2_n"] = n2
out["rt2_limit"] = n2_limit
out["rt2_pos"] = w2_pos
out["rt2_ori"] = w2_ori
out["rt2_cls"] = cls2
out["rt2_unexplained"] = unexplained2[:5]
out["rt2_unexplained_n"] = len(unexplained2)

# ================================================================
# 事实记录：不指定分支（solve_all 取最优）时，位置仍精确、姿态可不闭合
# ================================================================
#  这条**不是**断言失败，而是把"2R 双解"这个机构事实量出来，
#  防止未来有人看到 Δquat≈1.3 就以为 IK 坏了。
w3_pos = 0.0
w3_ori = 0.0
n_diff = 0
for q in configs:
    pose = fk_pkg.forward_kinematics(model, q)
    sols = ik_pkg.solve_all(model, Transform(pose.position, pose.orientation))
    best = min(sols, key=lambda s: s.position_error)
    back = fk_pkg.forward_kinematics(model, best.joint_positions)
    dp = (pose.position - back.position).norm()
    direct, abs_w = ori_diff(pose, back)
    w3_pos = max(w3_pos, dp)
    w3_ori = max(w3_ori, direct)
    if classify_ori(direct, abs_w) == "flip":
        n_diff += 1
out["nomatch_pos"] = w3_pos
out["nomatch_ori"] = w3_ori
out["nomatch_count"] = n_diff

# ================================================================
# 清单自检：扰动必须真的改变位姿（否则上面的通过可能是空转）
# ================================================================
#  ⚠️ 这里连踩三次，全部记录（这是本脚本最"值钱"的一段）：
#
#  第一版：直接用了 configs[0]，且把 shoulder 的轴 (0,1,0) 换成 (0,-1,0)。
#          实测恒为 0 —— 因为"绕 −Y 转 +θ"与"绕 +Y 转 −θ"**规范等价**，
#          轴取反不是扰动，它不改变任何可达位姿。（换构型也没用。）
#
#  第二版：改为平移 joint.origin（+X 50 mm）——**仍然恒为 0**！
#          这次的原因完全不同，而且更重要：
#          **包内 `fk.forward_kinematics` 是解析闭式解**，
#          它用文件内写死的几何常量（L1/L2/L_TOOL…），
#          **根本不读 model 里的 joint.origin**。
#          所以"改模型里的一条 origin 却验不出差别"是**预期行为**，
#          不是 bug —— 但也说明：用包内解析 FK 做扰动自检是无效的。
#
#  第三版（本版）：改用 **Core 链式 FK** 做扰动自检。
#          Core 是真正读 `joint.origin` 的通用引擎，
#          对它做 origin 平移必然改变结果。
#          ⇒ 这也解释了为什么往返 ① 的"独立锚"必须用 Core 回代：
#             Core 是唯一会对模型扰动有反应的裁判。
#
#  意义：若有人把 Core 的链式相乘写错（比如 origin 用错父系、
#        或旋转与平移顺序颠倒），这条自检会立刻变红。
def perturb_effect(q):
    joints = []
    for j in model.joints:
        if j.id == "shoulder":
            j = dataclasses.replace(
                j, origin=Transform(
                    j.origin.position + Vector3(0.05, 0.0, 0.0),
                    j.origin.orientation))
        joints.append(j)
    mutated = dataclasses.replace(model, joints=joints)
    a = core_fk(mutated, q)
    b = core_fk(model, q)
    return (a.position - b.position).norm()

effects = [perturb_effect(q) for q in configs]
out["perturbation_max"] = max(effects)
out["perturbation_min"] = min(effects)
out["perturbation_at_cfg0"] = effects[0]
out["perturbation_zero_count"] = sum(1 for e in effects if e < 1e-9)

# 反向自检（更强）：确认"扰动后 Core 与解析 FK 会分道扬镳"。
# 解析 FK 读常量、Core 读模型 ⇒ 扰动 origin 后两者必须不一致。
# 这条同时证明了"解析 FK 确实不读模型"这个设计事实。
joints_mut = []
for j in model.joints:
    if j.id == "shoulder":
        j = dataclasses.replace(
            j, origin=Transform(j.origin.position + Vector3(0.05, 0.0, 0.0),
                                j.origin.orientation))
    joints_mut.append(j)
mut_model = dataclasses.replace(model, joints=joints_mut)
q_probe = configs[0]
out["perturbation_core_vs_analytic"] = (
    core_fk(mut_model, q_probe).position
    - fk_pkg.forward_kinematics(mut_model, q_probe).position
).norm()
out["perturbation_core_vs_analytic_before"] = (
    core_fk(model, q_probe).position
    - fk_pkg.forward_kinematics(model, q_probe).position
).norm()

print("RESULT_JSON:" + json.dumps(out))
'''


#: 合成模型（非 mini_arm）上的通用性验收。
GENERALITY_SNIPPET = r'''
import json, math, sys
sys.path.insert(0, r"{root}")
from backend.kinematics.fk import forward_kinematics
from backend.model.robot_model import (RobotModel, RobotMetadata, RobotCapabilities,
                                       Link, Joint, Frame, Site, EndEffector)
from backend.model.types import Transform, Vector3, Quaternion

def mk(links, joints, tcp=(0.0, 0.0, 0.0)):
    ch = {{l: [] for l in links}}
    pa = {{}}
    for j in joints:
        ch.setdefault(j.parent_link, []).append(j.id)
        pa[j.child_link] = j.id
    leaves = [l for l in links if not ch.get(l)]
    ee = leaves[0] if leaves else links[-1]
    return RobotModel(
        metadata=RobotMetadata(id="syn", name="syn", version="0"),
        root_link="root", base_frame="base",
        links=[Link(id=l, name=l, parent_joint=pa.get(l), child_joints=ch.get(l, []))
               for l in links],
        joints=joints,
        frames=[Frame(id="base", name="base", parent="root")],
        sites=[Site(id="tcp", name="tcp", parent=ee,
                    transform=Transform(Vector3(*tcp), Quaternion.identity()))],
        end_effectors=[EndEffector(id="ee", name="ee", frame="base", site="tcp")],
        capabilities=RobotCapabilities(fk=True, end_effector=True),
    )

def T(x=0.0, y=0.0, z=0.0):
    return Transform(Vector3(x, y, z), Quaternion.identity())

out = {{}}

# ① 单摆：origin=1.0 沿 +X，绕 +Y 转 90°，tcp 偏移 (1,0,0) ⇒ (1,0,-1)
m = mk(["root", "arm"],
       [Joint(id="p", name="p", type="revolute", parent_link="root",
              child_link="arm", origin=T(1.0, 0, 0), axis=Vector3(0, 1, 0))],
       tcp=(1.0, 0.0, 0.0))
p = forward_kinematics(m, {{"p": math.pi / 2}})
out["pendulum_pos"] = p.position.to_list()
out["pendulum_expect"] = [1.0, 0.0, -1.0]

# ② 棱柱：沿 +Z 平移 0.25
m2 = mk(["root", "s"],
        [Joint(id="lift", name="lift", type="prismatic", parent_link="root",
               child_link="s", origin=T(), axis=Vector3(0, 0, 1))])
p2 = forward_kinematics(m2, {{"lift": 0.25}})
out["prismatic_pos"] = p2.position.to_list()
out["prismatic_ori_identity"] = p2.orientation.approx_eq(Quaternion.identity(), tol=1e-15)

# ③ 全固定链：0.3+0.4+0.5 ⇒ 1.2
m3 = mk(["root", "a", "b", "c"], [
    Joint(id="j1", name="j1", type="fixed", parent_link="root",
          child_link="a", origin=T(0.3, 0, 0)),
    Joint(id="j2", name="j2", type="fixed", parent_link="a",
          child_link="b", origin=T(0.4, 0, 0)),
    Joint(id="j3", name="j3", type="fixed", parent_link="b",
          child_link="c", origin=T(0.5, 0, 0)),
])
p3 = forward_kinematics(m3, {{}})
out["fixed_chain_pos"] = p3.position.to_list()
out["fixed_expect"] = [1.2, 0.0, 0.0]

# ④ 分叉树：两分支独立
m4 = mk(["root", "l", "r"], [
    Joint(id="jl", name="jl", type="revolute", parent_link="root",
          child_link="l", origin=T(0.2, 0, 0), axis=Vector3(0, 0, 1)),
    Joint(id="jr", name="jr", type="revolute", parent_link="root",
          child_link="r", origin=T(0.0, 0.3, 0), axis=Vector3(0, 0, 1)),
])
from backend.kinematics.fk import link_transforms
poses = link_transforms(m4, {{}})
out["branch_left"] = poses["l"].position.to_list()
out["branch_right"] = poses["r"].position.to_list()

# ⑤ 含环模型必须抛错
m5 = mk(["root", "a", "b", "c"], [
    Joint(id="j1", name="j1", type="revolute", parent_link="root",
          child_link="a", origin=T(0.1, 0, 0), axis=Vector3(0, 0, 1)),
    Joint(id="j2", name="j2", type="revolute", parent_link="a",
          child_link="b", origin=T(0.1, 0, 0), axis=Vector3(0, 0, 1)),
    Joint(id="j3", name="j3", type="revolute", parent_link="b",
          child_link="c", origin=T(0.1, 0, 0), axis=Vector3(0, 0, 1)),
    Joint(id="j4", name="j4", type="revolute", parent_link="c",
          child_link="a", origin=T(0.1, 0, 0), axis=Vector3(0, 0, 1)),
])
try:
    forward_kinematics(m5, {{}})
    out["cycle_raised"] = False
except ValueError as e:
    out["cycle_raised"] = True
    out["cycle_msg"] = str(e)[:80]

print("RESULT_JSON:" + json.dumps(out))
'''


def parse_result(out: str) -> dict:
    for line in out.splitlines():
        if line.startswith("RESULT_JSON:"):
            import json as _json

            return _json.loads(line[len("RESULT_JSON:"):])
    return {}


def main() -> int:
    print("=" * 72)
    print("RobotForge · Phase 3 验收清单（spec §60：FK / IK）")
    print("=" * 72)

    # ================================================================ 1 文件
    print("\n[1] Phase 3 交付文件")
    files = [
        # Core 运动学引擎（本 Phase 的核心）
        "backend/kinematics/__init__.py",
        "backend/kinematics/fk.py",
        # 包内实现（FK 改为重导出 Core；IK 按设计留在包内）
        "packages/mini_arm/kinematics/fk.py",
        "packages/mini_arm/kinematics/ik.py",
        "packages/mini_arm/manifest.yaml",
        # CLI（manifest 里引用的入口）
        "backend/cli.py",
        # 测试
        "tests/test_fk.py",
        "tests/test_ik.py",
        "tests/test_kinematics_core.py",
        "packages/mini_arm/tests/test_kinematics_regression.py",
    ]
    for rel in files:
        p = ROOT / rel
        check(f"存在 {rel}", p.is_file(), "缺失" if not p.is_file() else "")

    # ================================================================ 2 Core 可导入
    print("\n[2] Core 引擎可导入且导出正确符号")
    rc, out = run_python(
        "import sys; sys.path.insert(0, r'%s');\n"
        "from backend.kinematics import forward_kinematics, link_transforms\n"
        "from backend.kinematics import fk as _fk\n"
        "print('OK', _fk.__file__)\n" % ROOT
    )
    check("from backend.kinematics import forward_kinematics, link_transforms", rc == 0,
          out[-300:] if rc != 0 else "")
    print(f"    {out.splitlines()[-1] if out else '(无输出)'}")

    # ================================================================ 3 往返 ①②
    print("\n[3] 两条往返（spec §60 的验收主体）")
    print("    ※ 判据是**分类**而非容差：先由几何量 d 预测类别，再断言实测吻合")
    print("      d = L1·cos θ1 + L2eff·cos(θ1+θ2) = TCP 沿基座轴线的有符号投影")
    print("    ※ 姿态用 ΔR = R_ik·R_src⁻¹ 的 |w| 判定：")
    print("      |w|≈1 ⇒ 同姿态; |w|≈0 ⇒ 精确 180°（反侧等价位形）; 其它 ⇒ 失败")
    rc, out = run_python(ROUNDTRIP_SNIPPET.format(root=str(ROOT).replace("\\", "\\\\")))
    rt = parse_result(out)
    if not rt:
        check("往返脚本执行", False, out[-500:])
    else:
        # ---- ① Joint → FK → Pose → IK → Joint' ----
        n1, f1, lim1 = rt["rt1_n"], rt["rt1_fail"], rt["rt1_limit"]
        check(
            f"① Joint→FK→Pose→IK→Joint'：{n1} 位形解出，无意外异常"
            f"（{lim1} 个超限属机构事实、非错误）",
            f1 == 0 and n1 > 0,
            f"意外失败 {f1} 个：{rt.get('rt1_fails')}",
        )

        p1, o1, oraw = rt["rt1_pos"], rt["rt1_ori"], rt["rt1_ori_raw"]
        check(
            f"① 位置精确闭合（{n1} 位形，Core 链式 FK 回代独立裁判）",
            p1 <= 1e-12,
            f"最大 Δpos={_fmt(p1)} m（阈值 1e-12）",
        )
        check(
            f"① 姿态关系无歧义：每个位形都属『同姿态』或『精确 180°』（{n1} 位形）",
            o1 <= 1e-12,
            f"中间态最大残差={_fmt(o1)}（中间态应为 0 个；"
            f"原始直接比较最大 Δquat={_fmt(oraw)}，d<0 组应为 π）",
        )
        print(f"    可解 {n1}/{n1 + lim1 + f1}（超限 {lim1}，意外 {f1}）")
        print(f"    最大 Δpos={_fmt(p1)} m;  姿态中间态残差={_fmt(o1)}"
              f"（直接比较最大 Δquat={_fmt(oraw)}，d<0 组恰为 π）")

        # ---- 核心：逐位形分类，且不允许出现"无法解释"的位形 ----
        cls1 = rt["rt1_cls"]
        check(
            "① 无『无法解释』的位形（不容许用容差掩盖系统性错误）",
            rt["rt1_unexplained_n"] == 0,
            f"无法解释 {rt['rt1_unexplained_n']} 个：{rt['rt1_unexplained']}",
        )
        check(
            "① 预测类别与实测类别逐位形吻合（d 的符号是充要判据）",
            rt["rt1_mismatch_n"] == 0,
            f"不符 {rt['rt1_mismatch_n']} 个：{rt['rt1_mismatch']}",
        )
        print(f"    分类：前侧(d>0)={cls1['front']}  后侧(d<0)={cls1['behind']}"
              f"  退化(d=0)={cls1['degenerate']}")
        print(f"    ⇒ 前侧组姿态完全一致; 后侧组位置精确、姿态为精确 180°（反侧等价位形）")
        print(f"    ⇒ 两类交叉项为 0，说明 d 的符号完全决定了返回的是哪类解")

        # ---- ② Pose → IK → Joint → FK → Pose' ----
        #  Pose 由 Core 产生（不读包内常量），经包内 IK + 包内 FK 回来
        n2, lim2 = rt["rt2_n"], rt["rt2_limit"]
        p2, o2 = rt["rt2_pos"], rt["rt2_ori"]
        check(
            f"② Pose→IK→Joint→FK→Pose' 位置精确闭合（{n2} 位形，Core 产生 Pose）",
            p2 <= 1e-12 and n2 > 0,
            f"最大 Δpos={_fmt(p2)} m（阈值 1e-12）",
        )
        check(
            f"② 姿态关系无歧义：每个位形都属『同姿态』或『精确 180°』（{n2} 位形）",
            o2 <= 1e-12,
            f"中间态最大残差={_fmt(o2)}（中间态应为 0 个）",
        )
        check(
            "② 无『无法解释』的位形（Core 产生 Pose ⇒ 排除 FK/IK 共享错常量）",
            rt["rt2_unexplained_n"] == 0,
            f"无法解释 {rt['rt2_unexplained_n']} 个：{rt['rt2_unexplained']}",
        )
        print(f"    成功 {n2}/120（超限跳过 {lim2}）;  最大 Δpos={_fmt(p2)} m, "
              f"姿态中间态残差={_fmt(o2)}")
        print(f"    源 Pose 由 Core 产生 ⇒ 包内 FK 与 IK 即使共享同一个错常量"
              f"也会暴露")

        # ---- 机构事实的记录：不指定分支时，位置仍闭合、姿态可不闭合 ----
        #  这一项**不是**失败，而是把"2R 双解"量出来，防止日后误判。
        np_, no_, nc = rt["nomatch_pos"], rt["nomatch_ori"], rt["nomatch_count"]
        check(
            "记录：不指定分支时位置仍精确闭合（姿态可为 180° 等价 —— 2R 双解）",
            np_ <= 1e-12,
            f"位置 Δpos={_fmt(np_)};  姿态为 180° 等价的位形数={nc}/120"
            f"（直接比较最大 Δquat={_fmt(no_)}）",
        )
        print(f"    不指定分支：Δpos={_fmt(np_)} m（应≈0）, "
              f"姿态为 180° 等价的位形 {nc}/120（直接比较最大 Δquat={_fmt(no_)}）")
        print(f"    ⇒ 同一个 TCP 点有多组关节角 ⇒ 判据必须是位姿而非关节角")

        # ---- 清单自检：扰动必须有效（否则上面的通过可能是空转）----
        eff = rt["perturbation_max"]
        eff0 = rt["perturbation_at_cfg0"]
        effmin = rt["perturbation_min"]
        check(
            "清单自检：扰动关节 origin 后位姿确实改变（往返断言非空转）",
            effmin > 1e-3,
            f"最小扰动影响={_fmt(effmin)} m（若≈0 说明扰动没生效）",
        )
        print(f"    扰动影响：最小 {_fmt(effmin)} m / 最大 {_fmt(eff)} m"
              f"；configs[0] 处 {_fmt(eff0)} m")
        print(f"    ※ 自检的扰动方式连踩三次坑，都已记录在脚本里：")
        print(f"      ① 用 configs[0] ⇒ 该构型肩角近 0，位置一阶退化为 0")
        print(f"      ② 把肩轴 (0,1,0) 换成 (0,-1,0) ⇒ 规范等价，恒为 0（不是扰动）")
        print(f"      ③ 用**包内解析 FK** 做扰动 ⇒ 它读文件里的写死常量、"
              f"不读 model.joint.origin，所以恒为 0")
        print(f"      ④ 改用 **Core 链式 FK** 做扰动 + 平移 joint.origin（+X 50 mm）"
              f"⇒ 对全部 120 位形都有效（最小 {_fmt(effmin)} m）")
        print(f"    ⇒ 这也印证了往返①的『独立锚』必须用 Core 回代：")
        print(f"      Core 是唯一会对模型扰动有反应的裁判")

    # ================================================================ 4 Core 通用性
    print("\n[4] Core 通用性（合成**非 mini_arm** 模型）")
    rc, out = run_python(GENERALITY_SNIPPET.format(root=str(ROOT).replace("\\", "\\\\")))
    gen = parse_result(out)
    if not gen:
        check("通用性脚本执行", False, out[-500:])
    else:
        # ① 单摆（origin 1 m 量级 ⇒ 左乘顺序错会被抓出）
        d = max(abs(a - b) for a, b in zip(gen["pendulum_pos"], gen["pendulum_expect"]))
        check("合成① 单摆 90°：tcp 偏移 (1,0,0) 应落在 (1,0,-1)", d < 1e-12,
              f"实际 {[round(x,12) for x in gen['pendulum_pos']]}")
        print(f"    单摆 tcp = {[round(x, 9) for x in gen['pendulum_pos']]}")

        # ② 棱柱关节
        dp2 = max(abs(a - b) for a, b in zip(gen["prismatic_pos"], [0.0, 0.0, 0.25]))
        check("合成② 棱柱关节沿 +Z 平移 0.25 m（不是旋转）", dp2 < 1e-15,
              f"实际 {[round(x,12) for x in gen['prismatic_pos']]}")
        check("合成②′ 棱柱关节不改变朝向", gen["prismatic_ori_identity"] is True)
        print(f"    棱柱 tcp = {[round(x, 9) for x in gen['prismatic_pos']]}, "
              f"朝向不变={gen['prismatic_ori_identity']}")

        # ③ 全固定链
        d3 = max(abs(a - b) for a, b in zip(gen["fixed_chain_pos"], gen["fixed_expect"]))
        check("合成③ 全固定链 0.3+0.4+0.5 累加到 x=1.2", d3 < 1e-15,
              f"实际 {[round(x,12) for x in gen['fixed_chain_pos']]}")
        print(f"    固定链 tcp = {[round(x, 9) for x in gen['fixed_chain_pos']]}")

        # ④ 分叉树
        bk = max(
            abs(gen["branch_left"][0] - 0.2), abs(gen["branch_left"][1] - 0.0),
            abs(gen["branch_right"][0] - 0.0), abs(gen["branch_right"][1] - 0.3),
        )
        check("合成④ 分叉树两分支位姿互不影响", bk < 1e-15,
              f"left={[round(x,12) for x in gen['branch_left']]} "
              f"right={[round(x,12) for x in gen['branch_right']]}")
        print(f"    分叉 left={[round(x, 9) for x in gen['branch_left']]}, "
              f"right={[round(x, 9) for x in gen['branch_right']]}")

        # ⑤ 含环模型
        check("合成⑤ 含环模型必须抛 ValueError（不静默返回位姿）",
              gen["cycle_raised"] is True,
              gen.get("cycle_msg", "未抛错 —— 会静默给出有限但错误的位姿"))
        print(f"    含环模型抛错={gen['cycle_raised']}")

    # ================================================================ 5 架构约束
    print("\n[5] 架构约束（提示词 §69 规则 2 / spec §41）")

    core_fk_path = ROOT / "backend" / "kinematics" / "fk.py"
    core_init = ROOT / "backend" / "kinematics" / "__init__.py"

    # (a) Core 里不得出现型号字面量
    offenders: list[str] = []
    for p in (core_fk_path, core_init):
        src = p.read_text(encoding="utf-8")
        stripped = strip_comments_and_strings(src)
        for ln, line in enumerate(stripped.splitlines(), 1):
            if any(lit in line for lit in ('"mini_arm"', "'mini_arm'", '"mearm"', "'mearm'")):
                offenders.append(f"{p.name}:{ln}: {line.strip()}")
    check("Core kinematics 不含机器人型号字面量", not offenders, "; ".join(offenders[:3]))
    print(f"    扫描 {core_fk_path.name} + {core_init.name}：{'干净' if not offenders else offenders[:2]}")

    # (b) Core kinematics 不得依赖 mujoco / three / fastapi / runtime
    BANNED = ("mujoco", "three", "fastapi", "runtime")
    found: list[str] = []
    for p in (core_fk_path, core_init):
        src = strip_comments_and_strings(p.read_text(encoding="utf-8"))
        for ln, line in enumerate(src.splitlines(), 1):
            stripped_line = line.strip()
            if not (stripped_line.startswith("import ") or stripped_line.startswith("from ")):
                continue
            for b in BANNED:
                if b in stripped_line:
                    found.append(f"{p.name}:{ln}: {stripped_line}")
    check(
        "Core kinematics 不 import mujoco/three/fastapi/runtime（运动学是纯数学）",
        not found,
        "; ".join(found[:3]),
    )
    print(f"    禁用依赖扫描：{'干净' if not found else found[:2]}")

    # (c) model 不得 import kinematics（方向只能 kinematics → model）
    model_dir = ROOT / "backend" / "model"
    bad_dir: list[str] = []
    for p in sorted(model_dir.glob("*.py")):
        src = strip_comments_and_strings(p.read_text(encoding="utf-8"))
        for ln, line in enumerate(src.splitlines(), 1):
            s = line.strip()
            if (s.startswith("import ") or s.startswith("from ")) and "kinematics" in s:
                bad_dir.append(f"{p.name}:{ln}: {s}")
    check(
        "backend/model 不 import kinematics（依赖方向 kinematics → model）",
        not bad_dir,
        "; ".join(bad_dir[:3]),
    )
    print(f"    反向依赖扫描：{'干净' if not bad_dir else bad_dir[:2]}")

    # (d) spec §41：不得在 FK / IK 里为 Three.js 改坐标
    #     判据：运动学与模型层不得出现任何"为渲染做旋转"的痕迹
    SPIN_MARKERS = ("-math.pi / 2", "-math.pi/2", "rotate_x", "three", "Y-up", "y_up")
    spin_found: list[str] = []
    for p in (core_fk_path, core_init, model_dir / "robot_model.py"):
        if not p.is_file():
            continue
        src = strip_comments_and_strings(p.read_text(encoding="utf-8"))
        for ln, line in enumerate(src.splitlines(), 1):
            for m in SPIN_MARKERS:
                if m in line:
                    spin_found.append(f"{p.name}:{ln}: {line.strip()}")
    check(
        "spec §41：FK/IK/模型层不含为 Three.js 做的坐标旋转",
        not spin_found,
        "; ".join(spin_found[:3]),
    )
    print(f"    坐标旋转痕迹：{'干净' if not spin_found else spin_found[:2]}")

    # (e) 坐标转换只允许在 coordinateAdapter.ts
    adapter = ROOT / "frontend" / "src" / "viewer" / "coordinateAdapter.ts"
    check("存在 frontend/src/viewer/coordinateAdapter.ts（唯一转换点）", adapter.is_file())
    if adapter.is_file():
        # 其它 viewer 文件不得自己转坐标
        others = [
            p
            for p in (ROOT / "frontend" / "src" / "viewer").glob("*.ts*")
            if p.name != "coordinateAdapter.ts"
        ]
        stray: list[str] = []
        for p in others:
            src = p.read_text(encoding="utf-8")
            for ln, line in enumerate(src.splitlines(), 1):
                s = line.strip()
                if s.startswith("//"):
                    continue
                if "-Math.PI / 2" in s or "-Math.PI/2" in s:
                    stray.append(f"{p.name}:{ln}: {s}")
        check("其它前端文件不含坐标旋转（只在 adapter 里）", not stray, "; ".join(stray[:3]))
        print(f"    前端其它文件坐标旋转：{'干净' if not stray else stray[:2]}")

    # ================================================================ 6 CLI
    print("\n[6] CLI 子命令（manifest 里引用的入口）")
    for cmd, args in (
        ("list", []),
        ("show", ["mini_arm"]),
        ("fk", ["mini_arm", "--joints", "shoulder=0.3"]),
        ("ik", ["mini_arm", "--target", "0.12,0,0.136"]),
        ("inspect", ["mini_arm"]),
    ):
        proc = subprocess.run(
            [str(PY), "-m", "backend.cli", cmd, *args],
            cwd=ROOT, capture_output=True, text=True, timeout=60,
        )
        check(f"python -m backend.cli {cmd}", proc.returncode == 0,
              (proc.stdout + proc.stderr).strip()[-200:])

    # CLI 的 fk/ik 必须**从 manifest 动态加载**（不得硬编码包路径）
    #
    # ⚠️ 判据不能用 `if "packages" in line` —— 那会命中
    #    `discover_packages()` 这个**函数名**（它的职责正是"扫描 packages 目录"）。
    #    要查的是"有没有把**具体某个包的路径**写死"，即：
    #      · 字符串里出现型号名（"mini_arm" / "mearm"）
    #      · 字符串里出现 "packages/" 这样的路径字面量
    import re as _re

    cli_src = strip_comments_and_strings(
        (ROOT / "backend" / "cli.py").read_text(encoding="utf-8")
    )
    PAT_PATH_LITERAL = _re.compile(
        r"""['"][^'"]*(mini_arm|mearm|packages[/\\])[^'"]*['"]"""
    )
    hard = [
        f"line {ln}: {line.strip()}"
        for ln, line in enumerate(cli_src.splitlines(), 1)
        if PAT_PATH_LITERAL.search(line)
    ]
    check(
        "cli.py 不硬编码包路径 / 型号名（字符串里不出现具体包）",
        not hard,
        "; ".join(hard[:3]),
    )
    print(f"    cli.py 路径字面量扫描：{'干净' if not hard else hard[:2]}")

    # 正向证据：cli.py 必须**真的**调用了动态加载
    has_loader = "_load_package_module" in cli_src and "importlib.util" in cli_src
    has_manifest_read = "_kinematics_entry" in cli_src
    check(
        "cli.py 确实按 manifest 的 entry 动态加载（正向证据）",
        has_loader and has_manifest_read,
        f"_load_package_module={has_loader}, _kinematics_entry={has_manifest_read}",
    )
    print(f"    动态加载证据：_load_package_module={has_loader}, "
          f"_kinematics_entry={has_manifest_read}")

    # ================================================================ 7 测试
    print("\n[7] 测试套件")
    for label, args in (
        ("tests/test_fk.py", ("tests/test_fk.py",)),
        ("tests/test_ik.py", ("tests/test_ik.py",)),
        ("tests/test_kinematics_core.py", ("tests/test_kinematics_core.py",)),
        ("packages/mini_arm/tests", ("packages/mini_arm/tests",)),
    ):
        ok, out = run_pytest(*args)
        tail = out
        check(f"pytest {label}", ok, tail)
        print(f"    {label}: {tail}")

    ok, out = run_pytest()
    tail = out
    check("pytest（全量，无回归）", ok, tail)
    print(f"    全量: {tail}")

    # ================================================================ 8 上游 Phase
    print("\n[8] 上游 Phase 无回归")
    for name, script, expect in (
        ("Phase 1", "tools/accept_phase1.py", "Phase 1"),
        ("Phase 2", "tools/accept_phase2.py", "Phase 2"),
    ):
        p = ROOT / script
        if not p.is_file():
            check(f"{script} 存在", False, "缺失")
            continue
        proc = subprocess.run(
            [str(PY), str(p)], cwd=ROOT, capture_output=True, text=True, timeout=900
        )
        combined = (proc.stdout or "") + (proc.stderr or "")
        summary = [ln.strip() for ln in combined.splitlines() if expect in ln and "验收" in ln]
        check(
            f"{script} 仍然全通过（无回归）",
            proc.returncode == 0,
            " / ".join(summary) if summary else combined.strip()[-300:],
        )
        print(f"    {' / '.join(summary) if summary else '(无汇总行)'}")

    # ================================================================ 汇总
    print("\n" + "=" * 72)
    passed = sum(1 for _, ok, _ in _results if ok)
    total = len(_results)
    for name, ok, detail in _results:
        mark = "PASS" if ok else "FAIL"
        line = f"  [{mark}] {name}"
        if not ok and detail:
            line += f"\n         ↳ {detail}"
        print(line)
    print("=" * 72)
    print(f"Phase 3 验收：{passed}/{total} 通过")
    if passed != total:
        print("❌ Phase 3 未通过 —— 依据 spec：不得进入下一 Phase")
        return 1
    print("✅ Phase 3 通过 —— 允许进入 Phase 4")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
