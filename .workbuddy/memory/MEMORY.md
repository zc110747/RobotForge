# RobotForge · 项目长期约定（**索引 + 铁律**）

> ⚠️ 本文件刻意保持精简（每次会话整份注入，超长会被**截断**）。
> 铁律与结论在此；详细案例、踩坑过程、API 契约全文见：
> - `.workbuddy/memory/CONTRACTS.md` —— 各 Phase 契约 + API 契约 + 踩坑全集
> - `.workbuddy/memory/YYYY-MM-DD.md` —— 逐日工作日志
> - `.workbuddy/skills/robotforge-phase-gate/SKILL.md` —— Phase Gate 方法论

## 项目定位
按 spec `docs/1.protometer.md`（76 节）构建。`RobotModel` 是**唯一规范内部表示**；
只允许原生 MJCF；Core/Frontend **不得出现机器人型号分支**（§69 规则 2）。
坐标 `+X 前 / +Y 左 / +Z 上`（右手系，`X×Y=Z`），SI 单位，四元数 `[x,y,z,w]`。
**§41：不得在 FK/IK 里为 Three.js 改坐标** —— 唯一转换点
`frontend/src/viewer/coordinateAdapter.ts`。

## 铁律：Phase Gate
> **当前 Phase 未通过：不得进入下一 Phase。**
每 Phase 收尾跑 `tools/accept_phaseN.py` 全绿，数字必须显式报出。

## 当前基线（2026-09-15，Phase 8 收口）
```text
pytest -q → 488 passed ｜ pytest packages/mini_arm/tests → 34 passed
accept_phase1 → 32/32 ｜ 2 → 56/56 ｜ 3 → 47/47 ｜ 4 → 65/65 ｜ 5 → 67/67
accept_phase7 → 84/84 ｜ accept_phase8 → 96/96（后两者内含全部上游，约 12 min）
```
Phase 0 ✅ / 1 ✅ / 2 ✅ / 3 ✅ / 4 ✅ / 5 ✅ / 7 ✅ / 8 ✅（**spec 无 Phase 6**）

## 依赖方向铁律（不可反转）
```text
runtime    → model / kinematics / api.registry
kinematics → model          （纯数学）
model      ← 不得 import kinematics / runtime
```
Core `kinematics` 不得 import `mujoco`/`three`/`fastapi`/`runtime`；
`runtime` 不得 import `fastapi`/`mujoco`/`starlette`；
`runtime/backend.py` 不得 import `api.registry`。

## 五条"别改回去"的边界（详见 CONTRACTS.md）
```text
FK/IK 边界      FK 进 Core（通用链乘）；IK 留包内（闭式解依赖机构）
扩展点          分派键只有一处真值源：register(loader) 读 loader.format_name
                重复格式名 ⇒ 抛（不静默覆盖）；资产注册表 v0.1 默认 = []
Runtime         Backend 必须真的制造 State ≠ Command；必须由注入工厂产生
                end_effector_pose = None ≠ Transform.identity()
                Runtime 绑 lifespan，不是模块级单例
仿真层          关节序唯一来源 model.mobile_joint_ids()（start() 里断言已排序）
                限幅夹关节 range，**不是** actuator ctrlrange（超 ctrlrange 静默丢）
                get_state() 末端位姿用 Core FK，**不读** site_xpos（否则自我循环）
                DEFAULT_STEP_SECONDS=0.02 与 MJCF timestep **故意无关**
                §40/§35 四元数换算只允许出现在 mujoco_backend.py（两方向都留）
工程约定        packages/ 下**刻意没有 __init__.py**（不可跨包 import）
                包内模块按绝对路径加载 + **必须** sys.modules[name] = mod
                CLI 一律从 manifest.kinematics.<tag>.entry 动态加载
```

## 判据写法铁律（写测试前必读，**跨 Phase 反复验证**）
1. **分类优于容差**：往返验收不写 `Δ<TOL`，写"先由机制预测类别，再断言实测吻合，
   且不允许出现无法解释的类别"。（容差必须覆盖最坏情况 ⇒ 放过系统性小错）
2. **写不变量，不写期望终值**：串联路径（限幅→限速→目标）的判据要落在各自独立
   可观测点上。⇒ 判据描述"什么不该变"，不是"我预期变成什么"。
3. **判据不能与被验对象语义自相矛盾**：写完先问"这个表达式在什么情况下会是假？"
   答不出 ⇒ 恒真/恒假。期望值必须与**被测函数的输入/输出约定**逐字对齐
   （明文写出 `# 入参是 [w,x,y,z]` 再推，别凭记忆）。
   配套：**反例注射必须实测**（故意破坏被测对象，断言必须变红）。
4. **容差里命名残余误差来源**：纯运动学用机器精度（1e-12~1e-9）；物理量用 5e-3
   并注释"这个容差在承担什么"。阈值定到机器精度是**唯一**能发现"派生量时序错位"
   的手段（§4.14，实测 5.5e-08 m）。

## 验收脚本约定
- 每项独立可判，失败给"期望 vs 实际"；不依赖网络；负向测试自行清理。
- **必须后台跑**（沙箱给长命令 SIGTERM ⇒ 前台得 0 字节日志）：
  `run_in_background=true` + `-u` + 重定向到文件。
- 源码扫描用 `tokenize` **剥离注释与字符串**（正则处理不了三引号/f-string），
  且**必须配扫描器元测试**（否则"从没匹配到"也全绿）。
- 扰动自检先实测两件事：(a) 该"扰动"语义上真是扰动吗？(b) 被测函数真读这个量吗？
- 子进程结果解析用 `json.JSONDecoder().raw_decode()`，**不要**字符串切片
  （stderr 会粘在 JSON 上 ⇒ "Extra data"）。

## 环境坑（Windows / MSYS 沙箱）
```text
nohup ... &                    ✗ 只活到本次工具调用结束 → 用 run_in_background
cp x /tmp/x                    ✗ Permission denied → 放 .workbuddy/scratch/
grep -oE '...'                 ✗ "-u系统找不到指定的文件" → 用 Grep 工具
curl -o /dev/null -w 连跑      ✗ Exit 23 中断链 → 逐条跑或落文件
bash x.sh                      ✗ 解析到 System32\bash.exe（= WSL 存根）被沙箱拦，
                                 症状是 **exit=0 + 乱码**且无 trace
                                 → 用 `/usr/bin/bash x.sh` 或 `sh`
npm shim                       ✗ uname 判成 Linux → 调 wsl.exe 被拦
                                 → 用 tools/npm.sh
npm --prefix <dir> install     ✗ 不生效 → **先 cd 到目标目录**再跑
node versions/current          ✗ **不是目录**，是 9 字节普通文件（内容=版本名）
                                 → 挑版本目录必须 `[ -d "$d" ]`
heredoc 写非 ASCII / # 内容     ✗ 实测会**前置**而非追加 → 用 Write 工具
```

## 协作边界
- **每个 Phase 验收 PASS 后 Agent 自动 `git commit`**；**不执行 `git push`**。
- 严禁在任何记忆/日志/文档里记录 token、凭据、密钥。

## git 卫生（别改回去）
- `origin = https://github.com/zc110747/RobotForge.git`。
- **`node_modules/` 与 `.venv/` 必须忽略**（曾有 8211 个文件来自
  `frontend/node_modules`；忽略后待跟踪 8303 → 88）。同时忽略 `dist/`、`.vite/`、
  `*.tsbuildinfo`、`build/`、`__pycache__/`、`MUJOCO_LOG.TXT`。
- `.workbuddy/` **部分入库**：`memory/` 与 `skills/` 要提交，`scratch/` 忽略。
- 提交前 `git status --porcelain --untracked-files=all | wc -l` 核对文件数。
