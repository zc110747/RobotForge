# RobotForge · 契约与踩坑全集

> 本文件是 `MEMORY.md` 的**详细展开**。`MEMORY.md` 保持精简（避免注入截断），
> 需要具体细节时读这里。按主题索引。

---

## 1. 扩展点契约（Phase 7 / 8 决定，**别改回去**）

```text
模型：RobotModelLoader.format_name → LoaderRegistry → RobotModel
资产：GeometryRef.asset → GeometryAssetRegistry → AssetLoader → GeometryAsset
```

- **分派键只有一处真值源**：`register(loader)` 读 `loader.format_name`，
  **不**由调用方传名（传参会让注册名与 loader 自称的名字可能不一致）。
- **重复格式名 ⇒ 抛**（`LoaderRegistrationError` / `AssetLoaderRegistrationError`）。
  静默覆盖会让"注册没生效"表现成"加载出来的模型不对"。
- **`UnknownFormatError` 继承 `LoaderError`**（API 层统一按 500 处理），
  但独立类型以便测试精确断言"是因为没 loader，不是解析失败"。
  装配类错误（`LoaderRegistrationError`）**不**继承 `LoaderError`。
- **两个注册表是两个类、两套生命周期**：
  `LoaderRegistry`（整模型 / 加载期一次 / 格式名）vs
  `GeometryAssetRegistry`（单个资源 / 按需多次 / **扩展名**）。
  **缺 asset 不得阻止机器人启动**（§69）。
- `GeometryAsset.data: Any` + `kind`（mesh/uri/native）；`to_dict()` **不含** data（§24）。
- `resolve(None)` / `resolve("")` → `None`，**不抛**（v0.1 原生 geom 的**正常路径**）；
  "有引用但没人能加载"才抛 `UnsupportedAssetError`。
- **v0.1 默认状态**：模型注册表 = `["mjcf"]`；**资产注册表 = `[]`（设计如此）**。
- 模型注册表**懒装配**（模块级常量会在 import 时拖进 mujoco，违反 §69）。
- `api/registry.py` 的 `load_model()` 是**唯一**加载分派点；
  `_REGISTRY = [get_loader_registry()]` + `use_loader_registry()` 供测试注入/还原。

### 踩坑：asset 冒烟测试
- `resolve()` 把**整条引用串**交给 loader（asset 可能是 `meshes/wheel.stl`），
  显示名由 loader 决定 ⇒ 冒烟测试要传**能读到的路径**，否则抓到的是
  "文件不存在"而非"分派没生效"。
- `Path(".stl").suffix` 是**空串**（pathlib 把前导点当隐藏文件名标记）
  ⇒ `resolve()` 需要 后缀 → 手动切点 → 整串即扩展名 三级回退。

---

## 2. 目录与工程约定

```text
app 层：backend/{model,loaders,kinematics,runtime,api,simulation,cli.py}
机器人包：packages/<robot>/（manifest.yaml + model/*.xml + kinematics/ + tests/）
        ⚠️ packages/ 下**刻意没有 __init__.py** —— 包不能被跨包 import
第三方：third_party 或包内
```

- 包内模块按**绝对文件路径**加载（`importlib.util.spec_from_file_location`）
  且**必须** `sys.modules[name] = mod`（Python 3.13 `@dataclass` 需反查模块）。
- CLI 等入口一律从 `manifest.kinematics.<tag>.entry` **动态加载**，
  不得硬编码包路径或型号名。
- `manifest.yaml` 只放身份/能力/指针（坐标 `robotforge/right/x/y/z`、单位 SI）。

---

## 3. FK / IK 边界（Phase 3 决定，**别改回去**）

```text
FK  ✅ 提升为 Core 引擎   backend/kinematics/fk.py（通用链式相乘）
IK  ✅ 留在包内           packages/mini_arm/kinematics/ik.py
```

理由：FK 沿 Link-Joint 链乘 Transform，对任何 Tree 成立；
IK 闭式解**依赖机构**（2R 余弦定理、atan2 偏航、肘侧判别），
"通用 IK" 只能退化成数值迭代 ⇒ 引入容差 ⇒ 削弱 v0.1 的"精确往返"验收。
`manifest.yaml` 用 `kinematics.ik.type: package` 编码该边界。

包内 `fk.py` 结构：
- `forward_kinematics` —— 解析闭式解，**几何用文件内写死的常量**
  （`BASE_HEIGHT/SHOULDER_OFFSET/L1/L2/L_TOOL`），只从 model 取旋转轴。
- `forward_kinematics_generic` —— **Core 的重导出**（同一对象，`is` 断言过）。

⚠️ 后果：**改 model 的几何时解析 FK 输出不变** ⇒ 做扰动自检必须用 Core。
要验证"IK 自检真的用了传入的 model"，判据必须落在**朝向**上。

---

## 4. Runtime 层契约（Phase 4 决定，**别改回去**）

```text
RobotCommand  = Desired（robot / joint_targets / timestamp）
RobotState    = Actual （joint_positions / joint_velocities /
                         end_effector_pose / status / timestamp）
```

- **必须由 Backend 真的制造出 State ≠ Command**。`MockBackend` 故意
  有状态 + 限速（0.35 rad/步）+ 限幅（记录在 `.clamped`）。
  若改成原样回显，§61 的验收会"结构性通过、信息上一无所有"。
- Backend **必须**由注入的工厂产生（`backend_factory`），否则 §69 规则 10 作废。
- Runtime 绑在 FastAPI **lifespan** 上，不是模块级单例；WS 与 REST 共用同一实例。
- `end_effector_pose = None` ≠ `Transform.identity()`：前者"不知道"，后者是位姿。
- WS 命令两个名字都收（`joint_command` + `robot_command`）。
- JSON 按 RFC 8259 收严（`NaN`/`Infinity` 必须拒绝）。
- WS 帧里**只能有物理量**，不得有渲染信息（§49）。
- `RobotError` 类名带下划线：`RuntimeError_`（避免遮蔽内建）。
- 详见 `docs/runtime.md`。

---

## 5. 仿真层契约（Phase 5 决定，**别改回去**）

```text
backend/simulation/simulation_backend.py   引擎无关：关节序 / 限幅 / 拼 State
backend/simulation/mujoco_backend.py       引擎特定：积分 / 接触 / 执行器动力学
```

- **关节序只有一个定义**：`model.mobile_joint_ids()`。
  命令展开、状态拼装、与引擎 `qposadr` 对齐全部引用它；`start()` 里
  断言 `qpos_addr == sorted(qpos_addr)`，顺序错立刻报错。
- **限幅夹的是关节 `range`，不是 actuator `ctrlrange`**。
  MuJoCo 对超 `ctrlrange` 的 `ctrl` 是**静默丢弃（保留旧值）**，
  按 ctrlrange 夹会让"命令超限"表现成"机器人完全不动"且无任何报错。
- `limits is None` 或 `has_position_bounds()` 为假 ⇒ **不夹**
  （夹到某个默认 ±π 是伪造一个模型没声明的约束）。
- `get_state()` 的末端位姿用 **Core FK**，**不读** MuJoCo 的 `site_xpos`
  ⇒ 让两者成为互相独立的裁判（否则 §62"FK 与 Simulation 一致"是同义反复）。
- `DEFAULT_STEP_SECONDS = 0.02` 与 MJCF 的 `timestep` **故意无关**，
  比值 = 子步数；**永远不要改 MJCF 的 timestep** 来对齐步长。
- **§40/§35 四元数边界换算只允许出现在 `mujoco_backend.py`**：
  `mj_quat_to_xyzw` / `xyzw_to_mj_quat`，两个方向都留（单向=当初出 bug 的原因）。
  验收脚本用正则扫描全 `backend/` 断言只有这一个文件。
- 由 `mujoco_backend_factory(model)` 注入 `create_app(backend_factory=...)`，
  Runtime / WS / 路由**一行不改**。

### MJCF 上两处必要的建模修正（mini_arm）

```text
armature="0.01"（三个关节全加）
  位置执行器用 kp 直接当刚度，ω=sqrt(kp/I)；近端关节零位形惯量极小
  ⇒ ω·dt ≈ 1.55 > 2 的显式积分稳定界 ⇒ **1 步后 QACC/QVEL 变 NaN**
  加 armature 后 I ≈ 1e-2，ω·dt ≈ 0.155（约 10 倍余量）
  ⇒ 这是**建模修正**，不是数值补丁（改 kp 只会移动问题）

<contact><exclude> 8 对
  shoulder 命令 +1.5708（正好是 range 上限）只停在 0.9367 ⇒ 关节到不了声明的限位
  根因：base_column 视觉圆柱与上臂 capsule 自碰撞（dist=-0.0013）
  排除自碰撞是**正确**的：capabilities 从未声明 collision；
  几何是为好看选的、从未做过干涉检查；MuJoCo 只过滤相邻不对，不过滤隔代
  ⇒ 将来若真要做碰撞检测，删掉这个块并重新设计几何
```

---

## 6. 判据写法铁律：完整案例

### 铁律一案例：mini_arm 往返定律（全网格 11³ 双向验证，交叉项为 0）

```text
d(θ1,θ2) = L1·cos θ1 + L2eff·cos(θ1+θ2)   （TCP 沿基座轴的有符号投影）
d > 0 ⇒ 同分支解，位置与姿态都精确闭合
d < 0 ⇒ 位置精确；姿态差一个精确 180° 旋转（转轴随位形变化，无固定补偿）
d = 0 ⇒ elbow≡0 退化，两分支合并，位姿仍精确闭合

姿态判据：ΔR = R_ik·R_src⁻¹
  |ΔR.w| ≈ 1 ⇒ 同姿态（要求 Δquat ≤ 1e-12）
  |ΔR.w| ≈ 0 ⇒ 精确 180°，合法等价位形
  其它        ⇒ 中间态，判失败
```

### 铁律二案例：Phase 4 实测三例（引擎全对，判据全错）

```text
① 限幅：命令 99.0 后 1 步状态 = 0.35（限速），不是 position_max
   ⇒ 判据要看 backend.clamped（夹紧后的目标），不是状态
② 并发 Lock：两次串行 step 的**起点不同** ⇒ 结果本来就不该相同
   ⇒ 判据要写"单步位移 ≤ max_step"，不是"两个结果相等"
③ "未提及的关节保持原位"：shoulder 停在中途（0.35）是**正确**的
   ⇒ 判据要写"逐位不变"（invariant），不是"两个都到 0.4"
```
⇒ 一句话：**判据要描述"什么不该变"，而不是"我预期变成什么"**。

### 铁律三案例：Phase 5 抓到的四条

```text
① `not Path(td).exists()` 写在 `with TemporaryDirectory()` **块内部**
   ⇒ 删除发生在块**退出时**，此处必然存在 ⇒ 永远 FAIL，
     却与 Runtime/Backend 行为完全无关（失败信息把人往错方向引）
   ✅ 改判"目录里没有多出预期之外的文件"

② `assert "mearm" in stripped_src`，而 stripped 的定义是
   **删掉 COMMENT 和 STRING 两类 token** ⇒ 字符串正文必然消失 ⇒ 恒假
   ✅ 判据须含"字符串被删"+"裸标识符保留"+"代码形态保留"，
      后两者才证明剥离器没退化成 `return ""`

③ 期望集合漏了**前缀**
   TemporaryDirectory 的根是 td，包目录 two_dof_toy 在其下
   ⇒ 相对路径是 two_dof_toy/manifest.yaml，不是 manifest.yaml
   ⇒ 漏前缀 ⇒ 我们自己写的四个文件全被当成"残渣" ⇒ 恒 FAIL
   ✅ expected 必须与 relative_to(哪个根) 严格对应

④ 两个函数**入参序号不同**却用同一个变量喂
   mj_quat_to_xyzw(q_wxyz) 吃 MuJoCo 序 [w,x,y,z]
   xyzw_to_mj_quat(q_xyzw) 吃 Core   序 [x,y,z,w]
   ⇒ 拿 [w,x,y,z] 的值喂 rev ⇒ 得到 [0.3,0.5,0.1,0.2] ≠ 断言的 [0.5,...]
   **函数各自完全正确**，是断言喂错了序
   ✅ 每个函数用它**自己的**输入约定构造期望值

⇒ 这一族根因一条：期望值必须与**被测函数的输入/输出约定**逐字对齐。
   写期望值时不要凭记忆，要明文写出约定（`# 入参是 [w,x,y,z]`）再推。
```

配套：**反例注射**必须实测（故意破坏被测对象，断言必须变红）。
Phase 5 的写法是"把剥离器换成 `return ""`（整段抹平）"与
"换成 `return src`（空转）"，两者都必须让断言变红 —— 实测都变红了。

### 铁律四案例：容差里命名残余误差来源

```text
命令 0.4 → 稳态 0.40238711（残差 2.4e-3）
= 重力下垂（kp=60 有限 ⇒ 比例误差）+ 子步残余速度
⇒ 是**物理**，不是 bug。而 5e-3 仍能抓住真 bug：
  自碰撞把 shoulder 卡在 0.9367 而非 1.5708（差 0.63 rad，超容差百倍）
```
⇒ 阈值定到机器精度不是"更严格"，而是**唯一**能发现
"派生量时序错位"（§4.14，实测 5.5e-08 m）的手段。

---

## 7. 已知 API 契约（踩过的猜错）

```text
JointLimits.lower/upper      → position_min / position_max（且可为 None = 自由旋转）
Link.geometries              → link.visual + link.collision（GeometryRef 列表）
RobotModel.id                → model.metadata.id
Transform.pos/.translation   → .position；Vector3.length → .norm()
Joint.is_mobile / Link.is_root → 是**方法**，必须带 ()
loader.load()                → 返回 tuple (model, report)
model 无 .assets             → v0.1 里 asset 引用恒为 0
Joint.limits 可为 None       → 打印时要判 None（自由旋转关节）
forward_kinematics()         → 只吃 (model, positions) 两个参数，
                               自己按 EndEffector 解析 TCP；**没有**第三参数
Transform.from_parts()       → (position, orientation)，不是 *args
```

Phase 5 新增：

```text
mujoco.mju_mat2Quat(q, mat)  → q 必须是 **numpy 数组**（原地写入）；
                               传 Python list 报 "incompatible function arguments"，
                               且错误信息只列 "supported NDArray"，极易误读成
                               "参数顺序反了"。用 np.zeros(4) + mat.reshape(9)
mujoco.MjModel.from_xml_string / from_xml_path
                             → **必须按内容分派**。把 XML 文本传给 from_xml_path
                               会报 `Error opening file '<!--\n  ====...'`
                               （把 XML 前几行当路径回显），看起来像"文件损坏"
                               而不是"用错 API"。判据：source.lstrip().startswith("<")
Runtime.models()             → 返回 **dict**（不是 list）⇒ 断言写 list(rt.models())
mujoco 的 site_xpos          → **mj_step 之后比 qpos 落后一个子步**！
                               推进后要补 `mujoco.mj_forward(m, d)` 刷新派生量
                               （纯运动学、不积分、不改状态）
```

动手写测试前先跑 5 行探针把真实签名/字段打出来。

---

## 8. mini_arm 几何速查（实测值）

```text
BASE_HEIGHT=0.084  SHOULDER_OFFSET=0.052  L1=0.103  L2=0.065  L_TOOL=0.032
L2_EFF=0.097   r_max=0.200   r_min=0.006   r_lim(姿态)=0.076737164
关节：base_yaw +Z ±π ｜ shoulder +Y ±π/2 ｜ elbow +Y ±3π/4 ｜ ee_link 无 joint
零位 TCP=(0.200, 0, 0.136)   ← 若得到 0.168 说明返回的是法兰 ee_link
```

---

## 9. 验收脚本完整约定

- 每项独立可判，失败给"期望 vs 实际"；不依赖网络；负向测试自行清理。
- **必须后台跑**：沙箱会给长命令 SIGTERM ⇒ 前台跑得到 0 字节日志 + SIGTERM。
  用 Bash 工具 `run_in_background=true` + `-u` + 重定向到文件。
- 源码扫描必须用 `tokenize` **剥离注释与字符串**（正则处理不了三引号/f-string）。
- 扫描类检查必须配"扫描器元测试"（否则"从没匹配到任何东西"也是全绿）。
- **扰动自检**（防整份清单恒真）必须先实测两件事：
  (a) 这个"扰动"语义上真是扰动吗（轴取反 = 规范等价，是假扰动）？
  (b) 被测函数真的读这个量吗（解析 FK 不读 `model.origin`）？
- **子进程结果解析**用 `json.JSONDecoder().raw_decode()`，
  **不要** `line[len("RESULT_JSON:"):]` —— stderr 被拼在 stdout 后，
  而无尾随换行的 DeprecationWarning 会粘在 JSON 上 ⇒ `json.loads` 报
  "Extra data"。这是 Phase 4 实测踩到的。

---

## 10. 环境坑完整清单

```text
nohup ... &                    ✗ 只活到本次工具调用结束
Bash 工具 run_in_background     ✓
cp x /tmp/x                    ✗ Permission denied → 放 .workbuddy/scratch/
grep -oE '...'                 ✗ `-u系统找不到指定的文件` → 用 Grep 工具
curl -o /dev/null -w 连跑      ✗ Exit 23 中断链 → 逐条跑或落文件
bash x.sh                      ✗ 解析到 System32\bash.exe（= WSL 存根）被沙箱拦，
                                 症状是 **exit=0 + 乱码 `襜輣繈顣0`** 且无 trace
                                 （`which -a bash` 第一条就是 System32 那个）
                                 → 用 `/usr/bin/bash x.sh` 或 `sh`
npm shim (npm 这个 shell 脚本)  ✗ uname 判成 Linux → 调 wsl.exe 被拦
                                 → 用 tools/npm.sh（直接跑 npm-cli.js）
npm --prefix <dir> install     ✗ 不生效（被当配置）→ **先 cd 到目标目录**再跑
node 版本目录 versions/current  ✗ **不是目录**，是 9 字节普通文件（内容是版本名）
                                 → 挑版本目录必须 `[ -d "$d" ]`，别用 `ls|sort`
shell heredoc 写非 ASCII/# 内容 ✗ 实测会**前置**而非追加 → 用 Write 工具
```

---

## 11. git 卫生与协作边界

- 仓库 `origin = https://github.com/zc110747/RobotForge.git`。
- **每个 Phase 验收 PASS 后，由 Agent 自动 `git commit` 一次**（用户 2026-09-15 明确要求）。
  **不执行 `git push`** —— 远程推送由用户自行处理。
- **`node_modules/` 与 `.venv/` 必须忽略**。曾有 8000+ 待跟踪文件，
  其中 **8211 个来自 `frontend/node_modules`** ⇒ 上传会出问题。
  修法：`.gitignore` 里显式加 `node_modules/`（原先**没有**这条规则）。
  忽略后待跟踪文件 **8303 → 88**。
- 同时忽略：`dist/`、`.vite/`、`*.tsbuildinfo`、`build/`、`__pycache__/`、
  `MUJOCO_LOG.TXT`（仿真每次重写）。
- `.workbuddy/` **部分入库**：`memory/` 与 `skills/` 是**项目资产**（要提交），
  `scratch/` 是临时探针（忽略）。⇒ 用 `.workbuddy/scratch/` 精确规则，
  **不要**整体忽略 `.workbuddy/`。
- 提交前用 `git status --porcelain --untracked-files=all | wc -l` 核对文件数，
  数量异常（三位数以上）说明 .gitignore 漏了目录。
- 严禁在任何记忆/日志/文档里记录 token、凭据、密钥。
