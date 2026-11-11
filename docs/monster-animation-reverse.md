# MHWs 大型怪物动画系统逆向分析

> 本文由自动代理生成（tail-bot717），分析对象是 ticket 授权的只读目录 `D:\re解包\output`。
> 结论按证据强度分级：**观察**＝直接从文件字节 / 日志 / 目录读出；**推断**＝由命名、引用关系、结构位置与公开格式研究推得，未做运行时验证；**未知**＝当前材料无法判定。

## 1. 结论摘要

1. **观察** — 目录是《Monster Hunter Wilds》(RE Engine) 的原生数据根 `natives/stm`：`ReExtractor-tools/data/settings.json` 记录 `LastGameDirectory = d:\steam\steamapps\common\MonsterHunterWilds`，文件表名为 `MHWs_STM_Release.list`。共 265,677 个文件 / 107.2 GB，其中 `motion` 子树 11,100 个文件。
2. **观察** — 每只大型怪物在 `motion/enemy/emXXXX/` 下有一套完整动画资产：一个 `motbank`（分层银行表）+ 8 套动作列表（`_00`…`_09`）+ tree 动作图列表 + 1 个 `detach`（断尾）列表 + `common/`（IK、弯曲滤波、移动银行、注视、关节 LOD）；em0160 的 `50/` 变体另有独立 motbank 与 tree 列表，列表形态混合（继承式覆盖 + 自带动作）。
3. **观察** — 动作容器 `motlist.992` 内嵌 `mot.932` 运动本体；em0071 与 em0160 主线（`00` 套 + 断尾）合计 **1,374 条内嵌动作、约 52.5 分钟、113.4 MB**（113.4 MB 为全部 30 个 motlist 的总大小）；`50` 变体另有 164 条内嵌动作（`50_04`/`50_05`），未导出 FBX。em0071_02 单个列表就有 266 条动作、418.1 秒。
4. **观察** — 动作本体是 60 fps、182 关节（em0071）/ 225–242 关节（em0160）的骨骼关键帧；每条动作 180 个通道、约 190 条轨道（11 平移 + 179 旋转），带 0–2 个同步点和 1–4 条序列轨道。
5. **观察/推断** — 运行时不是"一个动作播到底"，而是**三层拼装**：`motbank` 把动作列表编成 bank（0–6、9 = 动作库；50 = 动作图；60 = 断尾；em0160 另有 20）；场景里的 `via.motion.Motion` 组件用**多个 layer**（bank + motion + joint mask + 混合模式/速率/速度）叠加采样；`tree` 列表提供**动作图**（每个 2–3 个 `MotionNode` 经 `OctSyncBlendNode` 同步混合，混合率由 `CharacterCommonVariables.uvar` 变量驱动）。
6. **观察/推断** — 大型怪物专用的程序化层齐备：腿部/脊柱 IK（带碰撞过滤 `Character_EnemyIK_Boss/Zako.cfil`）、多骨 IK、`Lookat`、落地减震、按步态切换的 `bendfilter`（walk/dash/fly）、`movebank`（walk/dash/air 根位移）、关节 LOD 与动画 LOD、断尾专用动作列表。
7. **对应验证** — 导出的 1,374 个动作 FBX 与本报告解析出的 1,374 条主线内嵌动作**逐条一一对应**（数量相同、ID 与文件名一致、时长一致）；Blender 复验显示 em0071_02 第 1 条动作为 48 帧 @60 fps、206 根骨，与解析值吻合。

## 2. 材料与范围

| 项 | 值 | 依据 |
| --- | --- | --- |
| 授权目录 | `D:\re解包\output` | ticket 来源消息 |
| 解包工具 | ReExtractor v1.4.4（`D:\re解包\ReExtractor-v1.4.4.exe`，配套 `ReExtractor-tools/`） | 目录 |
| 游戏识别 | Monster Hunter Wilds（STM/Release） | `settings.json`、`MHWs_STM_Release.list` |
| 数据规模 | 265,677 文件 / 107,228 MB；`motion` 子树 11,100 文件 | 递归统计 |
| 派生资产 | 3 个模型 FBX（`ch90_071`、`ch90_160_0000/5000`）+ `批量动画/` 1,374 个动作 FBX（26.5 GB）+ 12.1 GB 7z | 目录与导出日志 |

`output` 下除 `unpacked/` 外的 FBX 与 `批量动画.7z` **不是游戏原文件**，是 ReExtractor 用 Blender 管线导出的中间产物（`ReExtractor-tools/tools/export_animations_fbx.py`、`export_models_fbx.py`，日志 `logs/fbx-*.log`）。本报告的格式结论全部来自 `unpacked/natives/stm` 原始文件与导出日志，不来自 FBX。

## 3. 目录与资产形态

以 `em0071` 为例（em0160 同构）：

```
natives/stm/motion/enemy/em0071/00/
├─ em0071_00.motbank.4            # 分层银行表：动作库 + 动作图 + 断尾
├─ em0071_00_mcb.user.3           # 控制器银行 → 10 个 _mct
├─ em0071_00_meb.user.3           # 事件/扩展银行 → 10 个 _mex
├─ em0071_00/ … _09/              # 8 套动作列表，每套 = motlist + mct + mex
│    └─ em0071_02.motlist.992     # 266 条动作（内嵌 mot）
├─ em0071_tree/                   # 234 个动作图（mtree），bank 50
├─ em0071_detach/                 # 断尾动作列表（3 条、5 关节），bank 60
└─ common/
     ├─ jointmap/em0071_jointmap.jmap.26        # 关节掩码组 / IK 关节
     ├─ jointlod/em0071_jointlod.jointlodgroup.2
     ├─ ik/em0071_iklegspine.ikls.28            # 腿-脊柱 IK
     ├─ ik/em0071_ikmultibone.ikmulti.4
     ├─ ik/em0071_shockabsorberparameter.user.3
     ├─ lookat/em0071_lookat2.iklookat2.26
     ├─ bendfilter/em0071_bend{walk,dash,fly}.user.3
     └─ movebank/em0071_movebank_{walk,dash,air,movebanksetting}.user.3
```

em0160 额外有 `50/` 变体目录（8 套 `em0160_50_*` 列表 + 独立 motbank + tree + detach），`em0160_05/` 下还有 `fbx/` 存放 75 个独立 `.mot` 文件。

## 4. 容器格式（全部按字节解析验证）

### 4.1 `motbank.4`（magic `mbnk`）— 分层银行表

| 偏移 | 类型 | 含义 |
| --- | --- | --- |
| 0 | u32 | 版本 4 |
| 4 | char[4] | `mbnk` |
| 8 | u64 | 保留 |
| 16 | u64 | 银行项数组偏移 |
| 24 | u64 | 变量表（uvar）路径偏移 |
| 32 | u64 | 关节映射（jmap）路径偏移（v≥3） |
| 40 | i32 | 银行数量 |

银行项 24 字节：`u64 path / i32 bank_id / u32 bank_type / u64 mask_bits`。

em0071 实测（**观察**）：10 个银行；`uvar = Motion/Enemy/Decorate/UserVariable/Enemy_UserVariable.uvar`；`jmap = Motion/Enemy/em0071/00/Common/JointMap/em0071_JointMap.jmap`；bank 0–6、9 指向 8 套动作列表，**bank 50 = 动作图列表**，**bank 60 = 断尾列表**。em0160 为 11 个银行，多一个 **bank 20**（该列表文件在本 build 的文件表里也不存在，属可选/未使用槽位）。

### 4.2 `motlist.992`（magic `mlst`）— 动作列表

头 54 字节：

| 偏移 | 类型 | 含义 |
| --- | --- | --- |
| 0 | u32 | 版本 992 |
| 4 | char[4] | `mlst` |
| 8 | u32 | error_flags |
| 12 | u32 | master_size |
| 16 | u64 | 动作指针数组 |
| 24 | u64 | 行表（每行 72 字节） |
| 32 | u64 | 列表名（UTF-16） |
| 40 | u64 | base_list 路径（继承用） |
| 48 | u32 | 条目数 |
| 52 | u8 | joint_mask_mode |

行 72 字节：`u64 overrides / u32 motion_id / u32 joint_mask / u32 tag_hash / u8 type / u8 flags / u8 physics / u8 override_count / u32 tag[10] / u16 additional_sequences / u8 source_bank / u32 source_key`。

实测要点（**观察**）：
- `em0071_02`：条目 266，`motion_id` 非连续（1,2,3,7,8,9,13,14,20,21,22,26,27,28,32,33,39,48,51,52,60,61,…），说明 ID 由作者/工具分配，**不是数组下标**；名称 `em0071_02_001` … `em0071_02_973`。
- 每个条目按指针指向三种对象之一：内嵌 `mot.932`、内嵌 `mtree` 动作图、或 UTF-16 引用路径（`Flags_RefPath`）。
- `em0160_05`：76 个条目**全部是引用路径**，指向 `Motion/Enemy/em0160/00/em0160_05/fbx/em0160_05_001.mot` 等 75 个独立 `.mot` 文件——同一列表可以"内嵌"也可以"外链"。
- `em0160_50_00`…`50_03/06/09` 等继承式变体列表：条目 `flags=0x10`、指针为 0、`source_key` 指向基础 ID，`base_list` 指向 00 的对应列表——**基础列表继承 + 局部覆盖**（推断：variant/状态只写差异）；同一变体目录里的 `50_04`、`50_05` 却是独立列表（`base_list=None`，分别内嵌 88、76 条动作），说明"变体目录"并不等于"纯覆盖"。
- `master_size ≠ 0` 的 motion 不重复存骨架，只有首条（master）动作内嵌 182 关节骨架；对后续动作按 mot 起点解释 skeleton 指针会越界，故本文只从 master motion 取骨架。

### 4.3 `mot.932`（magic `mot `）— 运动本体

头 126 字节：`u32 version / "mot " / u32 error_flags / u32 master_size / u64 skeleton / u64 animation / u64 properties / u64 custom_keys / u64 sequences / u64 character(jmap 路径) / u64 sync / u64 raw_sync / u64 append / u64 name / f32 end_frame / f32 loop_frame / f32 raw_start / f32 raw_end / u16 joint_count / u16 animation_count(通道数) / u8 seq_count / u8 sync_count / u8 raw_sync_count / u16 fps / u16 property_count / u16 attributes`。

- 骨架记录 16 字节（`u64 joints / u32 count`），关节 80 字节：名称、父/子/兄弟指针、平移/旋转、joint_id、name_hash、extra_type。
- 通道 12 字节：`u16 joint_id / u8 transforms(1=平移,2=旋转,4=缩放) / u8 weight / u32 name_hash / u32 tracks`。
- 轨道 20 字节：`u32 packed{solver:12, encoding:8, frame_type:4} / u32 key_count / u32 frames / u32 values / u32 parameters`。solver 162=float、242=vector3、274=quaternion；frame_type 2=u8、4=u16、5=u32；关键帧按 encoding 做定点/量化压缩，配 `parameters` 解包参数。
- `attributes` 位标志含 GpuMot / JointPose 两种替代表示；本目录大型怪物动作均为 `0x0`，即**全部走 CPU 关键帧轨道**（观察）。

### 4.4 `mtree` v21（动作图）— 存于 `em0071_tree.motlist.992` 的 234 个条目

头 88 字节：`u32 version / "mtre" / u64 data / u64 paths / u64 nodes / u64 links / u64 name / u64 remaps / u64 choices / u16 node_count / u16 link_count / u16 root / u16 path_count / u16 child_motion_count / u16 remap_count`；节点 48 字节、链接 40 字节、重映射 6 字节、选择 8 字节（bank, motion）。节点类名按节点表顺序内联存放。

实测（**观察**）：em0071 的 234 个图、em0160 的 267 个图（`50` 变体的 267 个条目全部继承基础 tree 列表），节点类只有三种：

| 类 | em0071 | em0160 | 参数（实测） |
| --- | --- | --- | --- |
| `MotionNode` | 642 | 590 | `MotionID`、`BankID`、`Asset`、`Always` |
| `OctSyncBlendNode` | 234 | 283 | `BlendRate`、`CircledBlend` 等 |
| `VariablesNode` | 234 | 267 | `Asset = Motion/Common/Variable/CharacterCommonVariables.uvar` |

每个 em0071 图 = 2–3 个 `MotionNode` + 1 个 `OctSyncBlendNode` + 1 个 `VariablesNode`（174 个图 5 节点、60 个图 4 节点），`MotionID` 连续（如 `em0071_tree_001` → 11/12/13 = `em0071_00_011/012/013`），同步混合节点的动作输入接动作节点、数据输入接变量节点（`BlendRate` 等参数由变量提供，**推断**）。em0160 的图最多 10 节点、可含多个混合节点（嵌套混合）。**推断**：这是与 UE 混合空间/同步混合等价的"动作图"，混合率由共享变量表驱动；具体节点求值顺序需运行时确认（见 §9）。

### 4.5 辅助资产

| 文件 | 观测 | 作用（推断） |
| --- | --- | --- |
| `*.jmap.26` | 关节掩码组表；字符串含 `root`、`L/R_Arm_IK`、`L/R_Foot_IK`、`L/R_WingArm_IK` | layer 的 joint mask 分组与 IK 关节标记 |
| `*.jointlodgroup.2` | 组 `Default` + 关节名单（含 `*_HJ` 辅助关节） | 按距离/重要度分组更新关节 |
| `*.ikls.28` | 引用 `System/Collision/Filter/Character/Character_EnemyIK_{Boss,Zako}.cfil` | 腿-脊柱 IK，射线用碰撞过滤 |
| `*.ikmulti.4` / `*.iklookat2.26` | 小体量参数表 | 多骨 IK / 注视约束 |
| `*_shockabsorberparameter.user.3` | 引用 `Motion/Enemy/Decorate/IK/EmShockAbsorberState.user` | 落地/受击减震参数 |
| `*_bend{walk,dash,fly}.user.3` | 三种步态各一份 | 按步态的程序化弯曲（脊柱/尾/翼） |
| `*_movebank_{walk,dash,air}.user.3` + `movebanksetting` | 每份 2,352 B，引用 `MoveBankSetting.user` | 根位移/转向曲线银行 |
| `motion/common/animationlod/*` | `defaultanimationlod`、`motionlodconfig…`、`animationlodcameraparameter` 等 12 份 | 动画更新 LOD 的全局配置 |
| `*_mcb.user.3` / `*_meb.user.3` | 分别列出 10 个 `_mct` / 10 个 `_mex` 路径 | 控制器银行 / 事件（扩展）银行，按动作列表索引 |
| `*_mct.user.3` / `*_mex.user.3` | mct 内含 `_Default_` 名称并引用 `em0071_00_meb.user` | 每个动作列表的控制器表 / 事件表（RSZ 用户数据） |

`_mct/_mex/_meb/_mcb` 的类型名在二进制里是哈希，缺少 RSZ schema，本文只能确认引用关系与"按动作列表一一对应"的结构，语义按命名推断（§9）。

## 5. 运行时怎么运作

**装配链（观察 + 公开工具实现互证）：**

```
motbank.4 ──bank_id──> motlist.992 ──entry──> mot.932 (关键帧)
    │                        │                └─ mtree (动作图, bank 50)
    │                        └─ base_list (变体继承/覆盖) / 外部 .mot 引用
    ├─ uvar  → 共享变量（驱动混合率/状态）
    └─ jmap  → joint mask 组
场景 RSZ: via.motion.Motion 组件（多层 layer）
mcb → mct（控制器） ； meb → mex（事件/扩展）
common: IK / bendfilter / movebank / lookat / jointLOD / animationLOD
```

**分层播放（推断，依据公开工具 REasy 对 `via.motion.Motion` 组件的读取逻辑）：** 每个实体有一个 `via.motion.Motion` 组件，内含若干 layer；layer 字段为 混合模式（AddBlend / LayerBlend / Private 等）、混合率、bank、joint mask、mask 模式（ID / hash）、motion、enabled、stop update、frame、speed。即：**基础层播 locomotion，其他层用 joint mask 只驱动局部骨骼（尾、翼、面部、受击反应），按 Add/Override 合成**。这也是为什么动作列表要按 bank 分套、并单独提供 joint mask 表。

**时间与混合（观察）：** 所有动作 60 fps；`end_frame` 以帧计；`loop_frame` 指定循环点；每条动作最多 2 个 `sync` 点，用于两段动作对齐相位（如步态切换、左右脚交替）；`sequences`（1–4 条）= 动作内嵌时间轴（clip + 带过滤的轨道），用于动作期间的附加数据（推断：脚步/特效/部件动作等游戏侧事件）。

**动作图（观察）：** bank 50 的 234/267 个 `mtree` 把"一组同一动作的 2–3 个变体"做成 2/3 路同步混合（`OctSyncBlendNode`），混合率来自共享变量表（`CharacterCommonVariables.uvar`）——即**动作级混合图**，而不是单个动作的线性播放。em0160 的图可嵌套多个混合节点。

**程序化修正（观察文件 + 推断用途）：** IK（腿-脊柱、多骨、注视）、落地减震、弯曲滤波（走/冲/飞三套）、移动银行（走/冲/空三套根位移）、关节 LOD、动画 LOD。它们以 `.user`/专用容器与动作列表并列存在，属于"采样后修正"层。

## 6. 大型怪物专项观察

1. **骨架规模**：em0071 = 182 关节（部分动作 179/181，说明动作可只覆盖子集）；模型绑定骨架 = 206 根（`ch90_071_0000_LOD0.fbx` 与动作 FBX 都是 206，动作 FBX 还带一具绑定姿态参考网格）。em0160 的动作关节数为 225/242（个别 866/929，见 §9）。
2. **骨架拓扑**：em0071 含 `L/R_WingU/L`、`L/R_WingArm_IK`、`Tail0–4`、`L/R_Antennae`、`Jaw/Tongue0–1`、双手五指 + `L/R_Arm_IK`、`L/R_Foot_IK`、`MOT_wing_MT`/`MOT_Attach`；em0160 含两条 10 节刀臂（`L/R_BladeU0..U9` + `L/R_BladeL0..L9` + 控制骨/壳骨）、`Tail0–6` + 尾毛链、翼膜/毛发链、`Absorb_Const`、双 `Blade_IK`。**推断**：大型怪物把"可动长链结构"（尾、翼、刀臂、毛）做成显式骨链，交给动画 + 程序化层共同驱动。
3. **动作分套**：8 套动作列表按行为域拆分（0–6、9），另有动作图（50）和断尾（60）。以 em0071 为例，单套 4–266 条、整套 0.2–367.5 秒；em0071_03 最长单条 10.37 秒，em0160_04/06 最长 21.97 秒（推断：演出/处决类动作）。
4. **数据量**（观察）：

| 怪物 | 动作数 | 总时长 | 动作列表大小 | 关节数 |
| --- | --- | --- | --- | --- |
| em0071 | 764 内嵌（8 套）+ 3 断尾 | 1,607.8 s + 3.5 s | 51.4 MB | 179–182 |
| em0160 | 604 内嵌（8 套）+ 76 外链 + 3 断尾 | 1,536.9 s | 62.0 MB | 192–242 |
| 合计 | **1,374 内嵌** + 76 外链 | **3,151.9 s ≈ 52.5 min** | 113.4 MB | — |

上表只统计主线 `00` 套与断尾；`50/` 变体另内嵌 164 条（`50_04` 289.4 s + `50_05` 116.1 s），没有对应 FBX 导出；「动作列表大小」是全部 motlist（含 50 变体）的文件大小。

5. **存储变体**：同一逻辑动作列表可内嵌 `mot`（正常）、外链独立 `.mot`（em0160_05：75 个文件）、用 `base_list` 只写差异（em0160_50_00…`50_03/06/09`），而同一 `50/` 目录的 `50_04`/`50_05` 仍是自带动作的独立列表。**推断**：按更新/复用/补丁粒度选择存储方式，运行时对上层透明。

## 7. 与导出链路的一致性验证

| 验证 | 方法 | 结果 |
| --- | --- | --- |
| 动作数量 | 解析 em0071/em0160 主线（`00` 套 + 断尾）的内嵌动作数 vs `批量动画/` FBX 数 | 1,374 = 1,374（逐套相等，如 em0071_02 = 266；`50` 变体的 164 条与 em0160_05 的 76 条外链动作没有 FBX，与 `批量动画/` 无对应目录一致） |
| 动作 ID | 解析 `motion_id` vs FBX 文件名 `…motionNNN_idN.fbx` | 一致：1,2,3,7,8,9,13,…；导出日志 `REEXTRACTOR_PROGRESS:266/266` |
| 时长 | 解析 `end_frame=48`（60 fps）vs Blender 导入 FBX 的 action 帧范围 `1–49` | 一致（48 帧 = 0.8 s） |
| 骨架 | 解析 master motion 关节数 182 vs Blender 导入动作 FBX / 模型 FBX 骨数 | 动作 FBX 206 = 模型绑定骨 206；动作只覆盖 182 关节子集 |
| 格式 | 与公开 RE 引擎动作工具 REasy 的 `motlist 992` / `mot 932` 布局实现逐字段对照 | 字段偏移、行/通道/轨道尺寸一致，解析结果自洽 |

## 8. 与 REA 的关系

[REA](https://github.com/morluto/rea) 是面向**已发布二进制 / JS / .NET / 归档包**的逆向代理工具集（MCP 工具 + 工作流），核心是"先定目标、格式优先、证据可复现、明确未知"。本次授权目录里只有游戏**数据容器**，没有可执行文件，也没有场景/RSZ 存档，REA 的原生/JS provider 没有可作用的目标，因此**没有直接调用 REA**；采用的是它的方法骨架：目标识别 → 资产清单 → 容器格式解析 → 与独立证据（导出日志、Blender、公开格式实现）交叉验证 → 把未知显式列出。若后续需要确认运行时行为，需要补充游戏可执行文件或场景/预制体数据，再用 REA 的原生分析路径。

## 9. 未知与限制

1. **运行时绑定未知**：本目录没有场景/预制体 RSZ，无法证明某个行为具体用哪个 bank、哪几层、mask 是什么；§5 的分层模型来自公开工具对 `via.motion.Motion` 组件的支持 + 数据组织反推。
2. **动作图语义未知**：`OctSyncBlendNode`/`VariablesNode` 的求值语义按类名、连线与参数推断，未在运行时验证；`mtre` 的重映射/选择表未完全解码（本报告只用到节点、连线与参数）。
3. **`_mct/_mex/_meb/_mcb` 是哈希化 RSZ 用户数据**，缺 schema，本文只确认引用关系，具体控制器/事件语义未证实。
4. **866/929 关节的个别 em0160 动作**未解释（可能是附加骨架/特殊形态），需要单独样本分析。
5. **怪物显示名未知**：`em0071`/`em0160`/`ch90_*` 是内部 ID，未与本作怪物名录一一对应。
6. **未做运行时验证**：无游戏进程、无 Hook，所有"怎么运作"的动态结论都停在结构与引用层面。

## 10. 对 Sun / UE5 的启示

1. **分层 + 掩码**比"一个 AnimBP 打天下"更适合大型怪物：基础 locomotion 层 + 局部层（尾/翼/面部/受击），层间用关节掩码隔离，等价于 UE 的 Layered Blend per Bone + Slot。
2. **动作图（2–3 变体同步混合 + 变量驱动）**对应 UE 的 Sync Group / Blend Space；同步点是关键（RE 每条动作最多 2 个），UE 侧应保留同步标记而不是靠时长比例。
3. **程序化层单独成库**（IK、减震、按步态的弯曲、根位移银行），与动作解耦，便于按怪物体型复用。
4. **LOD 分两级**：关节 LOD（骨骼更新）+ 动画 LOD（更新频率/相机参数），对同屏多只大怪很关键。
5. **存储分层**（内嵌/外链/继承覆盖）对分包与热更友好；UE 侧对应 Primary Asset + 变体覆盖，不必照搬二进制格式。

## 附录 A 解析器与复现

仓库内 `tools/mot_inspect.py`（无第三方依赖，Python 3.9+）实现了本文用到的容器读取：

```bash
python tools/mot_inspect.py bank  "D:/re解包/output/unpacked/natives/stm/motion/enemy/em0071/00/em0071_00.motbank.4"
python tools/mot_inspect.py list  "D:/re解包/output/unpacked/natives/stm/motion/enemy/em0071/00/em0071_02/em0071_02.motlist.992"
python tools/mot_inspect.py mot   "D:/re解包/output/unpacked/natives/stm/motion/enemy/em0160/00/em0160_05/fbx/em0160_05_001.mot.932"
python tools/mot_inspect.py tree  "D:/re解包/output/unpacked/natives/stm/motion/enemy/em0071/00/em0071_tree/em0071_tree.motlist.992"
python tools/mot_inspect.py inventory "D:/re解包/output/unpacked/natives/stm/motion/enemy/em0071"
```

字段偏移以本文表格为准；`--json` 输出供脚本复用。

## 附录 B 字段速查

| 容器 | magic | 版本 | 关键结构 |
| --- | --- | --- | --- |
| motbank | `mbnk` | 4 | 头 44 B + 银行项 24 B |
| motlist | `mlst` | 992 | 头 54 B + 行 72 B + 指针数组 + 内嵌对象 |
| mot | `mot ` | 932 | 头 126 B + 骨架 16/80 B + 通道 12 B + 轨道 20 B |
| mtree | `mtre` | 21 | 头 88 B + 节点 48 B + 链接 40 B + 重映射 6 B + 选择 8 B |
| jmap | `jmap` | 26 | 关节掩码组表 |
| jointlodgroup | `jldg` | 2 | 关节 LOD 组 |
| ikls / ikmulti / iklookat2 | `ikls` / `ikml` / `ikl2` | 28 / 4 / 26 | IK 参数与关节链 |

