# MHWs 动画系统技术方案总览（解包数据视角）

> 本文由自动代理生成（tail-bot717），分析对象是 ticket 授权的只读目录 `D:\re解包\output\unpacked`（Monster Hunter Wilds / RE Engine）。
> 证据分级：**观察**＝目录、文件表或文件字节直接读出；**推断**＝由命名、引用关系、结构位置与公开工具实现推得，未做运行时验证；**未知**＝当前材料无法判定。
> 单体怪物的逐字段深挖见 [`monster-animation-reverse.md`](monster-animation-reverse.md)，本文只讲系统级技术方案。

## 1. 技术方案摘要

1. **观察** — 动画资产不是"每个动作一个文件"，而是**四层引用图**：`motbank`（银行表，给动作列表编号）→ `motlist`（动作列表，条目带 ID/标签/掩码/物理标志）→ `mot`（关键帧本体）或 `mtree`（动作图）→ 运行时按 layer 采样。全目录共 3,161 个 motbank、5,250 个 motlist、5,672 个独立 mot、11,100 个 motion 子树文件。
2. **观察** — 列表条目有**三种形态**：内嵌 `mot`、外链 `.mot` 文件、`mtree` 动作图；列表还可用 `base_list` 继承基础列表并只写差异（em0160 的 `50/` 变体即此机制）。
3. **观察** — 资产按**角色类别**组织：`motion/` 下 13 类（npc 5,943 文件、enemy 2,994、player 920、talk 457、otomo 276、gimmick 155、facial 153、porter 98、playernpc 45、npc_cat 23、actioncamera 16、common 12、guildcard 8），每类共享一份 `uvar` 变量表与一套关节/IK/LOD 配置。
4. **观察** — 角色差异体现在银行编号与列表语义：玩家 `common.motbank` 有 42 个银行（BaseMove/Mir/ItemUse/Slinger/Gesture/Damage/…/Ride*/AddBase/`*_tree`/Female/Male），14 把武器各有 `wpXX.motbank`（bank 20 动作 + bank 21 动作图）；怪物用 bank 0–9（动作库）、50（动作图）、60（断尾）；过场角色用 bank 5000 的私有银行。
5. **观察/推断** — 运行时是**分层叠加 + 程序化后处理**：实体上的 `via.motion.Motion` 组件按 layer（bank + motion + joint mask + 混合模式/速率/速度）叠加；`mtree` 用 `MotionNode` + `OctSyncBlendNode` + `VariablesNode` 做 2–3 路同步混合，混合率由 `uvar` 驱动；采样后再叠加 IK、减震、弯曲滤波、根位移银行、注视、关节 LOD 与动画 LOD。
6. **观察/推断** — 面部、相机、次级运动是**独立管线**：面部用 361 关节、159 通道的专用动作列表 + `facebasevariable.uvar` + `_jc.jcns` 约束；相机用 `mcamlist`/`motcam`/`mcambank`；头发/布料/尾巴/绳索由 1,354 个 `chain2` 动态链 + 883 个 `jcns` 约束集在采样后驱动。

## 2. 规模与目录拓扑

### 2.1 顶层分布（**观察**，来源：`MHWs_STM_Release.list`，265,678 条）

| 前缀 | 文件数 | 与动画的关系 |
| --- | --- | --- |
| `natives/stm/gamedesign` | 114,859 | 玩法数据；其中 2,805 motbank / 3,801 motlist / 103 mcamlist 属于过场（`event/**`） |
| `natives/stm/art` | 71,906 | 模型/材质；含 10 motbank、10 motlist、109 独立 mot、1,346 chain2、883 jcns、jmap/jointlod 等 |
| `natives/stm/streaming` | 43,155 | 流式资源，无动作容器 |
| `natives/stm/sound` | 15,394 | 音频 |
| `natives/stm/motion` | 11,100 | 动画主目录（13 类） |
| `natives/stm/gui` | 7,244 | 界面 |
| 其余 | 约 2,000 | system/systems/boot 等 |

> 解包目录实际 265,677 个文件，与文件表差 1（文件表按发布清单统计，可能含 1 个未落地条目）。

### 2.2 `motion/` 的 13 个类别（**观察**）

| 类别 | 文件数 | 组织方式（抽样） |
| --- | --- | --- |
| `npc` | 5,943 | 按人群：male/female/old/child/airou/chick/lobby/moriver/porter/pugee/ryujin*/unique/custom/story/acc/decorate |
| `enemy` | 2,994 | 每只怪 `emXXXX/00`：8 套动作库 + tree + detach + common |
| `player` | 920 | `common/`（40 个行为域）+ `weapon/wp00…wp13` + `weapongimmick/`（64 个）+ charamake/decorate/preview/slingergimmick |
| `talk` | 457 | 按声线/年龄：airou/child/female/male/old/moriver/ryujinf/common |
| `otomo` | 276 | `otomo_00…14` + `otomo_tree` + charamake/story/decorate |
| `gimmick` | 155 | itemmodel（`ch07_*`、`itXXXX`）/stagemodel/otmo_item/decorate |
| `facial` | 153 | 每角色 `list/`（alma/erick/jema/gilidcurd/favious…）+ `common/`（`facebasevariable.uvar`、`fcXXX_0000_rb.motbank`）+ charamake |
| `porter` | 98 | charamake/common/decorate/story |
| `playernpc` | 45 | `plnpc_uXXXXX` |
| `npc_cat` | 23 | 猫型 NPC |
| `actioncamera` | 16 | `enemy/emXXXX.mcamlist` + `player/*.mcamlist` + `plactioncamera.mcambank` |
| `common` | 12 | `animationlod/` 全局动画 LOD 配置 |
| `guildcard` | 8 | 名片用动作 |

### 2.3 角色模型分类（**观察**，`art/model/character/`）

`ch00`–`ch07`、`ch80`、`ch90`、`ch91` 共 11 类；文件量最大的是 `ch02`（10,728）、`ch03`（4,516）、`ch90`（1,982）、`ch04`（1,373）、`ch01`（1,132）、`ch05`（1,145）、`ch80`（976）、`ch91`（477）、`ch07`（395）、`ch00`（362）、`ch06`（15）。
已确认的对应关系：`ch90/071` ↔ `em0071`、`ch90/160` ↔ `em0160`（大型怪物，内部 ID 与模型/动作编号一致）；`ch07_900` 等出现在 `motion/gimmick/itemmodel/`（道具/机关模型）。`ch00/commonjntassets` 存放共用面部关节资产（`face_airou_jntmap.jmap` 等）。

## 3. 三层资产模型

```
类级共享（每类一份）                   角色实例                         场景/事件私有
motion/<cat>/decorate/uservariable/   motion/<cat>/<id>/<set>/         gamedesign/event/<ev>/<id>/
  <cat>_uservariable.uvar               <set>.motlist + _mct + _mex       chara/<actor>/*.motbank+motlist
motion/common/animationlod/*.user      <set>.motbank / *.mot / tree      camera/*.mcamlist
motion/<cat>/<id>/common/{ik,jmap,     art/model/character/<ch>/<id>/
  jointlod,bendfilter,movebank}          *.chain2 / *.jcns / jmap
```

- **类级共享**：`uvar` 只有 12 个（player/npc/otomo/enemy/tentacle-enemy/porter/gimmick/face 各一 + 4 个 VFX 用），说明变量是"按角色类别共用"而不是按实例复制。
- **角色实例**：动作列表、动作图、断尾列表、`common/` 下的 IK/弯曲/根位移/关节 LOD 都是每个角色（或每只怪）一份。
- **场景/事件私有**：过场角色的动作随事件打包（例：`gamedesign/event/egm/egm0002/egm0002_50/chara/npc101_00_002_00/…`），相机在 `…/camera/*.mcamlist`。

## 4. 容器格式

> 本节字段偏移均经实际文件解析验证；`tools/mot_inspect.py` 可复现 motbank/motlist/mot/tree 四项。

### 4.1 `motbank`（magic `mbnk`，版本 4）— 银行表

| 偏移 | 类型 | 含义 |
| --- | --- | --- |
| 0 | u32 | 版本 4 |
| 4 | char[4] | `mbnk` |
| 8 | u64 | 保留 |
| 16 | u64 | 银行项数组 |
| 24 | u64 | 变量表（uvar）路径 |
| 32 | u64 | 关节映射（jmap）路径 |
| 40 | i32 | 银行数量 |

银行项 24 字节：`u64 path / i32 bank_id / u32 bank_type / u64 mask_bits`。银行编号是**跨模块的约定**（**观察**，抽样）：

| 场景 | bank 编号 | 例 |
| --- | --- | --- |
| 玩家通用 | 0–30 行为域、50–60 骑乘、99 AddBase、100–107 `*_tree`、200–201 女性变体 | `motion/player/common/common.motbank`（42 个） |
| 玩家武器 | 20 动作、21 动作图 | `motion/player/weapon/wp00/wp00.motbank` |
| 怪物 | 0–9 动作库、50 动作图、60 断尾（em0160 另有 20） | `motion/enemy/em0071/00/em0071_00.motbank` |
| 过场角色 | 5000（事件私有） | `gamedesign/event/egm/egm0002/…/npc101_00_002_00.motbank` |
| 动作相机 | `mcambank`（magic `cbnk` v3）同构 | `motion/actioncamera/player/plactioncamera.mcambank` |

### 4.2 `motlist`（magic `mlst`，版本 992）— 动作列表

头 54 字节：`u32 version / "mlst" / u32 error_flags / u32 master_size / u64 motions / u64 rows / u64 name / u64 base_list / u32 count / u8 joint_mask_mode`；行 72 字节：`u64 overrides / u32 motion_id / u32 joint_mask / u32 tag_hash / u8 type / u8 flags / u8 physics / u8 override_count / u32 tag[10] / u16 additional_sequences / u8 source_bank / u32 source_key`。

条目三态（**观察**）：指针非空且 magic 为 `mot ` → 内嵌动作；magic 为 `mtre` → 动作图；否则为 UTF-16 引用路径。指针为空 → 继承（`flags=0x10`、`source_key` 指向基础 ID、`base_list` 指向基础列表）。
实测形态：em0160_05 的 76 条全部外链 75 个 `.mot`；em0160_50_00…`50_03/06/09` 全继承；`50_04`/`50_05` 自带 88/76 条动作。

### 4.3 `mot`（magic `mot `，版本 932）— 关键帧本体

头 126 字节：`u32 version / "mot " / error_flags / master_size / u64 skeleton / u64 animation / u64 properties / u64 custom_keys / u64 sequences / u64 character / u64 sync / u64 raw_sync / u64 append / u64 name / f32 end_frame / f32 loop_frame / f32 raw_start / f32 raw_end / u16 joint_count / u16 animation_count / u8 seq_count / u8 sync_count / u8 raw_sync_count / u16 fps / u16 property_count / u16 attributes`。
骨架 16 字节记录 + 80 字节关节；通道 12 字节（`joint_id / transforms 位掩码 1=平移 2=旋转 4=缩放 / weight / name_hash / track 指针`）；轨道 20 字节（`solver:12|encoding:8|frame_type:4`、key_count、frame/values/parameters 指针），solver 162=float、242=vector3、274=quaternion，帧类型 u8/u16/u32，关键帧按 encoding 量化压缩。所有大型怪物动作 `attributes=0x0`（CPU 关键帧，未用 GpuMot/JointPose 表示）。

### 4.4 `mtree`（magic `mtre`，版本 21）— 动作图

头 88 字节（node/link/name/remaps/choices 指针与计数）；节点 48 字节（名称/标签/参数/类哈希/类型），类名**按节点表顺序内联**存放；链接 40 字节。
实测节点类：`MotionNode`（Asset/BankID/MotionID/Always）、`OctSyncBlendNode`（BlendRate/CircledBlend 等）、`VariablesNode`（引用 `CharacterCommonVariables.uvar`）。em0071 的 234 个图每个 2–3 动作节点 + 1 混合节点 + 1 变量节点；em0160 的 267 个图最多 10 节点、可嵌套多个混合节点。

### 4.5 相机容器

| 容器 | magic / 版本 | 观察 |
| --- | --- | --- |
| `*.mcamlist` | `clst` / 22 | 头含名称与数量（0x30 计数、0x34 名称）；全库 118 个，其中 103 个在过场、15 个在 `motion/actioncamera`；抽样 `QuestClear` 15 条、`ItemUse` 8 条、`FastTravel` 10 条 |
| `*.motcam` | `mcam` / 12 | 单条相机动作；全库仅 1 个（`motion/actioncamera/player/charamake/fbx/charamake_001.motcam.12`） |
| `*.mcambank` | `cbnk` / 3 | 相机银行表；全库 1 个（玩家动作相机） |

**推断**：相机动作与角色动作分离成独立管线，过场按事件打包相机列表，播放时与角色动作图并行驱动。

### 4.6 辅助容器

| 容器 | 数量 | 观察 / 作用（推断） |
| --- | --- | --- |
| `*.jmap`（`jmap` 26） | 88 | 关节掩码组表；决定 layer 在哪些骨骼上生效 |
| `*.jointlodgroup`（`jldg` 2） | 64 | 关节更新 LOD 分组（含 `*_HJ` 辅助关节） |
| `*.ikls`（`ikls` 28） | 74 | 腿-脊柱 IK；引用 `Character_EnemyIK_{Boss,Zako}.cfil` 碰撞过滤 |
| `*.ikmulti`（`ikml` 4） | 26 | 多骨 IK 参数 |
| `*.iklookat2`（`ikl2` 26） | 49 | 注视约束（怪物、NPC 共用） |
| `*.uvar`（`uvar` 3） | 12 | 变量表：player/npc/otomo/enemy/tentacle-enemy/porter/gimmick/face 各一 + 4 个 VFX |
| `*.chain2`（`chn2` 13） | 1,354 | 动态骨链：头发/布料/尾巴/绳索；引用 `Character_Chain.cfil`/`Character_Default.cfil` 碰撞过滤 |
| `*.jcns`（`jcns` 29） | 883 | 关节约束集；面部文件含 `fcParam_*` 参数（例 `fc500_0000_jc.jcns`） |
| `*_mct` / `*_mex` | 1,154 / 1,164 | 每个动作列表的控制器表 / 事件（扩展）表，RSZ 用户数据 |
| `*_mcb` / `*_meb` | 184 / 188 | 控制器银行 / 事件银行，列出对应列表的 `_mct`/`_mex` |
| `motion/common/animationlod/*` | 12 | 全局动画 LOD：默认参数、相机参数、IK/关节控制 LOD、面部 LOD |

## 5. 运行时技术方案

### 5.1 分层采样（**推断**，依据公开工具 REasy 对 `via.motion.Motion` 组件的读取逻辑）

实体上的 `via.motion.Motion` 组件持有多个 layer：混合模式（AddBlend / LayerBlend / Private 等）、混合率、bank、joint mask、mask 模式（ID/hash）、motion、enabled、stop update、frame、speed。基础层播 locomotion，其余层用掩码只驱动局部骨骼（尾/翼/面部/受击/骑乘），按 Add/Override 合成。这解释了为什么动作要按 bank 分域、并单独提供 `jmap`。

### 5.2 动作图与变量（**观察**）

`*_tree` 列表（玩家 5 个通用 + 14 把武器各一、怪物 bank 50、伙伴 `otomo_tree`、面部 `AlmaFaceTree`）把同一动作的多个变体做同步混合，混合率由 `VariablesNode` 引用的 `uvar` 提供。变量表按类别共享，所以"动作图"是数据驱动的，不需要为每个实例重做逻辑。

### 5.3 时间模型（**观察**）

统一 60 fps；`end_frame` 以帧计；`loop_frame` 标循环点；每条动作最多 2 个 `sync` 点用于相位对齐；动作可带 1–4 条 `sequences`（内嵌时间轴 + 过滤轨道），用于动作内的附加数据（推断：脚步/特效/部件动作等游戏侧事件）。过场动作更长（抽样 661 帧、311 关节、203 通道）。

### 5.4 程序化层（**观察文件 + 推断用途**）

腿-脊柱 IK（碰撞过滤射线）、多骨 IK、注视、落地减震（`EmShockAbsorberState.user`）、按步态的弯曲滤波（walk/dash/fly）、根位移银行（walk/dash/air + 设置）、关节 LOD。它们与动作列表并列存放，属于"采样后修正"。

### 5.5 次级运动（**观察/推断**）

`chain2` 是动态骨链定义（1,354 个，覆盖角色头发/衣摆/尾巴、怪物尾/翼、绳索、吊挂链），引用角色碰撞过滤；`jcns` 是关节约束集（883 个，面部 `fcParam_*`）。二者在骨骼采样后由物理/约束求解，属于"动画 → 物理"的衔接层。

### 5.6 面部（**观察**）

`motion/facial/<角色>/list/*.motlist`：抽样 `AlmaState` 含 3 条动作 + 1 个 `AlmaFaceTree`，动作 361 关节、159 通道、2766 帧；共用 `facebasevariable.uvar` 与 `fcXXX_0000_rb` 银行；面部关节资产在 `art/model/character/ch00/commonjntassets`。**推断**：面部表情是"身体动作 + 面部通道 + 约束"的组合，而非独立骨骼动画系统。

### 5.7 相机与过场（**观察**）

`gamedesign/event/**/camera/*.mcamlist`（103 个）与事件角色私有 motbank（bank 5000）按事件打包；345 个事件目录三元组。**推断**：过场以"每角色一条动作列表 + 一组相机列表"组织，由事件时间轴统一驱动。

### 5.8 LOD（**观察**）

两级：`jointlodgroup` 控制每帧更新哪些关节；`motion/common/animationlod/*` 提供全局动画 LOD（默认参数、相机距离参数、IK/关节控制 LOD、面部 LOD）。

## 6. 资产组织与分类体系

| 类别 | 组织要点（**观察**） |
| --- | --- |
| 玩家 | 42 个通用银行 + 14 把武器各自 bank 20/21 + 64 个武器机关目录；另有 Mirror 变体（`*_Mir`）和女性变体（bank 200/201） |
| 怪物 | 每只怪 8 套动作库 + 动作图 + 断尾；`common/` 放 IK/弯曲/根位移/关节 LOD；体量差异大（em0071 764 条、em0160 680 条含外链） |
| NPC | 按人群与场景（male/female/old/child/airou/…）；大量独立 `.mot`（如 `npc/male/desert` 948 个）说明按变体/复用拆文件 |
| 伙伴（otomo） | 14 个编号 + 动作图 + 剧情；独立 charamake |
| 面部 | 每角色列表 + 共用基础变量 + 面部关节资产 |
| 对话 | 按声线/年龄分目录（`talk_male.motbank`） |
| 机关/道具 | `gimmick/itemmodel` 按模型 ID（`ch07_*`、`itXXXX`）建银行 |
| 相机 | 玩家动作相机 1 个银行 + 过场相机 103 个列表 |
| 其他 | `playernpc`、`npc_cat`、`guildcard`、`porter` 各自一套 |

## 7. 制作与导出链（**观察**）

- 解包工具 ReExtractor v1.4.4；动画导出走 Blender：`tools/export_animations_fbx.py`（GLB → FBX，60 fps，厘米单位，保留绑定姿态参考网格）与 `export_models_fbx.py`。
- 导出命名 `motionNNN_idN.fbx`，`idN` 即 motlist 的 `motion_id`；`批量动画/` 中 1,374 个 FBX 与 em0071/em0160 主线内嵌动作逐套相等（如 em0071_02 = 266，日志 `REEXTRACTOR_PROGRESS:266/266`）。
- 未导出项：`50/` 变体的 164 条、`em0160_05` 的 76 条外链动作（`批量动画/` 无对应目录）。

## 8. 关键数字（**观察**）

| 项 | 数量 |
| --- | --- |
| 解包文件总数 | 265,677（文件表 265,678） |
| 动画目录 `motion/` | 11,100 文件 |
| motbank / motlist / 独立 mot | 3,161 / 5,250 / 5,672 |
| 动作图列表（`*_tree.motlist`） | 129（enemy 84、npc 22、player 19、其他 4；em0071 的列表 234 个 mtree、em0160 267 个） |
| 控制器 mct / 事件 mex / 银行 mcb / meb | 1,154 / 1,164 / 184 / 188 |
| chain2 / jcns | 1,354 / 883 |
| jmap / jointlodgroup / ikls / ikmulti / iklookat2 | 88 / 64 / 74 / 26 / 49 |
| uvar / mcamlist | 12 / 118 |
| 过场事件目录 / 过场相机 | 345 / 103 |
| 大型怪物样本动作 | em0071 764 + 3 断尾；em0160 680（含 76 外链）+ 3 断尾 |

## 9. 未知与验证边界

1. **运行时绑定未知**：本目录没有场景/预制体 RSZ 与可执行文件，无法证明某个行为用哪个 bank、哪几层、mask 是什么；第 5 节的分层模型来自公开工具实现 + 数据组织反推。
2. **动作图求值语义未知**：`OctSyncBlendNode`/`VariablesNode` 的求值顺序按类名、连线与参数推断。
3. **`_mct/_mex/_mcb/_meb` 是哈希化 RSZ 用户数据**，缺 schema，只能确认引用关系。
4. **`chain2`/`jcns` 内部字段未解码**（仅确认 magic/版本、碰撞过滤引用与 `fcParam_*` 名称），物理参数与求解顺序未验证。
5. **相机条目结构未解码**（只确认列表头与计数），`motcam` 仅 1 个样本。
6. **怪物/角色显示名未知**：`emXXXX`/`chXX_XXX` 是内部 ID。
7. **未做运行时验证**：无游戏进程、无 Hook。

## 10. 对 UE5 的映射（简）

1. **银行分层 → Anim Layer + Slot**：按行为域拆 bank，等价于按层级/掩码组合动画；`jmap` 对应 Layered Blend per Bone 的骨骼集。
2. **动作图 → Sync Group / Blend Space**：2–3 路同步混合 + 变量驱动，可直接映射到 Blend Space 的轴与 Sync Group 的标记。
3. **序列/同步点 → Anim Notify + Sync Marker**：保留动作内事件与相位对齐，而不是靠时长比例。
4. **程序化层 → AnimGraph 后处理**：IK、减震、弯曲、根位移分别做成可复用节点，按怪物体型参数化。
5. **次级运动 → 物理资产**：`chain2` 对应 Chaos 物理链/AnimDynamics；`jcns` 对应约束。
6. **LOD 两级 → 骨骼 LOD + 更新频率 LOD**。
7. **过场/相机独立管线**：事件内角色动作与相机分离，对应 Level Sequence 的 Actor 轨道 + Camera 轨道。
