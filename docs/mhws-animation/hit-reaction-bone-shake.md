# MHWs 受击骨骼抖动（Hit Reaction）技术方案

> 本文由自动代理生成（tail-bot717），分析对象是本 ticket 授权的 `D:\re解包\output\unpacked`（Monster Hunter Wilds / RE Engine）。
> 证据分级：**观察**＝文件字节、容器头或曲线轨道直接读出；**推断**＝由命名、引用关系与公开工具实现推得，未做运行时验证；**未知**＝当前材料无法判定。
> 配套文档：[动画系统技术方案总览](animation-system-technical-overview.md)、[大型怪物动画系统逆向分析](monster-animation-reverse.md)。

## 1. 结论

RE Engine / MHWs **没有单独的“骨骼抖动参数”资产**，受击时的骨骼抖动是四层叠加的结果：

1. **受击动作本身是高频关键帧**（**观察**）：`plc_Damage` 的脊柱/髋/颈/头在 60 fps 下几乎每 1.0–1.5 帧一个关键帧（Head 129 键 / 128 帧），而普通待机/演出动作是每 2.8–400 帧一个键。
2. **受击动作走独立的动作集 + 同步混合**（**观察**）：每个角色有一份 `*_Damage` 动作列表；`plc_Damage_tree` 用 `OctSyncBlendNode` 把 3 条受击变体按变量同步混合。
3. **可叠加到基础动作上**（**观察/推断**）：运行时 layer 支持 `AddBlend`，并有 9 条“加性基座”动作（`plc_AddBase`，1 帧、63–84 通道），受击层可用 joint mask 只影响上半身。
4. **程序化与反馈层放大抖动感**（**观察/推断**）：敌人的 shock absorber（腿/髋吸收冲击）、按步态的 bendfilter、1,354 条 `chain2` 动态链（尾/毛/布）产生跟随抖动；同时有 hitstop（顿帧）与相机抖动（per-weapon `*_csk`、`EmCommon_AreaCameraShake`）。

一句话：**抖动是“动画数据”而不是“运行时噪声”**——高频关键帧的受击片段 + 三路同步混合（+ 可选加性层）+ 物理次级运动 + 顿帧/相机抖动。

## 2. 受击动作体系

**观察**：`gamedesign/enemy/em0071/00/action/em0071_00_actionid.user.3` 列出 300 个 action，其中 103 个与受击相关，例如：

```
cDamageMinFourLeg / cDamageMinSixLeg / cDamageMinFly
cDamageMaxFourLeg / cDamageMaxSixLeg / cDamageMaxFly
cDamageDownFourLeg / cDamageDownSixLeg / cDamageDownHeadSixToFourLeg
cDamageSleep… / cDamageParalyse… / cDamageStun… / cDamageTrap…
cDamageLostTailFourLeg / cDamageLostTailSixLeg
cDamageFallDown… / cToDamageFallDown…
```

- 同一只怪按**姿态**（四足/六足/飞行）与**受击程度**（Min/Max/Down/状态异常/断尾/坠落）拆分动作，由 `btable_damage`、`btable_reactiongm` 等行为表选择。
- 每个角色有独立受击动作列表（**观察**）：玩家 `plc_Damage`（295 条、785.5 秒、60 fps、72 通道、1–6 条序列）、`wp_Damage`、NPC 的 `*_Reaction`（如 `npc_Stand_Airou_Reaction`）。

## 3. 关键帧密度：抖动“写”在曲线里

用 `tools/mot_inspect.py tracks` 统计单个动作内各关节的关键帧间距（**观察**，60 fps）：

| 动作 | Head 关键帧 | 帧范围 | 平均间距 |
| --- | --- | --- | --- |
| `plc_Damage_002`（受击） | 129 | 0–128 | **1.00 帧** |
| `plc_Damage_040`（受击） | 160 | 0–162 | **1.02 帧** |
| `plc_lobby` 待机 | 107 | 0–295 | 2.76 帧 |
| `plc_gesture` 手势 | 9 | 0–230 | 25.6 帧 |
| `plc_variety` 演出 | 3 | 0–1298 | 432 帧 |

受击动作的髋（Hip 122 键/128 帧）、颈（Neck 129/128）、肩（R_Shoulder 111/128）同样密集；`plc_Damage_002` 共 90 条轨道、5 条序列（事件时间轴）。这解释了受击时“肉眼可见的骨骼抖动”——**它是逐帧作者化的高频曲线**，不是引擎侧后处理。

## 4. 动作树：三路受击变体

**观察**：`motion/player/common/plc_damage_tree/plc_damage_tree.motlist.992` 只有一个 `mtre`：

- 5 个节点：3 × `MotionNode` → 1 × `OctSyncBlendNode` → 1 × `VariablesNode`
- `MotionNode` 参数：`BankID=7`、`MotionID=350/351/352`
- `VariablesNode` 引用 `Motion/Common/Variable/CharacterCommonVariables.uvar`
- choice 表：`(bank 7, motion 352/350/351)`

**推断**：受击反应不是单条动作，而是按变量（受击方向/强度）在 3 条变体间做同步混合；变量表是角色类别共享的，因此同一套混合逻辑可用于所有受击来源。

> 注意（**未知**）：该 list 的条目里没有 350–352（其 `motion_id` 最大 522 且不含这三个值），说明树里的 MotionID 与列表 `motion_id` 不是直接对应，运行时如何解析 bank/motion 需要场景/运行时确认。

## 5. 加性层与掩码

**观察**：

- `motion/player/common/plc_addbase/plc_addbase.motlist.992` 有 9 条动作，全部 **1 帧**、63–84 通道——典型的“加性基座”（additive base pose）。
- 全库 129 个 `*_tree` 列表使用的节点类型：`MotionNode` 13,260、`OctSyncBlendNode` 5,526、`VariablesNode` 5,315、`SyncBlendNode` 278、`RangeNormalizeNode` 85、`ParameterNode` 56、`RotationGlobalOffsetNode` 23、`BlendRateNode` 23；**没有** `AddBlendNode`。
- 因此加性合成不在动作树里，而在**运行时 layer**（`via.motion.Motion` 的 AddBlend 模式 + joint mask，公开工具 REasy 的实现可读这些字段）。

**推断**：受击抖动通常作为“上半身/局部层”叠加在 locomotion 上，用 joint mask 限定骨骼（例如脊柱以上），这样腿脚仍保持移动，上半身单独抖。

## 6. 程序化与反馈层

| 层 | 证据（**观察**） | 作用（**推断**） |
| --- | --- | --- |
| Shock absorber | 26 个 `*_shockabsorberparameter.user.3`（每怪一份，2,184 B）+ `em0082_shockabsorber.user.3` + `EmShockAbsorberState.user`（状态：`ALWAYS/DEFAULT/DEAD/EM0157_VEILOFF`）；参数含 -90/180 角度与 3.0/0.3/2.0 等速率 | 腿/髋 IK 吸收冲击，身体整体“沉一下” |
| Bend filter | 52 个 `*_bend{walk,dash,fly}.user.3`（按步态） | 脊柱/躯干程序化弯曲，冲击后产生回弹 |
| Chain2 物理链 | 1,354 个 `*.chain2.13`，引用 `Character_Chain.cfil`/`Character_Default.cfil` 碰撞过滤 | 尾/毛/布/绳索的跟随抖动（受击后的二次运动） |
| Hitstop | `playerhitstopparam.user.3`、`PlayerHitStopType`（40+ 攻击类型：SLASH/STOMP/PARRY/THRUST…） | 命中瞬间冻结/减速，强化“顿挫+抖动”观感 |
| 相机抖动 | 每武器/通用 `*_csk.user.3`、`EmCommon_AreaCameraShake.user`、`emcameraimpactparamlist` | 屏幕抖动放大冲击，不属于骨骼动画 |

## 7. 还原出的执行链（伪代码）

```text
on_hit(hit_info):
    action = select_action(cDamageMin/Max/Down/..., stance, hit_dir)  # 行为表
    hitstop.play(PlayerHitStopType, frames)                            # 顿帧
    layer = motion.add_layer(bank=Damage, motion=action,
                             blend=AddBlend|Override, mask=upper_body)
    layer.blend  = damage_tree.blend(3 variants, variable=hit_dir/intensity)
    sample -> post_process:
        shock_absorber(hips/legs, impact)
        bend_filter(spine, gait)
        chain2_sim(tail/hair/cloth)
    camera.shake(weapon_csk) ; vfx ; pad_vibration
```

（伪代码是**综合推断**：动作选择、顿帧、层级叠加与后处理都能在数据里找到对应资产，但调用顺序未在运行时验证。）

## 8. 对 UE5 的落地建议

1. **用 Additive Animation + Bone Mask** 做受击层：基础 locomotion 不受影响，上半身/局部骨骼叠加受击抖动；对应 RE 的 AddBlend + joint mask。
2. **用同步混合 3 个方向/强度变体**（Blend Space 或 Sync Group + 变量驱动），不要为每个方向各做一套状态机。
3. **抖动写进动画曲线**：30–60 Hz 的关键帧密度足够，运行时不要再叠加全局噪声（会与物理链打架）。
4. **二次运动交给物理**：尾/毛/布用 AnimDynamics / Physics Asset，受击时给一个冲量；这与 RE 的 chain2 等价。
5. **反馈层单独做**：HitStop（Custom Time Dilation / 暂停动画采样）、CameraShake、音效/顿帧，不要塞进骨骼动画。
6. **冲击吸收**：用 IK（腿/髋）或 Additive 位移做一个短促下沉，对应 shock absorber。

## 9. 未知与限制

1. **动作→motion 的运行时映射未验证**：`actionparam.user.3` 是哈希化 RSZ，缺 schema；行为表到具体 motion 的映射未解出。
2. **`plc_Damage_tree` 的 MotionID 350–352 与列表 id 不一致**（见 §4），树内 bank/motion 的解析方式未知。
3. **shock absorber / chain2 / jcns 的内部参数未解码**（只确认结构、引用与少量浮点），物理求解细节未验证。
4. **未做运行时验证**：没有游戏进程/Hook，“抖动观感由哪几层叠加”是结构推断，不是实测。
5. 本文只覆盖本线程一直在分析的 MHWs 解包数据；若“这个项目”指的是 Sun/UE5 工程，需要另开 p4 工单看那边的实现。

## 10. 复现

```bash
# 受击动作列表与动作数量
python tools/mot_inspect.py list  ".../motion/player/common/plc_damage/plc_damage.motlist.992" --first 5

# 单条受击动作的逐关节关键帧密度（Head/Hip/Neck…）
python tools/mot_inspect.py tracks ".../motion/player/common/plc_damage/plc_damage.motlist.992" --motion 2

# 动作树（3 路同步混合）
python tools/mot_inspect.py list  ".../motion/player/common/plc_damage_tree/plc_damage_tree.motlist.992"

# 加性基座
python tools/mot_inspect.py list  ".../motion/player/common/plc_addbase/plc_addbase.motlist.992"

# 受击 action 词表
# 读 em0071_00_actionid.user.3 的 UTF-16 字符串（cDamageMin/Max/Down…）
```

关键帧密度 = 轨道 `key_count` 与首末帧差（`tracks` 命令直接输出 `spacing`）。
