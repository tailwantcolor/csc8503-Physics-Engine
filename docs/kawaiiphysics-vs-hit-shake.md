# KawaiiPhysics 与 MHWs 受击程序化抖动方案对比

> 本文由自动代理生成（tail-bot717）。
> KawaiiPhysics 一侧依据源码 [pafuhana1213/KawaiiPhysics](https://github.com/pafuhana1213/KawaiiPhysics) `3c8b5fd`（2026-11-08，UE 5.3–5.8，MIT）；行号指 `Plugins/KawaiiPhysics/Source/KawaiiPhysics/` 下的文件。
> MHWs 一侧依据 [PR #6](https://github.com/tailwantcolor/csc8503-Physics-Engine/pull/6) 的《受击程序化抖动设计拆解》《受击抖动在骨骼链间的传递、衰减与姿态影响》；那边的字段语义是从数值形态**推断**的（RSZ 字段名为哈希），本文沿用同样的证据等级。

## 1. 一句话结论

- **KawaiiPhysics 是"次级运动求解器"**：一个 AnimGraph 节点，对一条/多条骨链在**位置空间**跑 verlet 伪物理（速度重建 → 阻尼 → 重力/风/外力 → 拉回动画姿态 → 角度限制 → 碰撞 → 恢复骨长 → 反求旋转）。它解决的是"头发、裙子、饰品跟着动画和移动自然摆"，**本身没有受击概念**。
- **MHWs 方案是"受击反馈管线"**：动画采样之后叠四层——① 冲击吸收（关节**角空间**弹簧-阻尼 + 限位）、② 弯曲滤波（按步态分档限幅）、③ chain2 物理链、④ 顿帧 + 相机。
- **对应关系**：KawaiiPhysics ≈ 第 ③ 层（chain2）的同类物；①②④ 在 KawaiiPhysics 里没有对应模块，要靠外力脉冲、参数倍率和别的系统拼出来。

## 2. 逐维度对比

| 维度 | KawaiiPhysics | MHWs 受击抖动 |
| --- | --- | --- |
| 设计目标 | 次级摆动（发、裙、尾、饰品），常驻 | 受击反馈：被打偏 → 回弹 → 余震，事件驱动 |
| 作用对象 | 指定 RootBone 以下的末端摆动链（可多根、可加 Dummy 骨） | 身体主干关节（头/颈/脊柱/四肢/尾）+ 尾毛布等链 |
| 求解空间 | **骨骼位置**（verlet 质点），最后 `FindBetweenVectors` 反求父骨旋转 | ①② 在**关节角度**上直接操作；③ 才是质点链 |
| 积分方式 | `v=(x-x_prev)/dt` → `v*=(1-Damping)` → `+wind +g·dt` → 外力 → `x+=v·dt` | ① 推断为二阶弹簧-阻尼 `a=-kx-cv`；② 一阶跟随 `f+=(t-f)·w`；③ verlet |
| 回到动画姿态 | `Stiffness`：每步把骨拉向"父骨当前位置 + 动画中的父子偏移"，`1-(1-S)^Exponent` | ① 弹簧把偏移量拉回 0；② 滤波器跟随动画目标 |
| 参数粒度 | 节点级参数 × 按"离根长度比"（0–1）的曲线：Damping/Stiffness/WorldDamping/Radius/LimitAngle 各一条 | 每只怪 10 组关节参数（限位/速度/阻尼/回弹速率）；bend 按骨骼组 × 步态（walk/dash/fly）；chain2 按链 |
| 沿链衰减 | 由曲线随长度比连续变化 + verlet 的父子位置约束逐节传递 | 分段：每关节独立阻尼/限位，FK 把上游偏移带给下游 |
| 限位 | 锥形 `LimitAngle` 或逐轴 `PerAxisLimitAngle`（相对父骨，椭圆最近点投影）；平面约束 | 每关节角度区间（如 -90°~180°、-30°~120°）；bend 5/10/30/50° 限幅 |
| 受击触发 | **无内置**；可用 AnimNotify「Add ExternalForce (Pulse)」、`RequestTransientExternalForce`、`RequestTransientGust`（Rise/Hold/Decay 包络）、`RequestStartPhysicsSettingsMultiplier`（临时改刚度/阻尼） | 受击事件给 shock absorber 注入冲量（来源未解出），伴随顿帧与相机 |
| 状态开关 | `Alpha`/`SetAlpha` 通知、LOD、Multiplier 租约 | `EmShockAbsorberState`：ALWAYS / DEFAULT / DEAD / 特殊形态 |
| 碰撞 | 球/胶囊/锥胶囊/盒/平面、PhysicsAsset、共享碰撞、简易世界碰撞、XPBD 骨间约束（裙子防穿腿） | chain2 用角色专用碰撞过滤 `Character_Chain.cfil`；①② 只靠角度限位防穿 |
| 帧率 | 默认固定子步 `FixedDt=1/TargetFramerate`（默认 60），有最大子步上限 | 未知（无运行时验证） |
| 时间层效果 | 无顿帧/相机；用 Time Dilation 做顿帧时它的 dt 同步变慢，只把动画播放速率设 0 时它仍照常积分 | ④ 顿帧 + 相机抖动是管线的一部分 |
| 成本 | 不依赖 PhysX/Chaos，每骨几个向量运算 + 碰撞；适合大量角色 | ①② 是每关节标量运算，很便宜；③ 与 KawaiiPhysics 同量级 |

## 3. 两边核心算法对照

**KawaiiPhysics（源码）**——`AnimNode_KawaiiPhysicsSimulation.cpp`：

```cpp
// Simulate(), L1305；每个子步对每根骨
v  = (Location - PrevLocation) / dt_old;          // ComputeVerletStepVelocity, L1445
v *= (1 - Damping);
v += Wind + Gravity * dt;
ExternalForce.ApplyToVelocity(v);
Location += v * dt;                                // IntegrateVerletStepPosition, L1475
Location += SimpleExternalForce * dt;
Location += SkelCompMove * (1 - WorldDampingLocation);   // 跟随角色位移/旋转
ExternalForce.Apply(Location);                     // 外力直接改位置（Basic 按骨长曲线缩放）
Base = Parent.Location + (PoseLocation - Parent.PoseLocation);
Location += (Base - Location) * (1 - pow(1 - Stiffness, Exponent));  // ApplyStiffnessPull, L1506
// 之后：骨间约束 → 各类碰撞 → RestoreBoneLengthsAndLimits(L1251：角度限制 + 恢复骨长)
// 输出：ApplySimulateResult(L1717) 用 FindBetweenVectors(动画父子向量, 模拟父子向量) 改父骨旋转
```

**MHWs 冲击吸收（推断）**——角空间二阶系统：

```
on_impact(axis, impulse): v += impulse
each_frame: a = -k*x - c*v; v += a*dt; x += v*dt; x = clamp(x, min, max)
apply_to_joint(axis, x)        // 叠加在动画之上，FK 带动子树
```

关键差别：

1. **"拉回去"的对象不同**。KawaiiPhysics 的 Stiffness 是一个**位置插值**（每步把一部分距离拉回去），不带速度记忆，对应一个**过阻尼**的一阶回归；它的"弹"来自 verlet 的惯性。MHWs 冲击吸收是显式的**二阶弹簧**，阻尼 0.1–0.3 时会有可控的过冲次数，这是受击"晃几下"的核心。
2. **自由度不同**。KawaiiPhysics 每根骨只有位置（3 自由度），旋转是反求出来的，**骨自身的扭转（roll）不会被模拟**；MHWs 冲击吸收按关节/轴给参数，可以只抖某一个轴。
3. **作用位置不同**。KawaiiPhysics 只改它管的那条链（通常是末端附件），不适合直接挂在脊柱/颈这类承重主干上——那样它会跟 IK、动画叠加打架，且 Stiffness 太低时整个躯干会"软掉"。MHWs ①② 正是给主干用的。
4. **触发模型不同**。KawaiiPhysics 是"持续求解 + 偶尔加外力"；MHWs 是"受击事件驱动"，冲量大小/方向、顿帧时长都按攻击类型查表。

## 4. 骨骼链上的传递与姿态影响

| 问题 | KawaiiPhysics | MHWs |
| --- | --- | --- |
| 怎么沿链传 | 每根骨的拉回目标 = **父骨模拟后位置** + 动画偏移，再恢复骨长 → 父骨一动，子骨被硬约束带着走，同时自带惯性滞后 → 天然"甩鞭" | FK：父关节角偏移带动整棵子树；chain2 才有质点式滞后 |
| 怎么衰减 | Damping 曲线（按离根长度比）+ Stiffness 曲线 + 碰撞/约束耗能 | 每关节阻尼与限位分段衰减；bend 权重滤掉高频 |
| 姿态影响 | 拉回目标取自**当前动画姿态**，限位相对**父骨本地轴**；所以姿态改变方向、限位余量；重力作用在质点上，下垂/横置链频率不同 | 角偏移在关节本地空间施加，同样受方向、杠杆臂、限位余量、重力力矩影响 |
| 末端放大 | 通过外力曲线 `ForceRateByBoneLengthRate` 或 Damping 曲线手动补偿 | 杠杆臂自然放大（尾 8.7 单位，1°≈0.15） |

## 5. 用 KawaiiPhysics 做受击抖动的可行组合

KawaiiPhysics 可以负责"余震"（第 ③ 层），其余层要另外补：

1. **③ 余震（直接用 KawaiiPhysics）**：尾、毛、翼膜、饰品各一个节点；受击时在受击动画上放 `KawaiiPhysics: Add ExternalForce (Pulse)` 通知，方向取命中方向，用 `ForceRateByBoneLengthRate` 让末端更大；或在 C++/BP 调 `RequestTransientGust(Strength, Rise, Decay, Dir)` 得到带包络的冲击。
2. **受击瞬间"变软"**：`RequestStartPhysicsSettingsMultiplier` 在 Rise/Hold/Decay 时间内临时降低 Stiffness、Damping，结束后自动回到基础值——相当于 MHWs 按状态切参数。
3. **① 冲击吸收（需另做）**：KawaiiPhysics 没有角空间二阶弹簧。建议用 Control Rig 的 Spring Interp / 自写 AnimNode：每个主干关节一组 (k, c, min, max)，受击时注入角速度；放在 KawaiiPhysics 节点**之前**，让附件链自然继承主干的抖动。
4. **② 弯曲滤波（需另做）**：一阶跟随 + Clamp，参数按步态做 DataAsset。
5. **④ 顿帧/相机（需另做）**：Time Dilation / 动画暂停 + `UCameraShakeBase`。注意顿帧若只暂停动画播放，KawaiiPhysics 仍在积分：要么让它也"停"（用角色 Custom Time Dilation 顿帧），要么接受顿帧结束时附件已经先甩起来——后者往往更有冲击感，按需取舍。
6. **推荐节点顺序**：动画采样 → ① 冲击吸收 → ② 弯曲滤波 → KawaiiPhysics（③）→ 脚部 IK。

## 6. 选型建议

- 只需要"挨打时尾巴/头发甩一下"：KawaiiPhysics + Pulse 外力 + 参数倍率就够，成本最低。
- 需要"大型怪物挨打时身体被打偏再回弹"：KawaiiPhysics 不够，必须补第 ① 层角弹簧；这是 MHWs 方案相对 KawaiiPhysics 最大的差别。
- 需要按怪物/部位/步态精细调参：KawaiiPhysics 的曲线按"离根长度比"连续变化，适合同质链；MHWs 的"每关节独立参数 + 步态分档"更适合异质的主干，需要自建参数资产。

## 7. 局限与未知

1. MHWs 一侧的字段语义（阻尼/速率/限位）、冲量来源、时序都是推断，没有运行时验证；本文对比中凡涉及 MHWs 的数值都继承这个不确定性。
2. KawaiiPhysics 一侧只读了核心模拟路径与外力/通知接口，没有在 UE 里跑示例工程实测手感。
3. KawaiiPhysics 更新很快（本文基于 2026-11-08 的 `3c8b5fd`），后续版本可能新增与受击相关的功能。

## 8. 复现

```bash
git clone --depth 1 https://github.com/pafuhana1213/KawaiiPhysics.git
cd KawaiiPhysics/Plugins/KawaiiPhysics/Source/KawaiiPhysics
# 核心步进
sed -n '1305,1513p' Private/AnimNode_KawaiiPhysicsSimulation.cpp
# 角度限制与骨长恢复、输出旋转
sed -n '1251,1281p;1717,1791p' Private/AnimNode_KawaiiPhysicsSimulation.cpp
# 一次性外力 / 包络冲击 / 参数倍率接口
grep -n "RequestTransient\|RequestStartPhysicsSettingsMultiplier" Public/AnimNode_KawaiiPhysics.h
cat Public/AnimNotifies/AnimNotify_KawaiiPhysicsAddExternalForce.h
```
