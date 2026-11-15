# MHWs 动画系统逆向分析（RE Engine 数据视角）

> 目的：把《Monster Hunter Wilds》解包数据里"动画系统怎么运作"整理成一份可追溯、可复现的技术资料，重点是大型怪物。
> 对象：请求方授权的只读解包目录 `D:\re解包\output\unpacked`（STM / Release）。原始数据不进仓库，仓库只存分析结论与解析器。
> 性质：只做技术分析，不提取、不复用游戏资产；结论按证据强度分级，未做运行时验证的部分明确标为推断。

## 这份资料回答什么

- **资产怎么组织**：动作不是"一动作一文件"，而是 `motbank → motlist → mot / mtree` 的四层引用图，按角色类别（npc / enemy / player / …）分目录。
- **容器长什么样**：`motbank.4`、`motlist.992`、`mot.932`、`mtree.21` 的字节级布局，全部按实际文件解析验证。
- **运行时怎么拼**：`via.motion.Motion` 的多 layer 叠加、`mtree` 的三路同步混合、joint mask 与加性层，采样后的 IK / 减震 / 弯曲滤波 / 物理链 / LOD 后处理。
- **大型怪物专项**：四足 / 六足 / 飞行姿态拆分、断尾动作列表、每怪 IK 与 bendfilter、根位移银行。
- **受击抖动**：数据层高频关键帧 + 三路同步混合 + 加性层，加上程序化四层后处理，以及抖动在骨骼链间的传递、衰减与姿态影响。
- **怎么迁移**：每篇末尾都有对 UE5 / Sun 的落地建议。

## 文件地图

| 文件 | 回答的问题 | 主要证据 |
| --- | --- | --- |
| `animation-system-technical-overview.md` | 整个动画系统的技术方案：资产分层、容器格式、运行时叠加、资产分类体系 | 文件表与文件字节（观察）＋ 组件与树结构（推断） |
| `monster-animation-reverse.md` | 大型怪物逐字段深挖：单只怪的资产形态、容器布局、运行时链路、1,374 条动作统计 | 字节解析 + FBX 导出日志 + Blender 交叉验证 |
| `hit-reaction-bone-shake.md` | 受击骨骼抖动从哪来：高频受击曲线、`*_Damage` 动作集、三路受击树、加性层与掩码 | 关键帧间距、动作列表、tree 节点 |
| `procedural-hit-shake.md` | 程序化抖动怎么设计：冲击吸收、弯曲滤波、物理链、顿帧/相机四层拆解与调参手册 | `*_shockabsorberparameter`、`*_bend*.user`、`*.chain2` 参数 |
| `hit-shake-propagation.md` | 抖动如何在链间传递与衰减、骨骼自身姿态的影响 | 骨架层级、每关节/每链参数、杠杆臂换算 |
| [`tools/mot_inspect.py`](../../tools/mot_inspect.py) | 复现上述数字的只读解析器（零第三方依赖，Python 3.9+） | 实现 `bank / list / mot / tree / inventory / tracks` |

解析器留在仓库顶层 `tools/`（沿用仓库既有布局），文档里的命令都以**仓库根目录**为工作目录。

## 阅读顺序

1. **先看全局**：`animation-system-technical-overview.md` 第 1、3、4、5 节，建立"四层引用图 + 分层叠加"的模型。
2. **再看单体**：`monster-animation-reverse.md`，用 em0071 / em0160 把模型落到具体字节和具体数字。
3. **受击抖动按三条线读**：先 `hit-reaction-bone-shake.md`（数据层怎么产生抖动）→ 再 `procedural-hit-shake.md`（程序化四层怎么放大）→ 最后 `hit-shake-propagation.md`（怎么沿链传递、衰减、受姿态影响）。

## 证据分级约定

每篇开头都声明三种标记：

- **观察**：目录、文件表或文件字节直接读出，可复现；
- **推断**：由命名、引用关系、结构位置和公开工具实现推得，未做运行时验证；
- **未知**：当前材料无法判定，列在文末"未知与限制"。

看到"推断"就当成待验证假设；要升级为"观察"，需要补游戏可执行文件或场景/预制体数据做运行时确认。

## 复现

```bash
# 动作库（银行表）
python tools/mot_inspect.py bank "D:/re解包/output/unpacked/natives/stm/motion/enemy/em0071/00/em0071_00.motbank.4"

# 动作列表（条目、ID、标签、时长）
python tools/mot_inspect.py list "D:/re解包/output/unpacked/natives/stm/motion/enemy/em0071/00/em0071_02/em0071_02.motlist.992"

# 单个动作的关键帧轨道统计
python tools/mot_inspect.py tracks "D:/re解包/output/unpacked/natives/stm/motion/player/common/plc_damage/plc_damage.motlist.992" --motion 2

# 目录级清单
python tools/mot_inspect.py inventory "D:/re解包/output/unpacked/natives/stm/motion/enemy/em0071"
```

脚本只读、不改动被检查的文件；参数路径需指向本机授权解包目录，路径不存在时直接以 `FileNotFoundError` 报错（只有 `--help` 不需要数据目录）。

## 边界

- 全部结论来自磁盘数据与导出日志，**没有运行时验证**；运行时行为（layer 混合权重、参数实际生效值）属于推断。
- 仓库不含游戏原始数据、FBX 导出物或任何可复用的美术资产，只有 Markdown 分析与解析器代码。
- 数据授权范围由请求方提供；本目录不记录、也不分发原始内容。
