# 四轴机床轨迹详细设计

## 职责

四轴轨迹生产链将有效 `Rotary4Axis ProcessPlan` 转换为按加工顺序排列的 XYZ/A 机床运动。
该链同时支持无方管截面的普通四轴和带有效截面的截面增强四轴。
轨迹层负责安全移动、连续连接、A 轴展开、加工面 Z 修正和闭合过切。

## 生产入口

- 四轴导出进入 `NcProgramService::buildRotaryProgram()`。
- 该服务调用 `MachineTrajectoryService::buildRotaryTrajectory()` 构建轨迹。
- 轨迹服务捕获当前文档几何，按计划分配重新编译每条生产路径。
- `RotaryTrajectoryBuilder` 调用 `RotaryKinematics` 生成机床姿态并组织安全与切削运动。
- 当前轨迹仅在导出过程中按需生成，主窗口不长期保存轨迹对象。

## 输入与输出

输入包括当前文档、加工状态、有效四轴计划、可选方管截面、四轴配置、四轴转移配置和可选显式截面中心。
计划提供顺序、方向、起点和连续组；几何快照提供不可变源几何。
输出 `MachineTrajectory`，包含旋转上下文和按实体组织的接近、切削、连接及过切运动。
输出继续传给 `NcProgramBuilder`，轨迹层不生成 G-code 文本。

## 主要数据类型

- `GeometrySourceSnapshot`：在文档线程捕获的精确源几何和稳定实体属性。
- `TrajectoryEntityInput`：按计划方向、起点或计划片段编译后的 `Path3D`，同时区分源 assignment 顺序和实际执行片段顺序。
- `RotaryTrajectoryInput`：实体路径、计划分组、revision 和可选截面。
- `ToolClearancePolicy`：仅服务平面三轴安全移动的退刀与接近距离。
- `ToolTransferPolicy`：四轴加工单元间转移策略，包含旋转安全抬刀距离、同区空移离面距离和联动开关。
- `TransferMotionKind`：区分首次接近、同区表面空移、同区离面空移和跨区旋转转移。
- `TransferMotionPhase`：标识表面空移、联动离开、安全旋转和联动接近阶段。
- `PlanarTrajectory`：按加工单元边界保存三轴抬刀、转移、接近和落刀姿态，NC 层只负责将这些已确定姿态编码为快速运动。
- `RotaryMachinePolicy`：旋转轴、截面中心、四轴转移策略、Z 修正、过切和角度策略。
- `ProcessUnitExecutionResult`：保存实际片段序列、逐片段机床姿态、切削起点、闭环补齐、过切目标及最终源空间和机床位置。
- `RotarySurfaceRegion`：轨迹构建期间使用的顶、右、底、左、圆角、径向或未知表面分类。
- `TubeZone16`：规划阶段确定的生产区位；轨迹输入携带每个加工单元的最终 `ownerZone`，仅用于转移分类。
- `MachinePose4D`：单个 XYZ/A 机床姿态。
- `MachineMove`：快速、切削、连续切削连接或过切运动；快速转移同时携带转移类型和阶段。
- `MachineTrajectory`：轨迹实体、revision 和旋转安全上下文。

## 当前生产数据流

普通四轴：

```text
Rotary4Axis ProcessPlan
+ CadDocument
+ 旋转轴和运动配置
→ GeometrySourceSnapshot
→ ProcessUnitExecutionResolver 按 assignment 或 plannedFragments 展开实际执行路径
→ RotaryKinematics、连续 A 轴对齐、闭环补齐和过切终态
→ RotaryTrajectoryBuilder
→ MachineTrajectory
```

截面增强四轴：

```text
普通四轴输入
+ TubeSectionModel
+ 截面中心
→ 圆角匹配与表面方向
→ 截面边界上下文
→ 增强四轴 MachineTrajectory
```

中心选择：

```text
有效截面几何中心
→ 否则显式截面中心
→ 否则全部加工路径的 YZ 包围盒中心
```

## 状态所有权

`ProcessPlan` 由主窗口持有，`MachineTrajectory` 是导出调用中的临时值对象。
轨迹保存计划的文档 revision 和加工状态 revision，供 NC 阶段继续校验。
四轴配置由当前 `GProfile` 提供，构建时复制到 `RotaryMachinePolicy`。
方管截面由主窗口提供；业务状态按用户中心、自动中心、`(0,0)` 解析，并将有效中心同步到核心模型。
轨迹服务只读取可选核心模型和显式中心，不直接解析中心来源。

## 失效条件

- 计划模式不是 `Rotary4Axis` 时拒绝构建。
- 文档快照、计划、加工状态或截面 revision 不一致时拒绝构建。
- 计划未覆盖全部可加工且未排除的图元时拒绝构建。
- assignment 顺序不连续、源实体缺失或几何编译失败时拒绝构建。
- 旋转轴中心、截面中心或运动参数出现非有限值时拒绝构建。
- 旋转轴、转移距离、联动开关、初始位置、Z 修正、过切或 A 轴策略变化需要重新生成轨迹及下游结果。

## 关键业务规则

- 每条路径使用计划确定的 `reverse` 和 `startParameter` 重新编译，轨迹不重新排序。
- Break 断面和普通多图元闭环的计划片段使用与普通路径相同的生产采样策略编译完整源 `Path3D`，再按 `sourceParameterBegin`、`sourceParameterEnd` 和方向裁切。普通闭环内部入口可以来自 `Arc`/`Ellipse` 的真实几何切点，也可以来自 owner 区位可靠连续段的三维弧长中点；两者都通过真实源参数插值，不吸附到最近顶点。
- 同一闭环起点成员可以在轨迹执行序列中出现两个互补片段。`ProcessAssignment`、`ProcessUnitKey` 和 Viewer 身份仍只保存一次稳定 `EntityId`；轨迹通过 `sourceProcessOrder` 与 `fragmentOrder` 区分源计划身份和执行位置。
- `ProcessUnitExecutionResolver` 按展开后的片段顺序执行源空间和机床空间连续性检查。闭合组回起点和过切基于完整片段序列，不复制完整源图元。
- NC 构建器按通用轨迹执行块复用源图元元数据，并把块顺序重建为实际执行顺序；NC 与 G-code 层不判断 Break 类型。
- 智能懒旋转计划在有效方管截面下先冻结唯一 owner、Break 分区、16 区位顺序和每区 X 顺序，再精化普通多图元闭环以及自动起点的单图元圆和完整椭圆。结果固化在 `ProcessPlan`、assignment、转移预览签名和可选片段中，轨迹层不再次进行区位调度、入口选择或方向评分。
- 规划器使用前序实际切削终点、前后 `ownerZone` 和四轴转移策略计算 `previousTransferAnchor`。同区零离面时锚点为真实切削终点，同区离面时沿局部外法向偏移，跨区时使用旋转安全包络上的锚点；轨迹构建器复用同一组定义。
- 只有普通多图元闭环中的非闭合 `Arc` 和部分 `Ellipse` 使用真实几何切点优化；无有效切点时依次回退到 owner 区位最近连接点和可靠连续段中点，不使用极点或离散参数点兜底。
- 当前扫描区位是规划阶段的入口硬约束。跨区单元可以整体经过其他平面或圆角，但完成后不会提前改变扫描阶段；轨迹层只执行规划后的连续路径。
- 16 区位画像使用理想壳层投影记录跨面、跨圆角、分界母线接触和 X 范围。它属于规划派生数据，不替换 `RotaryKinematics` 对单条路径的生产表面分类。
- 规划阶段使用 Break 最终计划片段解析得到的强区位出口初始化下一分区，并按当前配置的顺时针或逆时针方向遍历全部 16 区位。区位扫描状态不进入 `DocumentProcessState`、轨迹或 NC。
- 单图元闭合圆和完整椭圆的自动入口使用候选相关的动态转移预览。`SameZoneSurfaceTransfer` 的最终接近起点是前序切削终点；`SameZoneClearanceTransfer` 使用 75% 联动位置；`CrossZoneRotaryTransfer` 和首次接近使用已达到目标 A 角及旋转安全高度的安全旋转终点。
- `RotaryTransferPlanner` 是规划与正式轨迹共用的唯一转移计算入口，保留既有 25%/75% 联动结构。规划器逐候选保存转移类型、最终接近起点、切削入口、目标姿态和阶段序列；正式轨迹重算后逐项校验，不一致时返回 `MachineTrajectoryTransferPreviewMismatch`。
- 入口精化与正式轨迹共同使用 `ProcessUnitExecutionResolver`。转移预览中的前序位置包含计划片段、连续 A 轴对齐、闭环补齐和过切；预览签名同时保存前序机床终点和源空间终点。
- 正式轨迹仍负责生成 `MachineMove`，但每个加工单元完成后的 `previousPose` 和 `previousSourcePose` 直接取共享执行结果。转移预览一致性检查保持硬失败，并报告规划值、实际值和对应差值。
- 轨迹服务使用 Assignment 中的最终起点参数和方向重新编译同一源几何。圆和完整椭圆的首段切线来自精确源几何参数，自动起点不吸附到采样顶点。
- 多图元闭合加工单元由规划阶段提供经过简单环验证的成员顺序和逐成员方向；轨迹层按该顺序连续编译，不重新执行最近端点选择。
- 多图元闭环的计划结束位置是经过物理首尾连接验证的真实入口，异常闭环在计划发布前被拒绝，不会把错误结束位置传入后续轨迹。
- `centerY`、`centerZ` 始终作为旋转轴中心参与坐标变换。
- 方管截面中心与旋转轴中心保持独立；用户中心不会被后续自动识别静默覆盖。
- 无截面时普通四轴继续运行；非固定平面路径按旋转轴进行径向展开。
- 有效截面存在时，固定表面分类以 `TubeSectionModel` 的真实 YZ 边界为权威；整条路径全部点必须共同满足同一平面候选，并按最大距离、平均距离消除多候选歧义。
- 固定平面不再依赖 Y/Z 跨度判断的先后顺序。顶、右、底、左面分别使用 0、90、180、-90 度原始 A 角，整条路径在应用反向、偏移和连续展开前保持同一原始角度。
- 圆角识别继续使用截面圆角中心，但仅在整条路径属于同一圆角时生效；平面路径端点落在圆角交界容差内不会改变整段平面分类。
- 无截面时继续使用跨度兼容规则；Y/Z 同时恒定的 X 向路径改为按路径平均位置相对管中心的径向主方向分类，径向分量接近时返回未知并停止错误轨迹输出。
- 表面分类容差独立于连接容差，取数值误差、`1e-6 mm` 和截面最大尺寸 `1e-8` 倍中的最大值。
- A 轴偏移、反向和连续角度展开在运动学转换中统一应用。
- 加工面 Z 修正加入变换后的机床 Z 坐标。
- 首刀从配置的初始机床位置按 `InitialApproach` 进入旋转安全包络，再完成 A 轴对位和联动下降。
- 四轴旋转安全高度为 `tubeCenterZ + maximumCollisionRadius + rotationSafetyClearance`。有效截面存在时碰撞半径来自真实截面边界；无截面时来自当前轨迹全部路径点。
- 相邻加工单元的 `ownerZone` 相同、A 轴无需变化且同区离面距离为零时，只生成一个保持 A 轴不变的表面 Rapid。
- 同区离面距离大于零时，转移按联动离开、局部离面空移和联动接近组织，A 轴保持不变；短距离产生的重复目标不写入轨迹。
- `ownerZone` 改变、元数据缺失或实际 A 轴变化超过容差时，转移升级为跨区旋转转移。联动离开阶段升至全局安全高度，安全旋转阶段保持该高度并完成 A 轴对位，联动接近阶段仅在 A 轴到位后下降。
- `coordinatedTransferEnabled` 只控制平移与升降是否按 25%/75% 位置重叠；关闭时仍执行同一安全包络和转移分类。
- 所有 A 轴变化只能出现在 `SafeRotaryTransfer`，且该段起止 Z 均不得低于旋转安全高度。轨迹发布前逐转移校验，失败返回 `MachineTrajectoryTransferSafetyViolation`，不回退为直接 Rapid。
- 同一 `processUnitIndex` 内使用切削连接，不生成抬刀、空移或重新接近；加工单元边界不再通过 `processGroupId` 猜测。相邻源路径满足连接容差后，还要校验变换后的机床 XYZ 端点距离和展开后的 A 轴连续性。
- 每个联动目标仍是一个包含完整 XYZ/A 的 `MachineMoveKind::Rapid`。NC 层保持单块多轴 Rapid，不把联动目标拆成串行单轴指令。
- 闭合组先精确回到组起点，再按原方向执行可选过切；过切不超过一圈总长。
- 圆和完整椭圆采用新入口后，闭合回起点和过切继续沿重新编译后的同一方向执行。

## 相关源码

- `src/application/machine/MachineTrajectoryService.cpp`：校验计划并把文档几何和配置组装为轨迹输入。
- `src/core/machine/RotaryKinematics.cpp`：将 `Path3D` 转换为连续 XYZ/A 姿态。
- `src/core/machine/ProcessUnitExecutionResolver.cpp`：统一展开计划片段、对齐 A 轴并计算闭环和过切后的加工单元最终状态。
- `src/core/machine/RotaryTrajectoryBuilder.cpp`：使用共享执行结果组织安全移动和连续切削。
- `src/core/machining/TubeSectionProjector.cpp`：提供规划生产排序使用的静态 16 区位画像，不生成机床姿态。
- `include/core/machine/MachineTrajectory.h`：定义三轴安全距离、四轴转移策略、转移阶段、轨迹和旋转上下文。
- `src/core/machine/PlanarTrajectoryBuilder.cpp`：按三轴 `+Z` 语义和加工单元边界生成平面安全移动。
- `include/infrastructure/config/GProfile.h`：定义当前四轴运动配置来源。
- `src/ui/dialogs/GProfileDialog.cpp`：在“运动安全”页签编辑旋转安全抬刀距离、同区空移离面距离和联动开关。
- `src/application/nc/NcProgramService.cpp`：在四轴 NC 生产链中调用轨迹服务。

## 当前实现差异

| 事项 | 当前生产实现 | 需求或概要设计要求 | 影响 |
|---|---|---|---|
| 无截面默认中心 | 状态模型解析为 `(0,0)`；轨迹服务无截面和显式中心时仍使用路径 YZ 包围盒中心 | 截面中心缺失时默认 `(0,0)` | 本阶段保留的轨迹回退仍可能改变表面方向和安全位置 |
| 手动中心输入 | 业务状态和接口已支持用户中心，UI 未提供独立中心输入 | 用户可单独设置 `(Ycenter, Zcenter)` | 当前用户仍无法通过界面直接校正中心 |
| 碰撞半径 | 有效截面使用真实截面边界；无截面时使用当前轨迹整体最大旋转半径 | 截面增强安全计算依赖真实工件外形 | 已按当前可用几何建立旋转安全包络 |
| 配置版本 | 轨迹保存文档和加工状态 revision，没有独立配置 revision | 运动配置变化应使轨迹及下游失效 | 当前依靠同步导出和上层清计划保证，不具备独立版本证明 |
| 轨迹持久状态 | 轨迹仅在导出调用内临时生成 | 概要设计允许轨迹作为派生结果管理 | 当前没有可供 UI 单独检查或复用的轨迹生命周期 |

## 对应需求与概要设计

需求：[MACH-031～MACH-040](../requirements/machining.md)、[PROCESS-008～PROCESS-010、PROCESS-013～PROCESS-017](../requirements/machining-process.md)、[GCODE-015～GCODE-020](../requirements/gcode-and-export.md)。

概要设计：[数据流](../architecture/data-flow.md)、[领域模型](../architecture/domain-model.md)、[状态与生命周期](../architecture/state-and-lifecycle.md)。
