# 加工上下文详细设计

## 职责

加工上下文汇集当前文档上的用户加工输入和自动分析状态，供加工规划读取。
派生的加工计划与展示快照单独保存，不属于加工上下文本身。
本文件描述当前生产代码的实际存储边界，不假定概要概念已有一一对应的数据结构。

## 生产入口

- 图元加工设置由桌面命令和 `CadEditer` 写入当前 `DocumentProcessState`。
- 方管截面由手动参数设置或截面识别命令写入主窗口持有的 `RotaryTubeSectionModel`。
- 手动内部线标记写入用户覆盖，由 `DocumentProcessState` 管理；内部线自动清理通过内缩窗口选择并物理删除图元，删除进入统一撤销栈。
- 文件导入、删除、复制、Undo/Redo 和几何编辑通过文档及编辑命令维护相关状态。
- 排序入口通过 `ProcessPlanningService` 捕获当前文档和加工状态，形成规划输入。

## 输入与输出

输入包括当前 CAD 文档、稳定图元身份、用户加工设置、手动内部线标记和可选方管截面。
输出是可供三轴或四轴规划读取的当前有效加工上下文。
加工上下文变化后，已有 `ProcessPlan` 和 `ProcessPresentationSnapshot` 会失效。
加工上下文不会直接生成机床轨迹或 G-code。

## 主要数据类型

- `DocumentProcessState`：按稳定 `EntityId` 保存逐图元加工状态，同时保存唯一加工单元序列并维护独立 revision。
- `ProcessOverride`：按图元保存加工启用、方向、起点、手动内部线三态、加工断面角色和断面组编号。
- `ProcessAnalysisState`：保存自动分析状态；当前内部线自动清理直接删除图元，不再写入自动排除结果。
- `EntityProcessState`：组合用户覆盖与自动分析状态，并解析有效内部线结果。
- `RotaryTubeSectionModel`：主窗口持有的方管截面业务状态，包含核心截面、尺寸、圆角、自动中心和用户中心。
- `ProcessPlan`：规划派生结果，记录文档 revision 和加工状态 revision。
- `ProcessPresentationSnapshot`：由有效计划生成的 Viewer 展示数据。
- `ProcessUnitPresentation`：由有效计划生成的单元展示项，保存单元身份、连续编号位置、成员执行顺序和锚点成员。
- `ProcessUnit`：不可拆分加工单元，保存规范身份、成员实际执行顺序和闭合状态。
- `ProcessUnitKey`：由单元全部稳定 `EntityId` 升序、去重后形成的非空规范身份。
- `ProcessUnitSequence`：Application 持有的唯一加工单元序列，序列位置形成连续加工编号并带独立 revision。

## 当前生产数据流

```text
用户图元操作
→ DocumentProcessState
→ revision 变化
→ 当前计划和展示失效
→ 下次排序重新捕获
```

```text
截面识别、手动设置或中心修改
→ RotaryTubeSectionModel
→ 当前计划和展示失效
→ 四轴规划按需读取核心截面
```

```text
CadDocument + DocumentProcessState + 可选方管截面
→ DocumentProcessPlanningAdapter
→ 按 ProcessSortIntent 解析本次方向和起点输入
→ PlanningInput
→ ProcessPlanningService
```

```text
ProcessPlan 中的加工单元顺序
→ DocumentProcessState::setProcessUnitSequence()
→ 加工状态 revision 变化
→ 新计划绑定更新后的 revision
```

## 状态所有权

主窗口拥有当前 `CadDocument`、`DocumentProcessState`、方管截面、当前计划和展示快照。
`DocumentProcessState` 以 `CadItem::m_entityId` 为键，不使用 Viewer 的对象地址键。
当前生产代码将加工启用、方向、起点、Break/Waste 角色和手动内部线状态按图元身份保存。
方管截面没有存入 `DocumentProcessState`，由主窗口中的独立模型保存。
当前计划和展示快照属于派生结果，可随时根据上下文重建。

`DocumentProcessState` 保存唯一 `ProcessUnitSequence`。顺序属于加工单元序列，不属于成员图元；单元身份只使用稳定 `EntityId` 形成的 `ProcessUnitKey`。

## 失效条件

- 文档导入会清空编辑历史和加工状态、重置方管截面并清除计划与展示。
- 新建、删除、复制、几何编辑、图层或颜色修改以及 Undo/Redo 会推进文档内容 revision。
- `DocumentProcessState` 的实际值变化会推进加工状态 revision；批量修改只推进一次。
- 加工单元序列实际变化时推进加工状态 revision；重复写入相同序列不推进 revision。
- 块内移尾把修改前和修改后的完整单元序列作为一个编辑命令；Undo/Redo 每次实际应用最多推进一次加工状态 revision。
- 目标已在选中范围末位、选中范围不连续或目标不在范围内时不修改状态，也不进入撤销栈。
- 单元遍历覆盖按稳定 `ProcessUnitKey` 保存成员执行顺序、每成员方向和起点参数；不向成员写入重复的组方向状态。
- 删除图元时移除包含该图元的旧单元身份；清空文档时清空加工单元序列。
- 删除、批量删除和替换命令保存操作前、操作后的完整单元序列；Undo 恢复操作前序列，Redo 恢复操作后序列。
- 上述命令将逐图元状态和单元序列恢复放在同一个批处理中，一次命令最多推进一次加工状态 revision。
- 主窗口监听文档变化；当前计划 revision 与文档或加工状态不一致时立即清除计划和展示。
- 方管截面一经识别或手动设置即持续有效，文档几何变化不会使截面失效；排序和轨迹继续使用当前截面参数，导出解算实际失败时再报错提示用户检查或重新设置截面。
- 用户截面中心变化会同步截面边界，并清除依赖截面的当前计划和展示。
- 选择状态、展示开关和 Viewer 临时状态不推进上述业务 revision。

## 关键业务规则

- 删除命令在删除前保存图元加工状态和加工单元序列，Undo 时按原 `EntityId` 和原序列恢复。
- 替换命令保存源图元状态及操作前后序列；新替换图元不自动加入旧序列，Redo 使用已保存的操作后序列。
- 批量删除、替换和恢复通过 `beginBatch()` / `endBatch()` 合并 revision 变化。
- 复制和阵列产生新 `EntityId`，新图元当前使用默认加工状态，不继承源图元状态。
- 复制图元不会复制手动内部线标记，新图元在后续排序中形成新的加工单元身份。
- 几何编辑保留原 `EntityId`，对应加工输入继续保留，但派生计划失效。
- Break 与 Waste 共用断面角色字段，通过角色和组编号区分。
- 手动内部线为三态：未指定、强制排除、强制参与加工。
- 有手动设置时使用手动值；清除手动设置后按默认参与加工。
- 内部线自动清理直接删除内缩窗口内的图元，不写入内部线状态；删除成功后把截面模型修订推进到新文档版本，撤销恢复图元后需重新识别截面。
- 加工顺序由唯一 `ProcessUnitSequence` 表达，不使用稀疏优先级，也不向组内每个成员双写相同编号。
- 旧逐图元人工顺序字段及其设置、查询和规划输入已经移除。
- 普通排序读取用户保存的方向、起点和当前加工单元序列。
- 智能排序只在本次规划输入中忽略人工方向和起点，并忽略当前加工单元序列；`DocumentProcessState` 中的用户设置保持不变。
- 规划成功后才用结果更新权威加工单元序列，并把计划重新绑定到更新后的加工状态 revision。
- 展示快照按权威序列位置生成单元编号；Viewer 标签选择只改变当前 CAD 选择，不改变加工状态 revision。
- Ctrl 单击单元标签时，Viewer 仅上报由稳定 `EntityId` 组成的选中单元键和目标单元键；Application 使用权威序列完成校验和修改。
- 块内移尾先重建并校验候选计划和展示快照，成功后才写入权威序列并发布新快照，失败时不留下部分更新状态。
- 整组反向保存反向前后的实际单元遍历和覆盖状态，Undo/Redo 复用同一计划重建与展示发布路径。
- 截面中心按“用户中心、自动中心、`(0,0)`”解析；自动识别保留已有用户中心。
- 规划捕获完成后再次比较文档与加工状态 revision，避免使用捕获期间已经变化的数据。

## 相关源码

- `include/application/process/DocumentProcessState.h`：定义逐图元加工输入、分析状态和 revision 接口。
- `src/application/process/DocumentProcessState.cpp`：实现状态去默认值、批处理和 revision 推进。
- `include/desktop/Gcode_postprocessing_system.h`：拥有当前文档、截面、加工状态、计划和展示快照。
- `src/desktop/Gcode_postprocessing_system_FileActions.cpp`：处理导入时的状态清理和自动后处理。
- `src/desktop/Gcode_postprocessing_system_SortActions.cpp`：写入加工状态并使派生计划失效。
- `src/cad/editing/CadEditer_CommandActions.cpp`：块内移尾和整组反向时保存操作前后状态，并通过 Undo/Redo 恢复。
- `src/application/planning/DocumentProcessPlanningAdapter.cpp`：将文档与加工状态捕获为核心规划输入。

## 当前实现差异

| 事项 | 当前生产实现 | 需求或概要设计要求 | 影响 |
|---|---|---|---|
| 单元序列人工编辑 | Ctrl 单击可将连续选中块内的目标单元移到块尾，并支持 Undo/Redo | 用户可编辑当前加工单元序列 | 块内移尾已实现；其他人工序列编辑尚未实现 |
| 单元遍历覆盖 | 双击单元编号或箭头可整组反向，普通排序应用覆盖，智能排序忽略但不删除 | 单元方向作为稳定业务输入 | 已实现 |
| 单元顺序展示 | `ProcessPresentationSnapshot` 提供单元级标签和起点箭头数据 | 一个加工单元只显示一个编号和一个箭头 | 已实现 |
| 连续关系变化 | 删除或替换时保存并切换命令前后序列；普通排序保留可匹配单元并在末尾追加新单元 | 连续关系变化后重新解析单元序列 | 已实现第一版稳定保序，不进行最优插入 |
| 截面中心输入 | 状态模型支持用户中心、清除用户中心和默认 `(0,0)` | 用户可单独设置 `(Ycenter, Zcenter)` | 当前 UI 尚未提供中心输入控件 |
| 手动截面优先级 | 用户中心可覆盖自动中心；手动尺寸模型仍由主窗口整体持有 | 手动截面参数应优先于自动识别结果 | 尺寸和圆角的自动覆盖保护仍沿用现有流程 |

## 对应需求与概要设计

需求：[MACH-041～MACH-043](../requirements/machining.md)、[PROCESS-018～PROCESS-023](../requirements/machining-process.md)。

概要设计：[领域模型](../architecture/domain-model.md)、[状态与生命周期](../architecture/state-and-lifecycle.md)、[数据流](../architecture/data-flow.md)。
