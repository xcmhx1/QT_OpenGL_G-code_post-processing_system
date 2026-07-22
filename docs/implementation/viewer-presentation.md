# Viewer 展示与加工单元交互详细设计

## 职责

Viewer 展示当前加工单元顺序、逐图元方向、排除状态和选择反馈。
加工顺序以加工单元为展示粒度，一个加工单元只显示一个编号。
Viewer 不拥有加工单元身份、加工单元序列或计划事实。

## 生产入口

- 当前计划成功后，Application 由 `ProcessPlan` 构建 `ProcessPresentationSnapshot` 并交给 Viewer。
- 普通排序和智能排序已经通过显式 `ProcessSortIntent` 产生不同单元序列，Viewer 仅接收最终计划展示结果。
- `CadViewer::renderProcessOrderLabels()` 构建并绘制当前加工顺序标签。
- 标签命中和点击由 `CadViewer_ProcessOrderLabels.cpp` 处理。
- Ctrl 单击标签时，Viewer 向 Application 上报当前选中单元键和目标单元键；Viewer 本身不修改加工单元序列。

## 输入与输出

当前输入包括 CAD 场景、同时含单元级和逐图元数据的 `ProcessPresentationSnapshot`、加工状态和 Viewer 选择状态。
当前输出是单元顺序标签、单元起点方向箭头、排除样式、整组选择和稳定单元键交互请求。

三轴连续链、严格闭环以及四轴加工组通过 `ProcessUnitPresentation` 进入 Viewer。普通单击只改变 Viewer 选择；Ctrl 单击仅上报稳定单元键，由 Application 校验并更新序列。

## 主要数据类型

- `ProcessPresentationSnapshot`：同时保存单元级展示项和逐图元方向、起点、连续组及排除结果。
- `ProcessUnitPresentation`：保存单元键、零基单元序号、实际成员执行顺序和锚点成员身份。
- `ProcessOrderLabelOverlay`：保存标签对应的单元键、单元序号、悬停和选择状态、屏幕位置及命中区域。
- `RenderEntityKey`：Viewer 对象生命周期内使用的渲染和选择键，不是加工身份。
- `ProcessUnitKey`：当前核心加工单元身份，由成员稳定 `EntityId` 的排序规范集合形成。
- `ProcessUnitSequence`：Application 当前持有的唯一单元序列，序列位置产生 `1..N` 编号。

## 当前生产数据流

```text
ProcessPlan.processUnits
+ ProcessUnitSequence
→ ProcessPresentationSnapshot::build()
→ 校验单元键、成员集合、执行顺序和计划序列
→ 每个 ProcessUnit 生成一个 ProcessUnitPresentation
→ CadViewer 以 anchorEntityId 查找首个实际执行成员
→ 使用该成员实际加工起点生成一个顺序标签
```

标签文字由 `unitOrder + 1` 生成，不读取逐图元 `processOrder`。逐图元展示条目继续为方向箭头、加工起点和排除状态提供数据。

## 状态所有权

Application 的 `DocumentProcessState` 拥有当前 `ProcessUnitSequence` 及其 revision。
Core 负责依据统一加工路径和连续关系形成 `ProcessUnit` 集合。
Viewer 仅保存当前屏幕命中区域、临时悬停和选择反馈。

当前单元编号锚点由展示快照指定为实际执行顺序中的首个成员。Viewer 只用该成员计算实际加工起点附近的屏幕位置，标签业务身份始终是完整 `ProcessUnitKey`。

Ctrl 单击时，Viewer 将当前选中图元的稳定 `EntityId` 映射到展示快照中的单元键；任一成员被选中即视为整个加工单元被选中。

## 失效条件

- 文档几何或连续关系变化后，旧单元身份映射、单元序列、计划和展示需要重新解析。
- 加工单元序列移动或被智能排序替换后，加工状态 revision 推进，旧计划和计划展示失效。
- 有效计划更新后，Application 重新发布包含单元数据的展示快照，不构造假的 `ProcessPlan`。
- 计划失效期间不显示基于旧计划的方向箭头、轨迹或计划排除结果。
- Viewer 缩放、旋转、主题和显示开关只改变展示，不改变加工单元序列。

## 关键业务规则

- 一个加工单元只显示一个加工顺序编号。
- 一个加工单元只显示一个起点方向箭头；箭头使用首个实际执行成员的 `reverse` 和 `startParameter` 计算。
- 单图元圆和完整椭圆的 Auto 入口由四轴规划器写入最终计划；展示快照直接转发该参数和方向，Viewer 不独立计算北极点或重新选择方向。
- 单元编号由 `ProcessUnitPresentation::unitOrder + 1` 产生。
- 任一成员已被选中时，该单元标签使用选中样式；标签悬停也以完整单元为粒度。
- 点击单元标签时，Viewer 清除原选择，并按稳定 `EntityId` 选择该单元全部成员。
- 标签单击不修改加工单元序列，也不推进加工状态 revision。
- Ctrl 单击不清除当前选择，只上报去重后的选中 `ProcessUnitKey` 集合和目标键。
- 标签双击不再修改单个成员方向，仅给出非阻塞提示。
- 人工块内编排只接受编号连续的一段加工单元，不接受多个不连续区间。
- Ctrl + 单击目标必须位于当前连续选择范围内。
- 有效操作将目标单元移动到范围末位，其他单元保持相对顺序。
- 点击范围末项不改变顺序，也不推进 revision。
- 双击单元编号或单元箭头上报同一 `ProcessUnitKey`，由 Application 倒序成员遍历、取反成员方向并保留闭合起点。
- 范围不连续或目标在范围外时，仅给出非阻塞提示，不改变序列。
- 操作成功后立即按序列位置生成连续 `1..N` 编号并刷新显示。
- 标签命中使用 `ProcessUnitKey`；仅在把稳定成员身份映射为当前场景选择时使用 Viewer 临时键。

示例：

```text
选中 1,2,3，Ctrl + 单击 2
→ 1,3,2

选中 4,5,6，Ctrl + 单击 4
→ 1,2,3,5,6,4,7...
```

## 相关源码

- `src/cad/view/CadViewer_ProcessOrderLabels.cpp`：单元顺序标签的构建、绘制、命中、整组选择和 Ctrl 交互请求处理。
- `src/application/process/ProcessPresentationSnapshot.cpp`：校验计划并生成单元级和逐图元展示数据。
- `include/application/process/ProcessPresentationSnapshot.h`：定义单元级与逐图元展示快照。
- `include/core/planning/ProcessPlan.h`：定义加工单元、单元序列和逐图元 assignment。
- `src/desktop/Gcode_postprocessing_system_SortActions.cpp`：校验连续选中范围，按值保存加工状态前后快照，并构造原子更新回调。
- `src/cad/editing/CadEditer_CommandActions.cpp`：通过通用回调命令把加工交互接入 CAD 共用的 Undo/Redo 栈，不解释加工单元类型。

## 当前实现差异

| 事项 | 当前生产实现 | 目标设计 | 影响 |
|---|---|---|---|
| 标签粒度 | 每个 `ProcessUnitPresentation` 生成一个标签 | 每个加工单元只生成一个标签 | 已实现 |
| 标签锚点 | 取实际执行顺序的首个成员，在其实际加工起点附近显示 | 单元级展示具有确定锚点 | 第一版锚点策略已实现，尚未计算整组几何中心 |
| 选择粒度 | 点击标签按稳定成员身份选中整个加工单元 | 单元标签支持整组选择 | 已实现；直接点击普通图元仍沿用现有图元选择 |
| 顺序状态 | 标签读取 `unitOrder`，不读取逐图元 `processOrder` | Viewer 读取单元序列位置 | 已实现 |
| 排序意图 | 普通排序保留匹配单元相对顺序，智能排序重建序列 | Viewer 不参与排序决策 | 已实现，Viewer 未修改排序 |
| 点击交互 | 单击选择全部成员；Ctrl 单击执行连续块内移尾；双击编号或箭头整组反向 | 单元级顺序与方向交互 | 已实现并支持 Undo/Redo |
| 交互历史 | Desktop/Application 构造块内移尾和整组反向的执行、撤销回调，`CadEditer` 只维护统一历史顺序 | Viewer 不保存业务状态，CAD 层不拥有加工命令类型 | 已迁移到通用历史入口 |
| 方向箭头 | 每个 `ProcessUnitPresentation` 只使用首执行成员生成一个箭头 | 一单元一起点方向箭头 | 已实现 |
| 失效刷新 | 展示随有效 `ProcessPlan` 创建或清除 | 序列编号与有效计划一致 | 当前仍不在计划失效期间单独展示序列 |

## 对应需求与概要设计

需求：[MACH-041～MACH-043](../requirements/machining.md)、[PROCESS-018～PROCESS-023](../requirements/machining-process.md)、[EDIT-029～EDIT-033](../requirements/editing-and-interaction.md)。

概要设计：[领域模型](../architecture/domain-model.md)、[数据流](../architecture/data-flow.md)、[模块边界](../architecture/module-boundaries.md)、[状态与生命周期](../architecture/state-and-lifecycle.md)。
