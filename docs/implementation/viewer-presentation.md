# Viewer 展示与加工单元交互详细设计

## 职责

Viewer 展示当前加工顺序、方向、排除状态和选择反馈，并把用户对加工编号的交互发送给 Application。
目标设计以加工单元为展示和顺序交互粒度，一个加工单元只显示一个编号。
Viewer 不拥有加工单元身份、加工单元序列或计划事实。

## 生产入口

- 当前计划成功后，Application 由 `ProcessPlan` 构建 `ProcessPresentationSnapshot` 并交给 Viewer。
- 普通排序和智能排序已经通过显式 `ProcessSortIntent` 产生不同单元序列，Viewer 仅接收最终计划展示结果。
- `CadViewer::renderProcessOrderLabels()` 构建并绘制当前加工顺序标签。
- 标签命中和点击由 `CadViewer_ProcessOrderLabels.cpp` 处理。
- 目标块内提首由 Viewer 上报已选加工单元范围和 Ctrl 单击目标，Application 校验并修改序列。

## 输入与输出

当前输入包括 CAD 场景、逐图元 `ProcessPresentationSnapshot`、加工状态和 Viewer 选择状态。
当前输出是逐图元顺序标签、方向箭头、排除样式和点击反馈。

三轴连续链、严格闭环以及四轴加工组已写入 `ProcessUnitKey`、当前 `ProcessUnitSequence` 和单元成员；Viewer 当前尚未读取这些字段。
目标输出是一单元一编号的展示、单元选择反馈及块内提首命令，不直接修改 `ProcessPlan`。

## 主要数据类型

- `ProcessPresentationSnapshot`：当前按稳定 `EntityId` 保存逐图元顺序、方向、起点、连续组和排除结果。
- `ProcessOrderLabelOverlay`：当前单个标签的屏幕位置、文字、图元和命中区域。
- `RenderEntityKey`：Viewer 对象生命周期内使用的渲染和选择键，不是加工身份。
- `ProcessUnitKey`：当前核心加工单元身份，由成员稳定 `EntityId` 的排序规范集合形成。
- `ProcessUnitSequence`：Application 当前持有的唯一单元序列，序列位置产生 `1..N` 编号。
- `ProcessUnitPresentation`：目标展示投影，包含单元身份、编号、成员身份、锚点和选择状态；具体类型尚未实现。

## 当前生产数据流

```text
ProcessPlan.assignments
→ ProcessPresentationSnapshot::build()
→ 每个 EntityId 一个展示 entry
→ CadViewer 遍历场景图元
→ 每个有效图元生成一个顺序标签
```

当前标签文字由逐图元 `processOrder + 1` 生成。标签锚点从单个图元的加工可视信息和屏幕投影得到。

目标数据流：

```text
ProcessUnitSequence
+ 单元成员和计划展示结果
→ Application 构建单元级展示投影
→ Viewer 每个 ProcessUnitKey 生成一个标签
→ 图元选择映射为加工单元选择
→ Ctrl + 单击命令返回 Application
→ 序列更新并立即刷新单元编号
```

## 状态所有权

Application 的 `DocumentProcessState` 拥有当前 `ProcessUnitSequence` 及其 revision。
Core 负责依据统一加工路径和连续关系形成 `ProcessUnit` 集合。
Viewer 仅保存当前屏幕命中区域、临时悬停和选择反馈。

目标加工单元编号锚点由 Application 的展示投影按整个单元几何确定，并与 `ProcessUnitKey` 一一对应。Viewer 只将该锚点投影到屏幕，不从某个成员图元推断单元身份。

## 失效条件

- 文档几何或连续关系变化后，旧单元身份映射、单元序列、计划和展示需要重新解析。
- 加工单元序列移动或被智能排序替换后，加工状态 revision 推进，旧计划和计划展示失效。
- Application 应立即发布新的单元序列编号展示，不构造假的 `ProcessPlan`。
- 计划失效期间不显示基于旧计划的方向箭头、轨迹或计划排除结果。
- Viewer 缩放、旋转、主题和显示开关只改变展示，不改变加工单元序列。

## 关键业务规则

- 一个加工单元只显示一个加工顺序编号。
- 点击任一成员图元时，Application 通过稳定 `EntityId` 将其映射到所属 `ProcessUnitKey`。
- 人工块内编排只接受编号连续的一段加工单元，不接受多个不连续区间。
- Ctrl + 单击目标必须位于当前连续选择范围内。
- 有效操作将目标单元移动到范围首位，其他单元保持相对顺序。
- 点击范围首项不改变顺序，也不推进 revision。
- 范围不连续或目标在范围外时，仅给出非阻塞提示，不改变序列。
- 操作成功后立即按序列位置生成连续 `1..N` 编号并刷新显示。
- 选择和命中可以使用 Viewer 临时键，写入加工序列的身份必须使用稳定 `ProcessUnitKey`。

示例：

```text
选中 1,2,3，Ctrl + 单击 2
→ 2,1,3

选中 4,5,6，Ctrl + 单击 6
→ 1,2,3,6,4,5,7...
```

## 相关源码

- `src/cad/view/CadViewer_ProcessOrderLabels.cpp`：当前逐图元顺序标签的构建、绘制、命中和点击处理。
- `src/application/process/ProcessPresentationSnapshot.cpp`：当前把逐图元计划 assignment 转换为展示 entry。
- `include/application/process/ProcessPresentationSnapshot.h`：定义当前逐图元展示快照。
- `include/core/planning/ProcessPlan.h`：定义当前 `ProcessGroup`、逐图元 assignment 和连续组编号。
- `src/desktop/Gcode_postprocessing_system_SortActions.cpp`：保存当前计划并向 Viewer 发布展示结果。

## 当前实现差异

| 事项 | 当前生产实现 | 目标设计 | 影响 |
|---|---|---|---|
| 标签粒度 | 三轴和四轴计划已形成加工单元，但每个有效图元仍按 assignment 生成标签 | 每个加工单元只生成一个标签 | 连续链和闭环当前仍可能显示多个编号 |
| 标签锚点 | 从单个图元的可视信息得到 | 从整个加工单元的单元级展示投影得到 | 当前没有稳定单元锚点 |
| 选择粒度 | Viewer 选择单个 `CadItem` | 成员图元选择映射为加工单元选择 | 当前不能校验连续单元范围 |
| 顺序状态 | Application 已持有唯一 `ProcessUnitSequence`；展示仍读取逐图元执行顺序 | Viewer 读取单元序列位置 | 单元状态已落地，展示接入尚未实现 |
| 排序意图 | 普通排序保留匹配单元相对顺序，智能排序重建序列；Viewer 只显示最终 assignment | Viewer 不参与排序决策 | 已分离，Viewer 行为未修改 |
| 点击交互 | 标签点击和双击围绕单图元处理 | Ctrl + 单击执行连续块内提首 | 目标交互尚未实现 |
| 失效刷新 | 展示随 `ProcessPlan` 创建或清除 | 序列编号可即时刷新，计划派生展示保持失效 | 当前缺少独立单元序列展示 |

## 对应需求与概要设计

需求：[MACH-041～MACH-043](../requirements/machining.md)、[PROCESS-018～PROCESS-023](../requirements/machining-process.md)、[EDIT-029～EDIT-033](../requirements/editing-and-interaction.md)。

概要设计：[领域模型](../architecture/domain-model.md)、[数据流](../architecture/data-flow.md)、[模块边界](../architecture/module-boundaries.md)、[状态与生命周期](../architecture/state-and-lifecycle.md)。
