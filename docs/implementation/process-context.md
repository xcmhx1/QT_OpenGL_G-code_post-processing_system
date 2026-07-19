# 加工上下文详细设计

## 职责

加工上下文汇集当前文档上的用户加工输入和自动分析状态，供加工规划读取。
派生的加工计划与展示快照单独保存，不属于加工上下文本身。
本文件描述当前生产代码的实际存储边界，不假定概要概念已有一一对应的数据结构。

## 生产入口

- 图元加工设置由桌面命令和 `CadEditer` 写入当前 `DocumentProcessState`。
- 方管截面由手动参数设置或截面识别命令写入主窗口持有的 `RotaryTubeSectionModel`。
- 内部线自动识别写入分析状态，手动指定写入用户覆盖，两者都由 `DocumentProcessState` 管理。
- 文件导入、删除、复制、Undo/Redo 和几何编辑通过文档及编辑命令维护相关状态。
- 排序入口通过 `ProcessPlanningService` 捕获当前文档和加工状态，形成规划输入。

## 输入与输出

输入包括当前 CAD 文档、稳定图元身份、用户加工设置、自动内部线结果和可选方管截面。
输出是可供三轴或四轴规划读取的当前有效加工上下文。
加工上下文变化后，已有 `ProcessPlan` 和 `ProcessPresentationSnapshot` 会失效。
加工上下文不会直接生成机床轨迹或 G-code。

## 主要数据类型

- `DocumentProcessState`：按稳定 `EntityId` 保存逐图元加工状态，并维护独立 revision。
- `ProcessOverride`：当前按图元保存加工启用、方向、起点、手动内部线三态、`manualProcessOrder`、加工断面角色和断面组编号。
- `ProcessAnalysisState`：保存自动识别得到的内部线排除结果。
- `EntityProcessState`：组合用户覆盖与自动分析状态，并解析有效内部线结果。
- `RotaryTubeSectionModel`：主窗口持有的方管截面业务状态，包含核心截面、尺寸、圆角、自动中心和用户中心。
- `ProcessPlan`：规划派生结果，记录文档 revision 和加工状态 revision。
- `ProcessPresentationSnapshot`：由有效计划生成的 Viewer 展示数据。
- `ProcessUnit`：目标设计中的不可拆分加工单元，由一个连续加工组或一个独立图元构成，当前尚无对应生产类型。
- `ProcessUnitKey`：目标设计中由单元全部稳定 `EntityId` 排序形成的规范集合，当前尚未实现。
- `ProcessUnitSequence`：目标设计中由 Application 持有的唯一加工单元序列，当前尚未实现。

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
→ PlanningInput
→ ProcessPlanningService
```

## 状态所有权

主窗口拥有当前 `CadDocument`、`DocumentProcessState`、方管截面、当前计划和展示快照。
`DocumentProcessState` 以 `CadItem::m_entityId` 为键，不使用 Viewer 的对象地址键。
当前生产代码将加工启用、方向、起点、`manualProcessOrder`、Break/Waste 角色和内部线状态按图元身份保存。
方管截面没有存入 `DocumentProcessState`，由主窗口中的独立模型保存。
当前计划和展示快照属于派生结果，可随时根据上下文重建。

目标设计由 Application 保存和编辑唯一 `ProcessUnitSequence`。顺序属于加工单元序列，不属于成员图元；单元身份只使用稳定 `EntityId` 形成的 `ProcessUnitKey`。

## 失效条件

- 文档导入会清空编辑历史和加工状态、重置方管截面并清除计划与展示。
- 新建、删除、复制、几何编辑、图层或颜色修改以及 Undo/Redo 会推进文档内容 revision。
- `DocumentProcessState` 的实际值变化会推进加工状态 revision；批量修改只推进一次。
- 当前设置、清除单个或批量清除 `manualProcessOrder` 均按上述 revision 规则处理。
- 目标加工单元序列发生插入、删除或移动时推进一次加工状态 revision，并立即生成连续的 `1..N` 编号。
- 主窗口监听文档变化；当前计划 revision 与文档或加工状态不一致时立即清除计划和展示。
- 方管截面携带文档内容 revision；四轴规划发现截面过期时拒绝继续使用。
- 用户截面中心变化会同步截面边界，并清除依赖截面的当前计划和展示。
- 选择状态、展示开关和 Viewer 临时状态不推进上述业务 revision。

## 关键业务规则

- 删除命令在删除前保存图元加工状态，Undo 时按原 `EntityId` 恢复。
- 批量删除和恢复通过 `beginBatch()` / `endBatch()` 合并 revision 变化。
- 复制和阵列产生新 `EntityId`，新图元当前使用默认加工状态，不继承源图元状态。
- 因此复制图元不会复制当前逐图元 `manualProcessOrder`，也不会复制自动内部线结果。
- 几何编辑保留原 `EntityId`，对应加工输入继续保留，但派生计划失效。
- Break 与 Waste 共用断面角色字段，通过角色和组编号区分。
- 手动内部线为三态：未指定、强制排除、强制参与加工。
- 有手动设置时使用手动值；没有手动设置时使用自动分析结果。
- 清除手动设置后立即重新使用当前自动结果，重新分析不会覆盖手动设置。
- 当前人工加工顺序以逐图元 `std::optional<int>` 保存，不要求唯一，也不自动重排编号；该字段属于待移除的旧模型。
- 目标顺序由唯一 `ProcessUnitSequence` 表达，不使用稀疏优先级，也不向组内每个成员双写相同编号。
- 截面中心按“用户中心、自动中心、`(0,0)`”解析；自动识别保留已有用户中心。
- 规划捕获完成后再次比较文档与加工状态 revision，避免使用捕获期间已经变化的数据。

## 相关源码

- `include/application/process/DocumentProcessState.h`：定义逐图元加工输入、分析状态和 revision 接口。
- `src/application/process/DocumentProcessState.cpp`：实现状态去默认值、批处理和 revision 推进。
- `include/desktop/Gcode_postprocessing_system.h`：拥有当前文档、截面、加工状态、计划和展示快照。
- `src/desktop/Gcode_postprocessing_system_FileActions.cpp`：处理导入时的状态清理和自动后处理。
- `src/desktop/Gcode_postprocessing_system_SortActions.cpp`：写入加工状态并使派生计划失效。
- `src/cad/editing/CadEditer_CommandActions.cpp`：删除和 Undo 时保存、移除及恢复加工状态。
- `src/application/planning/DocumentProcessPlanningAdapter.cpp`：将文档与加工状态捕获为核心规划输入。

## 当前实现差异

| 事项 | 当前生产实现 | 需求或概要设计要求 | 影响 |
|---|---|---|---|
| 加工顺序状态 | `manualProcessOrder` 按图元保存，可重复且不自动重编号 | Application 保存唯一 `ProcessUnitSequence`，顺序位置形成连续 `1..N` | 单元序列尚未实现，旧字段待移除 |
| 加工单元身份 | 当前计划使用 `ProcessGroup` 和逐图元 assignment，没有稳定 `ProcessUnitKey` | 单元全部稳定 `EntityId` 的排序规范集合构成身份 | 连续关系变化后尚不能按单元身份重新解析序列 |
| 截面中心输入 | 状态模型支持用户中心、清除用户中心和默认 `(0,0)` | 用户可单独设置 `(Ycenter, Zcenter)` | 当前 UI 尚未提供中心输入控件 |
| 手动截面优先级 | 用户中心可覆盖自动中心；手动尺寸模型仍由主窗口整体持有 | 手动截面参数应优先于自动识别结果 | 尺寸和圆角的自动覆盖保护仍沿用现有流程 |
| 导入后内部线 | 自动后处理在无有效截面时跳过整个内部线操作 | 拓扑内部线阶段不依赖截面 | 手动执行可做拓扑识别，导入自动流程当前不能 |

## 对应需求与概要设计

需求：[MACH-041～MACH-043](../requirements/machining.md)、[PROCESS-018～PROCESS-023](../requirements/machining-process.md)。

概要设计：[领域模型](../architecture/domain-model.md)、[状态与生命周期](../architecture/state-and-lifecycle.md)、[数据流](../architecture/data-flow.md)。
