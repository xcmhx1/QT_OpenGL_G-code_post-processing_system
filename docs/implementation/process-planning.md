# 加工规划详细设计

## 职责

加工规划将当前 CAD 几何和加工上下文转换为确定的 `ProcessPlan`。
当前计划按图元记录顺序、实际方向、实际起点和连续组；目标设计先形成加工单元集合，再以唯一加工单元序列确定顺序。
三轴与四轴在桌面排序入口处分流，并分别进入对应的核心计划构建器。

## 生产入口

- 桌面层“排序”和“智能排序”动作进入 `sortEntitiesByCurrentMode()`。
- 三轴入口最终调用 `ProcessPlanningService::buildPlanarPlan()`。
- 四轴入口最终调用 `ProcessPlanningService::buildRotaryPlan()`。
- 导出前没有当前有效计划时，`prepareDocumentForGCodeExport()` 自动调用对应模式的智能排序入口。
- 计划成功后立即生成 `ProcessPresentationSnapshot` 并交给 Viewer。

## 输入与输出

输入是 `CadDocument`、`DocumentProcessState`、规划策略和四轴可选的 `TubeSectionModel`。
文档适配器输出带稳定 `EntityId` 的精确源几何、统一 `Path3D`、图元属性及加工状态。
四轴输入额外包含统一 `PathTopology`、加工断面角色、可选截面及当前有效截面中心。
三轴和四轴实体输入当前都携带逐图元 `manualProcessOrder`，计划构建器不消费该字段。
输出 `ProcessPlan`，并由其派生 Viewer 使用的展示快照。

## 主要数据类型

- `ProcessPlanningPolicy`：四轴连接容差、起始位置、方向许可、闭环原子组和排序策略。
- `PlanarProcessPlanningPolicy`：三轴起始位置、连接误差和方向处理选项。
- `PlanningEntity`：核心规划使用的图元路径、状态、断面角色和来源属性。
- `manualProcessOrder`：当前从用户加工状态捕获的逐图元可选顺序，属于待移除的旧模型。
- `ProcessGroup`：单图元、连接链、闭环、Break 断面或 Waste 断面组成的加工组。
- `ProcessAssignment`：最终图元顺序、连续组、方向和起点。
- `ProcessExclusion`：内部线、Waste 区间或其他规划排除结果。
- `ProcessPlan`：保存模式、排序策略、revision、分组、约束、分配和排除项。
- `ProcessUnit` / `ProcessUnitKey` / `ProcessUnitSequence`：目标设计的加工单元、稳定单元身份和唯一单元序列，当前生产类型尚未建立。

## 当前生产数据流

普通排序生产链：

```text
“排序”动作
→ sortEntitiesByCurrentMode(false)
→ 三轴或四轴排序包装函数
→ ProcessPlanningService
→ ProcessPlan
→ ProcessPresentationSnapshot
```

智能排序生产链：

```text
“智能排序”动作
→ sortEntitiesByCurrentMode(true)
→ 三轴或四轴智能排序包装函数
→ 与普通排序相同的 ProcessPlanningService
→ ProcessPlan
→ ProcessPresentationSnapshot
```

三轴计划生产链：

```text
CadDocument + DocumentProcessState
→ DocumentProcessPlanningAdapter::capturePlanar
→ GeometryCompiler
→ PlanarProcessPlanBuilder
→ Planar3Axis ProcessPlan
```

四轴计划生产链：

```text
CadDocument + DocumentProcessState + 可选 TubeSectionModel
→ DocumentProcessPlanningAdapter::captureRotary
→ GeometryCompiler + PathTopology
→ ProcessPlanBuilder
→ Rotary4Axis ProcessPlan
```

目标普通排序生产链：

```text
可加工图元
→ 连续关系分析
→ ProcessUnit 集合
+ 当前 ProcessUnitSequence
+ 人工方向和人工起点
→ 尽量保留原相对顺序并补入新增或变化单元
→ 新 ProcessUnitSequence
→ ProcessPlan
```

目标智能排序生产链：

```text
可加工图元
→ 与普通排序相同的连续关系分析
→ ProcessUnit 集合
→ 忽略当前 ProcessUnitSequence
→ 重新计算全部单元顺序
→ 直接替换 ProcessUnitSequence
→ ProcessPlan
```

## 状态所有权

主窗口保存唯一的当前 `ProcessPlan` 和 `ProcessPresentationSnapshot`。
核心规划器只返回值对象，不持有活动文档或 Viewer。
计划通过 `contentRevision` 和 `processStateRevision` 绑定生成时的文档与加工状态。
Viewer 只读取展示快照，不反向修改计划或加工输入。

目标设计由 Application 持有当前 `ProcessUnitSequence`。Core 依据几何和连续关系形成加工单元并计算计划，Viewer 只显示序列和接收单元级交互，不拥有顺序状态。

## 失效条件

- 文档内容 revision 或加工状态 revision 变化会清除当前计划和展示。
- 目标加工单元序列被插入、删除、移动或智能排序替换时推进加工状态 revision，使旧计划和展示失效。
- 连续关系变化后，旧 `ProcessUnitKey` 和序列必须重新解析；未重新解析前不得沿用旧单元顺序。
- 计划模式与当前导出模式不一致时，导出前重新规划。
- 四轴截面 revision 与文档不一致时，规划返回冲突，不使用过期截面。
- 切换或修改当前 G-code 配置时，桌面层当前采用保守策略清除整个计划。
- 懒旋转策略依赖有效方管截面；缺失截面时该计划构建失败。
- 普通最近距离四轴规划可在没有截面的情况下继续。

## 关键业务规则

- 不可见、未启用、有效内部线排除和不支持的图元在规划输入或计划中排除。
- 有效内部线按手动覆盖优先、自动分析兜底解析，规划器只接收解析后的最终布尔值。
- Waste 断面本身及其定义的废弃区间不进入加工分配。
- Break 断面形成独立加工组和前后工艺约束。
- 严格闭环和需要连续加工的连通分量可形成原子连续组，组内不得被其他图元插入。
- 三轴当前使用最近距离计划，方向偏好会约束正向或反向候选。
- 四轴可按配置选择最近距离或懒旋转策略；懒旋转在后续选择中考虑 A 轴旋转代价。
- 四轴第一次组选择仍使用最近距离，再对后续组应用配置的排序策略。
- 当前计划中的每个逐图元 `processOrder` 必须连续且唯一，图元只出现一次。
- 展示快照复制计划中的顺序、方向、起点、连续组和排除原因。
- 目标设计中普通排序、智能排序和人工编排都以 `ProcessUnit` 为最小单位，不拆分连续单元。
- 目标普通排序尽量保留当前单元序列、人工方向和人工起点；新增或变化单元由现有算法补入。
- 目标智能排序忽略当前单元序列并重新计算全部单元顺序，结果直接替换当前序列，不保留隐藏旧顺序。
- 目标序列位置产生连续 `1..N` 编号，插入、删除和移动后立即重编号。

## 相关源码

- `src/desktop/Gcode_postprocessing_system_SortActions.cpp`：提供排序 UI 入口、策略选择和计划保存。
- `src/application/planning/ProcessPlanningService.cpp`：组织文档捕获、核心规划和 revision 复核。
- `src/application/planning/DocumentProcessPlanningAdapter.cpp`：将生产文档和加工状态转换为规划值对象。
- `src/core/planning/PlanarProcessPlanBuilder.cpp`：构建三轴最近距离加工计划。
- `src/core/planning/ProcessPlanBuilder.cpp`：构建四轴分组、断面约束和排序计划。
- `include/core/planning/ProcessPlan.h`：定义计划、分配、分组、排除和 revision。
- `src/application/process/ProcessPresentationSnapshot.cpp`：将有效计划转换为 Viewer 展示数据。

## 当前实现差异

| 事项 | 当前生产实现 | 需求或概要设计要求 | 影响 |
|---|---|---|---|
| 普通与智能排序 | 两种 UI 入口最终调用相同的三轴或四轴计划函数 | 普通排序尽量保留当前单元序列、人工方向和起点；智能排序替换当前单元序列 | 当前两种入口只有命令名称差异 |
| 加工单元集合 | `ProcessGroup` 能表达单图元、连接链和闭环，但计划仍按逐图元 assignment 编号 | 两种排序先建立相同 `ProcessUnit` 集合且不得拆分 | 尚无统一 `ProcessUnit` 生产模型 |
| 当前顺序 | `DocumentProcessState` 携带逐图元 `manualProcessOrder`，构建器尚未消费 | Application 保存唯一 `ProcessUnitSequence` | 当前没有可编辑的单元序列，旧字段待移除 |
| 人工方向 | 普通和智能入口都把现有方向偏好送入规划器 | 智能排序应忽略人工方向 | 智能排序当前仍受人工方向约束 |
| 智能排序替换 | 智能入口生成计划，但没有可替换的 `ProcessUnitSequence` | 智能结果直接替换当前序列且不保留隐藏旧顺序 | 目标替换语义尚未实现 |
| 单元编号 | `ProcessAssignment::processOrder` 按图元连续编号 | 加工编号属于单元序列位置，一个单元一个编号 | 当前连续组成员仍可能显示多个编号 |
| 配置失效层级 | 当前配置切换会清除整个计划 | 仅排序配置变化应使计划失效；运动和文本配置应分别影响下游 | 当前失效范围比概要设计更保守 |
| 首组懒旋转 | 首个加工组固定按最近距离选择 | 懒旋转工艺强调减少 A 轴往复 | 首组不使用旋转代价，后续组才使用 |

## 对应需求与概要设计

需求：[MACH-041～MACH-043](../requirements/machining.md)、[PROCESS-018～PROCESS-023](../requirements/machining-process.md)。

概要设计：[数据流](../architecture/data-flow.md)、[领域模型](../architecture/domain-model.md)、[状态与生命周期](../architecture/state-and-lifecycle.md)。
