# 加工规划详细设计

## 职责

加工规划将当前 CAD 几何和加工上下文转换为确定的 `ProcessPlan`。
计划同时保存加工单元身份、单元顺序、成员实际执行顺序，以及供轨迹和 NC 使用的逐图元 assignment。
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
输出 `ProcessPlan`，并由其派生 Viewer 使用的展示快照。

## 主要数据类型

- `ProcessPlanningPolicy`：四轴连接容差、起始位置、方向许可、闭环原子组和排序策略。
- `PlanarProcessPlanningPolicy`：三轴起始位置、连接容差、数值误差和方向处理选项。
- `ProcessSortIntent`：以 `PreserveCurrentSequence` 和 `RebuildSequence` 明确区分普通排序与智能排序。
- `PlanningEntity`：核心规划使用的图元路径、状态、断面角色和来源属性。
- `ProcessGroup`：单图元、连接链、闭环、Break 断面或 Waste 断面组成的加工组。
- `ProcessAssignment`：逐图元执行顺序、所属加工单元索引、连续组、方向和起点，继续供轨迹与 NC 使用。
- `ProcessExclusion`：内部线、Waste 区间或其他规划排除结果。
- `ProcessUnit`：保存规范身份、成员实际执行顺序和闭合状态。
- `ProcessUnitKey`：仅表达单元身份，成员稳定 `EntityId` 保持升序、唯一和非空。
- `ProcessUnitSequence`：保存当前单元身份顺序和序列 revision，编号由位置 `+1` 得到。
- `ProcessUnitPresentation`：把单元身份、序列位置、成员执行顺序和首成员锚点投影给 Viewer。
- `ProcessPlan`：保存模式、排序策略、revision、加工单元、单元序列、分组、约束、逐图元分配和排除项。

## 当前生产数据流

普通排序生产链：

```text
“排序”动作
→ sortEntitiesByCurrentMode(PreserveCurrentSequence)
→ 三轴或四轴策略显式携带排序意图
→ ProcessPlanningService
→ 自动候选计划
→ 按当前 ProcessUnitSequence 匹配并重排仍存在单元
→ 未匹配新单元按候选计划顺序追加
→ ProcessPlan
→ ProcessPresentationSnapshot
```

智能排序生产链：

```text
“智能排序”动作
→ sortEntitiesByCurrentMode(RebuildSequence)
→ 三轴或四轴策略显式携带排序意图
→ DocumentProcessPlanningAdapter 忽略本次输入中的人工方向和起点
→ ProcessPlanningService 忽略当前 ProcessUnitSequence
→ 核心规划器重建全部单元顺序、方向和起点
→ ProcessPlan
→ ProcessPresentationSnapshot
```

三轴计划生产链：

```text
CadDocument + DocumentProcessState
→ DocumentProcessPlanningAdapter::capturePlanar
→ GeometryCompiler
→ PathTopology 连通分量
→ PlanarProcessPlanBuilder
→ 连续端点遍历
→ SingleEntity / ConnectedChain / ClosedLoop ProcessUnit
→ Planar3Axis ProcessPlan
```

四轴计划生产链：

```text
CadDocument + DocumentProcessState + 可选 TubeSectionModel
→ DocumentProcessPlanningAdapter::captureRotary
→ GeometryCompiler + PathTopology
→ ProcessPlanBuilder
→ ProcessGroup 转换为 ProcessUnit
→ Rotary4Axis ProcessPlan
```

当前普通排序的单元状态链：

```text
可加工图元
→ 现有排序算法生成 ProcessGroup 和 directed traversal
→ 自动候选 ProcessUnit + ProcessUnitSequence
→ 按 ProcessUnitKey 匹配当前权威序列
→ 保留匹配单元相对顺序
→ 新增、拆分或合并后的未匹配单元按候选相对顺序追加
→ 重建 processUnitIndex 和逐图元 processOrder
→ Application 替换唯一 ProcessUnitSequence
→ ProcessPlan
```

当前智能排序的单元状态链：

```text
可加工图元
→ 忽略当前权威单元序列、人工方向和人工起点
→ 现有几何排序策略重新计算
→ ProcessGroup 和 directed traversal
→ ProcessUnit + ProcessUnitSequence
→ Application 替换唯一 ProcessUnitSequence
→ ProcessPlan
```

## 状态所有权

主窗口保存唯一的当前 `ProcessPlan` 和 `ProcessPresentationSnapshot`。
核心规划器只返回值对象，不持有活动文档或 Viewer。
计划通过 `contentRevision` 和 `processStateRevision` 绑定生成时的文档与加工状态。
Viewer 只读取展示快照，不反向修改计划或加工输入。

Application 的 `DocumentProcessState` 持有当前 `ProcessUnitSequence`。Core 依据现有 `ProcessGroup` 和最终 directed traversal 形成加工单元；Viewer 不拥有顺序状态。

## 失效条件

- 文档内容 revision 或加工状态 revision 变化会清除当前计划和展示。
- 加工单元序列被替换或清除时推进加工状态 revision，使旧计划和展示失效；刚生成的计划绑定更新后的 revision。
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
- 三轴对拓扑连通分量执行端点连续遍历：独立图元形成单图元单元，完整开放链形成 `ConnectedChain`，严格闭环形成 `ClosedLoop`。
- 三轴分量存在分支、路径中部相交或方向约束冲突，无法形成完整连续遍历时，不强制合并并沿用独立图元调度。
- 严格闭环和可完整遍历的连续链形成原子加工单元，组内 assignment 在最终执行序列中连续，不被其他单元插入。
- 三轴当前使用最近距离计划，方向偏好会约束正向或反向候选。
- 四轴可按配置选择最近距离或懒旋转策略；懒旋转在后续选择中考虑 A 轴旋转代价。
- 四轴第一次组选择仍使用最近距离，再对后续组应用配置的排序策略。
- 三轴由拓扑分量和连续遍历直接形成 `ProcessUnit`，四轴由现有 `ProcessGroup` 和 directed traversal 形成 `ProcessUnit`；Waste 排除组不进入加工单元序列。
- `ProcessUnitKey` 使用组内全部稳定 `EntityId` 的升序集合，`orderedMemberEntityIds` 使用最终实际加工遍历顺序。
- 每个逐图元 assignment 通过 `processUnitIndex` 关联唯一加工单元，并继续按执行顺序保持连续且唯一。
- 三轴和四轴共用加工单元完整性校验：键必须规范，成员集合和执行顺序必须一致，每个 assignment 只属于一个单元，单元在执行序列中必须连续，单元序列与计划单元必须一一对应。
- 普通排序按 `ProcessUnitKey` 精确匹配当前权威序列，只保留仍存在单元的相对顺序；未匹配单元按自动候选计划中的相对顺序追加到末尾。
- 单元级重排保持 `orderedMemberEntityIds`、方向、起点和连续组不变，只重建单元位置、`processUnitIndex` 和连续的逐图元 `processOrder`。
- 普通排序继续读取并保留人工方向和人工起点。
- 智能排序不读取当前单元序列，并在本次规划输入中把方向视为 `Auto`、人工起点视为未设置；用户状态本身不被删除或修改。
- 重排结果继续通过加工单元结构校验和 Break 前后约束校验，校验成功后才更新权威序列。
- 普通排序保序与人工块内移尾共用单元级计划重排：整体移动 `ProcessUnit`，保留 `orderedMemberEntityIds`，重建 `processUnitIndex` 和连续 `processOrder`。
- 人工块内移尾在候选计划通过单元结构和 Break 约束校验后才更新 `DocumentProcessState`，然后将计划重新绑定到新 revision 并重建展示快照。
- 普通排序在单元顺序稳定后应用按 `ProcessUnitKey` 保存的单元遍历覆盖；智能排序不读取该覆盖，也不删除用户状态。
- 整组反向倒序 `orderedMemberEntityIds`、取反每个 assignment 的 `reverse`，并保留原 `startParameter`；单元在 `ProcessUnitSequence` 中的位置不变。
- 块内移尾和整组反向由 Desktop/Application 按值保存修改前后状态并构造执行、撤销回调；回调复用同一计划校验和原子发布路径。
- 加工状态回调提交到 `CadEditer` 通用历史入口，与 CAD 编辑共用同一 Undo/Redo 时间顺序；失败命令不会迁移到另一历史栈。
- 展示快照按加工单元保存 `unitOrder` 和成员身份，同时保留逐图元方向、起点、连续组和排除原因。
- 单元加工编号只由 `ProcessUnitSequence` 的位置 `+1` 产生，不存入成员图元。
- Viewer 顺序标签只读取单元展示项；逐图元 `processOrder` 继续仅服务轨迹和 NC。

## 相关源码

- `src/desktop/Gcode_postprocessing_system_SortActions.cpp`：提供排序 UI 入口、策略选择和计划保存。
- `src/application/planning/ProcessPlanningService.cpp`：组织文档捕获、核心规划和 revision 复核。
- `src/cad/editing/CadEditer_CommandActions.cpp`：提供不包含加工业务类型的回调命令，并接入统一 Undo/Redo 栈。
- `src/application/planning/DocumentProcessPlanningAdapter.cpp`：将生产文档和加工状态转换为规划值对象。
- `src/core/planning/PlanarProcessPlanBuilder.cpp`：构建三轴最近距离加工计划。
- `src/core/planning/ProcessPlanBuilder.cpp`：构建四轴分组、断面约束和排序计划。
- `include/core/planning/ProcessPlan.h`：定义计划、分配、分组、排除和 revision。
- `src/application/process/ProcessPresentationSnapshot.cpp`：将有效计划转换为 Viewer 展示数据。

## 当前实现差异

| 事项 | 当前生产实现 | 需求或概要设计要求 | 影响 |
|---|---|---|---|
| 普通与智能排序 | 两种入口显式传递不同 `ProcessSortIntent`；普通排序保留匹配单元顺序，智能排序重建序列 | 两种排序语义分离 | 已实现，不依赖按钮文字或命令标题判断 |
| 加工单元集合 | 三轴从 `PathTopology` 连通分量形成连续链、严格闭环或独立图元单元；四轴从现有加工组形成单元 | 两种模式统一使用加工单元语义 | 已建立统一核心模型；复杂分支分量保持独立图元调度 |
| 当前顺序 | `DocumentProcessState` 保存唯一 `ProcessUnitSequence`；Ctrl 块内移尾可修改连续范围 | Application 保存唯一加工单元序列 | 状态和块内移尾入口已实现 |
| 人工方向与起点 | 普通排序读取用户状态；智能排序只在规划输入边界忽略，用户状态继续保留 | 智能排序重新计算方向和起点 | 已实现 |
| 智能排序替换 | 智能结果不匹配旧序列，直接更新唯一 `ProcessUnitSequence` | 智能结果直接替换当前序列 | 已实现 |
| 单元编号 | 单元编号由序列位置产生，Viewer 每个单元显示一个标签；assignment 保留逐图元执行顺序 | 一个加工单元一个显示编号 | 已实现 |
| 单元交互 | 标签单击选择全部成员，Ctrl 单击执行连续块内移尾，双击标签或箭头执行整组反向 | 支持单元级交互 | 已实现并支持 Undo/Redo |
| 历史边界 | Application 保存加工状态前后值并提交通用回调命令；CAD 与加工操作共用一个历史栈 | CAD 层不拥有加工单元人工编辑类型 | 块内移尾和整组反向已迁移；删除和替换命令仍暂时依赖 `DocumentProcessState` |
| 配置失效层级 | 当前配置切换会清除整个计划 | 仅排序配置变化应使计划失效；运动和文本配置应分别影响下游 | 当前失效范围比概要设计更保守 |
| 首组懒旋转 | 首个加工组固定按最近距离选择 | 懒旋转工艺强调减少 A 轴往复 | 首组不使用旋转代价，后续组才使用 |

## 对应需求与概要设计

需求：[MACH-041～MACH-043](../requirements/machining.md)、[PROCESS-018～PROCESS-023](../requirements/machining-process.md)。

概要设计：[数据流](../architecture/data-flow.md)、[领域模型](../architecture/domain-model.md)、[状态与生命周期](../architecture/state-and-lifecycle.md)。
