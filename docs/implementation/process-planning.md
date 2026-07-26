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

- `ProcessPlanningPolicy`：四轴连接容差、起始位置、方向许可、闭环原子组、排序策略、16 区位扫描策略和入口精化使用的转移距离语义。
- `Zone16SweepPolicy`：固定 `TopFace` 初始区位，并保存可配置的周向和轴向扫描方向。
- `PlanarProcessPlanningPolicy`：三轴起始位置、连接容差、数值误差和方向处理选项。
- `ProcessSortIntent`：以 `PreserveCurrentSequence` 和 `RebuildSequence` 明确区分普通排序与智能排序。
- `PlanningEntity`：核心规划使用的图元路径、状态、断面角色和来源属性。
- `ProcessGroup`：单图元、连接链、闭环、Break 断面或 Waste 断面组成的加工组。
- `ProcessAssignment`：逐图元执行顺序、所属加工单元索引、连续组、方向和起点，继续供轨迹与 NC 使用。
- `ProcessPathFragment`：为 `BreakBoundary` 或普通多图元 `ClosedLoop` 的圆弧、椭圆弧成员内部起刀保存源参数区间和片段执行顺序，不改变图元与加工单元身份。
- `ProcessExclusion`：内部线、Waste 区间或其他规划排除结果。
- `ProcessUnit`：保存规范身份、成员实际执行顺序、最终 `ownerZone` 和闭合状态。
- `ProcessUnitKey`：仅表达单元身份，成员稳定 `EntityId` 保持升序、唯一和非空。
- `ProcessUnitSequence`：保存当前单元身份顺序和序列 revision，编号由位置 `+1` 得到。
- `ProcessUnitPresentation`：把单元身份、序列位置、成员执行顺序和首成员锚点投影给 Viewer。
- `TubeZone16` 与 `TubeZoneMask`：按顺时针定义顶面、右上圆角、右面、右下圆角、底面、左下圆角、左面、左上圆角及八条相邻分界母线。
- `ProcessUnitZoneProfile`：记录一个最终加工单元经过的全部截面区位和各区位 X 范围；规划器另行生成可执行入口区位画像，避免把“经过区位”当成“可从该区位起刀”。
- `ZoneSweepOwnership`：在单个加工分区启动时冻结普通加工单元的唯一 `ownerZone`，仅用于本次计划构建的生产分桶。
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
→ DocumentProcessPlanningAdapter 对一般候选使用自动方向和起点，并保留人工约束用于 16 区位入口冲突校验
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

四轴 16 区位规划链：

```text
冻结的 ProcessGroup
+ 原始 Path3D
+ 有效 TubeSectionModel
→ TubeSectionProjector
→ ProcessUnitZoneProfile 占用画像
→ certainMask / possibleMask 占用画像
→ 按分区周向顺序冻结唯一 ownerZone
→ 为 ownerZone 补齐可执行入口
→ 每个单元进入一个生产区位桶
→ 当前区位完整完成后推进 16 区位扫描
→ 当前区位硬约束下选择入口和方向
→ ProcessPlan
→ EntryZoneProfile / ZoneOwnership / ZonePhase / EntrySelection / Zone16Sweep 诊断日志
```

Break 断面入口链：

```text
当前 16 区位扫描状态
→ preferredStartZone
→
唯一简单环
→ 两个环绕方向
→ preferredStartZone 内的可靠连续边段
→ 连续边段三维弧长中点
→ 首成员前半段 + 其他成员 + 首成员后半段
→ 从最终计划片段解析出口强区位
→ 初始化下一 16 区位加工分区
```

占用画像在调度前一次性计算，只描述原始路径经过的截面区位和各区位 X 范围。分区启动时按实际初始区位和周向方向，先在 `certainMask` 的四个平面和四个圆角强区位中选择唯一 `ownerZone`；没有可靠强区位时再检查 `possibleMask` 的强区位并记录警告，只有两者均无强区位时才使用分界母线并记录警告。入口画像由实际可执行的端点、闭合参数、闭环连接点、圆弧/椭圆弧成员内部切点和普通多图元闭环的可靠区位连续段中点生成；`connectionEntryMask`、`curveInteriorEntryMask` 与 `zoneRunMidpointEntryMask` 合并为 `legalEntryMask`。`legalEntryMask` 只表达入口能力，不参与 `ownerZone` 选择；owner 冻结后，入口算法必须适配该区位。这些临时画像和归属不写入 `DocumentProcessState` 或最终 `ProcessPlan`。

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
- 四轴可按配置选择最近距离或懒旋转策略。仅在智能重建、懒旋转且存在有效截面时启用 16 区位生产扫描；普通排序、最近距离和无截面四轴保持原流程。
- 每个普通 `ProcessGroup` 在调度前使用全部成员的原始 `Path3D` 形成静态占用画像。`certainMask` 和 `possibleMask` 表达路径经过区位，`legalEntryMask` 表达实际可执行入口区位；唯一 `ownerZone` 表达该单元在当前加工分区中的生产归属。
- Break 断面继续由现有工艺屏障划分加工分区。Break 专用遍历器仅在唯一简单环上分析四个平面和四个圆角强区位，八条分界母线不作为首选入口。
- 懒旋转扫描的初始工艺区位固定为 `TopFace`，该状态不由首个加工单元或最左断面的偶然位置推导。
- Break 起刀点取当前扫描区位内可靠连续边段的三维弧长中点；连续边段可以跨越多个源图元。首个 Break 接收当前 `TopFace`，完成闭环后出口仍回到该入口区位，后续继续从当前区位扫描。
- 普通多图元闭环的单元顺序、`ownerZone`、`scheduledZone`、区位顺序和 X 前沿先完成冻结，随后才精化该单元的入口与组内遍历；入口评分不参与加工单元选择，也不得改变已冻结的区位归属和顺序。
- 入口精化以实际前序切削终点按转移类型生成的 `previousTransferAnchor` 为接近起点，并在方管展开空间中比较接近距离和方向。跨区使用旋转安全包络锚点，同区正离面使用局部外法向锚点，同区零离面直接使用切削终点。
- owner 区位内的非闭合 `Arc` 使用圆方程解析求取真实外部切点；部分 `Ellipse` 使用 `cross(P-E(u), E'(u))=0` 在有效参数区间内扫描夹根并有限二分求根。切点必须位于源参数范围、可靠 owner 区位内部，并能形成完整连续闭环。
- 真实曲线切点同时枚举闭环两个方向，优先接近方向与首个非退化切削方向同向，再比较移动、轴反向、旋转和稳定身份。圆弧极点、固定比例参数点和离散路径顶点不参与切点候选。
- 当前 owner 区位没有通过全部校验的真实曲线切点时，先选择距离 `previousTransferAnchor` 最近的合法成员连接点；距离在并列容差内时稳定消歧，同一连接点选择与接近方向夹角更小的闭环方向。连接点也不存在时，才在最长可靠连续区位段上取三维弧长中点。
- 排序负责加工单元顺序和 `ownerZone`，入口精化负责单元内部实际起点与方向，轨迹层负责单元间安全转移；转移锚点只参与既定单元的入口评分，不反向改变排序结果。
- 闭环成员内部起刀通过两个互补 `ProcessPathFragment` 表达：先加工切点到成员出口，绕行其他成员后再加工成员入口到切点。规划发布前按源图元核对片段总弧长，并校验全部片段连续、闭合、无重复和无遗漏。
- `ProcessPathFragment` 仅允许属于匹配的 `BreakBoundary` 或普通 `ClosedLoop` 加工单元。开放单图元和开放连续链只允许从物理端点进入，不生成计划片段。
- 单图元闭合 `Circle` 和完整 `Ellipse` 在智能重建、懒旋转且截面有效时，也在加工单元顺序和 `ownerZone` 冻结后执行后置入口精化；闭合样条继续沿用独立入口规则。
- Break 的起点和出口必须属于同一强区位。出口从最终计划片段向落刀点反向累计可靠长度，短分界段和歧义段不作为强出口；仅在强信息不足时允许 possible 区位兜底并记录警告。
- Break 完成后直接使用片段解析得到的出口区位初始化下一分区，不再根据普通闭环尾成员的偶然末段猜测。出口解析、分区映射和分区启动分别返回结构化诊断。
- 没有 Break 时，首个分区也从策略固定的 `TopFace` 启动。
- 每个分区按实际入口区位以及配置的顺时针或逆时针方向遍历 16 区位。普通加工单元只进入 `ownerZone` 对应的一个生产桶；跨区闭环仍作为原子单元整体加工，不因占用或入口掩码包含其他区位而重复分桶。
- 每个区位拥有独立 X 前沿。`+X` 扫描使用该区位跨度的 `minimumX` 命中并以 `maximumX` 推进前沿，`-X` 扫描反向处理；进入新区位时从分区边界重新建立前沿。
- 区位和 X 命中先选定整个 `ProcessGroup`，再把当前 `TubeZone16` 作为入口硬约束枚举合法端点、闭合参数、成员连接点或闭环成员内部切点。错误区位候选在评分前排除，入口评分不能改变区位、X 顺序或加工单元顺序。
- 入口区位由入口点和其后的第一个非退化切削段共同确定；分界母线只参与消歧，歧义短段不能形成合法入口。合法候选在方管展开空间中比较入口反向、切线连续性、X 命中距离、旋转、移动和稳定身份。
- 跨区加工单元完成后只更新实际当前位置，当前扫描区位保持不变。当前区位的唯一生产桶内全部 owner 单元完成后才按周向策略切换下一区位；桶中仍有未完成但受前置约束阻塞的单元时直接返回结构化失败，不跳过区位或稍后返回。
- 每个分区记录区位 Enter/Complete 状态以及分区 started/finished 状态。区位只能进入一次且 Enter/Complete 必须成对，分区只能启动和结束一次；计划发布前再次校验单元归属、分桶和调度次数均为一。
- 前沿只允许单调推进。完整落在当前前沿之后的未加工单元会返回结构化回退诊断，不再通过回头扫描掩盖区位画像或分区问题。
- 四轴单图元闭合圆和完整椭圆的 Auto 起点在规划输入中保持为空。最终自动参数由后置精化产生，人工起点不进入该分支。
- 后置精化按冻结后的加工单元顺序传递真实切削终点。每个候选参数都通过共享动态转移预览器重新计算其 `finalApproachOrigin`，再在目标加工平面内求解 `cross(Q(u)-C(u), C'(u))=0` 或 `cross(Q(u)-E(u), E'(u))=0`。
- 周期参数搜索使用有限区间扫描和有限二分求根，最终起点来自源曲线参数，不使用极点、固定参数或最近离散顶点。每个切点同时比较正反方向，优先接近方向与首段切线同向，再依次比较明确的线性距离、A 轴反向次数、A 轴总变化和稳定参数。
- owner 区位内没有真实动态切点时，仅在可靠参数段内部执行 `ClosestOwnerZoneParameterFallback`；该模式明确记录为角度最小回退，不伪装成几何切点。
- 选中结果把 `startParameter`、`reverse` 和动态转移预览签名写入当前 `ProcessAssignment`。精化前后校验 `ProcessUnitKey` 序列和 `ownerZone` 完全一致，入口变化不反馈到区位、X 前沿或加工单元顺序。
- 四轴多图元 `ClosedLoop` 仅以冻结后的组成员建立端点图；每个成员作为一条边，端点按当前连接容差聚类，所有节点度数必须为 2，且边、节点和连通分量共同构成唯一简单环。
- 多图元闭环不再使用最近端点贪心拼接。规划器枚举起始成员及其正反入口，沿端点图唯一绕行全部成员，并按完整候选统一比较入口平滑度、旋转、表面和移动代价。
- 每个闭环成员只允许使用一次，成员方向继续服从人工方向偏好和全局反向许可；含自身闭合成员、分支、孤立边或方向冲突的多图元组直接拒绝，不回退通用遍历。
- 闭环候选先验证最后成员物理终点可连接到入口，再把组结束位置设置为真实入口。计划发布前再次校验成员集合、Assignment 连续性、相邻物理端点及首尾闭合，失败结果不会更新后续组的当前位置。
- 智能懒旋转的普通单元入口保留用户已设置的方向和起点作为硬约束输入；约束无法在当前扫描区位形成合法入口时拒绝计划，不静默改写用户状态。合法入口再依次比较区位边界距离、入口 X/周向反向数、展开空间切线代价、X 命中距离、旋转代价、移动距离和稳定身份。
- `NearestNext` 继续以移动距离为首要比较项；普通排序不启用本次智能平滑入口规则。
- 自动选出的入口参数和方向仅写入当前 `ProcessAssignment`，不回写 `DocumentProcessState`。
- 三轴由拓扑分量和连续遍历直接形成 `ProcessUnit`，四轴由现有 `ProcessGroup` 和 directed traversal 形成 `ProcessUnit`；Waste 排除组不进入加工单元序列。
- `ProcessUnitKey` 使用组内全部稳定 `EntityId` 的升序集合，`orderedMemberEntityIds` 使用最终实际加工遍历顺序。
- 每个逐图元 assignment 通过 `processUnitIndex` 关联唯一加工单元，并继续按执行顺序保持连续且唯一。
- 三轴和四轴共用加工单元完整性校验：键必须规范，成员集合和执行顺序必须一致，每个 assignment 只属于一个单元，单元在执行序列中必须连续，单元序列与计划单元必须一一对应。
- 普通排序按 `ProcessUnitKey` 精确匹配当前权威序列，只保留仍存在单元的相对顺序；未匹配单元按自动候选计划中的相对顺序追加到末尾。
- 单元级重排保持 `orderedMemberEntityIds`、方向、起点和连续组不变，只重建单元位置、`processUnitIndex` 和连续的逐图元 `processOrder`。
- 普通排序继续读取并保留人工方向和人工起点。
- 智能排序不读取当前单元序列；一般几何排序仍以自动方向和自动起点重建。16 区位懒加工额外携带原始人工方向和起点，仅用于验证当前区位入口约束，用户状态本身不被删除或修改。
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
- 16 区位掩码表达加工单元实际经过的全部区位，不选择单一主区位。默认顺时针顺序为 `TopFace → TopToTopRightBoundary → TopRightCorner → TopRightToRightBoundary → RightFace → RightToBottomRightBoundary → BottomRightCorner → BottomRightToBottomBoundary → BottomFace → BottomToBottomLeftBoundary → BottomLeftCorner → BottomLeftToLeftBoundary → LeftFace → LeftToTopLeftBoundary → TopLeftCorner → TopLeftToTopBoundary`；逆时针按相反方向遍历。
- 区位画像按最终 `Path3D` 线段计算；端点、线段中点和必要的临时细分共同识别跨区路径。临时细分只存在于分类过程，不修改生产路径或采样策略。
- 平面和圆角只有累计非零投影长度超过数值阈值才形成强占用；路径穿越、沿线运行或稳定端点接触可直接标记相应分界母线。
- 圆弧、椭圆和样条允许在独立投影容差内投影到理想方管壳层，原始路径保持不变；画像记录最大、平均壳层偏差，并把接近容差边缘或无法可靠消歧的结果标记为不确定。
- 智能懒旋转生产排序使用 `certainMask`、`possibleMask` 和各区位 X 范围确定唯一生产归属，再使用 `legalEntryMask` 建立 owner 区位入口。入口候选数量、切线代价、移动距离和最终入口不得反向改变归属；旧主表面、`Mixed` 和中值锚点不参与该生产分支。

## 相关源码

- `src/desktop/Gcode_postprocessing_system_SortActions.cpp`：提供排序 UI 入口、策略选择和计划保存。
- `src/application/planning/ProcessPlanningService.cpp`：组织文档捕获、核心规划和 revision 复核。
- `include/infrastructure/config/GProfile.h`：保存周向、轴向扫描方向及默认值。
- `src/ui/dialogs/GProfileDialog.cpp`：提供周向和轴向扫描方向配置入口。
- `src/cad/editing/CadEditer_CommandActions.cpp`：提供不包含加工业务类型的回调命令，并接入统一 Undo/Redo 栈。
- `src/application/planning/DocumentProcessPlanningAdapter.cpp`：将生产文档和加工状态转换为规划值对象。
- `src/core/planning/PlanarProcessPlanBuilder.cpp`：构建三轴最近距离加工计划。
- `src/core/planning/ProcessPlanBuilder.cpp`：构建四轴分组、断面约束和排序计划。
- `src/core/machining/TubeSectionProjector.cpp`：将 YZ 点投影到四个平面、四个圆角和八条分界母线，并生成加工单元区位画像。
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
| 懒旋转初始区位 | 生产策略固定从 `TopFace` 开始，可配置周向顺/逆时针及轴向 `+X/-X` | 初始区位和扫描方向具有明确工艺语义 | 已实现，不再由首单元或最左 Break 反推 |

## 对应需求与概要设计

需求：[MACH-041～MACH-043](../requirements/machining.md)、[PROCESS-018～PROCESS-023](../requirements/machining-process.md)。

概要设计：[数据流](../architecture/data-flow.md)、[领域模型](../architecture/domain-model.md)、[状态与生命周期](../architecture/state-and-lifecycle.md)。
