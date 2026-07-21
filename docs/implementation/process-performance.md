# 加工断面性能测量

## 1. 目的

当前生产实现对加工断面指定、自动识别、重新指定和清除操作进行分阶段耗时统计。
统计用于定位界面停顿发生在哪个生产阶段，不参与业务判断，也不改变断面识别、废面区间或加工状态结果。

## 2. 测量入口

以下操作会建立一次操作级报告：

- `AssignRotaryEndCut`：手动指定加工断面；
- `SmartAssignRotaryEndCut`：从选中图元智能扩展并指定；
- `RecognizeAllRotaryEndCuts`：识别文档中的全部加工断面；
- `ReassignRotaryEndCut`：选中范围同时包含已指定和未指定图元时重新指定；
- `ClearSelectedRotaryEndCut`：清除选中加工断面；
- `ClearAllRotaryEndCuts`：清除全部加工断面；
- `AssignWasteEndCut`：指定废面；
- `ClearAllBoundaryAndInternalAssignments`：清空加工断面和内部线状态。

嵌套调用共享同一份报告。一次用户操作结束后只输出一条以
`[Performance][BoundaryAssignment]` 开头的 `qInfo()` 日志。

## 3. 阶段耗时

| 字段 | 含义 |
| --- | --- |
| `totalMs` | 从用户操作入口到操作返回的总耗时 |
| `selectionExpansionMs` | 相连图元扩展和候选验证耗时 |
| `boundaryAnalysisMs` | `RotaryCutBoundaryAnalyzer::analyze()` 的累计耗时 |
| `boundaryOrderingMs` | 闭环提取、断面排序及相邻断面顺序验证耗时 |
| `wasteRefreshMs` | `refreshWasteProcessingExclusions()` 的累计耗时 |
| `topologyBuildMs` | 本次操作构建全场景拓扑的累计耗时 |
| `topologyAdapterMs` | `LegacyCadItemTopologyAdapter::convert()` 的总适配耗时 |
| `endpointCompileMs` | 适配阶段调用精确路径编译的累计耗时 |
| `pathCleanupMs` | 适配阶段复制、去重、闭合处理及记录构造的累计耗时 |
| `coreTopologyBuildMs` | `PathTopologyBuilder::build()` 的总耗时 |
| `connectivityScanMs` | Core 遍历记录对并建立邻接关系的耗时 |
| `recordMappingMs` | Core 记录映射为 `RotaryPathTopologyRecord` 的耗时 |
| `pathRebuildMs` | 实际调用 `rebuildRawPathPoints3D()` 的累计耗时 |
| `pointClassificationMs` | 批量调用 `classifyPointRelativeToBoundary()` 的累计耗时 |
| `processStateUpdateMs` | 断面状态批量更新及当前计划失效耗时 |
| `viewerRefreshMs` | Viewer 计划清除和刷新请求耗时 |
| `settingsSyncMs` | 加工设置面板状态同步耗时 |

`endpointCompileMs` 和 `pathCleanupMs` 是 `topologyAdapterMs` 的子阶段；
`connectivityScanMs` 是 `coreTopologyBuildMs` 的子阶段；适配、Core 构建和记录映射共同位于 `topologyBuildMs` 内。
嵌套阶段不能与父阶段直接相加，也不应直接相加后与 `totalMs` 比较。

## 4. 数据规模与调用次数

| 字段 | 含义 |
| --- | --- |
| `documentEntityCount` | 操作开始时的文档图元数 |
| `selectedEntityCount` | 操作开始时的选中图元数 |
| `boundaryGroupCount` | 废面刷新阶段收集到的加工断面和废面组数量 |
| `analyzedBoundaryCount` | 本次操作调用断面分析器的次数 |
| `topologyBuildCount` | 本次操作实际构建全场景拓扑的次数，不包含闭环提取 |
| `topologyReuseCount` | 候选扩展、断面分析和废面刷新复用已构建拓扑的次数 |
| `topologyRecordCount` | 成功进入拓扑输入的路径记录数 |
| `totalPathPointCount` | 全部拓扑记录包含的路径点总数 |
| `totalSegmentCount` | 全部拓扑记录包含的相邻点线段总数 |
| `recordPairCount` | Core 连接扫描实际检查的记录对数量 |
| `endpointToPathTestCount` | 记录对扫描中实际执行的端点到路径检测次数 |
| `segmentPairTestCount` | 端点检测未提前命中后实际执行的线段对精确距离检测次数 |
| `connectedRecordPairCount` | 判定为直接连接的记录对数量 |
| `adjacencyEdgeCount` | 建立的无向邻接边数量 |
| `rebuiltPathCount` | 实际执行路径重建的次数 |
| `reusedPathCount` | 直接复用既有加工路径的次数 |
| `classifiedEntityCount` | 废面区间判断实际处理的普通图元数 |
| `classificationCallCount` | 点相对断面分类函数的调用次数 |
| `samplePointCount` | 送入点分类的采样点总数 |

加工断面指定、重新指定和自动识别在单次操作内复用同一个全场景拓扑。
正常情况下 `topologyBuildCount` 为 `1`，`rebuiltPathCount` 接近本次过滤后的场景图元数；
废面刷新直接复用拓扑记录中的路径点，并计入 `reusedPathCount`。
若 `classificationCallCount` 随断面数和采样点数成倍增长，应结合 `pointClassificationMs` 判断点分类是否为主要热点。

## 5. 日志格式

日志使用单行固定字段，例如：

```text
[Performance][BoundaryAssignment] operation=AssignRotaryEndCut totalMs=... selectionExpansionMs=... boundaryAnalysisMs=... boundaryOrderingMs=... wasteRefreshMs=... topologyBuildMs=... topologyAdapterMs=... endpointCompileMs=... pathCleanupMs=... coreTopologyBuildMs=... connectivityScanMs=... recordMappingMs=... pathRebuildMs=... pointClassificationMs=... processStateUpdateMs=... viewerRefreshMs=... settingsSyncMs=... documentEntityCount=100 selectedEntityCount=1 boundaryGroupCount=4 analyzedBoundaryCount=5 topologyBuildCount=1 topologyReuseCount=7 topologyRecordCount=100 totalPathPointCount=... totalSegmentCount=... recordPairCount=4950 endpointToPathTestCount=... segmentPairTestCount=... connectedRecordPairCount=... adjacencyEdgeCount=... rebuiltPathCount=100 reusedPathCount=... classifiedEntityCount=... classificationCallCount=... samplePointCount=...
```

示例数值只用于说明字段格式，实际热点必须以目标 DXF 在 Release 构建中的输出为准。

## 6. 拓扑构建内部测量

适配器按图元累计精确端点编译和路径清理时间，不输出图元级日志。
Core 只在完整连接扫描阶段外层计时，记录对、端点到路径和线段对循环内部仅累加计数。
记录映射统计 EntityId 查找、坐标类型转换和兼容记录构造的总耗时。

统计对象仅由 Compatibility、Core 和 Application 同步填写，不参与连接判断、闭环选择或错误处理。
对于 100 条有效拓扑记录，完整两两扫描的 `recordPairCount` 应为 4950；其他检测次数受现有提前退出规则影响。

## 7. 操作级拓扑快照

操作入口按当前内部线和加工断面状态过滤一次场景图元，随后完成一次兼容适配和一次
`RotaryPathTopology` 构建。快照只保存本次操作的文档内容版本、连接容差、过滤后图元集合和拓扑对象。

候选连通扩展、每个加工断面分析以及废面刷新共享该快照。废面刷新中的普通图元分类读取拓扑记录中的路径点，
不会再次调用图元路径重建。没有外部快照的独立刷新入口会在函数内部构建一次局部快照。

快照不写入 `CadDocument` 或主窗口状态，不跨用户操作和文档内容版本复用，也不保存加工断面角色或 Waste 结果。
加工状态可以在同一操作中更新，但几何内容版本变化后必须重新构建快照。

## 8. 当前边界

当前只复用单次加工断面操作内的拓扑，不提供跨操作缓存、增量断面更新或异步计算，也未调整采样和点分类算法。
本次内部测量未增加粗筛、缓存或高频计时器。后续优化仍应以目标文件的完整汇总日志为依据。
