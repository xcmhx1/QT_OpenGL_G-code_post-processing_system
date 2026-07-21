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
| `selectionExpansionMs` | 选择集过滤、拓扑构建、相连图元扩展和候选验证耗时 |
| `boundaryAnalysisMs` | `RotaryCutBoundaryAnalyzer::analyze()` 的累计耗时 |
| `boundaryOrderingMs` | 闭环提取、断面排序及相邻断面顺序验证耗时 |
| `wasteRefreshMs` | `refreshWasteProcessingExclusions()` 的累计耗时 |
| `pathRebuildMs` | 实际调用 `rebuildRawPathPoints3D()` 的累计耗时 |
| `pointClassificationMs` | 批量调用 `classifyPointRelativeToBoundary()` 的累计耗时 |
| `processStateUpdateMs` | 断面状态批量更新及当前计划失效耗时 |
| `viewerRefreshMs` | Viewer 计划清除和刷新请求耗时 |
| `settingsSyncMs` | 加工设置面板状态同步耗时 |

部分阶段存在包含关系，例如断面分析内部会构建拓扑并重建路径，废面刷新内部也包含断面分析和点分类。
因此各阶段耗时可以重叠，不应直接相加后与 `totalMs` 比较。

## 4. 数据规模与调用次数

| 字段 | 含义 |
| --- | --- |
| `documentEntityCount` | 操作开始时的文档图元数 |
| `selectedEntityCount` | 操作开始时的选中图元数 |
| `boundaryGroupCount` | 废面刷新阶段收集到的加工断面和废面组数量 |
| `analyzedBoundaryCount` | 本次操作调用断面分析器的次数 |
| `rebuiltPathCount` | 实际执行路径重建的次数 |
| `reusedPathCount` | 直接复用既有加工路径的次数 |
| `classifiedEntityCount` | 废面区间判断实际处理的普通图元数 |
| `classificationCallCount` | 点相对断面分类函数的调用次数 |
| `samplePointCount` | 送入点分类的采样点总数 |

当前相关生产路径会显式重建路径，未引入缓存复用，因此 `reusedPathCount` 通常为 `0`。
若 `rebuiltPathCount` 明显大于 `documentEntityCount`，说明同一次操作中存在重复的全场景或局部路径重建。
若 `classificationCallCount` 随断面数和采样点数成倍增长，应结合 `pointClassificationMs` 判断点分类是否为主要热点。

## 5. 日志格式

日志使用单行固定字段，例如：

```text
[Performance][BoundaryAssignment] operation=AssignRotaryEndCut totalMs=3012.000 selectionExpansionMs=184.000 boundaryAnalysisMs=176.000 boundaryOrderingMs=92.000 wasteRefreshMs=2750.000 pathRebuildMs=1260.000 pointClassificationMs=1420.000 processStateUpdateMs=0.300 viewerRefreshMs=0.120 settingsSyncMs=0.900 documentEntityCount=846 selectedEntityCount=1 boundaryGroupCount=4 analyzedBoundaryCount=5 rebuiltPathCount=5076 reusedPathCount=0 classifiedEntityCount=814 classificationCallCount=152340 samplePointCount=152340
```

示例数值只用于说明字段格式，实际热点必须以目标 DXF 在 Release 构建中的输出为准。

## 6. 当前边界

本阶段只建立测量能力。当前未实施路径缓存、增量断面更新、异步计算或采样调整。
后续优化应先收集目标文件的完整汇总日志，再根据 `totalMs`、阶段耗时和调用规模确定修改位置。
