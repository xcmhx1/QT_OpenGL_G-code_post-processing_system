# AGENTS.md

本文件规定 Codex 和开发者修改本仓库时必须遵守的执行约束。项目当前事实、架构、构建和测试入口见 [README.md](./README.md)。

## 1. Think Before Coding

**不要猜测，不要隐藏不确定性。**

开始实现前：

- 明确用户动作、输入、应变化的数据和最终可见结果。
- 阅读 README 中与任务相关的章节，并检查当前源码。
- 多种解释存在时，说明差异；高风险歧义无法从仓库确认时先询问。
- 如果更简单的方案已能满足要求，应明确采用简单方案。
- 不根据提交信息单独推断现状，提交历史只用于定位。

## 2. Simplicity First

**使用解决问题所需的最少代码。**

- 不实现用户未要求的功能。
- 不为单一用途设计可配置框架。
- 不增加只做转发的层。
- 不为一个静态函数创建公共类。
- 单一调用方辅助代码优先放在 `.cpp` 私有区。
- 如果实现明显比问题复杂，先缩小设计。

## 3. Surgical Changes

**只修改任务需要的文件和代码。**

- 不顺手重构、改格式或清理相邻代码。
- 保持现有代码风格和文件职责。
- 不回退或覆盖用户已有的未提交修改。
- 意外变化与当前任务冲突时停止并说明；无冲突时忽略。
- 只删除本次修改造成的未使用代码。
- 不使用破坏性 Git 命令，不擅自 amend。
- `src/infrastructure/dxf/legacy/dx_iface.cpp`、`include/infrastructure/dxf/legacy/dx_iface.h`、`include/libdxfrw/` 和 `src/libdxfrw/` 默认视为第三方代码。

## 4. Data-Flow Verification

把任务按真实数据流处理：

```text
1. 触发动作或输入
2. 核心数据和状态转换
3. 最终显示、计划、轨迹或文件
4. 用端到端结果验证
```

- 不只验证被编辑的函数。
- 确认输入实际进入了正确生产入口。
- 确认版本、状态和结果沿完整链路传播。
- 确认最终 UI、`ProcessPlan`、`MachineTrajectory`、`NcProgram` 或输出文件符合目标。
- Bug 修复应定位最早出现错误的数据阶段，不在末端补偿错误结果。

## 5. Repository Facts

- `README.md` 是项目事实入口。
- 开发前必须阅读与任务相关的 README 章节。
- README 与源码冲突时，以源码和可重复测试为准，并在同一任务中修正文档。
- README 记录“项目现在是什么”，AGENTS 记录“修改时必须怎么做”。
- 不把开发日志、提交历史或理想架构写入 README。
- 不记录个人客户信息、许可证密钥、私钥或无必要的本机用户名。

## 6. Production Boundaries

生产事实所有权固定为：

- CAD 精确几何：`SourceEntity`。
- 统一计算路径：`Path3D`。
- 用户加工输入：`DocumentProcessState`。
- 自动规划结果：`ProcessPlan`。
- 四轴机床运动：`MachineTrajectory`。
- NC 语义：`NcProgram`。
- G-code 文本：`GCodePostProcessor`。

约束：

- `CadItem` 负责原始实体、编辑、显示、拾取和控制点，不拥有生产加工计划。
- `ProcessPresentationSnapshot` 只供 Viewer 展示，不参与计划或导出。
- `GGenerator` 只编排服务和写文件，不重新生成几何、A 轴、安全移动或过切。
- UI、Viewer 和主窗口不得成为几何、规划、轨迹或 NC 算法所有者。
- Core 不依赖 `CadItem`、DRW、QObject、QWidget 或 GUI。
- 稳定加工身份只能来自 `CadItem::m_entityId`；不得使用对象地址作为业务 `EntityId`。
- Viewer 短生命周期缓存和选择键必须使用 `RenderEntityKey`，不得跨对象生命周期持久保存。
- Compatibility 只处理边界兼容，不承接新功能所有权。
- 新核心不得依赖 compatibility。

## 7. Numerical Rules

- 核心几何计算使用 `double`。
- 世界坐标用于保存、显示、跨模块值对象和最终输出。
- 几何、拓扑、截面和断面解算优先使用确定性局部坐标。
- 使用稳定包围盒中心和必要的补偿求和。
- 不让 `QVector3D` 或 float 显示缓存进入核心计算。
- 不根据世界坐标大小放宽闭环、连接或工艺公差。
- 不用提高输出小数位掩盖数值错误。
- 不使用日志字符串控制业务。
- 严格闭环只接受语义闭合或物理连接点在 `numericalJoinEpsilon` 内重合。
- 数值修改必须验证平移不变性和远离原点场景。

## 8. Module Growth

- 优先修改 README “模块所有权”和“开发入口表”中已有所有者。
- 新公共模块必须同时具备独立契约、独立不变量和明确复用价值。
- 只有两个以上生产调用方时，才优先考虑抽取公共模块。
- 不新增没有业务不变量的 `Manager`、`Utils`、`Helper`、`Facade`。
- 不复制拓扑、截面、规划、运动学或后处理算法到兼容层和 UI。
- 不保留新旧生产算法的运行时 fallback。
- 明确兼容降级必须返回 `PartialSuccess` 和结构化 Warning，不能静默发生。

## 9. Geometry and Topology

- DXF/DRW 到核心值对象的转换归 `DxfGeometryAdapter`。
- 精确几何到采样路径归 `GeometryCompiler`。
- NURBS 求值归 `NurbsCurveEvaluator`。
- 连通、交点和严格闭环归 `PathTopology`。
- 方管截面和内部图元归 `TubeSectionAnalyzer`。
- 加工断面归 `TubeCutBoundaryClassifier`。
- 不用凸包替换真实边界图元。
- 不自动连接开放端点，不用毫米级容差把间隙改成闭环。
- 显示离散、兼容缓存和加工路径必须保持职责分离。

## 10. Planning, Machine and NC

- 三轴排序归 `PlanarProcessPlanBuilder`。
- 四轴排序、连续组、Break/Waste 和 precedence constraints 归 `ProcessPlanBuilder`。
- 用户方向和起点只从 `DocumentProcessState` 读取。
- 实际方向、起点、顺序和组只从 `ProcessPlan` 读取。
- A 轴、方管圆角刀头方向、安全移动、连接和过切归 `core/machine`。
- 三轴 NC 归 `PlanarNcProgramBuilder`。
- 四轴 NC 归 `NcProgramBuilder`/`NcProgramService`。
- 文本代码块、轴字格式、精度、CRLF 和文本优化归 `GCodePostProcessor`。
- 没有版本一致、模式匹配的计划时必须拒绝导出。
- 失败时不得生成或写入部分 NC 文本。

## 11. Revisions and State

- 几何和 CAD 内容变化推进 `contentRevision`。
- 加工启用、方向、起点、Break/Waste 和内部分析变化推进 process-state revision。
- 批量操作只推进一次对应版本。
- 选择、高亮、视图、主题和显示开关不推进业务版本。
- 自动计划结果不得写回用户加工输入。
- 应用计划、轨迹或 NC 前必须验证文档版本、process-state 版本和模式。
- 不通过清空 `CadItem` 加工字段表达计划失效。

## 12. Thread Boundaries

- `CadDocument`、`CadItem` 和 DRW 只能在文档所在线程捕获。
- worker 只能读取不可变值对象，不访问 QObject、DRW 或 GUI。
- 并行结果必须按稳定 `sourceIndex` 确定性合并。
- 取消必须等待已运行任务安全结束。
- 后台结果应用前必须检查 revision，过期结果直接丢弃。
- 工作线程不得直接更新 QWidget 或发布 UI sink。
- 不把尚未接入生产的并发能力描述为当前后台工作流。

## 13. Change Rules

- 只修改当前任务需要的代码，不做无关重构。
- 手动和自动入口复用同一业务函数和几何规则。
- Bug 修复定位最早错误阶段，不在后处理或 UI 末端打补丁。
- 不通过放宽容差让测试通过。
- 不修改黄金文件来掩盖行为回归。
- 行为变化必须增加最小回归。
- G-code 变化必须说明黄金变化原因和实际业务依据。
- 修改第三方 DXF 边界前先证明问题位于该层。
- 配置键变化必须提供兼容迁移或明确破坏性说明。
- 用户可见术语应在 UI、README 和诊断中保持一致。
- 删除遗留代码前必须分别检查生产 callers 和测试 callers；仅测试使用的 parity 不得编入主程序。
- 修改调用边界前优先使用 CodeGraph 检查 callers、callees 和依赖路径，结果必须再由源码、测试和构建验证。

## 14. Editing Rules

- 手工修改文件使用 `apply_patch`。
- 默认使用 ASCII；已有中文文档和用户可见中文文案可继续使用 Unicode。
- 只添加解释非显然约束的短注释。
- 不使用脚本重写少量代码。
- 格式化工具只能作用于任务涉及的文件。
- 生成文件、构建产物和第三方代码不做无关编辑。
- 自有 C/C++ 文件必须放入有明确职责的模块目录，不得直接新增到 `include/` 或 `src/` 根目录。
- 自有头文件和实现文件应保持 `include/<module>/` 与 `src/<module>/` 的物理目录镜像；引用头文件时从 `include/` 根使用完整模块路径，不新增子目录 include 搜索路径或转发头。
- 新增、删除或移动 C/C++ 文件时，必须同步 `G-code_post-processing_system.vcxproj`、`G-code_post-processing_system.vcxproj.filters` 和 `.ua/.understandignore`；测试工程直接引用该文件时也必须同步。

## 15. Verification

最低验证要求：

- 运行 `git diff --check`。
- 确认 diff 只包含任务范围内文件。
- 只要求 `Release|x64`，不新增 Debug 构建要求。
- 运行与任务相关的无 GUI 测试。
- G-code 相关修改比较全部相关黄金文件。
- 数值、拓扑、截面或加工断面修改运行 `TranslationInvarianceTests`。
- DXF 保存修改验证普通保存、安全导出和重新导入。
- UI 修改执行对应人工交互检查。
- 不声称未执行的测试已经通过。

Release 构建和测试命令以 README “快速开始”和“测试”章节为准。

## 16. Final Report

最终报告应简洁说明：

- 用户可见结果或数据流变化；
- 主要修改位置；
- 实际执行的构建和测试；
- 未执行的验证或剩余风险；
- 如果创建提交，给出提交哈希和范围。

不要用“已完成”代替可验证结果，不要逐文件罗列低价值改动。
