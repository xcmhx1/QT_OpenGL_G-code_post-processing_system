# G-code Post-processing System

本文档是当前项目的事实入口，供使用者、测试人员和开发者共同使用，负责记录：

- 当前支持的功能；
- 当前明确限制；
- 实际生产数据流；
- 模块所有权；
- 构建和测试方法；
- 修改功能时的入口；
- 当前技术债务和后续产品化方向。

当本文档与源码冲突时，以源码和可重复测试结果为准，并应立即修正文档。README 不是开发日志、提交记录汇总、理想架构提案或论文正文。

## 1. 项目简介

本项目是运行于 Windows 的桌面 CAD/CAM 程序，使用 Qt Widgets 构建界面，使用 OpenGL 完成 CAD 场景显示与交互。程序覆盖 DXF/DWG 读取、常用 CAD 绘制与编辑、加工状态管理、加工计划、机床轨迹和 G-code 输出。

当前提供两条加工链：

- `Planar3Axis`：面向平面三轴 G-code 输出。
- `Rotary4Axis`：面向特定方管工件、绕 X 轴旋转并以 A 轴展开的四轴 G-code 导出链。

本项目不是通用三维实体 CAD，不是通用四轴或五轴 CAM，也不提供通用多轴后处理。任何生成结果在实际加工前都必须经过仿真、空跑和人工复核。

## 2. 当前功能

### 2.1 文件与配置

- 读取 DXF 和 DWG；写出格式为 DXF。
- 普通 DXF 保存保留当前文档中的原始实体；安全 DXF 导出使用临时副本和较旧 DXF 版本提高兼容性，不修改活动文档。
- 导入位图并进行阈值、边缘、形态学、轮廓提取和矢量化。
- 管理 G-code 配置文件目录、当前配置和导出目录。
- 支持品牌配置、Lite/Pro 功能边界、机器码和 `license.dat` 授权流程，发布说明见 [COMMERCIAL_RELEASE.md](./COMMERCIAL_RELEASE.md)。

### 2.2 CAD 图元能力

下表描述当前生产代码的能力边界。“通用编辑”指选择、移动、旋转、缩放、复制和删除等操作，不代表存在完整的参数化编辑器。

| 图元 | 导入/显示 | 创建与编辑 | DXF 保存 | 加工 |
| --- | --- | --- | --- | --- |
| `POINT` | 支持 | 支持通用编辑 | 支持 | 不生成加工路径 |
| `LINE` | 支持 | 支持创建、控制点和通用编辑 | 支持 | 支持三轴和四轴 |
| `XLINE` | 支持 | 支持创建、基点/方向控制点和通用编辑 | 支持 | 辅助图元，不加工 |
| `ARC` | 支持 | 支持创建、控制点和通用编辑 | 支持 | 支持三轴和四轴 |
| `CIRCLE` | 支持 | 支持创建、控制点和通用编辑 | 支持 | 支持三轴和四轴 |
| `ELLIPSE` | 支持 | 支持创建、控制点和通用编辑 | 支持 | 通过离散路径参与加工 |
| `POLYLINE` | 支持 | 支持顶点和通用编辑 | 支持；安全模式可能转为兼容实体 | 支持，bulge 保留为精确圆弧段进入核心 |
| `LWPOLYLINE` | 支持 | 支持创建、顶点和通用编辑 | 支持 | 支持，bulge 保留为精确圆弧段进入核心 |
| `SPLINE` | 支持并保留原始控制点、节点和权重 | 支持通用变换、复制和删除；未提供完整样条控制点编辑 | 普通保存保留 SPLINE；安全导出副本可离散 | 通过 NURBS 求值和采样后的 `Path3D` 加工 |

矩形和多边形创建最终生成多段线。多边形支持 3 至 1024 边、内切/外切选项，并保存最近使用的边数。

### 2.3 编辑与交互

- 单选、框选、向右包含选择和向左碰选。
- 端点、中点、圆心/中心、交点、控制点和网格捕捉。
- 光标旁动态输入，正交和极轴状态反馈。
- 控制点编辑、图层、颜色和实体属性。
- 移动、旋转、缩放、镜像、偏移、矩形/环形阵列、修剪/延伸、合并、圆角/直角和删除。
- 批量命令按一次用户操作进入 Undo/Redo。
- 平移、滚轮缩放、轨道观察、标准视角和右上角三维视图方块。
- 浅色、深色和自定义外观；加工箭头、加工序号、加工断面、排除图元和网格显示开关。

### 2.4 加工功能

- 三轴和四轴分别建立 `ProcessPlan`。
- 用户可设置加工启用状态、加工方向和闭合图元起点。
- `NearestNext` 最近距离排序。
- 方管四轴 `LazyRotation` 懒旋转排序。
- 连续链和严格闭环作为连续加工组处理。
- `Break` 加工断面前置约束和 `Waste` 废弃区间排除。
- 方管垂直截面、真实外边界、圆角、内部图元和加工断面识别。
- 四轴安全移动、同组 `CuttingConnection`、A 轴连续解包和闭环过切。
- 文件、图层、颜色和实体类型 G-code 配置块；默认配置主要通过颜色规则区分工艺。

## 3. 明确限制

- 四轴链只适用于绕 X 轴回转、A 轴展开的特定方管/回转类场景，不能描述为通用 3D CAD/CAM、通用四轴或五轴系统。
- 当前仅建立 A 轴旋转链，不提供 B/C 轴、刀具姿态优化或通用机床运动学配置。
- 三轴仅对 XY、ZX、YZ 主平面圆弧输出对应圆弧插补；非主平面圆弧和圆通过离散线段输出。
- 完整圆的原生三轴圆弧输出限于 XY 平面；其他朝向使用统一离散路径。
- `ELLIPSE` 和 `SPLINE` 在加工输出中按采样路径生成线性运动，不输出机床原生椭圆或 NURBS 指令。
- `SPLINE` 使用精确 NURBS 数据时执行自适应采样；精确数据非法而拟合点可用时允许带 Warning 的兼容降级。当前没有完整样条控制点编辑器。
- `POINT`、`XLINE` 等辅助图元不生成加工轨迹。
- DWG 当前用于读取；程序不写出 DWG。DXF 安全导出会在临时副本中转换部分实体以换取兼容性。
- 自动方管截面和加工断面要求真实边界严格闭合、投影可映射且图纸拓扑有效；不会用毫米级容差自动补齐工程间隙。
- 当前后处理是项目配置驱动的 G/M 代码文本输出，没有覆盖或认证所有控制器方言。
- 位图矢量化、复杂修改命令、自动识别、排序和真实机床结果仍需要按实际图纸进行人工功能验收。
- 大文件性能、长时间操作的主线程响应和不同 Windows/显卡环境仍缺少完整量化基线。

## 4. 快速开始

### 4.1 开发环境

| 项目 | 当前工程配置 |
| --- | --- |
| 操作系统 | Windows x64；工程目标为 Windows 10 SDK |
| IDE | Visual Studio 2026 Insiders；当前安装内部版本为 18.8 |
| IDE 可执行文件 | `D:\Program Files\Microsoft Visual Studio\18\Insiders\Common7\IDE\devenv.exe` |
| 编译器 | MSVC `PlatformToolset=v145` |
| C++ | C++17 |
| Qt | Qt 6.9.3，套件标识 `msvc2022_64`，模块为 Core/Gui/Widgets/OpenGL/OpenGLWidgets |
| OpenCV | OpenCV 4.11.0，工程默认 `OpenCVRoot=D:\develop\opencv` |
| 图形 | 支持 OpenGL 的 Windows 图形环境；部署包含 Qt OpenGL 组件和软件 OpenGL 回退库 |
| 构建模式 | 仅维护 `Release|x64` 验证要求 |

如果 OpenCV 不在默认目录，应在 MSBuild 属性或本机工程设置中覆盖 `OpenCVRoot`，不要提交个人路径变更。

Qt 安装目录中的 `msvc2022_64` 是 Qt 官方预编译套件名称，不代表本项目使用 Visual Studio 2022；当前项目 IDE 是 Visual Studio 2026 Insiders，工程使用 v145 工具集。

使用当前 IDE 打开解决方案：

```powershell
& "D:\Program Files\Microsoft Visual Studio\18\Insiders\Common7\IDE\devenv.exe" `
  ".\G-code_post-processing_system.slnx"
```

### 4.2 构建

在仓库根目录执行：

```powershell
& "D:\Program Files\Microsoft Visual Studio\18\Insiders\MSBuild\Current\Bin\MSBuild.exe" `
  ".\G-code_post-processing_system.slnx" `
  /m /p:Configuration=Release /p:Platform=x64
```

输出目录：

```text
x64/Release/
```

Release 后生成事件会复制 OpenCV 和 MSVC 运行库，并执行 Qt 6.9.3 的 `windeployqt --release`。构建时可能出现找不到可选 `dxcompiler.dll`/`dxil.dll` 的部署警告；当前 OpenGL Widgets 主链可以构建，但发布前仍应在目标机器验证图形后端。

## 5. 用户操作流程

### 5.1 三轴流程

```text
导入图纸
→ 检查和编辑图元
→ 设置加工启用状态、方向或起点
→ 生成 Planar3Axis ProcessPlan
→ 查看加工顺序和方向
→ 选择 G-code 配置
→ PlanarNcProgramBuilder 生成 NcProgram
→ GCodePostProcessor 后处理
→ 导出 NC 文件
→ 仿真、空跑和人工复核
```

### 5.2 方管四轴流程

```text
导入图纸
→ 识别方管垂直截面
→ 识别或指定内部图元
→ 识别或指定 Break/Waste 加工断面
→ 选择 NearestNext 或 LazyRotation
→ 生成 Rotary4Axis ProcessPlan
→ 检查方向、顺序、连续组和排除项
→ MachineTrajectoryService 生成 MachineTrajectory
→ NcProgramBuilder 生成 NcProgram
→ GCodePostProcessor 后处理
→ 导出 NC 文件
→ 仿真、空跑和人工复核
```

几何或加工状态变化会使旧计划失效。三轴计划不能用于四轴导出，四轴计划不能用于三轴导出；没有版本一致且模式匹配的 `ProcessPlan` 时必须拒绝导出。当前生产链不存在从 `CadItem` 旧排序字段继续导出的 fallback。

## 6. 当前生产架构

```text
DXF / DWG / CAD 编辑
            ↓
CadDocument + CadItem
            ↓ 文档线程捕获
DocumentGeometrySnapshotBuilder
            ↓
GeometrySourceSnapshot / SourceEntity
            ↓
GeometryCompiler
            ↓
Path3D（批量编译时可形成 GeometrySnapshot）
            ↓
PathTopology
            ↓
TubeSection / InternalGeometry / TubeCutBoundary
            ↓
DocumentProcessState + ProcessPlanningService
            ↓
ProcessPlan
            ↓
┌────────────────────────────────────────────┐
│ Planar3Axis                                │
│ DocumentPlanarNcInputAdapter               │
│ → PlanarNcProgramBuilder → NcProgram       │
├────────────────────────────────────────────┤
│ Rotary4Axis                                │
│ MachineTrajectoryService                   │
│ → MachineTrajectory                        │
│ → NcProgramBuilder → NcProgram             │
└────────────────────────────────────────────┘
            ↓
GCodePostProcessor
            ↓
QString 程序文本
            ↓
GGenerator 文件选择、校验和写入
            ↓
NC 文件
```

生产规划当前在文档线程捕获不可变值对象，并由相应服务同步编排。`GeometrySnapshotCompiler` 已支持独立串行/并行批量编译，但当前规划、轨迹和 NC 生产入口没有整体切换为后台任务。

## 7. 核心数据模型

### 7.1 CadDocument / CadItem

`CadDocument` 拥有原始 `dx_data`、稳定顺序的 `CadItem` 集合、图层和 `EntityId`，并通过 `contentRevision` 表达 CAD 内容版本。`CadItem` 保留原始 DXF 实体指针，负责显示、拾取、控制点和编辑所需状态。

`rawPathPoints3D` 仍存在，但只用于尚未移除的兼容边界和部分旧 UI 分析调用，不是新的几何、规划或 NC 事实来源。`CadItem` 不拥有加工顺序、实际 reverse、连续组、Break/Waste、内部排除或四轴机床轨迹。

### 7.2 身份模型

`cadcam::geometry::EntityId` 是文档和加工链中的稳定业务身份，来源为 `CadItem::m_entityId`，贯穿 `SourceEntity`、`Path3D`、`DocumentProcessState`、`ProcessPlan`、`MachineTrajectory` 和 `NcProgram`。对象地址不得进入这些状态或输出模型。

`RenderEntityKey` 是 Viewer 内部的短生命周期强类型键，只用于 GPU buffer、拾取、临时选择和交互覆盖层。它由 `CadItem*` 经 `CadViewerUtils::toRenderEntityKey()` 创建，不可转换为稳定 `EntityId`，也不可跨对象或文档生命周期持久保存。`DRW_Entity*` 和 `CadItem*` 仅用于对象访问，不承担业务身份。

### 7.3 SourceEntity

`SourceEntity` 是从 DXF/CAD 捕获的精确几何值对象，使用稳定 `EntityId` 和 `SourceGeometryKind` 描述 Point、Line、Arc、Circle、Ellipse、Polyline 和 Spline 等几何。它不保存 `CadItem*`、`DRW_Entity*` 或 GUI 对象，是 CAD 精确事实进入核心计算的边界。

### 7.4 Path3D

`Path3D` 是统一计算路径，坐标为世界坐标 `double`，包含 `EntityId`、来源类型、顶点和 `sourceParameter`。`closed` 独立表达语义闭合，核心闭合路径不重复保存首点。

`Path3D` 是拓扑、规划和轨迹的重要输入，但它是采样后的计算表示，不替代 `SourceEntity` 中的精确圆弧、椭圆或 NURBS 事实。

### 7.5 DocumentProcessState

`DocumentProcessState` 保存用户和分析层的加工输入：

- 加工启用状态；
- 用户方向意图；
- 用户起点；
- `Break`/`Waste` 及断面组编号；
- 内部图元分析排除状态；
- 独立的 `processStateRevision`。

自动计划结果不得写回这些用户意图。

### 7.6 ProcessPlan

`ProcessPlan` 保存一次规划的不可变结果：

- `Planar3Axis` 或 `Rotary4Axis` 模式；
- 文档版本和 process-state 版本；
- `assignments` 中的 `processOrder`、实际 `reverse`、实际 `startParameter` 和连续组编号；
- 连续链、闭环和加工断面组；
- 排除项；
- 加工断面前置约束；
- `NearestNext` 或 `LazyRotation` 策略。

### 7.7 ProcessPresentationSnapshot

`ProcessPresentationSnapshot` 由 `ProcessPlan` 构建，只向 Viewer/显示层提供加工序号、方向、起点、连续组和排除状态。它不参与规划、机床轨迹或 NC 生成。

### 7.8 MachineTrajectory

`MachineTrajectory` 保存四轴机床运动事实：

- 世界机床姿态和 XYZ/A；
- `Rapid`、`Cutting`、`CuttingConnection`、`Overcut`；
- 每个实体的轨迹、顺序和组；
- 方管中心、旋转轴中心、安全 Z 和碰撞半径上下文；
- 文档版本和 process-state 版本。

它不保存 `CadItem`、DRW、GUI 对象或完整 G-code 字符串。

### 7.9 NcProgram

`NcProgram` 是与文本方言分离的 NC 语义模型，保存程序模式、注释、实体元数据块，以及 Rapid/Linear/Circular 运动。轴字为可选的 X/Y/Z/A/I/J/K/R，并保留 `EntityId`、`processOrder` 和 group 元数据。

它不保存 `GProfile` 指针或最终 `QString` 程序。

## 8. 模块所有权

| 模块 | 所有数据/契约 | 负责 | 不负责 |
| --- | --- | --- | --- |
| `core/geometry` | `SourceEntity`、`Path3D`、LocalFrame | 精确几何、NURBS、采样、局部坐标 | 文档、UI、排序 |
| `core/topology` | `TopologyInput`、`TopologyLoopResult` | 连通、交点、严格闭环、环提取 | 方管工艺、G-code |
| `core/machining` | `TubeSectionModel`、`TubeCutAnalysis` | 方管截面、内部图元、加工断面判定 | 排序、文本输出 |
| `core/planning` | `ProcessPlan` | 顺序、方向、分组、排除和前置约束 | 机床移动 |
| `core/machine` | `MachineTrajectory` | A 轴运动学、安全移动、连续连接和过切 | G-code 文本格式 |
| `core/nc` | `NcProgram` | 三轴/四轴 NC 语义 | `GProfile` 和文件写入 |
| `application` | snapshot、process state、services | 文档捕获、版本校验和业务编排 | 核心几何算法 |
| `infrastructure` | DXF adapter、G-code postprocessor | 外部格式适配和文本方言 | 加工工艺决策 |
| `compatibility` | legacy adapters | 尚未移除的 CadItem/DRW 边界兼容 | 新功能所有权 |
| UI/CAD | `CadItem`、Viewer、Controller、窗口 | 编辑、显示和用户操作 | 计划、机床轨迹和 NC 事实 |

`RotaryTubeGeometryAnalyzer` 和 `RotaryCutBoundaryAnalyzer` 当前是 Qt/CadItem 兼容入口；最终截面和加工断面算法分别归 `TubeSectionAnalyzer` 与 `TubeCutBoundaryClassifier` 所有。

### 8.1 Compatibility 状态

本阶段审计的 11 个重点兼容模块按生产调用和测试用途分类如下：

| 分类 | 数量 | 当前状态 |
| --- | ---: | --- |
| `DeleteNow` | 0 | 未发现同时满足无生产调用、无有效测试用途和无隐式注册的可直接删除项 |
| `TestOnlyParity` | 2 | 仅由无 GUI 测试使用，已从主程序工程移除并保留在测试工程 |
| `ProductionAdapter` | 5 | 正式入口仍需要其完成 CadItem/DRW 到现有核心值对象的边界转换 |
| `Blocked` | 3 | 仍涉及生产缓存、兼容采样或明确 fallback，当前删除会改变行为 |
| `ThirdPartyBoundary` | 1 | `infrastructure/dxf/legacy` 中的 `dx_iface` 承担 libdxfrw 格式边界，不属于待删除的旧生产算法 |

新的核心模块不得依赖 `compatibility`。兼容代码删除前必须同时确认生产 callers、测试 callers、工程项、Qt 隐式使用以及 Release 构建和行为测试，不能依据目录名或“零调用者”单独判断。

## 9. 数值稳定性

当前实现遵循以下原则：

- 世界坐标只用于保存、显示、跨模块值对象和最终输出。
- 几何解算尽量在确定性的局部坐标系中执行。
- Geometry Core 使用 `double`，不让 `QVector3D` 或 float 显示缓存进入核心计算。
- `LocalFrame2d`、`LocalFrame3d` 和 `PlaneFrame3d` 负责局部/世界坐标转换。
- 包围盒中心使用稳定形式计算，长度、面积、均值和协方差等累计量使用补偿求和。
- ARC、bulge、NURBS 齐次求值、拓扑平面拟合、相交、节点、截面和加工断面均在局部坐标中处理。
- 最终 `Path3D`、`MachineTrajectory` 和 `NcProgram` 恢复为世界坐标。
- 远离原点的图元不通过放宽闭环或工艺容差解决。
- 平移不变性回归覆盖到约 `1e9` 数量级世界坐标。

禁止以提高输出小数位代替数值算法修复，禁止根据世界坐标绝对值扩大闭环容差，禁止重新把显示缓存作为加工事实。

## 10. G-code 生产链

### 10.1 三轴

```text
ProcessPlan
→ NcProgramService
→ DocumentPlanarNcInputAdapter
→ PlanarNcProgramBuilder
→ NcProgram
→ GCodePostProcessor
→ QString
→ GGenerator 写入文件
```

### 10.2 四轴

```text
ProcessPlan
→ NcProgramService
→ MachineTrajectoryService
→ MachineTrajectory
→ NcProgramBuilder
→ NcProgram
→ GCodePostProcessor
→ QString
→ GGenerator 写入文件
```

`GGenerator` 负责导出模式和版本校验、调用 `NcProgramService`、选择路径以及 UTF-8 文件写入。它不再按具体 `CadItem` 类型生成几何，不计算 A 轴、安全高度、连续连接或过切，也不负责 M05/M03 文本优化。

`GCodePostProcessor` 负责文件/图层/颜色/类型代码块、运动轴字、精度、圆弧平面代码、相邻纯 M05/M03 优化和 CRLF 文本。三轴和四轴都必须使用版本一致、模式匹配的 `ProcessPlan`；失败时不得返回可写入的部分 NC 文本。

## 11. 三轴几何映射

- `LINE` 输出 Rapid 起点和 Linear 终点。
- XY、ZX、YZ 主平面 `ARC` 分别使用 G17/G18/G19 和 G02/G03；非主平面圆弧离散为线性路径。
- 完整 XY `CIRCLE` 使用圆弧运动；其他平面或任意空间朝向的圆离散为线性路径。
- 2D `POLYLINE`/`LWPOLYLINE` bulge 在 DXF 适配层转换为精确 `ArcGeometry`，当前三轴 XY 映射可输出 G02/G03；3D POLYLINE 按直线段处理。
- `ELLIPSE` 和 `SPLINE` 由 `GeometryCompiler` 离散为 `Path3D` 后输出 Linear 运动。
- 完整圆和完整椭圆的默认加工起点为图元局部北极 `M_PI_2`；规划选择正反方向但不改变该默认起点。
- 部分圆弧、部分椭圆和开放路径保留原始首尾语义；闭合多段线的 `startParameter` 表示原始顶点索引。
- `reverse` 和 `startParameter` 来自 `ProcessPlan`，不会通过修改原始 DXF 几何实现。

## 12. 方管四轴工艺

- 工件轴线按 X 方向建模，A 轴绕 X 轴旋转。
- `TubeSectionModel` 保存真实外边界、Y/Z 中心、Y 长、Z 宽、圆角中心和半径；真实边界不是理想凸包替代物。
- 方管中心用于表面法向、圆角刀头方向和碰撞计算；`rotaryAxisY/rotaryAxisZ` 用于实际旋转坐标变换，两者语义不同。
- “去除内部线条”分两遍执行：第一遍不依赖方管截面，使用规范方向 `Path3D` 和 `PathTopology` 按连通分量保留严格闭合的最大真实外环，排除同一组件中未参与外环的内部路径；第二遍仅在方管截面有效时，逐段识别进入真实截面边界内部的危险路径。
- LINE、ARC、CIRCLE、ELLIPSE、POLYLINE、LWPOLYLINE 和 SPLINE 均先经 `DxfGeometryAdapter` 与 `GeometryCompiler` 形成双精度 `Path3D`，内部线判断不读取显示缓存或按 DXF 类型复制算法。“日、目、田”结构保留外轮廓并排除内部横竖线；开放组件和无严格外环组件保持可加工。
- 内部线排除只写入 `DocumentProcessState`，不删除 CAD/DXF 图元；Break/Waste 加工断面不被自动内部线状态覆盖。拓扑内部和方管实体内部结果按 EntityId 去重，并在一次批处理中更新，重复相同识别不会产生无意义的 process-state revision。
- “断N”只标识同一加工断面的成员，不表示空间顺序或加工优先级；多个断面依据真实几何关系从 -X 到 +X 排列。
- `Break` 是完整工艺屏障：左侧加工组全部完成后立即加工该断面，右侧加工组必须等待断面完成；区域内部才应用 `NearestNext` 或 `LazyRotation`。
- `Waste` 与 `Break` 共用真实断面空间顺序，但只负责排除本体及相邻废弃区间；交叉、重合或无法严格区分左右的断面会拒绝规划。
- 闭环必须由 `Path3D.closed` 或所有物理连接点在 `numericalJoinEpsilon` 内真实重合成立；工程间隙不接受“近似闭合”。
- 加工断面按真实闭环的 YZ 投影映射到方管外边界，生成 `SurfaceSpan` 和 seam 周向统计。
- 有向周向行程得到整数 winding；`winding=0` 表示仍保留左右材料桥，不能标记为有效切断断面。
- 首个加工组从当前空间区域内满足硬前置约束的候选中，按到四轴初始位置 `(0, 0, 500)` 的真实入口移动距离选择；`LazyRotation` 从第二个加工组开始考虑 A 轴旋转代价。
- 同一连续组可生成 `CuttingConnection`，组间使用安全移动；闭合组始终精确回到组起点，`overcutDistance=0` 只关闭额外过切，默认过切距离为 `2.0 mm`。
- A 轴角度连续解包，避免相邻点产生无意义的正负 360 度跳变。

## 13. 诊断和结果

核心和应用服务使用：

- `OperationStatus`：Success、PartialSuccess、Cancelled、InvalidInput、NotSupported、Conflict、Failed、InternalError。
- `OperationResult<T>`：状态、可选值和结构化诊断。
- `OperationReport`：不返回业务值的结果。
- `Diagnostic`、`DiagnosticCode`、`DiagnosticSeverity`：稳定错误代码、严重级别、用户消息、技术细节和上下文。
- `OperationContext` 和 `correlationId`：关联一次完整操作的数据流。

Result 是控制和数据契约，Diagnostic 是结构化失败或警告。`PartialSuccess` 表示结果可用但包含明确降级或局部失败；`Conflict` 用于文档版本、process-state 版本或计划模式不一致。

`MessageCenter` 将诊断分发到调试和 UI sink。核心模块不得弹出 `QMessageBox`，业务代码不得解析日志文本决定流程。

## 14. 版本和状态失效

系统使用两个独立版本：

- `CadDocument::contentRevision()`：导入、清空、新增、删除、复制、Undo/Redo、移动、旋转、缩放、镜像、图层/颜色和原始几何修改时推进；批量修改只推进一次。
- `DocumentProcessState::revision()`：加工启用、用户方向、用户起点、Break/Waste、断面组和内部图元分析状态变化时推进。

选择、高亮、加工序号显示、方向箭头、网格、主题、视角和 `ProcessPresentationSnapshot` 不应推进这两个版本。自动计划结果不写回用户输入，因此也不应自行推进 process-state revision。

应用 `ProcessPlan`、生成 `MachineTrajectory` 或 `NcProgram` 前，必须同时比较 `contentRevision`、`processStateRevision` 和计划模式。不一致时返回 `Conflict` 并拒绝导出，而不是清空 `CadItem` 字段或自动补齐旧排序后继续。

## 15. 并发和线程边界

- `DocumentGeometrySnapshotBuilder::capture()` 必须在 `CadDocument` 所在线程读取 `CadDocument`、`CadItem` 和 DRW 实体，并生成不持有这些指针的 `GeometrySourceSnapshot`。
- `GeometrySnapshotCompiler` 支持 Serial 和 Parallel 两种批量编译模式；worker 只读取 `SourceEntity` 值对象，不访问 QObject、DRW 或 GUI。
- 并行结果按 `sourceIndex` 确定性合并，Diagnostic 保持图元内部顺序；相同输入重复运行应得到相同结果。
- `CancellationToken` 在图元任务边界检查取消，取消后等待已运行任务结束并保留已完成 entry；进度通过值对象回调返回。
- 快照应用前使用 `matchesRevision()` 检查版本，过期结果不得覆盖当前文档。
- 当前生产规划、拓扑、轨迹和 NC 编排仍主要是同步调用；`GeometrySnapshotCompiler` 的并行能力没有把整条生产链自动变成后台任务，长操作仍可能阻塞主线程。

## 16. 仓库目录

```text
include/desktop/          主窗口、应用入口、品牌和授权接口
src/desktop/              主窗口动作、应用入口、品牌和授权实现
include/ui/               对话框和复用控件接口
src/ui/                   对话框和复用控件实现
include/cad/              CAD 文档、图元、编辑、视图及渲染接口
src/cad/                  CAD 文档、图元、编辑、视图及渲染实现
include/core/             纯计算数据模型与算法公共接口
src/core/                 几何、拓扑、加工、规划、机床和 NC 核心实现
include/application/      文档快照、加工状态、消息、规划和导出接口
src/application/          版本校验、捕获、规划、加工分析和导出编排实现
include/infrastructure/   配置、图像和 DXF/G-code 外部格式适配接口
src/infrastructure/       配置、图像、DXF 适配和后处理文本实现
include/compatibility/    CadItem/DRW 旧边界兼容接口
src/compatibility/        尚未移除的兼容适配实现
include/platform/         Visual Studio 预编译头等平台接口
src/platform/             Visual Studio 预编译头实现
include/libdxfrw/         第三方 libdxfrw 头文件
src/libdxfrw/             第三方 libdxfrw/libdwgr 源码
tests/                    无 GUI characterization tests 与黄金数据
technical_file/           G/M 代码等工程参考资料
tools/                    授权生成工具
docxs/                    论文、实习和答辩材料，不参与程序构建
```

`include/` 与 `src/` 中的自有模块保持镜像目录，不在两者根目录直接放置 C/C++ 文件。工程只把 `include/` 作为自有头文件根目录，源码使用完整模块路径，例如 `#include "cad/items/CadItem.h"` 和 `#include "platform/pch.h"`，不为子目录增加额外 include 搜索路径。

`src/infrastructure/dxf/legacy/dx_iface.cpp`、`include/infrastructure/dxf/legacy/dx_iface.h` 及 libdxfrw 默认视为第三方边界。除非问题明确位于该层，否则不要修改；修改后必须验证导入、普通保存、安全导出和重新导入闭环。

## 17. 开发入口表

| 修改内容 | 首先查看 |
| --- | --- |
| DXF 适配 | `include/infrastructure/dxf/`、`src/infrastructure/dxf/` |
| 精确几何和采样 | `include/core/geometry/`、`src/core/geometry/` |
| 闭环和连通 | `include/core/topology/`、`src/core/topology/` |
| 方管截面 | `TubeSectionAnalyzer`、`core/machining/TubeSection` |
| 加工断面 | `TubeCutBoundaryClassifier`、`core/machining/TubeCutBoundary` |
| 三轴排序 | `PlanarProcessPlanBuilder` |
| 四轴排序 | `ProcessPlanBuilder` |
| 用户加工状态 | `DocumentProcessState` |
| 计划显示 | `ProcessPresentationSnapshot` |
| 四轴机床轨迹 | `core/machine`、`MachineTrajectoryService` |
| 三轴 NC | `DocumentPlanarNcInputAdapter`、`PlanarNcProgramBuilder` |
| 四轴 NC | `NcProgramBuilder`、`NcProgramService` |
| G-code 文本 | `GCodePostProcessor` |
| 文件导出 | `include/application/export/`、`src/application/export/`、`src/desktop/Gcode_postprocessing_system_GCodeActions.cpp` |
| CAD 编辑 | `include/cad/editing/`、`src/cad/editing/`、`include/cad/view/interaction/`、`src/cad/view/interaction/` |
| OpenGL 显示 | `include/cad/view/`、`src/cad/view/`，以及其 `rendering/`、`scene/`、`transform/` 子目录 |
| 诊断 | `include/core/diagnostics/`、`src/core/diagnostics/` |
| 配置 | `include/infrastructure/config/`、`src/infrastructure/config/`、`include/ui/dialogs/GProfileDialog.h` |
| 授权和部署 | `COMMERCIAL_RELEASE.md` |

## 18. 新功能开发规则

1. 先找到数据事实所有者，再沿真实数据流修改。
2. 优先扩展已有职责模块，不在 UI、Viewer 或主窗口复制核心算法。
3. 单一调用方的辅助函数优先放在 `.cpp` 私有区。
4. 不为一个静态函数新增公共类。
5. 不新增只做转发且没有不变量的 Service、Facade、Manager 或 Utils。
6. 只有存在独立契约、独立不变量和两个以上生产调用方时，才考虑公共模块。
7. UI 不拥有几何、规划、机床轨迹或 NC 事实。
8. Core 不依赖 `CadItem`、DRW 或 `QWidget`。
9. 不通过放宽容差掩盖几何或数值错误。
10. 新生产实现不保留旧算法运行时 fallback；明确的 compatibility 边界必须返回诊断。
11. 行为修改必须增加最小回归测试。
12. G-code 行为变化必须说明黄金文件变化原因，不能只更新预期结果让测试通过。

具体执行约束见 [AGENTS.md](./AGENTS.md)。

## 19. 测试

当前 `tests/GCodeCharacterizationTests.vcxproj` 构建单一无 GUI 测试程序：

```text
x64/Release/tests/GCodeCharacterizationTests.exe
```

测试入口覆盖：

- Geometry Core、Path3D 和多段线 bulge；
- SPLINE NURBS、拟合点降级、生产接入和保存/重载；
- GeometrySourceSnapshot、串行/并行确定性、取消和过期判断；
- PathTopology、严格闭环和 topology golden；
- TubeSection、无截面“日/目/田”拓扑内部线、曲线/弯折路径、方管实体内部路径和 TubeCutBoundary；
- Planar/Rotary ProcessPlan；
- MachineTrajectory 和 A 轴运动学；
- NcProgram 和 GCodePostProcessor；
- 三轴、四轴、连续组、过切和 SPLINE G-code golden；
- `TranslationInvarianceTests`，覆盖到约 `1e9` 世界坐标平移。

运行：

```powershell
$env:PATH="D:\Qt\6.9.3\msvc2022_64\bin;$env:PATH"
& ".\x64\Release\tests\GCodeCharacterizationTests.exe"
```

当前没有形成覆盖典型客户 DXF 的完整自动化端到端 fixture 集；`testdxf/` 中的样例主要用于人工和专项排查，不能宣称为完整功能测试框架。

## 20. 验证清单

### 20.1 文档和构建

- `git diff --check`。
- README 中引用的路径真实存在。
- README 中引用的主要类型可在仓库中搜索。
- 构建命令与 `.slnx` 和 `.vcxproj` 一致。
- 完成 `Release|x64` 构建。

### 20.2 核心回归

- 全部无 GUI tests。
- `TranslationInvarianceTests`。
- 三轴和四轴 G-code golden。
- SPLINE golden。
- topology golden。
- 若修改数值、闭环、截面、规划或轨迹，运行对应专项测试。

### 20.3 人工功能验收

- DXF/DWG 导入、DXF 保存和重新导入。
- 方管截面、内部线条和多个加工断面。
- 三轴计划、四轴计划、方向、连续组和排除状态显示。
- G-code 配置、导出、仿真和空跑。
- 绘制、修改、批量 Undo/Redo 和控制点。
- UI 缩放、主题、Dock、动态输入和视图导航。
- 大文件加载、渲染、命中测试和长操作响应。

## 21. 当前技术债务

- `compatibility/legacy` 及 `rawPathPoints3D` 仍服务少量旧 CadItem/Qt 边界，需要按调用点逐步收敛，但不能在无回归保护时直接删除。
- 典型客户 DXF 的自动化端到端 fixture 数量不足，当前测试以值对象、构造夹具和黄金输出为主。
- UI、复杂 CAD 修改、自动识别和真实机床工作流仍需要持续的用户系统功能验收。
- 大文件加载、几何编译、拓扑、规划、渲染和拾取尚未建立完整性能基线。
- 部分长时间生产操作仍同步执行，可能阻塞主线程。
- libdxfrw 构建警告和 `dxcompiler`/`dxil` 可选部署提示尚未完全消除。
- README 必须随生产边界变化同步维护，避免旧架构描述再次进入事实入口。

## 22. 后续方向

1. 建立典型 DXF 端到端 fixture。
2. 完成用户系统功能验收并记录可重复步骤。
3. 优先修复可复现 Bug，不启动无明确收益的大规模重构。
4. 优化命令反馈、错误诊断和实际加工工作流。
5. 建立大文件和长路径性能基线。
6. 减少主线程卡顿，按明确边界引入后台任务。
7. 优化大文件渲染和命中测试。
8. 完善 Release 打包、依赖、授权和目标机器检查。
