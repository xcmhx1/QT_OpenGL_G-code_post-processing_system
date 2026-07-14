# G-code Post Processing System

面向 Windows 的轻量 CAD/CAM 桌面程序，主要用于 DXF/DWG 图纸查看与编辑、加工顺序组织，以及二维和特定方管场景的 G 代码导出。

本文档同时承担以下职责：

- 为使用者说明当前可用功能和明确限制。
- 为开发者提供模块入口、数据流和扩展步骤。
- 为长期维护提供构建、验证和提交检查清单。

项目行为发生变化时，应同步更新本文档。阶段性开发记录不应继续堆叠在“功能列表”中，而应更新对应的数据流、模块职责或限制说明。

## 1. 项目边界

### 1.1 当前定位

项目当前包含三条主要能力链：

1. CAD 链：导入 DXF/DWG 或位图，构建内部图元，完成显示、选择、绘制和修改。
2. 二维后处理链：排序二维加工图元，并按配置输出 G 代码。
3. 方管四轴链：针对绕 `X` 轴回转、`A` 轴展开的特定方管/回转类工件生成四轴 G 代码。

`Mode3D` 只能描述为“绕 X 轴回转、A 轴展开的四轴 G 代码导出链”。它不是通用 3D CAD/CAM、通用四轴后处理器或五轴系统。

### 1.2 主要已实现能力

- 导入 `.dxf`、`.dwg`、`.bmp`、`.png`、`.jpg`、`.jpeg`。
- 显示和编辑点、直线、构造线、圆、圆弧、椭圆、多段线、轻量多段线。
- 将导入的样条曲线离散转换为多段线，复用多段线显示和加工链。
- 使用闭合多段线创建矩形和多边形。
- 支持绘制、选择、对象捕捉、控制点编辑、Undo/Redo 和常用修改命令。
- 支持二维排序、智能排序、加工方向和加工顺序控制。
- 支持方管垂直截面、加工断面和内部线条的识别与人工修正。
- 支持二维 G 代码和特定方管场景的四轴 G 代码输出。
- 支持文件级、图层、颜色和图元类型 G 代码规则配置。
- 支持浅色、深色和自定义外观。
- 支持应用标题/图标定制、Lite/Pro 本机授权和可部署目录生成。

### 1.3 明确不支持或仍有限制的内容

- 不支持通用三维实体建模、曲面建模或装配。
- 四轴链只适用于当前方管/回转工件模型，复杂机床运动学需要另行验证。
- `Point`、`Xline` 等辅助图元不生成加工轨迹。
- 椭圆加工使用离散路径，不依赖专用椭圆插补指令。
- 修剪、延伸、合并、圆角和倒角属于基础实现，不等价于完整 AutoCAD 几何内核。
- 自动截面、加工断面和内部线条识别依赖图纸拓扑质量与容差；实际生产前必须核对识别结果和 NC 文件。
- 当前仓库没有覆盖全部几何边界场景的自动化回归测试，复杂图纸仍需人工验证。

## 2. 快速开始

### 2.1 环境要求

- Windows 10/11 x64。
- Visual Studio 18 Insiders，对应项目工具集 `v145`。
- Qt `6.9.3_msvc2022_64`，包含 Core、Gui、Widgets、OpenGL、OpenGLWidgets。
- OpenCV 4.11，工程默认使用 `opencv_world4110`。
- 支持 OpenGL 4.5 Core Profile 的显卡和驱动。

工程文件中的 `OpenCVRoot` 当前默认为：

```text
D:\develop\opencv
```

如果本机路径不同，应通过项目属性或工程宏调整，不要在多个源文件中写入依赖路径。

### 2.2 Release 构建

在仓库根目录执行：

```powershell
& 'D:\Program Files\Microsoft Visual Studio\18\Insiders\MSBuild\Current\Bin\MSBuild.exe' `
  .\G-code_post-processing_system.vcxproj `
  /m /p:Configuration=Release /p:Platform=x64
```

正常输出目录为：

```text
x64\Release\
```

Release 后处理会调用 `windeployqt`，并复制 OpenCV 与 MSVC 运行库。若程序提示缺少 `Qt6Widgets.dll`、`Qt6Gui.dll` 等文件，应检查构建后处理是否执行成功，而不是只复制单独的 EXE。

### 2.3 最小启动检查

1. 启动 `G-code_post-processing_system.exe`。
2. 确认主窗口、顶部页签和右侧“加工设置”Dock 正常显示。
3. 导入一个简单 DXF，执行缩放适配并选择图元。
4. 关闭并重新启动，确认 Dock 位置、可见状态和主题被恢复。

## 3. 仓库结构

```text
.
├─ include/                         C++ 头文件
│  ├─ CadItem.h                     图元基类与加工状态
│  ├─ CadDocument.h                 文档和实体所有权
│  ├─ CadController.h               输入状态与命令控制
│  ├─ CadEditer.h                   绘制、修改和命令历史
│  ├─ CadViewer.h                   OpenGL 视图与交互信号
│  ├─ GGenerator.h                  G 代码生成器
│  ├─ GProfile.h                    后处理配置模型
│  ├─ RotaryTubeGeometryAnalyzer.h  方管截面和内部线条分析
│  ├─ RotaryCutBoundaryAnalyzer.h   加工断面周向校验
│  ├─ CadToolPanelWidget.h          顶部命令区
│  └─ MachiningSettingsWidget.h     右侧加工设置与状态面板
├─ src/                             C++ 实现
│  ├─ Gcode_postprocessing_system*.cpp 主窗口及文件/编辑/排序/G代码动作
│  ├─ CadController*.cpp            鼠标、键盘和动态输入
│  ├─ CadEditer*.cpp                绘制、修改和几何构造
│  ├─ CadViewer*.cpp                渲染、拾取、覆盖层和视图导航
│  ├─ Cad*Item.cpp                  各图元实现
│  ├─ GGenerator.cpp                二维/四轴输出组织
│  ├─ GProfile.cpp                  配置读写与默认值
│  └─ libdxfrw/                     第三方 DXF/DWG 库源码
├─ technical_file/G-M_Code.md       G/M 代码参考
├─ docxs/                           论文、实习和答辩材料，不参与程序构建
├─ tools/                           许可证生成工具
├─ Gcode_postprocessing_system.ui   主窗口 Qt Designer 文件
├─ Gcode_postprocessing_system.qrc  Qt 资源
├─ G-code_post-processing_system.vcxproj
├─ COMMERCIAL_RELEASE.md            商业构建、打包和授权流程
├─ AGENTS.md                        代码协作约束
└─ README.md                        项目维护基准
```

`src/libdxfrw/`、`include/libdxfrw/` 和 `src/dx_iface.cpp` 来源于第三方库。除非问题明确位于库适配层，否则不要直接修改；修改前应验证 DXF 导入、导出和重新导入闭环。

## 4. 总体架构

### 4.1 分层职责

```text
QMainWindow / CadToolPanelWidget / MachiningSettingsWidget
                         │ 用户动作与状态展示
                         ▼
Gcode_postprocessing_system
                         │ 业务编排、设置持久化、文件动作
              ┌──────────┼───────────┐
              ▼          ▼           ▼
        CadController  CadEditer  几何分析器
              │          │           │
              └──────┬───┴───────────┘
                     ▼
                CadDocument
                     │
                     ▼
             CadItem 派生图元
                │         │
                ▼         ▼
             CadViewer  GGenerator + GProfile
```

依赖方向原则：

- UI 控件只发出用户意图或显示状态，不直接修改 `CadDocument`。
- 主窗口负责连接信号、读取设置和编排业务函数。
- `CadController` 管理输入状态机，不负责文件和后处理配置。
- `CadEditer` 负责几何修改和 Undo/Redo 提交。
- `CadViewer` 负责显示、拾取和交互反馈，不承载加工排序业务。
- 几何分析器只接收图元和容差并返回结果，不弹出界面对话框。
- `GGenerator` 消费文档、排序状态和 `GProfile`，不修改原始 CAD 几何。

### 4.2 主窗口拆分

`Gcode_postprocessing_system` 按业务拆分为：

- `Gcode_postprocessing_system.cpp`：初始化、菜单、设置、顶部面板、Dock 和状态同步。
- `Gcode_postprocessing_system_FileActions.cpp`：文件导入、DXF 保存和导入后处理。
- `Gcode_postprocessing_system_EditActions.cpp`：主窗口编辑动作转发。
- `Gcode_postprocessing_system_SortActions.cpp`：排序、方管截面、加工断面、内部线条和排除状态。
- `Gcode_postprocessing_system_GCodeActions.cpp`：G 代码生成前检查、路径选择和文件输出。

新增主窗口功能时，应放入对应职责文件，避免重新把所有实现堆回主文件。

## 5. 核心数据模型

### 5.1 CadDocument

`CadDocument` 持有 `std::unique_ptr<CadItem>` 实体集合、图层信息和场景变更信号。任何会改变显示或加工状态的操作，都应确保最终触发必要的界面刷新。

### 5.2 CadItem

`CadItem` 及其派生类承载：

- 原始 DXF 实体和图元类型。
- 显示、拾取和控制点所需几何。
- `rawPathPoints3D` 和 `controlPoints4Axis` 四轴路径缓存。

图元几何变化后，应同步失效或重建相关缓存，不能只修改屏幕预览数据。

用户加工输入独立存放在 `DocumentProcessState` 中，自动排序结果由 `ProcessPlan` 保存，画布显示则读取由计划生成的 `ProcessPresentationSnapshot`。加工状态不写回 `CadItem`，避免自动排序结果污染用户设置。

### 5.3 方管截面模型

`RotaryTubeSectionModel` 保存真实外轮廓图元、YZ 边界、长宽、圆角信息和自动搜索诊断。最终模型必须引用实际 `outerBoundaryItems`，不能构造脱离文档图元的虚拟截面。

手动识别使用：

```cpp
RotaryTubeGeometryAnalyzer::buildSectionModel(selectedItems, sceneItems, tolerance);
```

导入后自动识别使用：

```cpp
RotaryTubeGeometryAnalyzer::findBestSectionModel(sceneItems, tolerance);
```

自动入口一次构建全部路径，按 X 位置、连接关系、圆角可靠性和多截面尺寸一致性选择候选；它不依赖当前选择集。

## 6. 关键业务流程

### 6.1 DXF/DWG 导入

入口：`Gcode_postprocessing_system::importDxfFile()`。

固定数据流：

```text
读取文件
→ 自动去重（可选）
→ 自动识别方管垂直截面（可选）
→ 自动识别全部加工断面（可选）
→ 自动清理内部线条（可选）
→ 刷新加工排除状态
→ 同步顶部面板和右侧加工设置
```

约束：

- 自动加工断面和自动内部线条依赖有效方管截面。
- 加工断面识别必须早于内部线条清理。
- 自动失败只写命令栏和状态栏，不阻塞文件导入。
- “没有重复图元”是正常去重结果。
- 导入新文件时必须清空旧截面模型和选择集。
- 手动和自动处理应复用相同业务函数，不能维护两套规则。

### 6.2 样条曲线导入

DXF `SPLINE` 不保留为独立内部图元，而是在导入阶段通过 `CadSplineConverter` 转换为多段线。转换必须保留起点和终点；采样精度应与实际加工精度匹配，不能单纯追求高密度点列。

### 6.3 位图导入

`CadBitmapImportDialog` 管理参数和预览，`CadBitmapVectorizer` 负责预处理、轮廓提取和规则图元拟合。当前支持阈值、自适应阈值、Canny、形态学操作、轮廓层级、规则图元优先和折线输出。

位图导入完成后生成普通 `CadItem`，后续显示、编辑和加工流程不应依赖位图对话框。

### 6.4 绘制与修改

```text
顶部按钮/动态命令/快捷键
→ CadViewer 输入事件
→ CadController 状态机
→ CadEditer 构造或修改实体
→ 批量命令提交 Undo/Redo
→ CadDocument 场景变化
→ Viewer 与状态面板刷新
```

修改类命令应保持一次用户操作对应一次 Undo/Redo 记录。实时预览使用 transient 数据，确认前不得污染文档实体。

### 6.5 方管截面、加工断面和内部线条

- 方管截面：在 YZ 投影中识别方管垂直截面尺寸和圆角。
- 加工断面：连续或近似连续、通过方管周向分离校验的真实轮廓；标记为 `RotaryEndCutRole::Break`。
- 废弃面：保留现有 `RotaryEndCutRole::Waste` 语义，用于加工排除区间。
- 内部线条：拓扑外轮廓内部或进入方管内部的无效加工图元；已标记加工断面的图元不得再次被内部线条清理排除。

自动加工断面识别复用 `recognizeAllRotaryEndCuts()` 和 `RotaryCutBoundaryAnalyzer::analyze()`。单个候选失败不得中止全部搜索，已有加工断面或废弃面标记的图元不得重复分配。

画布右键菜单提供截面识别、加工断面指定/恢复、全部断面识别/恢复、内部线条指定/恢复及清空状态。菜单只由 `CadViewer` 发出请求，业务仍由主窗口执行。

### 6.6 排序

排序统一从 `DocumentProcessState` 读取用户方向、起点和加工约束，并将最终顺序、实际方向及连续组写入 `ProcessPlan`。

- “排序（保留方向）”调整顺序，不主动覆盖用户方向。
- “智能排序”可联合考虑顺序、方向、连续性和闭合路径起刀点。
- 四轴方管排序会考虑加工断面分段、连续路径免抬刀、面组和 A 轴旋转代价。
- 排序属于启发式算法，不保证数学意义上的全局最优。

任何会改变图元集合、加工断面、内部线条或方向的操作，都应使失效的加工顺序重新计算。

### 6.7 G 代码生成

```text
用户选择导出
→ 解析 3轴/4轴模式
→ 检查或自动补齐排序
→ GGenerator 读取有序图元
→ 应用 GProfile 规则
→ 写入 CRLF 文本文件
→ 记录成功导出目录
```

当前加工几何支持 `Line`、`Arc`、`Circle`、`Ellipse`、`Polyline`、`LWPolyline`。辅助图元不会产生加工轨迹。

规则包裹顺序为：

```text
文件头
  图层头
    颜色头
      图元类型头
        图元加工路径
      图元类型尾
    颜色尾
  图层尾
文件尾
```

默认配置更偏向按颜色区分工艺。修改规则顺序前必须同时检查三轴和四轴输出。

## 7. 四轴方管链

### 7.1 坐标和职责

- 工件中心线按 X 轴建模。
- A 轴表示绕 X 轴旋转。
- 各 `CadItem` 派生类负责从原始几何生成 `rawPathPoints3D` 和 `controlPoints4Axis`。
- `GGenerator` 负责跨图元连续性、安全高度、规则包裹和 NC 文本输出。

### 7.2 刀头方向

- 路径映射在方管直边时，按对应表面法向加工。
- 路径映射在圆角区域时，刀头方向指向对应四分之一圆角圆心。
- A 轴方向变化应沿真实加工路径渐变，不能在交界点原地快速调头。

### 7.3 安全约束

- 统一安全高度基于全图最大离 X 轴距离和配置的额外距离。
- 连续路径允许免抬刀，但必须同时满足空间连接、方向连续和加工排除约束。
- 右侧加工设置中的截面、加工断面和内部线条状态应在导出前核对。
- 实际机床运行前必须进行空跑或仿真验证。

## 8. 用户界面

### 8.1 顶部页签

- `默认`：绘图、修改、图层和特性。
- `机加工`：只放高频命令，不放持久设置和状态。
- `显示`：加工方向箭头、加工序号、加工断面、排除图元和背景网格等显示选项。

机加工命令区：

- 导入导出：文件导入、G 代码导出。
- 排序：排序（保留方向）、智能排序。
- 几何处理：去重、识别方管截面、识别加工断面、清理内部线条。
- 配置：当前配置、配置选择、G 代码模式、G 代码配置、加工设置。

`CadToolPanelWidget` 只发出命令信号和维护配置选择，不应直接访问文档。

### 8.2 加工设置 Dock

`MachiningSettingsWidget` 由主窗口包装在 `machiningSettingsDock` 中，默认停靠右侧，支持左右停靠、浮动、关闭和从“视图 -> 加工设置”重新打开。

内容包括：

- 自动处理：导入后自动去重、自动识别方管截面、自动识别加工断面、自动清理内部线条。
- 导出设置：使用默认导出目录、使用 DXF 文件名。
- 方管识别状态：Y 长、Z 宽、圆角半径、圆角数量、加工断面数量和内部线条数量。

依赖规则：

- 启用自动加工断面或自动内部线条时，自动启用方管截面识别。
- 禁用方管截面识别时，同时禁用并取消两个子选项。
- 初始化同步不得触发设置写回信号。

加工断面数量按 `DocumentProcessState` 中唯一的断面组编号统计，仅计 `Break`；内部线条数量按分析排除状态统计。

### 8.3 输入和显示

- 绘图和修改参数优先使用光标旁动态输入面板。
- 状态栏捕捉支持基点、控制点、端点、中点、圆心/中心、交点和网格。
- 视图支持平移、滚轮缩放、轨道观察、标准视角和右上角视图方块。
- 框选采用向右包含、向左碰选语义。
- 选中图元显示控制点，重叠控制点支持候选切换。

## 9. 持久化设置

设置使用：

```cpp
QSettings("GCodePostProcessingSystem", "GCodePostProcessingSystem")
```

关键设置键：

| 设置键 | 默认值 | 含义 |
| --- | --- | --- |
| `dxf/autoDeduplicateOnImport` | `false` | 导入后自动去重 |
| `dxf/autoRecognizeRotaryTubeSectionOnImport` | `false` | 导入后自动识别方管截面 |
| `dxf/autoRecognizeRotaryEndCutsOnImport` | `false` | 截面成功后自动识别加工断面 |
| `dxf/autoRemoveInternalPathsOnImport` | `false` | 截面成功后自动清理内部线条 |
| `gcode/useDefaultExportPath` | `true` | 导出时复用上次目录 |
| `gcode/useDxfFileNameOnExport` | `false` | 使用当前 DXF 文件名 |
| `gcode/outputMode` | `auto` | 自动、3轴或4轴模式 |
| `ui/snapModeMask` | 默认掩码 | 对象捕捉组合 |
| `ui/mainWindowGeometry` | 空 | 主窗口位置和尺寸 |
| `ui/mainWindowState` | 空 | Dock 停靠、浮动和可见状态 |

不要修改已有设置键的语义。新增依赖选项时，应同时实现加载、保存、初始化无信号同步和用户操作即时写入。

## 10. 常用维护入口

| 需求 | 首要检查文件 |
| --- | --- |
| 文件导入/保存 | `src/Gcode_postprocessing_system_FileActions.cpp`、`CadDocument`、`dx_iface` |
| 导入后自动处理 | `runDxfImportPostProcessing()` |
| 主窗口菜单和设置 | `src/Gcode_postprocessing_system.cpp` |
| 顶部命令区 | `CadToolPanelWidget` |
| 右侧加工设置 | `MachiningSettingsWidget`、`syncMachiningSettingsState()` |
| 鼠标/键盘命令 | `CadController_*`、`CadViewer_EventHandling.cpp` |
| 图元创建和修改 | `CadEditer_*`、对应 `Cad*Item` |
| 渲染和拾取 | `CadViewer_*`、`CadEntityRenderer`、`CadEntityPicker` |
| 截面和内部线条 | `RotaryTubeGeometryAnalyzer` |
| 加工断面校验 | `RotaryCutBoundaryAnalyzer`、`Gcode_postprocessing_system_SortActions.cpp` |
| 排序与连续性 | `Gcode_postprocessing_system_SortActions.cpp` |
| G 代码输出 | `GGenerator`、`GProfile`、`Gcode_postprocessing_system_GCodeActions.cpp` |
| 位图矢量化 | `CadBitmapImportDialog`、`CadBitmapVectorizer` |
| 商业授权/品牌 | `AppLicense`、`AppBranding`、`COMMERCIAL_RELEASE.md` |

## 11. 扩展指南

### 11.1 增加图元类型

1. 确认 libdxfrw 的 `DRW::ETYPE` 和原始数据结构。
2. 新增或复用 `CadItem` 派生类，实现显示路径、拾取和控制点。
3. 在文档导入工厂中创建图元，检查 OCS/WCS 变换和 extrusion。
4. 按需求接入绘制、编辑、复制和 DXF 导出。
5. 只有需要加工时才实现二维/四轴路径；辅助图元应明确跳过 G 代码。
6. 更新 `.vcxproj` 和 `.vcxproj.filters`。
7. 验证导入、显示、保存、重新导入和 G 代码结果。

### 11.2 增加命令

1. 在 `CadToolPanelWidget` 或动态命令表中增加入口。
2. 由主窗口连接到现有业务函数，或启动 `CadController` 状态机。
3. 参数输入使用动态输入面板，不新增阻塞式参数对话框。
4. 预览写入 transient 状态，确认后通过 `CadEditer` 提交。
5. 多图元操作必须按一次命令进入 Undo/Redo。

### 11.3 增加导入后处理步骤

1. 先明确前置状态和输出状态。
2. 在 `runDxfImportPostProcessing()` 中安排固定顺序。
3. 自动和手动入口复用同一业务函数，并用 `interactive` 控制提示方式。
4. 自动失败不得阻塞导入。
5. 最后统一刷新排除状态和界面，避免每一步重复重绘。

### 11.4 增加设置

1. 在主窗口集中实现 `load...()` / `save...()`。
2. 在设置控件中只维护控件依赖和变化信号。
3. 使用 `m_updatingUi` 或等价机制避免初始化写回。
4. 设置变化后调用统一状态同步函数。
5. 在本文档设置表中记录键、默认值和迁移策略。

### 11.5 修改后处理规则

1. 同时检查 `GProfile` 默认值、配置读写和对话框。
2. 核对规则包裹顺序和颜色/图层/类型键生成。
3. 分别导出三轴和四轴样例。
4. 检查 CRLF、抬刀、安全高度、连续路径和文件头尾。
5. 不得在导出阶段永久修改文档几何。

## 12. 验证清单

### 12.1 每次代码修改

- 检查 `git diff --check`。
- 使用 Release x64 构建，确认无编译或链接错误。
- 启动构建目录中的 EXE，确认主窗口未立即退出。
- 检查修改动作的输入、状态变化和最终显示/文件结果，而不是只检查局部函数。
- 确认没有覆盖工作区中无关的用户修改。

### 12.2 CAD 回归

- 导入包含直线、圆弧、圆、椭圆、多段线、构造线和样条曲线的 DXF。
- 验证垂直平面、负坐标和 extrusion 不会造成显示翻转或黑屏。
- 保存为 DXF 后重新导入，检查图元数量和类型。
- 验证批量删除、移动、旋转、缩放及一次性 Undo/Redo。
- 验证对象捕捉、控制点编辑和框选。

### 12.3 方管加工回归

- 导入多个相同尺寸截面与干扰闭合轮廓的图纸。
- 检查自动截面诊断中的候选数、尺寸、圆角数和中心 X。
- 验证直角、三可靠圆角、四圆角候选优先级。
- 验证加工断面只在周向校验成功后标记。
- 验证加工断面不会被内部线条清理排除。
- 验证清空、指定、恢复后右侧统计立即更新。
- 验证自动选项父子依赖和重启持久化。

### 12.4 G 代码回归

- 三轴和四轴各导出一个包含多种图元和颜色规则的样例。
- 检查加工顺序、方向、起刀点、规则头尾和 CRLF。
- 检查连续相接图元是否避免无用抬刀。
- 检查方管圆角区域刀头方向是否指向圆角圆心。
- 对生产文件进行仿真、空跑和人工复核。

### 12.5 UI 回归

- 在 Windows 100%、125%、150% 缩放下检查顶部按钮和下拉框。
- 检查浅色、深色和自定义主题。
- 检查加工设置 Dock 左右停靠、浮动、关闭、菜单恢复和重启恢复。
- 检查窗口宽度或高度不足时没有控件重叠，Dock 内容可垂直滚动。

## 13. 发布与商业配置

发布流程、机器码、`license.dat`、品牌配置和构建目录打包见 [COMMERCIAL_RELEASE.md](./COMMERCIAL_RELEASE.md)。

发布前至少确认：

- 使用 Release x64 构建目录，不从中间对象目录取 EXE。
- Qt、OpenCV 和 MSVC 运行库已部署。
- `branding.json`、应用图标和 Lite/Pro 功能边界符合交付要求。
- 在目标机器或接近目标配置的旧机器上进行启动和基础导出测试。
- 不把开发侧许可证生成脚本和私钥材料交付给客户。

## 14. 协作和 Git 约束

开发前先阅读 [AGENTS.md](./AGENTS.md)。核心原则：

- 先沿真实数据流定位触发、状态、处理和最终结果。
- 使用最少代码解决明确问题，不做无关重构。
- 只修改任务需要的文件，不清理用户的无关改动。
- 手动和自动入口复用业务函数，避免规则分叉。
- 不把文档、几何、排序或 G 代码业务写入 Viewer/UI 控件。
- 对第三方 libdxfrw 改动保持谨慎。
- 功能修改和大规模文档修改建议分开提交。

推荐提交前执行：

```powershell
git status --short
git diff --check
git diff --stat
```

提交信息应说明用户可见结果或核心数据流变化，避免使用“更新代码”等无意义描述。

## 15. 相关文档

- [G/M 代码参考](./technical_file/G-M_Code.md)
- [商业发布与授权](./COMMERCIAL_RELEASE.md)
- [协作规则](./AGENTS.md)
- `docxs/PROJECT.md`：论文写作事实基准，不替代本维护文档。

当 README、代码和实际程序行为冲突时，应以源码和可重复验证结果为准，并立即修正文档。
