# G-code Post Processing System

[G/M 代码参考](./technical_file/G-M_Code.md)

本项目是一个基于 Qt 6 Widgets、OpenGL 4.5 Core Profile、Visual Studio 2026 的 Windows 桌面 CAD / G-code 后处理程序。当前代码主线已经具备 CAD 文件导入、位图矢量化导入、二维图元显示、基础交互、简单绘图与编辑命令，并提供 `2D` G 代码生成后端与 JSON Profile 配置能力；主窗口已接通导入文件、导入 `DXF`、导入 `DWG`、导入图片、保存文件（`Ctrl+S`）、导出为 `DXF`、导出为 `DXF`（安全模式）、反向加工以及按 `2D / 3D` 分类的排序菜单，其中 `2D` 下提供“排序（保留方向）”与“智能排序”，`3D` 排序逻辑保留，`3D` G 代码生成链路已整体清理并停用，等待按新的工艺模型重写。同时增加了仿 AutoCAD 风格的紧凑 Ribbon 工具面板，用于统一承载绘图、修改、图层与特性入口；其中“修改”面板现已接通 `移动`、`删除`、`旋转`、`复制`、`缩放`、`矩形阵列` 六类图元编辑操作，并提供显式的浅色 / 深色主题切换。当前“用户设置”菜单下也已接通 `G代码配置` 对话框，可按文件级、实体类型、图层规则、颜色规则四个层级定制导出行为，其中颜色规则已切换为以 AutoCAD 基础索引颜色为主的配置模型；Viewer 层也已支持加工方向箭头、加工顺序编号、选中图元基点/控制点手柄显示、AutoCAD 风格窗口框选（向左拖拽碰选、向右拖拽包含选）与框选过程实时候选高亮，底部状态栏也已接通基点 / 控制点 / 网格三类吸附开关，并已接入第一阶段控制点编辑（选中后点控点再点目标点，已覆盖 `Point`、`Line`、`Circle`、`Arc`、`Ellipse`、`Polyline`、`LWPolyline`）；夹点编辑现已支持实时几何预览，重叠夹点支持悬停 `1s` 弹出候选选择栏并可用 `Tab / Shift+Tab` 循环切换，候选项文案已改为夹点类型名且宽度自适应显示；近期又对方向箭头样式、主次网格对比和选中角标样式做了进一步优化。

## 项目现状

当前已实现：

- 导入 `.dxf` / `.dwg` 文件
- 导入 `.bmp` / `.png` / `.jpg` / `.jpeg` 位图，并通过 OpenCV 轮廓提取与规则图元拟合转换为 CAD 实体
- 使用 OpenGL 4.5 Core Profile 渲染 CAD 场景
- 支持点、直线、圆、圆弧、椭圆、多段线、轻量多段线的内部图元构建、显示与编辑
- 支持平移、缩放、顶视图切换、轨道观察、屏幕拾取
- 支持拖拽导入文件到视图区域
- 提供命令提示栏、命令历史栏、底部坐标状态栏
- 空闲态支持动态命令框：键入字符即弹出候选命令，支持 `Tab / Shift+Tab` 循环选择，`Enter` 或 `Space` 执行，`Esc` 取消
- 底部状态栏已提供 `基点`、`控制点`、`网格` 三类吸附开关，并与 Viewer 实际取点链路接通
- 网格已支持随缩放等级动态调整显示密度，网格吸附与当前显示网格步长保持一致；细分采用二分层级（如 `100 -> 50 -> 25 -> 12.5`），不会丢失上一级网格点
- 网格显示已区分主/次网格线：主网格线保持标准网格色，次网格线按背景色进一步淡化，降低高密度缩放下的视觉噪声
- 提供仿 AutoCAD 风格的 Ribbon 工具面板，包含“绘图”“修改”“图层”“特性”四个左对齐面板
- 提供统一的浅色 / 深色主题切换，入口位于菜单栏“用户设置”
- 提供点、线、圆、圆弧、椭圆、多段线、轻量多段线的交互式绘制
- 提供删除、移动、旋转、复制、缩放、矩形阵列、改色、改图层、Undo / Redo
- 提供第一阶段控制点编辑：在空闲态点击“当前选中图元”的可编辑控制点后，再次点击目标点提交；当前支持 `Point`、`Line`、`Circle`、`Arc`、`Ellipse`、`Polyline`、`LWPolyline`
- 控制点编辑已支持实时预览：在确认前即可看到图元变形、基点到目标点引导线与目标点高亮
- 当多个可编辑夹点发生重叠时，支持悬停 `1s` 弹出候选选择栏，并支持 `Tab / Shift+Tab` 循环切换候选
- 重叠夹点候选选择栏已改为显示“夹点类型名”（如 `基点`、`长轴控制点`、`顶点拉伸点`），并按文本宽度自适应布局
- 提供 transient 预览、十字光标叠加、选中高亮
- 选中高亮已升级为叠加式样：保留图元原始显示色，额外绘制轻描边与四角角标，降低完整包围框对视野的遮挡
- 提供 AutoCAD 风格窗口框选：向左拖拽为碰选，向右拖拽为包含选；框选拖拽过程中会实时高亮候选图元
- 选择集已支持 `Shift` 增量切换：`Shift + 左键` 与 `Shift + 框选` 均可对命中图元执行增选/反选
- 选中图元时已显示基点/控制点手柄；圆弧与椭圆边界端点使用三角箭头手柄强调方向端，且手柄支持悬停实时高亮
- Viewer 已支持加工方向箭头与加工顺序编号显示，并与排序 / 导出共用同一套加工路径语义；当前箭头为“起点锚定的小实心三角”样式
- 命令取点、移动基点/目标点和状态栏坐标显示已统一走同一套吸附后坐标
- 多图元修改已接通：`移动`、`删除`、`改色` 及 Ribbon 中的 `复制`、`旋转`、`缩放`、`阵列`、图层/颜色修改均可批量执行
- 多图元移动命令已支持实时预览：在目标点确认前会同时显示整组图元的平移后轮廓与引导线
- Ribbon“绘图”面板默认显示 6 种常用图元，隐藏项可通过标题旁下拉菜单继续进入
- Ribbon“图层”与“特性”面板已支持在“默认绘图属性”和“当前选中图元属性”之间自动切换显示
- 图层下拉项与颜色下拉项均以内嵌色块图标显示当前颜色语义
- 浅色主题下会对 Viewer 中低对比度线色做显示补偿，避免白色或近白色图元贴近背景后难以辨认
- `特性 -> 颜色` 中 `ByLayer` 色块固定显示所属图层颜色，不再错误跟随图元自定义颜色
- 提供 `GProfile` JSON 配置读写，支持文件级、实体类型、图层规则、颜色规则四类配置
- 提供 `用户设置 -> G代码配置...` 对话框，并已纳入当前浅色 / 深色主题
- 颜色规则默认内置 `BYLAYER`、`BYBLOCK` 与 AutoCAD 基础 `ACI:1` 至 `ACI:9` 索引颜色项，同时兼容真彩色扩展
- 提供 `GGenerator` 纯 `2D` G 代码生成后端，支持 `Line`、`Arc`、`Circle`、`Ellipse`、`Polyline`、`LWPolyline`
- `GProfile` 已新增 `A` 轴中心、角度偏移、连续展开、方向翻转、安全 `Z` 等旋转轴配置字段，并保留 JSON 读写
- 已接通主窗口“导入文件”“导入DXF...”“导入DWG...”“导入图片”“保存文件（Ctrl+S）”“导出为DXF...”“导出为DXF（安全模式）...”“反向加工”以及 `排序 -> 2D / 3D` 分类菜单动作
- 保存与导出统一写出 `.dxf`；保存在已有文档路径时会直接覆盖同名文件
- 提供基于 `CadItem::m_processOrder`、`CadItem::m_isReverse`、闭合路径起刀缝点参数的加工顺序与方向控制
- `2D` 智能排序会优先沿工件整体分布的主方向推进，并对明显“回头”的候选路径施加惩罚
- `2D` 智能排序已支持对 `Circle`、完整 `Ellipse`、闭合 `Polyline`、闭合 `LWPolyline` 自动联合优化加工方向与起刀缝点
- 圆形默认从顶部起刀，导出的 G 代码会跟随排序阶段确定的方向与缝点语义
- G 代码导出阶段会过滤无意义空行，减少脏输出

当前未完成或未接线：

- `3D` G 代码生成链路已从 `GGenerator` 中整体清理，当前导出 `Mode3D` 会直接提示待按新工艺模型重写
- `3D` 排序相关逻辑暂时保留，未随本次 `3D` G 代码生成清理一起移除
- 当前不提供可用的 `3D` G 代码导出，不提供完整 3D 建模/编辑工作流
- `3D` 导出的旋转轴参数当前仅纳入 `GProfile` 数据模型和 JSON 读写，配置对话框暂未暴露专门编辑界面
- 普通 DXF 导出会尽量保留原始实体；若包含兼容性较弱的普通 `POLYLINE`，建议改用“导出为DXF（安全模式）”

## 技术栈

- C++17
- Qt `6.9.3_msvc2022_64`
- Qt Widgets / OpenGL / OpenGL Widgets
- OpenGL 4.5 Core Profile
- Visual Studio 2026 / MSBuild
- `libdxfrw` 作为 DXF / DWG 读写底层库
- OpenCV `4.11.0`，用于位图预处理、轮廓提取与矢量化拟合

图形相关约束：

- 视图层基于 `QOpenGLFunctions_4_5_Core`
- 着色器与渲染路径按 OpenGL 4.5 Core Profile 设计
- 不应回退到旧式固定管线写法

## 构建环境

### 环境要求

- Windows x64
- Visual Studio 2026
- Qt `6.9.3_msvc2022_64`
- QtMsBuild 已正确安装并可被 Visual Studio 识别
- OpenCV `4.11.0`

当前工程文件 [G-code_post-processing_system.vcxproj](D:/projects/visual_studio_2026/G-code_post-processing_system/G-code_post-processing_system.vcxproj) 里默认使用以下 OpenCV 路径：

```text
OpenCVRoot     = D:\develop\opencv
OpenCVInclude  = $(OpenCVRoot)\build\include
OpenCVLib      = $(OpenCVRoot)\build\x64\vc16\lib
OpenCVBin      = $(OpenCVRoot)\build\x64\vc16\bin
```

如果本机 OpenCV 安装路径不同，需要先同步调整工程属性。

### 构建命令

```powershell
msbuild .\G-code_post-processing_system.vcxproj /p:Configuration=Debug /p:Platform=x64
msbuild .\G-code_post-processing_system.vcxproj /p:Configuration=Release /p:Platform=x64
devenv .\G-code_post-processing_system.slnx
```

若系统环境变量中未配置 `msbuild`，可直接使用 Visual Studio 2026 Insiders 的 `MSBuild.exe` 绝对路径：

```powershell
& "D:\Program Files\Microsoft Visual Studio\18\Insiders\MSBuild\Current\Bin\MSBuild.exe" .\G-code_post-processing_system.vcxproj /p:Configuration=Debug /p:Platform=x64
& "D:\Program Files\Microsoft Visual Studio\18\Insiders\MSBuild\Current\Bin\MSBuild.exe" .\G-code_post-processing_system.vcxproj /p:Configuration=Release /p:Platform=x64
```

建议优先使用 `Debug|x64` 进行日常开发和交互验证。

## 仓库结构

```text
G-code_post-processing_system/
|-- include/                                # 公共头文件
|   |-- Gcode_postprocessing_system.h       # 主窗口类声明
|   |-- AppTheme.h                          # 应用主题颜色定义
|   |-- CadViewer.h                         # OpenGL 视图主类
|   |-- CadController.h                     # 输入控制器
|   |-- DrawStateMachine.h                  # 绘图/编辑状态机
|   |-- CadEditer.h                         # 编辑器与命令栈接口
|   |-- CadDocument.h                       # 文档模型
|   |-- CadBitmapImportDialog.h             # 位图导入参数对话框
|   |-- CadBitmapVectorizer.h               # 位图预处理与矢量化拟合
|   |-- GProfile.h                          # G 代码 Profile 配置模型
|   |-- GProfileDialog.h                    # G 代码 Profile 配置对话框
|   |-- GGenerator.h                        # G 代码生成器
|   |-- CadItem.h			               # 图元基类
|   |-- CadLineItem.h                       # 直线图元
|   |-- CadCircleItem.h                     # 圆图元
|   |-- CadArcItem.h                        # 圆弧图元
|   |-- CadEllipseItem.h                    # 椭圆图元
|   |-- CadPointItem.h                      # 点图元
|   |-- CadPolylineItem.h                   # 多段线图元
|   |-- CadLWPolylineItem.h                 # 轻量多段线图元
|   |-- CadProcessVisualUtils.h             # 加工方向/顺序显示共用语义工具
|   |-- CadCommandLineWidget.h              # 命令栏组件
|   |-- CadStatusPaneWidget.h               # 状态栏组件
|   |-- CadToolPanelWidget.h                # Ribbon 工具面板组件
|   |-- CadGraphicsCoordinator.h            # 渲染协调层
|   |-- CadSceneCoordinator.h               # 场景协调层
|   |-- CadSceneContext.h                   # 场景上下文
|   |-- CadSceneRenderCache.h               # GPU 缓冲缓存
|   |-- CadEntityRenderer.h                 # 实体绘制器
|   |-- CadEntityPicker.h                   # 屏幕拾取器
|   |-- CadOverlayRenderer.h                # Overlay 绘制
|   |-- CadReferenceRenderer.h              # 网格/坐标轴参考图形绘制
|   |-- CadShaderManager.h                  # Shader 管理
|   |-- CadCrosshairBuilder.h               # 十字光标预览构建
|   |-- CadPreviewBuilder.h                 # 命令 transient 预览构建
|   |-- CadCamera*.h                        # 相机与视图数学
|   |-- CadViewTransform.h                  # 屏幕/世界坐标转换
|   |-- CadViewInteractionController.h      # 视图交互控制
|   |-- CadViewerUtils.h                    # Viewer 辅助方法
|   |-- CadRenderTypes.h                    # 渲染数据类型
|   |-- CadInteractionConstants.h           # 交互常量
|   |-- dx_data.h                           # DXF 中间数据结构
|   |-- dx_iface.h                          # libdxfrw 接入层
|   `-- libdxfrw/                           # 第三方头文件
|       `-- intern/                         # 第三方内部头文件
|-- src/                                    # 主要源码
|   |-- main.cpp                            # 程序入口
|   |-- Gcode_postprocessing_system.cpp     # 主窗口实现
|   |-- CadViewer.cpp                       # 视图与 OpenGL 主流程
|   |-- CadController.cpp                   # 输入解释与快捷键分发
|   |-- DrawStateMachine.cpp                # 状态机默认逻辑
|   |-- CadEditer.cpp                       # 绘图、编辑、命令栈实现
|   |-- CadDocument.cpp                     # 文档模型实现
|   |-- CadBitmapImportDialog.cpp           # 位图导入配置与预览
|   |-- CadBitmapVectorizer.cpp             # 位图预处理、轮廓提取、图元拟合
|   |-- GProfile.cpp                        # G 代码 Profile 配置读写
|   |-- GProfileDialog.cpp                  # G 代码 Profile 配置对话框实现
|   |-- GGenerator.cpp                      # 纯 2D G 代码生成，3D 生成链路已清理
|   |-- CadItem.cpp						  # 图元基类
|   |-- CadProcessVisualUtils.cpp           # 加工方向/顺序显示共用语义实现
|   |-- CadToolPanelWidget.cpp              # Ribbon 工具面板实现
|   |-- Cad*Item.cpp                        # 各类图元几何离散与显示数据生成
|   |-- CadGraphicsCoordinator.cpp          # 渲染协调实现
|   |-- CadSceneCoordinator.cpp             # 场景协调实现
|   |-- CadEntityRenderer.cpp               # 实体绘制实现
|   |-- CadEntityPicker.cpp                 # 拾取实现
|   |-- CadOverlayRenderer.cpp              # Overlay 渲染实现
|   |-- CadReferenceRenderer.cpp            # 参考图形渲染实现
|   |-- CadShaderManager.cpp                # Shader 初始化实现
|   |-- CadCrosshairBuilder.cpp             # 十字光标构建实现
|   |-- CadPreviewBuilder.cpp               # Transient 预览实现
|   |-- CadCamera*.cpp                      # 相机逻辑与数学
|   |-- CadViewTransform.cpp                # 坐标转换实现
|   |-- CadViewInteractionController.cpp    # 视图交互实现
|   |-- CadViewerUtils.cpp                  # Viewer 辅助实现
|   |-- CadOpenGLState.cpp                  # OpenGL 状态控制
|   |-- dx_iface.cpp                        # DXF / DWG 读写桥接
|   `-- libdxfrw/                           # 第三方源代码
|       `-- intern/                         # 第三方内部实现
|-- technical_file/
|   `-- G-M_Code.md                         # G/M 代码资料
|-- Gcode_postprocessing_system.ui          # Qt Widgets UI
|-- Gcode_postprocessing_system.qrc         # Qt 资源文件
|-- G-code_post-processing_system.vcxproj   # VS/Qt 工程
|-- G-code_post-processing_system.slnx      # 解决方案
|-- AGENTS.md                               # 仓库协作规则
|-- README.md                               # 当前文档
|-- x64/                                    # 构建输出，不提交
`-- .vs/                                    # IDE 状态，不提交
```

## 总体架构

项目当前按“主窗口 + Ribbon 工具面板 + Viewer + Controller + Editer + Document + 渲染协调层 + DXF Adapter + Bitmap Vectorizer + G-code Backend”的组合结构组织。相比早期版本，`CadViewer` 内部职责已经进一步拆分给 `CadSceneCoordinator`、`CadGraphicsCoordinator`、`CadEntityRenderer`、`CadEntityPicker` 等子模块。

```text
用户输入 / Qt 菜单 / 拖拽文件 / 键盘鼠标
        |
        v
Gcode_postprocessing_system                (窗口层 / 组装层)
        |
        +------> CadCommandLineWidget      (命令栏显示)
        +------> CadStatusPaneWidget       (坐标状态显示)
        +------> CadToolPanelWidget        (Ribbon 工具面板)
        |
        +------> 导入分发
        |              |
        |              +------> DXF / DWG -> CadDocument::readDxfDocument()
        |              |
        |              `------> Bitmap -> CadBitmapImportDialog -> CadBitmapVectorizer
        |
        v
CadViewer                                  (View)
        |
        +------> CadController             (输入解释)
        |              |
        |              +------> DrawStateMachine
        |              |
        |              `------> CadEditer
        |
        +------> CadSceneCoordinator       (文档绑定 / 包围盒 / 缓冲脏标记)
        |
        +------> CadGraphicsCoordinator    (OpenGL / Shader / 参考图形 / Overlay)
        |              |
        |              +------> CadEntityRenderer
        |              +------> CadEntityPicker
        |              +------> CadPreviewBuilder
        |              `------> CadCrosshairBuilder
        |
        v
CadDocument                                (Model)
        |
        +------> CadItem / Cad*Item
        |
        +------> dx_data / dx_iface
                        |
                        v
                     libdxfrw
        |
        `------> GGenerator + GProfile      (2D 后处理 / Profile 配置)
```

### 分层职责

`Gcode_postprocessing_system`

- 负责 UI 初始化与窗口装配
- 创建命令栏、状态栏并插入中心布局
- 负责将状态栏中的吸附开关接线到 Viewer
- 创建 Ribbon 工具面板，并负责其与 Viewer / Document / Editer 的状态同步
- 创建“用户设置 -> 主题 -> 浅色模式 / 深色模式”菜单，并负责主题持久化与统一下发
- 统一处理文件导入分发
- 接通导入文件/导入DXF/导入DWG/导入图片、保存文件（`Ctrl+S`）、普通/安全两种 DXF 导出、反向加工、排序相关菜单动作
- 维护默认绘图图层、默认绘图颜色以及工具面板显示状态
- 持有 `CadDocument` 与 `CadEditer`

`CadToolPanelWidget`

- 负责提供仿 AutoCAD 风格的紧凑 Ribbon 面板 UI
- 当前包含“绘图”“修改”“图层”“特性”四组工具
- 负责展示绘图入口、移动/删除/旋转/复制/缩放/矩形阵列、当前图层、当前颜色等面板控件
- 通过信号把绘图、修改、改图层、改颜色请求回传给主窗口

`CadViewer`

- 负责 `QOpenGLWidget` 生命周期和事件入口
- 管理相机、场景刷新、坐标转换
- 将键鼠事件转交 `CadController`
- 消费 `CadDocument` 数据并驱动渲染
- 对外提供开始绘图、开始移动、设置默认绘图属性等薄封装接口
- 对外提供交互取点坐标解析，并统一执行基点/控制点/网格吸附
- 接收主窗口下发的主题，并统一应用到背景、网格和加工编号气泡显示
- 在浅色主题下对低对比度实体颜色执行显示补偿，以保证图元可读性
- 负责加工方向箭头 overlay 与加工顺序屏幕编号显示
- 负责绘制当前选中图元的基点/控制点手柄 overlay

`CadCommandLineWidget` + `CadStatusPaneWidget`

- `CadCommandLineWidget` 负责显示命令提示与命令历史
- `CadStatusPaneWidget` 负责显示当前鼠标世界坐标
- `CadStatusPaneWidget` 当前已提供基点 / 控制点 / 网格三类吸附开关，并通过信号连接到 Viewer

`CadController` + `DrawStateMachine`

- 解释鼠标、键盘、滚轮输入
- 分流视图命令、绘图命令和编辑命令
- 维护当前命令状态、控制点、提示文本
- 保存默认绘图图层、默认绘图颜色索引等绘图状态
- 处理多段线圆弧/直线输入切换

`CadEditer`

- 根据状态机创建新实体
- 执行删除、移动、旋转、复制、缩放、矩形阵列、改色、改图层
- 执行反向加工切换、加工顺序写入和批量排序状态提交
- 执行闭合路径起刀缝点写入，并将其纳入 Undo / Redo
- 维护 Undo / Redo 命令栈
- 将模型修改统一提交给 `CadDocument`

`CadDocument`

- 持有原始 `dx_data`
- 持有场景中的 `CadItem` 容器
- 把导入得到的 `DRW_Entity` 转换成内部图元
- 维护图层表、图层颜色查询与缺失图层补建
- 通过 `sceneChanged` 驱动视图刷新

`CadBitmapImportDialog` + `CadBitmapVectorizer`

- 负责位图导入参数配置、预览和错误提示
- 使用 OpenCV 完成灰度化、阈值化、形态学处理、轮廓提取
- 将轮廓拟合为点、线、圆、圆弧、椭圆、多段线等实体
- 将生成实体追加或替换到当前文档

`GProfile` + `GGenerator`

- `GProfile` 负责读取、保存 JSON 格式的 G 代码配置
- 当前配置范围包括文件级、实体类型、图层规则、颜色规则
- 颜色规则优先使用 `BYLAYER`、`BYBLOCK` 与 `ACI:n` 索引键，也兼容旧的真彩色 `#RRGGBB` 键
- `GProfileDialog` 负责提供用户侧可编辑配置界面，并显式跟随应用浅色 / 深色主题
- `GGenerator` 直接解析现有 `CadItem` 子类和原始 `DRW_Entity` 参数生成 G 代码
- 导出时会跟随排序阶段写回的加工方向与闭合路径起刀缝点
- 当前仅实现纯 `2D` 激光加工输出，`3D` 模式暂未实现

`CadSceneCoordinator` + `CadGraphicsCoordinator`

- `CadSceneCoordinator` 负责文档绑定、边界计算、GPU 缓冲重建时机
- `CadGraphicsCoordinator` 负责 OpenGL 状态、Shader、网格、坐标轴和 Overlay 渲染
- `CadEntityRenderer` 负责实体绘制
- `CadEntityPicker` 负责屏幕空间拾取
- `CadPreviewBuilder` / `CadCrosshairBuilder` 负责 transient 预览

`CadProcessVisualUtils`

- 负责统一计算图元的加工起点、终点、方向与编号锚点
- 负责为点、线、圆、圆弧、椭圆、多段线、轻量多段线构建选中态基点/控制点手柄语义
- 供 Viewer 的加工辅助显示、主窗口排序逻辑与后续导出语义复用

## 核心流程

### CAD 文件导入

1. 用户通过菜单或拖拽方式导入 `.dxf` / `.dwg`
2. `Gcode_postprocessing_system` 调用 `CadDocument::readDxfDocument()`
3. `CadDocument` 通过 `dx_iface` 使用 `libdxfrw` 读取文件
4. 原始实体写入 `dx_data`
5. `CadDocument::init()` 将支持的实体转换为 `CadItem`
6. 文档发出 `sceneChanged`
7. `CadViewer` 重建缓冲并刷新显示

### 位图导入

1. 用户通过菜单或拖拽方式导入位图文件
2. `Gcode_postprocessing_system` 打开 `CadBitmapImportDialog`
3. 用户配置阈值、轮廓模式、拟合策略、插入比例、图层名等参数
4. `CadBitmapVectorizer` 使用 OpenCV 生成预处理结果并完成矢量化
5. 生成的 `DRW_Entity` 通过 `CadDocument::appendEntities()` 追加或替换到当前文档
6. Viewer 按需执行 `fitScene()` 并刷新显示

### 绘图与编辑

1. 用户通过快捷键或 Ribbon 工具面板进入绘图或编辑命令
2. `CadController` 更新 `DrawStateMachine`
3. `CadViewer` 先将鼠标位置投影到 `Z=0` 绘图平面，并按当前吸附开关执行基点 / 控制点 / 网格吸附修正
4. `CadEditer` 根据当前状态创建或修改 `DRW_Entity`
5. `CadDocument` 更新内部图元并发出 `sceneChanged`
6. Viewer 自动刷新，Undo / Redo 栈按事务规则更新

### G 代码生成

1. `GGenerator` 绑定当前 `CadDocument`
2. 生成时通过 `GProfile` 读取文件级、图层规则、颜色规则、实体类型配置
3. `GGenerator` 直接解析 `CadItem` 对应的原始实体参数，包括图层名和颜色键
4. 当前 `2D` 输出支持：
   `Line -> G01`
   `Arc/Circle -> G02/G03 + I/J`
   `Polyline/LWPolyline -> 直线段 + bulge 圆弧段`
   `Ellipse -> 离散为 G01`
5. 导出时会按“文件级 -> 图层规则 -> 颜色规则 -> 实体类型”的顺序组合头尾代码块
6. 对 `Circle` 默认以顶部为起刀点；对完整 `Ellipse` 与闭合 `Polyline/LWPolyline`，会按排序阶段确定的缝点导出
7. 导出时会跳过空白代码块中的无意义空行，减少对机床无效的文本噪声
8. 生成器在导出时弹出文件保存对话框并输出 `.nc/.gcode/.txt`

### 排序与方向控制

1. 用户可先通过“反向加工”修改当前选中图元的加工方向
2. “排序（保留方向）”会在保留当前 `m_isReverse` 的前提下，仅重新计算 `m_processOrder`
3. `排序 -> 2D -> 智能排序` 会同时优化加工顺序，并在需要时自动调整部分图元的 `m_isReverse`
4. `2D` 智能排序会结合工件整体分布估计一个主扫掠方向，优先沿该方向推进，并尽量减少明显回头
5. 对 `Circle`、完整 `Ellipse`、闭合 `Polyline`、闭合 `LWPolyline`，`2D` 智能排序会联合优化加工方向与起刀缝点，以减小缝点处顿挫
6. 两类排序结果都会写回 `CadItem::m_processOrder`；智能排序还会同步写回闭合路径的起刀缝点参数，并纳入 Undo / Redo
7. Viewer 会同步显示加工方向箭头与加工顺序编号，便于直接核对排序结果

## 使用说明

### 支持导入的文件类型

- CAD：`.dxf`、`.dwg`
- 位图：`.bmp`、`.png`、`.jpg`、`.jpeg`

### 当前支持显示与编辑的图元

- `Point`
- `Line`
- `Circle`
- `Arc`
- `Ellipse`
- `Polyline`
- `LWPolyline`

说明：

- `libdxfrw` 底层可读取更多实体类型
- 但当前 [src/CadDocument.cpp](D:/projects/visual_studio_2026/G-code_post-processing_system/src/CadDocument.cpp) 中 `createCadItemForEntity()` 只会把上面 7 类实体转换为项目内部 `CadItem`
- 其它实体即使被读入，也不会生成当前可显示、可编辑的内部图元

### 视图交互

- `F`：适配场景
- `T` 或 `Home`：回到顶视图
- `+` / `=`：放大
- `-` / `_`：缩小
- 中键拖动：平移
- `Shift + 中键拖动`：轨道旋转
- 左键：拾取图元
- 左键拖拽窗口框选：向左拖拽为碰选，向右拖拽为包含选
- `Shift + 左键`：对命中图元执行增选/反选
- `Shift + 左键拖拽框选`：对框内命中图元执行增选/反选
- 滚轮：以鼠标附近为锚点缩放

加工辅助显示补充：

- Viewer 会为可参与加工的图元绘制方向箭头；当前为“尖端位于加工起始点”的小实心三角，不再附带后置箭杆
- 已设置 `m_processOrder` 的图元会显示屏幕空间编号气泡，编号从 `1` 开始
- 当前默认配色中，绿色表示正向加工，红色表示反向加工，黄色表示当前选中图元
- 选中图元后会额外显示基点与控制点手柄，其中基点使用更大的圆点，圆弧/椭圆边界端点会使用三角箭头手柄
- 选中图元会显示轻描边与四角角标（替代完整矩形包围框），框选预览候选图元会实时显示低优先级高亮
- 基点/控制点手柄会在鼠标悬停时显示实时高亮环，便于确认可编辑目标
- 平移或轨道观察过程中会临时暂停加工辅助显示，以优先保证交互流畅度

### 吸附与状态栏

底部状态栏当前除了显示坐标，还提供三类吸附开关：

- `基点`：优先吸附到图元的基点，如点坐标、线起点、圆心、圆弧圆心、多段线首点等
- `控制点`：吸附到线终点、圆四象限点、圆弧起中终点、椭圆轴端点、多段线其余顶点等控制点
- `网格`：当未命中对象吸附点时，把当前位置量化到当前动态步长网格（随缩放自动调整）

当前行为说明：

- 命令取点、移动命令取点和底部状态栏坐标显示都会使用吸附后的坐标
- 对象吸附仅对“当前选中图元”生效；未选中图元不会参与基点/控制点吸附
- 对象吸附优先级高于网格吸附；在对象吸附中，基点优先级高于控制点
- 当前对象吸附阈值与网格吸附阈值均为固定屏幕距离阈值
- 十字准线与拾取框会跟随吸附后的坐标；命中吸附点时会显示额外高亮标记

### 绘图命令

- `P`：点
- `L`：直线
- `C`：圆
- `A`：圆弧
- `E`：椭圆
- `O`：多段线
- `W`：轻量多段线
- `Esc`：取消当前命令；当无活动命令且存在选中图元时，清空当前选中

多段线命令补充：

- `A`：切换为圆弧段输入
- `L`：切回直线段输入
- `Enter` / `Space`：结束多段线
- `C`：闭合多段线

### 编辑命令

- `Delete`：删除当前选中图元（支持多选）
- `M`：移动当前选中图元（支持多选）
- `K`：修改当前选中图元颜色（支持多选）
- 空闲态左键点击“当前选中图元”的可编辑控制点，再左键指定目标点：控制点编辑（第一阶段）
- `Ctrl + Z`：撤销
- `Ctrl + Y`：重做
- `Ctrl + Shift + Z`：重做
- `Ctrl + S`：保存当前文档（写回当前同名 `.dxf`）

补充说明：

- `旋转`、`缩放` 当前通过 Ribbon“修改”面板触发，支持多选并默认绕“整组图元”几何包围盒中心执行
- `复制` 当前通过 Ribbon“修改”面板触发，支持多选并按输入的 `X / Y` 偏移量批量生成副本
- `矩形阵列` 当前通过 Ribbon“修改”面板触发，支持多选并按输入的行数、列数、行间距、列间距批量复制
- 控制点编辑当前为第一阶段实现：采用“两次点击提交”交互，当前已支持实时预览与重叠夹点候选选择；已覆盖 `Point`、`Line`、`Circle`、`Arc`、`Ellipse`、`Polyline`、`LWPolyline` 的可编辑夹点
- 空闲态动态命令框已接通以下常用命令入口：`line`、`point`、`circle`、`arc`、`ellipse`、`polyline`、`lwpolyline`、`move`、`delete`、`color`、`fit`、`top`、`zoomin`、`zoomout`

### 动态命令框

- 仅在空闲态生效；当不在活动命令中，直接键盘输入即可进入命令匹配
- 支持英文全称、常见缩写和部分中文别名匹配
- `Tab`：下一个候选；`Shift + Tab`：上一个候选
- `Enter` 或 `Space`：执行当前高亮候选命令
- `Backspace`：删除输入字符；`Delete`：清空命令输入
- `Esc`：取消动态命令框，不影响其它常规交互状态

### Ribbon 工具面板

主窗口工具栏区域当前提供一组仿 AutoCAD 风格的紧凑 Ribbon 面板，默认左对齐排列：

- `绘图`：默认显示 `直线`、`圆`、`圆弧`、`椭圆`、`多段线`、`轻量多段线` 六种常用图元；隐藏图元可通过标题旁下拉菜单进入
- `修改`：当前提供 `移动`、`删除`、`旋转`、`复制`、`缩放`、`阵列`，其中“阵列”当前实现为矩形阵列
- `图层`：提供单一下拉框；闭合时显示当前图层，展开后列出全部图层，每项前面显示该图层颜色小方块
- `特性`：当前提供图层与颜色两项；图层项显示所属图层，颜色项以色块图标 + 文字方式展示

属性联动规则：

- 未选中图元时，`图层` 与 `特性` 显示“默认绘图属性”，用于控制后续新建图元
- 选中图元时，`图层` 与 `特性` 自动切换为显示该图元的真实属性
- 修改图层或颜色时，主窗口会根据当前是否有选中图元，决定写回默认绘图状态或直接修改图元
- 当图元带有自定义颜色时，`特性 -> 颜色` 中 `ByLayer` 项的小方块仍固定显示所属图层的默认颜色，而不是图元自己的颜色

### 主题切换

菜单栏当前提供 `用户设置 -> 主题 -> 浅色模式 / 深色模式`：

- 主题切换会统一作用于菜单栏、工具栏、Ribbon 面板、Viewer、命令栏和底部状态栏
- `用户设置 -> G代码配置...` 打开的 Profile 对话框也会显式跟随当前主题，而不依赖系统默认样式
- 主题选择会通过 `QSettings` 持久化，下次启动时自动恢复上次选择
- 浅色主题主要面向截图、文档和论文打印场景；深色主题主要面向日常长时间使用

浅色主题下的显示补偿规则：

- 如果实体颜色与 Viewer 背景对比度过低，渲染层会自动做显示补偿
- 对白色或近白色的中性色，优先显示为深色可见线
- 对带有色相但过亮的颜色，优先做保色加深，而不是一律改成黑色
- 该补偿仅影响屏幕显示，不修改文档中的真实图层颜色、实体颜色或导出语义

### 菜单动作

- 文件 -> `导入文件`：导入 `.dxf` / `.dwg` / 位图文件
- 文件 -> `导入DXF...`：仅导入 `.dxf`
- 文件 -> `导入DWG...`：仅导入 `.dwg`
- 文件 -> `导入图片`：直接进入位图导入对话框
- 文件 -> `保存文件`：保存当前文档（快捷键 `Ctrl+S`，默认覆盖当前同名 `.dxf`）
- 文件 -> `导出为DXF...`：普通导出（尽量保留当前实体）
- 文件 -> `导出为DXF（安全模式）...`：安全导出（仅导出当前支持图元、清理扩展字段、普通 `POLYLINE` 转 `LWPOLYLINE`）
- 编辑 -> `反向加工`：切换当前选中图元的加工方向
- 排序 -> `2D -> 排序（保留方向）`：保留当前方向设置，仅重排加工顺序
- 排序 -> `2D -> 智能排序`：同时优化加工顺序、图元方向和闭合路径起刀缝点
- 排序 -> `3D -> 排序（保留方向） / 智能排序`：排序相关代码保留，未随本次 `3D` G 代码生成清理一起移除
- 用户设置 -> `主题 -> 浅色模式 / 深色模式`：切换整套界面主题
- 用户设置 -> `G代码配置...`：打开当前活动 Profile 配置对话框

说明：

- 执行“反向加工”或两类排序后，Viewer 中的方向箭头和顺序编号会立即刷新
- 顺序编号、箭头方向与排序逻辑当前共用同一套加工路径语义

### G 代码生成说明

当前后处理后端已经具备以下能力：

- 支持纯 `2D` G 代码生成
- 支持 `Line`、`Arc`、`Circle`、`Ellipse`、`Polyline`、`LWPolyline`
- 支持读取 `CadItem::m_processOrder` 作为导出顺序
- 支持读取 `CadItem::m_isReverse` 作为反向加工标记
- 支持读取闭合图元的起刀缝点参数，并与 Viewer / 排序共用同一套路径语义
- 支持按 `GProfile` 套用文件级、图层规则、颜色规则、实体类型规则

`G代码配置` 对话框当前支持：

- 编辑 Profile 名称
- 编辑文件级 `header / footer / comment`
- 编辑实体类型级 `header / footer / comment`
- 编辑图层级 `header / footer / comment`
- 编辑颜色级 `header / footer / comment`
- 导入 / 导出 JSON
- 恢复默认配置

颜色规则说明：

- 默认内置 `BYLAYER`、`BYBLOCK`、`ACI:1` 到 `ACI:9`
- 其中 `ACI:1` 到 `ACI:9` 对应基础 AutoCAD 索引颜色
- 在此基础上仍可新增真彩色规则，用于特殊工艺场景

当前限制：

- `Point` 当前不会输出加工轨迹
- `Ellipse` 当前通过折线离散输出，不使用专门椭圆插补指令
- 闭合 `Polyline/LWPolyline` 的缝点优化当前以顶点为候选集合，不做边中点或弧长连续参数搜索
- `3D` G 代码生成链路已从 `GGenerator` 中整体清理，当前不会输出任何三维刀路代码
- `Mode3D` 当前会直接返回“待按新的工艺模型重写”

### 位图导入说明

位图导入对话框当前支持以下能力：

- 替换当前文档或追加到当前文档
- Otsu、固定阈值、自适应阈值、Canny 边缘等预处理方式
- 开运算、闭运算、膨胀、腐蚀等形态学处理
- 仅外轮廓或全部轮廓提取
- 优先拟合规则图元，或全部输出为折线
- 设置缩放比例、插入偏移、图层名、颜色和自动适配场景

## 绘图平面约束

当前绘图逻辑按二维 CAD 方式实现，所有新建图元统一落在世界坐标 `Z=0` 平面：

- `CadController` 会将鼠标位置投影到绘图平面
- `CadEditer` 在创建实体时再次将坐标压到 `Z=0`
- 即使当前视角处于轨道观察状态，新建图元也不会写入任意 Z 深度

这是一项显式设计。现阶段项目支持“二维绘制 + 三维观察”，不支持完整 3D 建模工作流。

## 已知注意事项

- 当前内部仅完整支持 7 类 CAD 图元的可视化与编辑
- 新建图元强制落在 `Z=0` 平面
- Ribbon“绘图”面板当前只把常用图元直接显示，其余入口需通过标题旁下拉菜单进入
- 浅色主题下 Viewer 会对低对比度线色做显示补偿；这是显示层行为，不会改写文档中的真实颜色数据
- `GProfile` 颜色规则当前以基础索引颜色与真彩色键混合管理，后续如需扩展完整 AutoCAD ACI 颜色表，还需要继续补齐 UI 与预设
- 位图导入依赖 OpenCV 运行时 DLL 拷贝到输出目录
- `GGenerator` 当前仅实现纯 `2D` 输出，原有 `3D` 生成代码已整体清理
- `排序 -> 2D -> 排序（保留方向）` 当前尊重用户已设置的加工方向与闭合路径缝点，不承担手动点选排序职责；手动排序接口代码已保留但暂未暴露到 UI
- `2D` 智能排序当前采用“局部最近邻 + 整体扫掠方向偏好 + 切线连续性惩罚”的启发式策略，目标是减少回头与顿挫，但不是严格全局最优解
- 加工方向箭头当前按世界坐标中的二维方向绘制，样式已简化为起点锚定的小实心三角；编号气泡会在屏幕空间做简单避让，但密集图元场景下仍可能出现局部遮挡
- 当前吸附主要作用于命令取点和状态栏坐标显示，普通空闲态拾取/选择仍按原始屏幕位置进行
- 基点/控制点吸附仅在图元已选中、且对应手柄可见时才会生效
- 当前对象吸附点集合主要来自图元基点与控制点，不包含交点、中点、切点、垂足等更完整的 CAD 吸附类型
- 控制点编辑当前仍为第一阶段实现，采用“两次点击提交”模式；已支持实时预览与重叠夹点候选选择
- `Ellipse` 当前按折线离散导出，精度取决于固定采样密度
- 第三方 `libdxfrw` 目录不应轻易修改
- 工程里仍存在部分历史遗留命名，修改构建配置前需要先核对 `.ui`、`.vcxproj` 与当前源码是否一致

## 协作约束摘要

- 默认使用中文沟通
- 遵循 MVC / 分层思路，避免把过多逻辑重新堆回 `CadViewer`
- 修改 `src/libdxfrw/` 或 `include/libdxfrw/` 时应明确说明原因和影响范围
- OpenGL 保持 4.5 Core Profile

