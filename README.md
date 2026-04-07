# G-code Post Processing System

[G/M 代码参考](./technical_file/G-M_Code.md)

本项目是一个基于 Qt 6 Widgets、OpenGL 4.5 Core Profile、Visual Studio 2026 的 Windows 桌面 CAD / G-code 后处理程序。当前代码主线已经具备 CAD 文件导入、位图矢量化导入、二维图元显示、基础交互、简单绘图与编辑命令，并新增了纯 `2D` 激光加工 G 代码生成后端与 JSON Profile 配置能力；不过导出菜单和参数配置界面尚未接线，`3D` 生成模式也还未实现。

## 项目现状

当前已实现：

- 导入 `.dxf` / `.dwg` 文件
- 导入 `.bmp` / `.png` / `.jpg` / `.jpeg` 位图，并通过 OpenCV 轮廓提取与规则图元拟合转换为 CAD 实体
- 使用 OpenGL 4.5 Core Profile 渲染 CAD 场景
- 支持点、直线、圆、圆弧、椭圆、多段线、轻量多段线的内部图元构建、显示与编辑
- 支持平移、缩放、顶视图切换、轨道观察、屏幕拾取
- 支持拖拽导入文件到视图区域
- 提供命令提示栏、命令历史栏、底部坐标状态栏
- 提供点、线、圆、圆弧、椭圆、多段线、轻量多段线的交互式绘制
- 提供删除、移动、改色、Undo / Redo
- 提供 transient 预览、十字光标叠加、选中高亮
- 提供 `GProfile` JSON 配置读写，支持文件头尾、实体类型头尾、实体颜色头尾三类配置
- 提供 `GGenerator` 纯 `2D` G 代码生成后端，支持 `Line`、`Arc`、`Circle`、`Ellipse`、`Polyline`、`LWPolyline`

当前未完成或未接线：

- `Gcode_postprocessing_system.ui` 中的“导出 G 代码”“反向加工”等菜单项当前仍没有业务绑定
- `GGenerator` 当前仅实现纯 `2D` 生成，`3D` 模式入口已预留但未实现
- [src/CadDocument.cpp](D:/projects/visual_studio_2026/G-code_post-processing_system/src/CadDocument.cpp) 中 `CadDocument::saveDxfDocument()` 与 `CadDocument::eportDxfDocument()` 仍为空实现
- 仓库没有独立自动化测试工程，验证仍以手工构建和手工交互检查为主

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

建议优先使用 `Debug|x64` 进行日常开发和交互验证。

## 仓库结构

```text
G-code_post-processing_system/
|-- include/                                # 公共头文件
|   |-- Gcode_postprocessing_system.h       # 主窗口类声明
|   |-- CadViewer.h                         # OpenGL 视图主类
|   |-- CadController.h                     # 输入控制器
|   |-- DrawStateMachine.h                  # 绘图/编辑状态机
|   |-- CadEditer.h                         # 编辑器与命令栈接口
|   |-- CadDocument.h                       # 文档模型
|   |-- CadBitmapImportDialog.h             # 位图导入参数对话框
|   |-- CadBitmapVectorizer.h               # 位图预处理与矢量化拟合
|   |-- GProfile.h                          # G 代码 Profile 配置模型
|   |-- GGenerator.h                        # G 代码生成器
|   |-- CadItem.h			               # 图元基类
|   |-- CadLineItem.h                       # 直线图元
|   |-- CadCircleItem.h                     # 圆图元
|   |-- CadArcItem.h                        # 圆弧图元
|   |-- CadEllipseItem.h                    # 椭圆图元
|   |-- CadPointItem.h                      # 点图元
|   |-- CadPolylineItem.h                   # 多段线图元
|   |-- CadLWPolylineItem.h                 # 轻量多段线图元
|   |-- CadCommandLineWidget.h              # 命令栏组件
|   |-- CadStatusPaneWidget.h               # 状态栏组件
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
|   |-- GGenerator.cpp                      # 纯 2D G 代码生成
|   |-- CadItem.cpp						  # 图元基类
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

项目当前按“主窗口 + Viewer + Controller + Editer + Document + 渲染协调层 + DXF Adapter + Bitmap Vectorizer + G-code Backend”的组合结构组织。相比早期版本，`CadViewer` 内部职责已经进一步拆分给 `CadSceneCoordinator`、`CadGraphicsCoordinator`、`CadEntityRenderer`、`CadEntityPicker` 等子模块。

```text
用户输入 / Qt 菜单 / 拖拽文件 / 键盘鼠标
        |
        v
Gcode_postprocessing_system                (窗口层 / 组装层)
        |
        +------> CadCommandLineWidget      (命令栏显示)
        +------> CadStatusPaneWidget       (坐标状态显示)
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
- 统一处理文件导入分发
- 持有 `CadDocument` 与 `CadEditer`

`CadViewer`

- 负责 `QOpenGLWidget` 生命周期和事件入口
- 管理相机、场景刷新、坐标转换
- 将键鼠事件转交 `CadController`
- 消费 `CadDocument` 数据并驱动渲染

`CadController` + `DrawStateMachine`

- 解释鼠标、键盘、滚轮输入
- 分流视图命令、绘图命令和编辑命令
- 维护当前命令状态、控制点、提示文本
- 处理多段线圆弧/直线输入切换

`CadEditer`

- 根据状态机创建新实体
- 执行删除、移动、改色
- 维护 Undo / Redo 命令栈
- 将模型修改统一提交给 `CadDocument`

`CadDocument`

- 持有原始 `dx_data`
- 持有场景中的 `CadItem` 容器
- 把导入得到的 `DRW_Entity` 转换成内部图元
- 通过 `sceneChanged` 驱动视图刷新

`CadBitmapImportDialog` + `CadBitmapVectorizer`

- 负责位图导入参数配置、预览和错误提示
- 使用 OpenCV 完成灰度化、阈值化、形态学处理、轮廓提取
- 将轮廓拟合为点、线、圆、圆弧、椭圆、多段线等实体
- 将生成实体追加或替换到当前文档

`GProfile` + `GGenerator`

- `GProfile` 负责读取、保存 JSON 格式的 G 代码配置
- 当前配置范围包括文件头尾、实体类型头尾、实体颜色头尾
- `GGenerator` 直接解析现有 `CadItem` 子类和原始 `DRW_Entity` 参数生成 G 代码
- 当前仅实现纯 `2D` 激光加工输出，`3D` 模式暂未实现

`CadSceneCoordinator` + `CadGraphicsCoordinator`

- `CadSceneCoordinator` 负责文档绑定、边界计算、GPU 缓冲重建时机
- `CadGraphicsCoordinator` 负责 OpenGL 状态、Shader、网格、坐标轴和 Overlay 渲染
- `CadEntityRenderer` 负责实体绘制
- `CadEntityPicker` 负责屏幕空间拾取
- `CadPreviewBuilder` / `CadCrosshairBuilder` 负责 transient 预览

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

1. 用户通过快捷键进入绘图或编辑命令
2. `CadController` 更新 `DrawStateMachine`
3. 鼠标位置被投影到 `Z=0` 绘图平面
4. `CadEditer` 根据当前状态创建或修改 `DRW_Entity`
5. `CadDocument` 更新内部图元并发出 `sceneChanged`
6. Viewer 自动刷新，Undo / Redo 栈按事务规则更新

### G 代码生成

1. `GGenerator` 绑定当前 `CadDocument`
2. 生成时通过 `GProfile` 读取文件头尾、实体类型头尾、实体颜色头尾配置
3. `GGenerator` 直接解析 `CadItem` 对应的原始实体参数
4. 当前 `2D` 输出支持：
   `Line -> G01`
   `Arc/Circle -> G02/G03 + I/J`
   `Polyline/LWPolyline -> 直线段 + bulge 圆弧段`
   `Ellipse -> 离散为 G01`
5. 生成器在导出时弹出文件保存对话框并输出 `.nc/.gcode/.txt`

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
- 滚轮：以鼠标附近为锚点缩放

### 绘图命令

- `P`：点
- `L`：直线
- `C`：圆
- `A`：圆弧
- `E`：椭圆
- `O`：多段线
- `W`：轻量多段线
- `Esc`：取消当前命令

多段线命令补充：

- `A`：切换为圆弧段输入
- `L`：切回直线段输入
- `Enter` / `Space`：结束多段线
- `C`：闭合多段线

### 编辑命令

- `Delete`：删除当前选中图元
- `M`：移动当前选中图元
- `K`：修改当前选中图元颜色
- `Ctrl + Z`：撤销
- `Ctrl + Y`：重做
- `Ctrl + Shift + Z`：重做

### G 代码生成说明

当前后处理后端已经具备以下能力：

- 支持纯 `2D` G 代码生成
- 支持 `Line`、`Arc`、`Circle`、`Ellipse`、`Polyline`、`LWPolyline`
- 支持读取 `CadItem::m_processOrder` 作为导出顺序
- 支持读取 `CadItem::m_isReverse` 作为反向加工标记
- 支持按 `GProfile` 套用文件头尾、实体类型头尾、实体颜色头尾

当前限制：

- 尚未绑定到主窗口菜单动作
- `Point` 当前不会输出加工轨迹
- `Ellipse` 当前通过折线离散输出，不使用专门椭圆插补指令
- `3D` 生成模式尚未实现

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

## 手工验证建议

当前仓库没有自动化测试工程。每次修改后建议至少完成以下检查：

1. 成功构建 `Debug|x64`
2. 导入一个包含点、线、圆、圆弧、椭圆、多段线的 DXF 文件
3. 导入一张简单黑白位图，验证预处理预览、矢量化结果和图层参数是否符合预期
4. 验证平移、缩放、顶视图、轨道观察、拾取是否正常
5. 验证绘图命令、移动、删除、改色、Undo / Redo 是否正常
6. 载入一份 `GProfile` 配置并验证 JSON 读写是否正常
7. 调用 `GGenerator` 导出一份纯 `2D` G 代码，检查头尾、类型配置、颜色配置和圆弧方向是否符合预期
8. 验证命令栏提示、状态栏坐标、拖拽导入是否正常

## 已知注意事项

- 当前内部仅完整支持 7 类 CAD 图元的可视化与编辑
- 新建图元强制落在 `Z=0` 平面
- 位图导入依赖 OpenCV 运行时 DLL 拷贝到输出目录
- `GGenerator` 当前仅实现纯 `2D` 输出，`3D` 模式未完成
- `GGenerator` 目前还是后端能力，尚未和主窗口导出菜单绑定
- `Ellipse` 当前按折线离散导出，精度取决于固定采样密度
- 第三方 `libdxfrw` 目录不应轻易修改
- 工程里仍存在部分历史遗留命名，修改构建配置前需要先核对 `.ui`、`.vcxproj` 与当前源码是否一致

## 协作约束摘要

- 默认使用中文沟通
- 遵循 MVC / 分层思路，避免把过多逻辑重新堆回 `CadViewer`
- 修改 `src/libdxfrw/` 或 `include/libdxfrw/` 时应明确说明原因和影响范围
- 文本文件统一保持 Windows `CRLF`
- OpenGL 保持 4.5 Core Profile
