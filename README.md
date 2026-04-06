# G-code Post Processing System

[G/M 代码参考](./technical_file/G-M_Code.md)

本项目是一个基于 Qt 6 Widgets、OpenGL 4.5 Core Profile、Visual Studio 2026 的 Windows 桌面 CAD / G-code 后处理程序。当前代码主线已经具备 DXF / DWG 导入、二维 CAD 图元显示、基础交互、简单绘图与编辑命令，但 G 代码导出链路仍未接通，项目整体仍处于 CAD 视图与编辑基础设施持续完善阶段。

## 项目现状

当前已实现：

- 导入 `.dxf` / `.dwg` 文件
- 使用 OpenGL 4.5 Core Profile 渲染 CAD 场景
- 支持点、直线、圆、圆弧、椭圆、多段线、轻量多段线的内部图元构建与显示
- 视图平移、缩放、顶视图切换、轨道观察、屏幕拾取
- 鼠标拖拽文件到视图区域直接导入
- 命令提示栏、命令历史栏、底部坐标状态栏
- 点/线/圆/圆弧/椭圆/多段线/轻量多段线的交互式绘制
- 删除、移动、改色、Undo / Redo
- transient 预览、十字光标叠加、选中高亮

当前未完成或未接线：

- `Gcode_postprocessing_system.ui` 中的“导出 G 代码”“反向加工”等菜单项目前没有业务绑定
- `CadDocument::saveDxfDocument()` 与 `CadDocument::eportDxfDocument()` 目前为空实现
- 仓库没有独立自动化测试工程，验证仍以手工构建和手工交互检查为主

## 技术栈

- C++17
- Qt `6.9.3_msvc2022_64`
- Qt Widgets
- OpenGL 4.5 Core Profile
- Visual Studio 2026 / MSBuild
- `libdxfrw` 作为 DXF / DWG 读写底层库

图形相关约束：

- 视图层基于 `QOpenGLFunctions_4_5_Core`
- 着色器与渲染路径按 OpenGL 4.5 Core Profile 设计
- 不应回退到旧式固定管线写法

## 快速开始

### 环境要求

- Windows x64
- Visual Studio 2026
- Qt `6.9.3_msvc2022_64`
- QtMsBuild 已正确安装并可被 Visual Studio 识别

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
|   |-- pch.h                               # 预编译头
|   |-- Gcode_postprocessing_system.h       # 主窗口类声明
|   |-- CadViewer.h                         # OpenGL 视图主类
|   |-- CadController.h                     # 输入控制器
|   |-- DrawStateMachine.h                  # 绘图/编辑状态机
|   |-- CadEditer.h                         # 编辑器与命令栈接口
|   |-- CadDocument.h                       # 文档模型
|   |-- CadItem.h / CadItem.cpp             # 图元基类
|   |-- CadLineItem.h / .hpp                # 直线图元
|   |-- CadCircleItem.h / .hpp              # 圆图元
|   |-- CadArcItem.h / .hpp                 # 圆弧图元
|   |-- CadEllipseItem.h / .hpp             # 椭圆图元
|   |-- CadPointItem.h / .hpp               # 点图元
|   |-- CadPolylineItem.h / .hpp            # 多段线图元
|   |-- CadLWPolylineItem.h / .hpp          # 轻量多段线图元
|   |-- CadCommandLineWidget.h / .hpp       # 命令栏组件
|   |-- CadStatusPaneWidget.h / .hpp        # 状态栏组件
|   |-- CadGraphicsCoordinator.h            # 渲染协调层
|   |-- CadSceneCoordinator.h               # 场景协调层
|   |-- CadSceneContext.h                   # 场景上下文
|   |-- CadSceneRenderCache.h               # GPU 缓冲缓存
|   |-- CadEntityRenderer.h                 # 实体绘制器
|   |-- CadEntityPicker.h                   # 屏幕拾取器
|   |-- CadOverlayRenderer.h                # overlay 绘制
|   |-- CadReferenceRenderer.h              # 网格/坐标轴等参考图形绘制
|   |-- CadShaderManager.h                  # shader 管理
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
|   |-- DxfDocument.hpp                     # 历史遗留兼容头
|   |-- DxfViewer.h                         # 工程/UI 中仍引用的历史包装头
|   `-- libdxfrw/                           # 第三方头文件
|       `-- intern/                         # 第三方内部头文件
|-- src/                                    # 主要源码
|   |-- pch.cpp                             # 预编译头实现
|   |-- main.cpp                            # 程序入口
|   |-- Gcode_postprocessing_system.cpp     # 主窗口实现
|   |-- CadViewer.cpp                       # 视图与 OpenGL 主流程
|   |-- CadController.cpp                   # 输入解释与快捷键分发
|   |-- DrawStateMachine.cpp                # 状态机默认逻辑
|   |-- CadEditer.cpp                       # 绘图、编辑、命令栈实现
|   |-- CadDocument.cpp                     # 文档模型实现
|   |-- CadLineItem.cpp                     # 直线几何离散
|   |-- CadCircleItem.cpp                   # 圆几何离散
|   |-- CadArcItem.cpp                      # 圆弧几何离散
|   |-- CadEllipseItem.cpp                  # 椭圆几何离散
|   |-- CadPointItem.cpp                    # 点几何离散
|   |-- CadPolylineItem.cpp                 # 多段线几何离散
|   |-- CadLWPolylineItem.cpp               # 轻量多段线几何离散
|   |-- CadCommandLineWidget.cpp            # 命令栏实现
|   |-- CadStatusPaneWidget.cpp             # 状态栏实现
|   |-- CadGraphicsCoordinator.cpp          # 渲染协调实现
|   |-- CadSceneCoordinator.cpp             # 场景协调实现
|   |-- CadSceneContext.cpp                 # 场景上下文实现
|   |-- CadSceneRenderCache.cpp             # GPU 缓冲缓存实现
|   |-- CadEntityRenderer.cpp               # 实体绘制实现
|   |-- CadEntityPicker.cpp                 # 拾取实现
|   |-- CadOverlayRenderer.cpp              # overlay 渲染实现
|   |-- CadReferenceRenderer.cpp            # 参考图形渲染实现
|   |-- CadShaderManager.cpp                # shader 初始化实现
|   |-- CadCrosshairBuilder.cpp             # 十字光标构建实现
|   |-- CadPreviewBuilder.cpp               # transient 预览实现
|   |-- CadCamera.cpp / CadCameraMath.cpp   # 相机逻辑
|   |-- CadCameraUtils.cpp                  # 相机辅助计算
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

项目当前按“主窗口 + Viewer + Controller + Editer + Document + 渲染协调层 + DXF Adapter”的组合结构组织。相比早期版本，`CadViewer` 内部职责已经进一步拆分给 `CadSceneCoordinator`、`CadGraphicsCoordinator`、`CadEntityRenderer`、`CadEntityPicker` 等子模块。

```text
用户输入 / Qt 菜单 / 拖拽文件 / 键盘鼠标
        |
        v
Gcode_postprocessing_system                (窗口层 / 组装层)
        |
        +------> CadCommandLineWidget      (命令栏显示)
        |
        +------> CadStatusPaneWidget       (坐标状态显示)
        |
        v
CadViewer                                  (View)
        |
        +------> CadController             (输入解释)
        |              |
        |              +------> DrawStateMachine
        |              |
        |              +------> CadEditer
        |                              |
        |                              +------> EditCommand 栈
        |
        +------> CadSceneCoordinator       (文档绑定 / 包围盒 / 缓冲脏标记)
        |
        +------> CadGraphicsCoordinator    (OpenGL / Shader / 参考图形 / Overlay)
        |              |
        |              +------> CadEntityRenderer
        |              +------> CadEntityPicker
        |              +------> CadPreviewBuilder
        |              +------> CadCrosshairBuilder
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
```

### 分层职责

#### 主窗口层

`Gcode_postprocessing_system`

- 负责 UI 初始化与窗口装配
- 创建命令栏、状态栏并插入中心布局
- 绑定“导入 Dxf”菜单与拖拽导入信号
- 持有 `CadDocument` 与 `CadEditer`

#### 视图层

`CadViewer`

- 负责 `QOpenGLWidget` 生命周期和事件入口
- 管理相机、场景刷新、坐标转换
- 将键鼠事件转交 `CadController`
- 消费 `CadDocument` 数据并驱动渲染

#### 输入控制层

`CadController` + `DrawStateMachine`

- 解释鼠标、键盘、滚轮输入
- 分流视图命令、绘图命令和编辑命令
- 维护当前命令状态、控制点、提示文本
- 处理多段线圆弧/直线输入切换

#### 编辑器层

`CadEditer`

- 根据状态机创建新实体
- 执行删除、移动、改色
- 维护 Undo / Redo 命令栈
- 将模型修改统一提交给 `CadDocument`

#### 文档模型层

`CadDocument`

- 持有原始 `dx_data`
- 持有场景中的 `CadItem` 容器
- 把导入得到的 `DRW_Entity` 转换成内部图元
- 通过 `sceneChanged` 驱动视图刷新

#### 渲染协调层

`CadSceneCoordinator` + `CadGraphicsCoordinator`

- `CadSceneCoordinator` 负责文档绑定、边界计算、GPU 缓冲重建时机
- `CadGraphicsCoordinator` 负责 OpenGL 状态、shader、网格、坐标轴和 overlay 渲染
- `CadEntityRenderer` 负责实体绘制
- `CadEntityPicker` 负责屏幕空间拾取
- `CadPreviewBuilder` / `CadCrosshairBuilder` 负责 transient 预览

#### DXF 适配层

`dx_iface` + `dx_data`

- `dx_iface` 实现 `DRW_Interface`
- `dx_data` 保存头、图层、块、实体等原始解析结果
- `CadDocument` 再将模型空间实体转换为内部图元

### 核心数据流

#### 导入流程

1. 用户通过菜单或拖拽方式导入 `.dxf` / `.dwg`
2. `Gcode_postprocessing_system` 调用 `CadDocument::readDxfDocument()`
3. `CadDocument` 通过 `dx_iface` 使用 `libdxfrw` 读取文件
4. 原始实体进入 `dx_data->mBlock->ent`
5. `CadDocument::init()` 将支持的实体转换为 `CadItem`
6. `CadDocument` 发出 `sceneChanged`
7. `CadViewer` 通过 `CadSceneCoordinator` / `CadGraphicsCoordinator` 重建缓冲并刷新显示

#### 绘图流程

1. 用户通过快捷键进入绘图命令
2. `CadController` 更新 `DrawStateMachine`
3. 鼠标点击位置被投影到 `Z=0` 绘图平面
4. `CadEditer` 根据当前子状态采集控制点并构造 `DRW_Entity`
5. 新实体进入 `CadDocument`
6. `CadDocument` 发出 `sceneChanged`
7. `CadViewer` 自动刷新，命令对象进入 Undo 栈

#### 编辑流程

1. 用户拾取实体并触发删除、移动、改色
2. `CadController` 将命令转给 `CadEditer`
3. `CadEditer` 执行具体命令对象
4. `CadDocument` 更新实体并重建内部图元几何
5. `CadDocument` 发出 `sceneChanged`
6. `CadViewer` 重建显示，Undo / Redo 栈按事务规则更新

## 使用说明

### 文件导入

- 菜单“文件 -> 导入Dxf”
- 直接将 `.dxf` 或 `.dwg` 文件拖入主视图

导入后会：

- 清空当前编辑历史
- 调用 `CadDocument::readDxfDocument()` 重新解析模型
- 自动刷新 Viewer
- 在命令栏和状态栏显示导入结果

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
- 但当前 `CadDocument::createCadItemForEntity()` 只会把上面 7 类实体转换为项目内部 `CadItem`
- 其它实体即使被读入，也不会生成当前可显示/可编辑的内部图元

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

## 绘图平面约束

当前绘图逻辑按二维 CAD 方式实现，所有新建图元统一落在世界坐标 `Z=0` 平面：

- `CadController` 会将鼠标位置投影到绘图平面
- `CadEditer` 在创建实体时再次将坐标压到 `Z=0`
- 即使当前视角处于轨道观察状态，新建图元也不会写入任意 Z 深度

这是一项显式设计。现阶段项目支持“二维绘制 + 三维观察”，不支持完整 3D 建模工作流。

## 渲染与显示子系统

围绕 `CadViewer` 的渲染子系统当前已经拆分为多个职责明确的模块：

- `CadGraphicsCoordinator`：初始化 OpenGL 状态、Shader、参考图元和叠加层渲染器
- `CadSceneCoordinator`：维护场景上下文、包围盒、GPU 缓冲重建时机
- `CadEntityRenderer`：实体绘制
- `CadEntityPicker`：屏幕空间拾取
- `CadPreviewBuilder`：命令过程中的 transient 图元预览
- `CadCrosshairBuilder`：十字光标叠加
- `CadViewTransform` / `CadCamera*`：视图变换、相机数学和观察控制

这意味着后续如果扩展显示功能，优先应在上述子模块中局部修改，而不是继续把逻辑堆回 `CadViewer.cpp`。

## UI 组件

### 命令栏

`CadCommandLineWidget`

- 默认显示最新消息或当前提示
- 点击后展开为历史模式
- 展开时显示 4 行高的只读历史记录
- 点击其它区域后自动收起

### 状态栏

`CadStatusPaneWidget`

- 实时显示当前鼠标对应的世界坐标
- 为“吸附 / 正交 / 极轴”等后续能力预留显示位置

## 手工验证建议

当前仓库没有自动化测试工程。每次修改后建议至少完成以下检查：

1. 成功构建 `Debug|x64`
2. 导入一个包含点、线、圆、圆弧、椭圆、多段线的 DXF 文件
3. 验证平移、缩放、顶视图、轨道观察、拾取是否正常
4. 验证绘图命令、移动、删除、改色、Undo / Redo 是否正常
5. 验证命令栏提示、状态栏坐标、拖拽导入是否正常

## 已知注意事项

- 当前内部仅完整支持 7 类 CAD 图元的可视化与编辑
- 新建图元强制落在 `Z=0` 平面
- 第三方 `libdxfrw` 目录不应轻易修改
- 工程里仍存在部分历史遗留命名与文件引用，修改构建配置前需要先核对 `.ui`、`.vcxproj` 与当前源码是否一致

## 协作约束摘要

- 默认使用中文沟通
- 遵循 MVC / 分层思路，不要把文档编辑逻辑塞进 Viewer
- 修改 `src/libdxfrw/` 或 `include/libdxfrw/` 时应明确说明原因和影响范围
- 文本文件统一保持 Windows `CRLF`
- OpenGL 保持 4.5 Core Profile
