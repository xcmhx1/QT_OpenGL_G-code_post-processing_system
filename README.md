# G-code Post Processing System

[G-M代码指导]: ./technical_file/G-M_Code.md

本项目是基于 Qt 6 Widgets、OpenGL 4.5 Core Profile、Visual Studio 2026 的 Windows 桌面 CAD / G-code 后处理程序。当前重点能力包括：

- DXF / DWG 文件导入与 OpenGL 显示
- 7 类基础 CAD 图元的离散化显示
- 顶视图 / 轨道观察 / 平移 / 缩放 / 拾取
- 视图内拖拽 `.dxf` / `.dwg` 文件导入
- 底部状态栏坐标显示与命令历史栏
- 绘图与移动过程中的 transient / 橡皮筋预览
- 基于 MVC 思路的文档模型、控制器、视图分层
- 基于命令模式的绘图创建、删除、移动、改色、Undo / Redo

当前支持的图元类型：

- 点 `Point`
- 直线 `Line`
- 圆 `Circle`
- 圆弧 `Arc`
- 椭圆 `Ellipse`
- 多段线 `Polyline`
- 轻量多段线 `LWPolyline`

当前绘图策略遵循 AutoCAD 风格的二维绘图约束：

- 所有新创建图元统一落在世界坐标 `Z=0` 平面
- 即使当前在 3D 观察角度下绘图，鼠标坐标也会先投影到 `Z=0` 平面再建模
- 已导入文件中的原始 Z 值仍会按文件内容显示，不会被强制改写

## 1. 仓库结构

```text
G-code_post-processing_system/
|-- include/
|   |-- pch.h                              # 预编译头
|   |-- Gcode_postprocessing_system.h      # 主窗口类声明
|   |-- DxfViewer.h                        # Viewer 包装类
|   |-- CadViewer.h                        # OpenGL 视图层
|   |-- CadController.h                    # 输入控制器
|   |-- CadCommandLineWidget.h / .hpp      # 命令历史栏组件
|   |-- CadStatusPaneWidget.h / .hpp       # 底部状态栏组件
|   |-- CadEditer.h / .hpp                 # 编辑器与命令栈接口
|   |-- DrawStateMachine.h                 # 绘图/编辑状态机
|   |-- CadDocument.h                      # 文档模型
|   |-- CadItem.h / .cpp                   # 图元基类
|   |-- CadLineItem.h                      # 直线图元声明
|   |-- CadCircleItem.h                    # 圆图元声明
|   |-- CadArcItem.h                       # 圆弧图元声明
|   |-- CadEllipseItem.h                   # 椭圆图元声明
|   |-- CadPointItem.h                     # 点图元声明
|   |-- CadPolylineItem.h                  # 多段线图元声明
|   |-- CadLWPolylineItem.h                # 轻量多段线图元声明
|   |-- dx_data.h                          # DXF 中间数据结构
|   |-- dx_iface.h                         # libdxfrw 接入层
|   `-- libdxfrw/                          # 第三方头文件
|       `-- intern/                        # 第三方内部头文件
|-- src/
|   |-- pch.cpp                            # 预编译头实现
|   |-- main.cpp                           # 程序入口
|   |-- Gcode_postprocessing_system.cpp    # 主窗口实现
|   |-- CadViewer.cpp                      # OpenGL 视图实现
|   |-- CadController.cpp                  # 交互控制逻辑实现
|   |-- CadCommandLineWidget.cpp           # 命令历史栏实现
|   |-- CadStatusPaneWidget.cpp            # 底部状态栏实现
|   |-- CadEditer.cpp                      # 编辑器、命令、绘图创建实现
|   |-- DrawStateMachine.cpp               # 状态机默认值与辅助逻辑
|   |-- CadDocument.cpp                    # 文档模型实现
|   |-- CadLineItem.cpp                    # 直线离散化
|   |-- CadCircleItem.cpp                  # 圆离散化
|   |-- CadArcItem.cpp                     # 圆弧离散化
|   |-- CadEllipseItem.cpp                 # 椭圆离散化
|   |-- CadPointItem.cpp                   # 点离散化
|   |-- CadPolylineItem.cpp                # 多段线离散化
|   |-- CadLWPolylineItem.cpp              # 轻量多段线离散化
|   |-- dx_iface.cpp                       # DXF 导入导出桥接
|   `-- libdxfrw/                          # 第三方源代码
|       `-- intern/                        # 第三方内部实现
|-- technical_file/
|   `-- G-M_Code.md                        # G/M 代码相关资料
|-- Gcode_postprocessing_system.ui         # Qt Widgets UI
|-- Gcode_postprocessing_system.qrc        # Qt 资源文件
|-- G-code_post-processing_system.vcxproj  # VS/Qt 工程
|-- G-code_post-processing_system.slnx     # 解决方案
|-- AGENTS.md                              # 仓库协作规则
|-- README.md                              # 当前文档
|-- x64/                                   # 构建输出，不提交
`-- .vs/                                   # IDE 状态，不提交
```

## 2. 当前总体架构

项目整体按 MVC 思路组织，但结合 Qt/OpenGL 的实际落地，使用的是“主窗口 + Viewer + Controller + Editer + Document + Item + DXF Adapter”的组合结构。

```text
用户输入 / Qt 菜单 / 键盘鼠标
        |
        v
Gcode_postprocessing_system               (窗口层 / 组装层)
        |
        v
CadViewer                                 (View)
        |
        +------> CadController            (Controller)
        |              |
        |              +------> DrawStateMachine
        |              |
        |              +------> CadEditer
        |                              |
        |                              +------> EditCommand 栈
        |
        v
CadDocument                               (Model)
        |
        +------> CadItem / Cad*Item
        |
        +------> dx_data / dx_iface
                        |
                        v
                     libdxfrw
```

### 2.1 各层职责

#### 主窗口层

`Gcode_postprocessing_system`

- 负责 UI 初始化
- 负责菜单动作绑定
- 负责把 `CadDocument`、`CadEditer`、`CadViewer` 组装在一起
- 当前导入文件后会清空编辑历史并刷新 Viewer

#### 视图层

`CadViewer`

- 负责 OpenGL 上下文和着色器初始化
- 负责网格、坐标轴、轨道中心、实体绘制
- 负责 transient 预览层绘制
- 负责屏幕坐标与世界坐标互转
- 负责实体拾取和高亮
- 负责拖拽导入入口和坐标 / 命令提示信号发出
- 不直接改业务数据，只消费 `CadDocument`
- 通过 `sceneChanged` 自动重建 GPU 缓冲

#### 控制器层

`CadController`

- 负责接收鼠标、键盘、滚轮事件
- 负责视图交互命令和绘图命令分流
- 维护 `DrawStateMachine`
- 在左键绘图时先把鼠标位置投影到 `Z=0` 平面
- 将创建 / 删除 / 移动 / 改色 / undo / redo 等操作转交 `CadEditer`

#### 编辑器层

`CadEditer`

- 接收 Controller 发起的编辑命令
- 根据 `DrawStateMachine` 的阶段信息完成图元创建
- 提供删除、移动、改色等模型操作
- 通过命令模式维护 undo / redo 栈
- 当前内置命令类型包括：
  - `AddEntityCommand`
  - `DeleteEntityCommand`
  - `MoveEntityCommand`
  - `ChangeColorCommand`

#### 状态机层

`DrawStateMachine`

- 维护当前是否在绘图
- 维护当前图元类型和颜色
- 维护点、线、圆、圆弧、椭圆、多段线的子状态
- 维护移动命令的编辑子状态
- 维护命令过程中采集的控制点 `commandPoints`
- 维护鼠标屏幕坐标、世界坐标、按钮和修饰键

#### 文档模型层

`CadDocument`

- 持有 `dx_data`
- 持有场景中的 `CadItem` 容器
- 从导入的原始 `DRW_Entity` 构建内部图元对象
- 对外提供统一的：
  - `appendEntity`
  - `takeEntity`
  - `refreshEntity`
  - `containsEntity`
- 通过 `sceneChanged` 通知 Viewer 重建显示

#### 图元层

`CadItem` + `Cad*Item`

- `CadItem` 是抽象基类
- 每个派生类负责把一个 DXF 实体离散为 `GeometryData.vertices`
- 负责颜色解析
- 负责加工方向向量生成

当前主要派生类：

- `CadPointItem`
- `CadLineItem`
- `CadCircleItem`
- `CadArcItem`
- `CadEllipseItem`
- `CadPolylineItem`
- `CadLWPolylineItem`

#### DXF 适配层

`dx_iface` + `dx_data`

- `dx_iface` 负责实现 `DRW_Interface`
- `dx_data` 负责保存导入后的头、图层、实体、块等数据
- `CadDocument` 再把 `dx_data->mBlock->ent` 转成 `CadItem`

#### 第三方库层

`libdxfrw`

- 负责底层 DXF / DWG 读写
- 本项目原则上只通过 `dx_iface` / `dx_data` 与其交互
- 非必要不要直接修改 `src/libdxfrw/` 和 `include/libdxfrw/`

## 3. 关键类速查

### 3.1 视图和交互

- `Gcode_postprocessing_system`
  - 主窗口
  - 导入菜单入口
- `DxfViewer`
  - `CadViewer` 的轻量包装，供 `.ui` 使用
- `CadViewer`
  - 场景显示、相机、拾取、OpenGL 缓冲
- `CadController`
  - 输入解释层
- `DrawStateMachine`
  - 绘图 / 编辑命令状态容器
- `CadEditer`
  - 命令执行与撤销重做层

### 3.2 模型与图元

- `CadDocument`
  - 当前场景数据容器
- `CadItem`
  - 通用图元基类
- `CadPointItem`
  - 点
- `CadLineItem`
  - 线
- `CadCircleItem`
  - 圆
- `CadArcItem`
  - 圆弧
- `CadEllipseItem`
  - 椭圆
- `CadPolylineItem`
  - 多段线
- `CadLWPolylineItem`
  - 轻量多段线

### 3.3 DXF 数据与适配

- `dx_data`
  - 导入后数据总表
- `dx_ifaceBlock`
  - 模型空间 / 块空间实体容器
- `dx_iface`
  - 读写桥接实现

## 4. 核心数据流

### 4.1 导入流程

1. 用户在主窗口触发“导入 Dxf”，或直接把 `.dxf/.dwg` 拖入 `CadViewer`
2. `Gcode_postprocessing_system` 调用 `CadDocument::readDxfDocument`
3. `CadDocument` 内部通过 `dx_iface` 使用 `libdxfrw` 解析文件
4. `dx_data->mBlock->ent` 收集所有原始 `DRW_Entity`
5. `CadDocument::init` 将其转换为 `CadItem`
6. `CadDocument` 发出 `sceneChanged`
7. `CadViewer` 重建缓冲并显示实体

### 4.2 绘图流程

1. 用户通过键盘命令进入绘图模式
2. `CadController` 更新 `DrawStateMachine`
3. 鼠标点击时，Controller 将屏幕射线投影到 `Z=0` 平面
4. `CadEditer` 根据当前子状态收集控制点
5. 当控制点足够时，`CadEditer` 构造新的 `DRW_Entity`
6. 新实体进入 `CadDocument`
7. `CadDocument` 发出 `sceneChanged`
8. `CadViewer` 自动刷新
9. 该操作作为命令压入 undo 栈

### 4.3 编辑流程

1. 用户选中图元
2. 触发删除 / 移动 / 改色命令
3. `CadController` 把命令转给 `CadEditer`
4. `CadEditer` 执行命令对象
5. `CadDocument` 更新模型并发出 `sceneChanged`
6. `CadViewer` 重建显示
7. 命令进入 undo 栈，redo 栈按标准事务规则清空

### 4.4 transient 预览与命令提示流程

1. `CadController` 在鼠标移动时持续更新 `DrawStateMachine.currentPos`
2. `CadViewer` 根据当前状态即时构造 transient 图元
3. transient 图元只参与渲染，不写入 `CadDocument`
4. 命令完成或取消后，transient 图元立即消失
5. `CadViewer` 同时发出：
   - 当前鼠标工作平面坐标
   - 当前命令 prompt
   - 最新 command message
6. `CadCommandLineWidget` 与 `CadStatusPaneWidget` 只负责显示，不参与业务决策

## 5. 当前交互与快捷键

### 5.1 视图

- `F`：适配场景
- `T` 或 `Home`：回顶视图
- `+`：放大
- `-`：缩小
- 中键拖动：平移
- `Shift + 中键拖动`：轨道旋转
- 左键：拾取图元

### 5.2 绘图

- `P`：点
- `L`：直线
- `C`：圆
- `A`：圆弧
- `E`：椭圆
- `O`：多段线
- `W`：轻量多段线
- 多段线 / 轻量多段线绘制中 `A`：切换为圆弧段输入
- 多段线 / 轻量多段线绘制中 `L`：切回直线段输入
- `Enter` / `Space`：结束多段线
- 多段线模式下 `C`：闭合多段线
- `Esc`：取消当前命令

多段线圆弧段当前采用“直接指定圆弧终点”的方式输入：

- 需先有至少一段前置线段或圆弧段，系统会沿前一段终点切线自动续接
- Viewer 会显示圆弧段的橡皮筋预览
- 最终数据写入 `Polyline / LWPolyline` 的 `bulge` 字段，而不是拆成独立 `Arc`

### 5.3 编辑

- `Delete`：删除当前选中图元
- `M`：移动当前选中图元
- `K`：修改当前选中图元颜色
- `Ctrl + Z`：撤销
- `Ctrl + Y`：重做
- `Ctrl + Shift + Z`：重做

### 5.4 UI 状态显示

- 底部状态栏显示当前鼠标对应的世界坐标块 `X / Y / Z`
- 状态栏右侧预留了吸附 / 正交 / 极轴等扩展位置
- 命令栏位于底部状态栏上方
- 平时只显示一行最新消息
- 点击命令栏后展开为四行高度，并显示带滚动条的历史消息
- 点击其它区域后自动收起

## 6. 绘图平面与 Z 轴策略

当前绘图功能按二维 CAD 方式处理，规则如下：

- 所有新建图元都写入世界坐标 `Z=0`
- 点、线、圆、圆弧、椭圆、多段线、轻量多段线全部遵循这一规则
- 控制器会先求“屏幕射线与 `Z=0` 平面”的交点
- 编辑器中的创建函数还会再次把坐标压到 `Z=0`，作为兜底
- 这样即使当前相机处于 3D 视图，新绘图元也不会“高高在上”

这部分是显式设计，不是偶然副作用。后续如果要支持真正的 3D 绘图，应单独引入：

- 用户工作平面 `UCS/WCS`
- 明确的绘图基准高程
- 3D 捕捉或构造平面

在未做这些扩展前，不应把当前二维绘图逻辑改回任意 Z 深度。

## 7. 主要源码文件说明

### 7.1 顶层窗口

- `src/Gcode_postprocessing_system.cpp`
  - 创建文档、编辑器、Viewer
  - 绑定导入动作

### 7.2 视图层

- `src/CadViewer.cpp`
  - OpenGL 初始化
  - 网格、坐标轴、实体绘制
  - transient 橡皮筋预览
  - 屏幕拾取
  - 视图拖拽导入
  - 坐标与命令提示信号
  - 文档变化后缓冲重建

### 7.3 控制器层

- `src/CadController.cpp`
  - 键盘鼠标解释
  - 绘图平面投影
  - 命令分发到 `CadEditer`
  - 命令 prompt 与 message 触发

### 7.3A UI 组件层

- `src/CadCommandLineWidget.cpp`
  - 命令栏折叠 / 展开
  - 最新消息与历史消息显示
- `src/CadStatusPaneWidget.cpp`
  - 当前鼠标工作平面坐标显示
  - 后续状态开关位预留

### 7.4 编辑器层

- `src/CadEditer.cpp`
  - 图元创建
  - 删除 / 移动 / 改色
  - Undo / Redo
  - 命令对象定义

### 7.5 文档层

- `src/CadDocument.cpp`
  - DXF 导入
  - `DRW_Entity` 到 `CadItem` 的转换
  - 文档实体增删改

### 7.6 图元层

- `src/CadLineItem.cpp`
- `src/CadCircleItem.cpp`
- `src/CadArcItem.cpp`
- `src/CadEllipseItem.cpp`
- `src/CadPointItem.cpp`
- `src/CadPolylineItem.cpp`
- `src/CadLWPolylineItem.cpp`

这些文件只应关注各图元的几何离散逻辑，不应直接承担窗口交互和业务命令职责。

## 8. 构建方式

### 8.1 Visual Studio

```powershell
devenv .\G-code_post-processing_system.slnx
```

### 8.2 MSBuild

```powershell
msbuild .\G-code_post-processing_system.vcxproj /p:Configuration=Debug /p:Platform=x64
msbuild .\G-code_post-processing_system.vcxproj /p:Configuration=Release /p:Platform=x64
```

### 8.3 环境要求

- Visual Studio 2026
- Qt `6.9.3_msvc2022_64`
- QtMsBuild 可用
- Windows x64

## 9. 当前已知约束

- 当前绘图只支持 `Z=0` 平面，不支持真正的 3D 建模
- 当前没有独立自动化测试工程，主要依赖手工验证
- `libdxfrw` 第三方头文件仍有既有编译警告，不属于本次业务层修改
- Viewer 使用 OpenGL 4.5 Core Profile，不允许退回固定管线写法

## 10. 后续扩展建议

后续如果继续迭代，建议优先按下面顺序扩展：

1. 增加对象捕捉、正交、极轴等状态开关并接到底部状态栏
2. 增加命令行文本输入，形成更接近 AutoCAD 的命令窗口
3. 增加导出 DXF / 保存修改后的文档
4. 增加图层、线型、颜色索引等更完整的 CAD 属性编辑
5. 增加自动化测试或最少的模型层回归测试
6. 当且仅当有明确需求时，再引入 3D 绘图工作平面体系

## 11. 协作约束摘要

- 默认使用中文沟通
- 保持 MVC 分层，不要把文档编辑逻辑塞入 Viewer
- 修改第三方目录 `src/libdxfrw/` 或 `include/libdxfrw/` 前先确认必要性
- 文本文件统一使用 Windows `CRLF`
- OpenGL 部分保持 4.5 Core Profile

## 12. 参考资料

- [G-M代码指导]
