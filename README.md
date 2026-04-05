[G-M代码指导]: ./technical_file/G-M_Code.md

目前支持7种图元类型：[点、直线、圆、圆弧、椭圆、轻量多段线、多段线]
## 项目结构

```text
G-code_post-processing_system/
|-- include/                     # 公共头文件
|   |-- Cad*.h / Cad*.hpp        # CAD 图元、文档与视图相关声明
|   |-- dx_iface.h               # DXF 读取接口封装
|   |-- dx_data.h                # DXF 中间数据结构
|   `-- libdxfrw/                # 第三方 libdxfrw 头文件
|-- src/                         # 主要 C++ 实现
|   |-- main.cpp                 # 程序入口
|   |-- Gcode_postprocessing_system.cpp
|   |-- CadViewer.cpp            # 视图与交互实现
|   |-- CadDocument.cpp          # 文档/场景数据实现
|   |-- Cad*Item.cpp             # 各类 CAD 图元实现
|   |-- dx_iface.cpp             # DXF 解析接入层
|   `-- libdxfrw/                # 第三方 libdxfrw 源码
|       `-- intern/              # libdxfrw 内部实现
|-- technical_file/              # 技术资料与 G/M 代码说明
|-- Gcode_postprocessing_system.ui
|                                # Qt Widgets 界面定义
|-- Gcode_postprocessing_system.qrc
|                                # Qt 资源清单
|-- G-code_post-processing_system.vcxproj
|                                # Visual Studio / Qt 工程文件
|-- G-code_post-processing_system.slnx
|                                # Visual Studio 解决方案
|-- AGENTS.md                    # 仓库协作约束
|-- README.md                    # 项目说明
|-- x64/                         # 构建输出目录，不提交
`-- .vs/                         # IDE 状态目录，不提交
```

## 模块划分

- `CadViewer`、`Gcode_postprocessing_system`：界面层与交互入口，负责窗口、视图和用户操作。
- `CadDocument`、`Cad*Item`：模型层，负责 CAD 图元数据及其绘制对象组织。
- `dx_iface`、`dx_data`：DXF 读取适配层，负责把 `libdxfrw` 的解析结果转换为本项目可用的数据结构。
- `src/libdxfrw/`、`include/libdxfrw/`：第三方 DXF/DWG 解析库源码与头文件，原则上不随意改动。

## 系统组织架构

系统整体采用“界面层 -> 视图层 -> 文档模型层 -> 数据导入层”的组织方式，核心目标是把 CAD 文件导入、三维图元组织、OpenGL 显示与后续 G-code 后处理能力解耦。

### 分层关系

```text
用户操作 / Qt Widgets UI
        |
        v
Gcode_postprocessing_system
        |
        v
CadViewer
        |
        v
CadDocument
        |
        v
CadItem / CadLineItem / CadArcItem / ...
        ^
        |
dx_iface / dx_data
        |
        v
libdxfrw
```

### 各层职责

- 界面层：由 `Gcode_postprocessing_system` 和 `.ui` 文件组成，负责菜单、导入动作、窗口布局和用户命令入口。
- 视图层：由 `CadViewer` 负责，承担 OpenGL 渲染、视角变换、点选、高亮、网格与坐标轴显示。
- 文档层：由 `CadDocument` 负责，统一管理当前场景中的 CAD 图元集合，是界面和数据之间的中枢对象。
- 图元层：由 `CadItem` 基类及各个 `Cad*Item` 派生类组成，负责不同 CAD 实体的几何离散、颜色、加工方向等对象级数据。
- 导入适配层：由 `dx_iface` 和 `dx_data` 负责，把 `libdxfrw` 的 DXF/DWG 解析结果转换成项目内部的 `CadDocument + CadItem` 结构。
- 第三方解析层：`libdxfrw` 提供底层文件解析能力，本项目原则上只在适配层消费其输出，不直接把第三方类型暴露给视图层。

### 典型工作流

1. 用户在界面层触发文件导入。
2. `Gcode_postprocessing_system` 调用 `dx_iface` 读取 DXF/DWG。
3. `dx_iface` 基于 `libdxfrw` 解析结果构建 `CadDocument` 和各类 `Cad*Item`。
4. `CadViewer` 接收 `CadDocument`，为各图元建立 GPU 缓冲并完成视图显示。
5. 用户继续进行缩放、平移、旋转、点选等交互，交互结果始终作用于 `CadViewer`，而图元数据仍由 `CadDocument` 统一持有。

### 架构特点

- 模型数据与渲染实现分离，便于后续把 CAD 几何继续送入 G-code 路径规划或工艺分析模块。
- 图元以独立类建模，便于逐类扩展三维几何、加工属性和后处理策略。
- 视图层只依赖项目内部文档模型，不直接依赖 `libdxfrw`，降低第三方库变更对上层交互逻辑的影响。
- 当前状态机式视图控制已经为后续扩展顶视、三维轨道、水平视图等专用观察模式预留了结构空间。

## 当前代码特征

- 当前仓库以 Qt 6 Widgets 桌面程序为主，工程由 Visual Studio 2026 + QtMsBuild 管理。
- 代码组织基本遵循 MVC 思路，Visual Studio 工程筛选器中也按 `View`、`Model`、`Item` 做了分组。
- 除手写业务代码外，仓库还保留了 `technical_file/G-M_Code.md` 作为后续 G-code 相关说明文档入口。
