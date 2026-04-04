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

## 当前代码特征

- 当前仓库以 Qt 6 Widgets 桌面程序为主，工程由 Visual Studio 2026 + QtMsBuild 管理。
- 代码组织基本遵循 MVC 思路，Visual Studio 工程筛选器中也按 `View`、`Model`、`Item` 做了分组。
- 除手写业务代码外，仓库还保留了 `technical_file/G-M_Code.md` 作为后续 G-code 相关说明文档入口。
