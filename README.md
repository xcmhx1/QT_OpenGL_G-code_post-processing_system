# G-code Post-processing System

本文件是项目入口，面向使用者和维护者说明产品定位、主要能力、构建方式与文档入口。
详细业务要求见 [docs/requirements/index.md](./docs/requirements/index.md)。

## 1. 项目简介

G-code Post-processing System 是运行于 Windows 的桌面 CAD/CAM 软件。

软件使用 Qt Widgets 构建桌面界面，使用 OpenGL 完成 CAD 图形显示、选择和交互。
当前功能覆盖 DXF/DWG 输入、常用 CAD 编辑、加工属性设置、加工顺序规划和 G-code 导出。

系统提供两类主要加工场景：

- 平面三轴加工；
- 面向方管的绕 X 轴回转、A 轴展开四轴加工。

产品能力仅覆盖曲线类 CAD 编辑、平面三轴加工，以及当前支持的方管绕 X 轴回转、A 轴展开四轴加工。

## 2. 核心能力概览

### 文件与配置

- 导入 DXF 和 DWG 文件；
- 保存和另存为 DXF 文件；
- 提供兼容性优先的安全 DXF 导出；
- 导入位图并执行轮廓矢量化；
- 管理 G-code 配置文件、配置目录和最近使用路径；
- 支持 Lite/Pro 产品配置和本地许可证文件。

### CAD 图元

- 显示和处理点、直线、构造线、圆弧、圆、椭圆、多段线、轻量多段线和样条曲线；
- 创建直线、构造线、圆弧、圆、矩形、多边形和多段线等图形；
- 矩形和多边形以多段线保存；
- 多边形支持 3～1024 条边、内接和外切方式，并记忆上次边数；
- 普通 DXF 保存保留受支持图元的原始类型；
- 样条曲线可参与显示、保存和加工路径生成，但暂不提供完整的样条控制点编辑器。

### 编辑与交互

- 单选、窗口框选和交叉框选；
- 端点、中点、圆心、交点、控制点和网格捕捉；
- 光标旁动态输入；
- 正交和极轴辅助；
- 图层、颜色和属性管理；
- 控制点编辑；
- 移动、旋转、缩放、镜像和偏移；
- 矩形阵列和环形阵列；
- 修剪、延伸、合并、圆角和直角；
- 批量删除与批量 Undo/Redo；
- 平移、滚轮缩放、三维视角旋转、标准视角和视图方块；
- 浅色、深色和自定义外观；
- 网格、加工方向、加工序号、加工断面和排除状态显示。

### 加工

- 为图元设置是否加工、加工方向和加工起点；
- 对三轴加工图元进行排序；
- 识别或手动设置方管垂直截面和圆角信息；
- 识别并排除拓扑内部线和进入方管内部的路径；
- 指定加工断面和废面；
- 处理连续路径、闭合路径和跨表面路径；
- 执行方管四轴加工排序和轨迹生成；
- 生成安全移动、切割连接、连续 A 轴和闭合路径过切；
- 在画布中显示加工方向、加工顺序和排除状态。

### G-code

- 使用配置控制文件头尾、图层、颜色和图元类型规则；
- 支持三轴和方管四轴 G-code 导出；
- 支持安全高度、进给、旋转轴、加工修正和过切等参数；
- 支持按颜色规则进行常用工艺区分；
- 导出前检查当前加工计划与文档状态是否一致。

## 3. 快速开始

### 开发环境

| 项目 | 当前环境 |
| --- | --- |
| 操作系统 | Windows x64 |
| IDE | Visual Studio 2026 Insiders |
| IDE 路径 | `D:\Program Files\Microsoft Visual Studio\18\Insiders\Common7\IDE` |
| 工具集 | MSVC v145，C++17 |
| Qt | Qt 6.9.3，`msvc2022_64` |
| OpenCV | OpenCV 4.11.0 |
| 解决方案 | `G-code_post-processing_system.slnx` |

默认 OpenCV 安装位置为 `D:\develop\opencv`。
若本机路径不同，应在本机工程环境中调整依赖路径，不要提交个人绝对路径配置。

### Release 构建

在项目根目录执行：

```powershell
& "D:\Program Files\Microsoft Visual Studio\18\Insiders\MSBuild\Current\Bin\MSBuild.exe" `
  ".\G-code_post-processing_system.slnx" `
  /m /p:Configuration=Release /p:Platform=x64
```

构建输出目录：

```text
x64/Release/
```

Release 构建会生成主程序，并按当前工程设置部署运行所需的 Qt、OpenCV 和 MSVC 运行库文件。

## 4. 基本使用流程

### 三轴

```text
导入文件
→ 编辑图形
→ 设置加工属性
→ 执行排序
→ 检查加工顺序
→ 导出 G-code
```

三轴加工前应确认图元是否启用、加工方向、起点、配置规则和安全高度。
文档或加工状态发生变化后，应重新排序再导出。

### 方管四轴

```text
导入文件
→ 识别或手动设置方管截面
→ 处理内部线
→ 指定加工断面
→ 执行排序
→ 检查加工顺序
→ 导出 G-code
```

方管四轴加工前应确认截面尺寸、圆角、加工断面、废面、连续路径和 A 轴方向。
截面或加工状态过期时，应重新识别和排序。

## 5. 文档导航

- [项目文档](./docs/index.md)
- [业务需求](./docs/requirements/index.md)
- [产品范围](./docs/requirements/product-scope.md)
- [文件和 CAD](./docs/requirements/file-and-cad.md)
- [编辑与交互](./docs/requirements/editing-and-interaction.md)
- [加工功能](./docs/requirements/machining.md)
- [加工工艺](./docs/requirements/machining-process.md)
- [G-code 与导出](./docs/requirements/gcode-and-export.md)

`docs/requirements/` 描述系统在业务层面应该完成什么。
架构文档和实现文档将在需求审阅完成后的后续阶段建立。

## 6. 安全说明

本软件生成的 G-code 不应未经检查直接用于实际机床。

实际加工前必须：

- 核对工件尺寸、坐标系、旋转轴方向和机床配置；
- 检查加工顺序、安全高度、进退刀、激光或主轴启停；
- 使用仿真软件验证完整刀路；
- 在关闭加工能量或其他安全条件下执行空跑；
- 由具备机床操作经验的人员进行最终人工复核。

自动识别、自动排序和位图矢量化结果均需要人工确认。
配置文件必须与实际控制器、机床结构和现场工艺一致。
