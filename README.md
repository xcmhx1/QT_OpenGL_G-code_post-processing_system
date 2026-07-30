# G-code Post-processing System

## 1. 软件作用

G-code Post-processing System 是运行于 Windows 的桌面 CAD/CAM 软件，使用 Qt Widgets 构建界面，使用 OpenGL 完成 CAD 图形显示与交互。

软件用于读取 DXF/DWG、完成曲线类 CAD 编辑、设置加工业务输入、组织加工顺序并导出客制化 G-code。当前加工场景包括：

- 平面三轴加工；
- 绕 X 轴回转、使用 A 轴展开的方管四轴加工。

完整业务范围和功能边界由 [需求文档](./docs/requirements/index.md) 统一说明。

## 2. 典型流程

### 平面三轴

```text
导入文件
→ 编辑图形
→ 设置加工启用、方向、起点和顺序
→ 执行普通排序或智能排序
→ 查看加工顺序和方向
→ 导出 G-code
```

### 方管四轴

普通四轴流程仅要求有效加工路径、旋转轴中心、运动参数、加工顺序和当前配置：

```text
导入文件
→ 编辑图形
→ 设置加工业务输入
→ 执行普通排序或智能排序
→ 查看加工顺序和方向
→ 导出 G-code
```

需要方管外形相关能力时，可增加截面增强流程：

```text
识别或手动设置方管截面尺寸、圆角和截面中心
→ 处理方管实体内部路径
→ 指定加工断面或废面
→ 选择懒旋转等截面增强工艺
→ 重新排序并导出 G-code
```

## 3. 开发环境与构建

| 项目 | 当前环境 |
| --- | --- |
| 操作系统 | Windows x64 |
| IDE | Visual Studio 2026 Insiders |
| IDE 路径 | `D:\Program Files\Microsoft Visual Studio\18\Insiders\Common7\IDE` |
| 工具集 | MSVC v145，C++17 |
| Qt | Qt 6.9.3，`msvc2022_64` |
| OpenCV | OpenCV 4.11.0 |
| 解决方案 | `G-code_post-processing_system.slnx` |

默认 OpenCV 安装位置为 `D:\develop\opencv`。本机路径不同时，应在本机工程环境中调整依赖路径。

在项目根目录构建 `Release|x64`：

```powershell
& "D:\Program Files\Microsoft Visual Studio\18\Insiders\MSBuild\Current\Bin\MSBuild.exe" `
  ".\G-code_post-processing_system.slnx" `
  /m /p:Configuration=Release /p:Platform=x64
```

输出目录：

```text
x64/Release/
```

## 4. 文档导航

- [文档入口](./docs/index.md)
- [用户指南](./docs/user-guide/index.md)
- [快速开始](./docs/user-guide/getting-started.md)
- [方管四轴使用指南](./docs/user-guide/rotary-four-axis.md)
- [常见问题排查](./docs/user-guide/troubleshooting.md)
- [需求索引](./docs/requirements/index.md)
- [概要设计](./docs/architecture/index.md)
- [产品范围](./docs/requirements/product-scope.md)
- [文件和 CAD](./docs/requirements/file-and-cad.md)
- [编辑与交互](./docs/requirements/editing-and-interaction.md)
- [加工功能](./docs/requirements/machining.md)
- [加工工艺](./docs/requirements/machining-process.md)
- [G-code 与导出](./docs/requirements/gcode-and-export.md)

## 5. 加工安全

生成的 G-code 在实际加工前必须完成：

- 工件尺寸、坐标系、旋转轴方向和机床配置核对；
- 加工顺序、安全高度、进退刀和加工能量启停检查；
- 完整刀路仿真；
- 安全条件下的机床空跑；
- 具备机床操作经验人员的最终复核。

自动识别和自动排序结果需要结合实际工件与现场工艺检查，所用配置必须与目标控制器和机床一致。
