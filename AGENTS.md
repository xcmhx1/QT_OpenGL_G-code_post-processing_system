# Repository Guidelines

## 项目结构与模块组织
本仓库是基于 Qt 6 和 Visual Studio 2026 的 Windows 桌面 CAD / G-code 后处理程序。手写 C++ 源码放在 `src/`，公共头文件放在 `include/`。第三方 DXF 解析代码集中在 `src/libdxfrw/` 与 `include/libdxfrw/`，修改前先确认必要性。界面资源位于仓库根目录，如 `Gcode_postprocessing_system.ui` 和 `Gcode_postprocessing_system.qrc`。技术资料放在 `technical_file/`。`x64/` 与 `.vs/` 为构建产物和 IDE 状态目录，不应提交。

## 构建、测试与开发命令
使用 Visual Studio 2026 或 `MSBuild` 构建当前项目：

```powershell
msbuild .\G-code_post-processing_system.vcxproj /p:Configuration=Debug /p:Platform=x64
msbuild .\G-code_post-processing_system.vcxproj /p:Configuration=Release /p:Platform=x64
devenv .\G-code_post-processing_system.slnx
```

`Debug|x64` 用于日常调试，`Release|x64` 用于发布验证。构建前确认本机已安装 Qt `6.9.3_msvc2022_64`，且 `QtMsBuild` 能被 Visual Studio 正常识别。

## 技术栈与图形约束
界面框架使用 Qt 6.9 Widgets。渲染层明确使用 OpenGL 4.5 Core Profile，代码中已通过 `QOpenGLFunctions_4_5_Core` 绑定；着色器版本与其保持一致。涉及图形初始化、缓冲区管理或着色器修改时，不要降级到更低的 OpenGL 版本，也不要混用旧式固定管线写法。

## 代码风格与命名规范
使用 C++17，括号采用 Allman 风格。类名使用 `PascalCase`，函数和局部变量使用 `camelCase`，成员变量使用 `m_` 前缀。保持现有分层，界面与视图逻辑放在 `CadViewer`、`Gcode_postprocessing_system` 一类模块中，数据读取与文档处理逻辑放在 `dx_iface`、`DxfDocument` 等模块中。

## 测试指南
当前仓库没有独立的自动化测试工程。每次提交前至少完成以下检查：

- 成功构建 `Debug|x64`，且不引入新的编译警告
- 启动程序并手动验证受影响的 DXF 图元或交互流程
- 在 PR 描述中写明验证所用文件、操作路径和结果

如果后续补充自动化测试，建议新增 `tests/` 目录或单独测试工程，并使用与功能对应的命名，例如 `CadViewerZoomTests`。

## 提交与合并请求规范
现有提交历史以简短、直接的中文说明为主，例如 `项目初始化`、`导入libdxfrw库`。提交信息应聚焦单一改动，避免把重构和功能修改混在一起。PR 需要包含变更目的、实现方式、手动验证步骤；如果涉及界面变化，附上截图。

## 代理协作说明
在本仓库内工作的代理默认使用中文回复。只有在涉及命令、路径、库名、类名、函数名或用户明确要求时，才保留必要英文。

## 开发协作约束
遵循 MVC 架构，按需查询 `README.md` 及其引用文档，避免无关上下文膨胀。修改第三方目录 `src/libdxfrw/` 或 `include/libdxfrw/` 时，应在提交说明中明确原因和影响范围。
