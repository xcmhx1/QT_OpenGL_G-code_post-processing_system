# 商业版打包与授权流程

本文档记录轻量商业交付流程，目标是生成客户可直接运行的软件包，并通过客户机器码生成唯一的 `license.dat`。

## 1. 方案说明

当前商业化方案采用内嵌程序品牌与运行目录授权文件：

- 程序名称和窗口图标固定编译进主程序。
- `branding.json`：可选，仅保存公司名、网站、支持和简介信息。
- `license.dat`：授权文件。不存在或无效时按 Lite 版本运行。
- `license_request.bat`：客户双击后生成机器码。

该方案可以正常配合 Qt 的 `windeployqt` 打包。为了简化操作，本文档默认直接在构建输出目录 `x64\Release` 中打包。`windeployqt` 只负责复制 Qt 运行库和插件，不会自动复制 `branding.json`、`license.dat`、`license_request.bat` 等业务文件，因此需要按交付内容手动复制。

## 2. 版本划分

Lite 版本：

- 不放有效 `license.dat` 时运行。
- 保留基础 CAD 绘图、DXF/DWG 导入导出、3轴 G 代码基础流程。
- 访问 Pro 功能时会提示需要授权。

Pro 版本：

- 在 exe 同目录放置有效 `license.dat` 后运行。
- 开放位图导入、4轴(绕A)导出、G代码配置、自定义外观等功能。

注意：授权逻辑属于轻量商业交付方案，目的是减少普通复制和误用，不属于强反逆向保护。

## 3. 构建 Release 程序

在项目根目录执行：

```powershell
& "D:\Program Files\Microsoft Visual Studio\18\Insiders\MSBuild\Current\Bin\MSBuild.exe" .\G-code_post-processing_system.vcxproj /t:Rebuild /p:Configuration=Release /p:Platform=x64 /m
```

构建入口仍是项目根目录下的 `G-code_post-processing_system.vcxproj`。源码按模块整理到 `src/` 和 `include/` 子目录后，发布输出目录仍为 `x64\Release`，打包步骤不变。

构建成功后，主程序位于：

```text
x64\Release\G-code_post-processing_system.exe
```

如果构建时出现 Qt License Service 提示，但最终为 `0 个错误`，可以先忽略该提示。若出现 `QOpenGLFunctions_4_5_Core` 找不到，检查 Release 配置中是否包含 Qt 模块 `opengl;openglwidgets`。

## 4. 准备构建输出目录

本文档直接使用 `x64\Release` 作为打包目录。构建完成后，确认主程序存在：

```powershell
Test-Path .\x64\Release\G-code_post-processing_system.exe
```

如果输出为 `False`，说明 Release 构建未成功，需要先重新构建。

OpenCV DLL 正常会由工程 PostBuildEvent 复制到 `x64\Release`。可以检查：

```powershell
Test-Path .\x64\Release\opencv_world4110.dll
```

## 5. 执行 Qt 打包

使用当前 Qt 版本的 `windeployqt`：

```powershell
& "D:\Qt\6.9.3\msvc2022_64\bin\windeployqt.exe" --release .\x64\Release\G-code_post-processing_system.exe
```

执行后，`x64\Release` 中应出现 Qt DLL、平台插件、样式插件等文件，例如：

```text
x64\Release\
  G-code_post-processing_system.exe
  Qt6Core.dll
  Qt6Gui.dll
  Qt6Widgets.dll
  Qt6OpenGL.dll
  Qt6OpenGLWidgets.dll
  platforms\
    qwindows.dll
```

如果程序启动提示缺少 DLL，优先检查：

- 是否使用了与编译一致的 `windeployqt`，即 `D:\Qt\6.9.3\msvc2022_64\bin\windeployqt.exe`。
- 是否复制了 OpenCV 运行库 `opencv_world4110.dll`。
- 是否在纯净机器上缺少 VC++ Runtime。必要时安装 Microsoft Visual C++ Redistributable。

## 6. 添加可选客户信息

如需显示客户支持信息，可将 `branding.example.json` 复制到构建输出目录并改名为 `branding.json`：

```powershell
Copy-Item .\branding.example.json .\x64\Release\branding.json
```

按客户情况修改 `branding.json`：

```json
{
  "companyName": "客户公司名称",
  "website": "https://example.com",
  "support": "技术支持：support@example.com",
  "about": "本软件用于二维 CAD 绘图、DXF/DWG 处理与 G 代码后处理。"
}
```

程序窗口图标已经作为 Qt Resource 编译进 EXE，不需要在发布目录复制外部图标文件。Windows 资源管理器中的 EXE 文件图标仍由已有 `.ico/.rc` 资源决定。

## 7. 添加机器码生成工具

将机器码生成脚本复制到构建输出目录：

```powershell
Copy-Item .\license_request.bat .\x64\Release\
```

客户双击 `license_request.bat` 后，只会在 exe 同目录生成 `机器码.txt`。

`机器码.txt` 中只有一行机器码，不包含客户名、软件名或其他说明。客户把这个文件发给你即可。

客户也可以手动运行：

```bat
G-code_post-processing_system.exe --license-request
```

客户发回内容示例：

```text
C875F0D6413D5C7D5A9471F1A0AFA0D0A69E8D8B6B9A92A56E5E9D5D1F83F1F0
```

生成授权时直接使用这一行机器码。

## 8. 生成 license.dat

在项目根目录运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\Generate-License.ps1 -MachineId "客户机器码" -OutputPath .\license.dat
```

如果客户发来的是 `机器码.txt` 文件，也可以直接使用：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\Generate-License.ps1 .\机器码.txt
```

更推荐的拖入方式是把 `机器码.txt` 拖到 `tools\Generate-License.bat` 上。该 bat 会调用 `Generate-License.ps1`，并默认在 `机器码.txt` 所在目录生成 `license.dat`。

`Generate-License.ps1` 本身也支持第一个位置参数传入机器码文件；如果当前 Windows 允许直接运行 `.ps1`，也可以把 `机器码.txt` 拖到 `Generate-License.ps1` 上。

参数说明：

- `-MachineId`：客户发回的机器码。
- `MachineCodeFile`：第一个位置参数，客户发回的 `机器码.txt` 文件，与 `-MachineId` 二选一。
- `-OutputPath`：生成的授权文件路径。

生成成功后，将 `license.dat` 发给客户。客户把它放到 exe 同目录后重新启动程序即可。

注意：`tools\Generate-License.ps1` 和 `tools\Generate-License.bat` 是你本地使用的授权生成工具，不要放进客户发布包。

## 9. 构建输出目录建议结构

未授权包：

```text
x64\Release\
  G-code_post-processing_system.exe
  branding.json（可选）
  license_request.bat
  opencv_world4110.dll
  Qt6Core.dll
  Qt6Gui.dll
  Qt6Widgets.dll
  Qt6OpenGL.dll
  Qt6OpenGLWidgets.dll
  platforms\
    qwindows.dll
```

已授权包：

```text
x64\Release\
  G-code_post-processing_system.exe
  branding.json（可选）
  license.dat
  license_request.bat
  opencv_world4110.dll
  Qt6Core.dll
  Qt6Gui.dll
  Qt6Widgets.dll
  Qt6OpenGL.dll
  Qt6OpenGLWidgets.dll
  platforms\
    qwindows.dll
```

如果客户先拿未授权包试用，后续只需要补发 `license.dat`，不需要重新发完整程序。

## 10. 本机验证流程

打包完成后，建议按以下顺序验证：

1. 删除 `x64\Release` 中的 `license.dat`。
2. 双击主程序，确认窗口标题显示 Lite，基础绘图和 DXF 导入可用。
3. 尝试点击位图导入、4轴导出、自定义外观或 G代码配置，确认提示需要 Pro 授权。
4. 双击 `license_request.bat`，确认能生成 `机器码.txt`，且文件内容只有一行机器码。
5. 使用 `tools\Generate-License.ps1` 生成本机对应的 `license.dat`。
6. 将 `license.dat` 放入 `x64\Release`。
7. 重新启动主程序，确认授权状态为 Pro，受限功能可以打开。
8. 在一台未安装 Qt 的机器或虚拟机上启动发布包，确认没有缺 DLL。

## 11. 推荐的一键打包命令

下面命令假设已经完成 Release 构建：

```powershell
$out = ".\x64\Release"
Copy-Item .\branding.example.json "$out\branding.json"
Copy-Item .\license_request.bat $out\
& "D:\Qt\6.9.3\msvc2022_64\bin\windeployqt.exe" --release "$out\G-code_post-processing_system.exe"
```

如果要直接做已授权包，再复制授权文件：

```powershell
Copy-Item .\license.dat .\x64\Release\
```

## 12. 常见问题

窗口或任务栏图标未显示：

- 确认使用最新 Release 构建，窗口图标来自 EXE 内嵌的 Qt Resource。
- 确认发布目录包含 `qsvgicon.dll`、`qsvg.dll` 及其 Qt 依赖。
- Windows 资源管理器中的 EXE 文件图标由 `.ico/.rc` 资源决定，与运行时窗口图标分开。

授权后仍显示 Lite：

- 检查 `license.dat` 是否在 exe 同目录。
- 检查生成授权时使用的机器码是否来自当前客户机器。
- 检查客户是否修改过 `license.dat` 内容。任何字段改动都会导致签名无效。

客户换电脑后不能授权：

- 这是预期行为。当前授权文件绑定机器码。
- 客户换电脑时，需要重新运行 `license_request.bat` 并重新生成 `license.dat`。

杀毒软件误报：

- 该方案没有做强壳或复杂反逆向，误报概率相对低。
- 若仍发生误报，优先考虑代码签名证书，而不是加壳。

## 13. 不建议随客户包发布的文件

以下文件只保留在开发目录：

- `tools\Generate-License.ps1`
- `tools\Generate-License.bat`
- `license.example.json`
- `branding.example.json`
- 源码、工程文件、调试符号、构建中间目录

客户包中只需要运行程序、依赖 DLL、授权申请脚本、可选客户信息文件，以及可选的 `license.dat`。
