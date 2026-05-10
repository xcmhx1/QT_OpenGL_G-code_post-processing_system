# 商业交付配置

## 打包方式

Qt 打包后，将以下文件放到 exe 同目录：

- `branding.json`：客户版标题、公司名、图标路径。
- `app.ico`：客户版图标，文件名需要与 `branding.json` 的 `iconPath` 一致。
- `license.dat`：授权文件。没有该文件时程序按 Lite 版本运行。
- `license_request.bat`：客户生成授权申请用。

## 客户申请授权

客户双击 `license_request.bat`，复制窗口中的 JSON 内容发回。

也可以手动运行：

```bat
G-code_post-processing_system.exe --license-request
```

## 本地生成 license.dat

保存客户发回的 JSON 为 `request.json`，然后在项目目录运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\Generate-License.ps1 -RequestFile .\request.json -Customer "客户公司名称" -Edition pro -Expires "2027-12-31" -OutputPath .\license.dat
```

把生成的 `license.dat` 发给客户，客户放到 exe 同目录后重新启动程序。

## 版本控制

- Lite：不放有效 `license.dat` 时运行，保留基础 CAD、DXF 与 3轴流程。
- Pro：放置有效 `license.dat` 后运行，开放位图导入、4轴(绕A)导出、G代码配置、自定义外观等功能。

## 注意

授权逻辑属于轻量商业交付方案，目的是减少普通复制和误用，不属于强反逆向保护。`tools\Generate-License.ps1` 只保留在你自己的工作目录，不随客户包发布。
