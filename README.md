# G-code Post Processing System

[G/M 代码参考](./technical_file/G-M_Code.md)

本项目是一个基于 Qt 6 Widgets、OpenGL 4.5 Core Profile、Visual Studio 2026 的 Windows 桌面 CAD / G-code 后处理程序。当前代码主线已经具备 CAD 文件导入、位图矢量化导入、二维图元显示、基础交互、简单绘图与编辑命令，并提供 `2D` / `4轴(绕A)` 两条 G 代码导出链与 JSON Profile 配置能力；主窗口已接通导入文件、导入 `DXF`、导入 `DWG`、导入图片、保存文件（`Ctrl+S`）、导出为 `DXF`、导出为 `DXF`（安全模式）、统一的 `导出G代码` 入口、反向加工以及统一的“排序（保留方向）/ 智能排序”入口，具体输出与排序逻辑会按当前 `G代码模式` 自动分流到 `3轴` 或 `4轴(绕A)`。`Mode3D` 当前已重新接入一条面向绕 `X` 轴回转、`A` 轴展开的四轴 G 代码导出链。该链路的机床控制点生成已下沉到各 `CadItem` 子类，由各图元自行负责从原始几何解算 `rawPathPoints3D` 与 `controlPoints4Axis`；`GGenerator` 负责按加工顺序组织图元、消除相接图元空跑、统一安全高度策略并输出最终 NC。当前这条四轴导出链已覆盖 `Line`、`Arc`、`Circle`、`Ellipse`、`Polyline`、`LWPolyline` 等主要加工图元，并围绕方管类工件补充了 `YZ` 截面方管边/圆角刀头方向模型：直边垂直加工，四角圆弧区域的刀头方向指向对应四分之一圆弧圆心，边角过渡时 `A` 轴会沿真实加工路径渐变，不在接点原地调头。同时增加了仿 AutoCAD 风格的紧凑工具面板，当前分为 `默认` 与 `机加工` 两个页签：`默认` 页签承载绘图、修改、图层、特性与显示入口，`显示` 面板可开关加工方向箭头与加工顺序标签；`机加工` 页签已收束为 `导入导出`、`排序`、`功能`、`配置`、`G代码配置` 五个面板，直接集成 `文件导入`、`G代码导出`、`排序(保留方向)`、`智能排序`、`去重`、`自动去重`、`使用默认导出路径`、`使用dxf文件名`、`当前配置`、`G代码模式`、`G代码配置` 等高频操作，用于减少反复打开菜单栏的步骤；其中“绘图”面板已提供直线、构造线、矩形、多边形、圆、圆弧、椭圆、多段线、轻量多段线等入口；“修改”面板现已提供 `移动`、`删除`、`旋转`、`复制`、`缩放`、`阵列` 按钮，并通过网格式标题下拉补充 `矩形阵列`、`环形阵列`、`镜像`、`偏移`、`修剪`、`延申`、`合并`、`圆角`、`直角（倒角）` 入口。当前“用户设置 -> 外观设置”已集中提供浅色 / 深色主题切换与自定义外观配置；“用户设置”菜单下也已接通 `G代码配置` 对话框，可按文件级、实体类型、图层规则、颜色规则四个层级定制导出行为，并已新增面向四轴导出的旋转轴配置字段、四轴默认模板与当前运行目录 JSON 自动识别 / 实时切换入口；其中“实体类型”页已改为与“图层规则 / 颜色规则”一致的左侧列表式浏览，不再需要逐个展开切换。Viewer 层也已支持加工方向箭头、加工顺序编号、选中图元基点/控制点手柄显示、AutoCAD 风格窗口框选（向左拖拽碰选、向右拖拽包含选）与框选过程实时候选高亮，且加工顺序编号框已支持直接交互：双击可切换该图元加工方向，依次点击两个编号框可互换两个图元的加工顺序；底部状态栏已收束为单一“捕捉设置”下拉入口，支持 `基点`、`控制点`、`端点`、`中点`、`圆心/中心`、`交点`、`网格` 七类捕捉开关，并已接入第一阶段控制点编辑（选中后点控点再点目标点，已覆盖 `Point`、`Line`、`Xline`、`Circle`、`Arc`、`Ellipse`、`Polyline`、`LWPolyline`）；夹点编辑现已支持实时几何预览，重叠夹点支持悬停 `1s` 弹出候选选择栏并可用 `Tab / Shift+Tab` 循环切换，候选项文案已改为夹点类型名且宽度自适应显示；菜单栏新增“帮助”入口，按快速上手、快捷命令、绘图教程、修改教程、机加工 / G代码、位图导入、外观与显示、关于分类提供内置说明；近期又对方向箭头样式、主次网格对比、选中角标样式、机加工页签布局、G 代码换行符、导出路径记忆、配置切换入口、外观设置与帮助文档入口做了进一步优化。

## 项目现状

当前已实现：

- 导入 `.dxf` / `.dwg` 文件
- 导入包含 `SPLINE` 的 DXF 时，会将样条曲线高精度离散转换为多段线，以复用多段线显示、编辑与 G 代码生成链路
- 导入 `.bmp` / `.png` / `.jpg` / `.jpeg` 位图，并通过 OpenCV 轮廓提取与规则图元拟合转换为 CAD 实体
- 使用 OpenGL 4.5 Core Profile 渲染 CAD 场景
- 支持点、直线、构造线、圆、圆弧、椭圆、多段线、轻量多段线的内部图元构建、显示与编辑；矩形、多边形通过闭合多段线创建
- 支持平移、缩放、顶视图切换、轨道观察、屏幕拾取
- 支持拖拽导入文件到视图区域
- 提供命令提示栏、命令历史栏、底部坐标状态栏
- 空闲态支持动态命令框：键入字符即弹出候选命令，支持 `Tab / Shift+Tab` 循环选择，`Enter` 或 `Space` 执行，`Esc` 取消
- 底部状态栏已提供单一“捕捉设置”下拉入口，可统一开关 `基点`、`控制点`、`端点`、`中点`、`圆心/中心`、`交点`、`网格` 七类捕捉
- 捕捉设置选择会通过 `QSettings` 持久化，下次启动自动恢复上次勾选状态（首次启动采用默认组合）
- 网格已支持随缩放等级动态调整显示密度，网格吸附与当前显示网格步长保持一致；细分采用二分层级（如 `100 -> 50 -> 25 -> 12.5`），不会丢失上一级网格点
- 网格显示已区分主/次网格线：主网格线保持标准网格色，次网格线按背景色进一步淡化，降低高密度缩放下的视觉噪声
- 提供仿 AutoCAD 风格的紧凑工具面板，当前分为 `默认` 与 `机加工` 两个页签；`默认` 页签包含“绘图”“修改”“图层”“特性”“显示”五个左对齐面板，`机加工` 页签已收束为“导入导出”“排序”“功能”“配置”“G代码配置”五个面板
- `机加工` 页签当前采用左对齐的紧凑布局：`文件导入` 位于 `G代码导出` 上方，`排序(保留方向)` 与 `智能排序` 采用上下布局，`功能` 面板集中承载 `自动去重`、`去重`、`使用默认导出路径`、`使用dxf文件名`
- 提供统一的浅色 / 深色主题切换与自定义外观设置，入口位于菜单栏“用户设置 -> 外观设置”
- 提供点、直线、构造线、矩形、多边形、圆、圆弧、椭圆、多段线、轻量多段线的交互式绘制；矩形与多边形底层为闭合多段线
- 提供删除、移动、旋转、复制、缩放、矩形阵列、环形阵列、镜像、偏移、修剪、延申、合并、圆角、直角（倒角）、改色、改图层、Undo / Redo
- 修改类命令已逐步向 AutoCAD 式交互靠齐：旋转、缩放采用“选择集 -> 指定基点 -> 鼠标实时预览 -> 点击或输入数值确认”；复制采用“指定基点 -> 指定第二点”并实时预览；偏移采用“输入距离 -> 指定偏移侧”；镜像、矩形阵列、环形阵列均已接入 transient 预览
- 批量修改的 Undo 粒度已进一步收束：批量删除、移动、旋转、缩放、复制、矩形阵列等操作按一次用户命令进入 Undo / Redo 栈，避免多选操作撤销时逐个图元回退
- 提供第一阶段控制点编辑：在空闲态点击“当前选中图元”的可编辑控制点后，再次点击目标点提交；当前支持 `Point`、`Line`、`Xline`、`Circle`、`Arc`、`Ellipse`、`Polyline`、`LWPolyline`
- 控制点编辑已支持实时预览：在确认前即可看到图元变形、基点到目标点引导线与目标点高亮
- 当多个可编辑夹点发生重叠时，支持悬停 `1s` 弹出候选选择栏，并支持 `Tab / Shift+Tab` 循环切换候选
- 重叠夹点候选选择栏已改为显示“夹点类型名”（如 `基点`、`长轴控制点`、`顶点拉伸点`），并按文本宽度自适应布局
- 提供 transient 预览、十字光标叠加、选中高亮
- 选中高亮已升级为叠加式样：保留图元原始显示色，额外绘制轻描边与四角角标，降低完整包围框对视野的遮挡
- 提供 AutoCAD 风格窗口框选：向左拖拽为碰选，向右拖拽为包含选；框选拖拽过程中会实时高亮候选图元
- 选择集已支持 `Shift` 增量切换：`Shift + 左键` 与 `Shift + 框选` 均可对命中图元执行增选/反选
- 选中图元时已显示基点/控制点手柄；圆弧与椭圆边界端点使用三角箭头手柄强调方向端，且手柄支持悬停实时高亮
- Viewer 已支持加工方向箭头与加工顺序编号显示，并与排序 / 导出共用同一套加工路径语义；当前箭头已简化为“箭杆 + 起点三角箭头头部”样式，不再附带额外圆点标记
- `默认 -> 显示 -> 显示机加工相关` 可统一控制加工方向箭头与加工顺序编号是否显示，并通过 `QSettings` 持久化
- 加工顺序编号框当前已支持直接交互：双击编号框可切换该图元加工方向，依次点击两个编号框可互换两个图元的加工顺序；相关操作纳入 Undo / Redo
- “机加工 -> 功能”已新增 `去重` 操作：仅在同类型图元之间用原始数据做比对，删除完全重叠实体并保留最后一个；当前覆盖 `Line`、`Circle`、`Arc`、`Ellipse/椭圆弧`、`Polyline`、`LWPolyline`
- “机加工 -> 功能”支持 `自动去重` 勾选；若启用，则在导出 G 代码前会先自动执行一次去重，再进入导出链
- 命令取点、移动基点/目标点和状态栏坐标显示已统一走同一套吸附后坐标
- 多图元修改已接通：`移动`、`删除`、`改色` 及 Ribbon 中的 `复制`、`旋转`、`缩放`、`阵列`、图层/颜色修改均可批量执行；其中复制、旋转、缩放、矩形阵列已使用批量命令提交，便于一次性撤销/重做
- 多图元移动命令已支持实时预览：在目标点确认前会同时显示整组图元的平移后轮廓与引导线
- Ribbon“绘图”面板使用网格式下拉菜单收纳完整绘图入口，常用图元直接显示在面板中
- Ribbon“图层”与“特性”面板已支持在“默认绘图属性”和“当前选中图元属性”之间自动切换显示
- 图层下拉项与颜色下拉项均以内嵌色块图标显示当前颜色语义
- 浅色主题下会对 Viewer 中低对比度线色做显示补偿，避免白色或近白色图元贴近背景后难以辨认
- `特性 -> 颜色` 中 `ByLayer` 色块固定显示所属图层颜色，不再错误跟随图元自定义颜色
- 提供 `GProfile` JSON 配置读写，支持文件级、实体类型、图层规则、颜色规则四类配置，并已新增四轴旋转轴配置字段、4轴默认配置模板与运行目录 JSON 自动识别机制
- 提供 `用户设置 -> G代码配置...` 对话框，并已纳入当前外观设置；当前对话框已补充 `四轴加工` 配置页，可设置四轴离轴额外距离、加工面 `Z` 修正等参数，并已拆分“恢复3轴默认 / 恢复4轴默认”
- `G代码配置` 对话框中的“实体类型”页当前已改为左侧列表 + 右侧编辑区的结构，和“图层规则 / 颜色规则”保持一致，可一次性浏览全部实体类型
- 程序启动时会自动扫描当前运行目录下的 `*.json` G 代码配置文件，并同步显示到“机加工 -> 配置 -> 当前配置”下拉框，支持运行时实时切换当前生效配置
- “机加工 -> 配置 -> 当前配置”按钮已接入 G 代码配置文件管理界面，可集中管理配置检索路径、查看各路径下的配置文件、删除配置文件，并通过双击或“使用”按钮切换当前配置
- 通过 `G代码配置` 从运行目录之外导入的 JSON 会追加到本次会话配置列表中，但不会写回启动时的自动识别列表
- “机加工 -> 配置”中的 `当前配置` 支持直接显示已加载配置名并实时切换；配置列表由运行目录自动识别项与本次会话临时导入项共同组成
- 颜色规则默认内置 `BYLAYER`、`BYBLOCK` 与 AutoCAD 基础 `ACI:1` 至 `ACI:9` 索引颜色项，同时兼容真彩色扩展
- 提供 `GGenerator` 纯 `2D` G 代码生成后端，支持 `Line`、`Arc`、`Circle`、`Ellipse`、`Polyline`、`LWPolyline`
- `GProfile` 已新增 `A` 轴中心、角度偏移、连续展开、方向翻转、离轴额外距离、加工面 `Z` 修正、初始机床点等旋转轴配置字段，并保留 JSON 读写
- `Mode3D` 已重新接通一条绕 `X` 轴回转的四轴 G 代码导出主链，当前由各图元子类负责生成机床控制点，`GGenerator` 负责跨图元组织与 NC 输出
- `CadItem` 基类已新增 `m_rawPathPoints3D` 与 `m_controlPoints4Axis` 两级缓存；各图元可覆写 `rebuildRawPathPoints3D()` / `rebuildControlPoints4Axis()` 实现自身四轴几何语义
- 当前 `Line`、`Arc`、`Circle`、`Ellipse`、`Polyline`、`LWPolyline` 已接入四轴控制点生成；`Point` 不输出加工轨迹
- 四轴导出当前会在图元边界对齐 `A` 角连续性，避免如 `180 -> -90` 这类跨象限切换走优弧空跑；连续边角过渡处会把 `A` 轴变化摊到后续真实加工路径中，避免在接点原地调头
- 四轴导出已补充方管截面工艺模型：以 `YZ` 投影判断路径位于方管直边或圆角，直边垂直加工，四角圆弧区域的刀头方向指向对应四分之一圆弧圆心
- 四轴导出会在 `Mode3D` 输出注释中写入识别到的方管截面边界与四角圆心，便于核对圆角姿态推断结果
- 四轴图元间空跑当前采用“导出开始时一次性解算出的全图最大离 `X` 轴距离 + 四轴离轴额外距离”作为统一安全高度；离轴额外距离默认值当前为 `5`
- 四轴加工当前支持在当前实时加工平面上追加可正可负的 `Z` 修正，用于给激光喷口留出额外贴面间隙
- 四轴导出支持从 Profile 中读取可选的初始机床点；若配置启用，则会在开始加工前先引入这段机床初始定位
- 四轴排序当前已增强对“小间隙相邻图元”的连续加工倾向，并把首个待加工工件优先选为最接近参考点 `(0, 0, 50)` 的图元，以减少起始阶段的 `A` 轴翻转
- 对于多图元拼接、仅留很小断点的近闭合图元组，排序当前会优先在该组内部从断点处起刀并优先完成当前组，再回到原有算法选择下一组或下一图元
- 生成的 G 代码文本统一使用 `CRLF` 行尾，避免不同数控软件对单独 `LF` 的兼容问题
- 生成 G 代码时不再修改原始图元几何；四轴导出链使用图元内部缓存的控制点与导出态数据，导出结束后界面会回到原始图元显示语义
- 已接通主窗口“导入文件”“导入DXF...”“导入DWG...”“导入图片”“保存文件（Ctrl+S）”“导出为DXF...”“导出为DXF（安全模式）...”“导出G代码”“反向加工”以及统一的“排序（保留方向）/ 智能排序”菜单动作；排序与导出会按当前 `G代码模式` 自动分流到 `3轴` 或 `4轴(绕A)` 逻辑
- 若文档尚未完成排序，点击 `导出G代码` 时会先按当前 `G代码模式` 自动执行一次智能排序，再继续导出
- 导入文件对话框会记忆上一次导入目录；G 代码导出会记忆上一次成功保存目录，并可通过“使用默认导出路径”直接复用
- 勾选“使用dxf文件名”后，导出 G 代码会优先使用当前 DXF 文件名作为 NC 文件名并落到默认导出目录；若存在重名文件，则会先提示覆盖确认
- 保存与导出统一写出 `.dxf`；保存在已有文档路径时会直接覆盖同名文件
- 提供基于 `CadItem::m_processOrder`、`CadItem::m_isReverse`、闭合路径起刀缝点参数的加工顺序与方向控制
- `2D` 智能排序会优先沿工件整体分布的主方向推进，并对明显“回头”的候选路径施加惩罚
- `2D` 智能排序已支持对 `Circle`、完整 `Ellipse`、闭合 `Polyline`、闭合 `LWPolyline` 自动联合优化加工方向与起刀缝点
- 圆形默认从顶部起刀，导出的 G 代码会跟随排序阶段确定的方向与缝点语义
- G 代码导出阶段会过滤无意义空行，减少脏输出
- 菜单栏新增 `帮助` 菜单，内置 `快速上手`、`快捷命令`、`绘图教程`、`修改教程`、`机加工 / G代码`、`位图导入`、`外观与显示`、`关于` 等分类说明
- 已接入轻量商业交付配置：运行目录中的 `branding.json` 可调整应用标题、公司名、窗口标题后缀和运行时图标
- 已接入本机机器码授权：未放置有效 `license.dat` 时按 `Lite` 版本运行，放置与本机机器码匹配的授权文件后开放 `Pro` 功能
- 客户侧可通过 `license_request.bat` 生成只包含机器码的一行 `机器码.txt`；开发侧通过 `tools/Generate-License.ps1` 或 `tools/Generate-License.bat` 生成对应 `license.dat`

当前未完成或未接线：

- 当前四轴导出链只对绕 `X` 轴回转、`A` 轴展开的特定回转 / 方管类工件场景建模，不是通用多轴后处理器
- 方管圆角圆心当前从导入路径在 `YZ` 平面上的直边切点自动推断；若图纸缺少足够直边或切点信息，圆角姿态会回退到中心线方向规则，需要结合实际图纸验证
- `3D` / `4轴(绕A)` 排序相关逻辑已开始围绕当前四轴控制点链路收束，但仍以基础启发式为主，后续还需要继续完善更复杂的工艺规则与边界场景
- `4轴(绕A)` 导出的旋转轴参数当前已接入 `G代码配置 -> 四轴加工` 配置页，但整体仍以基础参数为主，后续还需要继续补齐更完整的四轴工艺编辑界面
- 普通 DXF 导出会尽量保留原始实体；若包含兼容性较弱的普通 `POLYLINE`，建议改用“导出为DXF（安全模式）”

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

若系统环境变量中未配置 `msbuild`，可直接使用 Visual Studio 2026 Insiders 的 `MSBuild.exe` 绝对路径：

```powershell
& "D:\Program Files\Microsoft Visual Studio\18\Insiders\MSBuild\Current\Bin\MSBuild.exe" .\G-code_post-processing_system.vcxproj /p:Configuration=Debug /p:Platform=x64
& "D:\Program Files\Microsoft Visual Studio\18\Insiders\MSBuild\Current\Bin\MSBuild.exe" .\G-code_post-processing_system.vcxproj /p:Configuration=Release /p:Platform=x64
```

建议优先使用 `Debug|x64` 进行日常开发和交互验证。

## 商业打包与授权

完整打包步骤见 [COMMERCIAL_RELEASE.md](D:/projects/visual_studio_2026/G-code_post-processing_system/COMMERCIAL_RELEASE.md)。当前商业交付方案按运行目录文件工作：

- `branding.json`：由 `branding.example.json` 复制改名而来，用于配置客户版标题、公司名、标题后缀和运行时窗口图标路径
- `license_request.bat`：放在 exe 同目录，客户双击后由主程序生成 `机器码.txt`
- `机器码.txt`：客户侧生成的机器码文件，内容只有一行 64 位十六进制机器码
- `license.dat`：开发侧根据客户机器码生成的授权文件，客户放到 exe 同目录后重启软件即可启用 `Pro` 功能
- `tools/Generate-License.ps1` / `tools/Generate-License.bat`：开发侧授权生成工具，不随客户包发布

推荐流程：

1. 使用 `Release|x64` 构建程序。
2. 在 `x64\Release` 中运行 `windeployqt` 完成 Qt 依赖部署。
3. 复制 `branding.json`、`app.ico`、`license_request.bat` 到 exe 同目录。
4. 客户运行 `license_request.bat`，把生成的 `机器码.txt` 发回。
5. 开发侧把 `机器码.txt` 拖到 `tools\Generate-License.bat`，生成 `license.dat`。
6. 客户将 `license.dat` 放到 exe 同目录，重新启动程序。

当前授权机制是轻量商业交付方案，用于减少普通复制和误用，不等同于强反逆向或强 DRM。

## 仓库结构

```text
G-code_post-processing_system/
|-- include/                                # 公共头文件
|   |-- Gcode_postprocessing_system.h       # 主窗口类声明
|   |-- AppBranding.h                       # 运行目录品牌配置读取
|   |-- AppLicense.h                        # 本机机器码授权校验
|   |-- AppTheme.h                          # 应用主题颜色定义
|   |-- CadAppearanceSettingsDialog.h       # 自定义外观设置对话框
|   |-- CadHelpDialog.h                     # 帮助文档对话框
|   |-- CadViewer.h                         # OpenGL 视图主类
|   |-- CadController.h                     # 输入控制器
|   |-- DrawStateMachine.h                  # 绘图/编辑状态机
|   |-- CadEditer.h                         # 编辑器与命令栈接口
|   |-- CadDocument.h                       # 文档模型
|   |-- CadBitmapImportDialog.h             # 位图导入参数对话框
|   |-- CadBitmapVectorizer.h               # 位图预处理与矢量化拟合
|   |-- GProfile.h                          # G 代码 Profile 配置模型
|   |-- GProfileDialog.h                    # G 代码 Profile 配置对话框
|   |-- GProfileManagerDialog.h             # G 代码配置文件与路径管理对话框
|   |-- GProfilePathStore.h                 # G 代码配置检索路径持久化
|   |-- GGenerator.h                        # G 代码生成器（2D / 4轴导出组织）
|   |-- CadItem.h			               # 图元基类（含 rawPathPoints3D / controlPoints4Axis 缓存）
|   |-- CadLineItem.h                       # 直线图元
|   |-- CadXlineItem.h                      # 构造线图元
|   |-- CadCircleItem.h                     # 圆图元
|   |-- CadArcItem.h                        # 圆弧图元
|   |-- CadEllipseItem.h                    # 椭圆图元
|   |-- CadPointItem.h                      # 点图元
|   |-- CadPolylineItem.h                   # 多段线图元
|   |-- CadLWPolylineItem.h                 # 轻量多段线图元
|   |-- CadProcessVisualUtils.h             # 加工方向/顺序显示共用语义工具
|   |-- CadCommandLineWidget.h              # 命令栏组件
|   |-- CadStatusPaneWidget.h               # 状态栏组件
|   |-- CadToolPanelWidget.h                # Ribbon 工具面板组件
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
|   |-- AppBranding.cpp                     # branding.json 读取与图标解析
|   |-- AppLicense.cpp                      # license.dat 读取、机器码与签名校验
|   |-- Gcode_postprocessing_system.cpp     # 主窗口装配与共享状态同步
|   |-- Gcode_postprocessing_system_FileActions.cpp   # 主窗口文件导入/保存/DXF导出
|   |-- Gcode_postprocessing_system_EditActions.cpp   # 主窗口编辑命令入口
|   |-- Gcode_postprocessing_system_GCodeActions.cpp  # 主窗口G代码导出与导出前处理
|   |-- Gcode_postprocessing_system_SortActions.cpp   # 主窗口排序链
|   |-- CadViewer.cpp                       # Viewer 主渲染/文档接线壳
|   |-- CadViewer_EventHandling.cpp         # Viewer Qt 事件入口
|   |-- CadViewer_ViewNavigation.cpp        # Viewer 相机导航/坐标换算/网格步长
|   |-- CadViewer_SelectionState.cpp        # Viewer 正式选择状态写回
|   |-- CadViewer_SelectionPreview.cpp      # Viewer 框选预览与候选高亮
|   |-- CadViewer_ProcessOrderLabels.cpp    # Viewer 加工顺序标签交互
|   |-- CadViewer_InteractionOverlays.cpp   # Viewer 动态输入/夹点 popup/overlay
|   |-- CadViewer_TransientPrimitives.cpp   # Viewer 临时图元构建
|   |-- CadViewer_Snapping.cpp              # Viewer 捕捉解析与吸附高亮
|   |-- CadController.cpp                   # 输入控制器公共壳与共享状态
|   |-- CadController_KeyHandling.cpp       # 键盘处理
|   |-- CadController_MouseHandling.cpp     # 鼠标处理与空闲框选
|   |-- CadController_DynamicInput.cpp      # 动态输入/命令确认链
|   |-- DrawStateMachine.cpp                # 状态机默认逻辑
|   |-- CadEditer.cpp                       # 编辑器壳（生命周期/Undo/分发）
|   |-- CadEditer_CommandActions.cpp        # 编辑命令入口与命令类
|   |-- CadEditer_DrawHandlers.cpp          # 绘图/交互处理
|   |-- CadEditer_EntityFactories.cpp       # 图元创建工厂
|   |-- CadEditer_ModifyBuilders.cpp        # 偏移/修剪/合并/圆角/倒角构造
|   |-- CadEditer_EntityTransformHelpers.cpp # 实体变换/控制点读写
|   |-- CadEditer_GeometryHelpers.cpp       # 剩余通用几何 helper
|   |-- CadEditer_GeometryMath.cpp          # 纯几何计算
|   |-- CadDocument.cpp                     # 文档模型实现
|   |-- CadAppearanceSettingsDialog.cpp     # 自定义外观设置对话框实现
|   |-- CadHelpDialog.cpp                   # 帮助文档对话框实现
|   |-- CadBitmapImportDialog.cpp           # 位图导入配置与预览
|   |-- CadBitmapVectorizer.cpp             # 位图预处理、轮廓提取、图元拟合
|   |-- GProfile.cpp                        # G 代码 Profile 配置读写
|   |-- GProfileDialog.cpp                  # G 代码 Profile 配置对话框实现
|   |-- GProfileManagerDialog.cpp           # G 代码配置文件与路径管理实现
|   |-- GProfilePathStore.cpp               # G 代码配置检索路径读写
|   |-- GGenerator.cpp                      # 2D / 4轴 G 代码生成实现
|   |-- CadItem.cpp						  # 图元基类与四轴控制点公共工具
|   |-- CadProcessVisualUtils.cpp           # 加工方向/顺序显示共用语义实现
|   |-- CadToolPanelWidget.cpp              # Ribbon 工具面板实现
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
|-- branding.example.json                   # 商业交付品牌配置示例
|-- license.example.json                    # 授权文件结构示例
|-- license_request.bat                     # 客户侧机器码生成脚本
|-- COMMERCIAL_RELEASE.md                   # 商业打包与授权流程
|-- tools/
|   |-- Generate-License.ps1                # 开发侧授权生成脚本
|   `-- Generate-License.bat                # 拖入机器码文件生成授权的包装脚本
|-- AGENTS.md                               # 仓库协作规则
|-- README.md                               # 当前文档
|-- x64/                                    # 构建输出，不提交
`-- .vs/                                    # IDE 状态，不提交
```

## 总体架构

项目当前按“主窗口 + Ribbon 工具面板 + Viewer + Controller + Editer + Document + 渲染协调层 + DXF Adapter + Bitmap Vectorizer + G-code Backend”的组合结构组织。相比早期版本，主窗口、`CadViewer`、`CadController`、`CadEditer` 已经按输入链和数据流拆成多组实现文件：主窗口按 `文件/编辑/G代码/排序` 拆分，`CadViewer` 按 `事件/导航/选择/吸附/overlay/标签/临时图元` 拆分，`CadController` 按 `键盘/鼠标/动态输入` 拆分，`CadEditer` 按 `命令/绘图处理/图元工厂/修改构造/变换/几何` 拆分。

```text
用户输入 / Qt 菜单 / 拖拽文件 / 键盘鼠标
        |
        v
Gcode_postprocessing_system                (窗口层 / 组装层)
        |
        +------> CadCommandLineWidget      (命令栏显示)
        +------> CadStatusPaneWidget       (坐标状态显示)
        +------> CadToolPanelWidget        (Ribbon 工具面板)
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
        `------> GGenerator + GProfile      (2D / 4轴后处理 / Profile 配置)
```

## 近期四轴改造说明

这一轮重构的核心目标，是把原来集中堆在 `GGenerator` 里的“四轴几何离散 + 机床控制点生成”职责，重新下沉到各个图元子类自身。

- `CadItem` 基类现在统一提供四轴缓存与公共辅助能力：
  - `m_rawPathPoints3D`
  - `m_controlPoints4Axis`
  - `rebuildRawPathPoints3D()`
  - `rebuildControlPoints4Axis()`
- 各图元子类按自己的几何语义生成机床控制点，而不是再由 `GGenerator` 统一猜测：
  - `CadLineItem`
  - `CadArcItem`
  - `CadCircleItem`
  - `CadEllipseItem`
  - `CadPolylineItem`
  - `CadLWPolylineItem`
- `GGenerator` 在四轴模式下的职责已收束为：
  - 按现有加工顺序组织图元
  - 读取各图元下沉生成的 `controlPoints4Axis`
  - 处理图元间 `A` 角连续性，避免跨象限走优弧空跑
  - 结合全图 `YZ` 截面推断方管直边与四角圆心，并在导出阶段统一修正方管类工件的刀头方向
  - 处理图元间空跑与统一安全高度
  - 输出最终 NC

### 当前四轴链路适配范围

当前项目里的 `Mode3D`，实际对应的是一条面向“绕 `X` 轴回转、`A` 轴联动”的四轴后处理链路，而不是通用三维或通用五轴后处理器。

- 当前已接入四轴控制点生成：
  - `Line`
  - `Arc`
  - `Circle`
  - `Ellipse`
  - `Polyline`
  - `LWPolyline`
- `Point` 当前不输出加工轨迹

### 当前四轴导出策略

- 图元内部先生成自身原始几何路径，再生成四轴机床控制点
- 跨图元切换时会优先对齐 `A` 角连续性，避免 `180 -> -90` 之类的优弧空跑
- 对原始路径连续的边角接点，不再输出原地转 `A` 的零长度姿态点；需要改变刀头方向时，会把 `A` 轴变化沿后续真实 `G01` 加工路径渐变
- 方管类工件会基于全图 `YZ` 投影推断截面边界与四角圆心：映射在直边上的路径按垂直面加工，映射在四个圆角上的路径按对应四分之一圆弧圆心确定刀头方向
- 生成的 `Mode3D` G 代码注释中会包含方管截面 `Y/Z` 范围和识别到的角部圆心，便于调试实际图纸
- 图元间空跑使用统一安全高度：
  - 在本次导出开始时一次性解算全图所有图元到 `X` 轴的最大距离
  - 再额外加上可配置的“四轴离轴额外距离”
  - 当前默认值为 `5`
- 安全路径中的抬升、横移、旋转 `A`、下落全部走 `G00`
- 从安全高度回到加工起点时，不把这段路径当加工轨迹处理
- 四轴导出当前支持在实时加工平面上附加一个可正可负的 `Z` 修正，默认值为 `0`
- 四轴导出当前支持从 Profile 中读取可选的机床初始点；若启用，会在进入首个待加工图元前先引入这段机床定位
- 首个四轴待加工图元当前优先取最接近参考点 `(0, 0, 50)` 的候选，尽量避免刚开始就发生较大 `A` 轴翻转

### 当前已知限制

- 当前四轴链路仍然优先服务绕 `X` 轴回转、`A` 轴展开的回转类 / 方管类工件，不适合直接理解为任意自由曲面的通用后处理
- 方管圆角圆心依赖导入路径中存在足够的直边切点；对于缺少切点、截面不完整或非典型方管的图纸，仍需结合输出注释和实机需求验证
- `3D` 排序菜单与历史 `Mode3D` 语义仍有部分旧实现残留，后续还需要继续围绕新四轴主链收束

### 分层职责

`Gcode_postprocessing_system`

- 负责 UI 初始化与窗口装配
- 创建命令栏、状态栏并插入中心布局
- 负责将状态栏“捕捉设置”中的各类捕捉开关接线到 Viewer
- 创建 Ribbon 工具面板，并负责其与 Viewer / Document / Editer 的状态同步
- 创建“用户设置 -> 外观设置”菜单，统一管理浅色 / 深色主题、自定义外观持久化与下发
- 创建“帮助”菜单，并按快速上手、快捷命令、绘图教程、修改教程、机加工 / G代码、位图导入、外观与显示、关于分类打开内置说明
- 统一处理文件导入分发
- 接通导入文件/导入DXF/导入DWG/导入图片、保存文件（`Ctrl+S`）、普通/安全两种 DXF 导出、反向加工、排序相关菜单动作
- 维护默认绘图图层、默认绘图颜色以及工具面板显示状态
- 持有 `CadDocument` 与 `CadEditer`
- 当前实现已按 `Gcode_postprocessing_system.cpp`、`Gcode_postprocessing_system_FileActions.cpp`、`Gcode_postprocessing_system_EditActions.cpp`、`Gcode_postprocessing_system_GCodeActions.cpp`、`Gcode_postprocessing_system_SortActions.cpp` 拆分

`CadToolPanelWidget`

- 负责提供仿 AutoCAD 风格的紧凑 Ribbon 面板 UI
- 当前分为 `默认` 与 `机加工` 两个页签
- `默认` 页签提供“绘图”“修改”“图层”“特性”“显示”五组工具
- “修改”面板保留 `移动`、`删除`、`旋转`、`复制`、`缩放`、`阵列` 按钮，并通过标题下拉暴露 `矩形阵列`、`环形阵列`、`镜像`、`偏移`、`修剪`、`延申`、`合并`、`圆角`、`直角（倒角）`
- “显示”面板提供 `显示机加工相关` 开关，用于统一控制加工方向箭头与加工顺序编号显示
- `机加工` 页签当前收束为“导入导出”“排序”“功能”“配置”“G代码配置”五组工具
- `机加工` 页签当前提供 `文件导入`、`G代码导出`、`排序(保留方向)`、`智能排序`、`去重`、`当前配置`、`G代码模式`、`G代码配置` 等高频入口
- `当前配置` 下拉框会自动列出运行目录中的 JSON Profile，并支持运行时实时切换当前生效配置
- `当前配置` 按钮会打开配置文件管理界面，可集中查看配置检索路径、各路径下的 JSON 配置文件，并执行删除或切换使用
- 通过信号把绘图、修改、改图层、改颜色、导入导出、排序与 G 代码模式切换请求回传给主窗口

`CadViewer`

- 负责 `QOpenGLWidget` 生命周期和事件入口
- 管理相机、场景刷新、坐标转换
- 将键鼠事件转交 `CadController`
- 消费 `CadDocument` 数据并驱动渲染
- 对外提供开始绘图、开始移动、设置默认绘图属性等薄封装接口
- 对外提供交互取点坐标解析，并统一执行对象捕捉与网格捕捉
- 接收主窗口下发的主题，并统一应用到背景、网格和加工编号气泡显示
- 在浅色主题下对低对比度实体颜色执行显示补偿，以保证图元可读性
- 负责加工方向箭头 overlay 与加工顺序屏幕编号显示，并响应主窗口 / Ribbon 下发的显示开关
- 负责加工顺序编号框的命中检测与交互分发，支持双击切换方向、两次点击交换加工顺序
- 负责绘制当前选中图元的基点/控制点手柄 overlay
- 当前实现已按 `CadViewer.cpp`、`CadViewer_EventHandling.cpp`、`CadViewer_ViewNavigation.cpp`、`CadViewer_SelectionState.cpp`、`CadViewer_SelectionPreview.cpp`、`CadViewer_ProcessOrderLabels.cpp`、`CadViewer_InteractionOverlays.cpp`、`CadViewer_TransientPrimitives.cpp`、`CadViewer_Snapping.cpp` 拆分

`CadCommandLineWidget` + `CadStatusPaneWidget`

- `CadCommandLineWidget` 负责显示命令提示与命令历史
- `CadStatusPaneWidget` 负责显示当前鼠标世界坐标
- `CadStatusPaneWidget` 当前提供“捕捉设置”下拉入口，集中管理基点 / 控制点 / 端点 / 中点 / 圆心(中心) / 交点 / 网格七类捕捉开关，并通过信号连接到 Viewer

`CadController` + `DrawStateMachine`

- 解释鼠标、键盘、滚轮输入
- 分流视图命令、绘图命令和编辑命令
- 维护当前命令状态、控制点、提示文本
- 维护旋转、缩放、复制、镜像、阵列、偏移等修改类命令的动态输入阶段与 transient 预览状态
- 保存默认绘图图层、默认绘图颜色索引等绘图状态
- 处理多段线圆弧/直线输入切换
- 当前实现已按 `CadController.cpp`、`CadController_KeyHandling.cpp`、`CadController_MouseHandling.cpp`、`CadController_DynamicInput.cpp` 拆分

`CadEditer`

- 根据状态机创建新实体
- 执行删除、移动、旋转、复制、缩放、矩形阵列、环形阵列、镜像、偏移、修剪、延申、合并、圆角、直角（倒角）、改色、改图层
- 批量旋转、缩放、复制、矩形阵列等多图元修改通过批量命令对象提交，保证一次用户操作对应一次 Undo / Redo 事务
- 执行反向加工切换、加工顺序写入和批量排序状态提交
- 执行闭合路径起刀缝点写入，并将其纳入 Undo / Redo
- 维护 Undo / Redo 命令栈
- 将模型修改统一提交给 `CadDocument`
- 当前实现已按 `CadEditer.cpp`、`CadEditer_CommandActions.cpp`、`CadEditer_DrawHandlers.cpp`、`CadEditer_EntityFactories.cpp`、`CadEditer_ModifyBuilders.cpp`、`CadEditer_EntityTransformHelpers.cpp`、`CadEditer_GeometryHelpers.cpp`、`CadEditer_GeometryMath.cpp` 拆分

`CadDocument`

- 持有原始 `dx_data`
- 持有场景中的 `CadItem` 容器
- 把导入得到的 `DRW_Entity` 转换成内部图元
- 维护图层表、图层颜色查询与缺失图层补建
- 通过 `sceneChanged` 驱动视图刷新

`CadBitmapImportDialog` + `CadBitmapVectorizer`

- 负责位图导入参数配置、预览和错误提示
- 使用 OpenCV 完成灰度化、阈值化、形态学处理、轮廓提取
- 将轮廓拟合为点、线、圆、圆弧、椭圆、多段线等实体
- 将生成实体追加或替换到当前文档

`GProfile` + `GGenerator`

- `GProfile` 负责读取、保存 JSON 格式的 G 代码配置
- 当前配置范围包括文件级、实体类型、图层规则、颜色规则以及四轴加工参数
- 颜色规则优先使用 `BYLAYER`、`BYBLOCK` 与 `ACI:n` 索引键，也兼容旧的真彩色 `#RRGGBB` 键
- `GProfileDialog` 负责提供用户侧可编辑配置界面，并显式跟随当前外观设置
- `GProfileManagerDialog` 负责配置文件级管理：左侧维护配置检索路径，右侧按路径列出配置文件，支持删除文件、双击使用和底部“使用”按钮切换当前配置
- `GProfilePathStore` 负责将用户维护的配置检索路径持久化，下次启动后继续参与配置扫描
- 配置对话框当前已拆分“恢复3轴默认 / 恢复4轴默认”，其中：
  - 3轴默认说明为 `3轴G加工默认文件配置`
  - 4轴默认说明为 `4轴G加工默认文件配置`
- `GGenerator` 当前同时承担 `2D` 导出与 `Mode3D` 四轴导出组织
- 四轴模式下，`GGenerator` 不再为所有图元统一硬编码机床点，而是调用各 `CadItem` 子类自己的 `rebuildRawPathPoints3D()` / `rebuildControlPoints4Axis()`
- 各图元子类负责把自身几何下沉为 `m_rawPathPoints3D` 与 `m_controlPoints4Axis`；`GGenerator` 负责跨图元连接、空跑控制、安全高度与 NC 输出
- 针对方管类工件，`GGenerator` 会在 `Mode3D` 导出开始时从所有原始路径点推断 `YZ` 截面边界和四角圆心，并据此统一修正直边 / 圆角区域的刀头方向
- 导出时会跟随排序阶段写回的加工方向与闭合路径起刀缝点
- 主窗口启动时会自动扫描程序运行目录下的 JSON Profile，并与内置 3轴 / 4轴默认配置一起组成当前可选配置列表

`CadSceneCoordinator` + `CadGraphicsCoordinator`

- `CadSceneCoordinator` 负责文档绑定、边界计算、GPU 缓冲重建时机
- `CadGraphicsCoordinator` 负责 OpenGL 状态、Shader、网格、坐标轴和 Overlay 渲染
- `CadEntityRenderer` 负责实体绘制
- `CadEntityPicker` 负责屏幕空间拾取
- `CadPreviewBuilder` / `CadCrosshairBuilder` 负责 transient 预览

`CadProcessVisualUtils`

- 负责统一计算图元的加工起点、终点、方向与编号锚点
- 负责为点、直线、构造线、圆、圆弧、椭圆、多段线、轻量多段线构建选中态基点/控制点手柄语义
- 供 Viewer 的加工辅助显示、主窗口排序逻辑与后续导出语义复用

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

1. 用户通过快捷键或 Ribbon 工具面板进入绘图或编辑命令
2. `CadController` 更新 `DrawStateMachine`
3. `CadViewer` 先将鼠标位置投影到 `Z=0` 绘图平面，并按当前“捕捉设置”执行对象捕捉与网格吸附修正
4. `CadEditer` 根据当前状态创建或修改 `DRW_Entity`
5. `CadDocument` 更新内部图元并发出 `sceneChanged`
6. Viewer 自动刷新，Undo / Redo 栈按事务规则更新

### G 代码生成

1. `GGenerator` 绑定当前 `CadDocument`
2. 生成时通过 `GProfile` 读取文件级、图层规则、颜色规则、实体类型配置
3. `Mode2D` 下，`GGenerator` 直接解析 `CadItem` 对应的原始实体参数，包括图层名和颜色键
4. 当前 `2D` 输出支持：
   `Line -> G01`
   `Arc/Circle -> G02/G03 + I/J`
   `Polyline/LWPolyline -> 直线段 + bulge 圆弧段`
   `Ellipse -> 离散为 G01`
5. `Mode3D` 下，各图元子类先生成 `rawPathPoints3D` 与 `controlPoints4Axis`，当前主要围绕绕 `X` 轴回转、`A` 轴展开的四轴场景
6. 四轴模式下，`GGenerator` 会按照 `CadItem::m_processOrder` 组织图元，并从全图 raw path 的 `YZ` 投影推断方管截面范围与四角圆心
7. 方管直边路径按对应固定加工面垂直加工；方管四角圆弧路径按对应四分之一圆弧圆心计算刀头方向
8. 图元边界会对齐 `A` 角连续性，避免例如 `180 -> -90` 这类优弧空跑；若连续加工路径需要改变刀头方向，则沿后续真实加工段渐变 `A` 轴，不在接点原地调头
9. 四轴模式下，图元间空跑采用“导出开始时一次性解算的全图最大离 `X` 轴距离 + 四轴离轴额外距离”作为统一安全高度，默认额外距离为 `5`
10. 四轴模式下，安全路径的抬升、横移、转 `A`、下落全部使用 `G00`；从安全高度回到加工起点的移动不视为加工轨迹
11. 若当前 Profile 启用了四轴初始机床点，生成器会在进入首个待加工图元前先输出这段初始定位
12. 导出时会按“文件级 -> 图层规则 -> 颜色规则 -> 实体类型”的顺序组合头尾代码块
13. 对 `Circle` 默认以顶部为起刀点；对完整 `Ellipse` 与闭合 `Polyline/LWPolyline`，会按排序阶段确定的缝点导出
14. 导出时会跳过空白代码块中的无意义空行，减少对机床无效的文本噪声，并统一使用 `CRLF` 行尾
15. 生成器在导出时弹出文件保存对话框并输出 `.nc/.gcode/.txt`

### 当前四轴适配说明

- 当前四轴链路面向“工件绕 `X` 轴回转、`A` 轴联动”的场景，不是通用五轴或任意姿态机床后处理
- 四轴机床控制点的生成已下沉到各图元子类；每个子类自行把自身几何转换为 `rawPathPoints3D` 与 `controlPoints4Axis`
- `CadItem` 基类统一维护 `m_rawPathPoints3D`、`m_controlPoints4Axis` 缓存，并提供角度归一化 / 连续展开的公共工具
- `GGenerator` 在四轴模式下只负责：
  - 按 `m_processOrder` 组织图元
  - 调用各图元生成机床控制点
  - 处理图元边界的 `A` 角短路径连续化
  - 推断方管 `YZ` 截面边界与四角圆心
  - 将方管直边 / 圆角区域映射为对应的刀头方向
  - 统一计算空跑安全高度
  - 输出最终 NC
- 当前已接入四轴控制点生成的图元是：
  - `Line`
  - `Arc`
  - `Circle`
  - `Ellipse`
  - `Polyline`
  - `LWPolyline`
- `Point` 当前不输出加工轨迹

### 排序与方向控制

1. 用户可先通过“反向加工”修改当前选中图元的加工方向
2. “排序（保留方向）”会在保留当前 `m_isReverse` 的前提下，仅重新计算 `m_processOrder`
3. 当前统一的“智能排序”入口会按 `G代码模式` 自动分流；在 `3轴` 下会同时优化加工顺序，并在需要时自动调整部分图元的 `m_isReverse`
4. `3轴` 智能排序会结合工件整体分布估计一个主扫掠方向，优先沿该方向推进，并尽量减少明显回头
5. 对 `Circle`、完整 `Ellipse`、闭合 `Polyline`、闭合 `LWPolyline`，`3轴` 智能排序会联合优化加工方向与起刀缝点，以减小缝点处顿挫
6. 两类排序结果都会写回 `CadItem::m_processOrder`；智能排序还会同步写回闭合路径的起刀缝点参数，并纳入 Undo / Redo
7. Viewer 会同步显示加工方向箭头与加工顺序编号，便于直接核对排序结果
8. 当前编号框支持直接交互：双击编号框可切换对应图元的加工方向，先点一个编号框再点另一个编号框可互换两个图元的加工顺序
9. 当前排序已增加对“小间隙相邻图元”的连续加工倾向：若两个同路径候选之间只有很小的连接空隙，会优先保持顺接，减少先跳去别处再返回补加工的情况
10. `4轴(绕A)` 排序当前会优先把最接近参考点 `(0, 0, 50)` 的图元作为首个待加工图元，以减少一开始就发生较大的 `A` 轴翻转

## 使用说明

### 支持导入的文件类型

- CAD：`.dxf`、`.dwg`
- 位图：`.bmp`、`.png`、`.jpg`、`.jpeg`

### 当前支持显示与编辑的图元

- `Point`
- `Line`
- `Xline`
- `Circle`
- `Arc`
- `Ellipse`
- `Polyline`
- `LWPolyline`

说明：

- `libdxfrw` 底层可读取更多实体类型
- 但当前 [src/CadDocument.cpp](D:/projects/visual_studio_2026/G-code_post-processing_system/src/CadDocument.cpp) 中 `createCadItemForEntity()` 只会把上面 8 类实体转换为项目内部 `CadItem`
- 其它实体即使被读入，也不会生成当前可显示、可编辑的内部图元

### 视图交互

- `F`：适配场景
- `T` 或 `Home`：回到顶视图
- `+` / `=`：放大
- `-` / `_`：缩小
- 中键拖动：平移
- `Shift + 中键拖动`：轨道旋转
- 左键：拾取图元
- 左键拖拽窗口框选：向左拖拽为碰选，向右拖拽为包含选
- `Shift + 左键`：对命中图元执行增选/反选
- `Shift + 左键拖拽框选`：对框内命中图元执行增选/反选
- 滚轮：以鼠标附近为锚点缩放

加工辅助显示补充：

- Viewer 会为可参与加工的图元绘制方向箭头；当前样式为“箭杆 + 尖端位于加工起始点的三角箭头头部”，不再附带额外圆点标记
- 已设置 `m_processOrder` 的图元会显示屏幕空间编号气泡，编号从 `1` 开始
- 当前默认配色中，绿色表示正向加工，红色表示反向加工，黄色表示当前选中图元
- 选中图元后会额外显示基点与控制点手柄，其中基点使用更大的圆点，圆弧/椭圆边界端点会使用三角箭头手柄
- 选中图元会显示轻描边与四角角标（替代完整矩形包围框），框选预览候选图元会实时显示低优先级高亮
- 基点/控制点手柄会在鼠标悬停时显示实时高亮环，便于确认可编辑目标
- 平移或轨道观察过程中会临时暂停加工辅助显示，以优先保证交互流畅度

### 吸附与状态栏

底部状态栏当前除了显示坐标，还提供单一 `捕捉设置` 下拉按钮，集中管理以下捕捉开关：

- `基点`：吸附到当前选中图元的基点，如点坐标、线起点、圆心、圆弧圆心、多段线首点等
- `控制点`：吸附到当前选中图元的控制点，如线终点、圆四象限点、圆弧起中终点、椭圆轴端点、多段线其余顶点等
- `端点`：吸附到场景图元的几何端点（如线段端点、弧端点、多段线顶点）
- `中点`：吸附到线段中点，以及弧段/多段线离散段中点
- `圆心/中心`：吸附到圆、圆弧、椭圆中心
- `交点`：吸附到光标附近图元离散线段之间的交点
- `网格`：当未命中对象捕捉点时，把当前位置量化到当前动态步长网格（随缩放自动调整）

当前行为说明：

- 命令取点、移动命令取点和底部状态栏坐标显示都会使用吸附后的坐标
- 基点/控制点吸附仅对“当前选中图元”生效；端点/中点/圆心(中心)/交点会在当前场景范围参与计算
- 对象捕捉优先级高于网格捕捉；在对象捕捉中，优先级大致为 `基点 > 端点 > 交点 > 圆心/中心 > 中点 > 控制点`
- 当前对象吸附阈值与网格吸附阈值均为固定屏幕距离阈值
- 十字准线与拾取框会跟随吸附后的坐标；命中吸附点时会显示额外高亮标记
- 捕捉选项状态会写入 `QSettings`（键：`ui/snapModeMask`），重启后自动恢复；若无历史配置则使用默认掩码
- 默认掩码为：`基点`、`控制点`、`端点`、`中点`、`圆心/中心`、`网格` 默认开启；`交点` 默认关闭（避免在复杂场景下引入额外交互开销）

### 绘图命令

- `P`：点
- `L`：直线
- `X`：构造线
- `R`：矩形（底层创建为闭合多段线）
- `G`：多边形（底层创建为闭合多段线，支持边数与内切 / 外切选项）
- `C`：圆
- `A`：圆弧
- `E`：椭圆
- `O`：多段线
- `W`：轻量多段线
- `Esc`：取消当前命令；当无活动命令且存在选中图元时，清空当前选中

多段线命令补充：

- `A`：切换为圆弧段输入
- `L`：切回直线段输入
- `Enter` / `Space`：结束多段线
- `C`：闭合多段线

### 编辑命令

- `Delete`：删除当前选中图元（支持多选）
- `M`：移动当前选中图元（支持多选）
- `K`：修改当前选中图元颜色（支持多选）
- 空闲态左键点击“当前选中图元”的可编辑控制点，再左键指定目标点：控制点编辑（第一阶段）
- `Ctrl + Z`：撤销
- `Ctrl + Y`：重做
- `Ctrl + Shift + Z`：重做
- `Ctrl + S`：保存当前文档（写回当前同名 `.dxf`）

补充说明：

- `旋转` 当前通过 Ribbon“修改”面板或动态命令触发，支持多选；流程为指定旋转基点、移动鼠标实时预览角度，也可直接输入角度确认
- `缩放` 当前通过 Ribbon“修改”面板或动态命令触发，支持多选；流程为指定缩放基点、移动鼠标实时预览比例，也可直接输入倍率确认
- `复制` 当前通过 Ribbon“修改”面板或动态命令触发，支持多选；流程为指定基点、指定第二点或输入相对坐标，复制结果会实时预览
- `偏移` 当前通过 Ribbon“修改”面板或动态命令触发；流程为输入偏移距离后点击图元偏移侧，预览会按当前侧向显示
- `镜像` 当前支持按镜像线两点创建预览，并可在最后一步选择是否删除原图元
- `矩形阵列` 当前支持多选并按行数、列数、行间距、列间距生成预览，提交后作为一次批量命令进入 Undo / Redo
- `环形阵列` 当前支持多选并按项目总数、填充角度、阵列中心与是否旋转副本方向生成预览
- `修剪`、`延申`、`圆角`、`直角（倒角）`、`合并` 当前已接入 Ribbon 与动态命令入口，但底层几何能力仍以现有基础实现为准，尚不是完整 AutoCAD 几何内核
- 控制点编辑当前为第一阶段实现：采用“两次点击提交”交互，当前已支持实时预览与重叠夹点候选选择；已覆盖 `Point`、`Line`、`Xline`、`Circle`、`Arc`、`Ellipse`、`Polyline`、`LWPolyline` 的可编辑夹点
- 空闲态动态命令框已接通以下常用命令入口：`line`、`xline`、`rectangle`、`polygon`、`point`、`circle`、`arc`、`ellipse`、`polyline`、`lwpolyline`、`move`、`copy`、`rotate`、`scale`、`mirror`、`offset`、`arrayrect`、`arraypolar`、`trim`、`extend`、`join`、`fillet`、`chamfer`、`delete`、`color`、`fit`、`top`、`zoomin`、`zoomout`

### 动态命令框

- 仅在空闲态生效；当不在活动命令中，直接键盘输入即可进入命令匹配
- 支持英文全称、常见缩写和部分中文别名匹配
- `Tab`：下一个候选；`Shift + Tab`：上一个候选
- `Enter` 或 `Space`：执行当前高亮候选命令
- `Backspace`：删除输入字符；`Delete`：清空命令输入
- `Esc`：取消动态命令框，不影响其它常规交互状态

### Ribbon 工具面板

主窗口工具栏区域当前提供一组仿 AutoCAD 风格的紧凑工具面板，并按页签分为 `默认` 与 `机加工` 两部分：

- `默认`：保留 CAD 编辑主工作流，左对齐排列 `绘图`、`修改`、`图层`、`特性`、`显示` 五个面板
- `机加工`：采用与 `默认` 页签一致的紧凑 panel 风格，收纳高频加工操作，减少反复使用菜单栏的步骤

`默认` 页签中的面板功能如下：

- `绘图`：默认显示常用图元，隐藏图元可通过标题旁网格式下拉菜单进入；矩形与多边形最终创建为闭合多段线
- `修改`：当前提供 `移动`、`删除`、`旋转`、`复制`、`缩放`、`阵列`，标题旁网格式下拉菜单补充 `镜像`、`偏移`、`矩形阵列`、`环形阵列`、`修剪`、`延申`、`合并`、`圆角`、`直角（倒角）`
- 修改面板中的主要修改命令统一使用光标旁动态输入面板完成参数输入；旋转、缩放、复制、镜像、偏移、矩形阵列、环形阵列已提供操作过程预览
- `图层`：提供单一下拉框；闭合时显示当前图层，展开后列出全部图层，每项前面显示该图层颜色小方块
- `特性`：当前提供图层与颜色两项；图层项显示所属图层，颜色项以色块图标 + 文字方式展示
- `显示`：提供 `显示机加工相关` 开关，统一控制 Viewer 中加工方向箭头与加工顺序编号的显示状态

`机加工` 页签当前包含五组快捷面板：

- `导入导出`：`文件导入`、`G代码导出`
- `排序`：`排序(保留方向)`、`智能排序`
- `功能`：`自动去重`、`去重`、`使用默认导出路径`、`使用dxf文件名`
- `配置`：`当前配置` 下拉框
- `G代码配置`：`G代码模式` 下拉框与 `G代码配置`

其中 `G代码模式` 当前提供：

- `自动`
- `3轴`
- `4轴(绕A)`

属性联动规则：

- 未选中图元时，`图层` 与 `特性` 显示“默认绘图属性”，用于控制后续新建图元
- 选中图元时，`图层` 与 `特性` 自动切换为显示该图元的真实属性
- 修改图层或颜色时，主窗口会根据当前是否有选中图元，决定写回默认绘图状态或直接修改图元
- 当图元带有自定义颜色时，`特性 -> 颜色` 中 `ByLayer` 项的小方块仍固定显示所属图层的默认颜色，而不是图元自己的颜色

### 外观设置

菜单栏当前提供 `用户设置 -> 外观设置`：

- `主题 -> 浅色模式 / 深色模式` 会统一作用于菜单栏、工具栏、Ribbon 面板、Viewer、命令栏和底部状态栏
- `自定义外观...` 可从浅色模式或深色模式继承，并调整 Viewer、网格、选中高亮、机加工方向箭头、编号标签等显示颜色
- `用户设置 -> G代码配置...` 打开的 Profile 对话框也会显式跟随当前外观，而不依赖系统默认样式
- 外观选择会通过 `QSettings` 持久化，下次启动时自动恢复上次选择
- 浅色主题主要面向截图、文档和论文打印场景；深色主题主要面向日常长时间使用

浅色主题下的显示补偿规则：

- 如果实体颜色与 Viewer 背景对比度过低，渲染层会自动做显示补偿
- 对白色或近白色的中性色，优先显示为深色可见线
- 对带有色相但过亮的颜色，优先做保色加深，而不是一律改成黑色
- 该补偿仅影响屏幕显示，不修改文档中的真实图层颜色、实体颜色或导出语义

### 菜单动作

- 文件 -> `导入文件`：导入 `.dxf` / `.dwg` / 位图文件
- 文件 -> `导入DXF...`：仅导入 `.dxf`
- 文件 -> `导入DWG...`：仅导入 `.dwg`
- 文件 -> `导入图片`：直接进入位图导入对话框
- 文件 -> `保存文件`：保存当前文档（快捷键 `Ctrl+S`，默认覆盖当前同名 `.dxf`）
- 文件 -> `导出为DXF...`：普通导出（尽量保留当前实体）
- 文件 -> `导出为DXF（安全模式）...`：安全导出（仅导出当前支持图元、清理扩展字段、普通 `POLYLINE` 转 `LWPOLYLINE`）
- 文件 -> `导出G代码`：按当前 `G代码模式` 自动选择 `3轴` 或 `4轴(绕A)` 导出路径
- 编辑 -> `反向加工`：切换当前选中图元的加工方向
- 排序 -> `排序（保留方向）`：按当前 `G代码模式` 自动调用 `3轴` 或 `4轴(绕A)` 排序逻辑，并保留当前方向设置
- 排序 -> `智能排序`：按当前 `G代码模式` 自动调用对应智能排序逻辑；在 `3轴` 下会同时优化加工顺序、图元方向和闭合路径起刀缝点
- 用户设置 -> `外观设置 -> 主题 -> 浅色模式 / 深色模式`：切换整套界面主题
- 用户设置 -> `外观设置 -> 自定义外观...`：打开自定义外观对话框
- 用户设置 -> `G代码配置...`：打开当前活动 Profile 配置对话框
- 帮助 -> `快速上手`：打开基础操作说明
- 帮助 -> `快捷命令` / `绘图教程` / `修改教程` / `机加工 / G代码` / `位图导入` / `外观与显示` / `关于`：按分类打开内置帮助文档

说明：

- 执行“反向加工”或两类排序后，Viewer 中的方向箭头和顺序编号会立即刷新
- Viewer 中的加工顺序编号框支持直接交互：双击编号框切换方向，依次点击两个编号框交换加工顺序
- 顺序编号、箭头方向与排序逻辑当前共用同一套加工路径语义

### G 代码生成说明

当前后处理后端已经具备以下能力：

- 支持纯 `2D` G 代码生成
- 支持 `Line`、`Arc`、`Circle`、`Ellipse`、`Polyline`、`LWPolyline`
- 支持读取 `CadItem::m_processOrder` 作为导出顺序
- 支持读取 `CadItem::m_isReverse` 作为反向加工标记
- 支持读取闭合图元的起刀缝点参数，并与 Viewer / 排序共用同一套路径语义
- 支持按 `GProfile` 套用文件级、图层规则、颜色规则、实体类型规则

`G代码配置` 对话框当前支持：

- 编辑 Profile 名称
- 编辑文件级 `header / footer / comment`
- 编辑实体类型级 `header / footer / comment`
- 编辑图层级 `header / footer / comment`
- 编辑颜色级 `header / footer / comment`
- 编辑 `四轴加工` 配置页中的旋转轴附加参数
- 导入 / 导出 JSON
- 恢复默认配置

当前 `四轴加工` 配置页已接通的参数包括：

- 四轴离轴额外距离：用于在一次导出开始时，以“全图最大离 `X` 轴半径 + 额外距离”解算统一安全高度；默认值为 `5`
- 加工面 `Z` 修正：在实时加工面上追加可正可负的 `Z` 方向补偿，用于调整喷口 / 刀头贴面距离
- `A` 轴中心、角度偏移、方向翻转、连续展开和初始机床点等基础旋转轴参数

颜色规则说明：

- 默认内置 `BYLAYER`、`BYBLOCK`、`ACI:1` 到 `ACI:9`
- 其中 `ACI:1` 到 `ACI:9` 对应基础 AutoCAD 索引颜色
- 在此基础上仍可新增真彩色规则，用于特殊工艺场景

当前限制：

- `Point` 当前不会输出加工轨迹
- `Ellipse` 当前通过折线离散输出，不使用专门椭圆插补指令
- 闭合 `Polyline/LWPolyline` 的缝点优化当前以顶点为候选集合，不做边中点或弧长连续参数搜索
- `Mode3D` 当前已切换到新的四轴导出主链，但适配重点仍集中在绕 `X` 轴回转、`A` 轴展开的截面类 / 方管类工件
- 方管圆角圆心当前从 `YZ` 投影的直边切点推断；图纸缺失直边切点或截面不完整时，需要人工核对导出注释中的圆心信息

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

## 已知注意事项

- 当前内部完整支持 8 类 CAD 图元的可视化与编辑；矩形、多边形不新增图元类，创建结果为闭合多段线
- 新建图元强制落在 `Z=0` 平面
- Ribbon“绘图”面板当前只把常用图元直接显示，其余入口需通过标题旁网格式下拉菜单进入
- 浅色主题下 Viewer 会对低对比度线色做显示补偿；这是显示层行为，不会改写文档中的真实颜色数据
- `GProfile` 颜色规则当前以基础索引颜色与真彩色键混合管理，后续如需扩展完整 AutoCAD ACI 颜色表，还需要继续补齐 UI 与预设
- 位图导入依赖 OpenCV 运行时 DLL 拷贝到输出目录
- `Mode3D` 当前已经切到“图元类自生成机床控制点 + GGenerator 负责跨图元组织”的新链路，但还处于持续完善阶段
- 当前四轴导出的工艺语义和边界策略主要围绕绕 `X` 轴回转、`A` 轴展开的方管 / 截面类工件收敛，不应视为通用四轴或五轴后处理
- 方管四角刀头方向依赖 `YZ` 投影中的四分之一圆弧圆心推断，复杂或不完整图纸需要结合导出注释与实际机床工艺复核
- 统一“排序（保留方向）”入口当前尊重用户已设置的加工方向与闭合路径缝点；除菜单动作外，也可通过 Viewer 中的编号框直接交换图元加工顺序
- `2D` 智能排序当前采用“局部最近邻 + 整体扫掠方向偏好 + 切线连续性惩罚”的启发式策略，目标是减少回头与顿挫，但不是严格全局最优解
- 加工方向箭头当前按世界坐标中的二维方向绘制，样式已简化为箭杆加起点三角箭头头部；编号气泡会在屏幕空间做简单避让，但密集图元场景下仍可能出现局部遮挡
- 当前吸附主要作用于命令取点和状态栏坐标显示，普通空闲态拾取/选择仍按原始屏幕位置进行
- 基点/控制点吸附仅在图元已选中、且对应手柄可见时才会生效；端点/中点/圆心(中心)/交点不要求图元先选中
- 当前对象捕捉已覆盖基点、控制点、端点、中点、圆心(中心)、交点；暂不包含切点、垂足、象限点锁定等更完整 CAD 捕捉类型
- 控制点编辑当前仍为第一阶段实现，采用“两次点击提交”模式；已支持实时预览与重叠夹点候选选择
- 修改命令的交互已向 AutoCAD 风格靠齐，但 `修剪`、`延申`、`圆角`、`直角（倒角）`、`合并` 的底层几何仍是基础实现；当前主要覆盖直线及相接线性图元等有限场景，不应视为完整 AutoCAD 级任意曲线拓扑编辑
- 偏移预览当前按已有偏移构造能力显示，复杂多段线、圆弧组合或自交场景仍需后续继续验证
- `Ellipse` 当前按折线离散导出，精度取决于固定采样密度
- 第三方 `libdxfrw` 目录不应轻易修改
- 工程里仍存在部分历史遗留命名，修改构建配置前需要先核对 `.ui`、`.vcxproj` 与当前源码是否一致

## 协作约束摘要

- 默认使用中文沟通
- 遵循 MVC / 分层思路，避免把过多逻辑重新堆回 `CadViewer`
- 修改 `src/libdxfrw/` 或 `include/libdxfrw/` 时应明确说明原因和影响范围
- OpenGL 保持 4.5 Core Profile

