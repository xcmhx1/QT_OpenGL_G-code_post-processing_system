// 实现 CadHelpDialog 模块，集中维护菜单“帮助”中的分类说明。
#include "platform/pch.h"

#include "ui/dialogs/CadHelpDialog.h"

#include <QCoreApplication>
#include <QDialogButtonBox>
#include <QEvent>
#include <QGuiApplication>
#include <QMouseEvent>
#include <QPushButton>
#include <QTabWidget>
#include <QTextBrowser>
#include <QVBoxLayout>

namespace
{
    QString wrapHelpHtml(const QString& title, const QString& body)
    {
        return QString::fromUtf8(R"HTML(
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<style>
body { font-family: "Microsoft YaHei", "Segoe UI", sans-serif; font-size: 10pt; line-height: 1.58; }
h1 { font-size: 18pt; margin: 0 0 12px 0; }
h2 { font-size: 13pt; margin: 18px 0 8px 0; }
h3 { font-size: 11pt; margin: 12px 0 6px 0; }
p { margin: 6px 0; }
ul { margin: 6px 0 10px 18px; padding: 0; }
li { margin: 4px 0; }
table { border-collapse: collapse; width: 100%; margin: 8px 0 14px 0; }
th, td { border: 1px solid #9aa3ad; padding: 6px 8px; vertical-align: top; }
th { font-weight: 600; }
kbd { border: 1px solid #8c949d; border-radius: 3px; padding: 1px 5px; font-family: Consolas, monospace; }
code { font-family: Consolas, monospace; }
.note { border-left: 4px solid #4f89d8; padding: 6px 10px; margin: 8px 0; }
</style>
</head>
<body>
)HTML")
            + QStringLiteral("<h1>%1</h1>").arg(title)
            + body
            + QStringLiteral("</body></html>");
    }

    QString quickStartHtml()
    {
        return QString::fromUtf8(R"HTML(
<p>建议先确认加工模式和配置，再按“导入或绘制 → 检查图形 → 设置加工 → 排序 → 检查 → 导出”的顺序操作。</p>
<h2>平面三轴</h2>
<ol>
<li>通过“文件 -> 导入文件”导入 <code>DXF/DWG</code>，也可以直接把文件拖入视图区。</li>
<li>“文件 -> 最近打开”可以快速重新打开最近使用的 <code>DXF/DWG</code> 文件，最多保留 15 项。</li>
<li>在“默认”选项卡中检查、绘制或修改图元，必要时调整图层和颜色。</li>
<li>在“机加工 -> 配置”选择 <code>3轴</code> 和当前 G 代码配置。</li>
<li>执行“排序(保留方向)”或“智能排序”。</li>
<li>确认顺序标签和方向箭头后，点击“G代码导出”。</li>
</ol>
<h2>方管四轴</h2>
<ol>
<li>导入并检查 CAD 图形，在“机加工 -> 配置”选择 <code>4轴(绕A)</code>。</li>
<li>普通四轴可直接排序；需要表面、圆角、加工断面或内部线能力时，先识别或设置方管截面。</li>
<li>根据工艺需要清理内部线、识别加工断面，并在 G 代码配置中设置安全距离、过切和懒旋转。</li>
<li>排序后检查每个加工单元的编号、起点箭头和断面标记，再导出 G 代码。</li>
</ol>
<h2>交互原则</h2>
<ul>
<li>大多数绘图和修改命令都使用光标旁输入面板，不再依赖弹窗输入参数。</li>
<li><kbd>Enter</kbd>、<kbd>Space</kbd> 或右键用于确认当前命令步骤。</li>
<li><kbd>Esc</kbd> 用于取消当前命令或退出动态命令输入。</li>
<li>底部状态栏统一控制捕捉、正交和极轴追踪。</li>
<li>“显示”选项卡只控制画布显示，不改变加工状态、排序结果或输出。</li>
</ul>
<div class="note">导出前必须检查加工模式、配置、顺序、方向、安全移动和工件坐标。实际加工前还应完成仿真和空跑。</div>
)HTML");
    }

    QString shortcutsHtml()
    {
        return QString::fromUtf8(R"HTML(
<h2>文件与撤销</h2>
<table>
<tr><th>快捷键</th><th>功能</th></tr>
<tr><td><kbd>Ctrl</kbd> + <kbd>S</kbd></td><td>保存当前文档为 DXF。</td></tr>
<tr><td><kbd>Ctrl</kbd> + <kbd>Z</kbd></td><td>撤销。</td></tr>
<tr><td><kbd>Ctrl</kbd> + <kbd>Y</kbd> / <kbd>Ctrl</kbd> + <kbd>Shift</kbd> + <kbd>Z</kbd></td><td>重做。</td></tr>
</table>
<h2>视图</h2>
<table>
<tr><th>快捷键</th><th>功能</th></tr>
<tr><td><kbd>F</kbd></td><td>适配视图。</td></tr>
<tr><td><kbd>T</kbd> / <kbd>Home</kbd></td><td>切回顶视图。</td></tr>
<tr><td><kbd>+</kbd> / <kbd>=</kbd></td><td>放大。</td></tr>
<tr><td><kbd>-</kbd> / <kbd>_</kbd></td><td>缩小。</td></tr>
</table>
<h2>绘图快捷键</h2>
<table>
<tr><th>快捷键</th><th>命令</th></tr>
<tr><td><kbd>P</kbd></td><td>点</td></tr>
<tr><td><kbd>L</kbd></td><td>直线</td></tr>
<tr><td><kbd>X</kbd></td><td>构造线</td></tr>
<tr><td><kbd>R</kbd></td><td>矩形</td></tr>
<tr><td><kbd>G</kbd></td><td>多边形</td></tr>
<tr><td><kbd>C</kbd></td><td>圆</td></tr>
<tr><td><kbd>A</kbd></td><td>圆弧</td></tr>
<tr><td><kbd>E</kbd></td><td>椭圆</td></tr>
<tr><td><kbd>O</kbd></td><td>多段线</td></tr>
<tr><td><kbd>W</kbd></td><td>轻量多段线</td></tr>
</table>
<h2>编辑与辅助</h2>
<table>
<tr><th>快捷键</th><th>功能</th></tr>
<tr><td><kbd>Delete</kbd></td><td>删除选中图元。</td></tr>
<tr><td><kbd>M</kbd></td><td>移动选中图元。</td></tr>
<tr><td><kbd>K</kbd></td><td>修改选中图元颜色。</td></tr>
<tr><td><kbd>F8</kbd></td><td>切换正交。</td></tr>
<tr><td><kbd>F10</kbd></td><td>切换极轴追踪。</td></tr>
<tr><td><kbd>Tab</kbd> / <kbd>Shift</kbd> + <kbd>Tab</kbd></td><td>动态输入字段或候选项切换。</td></tr>
</table>
<h2>空闲态动态命令</h2>
<p>空闲时直接输入命令字符会弹出候选框，支持 <kbd>Tab</kbd> 切换、<kbd>Enter</kbd>/<kbd>Space</kbd> 执行。</p>
<table>
<tr><th>命令</th><th>常用别名</th></tr>
<tr><td><code>line</code></td><td><code>l</code>、直线</td></tr>
<tr><td><code>xline</code></td><td><code>x</code>、<code>xl</code>、构造线</td></tr>
<tr><td><code>rectangle</code></td><td><code>r</code>、<code>rect</code>、矩形</td></tr>
<tr><td><code>polygon</code></td><td><code>g</code>、<code>pg</code>、多边形</td></tr>
<tr><td><code>point</code> / <code>circle</code> / <code>arc</code> / <code>ellipse</code></td><td><code>p</code>、<code>c</code>、<code>a</code>、<code>e</code></td></tr>
<tr><td><code>polyline</code> / <code>lwpolyline</code></td><td><code>o</code>、<code>w</code></td></tr>
<tr><td><code>move</code> / <code>delete</code> / <code>color</code></td><td><code>m</code>、<code>del</code>、<code>k</code></td></tr>
<tr><td><code>fit</code> / <code>top</code> / <code>zoomin</code> / <code>zoomout</code></td><td><code>f</code>、<code>t</code>、<code>zin</code>、<code>zout</code></td></tr>
</table>
)HTML");
    }

    QString drawingHtml()
    {
        return QString::fromUtf8(R"HTML(
<h2>通用绘图步骤</h2>
<ul>
<li>从“默认 -> 绘图”选择图元，或使用绘图快捷键。</li>
<li>按命令提示点击点位，或直接在光标旁输入面板输入坐标 / 数值。</li>
<li>启用捕捉、正交、极轴后，鼠标点位和状态栏坐标都会按同一约束结果显示。</li>
</ul>
<h2>矩形与多边形</h2>
<ul>
<li>矩形实际创建为闭合多段线。</li>
<li>多边形实际创建为闭合多段线，边数范围为 3 到 1024。</li>
<li>多边形支持内切于圆和外切于圆；默认边数会沿用上一次设置。</li>
</ul>
<h2>多段线</h2>
<ul>
<li>多段线命令中，<kbd>A</kbd> 切换到圆弧段输入，<kbd>L</kbd> 切回直线段输入。</li>
<li><kbd>C</kbd> 可闭合当前多段线。</li>
</ul>
<h2>绘图平面</h2>
<p>当前绘图按二维 CAD 逻辑实现，新建图元统一落在世界坐标 <code>Z=0</code> 平面。</p>
)HTML");
    }

    QString editingHtml()
    {
        return QString::fromUtf8(R"HTML(
<h2>选择与批量修改</h2>
<ul>
<li>空闲态单击图元可选中；框选支持向右包含选、向左碰选。</li>
<li><kbd>Shift</kbd> + 框选用于图元级增量切换。</li>
<li>已有有效加工顺序时，<kbd>Ctrl</kbd> + 框选会把命中的图元扩展为完整加工单元；<kbd>Ctrl</kbd> + <kbd>Shift</kbd> + 框选按加工单元整体切换。</li>
<li>移动、删除、改色、复制、旋转、缩放、阵列、改图层、改颜色支持多选。</li>
</ul>
<h2>修改面板</h2>
<table>
<tr><th>功能</th><th>说明</th></tr>
<tr><td>移动</td><td>指定基点与目标点，支持实时预览。</td></tr>
<tr><td>复制</td><td>输入 X / Y 偏移量后批量生成副本。</td></tr>
<tr><td>旋转 / 缩放</td><td>默认绕选中集合几何包围盒中心执行。</td></tr>
<tr><td>矩形阵列 / 环形阵列</td><td>通过光标旁输入面板输入数量、间距、角度和中心等参数。</td></tr>
<tr><td>镜像 / 偏移</td><td>镜像按两点指定镜像线；偏移按输入距离生成偏移实体。</td></tr>
<tr><td>修剪 / 延申</td><td>基于选中图元和端点选择构造修改结果。</td></tr>
<tr><td>合并 / 圆角 / 倒角</td><td>用于相邻线段、多段线片段等几何构造。</td></tr>
</table>
<h2>控制点编辑</h2>
<p>选中图元后，点击可编辑控制点，再点击目标点完成控制点移动。重叠控制点悬停约 1 秒会弹出候选选择栏，可用 <kbd>Tab</kbd> 切换。</p>
)HTML");
    }

    QString machiningHtml()
    {
        return QString::fromUtf8(R"HTML(
<h2>加工单元</h2>
<p>相连的开放链或闭环形成一个加工单元，不连续的单个图元形成独立加工单元。排序、编号和人工编排都以加工单元为最小对象。</p>
<h2>加工显示</h2>
<ul>
<li>“显示 -> 画布显示”可分别控制加工方向箭头、加工顺序编号、加工断面标记和排除图元弱化。</li>
<li>每个加工单元只显示一个编号和一个起点方向箭头。</li>
<li>单击编号可选择整个加工单元；<kbd>Shift</kbd> + 单击可追加或取消该单元。</li>
<li>双击编号或方向箭头可反向整个加工单元，成员执行顺序和方向会一起反转。</li>
<li>选择一段编号连续的加工单元后，<kbd>Ctrl</kbd> + 单击范围内的编号可把目标移动到该范围末尾。</li>
</ul>
<h2>排序</h2>
<table>
<tr><th>入口</th><th>用途</th></tr>
<tr><td>排序(保留方向)</td><td>尽量保留当前加工单元相对顺序、人工方向和人工起点，并补入新增或变化的单元。</td></tr>
<tr><td>智能排序</td><td>忽略当前单元序列以及人工方向、起点，按当前模式重新计算全部加工单元；用户保存的人工状态不会被删除。</td></tr>
</table>
<p>几何、加工启用、方向、起点、内部线、加工断面或配置发生相关变化后，应重新排序。</p>
<h2>导出 G 代码</h2>
<ol>
<li>选择 G 代码模式：自动、3轴、4轴(绕A)。</li>
<li>选择当前配置；代码规则在“G代码配置”中设置，导入和导出习惯在“加工设置”中设置。</li>
<li>检查画布上的单元编号、起点箭头、排除状态和加工断面标记。</li>
<li>点击“G代码导出”。若文档未排序，系统会先按当前模式自动智能排序。</li>
</ol>
<div class="note">导出失败时不会写出部分程序。请根据命令栏中的失败阶段修正图形、加工状态或配置后重新排序。</div>
)HTML");
    }

    QString rotaryMachiningHtml()
    {
        return QString::fromUtf8(R"HTML(
<h2>普通四轴</h2>
<p>普通四轴适用于绕 X 轴、由 A 轴表示周向旋转的加工。有效加工路径、旋转轴中心、运动参数和排序结果齐全时，无方管截面也可以排序和导出。</p>
<h2>截面增强四轴</h2>
<ol>
<li>选中一个严格闭合的方管截面或其中一条边，点击“识别方管截面”。系统会扩展相连边界并显示 Y 长、Z 宽和圆角半径。</li>
<li>自动识别不合适时，可在右侧“加工设置 -> 方管识别状态”输入尺寸和圆角，并点击“应用手动截面”。</li>
<li>截面有效后，可使用“清理内部线条”和“识别加工断面”。</li>
<li>完成工艺设置后重新排序，再检查编号、箭头、断面和排除状态。</li>
</ol>
<h2>内部线</h2>
<ul>
<li>内部线清理要求当前存在有效方管截面。</li>
<li>点击一次“清理内部线条”，系统在截面中心生成 YZ 内缩窗口：外轮廓四边按最大圆角半径内缩，并附加额外安全内缩。</li>
<li>路径 YZ 范围与窗口相交（含接触）的图元会被物理删除；删除进入统一撤销栈，可按 Ctrl+Z 整体恢复。</li>
<li>与窗口不相交的图元会保留；清理不按外轮廓成员或断面角色过滤。</li>
<li>清理后可按 Ctrl+Z 整体恢复被删除的图元。</li>
</ul>
<h2>加工断面与废面</h2>
<ul>
<li>加工断面和废面都要求有效方管截面，并且候选图元必须形成严格闭环。</li>
<li>加工断面是工艺屏障：断面之前所属加工段的普通内容先完成，再加工该断面。</li>
<li>可从画布右键菜单指定、恢复加工断面，或一键清理内部线条。</li>
</ul>
<h2>四轴工艺参数</h2>
<table>
<tr><th>设置</th><th>说明</th></tr>
<tr><td>旋转安全抬刀距离</td><td>跨区位或需要旋转时，高于工件旋转包络的安全余量。</td></tr>
<tr><td>同区空移离面距离</td><td>同一区位加工单元之间空移时的离面距离，0 表示贴面转移。</td></tr>
<tr><td>联动转移</td><td>在安全约束下协调线性轴和 A 轴的离开、旋转与接近。</td></tr>
<tr><td>加工面 Z 修正</td><td>对四轴加工面位置施加统一补偿。</td></tr>
<tr><td>过切距离</td><td>闭合单元回到起点后继续沿原方向加工的距离，0 表示关闭。</td></tr>
<tr><td>懒旋转加工</td><td>截面有效时按方管区位集中加工，减少不必要的 A 轴往复旋转。</td></tr>
</table>
<div class="note">截面识别和自动排序属于辅助能力。导出前仍需人工检查表面归属、A 轴方向、安全距离和加工断面顺序。</div>
)HTML");
    }

    QString bitmapHtml()
    {
        return QString::fromUtf8(R"HTML(
<h2>支持文件</h2>
<p>位图导入支持 <code>bmp</code>、<code>png</code>、<code>jpg</code>、<code>jpeg</code>。</p>
<h2>导入流程</h2>
<ol>
<li>通过“文件 -> 导入图片”或“文件 -> 导入文件”选择位图。</li>
<li>在位图导入对话框中配置预处理、轮廓提取、拟合与插入参数。</li>
<li>预览确认后选择追加到当前文档或替换当前文档。</li>
</ol>
<h2>常用参数</h2>
<ul>
<li>自适应阈值适合光照不均的图片；固定阈值适合黑白边界明确的图片。</li>
<li>高斯模糊用于抑制噪点，但过强会吞掉小细节。</li>
<li>反向前景用于处理白底黑线和黑底白线的前景反转。</li>
<li>轮廓模式和折线拟合会直接影响最终图元数量与精度。</li>
</ul>
)HTML");
    }

    QString appearanceHtml()
    {
        return QString::fromUtf8(R"HTML(
<h2>主题与自定义外观</h2>
<ul>
<li>入口：用户设置 -> 外观设置。</li>
<li>“主题”中可以切换浅色模式或深色模式。</li>
<li>“自定义外观”可从浅色或深色继承，并修改界面、视图和机加工标注颜色。</li>
<li>外观设置会通过 QSettings 持久化，下次启动自动恢复。</li>
</ul>
<h2>显示选项</h2>
<p>“显示 -> 画布显示”用于控制背景网格、加工方向箭头、加工顺序编号、加工断面标记和排除图元弱化。显示开关不改变实际加工状态。</p>
)HTML");
    }

    QString troubleshootingHtml()
    {
        return QString::fromUtf8(R"HTML(
<h2>排序结果已过期</h2>
<p>修改几何、加工状态、截面、断面、内部线或相关配置后，旧计划会失效。重新执行“排序(保留方向)”或“智能排序”。</p>
<h2>截面或加工断面无法识别</h2>
<ul>
<li>确认候选边界连续且严格闭合；可测量的缝隙不能作为闭环。</li>
<li>优先选择真实外围边界中的一条边，让系统扩展完整闭环。</li>
<li>辅助线、引线和内部支线不应成为截面边界成员。</li>
<li>加工断面操作前必须先获得有效方管截面。</li>
</ul>
<h2>内部线条没有被清理</h2>
<ul>
<li>先确认右侧加工设置显示有效方管截面。</li>
<li>只有完整路径严格位于内缩窗口内的图元才会被删除。</li>
<li>接触窗口、安全带附近或穿越窗口边界的图元会保留。</li>
<li>圆角半径过大导致内缩窗口坍缩时不会删除任何图元。</li>
</ul>
<h2>无法建立合法区位或起刀点</h2>
<ul>
<li>懒旋转要求有效截面，并要求加工单元能够确定稳定的方管区位。</li>
<li>检查闭环成员是否真实相连、是否存在重复图元，以及路径是否误入方管内部。</li>
<li>普通四轴无需区位增强时，可关闭懒旋转后重新智能排序。</li>
</ul>
<h2>导出失败</h2>
<ul>
<li>查看底部命令栏中的具体失败阶段和图元编号。</li>
<li>确认当前 G 代码模式、排序结果和配置一致。</li>
<li>四轴应重点检查旋转轴中心、安全距离、连续路径和 A 轴方向。</li>
<li>失败时系统不会保留不完整的 G 代码文件。</li>
</ul>
<h2>看不到编号、箭头或排除状态</h2>
<p>打开“显示”选项卡，启用“显示机加工相关”及对应子项。显示开关不会改变已有加工状态。</p>
)HTML");
    }

    QString aboutHtml
    (
        const QString& aboutText,
        const QString& companyName,
        const QString& website,
        const QString& supportText,
        const QString& licenseText
    )
    {
        QString displayTitle =
            QGuiApplication::applicationDisplayName().trimmed();
        if (displayTitle.isEmpty())
        {
            displayTitle = QCoreApplication::applicationName().trimmed();
        }

        QString body = QStringLiteral("<h2>%1</h2>")
            .arg(displayTitle.toHtmlEscaped());
        const QString version =
            QCoreApplication::applicationVersion().trimmed();
        if (!version.isEmpty())
        {
            body += QStringLiteral("<p>版本：%1</p>")
                .arg(version.toHtmlEscaped());
        }
        const QString summary = aboutText.trimmed().isEmpty()
            ? QStringLiteral("本程序用于 CAD 图形处理、加工路径配置和 G 代码生成。")
            : aboutText.trimmed();
        body += QStringLiteral("<p>%1</p>")
            .arg(summary.toHtmlEscaped());
        if (!companyName.trimmed().isEmpty())
        {
            body += QStringLiteral("<p>软件提供方：%1</p>")
                .arg(companyName.trimmed().toHtmlEscaped());
        }
        if (!website.trimmed().isEmpty())
        {
            body += QStringLiteral("<p>网站：%1</p>")
                .arg(website.trimmed().toHtmlEscaped());
        }
        if (!supportText.trimmed().isEmpty())
        {
            body += QStringLiteral("<p>支持信息：%1</p>")
                .arg(supportText.trimmed().toHtmlEscaped());
        }
        if (!licenseText.trimmed().isEmpty())
        {
            body += QStringLiteral("<p>授权状态：%1</p>")
                .arg(licenseText.trimmed().toHtmlEscaped());
        }
        else
        {
            body += QStringLiteral(
                "<p>软件使用与授权范围以交付说明和有效授权为准。</p>");
        }
        return body;
    }
}

CadHelpDialog::CadHelpDialog(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(QStringLiteral("帮助"));
    setMinimumSize(780, 620);
    resize(980, 700);

    QVBoxLayout* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(10, 10, 10, 10);
    rootLayout->setSpacing(8);

    m_tabWidget = new QTabWidget(this);
    rootLayout->addWidget(m_tabWidget, 1);

    addSection(CadHelpSection::QuickStart, QStringLiteral("快速上手"), quickStartHtml());
    addSection(CadHelpSection::Shortcuts, QStringLiteral("快捷命令"), shortcutsHtml());
    addSection(CadHelpSection::Drawing, QStringLiteral("绘图教程"), drawingHtml());
    addSection(CadHelpSection::Editing, QStringLiteral("修改教程"), editingHtml());
    addSection(CadHelpSection::Machining, QStringLiteral("机加工 / G代码"), machiningHtml());
    addSection(CadHelpSection::RotaryMachining, QStringLiteral("方管四轴"), rotaryMachiningHtml());
    addSection(CadHelpSection::BitmapImport, QStringLiteral("位图导入"), bitmapHtml());
    addSection(CadHelpSection::Appearance, QStringLiteral("外观与显示"), appearanceHtml());
    addSection(CadHelpSection::Troubleshooting, QStringLiteral("问题排查"), troubleshootingHtml());
    m_aboutBrowser =
        addSection(CadHelpSection::About, QStringLiteral("关于"),
            aboutHtml(m_aboutText, m_companyName, m_website,
                m_supportText, m_licenseText));
    m_aboutBrowser->viewport()->installEventFilter(this);

    connect(m_tabWidget, &QTabWidget::currentChanged, this,
        [this](int)
        {
            if (m_tabWidget->currentWidget() != m_aboutBrowser)
            {
                resetAboutClickSequence();
            }
        });

    QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Close, this);
    buttonBox->button(QDialogButtonBox::Close)->setText(QStringLiteral("关闭"));
    rootLayout->addWidget(buttonBox);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(this, &QDialog::finished, this,
        [this](int) { resetAboutClickSequence(); });
    connect(qGuiApp, &QGuiApplication::applicationDisplayNameChanged,
        this, &CadHelpDialog::refreshAboutContent);
}

void CadHelpDialog::setCurrentSection(CadHelpSection section)
{
    if (m_tabWidget == nullptr)
    {
        return;
    }

    for (int index = 0; index < m_tabWidget->count(); ++index)
    {
        if (static_cast<CadHelpSection>(m_tabWidget->widget(index)->property("helpSection").toInt()) == section)
        {
            m_tabWidget->setCurrentIndex(index);
            return;
        }
    }
}

void CadHelpDialog::setAboutInformation
(
    const QString& aboutText,
    const QString& companyName,
    const QString& website,
    const QString& supportText,
    const QString& licenseText
)
{
    m_aboutText = aboutText;
    m_companyName = companyName;
    m_website = website;
    m_supportText = supportText;
    m_licenseText = licenseText;
    refreshAboutContent();
}

void CadHelpDialog::refreshAboutContent()
{
    if (m_aboutBrowser != nullptr)
    {
        m_aboutBrowser->setHtml
        (
            wrapHelpHtml
            (
                QStringLiteral("关于"),
                aboutHtml(m_aboutText, m_companyName, m_website,
                    m_supportText, m_licenseText)
            )
        );
    }
}

bool CadHelpDialog::eventFilter(QObject* watched, QEvent* event)
{
    if (m_aboutBrowser != nullptr
        && watched == m_aboutBrowser->viewport()
        && m_tabWidget != nullptr
        && m_tabWidget->currentWidget() == m_aboutBrowser
        && (event->type() == QEvent::MouseButtonPress
            || event->type() == QEvent::MouseButtonDblClick))
    {
        const QMouseEvent* mouseEvent =
            static_cast<const QMouseEvent*>(event);
        if (mouseEvent->button() == Qt::LeftButton)
        {
            if (!m_aboutClickTimer.isValid()
                || m_aboutClickTimer.elapsed() > 5000)
            {
                m_aboutClickTimer.start();
                m_aboutClickCount = 1;
            }
            else
            {
                ++m_aboutClickCount;
            }

            if (m_aboutClickCount >= 10)
            {
                resetAboutClickSequence();
                emit displayTitleConfigurationRequested();
            }
        }
    }
    return QDialog::eventFilter(watched, event);
}

void CadHelpDialog::resetAboutClickSequence()
{
    m_aboutClickCount = 0;
    m_aboutClickTimer.invalidate();
}

QTextBrowser* CadHelpDialog::addSection(CadHelpSection section, const QString& title, const QString& htmlBody)
{
    QTextBrowser* browser = new QTextBrowser(m_tabWidget);
    browser->setOpenExternalLinks(true);
    browser->setProperty("helpSection", static_cast<int>(section));
    browser->setHtml(wrapHelpHtml(title, htmlBody));
    m_tabWidget->addTab(browser, title);
    return browser;
}
