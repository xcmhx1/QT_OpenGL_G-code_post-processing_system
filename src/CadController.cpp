// CadController 实现文件
// 实现 CadController 模块，对应头文件中声明的主要行为和协作流程。
// 输入控制模块，负责解释键盘、鼠标、滚轮事件并驱动绘图/编辑命令。

#include "pch.h"

#include "CadController.h"

// Qt 核心模块
#include <QColorDialog>
#include <QCursor>
#include <QInputDialog>
#include <QMessageBox>
#include <QPushButton>

// CAD 模块内部依赖
#include "CadEditer.h"
#include "CadItem.h"
#include "CadProcessVisualUtils.h"
#include "CadViewer.h"

// 标准库
#include <algorithm>
#include <cmath>

// 匿名命名空间，存放局部辅助函数
namespace
{
    // 从点击切换到框选拖拽的最小屏幕位移阈值（像素）。
    constexpr int kWindowSelectionDragThresholdPixels = 6;

    // 将三维点投影到绘图平面（XY平面）
    // @param point 三维点
    // @return 投影到XY平面的点（Z坐标为0）
    QVector3D flattenToDrawingPlane(const QVector3D& point)
    {
        return QVector3D(point.x(), point.y(), 0.0f);
    }

    // 将绘制类型转换为对应的中文名称
    // @param drawType 绘制类型
    // @return 对应的中文名称字符串
    QString drawTypeName(DrawType drawType)
    {
        switch (drawType)
        {
        case DrawType::Point:
            return QStringLiteral("点");
        case DrawType::Line:
            return QStringLiteral("直线");
        case DrawType::Xline:
            return QStringLiteral("构造线");
        case DrawType::Rectangle:
            return QStringLiteral("矩形");
        case DrawType::Polygon:
            return QStringLiteral("多边形");
        case DrawType::Circle:
            return QStringLiteral("圆");
        case DrawType::Arc:
            return QStringLiteral("圆弧");
        case DrawType::Ellipse:
            return QStringLiteral("椭圆");
        case DrawType::Polyline:
            return QStringLiteral("多段线");
        case DrawType::LWPolyline:
            return QStringLiteral("轻量多段线");
        default:
            return QStringLiteral("空闲");
        }
    }

    QString polygonConstructionModeText(bool circumscribedAboutCircle)
    {
        return circumscribedAboutCircle
            ? QStringLiteral("外切于圆")
            : QStringLiteral("内切于圆");
    }

    // 判断是否属于坐标表达式字符（兼容输入：x,y / @dx,dy / d<a）。
    bool isDynamicExpressionCharacter(QChar character)
    {
        return character.isDigit()
            || character == QLatin1Char('.')
            || character == QLatin1Char('+')
            || character == QLatin1Char('-')
            || character == QLatin1Char(',')
            || character == QLatin1Char('@')
            || character == QLatin1Char('<');
    }

    // 判断是否属于字段化输入的数值字符。
    bool isDynamicFieldCharacter(QChar character)
    {
        return character.isDigit()
            || character == QLatin1Char('.')
            || character == QLatin1Char('+')
            || character == QLatin1Char('-');
    }

    bool tryParseCoordinatePair(const QString& text, double& first, double& second)
    {
        const QStringList parts = text.split(QLatin1Char(','), Qt::KeepEmptyParts);

        if (parts.size() != 2)
        {
            return false;
        }

        bool firstOk = false;
        bool secondOk = false;
        const double firstValue = parts.at(0).toDouble(&firstOk);
        const double secondValue = parts.at(1).toDouble(&secondOk);

        if (!firstOk || !secondOk)
        {
            return false;
        }

        first = firstValue;
        second = secondValue;
        return true;
    }

    struct DynamicCommandDefinition
    {
        QString canonical;
        QString displayName;
        QStringList aliases;
    };

    QString normalizedCommandToken(const QString& token)
    {
        return token.trimmed().toLower();
    }

    bool isDynamicCommandCharacter(QChar character)
    {
        return character.isLetterOrNumber()
            || character == QLatin1Char('_')
            || character == QLatin1Char('-');
    }

    const QVector<DynamicCommandDefinition>& dynamicCommandDefinitions()
    {
        static const QVector<DynamicCommandDefinition> definitions
        {
            { QStringLiteral("line"),       QStringLiteral("LINE  直线"),        { QStringLiteral("l"), QStringLiteral("line"), QStringLiteral("直线") } },
            { QStringLiteral("xline"),      QStringLiteral("XLINE 构造线"),      { QStringLiteral("x"), QStringLiteral("xl"), QStringLiteral("xline"), QStringLiteral("constructionline"), QStringLiteral("构造线"), QStringLiteral("无限线") } },
            { QStringLiteral("rectangle"),  QStringLiteral("RECTANGLE 矩形"),    { QStringLiteral("r"), QStringLiteral("rect"), QStringLiteral("rectangle"), QStringLiteral("矩形") } },
            { QStringLiteral("polygon"),    QStringLiteral("POLYGON 多边形"),    { QStringLiteral("g"), QStringLiteral("pg"), QStringLiteral("polygon"), QStringLiteral("多边形"), QStringLiteral("正多边形") } },
            { QStringLiteral("point"),      QStringLiteral("POINT 点"),          { QStringLiteral("p"), QStringLiteral("point"), QStringLiteral("点") } },
            { QStringLiteral("circle"),     QStringLiteral("CIRCLE 圆"),         { QStringLiteral("c"), QStringLiteral("circle"), QStringLiteral("圆") } },
            { QStringLiteral("arc"),        QStringLiteral("ARC   圆弧"),        { QStringLiteral("a"), QStringLiteral("arc"), QStringLiteral("圆弧") } },
            { QStringLiteral("ellipse"),    QStringLiteral("ELLIPSE 椭圆"),      { QStringLiteral("e"), QStringLiteral("ellipse"), QStringLiteral("椭圆") } },
            { QStringLiteral("polyline"),   QStringLiteral("POLYLINE 多段线"),   { QStringLiteral("o"), QStringLiteral("polyline"), QStringLiteral("pline"), QStringLiteral("多段线") } },
            { QStringLiteral("lwpolyline"), QStringLiteral("LWPOLYLINE 轻量多段线"), { QStringLiteral("w"), QStringLiteral("lwpolyline"), QStringLiteral("轻量多段线") } },
            { QStringLiteral("move"),       QStringLiteral("MOVE  移动"),        { QStringLiteral("m"), QStringLiteral("move"), QStringLiteral("移动") } },
            { QStringLiteral("delete"),     QStringLiteral("DELETE 删除"),       { QStringLiteral("del"), QStringLiteral("delete"), QStringLiteral("erase"), QStringLiteral("删除") } },
            { QStringLiteral("color"),      QStringLiteral("COLOR 改色"),        { QStringLiteral("k"), QStringLiteral("color"), QStringLiteral("改色"), QStringLiteral("颜色") } },
            { QStringLiteral("fit"),        QStringLiteral("FIT   适配视图"),    { QStringLiteral("f"), QStringLiteral("fit"), QStringLiteral("zoomextents"), QStringLiteral("适配") } },
            { QStringLiteral("top"),        QStringLiteral("TOP   顶视图"),      { QStringLiteral("t"), QStringLiteral("top"), QStringLiteral("home"), QStringLiteral("顶视图") } },
            { QStringLiteral("zoomin"),     QStringLiteral("ZOOMIN  放大"),      { QStringLiteral("zoomin"), QStringLiteral("zin"), QStringLiteral("放大") } },
            { QStringLiteral("zoomout"),    QStringLiteral("ZOOMOUT 缩小"),      { QStringLiteral("zoomout"), QStringLiteral("zout"), QStringLiteral("缩小") } },
        };

        return definitions;
    }

    bool commandAliasMatches(const DynamicCommandDefinition& definition, const QString& normalizedInput)
    {
        if (normalizedInput.isEmpty())
        {
            return true;
        }

        if (normalizedCommandToken(definition.canonical).startsWith(normalizedInput))
        {
            return true;
        }

        for (const QString& alias : definition.aliases)
        {
            if (normalizedCommandToken(alias).startsWith(normalizedInput))
            {
                return true;
            }
        }

        return false;
    }
}

// 设置关联的视图对象
// @param viewer CAD 视图对象指针
void CadController::setViewer(CadViewer* viewer)
{
    m_viewer = viewer;
}

// 设置关联的编辑器对象
// @param editer CAD 编辑器对象指针
void CadController::setEditer(CadEditer* editer)
{
    m_editer = editer;
}

// 重置控制器状态
void CadController::reset()
{
    // 如果有编辑器，取消其临时命令
    if (m_editer != nullptr)
    {
        m_editer->cancelTransientCommand();
    }

    // 重置绘图状态机
    m_drawState.reset();
    clearDynamicCommandMode();
    m_idleWindowSelectionTracking = false;
    m_idleWindowSelectionDragging = false;
    m_idleWindowSelectionAnchor = QPoint();
    m_idleWindowSelectionCurrent = QPoint();

    // 如果有视图，刷新命令提示
    if (m_viewer != nullptr)
    {
        m_viewer->hideSelectionWindowPreview();
        m_viewer->refreshCommandPrompt();
    }
}

void CadController::setDefaultDrawingProperties(const QString& layerName, const QColor& color, int colorIndex)
{
    m_drawState.drawingLayerName = layerName.trimmed().isEmpty() ? QStringLiteral("0") : layerName.trimmed();
    m_drawState.drawingColor = color.isValid() ? color : QColor(Qt::white);
    m_drawState.drawingColorIndex = colorIndex;
}

// 开始绘制指定类型的图元
// @param drawType 绘制类型
// @param color 图元颜色，默认为白色
void CadController::beginDrawing(DrawType drawType, const QColor& color)
{
    if (drawType == DrawType::Polygon && !configurePolygonDrawing())
    {
        return;
    }

    // 如果有编辑器，取消其临时命令
    if (m_editer != nullptr)
    {
        m_editer->cancelTransientCommand();
    }

    // 设置绘图状态
    m_drawState.isDrawing = true;
    m_drawState.drawType = drawType;
    m_drawState.drawingColor = color;
    m_drawState.editType = EditType::None;
    m_drawState.commandPoints.clear();
    m_drawState.commandBulges.clear();
    m_drawState.polylineArcMode = false;
    m_drawState.lwPolylineArcMode = false;
    clearDynamicCommandMode();
    resetPointDynamicInputSession();

    // 重置子模式
    resetSubModes();
    // 准备图元子模式
    preparePrimitiveSubMode();
    resetPointDynamicInputSession(currentPointInputStageKey());

    // 如果有视图，发送消息并刷新提示
    if (m_viewer != nullptr)
    {
        if (drawType == DrawType::Polygon)
        {
            m_viewer->appendCommandMessage
            (
                QStringLiteral("已进入多边形命令（%1 边，%2）")
                .arg(m_drawState.polygonSideCount)
                .arg(polygonConstructionModeText(m_drawState.polygonCircumscribedAboutCircle))
            );
        }
        else
        {
            m_viewer->appendCommandMessage(QStringLiteral("已进入%1命令").arg(drawTypeName(drawType)));
        }

        m_viewer->refreshCommandPrompt();
    }
}

bool CadController::configurePolygonDrawing()
{
    QWidget* parentWidget = m_viewer != nullptr ? static_cast<QWidget*>(m_viewer) : nullptr;
    bool sideCountAccepted = false;
    const int sideCount = QInputDialog::getInt
    (
        parentWidget,
        QStringLiteral("多边形"),
        QStringLiteral("请输入边数（3-1024）:"),
        std::clamp(m_drawState.polygonSideCount, 3, 1024),
        3,
        1024,
        1,
        &sideCountAccepted
    );

    if (!sideCountAccepted)
    {
        return false;
    }

    QMessageBox modeDialog(parentWidget);
    modeDialog.setWindowTitle(QStringLiteral("多边形"));
    modeDialog.setText(QStringLiteral("选择与参考圆的关系"));
    QAbstractButton* inscribedButton = modeDialog.addButton(QStringLiteral("内切于圆"), QMessageBox::AcceptRole);
    QAbstractButton* circumscribedButton = modeDialog.addButton(QStringLiteral("外切于圆"), QMessageBox::AcceptRole);
    modeDialog.addButton(QMessageBox::Cancel);
    modeDialog.exec();

    if (modeDialog.clickedButton() != inscribedButton && modeDialog.clickedButton() != circumscribedButton)
    {
        return false;
    }

    m_drawState.polygonSideCount = sideCount;
    m_drawState.polygonCircumscribedAboutCircle = modeDialog.clickedButton() == circumscribedButton;
    return true;
}

bool CadController::beginMoveSelected()
{
    if (m_editer == nullptr || m_viewer == nullptr)
    {
        return false;
    }

    const QVector<CadItem*> selectedItems = m_viewer->selectedEntities();
    const bool handled = m_editer->beginMove(m_drawState, selectedItems);

    if (handled)
    {
        clearDynamicCommandMode();
        resetPointDynamicInputSession(currentPointInputStageKey());
        m_viewer->appendCommandMessage
        (
            selectedItems.size() > 1
                ? QStringLiteral("已进入移动命令（%1 个图元）").arg(selectedItems.size())
                : QStringLiteral("已进入移动命令")
        );
        m_viewer->refreshCommandPrompt();
    }

    return handled;
}

// 取消当前绘制操作
void CadController::cancelDrawing()
{
    // 记录是否有活动命令
    const bool hadActiveCommand = m_drawState.hasActiveCommand();

    // 如果有编辑器，取消其临时命令
    if (m_editer != nullptr)
    {
        m_editer->cancelTransientCommand();
    }

    // 重置绘图状态
    m_drawState.isDrawing = false;
    m_drawState.drawType = DrawType::None;
    m_drawState.editType = EditType::None;
    m_drawState.commandPoints.clear();
    m_drawState.commandBulges.clear();
    m_drawState.polylineArcMode = false;
    m_drawState.lwPolylineArcMode = false;
    clearDynamicCommandMode();
    resetPointDynamicInputSession();

    // 重置子模式
    resetSubModes();
    m_idleWindowSelectionTracking = false;
    m_idleWindowSelectionDragging = false;
    m_idleWindowSelectionAnchor = QPoint();
    m_idleWindowSelectionCurrent = QPoint();

    // 如果有视图，发送取消消息并刷新提示
    if (m_viewer != nullptr)
    {
        m_viewer->hideSelectionWindowPreview();

        if (hadActiveCommand)
        {
            m_viewer->appendCommandMessage(QStringLiteral("命令已取消"));
        }

        m_viewer->refreshCommandPrompt();
    }
}

// 处理鼠标按下事件
// @param event 鼠标事件
// @return 如果事件被处理返回 true，否则返回 false
bool CadController::handleMousePress(QMouseEvent* event)
{
    if (m_viewer == nullptr)
    {
        return false;
    }

    // 获取当前世界坐标
    const QVector3D worldPos = currentWorldPos(event->pos());

    // 更新绘图状态机中的鼠标信息
    m_drawState.pressScreenPos = event->pos();
    m_drawState.lastScreenPos = event->pos();
    m_drawState.currentScreenPos = event->pos();
    m_drawState.activeButton = event->button();
    m_drawState.pressedButtons = event->buttons();
    m_drawState.keyboardModifiers = event->modifiers();
    m_drawState.lastPos = m_drawState.currentPos;
    m_drawState.currentPos = applyOrthoConstraint(worldPos);

    syncPointDynamicInputSession();
    syncCurrentPosWithCursor();

    if (isPointDynamicFieldModeActive())
    {
        m_drawState.currentPos = applyPointDynamicFieldOverride(m_drawState.currentPos, true);
    }

    // 处理中键按下：视图操作
    if (event->button() == Qt::MiddleButton)
    {
        if ((event->modifiers() & Qt::ShiftModifier) != 0)
        {
            // 中键+Shift：开始轨道旋转
            m_viewer->beginOrbitInteraction();
        }
        else
        {
            // 中键：开始平移
            m_viewer->beginPanInteraction();
        }

        return true;
    }

    // 处理左键按下：空闲态进入候选框选，命令态按既有流程执行。
    if (event->button() == Qt::LeftButton)
    {
        if (!m_drawState.hasActiveCommand())
        {
            beginIdleWindowSelection(event->pos());
            return true;
        }

        commitCommandPoint(worldPos);
        return true;
    }

    // 处理右键按下：命令态统一执行确认动作（与 Enter/Space 行为保持一致）。
    if (event->button() == Qt::RightButton && m_drawState.hasActiveCommand())
    {
        confirmActiveCommand();
        return true;
    }

    return false;
}

// 处理鼠标移动事件
// @param event 鼠标事件
// @return 如果事件被处理返回 true，否则返回 false
bool CadController::handleMouseMove(QMouseEvent* event)
{
    if (m_viewer == nullptr)
    {
        return false;
    }

    // 获取当前世界坐标
    const QVector3D worldPos = currentWorldPos(event->pos());

    // 更新绘图状态机中的鼠标信息
    m_drawState.lastScreenPos = m_drawState.currentScreenPos;
    m_drawState.currentScreenPos = event->pos();
    m_drawState.pressedButtons = event->buttons();
    m_drawState.keyboardModifiers = event->modifiers();
    m_drawState.lastPos = m_drawState.currentPos;
    m_drawState.currentPos = applyOrthoConstraint(worldPos);

    syncPointDynamicInputSession();
    syncCurrentPosWithCursor();

    if (isPointDynamicFieldModeActive())
    {
        m_drawState.currentPos = applyPointDynamicFieldOverride(m_drawState.currentPos, true);
    }

    // 如果是轨道交互且需要忽略下一次增量，则消费此标志
    if (m_viewer->interactionMode() == ViewInteractionMode::Orbiting && m_viewer->shouldIgnoreNextOrbitDelta())
    {
        m_viewer->consumeIgnoreNextOrbitDelta();
        return true;
    }

    // 计算鼠标移动增量
    const QPoint delta = m_drawState.currentScreenPos - m_drawState.lastScreenPos;

    // 根据当前交互模式处理移动
    switch (m_viewer->interactionMode())
    {
    case ViewInteractionMode::Panning:
        m_viewer->updatePanInteraction(delta);
        return true;
    case ViewInteractionMode::Orbiting:
        m_viewer->updateOrbitInteraction(delta);
        return true;
    default:
        break;
    }

    if (m_idleWindowSelectionTracking
        && (event->buttons() & Qt::LeftButton) != 0
        && !m_drawState.hasActiveCommand())
    {
        updateIdleWindowSelection(event->pos());
        return true;
    }

    return false;
}

// 处理鼠标释放事件
// @param event 鼠标事件
// @return 如果事件被处理返回 true，否则返回 false
bool CadController::handleMouseRelease(QMouseEvent* event)
{
    if (m_viewer == nullptr)
    {
        return false;
    }

    // 更新绘图状态机中的鼠标信息
    m_drawState.lastScreenPos = m_drawState.currentScreenPos;
    m_drawState.currentScreenPos = event->pos();
    m_drawState.activeButton = event->button();
    m_drawState.pressedButtons = event->buttons();
    m_drawState.keyboardModifiers = event->modifiers();

    // 处理中键释放：结束视图交互
    if (event->button() == Qt::MiddleButton)
    {
        m_viewer->endViewInteraction();
        return true;
    }

    // 空闲态左键释放：点击选中或窗口框选提交。
    if (event->button() == Qt::LeftButton && !m_drawState.hasActiveCommand())
    {
        return finishIdleWindowSelection(event->pos());
    }

    return false;
}

// 处理滚轮事件
// @param event 滚轮事件
// @return 如果事件被处理返回 true，否则返回 false
bool CadController::handleWheel(QWheelEvent* event)
{
    if (m_viewer == nullptr)
    {
        return false;
    }

    // 获取当前世界坐标
    const QVector3D worldPos = currentWorldPos(event->position().toPoint());
    m_drawState.currentPos = worldPos;
    m_drawState.keyboardModifiers = event->modifiers();

    // 计算缩放因子
    const float factor = event->angleDelta().y() > 0 ? 1.1f : (1.0f / 1.1f);

    // 在鼠标位置缩放
    m_viewer->zoomAtScreenPosition(event->position().toPoint(), factor);
    event->accept();
    return true;
}

// 处理键盘按下事件
// @param event 键盘事件
// @return 如果事件被处理返回 true，否则返回 false
bool CadController::handleKeyPress(QKeyEvent* event)
{
    // 更新键盘修饰符
    m_drawState.keyboardModifiers = event->modifiers();

    // 处理Ctrl组合键：撤销/重做
    if ((event->modifiers() & Qt::ControlModifier) != 0 && m_editer != nullptr)
    {
        // Ctrl+Shift+Z 或 Ctrl+Y：重做
        if (event->key() == Qt::Key_Z && (event->modifiers() & Qt::ShiftModifier) != 0)
        {
            const bool handled = m_editer->redo();

            if (handled && m_viewer != nullptr)
            {
                m_viewer->appendCommandMessage(QStringLiteral("重做完成"));
                m_viewer->refreshCommandPrompt();
            }

            return handled;
        }

        // Ctrl+Z：撤销
        if (event->key() == Qt::Key_Z)
        {
            const bool handled = m_editer->undo();

            if (handled && m_viewer != nullptr)
            {
                m_viewer->appendCommandMessage(QStringLiteral("撤销完成"));
                m_viewer->refreshCommandPrompt();
            }

            return handled;
        }

        // Ctrl+Y：重做
        if (event->key() == Qt::Key_Y)
        {
            const bool handled = m_editer->redo();

            if (handled && m_viewer != nullptr)
            {
                m_viewer->appendCommandMessage(QStringLiteral("重做完成"));
                m_viewer->refreshCommandPrompt();
            }

            return handled;
        }
    }

    // F8：切换正交约束
    if (event->key() == Qt::Key_F8)
    {
        m_drawState.orthoEnabled = !m_drawState.orthoEnabled;

        if (m_viewer != nullptr)
        {
            m_viewer->appendCommandMessage
            (
                m_drawState.orthoEnabled
                    ? QStringLiteral("正交约束: 开")
                    : QStringLiteral("正交约束: 关")
            );
            m_viewer->refreshCommandPrompt();
        }

        return true;
    }

    // ESC键：优先取消当前命令；无活动命令时清空当前选中
    if (event->key() == Qt::Key_Escape)
    {
        if (m_drawState.hasActiveCommand())
        {
            if (hasPendingDynamicKeyboardInput()
                || m_drawState.dynamicInputXLocked
                || m_drawState.dynamicInputYLocked)
            {
                resetPointDynamicInputSession(currentPointInputStageKey());

                if (m_viewer != nullptr)
                {
                    m_viewer->refreshCommandPrompt();
                    m_viewer->requestViewUpdate();
                }

                return true;
            }

            cancelDrawing();
            return true;
        }

        if (isDynamicCommandModeActive())
        {
            clearDynamicCommandMode();

            if (m_viewer != nullptr)
            {
                m_viewer->refreshCommandPrompt();
                m_viewer->requestViewUpdate();
            }

            return true;
        }

        if (m_viewer != nullptr && m_viewer->selectedEntity() != nullptr)
        {
            m_viewer->clearSelection();
            m_viewer->appendCommandMessage(QStringLiteral("已取消选中"));
            m_viewer->refreshCommandPrompt();
            return true;
        }

        return true;
    }

    // 有活动命令时优先处理动态输入和确认逻辑
    if (m_drawState.hasActiveCommand())
    {
        syncPointDynamicInputSession();
        syncCurrentPosWithCursor();

        if (event->key() == Qt::Key_Tab
            && (event->modifiers() == Qt::NoModifier || event->modifiers() == Qt::ShiftModifier)
            && isPointDynamicFieldModeActive())
        {
            return handleDynamicFieldTab((event->modifiers() & Qt::ShiftModifier) != 0 ? -1 : 1);
        }

        if (event->key() == Qt::Key_Backspace)
        {
            if (m_drawState.dynamicInputExpressionMode)
            {
                if (!m_drawState.dynamicInputBuffer.isEmpty())
                {
                    m_drawState.dynamicInputBuffer.chop(1);

                    if (m_viewer != nullptr)
                    {
                        m_viewer->refreshCommandPrompt();
                        m_viewer->requestViewUpdate();
                    }
                }
            }
            else if (isPointDynamicFieldModeActive())
            {
                if (!m_drawState.dynamicInputFieldBuffer.isEmpty())
                {
                    m_drawState.dynamicInputFieldBuffer.chop(1);
                    m_drawState.currentPos = applyPointDynamicFieldOverride(m_drawState.currentPos, true);

                    if (m_viewer != nullptr)
                    {
                        m_viewer->refreshCommandPrompt();
                        m_viewer->requestViewUpdate();
                    }
                }
            }
            else if (!m_drawState.dynamicInputBuffer.isEmpty())
            {
                m_drawState.dynamicInputBuffer.chop(1);

                if (m_viewer != nullptr)
                {
                    m_viewer->refreshCommandPrompt();
                }
            }

            return true;
        }

        const bool isPlainSpaceConfirm = event->key() == Qt::Key_Space
            && (event->modifiers() & (Qt::ControlModifier | Qt::AltModifier | Qt::MetaModifier)) == 0;

        if (event->key() == Qt::Key_Return
            || event->key() == Qt::Key_Enter
            || isPlainSpaceConfirm)
        {
            return confirmActiveCommand();
        }

        // 在绘图状态下处理多段线相关按键
        if (m_drawState.isDrawing && m_editer != nullptr)
        {
            // 多段线：A键切换到圆弧模式
            if ((m_drawState.drawType == DrawType::Polyline || m_drawState.drawType == DrawType::LWPolyline)
                && event->key() == Qt::Key_A)
            {
                return setPolylineInputMode(true);
            }

            // 多段线：L键切换到直线模式
            if ((m_drawState.drawType == DrawType::Polyline || m_drawState.drawType == DrawType::LWPolyline)
                && event->key() == Qt::Key_L)
            {
                return setPolylineInputMode(false);
            }

            // 多段线：C键闭合多段线
            if ((m_drawState.drawType == DrawType::Polyline || m_drawState.drawType == DrawType::LWPolyline)
                && event->key() == Qt::Key_C)
            {
                const bool handled = m_editer->finishActivePolyline(m_drawState, true);

                if (handled && m_viewer != nullptr)
                {
                    resetPointDynamicInputSession(currentPointInputStageKey());
                    m_viewer->appendCommandMessage(QStringLiteral("已创建闭合多段线图元"));
                    m_viewer->refreshCommandPrompt();
                }

                return handled;
            }
        }

        // 动态输入字符（字段模式优先，表达式模式兼容）
        if ((event->modifiers() & (Qt::ControlModifier | Qt::AltModifier | Qt::MetaModifier)) == 0)
        {
            const QString inputText = event->text();

            if (!inputText.isEmpty())
            {
                bool consumed = false;

                for (const QChar character : inputText)
                {
                    if (character.isSpace())
                    {
                        continue;
                    }

                    // 点参数输入：优先字段模式；在用户显式输入表达式字符时切到兼容表达式模式。
                    if (isAwaitingPointInput())
                    {
                        if (m_drawState.dynamicInputExpressionMode)
                        {
                            if (!isDynamicExpressionCharacter(character))
                            {
                                continue;
                            }

                            m_drawState.dynamicInputBuffer.append(character);
                            consumed = true;
                            continue;
                        }

                        if (character == QLatin1Char(',')
                            || character == QLatin1Char('@')
                            || character == QLatin1Char('<'))
                        {
                            m_drawState.dynamicInputExpressionMode = true;
                            m_drawState.dynamicInputBuffer.clear();
                            m_drawState.dynamicInputBuffer.append(character);
                            m_drawState.dynamicInputFieldBuffer.clear();
                            consumed = true;
                            continue;
                        }

                        if (isDynamicFieldCharacter(character) && isPointDynamicFieldModeActive())
                        {
                            m_drawState.dynamicInputFieldBuffer.append(character);
                            m_drawState.currentPos = applyPointDynamicFieldOverride(m_drawState.currentPos, true);
                            consumed = true;
                            continue;
                        }

                        if (isDynamicExpressionCharacter(character))
                        {
                            m_drawState.dynamicInputBuffer.append(character);
                            consumed = true;
                        }

                        continue;
                    }

                    if (isDynamicExpressionCharacter(character))
                    {
                        m_drawState.dynamicInputBuffer.append(character);
                        consumed = true;
                    }
                }

                if (consumed)
                {
                    if (m_viewer != nullptr)
                    {
                        m_viewer->refreshCommandPrompt();
                        m_viewer->requestViewUpdate();
                    }

                    return true;
                }
            }
        }

        // 在有活动命令时吞掉会触发全局快捷功能的按键
        switch (event->key())
        {
        case Qt::Key_F:     // 适配视图
        case Qt::Key_T:     // 顶视图
        case Qt::Key_Home:  // 顶视图
        case Qt::Key_Plus:  // 放大
        case Qt::Key_Equal: // 放大
        case Qt::Key_Minus: // 缩小
        case Qt::Key_Underscore: // 缩小
        case Qt::Key_P:     // 点
        case Qt::Key_L:     // 直线
        case Qt::Key_X:     // 构造线
        case Qt::Key_R:     // 矩形
        case Qt::Key_G:     // 多边形
        case Qt::Key_C:     // 圆
        case Qt::Key_A:     // 圆弧
        case Qt::Key_E:     // 椭圆
        case Qt::Key_Delete:// 删除
        case Qt::Key_K:     // 颜色
        case Qt::Key_M:     // 移动
        case Qt::Key_O:     // 多段线
        case Qt::Key_W:     // 轻量多段线
            return true;
        default:
            break;
        }
    }

    // 空闲态动态命令输入：键入字符后弹出命令匹配，Enter/Space 执行。
    if (!m_drawState.hasActiveCommand())
    {
        const bool plainInput = (event->modifiers() & (Qt::ControlModifier | Qt::AltModifier | Qt::MetaModifier)) == 0;

        if (isDynamicCommandModeActive())
        {
            if (event->key() == Qt::Key_Backspace)
            {
                if (!m_drawState.dynamicCommandBuffer.isEmpty())
                {
                    m_drawState.dynamicCommandBuffer.chop(1);

                    if (m_drawState.dynamicCommandBuffer.trimmed().isEmpty())
                    {
                        clearDynamicCommandMode();
                    }
                    else
                    {
                        normalizeDynamicCommandSelectionIndex();
                    }
                }

                if (m_viewer != nullptr)
                {
                    m_viewer->refreshCommandPrompt();
                    m_viewer->requestViewUpdate();
                }

                return true;
            }

            if (event->key() == Qt::Key_Delete)
            {
                clearDynamicCommandMode();

                if (m_viewer != nullptr)
                {
                    m_viewer->refreshCommandPrompt();
                    m_viewer->requestViewUpdate();
                }

                return true;
            }

            if (event->key() == Qt::Key_Tab
                && (event->modifiers() == Qt::NoModifier || event->modifiers() == Qt::ShiftModifier))
            {
                return cycleDynamicCommandSelection((event->modifiers() & Qt::ShiftModifier) != 0 ? -1 : 1);
            }

            if (event->key() == Qt::Key_Return
                || event->key() == Qt::Key_Enter
                || event->key() == Qt::Key_Space)
            {
                return executeSelectedDynamicCommand();
            }

            if (plainInput)
            {
                const QString text = event->text();
                bool consumed = false;

                for (const QChar character : text)
                {
                    if (character.isSpace())
                    {
                        continue;
                    }

                    if (!isDynamicCommandCharacter(character))
                    {
                        continue;
                    }

                    m_drawState.dynamicCommandBuffer.append(character);
                    consumed = true;
                }

                if (consumed)
                {
                    normalizeDynamicCommandSelectionIndex();

                    if (m_viewer != nullptr)
                    {
                        m_viewer->refreshCommandPrompt();
                        m_viewer->requestViewUpdate();
                    }

                    return true;
                }
            }
        }
        else if (plainInput)
        {
            const QString text = event->text();
            QString commandText;
            commandText.reserve(text.size());

            for (const QChar character : text)
            {
                if (character.isSpace())
                {
                    continue;
                }

                if (!isDynamicCommandCharacter(character))
                {
                    continue;
                }

                commandText.append(character);
            }

            if (!commandText.isEmpty())
            {
                m_drawState.dynamicCommandBuffer = commandText;
                m_drawState.dynamicCommandActiveIndex = 0;
                normalizeDynamicCommandSelectionIndex();

                if (m_viewer != nullptr)
                {
                    m_viewer->refreshCommandPrompt();
                    m_viewer->requestViewUpdate();
                }

                return true;
            }
        }
    }

    // Delete键：删除选中实体
    if (event->key() == Qt::Key_Delete && m_editer != nullptr && m_viewer != nullptr)
    {
        return deleteSelectedEntity();
    }

    // M键：开始移动命令
    if (event->key() == Qt::Key_M && m_editer != nullptr && m_viewer != nullptr)
    {
        return beginMoveSelected();
    }

    // K键：修改选中实体颜色
    if (event->key() == Qt::Key_K && m_editer != nullptr && m_viewer != nullptr)
    {
        return changeSelectedEntityColor();
    }

    // 处理其他功能键
    switch (event->key())
    {
    case Qt::Key_F:  // 适配视图
        if (m_viewer != nullptr)
        {
            m_viewer->fitSceneView();
            return true;
        }
        break;
    case Qt::Key_T:  // 顶视图
    case Qt::Key_Home:  // 顶视图
        if (m_viewer != nullptr)
        {
            m_viewer->resetToTopView();
            return true;
        }
        break;
    case Qt::Key_Plus:  // 放大
    case Qt::Key_Equal:  // 放大
        if (m_viewer != nullptr)
        {
            m_viewer->zoomIn();
            return true;
        }
        break;
    case Qt::Key_Minus:  // 缩小
    case Qt::Key_Underscore:  // 缩小
        if (m_viewer != nullptr)
        {
            m_viewer->zoomOut();
            return true;
        }
        break;
    case Qt::Key_P:  // 开始绘制点
        beginDrawing(DrawType::Point, m_drawState.drawingColor);
        return true;
    case Qt::Key_L:  // 开始绘制直线
        beginDrawing(DrawType::Line, m_drawState.drawingColor);
        return true;
    case Qt::Key_X:  // 开始绘制构造线
        beginDrawing(DrawType::Xline, m_drawState.drawingColor);
        return true;
    case Qt::Key_R:  // 开始绘制矩形
        beginDrawing(DrawType::Rectangle, m_drawState.drawingColor);
        return true;
    case Qt::Key_G:  // 开始绘制多边形
        beginDrawing(DrawType::Polygon, m_drawState.drawingColor);
        return true;
    case Qt::Key_C:  // 开始绘制圆
        beginDrawing(DrawType::Circle, m_drawState.drawingColor);
        return true;
    case Qt::Key_A:  // 开始绘制圆弧
        beginDrawing(DrawType::Arc, m_drawState.drawingColor);
        return true;
    case Qt::Key_E:  // 开始绘制椭圆
        beginDrawing(DrawType::Ellipse, m_drawState.drawingColor);
        return true;
    case Qt::Key_O:  // 开始绘制多段线
        beginDrawing(DrawType::Polyline, m_drawState.drawingColor);
        return true;
    case Qt::Key_W:  // 开始绘制轻量多段线
        beginDrawing(DrawType::LWPolyline, m_drawState.drawingColor);
        return true;
    default:
        break;
    }

    return false;
}

// 获取绘图状态机（可修改）
// @return 绘图状态机引用
DrawStateMachine& CadController::drawState()
{
    return m_drawState;
}

// 获取绘图状态机（只读）
// @return 绘图状态机常量引用
const DrawStateMachine& CadController::drawState() const
{
    return m_drawState;
}



bool CadController::deleteSelectedEntity()
{
    if (m_editer == nullptr || m_viewer == nullptr)
    {
        return false;
    }

    const QVector<CadItem*> selectedItems = m_viewer->selectedEntities();

    if (selectedItems.isEmpty())
    {
        return false;
    }

    int deletedCount = 0;

    for (CadItem* item : selectedItems)
    {
        if (item != nullptr && m_editer->deleteEntity(item))
        {
            ++deletedCount;
        }
    }

    if (deletedCount > 0)
    {
        m_viewer->appendCommandMessage
        (
            deletedCount > 1
                ? QStringLiteral("已删除 %1 个图元").arg(deletedCount)
                : QStringLiteral("已删除选中图元")
        );
        m_viewer->refreshCommandPrompt();
        return true;
    }

    return false;
}

bool CadController::changeSelectedEntityColor()
{
    if (m_editer == nullptr || m_viewer == nullptr)
    {
        return false;
    }

    const QVector<CadItem*> selectedItems = m_viewer->selectedEntities();

    if (selectedItems.isEmpty())
    {
        return true;
    }

    CadItem* selectedItem = m_viewer->selectedEntity();

    if (selectedItem == nullptr)
    {
        selectedItem = selectedItems.front();
    }

    const QColor color = QColorDialog::getColor
    (
        selectedItem->m_color,
        m_viewer,
        QStringLiteral("选择图元颜色")
    );

    if (!color.isValid())
    {
        return true;
    }

    int changedCount = 0;

    for (CadItem* item : selectedItems)
    {
        if (item != nullptr && m_editer->changeEntityColor(item, color))
        {
            ++changedCount;
        }
    }

    if (changedCount > 0)
    {
        m_viewer->appendCommandMessage
        (
            changedCount > 1
                ? QStringLiteral("已修改 %1 个图元颜色").arg(changedCount)
                : QStringLiteral("已修改图元颜色")
        );
        m_viewer->refreshCommandPrompt();
        return true;
    }

    return false;
}



void CadController::beginIdleWindowSelection(const QPoint& screenPos)
{
    m_idleWindowSelectionTracking = true;
    m_idleWindowSelectionDragging = false;
    m_idleWindowSelectionAnchor = screenPos;
    m_idleWindowSelectionCurrent = screenPos;

    if (m_viewer != nullptr)
    {
        m_viewer->hideSelectionWindowPreview();
    }
}

void CadController::updateIdleWindowSelection(const QPoint& screenPos)
{
    if (!m_idleWindowSelectionTracking || m_viewer == nullptr)
    {
        return;
    }

    m_idleWindowSelectionCurrent = screenPos;

    if (!m_idleWindowSelectionDragging)
    {
        const QPoint delta = m_idleWindowSelectionCurrent - m_idleWindowSelectionAnchor;

        if (delta.manhattanLength() < kWindowSelectionDragThresholdPixels)
        {
            return;
        }

        m_idleWindowSelectionDragging = true;
    }

    m_viewer->showSelectionWindowPreview(m_idleWindowSelectionAnchor, m_idleWindowSelectionCurrent);
}

bool CadController::finishIdleWindowSelection(const QPoint& screenPos)
{
    if (!m_idleWindowSelectionTracking || m_viewer == nullptr)
    {
        return false;
    }

    m_idleWindowSelectionCurrent = screenPos;
    const bool draggedSelection = m_idleWindowSelectionDragging;
    const QPoint selectionAnchor = m_idleWindowSelectionAnchor;

    m_idleWindowSelectionTracking = false;
    m_idleWindowSelectionDragging = false;
    m_idleWindowSelectionAnchor = QPoint();
    m_idleWindowSelectionCurrent = QPoint();
    m_viewer->hideSelectionWindowPreview();
    const bool shiftSelectionToggle = (m_drawState.keyboardModifiers & Qt::ShiftModifier) != 0;

    if (draggedSelection)
    {
        const bool crossingSelection = screenPos.x() < selectionAnchor.x();
        m_viewer->selectEntitiesInWindow
        (
            selectionAnchor,
            screenPos,
            crossingSelection,
            shiftSelectionToggle ? CadViewer::SelectionUpdateMode::Toggle : CadViewer::SelectionUpdateMode::Replace
        );
        m_viewer->appendCommandMessage
        (
            shiftSelectionToggle
                ? (crossingSelection
                    ? QStringLiteral("框选增量切换完成（碰选）")
                    : QStringLiteral("框选增量切换完成（包含选）"))
                : (crossingSelection
                    ? QStringLiteral("框选完成（碰选）")
                    : QStringLiteral("框选完成（包含选）"))
        );
        m_viewer->refreshCommandPrompt();
        return true;
    }

    // 空闲状态下优先尝试命中当前主选中图元的可编辑控制点
    if (m_editer != nullptr && !shiftSelectionToggle)
    {
        CadSelectionHandleInfo handleInfo;

        if (m_viewer->pickSelectedHandle(screenPos, &handleInfo))
        {
            if (m_editer->beginGripEdit(m_drawState, m_viewer->selectedEntity(), handleInfo))
            {
                resetPointDynamicInputSession(currentPointInputStageKey());
                m_viewer->appendCommandMessage(QStringLiteral("已进入控制点编辑"));
                m_viewer->refreshCommandPrompt();
                return true;
            }
        }
    }

    m_viewer->selectEntityAt
    (
        screenPos,
        shiftSelectionToggle ? CadViewer::SelectionUpdateMode::Toggle : CadViewer::SelectionUpdateMode::Replace
    );
    m_viewer->refreshCommandPrompt();
    return true;
}

// 重置所有子模式
void CadController::resetSubModes()
{
    m_drawState.pointSubMode = PointDrawSubMode::Idle;
    m_drawState.lineSubMode = LineDrawSubMode::Idle;
    m_drawState.circleSubMode = CircleDrawSubMode::Idle;
    m_drawState.rectangleSubMode = RectangleDrawSubMode::Idle;
    m_drawState.polygonSubMode = PolygonDrawSubMode::Idle;
    m_drawState.arcSubMode = ArcDrawSubMode::Idle;
    m_drawState.ellipseSubMode = EllipseDrawSubMode::Idle;
    m_drawState.polylineSubMode = PolylineDrawSubMode::Idle;
    m_drawState.lwPolylineSubMode = LWPolylineDrawSubMode::Idle;
    m_drawState.moveSubMode = MoveEditSubMode::Idle;
    m_drawState.gripSubMode = GripEditSubMode::Idle;
    m_drawState.gripPointIndex = -1;
}

// 准备图元子模式（进入绘图状态）
void CadController::preparePrimitiveSubMode()
{
    // 根据绘制类型设置相应的初始子模式
    switch (m_drawState.drawType)
    {
    case DrawType::Point:
        m_drawState.pointSubMode = PointDrawSubMode::AwaitPosition;
        break;
    case DrawType::Line:
    case DrawType::Xline:
        m_drawState.lineSubMode = LineDrawSubMode::AwaitStartPoint;
        break;
    case DrawType::Rectangle:
        m_drawState.rectangleSubMode = RectangleDrawSubMode::AwaitFirstCorner;
        break;
    case DrawType::Polygon:
        m_drawState.polygonSubMode = PolygonDrawSubMode::AwaitCenter;
        break;
    case DrawType::Circle:
        m_drawState.circleSubMode = CircleDrawSubMode::AwaitCenter;
        break;
    case DrawType::Arc:
        m_drawState.arcSubMode = ArcDrawSubMode::AwaitCenter;
        break;
    case DrawType::Ellipse:
        m_drawState.ellipseSubMode = EllipseDrawSubMode::AwaitCenter;
        break;
    case DrawType::Polyline:
        m_drawState.polylineSubMode = PolylineDrawSubMode::AwaitFirstPoint;
        break;
    case DrawType::LWPolyline:
        m_drawState.lwPolylineSubMode = LWPolylineDrawSubMode::AwaitFirstPoint;
        break;
    default:
        break;
    }
}

// 处理命令状态下的鼠标左键按下
// @param worldPos 世界坐标位置
void CadController::handleLeftPressInCommand(const QVector3D& worldPos)
{
    // 如果没有活动命令，则返回
    if (!m_drawState.hasActiveCommand())
    {
        return;
    }

    // 更新位置信息
    m_drawState.lastPos = worldPos;
    m_drawState.currentPos = worldPos;

    // 处理移动编辑
    if (m_drawState.editType == EditType::Move)
    {
        if (m_drawState.moveSubMode == MoveEditSubMode::AwaitBasePoint)
        {
            m_drawState.moveSubMode = MoveEditSubMode::AwaitTargetPoint;
        }
        else
        {
            m_drawState.moveSubMode = MoveEditSubMode::Idle;
        }

        return;
    }

    if (m_drawState.editType == EditType::GripEdit)
    {
        if (m_drawState.gripSubMode == GripEditSubMode::AwaitTargetPoint)
        {
            m_drawState.gripSubMode = GripEditSubMode::Idle;
        }

        return;
    }

    // 根据绘制类型处理左键按下
    switch (m_drawState.drawType)
    {
    case DrawType::Point:
        m_drawState.pointSubMode = PointDrawSubMode::AwaitPosition;
        break;
    case DrawType::Line:
    case DrawType::Xline:
        if (m_drawState.lineSubMode == LineDrawSubMode::AwaitStartPoint)
        {
            m_drawState.lineSubMode = LineDrawSubMode::AwaitEndPoint;
        }
        else
        {
            m_drawState.lineSubMode = LineDrawSubMode::AwaitStartPoint;
        }
        break;
    case DrawType::Rectangle:
        if (m_drawState.rectangleSubMode == RectangleDrawSubMode::AwaitFirstCorner)
        {
            m_drawState.rectangleSubMode = RectangleDrawSubMode::AwaitSecondCorner;
        }
        else
        {
            m_drawState.rectangleSubMode = RectangleDrawSubMode::AwaitFirstCorner;
        }
        break;
    case DrawType::Polygon:
        if (m_drawState.polygonSubMode == PolygonDrawSubMode::AwaitCenter)
        {
            m_drawState.polygonSubMode = PolygonDrawSubMode::AwaitRadius;
        }
        else
        {
            m_drawState.polygonSubMode = PolygonDrawSubMode::AwaitCenter;
        }
        break;
    case DrawType::Circle:
        if (m_drawState.circleSubMode == CircleDrawSubMode::AwaitCenter)
        {
            m_drawState.circleSubMode = CircleDrawSubMode::AwaitRadius;
        }
        else
        {
            m_drawState.circleSubMode = CircleDrawSubMode::AwaitCenter;
        }
        break;
    case DrawType::Arc:
        switch (m_drawState.arcSubMode)
        {
        case ArcDrawSubMode::AwaitCenter:
            m_drawState.arcSubMode = ArcDrawSubMode::AwaitRadius;
            break;
        case ArcDrawSubMode::AwaitRadius:
            m_drawState.arcSubMode = ArcDrawSubMode::AwaitStartAngle;
            break;
        case ArcDrawSubMode::AwaitStartAngle:
            m_drawState.arcSubMode = ArcDrawSubMode::AwaitEndAngle;
            break;
        default:
            m_drawState.arcSubMode = ArcDrawSubMode::AwaitCenter;
            break;
        }
        break;
    case DrawType::Ellipse:
        switch (m_drawState.ellipseSubMode)
        {
        case EllipseDrawSubMode::AwaitCenter:
            m_drawState.ellipseSubMode = EllipseDrawSubMode::AwaitMajorAxis;
            break;
        case EllipseDrawSubMode::AwaitMajorAxis:
            m_drawState.ellipseSubMode = EllipseDrawSubMode::AwaitMinorAxis;
            break;
        default:
            m_drawState.ellipseSubMode = EllipseDrawSubMode::AwaitCenter;
            break;
        }
        break;
    case DrawType::Polyline:
        if (m_drawState.polylineSubMode == PolylineDrawSubMode::AwaitFirstPoint)
        {
            m_drawState.polylineSubMode = m_drawState.polylineArcMode
                ? PolylineDrawSubMode::AwaitArcEndPoint
                : PolylineDrawSubMode::AwaitLineEndPoint;
        }
        break;
    case DrawType::LWPolyline:
        if (m_drawState.lwPolylineSubMode == LWPolylineDrawSubMode::AwaitFirstPoint)
        {
            m_drawState.lwPolylineSubMode = m_drawState.lwPolylineArcMode
                ? LWPolylineDrawSubMode::AwaitArcEndPoint
                : LWPolylineDrawSubMode::AwaitLineEndPoint;
        }
        break;
    default:
        break;
    }
}

// 检查是否有多段线命令处于活动状态
// @return 如果多段线命令活动则返回 true
bool CadController::isPolylineCommandActive() const
{
    return m_drawState.isDrawing
        && (m_drawState.drawType == DrawType::Polyline || m_drawState.drawType == DrawType::LWPolyline);
}

// 设置多段线输入模式
// @param useArc 是否使用圆弧模式
// @return 如果成功设置模式返回 true
bool CadController::setPolylineInputMode(bool useArc)
{
    if (!isPolylineCommandActive())
    {
        return false;
    }

    const bool lightweight = m_drawState.drawType == DrawType::LWPolyline;
    QVector<QVector3D>& commandPoints = m_drawState.commandPoints;

    // 圆弧模式需要至少两个点（一段前置线段）
    if (useArc && commandPoints.size() < 2)
    {
        if (m_viewer != nullptr)
        {
            m_viewer->appendCommandMessage(QStringLiteral("圆弧段需要先确定至少一段前置线段"));
            m_viewer->refreshCommandPrompt();
        }

        return true;
    }

    // 根据多段线类型设置相应模式
    if (lightweight)
    {
        m_drawState.lwPolylineArcMode = useArc;

        if (m_drawState.lwPolylineSubMode != LWPolylineDrawSubMode::AwaitFirstPoint)
        {
            m_drawState.lwPolylineSubMode = useArc ? LWPolylineDrawSubMode::AwaitArcEndPoint : LWPolylineDrawSubMode::AwaitLineEndPoint;
        }
    }
    else
    {
        m_drawState.polylineArcMode = useArc;

        if (m_drawState.polylineSubMode != PolylineDrawSubMode::AwaitFirstPoint)
        {
            m_drawState.polylineSubMode = useArc ? PolylineDrawSubMode::AwaitArcEndPoint : PolylineDrawSubMode::AwaitLineEndPoint;
        }
    }

    // 发送模式切换消息
    if (m_viewer != nullptr)
    {
        m_viewer->appendCommandMessage(useArc ? QStringLiteral("已切换到圆弧段输入") : QStringLiteral("已切换到直线段输入"));
        m_viewer->refreshCommandPrompt();
    }

    return true;
}

// 将屏幕坐标转换为当前世界坐标
// @param screenPos 屏幕坐标
// @return 对应的世界坐标
QVector3D CadController::currentWorldPos(const QPoint& screenPos) const
{
    if (m_viewer == nullptr)
    {
        return QVector3D();
    }

    return m_viewer->resolveInteractiveWorldPosition(screenPos);
}

void CadController::syncCurrentPosWithCursor()
{
    if (m_viewer == nullptr || !m_drawState.hasActiveCommand())
    {
        return;
    }

    const QPoint localCursorPos = m_viewer->mapFromGlobal(QCursor::pos());

    if (m_viewer->rect().contains(localCursorPos))
    {
        m_drawState.currentScreenPos = localCursorPos;
    }

    QVector3D resolvedWorldPos = applyOrthoConstraint(currentWorldPos(m_drawState.currentScreenPos));

    if (isPointDynamicFieldModeActive())
    {
        resolvedWorldPos = applyPointDynamicFieldOverride(resolvedWorldPos, true);
    }

    m_drawState.currentPos = resolvedWorldPos;
}

