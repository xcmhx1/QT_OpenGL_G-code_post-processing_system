// CadController 实现文件
// 实现 CadController 模块，对应头文件中声明的主要行为和协作流程。
// 输入控制模块，负责解释键盘、鼠标、滚轮事件并驱动绘图/编辑命令。

#include "pch.h"

#include "CadController.h"

// Qt 核心模块
#include <QColorDialog>
#include <QCursor>

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
        m_viewer->appendCommandMessage(QStringLiteral("已进入%1命令").arg(drawTypeName(drawType)));
        m_viewer->refreshCommandPrompt();
    }
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

// 获取当前命令提示文本
// @return 命令提示字符串
QString CadController::currentPrompt() const
{
    QString basePrompt = QStringLiteral("无活动命令");

    // 编辑命令提示
    if (m_drawState.editType == EditType::Move)
    {
        switch (m_drawState.moveSubMode)
        {
        case MoveEditSubMode::AwaitBasePoint:
            basePrompt = QStringLiteral("MOVE: 指定基点");
            break;
        case MoveEditSubMode::AwaitTargetPoint:
            basePrompt = QStringLiteral("MOVE: 指定目标点");
            break;
        default:
            basePrompt = QStringLiteral("MOVE");
            break;
        }

        return appendDynamicInputPromptState(basePrompt);
    }

    if (m_drawState.editType == EditType::GripEdit)
    {
        basePrompt = QStringLiteral("GRIP: 指定目标点");
        return appendDynamicInputPromptState(basePrompt);
    }

    // 无活动命令提示
    if (!m_drawState.isDrawing)
    {
        return appendDynamicInputPromptState(basePrompt);
    }

    // 根据当前绘制类型和子模式返回相应提示
    switch (m_drawState.drawType)
    {
    case DrawType::Point:
        basePrompt = QStringLiteral("POINT: 指定点位置");
        break;
    case DrawType::Line:
        basePrompt = m_drawState.lineSubMode == LineDrawSubMode::AwaitEndPoint
            ? QStringLiteral("LINE: 指定下一点")
            : QStringLiteral("LINE: 指定第一点");
        break;
    case DrawType::Circle:
        basePrompt = m_drawState.circleSubMode == CircleDrawSubMode::AwaitRadius
            ? QStringLiteral("CIRCLE: 指定半径")
            : QStringLiteral("CIRCLE: 指定圆心");
        break;
    case DrawType::Arc:
        switch (m_drawState.arcSubMode)
        {
        case ArcDrawSubMode::AwaitRadius:
            basePrompt = QStringLiteral("ARC: 指定半径");
            break;
        case ArcDrawSubMode::AwaitStartAngle:
            basePrompt = QStringLiteral("ARC: 指定起始角");
            break;
        case ArcDrawSubMode::AwaitEndAngle:
            basePrompt = QStringLiteral("ARC: 指定终止角");
            break;
        default:
            basePrompt = QStringLiteral("ARC: 指定圆心");
            break;
        }
        break;
    case DrawType::Ellipse:
        switch (m_drawState.ellipseSubMode)
        {
        case EllipseDrawSubMode::AwaitMajorAxis:
            basePrompt = QStringLiteral("ELLIPSE: 指定长轴端点");
            break;
        case EllipseDrawSubMode::AwaitMinorAxis:
            basePrompt = QStringLiteral("ELLIPSE: 指定短轴距离");
            break;
        default:
            basePrompt = QStringLiteral("ELLIPSE: 指定中心");
            break;
        }
        break;
    case DrawType::Polyline:
        if (m_drawState.polylineSubMode == PolylineDrawSubMode::AwaitArcEndPoint)
        {
            basePrompt = QStringLiteral("POLYLINE[圆弧]: 指定圆弧终点，L切换直线，Enter/Space结束，C闭合");
            break;
        }

        if (m_drawState.polylineSubMode == PolylineDrawSubMode::AwaitLineEndPoint)
        {
            basePrompt = QStringLiteral("POLYLINE[直线]: 指定下一点，A切换圆弧，Enter/Space结束，C闭合");
            break;
        }

        basePrompt = QStringLiteral("POLYLINE: 指定第一点");
        break;
    case DrawType::LWPolyline:
        if (m_drawState.lwPolylineSubMode == LWPolylineDrawSubMode::AwaitArcEndPoint)
        {
            basePrompt = QStringLiteral("LWPOLYLINE[圆弧]: 指定圆弧终点，L切换直线，Enter/Space结束，C闭合");
            break;
        }

        if (m_drawState.lwPolylineSubMode == LWPolylineDrawSubMode::AwaitLineEndPoint)
        {
            basePrompt = QStringLiteral("LWPOLYLINE[直线]: 指定下一点，A切换圆弧，Enter/Space结束，C闭合");
            break;
        }

        basePrompt = QStringLiteral("LWPOLYLINE: 指定第一点");
        break;
    default:
        break;
    }

    return appendDynamicInputPromptState(basePrompt);
}

// 获取当前命令名称
// @return 命令名称字符串
QString CadController::currentCommandName() const
{
    if (m_drawState.editType == EditType::Move)
    {
        return QStringLiteral("移动");
    }

    if (m_drawState.editType == EditType::GripEdit)
    {
        return QStringLiteral("控制点编辑");
    }

    return drawTypeName(m_drawState.drawType);
}

CadDynamicInputOverlayState CadController::dynamicInputOverlayState() const
{
    CadDynamicInputOverlayState state;

    if (!m_drawState.hasActiveCommand() || !isAwaitingPointInput())
    {
        return state;
    }

    state.visible = true;
    state.title = currentCommandName();

    if (state.title.trimmed().isEmpty() || state.title == QStringLiteral("空闲"))
    {
        state.title = QStringLiteral("动态输入");
    }

    state.stageHint = QStringLiteral("Tab切换字段，Enter/Space确认，Esc清空输入");
    state.xLocked = m_drawState.dynamicInputXLocked;
    state.yLocked = m_drawState.dynamicInputYLocked;
    state.xActive = m_drawState.dynamicInputActiveFieldIndex == 0;
    state.yActive = m_drawState.dynamicInputActiveFieldIndex == 1;
    state.expressionMode = m_drawState.dynamicInputExpressionMode;

    if (state.expressionMode)
    {
        state.expressionText = m_drawState.dynamicInputBuffer;
        return state;
    }

    const QVector3D previewPoint = applyPointDynamicFieldOverride(m_drawState.currentPos, true);
    state.xValueText = formatDynamicInputValue(previewPoint.x());
    state.yValueText = formatDynamicInputValue(previewPoint.y());

    if (!m_drawState.dynamicInputFieldBuffer.isEmpty())
    {
        if (state.xActive)
        {
            state.xValueText = m_drawState.dynamicInputFieldBuffer;
        }
        else
        {
            state.yValueText = m_drawState.dynamicInputFieldBuffer;
        }
    }

    return state;
}

CadDynamicCommandOverlayState CadController::dynamicCommandOverlayState() const
{
    CadDynamicCommandOverlayState state;

    if (!isDynamicCommandModeActive())
    {
        return state;
    }

    state.visible = true;
    state.inputText = m_drawState.dynamicCommandBuffer;
    state.hintText = QStringLiteral("Tab/Shift+Tab 选择，Enter/Space 执行，Esc 取消");

    const QVector<int> matchIndices = collectDynamicCommandMatchIndices();

    if (matchIndices.isEmpty())
    {
        state.candidates = { QStringLiteral("无匹配命令") };
        state.activeCandidateIndex = 0;
        return state;
    }

    const QVector<DynamicCommandDefinition>& definitions = dynamicCommandDefinitions();
    state.candidates.reserve(matchIndices.size());

    for (int matchIndex : matchIndices)
    {
        if (matchIndex >= 0 && matchIndex < definitions.size())
        {
            state.candidates.append(definitions.at(matchIndex).displayName);
        }
    }

    const int maxIndex = std::max(0, static_cast<int>(state.candidates.size()) - 1);
    state.activeCandidateIndex = std::clamp(m_drawState.dynamicCommandActiveIndex, 0, maxIndex);
    return state;
}

bool CadController::isDynamicCommandModeActive() const
{
    return !m_drawState.hasActiveCommand() && !m_drawState.dynamicCommandBuffer.trimmed().isEmpty();
}

void CadController::clearDynamicCommandMode()
{
    m_drawState.dynamicCommandBuffer.clear();
    m_drawState.dynamicCommandActiveIndex = 0;
}

QVector<int> CadController::collectDynamicCommandMatchIndices() const
{
    QVector<int> matches;

    if (m_drawState.hasActiveCommand())
    {
        return matches;
    }

    const QString normalizedInput = normalizedCommandToken(m_drawState.dynamicCommandBuffer);
    const QVector<DynamicCommandDefinition>& definitions = dynamicCommandDefinitions();

    for (int index = 0; index < definitions.size(); ++index)
    {
        if (commandAliasMatches(definitions.at(index), normalizedInput))
        {
            matches.append(index);
        }
    }

    return matches;
}

void CadController::normalizeDynamicCommandSelectionIndex()
{
    const QVector<int> matchIndices = collectDynamicCommandMatchIndices();

    if (matchIndices.isEmpty())
    {
        m_drawState.dynamicCommandActiveIndex = 0;
        return;
    }

    const int maxIndex = matchIndices.size() - 1;
    m_drawState.dynamicCommandActiveIndex = std::clamp(m_drawState.dynamicCommandActiveIndex, 0, maxIndex);
}

bool CadController::cycleDynamicCommandSelection(int step)
{
    if (!isDynamicCommandModeActive())
    {
        return false;
    }

    const QVector<int> matchIndices = collectDynamicCommandMatchIndices();

    if (matchIndices.isEmpty())
    {
        return true;
    }

    const int size = matchIndices.size();
    const int currentIndex = std::clamp(m_drawState.dynamicCommandActiveIndex, 0, size - 1);
    const int stepNormalized = step >= 0 ? 1 : -1;
    const int nextIndex = (currentIndex + stepNormalized + size) % size;
    m_drawState.dynamicCommandActiveIndex = nextIndex;

    if (m_viewer != nullptr)
    {
        m_viewer->refreshCommandPrompt();
        m_viewer->requestViewUpdate();
    }

    return true;
}

bool CadController::executeSelectedDynamicCommand()
{
    if (!isDynamicCommandModeActive())
    {
        return false;
    }

    const QVector<int> matchIndices = collectDynamicCommandMatchIndices();

    if (matchIndices.isEmpty())
    {
        if (m_viewer != nullptr)
        {
            m_viewer->appendCommandMessage(QStringLiteral("未找到可执行命令"));
            m_viewer->refreshCommandPrompt();
            m_viewer->requestViewUpdate();
        }

        return true;
    }

    const int selectedOrdinal = std::clamp(m_drawState.dynamicCommandActiveIndex, 0, static_cast<int>(matchIndices.size()) - 1);
    const int selectedIndex = matchIndices.at(selectedOrdinal);
    const QVector<DynamicCommandDefinition>& definitions = dynamicCommandDefinitions();
    const QString canonicalCommand = definitions.at(selectedIndex).canonical;
    clearDynamicCommandMode();
    const bool handled = executeIdleCommandByCanonical(canonicalCommand);

    if (!handled && m_viewer != nullptr)
    {
        m_viewer->appendCommandMessage(QStringLiteral("命令执行失败: %1").arg(canonicalCommand));
        m_viewer->refreshCommandPrompt();
        m_viewer->requestViewUpdate();
    }

    return true;
}

bool CadController::executeIdleCommandByCanonical(const QString& canonicalCommand)
{
    const QString normalized = normalizedCommandToken(canonicalCommand);

    if (normalized == QStringLiteral("point"))
    {
        beginDrawing(DrawType::Point, m_drawState.drawingColor);
        return true;
    }

    if (normalized == QStringLiteral("line"))
    {
        beginDrawing(DrawType::Line, m_drawState.drawingColor);
        return true;
    }

    if (normalized == QStringLiteral("circle"))
    {
        beginDrawing(DrawType::Circle, m_drawState.drawingColor);
        return true;
    }

    if (normalized == QStringLiteral("arc"))
    {
        beginDrawing(DrawType::Arc, m_drawState.drawingColor);
        return true;
    }

    if (normalized == QStringLiteral("ellipse"))
    {
        beginDrawing(DrawType::Ellipse, m_drawState.drawingColor);
        return true;
    }

    if (normalized == QStringLiteral("polyline"))
    {
        beginDrawing(DrawType::Polyline, m_drawState.drawingColor);
        return true;
    }

    if (normalized == QStringLiteral("lwpolyline"))
    {
        beginDrawing(DrawType::LWPolyline, m_drawState.drawingColor);
        return true;
    }

    if (normalized == QStringLiteral("move"))
    {
        return beginMoveSelected();
    }

    if (normalized == QStringLiteral("delete"))
    {
        return deleteSelectedEntity();
    }

    if (normalized == QStringLiteral("color"))
    {
        return changeSelectedEntityColor();
    }

    if (normalized == QStringLiteral("fit"))
    {
        if (m_viewer != nullptr)
        {
            m_viewer->fitSceneView();
            return true;
        }

        return false;
    }

    if (normalized == QStringLiteral("top"))
    {
        if (m_viewer != nullptr)
        {
            m_viewer->resetToTopView();
            return true;
        }

        return false;
    }

    if (normalized == QStringLiteral("zoomin"))
    {
        if (m_viewer != nullptr)
        {
            m_viewer->zoomIn();
            return true;
        }

        return false;
    }

    if (normalized == QStringLiteral("zoomout"))
    {
        if (m_viewer != nullptr)
        {
            m_viewer->zoomOut();
            return true;
        }

        return false;
    }

    return false;
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

QString CadController::currentPointInputStageKey() const
{
    if (!m_drawState.hasActiveCommand() || !isAwaitingPointInput())
    {
        return QString();
    }

    if (m_drawState.editType == EditType::Move)
    {
        if (m_drawState.moveSubMode == MoveEditSubMode::AwaitBasePoint)
        {
            return QStringLiteral("MOVE_BASE");
        }

        if (m_drawState.moveSubMode == MoveEditSubMode::AwaitTargetPoint)
        {
            return QStringLiteral("MOVE_TARGET");
        }
    }

    if (m_drawState.editType == EditType::GripEdit)
    {
        if (m_drawState.gripSubMode == GripEditSubMode::AwaitTargetPoint)
        {
            return QStringLiteral("GRIP_TARGET");
        }
    }

    switch (m_drawState.drawType)
    {
    case DrawType::Point:
        return QStringLiteral("POINT_POSITION");
    case DrawType::Line:
        return m_drawState.lineSubMode == LineDrawSubMode::AwaitEndPoint
            ? QStringLiteral("LINE_END")
            : QStringLiteral("LINE_START");
    case DrawType::Circle:
        return m_drawState.circleSubMode == CircleDrawSubMode::AwaitRadius
            ? QStringLiteral("CIRCLE_RADIUS")
            : QStringLiteral("CIRCLE_CENTER");
    case DrawType::Arc:
        switch (m_drawState.arcSubMode)
        {
        case ArcDrawSubMode::AwaitRadius:
            return QStringLiteral("ARC_RADIUS");
        case ArcDrawSubMode::AwaitStartAngle:
            return QStringLiteral("ARC_START");
        case ArcDrawSubMode::AwaitEndAngle:
            return QStringLiteral("ARC_END");
        default:
            return QStringLiteral("ARC_CENTER");
        }
    case DrawType::Ellipse:
        switch (m_drawState.ellipseSubMode)
        {
        case EllipseDrawSubMode::AwaitMajorAxis:
            return QStringLiteral("ELLIPSE_MAJOR");
        case EllipseDrawSubMode::AwaitMinorAxis:
            return QStringLiteral("ELLIPSE_MINOR");
        default:
            return QStringLiteral("ELLIPSE_CENTER");
        }
    case DrawType::Polyline:
        return m_drawState.polylineSubMode == PolylineDrawSubMode::AwaitFirstPoint
            ? QStringLiteral("POLYLINE_FIRST")
            : QStringLiteral("POLYLINE_NEXT");
    case DrawType::LWPolyline:
        return m_drawState.lwPolylineSubMode == LWPolylineDrawSubMode::AwaitFirstPoint
            ? QStringLiteral("LWPOLYLINE_FIRST")
            : QStringLiteral("LWPOLYLINE_NEXT");
    default:
        break;
    }

    return QString();
}

void CadController::syncPointDynamicInputSession()
{
    if (!m_drawState.hasActiveCommand() || !isAwaitingPointInput())
    {
        resetPointDynamicInputSession();
        return;
    }

    const QString stageKey = currentPointInputStageKey();

    if (stageKey != m_drawState.dynamicInputStageKey)
    {
        resetPointDynamicInputSession(stageKey);
    }
}

void CadController::resetPointDynamicInputSession(const QString& stageKey)
{
    m_drawState.dynamicInputStageKey = stageKey;
    m_drawState.dynamicInputExpressionMode = false;
    m_drawState.dynamicInputBuffer.clear();
    m_drawState.dynamicInputActiveFieldIndex = 0;
    m_drawState.dynamicInputFieldBuffer.clear();
    m_drawState.dynamicInputXLocked = false;
    m_drawState.dynamicInputYLocked = false;
    m_drawState.dynamicInputXValue = 0.0;
    m_drawState.dynamicInputYValue = 0.0;
}

bool CadController::isPointDynamicFieldModeActive() const
{
    return m_drawState.hasActiveCommand()
        && isAwaitingPointInput()
        && !m_drawState.dynamicInputExpressionMode;
}

bool CadController::hasPendingDynamicKeyboardInput() const
{
    if (m_drawState.dynamicInputExpressionMode)
    {
        return !m_drawState.dynamicInputBuffer.isEmpty();
    }

    if (isPointDynamicFieldModeActive())
    {
        return !m_drawState.dynamicInputFieldBuffer.isEmpty();
    }

    return !m_drawState.dynamicInputBuffer.isEmpty();
}

QVector3D CadController::applyPointDynamicFieldOverride(const QVector3D& worldPos, bool includeEditingValue) const
{
    QVector3D point = flattenToDrawingPlane(worldPos);

    if (m_drawState.dynamicInputXLocked)
    {
        point.setX(static_cast<float>(m_drawState.dynamicInputXValue));
    }

    if (m_drawState.dynamicInputYLocked)
    {
        point.setY(static_cast<float>(m_drawState.dynamicInputYValue));
    }

    if (!includeEditingValue || m_drawState.dynamicInputFieldBuffer.isEmpty())
    {
        return point;
    }

    bool parsedOk = false;
    const double parsedValue = m_drawState.dynamicInputFieldBuffer.toDouble(&parsedOk);

    if (!parsedOk)
    {
        return point;
    }

    if (m_drawState.dynamicInputActiveFieldIndex == 0)
    {
        point.setX(static_cast<float>(parsedValue));
    }
    else
    {
        point.setY(static_cast<float>(parsedValue));
    }

    return point;
}

bool CadController::tryParseDynamicFieldBuffer(double& value, QString& errorMessage) const
{
    if (m_drawState.dynamicInputFieldBuffer.trimmed().isEmpty())
    {
        errorMessage = QStringLiteral("请输入数值");
        return false;
    }

    bool parsedOk = false;
    value = m_drawState.dynamicInputFieldBuffer.toDouble(&parsedOk);

    if (!parsedOk)
    {
        errorMessage = QStringLiteral("数值输入无效");
        return false;
    }

    return true;
}

bool CadController::commitActiveDynamicField(QString& errorMessage)
{
    if (!isPointDynamicFieldModeActive())
    {
        return false;
    }

    const int activeFieldIndex = std::clamp(m_drawState.dynamicInputActiveFieldIndex, 0, 1);
    double fieldValue = (activeFieldIndex == 0) ? m_drawState.currentPos.x() : m_drawState.currentPos.y();

    if (!m_drawState.dynamicInputFieldBuffer.isEmpty())
    {
        if (!tryParseDynamicFieldBuffer(fieldValue, errorMessage))
        {
            return false;
        }
    }

    if (activeFieldIndex == 0)
    {
        m_drawState.dynamicInputXLocked = true;
        m_drawState.dynamicInputXValue = fieldValue;
    }
    else
    {
        m_drawState.dynamicInputYLocked = true;
        m_drawState.dynamicInputYValue = fieldValue;
    }

    m_drawState.dynamicInputFieldBuffer.clear();
    return true;
}

bool CadController::handleDynamicFieldTab(int step)
{
    if (!isPointDynamicFieldModeActive())
    {
        return false;
    }

    QString errorMessage;

    if (!commitActiveDynamicField(errorMessage))
    {
        if (m_viewer != nullptr && !errorMessage.isEmpty())
        {
            m_viewer->appendCommandMessage(errorMessage);
            m_viewer->refreshCommandPrompt();
        }

        return true;
    }

    const int normalizedStep = step >= 0 ? 1 : -1;
    int nextFieldIndex = m_drawState.dynamicInputActiveFieldIndex + normalizedStep;

    if (nextFieldIndex < 0)
    {
        nextFieldIndex = 1;
    }
    else if (nextFieldIndex > 1)
    {
        nextFieldIndex = 0;
    }

    m_drawState.dynamicInputActiveFieldIndex = nextFieldIndex;
    m_drawState.currentPos = applyPointDynamicFieldOverride(m_drawState.currentPos, false);

    if (m_viewer != nullptr)
    {
        m_viewer->refreshCommandPrompt();
        m_viewer->requestViewUpdate();
    }

    return true;
}

bool CadController::submitPointDynamicFieldInput()
{
    if (!isPointDynamicFieldModeActive())
    {
        return false;
    }

    QString errorMessage;

    if (!m_drawState.dynamicInputFieldBuffer.isEmpty() && !commitActiveDynamicField(errorMessage))
    {
        if (m_viewer != nullptr)
        {
            m_viewer->appendCommandMessage(errorMessage);
            m_viewer->refreshCommandPrompt();
        }

        return true;
    }

    const QVector3D resolvedPoint = applyPointDynamicFieldOverride(m_drawState.currentPos, false);

    if (commitCommandPoint(resolvedPoint))
    {
        return true;
    }

    if (m_viewer != nullptr)
    {
        m_viewer->appendCommandMessage(QStringLiteral("输入未被当前命令接受"));
        m_viewer->refreshCommandPrompt();
    }

    return true;
}

QString CadController::formatDynamicInputValue(double value)
{
    if (std::abs(value) < 1e-9)
    {
        value = 0.0;
    }

    QString text = QString::number(value, 'f', 6);

    while (text.contains(QLatin1Char('.')) && text.endsWith(QLatin1Char('0')))
    {
        text.chop(1);
    }

    if (text.endsWith(QLatin1Char('.')))
    {
        text.chop(1);
    }

    if (text.isEmpty() || text == QStringLiteral("-0"))
    {
        text = QStringLiteral("0");
    }

    return text;
}

bool CadController::isAwaitingPointInput() const
{
    if (m_drawState.editType == EditType::Move)
    {
        return m_drawState.moveSubMode == MoveEditSubMode::AwaitBasePoint
            || m_drawState.moveSubMode == MoveEditSubMode::AwaitTargetPoint;
    }

    if (m_drawState.editType == EditType::GripEdit)
    {
        return m_drawState.gripSubMode == GripEditSubMode::AwaitTargetPoint;
    }

    if (!m_drawState.isDrawing)
    {
        return false;
    }

    switch (m_drawState.drawType)
    {
    case DrawType::Point:
        return m_drawState.pointSubMode == PointDrawSubMode::AwaitPosition;
    case DrawType::Line:
        return m_drawState.lineSubMode == LineDrawSubMode::AwaitStartPoint
            || m_drawState.lineSubMode == LineDrawSubMode::AwaitEndPoint;
    case DrawType::Circle:
        return m_drawState.circleSubMode == CircleDrawSubMode::AwaitCenter
            || m_drawState.circleSubMode == CircleDrawSubMode::AwaitRadius;
    case DrawType::Arc:
        return m_drawState.arcSubMode == ArcDrawSubMode::AwaitCenter
            || m_drawState.arcSubMode == ArcDrawSubMode::AwaitRadius
            || m_drawState.arcSubMode == ArcDrawSubMode::AwaitStartAngle
            || m_drawState.arcSubMode == ArcDrawSubMode::AwaitEndAngle;
    case DrawType::Ellipse:
        return m_drawState.ellipseSubMode == EllipseDrawSubMode::AwaitCenter
            || m_drawState.ellipseSubMode == EllipseDrawSubMode::AwaitMajorAxis
            || m_drawState.ellipseSubMode == EllipseDrawSubMode::AwaitMinorAxis;
    case DrawType::Polyline:
        return m_drawState.polylineSubMode == PolylineDrawSubMode::AwaitFirstPoint
            || m_drawState.polylineSubMode == PolylineDrawSubMode::AwaitLineEndPoint
            || m_drawState.polylineSubMode == PolylineDrawSubMode::AwaitArcEndPoint;
    case DrawType::LWPolyline:
        return m_drawState.lwPolylineSubMode == LWPolylineDrawSubMode::AwaitFirstPoint
            || m_drawState.lwPolylineSubMode == LWPolylineDrawSubMode::AwaitLineEndPoint
            || m_drawState.lwPolylineSubMode == LWPolylineDrawSubMode::AwaitArcEndPoint;
    default:
        break;
    }

    return false;
}

QVector3D CadController::dynamicInputReferencePoint() const
{
    if (!m_drawState.commandPoints.isEmpty())
    {
        return flattenToDrawingPlane(m_drawState.commandPoints.back());
    }

    return flattenToDrawingPlane(m_drawState.currentPos);
}

QVector3D CadController::applyOrthoConstraint(const QVector3D& worldPos) const
{
    const QVector3D planarPoint = flattenToDrawingPlane(worldPos);

    if (!m_drawState.orthoEnabled
        || !m_drawState.hasActiveCommand()
        || !isAwaitingPointInput()
        || m_drawState.commandPoints.isEmpty())
    {
        return planarPoint;
    }

    const QVector3D basePoint = dynamicInputReferencePoint();
    const QVector3D delta = planarPoint - basePoint;

    if (std::abs(delta.x()) >= std::abs(delta.y()))
    {
        return QVector3D(planarPoint.x(), basePoint.y(), 0.0f);
    }

    return QVector3D(basePoint.x(), planarPoint.y(), 0.0f);
}

bool CadController::tryResolveDynamicInputPoint(const QString& inputText, QVector3D& worldPoint, QString& errorMessage) const
{
    QString normalizedInput = inputText;
    normalizedInput.remove(QLatin1Char(' '));
    normalizedInput.remove(QLatin1Char('\t'));

    if (normalizedInput.isEmpty())
    {
        errorMessage = QStringLiteral("请输入坐标值");
        return false;
    }

    const QVector3D referencePoint = dynamicInputReferencePoint();
    const int polarSeparator = normalizedInput.indexOf(QLatin1Char('<'));

    if (polarSeparator >= 0)
    {
        const QString distanceText = normalizedInput.left(polarSeparator).trimmed();
        const QString angleText = normalizedInput.mid(polarSeparator + 1).trimmed();
        const QString normalizedDistanceText = distanceText.startsWith(QLatin1Char('@'))
            ? distanceText.mid(1)
            : distanceText;

        bool distanceOk = false;
        bool angleOk = false;
        const double distance = normalizedDistanceText.toDouble(&distanceOk);
        const double angleDegrees = angleText.toDouble(&angleOk);

        if (!distanceOk || !angleOk)
        {
            errorMessage = QStringLiteral("极坐标输入格式无效，应为 距离<角度，例如 100<30");
            return false;
        }

        constexpr double kRadiansPerDegree = 3.14159265358979323846 / 180.0;
        const double radians = angleDegrees * kRadiansPerDegree;
        worldPoint = QVector3D
        (
            static_cast<float>(referencePoint.x() + distance * std::cos(radians)),
            static_cast<float>(referencePoint.y() + distance * std::sin(radians)),
            0.0f
        );
        worldPoint = applyOrthoConstraint(worldPoint);
        return true;
    }

    const bool relative = normalizedInput.startsWith(QLatin1Char('@'));
    const QString coordinateText = relative ? normalizedInput.mid(1) : normalizedInput;
    double firstValue = 0.0;
    double secondValue = 0.0;

    if (!tryParseCoordinatePair(coordinateText, firstValue, secondValue))
    {
        errorMessage = QStringLiteral("坐标输入格式无效，应为 x,y 或 @dx,dy");
        return false;
    }

    if (relative)
    {
        worldPoint = QVector3D
        (
            static_cast<float>(referencePoint.x() + firstValue),
            static_cast<float>(referencePoint.y() + secondValue),
            0.0f
        );
    }
    else
    {
        worldPoint = QVector3D(static_cast<float>(firstValue), static_cast<float>(secondValue), 0.0f);
    }

    worldPoint = applyOrthoConstraint(worldPoint);
    return true;
}

bool CadController::commitCommandPoint(const QVector3D& worldPos)
{
    if (!m_drawState.hasActiveCommand() || m_editer == nullptr)
    {
        return false;
    }

    syncPointDynamicInputSession();

    QVector3D constrainedWorldPos = applyOrthoConstraint(worldPos);

    if (isPointDynamicFieldModeActive())
    {
        constrainedWorldPos = applyPointDynamicFieldOverride(constrainedWorldPos, true);
    }

    m_drawState.currentPos = constrainedWorldPos;

    // 保存之前的状态用于比较
    const DrawStateMachine previousState = m_drawState;

    // 处理命令状态下的点提交
    handleLeftPressInCommand(constrainedWorldPos);

    // 尝试由编辑器处理点提交事件
    if (!m_editer->handleLeftPress(previousState, m_drawState, constrainedWorldPos))
    {
        return false;
    }

    // 每次点提交后都重置输入会话，避免同阶段残留锁定值影响下一击。
    resetPointDynamicInputSession(currentPointInputStageKey());

    // 键盘确认点后，立即按当前光标位置刷新下一阶段预览点。
    if (m_drawState.hasActiveCommand() && isAwaitingPointInput())
    {
        syncCurrentPosWithCursor();
    }

    if (m_viewer != nullptr)
    {
        if (previousState.editType == EditType::Move && m_drawState.editType == EditType::None)
        {
            m_viewer->appendCommandMessage(QStringLiteral("移动完成"));
        }
        else if (previousState.editType == EditType::GripEdit && m_drawState.editType == EditType::None)
        {
            m_viewer->appendCommandMessage(QStringLiteral("控制点编辑完成"));
        }
        else if (previousState.drawType == DrawType::Point)
        {
            m_viewer->appendCommandMessage(QStringLiteral("已创建点图元"));
        }
        else if (previousState.drawType == DrawType::Circle
            && previousState.circleSubMode == CircleDrawSubMode::AwaitRadius
            && m_drawState.circleSubMode == CircleDrawSubMode::AwaitCenter)
        {
            m_viewer->appendCommandMessage(QStringLiteral("已创建圆图元"));
        }
        else if (previousState.drawType == DrawType::Arc
            && previousState.arcSubMode == ArcDrawSubMode::AwaitEndAngle
            && m_drawState.arcSubMode == ArcDrawSubMode::AwaitCenter)
        {
            m_viewer->appendCommandMessage(QStringLiteral("已创建圆弧图元"));
        }
        else if (previousState.drawType == DrawType::Ellipse
            && previousState.ellipseSubMode == EllipseDrawSubMode::AwaitMinorAxis
            && m_drawState.ellipseSubMode == EllipseDrawSubMode::AwaitCenter)
        {
            m_viewer->appendCommandMessage(QStringLiteral("已创建椭圆图元"));
        }
        else if (previousState.drawType == DrawType::Line
            && previousState.lineSubMode == LineDrawSubMode::AwaitEndPoint
            && m_drawState.lineSubMode == LineDrawSubMode::AwaitEndPoint)
        {
            m_viewer->appendCommandMessage(QStringLiteral("已创建直线图元"));
        }

        m_viewer->refreshCommandPrompt();
        m_viewer->requestViewUpdate();
    }

    return true;
}

bool CadController::submitDynamicInputBuffer()
{
    syncPointDynamicInputSession();

    if (m_drawState.dynamicInputBuffer.isEmpty())
    {
        return false;
    }

    if (!isAwaitingPointInput())
    {
        if (m_viewer != nullptr)
        {
            m_viewer->appendCommandMessage(QStringLiteral("当前命令阶段不接受坐标输入"));
            m_viewer->refreshCommandPrompt();
        }

        return true;
    }

    QVector3D parsedPoint;
    QString errorMessage;

    if (!tryResolveDynamicInputPoint(m_drawState.dynamicInputBuffer, parsedPoint, errorMessage))
    {
        if (m_viewer != nullptr)
        {
            m_viewer->appendCommandMessage(errorMessage);
            m_viewer->refreshCommandPrompt();
        }

        return true;
    }

    if (commitCommandPoint(parsedPoint))
    {
        return true;
    }

    if (m_viewer != nullptr)
    {
        m_viewer->appendCommandMessage(QStringLiteral("输入未被当前命令接受"));
        m_viewer->refreshCommandPrompt();
    }

    return true;
}

bool CadController::confirmActiveCommand()
{
    if (!m_drawState.hasActiveCommand())
    {
        return false;
    }

    syncPointDynamicInputSession();
    syncCurrentPosWithCursor();

    if (isPointDynamicFieldModeActive())
    {
        if ((m_drawState.drawType == DrawType::Polyline || m_drawState.drawType == DrawType::LWPolyline)
            && m_editer != nullptr
            && !hasPendingDynamicKeyboardInput()
            && !m_drawState.dynamicInputXLocked
            && !m_drawState.dynamicInputYLocked)
        {
            const bool handled = m_editer->finishActivePolyline(m_drawState, false);

            if (handled && m_viewer != nullptr)
            {
                resetPointDynamicInputSession(currentPointInputStageKey());
                m_viewer->appendCommandMessage(QStringLiteral("已创建多段线图元"));
                m_viewer->refreshCommandPrompt();
            }

            return true;
        }

        return submitPointDynamicFieldInput();
    }

    if (!m_drawState.dynamicInputBuffer.isEmpty())
    {
        return submitDynamicInputBuffer();
    }

    if ((m_drawState.drawType == DrawType::Polyline || m_drawState.drawType == DrawType::LWPolyline)
        && m_editer != nullptr)
    {
        const bool handled = m_editer->finishActivePolyline(m_drawState, false);

        if (handled && m_viewer != nullptr)
        {
            resetPointDynamicInputSession(currentPointInputStageKey());
            m_viewer->appendCommandMessage(QStringLiteral("已创建多段线图元"));
            m_viewer->refreshCommandPrompt();
        }

        return true;
    }

    if (isAwaitingPointInput())
    {
        if (!commitCommandPoint(m_drawState.currentPos) && m_viewer != nullptr)
        {
            m_viewer->refreshCommandPrompt();
        }

        return true;
    }

    return true;
}

QString CadController::appendDynamicInputPromptState(const QString& basePrompt) const
{
    QString prompt = basePrompt;

    if (m_drawState.hasActiveCommand() && isAwaitingPointInput())
    {
        if (m_drawState.dynamicInputExpressionMode)
        {
            if (!m_drawState.dynamicInputBuffer.isEmpty())
            {
                prompt += QStringLiteral(" | 表达式: %1").arg(m_drawState.dynamicInputBuffer);
            }
        }
        else
        {
            const QVector3D previewPoint = applyPointDynamicFieldOverride(m_drawState.currentPos, true);
            QString xText = formatDynamicInputValue(previewPoint.x());
            QString yText = formatDynamicInputValue(previewPoint.y());

            if (!m_drawState.dynamicInputFieldBuffer.isEmpty())
            {
                if (m_drawState.dynamicInputActiveFieldIndex == 0)
                {
                    xText = m_drawState.dynamicInputFieldBuffer;
                }
                else
                {
                    yText = m_drawState.dynamicInputFieldBuffer;
                }
            }

            prompt += QStringLiteral(" | X%1=%2 | Y%3=%4 | Tab切换")
                .arg(m_drawState.dynamicInputXLocked ? QStringLiteral("[锁]") : QString())
                .arg(xText)
                .arg(m_drawState.dynamicInputYLocked ? QStringLiteral("[锁]") : QString())
                .arg(yText);
        }
    }
    else if (isDynamicCommandModeActive())
    {
        const QVector<int> matchIndices = collectDynamicCommandMatchIndices();

        if (!matchIndices.isEmpty())
        {
            const int selectedOrdinal = std::clamp(m_drawState.dynamicCommandActiveIndex, 0, static_cast<int>(matchIndices.size()) - 1);
            const QVector<DynamicCommandDefinition>& definitions = dynamicCommandDefinitions();
            const QString selectedDisplay = definitions.at(matchIndices.at(selectedOrdinal)).displayName;
            prompt += QStringLiteral(" | 命令: %1 | 候选: %2").arg(m_drawState.dynamicCommandBuffer, selectedDisplay);
        }
        else
        {
            prompt += QStringLiteral(" | 命令: %1 | 无匹配").arg(m_drawState.dynamicCommandBuffer);
        }
    }

    if (m_drawState.orthoEnabled)
    {
        prompt += QStringLiteral(" | [正交]");
    }

    return prompt;
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
        m_drawState.lineSubMode = LineDrawSubMode::AwaitStartPoint;
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
        if (m_drawState.lineSubMode == LineDrawSubMode::AwaitStartPoint)
        {
            m_drawState.lineSubMode = LineDrawSubMode::AwaitEndPoint;
        }
        else
        {
            m_drawState.lineSubMode = LineDrawSubMode::AwaitStartPoint;
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

