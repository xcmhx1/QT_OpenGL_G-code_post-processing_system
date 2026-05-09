// CadController 键盘输入处理实现
#include "pch.h"

#include "CadController.h"

#include "CadEditer.h"
#include "CadViewer.h"

#include <QGuiApplication>

namespace
{
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

    bool isDynamicFieldCharacter(QChar character)
    {
        return character.isDigit()
            || character == QLatin1Char('.')
            || character == QLatin1Char('+')
            || character == QLatin1Char('-');
    }

    bool isDynamicCommandCharacter(QChar character)
    {
        return character.isLetterOrNumber()
            || character == QLatin1Char('_')
            || character == QLatin1Char('-');
    }
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
        setOrthoEnabled(!m_drawState.orthoEnabled);

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

    // F10：切换极轴追踪
    if (event->key() == Qt::Key_F10)
    {
        setPolarTrackingEnabled(!m_drawState.polarTrackingEnabled);

        if (m_viewer != nullptr)
        {
            m_viewer->appendCommandMessage
            (
                m_drawState.polarTrackingEnabled
                    ? QStringLiteral("极轴追踪: 开（15°）")
                    : QStringLiteral("极轴追踪: 关")
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
            if (isParameterInputCommandActive())
            {
                if (isAwaitingParameterFieldInput() && !m_parameterInputSession.fieldBuffer.isEmpty())
                {
                    m_parameterInputSession.fieldBuffer.clear();

                    if (m_viewer != nullptr)
                    {
                        m_viewer->refreshCommandPrompt();
                        m_viewer->requestViewUpdate();
                    }

                    return true;
                }
            }
            else if (hasPendingDynamicKeyboardInput()
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

        if (isParameterInputCommandActive() && isAwaitingParameterFieldInput())
        {
            if (event->key() == Qt::Key_Backspace)
            {
                if (!m_parameterInputSession.fieldBuffer.isEmpty())
                {
                    m_parameterInputSession.fieldBuffer.chop(1);

                    if (m_viewer != nullptr)
                    {
                        m_viewer->refreshCommandPrompt();
                        m_viewer->requestViewUpdate();
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

            if ((event->modifiers() & (Qt::ControlModifier | Qt::AltModifier | Qt::MetaModifier)) == 0)
            {
                const QString inputText = event->text();
                bool consumed = false;

                for (const QChar character : inputText)
                {
                    if (character.isNull() || character == QChar::fromLatin1('\r') || character == QChar::fromLatin1('\n'))
                    {
                        continue;
                    }

                    if (character.isSpace())
                    {
                        continue;
                    }

                    m_parameterInputSession.fieldBuffer.append(character);
                    consumed = true;
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

            switch (event->key())
            {
            case Qt::Key_F:
            case Qt::Key_T:
            case Qt::Key_Home:
            case Qt::Key_Plus:
            case Qt::Key_Equal:
            case Qt::Key_Minus:
            case Qt::Key_Underscore:
            case Qt::Key_P:
            case Qt::Key_L:
            case Qt::Key_X:
            case Qt::Key_R:
            case Qt::Key_G:
            case Qt::Key_C:
            case Qt::Key_A:
            case Qt::Key_E:
            case Qt::Key_Delete:
            case Qt::Key_K:
            case Qt::Key_M:
            case Qt::Key_O:
            case Qt::Key_W:
                return true;
            default:
                break;
            }
        }

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

bool CadController::handleKeyRelease(QKeyEvent* event)
{
    Q_UNUSED(event);
    m_drawState.keyboardModifiers = QGuiApplication::keyboardModifiers();
    return false;
}
