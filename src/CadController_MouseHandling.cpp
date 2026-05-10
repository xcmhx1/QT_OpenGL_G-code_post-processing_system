// CadController 鼠标输入与空闲框选实现
#include "pch.h"

#include "CadController.h"

#include "CadEditer.h"
#include "CadViewer.h"

namespace
{
    constexpr int kWindowSelectionDragThresholdPixels = 6;
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

    const QVector3D worldPos = currentWorldPos(event->pos());

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

    if (event->button() == Qt::MiddleButton)
    {
        if ((event->modifiers() & Qt::ShiftModifier) != 0)
        {
            m_viewer->beginOrbitInteraction();
        }
        else
        {
            m_viewer->beginPanInteraction();
        }

        return true;
    }

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

    const QVector3D worldPos = currentWorldPos(event->pos());

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

    updateCopyPreviewFromCursor();
    updateRotatePreviewFromCursor();
    updateScalePreviewFromCursor();
    updateMirrorPreviewFromCursor();
    updateArrayPreviewState();
    updateOffsetPreviewFromCursor();

    if (m_viewer->interactionMode() == ViewInteractionMode::Orbiting && m_viewer->shouldIgnoreNextOrbitDelta())
    {
        m_viewer->consumeIgnoreNextOrbitDelta();
        return true;
    }

    const QPoint delta = m_drawState.currentScreenPos - m_drawState.lastScreenPos;

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

    m_drawState.lastScreenPos = m_drawState.currentScreenPos;
    m_drawState.currentScreenPos = event->pos();
    m_drawState.activeButton = event->button();
    m_drawState.pressedButtons = event->buttons();
    m_drawState.keyboardModifiers = event->modifiers();

    if (event->button() == Qt::MiddleButton)
    {
        m_viewer->endViewInteraction();
        return true;
    }

    if (event->button() == Qt::LeftButton && !m_drawState.hasActiveCommand())
    {
        return finishIdleWindowSelection(event->pos());
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
