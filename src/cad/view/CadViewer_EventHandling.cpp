// CadViewer Qt 事件入口实现
#include "platform/pch.h"

#include "cad/view/CadViewer.h"

#include <QMimeData>
#include <QUrl>

namespace
{
    bool isSupportedDropFile(const QString& localFile)
    {
        return localFile.endsWith(QStringLiteral(".dxf"), Qt::CaseInsensitive)
            || localFile.endsWith(QStringLiteral(".dwg"), Qt::CaseInsensitive)
            || localFile.endsWith(QStringLiteral(".bmp"), Qt::CaseInsensitive)
            || localFile.endsWith(QStringLiteral(".png"), Qt::CaseInsensitive)
            || localFile.endsWith(QStringLiteral(".jpg"), Qt::CaseInsensitive)
            || localFile.endsWith(QStringLiteral(".jpeg"), Qt::CaseInsensitive);
    }

    bool isPopupCycleModifierValid(Qt::KeyboardModifiers modifiers)
    {
        const Qt::KeyboardModifiers maskedModifiers = modifiers & ~(Qt::ShiftModifier);
        return maskedModifiers == Qt::NoModifier;
    }
}

void CadViewer::mousePressEvent(QMouseEvent* event)
{
    ensureBlankCursor();
    m_cursorScreenPos = event->pos();
    m_showCrosshairOverlay = rect().contains(event->pos());

    if (event->button() == Qt::LeftButton && handleViewCubeClick(event->pos()))
    {
        setCursor(Qt::ArrowCursor);
        m_showCrosshairOverlay = false;
        event->accept();
        return;
    }

    if (event->button() == Qt::LeftButton && handleOverlappedHandlePopupPress(event->pos()))
    {
        event->accept();
        updateHoveredWorldPosition(event->pos());
        return;
    }

    if (event->button() == Qt::LeftButton && handleProcessOrderLabelClick(event->pos()))
    {
        event->accept();
        updateHoveredWorldPosition(event->pos());
        refreshCommandPrompt();
        update();
        return;
    }

    const bool controllerHandled = m_controller.handleMousePress(event);

    if (!controllerHandled && event->button() == Qt::RightButton)
    {
        emit machiningContextMenuRequested(event->globalPosition().toPoint());
        event->accept();
    }
    else if (!controllerHandled)
    {
        QOpenGLWidget::mousePressEvent(event);
    }

    updateHoveredWorldPosition(event->pos());
    updateOverlappedHandleHoverState(event->pos());
    refreshCommandPrompt();
    update();
}

void CadViewer::mouseDoubleClickEvent(QMouseEvent* event)
{
    ensureBlankCursor();
    m_cursorScreenPos = event->pos();
    m_showCrosshairOverlay = rect().contains(event->pos());
    updateViewCubeHover(event->pos());

    if (event->button() == Qt::LeftButton && m_hoveredViewCubeFace != ViewCubeFace::None)
    {
        setCursor(Qt::ArrowCursor);
        m_showCrosshairOverlay = false;
        event->accept();
        return;
    }

    if (event->button() == Qt::LeftButton && handleProcessOrderLabelDoubleClick(event->pos()))
    {
        event->accept();
        updateHoveredWorldPosition(event->pos());
        refreshCommandPrompt();
        update();
        return;
    }

    QOpenGLWidget::mouseDoubleClickEvent(event);
}

void CadViewer::mouseMoveEvent(QMouseEvent* event)
{
    ensureBlankCursor();
    m_cursorScreenPos = event->pos();
    m_showCrosshairOverlay = rect().contains(event->pos());
    updateViewCubeHover(event->pos());

    if (m_hoveredViewCubeFace != ViewCubeFace::None)
    {
        setCursor(Qt::ArrowCursor);
        m_showCrosshairOverlay = false;
        resetOverlappedHandleHoverState();
        event->accept();
        update();
        return;
    }

    if (!m_controller.handleMouseMove(event))
    {
        QOpenGLWidget::mouseMoveEvent(event);
    }

    if (interactionMode() == ViewInteractionMode::Idle)
    {
        updateHoveredWorldPosition(event->pos());
        updateOverlappedHandleHoverState(event->pos());
    }
    else
    {
        resetOverlappedHandleHoverState();
    }

    update();
}

void CadViewer::leaveEvent(QEvent* event)
{
    m_showCrosshairOverlay = false;
    m_hoveredViewCubeFace = ViewCubeFace::None;
    resetOverlappedHandleHoverState();
    update();
    QOpenGLWidget::leaveEvent(event);
}

void CadViewer::enterEvent(QEnterEvent* event)
{
    ensureBlankCursor();
    m_showCrosshairOverlay = true;
    QOpenGLWidget::enterEvent(event);
    update();
}

void CadViewer::focusInEvent(QFocusEvent* event)
{
    ensureBlankCursor();
    QOpenGLWidget::focusInEvent(event);
}

void CadViewer::mouseReleaseEvent(QMouseEvent* event)
{
    ensureBlankCursor();
    m_cursorScreenPos = event->pos();
    m_showCrosshairOverlay = rect().contains(event->pos());

    if (!m_controller.handleMouseRelease(event))
    {
        QOpenGLWidget::mouseReleaseEvent(event);
    }

    updateHoveredWorldPosition(event->pos());
    updateOverlappedHandleHoverState(event->pos());
    refreshCommandPrompt();
    update();
}

void CadViewer::wheelEvent(QWheelEvent* event)
{
    ensureBlankCursor();
    if (!m_controller.handleWheel(event))
    {
        QOpenGLWidget::wheelEvent(event);
    }

    updateHoveredWorldPosition(event->position().toPoint());
}

void CadViewer::keyPressEvent(QKeyEvent* event)
{
    ensureBlankCursor();
    const bool dynamicCommandOverlayVisible = m_controller.dynamicCommandOverlayState().visible;
    const bool orthoEnabledBefore = m_controller.orthoEnabled();
    const bool polarTrackingEnabledBefore = m_controller.polarTrackingEnabled();

    if (!dynamicCommandOverlayVisible
        && event->key() == Qt::Key_Tab
        && isPopupCycleModifierValid(event->modifiers()))
    {
        const int step = (event->modifiers() & Qt::ShiftModifier) != 0 ? -1 : 1;

        if (cycleOverlappedHandleCandidate(step))
        {
            event->accept();
            return;
        }
    }

    if (!m_controller.handleKeyPress(event))
    {
        QOpenGLWidget::keyPressEvent(event);
    }

    if (m_controller.orthoEnabled() != orthoEnabledBefore)
    {
        emit orthoEnabledChanged(m_controller.orthoEnabled());
        updateHoveredWorldPosition(m_cursorScreenPos);
    }

    if (m_controller.polarTrackingEnabled() != polarTrackingEnabledBefore)
    {
        emit polarTrackingEnabledChanged(m_controller.polarTrackingEnabled());
        updateHoveredWorldPosition(m_cursorScreenPos);
    }

    updateOverlappedHandleHoverState(m_cursorScreenPos);
    refreshCommandPrompt();
    update();
}

void CadViewer::keyReleaseEvent(QKeyEvent* event)
{
    ensureBlankCursor();

    if (!m_controller.handleKeyRelease(event))
    {
        QOpenGLWidget::keyReleaseEvent(event);
    }

    updateOverlappedHandleHoverState(m_cursorScreenPos);
    refreshCommandPrompt();
    update();
}

bool CadViewer::focusNextPrevChild(bool next)
{
    Q_UNUSED(next);
    return false;
}

void CadViewer::dragEnterEvent(QDragEnterEvent* event)
{
    const QMimeData* mimeData = event->mimeData();

    if (mimeData == nullptr || !mimeData->hasUrls())
    {
        event->ignore();
        return;
    }

    for (const QUrl& url : mimeData->urls())
    {
        const QString localFile = url.toLocalFile();

        if (isSupportedDropFile(localFile))
        {
            event->acceptProposedAction();
            return;
        }
    }

    event->ignore();
}

void CadViewer::dropEvent(QDropEvent* event)
{
    const QMimeData* mimeData = event->mimeData();

    if (mimeData == nullptr || !mimeData->hasUrls())
    {
        event->ignore();
        return;
    }

    for (const QUrl& url : mimeData->urls())
    {
        const QString localFile = url.toLocalFile();

        if (isSupportedDropFile(localFile))
        {
            emit fileDropRequested(localFile);
            event->acceptProposedAction();
            return;
        }
    }

    event->ignore();
}
