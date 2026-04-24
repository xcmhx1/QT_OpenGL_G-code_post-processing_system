// CadViewer Qt 事件入口实现
#include "pch.h"

#include "CadViewer.h"

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

    if (!m_controller.handleMousePress(event))
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
