// CadViewer 交互 overlay 与重叠夹点实现
#include "pch.h"

#include "CadViewer.h"

#include "CadItem.h"
#include "CadProcessVisualUtils.h"
#include "CadViewerUtils.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include <QFontMetrics>
#include <QPainter>
#include <QPen>

namespace
{
    constexpr qint64 kOverlappedHandlePopupDelayMs = 1000;
    constexpr int kOverlappedHandleStablePixels = 4;
    constexpr float kOverlappedHandlePickScale = 1.25f;
    constexpr float kObjectSnapDistancePixels = 14.0f;

    struct HandlePickCandidate
    {
        int handleIndex = -1;
        float distanceSquared = std::numeric_limits<float>::max();
        int priority = 1;
    };

    struct OverlappedHandlePopupLayout
    {
        QRect panelRect;
        QVector<QRect> rowRects;
        QVector<QString> rowTexts;
        QRect badgeRect;
    };

    QVector<int> collectEditableHandleCandidates
    (
        const QVector<CadSelectionHandleInfo>& handles,
        const std::function<QPoint(const QVector3D&)>& worldToScreen,
        const QPoint& cursorScreenPos,
        float pickDistanceSquared
    )
    {
        QVector<HandlePickCandidate> candidates;

        for (int index = 0; index < handles.size(); ++index)
        {
            const CadSelectionHandleInfo& handle = handles.at(index);

            if (!handle.editable)
            {
                continue;
            }

            const QPoint handleScreenPos = worldToScreen(handle.position);
            const float dx = static_cast<float>(handleScreenPos.x() - cursorScreenPos.x());
            const float dy = static_cast<float>(handleScreenPos.y() - cursorScreenPos.y());
            const float distanceSquared = dx * dx + dy * dy;

            if (distanceSquared > pickDistanceSquared)
            {
                continue;
            }

            HandlePickCandidate candidate;
            candidate.handleIndex = index;
            candidate.distanceSquared = distanceSquared;
            candidate.priority = handle.isBasePoint ? 0 : 1;
            candidates.push_back(candidate);
        }

        std::sort
        (
            candidates.begin(),
            candidates.end(),
            [](const HandlePickCandidate& left, const HandlePickCandidate& right)
            {
                if (std::abs(left.distanceSquared - right.distanceSquared) > 1.0e-4f)
                {
                    return left.distanceSquared < right.distanceSquared;
                }

                if (left.priority != right.priority)
                {
                    return left.priority < right.priority;
                }

                return left.handleIndex < right.handleIndex;
            }
        );

        QVector<int> orderedIndices;
        orderedIndices.reserve(candidates.size());

        for (const HandlePickCandidate& candidate : candidates)
        {
            orderedIndices.push_back(candidate.handleIndex);
        }

        return orderedIndices;
    }

    QString handleTypeDisplayName(const CadItem* selectedItem, const CadSelectionHandleInfo& handle)
    {
        if (handle.isBasePoint)
        {
            return QStringLiteral("基点");
        }

        if (selectedItem == nullptr)
        {
            return QStringLiteral("拉伸点");
        }

        switch (selectedItem->m_type)
        {
        case DRW::ETYPE::POINT:
            return QStringLiteral("点位控制点");
        case DRW::ETYPE::LINE:
            return QStringLiteral("拉伸点");
        case DRW::ETYPE::XLINE:
            return handle.pointIndex == 1 ? QStringLiteral("方向控制点") : QStringLiteral("拉伸点");
        case DRW::ETYPE::CIRCLE:
            return QStringLiteral("半径控制点");
        case DRW::ETYPE::ARC:
            if (handle.pointIndex == 1)
            {
                return QStringLiteral("起点拉伸点");
            }

            if (handle.pointIndex == 2)
            {
                return QStringLiteral("半径控制点");
            }

            if (handle.pointIndex == 3)
            {
                return QStringLiteral("终点拉伸点");
            }

            return QStringLiteral("拉伸点");
        case DRW::ETYPE::ELLIPSE:
            if (handle.pointIndex == 1 || handle.pointIndex == 2)
            {
                return QStringLiteral("长轴控制点");
            }

            if (handle.pointIndex == 3 || handle.pointIndex == 4)
            {
                return QStringLiteral("短轴控制点");
            }

            if (handle.pointIndex == 5)
            {
                return QStringLiteral("起点拉伸点");
            }

            if (handle.pointIndex == 6)
            {
                return QStringLiteral("终点拉伸点");
            }

            return QStringLiteral("拉伸点");
        case DRW::ETYPE::POLYLINE:
        case DRW::ETYPE::LWPOLYLINE:
            return QStringLiteral("顶点拉伸点");
        default:
            return QStringLiteral("拉伸点");
        }
    }

    QString overlappedHandleRowText
    (
        const CadItem* selectedItem,
        const CadSelectionHandleInfo& handle,
        int ordinal
    )
    {
        return QStringLiteral("%1. %2")
            .arg(ordinal + 1)
            .arg(handleTypeDisplayName(selectedItem, handle));
    }

    bool computeOverlappedHandlePopupLayout
    (
        const CadItem* selectedItem,
        const QVector<CadSelectionHandleInfo>& handles,
        const QVector<int>& candidateIndices,
        int activeCandidateOrdinal,
        const QPoint& cursorScreenPos,
        const QSize& viewportSize,
        const QFontMetrics& metrics,
        OverlappedHandlePopupLayout& layout
    )
    {
        if (candidateIndices.size() < 2 || viewportSize.width() <= 0 || viewportSize.height() <= 0)
        {
            return false;
        }

        layout.rowTexts.clear();
        layout.rowTexts.reserve(candidateIndices.size());
        int maxTextWidth = 0;

        for (int ordinal = 0; ordinal < candidateIndices.size(); ++ordinal)
        {
            const int handleIndex = candidateIndices.at(ordinal);

            if (handleIndex < 0 || handleIndex >= handles.size())
            {
                continue;
            }

            const CadSelectionHandleInfo& handle = handles.at(handleIndex);
            const QString rowText = overlappedHandleRowText(selectedItem, handle, ordinal);
            layout.rowTexts.push_back(rowText);
            maxTextWidth = std::max(maxTextWidth, metrics.horizontalAdvance(rowText));
        }

        if (layout.rowTexts.size() < 2)
        {
            return false;
        }

        const int panelPaddingX = 14;
        const int panelPaddingY = 10;
        const int rowHeight = std::max(22, metrics.height() + 8);
        const int desiredPanelWidth = maxTextWidth + panelPaddingX * 2 + 24;
        const int maxPanelWidth = std::max(140, viewportSize.width() - 16);
        const int panelWidth = std::min(desiredPanelWidth, maxPanelWidth);
        const int panelHeight = panelPaddingY * 2 + rowHeight * layout.rowTexts.size();
        QPoint panelTopLeft = cursorScreenPos + QPoint(18, 16);

        if (panelTopLeft.x() + panelWidth > viewportSize.width() - 8)
        {
            panelTopLeft.setX(std::max(8, viewportSize.width() - panelWidth - 8));
        }

        if (panelTopLeft.y() + panelHeight > viewportSize.height() - 8)
        {
            panelTopLeft.setY(std::max(8, viewportSize.height() - panelHeight - 8));
        }

        layout.panelRect = QRect(panelTopLeft, QSize(panelWidth, panelHeight));
        layout.rowRects.clear();
        layout.rowRects.reserve(layout.rowTexts.size());

        for (int ordinal = 0; ordinal < layout.rowTexts.size(); ++ordinal)
        {
            const QRect rowRect
            (
                panelTopLeft.x() + panelPaddingX,
                panelTopLeft.y() + panelPaddingY + ordinal * rowHeight,
                panelWidth - panelPaddingX * 2,
                rowHeight
            );
            layout.rowRects.push_back(rowRect);
        }

        const int badgeRadius = 9;
        const QPoint badgeCenter = cursorScreenPos + QPoint(13, -13);
        layout.badgeRect = QRect
        (
            badgeCenter.x() - badgeRadius,
            badgeCenter.y() - badgeRadius,
            badgeRadius * 2,
            badgeRadius * 2
        );

        Q_UNUSED(activeCandidateOrdinal);
        return true;
    }
}

void CadViewer::resetOverlappedHandleHoverState()
{
    m_overlappedHandlePopupTimer.stop();
    m_overlappedHandleHoverState.entityId = 0;
    m_overlappedHandleHoverState.candidateIndices.clear();
    m_overlappedHandleHoverState.activeCandidateOrdinal = 0;
    m_overlappedHandleHoverState.anchorScreenPos = QPoint();
    m_overlappedHandleHoverState.popupVisible = false;
    m_overlappedHandleHoverState.hoverTimer.invalidate();
}

void CadViewer::updateOverlappedHandleHoverState(const QPoint& screenPos)
{
    if (interactionMode() != ViewInteractionMode::Idle
        || m_controller.drawState().hasActiveCommand()
        || m_selectionWindowPreview.visible
        || (m_controller.drawState().pressedButtons & Qt::LeftButton) != 0)
    {
        resetOverlappedHandleHoverState();
        return;
    }

    CadItem* selectedItem = selectedEntity();

    if (selectedItem == nullptr)
    {
        resetOverlappedHandleHoverState();
        return;
    }

    const QVector<CadSelectionHandleInfo> handles = buildSelectionHandleInfo(selectedItem);

    if (handles.isEmpty())
    {
        resetOverlappedHandleHoverState();
        return;
    }

    const float pickDistanceSquared = (kObjectSnapDistancePixels * kOverlappedHandlePickScale)
        * (kObjectSnapDistancePixels * kOverlappedHandlePickScale);
    const QVector<int> candidateIndices = collectEditableHandleCandidates
    (
        handles,
        [this](const QVector3D& worldPosition)
        {
            return worldToScreen(worldPosition);
        },
        screenPos,
        pickDistanceSquared
    );

    if (candidateIndices.size() < 2)
    {
        resetOverlappedHandleHoverState();
        return;
    }

    const EntityId entityId = CadViewerUtils::toEntityId(selectedItem);
    const bool sameEntity = m_overlappedHandleHoverState.entityId == entityId;
    const bool sameCandidates = m_overlappedHandleHoverState.candidateIndices == candidateIndices;
    const bool stableCursor = sameEntity
        && sameCandidates
        && (screenPos - m_overlappedHandleHoverState.anchorScreenPos).manhattanLength() <= kOverlappedHandleStablePixels;

    if (!stableCursor)
    {
        m_overlappedHandleHoverState.entityId = entityId;
        m_overlappedHandleHoverState.candidateIndices = candidateIndices;
        m_overlappedHandleHoverState.activeCandidateOrdinal = 0;
        m_overlappedHandleHoverState.anchorScreenPos = screenPos;
        m_overlappedHandleHoverState.popupVisible = false;
        m_overlappedHandleHoverState.hoverTimer.restart();
        m_overlappedHandlePopupTimer.start(static_cast<int>(kOverlappedHandlePopupDelayMs));
        return;
    }

    if (!m_overlappedHandleHoverState.popupVisible
        && m_overlappedHandleHoverState.hoverTimer.isValid()
        && m_overlappedHandleHoverState.hoverTimer.elapsed() >= kOverlappedHandlePopupDelayMs)
    {
        m_overlappedHandlePopupTimer.stop();
        m_overlappedHandleHoverState.popupVisible = true;
    }
    else if (!m_overlappedHandleHoverState.popupVisible && !m_overlappedHandlePopupTimer.isActive())
    {
        const qint64 elapsedMs = m_overlappedHandleHoverState.hoverTimer.isValid()
            ? m_overlappedHandleHoverState.hoverTimer.elapsed()
            : 0;
        const qint64 remainingMs = std::max<qint64>(1, kOverlappedHandlePopupDelayMs - elapsedMs);
        m_overlappedHandlePopupTimer.start(static_cast<int>(remainingMs));
    }

    m_overlappedHandleHoverState.activeCandidateOrdinal = std::clamp
    (
        m_overlappedHandleHoverState.activeCandidateOrdinal,
        0,
        static_cast<int>(m_overlappedHandleHoverState.candidateIndices.size()) - 1
    );
}

int CadViewer::resolveHoveredHandleIndex(const QVector<CadSelectionHandleInfo>& handles) const
{
    if (handles.isEmpty())
    {
        return -1;
    }

    const float pickDistanceSquared = (kObjectSnapDistancePixels * kOverlappedHandlePickScale)
        * (kObjectSnapDistancePixels * kOverlappedHandlePickScale);
    const QVector<int> candidateIndices = collectEditableHandleCandidates
    (
        handles,
        [this](const QVector3D& worldPosition)
        {
            return worldToScreen(worldPosition);
        },
        m_cursorScreenPos,
        pickDistanceSquared
    );

    if (candidateIndices.isEmpty())
    {
        return -1;
    }

    const CadItem* selectedItem = selectedEntity();
    const EntityId selectedEntityId = selectedItem != nullptr ? CadViewerUtils::toEntityId(selectedItem) : 0;

    if (candidateIndices.size() >= 2
        && m_overlappedHandleHoverState.entityId == selectedEntityId
        && m_overlappedHandleHoverState.candidateIndices == candidateIndices)
    {
        int resolvedOrdinal = std::clamp
        (
            m_overlappedHandleHoverState.activeCandidateOrdinal,
            0,
            static_cast<int>(candidateIndices.size()) - 1
        );
        return candidateIndices.at(resolvedOrdinal);
    }

    return candidateIndices.front();
}

bool CadViewer::handleOverlappedHandlePopupPress(const QPoint& screenPos)
{
    if (!m_overlappedHandleHoverState.popupVisible || m_overlappedHandleHoverState.candidateIndices.size() < 2)
    {
        return false;
    }

    const CadItem* selectedItem = selectedEntity();

    if (selectedItem == nullptr
        || CadViewerUtils::toEntityId(selectedItem) != m_overlappedHandleHoverState.entityId)
    {
        return false;
    }

    const QVector<CadSelectionHandleInfo> handles = buildSelectionHandleInfo(selectedItem);

    if (handles.isEmpty())
    {
        return false;
    }

    QFont popupFont = font();
    popupFont.setPointSize(9);
    const QFontMetrics metrics(popupFont);
    OverlappedHandlePopupLayout popupLayout;

    if (!computeOverlappedHandlePopupLayout
    (
        selectedItem,
        handles,
        m_overlappedHandleHoverState.candidateIndices,
        m_overlappedHandleHoverState.activeCandidateOrdinal,
        m_cursorScreenPos,
        size(),
        metrics,
        popupLayout
    ))
    {
        return false;
    }

    for (int ordinal = 0; ordinal < popupLayout.rowRects.size(); ++ordinal)
    {
        if (!popupLayout.rowRects.at(ordinal).contains(screenPos))
        {
            continue;
        }

        m_overlappedHandleHoverState.activeCandidateOrdinal = ordinal;
        m_overlappedHandleHoverState.popupVisible = true;
        m_overlappedHandleHoverState.anchorScreenPos = m_cursorScreenPos;
        m_overlappedHandleHoverState.hoverTimer.restart();
        m_overlappedHandlePopupTimer.stop();
        update();
        return true;
    }

    if (popupLayout.panelRect.contains(screenPos))
    {
        return true;
    }

    return false;
}

bool CadViewer::cycleOverlappedHandleCandidate(int step)
{
    if (m_overlappedHandleHoverState.candidateIndices.size() < 2 || step == 0)
    {
        return false;
    }

    const CadItem* selectedItem = selectedEntity();

    if (selectedItem == nullptr
        || CadViewerUtils::toEntityId(selectedItem) != m_overlappedHandleHoverState.entityId)
    {
        return false;
    }

    const int candidateCount = m_overlappedHandleHoverState.candidateIndices.size();
    int ordinal = m_overlappedHandleHoverState.activeCandidateOrdinal % candidateCount;

    if (ordinal < 0)
    {
        ordinal += candidateCount;
    }

    ordinal = (ordinal + step) % candidateCount;

    if (ordinal < 0)
    {
        ordinal += candidateCount;
    }

    m_overlappedHandleHoverState.activeCandidateOrdinal = ordinal;
    m_overlappedHandleHoverState.popupVisible = true;
    m_overlappedHandleHoverState.anchorScreenPos = m_cursorScreenPos;
    m_overlappedHandleHoverState.hoverTimer.restart();
    m_overlappedHandlePopupTimer.stop();
    update();
    return true;
}

void CadViewer::renderOverlappedHandlePopup()
{
    if (interactionMode() != ViewInteractionMode::Idle)
    {
        return;
    }

    const CadItem* selectedItem = selectedEntity();

    if (selectedItem == nullptr
        || CadViewerUtils::toEntityId(selectedItem) != m_overlappedHandleHoverState.entityId
        || m_overlappedHandleHoverState.candidateIndices.size() < 2)
    {
        return;
    }

    const QVector<CadSelectionHandleInfo> handles = buildSelectionHandleInfo(selectedItem);

    if (handles.isEmpty())
    {
        return;
    }

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::TextAntialiasing, true);

    QFont popupFont = painter.font();
    popupFont.setPointSize(9);
    painter.setFont(popupFont);
    const QFontMetrics metrics(popupFont);

    OverlappedHandlePopupLayout popupLayout;

    if (!computeOverlappedHandlePopupLayout
    (
        selectedItem,
        handles,
        m_overlappedHandleHoverState.candidateIndices,
        m_overlappedHandleHoverState.activeCandidateOrdinal,
        m_cursorScreenPos,
        size(),
        metrics,
        popupLayout
    ))
    {
        return;
    }

    painter.setPen(QPen(QColor(15, 22, 30, 210), 1.0));
    painter.setBrush(QColor(255, 186, 54, 235));
    painter.drawEllipse(popupLayout.badgeRect);
    painter.setPen(QColor(18, 24, 33));
    painter.drawText(popupLayout.badgeRect, Qt::AlignCenter, QString::number(m_overlappedHandleHoverState.candidateIndices.size()));

    if (!m_overlappedHandleHoverState.popupVisible)
    {
        return;
    }

    painter.setPen(QPen(QColor(94, 176, 255, 230), 1.0));
    painter.setBrush(QColor(20, 26, 34, 228));
    painter.drawRoundedRect(popupLayout.panelRect, 7.0, 7.0);

    const int rowCount = popupLayout.rowRects.size();
    const int activeOrdinal = std::clamp(m_overlappedHandleHoverState.activeCandidateOrdinal, 0, rowCount - 1);

    for (int ordinal = 0; ordinal < rowCount; ++ordinal)
    {
        const QRect rowRect = popupLayout.rowRects.at(ordinal);
        const QString rowText = ordinal < popupLayout.rowTexts.size()
            ? popupLayout.rowTexts.at(ordinal)
            : QStringLiteral("%1. 拉伸点").arg(ordinal + 1);
        const QString displayText = metrics.elidedText(rowText, Qt::ElideRight, std::max(20, rowRect.width() - 8));

        if (ordinal == activeOrdinal)
        {
            painter.setPen(Qt::NoPen);
            painter.setBrush(QColor(84, 166, 255, 96));
            painter.drawRoundedRect(rowRect.adjusted(0, 0, 0, -1), 4.0, 4.0);
        }

        painter.setPen(ordinal == activeOrdinal ? QColor(255, 248, 227) : QColor(220, 230, 242));
        painter.drawText(rowRect.adjusted(4, 0, -4, 0), Qt::AlignVCenter | Qt::AlignLeft, displayText);
    }

    const QString hintText = QStringLiteral("Tab/Shift+Tab 切换");
    const int hintTop = popupLayout.panelRect.bottom() + 5;
    QRect hintRect
    (
        popupLayout.panelRect.left(),
        hintTop,
        popupLayout.panelRect.width(),
        metrics.height() + 2
    );

    if (hintRect.bottom() > height() - 4)
    {
        hintRect.moveTop(popupLayout.panelRect.top() - hintRect.height() - 4);
    }

    painter.setPen(QColor(150, 172, 194, 228));
    painter.drawText(hintRect, Qt::AlignLeft | Qt::AlignVCenter, hintText);
}

void CadViewer::renderDynamicInputOverlay()
{
    if (interactionMode() != ViewInteractionMode::Idle)
    {
        return;
    }

    const CadDynamicInputOverlayState overlayState = m_controller.dynamicInputOverlayState();

    if (!overlayState.visible)
    {
        return;
    }

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::TextAntialiasing, true);

    QFont titleFont = painter.font();
    titleFont.setPointSize(9);
    titleFont.setBold(true);

    QFont valueFont = painter.font();
    valueFont.setPointSize(9);

    const QFontMetrics titleMetrics(titleFont);
    const QFontMetrics valueMetrics(valueFont);
    const int horizontalPadding = 8;
    const int verticalPadding = 6;
    const int rowHeight = valueMetrics.height() + 6;
    const int titleHeight = titleMetrics.height();
    const int hintHeight = valueMetrics.height();
    const int valueColumnWidth = 140;
    const int labelColumnWidth = 58;
    const int panelWidth = horizontalPadding * 2 + labelColumnWidth + valueColumnWidth;
    const int rowCount = overlayState.rows.isEmpty() ? 0 : overlayState.rows.size();
    const int panelHeight = verticalPadding * 2 + titleHeight + 4 + rowCount * rowHeight + 4 + hintHeight;

    QPoint panelTopLeft = m_cursorScreenPos + QPoint(16, 20);

    if (panelTopLeft.x() + panelWidth > width() - 6)
    {
        panelTopLeft.setX(std::max(6, width() - panelWidth - 6));
    }

    if (panelTopLeft.y() + panelHeight > height() - 6)
    {
        panelTopLeft.setY(std::max(6, m_cursorScreenPos.y() - panelHeight - 16));
    }

    const QRect panelRect(panelTopLeft, QSize(panelWidth, panelHeight));
    painter.setPen(QPen(QColor(88, 160, 245, 220), 1.0));
    painter.setBrush(QColor(16, 22, 30, 222));
    painter.drawRoundedRect(panelRect, 7.0, 7.0);

    QRect contentRect = panelRect.adjusted(horizontalPadding, verticalPadding, -horizontalPadding, -verticalPadding);

    painter.setFont(titleFont);
    painter.setPen(QColor(228, 238, 252, 235));
    painter.drawText(QRect(contentRect.left(), contentRect.top(), contentRect.width(), titleHeight), Qt::AlignLeft | Qt::AlignVCenter, overlayState.title);

    int rowTop = contentRect.top() + titleHeight + 4;

    auto drawValueRow = [&](int rowIndex, const QString& labelText, const QString& valueText, bool active)
    {
        QRect rowRect(contentRect.left(), rowTop + rowIndex * rowHeight, contentRect.width(), rowHeight);

        if (active)
        {
            painter.setPen(Qt::NoPen);
            painter.setBrush(QColor(82, 162, 245, 92));
            painter.drawRoundedRect(rowRect.adjusted(0, 0, 0, -1), 4.0, 4.0);
        }

        painter.setFont(valueFont);
        painter.setPen(active ? QColor(246, 251, 255) : QColor(212, 224, 239));
        painter.drawText(QRect(rowRect.left(), rowRect.top(), labelColumnWidth, rowRect.height()), Qt::AlignLeft | Qt::AlignVCenter, labelText);
        painter.drawText(QRect(rowRect.left() + labelColumnWidth, rowRect.top(), rowRect.width() - labelColumnWidth, rowRect.height()), Qt::AlignLeft | Qt::AlignVCenter, valueText);
    };

    if (overlayState.expressionMode)
    {
        const QString expressionText = overlayState.expressionText.trimmed().isEmpty()
            ? QStringLiteral("请输入表达式")
            : overlayState.expressionText;
        drawValueRow(0, QStringLiteral("表达式"), expressionText, true);
    }
    else
    {
        for (int rowIndex = 0; rowIndex < overlayState.rows.size(); ++rowIndex)
        {
            const CadDynamicInputOverlayRow& row = overlayState.rows.at(rowIndex);
            drawValueRow(rowIndex, row.label, row.valueText, row.active);
        }
    }

    const int hintTop = rowTop + rowCount * rowHeight + 4;
    const QString hintText = overlayState.stageHint.trimmed().isEmpty()
        ? QStringLiteral("Tab切换字段，Enter/Space确认")
        : overlayState.stageHint;
    painter.setFont(valueFont);
    painter.setPen(QColor(156, 178, 202, 228));
    painter.drawText(QRect(contentRect.left(), hintTop, contentRect.width(), hintHeight), Qt::AlignLeft | Qt::AlignVCenter, hintText);
}

void CadViewer::renderDynamicCommandOverlay()
{
    if (interactionMode() != ViewInteractionMode::Idle)
    {
        return;
    }

    const CadDynamicCommandOverlayState overlayState = m_controller.dynamicCommandOverlayState();

    if (!overlayState.visible)
    {
        return;
    }

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::TextAntialiasing, true);

    QFont titleFont = painter.font();
    titleFont.setPointSize(9);
    titleFont.setBold(true);

    QFont rowFont = painter.font();
    rowFont.setPointSize(9);

    const QFontMetrics titleMetrics(titleFont);
    const QFontMetrics rowMetrics(rowFont);
    const int horizontalPadding = 8;
    const int verticalPadding = 6;
    const int rowHeight = rowMetrics.height() + 6;
    const int titleHeight = titleMetrics.height();
    const int maxRows = std::min(6, static_cast<int>(overlayState.candidates.size()));
    const int panelWidth = 320;
    const int panelHeight = verticalPadding * 2 + titleHeight + 4 + maxRows * rowHeight + 4 + rowMetrics.height();

    QPoint panelTopLeft = m_cursorScreenPos + QPoint(18, 24);

    if (panelTopLeft.x() + panelWidth > width() - 6)
    {
        panelTopLeft.setX(std::max(6, width() - panelWidth - 6));
    }

    if (panelTopLeft.y() + panelHeight > height() - 6)
    {
        panelTopLeft.setY(std::max(6, m_cursorScreenPos.y() - panelHeight - 16));
    }

    const QRect panelRect(panelTopLeft, QSize(panelWidth, panelHeight));
    painter.setPen(QPen(QColor(90, 168, 252, 220), 1.0));
    painter.setBrush(QColor(18, 24, 34, 228));
    painter.drawRoundedRect(panelRect, 7.0, 7.0);

    const QRect contentRect = panelRect.adjusted(horizontalPadding, verticalPadding, -horizontalPadding, -verticalPadding);
    painter.setFont(titleFont);
    painter.setPen(QColor(238, 245, 255, 236));
    const QString titleText = QStringLiteral("命令: %1").arg(overlayState.inputText);
    painter.drawText(QRect(contentRect.left(), contentRect.top(), contentRect.width(), titleHeight), Qt::AlignLeft | Qt::AlignVCenter, titleText);

    painter.setFont(rowFont);
    const int rowsTop = contentRect.top() + titleHeight + 4;
    const int activeIndex = std::clamp(overlayState.activeCandidateIndex, 0, std::max(0, maxRows - 1));

    for (int row = 0; row < maxRows; ++row)
    {
        const QRect rowRect(contentRect.left(), rowsTop + row * rowHeight, contentRect.width(), rowHeight);
        const bool active = row == activeIndex;
        const QString rowText = overlayState.candidates.at(row);

        if (active)
        {
            painter.setPen(Qt::NoPen);
            painter.setBrush(QColor(82, 162, 246, 92));
            painter.drawRoundedRect(rowRect.adjusted(0, 0, 0, -1), 4.0, 4.0);
        }

        painter.setPen(active ? QColor(252, 253, 255) : QColor(214, 225, 239));
        painter.drawText(rowRect.adjusted(6, 0, -6, 0), Qt::AlignLeft | Qt::AlignVCenter, rowText);
    }

    const int hintTop = rowsTop + maxRows * rowHeight + 4;
    painter.setPen(QColor(156, 178, 202, 228));
    painter.drawText
    (
        QRect(contentRect.left(), hintTop, contentRect.width(), rowMetrics.height()),
        Qt::AlignLeft | Qt::AlignVCenter,
        overlayState.hintText
    );
}

bool CadViewer::pickSelectedHandle(const QPoint& screenPos, CadSelectionHandleInfo* outHandle) const
{
    const CadItem* selectedItem = selectedEntity();

    if (selectedItem == nullptr)
    {
        return false;
    }

    const QVector<CadSelectionHandleInfo> handles = buildSelectionHandleInfo(selectedItem);

    if (handles.isEmpty())
    {
        return false;
    }

    const float handlePickDistanceSquared = kObjectSnapDistancePixels * kObjectSnapDistancePixels;
    const QVector<int> candidateIndices = collectEditableHandleCandidates
    (
        handles,
        [this](const QVector3D& worldPosition)
        {
            return worldToScreen(worldPosition);
        },
        screenPos,
        handlePickDistanceSquared
    );

    if (candidateIndices.isEmpty())
    {
        return false;
    }

    int selectedIndex = candidateIndices.front();

    if (candidateIndices.size() >= 2
        && m_overlappedHandleHoverState.entityId == CadViewerUtils::toEntityId(selectedItem)
        && m_overlappedHandleHoverState.candidateIndices == candidateIndices)
    {
        int resolvedOrdinal = std::clamp
        (
            m_overlappedHandleHoverState.activeCandidateOrdinal,
            0,
            static_cast<int>(candidateIndices.size()) - 1
        );
        selectedIndex = candidateIndices.at(resolvedOrdinal);
    }

    if (outHandle != nullptr)
    {
        *outHandle = handles[selectedIndex];
    }

    return true;
}
