// CadViewer 加工顺序标签实现
#include "platform/pch.h"

#include "cad/view/CadViewer.h"

#include "cad/document/CadDocument.h"
#include "cad/editing/CadEditer.h"
#include "cad/items/CadItem.h"
#include "cad/view/rendering/CadProcessVisualUtils.h"
#include "cad/view/CadViewerUtils.h"

#include <QFontMetrics>
#include <QPainter>
#include <QPen>

#include <algorithm>
#include <map>
#include <memory>
#include <utility>
#include <vector>

namespace
{
    QColor withMaximumAlpha(QColor color, int maximumAlpha)
    {
        color.setAlpha(std::min(color.alpha(), maximumAlpha));
        return color;
    }
}

void CadViewer::renderProcessOrderLabels()
{
    const std::vector<ProcessOrderLabelOverlay> labels = buildProcessOrderLabelOverlays();

    if (labels.empty())
    {
        return;
    }

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::TextAntialiasing, true);

    QFont labelFont = painter.font();
    labelFont.setPointSize(9);
    labelFont.setBold(true);
    painter.setFont(labelFont);

    for (const ProcessOrderLabelOverlay& label : labels)
    {
        const bool pendingSwap = label.item != nullptr
            && CadViewerUtils::toEntityId(label.item) == m_pendingProcessOrderSwapEntityId;

        const bool emphasized = pendingSwap || label.selected;
        const QColor baseFillColor = pendingSwap
            ? m_theme.selectedProcessLabelFillColor
            : (label.selected ? m_theme.selectedProcessLabelFillColor : m_theme.processLabelFillColor);
        const QColor baseBorderColor = pendingSwap
            ? m_theme.selectedProcessLabelBorderColor
            : (label.selected ? m_theme.selectedProcessLabelBorderColor : m_theme.processLabelBorderColor);
        const QColor textColor = pendingSwap
            ? m_theme.selectedProcessLabelTextColor
            : (label.selected ? m_theme.selectedProcessLabelTextColor : m_theme.processLabelTextColor);
        const QColor fillColor = withMaximumAlpha(baseFillColor, emphasized ? 148 : 72);
        const QColor borderColor = withMaximumAlpha(baseBorderColor, emphasized ? 220 : 164);

        painter.setPen(QPen(borderColor, pendingSwap ? 1.4 : 1.0));
        painter.setBrush(fillColor);
        painter.drawRoundedRect(label.bubbleRect, 6.0, 6.0);
        painter.setPen(textColor);
        painter.drawText(label.bubbleRect, Qt::AlignCenter, label.text);
    }
}

void CadViewer::renderRotaryEndCutLabels()
{
    if (!m_processVisualsVisible || !m_rotaryEndCutsVisible)
    {
        return;
    }

    CadDocument* scene = m_sceneCoordinator.document();

    if (scene == nullptr || interactionMode() != ViewInteractionMode::Idle)
    {
        return;
    }

    struct CutLabelData
    {
        QVector3D pointSum;
        int pointCount = 0;
        int pairId = -1;
        cadcam::planning::BoundaryRole role = cadcam::planning::BoundaryRole::None;
    };

    std::map<int, CutLabelData> labels;

    for (const std::unique_ptr<CadItem>& entity : scene->m_entities)
    {
        if (entity == nullptr || m_processState == nullptr)
        {
            continue;
        }

        const auto state = m_processState->stateOrDefault(entity->m_entityId);
        if (state.overrideData.boundaryPairId < 0
            || state.overrideData.boundaryRole == cadcam::planning::BoundaryRole::None) continue;

        const int key = state.overrideData.boundaryPairId * 4
            + static_cast<int>(state.overrideData.boundaryRole);
        CutLabelData& label = labels[key];
        label.pairId = state.overrideData.boundaryPairId;
        label.role = state.overrideData.boundaryRole;

        for (const QVector3D& point : entity->m_geometry.vertices)
        {
            label.pointSum += point;
            ++label.pointCount;
        }
    }

    if (labels.empty())
    {
        return;
    }

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::TextAntialiasing, true);
    QFont labelFont = painter.font();
    labelFont.setPointSize(9);
    labelFont.setBold(true);
    painter.setFont(labelFont);
    const QFontMetrics metrics(labelFont);

    for (const auto& [key, label] : labels)
    {
        Q_UNUSED(key);

        if (label.pointCount <= 0)
        {
            continue;
        }

        const QPoint center = worldToScreen(label.pointSum / static_cast<float>(label.pointCount));

        if (center.x() < -30 || center.x() > width() + 30
            || center.y() < -30 || center.y() > height() + 30)
        {
            continue;
        }

        const QString roleText = label.role == cadcam::planning::BoundaryRole::Waste
            ? QStringLiteral("W")
            : QStringLiteral("断");
        const QString text = QStringLiteral("%1%2")
            .arg(roleText)
            .arg(label.pairId + 1);
        const QRect textRect = metrics.boundingRect(text);
        const QRect bubbleRect
        (
            center.x() - textRect.width() / 2 - 8,
            center.y() - textRect.height() / 2 - 5,
            textRect.width() + 16,
            textRect.height() + 10
        );
        const QColor accent = label.role == cadcam::planning::BoundaryRole::Waste
            ? QColor(242, 151, 36)
            : QColor(46, 166, 242);
        const QColor fill = withMaximumAlpha(m_theme.viewerBackgroundColor, 66);
        const QColor border = withMaximumAlpha(accent, 168);
        const QColor textColor = withMaximumAlpha(accent.lighter(125), 224);
        painter.setPen(QPen(border, 1.4));
        painter.setBrush(fill);
        painter.drawRoundedRect(bubbleRect, 7.0, 7.0);
        painter.setPen(textColor);
        painter.drawText(bubbleRect, Qt::AlignCenter, text);
    }
}

std::vector<CadViewer::ProcessOrderLabelOverlay> CadViewer::buildProcessOrderLabelOverlays() const
{
    // Compatibility projection of the current ProcessPlan. Not a planning or NC source of truth.
    if (!m_processVisualsVisible || !m_processOrderVisible)
    {
        return {};
    }

    CadDocument* scene = m_sceneCoordinator.document();

    if (scene == nullptr || interactionMode() != ViewInteractionMode::Idle)
    {
        return {};
    }

    std::vector<ProcessOrderLabelOverlay> labels;
    labels.reserve(scene->m_entities.size());

    QFont labelFont = font();
    labelFont.setPointSize(9);
    labelFont.setBold(true);
    const QFontMetrics metrics(labelFont);
    std::vector<QRect> occupiedRects;
    occupiedRects.reserve(scene->m_entities.size());

    for (const std::unique_ptr<CadItem>& entity : scene->m_entities)
    {
        if (entity == nullptr)
        {
            continue;
        }

        const auto* presentation = m_processPresentation != nullptr
            ? m_processPresentation->find(entity->m_entityId) : nullptr;
        const CadProcessVisualInfo info = buildProcessVisualInfo(entity.get(), presentation);

        if (!info.valid || info.processOrder < 0)
        {
            continue;
        }

        const QPoint screenPoint = worldToScreen(info.labelAnchor);

        if (screenPoint.x() < -24 || screenPoint.x() > width() + 24
            || screenPoint.y() < -24 || screenPoint.y() > height() + 24)
        {
            continue;
        }

        ProcessOrderLabelOverlay label;
        label.item = entity.get();
        label.order = info.processOrder;
        label.selected = entity->m_isSelected;
        label.center = screenPoint;
        label.text = QString::number(info.processOrder + 1);

        const QRect textRect = metrics.boundingRect(label.text);
        label.bubbleRect = QRect
        (
            label.center.x() - textRect.width() / 2 - 7,
            label.center.y() - textRect.height() / 2 - 4,
            textRect.width() + 14,
            textRect.height() + 8
        );

        if (!label.selected)
        {
            bool overlaps = false;

            for (const QRect& occupiedRect : occupiedRects)
            {
                if (label.bubbleRect.adjusted(-2, -2, 2, 2).intersects(occupiedRect))
                {
                    overlaps = true;
                    break;
                }
            }

            if (overlaps)
            {
                continue;
            }
        }

        occupiedRects.push_back(label.bubbleRect);
        labels.push_back(std::move(label));
    }

    std::sort
    (
        labels.begin(),
        labels.end(),
        [](const ProcessOrderLabelOverlay& left, const ProcessOrderLabelOverlay& right)
        {
            if (left.selected != right.selected)
            {
                return left.selected && !right.selected;
            }

            return left.order < right.order;
        }
    );

    return labels;
}

bool CadViewer::hitTestProcessOrderLabel(const QPoint& screenPos, ProcessOrderLabelOverlay* outLabel) const
{
    const std::vector<ProcessOrderLabelOverlay> labels = buildProcessOrderLabelOverlays();

    for (const ProcessOrderLabelOverlay& label : labels)
    {
        if (label.bubbleRect.adjusted(-2, -2, 2, 2).contains(screenPos))
        {
            if (outLabel != nullptr)
            {
                *outLabel = label;
            }

            return true;
        }
    }

    return false;
}

bool CadViewer::handleProcessOrderLabelClick(const QPoint& screenPos)
{
    ProcessOrderLabelOverlay clickedLabel;

    if (!hitTestProcessOrderLabel(screenPos, &clickedLabel) || clickedLabel.item == nullptr)
    {
        if (m_pendingProcessOrderSwapEntityId != 0)
        {
            m_pendingProcessOrderSwapEntityId = 0;
            update();
        }

        return false;
    }

    const EntityId clickedId = CadViewerUtils::toEntityId(clickedLabel.item);
    setSelectedEntityId(clickedId);

    m_pendingProcessOrderSwapEntityId = 0;
    appendCommandMessage(QStringLiteral("已选中加工顺序 %1。").arg(clickedLabel.order + 1));
    return true;
}

bool CadViewer::handleProcessOrderLabelDoubleClick(const QPoint& screenPos)
{
    ProcessOrderLabelOverlay clickedLabel;

    if (!hitTestProcessOrderLabel(screenPos, &clickedLabel) || clickedLabel.item == nullptr)
    {
        return false;
    }

    const EntityId clickedId = CadViewerUtils::toEntityId(clickedLabel.item);
    setSelectedEntityId(clickedId);
    m_pendingProcessOrderSwapEntityId = 0;

    emit processDirectionToggleRequested(clickedId);

    return true;
}
