// CadViewer 加工顺序标签实现
#include "pch.h"

#include "CadViewer.h"

#include "CadDocument.h"
#include "CadEditer.h"
#include "CadItem.h"
#include "CadProcessVisualUtils.h"
#include "CadViewerUtils.h"

#include <QFontMetrics>
#include <QPainter>
#include <QPen>

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

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

        const QColor fillColor = pendingSwap
            ? m_theme.selectedProcessLabelFillColor
            : (label.selected ? m_theme.selectedProcessLabelFillColor : m_theme.processLabelFillColor);
        const QColor borderColor = pendingSwap
            ? m_theme.selectedProcessLabelBorderColor
            : (label.selected ? m_theme.selectedProcessLabelBorderColor : m_theme.processLabelBorderColor);
        const QColor textColor = pendingSwap
            ? m_theme.selectedProcessLabelTextColor
            : (label.selected ? m_theme.selectedProcessLabelTextColor : m_theme.processLabelTextColor);

        painter.setPen(QPen(borderColor, pendingSwap ? 1.4 : 1.0));
        painter.setBrush(fillColor);
        painter.drawRoundedRect(label.bubbleRect, 6.0, 6.0);
        painter.setPen(textColor);
        painter.drawText(label.bubbleRect, Qt::AlignCenter, label.text);
    }
}

std::vector<CadViewer::ProcessOrderLabelOverlay> CadViewer::buildProcessOrderLabelOverlays() const
{
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

        const CadProcessVisualInfo info = buildProcessVisualInfo(entity.get());

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

    if (!hitTestProcessOrderLabel(screenPos, &clickedLabel) || clickedLabel.item == nullptr || m_editer == nullptr)
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

    if (m_pendingProcessOrderSwapEntityId == 0 || m_pendingProcessOrderSwapEntityId == clickedId)
    {
        m_pendingProcessOrderSwapEntityId = clickedId;
        appendCommandMessage(QStringLiteral("已选中加工顺序 %1，点击另一个顺序框可交换两者顺序。").arg(clickedLabel.order + 1));
        return true;
    }

    CadItem* firstItem = findEntityById(m_pendingProcessOrderSwapEntityId);
    CadItem* secondItem = clickedLabel.item;

    if (firstItem == nullptr || secondItem == nullptr)
    {
        m_pendingProcessOrderSwapEntityId = 0;
        return true;
    }

    std::vector<CadEditer::ProcessStateUpdate> updates;
    updates.push_back({ firstItem, secondItem->m_processOrder, firstItem->m_isReverse, firstItem->m_hasCustomProcessStart, firstItem->m_processStartParameter });
    updates.push_back({ secondItem, firstItem->m_processOrder, secondItem->m_isReverse, secondItem->m_hasCustomProcessStart, secondItem->m_processStartParameter });

    const bool swapped = m_editer->applyEntityProcessStates(updates);
    m_pendingProcessOrderSwapEntityId = 0;

    if (swapped)
    {
        appendCommandMessage
        (
            QStringLiteral("已交换加工顺序 %1 与 %2。")
            .arg(firstItem->m_processOrder + 1)
            .arg(secondItem->m_processOrder + 1)
        );
    }
    else
    {
        appendCommandMessage(QStringLiteral("交换加工顺序失败。"));
    }

    return true;
}

bool CadViewer::handleProcessOrderLabelDoubleClick(const QPoint& screenPos)
{
    ProcessOrderLabelOverlay clickedLabel;

    if (!hitTestProcessOrderLabel(screenPos, &clickedLabel) || clickedLabel.item == nullptr || m_editer == nullptr)
    {
        return false;
    }

    const EntityId clickedId = CadViewerUtils::toEntityId(clickedLabel.item);
    setSelectedEntityId(clickedId);
    m_pendingProcessOrderSwapEntityId = 0;

    if (m_editer->toggleEntityReverse(clickedLabel.item))
    {
        appendCommandMessage(QStringLiteral("已切换图元 %1 的加工方向。").arg(clickedLabel.order + 1));
    }
    else
    {
        appendCommandMessage(QStringLiteral("切换加工方向失败。"));
    }

    return true;
}
