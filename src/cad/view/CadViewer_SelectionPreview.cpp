// CadViewer 框选预览与选中高亮实现
#include "platform/pch.h"

#include "cad/view/CadViewer.h"

#include "cad/document/CadDocument.h"
#include "cad/view/interaction/CadEntityPicker.h"
#include "cad/items/CadItem.h"
#include "cad/view/CadViewerUtils.h"

#include <QPainter>
#include <QPen>
#include <QPolygonF>

#include <memory>
#include <vector>

namespace
{
    constexpr int kWindowSelectionMinimumPixels = 2;

    QRect normalizedSelectionRect(const QPoint& anchorScreenPos, const QPoint& currentScreenPos)
    {
        return QRect(anchorScreenPos, currentScreenPos).normalized();
    }
}

void CadViewer::updateSelectionWindowPreviewCandidates()
{
    m_windowPreviewEntityIds.clear();

    if (!m_selectionWindowPreview.visible)
    {
        return;
    }

    CadDocument* scene = m_sceneCoordinator.document();

    if (scene == nullptr)
    {
        return;
    }

    const QRect selectionRect = normalizedSelectionRect
    (
        m_selectionWindowPreview.anchorScreenPos,
        m_selectionWindowPreview.currentScreenPos
    );

    if (selectionRect.width() < kWindowSelectionMinimumPixels || selectionRect.height() < kWindowSelectionMinimumPixels)
    {
        return;
    }

    const std::vector<EntityId> previewIds = CadEntityPicker::pickEntitiesByWindow
    (
        scene->m_entities,
        m_camera.viewProjectionMatrix(aspectRatio()),
        m_viewportWidth,
        m_viewportHeight,
        QRectF(selectionRect),
        m_selectionWindowPreview.crossingSelection
    );

    for (EntityId id : previewIds)
    {
        if (id != 0)
        {
            m_windowPreviewEntityIds.insert(id);
        }
    }
}

void CadViewer::renderEntitySelectionOverlays()
{
    CadDocument* scene = m_sceneCoordinator.document();

    if (scene == nullptr || interactionMode() != ViewInteractionMode::Idle)
    {
        return;
    }

    if (m_selectedEntityIds.isEmpty() && m_windowPreviewEntityIds.isEmpty())
    {
        return;
    }

    const QMatrix4x4 viewProjection = m_camera.viewProjectionMatrix(aspectRatio());
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    for (const std::unique_ptr<CadItem>& entity : scene->m_entities)
    {
        if (entity == nullptr)
        {
            continue;
        }

        const EntityId id = CadViewerUtils::toEntityId(entity.get());
        const bool committedSelected = m_selectedEntityIds.contains(id);
        const bool previewSelected = m_windowPreviewEntityIds.contains(id);

        if (!committedSelected && !previewSelected)
        {
            continue;
        }

        const QVector<QVector3D>& worldVertices = entity->m_geometry.vertices;

        if (worldVertices.isEmpty())
        {
            continue;
        }

        QPolygonF projectedPolyline;
        projectedPolyline.reserve(worldVertices.size());
        bool validProjection = true;

        for (const QVector3D& worldVertex : worldVertices)
        {
            const QPointF screenPoint = CadViewerUtils::projectToScreen
            (
                worldVertex,
                viewProjection,
                m_viewportWidth,
                m_viewportHeight
            );

            if (!qIsFinite(screenPoint.x()) || !qIsFinite(screenPoint.y()))
            {
                validProjection = false;
                break;
            }

            projectedPolyline << screenPoint;
        }

        if (!validProjection || projectedPolyline.isEmpty())
        {
            continue;
        }

        const QColor haloColor = committedSelected
            ? QColor(60, 174, 255, 48)
            : (m_theme.dark ? QColor(255, 255, 255, 52) : QColor(42, 78, 104, 48));
        const QColor glowColor = committedSelected
            ? QColor(76, 184, 255, 105)
            : QColor(255, 255, 255, 125);
        const QColor coreColor = committedSelected
            ? QColor(105, 202, 255, 195)
            : QColor(255, 255, 255, 235);

        QPen haloPen(haloColor, 7.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        QPen glowPen(glowColor, 3.4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        QPen corePen(coreColor, 1.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);

        painter.setBrush(Qt::NoBrush);

        if (projectedPolyline.size() == 1)
        {
            const QPointF center = projectedPolyline.front();
            painter.setPen(haloPen);
            painter.drawEllipse(center, 8.0, 8.0);
            painter.setPen(glowPen);
            painter.drawEllipse(center, 6.0, 6.0);
            painter.setPen(corePen);
            painter.drawEllipse(center, 4.0, 4.0);
        }
        else
        {
            painter.setPen(haloPen);
            painter.drawPolyline(projectedPolyline);

            painter.setPen(glowPen);
            painter.drawPolyline(projectedPolyline);

            painter.setPen(corePen);
            painter.drawPolyline(projectedPolyline);
        }
    }
}

void CadViewer::renderSelectionWindowPreview()
{
    if (!m_selectionWindowPreview.visible || interactionMode() != ViewInteractionMode::Idle)
    {
        return;
    }

    const QRect selectionRect = normalizedSelectionRect
    (
        m_selectionWindowPreview.anchorScreenPos,
        m_selectionWindowPreview.currentScreenPos
    );

    if (selectionRect.width() < kWindowSelectionMinimumPixels || selectionRect.height() < kWindowSelectionMinimumPixels)
    {
        return;
    }

    const bool crossingSelection = m_selectionWindowPreview.crossingSelection;
    const QColor borderColor = crossingSelection
        ? QColor(66, 205, 104, 220)
        : QColor(74, 152, 250, 220);
    const QColor fillColor = crossingSelection
        ? QColor(66, 205, 104, 42)
        : QColor(74, 152, 250, 42);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, false);

    QPen borderPen(borderColor, 1.0, Qt::DashLine);
    borderPen.setDashPattern({ 7.0, 4.0 });

    painter.setPen(borderPen);
    painter.setBrush(fillColor);
    painter.drawRect(selectionRect);
}
