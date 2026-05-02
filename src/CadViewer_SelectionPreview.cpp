// CadViewer 框选预览与选中高亮实现
#include "pch.h"

#include "CadViewer.h"

#include "CadDocument.h"
#include "CadEntityPicker.h"
#include "CadItem.h"
#include "CadViewerUtils.h"

#include <QPainter>
#include <QPen>
#include <QPolygonF>

#include <algorithm>
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
    const QColor committedColor(0, 188, 255, 228);
    const QColor previewColor = m_selectionWindowPreview.crossingSelection
        ? QColor(66, 205, 104, 220)
        : QColor(74, 152, 250, 220);

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

        const QColor mainColor = committedSelected ? committedColor : previewColor;
        const QColor glowColor = committedSelected
            ? QColor(255, 255, 255, 235)
            : QColor(232, 246, 255, 220);

        QPen glowPen(glowColor, committedSelected ? 2.4 : 2.0, Qt::SolidLine);
        QPen mainPen(mainColor, committedSelected ? 1.5 : 1.2, Qt::DashLine);
        mainPen.setDashPattern(committedSelected ? QList<qreal>{ 7.0, 3.0 } : QList<qreal>{ 5.0, 4.0 });

        painter.setBrush(Qt::NoBrush);

        if (projectedPolyline.size() == 1)
        {
            const QPointF center = projectedPolyline.front();
            painter.setPen(glowPen);
            painter.drawEllipse(center, 5.8, 5.8);
            painter.setPen(mainPen);
            painter.drawEllipse(center, 4.0, 4.0);
        }
        else
        {
            painter.setPen(glowPen);
            painter.drawPolyline(projectedPolyline);

            painter.setPen(mainPen);
            painter.drawPolyline(projectedPolyline);
        }

        const QRectF bounds = projectedPolyline.boundingRect().adjusted(-4.0, -4.0, 4.0, 4.0);
        const qreal maxCornerLength = 10.0;
        const qreal minCornerLength = 4.0;
        const qreal cornerLength = std::clamp
        (
            std::min(bounds.width(), bounds.height()) * 0.28,
            minCornerLength,
            maxCornerLength
        );

        QPen cornerPen(mainColor, committedSelected ? 1.4 : 1.2, Qt::SolidLine);
        painter.setPen(cornerPen);

        const qreal left = bounds.left();
        const qreal right = bounds.right();
        const qreal top = bounds.top();
        const qreal bottom = bounds.bottom();

        painter.drawLine(QPointF(left, top), QPointF(left + cornerLength, top));
        painter.drawLine(QPointF(left, top), QPointF(left, top + cornerLength));
        painter.drawLine(QPointF(right, top), QPointF(right - cornerLength, top));
        painter.drawLine(QPointF(right, top), QPointF(right, top + cornerLength));
        painter.drawLine(QPointF(left, bottom), QPointF(left + cornerLength, bottom));
        painter.drawLine(QPointF(left, bottom), QPointF(left, bottom - cornerLength));
        painter.drawLine(QPointF(right, bottom), QPointF(right - cornerLength, bottom));
        painter.drawLine(QPointF(right, bottom), QPointF(right, bottom - cornerLength));
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
