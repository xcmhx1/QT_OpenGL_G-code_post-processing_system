#include "pch.h"

#include "CadViewer.h"

#include "CadDocument.h"
#include "CadItem.h"
#include "CadProcessVisualUtils.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

namespace
{
    constexpr float kObjectSnapDistancePixels = 14.0f;
    constexpr float kGridSnapDistancePixels = 10.0f;
    constexpr qint64 kIntersectionSnapMinIntervalMs = 33;
    constexpr int kIntersectionMaxCandidateSegments = 64;
    constexpr int kIntersectionMaxPairChecks = 2200;
    constexpr float kIntersectionEarlyAcceptDistancePixels = 3.5f;

    struct SnapCandidate
    {
        QVector3D position;
        float distanceSquared = std::numeric_limits<float>::max();
        int priority = 100;
        bool valid = false;
    };

    struct SnapSegmentScreenData
    {
        const CadItem* owner = nullptr;
        QVector3D startWorld;
        QVector3D endWorld;
        QPoint startScreen;
        QPoint endScreen;
        float cursorDistanceSquared = std::numeric_limits<float>::max();
    };

    float cross2D(float ax, float ay, float bx, float by)
    {
        return ax * by - ay * bx;
    }

    float pointToSegmentDistanceSquared(const QPoint& point, const QPoint& start, const QPoint& end)
    {
        const float dx = static_cast<float>(end.x() - start.x());
        const float dy = static_cast<float>(end.y() - start.y());
        const float lengthSquared = dx * dx + dy * dy;

        if (lengthSquared <= 1.0e-6f)
        {
            const float px = static_cast<float>(point.x() - start.x());
            const float py = static_cast<float>(point.y() - start.y());
            return px * px + py * py;
        }

        const float px = static_cast<float>(point.x() - start.x());
        const float py = static_cast<float>(point.y() - start.y());
        const float t = std::clamp((px * dx + py * dy) / lengthSquared, 0.0f, 1.0f);
        const float closestX = static_cast<float>(start.x()) + dx * t;
        const float closestY = static_cast<float>(start.y()) + dy * t;
        const float rx = static_cast<float>(point.x()) - closestX;
        const float ry = static_cast<float>(point.y()) - closestY;
        return rx * rx + ry * ry;
    }

    bool isFullEllipsePath(const DRW_Ellipse* ellipse)
    {
        if (ellipse == nullptr)
        {
            return false;
        }

        constexpr double kTwoPi = 6.28318530717958647692;
        const double span = ellipse->endparam - ellipse->staparam;
        return std::abs(span) < 1.0e-10 || std::abs(std::abs(span) - kTwoPi) < 1.0e-10;
    }

    QVector3D resolveNormal(const DRW_Coord& extPoint)
    {
        QVector3D normal(extPoint.x, extPoint.y, extPoint.z);

        if (normal.lengthSquared() <= 1.0e-9f)
        {
            return QVector3D(0.0f, 0.0f, 1.0f);
        }

        normal.normalize();
        return normal;
    }

    void buildPlaneBasis(const QVector3D& normal, QVector3D& axisX, QVector3D& axisY)
    {
        const QVector3D helper = std::abs(normal.z()) < 0.999f
            ? QVector3D(0.0f, 0.0f, 1.0f)
            : QVector3D(0.0f, 1.0f, 0.0f);

        axisX = QVector3D::crossProduct(helper, normal);

        if (axisX.lengthSquared() <= 1.0e-9f)
        {
            axisX = QVector3D(1.0f, 0.0f, 0.0f);
        }
        else
        {
            axisX.normalize();
        }

        axisY = QVector3D::crossProduct(normal, axisX);

        if (axisY.lengthSquared() <= 1.0e-9f)
        {
            axisY = QVector3D(0.0f, 1.0f, 0.0f);
        }
        else
        {
            axisY.normalize();
        }
    }

    QVector3D circleLikePointAt(const QVector3D& center, double radius, const DRW_Coord& extPoint, double parameter)
    {
        if (radius <= 0.0)
        {
            return center;
        }

        const QVector3D normal = resolveNormal(extPoint);
        QVector3D axisX;
        QVector3D axisY;
        buildPlaneBasis(normal, axisX, axisY);

        return center
            + axisX * static_cast<float>(std::cos(parameter) * radius)
            + axisY * static_cast<float>(std::sin(parameter) * radius);
    }

    bool trySegmentIntersection
    (
        const QVector3D& a0,
        const QVector3D& a1,
        const QVector3D& b0,
        const QVector3D& b1,
        QVector3D& intersection
    )
    {
        const float rX = a1.x() - a0.x();
        const float rY = a1.y() - a0.y();
        const float sX = b1.x() - b0.x();
        const float sY = b1.y() - b0.y();
        const float denominator = cross2D(rX, rY, sX, sY);

        if (std::abs(denominator) <= 1.0e-6f)
        {
            return false;
        }

        const float qpX = b0.x() - a0.x();
        const float qpY = b0.y() - a0.y();
        const float t = cross2D(qpX, qpY, sX, sY) / denominator;
        const float u = cross2D(qpX, qpY, rX, rY) / denominator;
        constexpr float kTolerance = 1.0e-4f;

        if (t < -kTolerance || t > 1.0f + kTolerance || u < -kTolerance || u > 1.0f + kTolerance)
        {
            return false;
        }

        intersection = QVector3D(a0.x() + rX * t, a0.y() + rY * t, 0.0f);
        return true;
    }
}

QVector3D CadViewer::resolveInteractiveWorldPosition(const QPoint& screenPos) const
{
    return applySnapToGroundPosition(screenPos, screenToGroundPlane(screenPos));
}

void CadViewer::updateHoveredWorldPosition(const QPoint& screenPos)
{
    if (m_controller.drawState().hasActiveCommand())
    {
        emit hoveredWorldPositionChanged(m_controller.drawState().currentPos);
        return;
    }

    emit hoveredWorldPositionChanged(resolveInteractiveWorldPosition(screenPos));
}

QVector3D CadViewer::applySnapToGroundPosition
(
    const QPoint& screenPos,
    const QVector3D& worldPos,
    bool* snapped,
    bool* objectSnap
) const
{
    if (snapped != nullptr)
    {
        *snapped = false;
    }

    if (objectSnap != nullptr)
    {
        *objectSnap = false;
    }

    const auto commitCacheAndReturn =
        [this, &screenPos, &worldPos, snapped, objectSnap](const QVector3D& resolvedPosition, bool isSnapped, bool isObjectSnap)
        {
            m_snapResolveCache.valid = true;
            m_snapResolveCache.revision = m_snapContextRevision;
            m_snapResolveCache.screenPos = screenPos;
            m_snapResolveCache.inputWorldPos = worldPos;
            m_snapResolveCache.resolvedWorldPos = resolvedPosition;
            m_snapResolveCache.snapped = isSnapped;
            m_snapResolveCache.objectSnap = isObjectSnap;

            if (snapped != nullptr)
            {
                *snapped = isSnapped;
            }

            if (objectSnap != nullptr)
            {
                *objectSnap = isObjectSnap;
            }

            return resolvedPosition;
        };

    if (m_snapResolveCache.valid
        && m_snapResolveCache.revision == m_snapContextRevision
        && m_snapResolveCache.screenPos == screenPos
        && (m_snapResolveCache.inputWorldPos - worldPos).lengthSquared() <= 1.0e-10f)
    {
        if (snapped != nullptr)
        {
            *snapped = m_snapResolveCache.snapped;
        }

        if (objectSnap != nullptr)
        {
            *objectSnap = m_snapResolveCache.objectSnap;
        }

        return m_snapResolveCache.resolvedWorldPos;
    }

    SnapCandidate bestCandidate;
    const float snapDistanceSquared = kObjectSnapDistancePixels * kObjectSnapDistancePixels;
    const auto tryConsumeCandidate =
        [this, &screenPos, &bestCandidate, snapDistanceSquared](const QVector3D& worldCandidate, int priority)
        {
            const QPoint candidateScreenPos = worldToScreen(worldCandidate);
            const float dx = static_cast<float>(candidateScreenPos.x() - screenPos.x());
            const float dy = static_cast<float>(candidateScreenPos.y() - screenPos.y());
            const float distanceSquared = dx * dx + dy * dy;

            if (distanceSquared > snapDistanceSquared)
            {
                return;
            }

            const bool sameDistance = std::abs(distanceSquared - bestCandidate.distanceSquared) <= 1.0e-4f;

            if (!bestCandidate.valid
                || distanceSquared < bestCandidate.distanceSquared
                || (sameDistance && priority < bestCandidate.priority))
            {
                bestCandidate.position = worldCandidate;
                bestCandidate.distanceSquared = distanceSquared;
                bestCandidate.priority = priority;
                bestCandidate.valid = true;
            }
        };

    const float intersectionEarlyAcceptDistanceSquared =
        kIntersectionEarlyAcceptDistancePixels * kIntersectionEarlyAcceptDistancePixels;

    if (m_basePointSnapEnabled || m_controlPointSnapEnabled)
    {
        CadItem* selectedItem = selectedEntity();

        if (selectedItem != nullptr)
        {
            const QVector<CadSelectionHandleInfo> handles = buildSelectionHandleInfo(selectedItem, xlineHandleWorldLength());

            for (const CadSelectionHandleInfo& handle : handles)
            {
                if ((handle.isBasePoint && !m_basePointSnapEnabled)
                    || (!handle.isBasePoint && !m_controlPointSnapEnabled))
                {
                    continue;
                }

                const int priority = handle.isBasePoint ? 0 : 6;
                tryConsumeCandidate(handle.position, priority);
            }
        }
    }

    const bool advancedObjectSnapEnabled = m_endpointSnapEnabled
        || m_midpointSnapEnabled
        || m_centerSnapEnabled
        || m_intersectionSnapEnabled;

    if (advancedObjectSnapEnabled)
    {
        const CadDocument* scene = m_sceneCoordinator.document();

        if (scene != nullptr)
        {
            QVector<SnapSegmentScreenData> nearCursorSegments;
            const float intersectionCollectDistanceSquared =
                (kObjectSnapDistancePixels * 1.8f) * (kObjectSnapDistancePixels * 1.8f);
            const qint64 nowMs = m_snapComputationTimer.isValid() ? m_snapComputationTimer.elapsed() : 0;
            const bool intersectionIntervalReady = !m_snapComputationTimer.isValid()
                || (nowMs - m_lastIntersectionComputeMs) >= kIntersectionSnapMinIntervalMs;
            const bool strongCandidateAlreadyFound = bestCandidate.valid
                && bestCandidate.distanceSquared <= intersectionEarlyAcceptDistanceSquared;
            const bool allowIntersectionEvaluation = m_intersectionSnapEnabled
                && intersectionIntervalReady
                && !strongCandidateAlreadyFound;

            const auto appendIntersectionSegment =
                [&nearCursorSegments](const SnapSegmentScreenData& segment)
                {
                    if (nearCursorSegments.size() < kIntersectionMaxCandidateSegments)
                    {
                        nearCursorSegments.push_back(segment);
                        return;
                    }

                    int worstIndex = -1;
                    float worstDistance = -1.0f;

                    for (int index = 0; index < nearCursorSegments.size(); ++index)
                    {
                        const float candidateDistance = nearCursorSegments.at(index).cursorDistanceSquared;

                        if (candidateDistance > worstDistance)
                        {
                            worstDistance = candidateDistance;
                            worstIndex = index;
                        }
                    }

                    if (worstIndex >= 0 && segment.cursorDistanceSquared < worstDistance)
                    {
                        nearCursorSegments[worstIndex] = segment;
                    }
                };

            for (const std::unique_ptr<CadItem>& entity : scene->m_entities)
            {
                const CadItem* item = entity.get();

                if (item == nullptr || item->m_nativeEntity == nullptr)
                {
                    continue;
                }

                auto appendEndpoint = [this, &tryConsumeCandidate](const QVector3D& position)
                {
                    tryConsumeCandidate(position, 1);
                };

                auto appendMidpoint = [this, &tryConsumeCandidate](const QVector3D& position)
                {
                    tryConsumeCandidate(position, 4);
                };

                auto appendCenter = [this, &tryConsumeCandidate](const QVector3D& position)
                {
                    tryConsumeCandidate(position, 3);
                };

                if (m_endpointSnapEnabled)
                {
                    switch (item->m_type)
                    {
                    case DRW::ETYPE::POINT:
                    {
                        const DRW_Point* point = static_cast<const DRW_Point*>(item->m_nativeEntity);
                        appendEndpoint(QVector3D(point->basePoint.x, point->basePoint.y, point->basePoint.z));
                        break;
                    }
                    case DRW::ETYPE::LINE:
                    {
                        const DRW_Line* line = static_cast<const DRW_Line*>(item->m_nativeEntity);
                        appendEndpoint(QVector3D(line->basePoint.x, line->basePoint.y, line->basePoint.z));
                        appendEndpoint(QVector3D(line->secPoint.x, line->secPoint.y, line->secPoint.z));
                        break;
                    }
                    case DRW::ETYPE::ARC:
                    {
                        const DRW_Arc* arc = static_cast<const DRW_Arc*>(item->m_nativeEntity);
                        const QVector3D center(arc->basePoint.x, arc->basePoint.y, arc->basePoint.z);
                        appendEndpoint(circleLikePointAt(center, arc->radious, arc->extPoint, arc->staangle));
                        appendEndpoint(circleLikePointAt(center, arc->radious, arc->extPoint, arc->endangle));
                        break;
                    }
                    case DRW::ETYPE::ELLIPSE:
                    {
                        const DRW_Ellipse* ellipse = static_cast<const DRW_Ellipse*>(item->m_nativeEntity);

                        if (!isFullEllipsePath(ellipse))
                        {
                            const QVector3D center(ellipse->basePoint.x, ellipse->basePoint.y, ellipse->basePoint.z);
                            QVector3D majorAxis(ellipse->secPoint.x, ellipse->secPoint.y, ellipse->secPoint.z);
                            QVector3D normal = resolveNormal(ellipse->extPoint);
                            QVector3D minorAxis = QVector3D::crossProduct(normal, majorAxis);

                            if (majorAxis.lengthSquared() > 1.0e-9f
                                && minorAxis.lengthSquared() > 1.0e-9f
                                && ellipse->ratio > 0.0)
                            {
                                minorAxis.normalize();
                                minorAxis *= static_cast<float>(majorAxis.length() * ellipse->ratio);
                                appendEndpoint(center + majorAxis * static_cast<float>(std::cos(ellipse->staparam))
                                    + minorAxis * static_cast<float>(std::sin(ellipse->staparam)));
                                appendEndpoint(center + majorAxis * static_cast<float>(std::cos(ellipse->endparam))
                                    + minorAxis * static_cast<float>(std::sin(ellipse->endparam)));
                            }
                        }
                        break;
                    }
                    case DRW::ETYPE::POLYLINE:
                    {
                        const DRW_Polyline* polyline = static_cast<const DRW_Polyline*>(item->m_nativeEntity);

                        for (const std::shared_ptr<DRW_Vertex>& vertex : polyline->vertlist)
                        {
                            if (vertex != nullptr)
                            {
                                appendEndpoint(QVector3D(vertex->basePoint.x, vertex->basePoint.y, vertex->basePoint.z));
                            }
                        }

                        break;
                    }
                    case DRW::ETYPE::LWPOLYLINE:
                    {
                        const DRW_LWPolyline* polyline = static_cast<const DRW_LWPolyline*>(item->m_nativeEntity);
                        const float z = static_cast<float>(polyline->elevation);

                        for (const std::shared_ptr<DRW_Vertex2D>& vertex : polyline->vertlist)
                        {
                            if (vertex != nullptr)
                            {
                                appendEndpoint(QVector3D(static_cast<float>(vertex->x), static_cast<float>(vertex->y), z));
                            }
                        }

                        break;
                    }
                    default:
                        break;
                    }
                }

                if (m_midpointSnapEnabled)
                {
                    switch (item->m_type)
                    {
                    case DRW::ETYPE::LINE:
                    {
                        const DRW_Line* line = static_cast<const DRW_Line*>(item->m_nativeEntity);
                        const QVector3D start(line->basePoint.x, line->basePoint.y, line->basePoint.z);
                        const QVector3D end(line->secPoint.x, line->secPoint.y, line->secPoint.z);
                        appendMidpoint((start + end) * 0.5f);
                        break;
                    }
                    case DRW::ETYPE::ARC:
                    {
                        const DRW_Arc* arc = static_cast<const DRW_Arc*>(item->m_nativeEntity);
                        double endAngle = arc->endangle;

                        while (endAngle <= arc->staangle)
                        {
                            endAngle += 6.28318530717958647692;
                        }

                        const double midAngle = (arc->staangle + endAngle) * 0.5;
                        const QVector3D center(arc->basePoint.x, arc->basePoint.y, arc->basePoint.z);
                        appendMidpoint(circleLikePointAt(center, arc->radious, arc->extPoint, midAngle));
                        break;
                    }
                    case DRW::ETYPE::POLYLINE:
                    case DRW::ETYPE::LWPOLYLINE:
                    {
                        const QVector<QVector3D>& vertices = item->m_geometry.vertices;

                        for (int index = 0; index + 1 < vertices.size(); ++index)
                        {
                            const QVector3D start = vertices.at(index);
                            const QVector3D end = vertices.at(index + 1);

                            if ((end - start).lengthSquared() <= 1.0e-9f)
                            {
                                continue;
                            }

                            appendMidpoint((start + end) * 0.5f);
                        }

                        break;
                    }
                    default:
                        break;
                    }
                }

                if (m_centerSnapEnabled)
                {
                    switch (item->m_type)
                    {
                    case DRW::ETYPE::CIRCLE:
                    {
                        const DRW_Circle* circle = static_cast<const DRW_Circle*>(item->m_nativeEntity);
                        appendCenter(QVector3D(circle->basePoint.x, circle->basePoint.y, circle->basePoint.z));
                        break;
                    }
                    case DRW::ETYPE::ARC:
                    {
                        const DRW_Arc* arc = static_cast<const DRW_Arc*>(item->m_nativeEntity);
                        appendCenter(QVector3D(arc->basePoint.x, arc->basePoint.y, arc->basePoint.z));
                        break;
                    }
                    case DRW::ETYPE::ELLIPSE:
                    {
                        const DRW_Ellipse* ellipse = static_cast<const DRW_Ellipse*>(item->m_nativeEntity);
                        appendCenter(QVector3D(ellipse->basePoint.x, ellipse->basePoint.y, ellipse->basePoint.z));
                        break;
                    }
                    default:
                        break;
                    }
                }

                if (allowIntersectionEvaluation)
                {
                    const QVector<QVector3D>& vertices = item->m_geometry.vertices;

                    if (vertices.size() >= 2)
                    {
                        for (int index = 0; index + 1 < vertices.size(); ++index)
                        {
                            const QVector3D start = vertices.at(index);
                            const QVector3D end = vertices.at(index + 1);

                            if ((end - start).lengthSquared() <= 1.0e-9f)
                            {
                                continue;
                            }

                            const QPoint startScreen = worldToScreen(start);
                            const QPoint endScreen = worldToScreen(end);
                            const float segmentDistanceSquared = pointToSegmentDistanceSquared(screenPos, startScreen, endScreen);

                            if (segmentDistanceSquared > intersectionCollectDistanceSquared)
                            {
                                continue;
                            }

                            SnapSegmentScreenData segment;
                            segment.owner = item;
                            segment.startWorld = start;
                            segment.endWorld = end;
                            segment.startScreen = startScreen;
                            segment.endScreen = endScreen;
                            segment.cursorDistanceSquared = segmentDistanceSquared;
                            appendIntersectionSegment(segment);
                        }
                    }
                }
            }

            if (allowIntersectionEvaluation && nearCursorSegments.size() >= 2)
            {
                int checkedPairCount = 0;

                for (int left = 0; left < nearCursorSegments.size() - 1; ++left)
                {
                    const SnapSegmentScreenData& first = nearCursorSegments.at(left);
                    bool reachPairLimit = false;

                    for (int right = left + 1; right < nearCursorSegments.size(); ++right)
                    {
                        if (checkedPairCount >= kIntersectionMaxPairChecks)
                        {
                            reachPairLimit = true;
                            break;
                        }

                        ++checkedPairCount;
                        const SnapSegmentScreenData& second = nearCursorSegments.at(right);

                        if (first.owner == second.owner)
                        {
                            continue;
                        }

                        QVector3D intersectionPoint;

                        if (!trySegmentIntersection
                        (
                            first.startWorld,
                            first.endWorld,
                            second.startWorld,
                            second.endWorld,
                            intersectionPoint
                        ))
                        {
                            continue;
                        }

                        tryConsumeCandidate(intersectionPoint, 2);
                    }

                    if (reachPairLimit)
                    {
                        break;
                    }
                }
            }

            if (allowIntersectionEvaluation)
            {
                m_lastIntersectionComputeMs = nowMs;
            }
        }
    }

    if (bestCandidate.valid)
    {
        return commitCacheAndReturn
        (
            QVector3D(bestCandidate.position.x(), bestCandidate.position.y(), 0.0f),
            true,
            true
        );
    }

    if (m_gridSnapEnabled)
    {
        const float gridStep = currentGridStep();
        const float snappedX = std::round(worldPos.x() / gridStep) * gridStep;
        const float snappedY = std::round(worldPos.y() / gridStep) * gridStep;
        const QVector3D snappedGridPosition(snappedX, snappedY, 0.0f);
        const QPoint snappedGridScreenPos = worldToScreen(snappedGridPosition);
        const float dx = static_cast<float>(snappedGridScreenPos.x() - screenPos.x());
        const float dy = static_cast<float>(snappedGridScreenPos.y() - screenPos.y());
        const float distanceSquared = dx * dx + dy * dy;
        const float gridSnapDistanceSquared = kGridSnapDistancePixels * kGridSnapDistancePixels;

        if (distanceSquared <= gridSnapDistanceSquared)
        {
            return commitCacheAndReturn(snappedGridPosition, true, false);
        }
    }

    return commitCacheAndReturn(QVector3D(worldPos.x(), worldPos.y(), 0.0f), false, false);
}

std::vector<TransientPrimitive> CadViewer::buildSnapHighlightPrimitives() const
{
    if (!m_showCrosshairOverlay)
    {
        return {};
    }

    bool snapped = false;
    bool objectSnap = false;
    const QVector3D snappedPosition = applySnapToGroundPosition
    (
        m_cursorScreenPos,
        screenToGroundPlane(m_cursorScreenPos),
        &snapped,
        &objectSnap
    );

    if (!snapped)
    {
        return {};
    }

    const QColor outerColor = objectSnap ? m_theme.selectedControlPointColor : m_theme.accentColor;
    const QColor innerColor = m_theme.viewerBackgroundColor;

    TransientPrimitive outerPoint;
    outerPoint.primitiveType = GL_POINTS;
    outerPoint.color =
    {
        static_cast<float>(outerColor.redF()),
        static_cast<float>(outerColor.greenF()),
        static_cast<float>(outerColor.blueF())
    };
    outerPoint.pointSize = objectSnap ? 15.0f : 13.0f;
    outerPoint.roundPoint = true;
    outerPoint.vertices = { snappedPosition };

    TransientPrimitive innerPoint;
    innerPoint.primitiveType = GL_POINTS;
    innerPoint.color =
    {
        static_cast<float>(innerColor.redF()),
        static_cast<float>(innerColor.greenF()),
        static_cast<float>(innerColor.blueF())
    };
    innerPoint.pointSize = objectSnap ? 7.0f : 5.5f;
    innerPoint.roundPoint = true;
    innerPoint.vertices = { snappedPosition };

    return { outerPoint, innerPoint };
}
