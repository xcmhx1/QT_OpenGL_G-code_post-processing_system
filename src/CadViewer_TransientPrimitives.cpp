// CadViewer 临时图元构建实现
#include "pch.h"

#include "CadViewer.h"

#include "CadDocument.h"
#include "CadItem.h"
#include "CadPreviewBuilder.h"
#include "CadProcessVisualUtils.h"

#include <QPointF>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <utility>
#include <vector>

namespace
{
    QVector3D buildPerpendicularDirection(const QVector3D& direction)
    {
        QVector3D normalizedDirection = direction;

        if (normalizedDirection.lengthSquared() <= 1.0e-6f)
        {
            return QVector3D(1.0f, 0.0f, 0.0f);
        }

        normalizedDirection.normalize();

        QVector3D referenceAxis =
            std::abs(QVector3D::dotProduct(normalizedDirection, QVector3D(0.0f, 0.0f, 1.0f))) < 0.95f
            ? QVector3D(0.0f, 0.0f, 1.0f)
            : QVector3D(1.0f, 0.0f, 0.0f);

        QVector3D perpendicular = QVector3D::crossProduct(normalizedDirection, referenceAxis);

        if (perpendicular.lengthSquared() <= 1.0e-6f)
        {
            referenceAxis = QVector3D(0.0f, 1.0f, 0.0f);
            perpendicular = QVector3D::crossProduct(normalizedDirection, referenceAxis);
        }

        if (perpendicular.lengthSquared() <= 1.0e-6f)
        {
            return QVector3D(1.0f, 0.0f, 0.0f);
        }

        perpendicular.normalize();
        return perpendicular;
    }

    QVector<QVector3D> buildTriangleVertices(const QVector3D& center, QVector3D direction, float size)
    {
        if (direction.lengthSquared() <= 1.0e-6f)
        {
            direction = QVector3D(0.0f, 1.0f, 0.0f);
        }
        else
        {
            direction.normalize();
        }

        const QVector3D perpendicular = buildPerpendicularDirection(direction);

        const QVector3D tip = center + direction * size;
        const QVector3D baseCenter = center - direction * (size * 0.8f);
        const QVector3D left = baseCenter + perpendicular * (size * 0.7f);
        const QVector3D right = baseCenter - perpendicular * (size * 0.7f);
        return { tip, left, right };
    }

    struct RotaryEndCutOverlayPoints
    {
        QVector<QVector3D> points;
        RotaryEndCutRole role = RotaryEndCutRole::None;
    };

    double cross2D(const QPointF& origin, const QPointF& first, const QPointF& second)
    {
        return (first.x() - origin.x()) * (second.y() - origin.y())
            - (first.y() - origin.y()) * (second.x() - origin.x());
    }

    QVector<QPointF> buildConvexHull(QVector<QPointF> points)
    {
        std::sort
        (
            points.begin(),
            points.end(),
            [](const QPointF& left, const QPointF& right)
            {
                return left.x() == right.x() ? left.y() < right.y() : left.x() < right.x();
            }
        );
        points.erase
        (
            std::unique
            (
                points.begin(),
                points.end(),
                [](const QPointF& left, const QPointF& right)
                {
                    return std::abs(left.x() - right.x()) <= 1.0e-6
                        && std::abs(left.y() - right.y()) <= 1.0e-6;
                }
            ),
            points.end()
        );

        if (points.size() < 3)
        {
            return {};
        }

        QVector<QPointF> hull;

        for (const QPointF& point : points)
        {
            while (hull.size() >= 2
                && cross2D(hull[hull.size() - 2], hull.back(), point) <= 0.0)
            {
                hull.removeLast();
            }

            hull.push_back(point);
        }

        const int lowerCount = hull.size();

        for (int index = points.size() - 2; index >= 0; --index)
        {
            const QPointF& point = points[index];

            while (hull.size() > lowerCount
                && cross2D(hull[hull.size() - 2], hull.back(), point) <= 0.0)
            {
                hull.removeLast();
            }

            hull.push_back(point);
        }

        hull.removeLast();
        return hull;
    }

    bool clipSegmentToConvexHull
    (
        const QPointF& start,
        const QPointF& end,
        const QVector<QPointF>& hull,
        QPointF& clippedStart,
        QPointF& clippedEnd
    )
    {
        if (hull.size() < 3)
        {
            return false;
        }

        const QPointF direction = end - start;
        double minimumT = 0.0;
        double maximumT = 1.0;

        for (int index = 0; index < hull.size(); ++index)
        {
            const QPointF& edgeStart = hull[index];
            const QPointF& edgeEnd = hull[(index + 1) % hull.size()];
            const double numerator = cross2D(edgeStart, edgeEnd, start);
            const double denominator = (edgeEnd.x() - edgeStart.x()) * direction.y()
                - (edgeEnd.y() - edgeStart.y()) * direction.x();

            if (std::abs(denominator) <= 1.0e-9)
            {
                if (numerator < 0.0)
                {
                    return false;
                }

                continue;
            }

            const double edgeT = -numerator / denominator;

            if (denominator > 0.0)
            {
                minimumT = std::max(minimumT, edgeT);
            }
            else
            {
                maximumT = std::min(maximumT, edgeT);
            }

            if (minimumT > maximumT)
            {
                return false;
            }
        }

        clippedStart = start + direction * minimumT;
        clippedEnd = start + direction * maximumT;
        return true;
    }

    bool buildRotaryEndCutPlane
    (
        const QVector<QVector3D>& points,
        QVector3D& origin,
        QVector3D& axisU,
        QVector3D& axisV,
        float& minU,
        float& maxU,
        float& minV,
        float& maxV,
        QVector<QPointF>& projectedHull
    )
    {
        if (points.size() < 3)
        {
            return false;
        }

        origin = QVector3D();

        QVector<QPointF> projectedPoints;
        projectedPoints.reserve(points.size());

        for (const QVector3D& point : points)
        {
            origin += point;
        }

        origin /= static_cast<float>(points.size());
        QVector3D firstDirection;

        for (const QVector3D& point : points)
        {
            const QVector3D direction = point - origin;

            if (direction.lengthSquared() > firstDirection.lengthSquared())
            {
                firstDirection = direction;
            }
        }

        if (firstDirection.lengthSquared() <= 1.0e-6f)
        {
            return false;
        }

        QVector3D normal;

        for (const QVector3D& point : points)
        {
            const QVector3D candidateNormal = QVector3D::crossProduct(firstDirection, point - origin);

            if (candidateNormal.lengthSquared() > normal.lengthSquared())
            {
                normal = candidateNormal;
            }
        }

        if (normal.lengthSquared() <= 1.0e-6f)
        {
            return false;
        }

        normal.normalize();
        axisV = QVector3D(0.0f, 0.0f, 1.0f)
            - normal * QVector3D::dotProduct(QVector3D(0.0f, 0.0f, 1.0f), normal);

        if (axisV.lengthSquared() <= 1.0e-6f)
        {
            axisV = QVector3D(0.0f, 1.0f, 0.0f)
                - normal * QVector3D::dotProduct(QVector3D(0.0f, 1.0f, 0.0f), normal);
        }

        if (axisV.lengthSquared() <= 1.0e-6f)
        {
            return false;
        }

        axisV.normalize();
        axisU = QVector3D::crossProduct(axisV, normal).normalized();

        if (QVector3D::dotProduct(axisU, QVector3D(0.0f, 1.0f, 0.0f)) < 0.0f)
        {
            axisU = -axisU;
        }
        minU = maxU = QVector3D::dotProduct(points.front() - origin, axisU);
        minV = maxV = QVector3D::dotProduct(points.front() - origin, axisV);

        for (const QVector3D& point : points)
        {
            const QVector3D offset = point - origin;
            const float projectedU = QVector3D::dotProduct(offset, axisU);
            const float projectedV = QVector3D::dotProduct(offset, axisV);
            minU = std::min(minU, projectedU);
            maxU = std::max(maxU, projectedU);
            minV = std::min(minV, projectedV);
            maxV = std::max(maxV, projectedV);
            projectedPoints.push_back(QPointF(projectedU, projectedV));
        }

        const float size = std::max(maxU - minU, maxV - minV);

        if (size <= 1.0e-4f)
        {
            return false;
        }

        projectedHull = buildConvexHull(std::move(projectedPoints));
        return projectedHull.size() >= 3;
    }

    QVector3D planePoint
    (
        const QVector3D& origin,
        const QVector3D& axisU,
        const QVector3D& axisV,
        float u,
        float v,
        const QVector3D& offset
    )
    {
        return origin + axisU * u + axisV * v + offset;
    }
}

std::vector<TransientPrimitive> CadViewer::buildTransientPrimitives() const
{
    return CadPreviewBuilder::buildTransientPrimitives(m_controller.drawState(), selectedEntity(), selectedEntities());
}

std::vector<TransientPrimitive> CadViewer::buildProcessDirectionPrimitives() const
{
    if (!m_processVisualsVisible || !m_processDirectionVisible)
    {
        return {};
    }

    CadDocument* scene = m_sceneCoordinator.document();

    if (scene == nullptr)
    {
        return {};
    }

    std::vector<TransientPrimitive> primitives;
    primitives.reserve(scene->m_entities.size());

    const float pixelScale = std::max(pixelToWorldScale(), 1.0e-4f);
    const float headLength = pixelScale * 13.0f;
    const float headHalfWidth = pixelScale * 5.0f;
    const QVector3D viewForward = m_camera.forwardDirection().lengthSquared() > 1.0e-6f
        ? m_camera.forwardDirection().normalized()
        : QVector3D(0.0f, 0.0f, -1.0f);

    for (const std::unique_ptr<CadItem>& entity : scene->m_entities)
    {
        if (entity == nullptr)
        {
            continue;
        }

        const CadProcessVisualInfo info = buildProcessVisualInfo(entity.get());

        if (!info.valid || info.direction.lengthSquared() <= 1.0e-6f)
        {
            continue;
        }

        QVector3D perpendicular = QVector3D::crossProduct(viewForward, info.direction);

        if (perpendicular.lengthSquared() <= 1.0e-6f)
        {
            perpendicular = QVector3D(-info.direction.y(), info.direction.x(), 0.0f);
        }

        if (perpendicular.lengthSquared() <= 1.0e-6f)
        {
            perpendicular = buildPerpendicularDirection(info.direction);
        }

        if (perpendicular.lengthSquared() > 1.0e-6f)
        {
            perpendicular.normalize();
        }

        if (perpendicular.lengthSquared() <= 1.0e-6f)
        {
            continue;
        }

        const QVector3D tip = info.startPoint;
        const QVector3D headBase = tip - info.direction * headLength;
        const QVector3D notch = tip - info.direction * (headLength * 0.72f);

        const QPoint tipScreen = worldToScreen(tip);
        const QPoint headBaseScreen = worldToScreen(headBase);
        const double projectedLength = std::hypot
        (
            static_cast<double>(tipScreen.x() - headBaseScreen.x()),
            static_cast<double>(tipScreen.y() - headBaseScreen.y())
        );

        if (projectedLength < 3.0)
        {
            continue;
        }

        QVector3D arrowColor;
        if (entity->m_isSelected)
        {
            arrowColor = QVector3D(1.0f, 0.78f, 0.30f);
        }
        else if (info.processOrder >= 0)
        {
            arrowColor = info.isReverse ? QVector3D(1.0f, 0.42f, 0.28f) : QVector3D(0.12f, 0.92f, 0.72f);
        }
        else
        {
            arrowColor = info.isReverse ? QVector3D(0.82f, 0.34f, 0.26f) : QVector3D(0.34f, 0.78f, 0.58f);
        }

        TransientPrimitive headPrimitive;
        headPrimitive.primitiveType = GL_TRIANGLES;
        headPrimitive.color = arrowColor;
        headPrimitive.vertices =
        {
            tip,
            headBase + perpendicular * headHalfWidth,
            notch,

            tip,
            notch,
            headBase - perpendicular * headHalfWidth
        };
        primitives.push_back(std::move(headPrimitive));
    }

    return primitives;
}

std::vector<TransientPrimitive> CadViewer::buildRotaryEndCutPrimitives() const
{
    if (!m_processVisualsVisible || !m_rotaryEndCutsVisible)
    {
        return {};
    }

    CadDocument* scene = m_sceneCoordinator.document();

    if (scene == nullptr)
    {
        return {};
    }

    std::map<int, RotaryEndCutOverlayPoints> overlays;

    for (const std::unique_ptr<CadItem>& entity : scene->m_entities)
    {
        if (entity == nullptr
            || entity->m_rotaryEndCutRole == RotaryEndCutRole::None
            || entity->m_rotaryEndCutPairId < 0)
        {
            continue;
        }

        const int key = entity->m_rotaryEndCutPairId * 4 + static_cast<int>(entity->m_rotaryEndCutRole);
        RotaryEndCutOverlayPoints& overlay = overlays[key];
        overlay.role = entity->m_rotaryEndCutRole;
        overlay.points += entity->m_geometry.vertices;
    }

    std::vector<TransientPrimitive> primitives;
    const QVector3D viewOffset = m_camera.forwardDirection().lengthSquared() > 1.0e-6f
        ? -m_camera.forwardDirection().normalized() * std::max(pixelToWorldScale() * 1.5f, 0.02f)
        : QVector3D();

    for (const auto& [key, overlay] : overlays)
    {
        Q_UNUSED(key);
        QVector3D origin;
        QVector3D axisU;
        QVector3D axisV;
        float minU = 0.0f;
        float maxU = 0.0f;
        float minV = 0.0f;
        float maxV = 0.0f;
        QVector<QPointF> projectedHull;

        if (!buildRotaryEndCutPlane(overlay.points, origin, axisU, axisV, minU, maxU, minV, maxV, projectedHull))
        {
            continue;
        }

        const QVector3D color = overlay.role == RotaryEndCutRole::Waste
            ? QVector3D(1.0f, 0.60f, 0.12f)
            : QVector3D(0.18f, 0.68f, 1.0f);

        TransientPrimitive lines;
        lines.primitiveType = GL_LINES;
        lines.color = color;
        lines.opacity = overlay.role == RotaryEndCutRole::Waste
            ? 0.82f
            : 0.68f;
        const QPoint extentStart = worldToScreen(planePoint(origin, axisU, axisV, minU, minV, viewOffset));
        const QPoint extentEnd = worldToScreen(planePoint(origin, axisU, axisV, maxU, maxV, viewOffset));
        const double projectedExtent = std::hypot
        (
            static_cast<double>(extentEnd.x() - extentStart.x()),
            static_cast<double>(extentEnd.y() - extentStart.y())
        );
        const int gridCount = std::clamp(static_cast<int>(std::round(projectedExtent / 70.0)), 4, 10);

        const auto appendClippedLine =
            [&lines, &origin, &axisU, &axisV, &viewOffset, &projectedHull](const QPointF& start, const QPointF& end)
            {
                QPointF clippedStart;
                QPointF clippedEnd;

                if (!clipSegmentToConvexHull(start, end, projectedHull, clippedStart, clippedEnd))
                {
                    return;
                }

                lines.vertices +=
                {
                    planePoint(origin, axisU, axisV, static_cast<float>(clippedStart.x()), static_cast<float>(clippedStart.y()), viewOffset),
                    planePoint(origin, axisU, axisV, static_cast<float>(clippedEnd.x()), static_cast<float>(clippedEnd.y()), viewOffset)
                };
            };

        for (int index = 1; index < gridCount; ++index)
        {
            const float t = static_cast<float>(index) / static_cast<float>(gridCount);
            const float u = minU + (maxU - minU) * t;
            const float v = minV + (maxV - minV) * t;
            const float reverseU = maxU - (maxU - minU) * t;
            const float reverseV = maxV - (maxV - minV) * t;
            // 两组相反斜率的线构成倾斜十字网格，并裁剪到实际切面轮廓内。
            appendClippedLine(QPointF(minU, v), QPointF(u, maxV));
            appendClippedLine(QPointF(u, minV), QPointF(maxU, v));
            appendClippedLine(QPointF(minU, v), QPointF(reverseU, minV));
            appendClippedLine(QPointF(u, maxV), QPointF(maxU, reverseV));
        }

        if (!lines.vertices.isEmpty())
        {
            primitives.push_back(std::move(lines));
        }
    }

    return primitives;
}

std::vector<TransientPrimitive> CadViewer::buildSelectedEntityHandlePrimitives() const
{
    const CadItem* selectedItem = selectedEntity();

    if (selectedItem == nullptr)
    {
        return {};
    }

    const QVector<CadSelectionHandleInfo> handles = buildSelectionHandleInfo(selectedItem, xlineHandleWorldLength());

    if (handles.isEmpty())
    {
        return {};
    }

    const int hoveredHandleIndex = resolveHoveredHandleIndex(handles);

    TransientPrimitive basePointOuterPrimitive;
    basePointOuterPrimitive.primitiveType = GL_POINTS;
    basePointOuterPrimitive.color =
    {
        static_cast<float>(m_theme.selectedBasePointColor.redF()),
        static_cast<float>(m_theme.selectedBasePointColor.greenF()),
        static_cast<float>(m_theme.selectedBasePointColor.blueF())
    };
    basePointOuterPrimitive.pointSize = 13.0f;
    basePointOuterPrimitive.roundPoint = true;

    TransientPrimitive basePointInnerPrimitive;
    basePointInnerPrimitive.primitiveType = GL_POINTS;
    basePointInnerPrimitive.color =
    {
        static_cast<float>(m_theme.viewerBackgroundColor.redF()),
        static_cast<float>(m_theme.viewerBackgroundColor.greenF()),
        static_cast<float>(m_theme.viewerBackgroundColor.blueF())
    };
    basePointInnerPrimitive.pointSize = 6.2f;
    basePointInnerPrimitive.roundPoint = true;

    TransientPrimitive controlPointOuterPrimitive;
    controlPointOuterPrimitive.primitiveType = GL_POINTS;
    controlPointOuterPrimitive.color =
    {
        static_cast<float>(m_theme.selectedControlPointColor.redF()),
        static_cast<float>(m_theme.selectedControlPointColor.greenF()),
        static_cast<float>(m_theme.selectedControlPointColor.blueF())
    };
    controlPointOuterPrimitive.pointSize = 10.0f;
    controlPointOuterPrimitive.roundPoint = true;

    TransientPrimitive controlPointInnerPrimitive;
    controlPointInnerPrimitive.primitiveType = GL_POINTS;
    controlPointInnerPrimitive.color =
    {
        static_cast<float>(m_theme.viewerBackgroundColor.redF()),
        static_cast<float>(m_theme.viewerBackgroundColor.greenF()),
        static_cast<float>(m_theme.viewerBackgroundColor.blueF())
    };
    controlPointInnerPrimitive.pointSize = 4.4f;
    controlPointInnerPrimitive.roundPoint = true;

    TransientPrimitive trianglePrimitive;
    trianglePrimitive.primitiveType = GL_TRIANGLES;
    trianglePrimitive.color = controlPointOuterPrimitive.color;

    TransientPrimitive hoveredPointOuterPrimitive;
    hoveredPointOuterPrimitive.primitiveType = GL_POINTS;
    hoveredPointOuterPrimitive.color =
    {
        static_cast<float>(m_theme.accentColor.redF()),
        static_cast<float>(m_theme.accentColor.greenF()),
        static_cast<float>(m_theme.accentColor.blueF())
    };
    hoveredPointOuterPrimitive.pointSize = 17.0f;
    hoveredPointOuterPrimitive.roundPoint = true;

    TransientPrimitive hoveredPointInnerPrimitive;
    hoveredPointInnerPrimitive.primitiveType = GL_POINTS;
    hoveredPointInnerPrimitive.color = { 1.0f, 1.0f, 1.0f };
    hoveredPointInnerPrimitive.pointSize = 9.0f;
    hoveredPointInnerPrimitive.roundPoint = true;

    TransientPrimitive hoveredTrianglePrimitive;
    hoveredTrianglePrimitive.primitiveType = GL_TRIANGLES;
    hoveredTrianglePrimitive.color = hoveredPointOuterPrimitive.color;

    const float pixelScale = std::max(pixelToWorldScale(), 1.0e-4f);
    const float triangleSize = pixelScale * 6.5f;
    const float hoveredTriangleSize = pixelScale * 8.4f;

    for (int handleIndex = 0; handleIndex < handles.size(); ++handleIndex)
    {
        const CadSelectionHandleInfo& handle = handles.at(handleIndex);

        if (handle.shape == CadSelectionHandleShape::Triangle)
        {
            const QVector<QVector3D> triangleVertices = buildTriangleVertices(handle.position, handle.direction, triangleSize);
            trianglePrimitive.vertices += triangleVertices;

            if (handleIndex == hoveredHandleIndex)
            {
                const QVector<QVector3D> hoveredTriangleVertices = buildTriangleVertices
                (
                    handle.position,
                    handle.direction,
                    hoveredTriangleSize
                );
                hoveredTrianglePrimitive.vertices += hoveredTriangleVertices;
            }
        }
        else if (handle.isBasePoint)
        {
            basePointOuterPrimitive.vertices.push_back(handle.position);
            basePointInnerPrimitive.vertices.push_back(handle.position);
        }
        else
        {
            controlPointOuterPrimitive.vertices.push_back(handle.position);
            controlPointInnerPrimitive.vertices.push_back(handle.position);
        }

        if (handleIndex == hoveredHandleIndex)
        {
            hoveredPointOuterPrimitive.vertices.push_back(handle.position);
            hoveredPointInnerPrimitive.vertices.push_back(handle.position);
        }
    }

    std::vector<TransientPrimitive> primitives;

    if (!hoveredPointOuterPrimitive.vertices.isEmpty())
    {
        primitives.push_back(std::move(hoveredPointOuterPrimitive));
    }

    if (!controlPointOuterPrimitive.vertices.isEmpty())
    {
        primitives.push_back(std::move(controlPointOuterPrimitive));
    }

    if (!controlPointInnerPrimitive.vertices.isEmpty())
    {
        primitives.push_back(std::move(controlPointInnerPrimitive));
    }

    if (!hoveredPointInnerPrimitive.vertices.isEmpty())
    {
        primitives.push_back(std::move(hoveredPointInnerPrimitive));
    }

    if (!trianglePrimitive.vertices.isEmpty())
    {
        primitives.push_back(std::move(trianglePrimitive));
    }

    if (!hoveredTrianglePrimitive.vertices.isEmpty())
    {
        primitives.push_back(std::move(hoveredTrianglePrimitive));
    }

    if (!basePointOuterPrimitive.vertices.isEmpty())
    {
        primitives.push_back(std::move(basePointOuterPrimitive));
    }

    if (!basePointInnerPrimitive.vertices.isEmpty())
    {
        primitives.push_back(std::move(basePointInnerPrimitive));
    }

    return primitives;
}
