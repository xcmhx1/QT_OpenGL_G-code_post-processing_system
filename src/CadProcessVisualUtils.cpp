#include "pch.h"

#include "CadProcessVisualUtils.h"

#include "CadItem.h"

#include <cmath>

namespace
{
    constexpr double kVisualEpsilon = 1.0e-9;
    constexpr double kTwoPi = 6.28318530717958647692;

    QVector3D normalizeOrZero(QVector3D vector)
    {
        if (vector.lengthSquared() <= kVisualEpsilon)
        {
            return QVector3D();
        }

        vector.normalize();
        return vector;
    }

    QVector3D ellipsePointAt(const DRW_Ellipse* ellipse, double parameter)
    {
        if (ellipse == nullptr)
        {
            return QVector3D();
        }

        const QVector3D center(ellipse->basePoint.x, ellipse->basePoint.y, ellipse->basePoint.z);
        const QVector3D majorAxis(ellipse->secPoint.x, ellipse->secPoint.y, ellipse->secPoint.z);

        if (majorAxis.lengthSquared() <= kVisualEpsilon || ellipse->ratio <= 0.0)
        {
            return QVector3D();
        }

        QVector3D normal(ellipse->extPoint.x, ellipse->extPoint.y, ellipse->extPoint.z);

        if (normal.lengthSquared() <= kVisualEpsilon)
        {
            normal = QVector3D(0.0f, 0.0f, 1.0f);
        }
        else
        {
            normal.normalize();
        }

        QVector3D minorAxis = QVector3D::crossProduct(normal, majorAxis);

        if (minorAxis.lengthSquared() <= kVisualEpsilon)
        {
            return QVector3D();
        }

        minorAxis.normalize();
        minorAxis *= static_cast<float>(majorAxis.length() * ellipse->ratio);

        return center
            + majorAxis * static_cast<float>(std::cos(parameter))
            + minorAxis * static_cast<float>(std::sin(parameter));
    }

    bool tryBuildGeometryAnchor(const CadItem* item, QVector3D& anchor)
    {
        if (item == nullptr || item->m_geometry.vertices.isEmpty())
        {
            return false;
        }

        QVector3D minPoint = item->m_geometry.vertices.front();
        QVector3D maxPoint = minPoint;

        for (const QVector3D& vertex : item->m_geometry.vertices)
        {
            minPoint.setX(std::min(minPoint.x(), vertex.x()));
            minPoint.setY(std::min(minPoint.y(), vertex.y()));
            minPoint.setZ(std::min(minPoint.z(), vertex.z()));
            maxPoint.setX(std::max(maxPoint.x(), vertex.x()));
            maxPoint.setY(std::max(maxPoint.y(), vertex.y()));
            maxPoint.setZ(std::max(maxPoint.z(), vertex.z()));
        }

        anchor = (minPoint + maxPoint) * 0.5f;
        return true;
    }
}

bool isProcessVisualizable(const CadItem* item)
{
    if (item == nullptr)
    {
        return false;
    }

    switch (item->m_type)
    {
    case DRW::ETYPE::LINE:
    case DRW::ETYPE::ARC:
    case DRW::ETYPE::CIRCLE:
    case DRW::ETYPE::ELLIPSE:
    case DRW::ETYPE::POLYLINE:
    case DRW::ETYPE::LWPOLYLINE:
        return true;
    default:
        return false;
    }
}

CadProcessVisualInfo buildProcessVisualInfo(const CadItem* item)
{
    CadProcessVisualInfo info;

    if (item == nullptr)
    {
        return info;
    }

    info.processOrder = item->m_processOrder;
    info.isReverse = item->m_isReverse;

    if (!isProcessVisualizable(item) || item->m_nativeEntity == nullptr)
    {
        return info;
    }

    QVector3D preferredAnchor;
    const bool hasGeometryAnchor = tryBuildGeometryAnchor(item, preferredAnchor);

    switch (item->m_type)
    {
    case DRW::ETYPE::LINE:
    {
        const DRW_Line* line = static_cast<const DRW_Line*>(item->m_nativeEntity);
        info.forwardStartPoint = QVector3D(line->basePoint.x, line->basePoint.y, line->basePoint.z);
        info.forwardEndPoint = QVector3D(line->secPoint.x, line->secPoint.y, line->secPoint.z);
        info.labelAnchor = (info.forwardStartPoint + info.forwardEndPoint) * 0.5f;
        break;
    }
    case DRW::ETYPE::ARC:
    {
        const DRW_Arc* arc = static_cast<const DRW_Arc*>(item->m_nativeEntity);
        const QVector3D center(arc->basePoint.x, arc->basePoint.y, arc->basePoint.z);
        info.forwardStartPoint = QVector3D
        (
            static_cast<float>(center.x() + std::cos(arc->staangle) * arc->radious),
            static_cast<float>(center.y() + std::sin(arc->staangle) * arc->radious),
            center.z()
        );
        info.forwardEndPoint = QVector3D
        (
            static_cast<float>(center.x() + std::cos(arc->endangle) * arc->radious),
            static_cast<float>(center.y() + std::sin(arc->endangle) * arc->radious),
            center.z()
        );
        info.labelAnchor = hasGeometryAnchor ? preferredAnchor : (info.forwardStartPoint + info.forwardEndPoint) * 0.5f;
        break;
    }
    case DRW::ETYPE::CIRCLE:
    {
        const DRW_Circle* circle = static_cast<const DRW_Circle*>(item->m_nativeEntity);
        info.closedPath = true;
        info.forwardStartPoint = QVector3D
        (
            static_cast<float>(circle->basePoint.x + circle->radious),
            static_cast<float>(circle->basePoint.y),
            static_cast<float>(circle->basePoint.z)
        );
        info.forwardEndPoint = info.forwardStartPoint;
        info.labelAnchor = QVector3D
        (
            static_cast<float>(circle->basePoint.x),
            static_cast<float>(circle->basePoint.y),
            static_cast<float>(circle->basePoint.z)
        );
        break;
    }
    case DRW::ETYPE::ELLIPSE:
    {
        const DRW_Ellipse* ellipse = static_cast<const DRW_Ellipse*>(item->m_nativeEntity);
        double startParam = ellipse->staparam;
        double endParam = ellipse->endparam;
        info.closedPath = std::abs(endParam - startParam) < 1.0e-10
            || std::abs(std::abs(endParam - startParam) - kTwoPi) < 1.0e-10;

        if (!info.closedPath)
        {
            while (endParam <= startParam)
            {
                endParam += kTwoPi;
            }
        }
        else
        {
            endParam = startParam;
        }

        info.forwardStartPoint = ellipsePointAt(ellipse, startParam);
        info.forwardEndPoint = ellipsePointAt(ellipse, endParam);
        info.labelAnchor = info.closedPath
            ? QVector3D(ellipse->basePoint.x, ellipse->basePoint.y, ellipse->basePoint.z)
            : (hasGeometryAnchor ? preferredAnchor : (info.forwardStartPoint + info.forwardEndPoint) * 0.5f);
        break;
    }
    case DRW::ETYPE::POLYLINE:
    {
        const DRW_Polyline* polyline = static_cast<const DRW_Polyline*>(item->m_nativeEntity);

        if (polyline->vertlist.empty())
        {
            return info;
        }

        const auto& firstVertex = polyline->vertlist.front();
        const auto& lastVertex = polyline->vertlist.back();
        info.closedPath = (polyline->flags & 1) != 0;
        info.forwardStartPoint = QVector3D(firstVertex->basePoint.x, firstVertex->basePoint.y, firstVertex->basePoint.z);
        info.forwardEndPoint = info.closedPath
            ? info.forwardStartPoint
            : QVector3D(lastVertex->basePoint.x, lastVertex->basePoint.y, lastVertex->basePoint.z);
        info.labelAnchor = hasGeometryAnchor ? preferredAnchor : info.forwardStartPoint;
        break;
    }
    case DRW::ETYPE::LWPOLYLINE:
    {
        const DRW_LWPolyline* polyline = static_cast<const DRW_LWPolyline*>(item->m_nativeEntity);

        if (polyline->vertlist.empty())
        {
            return info;
        }

        const auto& firstVertex = polyline->vertlist.front();
        const auto& lastVertex = polyline->vertlist.back();
        const float z = static_cast<float>(polyline->elevation);
        info.closedPath = (polyline->flags & 1) != 0;
        info.forwardStartPoint = QVector3D(static_cast<float>(firstVertex->x), static_cast<float>(firstVertex->y), z);
        info.forwardEndPoint = info.closedPath
            ? info.forwardStartPoint
            : QVector3D(static_cast<float>(lastVertex->x), static_cast<float>(lastVertex->y), z);
        info.labelAnchor = hasGeometryAnchor ? preferredAnchor : info.forwardStartPoint;
        break;
    }
    default:
        return info;
    }

    info.startPoint = info.isReverse ? info.forwardEndPoint : info.forwardStartPoint;
    info.endPoint = info.isReverse ? info.forwardStartPoint : info.forwardEndPoint;
    info.direction = normalizeOrZero(item->m_processDirection);

    if (info.direction.lengthSquared() <= kVisualEpsilon)
    {
        info.direction = normalizeOrZero(info.endPoint - info.startPoint);
    }

    if (item->m_type == DRW::ETYPE::CIRCLE && info.direction.lengthSquared() <= kVisualEpsilon)
    {
        info.direction = info.isReverse ? QVector3D(0.0f, -1.0f, 0.0f) : QVector3D(0.0f, 1.0f, 0.0f);
    }

    info.valid = true;
    return info;
}
