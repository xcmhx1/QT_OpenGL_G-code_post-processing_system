#include "pch.h"

#include "CadProcessVisualUtils.h"

#include "CadItem.h"

#include <cmath>

namespace
{
    constexpr double kVisualEpsilon = 1.0e-9;
    constexpr double kPi = 3.14159265358979323846;
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

    QVector3D leftPerpendicular(const QVector3D& vector)
    {
        return QVector3D(-vector.y(), vector.x(), 0.0f);
    }

    QVector3D bulgeArcCenter(const QVector3D& startPoint, const QVector3D& endPoint, double bulge, bool* valid = nullptr)
    {
        const QVector3D chord = endPoint - startPoint;
        const double chordLength = chord.length();

        if (valid != nullptr)
        {
            *valid = false;
        }

        if (chordLength <= kVisualEpsilon || std::abs(bulge) < 1.0e-8)
        {
            return QVector3D();
        }

        const QVector3D midpoint = (startPoint + endPoint) * 0.5f;
        const double centerOffset = chordLength * (1.0 / bulge - bulge) * 0.25;
        const QVector3D leftNormal
        (
            static_cast<float>(-chord.y() / chordLength),
            static_cast<float>(chord.x() / chordLength),
            0.0f
        );

        if (valid != nullptr)
        {
            *valid = true;
        }

        return midpoint + leftNormal * static_cast<float>(centerOffset);
    }

    QVector3D bulgeSegmentTangentAtStart(const QVector3D& startPoint, const QVector3D& endPoint, double bulge)
    {
        if (std::abs(bulge) < 1.0e-8)
        {
            return normalizeOrZero(endPoint - startPoint);
        }

        bool valid = false;
        const QVector3D center = bulgeArcCenter(startPoint, endPoint, bulge, &valid);

        if (!valid)
        {
            return normalizeOrZero(endPoint - startPoint);
        }

        const QVector3D radiusVector = startPoint - center;
        const QVector3D tangent = bulge > 0.0
            ? leftPerpendicular(radiusVector)
            : -leftPerpendicular(radiusVector);

        return normalizeOrZero(tangent);
    }

    QVector3D bulgeSegmentTangentAtEnd(const QVector3D& startPoint, const QVector3D& endPoint, double bulge)
    {
        if (std::abs(bulge) < 1.0e-8)
        {
            return normalizeOrZero(endPoint - startPoint);
        }

        bool valid = false;
        const QVector3D center = bulgeArcCenter(startPoint, endPoint, bulge, &valid);

        if (!valid)
        {
            return normalizeOrZero(endPoint - startPoint);
        }

        const QVector3D radiusVector = endPoint - center;
        const QVector3D tangent = bulge > 0.0
            ? leftPerpendicular(radiusVector)
            : -leftPerpendicular(radiusVector);

        return normalizeOrZero(tangent);
    }

    bool tryBuildEllipseAxes(const DRW_Ellipse* ellipse, QVector3D& majorAxis, QVector3D& minorAxis)
    {
        if (ellipse == nullptr)
        {
            return false;
        }

        majorAxis = QVector3D(ellipse->secPoint.x, ellipse->secPoint.y, ellipse->secPoint.z);

        if (majorAxis.lengthSquared() <= kVisualEpsilon || ellipse->ratio <= 0.0)
        {
            return false;
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

        minorAxis = QVector3D::crossProduct(normal, majorAxis);

        if (minorAxis.lengthSquared() <= kVisualEpsilon)
        {
            return false;
        }

        minorAxis.normalize();
        minorAxis *= static_cast<float>(majorAxis.length() * ellipse->ratio);
        return true;
    }

    QVector3D ellipsePointAt(const DRW_Ellipse* ellipse, double parameter)
    {
        if (ellipse == nullptr)
        {
            return QVector3D();
        }

        const QVector3D center(ellipse->basePoint.x, ellipse->basePoint.y, ellipse->basePoint.z);
        QVector3D majorAxis;
        QVector3D minorAxis;

        if (!tryBuildEllipseAxes(ellipse, majorAxis, minorAxis))
        {
            return QVector3D();
        }

        return center
            + majorAxis * static_cast<float>(std::cos(parameter))
            + minorAxis * static_cast<float>(std::sin(parameter));
    }

    double normalizeAnglePositive(double angle)
    {
        double normalized = std::fmod(angle, kTwoPi);

        if (normalized < 0.0)
        {
            normalized += kTwoPi;
        }

        return normalized;
    }

    bool isFullEllipsePath(const DRW_Ellipse* ellipse)
    {
        if (ellipse == nullptr)
        {
            return false;
        }

        const double span = ellipse->endparam - ellipse->staparam;
        return std::abs(span) < 1.0e-10
            || std::abs(std::abs(span) - kTwoPi) < 1.0e-10;
    }

    double effectiveCircleStartParameter(const CadItem* item)
    {
        if (item != nullptr && item->m_hasCustomProcessStart)
        {
            return normalizeAnglePositive(item->m_processStartParameter);
        }

        return kPi * 0.5;
    }

    double effectiveClosedEllipseStartParameter(const CadItem* item, const DRW_Ellipse* ellipse)
    {
        if (item != nullptr && item->m_hasCustomProcessStart)
        {
            return item->m_processStartParameter;
        }

        return ellipse != nullptr ? ellipse->staparam : 0.0;
    }

    QVector3D ellipseTangentAt(const DRW_Ellipse* ellipse, double parameter, bool reverseDirection)
    {
        QVector3D majorAxis;
        QVector3D minorAxis;

        if (!tryBuildEllipseAxes(ellipse, majorAxis, minorAxis))
        {
            return QVector3D();
        }

        QVector3D tangent
        (
            static_cast<float>(-std::sin(parameter)) * majorAxis
            + static_cast<float>(std::cos(parameter)) * minorAxis
        );

        if (reverseDirection)
        {
            tangent = -tangent;
        }

        return normalizeOrZero(tangent);
    }

    QVector3D arcTangentAt(double angle, bool reverseDirection)
    {
        QVector3D tangent
        (
            static_cast<float>(-std::sin(angle)),
            static_cast<float>(std::cos(angle)),
            0.0f
        );

        if (reverseDirection)
        {
            tangent = -tangent;
        }

        return normalizeOrZero(tangent);
    }

    QVector3D polylineForwardStartTangent(const DRW_Polyline* polyline)
    {
        if (polyline == nullptr || polyline->vertlist.size() < 2)
        {
            return QVector3D();
        }

        for (size_t index = 0; index + 1 < polyline->vertlist.size(); ++index)
        {
            const auto& current = polyline->vertlist.at(index);
            const auto& next = polyline->vertlist.at(index + 1);
            const QVector3D startPoint(current->basePoint.x, current->basePoint.y, current->basePoint.z);
            const QVector3D endPoint(next->basePoint.x, next->basePoint.y, next->basePoint.z);
            const QVector3D tangent = bulgeSegmentTangentAtStart(startPoint, endPoint, current->bulge);

            if (tangent.lengthSquared() > kVisualEpsilon)
            {
                return tangent;
            }
        }

        if ((polyline->flags & 1) != 0)
        {
            const auto& current = polyline->vertlist.back();
            const auto& next = polyline->vertlist.front();
            const QVector3D startPoint(current->basePoint.x, current->basePoint.y, current->basePoint.z);
            const QVector3D endPoint(next->basePoint.x, next->basePoint.y, next->basePoint.z);
            return bulgeSegmentTangentAtStart(startPoint, endPoint, current->bulge);
        }

        return QVector3D();
    }

    QVector3D polylineForwardStartTangentAt(const DRW_Polyline* polyline, size_t startIndex)
    {
        if (polyline == nullptr || polyline->vertlist.size() < 2)
        {
            return QVector3D();
        }

        const size_t count = polyline->vertlist.size();
        const size_t nextIndex = (startIndex + 1) % count;
        const auto& current = polyline->vertlist.at(startIndex);
        const auto& next = polyline->vertlist.at(nextIndex);
        const QVector3D startPoint(current->basePoint.x, current->basePoint.y, current->basePoint.z);
        const QVector3D endPoint(next->basePoint.x, next->basePoint.y, next->basePoint.z);
        return bulgeSegmentTangentAtStart(startPoint, endPoint, current->bulge);
    }

    QVector3D polylineReverseStartTangent(const DRW_Polyline* polyline)
    {
        if (polyline == nullptr || polyline->vertlist.size() < 2)
        {
            return QVector3D();
        }

        for (size_t index = polyline->vertlist.size() - 1; index > 0; --index)
        {
            const auto& current = polyline->vertlist.at(index);
            const auto& next = polyline->vertlist.at(index - 1);
            const QVector3D startPoint(current->basePoint.x, current->basePoint.y, current->basePoint.z);
            const QVector3D endPoint(next->basePoint.x, next->basePoint.y, next->basePoint.z);
            const QVector3D tangent = bulgeSegmentTangentAtStart(startPoint, endPoint, -next->bulge);

            if (tangent.lengthSquared() > kVisualEpsilon)
            {
                return tangent;
            }
        }

        if ((polyline->flags & 1) != 0)
        {
            const auto& current = polyline->vertlist.front();
            const auto& next = polyline->vertlist.back();
            const QVector3D startPoint(current->basePoint.x, current->basePoint.y, current->basePoint.z);
            const QVector3D endPoint(next->basePoint.x, next->basePoint.y, next->basePoint.z);
            return bulgeSegmentTangentAtStart(startPoint, endPoint, -next->bulge);
        }

        return QVector3D();
    }

    QVector3D polylineForwardEndTangentAt(const DRW_Polyline* polyline, size_t startIndex)
    {
        if (polyline == nullptr || polyline->vertlist.size() < 2)
        {
            return QVector3D();
        }

        const size_t count = polyline->vertlist.size();
        const size_t previousIndex = (startIndex + count - 1) % count;
        const auto& previous = polyline->vertlist.at(previousIndex);
        const auto& current = polyline->vertlist.at(startIndex);
        const QVector3D startPoint(previous->basePoint.x, previous->basePoint.y, previous->basePoint.z);
        const QVector3D endPoint(current->basePoint.x, current->basePoint.y, current->basePoint.z);
        return bulgeSegmentTangentAtEnd(startPoint, endPoint, previous->bulge);
    }

    QVector3D polylineReverseStartTangentAt(const DRW_Polyline* polyline, size_t startIndex)
    {
        return -polylineForwardEndTangentAt(polyline, startIndex);
    }

    QVector3D lwPolylineForwardStartTangent(const DRW_LWPolyline* polyline)
    {
        if (polyline == nullptr || polyline->vertlist.size() < 2)
        {
            return QVector3D();
        }

        const float z = static_cast<float>(polyline->elevation);

        for (size_t index = 0; index + 1 < polyline->vertlist.size(); ++index)
        {
            const auto& current = polyline->vertlist.at(index);
            const auto& next = polyline->vertlist.at(index + 1);
            const QVector3D startPoint(static_cast<float>(current->x), static_cast<float>(current->y), z);
            const QVector3D endPoint(static_cast<float>(next->x), static_cast<float>(next->y), z);
            const QVector3D tangent = bulgeSegmentTangentAtStart(startPoint, endPoint, current->bulge);

            if (tangent.lengthSquared() > kVisualEpsilon)
            {
                return tangent;
            }
        }

        if ((polyline->flags & 1) != 0)
        {
            const auto& current = polyline->vertlist.back();
            const auto& next = polyline->vertlist.front();
            const QVector3D startPoint(static_cast<float>(current->x), static_cast<float>(current->y), z);
            const QVector3D endPoint(static_cast<float>(next->x), static_cast<float>(next->y), z);
            return bulgeSegmentTangentAtStart(startPoint, endPoint, current->bulge);
        }

        return QVector3D();
    }

    QVector3D lwPolylineForwardStartTangentAt(const DRW_LWPolyline* polyline, size_t startIndex)
    {
        if (polyline == nullptr || polyline->vertlist.size() < 2)
        {
            return QVector3D();
        }

        const size_t count = polyline->vertlist.size();
        const size_t nextIndex = (startIndex + 1) % count;
        const float z = static_cast<float>(polyline->elevation);
        const auto& current = polyline->vertlist.at(startIndex);
        const auto& next = polyline->vertlist.at(nextIndex);
        const QVector3D startPoint(static_cast<float>(current->x), static_cast<float>(current->y), z);
        const QVector3D endPoint(static_cast<float>(next->x), static_cast<float>(next->y), z);
        return bulgeSegmentTangentAtStart(startPoint, endPoint, current->bulge);
    }

    QVector3D lwPolylineReverseStartTangent(const DRW_LWPolyline* polyline)
    {
        if (polyline == nullptr || polyline->vertlist.size() < 2)
        {
            return QVector3D();
        }

        const float z = static_cast<float>(polyline->elevation);

        for (size_t index = polyline->vertlist.size() - 1; index > 0; --index)
        {
            const auto& current = polyline->vertlist.at(index);
            const auto& next = polyline->vertlist.at(index - 1);
            const QVector3D startPoint(static_cast<float>(current->x), static_cast<float>(current->y), z);
            const QVector3D endPoint(static_cast<float>(next->x), static_cast<float>(next->y), z);
            const QVector3D tangent = bulgeSegmentTangentAtStart(startPoint, endPoint, -next->bulge);

            if (tangent.lengthSquared() > kVisualEpsilon)
            {
                return tangent;
            }
        }

        if ((polyline->flags & 1) != 0)
        {
            const auto& current = polyline->vertlist.front();
            const auto& next = polyline->vertlist.back();
            const QVector3D startPoint(static_cast<float>(current->x), static_cast<float>(current->y), z);
            const QVector3D endPoint(static_cast<float>(next->x), static_cast<float>(next->y), z);
            return bulgeSegmentTangentAtStart(startPoint, endPoint, -next->bulge);
        }

        return QVector3D();
    }

    QVector3D lwPolylineForwardEndTangentAt(const DRW_LWPolyline* polyline, size_t startIndex)
    {
        if (polyline == nullptr || polyline->vertlist.size() < 2)
        {
            return QVector3D();
        }

        const size_t count = polyline->vertlist.size();
        const size_t previousIndex = (startIndex + count - 1) % count;
        const float z = static_cast<float>(polyline->elevation);
        const auto& previous = polyline->vertlist.at(previousIndex);
        const auto& current = polyline->vertlist.at(startIndex);
        const QVector3D startPoint(static_cast<float>(previous->x), static_cast<float>(previous->y), z);
        const QVector3D endPoint(static_cast<float>(current->x), static_cast<float>(current->y), z);
        return bulgeSegmentTangentAtEnd(startPoint, endPoint, previous->bulge);
    }

    QVector3D lwPolylineReverseStartTangentAt(const DRW_LWPolyline* polyline, size_t startIndex)
    {
        return -lwPolylineForwardEndTangentAt(polyline, startIndex);
    }

    size_t effectiveClosedPolylineStartIndex(const CadItem* item, size_t vertexCount)
    {
        if (vertexCount == 0)
        {
            return 0;
        }

        if (item != nullptr && item->m_hasCustomProcessStart)
        {
            const int rawIndex = static_cast<int>(std::llround(item->m_processStartParameter));
            const int normalized = ((rawIndex % static_cast<int>(vertexCount)) + static_cast<int>(vertexCount)) % static_cast<int>(vertexCount);
            return static_cast<size_t>(normalized);
        }

        return 0;
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
        info.direction = normalizeOrZero(info.forwardEndPoint - info.forwardStartPoint);
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
        info.direction = arcTangentAt(arc->staangle, false);
        break;
    }
    case DRW::ETYPE::CIRCLE:
    {
        const DRW_Circle* circle = static_cast<const DRW_Circle*>(item->m_nativeEntity);
        const double startParameter = effectiveCircleStartParameter(item);
        const QVector3D center(circle->basePoint.x, circle->basePoint.y, circle->basePoint.z);
        info.closedPath = true;
        info.forwardStartPoint = QVector3D
        (
            static_cast<float>(center.x() + std::cos(startParameter) * circle->radious),
            static_cast<float>(center.y() + std::sin(startParameter) * circle->radious),
            center.z()
        );
        info.forwardEndPoint = info.forwardStartPoint;
        info.labelAnchor = center;
        info.direction = arcTangentAt(startParameter, false);
        break;
    }
    case DRW::ETYPE::ELLIPSE:
    {
        const DRW_Ellipse* ellipse = static_cast<const DRW_Ellipse*>(item->m_nativeEntity);
        double startParam = ellipse->staparam;
        double endParam = ellipse->endparam;
        info.closedPath = isFullEllipsePath(ellipse);

        if (info.closedPath)
        {
            startParam = effectiveClosedEllipseStartParameter(item, ellipse);
            endParam = startParam;
        }
        else
        {
            while (endParam <= startParam)
            {
                endParam += kTwoPi;
            }
        }

        info.forwardStartPoint = ellipsePointAt(ellipse, startParam);
        info.forwardEndPoint = ellipsePointAt(ellipse, endParam);
        info.labelAnchor = info.closedPath
            ? QVector3D(ellipse->basePoint.x, ellipse->basePoint.y, ellipse->basePoint.z)
            : (hasGeometryAnchor ? preferredAnchor : (info.forwardStartPoint + info.forwardEndPoint) * 0.5f);
        info.direction = ellipseTangentAt(ellipse, startParam, false);
        break;
    }
    case DRW::ETYPE::POLYLINE:
    {
        const DRW_Polyline* polyline = static_cast<const DRW_Polyline*>(item->m_nativeEntity);

        if (polyline->vertlist.empty())
        {
            return info;
        }

        info.closedPath = (polyline->flags & 1) != 0;
        const size_t seamIndex = info.closedPath
            ? effectiveClosedPolylineStartIndex(item, polyline->vertlist.size())
            : 0;
        const auto& firstVertex = polyline->vertlist.at(seamIndex);
        const auto& lastVertex = polyline->vertlist.back();
        info.forwardStartPoint = QVector3D(firstVertex->basePoint.x, firstVertex->basePoint.y, firstVertex->basePoint.z);
        info.forwardEndPoint = info.closedPath
            ? info.forwardStartPoint
            : QVector3D(lastVertex->basePoint.x, lastVertex->basePoint.y, lastVertex->basePoint.z);
        info.labelAnchor = hasGeometryAnchor ? preferredAnchor : info.forwardStartPoint;
        info.direction = info.closedPath
            ? polylineForwardStartTangentAt(polyline, seamIndex)
            : polylineForwardStartTangent(polyline);
        break;
    }
    case DRW::ETYPE::LWPOLYLINE:
    {
        const DRW_LWPolyline* polyline = static_cast<const DRW_LWPolyline*>(item->m_nativeEntity);

        if (polyline->vertlist.empty())
        {
            return info;
        }

        const float z = static_cast<float>(polyline->elevation);
        info.closedPath = (polyline->flags & 1) != 0;
        const size_t seamIndex = info.closedPath
            ? effectiveClosedPolylineStartIndex(item, polyline->vertlist.size())
            : 0;
        const auto& firstVertex = polyline->vertlist.at(seamIndex);
        const auto& lastVertex = polyline->vertlist.back();
        info.forwardStartPoint = QVector3D(static_cast<float>(firstVertex->x), static_cast<float>(firstVertex->y), z);
        info.forwardEndPoint = info.closedPath
            ? info.forwardStartPoint
            : QVector3D(static_cast<float>(lastVertex->x), static_cast<float>(lastVertex->y), z);
        info.labelAnchor = hasGeometryAnchor ? preferredAnchor : info.forwardStartPoint;
        info.direction = info.closedPath
            ? lwPolylineForwardStartTangentAt(polyline, seamIndex)
            : lwPolylineForwardStartTangent(polyline);
        break;
    }
    default:
        return info;
    }

    info.startPoint = info.isReverse ? info.forwardEndPoint : info.forwardStartPoint;
    info.endPoint = info.isReverse ? info.forwardStartPoint : info.forwardEndPoint;

    if (info.isReverse)
    {
        switch (item->m_type)
        {
        case DRW::ETYPE::LINE:
            info.direction = normalizeOrZero(info.endPoint - info.startPoint);
            break;
        case DRW::ETYPE::ARC:
        {
            const DRW_Arc* arc = static_cast<const DRW_Arc*>(item->m_nativeEntity);
            info.direction = arcTangentAt(arc->endangle, true);
            break;
        }
        case DRW::ETYPE::CIRCLE:
            info.direction = arcTangentAt(effectiveCircleStartParameter(item), true);
            break;
        case DRW::ETYPE::ELLIPSE:
        {
            const DRW_Ellipse* ellipse = static_cast<const DRW_Ellipse*>(item->m_nativeEntity);
            const double parameter = info.closedPath
                ? effectiveClosedEllipseStartParameter(item, ellipse)
                : ellipse->endparam;
            info.direction = ellipseTangentAt(ellipse, parameter, true);
            break;
        }
        case DRW::ETYPE::POLYLINE:
        {
            const DRW_Polyline* polyline = static_cast<const DRW_Polyline*>(item->m_nativeEntity);
            info.direction = info.closedPath
                ? polylineReverseStartTangentAt(polyline, effectiveClosedPolylineStartIndex(item, polyline->vertlist.size()))
                : polylineReverseStartTangent(polyline);
            break;
        }
        case DRW::ETYPE::LWPOLYLINE:
        {
            const DRW_LWPolyline* polyline = static_cast<const DRW_LWPolyline*>(item->m_nativeEntity);
            info.direction = info.closedPath
                ? lwPolylineReverseStartTangentAt(polyline, effectiveClosedPolylineStartIndex(item, polyline->vertlist.size()))
                : lwPolylineReverseStartTangent(polyline);
            break;
        }
        default:
            break;
        }
    }

    if (info.direction.lengthSquared() <= kVisualEpsilon)
    {
        info.direction = normalizeOrZero(info.endPoint - info.startPoint);
    }

    if (info.direction.lengthSquared() <= kVisualEpsilon)
    {
        info.direction = normalizeOrZero(item->m_processDirection);
    }

    info.valid = true;
    return info;
}
