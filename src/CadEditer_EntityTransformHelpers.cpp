// CadEditer 实体变换与控制点辅助
#include "pch.h"

#include "CadEditerWorkflowInternal.h"

#include <algorithm>
#include <cmath>

namespace CadEditerWorkflowInternal
{
    constexpr double kXlineGripHandleLength = 50.0;

    void translateCoord(DRW_Coord& point, const QVector3D& delta)
    {
        point.x += delta.x();
        point.y += delta.y();
        point.z += delta.z();
    }

    void translateEntity(DRW_Entity* entity, const QVector3D& delta)
    {
        if (entity == nullptr)
        {
            return;
        }

        switch (entity->eType)
        {
        case DRW::ETYPE::POINT:
        {
            translateCoord(static_cast<DRW_Point*>(entity)->basePoint, delta);
            break;
        }
        case DRW::ETYPE::LINE:
        {
            DRW_Line* line = static_cast<DRW_Line*>(entity);
            translateCoord(line->basePoint, delta);
            translateCoord(line->secPoint, delta);
            break;
        }
        case DRW::ETYPE::XLINE:
        {
            DRW_Xline* xline = static_cast<DRW_Xline*>(entity);
            translateCoord(xline->basePoint, delta);
            break;
        }
        case DRW::ETYPE::CIRCLE:
        {
            translateCoord(static_cast<DRW_Circle*>(entity)->basePoint, delta);
            break;
        }
        case DRW::ETYPE::ARC:
        {
            translateCoord(static_cast<DRW_Arc*>(entity)->basePoint, delta);
            break;
        }
        case DRW::ETYPE::ELLIPSE:
        {
            translateCoord(static_cast<DRW_Ellipse*>(entity)->basePoint, delta);
            break;
        }
        case DRW::ETYPE::LWPOLYLINE:
        {
            DRW_LWPolyline* polyline = static_cast<DRW_LWPolyline*>(entity);
            polyline->elevation += delta.z();

            for (const std::shared_ptr<DRW_Vertex2D>& vertex : polyline->vertlist)
            {
                vertex->x += delta.x();
                vertex->y += delta.y();
            }
            break;
        }
        case DRW::ETYPE::POLYLINE:
        {
            DRW_Polyline* polyline = static_cast<DRW_Polyline*>(entity);
            translateCoord(polyline->basePoint, delta);

            for (const std::shared_ptr<DRW_Vertex>& vertex : polyline->vertlist)
            {
                translateCoord(vertex->basePoint, delta);
            }
            break;
        }
        default:
            break;
        }
    }

    QVector3D rotatePlanarPoint(const QVector3D& point, const QVector3D& basePoint, double radians)
    {
        const QVector3D planarPoint(point.x(), point.y(), point.z());
        const QVector3D planarBasePoint(basePoint.x(), basePoint.y(), basePoint.z());
        const double cosineValue = std::cos(radians);
        const double sineValue = std::sin(radians);
        const double dx = planarPoint.x() - planarBasePoint.x();
        const double dy = planarPoint.y() - planarBasePoint.y();

        return QVector3D
        (
            static_cast<float>(planarBasePoint.x() + dx * cosineValue - dy * sineValue),
            static_cast<float>(planarBasePoint.y() + dx * sineValue + dy * cosineValue),
            point.z()
        );
    }

    QVector3D scalePlanarPoint(const QVector3D& point, const QVector3D& basePoint, double scaleFactor)
    {
        const QVector3D planarPoint(point.x(), point.y(), point.z());
        const QVector3D planarBasePoint(basePoint.x(), basePoint.y(), basePoint.z());
        const QVector3D delta = planarPoint - planarBasePoint;
        return QVector3D
        (
            planarBasePoint.x() + delta.x() * static_cast<float>(scaleFactor),
            planarBasePoint.y() + delta.y() * static_cast<float>(scaleFactor),
            point.z()
        );
    }

    void rotateCoordAround(DRW_Coord& point, const QVector3D& basePoint, double radians)
    {
        const QVector3D rotatedPoint = rotatePlanarPoint(QVector3D(point.x, point.y, point.z), basePoint, radians);
        point.x = rotatedPoint.x();
        point.y = rotatedPoint.y();
        point.z = rotatedPoint.z();
    }

    void scaleCoordAround(DRW_Coord& point, const QVector3D& basePoint, double scaleFactor)
    {
        const QVector3D scaledPoint = scalePlanarPoint(QVector3D(point.x, point.y, point.z), basePoint, scaleFactor);
        point.x = scaledPoint.x();
        point.y = scaledPoint.y();
        point.z = scaledPoint.z();
    }

    void rotateEntity(DRW_Entity* entity, const QVector3D& basePoint, double angleDegrees)
    {
        if (entity == nullptr)
        {
            return;
        }

        const double radians = angleDegrees * kPi / 180.0;

        switch (entity->eType)
        {
        case DRW::ETYPE::POINT:
            rotateCoordAround(static_cast<DRW_Point*>(entity)->basePoint, basePoint, radians);
            break;
        case DRW::ETYPE::LINE:
        {
            DRW_Line* line = static_cast<DRW_Line*>(entity);
            rotateCoordAround(line->basePoint, basePoint, radians);
            rotateCoordAround(line->secPoint, basePoint, radians);
            break;
        }
        case DRW::ETYPE::XLINE:
        {
            DRW_Xline* xline = static_cast<DRW_Xline*>(entity);
            rotateCoordAround(xline->basePoint, basePoint, radians);
            const QVector3D rotatedDirection = rotatePlanarPoint
            (
                QVector3D(xline->secPoint.x, xline->secPoint.y, xline->secPoint.z),
                QVector3D(0.0f, 0.0f, 0.0f),
                radians
            );
            xline->secPoint.x = rotatedDirection.x();
            xline->secPoint.y = rotatedDirection.y();
            xline->secPoint.z = rotatedDirection.z();
            break;
        }
        case DRW::ETYPE::CIRCLE:
            rotateCoordAround(static_cast<DRW_Circle*>(entity)->basePoint, basePoint, radians);
            break;
        case DRW::ETYPE::ARC:
        {
            DRW_Arc* arc = static_cast<DRW_Arc*>(entity);
            rotateCoordAround(arc->basePoint, basePoint, radians);
            const double angleOffset = radians;
            arc->staangle += angleOffset;
            arc->endangle += angleOffset;
            break;
        }
        case DRW::ETYPE::ELLIPSE:
        {
            DRW_Ellipse* ellipse = static_cast<DRW_Ellipse*>(entity);
            rotateCoordAround(ellipse->basePoint, basePoint, radians);
            const QVector3D rotatedMajorAxis = rotatePlanarPoint
            (
                QVector3D(ellipse->secPoint.x, ellipse->secPoint.y, ellipse->secPoint.z),
                QVector3D(0.0f, 0.0f, 0.0f),
                radians
            );
            ellipse->secPoint.x = rotatedMajorAxis.x();
            ellipse->secPoint.y = rotatedMajorAxis.y();
            ellipse->secPoint.z = rotatedMajorAxis.z();
            break;
        }
        case DRW::ETYPE::LWPOLYLINE:
        {
            DRW_LWPolyline* polyline = static_cast<DRW_LWPolyline*>(entity);

            for (const std::shared_ptr<DRW_Vertex2D>& vertex : polyline->vertlist)
            {
                const QVector3D rotatedVertex = rotatePlanarPoint
                (
                    QVector3D(vertex->x, vertex->y, polyline->elevation),
                    basePoint,
                    radians
                );
                vertex->x = rotatedVertex.x();
                vertex->y = rotatedVertex.y();
            }
            break;
        }
        case DRW::ETYPE::POLYLINE:
        {
            DRW_Polyline* polyline = static_cast<DRW_Polyline*>(entity);
            rotateCoordAround(polyline->basePoint, basePoint, radians);

            for (const std::shared_ptr<DRW_Vertex>& vertex : polyline->vertlist)
            {
                rotateCoordAround(vertex->basePoint, basePoint, radians);
            }
            break;
        }
        default:
            break;
        }
    }

    void scaleEntity(DRW_Entity* entity, const QVector3D& basePoint, double scaleFactor)
    {
        if (entity == nullptr)
        {
            return;
        }

        switch (entity->eType)
        {
        case DRW::ETYPE::POINT:
            scaleCoordAround(static_cast<DRW_Point*>(entity)->basePoint, basePoint, scaleFactor);
            break;
        case DRW::ETYPE::LINE:
        {
            DRW_Line* line = static_cast<DRW_Line*>(entity);
            scaleCoordAround(line->basePoint, basePoint, scaleFactor);
            scaleCoordAround(line->secPoint, basePoint, scaleFactor);
            break;
        }
        case DRW::ETYPE::XLINE:
        {
            DRW_Xline* xline = static_cast<DRW_Xline*>(entity);
            scaleCoordAround(xline->basePoint, basePoint, scaleFactor);
            break;
        }
        case DRW::ETYPE::CIRCLE:
        {
            DRW_Circle* circle = static_cast<DRW_Circle*>(entity);
            scaleCoordAround(circle->basePoint, basePoint, scaleFactor);
            circle->radious *= scaleFactor;
            break;
        }
        case DRW::ETYPE::ARC:
        {
            DRW_Arc* arc = static_cast<DRW_Arc*>(entity);
            scaleCoordAround(arc->basePoint, basePoint, scaleFactor);
            arc->radious *= scaleFactor;
            break;
        }
        case DRW::ETYPE::ELLIPSE:
        {
            DRW_Ellipse* ellipse = static_cast<DRW_Ellipse*>(entity);
            scaleCoordAround(ellipse->basePoint, basePoint, scaleFactor);
            ellipse->secPoint.x *= scaleFactor;
            ellipse->secPoint.y *= scaleFactor;
            ellipse->secPoint.z *= scaleFactor;
            break;
        }
        case DRW::ETYPE::LWPOLYLINE:
        {
            DRW_LWPolyline* polyline = static_cast<DRW_LWPolyline*>(entity);

            for (const std::shared_ptr<DRW_Vertex2D>& vertex : polyline->vertlist)
            {
                const QVector3D scaledVertex = scalePlanarPoint
                (
                    QVector3D(vertex->x, vertex->y, polyline->elevation),
                    basePoint,
                    scaleFactor
                );
                vertex->x = scaledVertex.x();
                vertex->y = scaledVertex.y();
            }
            break;
        }
        case DRW::ETYPE::POLYLINE:
        {
            DRW_Polyline* polyline = static_cast<DRW_Polyline*>(entity);
            scaleCoordAround(polyline->basePoint, basePoint, scaleFactor);

            for (const std::shared_ptr<DRW_Vertex>& vertex : polyline->vertlist)
            {
                scaleCoordAround(vertex->basePoint, basePoint, scaleFactor);
            }
            break;
        }
        default:
            break;
        }
    }

    bool readEditableControlPoint(const CadItem* item, int pointIndex, QVector3D& point)
    {
        if (item == nullptr || item->m_nativeEntity == nullptr || pointIndex < 0)
        {
            return false;
        }

        switch (item->m_type)
        {
        case DRW::ETYPE::POINT:
        {
            if (pointIndex != 0)
            {
                return false;
            }

            const DRW_Point* pointEntity = static_cast<const DRW_Point*>(item->m_nativeEntity);
            point = QVector3D(pointEntity->basePoint.x, pointEntity->basePoint.y, 0.0f);
            return true;
        }
        case DRW::ETYPE::LINE:
        {
            const DRW_Line* line = static_cast<const DRW_Line*>(item->m_nativeEntity);
            const QVector3D startPoint(line->basePoint.x, line->basePoint.y, 0.0f);
            const QVector3D endPoint(line->secPoint.x, line->secPoint.y, 0.0f);

            if (pointIndex == 0)
            {
                point = startPoint;
                return true;
            }

            if (pointIndex == 1)
            {
                point = endPoint;
                return true;
            }

            if (pointIndex == 2)
            {
                point = (startPoint + endPoint) * 0.5f;
                return true;
            }

            return false;
        }
        case DRW::ETYPE::XLINE:
        {
            const DRW_Xline* xline = static_cast<const DRW_Xline*>(item->m_nativeEntity);
            const QVector3D basePoint(xline->basePoint.x, xline->basePoint.y, 0.0f);
            const QVector3D direction = normalizedXlineDirection(xline);

            if (pointIndex == 0)
            {
                point = basePoint;
                return true;
            }

            if (pointIndex == 1)
            {
                point = basePoint + direction * static_cast<float>(kXlineGripHandleLength);
                return true;
            }

            if (pointIndex == 2)
            {
                point = basePoint - direction * static_cast<float>(kXlineGripHandleLength);
                return true;
            }

            return false;
        }
        case DRW::ETYPE::POLYLINE:
        {
            const DRW_Polyline* polyline = static_cast<const DRW_Polyline*>(item->m_nativeEntity);

            if (pointIndex >= static_cast<int>(polyline->vertlist.size()))
            {
                return false;
            }

            const std::shared_ptr<DRW_Vertex>& vertex = polyline->vertlist[static_cast<size_t>(pointIndex)];

            if (vertex == nullptr)
            {
                return false;
            }

            point = QVector3D(vertex->basePoint.x, vertex->basePoint.y, 0.0f);
            return true;
        }
        case DRW::ETYPE::LWPOLYLINE:
        {
            const DRW_LWPolyline* polyline = static_cast<const DRW_LWPolyline*>(item->m_nativeEntity);

            if (pointIndex >= static_cast<int>(polyline->vertlist.size()))
            {
                return false;
            }

            const std::shared_ptr<DRW_Vertex2D>& vertex = polyline->vertlist[static_cast<size_t>(pointIndex)];

            if (vertex == nullptr)
            {
                return false;
            }

            point = QVector3D(static_cast<float>(vertex->x), static_cast<float>(vertex->y), 0.0f);
            return true;
        }
        case DRW::ETYPE::CIRCLE:
        {
            const DRW_Circle* circle = static_cast<const DRW_Circle*>(item->m_nativeEntity);

            if (pointIndex == 0)
            {
                point = QVector3D(circle->basePoint.x, circle->basePoint.y, 0.0f);
                return true;
            }

            if (pointIndex == 1)
            {
                point = flattenToDrawingPlane(circlePointAt(circle, 0.0));
                return true;
            }

            if (pointIndex == 2)
            {
                point = flattenToDrawingPlane(circlePointAt(circle, kPi * 0.5));
                return true;
            }

            if (pointIndex == 3)
            {
                point = flattenToDrawingPlane(circlePointAt(circle, kPi));
                return true;
            }

            if (pointIndex == 4)
            {
                point = flattenToDrawingPlane(circlePointAt(circle, kPi * 1.5));
                return true;
            }

            return false;
        }
        case DRW::ETYPE::ARC:
        {
            const DRW_Arc* arc = static_cast<const DRW_Arc*>(item->m_nativeEntity);

            if (pointIndex == 0)
            {
                point = QVector3D(arc->basePoint.x, arc->basePoint.y, 0.0f);
                return true;
            }

            if (pointIndex == 1)
            {
                point = flattenToDrawingPlane(arcPointAt(arc, arc->staangle));
                return true;
            }

            if (pointIndex == 2)
            {
                point = flattenToDrawingPlane(arcPointAt(arc, arcMidAngle(arc)));
                return true;
            }

            if (pointIndex == 3)
            {
                double endAngle = arc->endangle;

                while (endAngle <= arc->staangle)
                {
                    endAngle += kTwoPi;
                }

                point = flattenToDrawingPlane(arcPointAt(arc, endAngle));
                return true;
            }

            return false;
        }
        case DRW::ETYPE::ELLIPSE:
        {
            const DRW_Ellipse* ellipse = static_cast<const DRW_Ellipse*>(item->m_nativeEntity);
            QVector3D majorAxis;
            QVector3D minorAxis;

            if (!tryBuildEllipseAxes(ellipse, majorAxis, minorAxis))
            {
                return false;
            }

            const QVector3D center(ellipse->basePoint.x, ellipse->basePoint.y, ellipse->basePoint.z);

            switch (pointIndex)
            {
            case 0:
                point = flattenToDrawingPlane(center);
                return true;
            case 1:
                point = flattenToDrawingPlane(center + majorAxis);
                return true;
            case 2:
                point = flattenToDrawingPlane(center - majorAxis);
                return true;
            case 3:
                point = flattenToDrawingPlane(center + minorAxis);
                return true;
            case 4:
                point = flattenToDrawingPlane(center - minorAxis);
                return true;
            case 5:
                point = flattenToDrawingPlane(ellipsePointAt(ellipse, ellipse->staparam));
                return true;
            case 6:
                point = flattenToDrawingPlane(ellipsePointAt(ellipse, ellipse->endparam));
                return true;
            default:
                return false;
            }
        }
        default:
            return false;
        }
    }

    bool applyEditableControlPoint(DRW_Entity* entity, int pointIndex, const QVector3D& worldPos)
    {
        if (entity == nullptr || pointIndex < 0)
        {
            return false;
        }

        const QVector3D point = flattenToDrawingPlane(worldPos);

        switch (entity->eType)
        {
        case DRW::ETYPE::POINT:
        {
            if (pointIndex != 0)
            {
                return false;
            }

            DRW_Point* pointEntity = static_cast<DRW_Point*>(entity);
            pointEntity->basePoint.x = point.x();
            pointEntity->basePoint.y = point.y();
            pointEntity->basePoint.z = point.z();
            return true;
        }
        case DRW::ETYPE::LINE:
        {
            DRW_Line* line = static_cast<DRW_Line*>(entity);

            if (pointIndex == 0)
            {
                line->basePoint.x = point.x();
                line->basePoint.y = point.y();
                line->basePoint.z = point.z();
                return true;
            }

            if (pointIndex == 1)
            {
                line->secPoint.x = point.x();
                line->secPoint.y = point.y();
                line->secPoint.z = point.z();
                return true;
            }

            if (pointIndex == 2)
            {
                const QVector3D startPoint(line->basePoint.x, line->basePoint.y, line->basePoint.z);
                const QVector3D endPoint(line->secPoint.x, line->secPoint.y, line->secPoint.z);
                const QVector3D delta = point - (startPoint + endPoint) * 0.5f;
                translateCoord(line->basePoint, delta);
                translateCoord(line->secPoint, delta);
                return true;
            }

            return false;
        }
        case DRW::ETYPE::XLINE:
        {
            DRW_Xline* xline = static_cast<DRW_Xline*>(entity);
            const QVector3D basePoint(xline->basePoint.x, xline->basePoint.y, xline->basePoint.z);

            if (pointIndex == 0)
            {
                xline->basePoint.x = point.x();
                xline->basePoint.y = point.y();
                xline->basePoint.z = point.z();
                return true;
            }

            if (pointIndex == 1 || pointIndex == 2)
            {
                QVector3D direction = pointIndex == 1
                    ? flattenToDrawingPlane(point - basePoint)
                    : flattenToDrawingPlane(basePoint - point);

                if (direction.lengthSquared() <= kGeometryEpsilon)
                {
                    return false;
                }

                direction.normalize();
                xline->secPoint.x = direction.x();
                xline->secPoint.y = direction.y();
                xline->secPoint.z = direction.z();
                return true;
            }

            return false;
        }
        case DRW::ETYPE::POLYLINE:
        {
            DRW_Polyline* polyline = static_cast<DRW_Polyline*>(entity);

            if (pointIndex >= static_cast<int>(polyline->vertlist.size()))
            {
                return false;
            }

            const std::shared_ptr<DRW_Vertex>& vertex = polyline->vertlist[static_cast<size_t>(pointIndex)];

            if (vertex == nullptr)
            {
                return false;
            }

            vertex->basePoint.x = point.x();
            vertex->basePoint.y = point.y();
            vertex->basePoint.z = point.z();
            return true;
        }
        case DRW::ETYPE::LWPOLYLINE:
        {
            DRW_LWPolyline* polyline = static_cast<DRW_LWPolyline*>(entity);

            if (pointIndex >= static_cast<int>(polyline->vertlist.size()))
            {
                return false;
            }

            const std::shared_ptr<DRW_Vertex2D>& vertex = polyline->vertlist[static_cast<size_t>(pointIndex)];

            if (vertex == nullptr)
            {
                return false;
            }

            vertex->x = point.x();
            vertex->y = point.y();
            return true;
        }
        case DRW::ETYPE::CIRCLE:
        {
            DRW_Circle* circle = static_cast<DRW_Circle*>(entity);
            const QVector3D center(circle->basePoint.x, circle->basePoint.y, circle->basePoint.z);

            if (pointIndex == 0)
            {
                circle->basePoint.x = point.x();
                circle->basePoint.y = point.y();
                circle->basePoint.z = point.z();
                return true;
            }

            if (pointIndex >= 1 && pointIndex <= 4)
            {
                const double radius = (point - center).length();

                if (radius <= kGeometryEpsilon)
                {
                    return false;
                }

                circle->radious = radius;
                return true;
            }

            return false;
        }
        case DRW::ETYPE::ARC:
        {
            DRW_Arc* arc = static_cast<DRW_Arc*>(entity);
            DRW_Circle circleProxy;
            circleProxy.basePoint = arc->basePoint;
            circleProxy.extPoint = arc->extPoint;
            circleProxy.radious = arc->radious;

            if (pointIndex == 0)
            {
                arc->basePoint.x = point.x();
                arc->basePoint.y = point.y();
                arc->basePoint.z = point.z();
                return true;
            }

            if (pointIndex == 2)
            {
                const QVector3D center(arc->basePoint.x, arc->basePoint.y, arc->basePoint.z);
                const double radius = (point - center).length();

                if (radius <= kGeometryEpsilon)
                {
                    return false;
                }

                arc->radious = radius;
                return true;
            }

            bool validAngle = false;
            const double targetAngle = angleFromPointOnCircle(&circleProxy, point, &validAngle);

            if (!validAngle)
            {
                return false;
            }

            if (pointIndex == 1)
            {
                arc->staangle = normalizeAnglePositive(targetAngle);
                arc->endangle = normalizeAnglePositive(arc->endangle);

                while (arc->endangle <= arc->staangle)
                {
                    arc->endangle += kTwoPi;
                }

                return true;
            }

            if (pointIndex == 3)
            {
                arc->staangle = normalizeAnglePositive(arc->staangle);
                arc->endangle = normalizeAnglePositive(targetAngle);

                while (arc->endangle <= arc->staangle)
                {
                    arc->endangle += kTwoPi;
                }

                return true;
            }

            return false;
        }
        case DRW::ETYPE::ELLIPSE:
        {
            DRW_Ellipse* ellipse = static_cast<DRW_Ellipse*>(entity);
            QVector3D majorAxis;
            QVector3D minorAxis;

            if (!tryBuildEllipseAxes(ellipse, majorAxis, minorAxis))
            {
                return false;
            }

            const QVector3D center(ellipse->basePoint.x, ellipse->basePoint.y, ellipse->basePoint.z);
            const double majorLength = majorAxis.length();

            if (majorLength <= kGeometryEpsilon)
            {
                return false;
            }

            QVector3D majorUnit = majorAxis;
            majorUnit.normalize();
            QVector3D minorUnit = minorAxis;
            minorUnit.normalize();

            if (pointIndex == 0)
            {
                ellipse->basePoint.x = point.x();
                ellipse->basePoint.y = point.y();
                ellipse->basePoint.z = point.z();
                return true;
            }

            if (pointIndex == 1 || pointIndex == 2)
            {
                const QVector3D direction = pointIndex == 1 ? (point - center) : (center - point);

                if (direction.lengthSquared() <= kGeometryEpsilon)
                {
                    return false;
                }

                ellipse->secPoint.x = direction.x();
                ellipse->secPoint.y = direction.y();
                ellipse->secPoint.z = direction.z();
                return true;
            }

            if (pointIndex == 3 || pointIndex == 4)
            {
                const QVector3D local = point - center;
                const double minorLength = std::abs(QVector3D::dotProduct(local, minorUnit));

                if (minorLength <= kGeometryEpsilon)
                {
                    return false;
                }

                ellipse->ratio = std::clamp(minorLength / majorLength, kMinEllipseRatio, 1.0 - 1.0e-6);
                return true;
            }

            if (pointIndex == 5 || pointIndex == 6)
            {
                double parameter = 0.0;

                if (!ellipseParameterFromPoint(ellipse, point, parameter))
                {
                    return false;
                }

                if (pointIndex == 5)
                {
                    ellipse->staparam = normalizeAnglePositive(parameter);
                }
                else
                {
                    ellipse->endparam = normalizeAnglePositive(parameter);
                }

                return true;
            }

            return false;
        }
        default:
            return false;
        }
    }

    std::unique_ptr<DRW_Entity> cloneEntity(const DRW_Entity* entity)
    {
        if (entity == nullptr)
        {
            return nullptr;
        }

        switch (entity->eType)
        {
        case DRW::ETYPE::POINT:
            return std::make_unique<DRW_Point>(*static_cast<const DRW_Point*>(entity));
        case DRW::ETYPE::LINE:
            return std::make_unique<DRW_Line>(*static_cast<const DRW_Line*>(entity));
        case DRW::ETYPE::XLINE:
            return std::make_unique<DRW_Xline>(*static_cast<const DRW_Xline*>(entity));
        case DRW::ETYPE::CIRCLE:
            return std::make_unique<DRW_Circle>(*static_cast<const DRW_Circle*>(entity));
        case DRW::ETYPE::ARC:
            return std::make_unique<DRW_Arc>(*static_cast<const DRW_Arc*>(entity));
        case DRW::ETYPE::ELLIPSE:
            return std::make_unique<DRW_Ellipse>(*static_cast<const DRW_Ellipse*>(entity));
        case DRW::ETYPE::LWPOLYLINE:
            return std::make_unique<DRW_LWPolyline>(*static_cast<const DRW_LWPolyline*>(entity));
        case DRW::ETYPE::POLYLINE:
            return std::make_unique<DRW_Polyline>(*static_cast<const DRW_Polyline*>(entity));
        default:
            return nullptr;
        }
    }
}
