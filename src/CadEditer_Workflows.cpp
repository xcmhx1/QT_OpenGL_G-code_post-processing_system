// CadEditer 实现文件
// 实现 CadEditer 模块，对应头文件中声明的主要行为和协作流程。
// 编辑器模块，负责绘图创建、实体修改以及 Undo/Redo 命令栈管理。
#include "pch.h"

#include "CadEditer.h"

#include "CadDocument.h"
#include "CadItem.h"
#include "DrawStateMachine.h"

#include <QSet>

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
    // 几何与角度计算使用的局部常量
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kTwoPi = 6.28318530717958647692;
    constexpr double kGeometryEpsilon = 1.0e-9;
    constexpr double kMinEllipseRatio = 1.0e-4;
    constexpr double kXlineGripHandleLength = 50.0;

    // 将任意点压回二维绘图平面
    // @param point 输入三维点
    // @return 压平后的点
    QVector3D flattenToDrawingPlane(const QVector3D& point)
    {
        return QVector3D(point.x(), point.y(), 0.0f);
    }

    QVector3D normalizedXlineDirection(const DRW_Xline* xline)
    {
        if (xline == nullptr)
        {
            return QVector3D(1.0f, 0.0f, 0.0f);
        }

        QVector3D direction(xline->secPoint.x, xline->secPoint.y, xline->secPoint.z);

        if (direction.lengthSquared() <= kGeometryEpsilon)
        {
            return QVector3D(1.0f, 0.0f, 0.0f);
        }

        direction.normalize();
        return direction;
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

    QVector3D resolveEntityNormal(const DRW_Coord& extPoint)
    {
        QVector3D normal(extPoint.x, extPoint.y, extPoint.z);

        if (normal.lengthSquared() <= kGeometryEpsilon)
        {
            return QVector3D(0.0f, 0.0f, 1.0f);
        }

        normal.normalize();
        return normal;
    }

    void buildPlaneBasis(const QVector3D& normal, QVector3D& axisX, QVector3D& axisY)
    {
        if (std::abs(normal.x()) <= 1.0e-6f && std::abs(normal.y()) <= 1.0e-6f)
        {
            axisX = QVector3D(1.0f, 0.0f, 0.0f);
            axisY = QVector3D::crossProduct(normal, axisX);

            if (axisY.lengthSquared() <= kGeometryEpsilon)
            {
                axisY = QVector3D(0.0f, 1.0f, 0.0f);
            }
            else
            {
                axisY.normalize();
            }

            return;
        }

        const QVector3D helper = std::abs(normal.z()) < 0.999f
            ? QVector3D(0.0f, 0.0f, 1.0f)
            : QVector3D(0.0f, 1.0f, 0.0f);

        axisX = QVector3D::crossProduct(helper, normal);

        if (axisX.lengthSquared() <= kGeometryEpsilon)
        {
            axisX = QVector3D(1.0f, 0.0f, 0.0f);
        }
        else
        {
            axisX.normalize();
        }

        axisY = QVector3D::crossProduct(normal, axisX);

        if (axisY.lengthSquared() <= kGeometryEpsilon)
        {
            axisY = QVector3D(0.0f, 1.0f, 0.0f);
        }
        else
        {
            axisY.normalize();
        }
    }

    QVector3D circlePointAt(const DRW_Circle* circle, double parameter)
    {
        if (circle == nullptr || circle->radious <= 0.0)
        {
            return QVector3D();
        }

        const QVector3D center(circle->basePoint.x, circle->basePoint.y, circle->basePoint.z);
        const QVector3D normal = resolveEntityNormal(circle->extPoint);
        QVector3D axisX;
        QVector3D axisY;
        buildPlaneBasis(normal, axisX, axisY);

        return center
            + axisX * static_cast<float>(std::cos(parameter) * circle->radious)
            + axisY * static_cast<float>(std::sin(parameter) * circle->radious);
    }

    QVector3D arcPointAt(const DRW_Arc* arc, double angle)
    {
        if (arc == nullptr || arc->radious <= 0.0)
        {
            return QVector3D();
        }

        const QVector3D center(arc->basePoint.x, arc->basePoint.y, arc->basePoint.z);
        const QVector3D normal = resolveEntityNormal(arc->extPoint);
        QVector3D axisX;
        QVector3D axisY;
        buildPlaneBasis(normal, axisX, axisY);

        return center
            + axisX * static_cast<float>(std::cos(angle) * arc->radious)
            + axisY * static_cast<float>(std::sin(angle) * arc->radious);
    }

    double arcMidAngle(const DRW_Arc* arc)
    {
        if (arc == nullptr)
        {
            return 0.0;
        }

        double endAngle = arc->endangle;

        while (endAngle <= arc->staangle)
        {
            endAngle += kTwoPi;
        }

        return (arc->staangle + endAngle) * 0.5;
    }

    double angleFromPointOnCircle(const DRW_Circle* circle, const QVector3D& point, bool* valid = nullptr)
    {
        if (valid != nullptr)
        {
            *valid = false;
        }

        if (circle == nullptr)
        {
            return 0.0;
        }

        const QVector3D center(circle->basePoint.x, circle->basePoint.y, circle->basePoint.z);
        const QVector3D local = flattenToDrawingPlane(point) - center;

        if (local.lengthSquared() <= kGeometryEpsilon)
        {
            return 0.0;
        }

        const QVector3D normal = resolveEntityNormal(circle->extPoint);
        QVector3D axisX;
        QVector3D axisY;
        buildPlaneBasis(normal, axisX, axisY);

        const double x = QVector3D::dotProduct(local, axisX);
        const double y = QVector3D::dotProduct(local, axisY);

        if (std::abs(x) <= kGeometryEpsilon && std::abs(y) <= kGeometryEpsilon)
        {
            return 0.0;
        }

        if (valid != nullptr)
        {
            *valid = true;
        }

        return std::atan2(y, x);
    }

    bool tryBuildEllipseAxes(const DRW_Ellipse* ellipse, QVector3D& majorAxis, QVector3D& minorAxis)
    {
        if (ellipse == nullptr)
        {
            return false;
        }

        majorAxis = QVector3D(ellipse->secPoint.x, ellipse->secPoint.y, ellipse->secPoint.z);
        const double majorLength = majorAxis.length();

        if (majorLength <= kGeometryEpsilon || ellipse->ratio <= kMinEllipseRatio)
        {
            return false;
        }

        const QVector3D normal = resolveEntityNormal(ellipse->extPoint);
        minorAxis = QVector3D::crossProduct(normal, majorAxis);

        if (minorAxis.lengthSquared() <= kGeometryEpsilon)
        {
            return false;
        }

        minorAxis.normalize();
        minorAxis *= static_cast<float>(majorLength * ellipse->ratio);
        return true;
    }

    QVector3D ellipsePointAt(const DRW_Ellipse* ellipse, double parameter)
    {
        if (ellipse == nullptr)
        {
            return QVector3D();
        }

        QVector3D majorAxis;
        QVector3D minorAxis;

        if (!tryBuildEllipseAxes(ellipse, majorAxis, minorAxis))
        {
            return QVector3D();
        }

        const QVector3D center(ellipse->basePoint.x, ellipse->basePoint.y, ellipse->basePoint.z);
        return center
            + majorAxis * static_cast<float>(std::cos(parameter))
            + minorAxis * static_cast<float>(std::sin(parameter));
    }

    bool ellipseParameterFromPoint(const DRW_Ellipse* ellipse, const QVector3D& worldPoint, double& parameter)
    {
        QVector3D majorAxis;
        QVector3D minorAxis;

        if (ellipse == nullptr || !tryBuildEllipseAxes(ellipse, majorAxis, minorAxis))
        {
            return false;
        }

        const double majorLength = majorAxis.length();
        const double minorLength = minorAxis.length();

        if (majorLength <= kGeometryEpsilon || minorLength <= kGeometryEpsilon)
        {
            return false;
        }

        QVector3D majorUnit = majorAxis;
        QVector3D minorUnit = minorAxis;
        majorUnit.normalize();
        minorUnit.normalize();

        const QVector3D center(ellipse->basePoint.x, ellipse->basePoint.y, ellipse->basePoint.z);
        const QVector3D local = flattenToDrawingPlane(worldPoint) - center;
        const double x = QVector3D::dotProduct(local, majorUnit);
        const double y = QVector3D::dotProduct(local, minorUnit);
        const double cosValue = x / majorLength;
        const double sinValue = y / minorLength;
        parameter = std::atan2(sinValue, cosValue);
        return true;
    }

    // 平移一个原生 DRW 坐标
    // @param point 待修改坐标
    // @param delta 平移量
    void translateCoord(DRW_Coord& point, const QVector3D& delta)
    {
        point.x += delta.x();
        point.y += delta.y();
        point.z += delta.z();
    }

    // 按实体类型平移原生 DXF 实体几何
    // @param entity 待平移实体
    // @param delta 平移量
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

            if (pointIndex == 0)
            {
                point = QVector3D(line->basePoint.x, line->basePoint.y, 0.0f);
                return true;
            }

            if (pointIndex == 1)
            {
                point = QVector3D(line->secPoint.x, line->secPoint.y, 0.0f);
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

            if (pointIndex == 1)
            {
                QVector3D direction = flattenToDrawingPlane(point - basePoint);

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

    QVector3D normalizedOrZero(const QVector3D& vector);

    std::unique_ptr<DRW_Entity> createPolylineEntity
    (
        const QVector<QVector3D>& points,
        const QVector<double>& bulges,
        const QString& layerName,
        const QColor& color,
        int colorIndex,
        bool closePolyline,
        bool lightweight
    );

    std::unique_ptr<DRW_Entity> createLineEntity
    (
        const QVector3D& startPoint,
        const QVector3D& endPoint,
        const QString& layerName,
        const QColor& color,
        int colorIndex
    );

    void applyEntityColor(DRW_Entity* entity, const QColor& color, int colorIndex);
    int entityColorIndexForCreation(const DRW_Entity* entity);
    QString entityLayerNameForCreation(const DRW_Entity* entity);

    double cross2D(const QVector3D& first, const QVector3D& second)
    {
        return static_cast<double>(first.x()) * second.y() - static_cast<double>(first.y()) * second.x();
    }

    double dot2D(const QVector3D& first, const QVector3D& second)
    {
        return static_cast<double>(first.x()) * second.x() + static_cast<double>(first.y()) * second.y();
    }

    bool pointsNear(const QVector3D& first, const QVector3D& second, double tolerance = 1.0e-4)
    {
        return (flattenToDrawingPlane(first) - flattenToDrawingPlane(second)).lengthSquared() <= tolerance * tolerance;
    }

    QVector3D leftNormal(const QVector3D& direction)
    {
        QVector3D normal(-direction.y(), direction.x(), 0.0f);

        if (normal.lengthSquared() <= kGeometryEpsilon)
        {
            return QVector3D();
        }

        normal.normalize();
        return normal;
    }

    bool lineLineIntersection
    (
        const QVector3D& firstStart,
        const QVector3D& firstEnd,
        const QVector3D& secondStart,
        const QVector3D& secondEnd,
        QVector3D& intersection,
        double* firstParameter = nullptr,
        double* secondParameter = nullptr
    )
    {
        const QVector3D p = flattenToDrawingPlane(firstStart);
        const QVector3D r = flattenToDrawingPlane(firstEnd) - p;
        const QVector3D q = flattenToDrawingPlane(secondStart);
        const QVector3D s = flattenToDrawingPlane(secondEnd) - q;
        const double denominator = cross2D(r, s);

        if (std::abs(denominator) <= kGeometryEpsilon)
        {
            return false;
        }

        const QVector3D qMinusP = q - p;
        const double t = cross2D(qMinusP, s) / denominator;
        const double u = cross2D(qMinusP, r) / denominator;
        intersection = p + r * static_cast<float>(t);

        if (firstParameter != nullptr)
        {
            *firstParameter = t;
        }

        if (secondParameter != nullptr)
        {
            *secondParameter = u;
        }

        return true;
    }

    QVector3D reflectPointAcrossLine(const QVector3D& point, const QVector3D& lineStart, const QVector3D& lineEnd)
    {
        const QVector3D planarPoint = flattenToDrawingPlane(point);
        const QVector3D planarLineStart = flattenToDrawingPlane(lineStart);
        QVector3D lineDirection = flattenToDrawingPlane(lineEnd) - planarLineStart;

        if (lineDirection.lengthSquared() <= kGeometryEpsilon)
        {
            return planarPoint;
        }

        lineDirection.normalize();
        const QVector3D relative = planarPoint - planarLineStart;
        const float projectionLength = QVector3D::dotProduct(relative, lineDirection);
        const QVector3D projectionPoint = planarLineStart + lineDirection * projectionLength;
        return projectionPoint * 2.0f - planarPoint;
    }

    bool mirrorEntityGeometry(DRW_Entity* entity, const QVector3D& lineStart, const QVector3D& lineEnd)
    {
        if (entity == nullptr)
        {
            return false;
        }

        switch (entity->eType)
        {
        case DRW::ETYPE::POINT:
        {
            DRW_Point* pointEntity = static_cast<DRW_Point*>(entity);
            const QVector3D mirrored = reflectPointAcrossLine(QVector3D(pointEntity->basePoint.x, pointEntity->basePoint.y, pointEntity->basePoint.z), lineStart, lineEnd);
            pointEntity->basePoint.x = mirrored.x();
            pointEntity->basePoint.y = mirrored.y();
            pointEntity->basePoint.z = mirrored.z();
            return true;
        }
        case DRW::ETYPE::LINE:
        {
            DRW_Line* line = static_cast<DRW_Line*>(entity);
            const QVector3D mirroredStart = reflectPointAcrossLine(QVector3D(line->basePoint.x, line->basePoint.y, line->basePoint.z), lineStart, lineEnd);
            const QVector3D mirroredEnd = reflectPointAcrossLine(QVector3D(line->secPoint.x, line->secPoint.y, line->secPoint.z), lineStart, lineEnd);
            line->basePoint.x = mirroredStart.x();
            line->basePoint.y = mirroredStart.y();
            line->secPoint.x = mirroredEnd.x();
            line->secPoint.y = mirroredEnd.y();
            return true;
        }
        case DRW::ETYPE::XLINE:
        {
            DRW_Xline* xline = static_cast<DRW_Xline*>(entity);
            const QVector3D basePoint(xline->basePoint.x, xline->basePoint.y, xline->basePoint.z);
            const QVector3D direction = normalizedXlineDirection(xline);
            const QVector3D mirroredBase = reflectPointAcrossLine(basePoint, lineStart, lineEnd);
            const QVector3D mirroredDirectionPoint = reflectPointAcrossLine(basePoint + direction, lineStart, lineEnd);
            QVector3D mirroredDirection = mirroredDirectionPoint - mirroredBase;

            if (mirroredDirection.lengthSquared() <= kGeometryEpsilon)
            {
                return false;
            }

            mirroredDirection.normalize();
            xline->basePoint.x = mirroredBase.x();
            xline->basePoint.y = mirroredBase.y();
            xline->secPoint.x = mirroredDirection.x();
            xline->secPoint.y = mirroredDirection.y();
            xline->secPoint.z = mirroredDirection.z();
            return true;
        }
        case DRW::ETYPE::CIRCLE:
        {
            DRW_Circle* circle = static_cast<DRW_Circle*>(entity);
            const QVector3D mirroredCenter = reflectPointAcrossLine(QVector3D(circle->basePoint.x, circle->basePoint.y, circle->basePoint.z), lineStart, lineEnd);
            circle->basePoint.x = mirroredCenter.x();
            circle->basePoint.y = mirroredCenter.y();
            circle->basePoint.z = mirroredCenter.z();
            return true;
        }
        case DRW::ETYPE::ARC:
        {
            DRW_Arc* arc = static_cast<DRW_Arc*>(entity);
            const QVector3D center(arc->basePoint.x, arc->basePoint.y, arc->basePoint.z);
            const QVector3D startPoint = flattenToDrawingPlane(arcPointAt(arc, arc->staangle));
            const QVector3D endPoint = flattenToDrawingPlane(arcPointAt(arc, arc->endangle));
            const QVector3D mirroredCenter = reflectPointAcrossLine(center, lineStart, lineEnd);
            const QVector3D mirroredStartPoint = reflectPointAcrossLine(startPoint, lineStart, lineEnd);
            const QVector3D mirroredEndPoint = reflectPointAcrossLine(endPoint, lineStart, lineEnd);
            arc->basePoint.x = mirroredCenter.x();
            arc->basePoint.y = mirroredCenter.y();
            arc->basePoint.z = mirroredCenter.z();
            arc->staangle = std::atan2(mirroredEndPoint.y() - mirroredCenter.y(), mirroredEndPoint.x() - mirroredCenter.x());
            arc->endangle = std::atan2(mirroredStartPoint.y() - mirroredCenter.y(), mirroredStartPoint.x() - mirroredCenter.x());

            while (arc->endangle <= arc->staangle)
            {
                arc->endangle += kTwoPi;
            }

            return true;
        }
        case DRW::ETYPE::ELLIPSE:
        {
            DRW_Ellipse* ellipse = static_cast<DRW_Ellipse*>(entity);
            const QVector3D center(ellipse->basePoint.x, ellipse->basePoint.y, ellipse->basePoint.z);
            const QVector3D majorEnd = center + QVector3D(ellipse->secPoint.x, ellipse->secPoint.y, ellipse->secPoint.z);
            const QVector3D mirroredCenter = reflectPointAcrossLine(center, lineStart, lineEnd);
            const QVector3D mirroredMajorEnd = reflectPointAcrossLine(majorEnd, lineStart, lineEnd);
            const QVector3D mirroredAxis = mirroredMajorEnd - mirroredCenter;

            if (mirroredAxis.lengthSquared() <= kGeometryEpsilon)
            {
                return false;
            }

            ellipse->basePoint.x = mirroredCenter.x();
            ellipse->basePoint.y = mirroredCenter.y();
            ellipse->basePoint.z = mirroredCenter.z();
            ellipse->secPoint.x = mirroredAxis.x();
            ellipse->secPoint.y = mirroredAxis.y();
            ellipse->secPoint.z = mirroredAxis.z();
            std::swap(ellipse->staparam, ellipse->endparam);
            return true;
        }
        case DRW::ETYPE::LWPOLYLINE:
        {
            DRW_LWPolyline* polyline = static_cast<DRW_LWPolyline*>(entity);

            for (const std::shared_ptr<DRW_Vertex2D>& vertex : polyline->vertlist)
            {
                if (vertex == nullptr)
                {
                    return false;
                }

                const QVector3D mirrored = reflectPointAcrossLine(QVector3D(vertex->x, vertex->y, polyline->elevation), lineStart, lineEnd);
                vertex->x = mirrored.x();
                vertex->y = mirrored.y();
                vertex->bulge = -vertex->bulge;
            }

            std::reverse(polyline->vertlist.begin(), polyline->vertlist.end());
            return true;
        }
        case DRW::ETYPE::POLYLINE:
        {
            DRW_Polyline* polyline = static_cast<DRW_Polyline*>(entity);

            for (const std::shared_ptr<DRW_Vertex>& vertex : polyline->vertlist)
            {
                if (vertex == nullptr)
                {
                    return false;
                }

                const QVector3D mirrored = reflectPointAcrossLine(QVector3D(vertex->basePoint.x, vertex->basePoint.y, vertex->basePoint.z), lineStart, lineEnd);
                vertex->basePoint.x = mirrored.x();
                vertex->basePoint.y = mirrored.y();
                vertex->basePoint.z = mirrored.z();
                vertex->bulge = -vertex->bulge;
            }

            std::reverse(polyline->vertlist.begin(), polyline->vertlist.end());
            return true;
        }
        default:
            break;
        }

        return false;
    }

    bool entityHasOnlyLinearSegments(const DRW_Entity* entity)
    {
        if (entity == nullptr)
        {
            return false;
        }

        if (entity->eType == DRW::ETYPE::LINE)
        {
            return true;
        }

        if (entity->eType == DRW::ETYPE::LWPOLYLINE)
        {
            const DRW_LWPolyline* polyline = static_cast<const DRW_LWPolyline*>(entity);

            for (const std::shared_ptr<DRW_Vertex2D>& vertex : polyline->vertlist)
            {
                if (vertex != nullptr && std::abs(vertex->bulge) > kGeometryEpsilon)
                {
                    return false;
                }
            }

            return polyline->vertlist.size() >= 2;
        }

        if (entity->eType == DRW::ETYPE::POLYLINE)
        {
            const DRW_Polyline* polyline = static_cast<const DRW_Polyline*>(entity);

            for (const std::shared_ptr<DRW_Vertex>& vertex : polyline->vertlist)
            {
                if (vertex != nullptr && std::abs(vertex->bulge) > kGeometryEpsilon)
                {
                    return false;
                }
            }

            return polyline->vertlist.size() >= 2;
        }

        return false;
    }

    bool extractLinearEntityPoints(const DRW_Entity* entity, QVector<QVector3D>& points, bool& closed)
    {
        points.clear();
        closed = false;

        if (entity == nullptr || !entityHasOnlyLinearSegments(entity))
        {
            return false;
        }

        if (entity->eType == DRW::ETYPE::LINE)
        {
            const DRW_Line* line = static_cast<const DRW_Line*>(entity);
            points = { QVector3D(line->basePoint.x, line->basePoint.y, 0.0f), QVector3D(line->secPoint.x, line->secPoint.y, 0.0f) };
            return true;
        }

        if (entity->eType == DRW::ETYPE::LWPOLYLINE)
        {
            const DRW_LWPolyline* polyline = static_cast<const DRW_LWPolyline*>(entity);
            closed = (polyline->flags & 1) != 0;

            for (const std::shared_ptr<DRW_Vertex2D>& vertex : polyline->vertlist)
            {
                if (vertex == nullptr)
                {
                    return false;
                }

                points.append(QVector3D(static_cast<float>(vertex->x), static_cast<float>(vertex->y), 0.0f));
            }

            return points.size() >= 2;
        }

        const DRW_Polyline* polyline = static_cast<const DRW_Polyline*>(entity);
        closed = (polyline->flags & 1) != 0;

        for (const std::shared_ptr<DRW_Vertex>& vertex : polyline->vertlist)
        {
            if (vertex == nullptr)
            {
                return false;
            }

            points.append(QVector3D(vertex->basePoint.x, vertex->basePoint.y, 0.0f));
        }

        return points.size() >= 2;
    }

    int entityColorIndexForCreation(const DRW_Entity* entity)
    {
        if (entity == nullptr)
        {
            return 256;
        }

        return entity->color24 >= 0 ? -1 : entity->color;
    }

    QString entityLayerNameForCreation(const DRW_Entity* entity)
    {
        return entity == nullptr ? QStringLiteral("0") : QString::fromUtf8(entity->layer.c_str());
    }

    std::unique_ptr<DRW_Entity> createOffsetEntity(const CadItem* item, double distance)
    {
        if (item == nullptr || item->m_nativeEntity == nullptr || std::abs(distance) <= kGeometryEpsilon)
        {
            return nullptr;
        }

        const QString layerName = entityLayerNameForCreation(item->m_nativeEntity);
        const QColor color = item->m_color.isValid() ? item->m_color : QColor(Qt::white);
        const int colorIndex = entityColorIndexForCreation(item->m_nativeEntity);

        switch (item->m_type)
        {
        case DRW::ETYPE::LINE:
        {
            const DRW_Line* line = static_cast<const DRW_Line*>(item->m_nativeEntity);
            const QVector3D startPoint(line->basePoint.x, line->basePoint.y, 0.0f);
            const QVector3D endPoint(line->secPoint.x, line->secPoint.y, 0.0f);
            QVector3D direction = endPoint - startPoint;

            if (direction.lengthSquared() <= kGeometryEpsilon)
            {
                return nullptr;
            }

            direction.normalize();
            const QVector3D normal = leftNormal(direction) * static_cast<float>(distance);
            return createLineEntity(startPoint + normal, endPoint + normal, layerName, color, colorIndex);
        }
        case DRW::ETYPE::CIRCLE:
        {
            std::unique_ptr<DRW_Entity> result = cloneEntity(item->m_nativeEntity);

            if (result == nullptr)
            {
                return nullptr;
            }

            DRW_Circle* circle = static_cast<DRW_Circle*>(result.get());
            circle->radious += distance;
            return circle->radious > kGeometryEpsilon ? std::move(result) : nullptr;
        }
        case DRW::ETYPE::ARC:
        {
            std::unique_ptr<DRW_Entity> result = cloneEntity(item->m_nativeEntity);

            if (result == nullptr)
            {
                return nullptr;
            }

            DRW_Arc* arc = static_cast<DRW_Arc*>(result.get());
            arc->radious += distance;
            return arc->radious > kGeometryEpsilon ? std::move(result) : nullptr;
        }
        case DRW::ETYPE::LWPOLYLINE:
        case DRW::ETYPE::POLYLINE:
        {
            QVector<QVector3D> points;
            bool closed = false;

            if (!extractLinearEntityPoints(item->m_nativeEntity, points, closed))
            {
                return nullptr;
            }

            QVector<QVector3D> offsetPoints;
            offsetPoints.reserve(points.size());

            const auto shiftedLine =
                [distance](const QVector3D& startPoint, const QVector3D& endPoint, QVector3D& shiftedStart, QVector3D& shiftedEnd) -> bool
                {
                    QVector3D direction = flattenToDrawingPlane(endPoint) - flattenToDrawingPlane(startPoint);

                    if (direction.lengthSquared() <= kGeometryEpsilon)
                    {
                        return false;
                    }

                    direction.normalize();
                    const QVector3D shift = leftNormal(direction) * static_cast<float>(distance);
                    shiftedStart = flattenToDrawingPlane(startPoint) + shift;
                    shiftedEnd = flattenToDrawingPlane(endPoint) + shift;
                    return true;
                };

            if (closed)
            {
                for (int index = 0; index < points.size(); ++index)
                {
                    QVector3D prevShiftStart;
                    QVector3D prevShiftEnd;
                    QVector3D nextShiftStart;
                    QVector3D nextShiftEnd;

                    if (!shiftedLine(points[(index + points.size() - 1) % points.size()], points[index], prevShiftStart, prevShiftEnd)
                        || !shiftedLine(points[index], points[(index + 1) % points.size()], nextShiftStart, nextShiftEnd))
                    {
                        return nullptr;
                    }

                    QVector3D intersection;

                    if (!lineLineIntersection(prevShiftStart, prevShiftEnd, nextShiftStart, nextShiftEnd, intersection))
                    {
                        intersection = prevShiftEnd;
                    }

                    offsetPoints.append(intersection);
                }
            }
            else
            {
                QVector3D firstShiftStart;
                QVector3D firstShiftEnd;

                if (!shiftedLine(points[0], points[1], firstShiftStart, firstShiftEnd))
                {
                    return nullptr;
                }

                offsetPoints.append(firstShiftStart);

                for (int index = 1; index + 1 < points.size(); ++index)
                {
                    QVector3D prevShiftStart;
                    QVector3D prevShiftEnd;
                    QVector3D nextShiftStart;
                    QVector3D nextShiftEnd;

                    if (!shiftedLine(points[index - 1], points[index], prevShiftStart, prevShiftEnd)
                        || !shiftedLine(points[index], points[index + 1], nextShiftStart, nextShiftEnd))
                    {
                        return nullptr;
                    }

                    QVector3D intersection;

                    if (!lineLineIntersection(prevShiftStart, prevShiftEnd, nextShiftStart, nextShiftEnd, intersection))
                    {
                        intersection = prevShiftEnd;
                    }

                    offsetPoints.append(intersection);
                }

                if (!shiftedLine(points[points.size() - 2], points.back(), firstShiftStart, firstShiftEnd))
                {
                    return nullptr;
                }

                offsetPoints.append(firstShiftEnd);
            }

            return createPolylineEntity(offsetPoints, {}, layerName, color, colorIndex, closed, true);
        }
        default:
            return nullptr;
        }
    }

    bool angleOnArc(const DRW_Arc* arc, double angle)
    {
        if (arc == nullptr)
        {
            return false;
        }

        double startAngle = normalizeAnglePositive(arc->staangle);
        double endAngle = normalizeAnglePositive(arc->endangle);
        double testAngle = normalizeAnglePositive(angle);

        while (endAngle <= startAngle)
        {
            endAngle += kTwoPi;
        }

        while (testAngle < startAngle)
        {
            testAngle += kTwoPi;
        }

        return testAngle >= startAngle - kGeometryEpsilon && testAngle <= endAngle + kGeometryEpsilon;
    }

    bool lineCircleIntersections(const QVector3D& lineStart, const QVector3D& lineEnd, const QVector3D& center, double radius, QVector<QVector3D>& intersections, QVector<double>* parameters = nullptr)
    {
        intersections.clear();

        if (parameters != nullptr)
        {
            parameters->clear();
        }

        const QVector3D d = flattenToDrawingPlane(lineEnd) - flattenToDrawingPlane(lineStart);
        const QVector3D f = flattenToDrawingPlane(lineStart) - flattenToDrawingPlane(center);
        const double a = dot2D(d, d);
        const double b = 2.0 * dot2D(f, d);
        const double c = dot2D(f, f) - radius * radius;
        const double discriminant = b * b - 4.0 * a * c;

        if (a <= kGeometryEpsilon || discriminant < -kGeometryEpsilon)
        {
            return false;
        }

        const double root = std::sqrt(std::max(0.0, discriminant));
        const double firstT = (-b - root) / (2.0 * a);
        const double secondT = (-b + root) / (2.0 * a);
        intersections.append(flattenToDrawingPlane(lineStart) + d * static_cast<float>(firstT));

        if (parameters != nullptr)
        {
            parameters->append(firstT);
        }

        if (std::abs(secondT - firstT) > kGeometryEpsilon)
        {
            intersections.append(flattenToDrawingPlane(lineStart) + d * static_cast<float>(secondT));

            if (parameters != nullptr)
            {
                parameters->append(secondT);
            }
        }

        return !intersections.isEmpty();
    }

    bool trimOrExtendLineEntity(DRW_Entity* targetEntity, const DRW_Entity* boundaryEntity, bool startSide, bool trimMode)
    {
        if (targetEntity == nullptr || boundaryEntity == nullptr || targetEntity->eType != DRW::ETYPE::LINE)
        {
            return false;
        }

        DRW_Line* targetLine = static_cast<DRW_Line*>(targetEntity);
        const QVector3D startPoint(targetLine->basePoint.x, targetLine->basePoint.y, 0.0f);
        const QVector3D endPoint(targetLine->secPoint.x, targetLine->secPoint.y, 0.0f);
        QVector<QVector3D> candidates;
        QVector<double> parameters;

        if (boundaryEntity->eType == DRW::ETYPE::LINE)
        {
            const DRW_Line* boundaryLine = static_cast<const DRW_Line*>(boundaryEntity);
            QVector3D intersection;
            double targetParameter = 0.0;

            if (!lineLineIntersection
            (
                startPoint,
                endPoint,
                QVector3D(boundaryLine->basePoint.x, boundaryLine->basePoint.y, 0.0f),
                QVector3D(boundaryLine->secPoint.x, boundaryLine->secPoint.y, 0.0f),
                intersection,
                &targetParameter,
                nullptr
            ))
            {
                return false;
            }

            candidates.append(intersection);
            parameters.append(targetParameter);
        }
        else if (boundaryEntity->eType == DRW::ETYPE::CIRCLE)
        {
            const DRW_Circle* circle = static_cast<const DRW_Circle*>(boundaryEntity);

            if (!lineCircleIntersections(startPoint, endPoint, QVector3D(circle->basePoint.x, circle->basePoint.y, 0.0f), circle->radious, candidates, &parameters))
            {
                return false;
            }
        }
        else if (boundaryEntity->eType == DRW::ETYPE::ARC)
        {
            const DRW_Arc* arc = static_cast<const DRW_Arc*>(boundaryEntity);
            QVector<QVector3D> allIntersections;
            QVector<double> allParameters;

            if (!lineCircleIntersections(startPoint, endPoint, QVector3D(arc->basePoint.x, arc->basePoint.y, 0.0f), arc->radious, allIntersections, &allParameters))
            {
                return false;
            }

            for (int index = 0; index < allIntersections.size(); ++index)
            {
                const QVector3D& candidate = allIntersections.at(index);
                const double angle = std::atan2(candidate.y() - arc->basePoint.y, candidate.x() - arc->basePoint.x);

                if (angleOnArc(arc, angle))
                {
                    candidates.append(candidate);
                    parameters.append(allParameters.at(index));
                }
            }
        }

        if (candidates.isEmpty())
        {
            return false;
        }

        int chosenIndex = -1;
        double bestScore = std::numeric_limits<double>::max();

        for (int index = 0; index < parameters.size(); ++index)
        {
            const double parameter = parameters.at(index);

            if (trimMode)
            {
                if (parameter <= 0.0 || parameter >= 1.0)
                {
                    continue;
                }
            }
            else
            {
                if (startSide && parameter >= 0.0)
                {
                    continue;
                }

                if (!startSide && parameter <= 1.0)
                {
                    continue;
                }
            }

            const double score = startSide ? std::abs(parameter) : std::abs(1.0 - parameter);

            if (score < bestScore)
            {
                bestScore = score;
                chosenIndex = index;
            }
        }

        if (chosenIndex < 0)
        {
            return false;
        }

        const QVector3D chosenPoint = candidates.at(chosenIndex);

        if (startSide)
        {
            targetLine->basePoint.x = chosenPoint.x();
            targetLine->basePoint.y = chosenPoint.y();
        }
        else
        {
            targetLine->secPoint.x = chosenPoint.x();
            targetLine->secPoint.y = chosenPoint.y();
        }

        return (QVector3D(targetLine->secPoint.x, targetLine->secPoint.y, 0.0f) - QVector3D(targetLine->basePoint.x, targetLine->basePoint.y, 0.0f)).lengthSquared() > kGeometryEpsilon;
    }

    QVector3D geometryCenterFromVertices(const QVector<QVector3D>& vertices)
    {
        if (vertices.isEmpty())
        {
            return QVector3D();
        }

        QVector3D minimumPoint = flattenToDrawingPlane(vertices.front());
        QVector3D maximumPoint = minimumPoint;

        for (int index = 1; index < vertices.size(); ++index)
        {
            const QVector3D point = flattenToDrawingPlane(vertices.at(index));
            minimumPoint.setX(std::min(minimumPoint.x(), point.x()));
            minimumPoint.setY(std::min(minimumPoint.y(), point.y()));
            maximumPoint.setX(std::max(maximumPoint.x(), point.x()));
            maximumPoint.setY(std::max(maximumPoint.y(), point.y()));
        }

        return (minimumPoint + maximumPoint) * 0.5f;
    }

    QVector3D itemGeometryCenter(const CadItem* item)
    {
        if (item == nullptr)
        {
            return QVector3D();
        }

        if (!item->m_geometry.vertices.isEmpty())
        {
            return geometryCenterFromVertices(item->m_geometry.vertices);
        }

        if (item->m_nativeEntity == nullptr)
        {
            return QVector3D();
        }

        switch (item->m_nativeEntity->eType)
        {
        case DRW::ETYPE::POINT:
        {
            const DRW_Point* point = static_cast<const DRW_Point*>(item->m_nativeEntity);
            return QVector3D(point->basePoint.x, point->basePoint.y, 0.0f);
        }
        case DRW::ETYPE::LINE:
        {
            const DRW_Line* line = static_cast<const DRW_Line*>(item->m_nativeEntity);
            return (QVector3D(line->basePoint.x, line->basePoint.y, 0.0f) + QVector3D(line->secPoint.x, line->secPoint.y, 0.0f)) * 0.5f;
        }
        case DRW::ETYPE::CIRCLE:
        case DRW::ETYPE::ARC:
        {
            const DRW_Circle* circle = static_cast<const DRW_Circle*>(item->m_nativeEntity);
            return QVector3D(circle->basePoint.x, circle->basePoint.y, 0.0f);
        }
        default:
            return QVector3D();
        }
    }

    std::unique_ptr<DRW_Entity> createStyledLineEntity(const QVector3D& startPoint, const QVector3D& endPoint, const CadItem* sourceItem)
    {
        if (sourceItem == nullptr || sourceItem->m_nativeEntity == nullptr)
        {
            return nullptr;
        }

        const QColor color = sourceItem->m_color.isValid() ? sourceItem->m_color : QColor(Qt::white);
        return createLineEntity
        (
            startPoint,
            endPoint,
            entityLayerNameForCreation(sourceItem->m_nativeEntity),
            color,
            entityColorIndexForCreation(sourceItem->m_nativeEntity)
        );
    }

    std::unique_ptr<DRW_Entity> createStyledArcEntity
    (
        const QVector3D& center,
        double radius,
        double startAngle,
        double endAngle,
        const CadItem* sourceItem
    )
    {
        if (sourceItem == nullptr || sourceItem->m_nativeEntity == nullptr || radius <= kGeometryEpsilon)
        {
            return nullptr;
        }

        while (endAngle <= startAngle)
        {
            endAngle += kTwoPi;
        }

        auto entity = std::make_unique<DRW_Arc>();
        entity->basePoint.x = center.x();
        entity->basePoint.y = center.y();
        entity->basePoint.z = 0.0;
        entity->radious = radius;
        entity->staangle = startAngle;
        entity->endangle = endAngle;
        entity->extPoint.x = 0.0;
        entity->extPoint.y = 0.0;
        entity->extPoint.z = 1.0;
        entity->layer = entityLayerNameForCreation(sourceItem->m_nativeEntity).toUtf8().constData();
        applyEntityColor
        (
            entity.get(),
            sourceItem->m_color.isValid() ? sourceItem->m_color : QColor(Qt::white),
            entityColorIndexForCreation(sourceItem->m_nativeEntity)
        );
        return entity;
    }

    bool setLineEndpoint(DRW_Entity* entity, bool startSide, const QVector3D& point)
    {
        if (entity == nullptr || entity->eType != DRW::ETYPE::LINE)
        {
            return false;
        }

        DRW_Line* line = static_cast<DRW_Line*>(entity);

        if (startSide)
        {
            line->basePoint.x = point.x();
            line->basePoint.y = point.y();
            line->basePoint.z = 0.0;
        }
        else
        {
            line->secPoint.x = point.x();
            line->secPoint.y = point.y();
            line->secPoint.z = 0.0;
        }

        return (QVector3D(line->secPoint.x, line->secPoint.y, 0.0f) - QVector3D(line->basePoint.x, line->basePoint.y, 0.0f)).lengthSquared() > kGeometryEpsilon;
    }

    bool resolveLineCorner
    (
        const CadItem* firstItem,
        const CadItem* secondItem,
        QVector3D& intersection,
        bool& firstUseStart,
        bool& secondUseStart,
        QVector3D& firstKeepDirection,
        QVector3D& secondKeepDirection
    )
    {
        if (firstItem == nullptr
            || secondItem == nullptr
            || firstItem->m_nativeEntity == nullptr
            || secondItem->m_nativeEntity == nullptr
            || firstItem->m_nativeEntity->eType != DRW::ETYPE::LINE
            || secondItem->m_nativeEntity->eType != DRW::ETYPE::LINE)
        {
            return false;
        }

        const DRW_Line* firstLine = static_cast<const DRW_Line*>(firstItem->m_nativeEntity);
        const DRW_Line* secondLine = static_cast<const DRW_Line*>(secondItem->m_nativeEntity);
        const QVector3D firstStart(firstLine->basePoint.x, firstLine->basePoint.y, 0.0f);
        const QVector3D firstEnd(firstLine->secPoint.x, firstLine->secPoint.y, 0.0f);
        const QVector3D secondStart(secondLine->basePoint.x, secondLine->basePoint.y, 0.0f);
        const QVector3D secondEnd(secondLine->secPoint.x, secondLine->secPoint.y, 0.0f);

        if (!lineLineIntersection(firstStart, firstEnd, secondStart, secondEnd, intersection))
        {
            return false;
        }

        firstUseStart = (firstStart - intersection).lengthSquared() <= (firstEnd - intersection).lengthSquared();
        secondUseStart = (secondStart - intersection).lengthSquared() <= (secondEnd - intersection).lengthSquared();

        const QVector3D firstFarPoint = firstUseStart ? firstEnd : firstStart;
        const QVector3D secondFarPoint = secondUseStart ? secondEnd : secondStart;
        firstKeepDirection = normalizedOrZero(firstFarPoint - intersection);
        secondKeepDirection = normalizedOrZero(secondFarPoint - intersection);

        if (firstKeepDirection.lengthSquared() <= kGeometryEpsilon || secondKeepDirection.lengthSquared() <= kGeometryEpsilon)
        {
            return false;
        }

        if (std::abs(cross2D(firstKeepDirection, secondKeepDirection)) <= kGeometryEpsilon)
        {
            return false;
        }

        return true;
    }

    bool buildJoinedPolylineEntity(const QVector<CadItem*>& items, std::unique_ptr<DRW_Entity>& joinedEntity)
    {
        joinedEntity.reset();

        if (items.size() < 2)
        {
            return false;
        }

        QVector<QVector<QVector3D>> chains;
        chains.reserve(items.size());

        for (CadItem* item : items)
        {
            if (item == nullptr || item->m_nativeEntity == nullptr)
            {
                return false;
            }

            QVector<QVector3D> points;
            bool closed = false;

            if (!extractLinearEntityPoints(item->m_nativeEntity, points, closed) || closed || points.size() < 2)
            {
                return false;
            }

            if (pointsNear(points.front(), points.back()))
            {
                points.removeLast();
            }

            chains.append(points);
        }

        QVector<QVector3D> mergedPoints = chains.front();
        chains.removeFirst();

        while (!chains.isEmpty())
        {
            bool merged = false;

            for (int index = 0; index < chains.size(); ++index)
            {
                QVector<QVector3D> candidate = chains.at(index);

                if (pointsNear(mergedPoints.back(), candidate.front()))
                {
                    for (int pointIndex = 1; pointIndex < candidate.size(); ++pointIndex)
                    {
                        mergedPoints.append(candidate.at(pointIndex));
                    }

                    chains.removeAt(index);
                    merged = true;
                    break;
                }

                if (pointsNear(mergedPoints.back(), candidate.back()))
                {
                    std::reverse(candidate.begin(), candidate.end());

                    for (int pointIndex = 1; pointIndex < candidate.size(); ++pointIndex)
                    {
                        mergedPoints.append(candidate.at(pointIndex));
                    }

                    chains.removeAt(index);
                    merged = true;
                    break;
                }

                if (pointsNear(mergedPoints.front(), candidate.back()))
                {
                    QVector<QVector3D> newPoints = candidate;
                    newPoints.removeLast();
                    newPoints += mergedPoints;
                    mergedPoints = newPoints;
                    chains.removeAt(index);
                    merged = true;
                    break;
                }

                if (pointsNear(mergedPoints.front(), candidate.front()))
                {
                    std::reverse(candidate.begin(), candidate.end());
                    candidate.removeLast();
                    candidate += mergedPoints;
                    mergedPoints = candidate;
                    chains.removeAt(index);
                    merged = true;
                    break;
                }
            }

            if (!merged)
            {
                return false;
            }
        }

        bool closePolyline = false;

        if (mergedPoints.size() >= 3 && pointsNear(mergedPoints.front(), mergedPoints.back()))
        {
            mergedPoints.removeLast();
            closePolyline = true;
        }

        const CadItem* styleSource = items.front();
        joinedEntity = createPolylineEntity
        (
            mergedPoints,
            {},
            entityLayerNameForCreation(styleSource->m_nativeEntity),
            styleSource->m_color.isValid() ? styleSource->m_color : QColor(Qt::white),
            entityColorIndexForCreation(styleSource->m_nativeEntity),
            closePolyline,
            true
        );
        return joinedEntity != nullptr;
    }

    bool buildChamferReplacementEntities
    (
        const CadItem* firstItem,
        const CadItem* secondItem,
        double firstDistance,
        double secondDistance,
        std::vector<std::unique_ptr<DRW_Entity>>& replacements
    )
    {
        replacements.clear();

        if (firstDistance < 0.0 || secondDistance < 0.0)
        {
            return false;
        }

        QVector3D intersection;
        bool firstUseStart = false;
        bool secondUseStart = false;
        QVector3D firstKeepDirection;
        QVector3D secondKeepDirection;

        if (!resolveLineCorner(firstItem, secondItem, intersection, firstUseStart, secondUseStart, firstKeepDirection, secondKeepDirection))
        {
            return false;
        }

        const QVector3D firstChamferPoint = intersection + firstKeepDirection * static_cast<float>(firstDistance);
        const QVector3D secondChamferPoint = intersection + secondKeepDirection * static_cast<float>(secondDistance);
        std::unique_ptr<DRW_Entity> firstReplacement = cloneEntity(firstItem->m_nativeEntity);
        std::unique_ptr<DRW_Entity> secondReplacement = cloneEntity(secondItem->m_nativeEntity);

        if (firstReplacement == nullptr
            || secondReplacement == nullptr
            || !setLineEndpoint(firstReplacement.get(), firstUseStart, firstChamferPoint)
            || !setLineEndpoint(secondReplacement.get(), secondUseStart, secondChamferPoint))
        {
            return false;
        }

        std::unique_ptr<DRW_Entity> bridge = createStyledLineEntity(firstChamferPoint, secondChamferPoint, firstItem);

        if (bridge == nullptr)
        {
            return false;
        }

        replacements.push_back(std::move(firstReplacement));
        replacements.push_back(std::move(secondReplacement));
        replacements.push_back(std::move(bridge));
        return true;
    }

    bool buildFilletReplacementEntities
    (
        const CadItem* firstItem,
        const CadItem* secondItem,
        double radius,
        std::vector<std::unique_ptr<DRW_Entity>>& replacements
    )
    {
        replacements.clear();

        if (radius <= kGeometryEpsilon)
        {
            return false;
        }

        QVector3D intersection;
        bool firstUseStart = false;
        bool secondUseStart = false;
        QVector3D firstKeepDirection;
        QVector3D secondKeepDirection;

        if (!resolveLineCorner(firstItem, secondItem, intersection, firstUseStart, secondUseStart, firstKeepDirection, secondKeepDirection))
        {
            return false;
        }

        const double angle = std::acos(std::clamp(dot2D(firstKeepDirection, secondKeepDirection), -1.0, 1.0));

        if (angle <= 1.0e-6 || std::abs(kPi - angle) <= 1.0e-6)
        {
            return false;
        }

        const double trimDistance = radius / std::tan(angle * 0.5);
        const QVector3D firstTangentPoint = intersection + firstKeepDirection * static_cast<float>(trimDistance);
        const QVector3D secondTangentPoint = intersection + secondKeepDirection * static_cast<float>(trimDistance);
        QVector3D bisector = firstKeepDirection + secondKeepDirection;

        if (bisector.lengthSquared() <= kGeometryEpsilon)
        {
            return false;
        }

        bisector.normalize();
        const QVector3D center = intersection + bisector * static_cast<float>(radius / std::sin(angle * 0.5));
        double startAngle = std::atan2(firstTangentPoint.y() - center.y(), firstTangentPoint.x() - center.x());
        double endAngle = std::atan2(secondTangentPoint.y() - center.y(), secondTangentPoint.x() - center.x());
        double sweepAngle = normalizeAnglePositive(endAngle - startAngle);

        if (sweepAngle > kPi)
        {
            std::swap(startAngle, endAngle);
        }

        std::unique_ptr<DRW_Entity> firstReplacement = cloneEntity(firstItem->m_nativeEntity);
        std::unique_ptr<DRW_Entity> secondReplacement = cloneEntity(secondItem->m_nativeEntity);

        if (firstReplacement == nullptr
            || secondReplacement == nullptr
            || !setLineEndpoint(firstReplacement.get(), firstUseStart, firstTangentPoint)
            || !setLineEndpoint(secondReplacement.get(), secondUseStart, secondTangentPoint))
        {
            return false;
        }

        std::unique_ptr<DRW_Entity> arc = createStyledArcEntity(center, radius, startAngle, endAngle, firstItem);

        if (arc == nullptr)
        {
            return false;
        }

        replacements.push_back(std::move(firstReplacement));
        replacements.push_back(std::move(secondReplacement));
        replacements.push_back(std::move(arc));
        return true;
    }

    // 把 QColor 编码为 DXF true color 整数
    // @param color 输入颜色
    // @return 24 位 RGB 整数
    int colorToTrueColor(const QColor& color)
    {
        return (color.red() << 16) | (color.green() << 8) | color.blue();
    }

    // 应用颜色到原生实体
    // @param entity 待修改实体
    // @param color 目标颜色
    // @param colorIndex 可选 ACI 索引，小于 0 时写入 true color
    void applyEntityColor(DRW_Entity* entity, const QColor& color, int colorIndex)
    {
        if (entity == nullptr)
        {
            return;
        }

        if (colorIndex >= 0)
        {
            entity->color = colorIndex;
            entity->color24 = -1;
            return;
        }

        entity->color = DRW::ColorByLayer;
        entity->color24 = colorToTrueColor(color);
    }

    // 由圆心和圆上一点计算半径
    double radiusFromPoints(const QVector3D& center, const QVector3D& point)
    {
        return (point - center).length();
    }

    // 将任意拾取点投影到指定半径的圆上
    QVector3D projectToCircle(const QVector3D& center, const QVector3D& radiusPoint, const QVector3D& pickPoint)
    {
        const double radius = radiusFromPoints(center, radiusPoint);

        if (radius <= kGeometryEpsilon)
        {
            return radiusPoint;
        }

        QVector3D direction = pickPoint - center;

        if (direction.lengthSquared() <= kGeometryEpsilon)
        {
            direction = radiusPoint - center;
        }

        if (direction.lengthSquared() <= kGeometryEpsilon)
        {
            return radiusPoint;
        }

        direction.normalize();
        return center + direction * static_cast<float>(radius);
    }

    // 安全归一化向量，退化时返回零向量
    QVector3D normalizedOrZero(const QVector3D& vector)
    {
        if (vector.lengthSquared() <= kGeometryEpsilon)
        {
            return QVector3D();
        }

        QVector3D normalized = vector;
        normalized.normalize();
        return normalized;
    }

    // 由起点、终点和 bulge 反推出圆弧圆心
    QVector3D bulgeArcCenter(const QVector3D& startPoint, const QVector3D& endPoint, double bulge, bool* valid = nullptr)
    {
        const QVector3D chord = endPoint - startPoint;
        const double chordLength = chord.length();

        if (valid != nullptr)
        {
            *valid = false;
        }

        if (chordLength <= kGeometryEpsilon || std::abs(bulge) <= kGeometryEpsilon)
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

    // 计算当前多段线末段的切向，供圆弧续接使用
    QVector3D polylineEndTangent
    (
        const QVector<QVector3D>& points,
        const QVector<double>& bulges
    )
    {
        if (points.size() < 2)
        {
            return QVector3D();
        }

        const QVector3D startPoint = flattenToDrawingPlane(points[points.size() - 2]);
        const QVector3D endPoint = flattenToDrawingPlane(points.back());
        const double bulge = bulges.size() >= points.size() - 1 ? bulges[points.size() - 2] : 0.0;

        if (std::abs(bulge) <= kGeometryEpsilon)
        {
            return normalizedOrZero(endPoint - startPoint);
        }

        bool hasCenter = false;
        const QVector3D center = bulgeArcCenter(startPoint, endPoint, bulge, &hasCenter);

        if (!hasCenter)
        {
            return normalizedOrZero(endPoint - startPoint);
        }

        const QVector3D radiusVector = endPoint - center;
        QVector3D tangent;

        if (bulge > 0.0)
        {
            tangent = QVector3D(-radiusVector.y(), radiusVector.x(), 0.0f);
        }
        else
        {
            tangent = QVector3D(radiusVector.y(), -radiusVector.x(), 0.0f);
        }

        return normalizedOrZero(tangent);
    }

    // 根据起点切向和终点反推 bulge
    double bulgeFromTangent(const QVector3D& startPoint, const QVector3D& tangentDirection, const QVector3D& endPoint)
    {
        const QVector3D planarStartPoint = flattenToDrawingPlane(startPoint);
        const QVector3D planarEndPoint = flattenToDrawingPlane(endPoint);
        const QVector3D planarTangent = normalizedOrZero(QVector3D(tangentDirection.x(), tangentDirection.y(), 0.0f));
        const QVector3D chordVector = planarEndPoint - planarStartPoint;

        if (planarTangent.lengthSquared() <= kGeometryEpsilon || chordVector.lengthSquared() <= kGeometryEpsilon)
        {
            return 0.0;
        }

        const double dotValue = QVector3D::dotProduct(planarTangent, chordVector);
        const double crossValue = planarTangent.x() * chordVector.y() - planarTangent.y() * chordVector.x();
        const double alpha = std::atan2(crossValue, dotValue);

        if (std::abs(std::abs(alpha) - kPi) <= 1.0e-6)
        {
            return std::numeric_limits<double>::infinity();
        }

        return std::tan(alpha * 0.5);
    }

    std::unique_ptr<DRW_Entity> createPolylineEntity
    (
        const QVector<QVector3D>& points,
        const QVector<double>& bulges,
        const QString& layerName,
        const QColor& color,
        int colorIndex,
        bool closePolyline,
        bool lightweight
    );

    std::unique_ptr<DRW_Entity> createLineEntity
    (
        const QVector3D& startPoint,
        const QVector3D& endPoint,
        const QString& layerName,
        const QColor& color,
        int colorIndex
    );

    void applyEntityColor(DRW_Entity* entity, const QColor& color, int colorIndex);
    int entityColorIndexForCreation(const DRW_Entity* entity);
    QString entityLayerNameForCreation(const DRW_Entity* entity);

    // 创建点实体
    std::unique_ptr<DRW_Entity> createPointEntity
    (
        const QVector3D& position,
        const QString& layerName,
        const QColor& color,
        int colorIndex
    )
    {
        const QVector3D planarPosition = flattenToDrawingPlane(position);
        auto entity = std::make_unique<DRW_Point>();
        entity->basePoint.x = planarPosition.x();
        entity->basePoint.y = planarPosition.y();
        entity->basePoint.z = 0.0;
        entity->layer = layerName.trimmed().isEmpty() ? "0" : layerName.toUtf8().constData();
        applyEntityColor(entity.get(), color, colorIndex);
        return entity;
    }

    // 创建直线实体
    std::unique_ptr<DRW_Entity> createLineEntity
    (
        const QVector3D& startPoint,
        const QVector3D& endPoint,
        const QString& layerName,
        const QColor& color,
        int colorIndex
    )
    {
        const QVector3D planarStartPoint = flattenToDrawingPlane(startPoint);
        const QVector3D planarEndPoint = flattenToDrawingPlane(endPoint);

        if ((planarEndPoint - planarStartPoint).lengthSquared() <= kGeometryEpsilon)
        {
            return nullptr;
        }

        auto entity = std::make_unique<DRW_Line>();
        entity->basePoint.x = planarStartPoint.x();
        entity->basePoint.y = planarStartPoint.y();
        entity->basePoint.z = 0.0;
        entity->secPoint.x = planarEndPoint.x();
        entity->secPoint.y = planarEndPoint.y();
        entity->secPoint.z = 0.0;
        entity->layer = layerName.trimmed().isEmpty() ? "0" : layerName.toUtf8().constData();
        applyEntityColor(entity.get(), color, colorIndex);
        return entity;
    }

    std::unique_ptr<DRW_Entity> createXlineEntity
    (
        const QVector3D& basePoint,
        const QVector3D& throughPoint,
        const QString& layerName,
        const QColor& color,
        int colorIndex
    )
    {
        const QVector3D planarBasePoint = flattenToDrawingPlane(basePoint);
        QVector3D direction = flattenToDrawingPlane(throughPoint) - planarBasePoint;

        if (direction.lengthSquared() <= kGeometryEpsilon)
        {
            return nullptr;
        }

        direction.normalize();

        auto entity = std::make_unique<DRW_Xline>();
        entity->basePoint.x = planarBasePoint.x();
        entity->basePoint.y = planarBasePoint.y();
        entity->basePoint.z = 0.0;
        entity->secPoint.x = direction.x();
        entity->secPoint.y = direction.y();
        entity->secPoint.z = 0.0;
        entity->layer = layerName.trimmed().isEmpty() ? "0" : layerName.toUtf8().constData();
        applyEntityColor(entity.get(), color, colorIndex);
        return entity;
    }

    std::unique_ptr<DRW_Entity> createRectangleEntity
    (
        const QVector3D& firstCorner,
        const QVector3D& secondCorner,
        const QString& layerName,
        const QColor& color,
        int colorIndex
    )
    {
        const QVector3D planarFirstCorner = flattenToDrawingPlane(firstCorner);
        const QVector3D planarSecondCorner = flattenToDrawingPlane(secondCorner);

        if (std::abs(planarSecondCorner.x() - planarFirstCorner.x()) <= kGeometryEpsilon
            || std::abs(planarSecondCorner.y() - planarFirstCorner.y()) <= kGeometryEpsilon)
        {
            return nullptr;
        }

        QVector<QVector3D> points;
        points.reserve(4);
        points.append(planarFirstCorner);
        points.append(QVector3D(planarSecondCorner.x(), planarFirstCorner.y(), 0.0f));
        points.append(planarSecondCorner);
        points.append(QVector3D(planarFirstCorner.x(), planarSecondCorner.y(), 0.0f));

        return createPolylineEntity(points, {}, layerName, color, colorIndex, true, true);
    }

    std::unique_ptr<DRW_Entity> createPolygonEntity
    (
        const QVector3D& center,
        const QVector3D& radiusPoint,
        int sideCount,
        bool circumscribedAboutCircle,
        const QString& layerName,
        const QColor& color,
        int colorIndex
    )
    {
        const QVector3D planarCenter = flattenToDrawingPlane(center);
        const QVector3D planarRadiusPoint = flattenToDrawingPlane(radiusPoint);
        const QVector3D radiusVector = planarRadiusPoint - planarCenter;
        const double referenceRadius = radiusVector.length();

        if (sideCount < 3 || sideCount > 1024 || referenceRadius <= kGeometryEpsilon)
        {
            return nullptr;
        }

        const double stepAngle = kTwoPi / static_cast<double>(sideCount);
        const double baseAngle = std::atan2(radiusVector.y(), radiusVector.x());
        const double startAngle = circumscribedAboutCircle ? (baseAngle + stepAngle * 0.5) : baseAngle;
        const double polygonRadius = circumscribedAboutCircle
            ? (referenceRadius / std::cos(stepAngle * 0.5))
            : referenceRadius;

        if (!std::isfinite(polygonRadius) || polygonRadius <= kGeometryEpsilon)
        {
            return nullptr;
        }

        QVector<QVector3D> points;
        points.reserve(sideCount);

        for (int index = 0; index < sideCount; ++index)
        {
            const double angle = startAngle + stepAngle * static_cast<double>(index);
            points.append
            (
                QVector3D
                (
                    static_cast<float>(planarCenter.x() + polygonRadius * std::cos(angle)),
                    static_cast<float>(planarCenter.y() + polygonRadius * std::sin(angle)),
                    0.0f
                )
            );
        }

        return createPolylineEntity(points, {}, layerName, color, colorIndex, true, true);
    }

    // 创建圆实体
    std::unique_ptr<DRW_Entity> createCircleEntity
    (
        const QVector3D& center,
        const QVector3D& radiusPoint,
        const QString& layerName,
        const QColor& color,
        int colorIndex
    )
    {
        const QVector3D planarCenter = flattenToDrawingPlane(center);
        const QVector3D planarRadiusPoint = flattenToDrawingPlane(radiusPoint);
        const double radius = radiusFromPoints(planarCenter, planarRadiusPoint);

        if (radius <= kGeometryEpsilon)
        {
            return nullptr;
        }

        auto entity = std::make_unique<DRW_Circle>();
        entity->basePoint.x = planarCenter.x();
        entity->basePoint.y = planarCenter.y();
        entity->basePoint.z = 0.0;
        entity->radious = radius;
        entity->extPoint.x = 0.0;
        entity->extPoint.y = 0.0;
        entity->extPoint.z = 1.0;
        entity->layer = layerName.trimmed().isEmpty() ? "0" : layerName.toUtf8().constData();
        applyEntityColor(entity.get(), color, colorIndex);
        return entity;
    }

    // 创建圆弧实体
    std::unique_ptr<DRW_Entity> createArcEntity
    (
        const QVector3D& center,
        const QVector3D& radiusPoint,
        const QVector3D& startAnglePoint,
        const QVector3D& endAnglePoint,
        const QString& layerName,
        const QColor& color,
        int colorIndex
    )
    {
        const QVector3D planarCenter = flattenToDrawingPlane(center);
        const QVector3D planarRadiusPoint = flattenToDrawingPlane(radiusPoint);
        const QVector3D planarStartAnglePoint = flattenToDrawingPlane(startAnglePoint);
        const QVector3D planarEndAnglePoint = flattenToDrawingPlane(endAnglePoint);
        const double radius = radiusFromPoints(planarCenter, planarRadiusPoint);

        if (radius <= kGeometryEpsilon)
        {
            return nullptr;
        }

        const QVector3D startPoint = projectToCircle(planarCenter, planarRadiusPoint, planarStartAnglePoint);
        const QVector3D endPoint = projectToCircle(planarCenter, planarRadiusPoint, planarEndAnglePoint);

        auto entity = std::make_unique<DRW_Arc>();
        entity->basePoint.x = planarCenter.x();
        entity->basePoint.y = planarCenter.y();
        entity->basePoint.z = 0.0;
        entity->radious = radius;
        entity->staangle = std::atan2(startPoint.y() - planarCenter.y(), startPoint.x() - planarCenter.x());
        entity->endangle = std::atan2(endPoint.y() - planarCenter.y(), endPoint.x() - planarCenter.x());
        entity->extPoint.x = 0.0;
        entity->extPoint.y = 0.0;
        entity->extPoint.z = 1.0;

        if (std::abs(entity->endangle - entity->staangle) <= kGeometryEpsilon)
        {
            entity->endangle = entity->staangle + kTwoPi;
        }

        entity->layer = layerName.trimmed().isEmpty() ? "0" : layerName.toUtf8().constData();
        applyEntityColor(entity.get(), color, colorIndex);
        return entity;
    }

    // 创建椭圆实体
    std::unique_ptr<DRW_Entity> createEllipseEntity
    (
        const QVector3D& center,
        const QVector3D& majorAxisPoint,
        const QVector3D& ratioPoint,
        const QString& layerName,
        const QColor& color,
        int colorIndex
    )
    {
        const QVector3D planarCenter = flattenToDrawingPlane(center);
        const QVector3D planarMajorAxisPoint = flattenToDrawingPlane(majorAxisPoint);
        const QVector3D planarRatioPoint = flattenToDrawingPlane(ratioPoint);
        const QVector3D majorAxis = planarMajorAxisPoint - planarCenter;
        const double majorLength = majorAxis.length();

        if (majorLength <= kGeometryEpsilon)
        {
            return nullptr;
        }

        const QVector3D toRatioPoint = planarRatioPoint - planarCenter;
        const double projectedLength = QVector3D::dotProduct(toRatioPoint, majorAxis) / majorLength;
        const double minorSquared = std::max(0.0, static_cast<double>(toRatioPoint.lengthSquared()) - projectedLength * projectedLength);
        const double minorLength = std::sqrt(minorSquared);

        if (minorLength <= kGeometryEpsilon)
        {
            return nullptr;
        }

        auto entity = std::make_unique<DRW_Ellipse>();
        entity->basePoint.x = planarCenter.x();
        entity->basePoint.y = planarCenter.y();
        entity->basePoint.z = 0.0;
        entity->secPoint.x = majorAxis.x();
        entity->secPoint.y = majorAxis.y();
        entity->secPoint.z = 0.0;
        entity->ratio = minorLength / majorLength;
        entity->staparam = 0.0;
        entity->endparam = 2.0 * kPi;
        entity->extPoint.x = 0.0;
        entity->extPoint.y = 0.0;
        entity->extPoint.z = 1.0;
        entity->layer = layerName.trimmed().isEmpty() ? "0" : layerName.toUtf8().constData();
        applyEntityColor(entity.get(), color, colorIndex);
        return entity;
    }

    // 创建多段线或轻量多段线实体
    std::unique_ptr<DRW_Entity> createPolylineEntity
    (
        const QVector<QVector3D>& points,
        const QVector<double>& bulges,
        const QString& layerName,
        const QColor& color,
        int colorIndex,
        bool closePolyline,
        bool lightweight
    )
    {
        if (points.size() < 2)
        {
            return nullptr;
        }

        if (lightweight)
        {
            auto entity = std::make_unique<DRW_LWPolyline>();
            entity->flags = closePolyline ? 1 : 0;
            entity->elevation = 0.0;

            for (const QVector3D& point : points)
            {
                const QVector3D planarPoint = flattenToDrawingPlane(point);
                std::shared_ptr<DRW_Vertex2D> vertex = std::make_shared<DRW_Vertex2D>();
                vertex->x = planarPoint.x();
                vertex->y = planarPoint.y();
                entity->vertlist.push_back(vertex);
            }

            for (int i = 0; i < entity->vertlist.size(); ++i)
            {
                entity->vertlist[static_cast<size_t>(i)]->bulge = i < bulges.size() ? bulges[i] : 0.0;
            }

            entity->vertexnum = static_cast<int>(entity->vertlist.size());
            entity->layer = layerName.trimmed().isEmpty() ? "0" : layerName.toUtf8().constData();
            applyEntityColor(entity.get(), color, colorIndex);
            return entity;
        }

        auto entity = std::make_unique<DRW_Polyline>();
        entity->flags = closePolyline ? 1 : 0;

        for (const QVector3D& point : points)
        {
            const QVector3D planarPoint = flattenToDrawingPlane(point);
            std::shared_ptr<DRW_Vertex> vertex = std::make_shared<DRW_Vertex>();
            vertex->basePoint.x = planarPoint.x();
            vertex->basePoint.y = planarPoint.y();
            vertex->basePoint.z = 0.0;
            entity->vertlist.push_back(vertex);
        }

        for (int i = 0; i < entity->vertlist.size(); ++i)
        {
            entity->vertlist[static_cast<size_t>(i)]->bulge = i < bulges.size() ? bulges[i] : 0.0;
        }

        entity->vertexcount = static_cast<int>(entity->vertlist.size());
        entity->layer = layerName.trimmed().isEmpty() ? "0" : layerName.toUtf8().constData();
        applyEntityColor(entity.get(), color, colorIndex);
        return entity;
    }
}

// 添加实体命令：
// 负责把新建原生实体与对应 CadItem 一起插入文档，并支持撤销恢复。
class AddEntityCommand final : public CadEditer::EditCommand
{
public:
    AddEntityCommand(CadDocument* document, std::unique_ptr<DRW_Entity> entity)
        : m_document(document)
        , m_entity(std::move(entity))
    {
    }

    bool execute() override
    {
        // 第一次执行时创建 CadItem，后续 redo 复用已保存对象
        if (m_document == nullptr || m_entity == nullptr)
        {
            return false;
        }

        if (m_item == nullptr)
        {
            m_item = CadDocument::createCadItemForEntity(m_entity.get());
        }

        if (m_item == nullptr)
        {
            return false;
        }

        m_itemPtr = m_item.get();
        return m_document->appendEntity(std::move(m_entity), std::move(m_item)) != nullptr;
    }

    bool undo() override
    {
        // 撤销时把实体和图元从文档中整体取回，以便后续 redo 重新插入
        if (m_document == nullptr || m_itemPtr == nullptr)
        {
            return false;
        }

        auto [entity, item] = m_document->takeEntity(m_itemPtr);

        if (entity == nullptr || item == nullptr)
        {
            return false;
        }

        m_entity = std::move(entity);
        m_item = std::move(item);
        m_itemPtr = m_item.get();
        return true;
    }

private:
    // 目标文档
    CadDocument* m_document = nullptr;

    // 待插入或撤销后保存的原生实体
    std::unique_ptr<DRW_Entity> m_entity;

    // 与原生实体对应的图元对象
    std::unique_ptr<CadItem> m_item;

    // 当前图元裸指针，用于与文档接口协作
    CadItem* m_itemPtr = nullptr;
};

class AddEntitiesCommand final : public CadEditer::EditCommand
{
public:
    AddEntitiesCommand(CadDocument* document, std::vector<std::unique_ptr<DRW_Entity>> entities)
        : m_document(document)
        , m_entities(std::move(entities))
        , m_items(m_entities.size())
        , m_itemPtrs(m_entities.size(), nullptr)
    {
    }

    bool execute() override
    {
        if (m_document == nullptr || m_entities.empty())
        {
            return false;
        }

        for (int index = 0; index < static_cast<int>(m_entities.size()); ++index)
        {
            if (m_entities[index] == nullptr)
            {
                return false;
            }

            if (m_items[index] == nullptr)
            {
                m_items[index] = CadDocument::createCadItemForEntity(m_entities[index].get());
            }

            if (m_items[index] == nullptr)
            {
                return false;
            }

            m_itemPtrs[index] = m_items[index].get();

            if (m_document->appendEntity(std::move(m_entities[index]), std::move(m_items[index])) == nullptr)
            {
                return false;
            }
        }

        return true;
    }

    bool undo() override
    {
        if (m_document == nullptr)
        {
            return false;
        }

        for (int index = static_cast<int>(m_itemPtrs.size()) - 1; index >= 0; --index)
        {
            if (m_itemPtrs[index] == nullptr)
            {
                return false;
            }

            auto [entity, item] = m_document->takeEntity(m_itemPtrs[index]);

            if (entity == nullptr || item == nullptr)
            {
                return false;
            }

            m_entities[index] = std::move(entity);
            m_items[index] = std::move(item);
            m_itemPtrs[index] = m_items[index].get();
        }

        return true;
    }

private:
    CadDocument* m_document = nullptr;
    std::vector<std::unique_ptr<DRW_Entity>> m_entities;
    std::vector<std::unique_ptr<CadItem>> m_items;
    std::vector<CadItem*> m_itemPtrs;
};

class ReplaceEntitiesCommand final : public CadEditer::EditCommand
{
public:
    ReplaceEntitiesCommand
    (
        CadDocument* document,
        const QVector<CadItem*>& sourceItems,
        std::vector<std::unique_ptr<DRW_Entity>> replacementEntities
    )
        : m_document(document)
        , m_replacementEntities(std::move(replacementEntities))
        , m_replacementItems(m_replacementEntities.size())
        , m_replacementItemPtrs(m_replacementEntities.size(), nullptr)
    {
        QSet<CadItem*> deduplicated;

        for (CadItem* item : sourceItems)
        {
            if (item == nullptr || deduplicated.contains(item))
            {
                continue;
            }

            deduplicated.insert(item);
            m_sourceStates.push_back({ item, {}, {} });
        }
    }

    bool execute() override
    {
        if (m_document == nullptr || m_sourceStates.empty() || m_replacementEntities.empty())
        {
            return false;
        }

        for (SourceState& state : m_sourceStates)
        {
            if (state.itemPtr == nullptr)
            {
                return false;
            }

            auto [entity, item] = m_document->takeEntity(state.itemPtr);

            if (entity == nullptr || item == nullptr)
            {
                return false;
            }

            state.entity = std::move(entity);
            state.item = std::move(item);
            state.itemPtr = state.item.get();
        }

        for (int index = 0; index < static_cast<int>(m_replacementEntities.size()); ++index)
        {
            if (m_replacementEntities[index] == nullptr)
            {
                return false;
            }

            if (m_replacementItems[index] == nullptr)
            {
                m_replacementItems[index] = CadDocument::createCadItemForEntity(m_replacementEntities[index].get());
            }

            if (m_replacementItems[index] == nullptr)
            {
                return false;
            }

            m_replacementItemPtrs[index] = m_replacementItems[index].get();

            if (m_document->appendEntity(std::move(m_replacementEntities[index]), std::move(m_replacementItems[index])) == nullptr)
            {
                return false;
            }
        }

        return true;
    }

    bool undo() override
    {
        if (m_document == nullptr)
        {
            return false;
        }

        for (int index = static_cast<int>(m_replacementItemPtrs.size()) - 1; index >= 0; --index)
        {
            if (m_replacementItemPtrs[index] == nullptr)
            {
                return false;
            }

            auto [entity, item] = m_document->takeEntity(m_replacementItemPtrs[index]);

            if (entity == nullptr || item == nullptr)
            {
                return false;
            }

            m_replacementEntities[index] = std::move(entity);
            m_replacementItems[index] = std::move(item);
            m_replacementItemPtrs[index] = m_replacementItems[index].get();
        }

        for (SourceState& state : m_sourceStates)
        {
            if (state.entity == nullptr || state.item == nullptr)
            {
                return false;
            }

            state.itemPtr = state.item.get();

            if (m_document->appendEntity(std::move(state.entity), std::move(state.item)) == nullptr)
            {
                return false;
            }
        }

        return true;
    }

private:
    struct SourceState
    {
        CadItem* itemPtr = nullptr;
        std::unique_ptr<DRW_Entity> entity;
        std::unique_ptr<CadItem> item;
    };

    CadDocument* m_document = nullptr;
    std::vector<SourceState> m_sourceStates;
    std::vector<std::unique_ptr<DRW_Entity>> m_replacementEntities;
    std::vector<std::unique_ptr<CadItem>> m_replacementItems;
    std::vector<CadItem*> m_replacementItemPtrs;
};

// 删除实体命令：
// 执行时从文档中摘出实体，撤销时再插回原位。
class DeleteEntityCommand final : public CadEditer::EditCommand
{
public:
    DeleteEntityCommand(CadDocument* document, CadItem* item)
        : m_document(document)
        , m_itemPtr(item)
    {
    }

    bool execute() override
    {
        if (m_document == nullptr || m_itemPtr == nullptr)
        {
            return false;
        }

        auto [entity, item] = m_document->takeEntity(m_itemPtr);

        if (entity == nullptr || item == nullptr)
        {
            return false;
        }

        m_entity = std::move(entity);
        m_item = std::move(item);
        m_itemPtr = m_item.get();
        return true;
    }

    bool undo() override
    {
        if (m_document == nullptr || m_entity == nullptr || m_item == nullptr)
        {
            return false;
        }

        m_itemPtr = m_item.get();
        return m_document->appendEntity(std::move(m_entity), std::move(m_item)) != nullptr;
    }

private:
    // 目标文档
    CadDocument* m_document = nullptr;

    // 被删除后缓存的原生实体
    std::unique_ptr<DRW_Entity> m_entity;

    // 被删除后缓存的图元对象
    std::unique_ptr<CadItem> m_item;

    // 当前图元裸指针
    CadItem* m_itemPtr = nullptr;
};

class DeleteEntitiesCommand final : public CadEditer::EditCommand
{
public:
    DeleteEntitiesCommand(CadDocument* document, const QVector<CadItem*>& items)
        : m_document(document)
    {
        QSet<CadItem*> deduplicated;

        for (CadItem* item : items)
        {
            if (item == nullptr || deduplicated.contains(item))
            {
                continue;
            }

            deduplicated.insert(item);
            m_states.push_back({ item, {}, {} });
        }
    }

    bool execute() override
    {
        if (m_document == nullptr || m_states.empty())
        {
            return false;
        }

        for (ItemState& state : m_states)
        {
            if (state.itemPtr == nullptr)
            {
                return false;
            }

            auto [entity, item] = m_document->takeEntity(state.itemPtr);

            if (entity == nullptr || item == nullptr)
            {
                return false;
            }

            state.entity = std::move(entity);
            state.item = std::move(item);
            state.itemPtr = state.item.get();
        }

        return true;
    }

    bool undo() override
    {
        if (m_document == nullptr || m_states.empty())
        {
            return false;
        }

        for (ItemState& state : m_states)
        {
            if (state.entity == nullptr || state.item == nullptr)
            {
                return false;
            }

            state.itemPtr = state.item.get();

            if (m_document->appendEntity(std::move(state.entity), std::move(state.item)) == nullptr)
            {
                return false;
            }
        }

        return true;
    }

private:
    struct ItemState
    {
        CadItem* itemPtr = nullptr;
        std::unique_ptr<DRW_Entity> entity;
        std::unique_ptr<CadItem> item;
    };

    CadDocument* m_document = nullptr;
    std::vector<ItemState> m_states;
};

// 移动实体命令：
// 通过对原生实体做几何平移，再触发文档刷新来实现可撤销移动。
class MoveEntityCommand final : public CadEditer::EditCommand
{
public:
    MoveEntityCommand(CadDocument* document, CadItem* item, const QVector3D& delta)
        : m_document(document)
        , m_item(item)
        , m_delta(delta)
    {
    }

    bool execute() override
    {
        return applyDelta(m_delta);
    }

    bool undo() override
    {
        return applyDelta(-m_delta);
    }

private:
    // 应用一次平移增量；Undo 通过传入相反向量复用同一逻辑
    bool applyDelta(const QVector3D& delta)
    {
        if (m_document == nullptr || m_item == nullptr || !m_document->containsEntity(m_item))
        {
            return false;
        }

        translateEntity(m_item->m_nativeEntity, delta);
        return m_document->refreshEntity(m_item);
    }

private:
    // 目标文档
    CadDocument* m_document = nullptr;

    // 目标实体
    CadItem* m_item = nullptr;

    // 平移增量
    QVector3D m_delta;
};

class MoveEntitiesCommand final : public CadEditer::EditCommand
{
public:
    MoveEntitiesCommand(CadDocument* document, const QVector<CadItem*>& items, const QVector3D& delta)
        : m_document(document)
        , m_items(items)
        , m_delta(delta)
    {
    }

    bool execute() override
    {
        return applyDelta(m_delta);
    }

    bool undo() override
    {
        return applyDelta(-m_delta);
    }

private:
    bool applyDelta(const QVector3D& delta)
    {
        if (m_document == nullptr || m_items.isEmpty())
        {
            return false;
        }

        for (CadItem* item : m_items)
        {
            if (item == nullptr || !m_document->containsEntity(item))
            {
                return false;
            }
        }

        for (CadItem* item : m_items)
        {
            translateEntity(item->m_nativeEntity, delta);

            if (!m_document->refreshEntity(item))
            {
                return false;
            }
        }

        return true;
    }

private:
    CadDocument* m_document = nullptr;
    QVector<CadItem*> m_items;
    QVector3D m_delta;
};

class GripPointEditCommand final : public CadEditer::EditCommand
{
public:
    GripPointEditCommand(CadDocument* document, CadItem* item, int pointIndex, const QVector3D& newPoint)
        : m_document(document)
        , m_item(item)
        , m_pointIndex(pointIndex)
        , m_newPoint(flattenToDrawingPlane(newPoint))
    {
        if (m_item != nullptr)
        {
            m_valid = readEditableControlPoint(m_item, m_pointIndex, m_oldPoint);
        }
    }

    bool execute() override
    {
        return apply(m_newPoint);
    }

    bool undo() override
    {
        return apply(m_oldPoint);
    }

private:
    bool apply(const QVector3D& point)
    {
        if (!m_valid
            || m_document == nullptr
            || m_item == nullptr
            || m_item->m_nativeEntity == nullptr
            || !m_document->containsEntity(m_item))
        {
            return false;
        }

        if (!applyEditableControlPoint(m_item->m_nativeEntity, m_pointIndex, point))
        {
            return false;
        }

        return m_document->refreshEntity(m_item);
    }

private:
    CadDocument* m_document = nullptr;
    CadItem* m_item = nullptr;
    int m_pointIndex = -1;
    QVector3D m_oldPoint;
    QVector3D m_newPoint;
    bool m_valid = false;
};

class CopyEntityCommand final : public CadEditer::EditCommand
{
public:
    CopyEntityCommand(CadDocument* document, CadItem* sourceItem, const QVector3D& delta)
        : m_document(document)
        , m_sourceItem(sourceItem)
        , m_delta(delta)
    {
    }

    bool execute() override
    {
        if (m_document == nullptr || m_sourceItem == nullptr)
        {
            return false;
        }

        if (m_entity == nullptr)
        {
            m_entity = cloneEntity(m_sourceItem->m_nativeEntity);

            if (m_entity == nullptr)
            {
                return false;
            }

            translateEntity(m_entity.get(), m_delta);
        }

        if (m_item == nullptr)
        {
            m_item = CadDocument::createCadItemForEntity(m_entity.get());
        }

        if (m_item == nullptr)
        {
            return false;
        }

        m_itemPtr = m_item.get();
        return m_document->appendEntity(std::move(m_entity), std::move(m_item)) != nullptr;
    }

    bool undo() override
    {
        if (m_document == nullptr || m_itemPtr == nullptr)
        {
            return false;
        }

        auto [entity, item] = m_document->takeEntity(m_itemPtr);

        if (entity == nullptr || item == nullptr)
        {
            return false;
        }

        m_entity = std::move(entity);
        m_item = std::move(item);
        m_itemPtr = m_item.get();
        return true;
    }

private:
    CadDocument* m_document = nullptr;
    CadItem* m_sourceItem = nullptr;
    QVector3D m_delta;
    std::unique_ptr<DRW_Entity> m_entity;
    std::unique_ptr<CadItem> m_item;
    CadItem* m_itemPtr = nullptr;
};

class RotateEntityCommand final : public CadEditer::EditCommand
{
public:
    RotateEntityCommand(CadDocument* document, CadItem* item, const QVector3D& basePoint, double angleDegrees)
        : m_document(document)
        , m_item(item)
        , m_basePoint(basePoint)
        , m_angleDegrees(angleDegrees)
    {
    }

    bool execute() override
    {
        return apply(m_angleDegrees);
    }

    bool undo() override
    {
        return apply(-m_angleDegrees);
    }

private:
    bool apply(double angleDegrees)
    {
        if (m_document == nullptr || m_item == nullptr || !m_document->containsEntity(m_item))
        {
            return false;
        }

        rotateEntity(m_item->m_nativeEntity, m_basePoint, angleDegrees);
        return m_document->refreshEntity(m_item);
    }

private:
    CadDocument* m_document = nullptr;
    CadItem* m_item = nullptr;
    QVector3D m_basePoint;
    double m_angleDegrees = 0.0;
};

class ScaleEntityCommand final : public CadEditer::EditCommand
{
public:
    ScaleEntityCommand(CadDocument* document, CadItem* item, const QVector3D& basePoint, double scaleFactor)
        : m_document(document)
        , m_item(item)
        , m_basePoint(basePoint)
        , m_scaleFactor(scaleFactor)
    {
    }

    bool execute() override
    {
        return apply(m_scaleFactor);
    }

    bool undo() override
    {
        if (std::abs(m_scaleFactor) <= kGeometryEpsilon)
        {
            return false;
        }

        return apply(1.0 / m_scaleFactor);
    }

private:
    bool apply(double scaleFactor)
    {
        if (m_document == nullptr || m_item == nullptr || !m_document->containsEntity(m_item) || scaleFactor <= kGeometryEpsilon)
        {
            return false;
        }

        scaleEntity(m_item->m_nativeEntity, m_basePoint, scaleFactor);
        return m_document->refreshEntity(m_item);
    }

private:
    CadDocument* m_document = nullptr;
    CadItem* m_item = nullptr;
    QVector3D m_basePoint;
    double m_scaleFactor = 1.0;
};

class ArrayEntityCommand final : public CadEditer::EditCommand
{
public:
    ArrayEntityCommand
    (
        CadDocument* document,
        CadItem* sourceItem,
        int rowCount,
        int columnCount,
        const QVector3D& rowOffset,
        const QVector3D& columnOffset
    )
        : m_document(document)
        , m_sourceItem(sourceItem)
        , m_rowCount(rowCount)
        , m_columnCount(columnCount)
        , m_rowOffset(rowOffset)
        , m_columnOffset(columnOffset)
    {
    }

    bool execute() override
    {
        if (m_document == nullptr || m_sourceItem == nullptr || m_rowCount < 1 || m_columnCount < 1)
        {
            return false;
        }

        if (m_entities.empty())
        {
            for (int row = 0; row < m_rowCount; ++row)
            {
                for (int column = 0; column < m_columnCount; ++column)
                {
                    if (row == 0 && column == 0)
                    {
                        continue;
                    }

                    std::unique_ptr<DRW_Entity> entity = cloneEntity(m_sourceItem->m_nativeEntity);

                    if (entity == nullptr)
                    {
                        return false;
                    }

                    const QVector3D delta = m_rowOffset * static_cast<float>(row) + m_columnOffset * static_cast<float>(column);
                    translateEntity(entity.get(), delta);
                    m_entities.push_back(std::move(entity));
                    m_items.push_back(nullptr);
                    m_itemPtrs.push_back(nullptr);
                }
            }
        }

        for (int index = 0; index < static_cast<int>(m_entities.size()); ++index)
        {
            if (m_entities[index] == nullptr)
            {
                return false;
            }

            if (m_items[index] == nullptr)
            {
                m_items[index] = CadDocument::createCadItemForEntity(m_entities[index].get());
            }

            if (m_items[index] == nullptr)
            {
                return false;
            }

            m_itemPtrs[index] = m_items[index].get();

            if (m_document->appendEntity(std::move(m_entities[index]), std::move(m_items[index])) == nullptr)
            {
                return false;
            }
        }

        return true;
    }

    bool undo() override
    {
        if (m_document == nullptr)
        {
            return false;
        }

        for (int index = static_cast<int>(m_itemPtrs.size()) - 1; index >= 0; --index)
        {
            if (m_itemPtrs[index] == nullptr)
            {
                return false;
            }

            auto [entity, item] = m_document->takeEntity(m_itemPtrs[index]);

            if (entity == nullptr || item == nullptr)
            {
                return false;
            }

            m_entities[index] = std::move(entity);
            m_items[index] = std::move(item);
            m_itemPtrs[index] = m_items[index].get();
        }

        return true;
    }

private:
    CadDocument* m_document = nullptr;
    CadItem* m_sourceItem = nullptr;
    int m_rowCount = 1;
    int m_columnCount = 1;
    QVector3D m_rowOffset;
    QVector3D m_columnOffset;
    std::vector<std::unique_ptr<DRW_Entity>> m_entities;
    std::vector<std::unique_ptr<CadItem>> m_items;
    std::vector<CadItem*> m_itemPtrs;
};

// 修改颜色命令：
// 记录新旧颜色信息，执行与撤销都通过刷新原生实体颜色完成。
class ChangeColorCommand final : public CadEditer::EditCommand
{
public:
    ChangeColorCommand(CadDocument* document, CadItem* item, const QColor& color, int colorIndex)
        : m_document(document)
        , m_item(item)
        , m_newColor(color)
        , m_newColorIndex(colorIndex)
    {
        if (m_item != nullptr && m_item->m_nativeEntity != nullptr)
        {
            m_oldColorIndex = m_item->m_nativeEntity->color;
            m_oldTrueColor = m_item->m_nativeEntity->color24;
        }
    }

    bool execute() override
    {
        return apply(m_newColor, m_newColorIndex);
    }

    bool undo() override
    {
        return apply(m_newColor, m_oldColorIndex, m_oldTrueColor);
    }

private:
    // 按颜色与索引应用颜色，自动决定 true color 或 ACI 方案
    bool apply(const QColor& color, int colorIndex)
    {
        return apply(color, colorIndex, colorIndex >= 0 ? -1 : colorToTrueColor(color));
    }

    bool apply(const QColor& color, int colorIndex, int trueColor)
    {
        if (m_document == nullptr || m_item == nullptr || m_item->m_nativeEntity == nullptr || !m_document->containsEntity(m_item))
        {
            return false;
        }

        if (colorIndex >= 0)
        {
            m_item->m_nativeEntity->color = colorIndex;
            m_item->m_nativeEntity->color24 = -1;
        }
        else
        {
            m_item->m_nativeEntity->color = DRW::ColorByLayer;
            m_item->m_nativeEntity->color24 = trueColor;
        }

        Q_UNUSED(color);
        return m_document->refreshEntity(m_item);
    }

private:
    // 目标文档
    CadDocument* m_document = nullptr;

    // 目标实体
    CadItem* m_item = nullptr;

    // 新颜色
    QColor m_newColor;

    // 新颜色索引
    int m_newColorIndex = -1;

    // 旧颜色索引
    int m_oldColorIndex = DRW::ColorByLayer;

    // 旧 true color
    int m_oldTrueColor = -1;
};

class ChangeLayerCommand final : public CadEditer::EditCommand
{
public:
    ChangeLayerCommand(CadDocument* document, CadItem* item, const QString& layerName)
        : m_document(document)
        , m_item(item)
        , m_newLayerName(layerName.trimmed().isEmpty() ? QStringLiteral("0") : layerName.trimmed())
    {
        if (m_item != nullptr && m_item->m_nativeEntity != nullptr)
        {
            m_oldLayerName = QString::fromUtf8(m_item->m_nativeEntity->layer.c_str());
        }
    }

    bool execute() override
    {
        return apply(m_newLayerName);
    }

    bool undo() override
    {
        return apply(m_oldLayerName);
    }

private:
    bool apply(const QString& layerName)
    {
        if (m_document == nullptr || m_item == nullptr || m_item->m_nativeEntity == nullptr || !m_document->containsEntity(m_item))
        {
            return false;
        }

        const QString normalizedLayerName = layerName.trimmed().isEmpty() ? QStringLiteral("0") : layerName.trimmed();
        m_document->ensureLayerExists(normalizedLayerName);
        m_item->m_nativeEntity->layer = normalizedLayerName.toUtf8().constData();
        return m_document->refreshEntity(m_item);
    }

private:
    CadDocument* m_document = nullptr;
    CadItem* m_item = nullptr;
    QString m_oldLayerName = QStringLiteral("0");
    QString m_newLayerName = QStringLiteral("0");
};

// 切换反向加工命令：
// 记录实体反向状态，在执行与撤销之间来回切换。
class ToggleReverseCommand final : public CadEditer::EditCommand
{
public:
    ToggleReverseCommand(CadDocument* document, CadItem* item)
        : m_document(document)
        , m_item(item)
    {
        if (m_item != nullptr)
        {
            m_originalReverse = m_item->m_isReverse;
        }
    }

    bool execute() override
    {
        return apply(!m_originalReverse);
    }

    bool undo() override
    {
        return apply(m_originalReverse);
    }

private:
    bool apply(bool isReverse)
    {
        if (m_document == nullptr || m_item == nullptr || !m_document->containsEntity(m_item))
        {
            return false;
        }

        m_item->m_isReverse = isReverse;
        return m_document->refreshEntity(m_item);
    }

private:
    // 目标文档
    CadDocument* m_document = nullptr;

    // 目标实体
    CadItem* m_item = nullptr;

    // 切换前的反向状态
    bool m_originalReverse = false;
};

// 批量更新加工状态命令：
// 用于一次性提交加工顺序和反向加工状态，支持撤销与重做。
class UpdateProcessStatesCommand final : public CadEditer::EditCommand
{
public:
    struct ItemProcessState
    {
        CadItem* item = nullptr;
        int oldProcessOrder = -1;
        int newProcessOrder = -1;
        bool oldReverse = false;
        bool newReverse = false;
        bool oldHasCustomStart = false;
        bool newHasCustomStart = false;
        double oldProcessStartParameter = 0.0;
        double newProcessStartParameter = 0.0;
    };

public:
    UpdateProcessStatesCommand(CadDocument* document, std::vector<ItemProcessState> states)
        : m_document(document)
        , m_states(std::move(states))
    {
    }

    bool execute() override
    {
        return apply(true);
    }

    bool undo() override
    {
        return apply(false);
    }

private:
    bool apply(bool useNewState)
    {
        if (m_document == nullptr || m_states.empty())
        {
            return false;
        }

        for (const ItemProcessState& state : m_states)
        {
            if (state.item == nullptr || !m_document->containsEntity(state.item))
            {
                return false;
            }
        }

        for (const ItemProcessState& state : m_states)
        {
            state.item->m_processOrder = useNewState ? state.newProcessOrder : state.oldProcessOrder;
            state.item->m_isReverse = useNewState ? state.newReverse : state.oldReverse;
            state.item->m_hasCustomProcessStart = useNewState ? state.newHasCustomStart : state.oldHasCustomStart;
            state.item->m_processStartParameter = useNewState ? state.newProcessStartParameter : state.oldProcessStartParameter;
            state.item->buildProcessDirection();
        }

        m_document->notifySceneChanged();
        return true;
    }

private:
    // 目标文档
    CadDocument* m_document = nullptr;

    // 变更前后状态集合
    std::vector<ItemProcessState> m_states;
};

// 结束当前活动多段线命令
// @param drawState 当前绘图状态机
// @param closePolyline 是否闭合多段线
// @return 如果成功生成实体返回 true，否则返回 false
bool CadEditer::finishActivePolyline(DrawStateMachine& drawState, bool closePolyline)
{
    if (m_document == nullptr)
    {
        return false;
    }

    if (drawState.drawType != DrawType::Polyline && drawState.drawType != DrawType::LWPolyline)
    {
        return false;
    }

    if (drawState.commandPoints.size() < 2)
    {
        return false;
    }

    const bool lightweight = drawState.drawType == DrawType::LWPolyline;
    std::unique_ptr<DRW_Entity> entity = createPolylineEntity
    (
        drawState.commandPoints,
        drawState.commandBulges,
        drawState.drawingLayerName,
        drawState.drawingColor,
        drawState.drawingColorIndex,
        closePolyline,
        lightweight
    );

    if (!addEntity(std::move(entity)))
    {
        return false;
    }

    // 成功落库后清空暂存点列，并把状态机复位到等待第一点
    drawState.commandPoints.clear();
    drawState.commandBulges.clear();

    if (lightweight)
    {
        drawState.lwPolylineSubMode = LWPolylineDrawSubMode::AwaitFirstPoint;
        drawState.lwPolylineArcMode = false;
    }
    else
    {
        drawState.polylineSubMode = PolylineDrawSubMode::AwaitFirstPoint;
        drawState.polylineArcMode = false;
    }

    return true;
}

// 开始移动编辑命令
// @param drawState 当前绘图状态机
// @param item 待移动实体
// @return 如果命令成功进入活动状态返回 true，否则返回 false
bool CadEditer::beginMove(DrawStateMachine& drawState, CadItem* item)
{
    return beginMove(drawState, QVector<CadItem*>{ item });
}

bool CadEditer::beginMove(DrawStateMachine& drawState, const QVector<CadItem*>& items)
{
    if (m_document == nullptr || items.isEmpty())
    {
        return false;
    }

    QSet<CadItem*> dedup;
    QVector<CadItem*> validatedTargets;
    validatedTargets.reserve(items.size());

    for (CadItem* item : items)
    {
        if (item == nullptr || !m_document->containsEntity(item) || dedup.contains(item))
        {
            continue;
        }

        dedup.insert(item);
        validatedTargets.append(item);
    }

    if (validatedTargets.isEmpty())
    {
        return false;
    }

    drawState.commandPoints.clear();
    drawState.commandBulges.clear();
    drawState.isDrawing = false;
    drawState.drawType = DrawType::None;
    drawState.editType = EditType::Move;
    drawState.moveSubMode = MoveEditSubMode::AwaitBasePoint;
    drawState.gripSubMode = GripEditSubMode::Idle;
    drawState.gripPointIndex = -1;
    m_moveTargets = validatedTargets;
    m_moveTarget = m_moveTargets.front();
    m_gripTarget = nullptr;
    m_gripPointIndex = -1;
    return true;
}

bool CadEditer::beginGripEdit(DrawStateMachine& drawState, CadItem* item, const CadSelectionHandleInfo& handle)
{
    if (m_document == nullptr
        || item == nullptr
        || !m_document->containsEntity(item)
        || !handle.editable
        || handle.pointIndex < 0)
    {
        return false;
    }

    QVector3D currentPoint;

    if (!readEditableControlPoint(item, handle.pointIndex, currentPoint))
    {
        return false;
    }

    drawState.commandPoints = { currentPoint };
    drawState.commandBulges.clear();
    drawState.isDrawing = false;
    drawState.drawType = DrawType::None;
    drawState.editType = EditType::GripEdit;
    drawState.moveSubMode = MoveEditSubMode::Idle;
    drawState.gripSubMode = GripEditSubMode::AwaitTargetPoint;
    drawState.gripPointIndex = handle.pointIndex;
    m_moveTarget = nullptr;
    m_moveTargets.clear();
    m_gripTarget = item;
    m_gripPointIndex = handle.pointIndex;
    return true;
}

// 删除指定实体
// @param item 待删除实体
// @return 如果删除成功返回 true，否则返回 false
bool CadEditer::deleteEntity(CadItem* item)
{
    if (m_document == nullptr || item == nullptr || !m_document->containsEntity(item))
    {
        return false;
    }

    if (item == m_moveTarget)
    {
        m_moveTarget = nullptr;
    }

    const int moveIndex = m_moveTargets.indexOf(item);

    if (moveIndex >= 0)
    {
        m_moveTargets.removeAt(moveIndex);
    }

    if (item == m_gripTarget)
    {
        m_gripTarget = nullptr;
        m_gripPointIndex = -1;
    }

    return executeCommand(std::make_unique<DeleteEntityCommand>(m_document, item));
}

bool CadEditer::deleteEntities(const QVector<CadItem*>& items)
{
    if (m_document == nullptr || items.isEmpty())
    {
        return false;
    }

    QVector<CadItem*> validItems;
    QSet<CadItem*> deduplicated;

    for (CadItem* item : items)
    {
        if (item == nullptr || !m_document->containsEntity(item) || deduplicated.contains(item))
        {
            continue;
        }

        deduplicated.insert(item);
        validItems.push_back(item);

        if (item == m_moveTarget)
        {
            m_moveTarget = nullptr;
        }

        m_moveTargets.removeAll(item);

        if (item == m_gripTarget)
        {
            m_gripTarget = nullptr;
            m_gripPointIndex = -1;
        }
    }

    if (validItems.isEmpty())
    {
        return false;
    }

    return executeCommand(std::make_unique<DeleteEntitiesCommand>(m_document, validItems));
}

bool CadEditer::copyEntity(CadItem* item, const QVector3D& delta)
{
    if (m_document == nullptr || item == nullptr || !m_document->containsEntity(item))
    {
        return false;
    }

    if (delta.lengthSquared() <= kGeometryEpsilon)
    {
        return false;
    }

    return executeCommand(std::make_unique<CopyEntityCommand>(m_document, item, delta));
}

bool CadEditer::rotateEntity(CadItem* item, const QVector3D& basePoint, double angleDegrees)
{
    if (m_document == nullptr || item == nullptr || !m_document->containsEntity(item) || std::abs(angleDegrees) <= kGeometryEpsilon)
    {
        return false;
    }

    return executeCommand(std::make_unique<RotateEntityCommand>(m_document, item, basePoint, angleDegrees));
}

bool CadEditer::scaleEntity(CadItem* item, const QVector3D& basePoint, double scaleFactor)
{
    if (m_document == nullptr || item == nullptr || !m_document->containsEntity(item) || scaleFactor <= kGeometryEpsilon)
    {
        return false;
    }

    return executeCommand(std::make_unique<ScaleEntityCommand>(m_document, item, basePoint, scaleFactor));
}

bool CadEditer::arrayEntity(CadItem* item, int rowCount, int columnCount, const QVector3D& rowOffset, const QVector3D& columnOffset)
{
    if (m_document == nullptr || item == nullptr || !m_document->containsEntity(item))
    {
        return false;
    }

    if (rowCount < 1 || columnCount < 1 || (rowCount == 1 && columnCount == 1))
    {
        return false;
    }

    return executeCommand
    (
        std::make_unique<ArrayEntityCommand>(m_document, item, rowCount, columnCount, rowOffset, columnOffset)
    );
}

bool CadEditer::mirrorEntities
(
    const QVector<CadItem*>& items,
    const QVector3D& firstPoint,
    const QVector3D& secondPoint,
    bool eraseSource
)
{
    if (m_document == nullptr || items.isEmpty() || pointsNear(firstPoint, secondPoint))
    {
        return false;
    }

    QVector<CadItem*> validItems;
    QSet<CadItem*> deduplicated;
    std::vector<std::unique_ptr<DRW_Entity>> mirroredEntities;

    for (CadItem* item : items)
    {
        if (item == nullptr || !m_document->containsEntity(item) || item->m_nativeEntity == nullptr || deduplicated.contains(item))
        {
            continue;
        }

        std::unique_ptr<DRW_Entity> mirrored = cloneEntity(item->m_nativeEntity);

        if (mirrored == nullptr || !mirrorEntityGeometry(mirrored.get(), firstPoint, secondPoint))
        {
            return false;
        }

        deduplicated.insert(item);
        validItems.push_back(item);
        mirroredEntities.push_back(std::move(mirrored));
    }

    if (validItems.isEmpty())
    {
        return false;
    }

    if (eraseSource)
    {
        return executeCommand(std::make_unique<ReplaceEntitiesCommand>(m_document, validItems, std::move(mirroredEntities)));
    }

    return executeCommand(std::make_unique<AddEntitiesCommand>(m_document, std::move(mirroredEntities)));
}

bool CadEditer::polarArrayEntities
(
    const QVector<CadItem*>& items,
    const QVector3D& center,
    int itemCount,
    double totalAngleDegrees,
    bool rotateItems
)
{
    if (m_document == nullptr || items.isEmpty() || itemCount < 2 || std::abs(totalAngleDegrees) <= kGeometryEpsilon)
    {
        return false;
    }

    QVector<CadItem*> validItems;
    QSet<CadItem*> deduplicated;
    std::vector<std::unique_ptr<DRW_Entity>> arrayEntities;
    const double useFullCircleSpacing = std::abs(std::abs(totalAngleDegrees) - 360.0) <= 1.0e-6 ? static_cast<double>(itemCount) : static_cast<double>(itemCount - 1);
    const double angleStep = totalAngleDegrees / useFullCircleSpacing;

    for (CadItem* item : items)
    {
        if (item == nullptr || !m_document->containsEntity(item) || item->m_nativeEntity == nullptr || deduplicated.contains(item))
        {
            continue;
        }

        deduplicated.insert(item);
        validItems.push_back(item);

        for (int index = 1; index < itemCount; ++index)
        {
            const double angle = angleStep * static_cast<double>(index);
            std::unique_ptr<DRW_Entity> entity = cloneEntity(item->m_nativeEntity);

            if (entity == nullptr)
            {
                return false;
            }

            if (rotateItems)
            {
                ::rotateEntity(entity.get(), center, angle);
            }
            else
            {
                const QVector3D itemCenter = itemGeometryCenter(item);
                const QVector3D rotatedCenter = rotatePlanarPoint(itemCenter, center, angle * kPi / 180.0);
                translateEntity(entity.get(), rotatedCenter - itemCenter);
            }

            arrayEntities.push_back(std::move(entity));
        }
    }

    if (validItems.isEmpty() || arrayEntities.empty())
    {
        return false;
    }

    return executeCommand(std::make_unique<AddEntitiesCommand>(m_document, std::move(arrayEntities)));
}

bool CadEditer::offsetEntity(CadItem* item, double distance)
{
    if (m_document == nullptr || item == nullptr || !m_document->containsEntity(item))
    {
        return false;
    }

    std::unique_ptr<DRW_Entity> offset = createOffsetEntity(item, distance);

    if (offset == nullptr)
    {
        return false;
    }

    return executeCommand(std::make_unique<AddEntityCommand>(m_document, std::move(offset)));
}

bool CadEditer::trimEntity(CadItem* boundaryItem, CadItem* targetItem, bool trimStart)
{
    if (m_document == nullptr
        || boundaryItem == nullptr
        || targetItem == nullptr
        || boundaryItem == targetItem
        || !m_document->containsEntity(boundaryItem)
        || !m_document->containsEntity(targetItem))
    {
        return false;
    }

    std::unique_ptr<DRW_Entity> trimmed = cloneEntity(targetItem->m_nativeEntity);

    if (trimmed == nullptr || !trimOrExtendLineEntity(trimmed.get(), boundaryItem->m_nativeEntity, trimStart, true))
    {
        return false;
    }

    std::vector<std::unique_ptr<DRW_Entity>> replacements;
    replacements.push_back(std::move(trimmed));
    return executeCommand(std::make_unique<ReplaceEntitiesCommand>(m_document, QVector<CadItem*>{ targetItem }, std::move(replacements)));
}

bool CadEditer::extendEntity(CadItem* boundaryItem, CadItem* targetItem, bool extendStart)
{
    if (m_document == nullptr
        || boundaryItem == nullptr
        || targetItem == nullptr
        || boundaryItem == targetItem
        || !m_document->containsEntity(boundaryItem)
        || !m_document->containsEntity(targetItem))
    {
        return false;
    }

    std::unique_ptr<DRW_Entity> extended = cloneEntity(targetItem->m_nativeEntity);

    if (extended == nullptr || !trimOrExtendLineEntity(extended.get(), boundaryItem->m_nativeEntity, extendStart, false))
    {
        return false;
    }

    std::vector<std::unique_ptr<DRW_Entity>> replacements;
    replacements.push_back(std::move(extended));
    return executeCommand(std::make_unique<ReplaceEntitiesCommand>(m_document, QVector<CadItem*>{ targetItem }, std::move(replacements)));
}

bool CadEditer::joinEntities(const QVector<CadItem*>& items)
{
    if (m_document == nullptr || items.size() < 2)
    {
        return false;
    }

    QVector<CadItem*> validItems;
    QSet<CadItem*> deduplicated;

    for (CadItem* item : items)
    {
        if (item == nullptr || !m_document->containsEntity(item) || deduplicated.contains(item))
        {
            continue;
        }

        deduplicated.insert(item);
        validItems.push_back(item);
    }

    if (validItems.size() < 2)
    {
        return false;
    }

    std::unique_ptr<DRW_Entity> joinedEntity;

    if (!buildJoinedPolylineEntity(validItems, joinedEntity))
    {
        return false;
    }

    std::vector<std::unique_ptr<DRW_Entity>> replacements;
    replacements.push_back(std::move(joinedEntity));
    return executeCommand(std::make_unique<ReplaceEntitiesCommand>(m_document, validItems, std::move(replacements)));
}

bool CadEditer::filletEntities(CadItem* firstItem, CadItem* secondItem, double radius)
{
    if (m_document == nullptr
        || firstItem == nullptr
        || secondItem == nullptr
        || firstItem == secondItem
        || !m_document->containsEntity(firstItem)
        || !m_document->containsEntity(secondItem))
    {
        return false;
    }

    std::vector<std::unique_ptr<DRW_Entity>> replacements;

    if (!buildFilletReplacementEntities(firstItem, secondItem, radius, replacements))
    {
        return false;
    }

    return executeCommand
    (
        std::make_unique<ReplaceEntitiesCommand>(m_document, QVector<CadItem*>{ firstItem, secondItem }, std::move(replacements))
    );
}

bool CadEditer::chamferEntities(CadItem* firstItem, CadItem* secondItem, double firstDistance, double secondDistance)
{
    if (m_document == nullptr
        || firstItem == nullptr
        || secondItem == nullptr
        || firstItem == secondItem
        || !m_document->containsEntity(firstItem)
        || !m_document->containsEntity(secondItem))
    {
        return false;
    }

    std::vector<std::unique_ptr<DRW_Entity>> replacements;

    if (!buildChamferReplacementEntities(firstItem, secondItem, firstDistance, secondDistance, replacements))
    {
        return false;
    }

    return executeCommand
    (
        std::make_unique<ReplaceEntitiesCommand>(m_document, QVector<CadItem*>{ firstItem, secondItem }, std::move(replacements))
    );
}

// 修改指定实体颜色
// @param item 目标实体
// @param color 新颜色
// @param colorIndex 可选 ACI 颜色索引，小于 0 时使用 true color
// @return 如果修改成功返回 true，否则返回 false
bool CadEditer::changeEntityColor(CadItem* item, const QColor& color, int colorIndex)
{
    if (m_document == nullptr || item == nullptr || !m_document->containsEntity(item) || !color.isValid())
    {
        return false;
    }

    return executeCommand(std::make_unique<ChangeColorCommand>(m_document, item, color, colorIndex));
}

bool CadEditer::changeEntityLayer(CadItem* item, const QString& layerName)
{
    if (m_document == nullptr || item == nullptr || !m_document->containsEntity(item))
    {
        return false;
    }

    const QString normalizedLayerName = layerName.trimmed().isEmpty() ? QStringLiteral("0") : layerName.trimmed();
    return executeCommand(std::make_unique<ChangeLayerCommand>(m_document, item, normalizedLayerName));
}

// 切换指定实体的反向加工标记
// @param item 目标实体
// @return 如果切换成功返回 true，否则返回 false
bool CadEditer::toggleEntityReverse(CadItem* item)
{
    if (m_document == nullptr || item == nullptr || !m_document->containsEntity(item))
    {
        return false;
    }

    return executeCommand(std::make_unique<ToggleReverseCommand>(m_document, item));
}

// 设置指定实体的加工顺序
// @param item 目标实体
// @param processOrder 新的加工顺序
// @return 如果设置成功返回 true，否则返回 false
bool CadEditer::setEntityProcessOrder(CadItem* item, int processOrder)
{
    if (m_document == nullptr || item == nullptr || !m_document->containsEntity(item) || processOrder < 0)
    {
        return false;
    }

    std::vector<UpdateProcessStatesCommand::ItemProcessState> states;
    states.push_back
    ({
        item,
        item->m_processOrder,
        processOrder,
        item->m_isReverse,
        item->m_isReverse,
        item->m_hasCustomProcessStart,
        item->m_hasCustomProcessStart,
        item->m_processStartParameter,
        item->m_processStartParameter
    });

    return executeCommand(std::make_unique<UpdateProcessStatesCommand>(m_document, std::move(states)));
}

// 批量更新实体的加工顺序、反向加工状态与闭合图元起刀缝点
// @param updates 目标实体的加工状态更新数组
// @return 如果批量更新成功返回 true，否则返回 false
bool CadEditer::applyEntityProcessStates(const std::vector<ProcessStateUpdate>& updates)
{
    if (m_document == nullptr
        || updates.empty())
    {
        return false;
    }

    std::vector<UpdateProcessStatesCommand::ItemProcessState> states;
    states.reserve(updates.size());

    for (const ProcessStateUpdate& update : updates)
    {
        CadItem* item = update.item;

        if (item == nullptr || !m_document->containsEntity(item) || update.processOrder < 0)
        {
            return false;
        }

        states.push_back
        ({
            item,
            item->m_processOrder,
            update.processOrder,
            item->m_isReverse,
            update.isReverse,
            item->m_hasCustomProcessStart,
            update.hasCustomStart,
            item->m_processStartParameter,
            update.processStartParameter
        });
    }

    return executeCommand(std::make_unique<UpdateProcessStatesCommand>(m_document, std::move(states)));
}

// 执行命令并压入 Undo 栈
// @param command 待执行的命令对象
// @return 如果执行成功返回 true，否则返回 false
bool CadEditer::executeCommand(std::unique_ptr<EditCommand> command)
{
    // 一旦产生新命令，Redo 栈就失效
    if (command == nullptr)
    {
        return false;
    }

    if (!command->execute())
    {
        return false;
    }

    m_redoStack.clear();
    m_undoStack.push_back(std::move(command));
    return true;
}

// 处理点绘制命令
bool CadEditer::handlePointDrawing
(
    const DrawStateMachine& previousState,
    DrawStateMachine& currentState,
    const QVector3D& worldPos
)
{
    Q_UNUSED(currentState);

    if (previousState.pointSubMode != PointDrawSubMode::AwaitPosition)
    {
        return false;
    }

    return addEntity
    (
        createPointEntity
        (
            worldPos,
            previousState.drawingLayerName,
            previousState.drawingColor,
            previousState.drawingColorIndex
        )
    );
}

// 处理直线绘制命令
bool CadEditer::handleLineDrawing
(
    const DrawStateMachine& previousState,
    DrawStateMachine& currentState,
    const QVector3D& worldPos
)
{
    if (previousState.lineSubMode == LineDrawSubMode::AwaitStartPoint)
    {
        // 第一次点击只记录起点
        currentState.commandPoints = { worldPos };
        return true;
    }

    if (previousState.lineSubMode == LineDrawSubMode::AwaitEndPoint && !currentState.commandPoints.isEmpty())
    {
        const QVector3D startPoint = currentState.commandPoints.front();

        if (!addEntity
        (
            createLineEntity
            (
                startPoint,
                worldPos,
                previousState.drawingLayerName,
                previousState.drawingColor,
                previousState.drawingColorIndex
            )
        ))
        {
            return false;
        }

        // 成功创建一段线后，把终点作为下一段的起点，支持连续折线式输入
        currentState.commandPoints = { worldPos };
        currentState.lineSubMode = LineDrawSubMode::AwaitEndPoint;
        return true;
    }

    return false;
}

bool CadEditer::handleXlineDrawing
(
    const DrawStateMachine& previousState,
    DrawStateMachine& currentState,
    const QVector3D& worldPos
)
{
    if (previousState.lineSubMode == LineDrawSubMode::AwaitStartPoint)
    {
        currentState.commandPoints = { worldPos };
        return true;
    }

    if (previousState.lineSubMode == LineDrawSubMode::AwaitEndPoint && !currentState.commandPoints.isEmpty())
    {
        const QVector3D basePoint = currentState.commandPoints.front();

        if (!addEntity
        (
            createXlineEntity
            (
                basePoint,
                worldPos,
                previousState.drawingLayerName,
                previousState.drawingColor,
                previousState.drawingColorIndex
            )
        ))
        {
            return false;
        }

        currentState.commandPoints.clear();
        currentState.lineSubMode = LineDrawSubMode::AwaitStartPoint;
        return true;
    }

    return false;
}

bool CadEditer::handleRectangleDrawing
(
    const DrawStateMachine& previousState,
    DrawStateMachine& currentState,
    const QVector3D& worldPos
)
{
    if (previousState.rectangleSubMode == RectangleDrawSubMode::AwaitFirstCorner)
    {
        currentState.commandPoints = { flattenToDrawingPlane(worldPos) };
        return true;
    }

    if (previousState.rectangleSubMode == RectangleDrawSubMode::AwaitSecondCorner && !currentState.commandPoints.isEmpty())
    {
        const QVector3D firstCorner = flattenToDrawingPlane(currentState.commandPoints.front());

        if (!addEntity
        (
            createRectangleEntity
            (
                firstCorner,
                worldPos,
                previousState.drawingLayerName,
                previousState.drawingColor,
                previousState.drawingColorIndex
            )
        ))
        {
            return false;
        }

        currentState.commandPoints.clear();
        currentState.rectangleSubMode = RectangleDrawSubMode::AwaitFirstCorner;
        return true;
    }

    return false;
}

bool CadEditer::handlePolygonDrawing
(
    const DrawStateMachine& previousState,
    DrawStateMachine& currentState,
    const QVector3D& worldPos
)
{
    if (previousState.polygonSubMode == PolygonDrawSubMode::AwaitCenter)
    {
        currentState.commandPoints = { flattenToDrawingPlane(worldPos) };
        return true;
    }

    if (previousState.polygonSubMode == PolygonDrawSubMode::AwaitRadius && !currentState.commandPoints.isEmpty())
    {
        const QVector3D center = flattenToDrawingPlane(currentState.commandPoints.front());

        if (!addEntity
        (
            createPolygonEntity
            (
                center,
                worldPos,
                previousState.polygonSideCount,
                previousState.polygonCircumscribedAboutCircle,
                previousState.drawingLayerName,
                previousState.drawingColor,
                previousState.drawingColorIndex
            )
        ))
        {
            return false;
        }

        currentState.commandPoints.clear();
        currentState.polygonSubMode = PolygonDrawSubMode::AwaitCenter;
        return true;
    }

    return false;
}

// 处理圆绘制命令
bool CadEditer::handleCircleDrawing
(
    const DrawStateMachine& previousState,
    DrawStateMachine& currentState,
    const QVector3D& worldPos
)
{
    if (previousState.circleSubMode == CircleDrawSubMode::AwaitCenter)
    {
        currentState.commandPoints = { worldPos };
        return true;
    }

    if (previousState.circleSubMode == CircleDrawSubMode::AwaitRadius && !currentState.commandPoints.isEmpty())
    {
        const QVector3D center = currentState.commandPoints.front();

        if (!addEntity
        (
            createCircleEntity
            (
                center,
                worldPos,
                previousState.drawingLayerName,
                previousState.drawingColor,
                previousState.drawingColorIndex
            )
        ))
        {
            return false;
        }

        currentState.commandPoints.clear();
        currentState.circleSubMode = CircleDrawSubMode::AwaitCenter;
        return true;
    }

    return false;
}

// 处理圆弧绘制命令
bool CadEditer::handleArcDrawing
(
    const DrawStateMachine& previousState,
    DrawStateMachine& currentState,
    const QVector3D& worldPos
)
{
    switch (previousState.arcSubMode)
    {
    case ArcDrawSubMode::AwaitCenter:
        currentState.commandPoints = { worldPos };
        return true;

    case ArcDrawSubMode::AwaitRadius:
        if (currentState.commandPoints.isEmpty())
        {
            return false;
        }

        if (currentState.commandPoints.size() == 1)
        {
            currentState.commandPoints.append(worldPos);
        }
        else
        {
            currentState.commandPoints[1] = worldPos;
        }
        return true;

    case ArcDrawSubMode::AwaitStartAngle:
        if (currentState.commandPoints.size() < 2)
        {
            return false;
        }

        if (currentState.commandPoints.size() == 2)
        {
            currentState.commandPoints.append(worldPos);
        }
        else
        {
            currentState.commandPoints[2] = worldPos;
        }
        return true;

    case ArcDrawSubMode::AwaitEndAngle:
        if (currentState.commandPoints.size() < 3)
        {
            return false;
        }

        // 圆弧绘制按“圆心 -> 半径 -> 起始角 -> 终止角”四步完成
        if (!addEntity
        (
            createArcEntity
            (
                currentState.commandPoints[0],
                currentState.commandPoints[1],
                currentState.commandPoints[2],
                worldPos,
                previousState.drawingLayerName,
                previousState.drawingColor,
                previousState.drawingColorIndex
            )
        ))
        {
            return false;
        }

        currentState.commandPoints.clear();
        currentState.arcSubMode = ArcDrawSubMode::AwaitCenter;
        return true;

    default:
        break;
    }

    return false;
}

// 处理椭圆绘制命令
bool CadEditer::handleEllipseDrawing
(
    const DrawStateMachine& previousState,
    DrawStateMachine& currentState,
    const QVector3D& worldPos
)
{
    switch (previousState.ellipseSubMode)
    {
    case EllipseDrawSubMode::AwaitCenter:
        currentState.commandPoints = { worldPos };
        return true;

    case EllipseDrawSubMode::AwaitMajorAxis:
        if (currentState.commandPoints.isEmpty())
        {
            return false;
        }

        if (currentState.commandPoints.size() == 1)
        {
            currentState.commandPoints.append(worldPos);
        }
        else
        {
            currentState.commandPoints[1] = worldPos;
        }
        return true;

    case EllipseDrawSubMode::AwaitMinorAxis:
        if (currentState.commandPoints.size() < 2)
        {
            return false;
        }

        if (!addEntity
        (
            createEllipseEntity
            (
                currentState.commandPoints[0],
                currentState.commandPoints[1],
                worldPos,
                previousState.drawingLayerName,
                previousState.drawingColor,
                previousState.drawingColorIndex
            )
        ))
        {
            return false;
        }

        currentState.commandPoints.clear();
        currentState.ellipseSubMode = EllipseDrawSubMode::AwaitCenter;
        return true;

    default:
        break;
    }

    return false;
}

// 处理多段线/轻量多段线绘制命令
bool CadEditer::handlePolylineDrawing
(
    const DrawStateMachine& previousState,
    DrawStateMachine& currentState,
    const QVector3D& worldPos,
    bool lightweight
)
{
    const QVector3D planarPoint = flattenToDrawingPlane(worldPos);
    const bool arcMode = lightweight ? currentState.lwPolylineArcMode : currentState.polylineArcMode;

    if ((lightweight && previousState.lwPolylineSubMode == LWPolylineDrawSubMode::AwaitFirstPoint)
        || (!lightweight && previousState.polylineSubMode == PolylineDrawSubMode::AwaitFirstPoint))
    {
        // 第一击只建立起始点，并切换到后续线段/圆弧段输入状态
        currentState.commandPoints = { planarPoint };
        currentState.commandBulges.clear();

        if (lightweight)
        {
            currentState.lwPolylineSubMode = arcMode ? LWPolylineDrawSubMode::AwaitArcEndPoint : LWPolylineDrawSubMode::AwaitLineEndPoint;
        }
        else
        {
            currentState.polylineSubMode = arcMode ? PolylineDrawSubMode::AwaitArcEndPoint : PolylineDrawSubMode::AwaitLineEndPoint;
        }

        return true;
    }

    if (currentState.commandPoints.isEmpty())
    {
        return false;
    }

    const QVector3D lastPoint = flattenToDrawingPlane(currentState.commandPoints.back());

    if ((planarPoint - lastPoint).lengthSquared() <= kGeometryEpsilon)
    {
        return false;
    }

    if ((lightweight && previousState.lwPolylineSubMode == LWPolylineDrawSubMode::AwaitArcEndPoint)
        || (!lightweight && previousState.polylineSubMode == PolylineDrawSubMode::AwaitArcEndPoint))
    {
        // 圆弧续接依赖上一段末切向，按切向与新终点实时反推 bulge
        const QVector3D tangentDirection = polylineEndTangent(currentState.commandPoints, currentState.commandBulges);

        if (tangentDirection.lengthSquared() <= kGeometryEpsilon)
        {
            return false;
        }

        const double bulge = bulgeFromTangent(lastPoint, tangentDirection, planarPoint);

        if (!std::isfinite(bulge))
        {
            return false;
        }

        currentState.commandBulges.append(bulge);
        currentState.commandPoints.append(planarPoint);

        if (lightweight)
        {
            currentState.lwPolylineSubMode = LWPolylineDrawSubMode::AwaitArcEndPoint;
        }
        else
        {
            currentState.polylineSubMode = PolylineDrawSubMode::AwaitArcEndPoint;
        }

        return true;
    }

    // 直线段直接以 bulge=0 追加
    currentState.commandBulges.append(0.0);
    currentState.commandPoints.append(planarPoint);

    if (lightweight)
    {
        currentState.lwPolylineSubMode = LWPolylineDrawSubMode::AwaitLineEndPoint;
    }
    else
    {
        currentState.polylineSubMode = PolylineDrawSubMode::AwaitLineEndPoint;
    }

    return true;
}

// 处理移动编辑命令
bool CadEditer::handleMoveEditing
(
    const DrawStateMachine& previousState,
    DrawStateMachine& currentState,
    const QVector3D& worldPos
)
{
    // 目标失效时立刻退出 Move 模式，避免悬空编辑状态
    if (m_document == nullptr)
    {
        currentState.commandPoints.clear();
        currentState.editType = EditType::None;
        currentState.moveSubMode = MoveEditSubMode::Idle;
        m_moveTarget = nullptr;
        m_moveTargets.clear();
        return false;
    }

    QVector<CadItem*> validTargets;
    validTargets.reserve(m_moveTargets.size());

    for (CadItem* item : m_moveTargets)
    {
        if (item != nullptr && m_document->containsEntity(item))
        {
            validTargets.append(item);
        }
    }

    if (validTargets.isEmpty())
    {
        currentState.commandPoints.clear();
        currentState.editType = EditType::None;
        currentState.moveSubMode = MoveEditSubMode::Idle;
        m_moveTarget = nullptr;
        m_moveTargets.clear();
        return false;
    }

    m_moveTargets = validTargets;
    m_moveTarget = m_moveTargets.front();

    if (previousState.moveSubMode == MoveEditSubMode::AwaitBasePoint)
    {
        // 第一次点击记录基点
        currentState.commandPoints = { worldPos };
        return true;
    }

    if (previousState.moveSubMode == MoveEditSubMode::AwaitTargetPoint && !currentState.commandPoints.isEmpty())
    {
        // 第二次点击确定目标点，并以两点差值作为移动增量
        const QVector3D basePoint = currentState.commandPoints.front();
        const QVector3D delta = worldPos - basePoint;

        if (delta.lengthSquared() > kGeometryEpsilon)
        {
            if (!executeCommand(std::make_unique<MoveEntitiesCommand>(m_document, m_moveTargets, delta)))
            {
                return false;
            }
        }

        // 移动完成后清空编辑状态
        currentState.commandPoints.clear();
        currentState.editType = EditType::None;
        currentState.moveSubMode = MoveEditSubMode::Idle;
        m_moveTarget = nullptr;
        m_moveTargets.clear();
        return true;
    }

    return false;
}

bool CadEditer::handleGripEditing
(
    const DrawStateMachine& previousState,
    DrawStateMachine& currentState,
    const QVector3D& worldPos
)
{
    if (m_gripTarget == nullptr
        || m_document == nullptr
        || !m_document->containsEntity(m_gripTarget)
        || m_gripPointIndex < 0)
    {
        currentState.commandPoints.clear();
        currentState.editType = EditType::None;
        currentState.gripSubMode = GripEditSubMode::Idle;
        currentState.gripPointIndex = -1;
        m_gripTarget = nullptr;
        m_gripPointIndex = -1;
        return false;
    }

    if (previousState.gripSubMode == GripEditSubMode::AwaitTargetPoint && !currentState.commandPoints.isEmpty())
    {
        const QVector3D basePoint = currentState.commandPoints.front();
        const QVector3D targetPoint = flattenToDrawingPlane(worldPos);

        if ((targetPoint - basePoint).lengthSquared() > kGeometryEpsilon)
        {
            if (!executeCommand(std::make_unique<GripPointEditCommand>(m_document, m_gripTarget, m_gripPointIndex, targetPoint)))
            {
                return false;
            }
        }

        currentState.commandPoints.clear();
        currentState.editType = EditType::None;
        currentState.gripSubMode = GripEditSubMode::Idle;
        currentState.gripPointIndex = -1;
        m_gripTarget = nullptr;
        m_gripPointIndex = -1;
        return true;
    }

    return false;
}

// 向文档追加新实体
// @param entity 待追加的原生 DXF 实体
// @return 如果追加成功返回 true，否则返回 false
bool CadEditer::addEntity(std::unique_ptr<DRW_Entity> entity)
{
    if (entity == nullptr)
    {
        return false;
    }

    return executeCommand(std::make_unique<AddEntityCommand>(m_document, std::move(entity)));
}

