// CadEditer 图元工厂
#include "pch.h"

#include "CadEditerWorkflowInternal.h"

#include <cmath>
#include <limits>

namespace CadEditerWorkflowInternal
{
    double radiusFromPoints(const QVector3D& center, const QVector3D& point)
    {
        return (point - center).length();
    }

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

    int colorToTrueColor(const QColor& color)
    {
        return (color.red() << 16) | (color.green() << 8) | color.blue();
    }

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
