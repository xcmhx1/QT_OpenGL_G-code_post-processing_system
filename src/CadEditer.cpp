// CadEditer 实现文件
// 实现 CadEditer 模块，对应头文件中声明的主要行为和协作流程。
// 编辑器模块，负责绘图创建、实体修改以及 Undo/Redo 命令栈管理。
#include "pch.h"

#include "CadEditer.h"

#include "CadDocument.h"
#include "CadItem.h"
#include "DrawStateMachine.h"

#include <cmath>
#include <limits>

namespace
{
    // 几何与角度计算使用的局部常量
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kTwoPi = 6.28318530717958647692;
    constexpr double kGeometryEpsilon = 1.0e-9;

    // 将任意点压回二维绘图平面
    // @param point 输入三维点
    // @return 压平后的点
    QVector3D flattenToDrawingPlane(const QVector3D& point)
    {
        return QVector3D(point.x(), point.y(), 0.0f);
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

// 析构编辑器对象
CadEditer::~CadEditer() = default;

// 绑定当前编辑目标文档
// @param document 文档对象指针
void CadEditer::setDocument(CadDocument* document)
{
    if (m_document == document)
    {
        return;
    }

    clearHistory();
    m_document = document;
}

// 清空 Undo / Redo 历史
void CadEditer::clearHistory()
{
    m_undoStack.clear();
    m_redoStack.clear();
    cancelTransientCommand();
}

// 取消当前 transient 编辑命令
void CadEditer::cancelTransientCommand()
{
    m_moveTarget = nullptr;
    m_gripTarget = nullptr;
    m_gripPointIndex = -1;
}

// 查询是否可以撤销
// @return 如果撤销栈非空返回 true，否则返回 false
bool CadEditer::canUndo() const
{
    return !m_undoStack.empty();
}

// 查询是否可以重做
// @return 如果重做栈非空返回 true，否则返回 false
bool CadEditer::canRedo() const
{
    return !m_redoStack.empty();
}

// 执行撤销
// @return 如果撤销成功返回 true，否则返回 false
bool CadEditer::undo()
{
    // 从 Undo 栈弹出命令，撤销成功后转移到 Redo 栈
    if (m_undoStack.empty())
    {
        return false;
    }

    std::unique_ptr<EditCommand> command = std::move(m_undoStack.back());
    m_undoStack.pop_back();

    if (!command->undo())
    {
        return false;
    }

    m_redoStack.push_back(std::move(command));
    return true;
}

// 执行重做
// @return 如果重做成功返回 true，否则返回 false
bool CadEditer::redo()
{
    // 从 Redo 栈弹出命令，重做成功后重新压回 Undo 栈
    if (m_redoStack.empty())
    {
        return false;
    }

    std::unique_ptr<EditCommand> command = std::move(m_redoStack.back());
    m_redoStack.pop_back();

    if (!command->execute())
    {
        return false;
    }

    m_undoStack.push_back(std::move(command));
    return true;
}

// 处理左键点击驱动的绘图或编辑逻辑
bool CadEditer::handleLeftPress
(
    const DrawStateMachine& previousState,
    DrawStateMachine& currentState,
    const QVector3D& worldPos
)
{
    // Move 编辑优先于绘图命令处理
    if (m_document == nullptr)
    {
        return false;
    }

    if (currentState.editType == EditType::Move || previousState.editType == EditType::Move)
    {
        return handleMoveEditing(previousState, currentState, worldPos);
    }

    if (currentState.editType == EditType::GripEdit || previousState.editType == EditType::GripEdit)
    {
        return handleGripEditing(previousState, currentState, worldPos);
    }

    if (!currentState.isDrawing && !previousState.isDrawing)
    {
        return false;
    }

    // 其它绘图命令按前一时刻的 drawType 分发，确保状态机切换前后语义一致
    switch (previousState.drawType)
    {
    case DrawType::Point:
        return handlePointDrawing(previousState, currentState, worldPos);
    case DrawType::Line:
        return handleLineDrawing(previousState, currentState, worldPos);
    case DrawType::Circle:
        return handleCircleDrawing(previousState, currentState, worldPos);
    case DrawType::Arc:
        return handleArcDrawing(previousState, currentState, worldPos);
    case DrawType::Ellipse:
        return handleEllipseDrawing(previousState, currentState, worldPos);
    case DrawType::Polyline:
        return handlePolylineDrawing(previousState, currentState, worldPos, false);
    case DrawType::LWPolyline:
        return handlePolylineDrawing(previousState, currentState, worldPos, true);
    default:
        break;
    }

    return false;
}

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
    if (m_document == nullptr || item == nullptr || !m_document->containsEntity(item))
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
    m_moveTarget = item;
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
    m_moveTarget = nullptr;
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

    if (item == m_gripTarget)
    {
        m_gripTarget = nullptr;
        m_gripPointIndex = -1;
    }

    return executeCommand(std::make_unique<DeleteEntityCommand>(m_document, item));
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
    if (m_moveTarget == nullptr || m_document == nullptr || !m_document->containsEntity(m_moveTarget))
    {
        currentState.commandPoints.clear();
        currentState.editType = EditType::None;
        currentState.moveSubMode = MoveEditSubMode::Idle;
        m_moveTarget = nullptr;
        return false;
    }

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
            if (!executeCommand(std::make_unique<MoveEntityCommand>(m_document, m_moveTarget, delta)))
            {
                return false;
            }
        }

        // 移动完成后清空编辑状态
        currentState.commandPoints.clear();
        currentState.editType = EditType::None;
        currentState.moveSubMode = MoveEditSubMode::Idle;
        m_moveTarget = nullptr;
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
