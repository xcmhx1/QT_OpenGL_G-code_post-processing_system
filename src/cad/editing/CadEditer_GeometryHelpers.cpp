// CadEditer 实现文件
// 实现 CadEditer 模块，对应头文件中声明的主要行为和协作流程。
// 编辑器模块，负责绘图创建、实体修改以及 Undo/Redo 命令栈管理。
#include "platform/pch.h"

#include "cad/editing/CadEditer.h"

#include "cad/document/CadDocument.h"
#include "cad/items/CadItem.h"
#include "cad/editing/DrawStateMachine.h"

#include <QSet>

#include <algorithm>
#include <cmath>
#include <limits>

#include "cad/editing/CadEditerWorkflowInternal.h"

namespace CadEditerWorkflowInternal
{
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

    bool pointsNear(const QVector3D& first, const QVector3D& second, double tolerance)
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
        case DRW::ETYPE::SPLINE:
        {
            DRW_Spline* spline = static_cast<DRW_Spline*>(entity);
            const auto reflectCoordinate = [&lineStart, &lineEnd]
            (const std::shared_ptr<DRW_Coord>& coordinate)
            {
                if (coordinate == nullptr)
                {
                    return;
                }
                const QVector3D mirrored = reflectPointAcrossLine
                (
                    QVector3D(coordinate->x, coordinate->y, coordinate->z),
                    lineStart,
                    lineEnd
                );
                coordinate->x = mirrored.x();
                coordinate->y = mirrored.y();
                coordinate->z = mirrored.z();
            };
            for (const std::shared_ptr<DRW_Coord>& point : spline->controllist)
            {
                reflectCoordinate(point);
            }
            for (const std::shared_ptr<DRW_Coord>& point : spline->fitlist)
            {
                reflectCoordinate(point);
            }
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

}

