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

#include "CadEditerWorkflowInternal.h"

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

