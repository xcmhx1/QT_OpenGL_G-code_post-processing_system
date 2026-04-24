// CadEditer 编辑构造辅助
#include "pch.h"

#include "CadEditerWorkflowInternal.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace CadEditerWorkflowInternal
{
    QVector3D normalizedOrZero(const QVector3D& vector);
    QVector3D leftNormal(const QVector3D& direction);
    double cross2D(const QVector3D& first, const QVector3D& second);
    double dot2D(const QVector3D& first, const QVector3D& second);
    bool lineLineIntersection
    (
        const QVector3D& firstStart,
        const QVector3D& firstEnd,
        const QVector3D& secondStart,
        const QVector3D& secondEnd,
        QVector3D& intersection,
        double* firstParameter = nullptr,
        double* secondParameter = nullptr
    );
    bool extractLinearEntityPoints(const DRW_Entity* entity, QVector<QVector3D>& points, bool& closed);
    int entityColorIndexForCreation(const DRW_Entity* entity);
    QString entityLayerNameForCreation(const DRW_Entity* entity);
    bool angleOnArc(const DRW_Arc* arc, double angle);
    bool lineCircleIntersections
    (
        const QVector3D& lineStart,
        const QVector3D& lineEnd,
        const QVector3D& center,
        double radius,
        QVector<QVector3D>& intersections,
        QVector<double>* parameters = nullptr
    );
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

        return std::abs(cross2D(firstKeepDirection, secondKeepDirection)) > kGeometryEpsilon;
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
}
