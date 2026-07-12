#include "pch.h"

#include "RotaryTubeGeometryAnalyzer.h"

#include "CadItem.h"

#include <QSet>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{
    constexpr double kEpsilon = 1.0e-9;

    QVector<QVector3D> itemPath(CadItem* item)
    {
        QVector<QVector3D> path;

        if (item == nullptr)
        {
            return path;
        }

        item->rebuildRawPathPoints3D();
        path.reserve(static_cast<qsizetype>(item->rawPathPoints3D().size()));

        for (const RawPathPoint3D& point : item->rawPathPoints3D())
        {
            path.push_back(QVector3D(point.x, point.y, point.z));
        }

        return path;
    }

    double distance3D(const QVector3D& left, const QVector3D& right)
    {
        return static_cast<double>((left - right).length());
    }

    double cross2D(const QVector2D& origin, const QVector2D& left, const QVector2D& right)
    {
        return static_cast<double>(left.x() - origin.x()) * static_cast<double>(right.y() - origin.y())
            - static_cast<double>(left.y() - origin.y()) * static_cast<double>(right.x() - origin.x());
    }

    QVector<QVector2D> convexHull(QVector<QVector2D> points)
    {
        std::sort(points.begin(), points.end(), [](const QVector2D& left, const QVector2D& right)
        {
            return left.x() < right.x() || (left.x() == right.x() && left.y() < right.y());
        });

        points.erase(std::unique(points.begin(), points.end(), [](const QVector2D& left, const QVector2D& right)
        {
            return (left - right).lengthSquared() <= 1.0e-12f;
        }), points.end());

        if (points.size() < 3)
        {
            return {};
        }

        QVector<QVector2D> hull;
        hull.reserve(points.size() * 2);

        for (const QVector2D& point : points)
        {
            while (hull.size() >= 2 && cross2D(hull[hull.size() - 2], hull.back(), point) <= kEpsilon)
            {
                hull.pop_back();
            }

            hull.push_back(point);
        }

        const int lowerSize = hull.size();

        for (int index = points.size() - 2; index >= 0; --index)
        {
            const QVector2D point = points[index];

            while (hull.size() > lowerSize && cross2D(hull[hull.size() - 2], hull.back(), point) <= kEpsilon)
            {
                hull.pop_back();
            }

            hull.push_back(point);
        }

        hull.pop_back();
        return hull;
    }

    double pointSegmentDistance(const QVector2D& point, const QVector2D& start, const QVector2D& end)
    {
        const QVector2D edge = end - start;
        const double lengthSquared = static_cast<double>(edge.lengthSquared());

        if (lengthSquared <= kEpsilon)
        {
            return static_cast<double>((point - start).length());
        }

        const double factor = std::clamp
        (
            static_cast<double>(QVector2D::dotProduct(point - start, edge)) / lengthSquared,
            0.0,
            1.0
        );
        return static_cast<double>((point - (start + edge * static_cast<float>(factor))).length());
    }

    double distanceToHull(const QVector2D& point, const QVector<QVector2D>& hull)
    {
        double distance = std::numeric_limits<double>::max();

        for (int index = 0; index < hull.size(); ++index)
        {
            distance = std::min(distance, pointSegmentDistance(point, hull[index], hull[(index + 1) % hull.size()]));
        }

        return distance;
    }

    bool pointInsideConvexHull(const QVector2D& point, const QVector<QVector2D>& hull)
    {
        if (hull.size() < 3)
        {
            return false;
        }

        for (int index = 0; index < hull.size(); ++index)
        {
            if (cross2D(hull[index], hull[(index + 1) % hull.size()], point) < -kEpsilon)
            {
                return false;
            }
        }

        return true;
    }

    bool segmentEntersHullInterior
    (
        const QVector2D& start,
        const QVector2D& end,
        const QVector<QVector2D>& hull,
        double interiorTolerance
    )
    {
        QVector<double> parameters{ 0.0, 1.0 };
        const QVector2D segment = end - start;

        for (int index = 0; index < hull.size(); ++index)
        {
            const QVector2D edgeStart = hull[index];
            const QVector2D edge = hull[(index + 1) % hull.size()] - edgeStart;
            const double denominator = cross2D(QVector2D(), segment, edge);

            if (std::abs(denominator) <= kEpsilon)
            {
                continue;
            }

            const QVector2D offset = edgeStart - start;
            const double segmentParameter = cross2D(QVector2D(), offset, edge) / denominator;
            const double edgeParameter = cross2D(QVector2D(), offset, segment) / denominator;

            if (segmentParameter > kEpsilon && segmentParameter < 1.0 - kEpsilon
                && edgeParameter >= -kEpsilon && edgeParameter <= 1.0 + kEpsilon)
            {
                parameters.push_back(segmentParameter);
            }
        }

        std::sort(parameters.begin(), parameters.end());
        parameters.erase(std::unique(parameters.begin(), parameters.end(), [](double left, double right)
        {
            return std::abs(left - right) <= kEpsilon;
        }), parameters.end());

        for (int index = 0; index + 1 < parameters.size(); ++index)
        {
            const double parameter = (parameters[index] + parameters[index + 1]) * 0.5;
            const QVector2D sample = start + segment * static_cast<float>(parameter);

            if (pointInsideConvexHull(sample, hull)
                && distanceToHull(sample, hull) > interiorTolerance)
            {
                return true;
            }
        }

        return false;
    }

    double distancePointToPath(const QVector3D& point, const QVector<QVector3D>& path);

    bool pathsTouch(const QVector<QVector3D>& left, const QVector<QVector3D>& right, double tolerance)
    {
        if (left.isEmpty() || right.isEmpty())
        {
            return false;
        }

        return distancePointToPath(left.front(), right) <= tolerance
            || distancePointToPath(left.back(), right) <= tolerance
            || distancePointToPath(right.front(), left) <= tolerance
            || distancePointToPath(right.back(), left) <= tolerance;
    }

    QVector<int> selectedConnectedItems
    (
        const QVector<CadItem*>& selectedItems,
        const QVector<CadItem*>& sceneItems,
        const QVector<QVector<QVector3D>>& paths,
        double tolerance
    )
    {
        QVector<int> pending;
        QVector<bool> included(sceneItems.size(), false);

        for (CadItem* selected : selectedItems)
        {
            const int index = sceneItems.indexOf(selected);

            if (index >= 0)
            {
                included[index] = true;
                pending.push_back(index);
                break;
            }
        }

        for (int pendingIndex = 0; pendingIndex < pending.size(); ++pendingIndex)
        {
            const int current = pending[pendingIndex];

            for (int candidate = 0; candidate < sceneItems.size(); ++candidate)
            {
                if (!included[candidate] && pathsTouch(paths[current], paths[candidate], tolerance))
                {
                    included[candidate] = true;
                    pending.push_back(candidate);
                }
            }
        }

        return pending;
    }

    QVector<int> peelDanglingItems
    (
        const QVector<int>& component,
        const QVector<QVector<QVector3D>>& paths,
        double tolerance
    )
    {
        QVector<bool> active(component.size(), true);
        bool changed = true;

        while (changed)
        {
            changed = false;
            QVector<bool> remove(component.size(), false);

            for (int localIndex = 0; localIndex < component.size(); ++localIndex)
            {
                if (!active[localIndex])
                {
                    continue;
                }

                const QVector<QVector3D>& path = paths[component[localIndex]];

                if (path.size() < 2 || distance3D(path.front(), path.back()) <= tolerance)
                {
                    continue;
                }

                int connectedEnds = 0;

                for (const QVector3D& endpoint : { path.front(), path.back() })
                {
                    bool connected = false;

                    for (int other = 0; other < component.size() && !connected; ++other)
                    {
                        if (other == localIndex || !active[other] || paths[component[other]].isEmpty())
                        {
                            continue;
                        }

                        connected = distance3D(endpoint, paths[component[other]].front()) <= tolerance
                            || distance3D(endpoint, paths[component[other]].back()) <= tolerance;
                    }

                    connectedEnds += connected ? 1 : 0;
                }

                if (connectedEnds < 2)
                {
                    remove[localIndex] = true;
                }
            }

            for (int index = 0; index < remove.size(); ++index)
            {
                if (remove[index])
                {
                    active[index] = false;
                    changed = true;
                }
            }
        }

        QVector<int> core;

        for (int index = 0; index < component.size(); ++index)
        {
            if (active[index])
            {
                core.push_back(component[index]);
            }
        }

        return core;
    }

    QVector2D projectPoint(const QVector3D& point, int projection)
    {
        if (projection == 0) return QVector2D(point.x(), point.y());
        if (projection == 1) return QVector2D(point.x(), point.z());
        return QVector2D(point.y(), point.z());
    }

    double hullArea(const QVector<QVector2D>& hull)
    {
        double area = 0.0;

        for (int index = 0; index < hull.size(); ++index)
        {
            const QVector2D& start = hull[index];
            const QVector2D& end = hull[(index + 1) % hull.size()];
            area += static_cast<double>(start.x()) * end.y() - static_cast<double>(end.x()) * start.y();
        }

        return std::abs(area) * 0.5;
    }

    double distancePointToPath(const QVector3D& point, const QVector<QVector3D>& path)
    {
        double best = std::numeric_limits<double>::max();

        for (int index = 0; index + 1 < path.size(); ++index)
        {
            const QVector3D edge = path[index + 1] - path[index];
            const double lengthSquared = static_cast<double>(edge.lengthSquared());
            const double factor = lengthSquared <= kEpsilon ? 0.0 : std::clamp
            (
                static_cast<double>(QVector3D::dotProduct(point - path[index], edge)) / lengthSquared,
                0.0,
                1.0
            );
            best = std::min(best, distance3D(point, path[index] + edge * static_cast<float>(factor)));
        }

        return best;
    }

    struct TopologyNode
    {
        QVector3D spatialPoint;
        QVector2D projectedPoint;
        QVector<int> outgoingHalfEdges;
    };

    struct TopologyEdge
    {
        int firstNode = -1;
        int secondNode = -1;
        int itemIndex = -1;
    };

    int findOrAddNode
    (
        QVector<TopologyNode>& nodes,
        const QVector3D& spatialPoint,
        const QVector2D& projectedPoint,
        double tolerance
    )
    {
        for (int index = 0; index < nodes.size(); ++index)
        {
            if (distance3D(nodes[index].spatialPoint, spatialPoint) <= tolerance)
            {
                return index;
            }
        }

        nodes.push_back({ spatialPoint, projectedPoint, {} });
        return nodes.size() - 1;
    }

    double pointSegmentParameter3D
    (
        const QVector3D& point,
        const QVector3D& start,
        const QVector3D& end,
        double& distance
    )
    {
        const QVector3D edge = end - start;
        const double lengthSquared = static_cast<double>(edge.lengthSquared());
        const double factor = lengthSquared <= kEpsilon ? 0.0 : std::clamp
        (
            static_cast<double>(QVector3D::dotProduct(point - start, edge)) / lengthSquared,
            0.0,
            1.0
        );
        distance = distance3D(point, start + edge * static_cast<float>(factor));
        return factor;
    }

    QSet<int> buildLargestOuterBoundary
    (
        const QVector<int>& component,
        const QVector<QVector<QVector3D>>& paths,
        int projection,
        double tolerance
    )
    {
        QVector<TopologyNode> nodes;
        QVector<TopologyEdge> edges;

        for (int itemIndex : component)
        {
            const QVector<QVector3D>& path = paths[itemIndex];

            for (int segmentIndex = 0; segmentIndex + 1 < path.size(); ++segmentIndex)
            {
                const QVector3D start = path[segmentIndex];
                const QVector3D end = path[segmentIndex + 1];
                QVector<double> splitParameters{ 0.0, 1.0 };

                for (int otherItemIndex : component)
                {
                    const QVector<QVector3D>& otherPath = paths[otherItemIndex];

                    if (otherPath.isEmpty())
                    {
                        continue;
                    }

                    for (const QVector3D& endpoint : { otherPath.front(), otherPath.back() })
                    {
                        double endpointDistance = 0.0;
                        const double factor = pointSegmentParameter3D(endpoint, start, end, endpointDistance);

                        if (endpointDistance <= tolerance
                            && factor > kEpsilon && factor < 1.0 - kEpsilon)
                        {
                            splitParameters.push_back(factor);
                        }
                    }
                }

                std::sort(splitParameters.begin(), splitParameters.end());
                splitParameters.erase
                (
                    std::unique(splitParameters.begin(), splitParameters.end(), [](double left, double right)
                    {
                        return std::abs(left - right) <= kEpsilon;
                    }),
                    splitParameters.end()
                );

                for (int splitIndex = 0; splitIndex + 1 < splitParameters.size(); ++splitIndex)
                {
                    const QVector3D firstPoint = start + (end - start) * static_cast<float>(splitParameters[splitIndex]);
                    const QVector3D secondPoint = start + (end - start) * static_cast<float>(splitParameters[splitIndex + 1]);
                    const int firstNode = findOrAddNode
                    (
                        nodes,
                        firstPoint,
                        projectPoint(firstPoint, projection),
                        tolerance
                    );
                    const int secondNode = findOrAddNode
                    (
                        nodes,
                        secondPoint,
                        projectPoint(secondPoint, projection),
                        tolerance
                    );

                    if (firstNode != secondNode)
                    {
                        edges.push_back({ firstNode, secondNode, itemIndex });
                    }
                }
            }
        }

        if (edges.size() < 3)
        {
            return {};
        }

        for (int edgeIndex = 0; edgeIndex < edges.size(); ++edgeIndex)
        {
            nodes[edges[edgeIndex].firstNode].outgoingHalfEdges.push_back(edgeIndex * 2);
            nodes[edges[edgeIndex].secondNode].outgoingHalfEdges.push_back(edgeIndex * 2 + 1);
        }

        auto halfEdgeStart = [&edges](int halfEdge)
        {
            const TopologyEdge& edge = edges[halfEdge / 2];
            return (halfEdge % 2 == 0) ? edge.firstNode : edge.secondNode;
        };
        auto halfEdgeEnd = [&edges](int halfEdge)
        {
            const TopologyEdge& edge = edges[halfEdge / 2];
            return (halfEdge % 2 == 0) ? edge.secondNode : edge.firstNode;
        };

        for (TopologyNode& node : nodes)
        {
            std::sort(node.outgoingHalfEdges.begin(), node.outgoingHalfEdges.end(), [&](int left, int right)
            {
                const QVector2D leftDirection = nodes[halfEdgeEnd(left)].projectedPoint - node.projectedPoint;
                const QVector2D rightDirection = nodes[halfEdgeEnd(right)].projectedPoint - node.projectedPoint;
                return std::atan2(leftDirection.y(), leftDirection.x())
                    < std::atan2(rightDirection.y(), rightDirection.x());
            });
        }

        QVector<bool> visited(edges.size() * 2, false);
        QSet<int> largestBoundaryItems;
        double largestArea = 0.0;

        for (int initialHalfEdge = 0; initialHalfEdge < visited.size(); ++initialHalfEdge)
        {
            if (visited[initialHalfEdge])
            {
                continue;
            }

            QVector<QVector2D> face;
            QSet<int> faceItems;
            int currentHalfEdge = initialHalfEdge;
            bool closed = false;

            for (int step = 0; step <= visited.size(); ++step)
            {
                if (visited[currentHalfEdge])
                {
                    closed = currentHalfEdge == initialHalfEdge;
                    break;
                }

                visited[currentHalfEdge] = true;
                face.push_back(nodes[halfEdgeStart(currentHalfEdge)].projectedPoint);
                faceItems.insert(edges[currentHalfEdge / 2].itemIndex);
                const int endNode = halfEdgeEnd(currentHalfEdge);
                const QVector<int>& outgoing = nodes[endNode].outgoingHalfEdges;
                const int reverseIndex = outgoing.indexOf(currentHalfEdge ^ 1);

                if (reverseIndex < 0 || outgoing.isEmpty())
                {
                    break;
                }

                currentHalfEdge = outgoing[(reverseIndex - 1 + outgoing.size()) % outgoing.size()];
            }

            if (!closed || face.size() < 3)
            {
                continue;
            }

            const double area = hullArea(face);

            if (area > largestArea)
            {
                largestArea = area;
                largestBoundaryItems = std::move(faceItems);
            }
        }

        return largestArea > tolerance * tolerance ? largestBoundaryItems : QSet<int>();
    }

    double estimateRoundedCornerRadius(const QVector<QVector2D>& hull, double tolerance)
    {
        if (hull.size() < 4)
        {
            return 0.0;
        }

        double minY = hull.front().x();
        double maxY = minY;
        double minZ = hull.front().y();
        double maxZ = minZ;

        for (const QVector2D& point : hull)
        {
            minY = std::min(minY, static_cast<double>(point.x()));
            maxY = std::max(maxY, static_cast<double>(point.x()));
            minZ = std::min(minZ, static_cast<double>(point.y()));
            maxZ = std::max(maxZ, static_cast<double>(point.y()));
        }

        const double cornerSearchRange = std::min(maxY - minY, maxZ - minZ) * 0.5;
        const double exactSupportTolerance = std::max(1.0e-5, cornerSearchRange * 1.0e-6);
        QVector<double> cornerRadii;

        for (int ySide : { -1, 1 })
        {
            for (int zSide : { -1, 1 })
            {
                const double cornerY = ySide < 0 ? minY : maxY;
                const double cornerZ = zSide < 0 ? minZ : maxZ;
                bool hasSharpCorner = false;
                QVector<double> arcCandidates;
                QVector<double> yTangentCandidates;
                QVector<double> zTangentCandidates;

                for (const QVector2D& point : hull)
                {
                    const double yOffset = std::abs(static_cast<double>(point.x()) - cornerY);
                    const double zOffset = std::abs(static_cast<double>(point.y()) - cornerZ);

                    if (yOffset > cornerSearchRange || zOffset > cornerSearchRange)
                    {
                        continue;
                    }

                    if (std::hypot(yOffset, zOffset) <= exactSupportTolerance)
                    {
                        hasSharpCorner = true;
                        break;
                    }

                    const double candidateRadius = yOffset + zOffset + std::sqrt(2.0 * yOffset * zOffset);

                    if (yOffset > exactSupportTolerance && zOffset > exactSupportTolerance)
                    {
                        arcCandidates.push_back(candidateRadius);
                    }

                    if (zOffset <= exactSupportTolerance && yOffset > exactSupportTolerance)
                    {
                        yTangentCandidates.push_back(yOffset);
                    }

                    if (yOffset <= exactSupportTolerance && zOffset > exactSupportTolerance)
                    {
                        zTangentCandidates.push_back(zOffset);
                    }
                }

                if (hasSharpCorner)
                {
                    cornerRadii.push_back(0.0);
                    continue;
                }

                const bool hasYTangent = !yTangentCandidates.isEmpty();
                const bool hasZTangent = !zTangentCandidates.isEmpty();

                if (!arcCandidates.isEmpty())
                {
                    std::sort(arcCandidates.begin(), arcCandidates.end());
                    const double arcRadius = arcCandidates[arcCandidates.size() / 2];
                    const int lowIndex = arcCandidates.size() / 10;
                    const int highIndex = ((arcCandidates.size() - 1) * 9) / 10;
                    const double relativeSpread = arcRadius > kEpsilon
                        ? (arcCandidates[highIndex] - arcCandidates[lowIndex]) / arcRadius
                        : 0.0;

                    if (relativeSpread <= 0.08 || !hasYTangent || !hasZTangent)
                    {
                        cornerRadii.push_back(arcRadius);
                        continue;
                    }

                    const double yRadius = hasYTangent
                        ? *std::min_element(yTangentCandidates.begin(), yTangentCandidates.end())
                        : 0.0;
                    const double zRadius = hasZTangent
                        ? *std::min_element(zTangentCandidates.begin(), zTangentCandidates.end())
                        : 0.0;
                    cornerRadii.push_back(hasYTangent && hasZTangent
                        ? (yRadius + zRadius) * 0.5
                        : (hasYTangent ? yRadius : zRadius));
                    continue;
                }

                if (hasYTangent || hasZTangent)
                {
                    const double yRadius = hasYTangent
                        ? *std::min_element(yTangentCandidates.begin(), yTangentCandidates.end())
                        : 0.0;
                    const double zRadius = hasZTangent
                        ? *std::min_element(zTangentCandidates.begin(), zTangentCandidates.end())
                        : 0.0;
                    cornerRadii.push_back(hasYTangent && hasZTangent
                        ? (yRadius + zRadius) * 0.5
                        : (hasYTangent ? yRadius : zRadius));
                }
            }
        }

        if (cornerRadii.isEmpty())
        {
            return 0.0;
        }

        std::sort(cornerRadii.begin(), cornerRadii.end());
        return cornerRadii[cornerRadii.size() / 2] <= tolerance
            ? 0.0
            : cornerRadii[cornerRadii.size() / 2];
    }

    double verticalSectionProjectionFactor(const DRW_Coord& extrusion)
    {
        QVector3D normal(extrusion.x, extrusion.y, extrusion.z);

        if (normal.lengthSquared() <= 1.0e-12f)
        {
            normal = QVector3D(0.0f, 0.0f, 1.0f);
        }
        else
        {
            normal.normalize();
        }

        return std::clamp(std::abs(static_cast<double>(normal.x())), 0.0, 1.0);
    }

    void appendPolylineCurvatureRadii
    (
        const QVector<QVector3D>& path,
        double maximumRadius,
        QVector<double>& radii
    )
    {
        if (path.size() < 5)
        {
            return;
        }

        constexpr int sampleStep = 2;

        for (int index = sampleStep; index + sampleStep < path.size(); ++index)
        {
            const QVector3D first = path[index - sampleStep];
            const QVector3D middle = path[index];
            const QVector3D last = path[index + sampleStep];
            const QVector3D firstChord = middle - first;
            const QVector3D secondChord = last - middle;
            const QVector3D fullChord = last - first;
            const double firstLength = static_cast<double>(firstChord.length());
            const double secondLength = static_cast<double>(secondChord.length());
            const double fullLength = static_cast<double>(fullChord.length());
            QVector3D cross = QVector3D::crossProduct(firstChord, secondChord);
            const double crossLength = static_cast<double>(cross.length());

            if (firstLength <= kEpsilon
                || secondLength <= kEpsilon
                || fullLength <= kEpsilon
                || crossLength / (firstLength * secondLength) < 0.002)
            {
                continue;
            }

            const double localRadius = firstLength * secondLength * fullLength / (2.0 * crossLength);

            if (!std::isfinite(localRadius) || localRadius <= kEpsilon || localRadius > maximumRadius)
            {
                continue;
            }

            cross.normalize();
            const double correctedRadius = localRadius * std::abs(static_cast<double>(cross.x()));

            if (correctedRadius > kEpsilon)
            {
                radii.push_back(correctedRadius);
            }
        }
    }

    double estimateCornerRadiusFromEntities
    (
        const QVector<CadItem*>& outerBoundaryItems,
        const QVector<QVector<QVector3D>>& paths,
        const QVector<CadItem*>& sceneItems,
        double maximumRadius,
        double fallbackRadius
    )
    {
        QVector<double> exactRadii;
        QVector<double> polylineRadii;

        for (CadItem* item : outerBoundaryItems)
        {
            if (item == nullptr || item->m_nativeEntity == nullptr)
            {
                continue;
            }

            if (item->m_type == DRW::ETYPE::ARC || item->m_type == DRW::ETYPE::CIRCLE)
            {
                const DRW_Circle* circle = static_cast<const DRW_Circle*>(item->m_nativeEntity);
                const double radius = circle->radious * verticalSectionProjectionFactor(circle->extPoint);

                if (radius > kEpsilon && radius <= maximumRadius)
                {
                    exactRadii.push_back(radius);
                }

                continue;
            }

            if (item->m_type == DRW::ETYPE::ELLIPSE)
            {
                const DRW_Ellipse* ellipse = static_cast<const DRW_Ellipse*>(item->m_nativeEntity);
                const double majorSemiAxis = std::hypot
                (
                    std::hypot(ellipse->secPoint.x, ellipse->secPoint.y),
                    ellipse->secPoint.z
                ) * std::max(1.0, ellipse->ratio);
                const double radius = majorSemiAxis * verticalSectionProjectionFactor(ellipse->extPoint);

                if (radius > kEpsilon && radius <= maximumRadius)
                {
                    exactRadii.push_back(radius);
                }

                continue;
            }

            if (item->m_type == DRW::ETYPE::LWPOLYLINE || item->m_type == DRW::ETYPE::POLYLINE)
            {
                const int itemIndex = sceneItems.indexOf(item);

                if (itemIndex >= 0 && itemIndex < paths.size())
                {
                    appendPolylineCurvatureRadii(paths[itemIndex], maximumRadius, polylineRadii);
                }
            }
        }

        QVector<double>& candidates = !exactRadii.isEmpty() ? exactRadii : polylineRadii;

        if (candidates.isEmpty())
        {
            return fallbackRadius;
        }

        std::sort(candidates.begin(), candidates.end());
        return candidates[candidates.size() / 2];
    }
}

RotaryTubeSectionModel RotaryTubeGeometryAnalyzer::buildSectionModel
(
    const QVector<CadItem*>& selectedItems,
    const QVector<CadItem*>& sceneItems,
    double connectionTolerance
)
{
    RotaryTubeSectionModel model;

    if (selectedItems.isEmpty())
    {
        model.errorMessage = QStringLiteral("请先选中方管垂直截面中的一个或部分图元。");
        return model;
    }

    QVector<QVector<QVector3D>> paths;
    paths.reserve(sceneItems.size());

    for (CadItem* item : sceneItems)
    {
        paths.push_back(itemPath(item));
    }

    const QVector<int> component = selectedConnectedItems(selectedItems, sceneItems, paths, connectionTolerance);

    for (CadItem* selectedItem : selectedItems)
    {
        const int selectedIndex = sceneItems.indexOf(selectedItem);

        if (selectedIndex < 0 || !component.contains(selectedIndex))
        {
            model.errorMessage = QStringLiteral("请选择同一个连续方管截面中的图元，不要同时选择多个独立轮廓。");
            return model;
        }
    }

    const QVector<int> core = peelDanglingItems(component, paths, connectionTolerance);

    if (core.isEmpty())
    {
        model.errorMessage = QStringLiteral("选中图元未能形成闭合或近似闭合的方管垂直截面；无用支线已自动忽略。");
        return model;
    }

    const QSet<int> outerBoundaryItems = buildLargestOuterBoundary(core, paths, 2, connectionTolerance);

    if (outerBoundaryItems.isEmpty())
    {
        model.errorMessage = QStringLiteral("未能从选中图元组中提取唯一的最大闭合外轮廓。");
        return model;
    }

    QVector<QVector2D> sectionPoints;

    for (int itemIndex : outerBoundaryItems)
    {
        model.outerBoundaryItems.push_back(sceneItems[itemIndex]);

        for (const QVector3D& point : paths[itemIndex])
        {
            sectionPoints.push_back(QVector2D(point.y(), point.z()));
        }
    }

    model.sectionHull = convexHull(std::move(sectionPoints));

    if (model.sectionHull.size() < 3 || hullArea(model.sectionHull) <= kEpsilon)
    {
        model.sectionHull.clear();
        model.errorMessage = QStringLiteral("最大外轮廓在 YZ 平面上的投影无效，无法识别方管垂直截面。");
        return model;
    }

    double minY = model.sectionHull.front().x();
    double maxY = minY;
    double minZ = model.sectionHull.front().y();
    double maxZ = minZ;

    for (const QVector2D& point : model.sectionHull)
    {
        minY = std::min(minY, static_cast<double>(point.x()));
        maxY = std::max(maxY, static_cast<double>(point.x()));
        minZ = std::min(minZ, static_cast<double>(point.y()));
        maxZ = std::max(maxZ, static_cast<double>(point.y()));
    }

    model.yLength = maxY - minY;
    model.zWidth = maxZ - minZ;
    const double fallbackRadius = estimateRoundedCornerRadius(model.sectionHull, 0.05);
    model.cornerRadius = estimateCornerRadiusFromEntities
    (
        model.outerBoundaryItems,
        paths,
        sceneItems,
        std::min(model.yLength, model.zWidth) * 0.5,
        fallbackRadius
    );
    model.valid = true;
    return model;
}

RotaryInternalPathResult RotaryTubeGeometryAnalyzer::findInternalPaths
(
    const RotaryTubeSectionModel& model,
    const QVector<CadItem*>& sceneItems,
    double connectionTolerance
)
{
    RotaryInternalPathResult result;

    const bool hasSectionModel = model.valid && model.sectionHull.size() >= 3;

    QVector<QVector<QVector3D>> paths;
    paths.reserve(sceneItems.size());

    for (CadItem* item : sceneItems)
    {
        paths.push_back(itemPath(item));
    }

    // A shallow entry is still unsafe for the laser head. Keep only a small
    // numerical margin so boundary noise is not treated as an interior cut.
    const double interiorTolerance = std::max(0.01, connectionTolerance * 0.02);
    QVector<bool> physicalInterior(sceneItems.size(), false);

    for (int itemIndex = 0; itemIndex < sceneItems.size(); ++itemIndex)
    {
        CadItem* item = sceneItems[itemIndex];
        const QVector<QVector3D>& path = paths[itemIndex];

        if (!hasSectionModel
            || item == nullptr
            || path.isEmpty()
            || item->m_rotaryEndCutRole != RotaryEndCutRole::None)
        {
            continue;
        }

        bool entersPhysicalInterior = false;

        for (const QVector3D& point : path)
        {
            const QVector2D sectionPoint(point.y(), point.z());

            if (pointInsideConvexHull(sectionPoint, model.sectionHull)
                && distanceToHull(sectionPoint, model.sectionHull) > interiorTolerance)
            {
                entersPhysicalInterior = true;
                break;
            }
        }

        for (int pointIndex = 0; !entersPhysicalInterior && pointIndex + 1 < path.size(); ++pointIndex)
        {
            entersPhysicalInterior = segmentEntersHullInterior
            (
                QVector2D(path[pointIndex].y(), path[pointIndex].z()),
                QVector2D(path[pointIndex + 1].y(), path[pointIndex + 1].z()),
                model.sectionHull,
                interiorTolerance
            );
        }

        if (entersPhysicalInterior)
        {
            physicalInterior[itemIndex] = true;
            result.physicalInteriorItems.push_back(item);
        }
    }

    QVector<bool> visited(sceneItems.size(), false);

    for (int seed = 0; seed < sceneItems.size(); ++seed)
    {
        if (visited[seed] || physicalInterior[seed] || paths[seed].size() < 2)
        {
            continue;
        }

        QVector<int> component{ seed };
        visited[seed] = true;

        for (int cursor = 0; cursor < component.size(); ++cursor)
        {
            for (int candidate = 0; candidate < sceneItems.size(); ++candidate)
            {
                if (!visited[candidate]
                    && !physicalInterior[candidate]
                    && pathsTouch(paths[component[cursor]], paths[candidate], connectionTolerance))
                {
                    visited[candidate] = true;
                    component.push_back(candidate);
                }
            }
        }

        const QVector<int> closedCore = peelDanglingItems(component, paths, connectionTolerance);

        if (closedCore.isEmpty())
        {
            continue;
        }

        int bestProjection = 0;
        double bestArea = 0.0;

        for (int projection = 0; projection < 3; ++projection)
        {
            QVector<QVector2D> projectedPoints;

            for (int itemIndex : closedCore)
            {
                for (const QVector3D& point : paths[itemIndex])
                {
                    projectedPoints.push_back(projectPoint(point, projection));
                }
            }

            QVector<QVector2D> hull = convexHull(std::move(projectedPoints));
            const double area = hullArea(hull);

            if (area > bestArea)
            {
                bestArea = area;
                bestProjection = projection;
            }
        }

        const QSet<int> outerBoundaryItems = buildLargestOuterBoundary
        (
            closedCore,
            paths,
            bestProjection,
            connectionTolerance
        );

        if (outerBoundaryItems.isEmpty())
        {
            continue;
        }

        for (int itemIndex : component)
        {
            CadItem* item = sceneItems[itemIndex];

            if (item == nullptr
                || item->m_rotaryEndCutRole != RotaryEndCutRole::None
                || outerBoundaryItems.contains(itemIndex))
            {
                continue;
            }

            result.topologicalInteriorItems.push_back(item);
        }
    }

    return result;
}
