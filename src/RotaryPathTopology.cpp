#include "pch.h"

#include "RotaryPathTopology.h"

#include "CadItem.h"

#include <QHash>
#include <QSet>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <vector>

namespace
{
    constexpr double kEpsilon = 1.0e-9;

    struct Marker
    {
        int recordIndex = -1;
        double position = 0.0;
        QVector3D point;
    };

    struct GraphEdge
    {
        int firstNode = -1;
        int secondNode = -1;
        int recordIndex = -1;
        QVector<QVector3D> points;
        double length = 0.0;
    };

    struct GraphData
    {
        QVector<Marker> markers;
        QVector<QVector<int>> markerIndicesByRecord;
        QVector<QVector3D> nodes;
        QVector<GraphEdge> edges;
        QVector<QVector<int>> incidentEdges;
    };

    struct LoopCandidate
    {
        QVector<QVector3D> orderedPath;
        QSet<int> recordIndices;
        bool approximatelyClosed = false;
        double closureGap = 0.0;
        double length = 0.0;
        double projectedArea = 0.0;
    };

    class DisjointSet
    {
    public:
        explicit DisjointSet(int count)
            : m_parent(static_cast<size_t>(count)), m_rank(static_cast<size_t>(count), 0)
        {
            for (int index = 0; index < count; ++index)
            {
                m_parent[static_cast<size_t>(index)] = index;
            }
        }

        int find(int index)
        {
            int& parent = m_parent[static_cast<size_t>(index)];

            if (parent != index)
            {
                parent = find(parent);
            }

            return parent;
        }

        void unite(int left, int right)
        {
            left = find(left);
            right = find(right);

            if (left == right)
            {
                return;
            }

            int& leftRank = m_rank[static_cast<size_t>(left)];
            int& rightRank = m_rank[static_cast<size_t>(right)];

            if (leftRank < rightRank)
            {
                std::swap(left, right);
            }

            m_parent[static_cast<size_t>(right)] = left;

            if (leftRank == rightRank)
            {
                ++leftRank;
            }
        }

    private:
        std::vector<int> m_parent;
        std::vector<int> m_rank;
    };

    double distance3D(const QVector3D& left, const QVector3D& right)
    {
        return static_cast<double>((left - right).length());
    }

    double pathLength(const QVector<QVector3D>& path)
    {
        double length = 0.0;

        for (int index = 1; index < path.size(); ++index)
        {
            length += distance3D(path[index - 1], path[index]);
        }

        return length;
    }

    bool nativeEntityIsClosed(const CadItem* item)
    {
        if (item == nullptr || item->m_nativeEntity == nullptr)
        {
            return false;
        }

        switch (item->m_type)
        {
        case DRW::CIRCLE:
            return true;
        case DRW::ELLIPSE:
        {
            const DRW_Ellipse* ellipse = static_cast<const DRW_Ellipse*>(item->m_nativeEntity);
            const double span = std::abs(ellipse->endparam - ellipse->staparam);
            return std::abs(span - 2.0 * M_PI) <= 1.0e-6 || span >= 2.0 * M_PI - 1.0e-6;
        }
        case DRW::LWPOLYLINE:
            return (static_cast<const DRW_LWPolyline*>(item->m_nativeEntity)->flags & 1) != 0;
        case DRW::POLYLINE:
            return (static_cast<const DRW_Polyline*>(item->m_nativeEntity)->flags & 1) != 0;
        default:
            return false;
        }
    }

    QVector<QVector3D> cleanPath(CadItem* item, double minimumEdgeLength)
    {
        QVector<QVector3D> result;

        if (item == nullptr)
        {
            return result;
        }

        item->rebuildRawPathPoints3D();
        result.reserve(static_cast<qsizetype>(item->rawPathPoints3D().size()));

        for (const RawPathPoint3D& rawPoint : item->rawPathPoints3D())
        {
            const QVector3D point(rawPoint.x, rawPoint.y, rawPoint.z);

            if (result.isEmpty() || distance3D(result.back(), point) > minimumEdgeLength)
            {
                result.push_back(point);
            }
        }

        return result;
    }

    double pointSegmentDistance
    (
        const QVector3D& point,
        const QVector3D& start,
        const QVector3D& end,
        double* parameter = nullptr,
        QVector3D* projection = nullptr
    )
    {
        const QVector3D edge = end - start;
        const double lengthSquared = static_cast<double>(edge.lengthSquared());
        const double factor = lengthSquared <= kEpsilon
            ? 0.0
            : std::clamp(static_cast<double>(QVector3D::dotProduct(point - start, edge)) / lengthSquared, 0.0, 1.0);
        const QVector3D projected = start + edge * static_cast<float>(factor);

        if (parameter != nullptr)
        {
            *parameter = factor;
        }

        if (projection != nullptr)
        {
            *projection = projected;
        }

        return distance3D(point, projected);
    }

    double distancePointToPath
    (
        const QVector3D& point,
        const QVector<QVector3D>& path,
        double* pathPosition = nullptr,
        QVector3D* projection = nullptr
    )
    {
        double bestDistance = std::numeric_limits<double>::max();

        for (int segmentIndex = 0; segmentIndex + 1 < path.size(); ++segmentIndex)
        {
            double parameter = 0.0;
            QVector3D candidateProjection;
            const double distance = pointSegmentDistance
            (
                point,
                path[segmentIndex],
                path[segmentIndex + 1],
                &parameter,
                &candidateProjection
            );

            if (distance < bestDistance)
            {
                bestDistance = distance;

                if (pathPosition != nullptr)
                {
                    *pathPosition = static_cast<double>(segmentIndex) + parameter;
                }

                if (projection != nullptr)
                {
                    *projection = candidateProjection;
                }
            }
        }

        return bestDistance;
    }

    double segmentSegmentDistance
    (
        const QVector3D& firstStart,
        const QVector3D& firstEnd,
        const QVector3D& secondStart,
        const QVector3D& secondEnd,
        double& firstParameter,
        double& secondParameter,
        QVector3D& firstPoint,
        QVector3D& secondPoint
    )
    {
        const QVector3D firstDirection = firstEnd - firstStart;
        const QVector3D secondDirection = secondEnd - secondStart;
        const QVector3D offset = firstStart - secondStart;
        const double a = QVector3D::dotProduct(firstDirection, firstDirection);
        const double e = QVector3D::dotProduct(secondDirection, secondDirection);
        const double f = QVector3D::dotProduct(secondDirection, offset);

        if (a <= kEpsilon && e <= kEpsilon)
        {
            firstParameter = secondParameter = 0.0;
        }
        else if (a <= kEpsilon)
        {
            firstParameter = 0.0;
            secondParameter = std::clamp(f / e, 0.0, 1.0);
        }
        else
        {
            const double c = QVector3D::dotProduct(firstDirection, offset);

            if (e <= kEpsilon)
            {
                secondParameter = 0.0;
                firstParameter = std::clamp(-c / a, 0.0, 1.0);
            }
            else
            {
                const double b = QVector3D::dotProduct(firstDirection, secondDirection);
                const double denominator = a * e - b * b;
                firstParameter = std::abs(denominator) > kEpsilon
                    ? std::clamp((b * f - c * e) / denominator, 0.0, 1.0)
                    : 0.0;
                secondParameter = (b * firstParameter + f) / e;

                if (secondParameter < 0.0)
                {
                    secondParameter = 0.0;
                    firstParameter = std::clamp(-c / a, 0.0, 1.0);
                }
                else if (secondParameter > 1.0)
                {
                    secondParameter = 1.0;
                    firstParameter = std::clamp((b - c) / a, 0.0, 1.0);
                }
            }
        }

        firstPoint = firstStart + firstDirection * static_cast<float>(firstParameter);
        secondPoint = secondStart + secondDirection * static_cast<float>(secondParameter);
        return distance3D(firstPoint, secondPoint);
    }

    bool pathsConnected
    (
        const RotaryPathTopologyRecord& left,
        const RotaryPathTopologyRecord& right,
        const RotaryPathTopologyTolerance& tolerance
    )
    {
        if (left.points.size() < 2 || right.points.size() < 2)
        {
            return false;
        }

        for (const QVector3D& endpoint : { left.points.front(), left.points.back() })
        {
            if (distancePointToPath(endpoint, right.points) <= tolerance.nodeSnap)
            {
                return true;
            }
        }

        for (const QVector3D& endpoint : { right.points.front(), right.points.back() })
        {
            if (distancePointToPath(endpoint, left.points) <= tolerance.nodeSnap)
            {
                return true;
            }
        }

        for (int leftSegment = 0; leftSegment + 1 < left.points.size(); ++leftSegment)
        {
            for (int rightSegment = 0; rightSegment + 1 < right.points.size(); ++rightSegment)
            {
                double leftParameter = 0.0;
                double rightParameter = 0.0;
                QVector3D leftPoint;
                QVector3D rightPoint;

                if (segmentSegmentDistance
                (
                    left.points[leftSegment],
                    left.points[leftSegment + 1],
                    right.points[rightSegment],
                    right.points[rightSegment + 1],
                    leftParameter,
                    rightParameter,
                    leftPoint,
                    rightPoint
                ) <= tolerance.intersection)
                {
                    return true;
                }
            }
        }

        return false;
    }

    void addMarker(QVector<Marker>& markers, int recordIndex, double position, const QVector3D& point)
    {
        for (const Marker& marker : markers)
        {
            if (marker.recordIndex == recordIndex && std::abs(marker.position - position) <= 1.0e-7)
            {
                return;
            }
        }

        markers.push_back({ recordIndex, position, point });
    }

    QVector3D pointAtPosition(const QVector<QVector3D>& path, double position)
    {
        const int segmentIndex = std::clamp
        (
            static_cast<int>(std::floor(position)),
            0,
            static_cast<int>(path.size()) - 2
        );
        const double parameter = std::clamp(position - segmentIndex, 0.0, 1.0);
        return path[segmentIndex] + (path[segmentIndex + 1] - path[segmentIndex]) * static_cast<float>(parameter);
    }

    QVector<QVector3D> pathBetween(const QVector<QVector3D>& path, double startPosition, double endPosition)
    {
        QVector<QVector3D> result;
        result.push_back(pointAtPosition(path, startPosition));

        const int firstVertex = static_cast<int>(std::floor(startPosition)) + 1;
        const int lastVertex = static_cast<int>(std::floor(endPosition));

        for (int vertex = firstVertex; vertex <= lastVertex && vertex < path.size(); ++vertex)
        {
            if (distance3D(result.back(), path[vertex]) > kEpsilon)
            {
                result.push_back(path[vertex]);
            }
        }

        const QVector3D endPoint = pointAtPosition(path, endPosition);

        if (distance3D(result.back(), endPoint) > kEpsilon)
        {
            result.push_back(endPoint);
        }

        return result;
    }

    bool groupsConflict
    (
        DisjointSet& groups,
        const QVector<Marker>& markers,
        int leftMarker,
        int rightMarker
    )
    {
        const int leftRoot = groups.find(leftMarker);
        const int rightRoot = groups.find(rightMarker);

        for (int left = 0; left < markers.size(); ++left)
        {
            if (groups.find(left) != leftRoot)
            {
                continue;
            }

            for (int right = 0; right < markers.size(); ++right)
            {
                if (groups.find(right) == rightRoot
                    && markers[left].recordIndex == markers[right].recordIndex
                    && std::abs(markers[left].position - markers[right].position) > 1.0e-7)
                {
                    return true;
                }
            }
        }

        return false;
    }

    GraphData buildGraph
    (
        const QVector<RotaryPathTopologyRecord>& records,
        const QVector<int>& candidateIndices,
        const RotaryPathTopologyTolerance& tolerance
    )
    {
        GraphData graph;
        graph.markerIndicesByRecord.resize(records.size());

        for (int recordIndex : candidateIndices)
        {
            const QVector<QVector3D>& path = records[recordIndex].points;

            if (path.size() >= 2 && !records[recordIndex].semanticallyClosed)
            {
                addMarker(graph.markers, recordIndex, 0.0, path.front());
                addMarker(graph.markers, recordIndex, static_cast<double>(path.size() - 1), path.back());
            }
        }

        for (int leftLocal = 0; leftLocal < candidateIndices.size(); ++leftLocal)
        {
            const int leftIndex = candidateIndices[leftLocal];
            const QVector<QVector3D>& leftPath = records[leftIndex].points;

            for (int rightLocal = leftLocal + 1; rightLocal < candidateIndices.size(); ++rightLocal)
            {
                const int rightIndex = candidateIndices[rightLocal];
                const QVector<QVector3D>& rightPath = records[rightIndex].points;

                if (leftPath.size() < 2 || rightPath.size() < 2)
                {
                    continue;
                }

                if (!records[leftIndex].semanticallyClosed)
                {
                    for (const double endpointPosition : { 0.0, static_cast<double>(leftPath.size() - 1) })
                    {
                        double rightPosition = 0.0;
                        QVector3D projection;
                        const QVector3D endpoint = pointAtPosition(leftPath, endpointPosition);

                        if (distancePointToPath(endpoint, rightPath, &rightPosition, &projection) <= tolerance.nodeSnap)
                        {
                            addMarker(graph.markers, leftIndex, endpointPosition, endpoint);

                            if (!records[rightIndex].semanticallyClosed)
                            {
                                addMarker(graph.markers, rightIndex, rightPosition, projection);
                            }
                        }
                    }
                }

                if (!records[rightIndex].semanticallyClosed)
                {
                    for (const double endpointPosition : { 0.0, static_cast<double>(rightPath.size() - 1) })
                    {
                        double leftPosition = 0.0;
                        QVector3D projection;
                        const QVector3D endpoint = pointAtPosition(rightPath, endpointPosition);

                        if (distancePointToPath(endpoint, leftPath, &leftPosition, &projection) <= tolerance.nodeSnap)
                        {
                            addMarker(graph.markers, rightIndex, endpointPosition, endpoint);

                            if (!records[leftIndex].semanticallyClosed)
                            {
                                addMarker(graph.markers, leftIndex, leftPosition, projection);
                            }
                        }
                    }
                }

                if (records[leftIndex].semanticallyClosed || records[rightIndex].semanticallyClosed)
                {
                    continue;
                }

                for (int leftSegment = 0; leftSegment + 1 < leftPath.size(); ++leftSegment)
                {
                    for (int rightSegment = 0; rightSegment + 1 < rightPath.size(); ++rightSegment)
                    {
                        double leftParameter = 0.0;
                        double rightParameter = 0.0;
                        QVector3D leftPoint;
                        QVector3D rightPoint;
                        const double distance = segmentSegmentDistance
                        (
                            leftPath[leftSegment],
                            leftPath[leftSegment + 1],
                            rightPath[rightSegment],
                            rightPath[rightSegment + 1],
                            leftParameter,
                            rightParameter,
                            leftPoint,
                            rightPoint
                        );

                        if (distance <= tolerance.intersection)
                        {
                            addMarker(graph.markers, leftIndex, leftSegment + leftParameter, leftPoint);
                            addMarker(graph.markers, rightIndex, rightSegment + rightParameter, rightPoint);
                        }
                    }
                }
            }
        }

        // Normalize projections close to an endpoint before clustering. This avoids
        // creating a tiny false branch when several rounded-corner endpoints coexist.
        for (Marker& marker : graph.markers)
        {
            const QVector<QVector3D>& path = records[marker.recordIndex].points;

            if (path.size() < 2)
            {
                continue;
            }

            if (distance3D(marker.point, path.front()) <= tolerance.nodeSnap)
            {
                marker.position = 0.0;
                marker.point = path.front();
            }
            else if (distance3D(marker.point, path.back()) <= tolerance.nodeSnap)
            {
                marker.position = static_cast<double>(path.size() - 1);
                marker.point = path.back();
            }
        }

        QVector<Marker> uniqueMarkers;

        for (const Marker& marker : graph.markers)
        {
            addMarker(uniqueMarkers, marker.recordIndex, marker.position, marker.point);
        }

        graph.markers = std::move(uniqueMarkers);

        for (int markerIndex = 0; markerIndex < graph.markers.size(); ++markerIndex)
        {
            graph.markerIndicesByRecord[graph.markers[markerIndex].recordIndex].push_back(markerIndex);
        }

        DisjointSet groups(graph.markers.size());
        struct MergeCandidate { int left = -1; int right = -1; double distance = 0.0; };
        QVector<MergeCandidate> mergeCandidates;

        for (int left = 0; left < graph.markers.size(); ++left)
        {
            for (int right = left + 1; right < graph.markers.size(); ++right)
            {
                if (graph.markers[left].recordIndex == graph.markers[right].recordIndex)
                {
                    continue;
                }

                const double distance = distance3D(graph.markers[left].point, graph.markers[right].point);

                if (distance <= tolerance.nodeSnap)
                {
                    mergeCandidates.push_back({ left, right, distance });
                }
            }
        }

        std::sort(mergeCandidates.begin(), mergeCandidates.end(), [](const MergeCandidate& left, const MergeCandidate& right)
        {
            if (std::abs(left.distance - right.distance) > kEpsilon)
            {
                return left.distance < right.distance;
            }

            return left.left < right.left || (left.left == right.left && left.right < right.right);
        });

        for (const MergeCandidate& candidate : mergeCandidates)
        {
            if (!groupsConflict(groups, graph.markers, candidate.left, candidate.right))
            {
                groups.unite(candidate.left, candidate.right);
            }
        }

        QHash<int, int> nodeByRoot;
        QVector<QVector<int>> markersByNode;

        for (int markerIndex = 0; markerIndex < graph.markers.size(); ++markerIndex)
        {
            const int root = groups.find(markerIndex);

            if (!nodeByRoot.contains(root))
            {
                nodeByRoot.insert(root, graph.nodes.size());
                graph.nodes.push_back({});
                markersByNode.push_back({});
            }

            markersByNode[nodeByRoot.value(root)].push_back(markerIndex);
        }

        for (int nodeIndex = 0; nodeIndex < graph.nodes.size(); ++nodeIndex)
        {
            QVector3D average;

            for (int markerIndex : markersByNode[nodeIndex])
            {
                average += graph.markers[markerIndex].point;
            }

            graph.nodes[nodeIndex] = average / static_cast<float>(markersByNode[nodeIndex].size());
        }

        for (int recordIndex : candidateIndices)
        {
            if (records[recordIndex].semanticallyClosed)
            {
                continue;
            }

            QVector<int> markerIndices = graph.markerIndicesByRecord[recordIndex];
            std::sort(markerIndices.begin(), markerIndices.end(), [&graph](int left, int right)
            {
                return graph.markers[left].position < graph.markers[right].position;
            });

            for (int marker = 0; marker + 1 < markerIndices.size(); ++marker)
            {
                const Marker& start = graph.markers[markerIndices[marker]];
                const Marker& end = graph.markers[markerIndices[marker + 1]];
                QVector<QVector3D> points = pathBetween(records[recordIndex].points, start.position, end.position);
                const double length = pathLength(points);

                if (length <= tolerance.minimumEdgeLength)
                {
                    continue;
                }

                const int firstNode = nodeByRoot.value(groups.find(markerIndices[marker]));
                const int secondNode = nodeByRoot.value(groups.find(markerIndices[marker + 1]));

                if (firstNode == secondNode)
                {
                    continue;
                }

                graph.edges.push_back({ firstNode, secondNode, recordIndex, std::move(points), length });
            }
        }

        graph.incidentEdges.resize(graph.nodes.size());

        for (int edgeIndex = 0; edgeIndex < graph.edges.size(); ++edgeIndex)
        {
            graph.incidentEdges[graph.edges[edgeIndex].firstNode].push_back(edgeIndex);
            graph.incidentEdges[graph.edges[edgeIndex].secondNode].push_back(edgeIndex);
        }

        return graph;
    }

    void appendEdgePoints(QVector<QVector3D>& path, const GraphEdge& edge, int fromNode)
    {
        if (fromNode == edge.firstNode)
        {
            for (const QVector3D& point : edge.points)
            {
                if (path.isEmpty() || distance3D(path.back(), point) > kEpsilon)
                {
                    path.push_back(point);
                }
            }
        }
        else
        {
            for (int index = edge.points.size() - 1; index >= 0; --index)
            {
                if (path.isEmpty() || distance3D(path.back(), edge.points[index]) > kEpsilon)
                {
                    path.push_back(edge.points[index]);
                }
            }
        }
    }

    double projectedAreaYZ(const QVector<QVector3D>& path)
    {
        double area = 0.0;

        for (int index = 0; index < path.size(); ++index)
        {
            const QVector3D& start = path[index];
            const QVector3D& end = path[(index + 1) % path.size()];
            area += static_cast<double>(start.y()) * end.z() - static_cast<double>(end.y()) * start.z();
        }

        return std::abs(area) * 0.5;
    }

    LoopCandidate candidateFromEdgeSequence
    (
        const GraphData& graph,
        const QVector<int>& edgeSequence,
        int startNode,
        bool approximatelyClosed,
        double closureGap
    )
    {
        LoopCandidate candidate;
        candidate.approximatelyClosed = approximatelyClosed;
        candidate.closureGap = closureGap;
        int currentNode = startNode;
        QVector3D firstPhysicalPoint;
        QVector3D previousPhysicalPoint;
        bool hasPhysicalPoint = false;

        for (int edgeIndex : edgeSequence)
        {
            const GraphEdge& edge = graph.edges[edgeIndex];
            const QVector3D physicalStart = currentNode == edge.firstNode
                ? edge.points.front()
                : edge.points.back();
            const QVector3D physicalEnd = currentNode == edge.firstNode
                ? edge.points.back()
                : edge.points.front();

            if (!hasPhysicalPoint)
            {
                firstPhysicalPoint = physicalStart;
                hasPhysicalPoint = true;
            }
            else
            {
                candidate.closureGap = std::max
                (
                    candidate.closureGap,
                    distance3D(previousPhysicalPoint, physicalStart)
                );
            }

            appendEdgePoints(candidate.orderedPath, edge, currentNode);
            candidate.recordIndices.insert(edge.recordIndex);
            candidate.length += edge.length;
            previousPhysicalPoint = physicalEnd;
            currentNode = edge.firstNode == currentNode ? edge.secondNode : edge.firstNode;
        }

        if (hasPhysicalPoint)
        {
            candidate.closureGap = std::max
            (
                candidate.closureGap,
                distance3D(previousPhysicalPoint, firstPhysicalPoint)
            );
            candidate.approximatelyClosed = candidate.approximatelyClosed
                || candidate.closureGap > kEpsilon;
        }

        candidate.projectedArea = projectedAreaYZ(candidate.orderedPath);
        return candidate;
    }

    QVector<int> edgeComponent
    (
        const GraphData& graph,
        int startEdge,
        const QVector<bool>& allowedEdges,
        QVector<bool>& visited
    )
    {
        QVector<int> result{ startEdge };
        visited[startEdge] = true;

        for (int cursor = 0; cursor < result.size(); ++cursor)
        {
            const GraphEdge& edge = graph.edges[result[cursor]];

            for (int node : { edge.firstNode, edge.secondNode })
            {
                for (int neighbor : graph.incidentEdges[node])
                {
                    if (allowedEdges[neighbor] && !visited[neighbor])
                    {
                        visited[neighbor] = true;
                        result.push_back(neighbor);
                    }
                }
            }
        }

        return result;
    }

    QVector<int> orderedCycleEdges(const GraphData& graph, const QVector<int>& component, int& startNode)
    {
        QSet<int> componentSet(component.begin(), component.end());
        QHash<int, int> degree;

        for (int edgeIndex : component)
        {
            ++degree[graph.edges[edgeIndex].firstNode];
            ++degree[graph.edges[edgeIndex].secondNode];
        }

        for (auto iterator = degree.cbegin(); iterator != degree.cend(); ++iterator)
        {
            if (iterator.value() != 2)
            {
                return {};
            }
        }

        startNode = std::min_element(degree.cbegin(), degree.cend(), [](int left, int right) { return left < right; }).key();
        QVector<int> ordered;
        QSet<int> used;
        int currentNode = startNode;

        while (ordered.size() < component.size())
        {
            int nextEdge = -1;

            for (int edgeIndex : graph.incidentEdges[currentNode])
            {
                if (componentSet.contains(edgeIndex) && !used.contains(edgeIndex))
                {
                    nextEdge = edgeIndex;
                    break;
                }
            }

            if (nextEdge < 0)
            {
                return {};
            }

            ordered.push_back(nextEdge);
            used.insert(nextEdge);
            const GraphEdge& edge = graph.edges[nextEdge];
            currentNode = edge.firstNode == currentNode ? edge.secondNode : edge.firstNode;
        }

        return currentNode == startNode ? ordered : QVector<int>();
    }

    QVector<int> pathBetweenNodes
    (
        const GraphData& graph,
        const QVector<int>& component,
        int startNode,
        int endNode
    )
    {
        QSet<int> componentSet(component.begin(), component.end());
        QHash<int, int> previousNode;
        QHash<int, int> previousEdge;
        std::queue<int> pending;
        pending.push(startNode);
        previousNode.insert(startNode, -1);

        while (!pending.empty() && !previousNode.contains(endNode))
        {
            const int node = pending.front();
            pending.pop();

            for (int edgeIndex : graph.incidentEdges[node])
            {
                if (!componentSet.contains(edgeIndex))
                {
                    continue;
                }

                const GraphEdge& edge = graph.edges[edgeIndex];
                const int neighbor = edge.firstNode == node ? edge.secondNode : edge.firstNode;

                if (!previousNode.contains(neighbor))
                {
                    previousNode.insert(neighbor, node);
                    previousEdge.insert(neighbor, edgeIndex);
                    pending.push(neighbor);
                }
            }
        }

        if (!previousNode.contains(endNode))
        {
            return {};
        }

        QVector<int> reversed;

        for (int node = endNode; node != startNode; node = previousNode.value(node))
        {
            reversed.push_back(previousEdge.value(node));
        }

        std::reverse(reversed.begin(), reversed.end());
        return reversed;
    }

    QVector<QPair<int, QVector<int>>> enumerateSimpleCycles(const GraphData& graph, int maximumCycleCount)
    {
        QVector<QPair<int, QVector<int>>> cycles;
        QSet<QString> cycleKeys;

        for (int startNode = 0; startNode < graph.nodes.size() && cycles.size() < maximumCycleCount; ++startNode)
        {
            QSet<int> visitedNodes{ startNode };
            QSet<int> usedEdges;
            QVector<int> currentEdges;
            std::function<void(int)> visit = [&](int currentNode)
            {
                if (cycles.size() >= maximumCycleCount)
                {
                    return;
                }

                for (int edgeIndex : graph.incidentEdges[currentNode])
                {
                    if (usedEdges.contains(edgeIndex))
                    {
                        continue;
                    }

                    const GraphEdge& edge = graph.edges[edgeIndex];
                    const int neighbor = edge.firstNode == currentNode ? edge.secondNode : edge.firstNode;

                    if (neighbor == startNode)
                    {
                        if (!currentEdges.isEmpty())
                        {
                            QVector<int> cycleEdges = currentEdges;
                            cycleEdges.push_back(edgeIndex);
                            QVector<int> sortedEdges = cycleEdges;
                            std::sort(sortedEdges.begin(), sortedEdges.end());
                            QStringList keyParts;

                            for (int sortedEdge : sortedEdges)
                            {
                                keyParts.push_back(QString::number(sortedEdge));
                            }

                            const QString key = keyParts.join(QLatin1Char(','));

                            if (!cycleKeys.contains(key))
                            {
                                cycleKeys.insert(key);
                                cycles.push_back(qMakePair(startNode, cycleEdges));
                            }
                        }

                        continue;
                    }

                    if (visitedNodes.contains(neighbor) || neighbor < startNode)
                    {
                        continue;
                    }

                    visitedNodes.insert(neighbor);
                    usedEdges.insert(edgeIndex);
                    currentEdges.push_back(edgeIndex);
                    visit(neighbor);
                    currentEdges.pop_back();
                    usedEdges.remove(edgeIndex);
                    visitedNodes.remove(neighbor);
                }
            };
            visit(startNode);
        }

        return cycles;
    }
}

RotaryPathTopologyTolerance RotaryPathTopologyTolerance::fromConnectionTolerance(double connectionTolerance)
{
    const double nodeSnap = std::max(1.0e-9, connectionTolerance);
    return
    {
        nodeSnap,
        nodeSnap,
        std::max(1.0e-9, nodeSnap * 0.01),
        1.0e-6
    };
}

RotaryPathTopology::RotaryPathTopology
(
    const QVector<CadItem*>& items,
    const RotaryPathTopologyTolerance& tolerance
)
    : m_tolerance(tolerance)
{
    m_records.reserve(items.size());

    for (int itemIndex = 0; itemIndex < items.size(); ++itemIndex)
    {
        RotaryPathTopologyRecord record;
        record.sourceItem = items[itemIndex];
        record.sourceItemIndex = itemIndex;
        record.points = cleanPath(items[itemIndex], tolerance.minimumEdgeLength);
        record.semanticallyClosed = nativeEntityIsClosed(items[itemIndex]);

        if (record.semanticallyClosed
            && record.points.size() >= 3
            && distance3D(record.points.front(), record.points.back()) > tolerance.minimumEdgeLength)
        {
            record.points.push_back(record.points.front());
        }

        m_records.push_back(std::move(record));
    }

    m_itemAdjacency.resize(m_records.size());

    for (int left = 0; left < m_records.size(); ++left)
    {
        for (int right = left + 1; right < m_records.size(); ++right)
        {
            if (pathsConnected(m_records[left], m_records[right], tolerance))
            {
                m_itemAdjacency[left].push_back(right);
                m_itemAdjacency[right].push_back(left);
            }
        }
    }
}

const QVector<RotaryPathTopologyRecord>& RotaryPathTopology::records() const
{
    return m_records;
}

std::vector<int> RotaryPathTopology::itemComponentIds(const QVector<CadItem*>& subset) const
{
    QVector<int> indices;

    if (subset.isEmpty())
    {
        indices.reserve(m_records.size());

        for (int index = 0; index < m_records.size(); ++index)
        {
            indices.push_back(index);
        }
    }
    else
    {
        indices.reserve(subset.size());

        for (CadItem* item : subset)
        {
            for (int index = 0; index < m_records.size(); ++index)
            {
                if (m_records[index].sourceItem == item)
                {
                    indices.push_back(index);
                    break;
                }
            }
        }
    }

    QHash<int, int> localByGlobal;

    for (int local = 0; local < indices.size(); ++local)
    {
        localByGlobal.insert(indices[local], local);
    }

    std::vector<int> componentIds(static_cast<size_t>(indices.size()), -1);
    int nextComponent = 0;

    for (int start = 0; start < indices.size(); ++start)
    {
        if (componentIds[static_cast<size_t>(start)] >= 0)
        {
            continue;
        }

        QVector<int> pending{ start };
        componentIds[static_cast<size_t>(start)] = nextComponent;

        for (int cursor = 0; cursor < pending.size(); ++cursor)
        {
            const int local = pending[cursor];
            const int global = indices[local];

            for (int neighborGlobal : m_itemAdjacency[global])
            {
                const int neighborLocal = localByGlobal.value(neighborGlobal, -1);

                if (neighborLocal >= 0 && componentIds[static_cast<size_t>(neighborLocal)] < 0)
                {
                    componentIds[static_cast<size_t>(neighborLocal)] = nextComponent;
                    pending.push_back(neighborLocal);
                }
            }
        }

        ++nextComponent;
    }

    return componentIds;
}

bool RotaryPathTopology::itemsDirectlyConnected(CadItem* left, CadItem* right) const
{
    int leftIndex = -1;
    int rightIndex = -1;

    for (int index = 0; index < m_records.size(); ++index)
    {
        leftIndex = m_records[index].sourceItem == left ? index : leftIndex;
        rightIndex = m_records[index].sourceItem == right ? index : rightIndex;
    }

    return leftIndex >= 0 && rightIndex >= 0 && m_itemAdjacency[leftIndex].contains(rightIndex);
}

RotaryPathLoopResult RotaryPathTopology::extractBestLoop
(
    const QVector<CadItem*>& candidateItems,
    const QVector<CadItem*>& preferredItems
) const
{
    RotaryPathLoopResult result;
    QVector<int> candidateIndices;
    QSet<CadItem*> preferredSet(preferredItems.begin(), preferredItems.end());

    for (int recordIndex = 0; recordIndex < m_records.size(); ++recordIndex)
    {
        if (candidateItems.isEmpty() || candidateItems.contains(m_records[recordIndex].sourceItem))
        {
            candidateIndices.push_back(recordIndex);
        }
    }

    if (candidateIndices.isEmpty())
    {
        result.errorMessage = QStringLiteral("加工断面候选图元为空。");
        return result;
    }

    QVector<CadItem*> candidateSources;

    for (int index : candidateIndices)
    {
        candidateSources.push_back(m_records[index].sourceItem);
    }

    const std::vector<int> componentIds = itemComponentIds(candidateSources);
    result.connectedComponentCount = componentIds.empty()
        ? 0
        : (*std::max_element(componentIds.begin(), componentIds.end()) + 1);

    if (result.connectedComponentCount > 1 && !preferredItems.isEmpty())
    {
        QSet<int> preferredComponents;

        for (int localIndex = 0; localIndex < candidateSources.size(); ++localIndex)
        {
            if (preferredSet.contains(candidateSources[localIndex]))
            {
                preferredComponents.insert(componentIds[static_cast<size_t>(localIndex)]);
            }
        }

        if (preferredComponents.size() > 1)
        {
            result.errorMessage = QStringLiteral("选择集中包含 %1 个互不相连的轮廓，请只选择一个加工断面。")
                .arg(preferredComponents.size());
            return result;
        }
    }

    QVector<LoopCandidate> candidates;

    for (int recordIndex : candidateIndices)
    {
        const RotaryPathTopologyRecord& record = m_records[recordIndex];

        if (!record.semanticallyClosed || record.points.size() < 3)
        {
            continue;
        }

        LoopCandidate candidate;
        candidate.orderedPath = record.points;
        candidate.recordIndices.insert(recordIndex);
        candidate.length = pathLength(record.points);
        candidate.projectedArea = projectedAreaYZ(record.points);
        candidates.push_back(std::move(candidate));
    }

    const GraphData graph = buildGraph(m_records, candidateIndices, m_tolerance);
    QVector<int> degree(graph.nodes.size(), 0);

    for (const GraphEdge& edge : graph.edges)
    {
        ++degree[edge.firstNode];
        ++degree[edge.secondNode];
    }

    for (int value : degree)
    {
        result.openNodeCount += value == 1 ? 1 : 0;
        result.branchNodeCount += value > 2 ? 1 : 0;
    }

    QVector<bool> active(graph.edges.size(), true);
    QVector<int> activeDegree = degree;
    std::queue<int> leaves;

    for (int node = 0; node < activeDegree.size(); ++node)
    {
        if (activeDegree[node] <= 1)
        {
            leaves.push(node);
        }
    }

    while (!leaves.empty())
    {
        const int node = leaves.front();
        leaves.pop();

        for (int edgeIndex : graph.incidentEdges[node])
        {
            if (!active[edgeIndex])
            {
                continue;
            }

            active[edgeIndex] = false;
            const GraphEdge& edge = graph.edges[edgeIndex];
            const int neighbor = edge.firstNode == node ? edge.secondNode : edge.firstNode;
            --activeDegree[node];
            --activeDegree[neighbor];

            if (activeDegree[neighbor] == 1)
            {
                leaves.push(neighbor);
            }
        }
    }

    QVector<bool> visitedCore(graph.edges.size(), false);
    bool foundCoreCycle = false;

    for (int edgeIndex = 0; edgeIndex < graph.edges.size(); ++edgeIndex)
    {
        if (!active[edgeIndex] || visitedCore[edgeIndex])
        {
            continue;
        }

        const QVector<int> component = edgeComponent(graph, edgeIndex, active, visitedCore);
        int startNode = -1;
        const QVector<int> orderedEdges = orderedCycleEdges(graph, component, startNode);

        if (!orderedEdges.isEmpty())
        {
            candidates.push_back(candidateFromEdgeSequence(graph, orderedEdges, startNode, false, 0.0));
            foundCoreCycle = true;
        }
    }

    if (result.branchNodeCount > 0 && !foundCoreCycle)
    {
        const QVector<QPair<int, QVector<int>>> cycles = enumerateSimpleCycles(graph, 512);

        for (const auto& cycle : cycles)
        {
            candidates.push_back(candidateFromEdgeSequence(graph, cycle.second, cycle.first, false, 0.0));
        }
    }

    QVector<bool> allEdges(graph.edges.size(), true);
    QVector<bool> visitedAll(graph.edges.size(), false);

    for (int edgeIndex = 0; edgeIndex < graph.edges.size(); ++edgeIndex)
    {
        if (visitedAll[edgeIndex])
        {
            continue;
        }

        const QVector<int> component = edgeComponent(graph, edgeIndex, allEdges, visitedAll);
        QSet<int> componentNodes;

        for (int componentEdge : component)
        {
            componentNodes.insert(graph.edges[componentEdge].firstNode);
            componentNodes.insert(graph.edges[componentEdge].secondNode);
        }

        QVector<int> openNodes;

        for (int node : componentNodes)
        {
            int componentDegree = 0;

            for (int incident : graph.incidentEdges[node])
            {
                componentDegree += component.contains(incident) ? 1 : 0;
            }

            if (componentDegree == 1)
            {
                openNodes.push_back(node);
            }
        }

        for (int left = 0; left < openNodes.size(); ++left)
        {
            for (int right = left + 1; right < openNodes.size(); ++right)
            {
                const double gap = distance3D(graph.nodes[openNodes[left]], graph.nodes[openNodes[right]]);

                if (gap > m_tolerance.closure)
                {
                    continue;
                }

                const QVector<int> pathEdges = pathBetweenNodes
                (
                    graph,
                    component,
                    openNodes[left],
                    openNodes[right]
                );
                const LoopCandidate candidate = candidateFromEdgeSequence
                (
                    graph,
                    pathEdges,
                    openNodes[left],
                    true,
                    gap
                );

                if (!pathEdges.isEmpty()
                    && candidate.orderedPath.size() >= 3
                    && candidate.length > std::max(4.0 * m_tolerance.closure, 4.0 * gap))
                {
                    candidates.push_back(candidate);
                }
            }
        }
    }

    if (candidates.isEmpty())
    {
        double maximumOpenGap = 0.0;

        for (int left = 0; left < graph.nodes.size(); ++left)
        {
            if (degree[left] != 1)
            {
                continue;
            }

            for (int right = left + 1; right < graph.nodes.size(); ++right)
            {
                if (degree[right] == 1)
                {
                    maximumOpenGap = std::max(maximumOpenGap, distance3D(graph.nodes[left], graph.nodes[right]));
                }
            }
        }

        if (result.openNodeCount == 2 && maximumOpenGap > m_tolerance.closure)
        {
            result.errorMessage = QStringLiteral("加工断面闭合缺口超限：连通分量 %1，开放节点 %2，缺口 %3 mm，允许 %4 mm。")
                .arg(result.connectedComponentCount)
                .arg(result.openNodeCount)
                .arg(maximumOpenGap, 0, 'f', 3)
                .arg(m_tolerance.closure, 0, 'f', 3);
        }
        else if (result.branchNodeCount > 0)
        {
            result.errorMessage = QStringLiteral("加工断面存在无法消除的分叉：连通分量 %1，开放节点 %2，分叉节点 %3，最大开口 %4 mm。")
                .arg(result.connectedComponentCount)
                .arg(result.openNodeCount)
                .arg(result.branchNodeCount)
                .arg(maximumOpenGap, 0, 'f', 3);
        }
        else
        {
            result.errorMessage = QStringLiteral("加工断面路径不连续：连通分量 %1，开放节点 %2，最大开口 %3 mm。")
                .arg(result.connectedComponentCount)
                .arg(result.openNodeCount)
                .arg(maximumOpenGap, 0, 'f', 3);
        }

        return result;
    }

    auto preferredCount = [&preferredSet, this](const LoopCandidate& candidate)
    {
        int count = 0;

        for (int recordIndex : candidate.recordIndices)
        {
            count += preferredSet.contains(m_records[recordIndex].sourceItem) ? 1 : 0;
        }

        return count;
    };

    std::sort(candidates.begin(), candidates.end(), [&preferredCount](const LoopCandidate& left, const LoopCandidate& right)
    {
        const int leftPreferred = preferredCount(left);
        const int rightPreferred = preferredCount(right);

        if (leftPreferred != rightPreferred)
        {
            return leftPreferred > rightPreferred;
        }

        if (std::abs(left.projectedArea - right.projectedArea) > kEpsilon)
        {
            return left.projectedArea > right.projectedArea;
        }

        if (left.recordIndices.size() != right.recordIndices.size())
        {
            return left.recordIndices.size() > right.recordIndices.size();
        }

        return left.length > right.length;
    });

    if (candidates.size() > 1)
    {
        const LoopCandidate& first = candidates[0];
        const LoopCandidate& second = candidates[1];

        if (preferredCount(first) == preferredCount(second)
            && std::abs(first.projectedArea - second.projectedArea) <= m_tolerance.intersection
            && first.recordIndices.size() == second.recordIndices.size())
        {
            result.errorMessage = QStringLiteral("候选图元中存在多个同等优先级的闭环，请只选择一个加工断面中的图元。");
            return result;
        }
    }

    const LoopCandidate& best = candidates.front();
    result.valid = true;
    result.connectedLoop = true;
    result.approximatelyClosed = best.approximatelyClosed;
    result.closureGap = best.closureGap;
    result.orderedPath = best.orderedPath;

    for (int recordIndex : best.recordIndices)
    {
        result.usedItems.push_back(m_records[recordIndex].sourceItem);
    }

    for (int recordIndex : candidateIndices)
    {
        if (!best.recordIndices.contains(recordIndex))
        {
            result.ignoredBranchItems.push_back(m_records[recordIndex].sourceItem);
        }
    }

    result.ignoredBranchItemCount = result.ignoredBranchItems.size();
    return result;
}
