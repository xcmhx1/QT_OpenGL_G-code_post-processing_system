#include "core/topology/PathTopology.h"

#include <QVariantList>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <utility>

namespace cadcam::topology
{
namespace
{
    constexpr double kEpsilon = 1.0e-9;
    constexpr double kMaximumNumericalJoinEpsilon = 1.0e-4;
    using geometry::EntityId;
    using geometry::Vector3d;

    Vector3d add(const Vector3d& left, const Vector3d& right)
    {
        return { left.x + right.x, left.y + right.y, left.z + right.z };
    }

    Vector3d subtract(const Vector3d& left, const Vector3d& right)
    {
        return { left.x - right.x, left.y - right.y, left.z - right.z };
    }

    Vector3d multiply(const Vector3d& point, double factor)
    {
        return { point.x * factor, point.y * factor, point.z * factor };
    }

    double dot(const Vector3d& left, const Vector3d& right)
    {
        return left.x * right.x + left.y * right.y + left.z * right.z;
    }

    double distance3D(const Vector3d& left, const Vector3d& right)
    {
        const Vector3d difference = subtract(left, right);
        return std::sqrt(dot(difference, difference));
    }

    double pathLength(const std::vector<Vector3d>& path)
    {
        double length = 0.0;
        for (std::size_t index = 1; index < path.size(); ++index)
        {
            length += distance3D(path[index - 1U], path[index]);
        }
        return length;
    }

    bool contains(const std::vector<int>& values, int value)
    {
        return std::find(values.cbegin(), values.cend(), value) != values.cend();
    }

    struct Marker
    {
        int recordIndex = -1;
        double position = 0.0;
        Vector3d point;
    };

    struct GraphEdge
    {
        int firstNode = -1;
        int secondNode = -1;
        int recordIndex = -1;
        std::vector<Vector3d> points;
        double length = 0.0;
    };

    struct TopologyPlaneFit
    {
        bool valid = false;
        double a = 0.0;
        double b = 0.0;
        double c = 0.0;
        double maximumDeviation = 0.0;
    };

    struct GraphData
    {
        std::vector<Marker> markers;
        std::vector<std::vector<int>> markerIndicesByRecord;
        std::vector<Vector3d> nodes;
        std::vector<GraphEdge> edges;
        std::vector<std::vector<int>> incidentEdges;
        TopologyPlaneFit planeFit;
    };

    struct LoopCandidate
    {
        std::vector<Vector3d> orderedPath;
        std::set<int> recordIndices;
        double maximumJoinGap = 0.0;
        double length = 0.0;
        double projectedArea = 0.0;
        std::size_t minimumSourceIndex = std::numeric_limits<std::size_t>::max();
        EntityId minimumEntityId = std::numeric_limits<EntityId>::max();
    };

    class DisjointSet
    {
    public:
        explicit DisjointSet(int count)
            : m_parent(static_cast<std::size_t>(count)),
              m_rank(static_cast<std::size_t>(count), 0)
        {
            for (int index = 0; index < count; ++index)
            {
                m_parent[static_cast<std::size_t>(index)] = index;
            }
        }

        int find(int index)
        {
            int& parent = m_parent[static_cast<std::size_t>(index)];
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
            int& leftRank = m_rank[static_cast<std::size_t>(left)];
            int& rightRank = m_rank[static_cast<std::size_t>(right)];
            if (leftRank < rightRank)
            {
                std::swap(left, right);
            }
            m_parent[static_cast<std::size_t>(right)] = left;
            if (leftRank == rightRank)
            {
                ++leftRank;
            }
        }

    private:
        std::vector<int> m_parent;
        std::vector<int> m_rank;
    };

    Diagnostic topologyDiagnostic
    (
        const OperationContext& context,
        DiagnosticCode code,
        DiagnosticSeverity severity,
        const QString& detail,
        const PathTopologyTolerance& tolerance,
        std::size_t recordCount,
        std::size_t nodeCount = 0U,
        std::size_t edgeCount = 0U,
        int componentCount = 0,
        int openNodeCount = 0,
        int branchNodeCount = 0,
        double maximumJoinGap = 0.0
    )
    {
        Diagnostic diagnostic;
        diagnostic.code = code;
        diagnostic.severity = severity;
        diagnostic.component = QStringLiteral("PathTopologyBuilder");
        diagnostic.operation = QStringLiteral("BuildPathTopology");
        diagnostic.stage = QStringLiteral("TopologyGraph");
        diagnostic.userMessage = code == DiagnosticCode::TopologyLoopDiscontinuous
            ? QStringLiteral("候选路径存在未连接间隙，不能作为闭合断面。")
            : (code == DiagnosticCode::TopologyLoopNotFound
                ? QStringLiteral("未找到符合条件的闭环。")
                : QStringLiteral("拓扑构建或提取未完成。"));
        diagnostic.technicalDetail = detail;
        diagnostic.correlationId = context.correlationId;
        diagnostic.context.insert(QStringLiteral("entityId"), static_cast<qulonglong>(0U));
        diagnostic.context.insert(QStringLiteral("sourceIndex"), static_cast<qulonglong>(0U));
        diagnostic.context.insert(QStringLiteral("recordCount"), static_cast<qulonglong>(recordCount));
        diagnostic.context.insert(QStringLiteral("nodeCount"), static_cast<qulonglong>(nodeCount));
        diagnostic.context.insert(QStringLiteral("edgeCount"), static_cast<qulonglong>(edgeCount));
        diagnostic.context.insert(QStringLiteral("componentCount"), componentCount);
        diagnostic.context.insert(QStringLiteral("openNodeCount"), openNodeCount);
        diagnostic.context.insert(QStringLiteral("branchNodeCount"), branchNodeCount);
        diagnostic.context.insert(QStringLiteral("maximumJoinGap"), maximumJoinGap);
        diagnostic.context.insert
            (QStringLiteral("numericalJoinEpsilon"), tolerance.numericalJoinEpsilon);
        diagnostic.context.insert(QStringLiteral("connectionTolerance"), tolerance.nodeSnap);
        diagnostic.context.insert(QStringLiteral("nodeSnap"), tolerance.nodeSnap);
        diagnostic.context.insert(QStringLiteral("intersectionTolerance"), tolerance.intersection);
        return diagnostic;
    }

    TopologyPlaneFit fitTopologyPlane
    (
        const std::vector<TopologyPathRecord>& records,
        const std::vector<int>& candidateIndices,
        double planeTolerance
    )
    {
        TopologyPlaneFit fit;
        double count = 0.0;
        double meanX = 0.0;
        double meanY = 0.0;
        double meanZ = 0.0;
        for (int recordIndex : candidateIndices)
        {
            for (const Vector3d& point : records[static_cast<std::size_t>(recordIndex)].points)
            {
                count += 1.0;
                meanX += point.x;
                meanY += point.y;
                meanZ += point.z;
            }
        }
        if (count < 3.0)
        {
            return fit;
        }
        meanX /= count;
        meanY /= count;
        meanZ /= count;
        double yy = 0.0;
        double yz = 0.0;
        double zz = 0.0;
        double xy = 0.0;
        double xz = 0.0;
        for (int recordIndex : candidateIndices)
        {
            for (const Vector3d& point : records[static_cast<std::size_t>(recordIndex)].points)
            {
                const double dx = point.x - meanX;
                const double dy = point.y - meanY;
                const double dz = point.z - meanZ;
                yy += dy * dy;
                yz += dy * dz;
                zz += dz * dz;
                xy += dx * dy;
                xz += dx * dz;
            }
        }
        const double determinant = yy * zz - yz * yz;
        if (std::abs(determinant) <= kEpsilon)
        {
            return fit;
        }
        fit.a = (xy * zz - xz * yz) / determinant;
        fit.b = (xz * yy - xy * yz) / determinant;
        fit.c = meanX - fit.a * meanY - fit.b * meanZ;
        const double normalLength = std::sqrt(1.0 + fit.a * fit.a + fit.b * fit.b);
        for (int recordIndex : candidateIndices)
        {
            for (const Vector3d& point : records[static_cast<std::size_t>(recordIndex)].points)
            {
                const double deviation = std::abs
                    (point.x - fit.a * point.y - fit.b * point.z - fit.c) / normalLength;
                fit.maximumDeviation = std::max(fit.maximumDeviation, deviation);
            }
        }
        fit.valid = fit.maximumDeviation <= planeTolerance;
        return fit;
    }

    Vector3d projectToTopologyPlane(const Vector3d& point, const TopologyPlaneFit& fit)
    {
        if (!fit.valid)
        {
            return point;
        }
        const Vector3d normal{ 1.0, -fit.a, -fit.b };
        const double factor =
            (point.x - fit.a * point.y - fit.b * point.z - fit.c) / dot(normal, normal);
        return subtract(point, multiply(normal, factor));
    }

    double pointSegmentDistance
    (
        const Vector3d& point,
        const Vector3d& start,
        const Vector3d& end,
        double* parameter = nullptr,
        Vector3d* projection = nullptr
    )
    {
        const Vector3d edge = subtract(end, start);
        const double lengthSquared = dot(edge, edge);
        const double factor = lengthSquared <= kEpsilon
            ? 0.0
            : std::clamp(dot(subtract(point, start), edge) / lengthSquared, 0.0, 1.0);
        const Vector3d projected = add(start, multiply(edge, factor));
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
        const Vector3d& point,
        const std::vector<Vector3d>& path,
        double* pathPosition = nullptr,
        Vector3d* projection = nullptr
    )
    {
        double bestDistance = std::numeric_limits<double>::max();
        for (std::size_t segment = 0; segment + 1U < path.size(); ++segment)
        {
            double parameter = 0.0;
            Vector3d candidateProjection;
            const double distance = pointSegmentDistance
                (point, path[segment], path[segment + 1U], &parameter, &candidateProjection);
            if (distance < bestDistance)
            {
                bestDistance = distance;
                if (pathPosition != nullptr)
                {
                    *pathPosition = static_cast<double>(segment) + parameter;
                }
                if (projection != nullptr)
                {
                    *projection = candidateProjection;
                }
            }
        }
        return bestDistance;
    }

    double distancePointToPathOnTopologyPlane
    (
        const Vector3d& point,
        const std::vector<Vector3d>& path,
        const TopologyPlaneFit& planeFit,
        double* pathPosition = nullptr,
        Vector3d* projection = nullptr
    )
    {
        if (!planeFit.valid)
        {
            return distancePointToPath(point, path, pathPosition, projection);
        }
        const Vector3d projectedPoint = projectToTopologyPlane(point, planeFit);
        double bestDistance = std::numeric_limits<double>::max();
        for (std::size_t segment = 0; segment + 1U < path.size(); ++segment)
        {
            double parameter = 0.0;
            Vector3d candidateProjection;
            const double distance = pointSegmentDistance
            (
                projectedPoint,
                projectToTopologyPlane(path[segment], planeFit),
                projectToTopologyPlane(path[segment + 1U], planeFit),
                &parameter,
                &candidateProjection
            );
            if (distance < bestDistance)
            {
                bestDistance = distance;
                if (pathPosition != nullptr)
                {
                    *pathPosition = static_cast<double>(segment) + parameter;
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
        const Vector3d& firstStart,
        const Vector3d& firstEnd,
        const Vector3d& secondStart,
        const Vector3d& secondEnd,
        double& firstParameter,
        double& secondParameter,
        Vector3d& firstPoint,
        Vector3d& secondPoint
    )
    {
        const Vector3d firstDirection = subtract(firstEnd, firstStart);
        const Vector3d secondDirection = subtract(secondEnd, secondStart);
        const Vector3d offset = subtract(firstStart, secondStart);
        const double a = dot(firstDirection, firstDirection);
        const double e = dot(secondDirection, secondDirection);
        const double f = dot(secondDirection, offset);
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
            const double c = dot(firstDirection, offset);
            if (e <= kEpsilon)
            {
                secondParameter = 0.0;
                firstParameter = std::clamp(-c / a, 0.0, 1.0);
            }
            else
            {
                const double b = dot(firstDirection, secondDirection);
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
        firstPoint = add(firstStart, multiply(firstDirection, firstParameter));
        secondPoint = add(secondStart, multiply(secondDirection, secondParameter));
        return distance3D(firstPoint, secondPoint);
    }

    bool pathsConnected
    (
        const TopologyPathRecord& left,
        const TopologyPathRecord& right,
        const PathTopologyTolerance& tolerance
    )
    {
        if (left.points.size() < 2U || right.points.size() < 2U)
        {
            return false;
        }
        for (const Vector3d* endpoint : { &left.points.front(), &left.points.back() })
        {
            if (distancePointToPath(*endpoint, right.points) <= tolerance.nodeSnap)
            {
                return true;
            }
        }
        for (const Vector3d* endpoint : { &right.points.front(), &right.points.back() })
        {
            if (distancePointToPath(*endpoint, left.points) <= tolerance.nodeSnap)
            {
                return true;
            }
        }
        for (std::size_t leftSegment = 0; leftSegment + 1U < left.points.size(); ++leftSegment)
        {
            for (std::size_t rightSegment = 0; rightSegment + 1U < right.points.size(); ++rightSegment)
            {
                double leftParameter = 0.0;
                double rightParameter = 0.0;
                Vector3d leftPoint;
                Vector3d rightPoint;
                if (segmentSegmentDistance
                (
                    left.points[leftSegment], left.points[leftSegment + 1U],
                    right.points[rightSegment], right.points[rightSegment + 1U],
                    leftParameter, rightParameter, leftPoint, rightPoint
                ) <= tolerance.intersection)
                {
                    return true;
                }
            }
        }
        return false;
    }

    void addMarker
    (
        std::vector<Marker>& markers,
        int recordIndex,
        double position,
        const Vector3d& point
    )
    {
        for (const Marker& marker : markers)
        {
            if (marker.recordIndex == recordIndex
                && std::abs(marker.position - position) <= 1.0e-7)
            {
                return;
            }
        }
        markers.push_back({ recordIndex, position, point });
    }

    Vector3d pointAtPosition(const std::vector<Vector3d>& path, double position)
    {
        const int segmentIndex = std::clamp
        (
            static_cast<int>(std::floor(position)),
            0,
            static_cast<int>(path.size()) - 2
        );
        const double parameter = std::clamp(position - segmentIndex, 0.0, 1.0);
        return add
        (
            path[static_cast<std::size_t>(segmentIndex)],
            multiply
            (
                subtract
                (
                    path[static_cast<std::size_t>(segmentIndex + 1)],
                    path[static_cast<std::size_t>(segmentIndex)]
                ),
                parameter
            )
        );
    }

    std::vector<Vector3d> pathBetween
    (
        const std::vector<Vector3d>& path,
        double startPosition,
        double endPosition
    )
    {
        std::vector<Vector3d> result{ pointAtPosition(path, startPosition) };
        const int firstVertex = static_cast<int>(std::floor(startPosition)) + 1;
        const int lastVertex = static_cast<int>(std::floor(endPosition));
        for (int vertex = firstVertex;
            vertex <= lastVertex && vertex < static_cast<int>(path.size()); ++vertex)
        {
            if (distance3D(result.back(), path[static_cast<std::size_t>(vertex)]) > kEpsilon)
            {
                result.push_back(path[static_cast<std::size_t>(vertex)]);
            }
        }
        const Vector3d endPoint = pointAtPosition(path, endPosition);
        if (distance3D(result.back(), endPoint) > kEpsilon)
        {
            result.push_back(endPoint);
        }
        return result;
    }

    std::vector<Vector3d> cyclicPathBetween
    (
        const std::vector<Vector3d>& path,
        double startPosition,
        double endPosition
    )
    {
        if (endPosition > startPosition)
        {
            return pathBetween(path, startPosition, endPosition);
        }
        std::vector<Vector3d> result = pathBetween
            (path, startPosition, static_cast<double>(path.size() - 1U));
        const std::vector<Vector3d> head = pathBetween(path, 0.0, endPosition);
        for (const Vector3d& point : head)
        {
            if (result.empty() || distance3D(result.back(), point) > kEpsilon)
            {
                result.push_back(point);
            }
        }
        return result;
    }

    bool groupsConflict
    (
        DisjointSet& groups,
        const std::vector<Marker>& markers,
        int leftMarker,
        int rightMarker
    )
    {
        const int leftRoot = groups.find(leftMarker);
        const int rightRoot = groups.find(rightMarker);
        for (int left = 0; left < static_cast<int>(markers.size()); ++left)
        {
            if (groups.find(left) != leftRoot)
            {
                continue;
            }
            for (int right = 0; right < static_cast<int>(markers.size()); ++right)
            {
                if (groups.find(right) == rightRoot
                    && markers[static_cast<std::size_t>(left)].recordIndex
                        == markers[static_cast<std::size_t>(right)].recordIndex
                    && std::abs(markers[static_cast<std::size_t>(left)].position
                        - markers[static_cast<std::size_t>(right)].position) > 1.0e-7)
                {
                    return true;
                }
            }
        }
        return false;
    }

    bool buildGraph
    (
        const std::vector<TopologyPathRecord>& records,
        const std::vector<int>& candidateIndices,
        const PathTopologyTolerance& tolerance,
        const CancellationToken& cancellationToken,
        GraphData& graph
    )
    {
        graph.planeFit = fitTopologyPlane
        (
            records,
            candidateIndices,
            std::max(0.05, tolerance.nodeSnap * 0.25)
        );
        const auto topologyPoint = [&graph](const Vector3d& point)
        {
            return projectToTopologyPlane(point, graph.planeFit);
        };
        graph.markerIndicesByRecord.resize(records.size());

        for (int recordIndex : candidateIndices)
        {
            const TopologyPathRecord& record = records[static_cast<std::size_t>(recordIndex)];
            if (record.points.size() >= 2U && !record.semanticallyClosed)
            {
                addMarker(graph.markers, recordIndex, 0.0, topologyPoint(record.points.front()));
                addMarker
                (
                    graph.markers,
                    recordIndex,
                    static_cast<double>(record.points.size() - 1U),
                    topologyPoint(record.points.back())
                );
            }
        }

        for (std::size_t leftLocal = 0; leftLocal < candidateIndices.size(); ++leftLocal)
        {
            if (cancellationToken.isCancellationRequested())
            {
                return false;
            }
            const int leftIndex = candidateIndices[leftLocal];
            const TopologyPathRecord& leftRecord = records[static_cast<std::size_t>(leftIndex)];
            const std::vector<Vector3d>& leftPath = leftRecord.points;
            for (std::size_t rightLocal = leftLocal + 1U;
                rightLocal < candidateIndices.size(); ++rightLocal)
            {
                const int rightIndex = candidateIndices[rightLocal];
                const TopologyPathRecord& rightRecord = records[static_cast<std::size_t>(rightIndex)];
                const std::vector<Vector3d>& rightPath = rightRecord.points;
                if (leftPath.size() < 2U || rightPath.size() < 2U)
                {
                    continue;
                }

                if (!leftRecord.semanticallyClosed)
                {
                    for (double endpointPosition :
                        { 0.0, static_cast<double>(leftPath.size() - 1U) })
                    {
                        double rightPosition = 0.0;
                        Vector3d projection;
                        const Vector3d endpoint = pointAtPosition(leftPath, endpointPosition);
                        if (distancePointToPathOnTopologyPlane
                        (
                            endpoint, rightPath, graph.planeFit,
                            &rightPosition, &projection
                        ) <= tolerance.nodeSnap)
                        {
                            addMarker
                                (graph.markers, leftIndex, endpointPosition, topologyPoint(endpoint));
                            addMarker(graph.markers, rightIndex, rightPosition, projection);
                        }
                    }
                }

                if (!rightRecord.semanticallyClosed)
                {
                    for (double endpointPosition :
                        { 0.0, static_cast<double>(rightPath.size() - 1U) })
                    {
                        double leftPosition = 0.0;
                        Vector3d projection;
                        const Vector3d endpoint = pointAtPosition(rightPath, endpointPosition);
                        if (distancePointToPathOnTopologyPlane
                        (
                            endpoint, leftPath, graph.planeFit,
                            &leftPosition, &projection
                        ) <= tolerance.nodeSnap)
                        {
                            addMarker
                                (graph.markers, rightIndex, endpointPosition, topologyPoint(endpoint));
                            addMarker(graph.markers, leftIndex, leftPosition, projection);
                        }
                    }
                }

                for (std::size_t leftSegment = 0; leftSegment + 1U < leftPath.size(); ++leftSegment)
                {
                    for (std::size_t rightSegment = 0;
                        rightSegment + 1U < rightPath.size(); ++rightSegment)
                    {
                        double leftParameter = 0.0;
                        double rightParameter = 0.0;
                        Vector3d leftPoint;
                        Vector3d rightPoint;
                        const double distance = segmentSegmentDistance
                        (
                            topologyPoint(leftPath[leftSegment]),
                            topologyPoint(leftPath[leftSegment + 1U]),
                            topologyPoint(rightPath[rightSegment]),
                            topologyPoint(rightPath[rightSegment + 1U]),
                            leftParameter, rightParameter, leftPoint, rightPoint
                        );
                        if (distance <= tolerance.intersection)
                        {
                            addMarker
                            (
                                graph.markers, leftIndex,
                                static_cast<double>(leftSegment) + leftParameter,
                                leftPoint
                            );
                            addMarker
                            (
                                graph.markers, rightIndex,
                                static_cast<double>(rightSegment) + rightParameter,
                                rightPoint
                            );
                        }
                    }
                }
            }
        }

        for (Marker& marker : graph.markers)
        {
            const std::vector<Vector3d>& path =
                records[static_cast<std::size_t>(marker.recordIndex)].points;
            if (path.size() < 2U)
            {
                continue;
            }
            const Vector3d firstPoint = topologyPoint(path.front());
            const Vector3d lastPoint = topologyPoint(path.back());
            if (distance3D(marker.point, firstPoint) <= tolerance.nodeSnap)
            {
                marker.position = 0.0;
                marker.point = firstPoint;
            }
            else if (distance3D(marker.point, lastPoint) <= tolerance.nodeSnap)
            {
                marker.position = static_cast<double>(path.size() - 1U);
                marker.point = lastPoint;
            }
        }

        std::vector<Marker> uniqueMarkers;
        for (const Marker& marker : graph.markers)
        {
            addMarker(uniqueMarkers, marker.recordIndex, marker.position, marker.point);
        }
        graph.markers = std::move(uniqueMarkers);

        for (int recordIndex : candidateIndices)
        {
            const TopologyPathRecord& record = records[static_cast<std::size_t>(recordIndex)];
            if (!record.semanticallyClosed || record.points.size() < 3U)
            {
                continue;
            }
            std::vector<Marker> recordMarkers;
            for (const Marker& marker : graph.markers)
            {
                if (marker.recordIndex == recordIndex)
                {
                    recordMarkers.push_back(marker);
                }
            }
            if (recordMarkers.size() == 1U)
            {
                const double period = static_cast<double>(record.points.size() - 1U);
                double oppositePosition = recordMarkers.front().position + period * 0.5;
                if (oppositePosition >= period)
                {
                    oppositePosition -= period;
                }
                addMarker
                (
                    graph.markers, recordIndex, oppositePosition,
                    topologyPoint(pointAtPosition(record.points, oppositePosition))
                );
            }
        }

        for (int markerIndex = 0; markerIndex < static_cast<int>(graph.markers.size()); ++markerIndex)
        {
            graph.markerIndicesByRecord
                [static_cast<std::size_t>(graph.markers[static_cast<std::size_t>(markerIndex)].recordIndex)]
                .push_back(markerIndex);
        }

        DisjointSet groups(static_cast<int>(graph.markers.size()));
        struct MergeCandidate
        {
            int left = -1;
            int right = -1;
            double distance = 0.0;
        };
        std::vector<MergeCandidate> mergeCandidates;
        for (int left = 0; left < static_cast<int>(graph.markers.size()); ++left)
        {
            for (int right = left + 1; right < static_cast<int>(graph.markers.size()); ++right)
            {
                if (graph.markers[static_cast<std::size_t>(left)].recordIndex
                    == graph.markers[static_cast<std::size_t>(right)].recordIndex)
                {
                    continue;
                }
                const double distance = distance3D
                (
                    graph.markers[static_cast<std::size_t>(left)].point,
                    graph.markers[static_cast<std::size_t>(right)].point
                );
                if (distance <= tolerance.nodeSnap)
                {
                    mergeCandidates.push_back({ left, right, distance });
                }
            }
        }
        std::sort
        (
            mergeCandidates.begin(), mergeCandidates.end(),
            [](const MergeCandidate& left, const MergeCandidate& right)
            {
                if (std::abs(left.distance - right.distance) > kEpsilon)
                {
                    return left.distance < right.distance;
                }
                return left.left < right.left
                    || (left.left == right.left && left.right < right.right);
            }
        );
        for (const MergeCandidate& candidate : mergeCandidates)
        {
            if (!groupsConflict(groups, graph.markers, candidate.left, candidate.right))
            {
                groups.unite(candidate.left, candidate.right);
            }
        }

        std::map<int, int> nodeByRoot;
        std::vector<std::vector<int>> markersByNode;
        for (int markerIndex = 0; markerIndex < static_cast<int>(graph.markers.size()); ++markerIndex)
        {
            const int root = groups.find(markerIndex);
            auto [iterator, inserted] = nodeByRoot.emplace(root, static_cast<int>(graph.nodes.size()));
            if (inserted)
            {
                graph.nodes.push_back({});
                markersByNode.push_back({});
            }
            markersByNode[static_cast<std::size_t>(iterator->second)].push_back(markerIndex);
        }
        for (std::size_t nodeIndex = 0; nodeIndex < graph.nodes.size(); ++nodeIndex)
        {
            Vector3d average;
            for (int markerIndex : markersByNode[nodeIndex])
            {
                average = add
                    (average, graph.markers[static_cast<std::size_t>(markerIndex)].point);
            }
            graph.nodes[nodeIndex] = multiply
                (average, 1.0 / static_cast<double>(markersByNode[nodeIndex].size()));
        }

        for (int recordIndex : candidateIndices)
        {
            std::vector<int> markerIndices =
                graph.markerIndicesByRecord[static_cast<std::size_t>(recordIndex)];
            std::sort
            (
                markerIndices.begin(), markerIndices.end(),
                [&graph](int left, int right)
                {
                    return graph.markers[static_cast<std::size_t>(left)].position
                        < graph.markers[static_cast<std::size_t>(right)].position;
                }
            );
            const TopologyPathRecord& record = records[static_cast<std::size_t>(recordIndex)];
            const int chunkCount = record.semanticallyClosed && markerIndices.size() >= 2U
                ? static_cast<int>(markerIndices.size())
                : std::max(0, static_cast<int>(markerIndices.size()) - 1);
            for (int marker = 0; marker < chunkCount; ++marker)
            {
                const Marker& start = graph.markers
                    [static_cast<std::size_t>(markerIndices[static_cast<std::size_t>(marker)])];
                const int nextMarker = (marker + 1) % static_cast<int>(markerIndices.size());
                const Marker& end = graph.markers
                    [static_cast<std::size_t>(markerIndices[static_cast<std::size_t>(nextMarker)])];
                std::vector<Vector3d> points = record.semanticallyClosed
                    ? cyclicPathBetween(record.points, start.position, end.position)
                    : pathBetween(record.points, start.position, end.position);
                const double length = pathLength(points);
                if (length <= tolerance.minimumEdgeLength)
                {
                    continue;
                }
                const int firstNode = nodeByRoot.at
                    (groups.find(markerIndices[static_cast<std::size_t>(marker)]));
                const int secondNode = nodeByRoot.at
                    (groups.find(markerIndices[static_cast<std::size_t>(nextMarker)]));
                if (firstNode == secondNode)
                {
                    continue;
                }
                graph.edges.push_back
                    ({ firstNode, secondNode, recordIndex, std::move(points), length });
            }
        }

        graph.incidentEdges.resize(graph.nodes.size());
        for (int edgeIndex = 0; edgeIndex < static_cast<int>(graph.edges.size()); ++edgeIndex)
        {
            const GraphEdge& edge = graph.edges[static_cast<std::size_t>(edgeIndex)];
            graph.incidentEdges[static_cast<std::size_t>(edge.firstNode)].push_back(edgeIndex);
            graph.incidentEdges[static_cast<std::size_t>(edge.secondNode)].push_back(edgeIndex);
        }
        return true;
    }

    std::vector<int> graphRecordComponentIds
    (
        const GraphData& graph,
        const std::vector<int>& candidateIndices
    )
    {
        std::map<int, int> localByRecord;
        for (int local = 0; local < static_cast<int>(candidateIndices.size()); ++local)
        {
            localByRecord.emplace(candidateIndices[static_cast<std::size_t>(local)], local);
        }
        DisjointSet groups(static_cast<int>(candidateIndices.size()));
        for (const std::vector<int>& incidentEdges : graph.incidentEdges)
        {
            int firstLocal = -1;
            for (int edgeIndex : incidentEdges)
            {
                const int recordIndex = graph.edges[static_cast<std::size_t>(edgeIndex)].recordIndex;
                const auto local = localByRecord.find(recordIndex);
                if (local == localByRecord.end())
                {
                    continue;
                }
                if (firstLocal < 0)
                {
                    firstLocal = local->second;
                }
                else
                {
                    groups.unite(firstLocal, local->second);
                }
            }
        }
        std::map<int, int> componentByRoot;
        std::vector<int> result(candidateIndices.size(), -1);
        for (int local = 0; local < static_cast<int>(candidateIndices.size()); ++local)
        {
            const int root = groups.find(local);
            const auto [iterator, inserted] = componentByRoot.emplace
                (root, static_cast<int>(componentByRoot.size()));
            result[static_cast<std::size_t>(local)] = iterator->second;
        }
        return result;
    }

    void appendEdgePoints
    (
        std::vector<Vector3d>& path,
        const GraphEdge& edge,
        int fromNode
    )
    {
        if (fromNode == edge.firstNode)
        {
            for (const Vector3d& point : edge.points)
            {
                if (path.empty() || distance3D(path.back(), point) > kEpsilon)
                {
                    path.push_back(point);
                }
            }
        }
        else
        {
            for (auto point = edge.points.crbegin(); point != edge.points.crend(); ++point)
            {
                if (path.empty() || distance3D(path.back(), *point) > kEpsilon)
                {
                    path.push_back(*point);
                }
            }
        }
    }

    double projectedAreaYZ(const std::vector<Vector3d>& path)
    {
        double area = 0.0;
        for (std::size_t index = 0; index < path.size(); ++index)
        {
            const Vector3d& start = path[index];
            const Vector3d& end = path[(index + 1U) % path.size()];
            area += start.y * end.z - end.y * start.z;
        }
        return std::abs(area) * 0.5;
    }

    void updateStableKeys
    (
        LoopCandidate& candidate,
        const std::vector<TopologyPathRecord>& records
    )
    {
        for (int recordIndex : candidate.recordIndices)
        {
            const TopologyPathRecord& record = records[static_cast<std::size_t>(recordIndex)];
            candidate.minimumSourceIndex = std::min
                (candidate.minimumSourceIndex, record.sourceIndex);
            candidate.minimumEntityId = std::min(candidate.minimumEntityId, record.entityId);
        }
    }

    LoopCandidate candidateFromEdgeSequence
    (
        const GraphData& graph,
        const std::vector<int>& edgeSequence,
        int startNode,
        const std::vector<TopologyPathRecord>& records
    )
    {
        LoopCandidate candidate;
        int currentNode = startNode;
        Vector3d firstPhysicalPoint;
        Vector3d previousPhysicalPoint;
        bool hasPhysicalPoint = false;
        for (int edgeIndex : edgeSequence)
        {
            const GraphEdge& edge = graph.edges[static_cast<std::size_t>(edgeIndex)];
            const Vector3d physicalStart = currentNode == edge.firstNode
                ? edge.points.front() : edge.points.back();
            const Vector3d physicalEnd = currentNode == edge.firstNode
                ? edge.points.back() : edge.points.front();
            if (!hasPhysicalPoint)
            {
                firstPhysicalPoint = physicalStart;
                hasPhysicalPoint = true;
            }
            else
            {
                candidate.maximumJoinGap = std::max
                    (candidate.maximumJoinGap,
                     distance3D(previousPhysicalPoint, physicalStart));
            }
            appendEdgePoints(candidate.orderedPath, edge, currentNode);
            candidate.recordIndices.insert(edge.recordIndex);
            candidate.length += edge.length;
            previousPhysicalPoint = physicalEnd;
            currentNode = edge.firstNode == currentNode ? edge.secondNode : edge.firstNode;
        }
        if (hasPhysicalPoint)
        {
            candidate.maximumJoinGap = std::max
                (candidate.maximumJoinGap,
                 distance3D(previousPhysicalPoint, firstPhysicalPoint));
        }
        candidate.projectedArea = projectedAreaYZ(candidate.orderedPath);
        updateStableKeys(candidate, records);
        return candidate;
    }

    Diagnostic discontinuousLoopDiagnostic
    (
        const OperationContext& context,
        DiagnosticSeverity severity,
        const PathTopologyTolerance& tolerance,
        const LoopCandidate& candidate,
        const std::vector<TopologyPathRecord>& records,
        std::size_t nodeCount,
        std::size_t edgeCount
    )
    {
        Diagnostic diagnostic = topologyDiagnostic
        (
            context,
            DiagnosticCode::TopologyLoopDiscontinuous,
            severity,
            QStringLiteral("candidate graph loop maximum physical join gap is %1 mm")
                .arg(candidate.maximumJoinGap, 0, 'g', 12),
            tolerance,
            records.size(),
            nodeCount,
            edgeCount,
            0,
            0,
            0,
            candidate.maximumJoinGap
        );
        QVariantList usedEntityIds;
        const TopologyPathRecord* firstRecord = nullptr;
        for (int recordIndex : candidate.recordIndices)
        {
            const TopologyPathRecord& record = records[static_cast<std::size_t>(recordIndex)];
            usedEntityIds.push_back(QVariant::fromValue<qulonglong>(record.entityId));
            if (firstRecord == nullptr || record.sourceIndex < firstRecord->sourceIndex)
            {
                firstRecord = &record;
            }
        }
        diagnostic.context.insert(QStringLiteral("usedEntityIds"), usedEntityIds);
        diagnostic.userMessage =
            QStringLiteral("候选路径存在未连接间隙，不能作为闭合断面。最大连接间隙：%1 mm。")
                .arg(candidate.maximumJoinGap, 0, 'g', 12);
        if (firstRecord != nullptr)
        {
            diagnostic.entityId = firstRecord->entityId;
            diagnostic.context.insert
                (QStringLiteral("entityId"), static_cast<qulonglong>(firstRecord->entityId));
            diagnostic.context.insert
                (QStringLiteral("sourceIndex"),
                 static_cast<qulonglong>(firstRecord->sourceIndex));
        }
        return diagnostic;
    }

    std::vector<int> edgeComponent
    (
        const GraphData& graph,
        int startEdge,
        const std::vector<bool>& allowedEdges,
        std::vector<bool>& visited
    )
    {
        std::vector<int> result{ startEdge };
        visited[static_cast<std::size_t>(startEdge)] = true;
        for (std::size_t cursor = 0; cursor < result.size(); ++cursor)
        {
            const GraphEdge& edge = graph.edges[static_cast<std::size_t>(result[cursor])];
            for (int node : { edge.firstNode, edge.secondNode })
            {
                for (int neighbor : graph.incidentEdges[static_cast<std::size_t>(node)])
                {
                    if (allowedEdges[static_cast<std::size_t>(neighbor)]
                        && !visited[static_cast<std::size_t>(neighbor)])
                    {
                        visited[static_cast<std::size_t>(neighbor)] = true;
                        result.push_back(neighbor);
                    }
                }
            }
        }
        return result;
    }

    std::vector<int> orderedCycleEdges
    (
        const GraphData& graph,
        const std::vector<int>& component,
        int& startNode
    )
    {
        const std::set<int> componentSet(component.begin(), component.end());
        std::map<int, int> degree;
        for (int edgeIndex : component)
        {
            ++degree[graph.edges[static_cast<std::size_t>(edgeIndex)].firstNode];
            ++degree[graph.edges[static_cast<std::size_t>(edgeIndex)].secondNode];
        }
        for (const auto& [node, value] : degree)
        {
            if (value != 2)
            {
                return {};
            }
        }
        startNode = degree.begin()->first;
        std::vector<int> ordered;
        std::set<int> used;
        int currentNode = startNode;
        while (ordered.size() < component.size())
        {
            int nextEdge = -1;
            for (int edgeIndex : graph.incidentEdges[static_cast<std::size_t>(currentNode)])
            {
                if (componentSet.count(edgeIndex) != 0U && used.count(edgeIndex) == 0U)
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
            const GraphEdge& edge = graph.edges[static_cast<std::size_t>(nextEdge)];
            currentNode = edge.firstNode == currentNode ? edge.secondNode : edge.firstNode;
        }
        return currentNode == startNode ? ordered : std::vector<int>{};
    }

    bool enumerateSimpleCycles
    (
        const GraphData& graph,
        int maximumCycleCount,
        const CancellationToken& cancellationToken,
        std::vector<std::pair<int, std::vector<int>>>& cycles
    )
    {
        std::set<std::vector<int>> cycleKeys;
        for (int startNode = 0;
            startNode < static_cast<int>(graph.nodes.size())
            && static_cast<int>(cycles.size()) < maximumCycleCount; ++startNode)
        {
            if (cancellationToken.isCancellationRequested())
            {
                return false;
            }
            std::set<int> visitedNodes{ startNode };
            std::set<int> usedEdges;
            std::vector<int> currentEdges;
            std::function<void(int)> visit = [&](int currentNode)
            {
                if (cancellationToken.isCancellationRequested()
                    || static_cast<int>(cycles.size()) >= maximumCycleCount)
                {
                    return;
                }
                for (int edgeIndex : graph.incidentEdges[static_cast<std::size_t>(currentNode)])
                {
                    if (usedEdges.count(edgeIndex) != 0U)
                    {
                        continue;
                    }
                    const GraphEdge& edge = graph.edges[static_cast<std::size_t>(edgeIndex)];
                    const int neighbor = edge.firstNode == currentNode
                        ? edge.secondNode : edge.firstNode;
                    if (neighbor == startNode)
                    {
                        if (!currentEdges.empty())
                        {
                            std::vector<int> cycleEdges = currentEdges;
                            cycleEdges.push_back(edgeIndex);
                            std::vector<int> key = cycleEdges;
                            std::sort(key.begin(), key.end());
                            if (cycleKeys.insert(key).second)
                            {
                                cycles.emplace_back(startNode, std::move(cycleEdges));
                            }
                        }
                        continue;
                    }
                    if (visitedNodes.count(neighbor) != 0U || neighbor < startNode)
                    {
                        continue;
                    }
                    visitedNodes.insert(neighbor);
                    usedEdges.insert(edgeIndex);
                    currentEdges.push_back(edgeIndex);
                    visit(neighbor);
                    currentEdges.pop_back();
                    usedEdges.erase(edgeIndex);
                    visitedNodes.erase(neighbor);
                }
            };
            visit(startNode);
        }
        return !cancellationToken.isCancellationRequested();
    }

    std::vector<int> indicesForIds
    (
        const std::vector<TopologyPathRecord>& records,
        const std::vector<EntityId>& ids,
        bool allWhenEmpty
    )
    {
        std::set<EntityId> requested(ids.begin(), ids.end());
        std::vector<int> result;
        for (int index = 0; index < static_cast<int>(records.size()); ++index)
        {
            if ((ids.empty() && allWhenEmpty)
                || requested.count(records[static_cast<std::size_t>(index)].entityId) != 0U)
            {
                result.push_back(index);
            }
        }
        return result;
    }

    int countComponents(const std::vector<int>& componentIds)
    {
        return componentIds.empty()
            ? 0 : *std::max_element(componentIds.cbegin(), componentIds.cend()) + 1;
    }
}

PathTopologyTolerance PathTopologyTolerance::fromConnectionTolerance
(double connectionTolerance)
{
    const double nodeSnap = std::max(1.0e-9, connectionTolerance);
    return
    {
        nodeSnap,
        1.0e-5,
        std::max(1.0e-9, nodeSnap * 0.01),
        1.0e-6
    };
}

const std::vector<TopologyPathRecord>& PathTopology::records() const
{
    return m_records;
}

const std::vector<std::vector<int>>& PathTopology::adjacency() const
{
    return m_adjacency;
}

const PathTopologyStatistics& PathTopology::statistics() const
{
    return m_statistics;
}

std::vector<int> PathTopology::componentIds(const std::vector<EntityId>& subset) const
{
    const std::vector<int> indices = indicesForIds(m_records, subset, true);
    std::map<int, int> localByGlobal;
    for (int local = 0; local < static_cast<int>(indices.size()); ++local)
    {
        localByGlobal.emplace(indices[static_cast<std::size_t>(local)], local);
    }
    std::vector<int> result(indices.size(), -1);
    int nextComponent = 0;
    for (int start = 0; start < static_cast<int>(indices.size()); ++start)
    {
        if (result[static_cast<std::size_t>(start)] >= 0)
        {
            continue;
        }
        std::vector<int> pending{ start };
        result[static_cast<std::size_t>(start)] = nextComponent;
        for (std::size_t cursor = 0; cursor < pending.size(); ++cursor)
        {
            const int local = pending[cursor];
            const int global = indices[static_cast<std::size_t>(local)];
            for (int neighborGlobal : m_adjacency[static_cast<std::size_t>(global)])
            {
                const auto neighbor = localByGlobal.find(neighborGlobal);
                if (neighbor != localByGlobal.end()
                    && result[static_cast<std::size_t>(neighbor->second)] < 0)
                {
                    result[static_cast<std::size_t>(neighbor->second)] = nextComponent;
                    pending.push_back(neighbor->second);
                }
            }
        }
        ++nextComponent;
    }
    return result;
}

bool PathTopology::directlyConnected(EntityId left, EntityId right) const
{
    int leftIndex = -1;
    int rightIndex = -1;
    for (int index = 0; index < static_cast<int>(m_records.size()); ++index)
    {
        leftIndex = m_records[static_cast<std::size_t>(index)].entityId == left
            ? index : leftIndex;
        rightIndex = m_records[static_cast<std::size_t>(index)].entityId == right
            ? index : rightIndex;
    }
    return leftIndex >= 0 && rightIndex >= 0
        && contains(m_adjacency[static_cast<std::size_t>(leftIndex)], rightIndex);
}

OperationResult<TopologyLoopResult> PathTopology::extractSeededLoop
(const std::vector<EntityId>& seeds) const
{
    OperationResult<TopologyLoopResult> result;
    if (seeds.empty())
    {
        result.status = OperationStatus::InvalidInput;
        result.addDiagnostic(topologyDiagnostic
        (
            m_context, DiagnosticCode::TopologySeedNotFound, DiagnosticSeverity::Error,
            QStringLiteral("seed EntityId set is empty"), m_tolerance, m_records.size()
        ));
        return result;
    }

    std::vector<int> pending;
    std::vector<bool> included(m_records.size(), false);
    for (EntityId seed : seeds)
    {
        int recordIndex = -1;
        for (int index = 0; index < static_cast<int>(m_records.size()); ++index)
        {
            if (m_records[static_cast<std::size_t>(index)].entityId == seed)
            {
                recordIndex = index;
                break;
            }
        }
        if (recordIndex < 0)
        {
            result.status = OperationStatus::Failed;
            Diagnostic diagnostic = topologyDiagnostic
            (
                m_context, DiagnosticCode::TopologySeedNotFound, DiagnosticSeverity::Error,
                QStringLiteral("seed EntityId is not present in topology"),
                m_tolerance, m_records.size()
            );
            diagnostic.entityId = seed;
            diagnostic.context.insert
                (QStringLiteral("entityId"), static_cast<qulonglong>(seed));
            result.addDiagnostic(diagnostic);
            return result;
        }
        if (!included[static_cast<std::size_t>(recordIndex)])
        {
            included[static_cast<std::size_t>(recordIndex)] = true;
            pending.push_back(recordIndex);
        }
    }
    for (std::size_t cursor = 0; cursor < pending.size(); ++cursor)
    {
        const int current = pending[cursor];
        for (int neighbor : m_adjacency[static_cast<std::size_t>(current)])
        {
            if (!included[static_cast<std::size_t>(neighbor)])
            {
                included[static_cast<std::size_t>(neighbor)] = true;
                pending.push_back(neighbor);
            }
        }
    }
    std::vector<EntityId> localIds;
    for (std::size_t index = 0; index < included.size(); ++index)
    {
        if (included[index])
        {
            localIds.push_back(m_records[index].entityId);
        }
    }

    result = extractBestLoop(localIds, seeds);
    if (!result.succeeded() || !result.value.has_value() || !result.value->connectedLoop)
    {
        OperationResult<TopologyLoopResult> graphResult = extractBestLoop({}, seeds);
        if (graphResult.succeeded() && graphResult.value.has_value()
            && graphResult.value->connectedLoop)
        {
            result = std::move(graphResult);
        }
    }
    if (!result.value.has_value() || !result.value->connectedLoop)
    {
        return result;
    }
    const std::set<EntityId> used
        (result.value->usedEntityIds.begin(), result.value->usedEntityIds.end());
    for (EntityId seed : seeds)
    {
        if (used.count(seed) == 0U)
        {
            result.status = OperationStatus::Failed;
            result.value->connectedLoop = false;
            Diagnostic diagnostic = topologyDiagnostic
            (
                m_context, DiagnosticCode::TopologyLoopNotFound, DiagnosticSeverity::Error,
                QStringLiteral("selected loop does not contain every seed EntityId"),
                m_tolerance, m_records.size(), 0U, 0U,
                result.value->connectedComponentCount,
                result.value->openNodeCount,
                result.value->branchNodeCount,
                result.value->maximumJoinGap
            );
            diagnostic.entityId = seed;
            diagnostic.context.insert
                (QStringLiteral("entityId"), static_cast<qulonglong>(seed));
            result.addDiagnostic(diagnostic);
            return result;
        }
    }
    return result;
}

OperationResult<TopologyLoopResult> PathTopology::extractBestLoop
(
    const std::vector<EntityId>& candidateIds,
    const std::vector<EntityId>& preferredIds
) const
{
    OperationResult<TopologyLoopResult> result;
    TopologyLoopResult loopResult;
    const std::vector<int> candidateIndices = indicesForIds(m_records, candidateIds, true);
    if (candidateIndices.empty())
    {
        result.status = OperationStatus::InvalidInput;
        result.addDiagnostic(topologyDiagnostic
        (
            m_context, DiagnosticCode::TopologyInputInvalid, DiagnosticSeverity::Error,
            QStringLiteral("topology candidate set is empty"), m_tolerance, m_records.size()
        ));
        return result;
    }
    const std::set<EntityId> preferred(preferredIds.begin(), preferredIds.end());
    std::vector<LoopCandidate> candidates;
    std::vector<LoopCandidate> discontinuousCandidates;
    const auto considerCandidate =
        [this, &candidates, &discontinuousCandidates](LoopCandidate candidate)
    {
        const bool strictlyConnected =
            candidate.maximumJoinGap <= m_tolerance.numericalJoinEpsilon;
        if (strictlyConnected)
        {
            candidates.push_back(std::move(candidate));
        }
        else
        {
            discontinuousCandidates.push_back(std::move(candidate));
        }
        return strictlyConnected;
    };
    for (int recordIndex : candidateIndices)
    {
        const TopologyPathRecord& record = m_records[static_cast<std::size_t>(recordIndex)];
        if (record.semanticallyClosed && record.points.size() >= 3U)
        {
            LoopCandidate candidate;
            candidate.orderedPath = record.points;
            candidate.recordIndices.insert(recordIndex);
            candidate.length = pathLength(record.points);
            candidate.projectedArea = projectedAreaYZ(record.points);
            updateStableKeys(candidate, m_records);
            considerCandidate(std::move(candidate));
        }
        else if (record.points.size() >= 3U)
        {
            const double joinGap = distance3D(record.points.front(), record.points.back());
            if (joinGap <= m_tolerance.nodeSnap)
            {
                LoopCandidate candidate;
                candidate.orderedPath = record.points;
                candidate.recordIndices.insert(recordIndex);
                candidate.maximumJoinGap = joinGap;
                candidate.length = pathLength(record.points);
                candidate.projectedArea = projectedAreaYZ(record.points);
                updateStableKeys(candidate, m_records);
                considerCandidate(std::move(candidate));
            }
        }
    }

    GraphData graph;
    if (!buildGraph
    (
        m_records, candidateIndices, m_tolerance, m_cancellationToken, graph
    ))
    {
        result.status = OperationStatus::Cancelled;
        result.value = loopResult;
        result.addDiagnostic(topologyDiagnostic
        (
            m_context, DiagnosticCode::TopologyBuildFailure, DiagnosticSeverity::Notice,
            QStringLiteral("topology graph build cancelled"), m_tolerance,
            m_records.size(), graph.nodes.size(), graph.edges.size()
        ));
        return result;
    }

    const std::vector<int> graphComponents =
        graphRecordComponentIds(graph, candidateIndices);
    loopResult.connectedComponentCount = countComponents(graphComponents);
    std::vector<int> degree(graph.nodes.size(), 0);
    for (const GraphEdge& edge : graph.edges)
    {
        ++degree[static_cast<std::size_t>(edge.firstNode)];
        ++degree[static_cast<std::size_t>(edge.secondNode)];
    }
    for (int value : degree)
    {
        loopResult.openNodeCount += value == 1 ? 1 : 0;
        loopResult.branchNodeCount += value > 2 ? 1 : 0;
    }

    std::vector<bool> active(graph.edges.size(), true);
    std::vector<int> activeDegree = degree;
    std::queue<int> leaves;
    for (int node = 0; node < static_cast<int>(activeDegree.size()); ++node)
    {
        if (activeDegree[static_cast<std::size_t>(node)] <= 1)
        {
            leaves.push(node);
        }
    }
    while (!leaves.empty())
    {
        const int node = leaves.front();
        leaves.pop();
        for (int edgeIndex : graph.incidentEdges[static_cast<std::size_t>(node)])
        {
            if (!active[static_cast<std::size_t>(edgeIndex)])
            {
                continue;
            }
            active[static_cast<std::size_t>(edgeIndex)] = false;
            const GraphEdge& edge = graph.edges[static_cast<std::size_t>(edgeIndex)];
            const int neighbor = edge.firstNode == node ? edge.secondNode : edge.firstNode;
            --activeDegree[static_cast<std::size_t>(node)];
            --activeDegree[static_cast<std::size_t>(neighbor)];
            if (activeDegree[static_cast<std::size_t>(neighbor)] == 1)
            {
                leaves.push(neighbor);
            }
        }
    }

    std::vector<bool> visitedCore(graph.edges.size(), false);
    bool foundCoreCycle = false;
    for (int edgeIndex = 0; edgeIndex < static_cast<int>(graph.edges.size()); ++edgeIndex)
    {
        if (!active[static_cast<std::size_t>(edgeIndex)]
            || visitedCore[static_cast<std::size_t>(edgeIndex)])
        {
            continue;
        }
        const std::vector<int> component =
            edgeComponent(graph, edgeIndex, active, visitedCore);
        int startNode = -1;
        const std::vector<int> orderedEdges =
            orderedCycleEdges(graph, component, startNode);
        if (!orderedEdges.empty())
        {
            foundCoreCycle = considerCandidate(candidateFromEdgeSequence
                (graph, orderedEdges, startNode, m_records)) || foundCoreCycle;
        }
    }

    if (loopResult.branchNodeCount > 0 && !foundCoreCycle)
    {
        std::vector<std::pair<int, std::vector<int>>> cycles;
        if (!enumerateSimpleCycles(graph, 512, m_cancellationToken, cycles))
        {
            result.status = OperationStatus::Cancelled;
            result.value = loopResult;
            result.addDiagnostic(topologyDiagnostic
            (
                m_context, DiagnosticCode::TopologyBuildFailure, DiagnosticSeverity::Notice,
                QStringLiteral("topology cycle enumeration cancelled"), m_tolerance,
                m_records.size(), graph.nodes.size(), graph.edges.size(),
                loopResult.connectedComponentCount,
                loopResult.openNodeCount, loopResult.branchNodeCount
            ));
            return result;
        }
        for (const auto& cycle : cycles)
        {
            considerCandidate(candidateFromEdgeSequence
                (graph, cycle.second, cycle.first, m_records));
        }
    }

    if (candidates.empty())
    {
        result.status = OperationStatus::Failed;
        result.value = loopResult;
        if (!discontinuousCandidates.empty())
        {
            const auto closest = std::min_element
            (
                discontinuousCandidates.cbegin(), discontinuousCandidates.cend(),
                [](const LoopCandidate& left, const LoopCandidate& right)
                {
                    if (std::abs(left.maximumJoinGap - right.maximumJoinGap) > kEpsilon)
                    {
                        return left.maximumJoinGap < right.maximumJoinGap;
                    }
                    return left.minimumSourceIndex < right.minimumSourceIndex;
                }
            );
            loopResult.maximumJoinGap = closest->maximumJoinGap;
            for (int recordIndex : closest->recordIndices)
            {
                loopResult.usedEntityIds.push_back
                    (m_records[static_cast<std::size_t>(recordIndex)].entityId);
            }
            result.value = loopResult;
            result.addDiagnostic(discontinuousLoopDiagnostic
            (
                m_context, DiagnosticSeverity::Error, m_tolerance, *closest,
                m_records, graph.nodes.size(), graph.edges.size()
            ));
        }
        else
        {
            result.addDiagnostic(topologyDiagnostic
            (
                m_context, DiagnosticCode::TopologyLoopNotFound, DiagnosticSeverity::Error,
                QStringLiteral("no strictly closed candidate was found"),
                m_tolerance, m_records.size(), graph.nodes.size(), graph.edges.size(),
                loopResult.connectedComponentCount,
                loopResult.openNodeCount, loopResult.branchNodeCount
            ));
        }
        return result;
    }

    if (loopResult.connectedComponentCount > 1 && !preferred.empty())
    {
        std::set<int> preferredComponents;
        for (std::size_t local = 0; local < candidateIndices.size(); ++local)
        {
            const TopologyPathRecord& record =
                m_records[static_cast<std::size_t>(candidateIndices[local])];
            if (preferred.count(record.entityId) != 0U)
            {
                preferredComponents.insert(graphComponents[local]);
            }
        }
        if (preferredComponents.size() > 1U)
        {
            result.status = OperationStatus::Failed;
            result.value = loopResult;
            result.addDiagnostic(topologyDiagnostic
            (
                m_context, DiagnosticCode::TopologyLoopNotFound, DiagnosticSeverity::Error,
                QStringLiteral("preferred EntityIds span multiple components"),
                m_tolerance, m_records.size(), graph.nodes.size(), graph.edges.size(),
                loopResult.connectedComponentCount,
                loopResult.openNodeCount, loopResult.branchNodeCount
            ));
            return result;
        }
    }

    const auto preferredCount = [&preferred, this](const LoopCandidate& candidate)
    {
        int count = 0;
        for (int recordIndex : candidate.recordIndices)
        {
            count += preferred.count
                (m_records[static_cast<std::size_t>(recordIndex)].entityId) != 0U ? 1 : 0;
        }
        return count;
    };
    std::stable_sort
    (
        candidates.begin(), candidates.end(),
        [&preferredCount](const LoopCandidate& left, const LoopCandidate& right)
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
            if (std::abs(left.length - right.length) > kEpsilon)
            {
                return left.length > right.length;
            }
            if (left.minimumSourceIndex != right.minimumSourceIndex)
            {
                return left.minimumSourceIndex < right.minimumSourceIndex;
            }
            return left.minimumEntityId < right.minimumEntityId;
        }
    );
    if (candidates.size() > 1U
        && preferredCount(candidates[0]) == preferredCount(candidates[1])
        && std::abs(candidates[0].projectedArea - candidates[1].projectedArea)
            <= m_tolerance.intersection
        && candidates[0].recordIndices.size() == candidates[1].recordIndices.size()
    )
    {
        result.status = OperationStatus::Failed;
        result.value = loopResult;
        result.addDiagnostic(topologyDiagnostic
        (
            m_context, DiagnosticCode::TopologyLoopNotFound, DiagnosticSeverity::Error,
            QStringLiteral("multiple equally ranked loops remain"),
            m_tolerance, m_records.size(), graph.nodes.size(), graph.edges.size(),
            loopResult.connectedComponentCount,
            loopResult.openNodeCount, loopResult.branchNodeCount
        ));
        return result;
    }

    const LoopCandidate& best = candidates.front();
    loopResult.connectedLoop = true;
    loopResult.maximumJoinGap = best.maximumJoinGap;
    loopResult.orderedPath = best.orderedPath;
    for (int recordIndex : best.recordIndices)
    {
        loopResult.usedEntityIds.push_back
            (m_records[static_cast<std::size_t>(recordIndex)].entityId);
    }
    for (int recordIndex : candidateIndices)
    {
        if (best.recordIndices.count(recordIndex) == 0U)
        {
            loopResult.ignoredBranchEntityIds.push_back
                (m_records[static_cast<std::size_t>(recordIndex)].entityId);
        }
    }
    loopResult.ignoredBranchRecordCount =
        static_cast<int>(loopResult.ignoredBranchEntityIds.size());
    result.status = OperationStatus::Success;
    if (loopResult.ignoredBranchRecordCount > 0)
    {
        result.status = OperationStatus::PartialSuccess;
        result.addDiagnostic(topologyDiagnostic
        (
            m_context, DiagnosticCode::TopologyBranchIgnored, DiagnosticSeverity::Warning,
            QStringLiteral("branch records were excluded from the selected loop"),
            m_tolerance, m_records.size(), graph.nodes.size(), graph.edges.size(),
            loopResult.connectedComponentCount,
            loopResult.openNodeCount, loopResult.branchNodeCount,
            loopResult.maximumJoinGap
        ));
    }
    if (!discontinuousCandidates.empty())
    {
        result.status = OperationStatus::PartialSuccess;
        const auto closest = std::min_element
        (
            discontinuousCandidates.cbegin(), discontinuousCandidates.cend(),
            [](const LoopCandidate& left, const LoopCandidate& right)
            {
                return left.maximumJoinGap < right.maximumJoinGap;
            }
        );
        result.addDiagnostic(discontinuousLoopDiagnostic
        (
            m_context, DiagnosticSeverity::Warning, m_tolerance, *closest,
            m_records, graph.nodes.size(), graph.edges.size()
        ));
    }
    result.value = std::move(loopResult);
    return result;
}

OperationResult<PathTopology> PathTopologyBuilder::build
(
    const TopologyInput& input,
    const PathTopologyTolerance& tolerance,
    const TaskContext& taskContext
) const
{
    OperationResult<PathTopology> result;
    const bool toleranceValid = input.contentRevision != 0U
        && std::isfinite(tolerance.nodeSnap) && tolerance.nodeSnap > 0.0
        && std::isfinite(tolerance.numericalJoinEpsilon)
        && tolerance.numericalJoinEpsilon > 0.0
        && tolerance.numericalJoinEpsilon <= kMaximumNumericalJoinEpsilon
        && std::isfinite(tolerance.intersection) && tolerance.intersection > 0.0
        && std::isfinite(tolerance.minimumEdgeLength) && tolerance.minimumEdgeLength >= 0.0;
    if (!toleranceValid)
    {
        result.status = OperationStatus::InvalidInput;
        result.addDiagnostic(topologyDiagnostic
        (
            taskContext.operationContext,
            DiagnosticCode::TopologyInputInvalid,
            DiagnosticSeverity::Error,
            QStringLiteral("topology revision or tolerance is invalid"),
            tolerance,
            input.records.size()
        ));
        return result;
    }

    PathTopology topology;
    topology.m_tolerance = tolerance;
    topology.m_records = input.records;
    topology.m_context = taskContext.operationContext;
    topology.m_cancellationToken = taskContext.cancellationToken;
    std::stable_sort
    (
        topology.m_records.begin(), topology.m_records.end(),
        [](const TopologyPathRecord& left, const TopologyPathRecord& right)
        {
            if (left.sourceIndex != right.sourceIndex)
            {
                return left.sourceIndex < right.sourceIndex;
            }
            return left.entityId < right.entityId;
        }
    );

    std::set<EntityId> entityIds;
    for (const TopologyPathRecord& record : topology.m_records)
    {
        if (record.entityId == 0U || !entityIds.insert(record.entityId).second
            || record.points.size() < 2U)
        {
            result.status = OperationStatus::InvalidInput;
            Diagnostic diagnostic = topologyDiagnostic
            (
                taskContext.operationContext,
                DiagnosticCode::TopologyInputInvalid,
                DiagnosticSeverity::Error,
                QStringLiteral("record has duplicate/zero EntityId or fewer than two points"),
                tolerance,
                topology.m_records.size()
            );
            diagnostic.entityId = record.entityId;
            diagnostic.context.insert
                (QStringLiteral("entityId"), static_cast<qulonglong>(record.entityId));
            diagnostic.context.insert
                (QStringLiteral("sourceIndex"), static_cast<qulonglong>(record.sourceIndex));
            result.addDiagnostic(diagnostic);
            return result;
        }
    }

    topology.m_adjacency.resize(topology.m_records.size());
    taskContext.reportProgress(0U, topology.m_records.size());
    for (std::size_t left = 0; left < topology.m_records.size(); ++left)
    {
        if (taskContext.cancellationToken.isCancellationRequested())
        {
            result.status = OperationStatus::Cancelled;
            result.addDiagnostic(topologyDiagnostic
            (
                taskContext.operationContext,
                DiagnosticCode::TopologyBuildFailure,
                DiagnosticSeverity::Notice,
                QStringLiteral("topology adjacency build cancelled"),
                tolerance,
                topology.m_records.size()
            ));
            return result;
        }
        for (std::size_t right = left + 1U; right < topology.m_records.size(); ++right)
        {
            if (pathsConnected(topology.m_records[left], topology.m_records[right], tolerance))
            {
                topology.m_adjacency[left].push_back(static_cast<int>(right));
                topology.m_adjacency[right].push_back(static_cast<int>(left));
            }
        }
        taskContext.reportProgress(left + 1U, topology.m_records.size());
    }

    topology.m_statistics.recordCount = topology.m_records.size();
    topology.m_statistics.componentCount = countComponents(topology.componentIds());
    result.status = input.diagnostics.isEmpty()
        ? OperationStatus::Success : OperationStatus::PartialSuccess;
    result.diagnostics = input.diagnostics;
    result.value = std::move(topology);
    return result;
}
}
