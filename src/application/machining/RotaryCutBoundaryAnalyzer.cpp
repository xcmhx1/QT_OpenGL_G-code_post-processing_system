#include "platform/pch.h"

#include "application/machining/RotaryCutBoundaryAnalyzer.h"

#include "cad/items/CadItem.h"
#include "application/machining/RotaryPathTopology.h"
#include "application/machining/RotaryTubeGeometryAnalyzer.h"
#include "core/diagnostics/OperationContext.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <QVariantList>

namespace
{
    using cadcam::geometry::EntityId;
    using cadcam::geometry::Vector2d;
    using cadcam::geometry::Vector3d;
    using cadcam::machining::TubeCutAnalysis;
    using cadcam::machining::TubeCutBoundaryClassifier;
    using cadcam::machining::TubeSectionGeometry;

    constexpr double kCalculationEpsilon = 1.0e-12;
    constexpr double kDuplicatePointTolerance = 1.0e-6;

    double distance3D(const QVector3D& left, const QVector3D& right)
    {
        return static_cast<double>((left - right).length());
    }

    double distance2D(const QVector2D& left, const QVector2D& right)
    {
        return static_cast<double>((left - right).length());
    }

    QVector2D nearestPointOnSegment
    (
        const QVector2D& point,
        const QVector2D& start,
        const QVector2D& end,
        double& factor
    )
    {
        const QVector2D delta = end - start;
        const double lengthSquared = static_cast<double>(QVector2D::dotProduct(delta, delta));
        factor = lengthSquared > kCalculationEpsilon
            ? std::clamp
            (
                static_cast<double>(QVector2D::dotProduct(point - start, delta)) / lengthSquared,
                0.0,
                1.0
            )
            : 0.0;
        return start + delta * static_cast<float>(factor);
    }

    bool mapToSection
    (
        const QVector2D& point,
        const QVector<QVector2D>& boundary,
        double& perimeterPosition,
        double& deviation
    )
    {
        if (boundary.size() < 3)
        {
            return false;
        }

        double cumulative = 0.0;
        deviation = std::numeric_limits<double>::max();
        perimeterPosition = 0.0;

        for (int index = 0; index < boundary.size(); ++index)
        {
            const QVector2D& start = boundary[index];
            const QVector2D& end = boundary[(index + 1) % boundary.size()];
            const double length = distance2D(start, end);

            if (length <= kCalculationEpsilon)
            {
                continue;
            }

            double factor = 0.0;
            const QVector2D nearest = nearestPointOnSegment(point, start, end, factor);
            const double candidateDeviation = distance2D(point, nearest);

            if (candidateDeviation < deviation)
            {
                deviation = candidateDeviation;
                perimeterPosition = cumulative + length * factor;
            }

            cumulative += length;
        }

        return std::isfinite(deviation);
    }

    QString firstDiagnosticMessage(const QVector<Diagnostic>& diagnostics)
    {
        for (const Diagnostic& diagnostic : diagnostics)
        {
            if (!diagnostic.userMessage.isEmpty())
            {
                return diagnostic.userMessage;
            }
        }

        return {};
    }

    Diagnostic topologyDiagnostic
    (
        const RotaryPathLoopResult& loop,
        const QVector<CadItem*>& candidateItems,
        const OperationContext& context
    )
    {
        Diagnostic diagnostic;
        diagnostic.code = DiagnosticCode::CutBoundaryTopologyInvalid;
        diagnostic.severity = DiagnosticSeverity::Error;
        diagnostic.component = QStringLiteral("RotaryCutBoundaryAnalyzer");
        diagnostic.operation = context.operationName;
        diagnostic.stage = QStringLiteral("strict-loop-extraction");
        diagnostic.userMessage = loop.errorMessage.isEmpty()
            ? QStringLiteral("加工断面候选无法形成严格闭环。")
            : loop.errorMessage;
        diagnostic.technicalDetail = QStringLiteral("RotaryPathTopology::extractBestLoop failed.");
        diagnostic.correlationId = context.correlationId;
        QVariantList entityIds;

        for (const CadItem* item : candidateItems)
        {
            if (item != nullptr)
            {
                entityIds.push_back(QVariant::fromValue<qulonglong>(item->m_entityId));
            }
        }

        diagnostic.context.insert(QStringLiteral("boundaryEntityIds"), entityIds);
        diagnostic.context.insert(QStringLiteral("maximumJoinGap"), loop.maximumJoinGap);
        diagnostic.context.insert(QStringLiteral("maximumSurfaceDeviation"), 0.0);
        diagnostic.context.insert(QStringLiteral("maximumProjectionCoverageGap"), 0.0);
        diagnostic.context.insert(QStringLiteral("projectedCenterY"), 0.0);
        diagnostic.context.insert(QStringLiteral("projectedCenterZ"), 0.0);
        diagnostic.context.insert(QStringLiteral("expectedCenterY"), 0.0);
        diagnostic.context.insert(QStringLiteral("expectedCenterZ"), 0.0);
        diagnostic.context.insert(QStringLiteral("projectedYLength"), 0.0);
        diagnostic.context.insert(QStringLiteral("projectedZWidth"), 0.0);
        diagnostic.context.insert(QStringLiteral("expectedYLength"), 0.0);
        diagnostic.context.insert(QStringLiteral("expectedZWidth"), 0.0);
        diagnostic.context.insert(QStringLiteral("globalWinding"), 0);
        diagnostic.context.insert(QStringLiteral("seam0Winding"), 0);
        diagnostic.context.insert(QStringLiteral("seam1Winding"), 0);
        diagnostic.context.insert(QStringLiteral("seam2Winding"), 0);
        diagnostic.context.insert(QStringLiteral("seam3Winding"), 0);
        diagnostic.context.insert(QStringLiteral("positiveCrossingCount"), 0);
        diagnostic.context.insert(QStringLiteral("negativeCrossingCount"), 0);
        diagnostic.context.insert(QStringLiteral("touchCount"), 0);
        diagnostic.context.insert(QStringLiteral("overlapRunCount"), 0);
        return diagnostic;
    }
}

RotaryCutBoundaryAnalysis RotaryCutBoundaryAnalyzer::analyze
(
    const QVector<CadItem*>& candidateItems,
    const QVector<CadItem*>& sceneItems,
    const RotaryTubeSectionModel& sectionModel,
    double connectionTolerance
)
{
    RotaryCutBoundaryAnalysis analysis;
    const RotaryPathTopology topology
    (
        sceneItems,
        RotaryPathTopologyTolerance::fromConnectionTolerance(connectionTolerance)
    );
    const RotaryPathLoopResult loop = topology.extractBestLoop(candidateItems, candidateItems);
    const OperationContext context = createOperationContext(QStringLiteral("AnalyzeRotaryCutBoundary"));
    analysis.maximumJoinGap = loop.maximumJoinGap;
    analysis.boundaryItems = loop.usedItems;
    analysis.connectedLoop = loop.valid && loop.connectedLoop;

    if (!loop.valid || !loop.connectedLoop || loop.orderedPath.size() < 3)
    {
        analysis.diagnostics.push_back(topologyDiagnostic(loop, candidateItems, context));
        analysis.errorMessage = analysis.diagnostics.front().userMessage;
        return analysis;
    }

    std::vector<Vector3d> orderedPath;
    orderedPath.reserve(static_cast<std::size_t>(loop.orderedPath.size()));

    for (const QVector3D& point : loop.orderedPath)
    {
        orderedPath.push_back({ point.x(), point.y(), point.z() });
    }

    std::vector<EntityId> entityIds;
    entityIds.reserve(static_cast<std::size_t>(loop.usedItems.size()));

    for (const CadItem* item : loop.usedItems)
    {
        if (item != nullptr)
        {
            entityIds.push_back(item->m_entityId);
        }
    }

    const TubeSectionGeometry sourceSection = sectionModel.coreModel.has_value()
        ? sectionModel.coreModel->geometry
        : TubeSectionGeometry{};
    // Independent DXF ellipse tessellations can differ by several microns even
    // when they represent the same rounded tube surface.
    const double surfaceMappingTolerance = std::clamp
        (connectionTolerance * 0.01, 1.0e-4, 0.01);
    const OperationResult<TubeCutAnalysis> coreResult = TubeCutBoundaryClassifier::analyze
    (
        orderedPath,
        entityIds,
        loop.maximumJoinGap,
        sourceSection,
        context,
        surfaceMappingTolerance
    );
    analysis.diagnostics = coreResult.diagnostics;

    if (!coreResult.value.has_value())
    {
        analysis.errorMessage = firstDiagnosticMessage(coreResult.diagnostics);

        if (analysis.errorMessage.isEmpty())
        {
            analysis.errorMessage = QStringLiteral("加工断面核心判定失败。");
        }

        return analysis;
    }

    const TubeCutAnalysis& core = *coreResult.value;
    analysis.result = core.result;
    analysis.projectionMatchesSection = core.projectionMatchesSection;
    analysis.surfaceMappingValid = core.surfaceMappingValid;
    analysis.winding = core.winding;
    analysis.seamResults = core.seamResults;
    analysis.maximumSurfaceDeviation = core.maximumSurfaceDeviation;
    analysis.maximumProjectionCoverageGap = core.maximumProjectionCoverageGap;
    analysis.projectedCenterY = core.projectedCenterY;
    analysis.projectedCenterZ = core.projectedCenterZ;
    analysis.projectedYLength = core.projectedYLength;
    analysis.projectedZWidth = core.projectedZWidth;

    const auto preparedSection = TubeCutBoundaryClassifier::prepareSection(sourceSection, context);

    if (preparedSection.value.has_value())
    {
        analysis.sectionPerimeter = preparedSection.value->perimeter;
        analysis.seamPositions = preparedSection.value->seamPositions;
        analysis.sectionBoundary.reserve
            (static_cast<qsizetype>(preparedSection.value->boundary.size()));

        for (const Vector2d& point : preparedSection.value->boundary)
        {
            analysis.sectionBoundary.push_back
                (QVector2D(static_cast<float>(point.x), static_cast<float>(point.y)));
        }
    }

    analysis.orderedPath.reserve(static_cast<qsizetype>(core.orderedPath.size()));

    for (const Vector3d& point : core.orderedPath)
    {
        analysis.orderedPath.push_back
            (QVector3D(static_cast<float>(point.x), static_cast<float>(point.y), static_cast<float>(point.z)));
    }

    analysis.unwrappedBoundary.reserve
        (static_cast<qsizetype>(core.unwrappedBoundary.size()));

    for (const cadcam::machining::UnwrappedBoundaryPoint& point : core.unwrappedBoundary)
    {
        analysis.unwrappedBoundary.push_back({ point.x, point.perimeterPosition });
    }

    analysis.valid = core.result == TubeCutResult::CutsLeftAndRight;
    analysis.errorMessage = analysis.valid ? QString() : firstDiagnosticMessage(coreResult.diagnostics);

    if (!analysis.valid && analysis.errorMessage.isEmpty())
    {
        analysis.errorMessage = core.result == TubeCutResult::KeepsLeftAndRight
            ? QStringLiteral("候选曲线的 YZ 投影覆盖完整方管截面，但周向有向绕数为 0，切割后仍存在连接左右两端的材料桥。")
            : QStringLiteral("加工断面判定结果不确定。");
    }

    return analysis;
}

RotaryBoundarySide RotaryCutBoundaryAnalyzer::classifyPointRelativeToBoundary
(
    const RotaryCutBoundaryAnalysis& analysis,
    const QVector3D& point,
    double tolerance,
    RotaryBoundaryPointClassificationDiagnostics* diagnostics
)
{
    if (diagnostics != nullptr)
    {
        *diagnostics = {};
    }

    if (!analysis.valid
        || analysis.sectionBoundary.size() < 3
        || analysis.unwrappedBoundary.size() < 2
        || analysis.sectionPerimeter <= kCalculationEpsilon)
    {
        return RotaryBoundarySide::Ambiguous;
    }

    double queryPosition = 0.0;
    double surfaceDeviation = 0.0;

    if (!mapToSection
    (
        QVector2D(point.y(), point.z()),
        analysis.sectionBoundary,
        queryPosition,
        surfaceDeviation
    ))
    {
        return RotaryBoundarySide::Ambiguous;
    }

    const double perimeter = analysis.sectionPerimeter;
    const double normalizedQueryPosition = std::fmod(queryPosition + perimeter, perimeter);

    if (diagnostics != nullptr)
    {
        diagnostics->validProjection = true;
        diagnostics->mappedPerimeterPosition = normalizedQueryPosition;
        diagnostics->distanceToPerimeterSeam = perimeter;

        for (const double seam : analysis.seamPositions)
        {
            const double difference = std::abs(normalizedQueryPosition - seam);
            diagnostics->distanceToPerimeterSeam = std::min
            (
                diagnostics->distanceToPerimeterSeam,
                std::min(difference, perimeter - difference)
            );
        }
    }

    double minimumPosition = analysis.unwrappedBoundary.front().perimeterPosition;
    double maximumPosition = minimumPosition;

    for (const RotaryCutBoundaryUnwrappedSample& sample : analysis.unwrappedBoundary)
    {
        minimumPosition = std::min(minimumPosition, sample.perimeterPosition);
        maximumPosition = std::max(maximumPosition, sample.perimeterPosition);
    }

    const double intervalCenter = (minimumPosition + maximumPosition) * 0.5;

    while (queryPosition - intervalCenter > perimeter * 0.5) queryPosition -= perimeter;
    while (queryPosition - intervalCenter < -perimeter * 0.5) queryPosition += perimeter;

    const double queryX = point.x();
    const double safeTolerance = std::max(1.0e-6, tolerance);
    struct RayIntersection
    {
        double x = 0.0;
        int segmentIndex = -1;
    };
    std::vector<RayIntersection> rawIntersections;
    double minimumBoundaryDistance = std::numeric_limits<double>::max();
    bool onBoundary = false;

    for (const double shift : { -perimeter, 0.0, perimeter })
    {
        for (int index = 0; index + 1 < analysis.unwrappedBoundary.size(); ++index)
        {
            const RotaryCutBoundaryUnwrappedSample& first = analysis.unwrappedBoundary[index];
            const RotaryCutBoundaryUnwrappedSample& second = analysis.unwrappedBoundary[index + 1];
            const double firstS = first.perimeterPosition + shift;
            const double secondS = second.perimeterPosition + shift;
            const double edgeX = second.x - first.x;
            const double edgeS = secondS - firstS;
            const double edgeLengthSquared = edgeX * edgeX + edgeS * edgeS;

            if (edgeLengthSquared > kCalculationEpsilon)
            {
                const double factor = std::clamp
                (
                    ((queryX - first.x) * edgeX + (queryPosition - firstS) * edgeS)
                        / edgeLengthSquared,
                    0.0,
                    1.0
                );
                const double nearestX = first.x + edgeX * factor;
                const double nearestS = firstS + edgeS * factor;
                const double boundaryDistance = std::hypot
                    (queryX - nearestX, queryPosition - nearestS);
                minimumBoundaryDistance = std::min(minimumBoundaryDistance, boundaryDistance);
                onBoundary = onBoundary || boundaryDistance <= safeTolerance;
            }

            const bool crosses = (firstS <= queryPosition && queryPosition < secondS)
                || (secondS <= queryPosition && queryPosition < firstS);

            if (!crosses || std::abs(edgeS) <= kCalculationEpsilon)
            {
                continue;
            }

            const double factor = (queryPosition - firstS) / edgeS;
            const double intersectionX = first.x + edgeX * factor;
            rawIntersections.push_back({ intersectionX, index });
            onBoundary = onBoundary || std::abs(intersectionX - queryX) <= safeTolerance;
        }
    }

    std::sort(rawIntersections.begin(), rawIntersections.end(), [](const auto& left, const auto& right)
    {
        return left.x < right.x;
    });
    std::vector<RayIntersection> uniqueIntersections;

    for (const RayIntersection& intersection : rawIntersections)
    {
        if (!uniqueIntersections.empty()
            && std::abs(intersection.x - uniqueIntersections.back().x) <= safeTolerance)
        {
            uniqueIntersections.back().x = (uniqueIntersections.back().x + intersection.x) * 0.5;
        }
        else
        {
            uniqueIntersections.push_back(intersection);
        }
    }

    if (diagnostics != nullptr)
    {
        diagnostics->minimumBoundaryDistance = std::isfinite(minimumBoundaryDistance)
            ? minimumBoundaryDistance
            : 0.0;

        for (const RayIntersection& intersection : rawIntersections)
        {
            diagnostics->rawRayIntersectionXs.push_back(intersection.x);
        }

        for (const RayIntersection& intersection : uniqueIntersections)
        {
            diagnostics->deduplicatedRayIntersectionXs.push_back(intersection.x);
        }
    }

    if (onBoundary)
    {
        return RotaryBoundarySide::OnBoundary;
    }

    const int crossings = static_cast<int>(std::count_if
    (
        uniqueIntersections.cbegin(),
        uniqueIntersections.cend(),
        [queryX, safeTolerance](const RayIntersection& intersection)
        {
            return intersection.x < queryX - safeTolerance;
        }
    ));
    return crossings % 2 == 0 ? RotaryBoundarySide::Before : RotaryBoundarySide::After;
}

QVector<QVector3D> RotaryCutBoundaryAnalyzer::buildBoundaryOrderTestPoints
(
    const RotaryCutBoundaryAnalysis& analysis,
    double tolerance
)
{
    QVector<QVector3D> testPoints;

    if (!analysis.valid || analysis.orderedPath.size() < 2)
    {
        return testPoints;
    }

    const double duplicateTolerance = std::max
        (kDuplicatePointTolerance, std::abs(tolerance) * 1.0e-6);
    QVector<QVector3D> path;

    for (const QVector3D& point : analysis.orderedPath)
    {
        if (path.isEmpty() || distance3D(path.back(), point) > duplicateTolerance)
        {
            path.push_back(point);
        }
    }

    if (path.size() > 1 && distance3D(path.front(), path.back()) <= duplicateTolerance)
    {
        path.removeLast();
    }

    for (int index = 0; index < path.size(); ++index)
    {
        const QVector3D& start = path[index];
        const QVector3D& end = path[(index + 1) % path.size()];

        if (distance3D(start, end) > duplicateTolerance)
        {
            testPoints.push_back((start + end) * 0.5f);
        }
    }

    return testPoints;
}

bool RotaryCutBoundaryAnalyzer::boundariesIntersect
(
    const RotaryCutBoundaryAnalysis& left,
    const RotaryCutBoundaryAnalysis& right,
    double tolerance
)
{
    if (!left.valid || !right.valid
        || left.unwrappedBoundary.size() < 2 || right.unwrappedBoundary.size() < 2
        || left.sectionPerimeter <= kCalculationEpsilon
        || std::abs(left.sectionPerimeter - right.sectionPerimeter) > std::max(tolerance, 1.0e-6))
    {
        return true;
    }

    struct Point { double x = 0.0; double s = 0.0; };
    const double safeTolerance = std::max(tolerance, 1.0e-6);
    const auto cross = [](const Point& origin, const Point& first, const Point& second)
    {
        return (first.x - origin.x) * (second.s - origin.s)
            - (first.s - origin.s) * (second.x - origin.x);
    };
    const auto pointOnSegment = [safeTolerance, &cross]
    (const Point& point, const Point& start, const Point& end)
    {
        const double length = std::hypot(end.x - start.x, end.s - start.s);

        if (std::abs(cross(start, end, point)) > safeTolerance * std::max(1.0, length))
        {
            return false;
        }

        return point.x >= std::min(start.x, end.x) - safeTolerance
            && point.x <= std::max(start.x, end.x) + safeTolerance
            && point.s >= std::min(start.s, end.s) - safeTolerance
            && point.s <= std::max(start.s, end.s) + safeTolerance;
    };
    const auto segmentsIntersect = [&cross, &pointOnSegment]
    (const Point& a, const Point& b, const Point& c, const Point& d)
    {
        const double abC = cross(a, b, c);
        const double abD = cross(a, b, d);
        const double cdA = cross(c, d, a);
        const double cdB = cross(c, d, b);

        if (((abC < 0.0 && abD > 0.0) || (abC > 0.0 && abD < 0.0))
            && ((cdA < 0.0 && cdB > 0.0) || (cdA > 0.0 && cdB < 0.0)))
        {
            return true;
        }

        return pointOnSegment(c, a, b) || pointOnSegment(d, a, b)
            || pointOnSegment(a, c, d) || pointOnSegment(b, c, d);
    };
    const double perimeter = left.sectionPerimeter;

    for (const double shift : { -perimeter, 0.0, perimeter })
    {
        for (int leftIndex = 0; leftIndex + 1 < left.unwrappedBoundary.size(); ++leftIndex)
        {
            const Point leftStart
                { left.unwrappedBoundary[leftIndex].x, left.unwrappedBoundary[leftIndex].perimeterPosition };
            const Point leftEnd
                { left.unwrappedBoundary[leftIndex + 1].x, left.unwrappedBoundary[leftIndex + 1].perimeterPosition };

            for (int rightIndex = 0; rightIndex + 1 < right.unwrappedBoundary.size(); ++rightIndex)
            {
                const Point rightStart
                {
                    right.unwrappedBoundary[rightIndex].x,
                    right.unwrappedBoundary[rightIndex].perimeterPosition + shift
                };
                const Point rightEnd
                {
                    right.unwrappedBoundary[rightIndex + 1].x,
                    right.unwrappedBoundary[rightIndex + 1].perimeterPosition + shift
                };

                if (segmentsIntersect(leftStart, leftEnd, rightStart, rightEnd))
                {
                    return true;
                }
            }
        }
    }

    return false;
}
