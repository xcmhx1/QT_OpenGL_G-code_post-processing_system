#include "pch.h"

#include "RotaryCutBoundaryAnalyzer.h"

#include "CadItem.h"
#include "RotaryPathTopology.h"

#include <QDebug>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{
    constexpr double kEpsilon = 1.0e-9;
    constexpr double kSamplingDuplicateTolerance = 1.0e-6;
    constexpr double kProfilePhaseTolerance = 1.0e-9;
    constexpr double kProfileXTolerance = 1.0e-6;

    struct SectionPoint
    {
        double y = 0.0;
        double z = 0.0;
    };

    struct PerimeterTraversalMetrics
    {
        double signedTravel = 0.0;
        double absoluteTravel = 0.0;
        double windingNumber = 0.0;
        double coverage = 0.0;
        double travelRatio = 0.0;
        double backtrackRatio = 0.0;
        QVector<double> unwrappedPositions;
    };

    struct SurfaceProjectionCandidate
    {
        int hullEdgeIndex = -1;
        double perimeterPosition = 0.0;
        double deviation = 0.0;
    };

    double distance3D(const QVector3D& left, const QVector3D& right)
    {
        return static_cast<double>((left - right).length());
    }

    RotaryCutPlaneFit fitRotaryCutPlane(const QVector<QVector3D>& points, double planeTolerance)
    {
        RotaryCutPlaneFit fit;

        if (points.size() < 3)
        {
            return fit;
        }

        double matrix[3][4] = {};

        for (const QVector3D& point : points)
        {
            const double y = point.y();
            const double z = point.z();
            const double x = point.x();
            matrix[0][0] += y * y;
            matrix[0][1] += y * z;
            matrix[0][2] += y;
            matrix[0][3] += y * x;
            matrix[1][0] += y * z;
            matrix[1][1] += z * z;
            matrix[1][2] += z;
            matrix[1][3] += z * x;
            matrix[2][0] += y;
            matrix[2][1] += z;
            matrix[2][2] += 1.0;
            matrix[2][3] += x;
        }

        for (int column = 0; column < 3; ++column)
        {
            int pivot = column;

            for (int row = column + 1; row < 3; ++row)
            {
                if (std::abs(matrix[row][column]) > std::abs(matrix[pivot][column]))
                {
                    pivot = row;
                }
            }

            if (std::abs(matrix[pivot][column]) <= kEpsilon)
            {
                return fit;
            }

            for (int entry = column; entry < 4; ++entry)
            {
                std::swap(matrix[column][entry], matrix[pivot][entry]);
            }

            const double divisor = matrix[column][column];

            for (int entry = column; entry < 4; ++entry)
            {
                matrix[column][entry] /= divisor;
            }

            for (int row = 0; row < 3; ++row)
            {
                if (row == column) continue;
                const double factor = matrix[row][column];

                for (int entry = column; entry < 4; ++entry)
                {
                    matrix[row][entry] -= factor * matrix[column][entry];
                }
            }
        }

        fit.a = matrix[0][3];
        fit.b = matrix[1][3];
        fit.c = matrix[2][3];
        double squaredDeviation = 0.0;

        for (const QVector3D& point : points)
        {
            const double deviation = std::abs
            (
                static_cast<double>(point.x())
                - (fit.a * point.y() + fit.b * point.z() + fit.c)
            ) / std::sqrt(1.0 + fit.a * fit.a + fit.b * fit.b);
            squaredDeviation += deviation * deviation;
            fit.maximumDeviation = std::max(fit.maximumDeviation, deviation);
        }

        fit.rmsDeviation = std::sqrt(squaredDeviation / points.size());
        fit.valid = fit.maximumDeviation <= planeTolerance;
        return fit;
    }

    double sectionDistance(const SectionPoint& left, const SectionPoint& right)
    {
        return std::hypot(left.y - right.y, left.z - right.z);
    }

    double sectionCross(const SectionPoint& origin, const SectionPoint& left, const SectionPoint& right)
    {
        return (left.y - origin.y) * (right.z - origin.z)
            - (left.z - origin.z) * (right.y - origin.y);
    }

    std::vector<SectionPoint> buildSectionHull(const QVector<RotaryPathTopologyRecord>& records)
    {
        std::vector<SectionPoint> points;

        for (const RotaryPathTopologyRecord& record : records)
        {
            for (const QVector3D& point : record.points)
            {
                points.push_back({ point.y(), point.z() });
            }
        }

        std::sort
        (
            points.begin(),
            points.end(),
            [](const SectionPoint& left, const SectionPoint& right)
            {
                if (std::abs(left.y - right.y) > kEpsilon)
                {
                    return left.y < right.y;
                }

                return left.z < right.z;
            }
        );

        points.erase
        (
            std::unique
            (
                points.begin(),
                points.end(),
                [](const SectionPoint& left, const SectionPoint& right)
                {
                    return sectionDistance(left, right) <= kEpsilon;
                }
            ),
            points.end()
        );

        if (points.size() < 3)
        {
            return {};
        }

        std::vector<SectionPoint> hull(points.size() * 2);
        size_t hullSize = 0;

        for (const SectionPoint& point : points)
        {
            while (hullSize >= 2 && sectionCross(hull[hullSize - 2], hull[hullSize - 1], point) <= kEpsilon)
            {
                --hullSize;
            }

            hull[hullSize++] = point;
        }

        const size_t lowerSize = hullSize;

        for (size_t index = points.size() - 1; index > 0; --index)
        {
            const SectionPoint& point = points[index - 1];

            while (hullSize > lowerSize && sectionCross(hull[hullSize - 2], hull[hullSize - 1], point) <= kEpsilon)
            {
                --hullSize;
            }

            hull[hullSize++] = point;
        }

        if (hullSize > 1)
        {
            --hullSize;
        }

        hull.resize(hullSize);
        return hull;
    }

    QVector<QVector3D> normalizeClosedSamplingPath
    (
        const QVector<QVector3D>& path,
        double duplicatePointTolerance
    )
    {
        QVector<QVector3D> normalized;
        normalized.reserve(path.size());

        for (const QVector3D& point : path)
        {
            if (normalized.isEmpty()
                || distance3D(normalized.back(), point) > duplicatePointTolerance)
            {
                normalized.push_back(point);
            }
        }

        if (normalized.size() > 1
            && distance3D(normalized.front(), normalized.back()) <= duplicatePointTolerance)
        {
            normalized.pop_back();
        }

        return normalized;
    }

    QVector<QVector3D> densifyClosedPath(const QVector<QVector3D>& path, double maximumSpacing)
    {
        QVector<QVector3D> dense;
        const QVector<QVector3D> normalized = normalizeClosedSamplingPath
        (
            path,
            kSamplingDuplicateTolerance
        );

        if (normalized.size() < 2 || maximumSpacing <= 0.0)
        {
            return dense;
        }

        dense.push_back(normalized.front());

        for (int index = 0; index < normalized.size(); ++index)
        {
            const QVector3D start = normalized[index];
            const QVector3D end = index + 1 < normalized.size()
                ? normalized[index + 1]
                : normalized.front();
            const double length = distance3D(start, end);

            if (length <= kSamplingDuplicateTolerance)
            {
                continue;
            }

            const int stepCount = std::max(1, static_cast<int>(std::ceil(length / maximumSpacing)));

            for (int step = 1; step <= stepCount; ++step)
            {
                dense.push_back(step == stepCount
                    ? end
                    : start + (end - start) * (static_cast<float>(step) / static_cast<float>(stepCount)));
            }
        }

        return dense;
    }

    PerimeterTraversalMetrics calculatePerimeterTraversal
    (
        const QVector<double>& wrappedPositions,
        double perimeter
    )
    {
        PerimeterTraversalMetrics metrics;

        if (wrappedPositions.isEmpty() || perimeter <= kEpsilon)
        {
            return metrics;
        }

        double previousPerimeterPosition = wrappedPositions.front();
        double minimumUnwrappedPosition = 0.0;
        double maximumUnwrappedPosition = 0.0;
        metrics.unwrappedPositions.reserve(wrappedPositions.size());
        metrics.unwrappedPositions.push_back(0.0);

        for (int index = 1; index < wrappedPositions.size(); ++index)
        {
            double delta = wrappedPositions[index] - previousPerimeterPosition;

            while (delta > perimeter * 0.5)
            {
                delta -= perimeter;
            }

            while (delta < -perimeter * 0.5)
            {
                delta += perimeter;
            }

            metrics.signedTravel += delta;
            metrics.absoluteTravel += std::abs(delta);
            minimumUnwrappedPosition = std::min(minimumUnwrappedPosition, metrics.signedTravel);
            maximumUnwrappedPosition = std::max(maximumUnwrappedPosition, metrics.signedTravel);
            previousPerimeterPosition = wrappedPositions[index];
            metrics.unwrappedPositions.push_back(metrics.signedTravel);
        }

        metrics.windingNumber = metrics.signedTravel / perimeter;
        metrics.travelRatio = metrics.absoluteTravel / perimeter;
        metrics.coverage = std::min
        (
            1.0,
            (maximumUnwrappedPosition - minimumUnwrappedPosition) / perimeter
        );
        metrics.backtrackRatio = std::max
        (
            0.0,
            (metrics.absoluteTravel - std::abs(metrics.signedTravel)) * 0.5 / perimeter
        );
        return metrics;
    }

    bool normalizeBoundaryProfile
    (
        QVector<RotaryCutBoundaryProfileSample>& profile,
        bool& singleValued,
        int& multiValuePhaseCount,
        double& maximumMultiValueXSpan
    )
    {
        for (RotaryCutBoundaryProfileSample& sample : profile)
        {
            if (sample.phase >= 1.0 - kProfilePhaseTolerance)
            {
                sample.phase = 0.0;
            }
        }

        std::sort
        (
            profile.begin(),
            profile.end(),
            [](const RotaryCutBoundaryProfileSample& left, const RotaryCutBoundaryProfileSample& right)
            {
                return left.phase < right.phase;
            }
        );

        QVector<RotaryCutBoundaryProfileSample> normalized;
        normalized.reserve(profile.size());

        for (const RotaryCutBoundaryProfileSample& sample : profile)
        {
            if (normalized.isEmpty()
                || std::abs(sample.phase - normalized.back().phase) > kProfilePhaseTolerance)
            {
                normalized.push_back(sample);
                continue;
            }

            if (std::abs(sample.x - normalized.back().x) > kProfileXTolerance)
            {
                singleValued = false;
                ++multiValuePhaseCount;
                maximumMultiValueXSpan = std::max
                (
                    maximumMultiValueXSpan,
                    std::abs(sample.x - normalized.back().x)
                );
                normalized.push_back(sample);
            }
        }

        profile = std::move(normalized);
        return profile.size() >= 2;
    }

    void diagnoseMultiValueBoundary(RotaryCutBoundaryAnalysis& analysis)
    {
        if (analysis.unwrappedBoundary.size() < 2 || analysis.sectionPerimeter <= kEpsilon)
        {
            analysis.singleValuedProfile = false;
            return;
        }

        constexpr int kDiagnosticSamples = 128;
        analysis.singleValuedProfile = true;
        analysis.multiValuePhaseCount = 0;
        analysis.maximumMultiValueXSpan = 0.0;

        for (int sampleIndex = 0; sampleIndex < kDiagnosticSamples; ++sampleIndex)
        {
            const double queryS = analysis.sectionPerimeter
                * static_cast<double>(sampleIndex) / static_cast<double>(kDiagnosticSamples);
            QVector<double> intersections;

            for (const double shift : { -analysis.sectionPerimeter, 0.0, analysis.sectionPerimeter })
            {
                for (int index = 0; index + 1 < analysis.unwrappedBoundary.size(); ++index)
                {
                    const RotaryCutBoundaryUnwrappedSample& first = analysis.unwrappedBoundary[index];
                    const RotaryCutBoundaryUnwrappedSample& second = analysis.unwrappedBoundary[index + 1];
                    const double firstS = first.perimeterPosition + shift;
                    const double secondS = second.perimeterPosition + shift;
                    const bool crosses = (firstS <= queryS && queryS < secondS)
                        || (secondS <= queryS && queryS < firstS);

                    if (!crosses || std::abs(secondS - firstS) <= kEpsilon)
                    {
                        continue;
                    }

                    const double factor = (queryS - firstS) / (secondS - firstS);
                    intersections.push_back(first.x + (second.x - first.x) * factor);
                }
            }

            std::sort(intersections.begin(), intersections.end());
            intersections.erase
            (
                std::unique(intersections.begin(), intersections.end(), [](double left, double right)
                {
                    return std::abs(left - right) <= kProfileXTolerance;
                }),
                intersections.end()
            );

            if (intersections.size() > 1)
            {
                analysis.singleValuedProfile = false;
                ++analysis.multiValuePhaseCount;
                analysis.maximumMultiValueXSpan = std::max
                (
                    analysis.maximumMultiValueXSpan,
                    intersections.back() - intersections.front()
                );
            }
        }
    }

    QVector<SurfaceProjectionCandidate> mapToHullPerimeterCandidates
    (
        const SectionPoint& point,
        const std::vector<SectionPoint>& hull,
        const std::vector<double>& cumulativeLengths
    )
    {
        if (hull.size() < 3 || cumulativeLengths.size() != hull.size() + 1)
        {
            return {};
        }

        QVector<SurfaceProjectionCandidate> allCandidates;
        double minimumDeviation = std::numeric_limits<double>::max();

        for (size_t index = 0; index < hull.size(); ++index)
        {
            const SectionPoint& start = hull[index];
            const SectionPoint& end = hull[(index + 1) % hull.size()];
            const double edgeY = end.y - start.y;
            const double edgeZ = end.z - start.z;
            const double edgeLengthSquared = edgeY * edgeY + edgeZ * edgeZ;

            if (edgeLengthSquared <= kEpsilon)
            {
                continue;
            }

            const double factor = std::clamp
            (
                ((point.y - start.y) * edgeY + (point.z - start.z) * edgeZ) / edgeLengthSquared,
                0.0,
                1.0
            );
            const SectionPoint projection{ start.y + edgeY * factor, start.z + edgeZ * factor };
            const double candidateDeviation = sectionDistance(point, projection);

            minimumDeviation = std::min(minimumDeviation, candidateDeviation);
            allCandidates.push_back
            ({
                static_cast<int>(index),
                cumulativeLengths[index] + std::sqrt(edgeLengthSquared) * factor,
                candidateDeviation
            });
        }

        const double perimeter = cumulativeLengths.back();
        const double ambiguityTolerance = std::max(1.0e-6, perimeter * 1.0e-8);
        QVector<SurfaceProjectionCandidate> candidates;

        for (const SurfaceProjectionCandidate& candidate : allCandidates)
        {
            if (candidate.deviation <= minimumDeviation + ambiguityTolerance)
            {
                candidates.push_back(candidate);
            }
        }

        return candidates;
    }

    bool mapToHullPerimeter
    (
        const SectionPoint& point,
        const std::vector<SectionPoint>& hull,
        const std::vector<double>& cumulativeLengths,
        double& perimeterPosition,
        double& deviation
    )
    {
        const QVector<SurfaceProjectionCandidate> candidates = mapToHullPerimeterCandidates
        (
            point,
            hull,
            cumulativeLengths
        );

        if (candidates.isEmpty())
        {
            return false;
        }

        const auto best = std::min_element
        (
            candidates.begin(),
            candidates.end(),
            [](const SurfaceProjectionCandidate& left, const SurfaceProjectionCandidate& right)
            {
                return left.deviation < right.deviation;
            }
        );
        perimeterPosition = best->perimeterPosition;
        deviation = best->deviation;
        return true;
    }

    bool selectContinuousSurfaceProjection
    (
        const QVector<QVector3D>& path,
        const std::vector<SectionPoint>& hull,
        const std::vector<double>& cumulativeLengths,
        QVector<double>& wrappedPositions,
        QVector<double>& deviations,
        int& ambiguousPointCount,
        double& maximumPerimeterJump
    )
    {
        struct State
        {
            double cost = std::numeric_limits<double>::max();
            double unwrappedPosition = 0.0;
            int previous = -1;
        };

        const double perimeter = cumulativeLengths.back();
        QVector<QVector<SurfaceProjectionCandidate>> candidatesByPoint;
        candidatesByPoint.reserve(path.size());

        for (const QVector3D& point : path)
        {
            QVector<SurfaceProjectionCandidate> candidates = mapToHullPerimeterCandidates
            (
                { point.y(), point.z() },
                hull,
                cumulativeLengths
            );

            if (candidates.isEmpty())
            {
                return false;
            }

            ambiguousPointCount += candidates.size() > 1 ? 1 : 0;
            candidatesByPoint.push_back(std::move(candidates));
        }

        QVector<QVector<State>> states(path.size());
        states[0].resize(candidatesByPoint[0].size());

        for (int candidateIndex = 0; candidateIndex < candidatesByPoint[0].size(); ++candidateIndex)
        {
            states[0][candidateIndex].cost = candidatesByPoint[0][candidateIndex].deviation
                * candidatesByPoint[0][candidateIndex].deviation;
            states[0][candidateIndex].unwrappedPosition = candidatesByPoint[0][candidateIndex].perimeterPosition;
        }

        for (int pointIndex = 1; pointIndex < path.size(); ++pointIndex)
        {
            states[pointIndex].resize(candidatesByPoint[pointIndex].size());
            const double spatialStep = distance3D(path[pointIndex - 1], path[pointIndex]);

            for (int currentIndex = 0; currentIndex < candidatesByPoint[pointIndex].size(); ++currentIndex)
            {
                const SurfaceProjectionCandidate& current = candidatesByPoint[pointIndex][currentIndex];

                for (int previousIndex = 0; previousIndex < candidatesByPoint[pointIndex - 1].size(); ++previousIndex)
                {
                    const State& previousState = states[pointIndex - 1][previousIndex];
                    const double previousWrapped = candidatesByPoint[pointIndex - 1][previousIndex].perimeterPosition;
                    double delta = current.perimeterPosition - previousWrapped;

                    while (delta > perimeter * 0.5) delta -= perimeter;
                    while (delta < -perimeter * 0.5) delta += perimeter;

                    const double abnormalJump = std::max(0.0, std::abs(delta) - std::max(spatialStep * 4.0, perimeter * 0.1));
                    const double transitionCost = std::abs(std::abs(delta) - spatialStep) * 0.05
                        + abnormalJump * abnormalJump * 100.0
                        + current.deviation * current.deviation * 4.0;
                    const double cost = previousState.cost + transitionCost;

                    if (cost < states[pointIndex][currentIndex].cost)
                    {
                        states[pointIndex][currentIndex].cost = cost;
                        states[pointIndex][currentIndex].previous = previousIndex;
                        states[pointIndex][currentIndex].unwrappedPosition = previousState.unwrappedPosition + delta;
                    }
                }
            }
        }

        int selectedIndex = 0;

        for (int index = 1; index < states.back().size(); ++index)
        {
            if (states.back()[index].cost < states.back()[selectedIndex].cost)
            {
                selectedIndex = index;
            }
        }

        wrappedPositions.resize(path.size());
        deviations.resize(path.size());

        for (int pointIndex = path.size() - 1; pointIndex >= 0; --pointIndex)
        {
            const SurfaceProjectionCandidate& selected = candidatesByPoint[pointIndex][selectedIndex];
            wrappedPositions[pointIndex] = selected.perimeterPosition;
            deviations[pointIndex] = selected.deviation;

            if (pointIndex > 0)
            {
                const double jump = std::abs
                (
                    states[pointIndex][selectedIndex].unwrappedPosition
                    - states[pointIndex - 1][states[pointIndex][selectedIndex].previous].unwrappedPosition
                );
                maximumPerimeterJump = std::max(maximumPerimeterJump, jump);
                selectedIndex = states[pointIndex][selectedIndex].previous;
            }
        }

        return true;
    }

    double crossUnwrapped(const QVector2D& origin, const QVector2D& left, const QVector2D& right)
    {
        return static_cast<double>(left.x() - origin.x()) * static_cast<double>(right.y() - origin.y())
            - static_cast<double>(left.y() - origin.y()) * static_cast<double>(right.x() - origin.x());
    }

    bool hasProperSelfIntersection(const QVector<QVector2D>& path)
    {
        if (path.size() < 4)
        {
            return false;
        }

        for (int firstIndex = 0; firstIndex + 1 < path.size(); ++firstIndex)
        {
            for (int secondIndex = firstIndex + 2; secondIndex + 1 < path.size(); ++secondIndex)
            {
                const double firstSideStart = crossUnwrapped(path[firstIndex], path[firstIndex + 1], path[secondIndex]);
                const double firstSideEnd = crossUnwrapped(path[firstIndex], path[firstIndex + 1], path[secondIndex + 1]);
                const double secondSideStart = crossUnwrapped(path[secondIndex], path[secondIndex + 1], path[firstIndex]);
                const double secondSideEnd = crossUnwrapped(path[secondIndex], path[secondIndex + 1], path[firstIndex + 1]);

                if (firstSideStart * firstSideEnd < -kEpsilon
                    && secondSideStart * secondSideEnd < -kEpsilon)
                {
                    return true;
                }
            }
        }

        return false;
    }

    bool hasPeriodicSelfIntersection(const QVector<QVector2D>& path, double perimeter)
    {
        if (path.size() < 2 || perimeter <= kEpsilon)
        {
            return false;
        }

        for (const double shift : { -perimeter, perimeter })
        {
            for (int firstIndex = 0; firstIndex + 1 < path.size(); ++firstIndex)
            {
                for (int secondIndex = 0; secondIndex + 1 < path.size(); ++secondIndex)
                {
                    const QVector2D shiftedStart
                    (
                        path[secondIndex].x(),
                        path[secondIndex].y() + static_cast<float>(shift)
                    );
                    const QVector2D shiftedEnd
                    (
                        path[secondIndex + 1].x(),
                        path[secondIndex + 1].y() + static_cast<float>(shift)
                    );
                    const double firstSideStart = crossUnwrapped(path[firstIndex], path[firstIndex + 1], shiftedStart);
                    const double firstSideEnd = crossUnwrapped(path[firstIndex], path[firstIndex + 1], shiftedEnd);
                    const double secondSideStart = crossUnwrapped(shiftedStart, shiftedEnd, path[firstIndex]);
                    const double secondSideEnd = crossUnwrapped(shiftedStart, shiftedEnd, path[firstIndex + 1]);

                    if (firstSideStart * firstSideEnd < -kEpsilon
                        && secondSideStart * secondSideEnd < -kEpsilon)
                    {
                        return true;
                    }
                }
            }
        }

        return false;
    }
}

RotaryCutBoundaryAnalysis RotaryCutBoundaryAnalyzer::analyze
(
    const QVector<CadItem*>& candidateItems,
    const QVector<CadItem*>& sceneItems,
    double connectionTolerance,
    const QVector<QVector2D>& configuredSectionHull
)
{
    RotaryCutBoundaryAnalysis analysis;
    qInfo().noquote() << QStringLiteral("[断面候选] 周向分析输入：%1")
        .arg(describeRotaryPathItems(candidateItems));
    QVector<CadItem*> topologyItems = sceneItems;

    for (CadItem* item : candidateItems)
    {
        if (item != nullptr && !topologyItems.contains(item))
        {
            topologyItems.push_back(item);
        }
    }

    const RotaryPathTopology topology
    (
        topologyItems,
        RotaryPathTopologyTolerance::fromConnectionTolerance(connectionTolerance)
    );
    const RotaryPathLoopResult loop = topology.extractBestLoop(candidateItems, candidateItems);
    qInfo().noquote() << QStringLiteral("[断面候选] 最终闭环：%1")
        .arg(describeRotaryPathItems(loop.usedItems));
    analysis.connectedComponentCount = loop.connectedComponentCount;
    analysis.openNodeCount = loop.openNodeCount;
    analysis.branchNodeCount = loop.branchNodeCount;
    analysis.ignoredBranchItemCount = loop.ignoredBranchItemCount;
    analysis.approximatelyClosed = loop.approximatelyClosed;
    analysis.closureGap = loop.closureGap;
    analysis.boundaryItems = loop.usedItems;

    if (!loop.valid)
    {
        analysis.errorMessage = loop.errorMessage;
        return analysis;
    }

    analysis.orderedPath = loop.orderedPath;
    analysis.connectedLoop = true;
    analysis.planeFit = fitRotaryCutPlane
    (
        analysis.orderedPath,
        std::max(0.1, connectionTolerance * 0.25)
    );
    std::vector<SectionPoint> hull;

    if (configuredSectionHull.size() >= 3)
    {
        hull.reserve(static_cast<size_t>(configuredSectionHull.size()));

        for (const QVector2D& point : configuredSectionHull)
        {
            hull.push_back({ point.x(), point.y() });
        }
    }
    else
    {
        hull = buildSectionHull(topology.records());
    }

    if (hull.size() < 3)
    {
        analysis.errorMessage = QStringLiteral("无法从当前图纸提取有效的方管 YZ 截面外轮廓。");
        return analysis;
    }

    std::vector<double> cumulativeLengths(hull.size() + 1, 0.0);

    for (size_t index = 0; index < hull.size(); ++index)
    {
        cumulativeLengths[index + 1] = cumulativeLengths[index]
            + sectionDistance(hull[index], hull[(index + 1) % hull.size()]);
    }

    const double perimeter = cumulativeLengths.back();

    if (perimeter <= kEpsilon)
    {
        analysis.errorMessage = QStringLiteral("方管 YZ 截面外轮廓周长无效。");
        return analysis;
    }

    double minY = hull.front().y;
    double maxY = hull.front().y;
    double minZ = hull.front().z;
    double maxZ = hull.front().z;

    for (const SectionPoint& point : hull)
    {
        minY = std::min(minY, point.y);
        maxY = std::max(maxY, point.y);
        minZ = std::min(minZ, point.z);
        maxZ = std::max(maxZ, point.z);
    }

    const double sectionDiagonal = std::hypot(maxY - minY, maxZ - minZ);
    const double surfaceTolerance = std::max(0.1, sectionDiagonal * 0.002);
    const double sampleSpacing = std::max(0.25, sectionDiagonal / 240.0);
    const QVector<QVector3D> densePath = densifyClosedPath(analysis.orderedPath, sampleSpacing);

    if (densePath.size() < 4)
    {
        analysis.errorMessage = QStringLiteral("加工断面路径采样点不足，无法进行拓扑验证。");
        return analysis;
    }

    QVector<double> wrappedPerimeterPositions;
    QVector<double> projectionDeviations;
    analysis.sectionPerimeter = perimeter;

    analysis.sectionHull.reserve(static_cast<qsizetype>(hull.size()));

    for (const SectionPoint& point : hull)
    {
        analysis.sectionHull.push_back(QVector2D(static_cast<float>(point.y), static_cast<float>(point.z)));
    }

    if (!selectContinuousSurfaceProjection
    (
        densePath,
        hull,
        cumulativeLengths,
        wrappedPerimeterPositions,
        projectionDeviations,
        analysis.ambiguousProjectionPointCount,
        analysis.maximumPerimeterJump
    ))
    {
        analysis.errorMessage = QStringLiteral("加工断面路径无法连续映射到方管截面周长。");
        return analysis;
    }

    analysis.initialPerimeterPosition = wrappedPerimeterPositions.front();

    for (int pointIndex = 0; pointIndex < densePath.size(); ++pointIndex)
    {
        const QVector3D& point = densePath[pointIndex];
        const double perimeterPosition = wrappedPerimeterPositions[pointIndex];
        const double deviation = projectionDeviations[pointIndex];

        analysis.maximumSurfaceDeviation = std::max(analysis.maximumSurfaceDeviation, deviation);
        analysis.boundaryProfile.push_back
        ({
            std::clamp(perimeterPosition / perimeter, 0.0, 1.0),
            static_cast<double>(point.x())
        });

        if (deviation > surfaceTolerance)
        {
            analysis.errorMessage = QStringLiteral("候选路径没有贴合方管外表面，最大偏差 %1 mm，允许偏差 %2 mm。")
                .arg(analysis.maximumSurfaceDeviation, 0, 'f', 3)
                .arg(surfaceTolerance, 0, 'f', 3);
            return analysis;
        }
    }

    const PerimeterTraversalMetrics traversal = calculatePerimeterTraversal
    (
        wrappedPerimeterPositions,
        perimeter
    );
    QVector<QVector2D> unwrappedPath;
    unwrappedPath.reserve(densePath.size());

    for (int index = 0; index < densePath.size(); ++index)
    {
        unwrappedPath.push_back(QVector2D
        (
            densePath[index].x(),
            static_cast<float>(traversal.unwrappedPositions[index])
        ));
        analysis.unwrappedBoundary.push_back
        ({
            static_cast<double>(densePath[index].x()),
            traversal.unwrappedPositions[index]
        });
    }

    analysis.surfaceConforming = true;
    analysis.signedPerimeterTravel = traversal.signedTravel;
    analysis.windingNumber = traversal.windingNumber;
    analysis.perimeterTravelRatio = traversal.travelRatio;
    analysis.perimeterCoverage = traversal.coverage;
    analysis.backtrackRatio = traversal.backtrackRatio;
    const double absoluteWinding = std::abs(analysis.windingNumber);

    if (std::abs(absoluteWinding - 1.0) > 0.12 || analysis.perimeterCoverage < 0.90)
    {
        analysis.errorMessage = QStringLiteral("候选路径未完成一次完整周向绕行：绕行数 %1，覆盖率 %2%，累计行程 %3%，回退 %4%。")
            .arg(analysis.windingNumber, 0, 'f', 3)
            .arg(analysis.perimeterCoverage * 100.0, 0, 'f', 1)
            .arg(analysis.perimeterTravelRatio * 100.0, 0, 'f', 1)
            .arg(analysis.backtrackRatio * 100.0, 0, 'f', 1);
        return analysis;
    }

    if (hasProperSelfIntersection(unwrappedPath))
    {
        analysis.errorMessage = QStringLiteral("候选路径在方管展开面上存在自相交，不能作为唯一分离边界。");
        return analysis;
    }

    if (hasPeriodicSelfIntersection(unwrappedPath, perimeter))
    {
        analysis.errorMessage = QStringLiteral("候选路径与其周期副本在方管展开面上相交，不能形成稳定分离边界。");
        return analysis;
    }

    analysis.separating = true;
    diagnoseMultiValueBoundary(analysis);

    if (!normalizeBoundaryProfile
    (
        analysis.boundaryProfile,
        analysis.singleValuedProfile,
        analysis.multiValuePhaseCount,
        analysis.maximumMultiValueXSpan
    ))
    {
        if (analysis.errorMessage.isEmpty())
        {
            analysis.errorMessage = QStringLiteral("加工断面边界采样不足，无法进行周期插值。");
        }

        analysis.separating = false;
        return analysis;
    }

    analysis.valid = true;
    analysis.errorMessage.clear();
    qInfo().noquote() << QStringLiteral
    (
        "[加工断面映射] 歧义采样点=%1，最大周向跳变=%2 mm，净绕行=%3，"
        "累计行程=%4%，覆盖率=%5%，回退=%6%，真实多值相位=%7，最大同相位X跨度=%8 mm。"
    )
        .arg(analysis.ambiguousProjectionPointCount)
        .arg(analysis.maximumPerimeterJump, 0, 'f', 3)
        .arg(analysis.windingNumber, 0, 'f', 3)
        .arg(analysis.perimeterTravelRatio * 100.0, 0, 'f', 1)
        .arg(analysis.perimeterCoverage * 100.0, 0, 'f', 1)
        .arg(analysis.backtrackRatio * 100.0, 0, 'f', 1)
        .arg(analysis.multiValuePhaseCount)
        .arg(analysis.maximumMultiValueXSpan, 0, 'f', 3);
    return analysis;
}

bool RotaryCutBoundaryAnalyzer::boundaryXAtPoint
(
    const RotaryCutBoundaryAnalysis& analysis,
    const QVector3D& point,
    double& boundaryX
)
{
    if (!analysis.valid
        || !analysis.singleValuedProfile
        || analysis.sectionHull.size() < 3
        || analysis.boundaryProfile.size() < 2)
    {
        return false;
    }

    std::vector<SectionPoint> hull;
    hull.reserve(static_cast<size_t>(analysis.sectionHull.size()));

    for (const QVector2D& hullPoint : analysis.sectionHull)
    {
        hull.push_back({ hullPoint.x(), hullPoint.y() });
    }

    std::vector<double> cumulativeLengths(hull.size() + 1, 0.0);

    for (size_t index = 0; index < hull.size(); ++index)
    {
        cumulativeLengths[index + 1] = cumulativeLengths[index]
            + sectionDistance(hull[index], hull[(index + 1) % hull.size()]);
    }

    const double perimeter = cumulativeLengths.back();
    double perimeterPosition = 0.0;
    double deviation = 0.0;

    if (perimeter <= kEpsilon
        || !mapToHullPerimeter
        (
            { point.y(), point.z() },
            hull,
            cumulativeLengths,
            perimeterPosition,
            deviation
        ))
    {
        return false;
    }

    const double phase = std::clamp(perimeterPosition / perimeter, 0.0, 1.0);
    const auto upper = std::upper_bound
    (
        analysis.boundaryProfile.begin(),
        analysis.boundaryProfile.end(),
        phase,
        [](double value, const RotaryCutBoundaryProfileSample& sample)
        {
            return value < sample.phase;
        }
    );

    RotaryCutBoundaryProfileSample left;
    RotaryCutBoundaryProfileSample right;

    if (upper == analysis.boundaryProfile.begin())
    {
        left = analysis.boundaryProfile.back();
        left.phase -= 1.0;
        right = analysis.boundaryProfile.front();
    }
    else if (upper == analysis.boundaryProfile.end())
    {
        left = analysis.boundaryProfile.back();
        right = analysis.boundaryProfile.front();
        right.phase += 1.0;
    }
    else
    {
        left = *(upper - 1);
        right = *upper;
    }

    double adjustedPhase = phase;

    if (adjustedPhase < left.phase)
    {
        adjustedPhase += 1.0;
    }

    const double phaseSpan = right.phase - left.phase;
    const double factor = phaseSpan > kEpsilon
        ? std::clamp((adjustedPhase - left.phase) / phaseSpan, 0.0, 1.0)
        : 0.0;
    boundaryX = left.x + (right.x - left.x) * factor;
    return std::isfinite(boundaryX);
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
        || analysis.sectionHull.size() < 3
        || analysis.unwrappedBoundary.size() < 2
        || analysis.sectionPerimeter <= kEpsilon)
    {
        return RotaryBoundarySide::Ambiguous;
    }

    std::vector<SectionPoint> hull;
    hull.reserve(static_cast<size_t>(analysis.sectionHull.size()));

    for (const QVector2D& hullPoint : analysis.sectionHull)
    {
        hull.push_back({ hullPoint.x(), hullPoint.y() });
    }

    std::vector<double> cumulativeLengths(hull.size() + 1, 0.0);

    for (size_t index = 0; index < hull.size(); ++index)
    {
        cumulativeLengths[index + 1] = cumulativeLengths[index]
            + sectionDistance(hull[index], hull[(index + 1) % hull.size()]);
    }

    double wrappedPosition = 0.0;
    double deviation = 0.0;

    if (!mapToHullPerimeter
    (
        { point.y(), point.z() },
        hull,
        cumulativeLengths,
        wrappedPosition,
        deviation
    ))
    {
        return RotaryBoundarySide::Ambiguous;
    }

    const double perimeter = analysis.sectionPerimeter;
    double queryPosition = wrappedPosition - analysis.initialPerimeterPosition;
    double normalizedQueryPosition = std::fmod(queryPosition, perimeter);

    if (normalizedQueryPosition < 0.0)
    {
        normalizedQueryPosition += perimeter;
    }

    if (diagnostics != nullptr)
    {
        diagnostics->validProjection = true;
        diagnostics->mappedPerimeterPosition = normalizedQueryPosition;
        diagnostics->distanceToPerimeterSeam = std::min
        (
            normalizedQueryPosition,
            perimeter - normalizedQueryPosition
        );
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
        int periodicCopy = 0;
    };

    std::vector<RayIntersection> rawIntersections;
    double minimumBoundaryDistance = std::numeric_limits<double>::max();
    bool onBoundary = false;
    int periodicCopy = -1;

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

            if (edgeLengthSquared > kEpsilon)
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
                const double boundaryDistance = std::hypot(queryX - nearestX, queryPosition - nearestS);
                minimumBoundaryDistance = std::min(minimumBoundaryDistance, boundaryDistance);

                if (boundaryDistance <= safeTolerance)
                {
                    onBoundary = true;
                }
            }

            const bool crosses = (firstS <= queryPosition && queryPosition < secondS)
                || (secondS <= queryPosition && queryPosition < firstS);

            if (!crosses || std::abs(edgeS) <= kEpsilon)
            {
                continue;
            }

            const double factor = (queryPosition - firstS) / edgeS;
            const double intersectionX = first.x + edgeX * factor;
            rawIntersections.push_back({ intersectionX, index, periodicCopy });

            if (std::abs(intersectionX - queryX) <= safeTolerance)
            {
                onBoundary = true;
            }
        }

        ++periodicCopy;
    }

    std::sort
    (
        rawIntersections.begin(),
        rawIntersections.end(),
        [](const RayIntersection& left, const RayIntersection& right)
        {
            if (left.x != right.x)
            {
                return left.x < right.x;
            }

            if (left.segmentIndex != right.segmentIndex)
            {
                return left.segmentIndex < right.segmentIndex;
            }

            return left.periodicCopy < right.periodicCopy;
        }
    );

    const int segmentCount = analysis.unwrappedBoundary.size() - 1;
    std::vector<RayIntersection> uniqueIntersections;
    uniqueIntersections.reserve(rawIntersections.size());

    for (const RayIntersection& intersection : rawIntersections)
    {
        bool merged = false;

        for (auto existing = uniqueIntersections.rbegin(); existing != uniqueIntersections.rend(); ++existing)
        {
            if (intersection.x - existing->x > safeTolerance)
            {
                break;
            }

            const int segmentDistance = std::abs(intersection.segmentIndex - existing->segmentIndex);
            const bool sameSourceSegment = intersection.segmentIndex == existing->segmentIndex;
            const bool adjacentSourceSegments = segmentDistance == 1
                || (segmentCount > 1 && segmentDistance == segmentCount - 1);

            if (std::abs(intersection.x - existing->x) <= safeTolerance
                && (sameSourceSegment || adjacentSourceSegments))
            {
                existing->x = (existing->x + intersection.x) * 0.5;
                merged = true;
                break;
            }
        }

        if (!merged)
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

    return (crossings % 2) == 0
        ? RotaryBoundarySide::Before
        : RotaryBoundarySide::After;
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
    (
        kSamplingDuplicateTolerance,
        std::abs(tolerance) * 1.0e-6
    );
    QVector<QVector3D> path;
    path.reserve(analysis.orderedPath.size());

    for (const QVector3D& point : analysis.orderedPath)
    {
        if (path.isEmpty() || distance3D(path.back(), point) > duplicateTolerance)
        {
            path.push_back(point);
        }
    }

    const bool repeatedClosingPoint = path.size() > 1
        && distance3D(path.front(), path.back()) <= duplicateTolerance;

    if (repeatedClosingPoint)
    {
        path.removeLast();
    }

    if (path.size() < 2)
    {
        return testPoints;
    }

    testPoints.reserve(path.size());

    for (int index = 0; index + 1 < path.size(); ++index)
    {
        if (distance3D(path[index], path[index + 1]) <= duplicateTolerance)
        {
            continue;
        }

        testPoints.push_back((path[index] + path[index + 1]) * 0.5f);
    }

    if ((repeatedClosingPoint || analysis.closureGap <= duplicateTolerance)
        && distance3D(path.back(), path.front()) > duplicateTolerance)
    {
        testPoints.push_back((path.back() + path.front()) * 0.5f);
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
    if (!left.valid
        || !right.valid
        || left.unwrappedBoundary.size() < 2
        || right.unwrappedBoundary.size() < 2
        || left.sectionPerimeter <= kEpsilon
        || std::abs(left.sectionPerimeter - right.sectionPerimeter) > std::max(tolerance, 1.0e-6))
    {
        return true;
    }

    struct Point
    {
        double x = 0.0;
        double s = 0.0;
    };

    const double safeTolerance = std::max(tolerance, 1.0e-6);
    const auto cross = [](const Point& origin, const Point& first, const Point& second)
    {
        return (first.x - origin.x) * (second.s - origin.s)
            - (first.s - origin.s) * (second.x - origin.x);
    };
    const auto pointOnSegment = [safeTolerance, &cross]
    (
        const Point& point,
        const Point& start,
        const Point& end
    )
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
    (
        const Point& firstStart,
        const Point& firstEnd,
        const Point& secondStart,
        const Point& secondEnd
    )
    {
        const double firstSideStart = cross(firstStart, firstEnd, secondStart);
        const double firstSideEnd = cross(firstStart, firstEnd, secondEnd);
        const double secondSideStart = cross(secondStart, secondEnd, firstStart);
        const double secondSideEnd = cross(secondStart, secondEnd, firstEnd);

        if ((firstSideStart < 0.0 && firstSideEnd > 0.0
                || firstSideStart > 0.0 && firstSideEnd < 0.0)
            && (secondSideStart < 0.0 && secondSideEnd > 0.0
                || secondSideStart > 0.0 && secondSideEnd < 0.0))
        {
            return true;
        }

        return pointOnSegment(secondStart, firstStart, firstEnd)
            || pointOnSegment(secondEnd, firstStart, firstEnd)
            || pointOnSegment(firstStart, secondStart, secondEnd)
            || pointOnSegment(firstEnd, secondStart, secondEnd);
    };

    const double perimeter = left.sectionPerimeter;

    for (const double shift : { -perimeter, 0.0, perimeter })
    {
        for (int leftIndex = 0; leftIndex + 1 < left.unwrappedBoundary.size(); ++leftIndex)
        {
            const Point leftStart
            {
                left.unwrappedBoundary[leftIndex].x,
                left.unwrappedBoundary[leftIndex].perimeterPosition + left.initialPerimeterPosition
            };
            const Point leftEnd
            {
                left.unwrappedBoundary[leftIndex + 1].x,
                left.unwrappedBoundary[leftIndex + 1].perimeterPosition + left.initialPerimeterPosition
            };

            for (int rightIndex = 0; rightIndex + 1 < right.unwrappedBoundary.size(); ++rightIndex)
            {
                const Point rightStart
                {
                    right.unwrappedBoundary[rightIndex].x,
                    right.unwrappedBoundary[rightIndex].perimeterPosition
                        + right.initialPerimeterPosition + shift
                };
                const Point rightEnd
                {
                    right.unwrappedBoundary[rightIndex + 1].x,
                    right.unwrappedBoundary[rightIndex + 1].perimeterPosition
                        + right.initialPerimeterPosition + shift
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
