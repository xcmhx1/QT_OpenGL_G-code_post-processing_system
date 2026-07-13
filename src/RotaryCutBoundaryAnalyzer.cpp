#include "pch.h"

#include "RotaryCutBoundaryAnalyzer.h"

#include "CadItem.h"
#include "RotaryPathTopology.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{
    constexpr double kEpsilon = 1.0e-9;

    struct SectionPoint
    {
        double y = 0.0;
        double z = 0.0;
    };

    double distance3D(const QVector3D& left, const QVector3D& right)
    {
        return static_cast<double>((left - right).length());
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

    QVector<QVector3D> densifyClosedPath(const QVector<QVector3D>& path, double maximumSpacing)
    {
        QVector<QVector3D> dense;

        if (path.size() < 2 || maximumSpacing <= 0.0)
        {
            return dense;
        }

        dense.push_back(path.front());

        for (int index = 0; index < path.size(); ++index)
        {
            const QVector3D start = path[index];
            const QVector3D end = index + 1 < path.size() ? path[index + 1] : path.front();
            const double length = distance3D(start, end);
            const int stepCount = std::max(1, static_cast<int>(std::ceil(length / maximumSpacing)));

            for (int step = 1; step <= stepCount; ++step)
            {
                dense.push_back(start + (end - start) * (static_cast<float>(step) / static_cast<float>(stepCount)));
            }
        }

        return dense;
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
        if (hull.size() < 3 || cumulativeLengths.size() != hull.size() + 1)
        {
            return false;
        }

        deviation = std::numeric_limits<double>::max();
        perimeterPosition = 0.0;

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

            if (candidateDeviation < deviation)
            {
                deviation = candidateDeviation;
                perimeterPosition = cumulativeLengths[index] + std::sqrt(edgeLengthSquared) * factor;
            }
        }

        return std::isfinite(deviation);
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

    double previousPosition = 0.0;
    double unwrappedPosition = 0.0;
    double totalTravel = 0.0;
    bool hasPreviousPosition = false;
    QVector<QVector2D> unwrappedPath;
    unwrappedPath.reserve(densePath.size());

    analysis.sectionHull.reserve(static_cast<qsizetype>(hull.size()));

    for (const SectionPoint& point : hull)
    {
        analysis.sectionHull.push_back(QVector2D(static_cast<float>(point.y), static_cast<float>(point.z)));
    }

    for (const QVector3D& point : densePath)
    {
        double perimeterPosition = 0.0;
        double deviation = 0.0;

        if (!mapToHullPerimeter
        (
            { point.y(), point.z() },
            hull,
            cumulativeLengths,
            perimeterPosition,
            deviation
        ))
        {
            analysis.errorMessage = QStringLiteral("加工断面路径无法映射到方管截面周长。");
            return analysis;
        }

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

        if (!hasPreviousPosition)
        {
            previousPosition = perimeterPosition;
            unwrappedPosition = perimeterPosition;
            hasPreviousPosition = true;
            unwrappedPath.push_back(QVector2D(point.x(), static_cast<float>(unwrappedPosition)));
            continue;
        }

        double delta = perimeterPosition - previousPosition;

        while (delta > perimeter * 0.5)
        {
            delta -= perimeter;
        }

        while (delta < -perimeter * 0.5)
        {
            delta += perimeter;
        }

        unwrappedPosition += delta;
        totalTravel += std::abs(delta);
        previousPosition = perimeterPosition;
        unwrappedPath.push_back(QVector2D(point.x(), static_cast<float>(unwrappedPosition)));
    }

    analysis.surfaceConforming = true;
    analysis.windingNumber = unwrappedPosition / perimeter;
    const double absoluteWinding = std::abs(analysis.windingNumber);
    const double perimeterCoverage = totalTravel / perimeter;

    if (std::abs(absoluteWinding - 1.0) > 0.12 || perimeterCoverage < 0.90)
    {
        analysis.errorMessage = QStringLiteral("候选路径未在方管外表面完成一次完整周向绕行，绕行数为 %1。")
            .arg(analysis.windingNumber, 0, 'f', 3);
        return analysis;
    }

    if (hasProperSelfIntersection(unwrappedPath))
    {
        analysis.errorMessage = QStringLiteral("候选路径在方管展开面上存在自相交，不能作为唯一分离边界。");
        return analysis;
    }

    analysis.separating = true;
    std::sort
    (
        analysis.boundaryProfile.begin(),
        analysis.boundaryProfile.end(),
        [](const RotaryCutBoundaryProfileSample& left, const RotaryCutBoundaryProfileSample& right)
        {
            return left.phase < right.phase;
        }
    );
    analysis.valid = true;
    analysis.errorMessage.clear();
    return analysis;
}

bool RotaryCutBoundaryAnalyzer::boundaryXAtPoint
(
    const RotaryCutBoundaryAnalysis& analysis,
    const QVector3D& point,
    double& boundaryX
)
{
    if (!analysis.valid || analysis.sectionHull.size() < 3 || analysis.boundaryProfile.size() < 2)
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
