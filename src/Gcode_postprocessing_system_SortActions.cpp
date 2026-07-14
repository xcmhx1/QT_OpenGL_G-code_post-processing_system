#include "pch.h"

#include "Gcode_postprocessing_system.h"

#include "CadItem.h"
#include "CadCircleItem.h"
#include "CadEllipseItem.h"
#include "CadEllipseGeometry.h"
#include "CadOcsGeometry.h"
#include "CadProcessVisualUtils.h"
#include "RotaryCutBoundaryAnalyzer.h"
#include "RotaryPathTopology.h"

#include <QDebug>
#include <QHash>
#include <QMessageBox>
#include <QSet>
#include <QStatusBar>
#include <QStringList>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <vector>

namespace
{
    constexpr double kSortEpsilon = 1.0e-9;
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kTwoPi = 6.28318530717958647692;
    constexpr double kNextDistanceWeight = 0.15;
    constexpr double kDirectionPenaltyWeight = 0.35;
    constexpr double kBacktrackPenaltyWeight = 1.2;
    constexpr double kRotaryAngleDistanceWeight = 0.08;
    constexpr double kRotaryNextDistanceWeight = 0.12;
    constexpr double kRotaryBacktrackPenaltyWeight = 1.35;
    constexpr double kRotaryDirectionPenaltyWeight = 0.2;
    constexpr double kSortConnectionEpsilon = 1.0e-6;
    constexpr double kEndCutConnectionTolerance = 1.0;
    constexpr double kNearGapPriorityDistance2D = 1.0;
    constexpr double kNearGapPriorityDistance3D = 1.0;
    constexpr double kPreferredStartGapDistance2D = 1.0;
    constexpr double kPreferredStartGapDistance3D = 1.0;
    constexpr double kRotaryPlaneMatchToleranceDegrees = 3.0;
    constexpr double kSurfaceSweepBoundaryTolerance = 1.0e-4;
    constexpr double kSquareTubeSectionToleranceRatio = 0.015;
    constexpr double kSquareTubeEndCutCoverageThreshold = 0.72;
    constexpr double kSortDedupCoordinateTolerance = 1.0e-4;
    const QVector3D kSortOrigin(0.0f, 0.0f, 0.0f);
    // The machine starts above the workpiece at this pose; use it to select the
    // left end-cut seam that requires the least initial A-axis rotation.
    const QVector3D kRotaryInitialSortOrigin(0.0f, 0.0f, 500.0f);

    enum class SortStrategy
    {
        KeepDirection,
        Smart
    };

    struct SortCandidate
    {
        int index = -1;
        bool reverse = false;
        bool hasCustomStart = false;
        double processStartParameter = 0.0;
        double connectionDistance = std::numeric_limits<double>::max();
        double priorityDistance = std::numeric_limits<double>::max();
        double gapDistance = std::numeric_limits<double>::max();
        double score = std::numeric_limits<double>::max();
        bool startsOnInitialTopPlane = false;
        bool startsOnSweepBoundary = false;
        bool matchesCurrentRotaryPlane = false;
        QVector3D startPoint;
        QVector3D endPoint;
    };

    struct ProcessPathOption
    {
        bool reverse = false;
        bool hasCustomStart = false;
        double processStartParameter = 0.0;
        QVector3D startPoint;
        QVector3D endPoint;
        QVector3D startTangent;
        QVector3D endTangent;
    };

    struct RotarySortPoint
    {
        double axis = 0.0;
        double angleDegrees = 0.0;
    };

    struct ProcessConnectionSegment
    {
        QVector3D startPoint;
        QVector3D endPoint;
    };

    struct EndpointNode
    {
        size_t itemIndex = 0;
        QVector3D point;
    };

    struct GapStartSelectionContext
    {
        std::vector<int> componentIds;
        std::vector<std::vector<QVector3D>> preferredStartPointsByComponent;
    };

    enum class RotarySurfaceGroup
    {
        Top,
        Right,
        Bottom,
        Left,
        Unknown
    };

    struct RotaryPathBounds
    {
        bool valid = false;
        double minX = 0.0;
        double maxX = 0.0;
        double minY = 0.0;
        double maxY = 0.0;
        double minZ = 0.0;
        double maxZ = 0.0;
        double sumX = 0.0;
        double sumY = 0.0;
        double sumZ = 0.0;
        size_t pointCount = 0;
    };

    struct RotaryItemAnalysis
    {
        size_t itemIndex = 0;
        RotaryPathBounds bounds;
        RotarySurfaceGroup surface = RotarySurfaceGroup::Unknown;
    };

    struct RotaryFeatureComponent
    {
        std::vector<size_t> itemIndices;
        RotaryPathBounds bounds;
        bool isEndCut = false;
    };

    struct RotaryCutCluster
    {
        std::vector<size_t> itemIndices;
        double centerX = 0.0;
    };

    struct ManualRotaryBreakBoundary
    {
        std::vector<size_t> itemIndices;
        int clusterIndex = -1;
        RotaryCutBoundaryAnalysis analysis;
    };

    struct RotaryLazySegment
    {
        int leftClusterIndex = -1;
        int rightClusterIndex = -1;
        double centerX = 0.0;
        std::vector<std::vector<size_t>> continuousGroups;
        std::vector<size_t> topItems;
        std::vector<size_t> rightItems;
        std::vector<size_t> bottomItems;
        std::vector<size_t> leftItems;
        std::vector<size_t> unknownItems;
    };

    struct SortableDedupResult
    {
        size_t removedCount = 0;
        QVector<CadItem*> duplicateItems;
    };

    struct BoundaryOrderDirectionStats
    {
        int expectedCount = 0;
        int wrongCount = 0;
        int onBoundaryCount = 0;
        int ambiguousCount = 0;
        int stateChangeCount = 0;
        double wrongRatio = 0.0;
        double maximumReverseRunLength = 0.0;
        double abnormalPerimeterSpan = 0.0;
        bool allWrongPointsTolerable = true;
        bool significantReverse = false;
        QVector<int> wrongIndices;
    };

    std::vector<ProcessPathOption> buildPathOptionsForItem(const CadItem* item, SortStrategy strategy);
    bool isPointLexicographicallyLess(const QVector3D& left, const QVector3D& right)
    {
        if (left.x() != right.x())
        {
            return left.x() < right.x();
        }

        if (left.y() != right.y())
        {
            return left.y() < right.y();
        }

        return left.z() < right.z();
    }

    double planarDistanceSquared(const QVector3D& left, const QVector3D& right)
    {
        const double dx = static_cast<double>(left.x()) - static_cast<double>(right.x());
        const double dy = static_cast<double>(left.y()) - static_cast<double>(right.y());
        return dx * dx + dy * dy;
    }

    double spatialDistanceSquared(const QVector3D& left, const QVector3D& right)
    {
        const double dx = static_cast<double>(left.x()) - static_cast<double>(right.x());
        const double dy = static_cast<double>(left.y()) - static_cast<double>(right.y());
        const double dz = static_cast<double>(left.z()) - static_cast<double>(right.z());
        return dx * dx + dy * dy + dz * dz;
    }

    double circularPerimeterDistance(double left, double right, double perimeter)
    {
        if (perimeter <= kSortEpsilon)
        {
            return std::abs(left - right);
        }

        const double distance = std::abs(left - right);
        return std::min(distance, std::max(0.0, perimeter - distance));
    }

    double minimumCircularCoveringSpan(QVector<double> positions, double perimeter)
    {
        if (positions.size() < 2 || perimeter <= kSortEpsilon)
        {
            return 0.0;
        }

        std::sort(positions.begin(), positions.end());
        double maximumGap = 0.0;

        for (int index = 0; index + 1 < positions.size(); ++index)
        {
            maximumGap = std::max(maximumGap, positions[index + 1] - positions[index]);
        }

        maximumGap = std::max(maximumGap, positions.front() + perimeter - positions.back());
        return std::max(0.0, perimeter - maximumGap);
    }

    QString formatDiagnosticValues(const QVector<double>& values)
    {
        QStringList parts;
        parts.reserve(values.size());

        for (const double value : values)
        {
            parts.push_back(QString::number(value, 'f', 6));
        }

        return QStringLiteral("[%1]").arg(parts.join(QStringLiteral(", ")));
    }

    QString formatDiagnosticIndices(const QVector<int>& indices)
    {
        QStringList parts;
        parts.reserve(indices.size());

        for (const int index : indices)
        {
            parts.push_back(QString::number(index));
        }

        return QStringLiteral("[%1]").arg(parts.join(QStringLiteral(", ")));
    }

    BoundaryOrderDirectionStats analyzeBoundaryOrderDirection
    (
        const RotaryCutBoundaryAnalysis& referenceBoundary,
        const RotaryCutBoundaryAnalysis& candidateBoundary,
        RotaryBoundarySide expectedSide,
        double tolerance,
        const QString& diagnosticLabel
    )
    {
        BoundaryOrderDirectionStats stats;
        const QVector<QVector3D> testPoints = RotaryCutBoundaryAnalyzer::buildBoundaryOrderTestPoints
        (
            candidateBoundary,
            tolerance
        );
        const double safeTolerance = std::max(1.0e-6, tolerance);
        const double seamTolerance = std::max
        (
            safeTolerance,
            referenceBoundary.sectionPerimeter * 0.01
        );
        const double duplicateToleranceSquared = std::max
        (
            kSortEpsilon,
            safeTolerance * safeTolerance * 1.0e-12
        );
        QVector<bool> wrongMask(testPoints.size(), false);
        QVector<double> mappedPositions(testPoints.size(), 0.0);
        QVector<int> directionalStates;
        directionalStates.reserve(testPoints.size());

        for (int index = 0; index < testPoints.size(); ++index)
        {
            RotaryBoundaryPointClassificationDiagnostics diagnostics;
            const RotaryBoundarySide side = RotaryCutBoundaryAnalyzer::classifyPointRelativeToBoundary
            (
                referenceBoundary,
                testPoints[index],
                tolerance,
                &diagnostics
            );
            mappedPositions[index] = diagnostics.mappedPerimeterPosition;

            if (side == expectedSide)
            {
                ++stats.expectedCount;
                directionalStates.push_back(0);
                continue;
            }

            if (side == RotaryBoundarySide::OnBoundary)
            {
                ++stats.onBoundaryCount;
                continue;
            }

            if (side == RotaryBoundarySide::Ambiguous)
            {
                ++stats.ambiguousCount;
                continue;
            }

            ++stats.wrongCount;
            directionalStates.push_back(1);
            wrongMask[index] = true;
            stats.wrongIndices.push_back(index);

            const bool isFirst = index == 0;
            const bool isLast = index + 1 == testPoints.size();
            const bool repeatsFirst = index > 0
                && spatialDistanceSquared(testPoints[index], testPoints.front()) <= duplicateToleranceSquared;
            const bool nearSeam = diagnostics.validProjection
                && diagnostics.distanceToPerimeterSeam <= seamTolerance;
            const bool nearBoundary = diagnostics.validProjection
                && diagnostics.minimumBoundaryDistance <= safeTolerance * 2.0;
            stats.allWrongPointsTolerable = stats.allWrongPointsTolerable
                && (nearSeam || repeatsFirst || nearBoundary);

            qWarning().noquote() << QStringLiteral
            (
                "[智能分段] %1 错误侧别点：索引=%2，X/Y/Z=(%3, %4, %5)，首点=%6，末点=%7，与首点重复=%8，周向位置=%9，接缝距离=%10，位于接缝=%11，边界容差附近=%12，边界距离=%13，射线交点X=%14，去重交点X=%15，去重前/后=%16/%17。"
            )
                .arg(diagnosticLabel)
                .arg(index)
                .arg(testPoints[index].x(), 0, 'f', 6)
                .arg(testPoints[index].y(), 0, 'f', 6)
                .arg(testPoints[index].z(), 0, 'f', 6)
                .arg(isFirst ? QStringLiteral("是") : QStringLiteral("否"))
                .arg(isLast ? QStringLiteral("是") : QStringLiteral("否"))
                .arg(repeatsFirst ? QStringLiteral("是") : QStringLiteral("否"))
                .arg(diagnostics.mappedPerimeterPosition, 0, 'f', 6)
                .arg(diagnostics.distanceToPerimeterSeam, 0, 'f', 6)
                .arg(nearSeam ? QStringLiteral("是") : QStringLiteral("否"))
                .arg(nearBoundary ? QStringLiteral("是") : QStringLiteral("否"))
                .arg(diagnostics.minimumBoundaryDistance, 0, 'f', 6)
                .arg(formatDiagnosticValues(diagnostics.rawRayIntersectionXs))
                .arg(formatDiagnosticValues(diagnostics.deduplicatedRayIntersectionXs))
                .arg(diagnostics.rawRayIntersectionXs.size())
                .arg(diagnostics.deduplicatedRayIntersectionXs.size());
        }

        const int classifiedCount = stats.expectedCount + stats.wrongCount;
        stats.wrongRatio = classifiedCount > 0
            ? static_cast<double>(stats.wrongCount) / static_cast<double>(classifiedCount)
            : 0.0;

        if (directionalStates.size() > 1)
        {
            for (int index = 0; index < directionalStates.size(); ++index)
            {
                if (directionalStates[index] != directionalStates[(index + 1) % directionalStates.size()])
                {
                    ++stats.stateChangeCount;
                }
            }
        }

        QVector<double> wrongPositions;

        for (const int index : stats.wrongIndices)
        {
            if (index >= 0 && index < mappedPositions.size())
            {
                wrongPositions.push_back(mappedPositions[index]);
            }
        }

        stats.abnormalPerimeterSpan = minimumCircularCoveringSpan
        (
            wrongPositions,
            referenceBoundary.sectionPerimeter
        );

        if (stats.wrongCount == testPoints.size() && !testPoints.isEmpty())
        {
            stats.maximumReverseRunLength = referenceBoundary.sectionPerimeter;
        }
        else
        {
            for (int index = 0; index < wrongMask.size(); ++index)
            {
                const int previousIndex = (index + wrongMask.size() - 1) % wrongMask.size();

                if (!wrongMask[index] || wrongMask[previousIndex])
                {
                    continue;
                }

                double runLength = 0.0;
                int currentIndex = index;
                int previousWrongIndex = -1;

                for (int step = 0; step < wrongMask.size() && wrongMask[currentIndex]; ++step)
                {
                    if (previousWrongIndex >= 0)
                    {
                        runLength += circularPerimeterDistance
                        (
                            mappedPositions[previousWrongIndex],
                            mappedPositions[currentIndex],
                            referenceBoundary.sectionPerimeter
                        );
                    }

                    previousWrongIndex = currentIndex;
                    currentIndex = (currentIndex + 1) % wrongMask.size();
                }

                stats.maximumReverseRunLength = std::max(stats.maximumReverseRunLength, runLength);
            }
        }

        stats.significantReverse = stats.wrongCount > 0
            && (!stats.allWrongPointsTolerable
                || stats.maximumReverseRunLength > safeTolerance
                || stats.abnormalPerimeterSpan > safeTolerance);
        return stats;
    }

    bool isPointNearAnyPreferredStart(const QVector3D& point, const std::vector<QVector3D>& preferredPoints, double maxDistance)
    {
        const double maxDistanceSquared = maxDistance * maxDistance;

        for (const QVector3D& preferredPoint : preferredPoints)
        {
            if (spatialDistanceSquared(point, preferredPoint) <= maxDistanceSquared)
            {
                return true;
            }
        }

        return false;
    }

    bool documentContainsThreeDimensionalGeometry(const CadDocument& document)
    {
        constexpr float kThreeDimensionalTolerance = 1.0e-5f;

        for (const std::unique_ptr<CadItem>& entity : document.m_entities)
        {
            if (entity == nullptr)
            {
                continue;
            }

            for (const QVector3D& vertex : entity->m_geometry.vertices)
            {
                if (std::abs(vertex.z()) > kThreeDimensionalTolerance)
                {
                    return true;
                }
            }
        }

        return false;
    }

    double unwrapAngleDegrees(double referenceDegrees, double wrappedDegrees)
    {
        return referenceDegrees + std::remainder(wrappedDegrees - referenceDegrees, 360.0);
    }

    bool tryBuildRotarySortPoint(const QVector3D& point, const GProfileRotaryAxisConfig& config, RotarySortPoint& rotaryPoint)
    {
        const double relativeY = static_cast<double>(point.y()) - config.centerY;
        const double relativeZ = static_cast<double>(point.z()) - config.centerZ;

        if (std::hypot(relativeY, relativeZ) <= kSortEpsilon)
        {
            return false;
        }

        double angleDegrees = std::atan2(relativeZ, relativeY) * 180.0 / kPi;

        if (config.invertAAxisDirection)
        {
            angleDegrees = -angleDegrees;
        }

        rotaryPoint.axis = static_cast<double>(point.x());
        rotaryPoint.angleDegrees = angleDegrees + config.aAxisOffsetDegrees;
        return true;
    }

    double rotarySortTravelDistance
    (
        const QVector3D& fromPoint,
        const QVector3D& toPoint,
        const GProfileRotaryAxisConfig& config,
        double* resolvedToAngleDegrees = nullptr
    )
    {
        const double dx = static_cast<double>(toPoint.x()) - static_cast<double>(fromPoint.x());
        const double spatialDistance = static_cast<double>((toPoint - fromPoint).length());
        RotarySortPoint fromRotaryPoint;
        RotarySortPoint toRotaryPoint;

        if (!tryBuildRotarySortPoint(fromPoint, config, fromRotaryPoint) || !tryBuildRotarySortPoint(toPoint, config, toRotaryPoint))
        {
            if (resolvedToAngleDegrees != nullptr)
            {
                *resolvedToAngleDegrees = 0.0;
            }

            return spatialDistance;
        }

        const double resolvedToAngle = unwrapAngleDegrees(fromRotaryPoint.angleDegrees, toRotaryPoint.angleDegrees);
        const double angleDistance = std::abs(resolvedToAngle - fromRotaryPoint.angleDegrees);

        if (resolvedToAngleDegrees != nullptr)
        {
            *resolvedToAngleDegrees = resolvedToAngle;
        }

        return std::abs(dx) + spatialDistance + angleDistance * kRotaryAngleDistanceWeight;
    }

    QVector3D flattenToSortPlane(const QVector3D& point)
    {
        return QVector3D(point.x(), point.y(), 0.0f);
    }

    double pointToSegmentDistanceSquared(const QVector3D& point, const QVector3D& segmentStart, const QVector3D& segmentEnd)
    {
        const QVector3D segment = segmentEnd - segmentStart;
        const double segmentLengthSquared = static_cast<double>(QVector3D::dotProduct(segment, segment));

        if (segmentLengthSquared <= kSortEpsilon)
        {
            return static_cast<double>((point - segmentStart).lengthSquared());
        }

        const double t = std::clamp
        (
            static_cast<double>(QVector3D::dotProduct(point - segmentStart, segment)) / segmentLengthSquared,
            0.0,
            1.0
        );
        const QVector3D projection = segmentStart + segment * static_cast<float>(t);
        return static_cast<double>((point - projection).lengthSquared());
    }

    double segmentToSegmentDistanceSquared(const QVector3D& firstStart, const QVector3D& firstEnd, const QVector3D& secondStart, const QVector3D& secondEnd)
    {
        const QVector3D u = firstEnd - firstStart;
        const QVector3D v = secondEnd - secondStart;
        const QVector3D w = firstStart - secondStart;
        const double a = static_cast<double>(QVector3D::dotProduct(u, u));
        const double b = static_cast<double>(QVector3D::dotProduct(u, v));
        const double c = static_cast<double>(QVector3D::dotProduct(v, v));

        if (a <= kSortEpsilon && c <= kSortEpsilon)
        {
            return static_cast<double>((firstStart - secondStart).lengthSquared());
        }

        if (a <= kSortEpsilon)
        {
            return pointToSegmentDistanceSquared(firstStart, secondStart, secondEnd);
        }

        if (c <= kSortEpsilon)
        {
            return pointToSegmentDistanceSquared(secondStart, firstStart, firstEnd);
        }

        const double d = static_cast<double>(QVector3D::dotProduct(u, w));
        const double e = static_cast<double>(QVector3D::dotProduct(v, w));
        const double denominator = a * c - b * b;

        double sNumerator = 0.0;
        double sDenominator = denominator;
        double tNumerator = 0.0;
        double tDenominator = denominator;

        if (denominator <= kSortEpsilon)
        {
            sNumerator = 0.0;
            sDenominator = 1.0;
            tNumerator = e;
            tDenominator = c;
        }
        else
        {
            sNumerator = b * e - c * d;
            tNumerator = a * e - b * d;

            if (sNumerator < 0.0)
            {
                sNumerator = 0.0;
                tNumerator = e;
                tDenominator = c;
            }
            else if (sNumerator > sDenominator)
            {
                sNumerator = sDenominator;
                tNumerator = e + b;
                tDenominator = c;
            }
        }

        if (tNumerator < 0.0)
        {
            tNumerator = 0.0;

            if (-d < 0.0)
            {
                sNumerator = 0.0;
            }
            else if (-d > a)
            {
                sNumerator = sDenominator;
            }
            else
            {
                sNumerator = -d;
                sDenominator = a;
            }
        }
        else if (tNumerator > tDenominator)
        {
            tNumerator = tDenominator;

            if ((-d + b) < 0.0)
            {
                sNumerator = 0.0;
            }
            else if ((-d + b) > a)
            {
                sNumerator = sDenominator;
            }
            else
            {
                sNumerator = -d + b;
                sDenominator = a;
            }
        }

        const double s = std::abs(sNumerator) <= kSortEpsilon ? 0.0 : sNumerator / sDenominator;
        const double t = std::abs(tNumerator) <= kSortEpsilon ? 0.0 : tNumerator / tDenominator;
        const QVector3D delta = w + u * static_cast<float>(s) - v * static_cast<float>(t);
        return static_cast<double>(QVector3D::dotProduct(delta, delta));
    }

    double planarSegmentToSegmentDistance(const QVector3D& firstStart, const QVector3D& firstEnd, const QVector3D& secondStart, const QVector3D& secondEnd)
    {
        return std::sqrt
        (
            segmentToSegmentDistanceSquared
            (
                flattenToSortPlane(firstStart),
                flattenToSortPlane(firstEnd),
                flattenToSortPlane(secondStart),
                flattenToSortPlane(secondEnd)
            )
        );
    }

    double spatialSegmentToSegmentDistance(const QVector3D& firstStart, const QVector3D& firstEnd, const QVector3D& secondStart, const QVector3D& secondEnd)
    {
        return std::sqrt(segmentToSegmentDistanceSquared(firstStart, firstEnd, secondStart, secondEnd));
    }

    double computeClosestConnectionDistance2D
    (
        const std::vector<ProcessConnectionSegment>& processedSegments,
        const QVector3D& candidateStartPoint,
        const QVector3D& candidateEndPoint
    )
    {
        if (processedSegments.empty())
        {
            return std::numeric_limits<double>::max();
        }

        double bestDistance = std::numeric_limits<double>::max();

        for (const ProcessConnectionSegment& segment : processedSegments)
        {
            bestDistance = std::min
            (
                bestDistance,
                planarSegmentToSegmentDistance(segment.startPoint, segment.endPoint, candidateStartPoint, candidateEndPoint)
            );

            if (bestDistance <= kSortConnectionEpsilon)
            {
                return 0.0;
            }
        }

        return bestDistance;
    }

    double computeClosestConnectionDistance3D
    (
        const std::vector<ProcessConnectionSegment>& processedSegments,
        const QVector3D& candidateStartPoint,
        const QVector3D& candidateEndPoint
    )
    {
        if (processedSegments.empty())
        {
            return std::numeric_limits<double>::max();
        }

        double bestDistance = std::numeric_limits<double>::max();

        for (const ProcessConnectionSegment& segment : processedSegments)
        {
            bestDistance = std::min
            (
                bestDistance,
                spatialSegmentToSegmentDistance(segment.startPoint, segment.endPoint, candidateStartPoint, candidateEndPoint)
            );

            if (bestDistance <= kSortConnectionEpsilon)
            {
                return 0.0;
            }
        }

        return bestDistance;
    }

    QVector3D normalizeOrZero(QVector3D vector)
    {
        vector.setZ(0.0f);

        if (vector.lengthSquared() <= kSortEpsilon)
        {
            return QVector3D();
        }

        vector.normalize();
        return vector;
    }

    QVector3D leftPerpendicular(const QVector3D& vector)
    {
        return QVector3D(-vector.y(), vector.x(), 0.0f);
    }

    double normalizeAnglePositive(double angle)
    {
        double normalized = std::fmod(angle, kTwoPi);

        if (normalized < 0.0)
        {
            normalized += kTwoPi;
        }

        return normalized;
    }

    bool isFullEllipsePath(const DRW_Ellipse* ellipse)
    {
        return ellipse != nullptr && CadEllipseGeometryUtils::isFullEllipseParameterRange
        (
            ellipse->staparam,
            ellipse->endparam
        );
    }

    QVector3D bulgeArcCenter(const QVector3D& startPoint, const QVector3D& endPoint, double bulge, bool* valid = nullptr)
    {
        const QVector3D chord = endPoint - startPoint;
        const double chordLength = chord.length();

        if (valid != nullptr)
        {
            *valid = false;
        }

        if (chordLength <= kSortEpsilon || std::abs(bulge) < 1.0e-8)
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

    QVector3D bulgeSegmentTangentAtStart(const QVector3D& startPoint, const QVector3D& endPoint, double bulge)
    {
        if (std::abs(bulge) < 1.0e-8)
        {
            return normalizeOrZero(endPoint - startPoint);
        }

        bool valid = false;
        const QVector3D center = bulgeArcCenter(startPoint, endPoint, bulge, &valid);

        if (!valid)
        {
            return normalizeOrZero(endPoint - startPoint);
        }

        const QVector3D radiusVector = startPoint - center;
        const QVector3D tangent = bulge > 0.0
            ? leftPerpendicular(radiusVector)
            : -leftPerpendicular(radiusVector);

        return normalizeOrZero(tangent);
    }

    QVector3D bulgeSegmentTangentAtEnd(const QVector3D& startPoint, const QVector3D& endPoint, double bulge)
    {
        if (std::abs(bulge) < 1.0e-8)
        {
            return normalizeOrZero(endPoint - startPoint);
        }

        bool valid = false;
        const QVector3D center = bulgeArcCenter(startPoint, endPoint, bulge, &valid);

        if (!valid)
        {
            return normalizeOrZero(endPoint - startPoint);
        }

        const QVector3D radiusVector = endPoint - center;
        const QVector3D tangent = bulge > 0.0
            ? leftPerpendicular(radiusVector)
            : -leftPerpendicular(radiusVector);

        return normalizeOrZero(tangent);
    }

    bool tryBuildEllipseAxes(const DRW_Ellipse* ellipse, QVector3D& majorAxis, QVector3D& minorAxis)
    {
        CadEllipseGeometry geometry;

        if (!CadEllipseGeometryUtils::buildEllipseGeometry(ellipse, geometry))
        {
            return false;
        }

        majorAxis = geometry.majorAxis;
        minorAxis = geometry.minorAxis;
        return true;
    }

    QVector3D resolveNormal(const DRW_Coord& extPoint)
    {
        QVector3D normal(extPoint.x, extPoint.y, extPoint.z);

        if (normal.lengthSquared() <= kSortEpsilon)
        {
            return QVector3D(0.0f, 0.0f, 1.0f);
        }

        normal.normalize();
        return normal;
    }

    void buildPlaneBasis(const QVector3D& normal, QVector3D& axisX, QVector3D& axisY)
    {
        if (std::abs(normal.x()) <= 1.0e-6f && std::abs(normal.y()) <= 1.0e-6f)
        {
            axisX = QVector3D(1.0f, 0.0f, 0.0f);
            axisY = QVector3D::crossProduct(normal, axisX);

            if (axisY.lengthSquared() <= kSortEpsilon)
            {
                axisY = QVector3D(0.0f, 1.0f, 0.0f);
            }
            else
            {
                axisY.normalize();
            }

            return;
        }

        const QVector3D helper = std::abs(normal.z()) < 0.999f
            ? QVector3D(0.0f, 0.0f, 1.0f)
            : QVector3D(0.0f, 1.0f, 0.0f);

        axisX = QVector3D::crossProduct(helper, normal);

        if (axisX.lengthSquared() <= kSortEpsilon)
        {
            axisX = QVector3D(1.0f, 0.0f, 0.0f);
        }
        else
        {
            axisX.normalize();
        }

        axisY = QVector3D::crossProduct(normal, axisX);

        if (axisY.lengthSquared() <= kSortEpsilon)
        {
            axisY = QVector3D(0.0f, 1.0f, 0.0f);
        }
        else
        {
            axisY.normalize();
        }
    }

    QVector3D arcPointAt(const DRW_Arc* arc, double angle)
    {
        if (arc == nullptr || arc->radious <= 0.0)
        {
            return QVector3D();
        }

        const QVector3D center = CadOcsGeometry::center(arc);
        QVector3D normal = CadOcsGeometry::normal(arc->extPoint);
        QVector3D axisX;
        QVector3D axisY;
        CadOcsGeometry::basis(arc->extPoint, axisX, axisY, normal);

        return center
            + axisX * static_cast<float>(std::cos(angle) * arc->radious)
            + axisY * static_cast<float>(std::sin(angle) * arc->radious);
    }

    QVector3D arcTangentAt(const DRW_Arc* arc, double angle, bool reverseDirection)
    {
        if (arc == nullptr || arc->radious <= 0.0)
        {
            return QVector3D();
        }

        QVector3D normal = CadOcsGeometry::normal(arc->extPoint);
        QVector3D axisX;
        QVector3D axisY;
        CadOcsGeometry::basis(arc->extPoint, axisX, axisY, normal);

        QVector3D tangent
        (
            axisX * static_cast<float>(-std::sin(angle))
            + axisY * static_cast<float>(std::cos(angle))
        );

        if (reverseDirection)
        {
            tangent = -tangent;
        }

        return normalizeOrZero(tangent);
    }

    QVector3D circlePointAt(const DRW_Circle* circle, double parameter)
    {
        if (circle == nullptr || circle->radious <= 0.0)
        {
            return QVector3D();
        }

        return CadOcsGeometry::pointAt(circle, parameter);
    }

    QVector3D circleTangentAt(const DRW_Circle* circle, double parameter, bool reverseDirection)
    {
        if (circle == nullptr || circle->radious <= 0.0)
        {
            return QVector3D();
        }

        return CadOcsGeometry::tangentAt(circle, parameter, reverseDirection);
    }

    QVector3D ellipsePointAt(const DRW_Ellipse* ellipse, double parameter)
    {
        CadEllipseGeometry geometry;

        if (!CadEllipseGeometryUtils::buildEllipseGeometry(ellipse, geometry))
        {
            return QVector3D();
        }

        return CadEllipseGeometryUtils::ellipsePointAt(geometry, parameter);
    }

    QVector3D ellipseTangentAt(const DRW_Ellipse* ellipse, double parameter, bool reverseDirection)
    {
        CadEllipseGeometry geometry;

        if (!CadEllipseGeometryUtils::buildEllipseGeometry(ellipse, geometry))
        {
            return QVector3D();
        }
        return CadEllipseGeometryUtils::ellipseTangentAt(geometry, parameter, reverseDirection);
    }

    QVector3D polylineForwardStartTangent(const DRW_Polyline* polyline)
    {
        if (polyline == nullptr || polyline->vertlist.size() < 2)
        {
            return QVector3D();
        }

        for (size_t index = 0; index + 1 < polyline->vertlist.size(); ++index)
        {
            const auto& current = polyline->vertlist.at(index);
            const auto& next = polyline->vertlist.at(index + 1);
            const QVector3D startPoint(current->basePoint.x, current->basePoint.y, current->basePoint.z);
            const QVector3D endPoint(next->basePoint.x, next->basePoint.y, next->basePoint.z);
            const QVector3D tangent = bulgeSegmentTangentAtStart(startPoint, endPoint, current->bulge);

            if (tangent.lengthSquared() > kSortEpsilon)
            {
                return tangent;
            }
        }

        if ((polyline->flags & 1) != 0)
        {
            const auto& current = polyline->vertlist.back();
            const auto& next = polyline->vertlist.front();
            const QVector3D startPoint(current->basePoint.x, current->basePoint.y, current->basePoint.z);
            const QVector3D endPoint(next->basePoint.x, next->basePoint.y, next->basePoint.z);
            return bulgeSegmentTangentAtStart(startPoint, endPoint, current->bulge);
        }

        return QVector3D();
    }

    QVector3D polylineForwardStartTangentAt(const DRW_Polyline* polyline, size_t startIndex)
    {
        if (polyline == nullptr || polyline->vertlist.size() < 2)
        {
            return QVector3D();
        }

        const size_t count = polyline->vertlist.size();
        const size_t nextIndex = (startIndex + 1) % count;

        if (nextIndex == startIndex)
        {
            return QVector3D();
        }

        const auto& current = polyline->vertlist.at(startIndex);
        const auto& next = polyline->vertlist.at(nextIndex);
        const QVector3D startPoint(current->basePoint.x, current->basePoint.y, current->basePoint.z);
        const QVector3D endPoint(next->basePoint.x, next->basePoint.y, next->basePoint.z);
        return bulgeSegmentTangentAtStart(startPoint, endPoint, current->bulge);
    }

    QVector3D polylineForwardEndTangentAt(const DRW_Polyline* polyline, size_t startIndex)
    {
        if (polyline == nullptr || polyline->vertlist.size() < 2)
        {
            return QVector3D();
        }

        const size_t count = polyline->vertlist.size();
        const size_t previousIndex = (startIndex + count - 1) % count;
        const auto& previous = polyline->vertlist.at(previousIndex);
        const auto& current = polyline->vertlist.at(startIndex);
        const QVector3D startPoint(previous->basePoint.x, previous->basePoint.y, previous->basePoint.z);
        const QVector3D endPoint(current->basePoint.x, current->basePoint.y, current->basePoint.z);
        return bulgeSegmentTangentAtEnd(startPoint, endPoint, previous->bulge);
    }

    QVector3D polylineReverseStartTangent(const DRW_Polyline* polyline)
    {
        if (polyline == nullptr || polyline->vertlist.size() < 2)
        {
            return QVector3D();
        }

        for (size_t index = polyline->vertlist.size() - 1; index > 0; --index)
        {
            const auto& current = polyline->vertlist.at(index);
            const auto& next = polyline->vertlist.at(index - 1);
            const QVector3D startPoint(current->basePoint.x, current->basePoint.y, current->basePoint.z);
            const QVector3D endPoint(next->basePoint.x, next->basePoint.y, next->basePoint.z);
            const QVector3D tangent = bulgeSegmentTangentAtStart(startPoint, endPoint, -next->bulge);

            if (tangent.lengthSquared() > kSortEpsilon)
            {
                return tangent;
            }
        }

        if ((polyline->flags & 1) != 0)
        {
            const auto& current = polyline->vertlist.front();
            const auto& next = polyline->vertlist.back();
            const QVector3D startPoint(current->basePoint.x, current->basePoint.y, current->basePoint.z);
            const QVector3D endPoint(next->basePoint.x, next->basePoint.y, next->basePoint.z);
            return bulgeSegmentTangentAtStart(startPoint, endPoint, -next->bulge);
        }

        return QVector3D();
    }

    QVector3D polylineReverseStartTangentAt(const DRW_Polyline* polyline, size_t startIndex)
    {
        return -polylineForwardEndTangentAt(polyline, startIndex);
    }

    QVector3D polylineReverseEndTangentAt(const DRW_Polyline* polyline, size_t startIndex)
    {
        return -polylineForwardStartTangentAt(polyline, startIndex);
    }

    QVector3D lwPolylineForwardStartTangent(const DRW_LWPolyline* polyline)
    {
        if (polyline == nullptr || polyline->vertlist.size() < 2)
        {
            return QVector3D();
        }

        const float z = static_cast<float>(polyline->elevation);

        for (size_t index = 0; index + 1 < polyline->vertlist.size(); ++index)
        {
            const auto& current = polyline->vertlist.at(index);
            const auto& next = polyline->vertlist.at(index + 1);
            const QVector3D startPoint(static_cast<float>(current->x), static_cast<float>(current->y), z);
            const QVector3D endPoint(static_cast<float>(next->x), static_cast<float>(next->y), z);
            const QVector3D tangent = bulgeSegmentTangentAtStart(startPoint, endPoint, current->bulge);

            if (tangent.lengthSquared() > kSortEpsilon)
            {
                return tangent;
            }
        }

        if ((polyline->flags & 1) != 0)
        {
            const auto& current = polyline->vertlist.back();
            const auto& next = polyline->vertlist.front();
            const QVector3D startPoint(static_cast<float>(current->x), static_cast<float>(current->y), z);
            const QVector3D endPoint(static_cast<float>(next->x), static_cast<float>(next->y), z);
            return bulgeSegmentTangentAtStart(startPoint, endPoint, current->bulge);
        }

        return QVector3D();
    }

    QVector3D lwPolylineForwardStartTangentAt(const DRW_LWPolyline* polyline, size_t startIndex)
    {
        if (polyline == nullptr || polyline->vertlist.size() < 2)
        {
            return QVector3D();
        }

        const size_t count = polyline->vertlist.size();
        const size_t nextIndex = (startIndex + 1) % count;
        const float z = static_cast<float>(polyline->elevation);
        const auto& current = polyline->vertlist.at(startIndex);
        const auto& next = polyline->vertlist.at(nextIndex);
        const QVector3D startPoint(static_cast<float>(current->x), static_cast<float>(current->y), z);
        const QVector3D endPoint(static_cast<float>(next->x), static_cast<float>(next->y), z);
        return bulgeSegmentTangentAtStart(startPoint, endPoint, current->bulge);
    }

    QVector3D lwPolylineForwardEndTangentAt(const DRW_LWPolyline* polyline, size_t startIndex)
    {
        if (polyline == nullptr || polyline->vertlist.size() < 2)
        {
            return QVector3D();
        }

        const size_t count = polyline->vertlist.size();
        const size_t previousIndex = (startIndex + count - 1) % count;
        const float z = static_cast<float>(polyline->elevation);
        const auto& previous = polyline->vertlist.at(previousIndex);
        const auto& current = polyline->vertlist.at(startIndex);
        const QVector3D startPoint(static_cast<float>(previous->x), static_cast<float>(previous->y), z);
        const QVector3D endPoint(static_cast<float>(current->x), static_cast<float>(current->y), z);
        return bulgeSegmentTangentAtEnd(startPoint, endPoint, previous->bulge);
    }

    QVector3D lwPolylineReverseStartTangent(const DRW_LWPolyline* polyline)
    {
        if (polyline == nullptr || polyline->vertlist.size() < 2)
        {
            return QVector3D();
        }

        const float z = static_cast<float>(polyline->elevation);

        for (size_t index = polyline->vertlist.size() - 1; index > 0; --index)
        {
            const auto& current = polyline->vertlist.at(index);
            const auto& next = polyline->vertlist.at(index - 1);
            const QVector3D startPoint(static_cast<float>(current->x), static_cast<float>(current->y), z);
            const QVector3D endPoint(static_cast<float>(next->x), static_cast<float>(next->y), z);
            const QVector3D tangent = bulgeSegmentTangentAtStart(startPoint, endPoint, -next->bulge);

            if (tangent.lengthSquared() > kSortEpsilon)
            {
                return tangent;
            }
        }

        if ((polyline->flags & 1) != 0)
        {
            const auto& current = polyline->vertlist.front();
            const auto& next = polyline->vertlist.back();
            const QVector3D startPoint(static_cast<float>(current->x), static_cast<float>(current->y), z);
            const QVector3D endPoint(static_cast<float>(next->x), static_cast<float>(next->y), z);
            return bulgeSegmentTangentAtStart(startPoint, endPoint, -next->bulge);
        }

        return QVector3D();
    }

    QVector3D lwPolylineReverseStartTangentAt(const DRW_LWPolyline* polyline, size_t startIndex)
    {
        return -lwPolylineForwardEndTangentAt(polyline, startIndex);
    }

    QVector3D lwPolylineReverseEndTangentAt(const DRW_LWPolyline* polyline, size_t startIndex)
    {
        return -lwPolylineForwardStartTangentAt(polyline, startIndex);
    }

    size_t effectiveClosedPolylineStartIndex(const CadItem* item, size_t vertexCount)
    {
        if (vertexCount == 0)
        {
            return 0;
        }

        if (item != nullptr && item->m_hasCustomProcessStart)
        {
            const int rawIndex = static_cast<int>(std::llround(item->m_processStartParameter));
            const int normalized = ((rawIndex % static_cast<int>(vertexCount)) + static_cast<int>(vertexCount)) % static_cast<int>(vertexCount);
            return static_cast<size_t>(normalized);
        }

        return 0;
    }

    QVector3D computeSweepDirection(const std::vector<CadItem*>& sortableItems)
    {
        bool hasAnchor = false;
        QVector3D minPoint;
        QVector3D maxPoint;

        for (CadItem* item : sortableItems)
        {
            const CadProcessVisualInfo info = buildProcessVisualInfo(item);

            if (!info.valid)
            {
                continue;
            }

            if (!hasAnchor)
            {
                minPoint = info.labelAnchor;
                maxPoint = info.labelAnchor;
                hasAnchor = true;
                continue;
            }

            minPoint.setX(std::min(minPoint.x(), info.labelAnchor.x()));
            minPoint.setY(std::min(minPoint.y(), info.labelAnchor.y()));
            maxPoint.setX(std::max(maxPoint.x(), info.labelAnchor.x()));
            maxPoint.setY(std::max(maxPoint.y(), info.labelAnchor.y()));
        }

        if (!hasAnchor)
        {
            return normalizeOrZero(QVector3D(1.0f, 1.0f, 0.0f));
        }

        const QVector3D diagonal(maxPoint.x() - minPoint.x(), maxPoint.y() - minPoint.y(), 0.0f);
        const QVector3D normalized = normalizeOrZero(diagonal);
        return normalized.lengthSquared() > kSortEpsilon
            ? normalized
            : normalizeOrZero(QVector3D(1.0f, 1.0f, 0.0f));
    }

    QVector3D computeRotarySweepDirection(const std::vector<CadItem*>& sortableItems, const GProfileRotaryAxisConfig& config)
    {
        bool hasBounds = false;
        double minAxis = 0.0;
        double maxAxis = 0.0;
        double minAngle = 0.0;
        double maxAngle = 0.0;
        double referenceAngle = 0.0;

        for (CadItem* item : sortableItems)
        {
            const std::vector<ProcessPathOption> options = buildPathOptionsForItem(item, SortStrategy::KeepDirection);

            if (options.empty())
            {
                continue;
            }

            RotarySortPoint rotaryPoint;

            if (!tryBuildRotarySortPoint(options.front().startPoint, config, rotaryPoint))
            {
                continue;
            }

            if (!hasBounds)
            {
                minAxis = maxAxis = rotaryPoint.axis;
                minAngle = maxAngle = rotaryPoint.angleDegrees;
                referenceAngle = rotaryPoint.angleDegrees;
                hasBounds = true;
                continue;
            }

            const double resolvedAngle = unwrapAngleDegrees(referenceAngle, rotaryPoint.angleDegrees);
            minAxis = std::min(minAxis, rotaryPoint.axis);
            maxAxis = std::max(maxAxis, rotaryPoint.axis);
            minAngle = std::min(minAngle, resolvedAngle);
            maxAngle = std::max(maxAngle, resolvedAngle);
        }

        if (!hasBounds)
        {
            return normalizeOrZero(QVector3D(1.0f, 1.0f, 0.0f));
        }

        const QVector3D diagonal
        (
            static_cast<float>(maxAxis - minAxis),
            static_cast<float>(maxAngle - minAngle),
            0.0f
        );
        const QVector3D normalized = normalizeOrZero(diagonal);
        return normalized.lengthSquared() > kSortEpsilon
            ? normalized
            : normalizeOrZero(QVector3D(1.0f, 1.0f, 0.0f));
    }

    double movementContinuityPenalty(const QVector3D& moveVector, const QVector3D& tangentVector)
    {
        const QVector3D normalizedMove = normalizeOrZero(moveVector);
        const QVector3D normalizedTangent = normalizeOrZero(tangentVector);

        if (normalizedMove.lengthSquared() <= kSortEpsilon || normalizedTangent.lengthSquared() <= kSortEpsilon)
        {
            return 0.0;
        }

        const double alignment = std::clamp(static_cast<double>(QVector3D::dotProduct(normalizedMove, normalizedTangent)), -1.0, 1.0);
        return 1.0 - alignment;
    }

    double rotaryMovementContinuityPenalty
    (
        const QVector3D& fromPoint,
        const QVector3D& toPoint,
        const QVector3D& tangentVector,
        const GProfileRotaryAxisConfig& config
    )
    {
        RotarySortPoint fromRotaryPoint;
        RotarySortPoint toRotaryPoint;

        if (!tryBuildRotarySortPoint(fromPoint, config, fromRotaryPoint) || !tryBuildRotarySortPoint(toPoint, config, toRotaryPoint))
        {
            return movementContinuityPenalty(toPoint - fromPoint, tangentVector);
        }

        const double resolvedToAngle = unwrapAngleDegrees(fromRotaryPoint.angleDegrees, toRotaryPoint.angleDegrees);
        QVector3D movementVector
        (
            static_cast<float>(toPoint.x() - fromPoint.x()),
            static_cast<float>(resolvedToAngle - fromRotaryPoint.angleDegrees),
            0.0f
        );
        QVector3D tangentRotary
        (
            tangentVector.x(),
            tangentVector.y() * static_cast<float>(kRotaryAngleDistanceWeight),
            0.0f
        );

        return movementContinuityPenalty(movementVector, tangentRotary);
    }

    std::vector<ProcessPathOption> buildPathOptionsForItem(const CadItem* item, SortStrategy strategy)
    {
        std::vector<ProcessPathOption> options;

        if (item == nullptr || item->m_nativeEntity == nullptr)
        {
            return options;
        }

        switch (item->m_type)
        {
        case DRW::ETYPE::LINE:
        {
            const DRW_Line* line = static_cast<const DRW_Line*>(item->m_nativeEntity);
            const QVector3D forwardStart(line->basePoint.x, line->basePoint.y, line->basePoint.z);
            const QVector3D forwardEnd(line->secPoint.x, line->secPoint.y, line->secPoint.z);
            const QVector3D forwardTangent = normalizeOrZero(forwardEnd - forwardStart);
            const std::initializer_list<bool> reverseOptions = strategy == SortStrategy::Smart
                ? std::initializer_list<bool>{ false, true }
                : std::initializer_list<bool>{ item->m_isReverse };

            for (const bool reverse : reverseOptions)
            {
                ProcessPathOption option;
                option.reverse = reverse;
                option.startPoint = reverse ? forwardEnd : forwardStart;
                option.endPoint = reverse ? forwardStart : forwardEnd;
                option.startTangent = reverse ? -forwardTangent : forwardTangent;
                option.endTangent = option.startTangent;
                options.push_back(option);
            }

            break;
        }
        case DRW::ETYPE::ARC:
        {
            const DRW_Arc* arc = static_cast<const DRW_Arc*>(item->m_nativeEntity);
            const std::initializer_list<bool> reverseOptions = strategy == SortStrategy::Smart
                ? std::initializer_list<bool>{ false, true }
                : std::initializer_list<bool>{ item->m_isReverse };

            for (const bool reverse : reverseOptions)
            {
                ProcessPathOption option;
                option.reverse = reverse;
                option.startPoint = reverse ? arcPointAt(arc, arc->endangle) : arcPointAt(arc, arc->staangle);
                option.endPoint = reverse ? arcPointAt(arc, arc->staangle) : arcPointAt(arc, arc->endangle);
                option.startTangent = reverse ? arcTangentAt(arc, arc->endangle, true) : arcTangentAt(arc, arc->staangle, false);
                option.endTangent = reverse ? arcTangentAt(arc, arc->staangle, true) : arcTangentAt(arc, arc->endangle, false);
                options.push_back(option);
            }

            break;
        }
        case DRW::ETYPE::CIRCLE:
        {
            const DRW_Circle* circle = static_cast<const DRW_Circle*>(item->m_nativeEntity);
            const CadCircleItem* circleItem = static_cast<const CadCircleItem*>(item);
            const std::initializer_list<bool> reverseOptions = strategy == SortStrategy::Smart
                ? std::initializer_list<bool>{ false, true }
                : std::initializer_list<bool>{ item->m_isReverse };

            const double startParameter = circleItem->defaultProcessStartParameter();
            for (const bool reverse : reverseOptions)
            {
                ProcessPathOption option;
                option.reverse = reverse;
                option.hasCustomStart = false;
                option.processStartParameter = startParameter;
                option.startPoint = circlePointAt(circle, startParameter);
                option.endPoint = option.startPoint;
                option.startTangent = circleTangentAt(circle, startParameter, reverse);
                option.endTangent = option.startTangent;
                options.push_back(option);
            }

            break;
        }
        case DRW::ETYPE::ELLIPSE:
        {
            const DRW_Ellipse* ellipse = static_cast<const DRW_Ellipse*>(item->m_nativeEntity);
            const bool isClosed = isFullEllipsePath(ellipse);

            if (isClosed && strategy == SortStrategy::Smart)
            {
                const CadEllipseItem* ellipseItem = static_cast<const CadEllipseItem*>(item);
                const double parameter = ellipseItem->defaultProcessStartParameter();

                for (const bool reverse : { false, true })
                {
                    ProcessPathOption option;
                    option.reverse = reverse;
                    option.hasCustomStart = false;
                    option.processStartParameter = parameter;
                    option.startPoint = ellipsePointAt(ellipse, parameter);
                    option.endPoint = option.startPoint;
                    option.startTangent = ellipseTangentAt(ellipse, parameter, reverse);
                    option.endTangent = option.startTangent;
                    options.push_back(option);
                }
            }
            else
            {
                const std::initializer_list<bool> reverseOptions = strategy == SortStrategy::Smart
                    ? std::initializer_list<bool>{ false, true }
                    : std::initializer_list<bool>{ item->m_isReverse };

                double startParam = ellipse->staparam;
                double endParam = ellipse->endparam;
                bool hasCustomStart = false;

                if (isClosed)
                {
                    const CadEllipseItem* ellipseItem = static_cast<const CadEllipseItem*>(item);
                    startParam = ellipseItem->defaultProcessStartParameter();
                    endParam = startParam;
                }
                else
                {
                    while (endParam <= startParam)
                    {
                        endParam += kTwoPi;
                    }
                }

                for (const bool reverse : reverseOptions)
                {
                    ProcessPathOption option;
                    option.reverse = reverse;
                    option.hasCustomStart = hasCustomStart;
                    option.processStartParameter = startParam;
                    option.startPoint = reverse ? ellipsePointAt(ellipse, endParam) : ellipsePointAt(ellipse, startParam);
                    option.endPoint = reverse ? ellipsePointAt(ellipse, startParam) : ellipsePointAt(ellipse, endParam);
                    option.startTangent = reverse ? ellipseTangentAt(ellipse, endParam, true) : ellipseTangentAt(ellipse, startParam, false);
                    option.endTangent = reverse ? ellipseTangentAt(ellipse, startParam, true) : ellipseTangentAt(ellipse, endParam, false);
                    options.push_back(option);
                }
            }

            break;
        }
        case DRW::ETYPE::POLYLINE:
        {
            const DRW_Polyline* polyline = static_cast<const DRW_Polyline*>(item->m_nativeEntity);

            if (polyline->vertlist.empty())
            {
                break;
            }

            const bool isClosed = (polyline->flags & 1) != 0;

            if (isClosed && strategy == SortStrategy::Smart)
            {
                const size_t count = polyline->vertlist.size();

                for (size_t startIndex = 0; startIndex < count; ++startIndex)
                {
                    const auto& seamVertex = polyline->vertlist.at(startIndex);
                    const QVector3D seamPoint(seamVertex->basePoint.x, seamVertex->basePoint.y, seamVertex->basePoint.z);

                    for (const bool reverse : { false, true })
                    {
                        ProcessPathOption option;
                        option.reverse = reverse;
                        option.hasCustomStart = true;
                        option.processStartParameter = static_cast<double>(startIndex);
                        option.startPoint = seamPoint;
                        option.endPoint = seamPoint;
                        option.startTangent = reverse
                            ? polylineReverseStartTangentAt(polyline, startIndex)
                            : polylineForwardStartTangentAt(polyline, startIndex);
                        option.endTangent = reverse
                            ? polylineReverseEndTangentAt(polyline, startIndex)
                            : polylineForwardEndTangentAt(polyline, startIndex);
                        options.push_back(option);
                    }
                }
            }
            else
            {
                const auto& firstVertex = polyline->vertlist.front();
                const auto& lastVertex = polyline->vertlist.back();
                const size_t seamIndex = isClosed
                    ? effectiveClosedPolylineStartIndex(item, polyline->vertlist.size())
                    : 0;
                const QVector3D forwardStart = isClosed
                    ? QVector3D(polyline->vertlist.at(seamIndex)->basePoint.x, polyline->vertlist.at(seamIndex)->basePoint.y, polyline->vertlist.at(seamIndex)->basePoint.z)
                    : QVector3D(firstVertex->basePoint.x, firstVertex->basePoint.y, firstVertex->basePoint.z);
                const QVector3D forwardEnd = isClosed
                    ? forwardStart
                    : QVector3D(lastVertex->basePoint.x, lastVertex->basePoint.y, lastVertex->basePoint.z);
                const QVector3D forwardStartTangent = isClosed
                    ? polylineForwardStartTangentAt(polyline, seamIndex)
                    : polylineForwardStartTangent(polyline);
                const QVector3D reverseStartTangent = isClosed
                    ? polylineReverseStartTangentAt(polyline, seamIndex)
                    : polylineReverseStartTangent(polyline);
                const QVector3D forwardEndTangent = isClosed
                    ? polylineForwardEndTangentAt(polyline, seamIndex)
                    : -reverseStartTangent;
                const QVector3D reverseEndTangent = isClosed
                    ? polylineReverseEndTangentAt(polyline, seamIndex)
                    : -forwardStartTangent;
                const std::initializer_list<bool> reverseOptions = strategy == SortStrategy::Smart
                    ? std::initializer_list<bool>{ false, true }
                    : std::initializer_list<bool>{ item->m_isReverse };

                for (const bool reverse : reverseOptions)
                {
                    ProcessPathOption option;
                    option.reverse = reverse;
                    option.hasCustomStart = isClosed && item->m_hasCustomProcessStart;
                    option.processStartParameter = isClosed ? static_cast<double>(seamIndex) : 0.0;
                    option.startPoint = reverse ? forwardEnd : forwardStart;
                    option.endPoint = reverse ? forwardStart : forwardEnd;
                    option.startTangent = reverse ? reverseStartTangent : forwardStartTangent;
                    option.endTangent = reverse ? reverseEndTangent : forwardEndTangent;
                    options.push_back(option);
                }
            }

            break;
        }
        case DRW::ETYPE::LWPOLYLINE:
        {
            const DRW_LWPolyline* polyline = static_cast<const DRW_LWPolyline*>(item->m_nativeEntity);

            if (polyline->vertlist.empty())
            {
                break;
            }

            const bool isClosed = (polyline->flags & 1) != 0;

            if (isClosed && strategy == SortStrategy::Smart)
            {
                const size_t count = polyline->vertlist.size();
                const float z = static_cast<float>(polyline->elevation);

                for (size_t startIndex = 0; startIndex < count; ++startIndex)
                {
                    const auto& seamVertex = polyline->vertlist.at(startIndex);
                    const QVector3D seamPoint(static_cast<float>(seamVertex->x), static_cast<float>(seamVertex->y), z);

                    for (const bool reverse : { false, true })
                    {
                        ProcessPathOption option;
                        option.reverse = reverse;
                        option.hasCustomStart = true;
                        option.processStartParameter = static_cast<double>(startIndex);
                        option.startPoint = seamPoint;
                        option.endPoint = seamPoint;
                        option.startTangent = reverse
                            ? lwPolylineReverseStartTangentAt(polyline, startIndex)
                            : lwPolylineForwardStartTangentAt(polyline, startIndex);
                        option.endTangent = reverse
                            ? lwPolylineReverseEndTangentAt(polyline, startIndex)
                            : lwPolylineForwardEndTangentAt(polyline, startIndex);
                        options.push_back(option);
                    }
                }
            }
            else
            {
                const auto& firstVertex = polyline->vertlist.front();
                const auto& lastVertex = polyline->vertlist.back();
                const float z = static_cast<float>(polyline->elevation);
                const size_t seamIndex = isClosed
                    ? effectiveClosedPolylineStartIndex(item, polyline->vertlist.size())
                    : 0;
                const QVector3D forwardStart = isClosed
                    ? QVector3D(static_cast<float>(polyline->vertlist.at(seamIndex)->x), static_cast<float>(polyline->vertlist.at(seamIndex)->y), z)
                    : QVector3D(static_cast<float>(firstVertex->x), static_cast<float>(firstVertex->y), z);
                const QVector3D forwardEnd = isClosed
                    ? forwardStart
                    : QVector3D(static_cast<float>(lastVertex->x), static_cast<float>(lastVertex->y), z);
                const QVector3D forwardStartTangent = isClosed
                    ? lwPolylineForwardStartTangentAt(polyline, seamIndex)
                    : lwPolylineForwardStartTangent(polyline);
                const QVector3D reverseStartTangent = isClosed
                    ? lwPolylineReverseStartTangentAt(polyline, seamIndex)
                    : lwPolylineReverseStartTangent(polyline);
                const QVector3D forwardEndTangent = isClosed
                    ? lwPolylineForwardEndTangentAt(polyline, seamIndex)
                    : -reverseStartTangent;
                const QVector3D reverseEndTangent = isClosed
                    ? lwPolylineReverseEndTangentAt(polyline, seamIndex)
                    : -forwardStartTangent;
                const std::initializer_list<bool> reverseOptions = strategy == SortStrategy::Smart
                    ? std::initializer_list<bool>{ false, true }
                    : std::initializer_list<bool>{ item->m_isReverse };

                for (const bool reverse : reverseOptions)
                {
                    ProcessPathOption option;
                    option.reverse = reverse;
                    option.hasCustomStart = isClosed && item->m_hasCustomProcessStart;
                    option.processStartParameter = isClosed ? static_cast<double>(seamIndex) : 0.0;
                    option.startPoint = reverse ? forwardEnd : forwardStart;
                    option.endPoint = reverse ? forwardStart : forwardEnd;
                    option.startTangent = reverse ? reverseStartTangent : forwardStartTangent;
                    option.endTangent = reverse ? reverseEndTangent : forwardEndTangent;
                    options.push_back(option);
                }
            }

            break;
        }
        case DRW::ETYPE::SPLINE:
        {
            const QVector<QVector3D>& vertices = item->m_geometry.vertices;
            if (vertices.size() < 2)
            {
                break;
            }
            const DRW_Spline* spline =
                static_cast<const DRW_Spline*>(item->m_nativeEntity);
            const bool isClosed = (spline->flags & (1 | 2)) != 0;
            const QVector3D forwardStart = vertices.constFirst();
            const QVector3D forwardEnd = isClosed
                ? forwardStart : vertices.constLast();
            const QVector3D forwardStartTangent =
                normalizeOrZero(vertices.at(1) - vertices.constFirst());
            const QVector3D reverseStartTangent =
                normalizeOrZero(vertices.at(vertices.size() - 2) - vertices.constLast());
            const QVector3D forwardEndTangent = isClosed
                ? -reverseStartTangent
                : normalizeOrZero(vertices.constLast() - vertices.at(vertices.size() - 2));
            const QVector3D reverseEndTangent = -forwardStartTangent;
            const std::initializer_list<bool> reverseOptions = strategy == SortStrategy::Smart
                ? std::initializer_list<bool>{ false, true }
                : std::initializer_list<bool>{ item->m_isReverse };
            for (const bool reverse : reverseOptions)
            {
                ProcessPathOption option;
                option.reverse = reverse;
                option.startPoint = reverse ? forwardEnd : forwardStart;
                option.endPoint = reverse ? forwardStart : forwardEnd;
                option.startTangent = reverse
                    ? reverseStartTangent : forwardStartTangent;
                option.endTangent = reverse
                    ? reverseEndTangent : forwardEndTangent;
                options.push_back(option);
            }
            break;
        }
        default:
            break;
        }

        return options;
    }

    std::vector<EndpointNode> collectOpenEndpointNodes(const std::vector<CadItem*>& sortableItems)
    {
        std::vector<EndpointNode> endpoints;

        for (size_t index = 0; index < sortableItems.size(); ++index)
        {
            const std::vector<ProcessPathOption> options = buildPathOptionsForItem(sortableItems[index], SortStrategy::KeepDirection);

            if (options.empty())
            {
                continue;
            }

            const ProcessPathOption& option = options.front();

            if (spatialDistanceSquared(option.startPoint, option.endPoint) <= kSortConnectionEpsilon * kSortConnectionEpsilon)
            {
                continue;
            }

            endpoints.push_back({ index, option.startPoint });
            endpoints.push_back({ index, option.endPoint });
        }

        return endpoints;
    }

    std::vector<int> collectLooseEndpointIndices
    (
        const std::vector<EndpointNode>& endpoints,
        double exactConnectionDistance
    )
    {
        std::vector<int> looseIndices;
        const double exactConnectionDistanceSquared = exactConnectionDistance * exactConnectionDistance;

        for (size_t index = 0; index < endpoints.size(); ++index)
        {
            bool hasExactMatch = false;

            for (size_t otherIndex = 0; otherIndex < endpoints.size(); ++otherIndex)
            {
                if (index == otherIndex || endpoints[index].itemIndex == endpoints[otherIndex].itemIndex)
                {
                    continue;
                }

                if (spatialDistanceSquared(endpoints[index].point, endpoints[otherIndex].point) <= exactConnectionDistanceSquared)
                {
                    hasExactMatch = true;
                    break;
                }
            }

            if (!hasExactMatch)
            {
                looseIndices.push_back(static_cast<int>(index));
            }
        }

        return looseIndices;
    }

    std::vector<int> buildItemConnectivityComponents
    (
        const std::vector<CadItem*>& sortableItems,
        double connectionDistance = kSortConnectionEpsilon
    )
    {
        QVector<CadItem*> items;
        items.reserve(static_cast<qsizetype>(sortableItems.size()));

        for (CadItem* item : sortableItems)
        {
            items.push_back(item);
        }

        const RotaryPathTopology topology
        (
            items,
            RotaryPathTopologyTolerance::fromConnectionTolerance(connectionDistance)
        );
        return topology.itemComponentIds();
    }

    std::vector<std::vector<QVector3D>> detectPreferredGapStartPointsByComponent
    (
        const std::vector<CadItem*>& sortableItems,
        const std::vector<int>& componentIds,
        double preferredGapDistance
    )
    {
        const std::vector<EndpointNode> endpoints = collectOpenEndpointNodes(sortableItems);
        const std::vector<int> looseIndices = collectLooseEndpointIndices(endpoints, kSortConnectionEpsilon);
        const int componentCount = componentIds.empty()
            ? 0
            : (*std::max_element(componentIds.begin(), componentIds.end()) + 1);
        std::vector<std::vector<QVector3D>> preferredPointsByComponent(static_cast<size_t>(std::max(0, componentCount)));

        if (looseIndices.size() < 2 || componentCount <= 0)
        {
            return preferredPointsByComponent;
        }

        std::vector<int> nearestLooseIndex(looseIndices.size(), -1);
        std::vector<double> nearestLooseDistance(looseIndices.size(), std::numeric_limits<double>::max());

        for (size_t localIndex = 0; localIndex < looseIndices.size(); ++localIndex)
        {
            const EndpointNode& node = endpoints[static_cast<size_t>(looseIndices[localIndex])];
            const int componentId = componentIds[node.itemIndex];

            if (componentId < 0)
            {
                continue;
            }

            for (size_t otherLocalIndex = 0; otherLocalIndex < looseIndices.size(); ++otherLocalIndex)
            {
                if (localIndex == otherLocalIndex)
                {
                    continue;
                }

                const EndpointNode& otherNode = endpoints[static_cast<size_t>(looseIndices[otherLocalIndex])];

                if (node.itemIndex == otherNode.itemIndex)
                {
                    continue;
                }

                if (componentIds[otherNode.itemIndex] != componentId)
                {
                    continue;
                }

                const double distance = std::sqrt(spatialDistanceSquared(node.point, otherNode.point));

                const bool shouldReplace = nearestLooseIndex[localIndex] < 0
                    || distance < nearestLooseDistance[localIndex] - kSortEpsilon
                    || (std::abs(distance - nearestLooseDistance[localIndex]) <= kSortEpsilon
                        && isPointLexicographicallyLess
                        (
                            otherNode.point,
                            endpoints[static_cast<size_t>(looseIndices[static_cast<size_t>(nearestLooseIndex[localIndex])])].point
                        ));

                if (shouldReplace)
                {
                    nearestLooseDistance[localIndex] = distance;
                    nearestLooseIndex[localIndex] = static_cast<int>(otherLocalIndex);
                }
            }
        }

        std::vector<QVector3D> preferredPoints;

        for (size_t localIndex = 0; localIndex < looseIndices.size(); ++localIndex)
        {
            const int nearestIndex = nearestLooseIndex[localIndex];
            const EndpointNode& firstNode = endpoints[static_cast<size_t>(looseIndices[localIndex])];
            const int componentId = componentIds[firstNode.itemIndex];

            if (nearestIndex < 0
                || componentId < 0
                || nearestLooseDistance[localIndex] > preferredGapDistance + kSortEpsilon
                || nearestLooseIndex[static_cast<size_t>(nearestIndex)] != static_cast<int>(localIndex))
            {
                continue;
            }

            std::vector<QVector3D>& preferredPoints = preferredPointsByComponent[static_cast<size_t>(componentId)];
            const QVector3D firstPoint = firstNode.point;
            const QVector3D secondPoint = endpoints[static_cast<size_t>(looseIndices[static_cast<size_t>(nearestIndex)])].point;

            if (!isPointNearAnyPreferredStart(firstPoint, preferredPoints, kSortConnectionEpsilon))
            {
                preferredPoints.push_back(firstPoint);
            }

            if (!isPointNearAnyPreferredStart(secondPoint, preferredPoints, kSortConnectionEpsilon))
            {
                preferredPoints.push_back(secondPoint);
            }
        }

        return preferredPointsByComponent;
    }

    std::vector<bool> buildVisitedComponentMask(const std::vector<bool>& visited, const std::vector<int>& componentIds)
    {
        const int componentCount = componentIds.empty()
            ? 0
            : (*std::max_element(componentIds.begin(), componentIds.end()) + 1);
        std::vector<bool> visitedComponents(static_cast<size_t>(std::max(0, componentCount)), false);

        for (size_t itemIndex = 0; itemIndex < visited.size() && itemIndex < componentIds.size(); ++itemIndex)
        {
            const int componentId = componentIds[itemIndex];

            if (visited[itemIndex] && componentId >= 0)
            {
                visitedComponents[static_cast<size_t>(componentId)] = true;
            }
        }

        return visitedComponents;
    }

    bool hasRemainingUnvisitedInComponent
    (
        const std::vector<bool>& visited,
        const std::vector<int>& componentIds,
        int componentId
    )
    {
        if (componentId < 0)
        {
            return false;
        }

        for (size_t itemIndex = 0; itemIndex < visited.size() && itemIndex < componentIds.size(); ++itemIndex)
        {
            if (!visited[itemIndex] && componentIds[itemIndex] == componentId)
            {
                return true;
            }
        }

        return false;
    }

    GapStartSelectionContext buildGapStartSelectionContext(const std::vector<CadItem*>& sortableItems, double preferredGapDistance)
    {
        GapStartSelectionContext context;
        context.componentIds = buildItemConnectivityComponents(sortableItems);
        context.preferredStartPointsByComponent = detectPreferredGapStartPointsByComponent
        (
            sortableItems,
            context.componentIds,
            preferredGapDistance
        );
        return context;
    }

    QString sortDedupPointToken(const RawPathPoint3D& point)
    {
        const qint64 x = static_cast<qint64>(std::llround(point.x / kSortDedupCoordinateTolerance));
        const qint64 y = static_cast<qint64>(std::llround(point.y / kSortDedupCoordinateTolerance));
        const qint64 z = static_cast<qint64>(std::llround(point.z / kSortDedupCoordinateTolerance));
        return QStringLiteral("%1,%2,%3").arg(x).arg(y).arg(z);
    }

    QString sortDedupJoinTokens(const std::vector<QString>& tokens, size_t startIndex, bool reverse)
    {
        QString key;
        key.reserve(static_cast<int>(tokens.size() * 24));

        for (size_t offset = 0; offset < tokens.size(); ++offset)
        {
            const size_t index = reverse
                ? (startIndex + tokens.size() - offset) % tokens.size()
                : (startIndex + offset) % tokens.size();

            if (!key.isEmpty())
            {
                key.append(QLatin1Char(';'));
            }

            key.append(tokens[index]);
        }

        return key;
    }

    QString sortDedupCanonicalPathKey(CadItem* item)
    {
        if (item == nullptr)
        {
            return QString();
        }

        item->rebuildRawPathPoints3D();

        std::vector<QString> tokens;
        tokens.reserve(item->rawPathPoints3D().size());

        for (const RawPathPoint3D& point : item->rawPathPoints3D())
        {
            const QString token = sortDedupPointToken(point);

            if (!tokens.empty() && tokens.back() == token)
            {
                continue;
            }

            tokens.push_back(token);
        }

        if (tokens.empty())
        {
            return QString();
        }

        bool closed = false;

        if (tokens.size() > 1 && tokens.front() == tokens.back())
        {
            tokens.pop_back();
            closed = true;
        }

        if (tokens.empty())
        {
            return QString();
        }

        QString bestKey;

        if (closed)
        {
            for (size_t index = 0; index < tokens.size(); ++index)
            {
                const QString forwardKey = sortDedupJoinTokens(tokens, index, false);
                const QString reverseKey = sortDedupJoinTokens(tokens, index, true);

                if (bestKey.isEmpty() || forwardKey < bestKey)
                {
                    bestKey = forwardKey;
                }

                if (reverseKey < bestKey)
                {
                    bestKey = reverseKey;
                }
            }
        }
        else
        {
            const QString forwardKey = sortDedupJoinTokens(tokens, 0, false);
            const QString reverseKey = sortDedupJoinTokens(tokens, tokens.size() - 1, true);
            bestKey = forwardKey < reverseKey ? forwardKey : reverseKey;
        }

        return QStringLiteral("%1|%2|%3")
            .arg(static_cast<int>(item->m_type))
            .arg(static_cast<qulonglong>(item->buildColor().rgba()))
            .arg(bestKey);
    }

    SortableDedupResult deduplicateSortableItems(std::vector<CadItem*>& sortableItems)
    {
        SortableDedupResult result;
        std::vector<CadItem*> uniqueItems;
        std::vector<QString> uniqueKeys;
        uniqueItems.reserve(sortableItems.size());
        uniqueKeys.reserve(sortableItems.size());

        for (CadItem* item : sortableItems)
        {
            const QString key = sortDedupCanonicalPathKey(item);

            if (key.isEmpty())
            {
                uniqueItems.push_back(item);
                continue;
            }

            const bool duplicate = std::find(uniqueKeys.begin(), uniqueKeys.end(), key) != uniqueKeys.end();

            if (duplicate)
            {
                result.duplicateItems.append(item);
                ++result.removedCount;
                continue;
            }

            uniqueKeys.push_back(key);
            uniqueItems.push_back(item);
        }

        sortableItems = std::move(uniqueItems);
        return result;
    }

    bool tryFindNearestNextStartPoint
    (
        const std::vector<CadItem*>& sortableItems,
        const std::vector<bool>& visited,
        SortStrategy strategy,
        size_t currentIndex,
        const QVector3D& currentEndPoint,
        QVector3D& nextStartPoint
    )
    {
        int bestIndex = -1;
        double bestDistance = std::numeric_limits<double>::max();
        QVector3D bestStartPoint;

        for (size_t index = 0; index < sortableItems.size(); ++index)
        {
            if (index == currentIndex || visited[index])
            {
                continue;
            }

            const std::vector<ProcessPathOption> options = buildPathOptionsForItem(sortableItems[index], strategy);

            for (const ProcessPathOption& option : options)
            {
                const double distance = std::sqrt(planarDistanceSquared(option.startPoint, currentEndPoint));
                const bool shouldReplace = bestIndex < 0
                    || distance < bestDistance - kSortEpsilon
                    || (std::abs(distance - bestDistance) <= kSortEpsilon
                        && isPointLexicographicallyLess(option.startPoint, bestStartPoint));

                if (!shouldReplace)
                {
                    continue;
                }

                bestIndex = static_cast<int>(index);
                bestDistance = distance;
                bestStartPoint = option.startPoint;
            }
        }

        if (bestIndex < 0)
        {
            return false;
        }

        nextStartPoint = bestStartPoint;
        return true;
    }

    bool tryFindNearestNextStartPoint3D
    (
        const std::vector<CadItem*>& sortableItems,
        const std::vector<bool>& visited,
        SortStrategy strategy,
        size_t currentIndex,
        const QVector3D& currentEndPoint,
        const GProfileRotaryAxisConfig& config,
        QVector3D& nextStartPoint
    )
    {
        int bestIndex = -1;
        double bestDistance = std::numeric_limits<double>::max();
        QVector3D bestStartPoint;

        for (size_t index = 0; index < sortableItems.size(); ++index)
        {
            if (index == currentIndex || visited[index])
            {
                continue;
            }

            const std::vector<ProcessPathOption> options = buildPathOptionsForItem(sortableItems[index], strategy);

            for (const ProcessPathOption& option : options)
            {
                const double distance = rotarySortTravelDistance(currentEndPoint, option.startPoint, config);
                const bool shouldReplace = bestIndex < 0
                    || distance < bestDistance - kSortEpsilon
                    || (std::abs(distance - bestDistance) <= kSortEpsilon
                        && isPointLexicographicallyLess(option.startPoint, bestStartPoint));

                if (!shouldReplace)
                {
                    continue;
                }

                bestIndex = static_cast<int>(index);
                bestDistance = distance;
                bestStartPoint = option.startPoint;
            }
        }

        if (bestIndex < 0)
        {
            return false;
        }

        nextStartPoint = bestStartPoint;
        return true;
    }

    SortCandidate chooseNext2DSortCandidate
    (
        const std::vector<CadItem*>& sortableItems,
        const std::vector<bool>& visited,
        const std::vector<ProcessConnectionSegment>& processedSegments,
        const GapStartSelectionContext& gapStartContext,
        SortStrategy strategy,
        int currentComponentId,
        int restrictedComponentId,
        bool preferPreferredGapStart,
        bool hasCurrentEndPoint,
        const QVector3D& currentEndPoint,
        const QVector3D& sweepDirection
    )
    {
        SortCandidate bestCandidate;
        const std::vector<bool> visitedComponents = buildVisitedComponentMask(visited, gapStartContext.componentIds);
        const bool mustStayInCurrentComponent = hasCurrentEndPoint
            && hasRemainingUnvisitedInComponent(visited, gapStartContext.componentIds, currentComponentId);
        const QVector3D referencePoint = hasCurrentEndPoint ? currentEndPoint : kSortOrigin;
        const QVector3D normalizedSweepDirection = normalizeOrZero(sweepDirection);
        const double referenceProgress = static_cast<double>(QVector3D::dotProduct(referencePoint, normalizedSweepDirection));

        for (size_t index = 0; index < sortableItems.size(); ++index)
        {
            if (visited[index])
            {
                continue;
            }

            const std::vector<ProcessPathOption> options = buildPathOptionsForItem(sortableItems[index], strategy);

            for (const ProcessPathOption& option : options)
            {
                const int componentId = index < gapStartContext.componentIds.size()
                    ? gapStartContext.componentIds[index]
                    : -1;

                if (mustStayInCurrentComponent && componentId != currentComponentId)
                {
                    continue;
                }

                if (restrictedComponentId >= 0 && componentId != restrictedComponentId)
                {
                    continue;
                }

                const std::vector<QVector3D> emptyPreferredPoints;
                const std::vector<QVector3D>& componentPreferredPoints =
                    (componentId >= 0 && static_cast<size_t>(componentId) < gapStartContext.preferredStartPointsByComponent.size())
                    ? gapStartContext.preferredStartPointsByComponent[static_cast<size_t>(componentId)]
                    : emptyPreferredPoints;
                const double connectionDistance = computeClosestConnectionDistance2D(processedSegments, option.startPoint, option.endPoint);
                const bool directlyConnected = connectionDistance <= kSortConnectionEpsilon;
                const bool bestDirectlyConnected = bestCandidate.connectionDistance <= kSortConnectionEpsilon;
                const double entryDistance = std::sqrt(planarDistanceSquared(option.startPoint, referencePoint));
                const double currentGapDistance = hasCurrentEndPoint
                    ? std::sqrt(planarDistanceSquared(option.startPoint, currentEndPoint))
                    : entryDistance;
                const bool nearCurrentGap = hasCurrentEndPoint && currentGapDistance <= kNearGapPriorityDistance2D;
                const bool bestNearCurrentGap = hasCurrentEndPoint && bestCandidate.gapDistance <= kNearGapPriorityDistance2D;
                const bool preferredGapStart = preferPreferredGapStart
                    && componentId == restrictedComponentId
                    && isPointNearAnyPreferredStart(option.startPoint, componentPreferredPoints, kPreferredStartGapDistance2D);
                const bool bestPreferredGapStart =
                    preferPreferredGapStart
                    && restrictedComponentId >= 0
                    && bestCandidate.index >= 0
                    && static_cast<size_t>(bestCandidate.index) < gapStartContext.componentIds.size()
                    && gapStartContext.componentIds[static_cast<size_t>(bestCandidate.index)] == restrictedComponentId
                    && static_cast<size_t>(restrictedComponentId) < gapStartContext.preferredStartPointsByComponent.size()
                    && isPointNearAnyPreferredStart
                    (
                        bestCandidate.startPoint,
                        gapStartContext.preferredStartPointsByComponent[static_cast<size_t>(restrictedComponentId)],
                        kPreferredStartGapDistance2D
                    );
                QVector3D nextStartPoint;
                const bool hasNextStartPoint = tryFindNearestNextStartPoint
                (
                    sortableItems,
                    visited,
                    strategy,
                    index,
                    option.endPoint,
                    nextStartPoint
                );
                const double nextDistance = hasNextStartPoint
                    ? std::sqrt(planarDistanceSquared(nextStartPoint, option.endPoint))
                    : 0.0;
                const double candidateProgress = static_cast<double>(QVector3D::dotProduct(option.startPoint, normalizedSweepDirection));
                const double backtrackDistance = hasCurrentEndPoint && normalizedSweepDirection.lengthSquared() > kSortEpsilon
                    ? std::max(0.0, referenceProgress - candidateProgress)
                    : 0.0;
                const double continuityPenalty =
                    movementContinuityPenalty(option.startPoint - referencePoint, option.startTangent)
                    + (hasNextStartPoint ? movementContinuityPenalty(nextStartPoint - option.endPoint, option.endTangent) : 0.0);
                const double continuityScale = std::max(1.0, 0.5 * (entryDistance + nextDistance));
                const double optionScore = entryDistance
                    + nextDistance * kNextDistanceWeight
                    + backtrackDistance * kBacktrackPenaltyWeight
                    + continuityScale * kDirectionPenaltyWeight * continuityPenalty;

                const bool shouldReplace = bestCandidate.index < 0
                    || (preferPreferredGapStart && preferredGapStart && !bestPreferredGapStart)
                    || (preferPreferredGapStart
                        && preferredGapStart == bestPreferredGapStart
                        && (entryDistance < bestCandidate.priorityDistance - kSortEpsilon
                            || (std::abs(entryDistance - bestCandidate.priorityDistance) <= kSortEpsilon
                                && (optionScore < bestCandidate.score - kSortEpsilon
                                    || (std::abs(optionScore - bestCandidate.score) <= kSortEpsilon
                                        && isPointLexicographicallyLess(option.startPoint, bestCandidate.startPoint))))))
                    || (nearCurrentGap && !bestNearCurrentGap)
                    || (directlyConnected && !bestDirectlyConnected)
                    || (nearCurrentGap == bestNearCurrentGap
                        && directlyConnected == bestDirectlyConnected
                        && (!preferPreferredGapStart || preferredGapStart == bestPreferredGapStart)
                        && (connectionDistance < bestCandidate.connectionDistance - kSortEpsilon
                            || (std::abs(connectionDistance - bestCandidate.connectionDistance) <= kSortEpsilon
                                && (optionScore < bestCandidate.score - kSortEpsilon
                                    || (std::abs(optionScore - bestCandidate.score) <= kSortEpsilon
                                        && (entryDistance < bestCandidate.priorityDistance - kSortEpsilon
                                            || (std::abs(entryDistance - bestCandidate.priorityDistance) <= kSortEpsilon
                                                && isPointLexicographicallyLess(option.startPoint, bestCandidate.startPoint))))))));

                if (!shouldReplace)
                {
                    continue;
                }

                bestCandidate.index = static_cast<int>(index);
                bestCandidate.reverse = option.reverse;
                bestCandidate.hasCustomStart = option.hasCustomStart;
                bestCandidate.processStartParameter = option.processStartParameter;
                bestCandidate.connectionDistance = connectionDistance;
                bestCandidate.priorityDistance = entryDistance;
                bestCandidate.gapDistance = currentGapDistance;
                bestCandidate.score = optionScore;
                bestCandidate.startPoint = option.startPoint;
                bestCandidate.endPoint = option.endPoint;
            }
        }

        return bestCandidate;
    }

    SortCandidate chooseNext3DSortCandidate
    (
        const std::vector<CadItem*>& sortableItems,
        const std::vector<bool>& visited,
        const std::vector<ProcessConnectionSegment>& processedSegments,
        const GapStartSelectionContext& gapStartContext,
        SortStrategy strategy,
        int currentComponentId,
        int restrictedComponentId,
        bool preferPreferredGapStart,
        bool hasCurrentEndPoint,
        const QVector3D& currentEndPoint,
        const QVector3D& sweepDirection,
        const GProfileRotaryAxisConfig& config,
        bool preferSweepBoundary = false,
        double sweepBoundaryX = 0.0
    )
    {
        SortCandidate bestCandidate;
        const std::vector<bool> visitedComponents = buildVisitedComponentMask(visited, gapStartContext.componentIds);
        const bool mustStayInCurrentComponent = hasCurrentEndPoint
            && hasRemainingUnvisitedInComponent(visited, gapStartContext.componentIds, currentComponentId);
        const QVector3D referencePoint = hasCurrentEndPoint ? currentEndPoint : kRotaryInitialSortOrigin;
        RotarySortPoint referenceRotaryPoint;
        const bool hasReferenceRotaryPoint = tryBuildRotarySortPoint(referencePoint, config, referenceRotaryPoint);
        const bool preferMatchingRotaryPlane = strategy == SortStrategy::Smart
            && hasCurrentEndPoint
            && !mustStayInCurrentComponent
            && hasReferenceRotaryPoint;
        bool hasInitialTopPlane = false;
        double initialTopZ = 0.0;
        double initialTopTolerance = 0.0;

        if (!hasCurrentEndPoint)
        {
            double minimumZ = 0.0;

            for (const CadItem* item : sortableItems)
            {
                if (item == nullptr)
                {
                    continue;
                }

                for (const QVector3D& point : item->m_geometry.vertices)
                {
                    if (!hasInitialTopPlane)
                    {
                        minimumZ = point.z();
                        initialTopZ = point.z();
                        hasInitialTopPlane = true;
                        continue;
                    }

                    minimumZ = std::min(minimumZ, static_cast<double>(point.z()));
                    initialTopZ = std::max(initialTopZ, static_cast<double>(point.z()));
                }
            }

            if (hasInitialTopPlane)
            {
                initialTopTolerance = std::max(0.25, (initialTopZ - minimumZ) * kSquareTubeSectionToleranceRatio);
            }
        }
        const QVector3D normalizedSweepDirection = normalizeOrZero(sweepDirection);
        const double referenceProgress = hasReferenceRotaryPoint
            ? static_cast<double>(referencePoint.x()) * static_cast<double>(normalizedSweepDirection.x())
                + referenceRotaryPoint.angleDegrees * static_cast<double>(normalizedSweepDirection.y())
            : static_cast<double>(referencePoint.x()) * static_cast<double>(normalizedSweepDirection.x());

        for (size_t index = 0; index < sortableItems.size(); ++index)
        {
            if (visited[index])
            {
                continue;
            }

            const std::vector<ProcessPathOption> options = buildPathOptionsForItem(sortableItems[index], strategy);

            for (const ProcessPathOption& option : options)
            {
                const int componentId = index < gapStartContext.componentIds.size()
                    ? gapStartContext.componentIds[index]
                    : -1;

                if (mustStayInCurrentComponent && componentId != currentComponentId)
                {
                    continue;
                }

                if (restrictedComponentId >= 0 && componentId != restrictedComponentId)
                {
                    continue;
                }

                const std::vector<QVector3D> emptyPreferredPoints;
                const std::vector<QVector3D>& componentPreferredPoints =
                    (componentId >= 0 && static_cast<size_t>(componentId) < gapStartContext.preferredStartPointsByComponent.size())
                    ? gapStartContext.preferredStartPointsByComponent[static_cast<size_t>(componentId)]
                    : emptyPreferredPoints;
                const double connectionDistance = computeClosestConnectionDistance3D(processedSegments, option.startPoint, option.endPoint);
                const bool directlyConnected = connectionDistance <= kSortConnectionEpsilon;
                const bool bestDirectlyConnected = bestCandidate.connectionDistance <= kSortConnectionEpsilon;
                const bool startsOnInitialTopPlane = hasInitialTopPlane
                    && std::abs(static_cast<double>(option.startPoint.z()) - initialTopZ) <= initialTopTolerance;
                const bool startsOnSweepBoundary = preferSweepBoundary
                    && std::abs(static_cast<double>(option.startPoint.x()) - sweepBoundaryX) <= kSurfaceSweepBoundaryTolerance;
                RotarySortPoint candidateRotaryPoint;
                const bool matchesCurrentRotaryPlane = preferMatchingRotaryPlane
                    && tryBuildRotarySortPoint(option.startPoint, config, candidateRotaryPoint)
                    && std::abs
                    (
                        unwrapAngleDegrees(referenceRotaryPoint.angleDegrees, candidateRotaryPoint.angleDegrees)
                            - referenceRotaryPoint.angleDegrees
                    ) <= kRotaryPlaneMatchToleranceDegrees;
                double resolvedCandidateAngle = 0.0;
                const double entryDistance = rotarySortTravelDistance(referencePoint, option.startPoint, config, &resolvedCandidateAngle);
                const double currentGapDistance = hasCurrentEndPoint
                    ? std::sqrt(spatialDistanceSquared(option.startPoint, currentEndPoint))
                    : std::sqrt(spatialDistanceSquared(option.startPoint, kRotaryInitialSortOrigin));
                const bool nearCurrentGap = hasCurrentEndPoint && currentGapDistance <= kNearGapPriorityDistance3D;
                const bool bestNearCurrentGap = hasCurrentEndPoint && bestCandidate.gapDistance <= kNearGapPriorityDistance3D;
                const bool preferredGapStart = preferPreferredGapStart
                    && componentId == restrictedComponentId
                    && isPointNearAnyPreferredStart(option.startPoint, componentPreferredPoints, kPreferredStartGapDistance3D);
                const bool bestPreferredGapStart =
                    preferPreferredGapStart
                    && restrictedComponentId >= 0
                    && bestCandidate.index >= 0
                    && static_cast<size_t>(bestCandidate.index) < gapStartContext.componentIds.size()
                    && gapStartContext.componentIds[static_cast<size_t>(bestCandidate.index)] == restrictedComponentId
                    && static_cast<size_t>(restrictedComponentId) < gapStartContext.preferredStartPointsByComponent.size()
                    && isPointNearAnyPreferredStart
                    (
                        bestCandidate.startPoint,
                        gapStartContext.preferredStartPointsByComponent[static_cast<size_t>(restrictedComponentId)],
                        kPreferredStartGapDistance3D
                    );
                QVector3D nextStartPoint;
                const bool hasNextStartPoint = tryFindNearestNextStartPoint3D
                (
                    sortableItems,
                    visited,
                    strategy,
                    index,
                    option.endPoint,
                    config,
                    nextStartPoint
                );
                const double nextDistance = hasNextStartPoint
                    ? rotarySortTravelDistance(option.endPoint, nextStartPoint, config)
                    : 0.0;
                const double candidateProgress = static_cast<double>(option.startPoint.x()) * static_cast<double>(normalizedSweepDirection.x())
                    + resolvedCandidateAngle * static_cast<double>(normalizedSweepDirection.y());
                const double backtrackDistance = hasCurrentEndPoint && normalizedSweepDirection.lengthSquared() > kSortEpsilon
                    ? std::max(0.0, referenceProgress - candidateProgress)
                    : 0.0;
                const double continuityPenalty =
                    rotaryMovementContinuityPenalty(referencePoint, option.startPoint, option.startTangent, config)
                    + (hasNextStartPoint ? rotaryMovementContinuityPenalty(option.endPoint, nextStartPoint, option.endTangent, config) : 0.0);
                const double continuityScale = std::max(1.0, 0.5 * (entryDistance + nextDistance));
                const double optionScore = entryDistance
                    + nextDistance * kRotaryNextDistanceWeight
                    + backtrackDistance * kRotaryBacktrackPenaltyWeight
                    + continuityScale * kRotaryDirectionPenaltyWeight * continuityPenalty;

                const bool rotaryPlanePreferenceDecides = preferMatchingRotaryPlane
                    && matchesCurrentRotaryPlane != bestCandidate.matchesCurrentRotaryPlane;
                const bool initialTopPlanePreferenceDecides = hasInitialTopPlane
                    && startsOnInitialTopPlane != bestCandidate.startsOnInitialTopPlane;
                const bool sweepBoundaryPreferenceDecides = preferSweepBoundary
                    && startsOnSweepBoundary != bestCandidate.startsOnSweepBoundary;
                const bool shouldReplace = bestCandidate.index < 0
                    || (initialTopPlanePreferenceDecides && startsOnInitialTopPlane)
                    || (!initialTopPlanePreferenceDecides
                        && sweepBoundaryPreferenceDecides
                        && startsOnSweepBoundary)
                    || (!initialTopPlanePreferenceDecides
                        && !sweepBoundaryPreferenceDecides
                        && rotaryPlanePreferenceDecides
                        && matchesCurrentRotaryPlane)
                    || (!initialTopPlanePreferenceDecides
                        && !sweepBoundaryPreferenceDecides
                        && !rotaryPlanePreferenceDecides
                        && ((preferPreferredGapStart && preferredGapStart && !bestPreferredGapStart)
                            || (preferPreferredGapStart
                                && preferredGapStart == bestPreferredGapStart
                                && (entryDistance < bestCandidate.priorityDistance - kSortEpsilon
                                    || (std::abs(entryDistance - bestCandidate.priorityDistance) <= kSortEpsilon
                                        && (optionScore < bestCandidate.score - kSortEpsilon
                                            || (std::abs(optionScore - bestCandidate.score) <= kSortEpsilon
                                                && isPointLexicographicallyLess(option.startPoint, bestCandidate.startPoint))))))
                            || (nearCurrentGap && !bestNearCurrentGap)
                            || (directlyConnected && !bestDirectlyConnected)
                            || (nearCurrentGap == bestNearCurrentGap
                                && directlyConnected == bestDirectlyConnected
                                && (!preferPreferredGapStart || preferredGapStart == bestPreferredGapStart)
                                && (connectionDistance < bestCandidate.connectionDistance - kSortEpsilon
                                    || (std::abs(connectionDistance - bestCandidate.connectionDistance) <= kSortEpsilon
                                        && (optionScore < bestCandidate.score - kSortEpsilon
                                            || (std::abs(optionScore - bestCandidate.score) <= kSortEpsilon
                                                && (entryDistance < bestCandidate.priorityDistance - kSortEpsilon
                                                    || (std::abs(entryDistance - bestCandidate.priorityDistance) <= kSortEpsilon
                                                        && isPointLexicographicallyLess(option.startPoint, bestCandidate.startPoint))))))))));

                if (!shouldReplace)
                {
                    continue;
                }

                bestCandidate.index = static_cast<int>(index);
                bestCandidate.reverse = option.reverse;
                bestCandidate.hasCustomStart = option.hasCustomStart;
                bestCandidate.processStartParameter = option.processStartParameter;
                bestCandidate.connectionDistance = connectionDistance;
                bestCandidate.priorityDistance = entryDistance;
                bestCandidate.gapDistance = currentGapDistance;
                bestCandidate.score = optionScore;
                bestCandidate.startsOnInitialTopPlane = startsOnInitialTopPlane;
                bestCandidate.startsOnSweepBoundary = startsOnSweepBoundary;
                bestCandidate.matchesCurrentRotaryPlane = matchesCurrentRotaryPlane;
                bestCandidate.startPoint = option.startPoint;
                bestCandidate.endPoint = option.endPoint;
            }
        }

        return bestCandidate;
    }

    void expandRotaryBounds(RotaryPathBounds& bounds, const RawPathPoint3D& point)
    {
        if (!bounds.valid)
        {
            bounds.valid = true;
            bounds.minX = bounds.maxX = point.x;
            bounds.minY = bounds.maxY = point.y;
            bounds.minZ = bounds.maxZ = point.z;
        }
        else
        {
            bounds.minX = std::min(bounds.minX, point.x);
            bounds.maxX = std::max(bounds.maxX, point.x);
            bounds.minY = std::min(bounds.minY, point.y);
            bounds.maxY = std::max(bounds.maxY, point.y);
            bounds.minZ = std::min(bounds.minZ, point.z);
            bounds.maxZ = std::max(bounds.maxZ, point.z);
        }

        bounds.sumX += point.x;
        bounds.sumY += point.y;
        bounds.sumZ += point.z;
        ++bounds.pointCount;
    }

    void mergeRotaryBounds(RotaryPathBounds& target, const RotaryPathBounds& source)
    {
        if (!source.valid)
        {
            return;
        }

        if (!target.valid)
        {
            target = source;
            return;
        }

        target.minX = std::min(target.minX, source.minX);
        target.maxX = std::max(target.maxX, source.maxX);
        target.minY = std::min(target.minY, source.minY);
        target.maxY = std::max(target.maxY, source.maxY);
        target.minZ = std::min(target.minZ, source.minZ);
        target.maxZ = std::max(target.maxZ, source.maxZ);
        target.sumX += source.sumX;
        target.sumY += source.sumY;
        target.sumZ += source.sumZ;
        target.pointCount += source.pointCount;
    }

    double rotaryBoundsCenterX(const RotaryPathBounds& bounds)
    {
        return bounds.pointCount > 0
            ? bounds.sumX / static_cast<double>(bounds.pointCount)
            : (bounds.minX + bounds.maxX) * 0.5;
    }

    double rotaryBoundsCenterY(const RotaryPathBounds& bounds)
    {
        return bounds.pointCount > 0
            ? bounds.sumY / static_cast<double>(bounds.pointCount)
            : (bounds.minY + bounds.maxY) * 0.5;
    }

    double rotaryBoundsCenterZ(const RotaryPathBounds& bounds)
    {
        return bounds.pointCount > 0
            ? bounds.sumZ / static_cast<double>(bounds.pointCount)
            : (bounds.minZ + bounds.maxZ) * 0.5;
    }

    RotarySurfaceGroup classifySquareTubeSurface
    (
        const CadItem* item,
        const RotaryPathBounds& sectionBounds,
        double sectionTolerance
    )
    {
        if (item == nullptr || !sectionBounds.valid)
        {
            return RotarySurfaceGroup::Unknown;
        }

        bool onlyTop = true;
        bool onlyRight = true;
        bool onlyBottom = true;
        bool onlyLeft = true;
        bool hasPoint = false;

        for (const RawPathPoint3D& point : item->rawPathPoints3D())
        {
            hasPoint = true;
            onlyTop = onlyTop && std::abs(point.z - sectionBounds.maxZ) <= sectionTolerance;
            onlyRight = onlyRight && std::abs(point.y - sectionBounds.maxY) <= sectionTolerance;
            onlyBottom = onlyBottom && std::abs(point.z - sectionBounds.minZ) <= sectionTolerance;
            onlyLeft = onlyLeft && std::abs(point.y - sectionBounds.minY) <= sectionTolerance;
        }

        if (!hasPoint)
        {
            return RotarySurfaceGroup::Unknown;
        }

        if (onlyTop)
        {
            return RotarySurfaceGroup::Top;
        }

        if (onlyRight)
        {
            return RotarySurfaceGroup::Right;
        }

        if (onlyBottom)
        {
            return RotarySurfaceGroup::Bottom;
        }

        if (onlyLeft)
        {
            return RotarySurfaceGroup::Left;
        }

        return RotarySurfaceGroup::Unknown;
    }

    bool componentLiesOnEndCutPlane
    (
        const RotaryFeatureComponent& component,
        const std::vector<CadItem*>& sortableItems,
        double sectionTolerance
    )
    {
        // 方管端部切面可表示为 x = a*y + b*z + c；该形式同时覆盖垂直与倾斜切面。
        double normal[3][4] = {};
        size_t pointCount = 0;

        size_t coplanarPointCount = 0;

        for (const size_t itemIndex : component.itemIndices)
        {
            if (itemIndex >= sortableItems.size() || sortableItems[itemIndex] == nullptr)
            {
                continue;
            }

            for (const RawPathPoint3D& point : sortableItems[itemIndex]->rawPathPoints3D())
            {
                const double values[3] = { point.y, point.z, 1.0 };

                for (int row = 0; row < 3; ++row)
                {
                    for (int column = 0; column < 3; ++column)
                    {
                        normal[row][column] += values[row] * values[column];
                    }

                    normal[row][3] += values[row] * point.x;
                }

                ++pointCount;
            }
        }

        if (pointCount < 3)
        {
            return false;
        }

        for (int pivotColumn = 0; pivotColumn < 3; ++pivotColumn)
        {
            int pivotRow = pivotColumn;

            for (int row = pivotColumn + 1; row < 3; ++row)
            {
                if (std::abs(normal[row][pivotColumn]) > std::abs(normal[pivotRow][pivotColumn]))
                {
                    pivotRow = row;
                }
            }

            if (std::abs(normal[pivotRow][pivotColumn]) <= kSortEpsilon)
            {
                return false;
            }

            if (pivotRow != pivotColumn)
            {
                for (int column = pivotColumn; column < 4; ++column)
                {
                    std::swap(normal[pivotRow][column], normal[pivotColumn][column]);
                }
            }

            const double pivot = normal[pivotColumn][pivotColumn];

            for (int column = pivotColumn; column < 4; ++column)
            {
                normal[pivotColumn][column] /= pivot;
            }

            for (int row = 0; row < 3; ++row)
            {
                if (row == pivotColumn)
                {
                    continue;
                }

                const double factor = normal[row][pivotColumn];

                for (int column = pivotColumn; column < 4; ++column)
                {
                    normal[row][column] -= factor * normal[pivotColumn][column];
                }
            }
        }

        const double planeTolerance = std::max(sectionTolerance * 2.0, 0.5);

        for (const size_t itemIndex : component.itemIndices)
        {
            if (itemIndex >= sortableItems.size() || sortableItems[itemIndex] == nullptr)
            {
                continue;
            }

            for (const RawPathPoint3D& point : sortableItems[itemIndex]->rawPathPoints3D())
            {
                const double expectedX = normal[0][3] * point.y + normal[1][3] * point.z + normal[2][3];

                if (std::abs(point.x - expectedX) > planeTolerance)
                {
                    continue;
                }

                ++coplanarPointCount;
            }
        }

        // 允许少量引线、断点等辅助路径偏离切面，但主体必须仍是同一切面。
        return coplanarPointCount * 4 >= pointCount * 3;
    }

    bool componentMatchesSquareTubeEndCut
    (
        const RotaryFeatureComponent& component,
        const std::vector<CadItem*>& sortableItems,
        const RotaryPathBounds& sectionBounds,
        double sectionTolerance
    )
    {
        if (!component.bounds.valid || !sectionBounds.valid)
        {
            return false;
        }

        const double spanY = sectionBounds.maxY - sectionBounds.minY;
        const double spanZ = sectionBounds.maxZ - sectionBounds.minZ;

        if (spanY <= kSortEpsilon || spanZ <= kSortEpsilon)
        {
            return false;
        }

        const double coverageY = (component.bounds.maxY - component.bounds.minY) / spanY;
        const double coverageZ = (component.bounds.maxZ - component.bounds.minZ) / spanZ;

        if (coverageY < kSquareTubeEndCutCoverageThreshold || coverageZ < kSquareTubeEndCutCoverageThreshold)
        {
            return false;
        }

        if (!componentLiesOnEndCutPlane(component, sortableItems, sectionTolerance))
        {
            return false;
        }

        bool touchesTop = false;
        bool touchesRight = false;
        bool touchesBottom = false;
        bool touchesLeft = false;

        for (const size_t itemIndex : component.itemIndices)
        {
            if (itemIndex >= sortableItems.size() || sortableItems[itemIndex] == nullptr)
            {
                continue;
            }

            for (const RawPathPoint3D& point : sortableItems[itemIndex]->rawPathPoints3D())
            {
                touchesTop = touchesTop || std::abs(point.z - sectionBounds.maxZ) <= sectionTolerance;
                touchesRight = touchesRight || std::abs(point.y - sectionBounds.maxY) <= sectionTolerance;
                touchesBottom = touchesBottom || std::abs(point.z - sectionBounds.minZ) <= sectionTolerance;
                touchesLeft = touchesLeft || std::abs(point.y - sectionBounds.minY) <= sectionTolerance;
            }
        }

        return touchesTop && touchesRight && touchesBottom && touchesLeft;
    }

    std::vector<size_t>& segmentItemsForSurface(RotaryLazySegment& segment, RotarySurfaceGroup surface)
    {
        switch (surface)
        {
        case RotarySurfaceGroup::Top:
            return segment.topItems;
        case RotarySurfaceGroup::Right:
            return segment.rightItems;
        case RotarySurfaceGroup::Bottom:
            return segment.bottomItems;
        case RotarySurfaceGroup::Left:
            return segment.leftItems;
        case RotarySurfaceGroup::Unknown:
        default:
            return segment.unknownItems;
        }
    }

    bool isCompleteCircleOrEllipse(const CadItem* item)
    {
        if (item == nullptr || item->m_nativeEntity == nullptr)
        {
            return false;
        }

        if (item->m_type == DRW::ETYPE::CIRCLE)
        {
            return true;
        }

        if (item->m_type != DRW::ETYPE::ELLIPSE)
        {
            return false;
        }

        const DRW_Ellipse* ellipse = static_cast<const DRW_Ellipse*>(item->m_nativeEntity);
        return isFullEllipsePath(ellipse);
    }

    bool isIndividuallyClosedProcessItem(const CadItem* item)
    {
        if (isCompleteCircleOrEllipse(item))
        {
            return true;
        }

        if (item == nullptr || item->m_nativeEntity == nullptr)
        {
            return false;
        }

        if (item->m_type == DRW::ETYPE::POLYLINE)
        {
            const DRW_Polyline* polyline = static_cast<const DRW_Polyline*>(item->m_nativeEntity);
            return (polyline->flags & 1) != 0;
        }

        if (item->m_type == DRW::ETYPE::LWPOLYLINE)
        {
            const DRW_LWPolyline* polyline = static_cast<const DRW_LWPolyline*>(item->m_nativeEntity);
            return (polyline->flags & 1) != 0;
        }

        if (item->m_type == DRW::ETYPE::SPLINE)
        {
            const DRW_Spline* spline =
                static_cast<const DRW_Spline*>(item->m_nativeEntity);
            return (spline->flags & (1 | 2)) != 0;
        }

        return false;
    }

    bool isClosedProcessGroup
    (
        const std::vector<CadItem*>& sortableItems,
        const std::vector<size_t>& groupIndices
    )
    {
        if (groupIndices.empty())
        {
            return false;
        }

        struct GroupEndpoints
        {
            QVector3D start;
            QVector3D end;
            bool individuallyClosed = false;
        };

        std::vector<GroupEndpoints> endpoints;
        endpoints.reserve(groupIndices.size());

        for (const size_t globalIndex : groupIndices)
        {
            if (globalIndex >= sortableItems.size() || sortableItems[globalIndex] == nullptr)
            {
                return false;
            }

            CadItem* item = sortableItems[globalIndex];
            const std::vector<ProcessPathOption> options =
                buildPathOptionsForItem(item, SortStrategy::KeepDirection);

            if (options.empty())
            {
                return false;
            }

            endpoints.push_back
            ({
                options.front().startPoint,
                options.front().endPoint,
                isIndividuallyClosedProcessItem(item)
            });
        }

        if (endpoints.size() == 1)
        {
            return endpoints.front().individuallyClosed;
        }

        const double toleranceSquared = kEndCutConnectionTolerance * kEndCutConnectionTolerance;
        for (size_t endpointIndex = 0; endpointIndex < endpoints.size(); ++endpointIndex)
        {
            if (endpoints[endpointIndex].individuallyClosed)
            {
                continue;
            }

            for (const QVector3D& point : { endpoints[endpointIndex].start, endpoints[endpointIndex].end })
            {
                bool matched = false;

                for (size_t otherIndex = 0; otherIndex < endpoints.size() && !matched; ++otherIndex)
                {
                    if (endpointIndex == otherIndex)
                    {
                        continue;
                    }

                    matched = spatialDistanceSquared(point, endpoints[otherIndex].start) <= toleranceSquared
                        || spatialDistanceSquared(point, endpoints[otherIndex].end) <= toleranceSquared;
                }

                if (!matched)
                {
                    return false;
                }
            }
        }

        return true;
    }

    void assignClosedComponentGroupIds
    (
        const std::vector<CadItem*>& sortableItems,
        const std::vector<int>& componentIds,
        std::vector<CadEditer::ProcessStateUpdate>& updates
    )
    {
        if (componentIds.size() != sortableItems.size())
        {
            return;
        }

        const int componentCount = componentIds.empty()
            ? 0
            : *std::max_element(componentIds.cbegin(), componentIds.cend()) + 1;
        std::vector<std::vector<size_t>> componentIndices(static_cast<size_t>(std::max(0, componentCount)));

        for (size_t itemIndex = 0; itemIndex < componentIds.size(); ++itemIndex)
        {
            const int componentId = componentIds[itemIndex];
            if (componentId >= 0 && componentId < componentCount)
            {
                componentIndices[static_cast<size_t>(componentId)].push_back(itemIndex);
            }
        }

        QHash<CadItem*, int> groupIdByItem;
        int nextGroupId = 0;

        for (const std::vector<size_t>& indices : componentIndices)
        {
            std::vector<size_t> remainingIndices;
            remainingIndices.reserve(indices.size());

            for (const size_t index : indices)
            {
                if (isIndividuallyClosedProcessItem(sortableItems[index]))
                {
                    groupIdByItem.insert(sortableItems[index], nextGroupId++);
                }
                else
                {
                    remainingIndices.push_back(index);
                }
            }

            if (!isClosedProcessGroup(sortableItems, remainingIndices))
            {
                continue;
            }

            const int groupId = nextGroupId++;
            for (const size_t index : remainingIndices)
            {
                groupIdByItem.insert(sortableItems[index], groupId);
            }
        }

        for (CadEditer::ProcessStateUpdate& update : updates)
        {
            update.continuousGroupId = groupIdByItem.value(update.item, -1);
        }
    }

    bool appendSorted3DGroup
    (
        const std::vector<CadItem*>& sortableItems,
        const std::vector<size_t>& groupIndices,
        const GProfileRotaryAxisConfig& rotaryAxisConfig,
        std::vector<CadEditer::ProcessStateUpdate>& processUpdates,
        std::vector<ProcessConnectionSegment>& processedSegments,
        bool& hasCurrentEndPoint,
        QVector3D& currentEndPoint,
        bool preferSweepBoundary = false,
        double sweepBoundaryX = 0.0,
        int continuousGroupId = -1
    )
    {
        if (groupIndices.empty())
        {
            return true;
        }

        std::vector<CadItem*> localItems;
        std::vector<size_t> localToGlobal;
        localItems.reserve(groupIndices.size());
        localToGlobal.reserve(groupIndices.size());

        for (const size_t index : groupIndices)
        {
            if (index >= sortableItems.size() || sortableItems[index] == nullptr)
            {
                continue;
            }

            localItems.push_back(sortableItems[index]);
            localToGlobal.push_back(index);
        }

        if (localItems.empty())
        {
            return true;
        }

        const QVector3D sweepDirection = computeRotarySweepDirection(localItems, rotaryAxisConfig);
        const GapStartSelectionContext gapStartContext = buildGapStartSelectionContext(localItems, kPreferredStartGapDistance3D);
        std::vector<bool> visited(localItems.size(), false);
        int currentComponentId = -1;

        for (size_t order = 0; order < localItems.size(); ++order)
        {
            SortCandidate bestCandidate = chooseNext3DSortCandidate
            (
                localItems,
                visited,
                processedSegments,
                gapStartContext,
                SortStrategy::Smart,
                currentComponentId,
                -1,
                false,
                hasCurrentEndPoint,
                currentEndPoint,
                sweepDirection,
                rotaryAxisConfig,
                preferSweepBoundary && order == 0,
                sweepBoundaryX
            );

            const std::vector<bool> visitedComponents = buildVisitedComponentMask(visited, gapStartContext.componentIds);
            const int selectedComponentId =
                bestCandidate.index >= 0 && static_cast<size_t>(bestCandidate.index) < gapStartContext.componentIds.size()
                ? gapStartContext.componentIds[static_cast<size_t>(bestCandidate.index)]
                : -1;
            const bool enteringFreshPreferredComponent =
                selectedComponentId >= 0
                && static_cast<size_t>(selectedComponentId) < visitedComponents.size()
                && !visitedComponents[static_cast<size_t>(selectedComponentId)]
                && static_cast<size_t>(selectedComponentId) < gapStartContext.preferredStartPointsByComponent.size()
                && !gapStartContext.preferredStartPointsByComponent[static_cast<size_t>(selectedComponentId)].empty();

            if (enteringFreshPreferredComponent)
            {
                bestCandidate = chooseNext3DSortCandidate
                (
                    localItems,
                    visited,
                    processedSegments,
                    gapStartContext,
                    SortStrategy::Smart,
                    currentComponentId,
                    selectedComponentId,
                    true,
                    hasCurrentEndPoint,
                    currentEndPoint,
                    sweepDirection,
                    rotaryAxisConfig,
                    preferSweepBoundary && order == 0,
                    sweepBoundaryX
                );
            }

            if (bestCandidate.index < 0 || static_cast<size_t>(bestCandidate.index) >= localToGlobal.size())
            {
                return false;
            }

            const size_t localIndex = static_cast<size_t>(bestCandidate.index);
            const size_t globalIndex = localToGlobal[localIndex];
            visited[localIndex] = true;
            processUpdates.push_back
            ({
                sortableItems[globalIndex],
                static_cast<int>(processUpdates.size()),
                bestCandidate.reverse,
                bestCandidate.hasCustomStart,
                bestCandidate.processStartParameter,
                continuousGroupId
            });
            hasCurrentEndPoint = true;
            currentComponentId =
                localIndex < gapStartContext.componentIds.size()
                ? gapStartContext.componentIds[localIndex]
                : -1;
            currentEndPoint = bestCandidate.endPoint;
            processedSegments.push_back({ bestCandidate.startPoint, bestCandidate.endPoint });
        }

        return true;
    }

    bool appendContinuous3DGroup
    (
        const std::vector<CadItem*>& sortableItems,
        const std::vector<size_t>& groupIndices,
        const GProfileRotaryAxisConfig& rotaryAxisConfig,
        std::vector<CadEditer::ProcessStateUpdate>& processUpdates,
        std::vector<ProcessConnectionSegment>& processedSegments,
        bool& hasCurrentEndPoint,
        QVector3D& currentEndPoint,
        int continuousGroupId,
        QString* failureReason
    )
    {
        struct ContinuousItem
        {
            size_t globalIndex = 0;
            std::vector<ProcessPathOption> options;
            ProcessPathOption fixedOption;
        };

        std::vector<ContinuousItem> items;
        items.reserve(groupIndices.size());

        for (const size_t index : groupIndices)
        {
            if (index >= sortableItems.size() || sortableItems[index] == nullptr)
            {
                if (failureReason != nullptr)
                {
                    *failureReason = QStringLiteral("连续加工组包含无效图元索引 %1。").arg(index);
                }
                return false;
            }

            ContinuousItem item;
            item.globalIndex = index;
            item.options = buildPathOptionsForItem(sortableItems[index], SortStrategy::Smart);
            const std::vector<ProcessPathOption> fixedOptions =
                buildPathOptionsForItem(sortableItems[index], SortStrategy::KeepDirection);

            if (item.options.empty() || fixedOptions.empty())
            {
                if (failureReason != nullptr)
                {
                    *failureReason = QStringLiteral("连续加工组图元 %1 没有可用端点。").arg(index);
                }
                return false;
            }

            item.fixedOption = fixedOptions.front();
            items.push_back(std::move(item));
        }

        if (items.empty())
        {
            return true;
        }

        const double connectionToleranceSquared =
            kEndCutConnectionTolerance * kEndCutConnectionTolerance;
        std::vector<std::vector<QVector3D>> looseEndpoints(items.size());

        for (size_t itemIndex = 0; itemIndex < items.size(); ++itemIndex)
        {
            const ProcessPathOption& option = items[itemIndex].fixedOption;
            if (spatialDistanceSquared(option.startPoint, option.endPoint) <= kSortConnectionEpsilon * kSortConnectionEpsilon)
            {
                continue;
            }

            for (const QVector3D& endpoint : { option.startPoint, option.endPoint })
            {
                bool connected = false;

                for (size_t otherIndex = 0; otherIndex < items.size() && !connected; ++otherIndex)
                {
                    if (itemIndex == otherIndex)
                    {
                        continue;
                    }

                    const ProcessPathOption& other = items[otherIndex].fixedOption;
                    connected = spatialDistanceSquared(endpoint, other.startPoint) <= connectionToleranceSquared
                        || spatialDistanceSquared(endpoint, other.endPoint) <= connectionToleranceSquared;
                }

                if (!connected)
                {
                    looseEndpoints[itemIndex].push_back(endpoint);
                }
            }
        }

        const bool hasLooseEndpoint = std::any_of
        (
            looseEndpoints.cbegin(),
            looseEndpoints.cend(),
            [](const std::vector<QVector3D>& endpoints) { return !endpoints.empty(); }
        );
        std::vector<bool> visited(items.size(), false);
        const QVector3D initialReference = hasCurrentEndPoint ? currentEndPoint : kRotaryInitialSortOrigin;
        int selectedItem = -1;
        const ProcessPathOption* selectedOption = nullptr;
        double bestEntryDistance = std::numeric_limits<double>::max();

        for (size_t itemIndex = 0; itemIndex < items.size(); ++itemIndex)
        {
            for (const ProcessPathOption& option : items[itemIndex].options)
            {
                if (hasLooseEndpoint)
                {
                    const bool startsAtLooseEndpoint = std::any_of
                    (
                        looseEndpoints[itemIndex].cbegin(),
                        looseEndpoints[itemIndex].cend(),
                        [&option, connectionToleranceSquared](const QVector3D& point)
                        {
                            return spatialDistanceSquared(option.startPoint, point) <= connectionToleranceSquared;
                        }
                    );

                    if (!startsAtLooseEndpoint)
                    {
                        continue;
                    }
                }

                const double entryDistance = rotarySortTravelDistance
                (
                    initialReference,
                    option.startPoint,
                    rotaryAxisConfig
                );

                if (selectedOption == nullptr
                    || entryDistance < bestEntryDistance - kSortEpsilon
                    || (std::abs(entryDistance - bestEntryDistance) <= kSortEpsilon
                        && isPointLexicographicallyLess(option.startPoint, selectedOption->startPoint)))
                {
                    selectedItem = static_cast<int>(itemIndex);
                    selectedOption = &option;
                    bestEntryDistance = entryDistance;
                }
            }
        }

        for (size_t order = 0; order < items.size(); ++order)
        {
            if (order > 0)
            {
                selectedItem = -1;
                selectedOption = nullptr;
                double bestConnectionDistance = std::numeric_limits<double>::max();

                for (size_t itemIndex = 0; itemIndex < items.size(); ++itemIndex)
                {
                    if (visited[itemIndex])
                    {
                        continue;
                    }

                    for (const ProcessPathOption& option : items[itemIndex].options)
                    {
                        const double connectionDistance = std::sqrt
                        (
                            spatialDistanceSquared(currentEndPoint, option.startPoint)
                        );

                        if (connectionDistance > kEndCutConnectionTolerance + kSortEpsilon)
                        {
                            continue;
                        }

                        if (selectedOption == nullptr
                            || connectionDistance < bestConnectionDistance - kSortEpsilon
                            || (std::abs(connectionDistance - bestConnectionDistance) <= kSortEpsilon
                                && isPointLexicographicallyLess(option.startPoint, selectedOption->startPoint)))
                        {
                            selectedItem = static_cast<int>(itemIndex);
                            selectedOption = &option;
                            bestConnectionDistance = connectionDistance;
                        }
                    }
                }
            }

            if (selectedItem < 0 || selectedOption == nullptr)
            {
                double nearestDistance = std::numeric_limits<double>::max();
                size_t nearestGlobalIndex = 0;

                for (size_t itemIndex = 0; itemIndex < items.size(); ++itemIndex)
                {
                    if (visited[itemIndex])
                    {
                        continue;
                    }

                    for (const ProcessPathOption& option : items[itemIndex].options)
                    {
                        const double distance = std::sqrt(spatialDistanceSquared(currentEndPoint, option.startPoint));
                        if (distance < nearestDistance)
                        {
                            nearestDistance = distance;
                            nearestGlobalIndex = items[itemIndex].globalIndex;
                        }
                    }
                }

                if (failureReason != nullptr)
                {
                    *failureReason = QStringLiteral
                    (
                        "连续加工组在加工序号 %1 后中断：下一图元索引=%2，端点距离=%3 mm，允许距离=%4 mm。"
                    )
                        .arg(processUpdates.size())
                        .arg(nearestGlobalIndex)
                        .arg(nearestDistance, 0, 'f', 6)
                        .arg(kEndCutConnectionTolerance, 0, 'f', 3);
                }
                return false;
            }

            const size_t localIndex = static_cast<size_t>(selectedItem);
            const size_t globalIndex = items[localIndex].globalIndex;
            const double connectionDistance = hasCurrentEndPoint && order > 0
                ? std::sqrt(spatialDistanceSquared(currentEndPoint, selectedOption->startPoint))
                : 0.0;

            if (order > 0 && connectionDistance > kEndCutConnectionTolerance + kSortEpsilon)
            {
                if (failureReason != nullptr)
                {
                    *failureReason = QStringLiteral
                    (
                        "连续加工组校验失败：图元索引=%1，端点距离=%2 mm，加工序号=%3。"
                    )
                        .arg(globalIndex)
                        .arg(connectionDistance, 0, 'f', 6)
                        .arg(processUpdates.size());
                }
                return false;
            }

            processUpdates.push_back
            ({
                sortableItems[globalIndex],
                static_cast<int>(processUpdates.size()),
                selectedOption->reverse,
                selectedOption->hasCustomStart,
                selectedOption->processStartParameter,
                continuousGroupId
            });
            visited[localIndex] = true;
            hasCurrentEndPoint = true;
            currentEndPoint = selectedOption->endPoint;
            processedSegments.push_back({ selectedOption->startPoint, selectedOption->endPoint });
        }

        return true;
    }

    bool tryBuildSquareTubeLazyRotaryProcessUpdates
    (
        const std::vector<CadItem*>& sortableItems,
        const QVector<CadItem*>& documentItems,
        const GProfileRotaryAxisConfig& rotaryAxisConfig,
        std::vector<CadEditer::ProcessStateUpdate>& processUpdates,
        const RotaryTubeSectionModel& configuredSection,
        QString* failureReason
    )
    {
        const auto fail = [&processUpdates, failureReason](const QString& reason)
        {
            processUpdates.clear();
            const bool alreadyReported = failureReason != nullptr && *failureReason == reason;

            if (failureReason != nullptr)
            {
                *failureReason = reason;
            }

            if (!alreadyReported)
            {
                qWarning().noquote() << QStringLiteral("[智能分段]") << reason;
            }

            return false;
        };

        if (failureReason != nullptr)
        {
            failureReason->clear();
        }

        const auto failCurrentOr = [&fail, failureReason](const QString& fallbackReason)
        {
            return fail(failureReason != nullptr && !failureReason->isEmpty()
                ? *failureReason
                : fallbackReason);
        };

        if (sortableItems.size() < 3)
        {
            return fail(QStringLiteral("有效加工图元不足 3 个，无法建立方管加工分段。"));
        }

        std::vector<RotaryItemAnalysis> itemAnalyses(sortableItems.size());
        RotaryPathBounds sectionBounds;
        QHash<CadItem*, size_t> sortableIndexByItem;

        for (size_t index = 0; index < sortableItems.size(); ++index)
        {
            CadItem* item = sortableItems[index];

            if (item == nullptr)
            {
                return fail(QStringLiteral("有效加工图元集合中存在空图元。"));
            }

            sortableIndexByItem.insert(item, index);

            item->rebuildRawPathPoints3D();

            RotaryItemAnalysis& analysis = itemAnalyses[index];
            analysis.itemIndex = index;

            for (const RawPathPoint3D& point : item->rawPathPoints3D())
            {
                expandRotaryBounds(analysis.bounds, point);
                expandRotaryBounds(sectionBounds, point);
            }

            if (!analysis.bounds.valid)
            {
                return fail(QStringLiteral("图元 %1 没有有效三维加工路径。").arg(index + 1));
            }
        }

        const double spanX = sectionBounds.maxX - sectionBounds.minX;
        const double spanY = sectionBounds.maxY - sectionBounds.minY;
        const double spanZ = sectionBounds.maxZ - sectionBounds.minZ;
        const double sectionSize = std::max(spanY, spanZ);

        if (spanX <= kSortEpsilon || sectionSize <= kSortEpsilon)
        {
            return fail(QStringLiteral("加工范围尺寸无效：X跨度=%1，截面尺寸=%2。")
                .arg(spanX, 0, 'f', 3)
                .arg(sectionSize, 0, 'f', 3));
        }

        const double sectionTolerance = std::max(0.25, sectionSize * kSquareTubeSectionToleranceRatio);
        std::map<int, QVector<CadItem*>> boundaryItemsById;
        QSet<CadItem*> boundaryItems;
        QVector<CadItem*> analysisSceneItems;

        for (CadItem* item : documentItems)
        {
            if (item == nullptr)
            {
                continue;
            }

            const bool isBoundary = item->m_rotaryEndCutRole != RotaryEndCutRole::None
                && item->m_rotaryEndCutPairId >= 0;

            if (!item->m_excludedAsInternalGeometry)
            {
                analysisSceneItems.push_back(item);
            }

            if (!isBoundary)
            {
                continue;
            }

            boundaryItems.insert(item);

            if (item->m_rotaryEndCutRole == RotaryEndCutRole::Break)
            {
                boundaryItemsById[item->m_rotaryEndCutPairId].push_back(item);
            }
        }

        const bool useManualBreakBoundaries = !boundaryItemsById.empty();
        std::vector<CadItem*> connectivityItems;
        std::vector<size_t> connectivityToGlobal;

        for (size_t itemIndex = 0; itemIndex < sortableItems.size(); ++itemIndex)
        {
            CadItem* item = sortableItems[itemIndex];

            if (useManualBreakBoundaries && boundaryItems.contains(item))
            {
                continue;
            }

            connectivityItems.push_back(item);
            connectivityToGlobal.push_back(itemIndex);
        }

        const std::vector<int> componentIds = buildItemConnectivityComponents
        (
            connectivityItems,
            kEndCutConnectionTolerance
        );

        if (componentIds.size() != connectivityItems.size())
        {
            return fail(QStringLiteral("普通图元连通分组结果数量不一致：输入 %1，结果 %2。")
                .arg(connectivityItems.size())
                .arg(componentIds.size()));
        }

        const int componentCount = componentIds.empty()
            ? 0
            : (*std::max_element(componentIds.begin(), componentIds.end()) + 1);

        if (componentCount <= 0 && !useManualBreakBoundaries)
        {
            return fail(QStringLiteral("没有可用于加工分段的普通图元连通分量。"));
        }

        std::vector<RotaryFeatureComponent> components(static_cast<size_t>(componentCount));

        for (size_t localIndex = 0; localIndex < connectivityItems.size(); ++localIndex)
        {
            const int componentId = componentIds[localIndex];

            if (componentId < 0 || componentId >= componentCount)
            {
                return fail(QStringLiteral("普通图元 %1 的连通分量编号无效：%2。")
                    .arg(localIndex + 1)
                    .arg(componentId));
            }

            RotaryFeatureComponent& component = components[static_cast<size_t>(componentId)];
            const size_t itemIndex = connectivityToGlobal[localIndex];
            component.itemIndices.push_back(itemIndex);
            mergeRotaryBounds(component.bounds, itemAnalyses[itemIndex].bounds);
        }

        std::vector<bool> componentIsEndCut(components.size(), false);

        std::vector<ManualRotaryBreakBoundary> manualBreakBoundaries;
        manualBreakBoundaries.reserve(boundaryItemsById.size());

        for (const auto& [boundaryId, specifiedItems] : boundaryItemsById)
        {
            if (specifiedItems.isEmpty())
            {
                return fail(QStringLiteral("断面 %1 没有指定图元。").arg(boundaryId + 1));
            }

            ManualRotaryBreakBoundary analyzedBoundary;
            QVector<CadItem*> missingItems;

            for (CadItem* item : specifiedItems)
            {
                const auto sortableIndex = sortableIndexByItem.constFind(item);

                if (sortableIndex == sortableIndexByItem.cend())
                {
                    missingItems.push_back(item);
                    continue;
                }

                analyzedBoundary.itemIndices.push_back(sortableIndex.value());
            }

            if (!missingItems.isEmpty())
            {
                return fail(QStringLiteral("断面 %1 图元缺失：指定 %2 个，可排序 %3 个，缺少 %4。")
                    .arg(boundaryId + 1)
                    .arg(specifiedItems.size())
                    .arg(analyzedBoundary.itemIndices.size())
                    .arg(describeRotaryPathItems(missingItems)));
            }

            QVector<CadItem*> boundarySceneItems = analysisSceneItems;

            for (CadItem* item : specifiedItems)
            {
                if (item != nullptr && !boundarySceneItems.contains(item))
                {
                    boundarySceneItems.push_back(item);
                }
            }

            analyzedBoundary.analysis = RotaryCutBoundaryAnalyzer::analyze
            (
                specifiedItems,
                boundarySceneItems,
                configuredSection,
                kEndCutConnectionTolerance
            );

            if (!analyzedBoundary.analysis.valid)
            {
                return fail(QStringLiteral("断面 %1 重新分析失败：%2；图元 %3。")
                    .arg(boundaryId + 1)
                    .arg(analyzedBoundary.analysis.errorMessage)
                    .arg(describeRotaryPathItems(specifiedItems)));
            }

            const QSet<CadItem*> specifiedItemSet(specifiedItems.cbegin(), specifiedItems.cend());
            const QSet<CadItem*> analyzedItemSet
            (
                analyzedBoundary.analysis.boundaryItems.cbegin(),
                analyzedBoundary.analysis.boundaryItems.cend()
            );
            QVector<CadItem*> missingAnalyzedItems;
            QVector<CadItem*> unexpectedAnalyzedItems;

            for (CadItem* item : specifiedItems)
            {
                if (!analyzedItemSet.contains(item))
                {
                    missingAnalyzedItems.push_back(item);
                }
            }

            for (CadItem* item : analyzedBoundary.analysis.boundaryItems)
            {
                if (!specifiedItemSet.contains(item))
                {
                    unexpectedAnalyzedItems.push_back(item);
                }
            }

            if (!unexpectedAnalyzedItems.isEmpty())
            {
                return fail(QStringLiteral("断面 %1 组中混入普通或其他断面图元：%2。")
                    .arg(boundaryId + 1)
                    .arg(describeRotaryPathItems(unexpectedAnalyzedItems)));
            }

            if (!missingAnalyzedItems.isEmpty())
            {
                return fail(QStringLiteral("断面 %1 重新分析失败：候选 %2 个，原指定 %3 个，缺少 %4。")
                    .arg(boundaryId + 1)
                    .arg(analyzedBoundary.analysis.boundaryItems.size())
                    .arg(specifiedItems.size())
                    .arg(describeRotaryPathItems(missingAnalyzedItems)));
            }

            qInfo().noquote() << QStringLiteral
            (
                "[智能分段] 断面 ID=%1：指定=%2，重新收集=%3，%4，分析=成功，整数绕数=%5。"
            )
                .arg(boundaryId)
                .arg(specifiedItems.size())
                .arg(analyzedBoundary.analysis.boundaryItems.size())
                .arg(describeRotaryPathItems(analyzedBoundary.analysis.boundaryItems))
                .arg(analyzedBoundary.analysis.winding);

            manualBreakBoundaries.push_back(std::move(analyzedBoundary));
        }

        if (!useManualBreakBoundaries)
        {

        for (size_t componentIndex = 0; componentIndex < components.size(); ++componentIndex)
        {
            RotaryFeatureComponent& component = components[componentIndex];
            component.isEndCut = componentMatchesSquareTubeEndCut(component, sortableItems, sectionBounds, sectionTolerance);
            componentIsEndCut[componentIndex] = component.isEndCut;
        }

        std::vector<size_t> componentOrder(components.size());

        for (size_t componentIndex = 0; componentIndex < components.size(); ++componentIndex)
        {
            componentOrder[componentIndex] = componentIndex;
        }

        std::sort
        (
            componentOrder.begin(),
            componentOrder.end(),
            [&components](size_t left, size_t right)
            {
                return rotaryBoundsCenterX(components[left].bounds) < rotaryBoundsCenterX(components[right].bounds);
            }
        );

        // 倾斜切面中，不同侧面的 X 中心可能相差一个甚至多个截面尺寸。
        const double disconnectedCutMaxSpanX = std::max(sectionSize * 3.0, spanX * 0.10);
        const double disconnectedCutClusterTolerance = disconnectedCutMaxSpanX;
        std::vector<size_t> pendingComponentCluster;
        double pendingClusterCenterX = 0.0;

        auto flushPendingComponentCluster = [&]()
        {
            if (pendingComponentCluster.empty())
            {
                return;
            }

            RotaryFeatureComponent aggregateComponent;

            for (const size_t componentIndex : pendingComponentCluster)
            {
                if (componentIndex >= components.size())
                {
                    continue;
                }

                aggregateComponent.itemIndices.insert
                (
                    aggregateComponent.itemIndices.end(),
                    components[componentIndex].itemIndices.begin(),
                    components[componentIndex].itemIndices.end()
                );
                mergeRotaryBounds(aggregateComponent.bounds, components[componentIndex].bounds);
            }

            const double aggregateSpanX = aggregateComponent.bounds.valid
                ? aggregateComponent.bounds.maxX - aggregateComponent.bounds.minX
                : 0.0;

            if (aggregateSpanX <= disconnectedCutMaxSpanX
                && componentMatchesSquareTubeEndCut(aggregateComponent, sortableItems, sectionBounds, sectionTolerance))
            {
                for (const size_t componentIndex : pendingComponentCluster)
                {
                    if (componentIndex < componentIsEndCut.size())
                    {
                        componentIsEndCut[componentIndex] = true;
                        components[componentIndex].isEndCut = true;
                    }
                }
            }
        };

        for (const size_t componentIndex : componentOrder)
        {
            if (!components[componentIndex].bounds.valid)
            {
                continue;
            }

            const double componentCenterX = rotaryBoundsCenterX(components[componentIndex].bounds);

            if (pendingComponentCluster.empty())
            {
                pendingComponentCluster.push_back(componentIndex);
                pendingClusterCenterX = componentCenterX;
                continue;
            }

            if (std::abs(componentCenterX - pendingClusterCenterX) > disconnectedCutClusterTolerance)
            {
                flushPendingComponentCluster();
                pendingComponentCluster.clear();
                pendingComponentCluster.push_back(componentIndex);
                pendingClusterCenterX = componentCenterX;
                continue;
            }

            const size_t oldCount = pendingComponentCluster.size();
            pendingComponentCluster.push_back(componentIndex);
            pendingClusterCenterX =
                (pendingClusterCenterX * static_cast<double>(oldCount) + componentCenterX)
                / static_cast<double>(pendingComponentCluster.size());
        }

        flushPendingComponentCluster();
        }

        for (RotaryItemAnalysis& analysis : itemAnalyses)
        {
            analysis.surface = classifySquareTubeSurface
            (
                sortableItems[analysis.itemIndex],
                sectionBounds,
                sectionTolerance
            );
        }

        std::vector<RotaryCutCluster> cutClusters;
        std::vector<int> manualBoundaryClusters;

        if (useManualBreakBoundaries)
        {
            const auto buildManualCutCluster = [&itemAnalyses](const std::vector<size_t>& itemIndices)
            {
                RotaryCutCluster cluster;
                RotaryPathBounds bounds;

                for (const size_t itemIndex : itemIndices)
                {
                    if (itemIndex < itemAnalyses.size())
                    {
                        cluster.itemIndices.push_back(itemIndex);
                        mergeRotaryBounds(bounds, itemAnalyses[itemIndex].bounds);
                    }
                }

                cluster.centerX = rotaryBoundsCenterX(bounds);
                return cluster;
            };

            for (ManualRotaryBreakBoundary& boundary : manualBreakBoundaries)
            {
                boundary.clusterIndex = static_cast<int>(cutClusters.size());
                cutClusters.push_back(buildManualCutCluster(boundary.itemIndices));
                manualBoundaryClusters.push_back(boundary.clusterIndex);
            }

            std::sort
            (
                manualBoundaryClusters.begin(),
                manualBoundaryClusters.end(),
                [&cutClusters](int left, int right)
                {
                    return cutClusters[static_cast<size_t>(left)].centerX
                        < cutClusters[static_cast<size_t>(right)].centerX;
                }
            );

            const auto analysisForCluster = [&manualBreakBoundaries](int clusterIndex) -> const RotaryCutBoundaryAnalysis*
                {
                    const auto boundary = std::find_if
                    (
                        manualBreakBoundaries.begin(),
                        manualBreakBoundaries.end(),
                        [clusterIndex](const ManualRotaryBreakBoundary& candidate)
                        {
                            return candidate.clusterIndex == clusterIndex;
                        }
                    );
                    return boundary != manualBreakBoundaries.end() ? &boundary->analysis : nullptr;
                };

            for (size_t boundaryIndex = 1; boundaryIndex < manualBoundaryClusters.size(); ++boundaryIndex)
            {
                const RotaryCutBoundaryAnalysis* leftAnalysis = analysisForCluster(manualBoundaryClusters[boundaryIndex - 1]);
                const RotaryCutBoundaryAnalysis* rightAnalysis = analysisForCluster(manualBoundaryClusters[boundaryIndex]);

                if (leftAnalysis == nullptr || rightAnalysis == nullptr)
                {
                    return fail(QStringLiteral("相邻断面分析结果缺失：左序号=%1，右序号=%2。")
                        .arg(boundaryIndex)
                        .arg(boundaryIndex + 1));
                }

                if (RotaryCutBoundaryAnalyzer::boundariesIntersect
                (
                    *leftAnalysis,
                    *rightAnalysis,
                    kEndCutConnectionTolerance
                ))
                {
                    return fail(QStringLiteral("相邻断面相交：断面 %1 与断面 %2 在方管展开边界上发生交叉。")
                        .arg(boundaryIndex)
                        .arg(boundaryIndex + 1));
                }

                const BoundaryOrderDirectionStats rightToLeft = analyzeBoundaryOrderDirection
                (
                    *leftAnalysis,
                    *rightAnalysis,
                    RotaryBoundarySide::After,
                    kEndCutConnectionTolerance,
                    QStringLiteral("相邻断面 %1/%2 右对左").arg(boundaryIndex).arg(boundaryIndex + 1)
                );
                const BoundaryOrderDirectionStats leftToRight = analyzeBoundaryOrderDirection
                (
                    *rightAnalysis,
                    *leftAnalysis,
                    RotaryBoundarySide::Before,
                    kEndCutConnectionTolerance,
                    QStringLiteral("相邻断面 %1/%2 左对右").arg(boundaryIndex).arg(boundaryIndex + 1)
                );
                const double comparisonPerimeter = std::min
                (
                    leftAnalysis->sectionPerimeter,
                    rightAnalysis->sectionPerimeter
                );
                const double maximumReverseSpan = comparisonPerimeter * 0.01;
                const auto directionIsStable = [maximumReverseSpan](const BoundaryOrderDirectionStats& stats)
                {
                    return stats.ambiguousCount == 0
                        && stats.expectedCount > stats.wrongCount
                        && stats.expectedCount > 0
                        && stats.wrongRatio <= 0.02
                        && stats.allWrongPointsTolerable
                        && stats.stateChangeCount <= 2
                        && stats.maximumReverseRunLength <= maximumReverseSpan
                        && stats.abnormalPerimeterSpan <= maximumReverseSpan;
                };
                const bool bidirectionalReverse = rightToLeft.significantReverse
                    && leftToRight.significantReverse;

                if (!directionIsStable(rightToLeft)
                    || !directionIsStable(leftToRight)
                    || bidirectionalReverse)
                {
                    return fail(QStringLiteral
                    (
                        "相邻断面顺序不确定：断面 %1/%2；右对左 After=%3，错误=%4（%5%），OnBoundary=%6，Ambiguous=%7，错误索引=%8，最大连续反向=%9 mm，异常周向跨度=%10 mm，接缝/边界容错=%11，状态切换=%12；左对右 Before=%13，错误=%14（%15%），OnBoundary=%16，Ambiguous=%17，错误索引=%18，最大连续反向=%19 mm，异常周向跨度=%20 mm，接缝/边界容错=%21，状态切换=%22；双向明显反向=%23。"
                    )
                        .arg(boundaryIndex)
                        .arg(boundaryIndex + 1)
                        .arg(rightToLeft.expectedCount)
                        .arg(rightToLeft.wrongCount)
                        .arg(rightToLeft.wrongRatio * 100.0, 0, 'f', 2)
                        .arg(rightToLeft.onBoundaryCount)
                        .arg(rightToLeft.ambiguousCount)
                        .arg(formatDiagnosticIndices(rightToLeft.wrongIndices))
                        .arg(rightToLeft.maximumReverseRunLength, 0, 'f', 6)
                        .arg(rightToLeft.abnormalPerimeterSpan, 0, 'f', 6)
                        .arg(rightToLeft.allWrongPointsTolerable ? QStringLiteral("是") : QStringLiteral("否"))
                        .arg(rightToLeft.stateChangeCount)
                        .arg(leftToRight.expectedCount)
                        .arg(leftToRight.wrongCount)
                        .arg(leftToRight.wrongRatio * 100.0, 0, 'f', 2)
                        .arg(leftToRight.onBoundaryCount)
                        .arg(leftToRight.ambiguousCount)
                        .arg(formatDiagnosticIndices(leftToRight.wrongIndices))
                        .arg(leftToRight.maximumReverseRunLength, 0, 'f', 6)
                        .arg(leftToRight.abnormalPerimeterSpan, 0, 'f', 6)
                        .arg(leftToRight.allWrongPointsTolerable ? QStringLiteral("是") : QStringLiteral("否"))
                        .arg(leftToRight.stateChangeCount)
                        .arg(bidirectionalReverse ? QStringLiteral("是") : QStringLiteral("否")));
                }

                qInfo().noquote() << QStringLiteral
                (
                    "[智能分段] 相邻断面 %1/%2 顺序确认：右对左 After=%3，接缝异常=%4；左对右 Before=%5，接缝异常=%6；边界不相交，异常周向跨度=%7 mm。"
                )
                    .arg(boundaryIndex)
                    .arg(boundaryIndex + 1)
                    .arg(rightToLeft.expectedCount)
                    .arg(rightToLeft.wrongCount)
                    .arg(leftToRight.expectedCount)
                    .arg(leftToRight.wrongCount)
                    .arg(std::max(rightToLeft.abnormalPerimeterSpan, leftToRight.abnormalPerimeterSpan), 0, 'f', 6);
            }
        }
        else
        {
            std::vector<size_t> endCutComponentIndices;
            double maxEndCutSpanX = 0.0;

            for (size_t componentIndex = 0; componentIndex < components.size(); ++componentIndex)
            {
                if (!componentIsEndCut[componentIndex])
                {
                    continue;
                }

                endCutComponentIndices.push_back(componentIndex);
                maxEndCutSpanX = std::max(maxEndCutSpanX, components[componentIndex].bounds.maxX - components[componentIndex].bounds.minX);
            }

            if (endCutComponentIndices.size() < 2)
            {
                return fail(QStringLiteral("自动加工分段失败：只识别到 %1 个端部断面候选，至少需要 2 个。")
                    .arg(endCutComponentIndices.size()));
            }

            std::sort
            (
                endCutComponentIndices.begin(),
                endCutComponentIndices.end(),
                [&components](size_t left, size_t right)
                {
                    return rotaryBoundsCenterX(components[left].bounds) < rotaryBoundsCenterX(components[right].bounds);
                }
            );

            const double clusterTolerance = std::max(1.0, std::min(std::max(2.0, spanX * 0.01), std::max(2.0, maxEndCutSpanX * 0.75 + 1.0)));

            for (const size_t componentIndex : endCutComponentIndices)
            {
                const RotaryFeatureComponent& component = components[componentIndex];
                const double centerX = rotaryBoundsCenterX(component.bounds);

                if (cutClusters.empty() || std::abs(centerX - cutClusters.back().centerX) > clusterTolerance)
                {
                    RotaryCutCluster cluster;
                    cluster.centerX = centerX;
                    cluster.itemIndices = component.itemIndices;
                    cutClusters.push_back(cluster);
                    continue;
                }

                RotaryCutCluster& cluster = cutClusters.back();
                const size_t oldCount = cluster.itemIndices.size();
                cluster.itemIndices.insert(cluster.itemIndices.end(), component.itemIndices.begin(), component.itemIndices.end());
                cluster.centerX =
                    (cluster.centerX * static_cast<double>(oldCount) + centerX * static_cast<double>(component.itemIndices.size()))
                    / static_cast<double>(cluster.itemIndices.size());
            }
        }

        if ((!useManualBreakBoundaries && cutClusters.size() < 2)
            || (useManualBreakBoundaries && cutClusters.empty()))
        {
            return fail(QStringLiteral("断面聚类失败：人工断面=%1，断面组=%2。")
                .arg(useManualBreakBoundaries ? manualBreakBoundaries.size() : 0)
                .arg(cutClusters.size()));
        }

        std::vector<RotaryLazySegment> segments;
        const double boundaryTolerance = std::max(1.0, spanX * 0.001);

        if (useManualBreakBoundaries)
        {
            // 相邻边界之间各形成一段，首段和尾段只含一侧边界。
            for (size_t boundaryIndex = 0; boundaryIndex <= manualBoundaryClusters.size(); ++boundaryIndex)
            {
                RotaryLazySegment segment;
                segment.leftClusterIndex = boundaryIndex > 0
                    ? manualBoundaryClusters[boundaryIndex - 1]
                    : -1;
                segment.rightClusterIndex = boundaryIndex < manualBoundaryClusters.size()
                    ? manualBoundaryClusters[boundaryIndex]
                    : -1;
                segment.centerX = segment.rightClusterIndex >= 0
                    ? cutClusters[static_cast<size_t>(segment.rightClusterIndex)].centerX
                    : cutClusters[static_cast<size_t>(segment.leftClusterIndex)].centerX + boundaryTolerance;
                segments.push_back(segment);
            }
        }

        std::vector<bool> componentRequiresContinuousGroup(components.size(), false);

        for (size_t componentIndex = 0; componentIndex < components.size(); ++componentIndex)
        {
            if (componentIsEndCut[componentIndex])
            {
                continue;
            }

            RotarySurfaceGroup sharedSurface = RotarySurfaceGroup::Unknown;

            for (const size_t itemIndex : components[componentIndex].itemIndices)
            {
                if (itemIndex >= itemAnalyses.size())
                {
                    return fail(QStringLiteral("普通分量 %1 包含无效图元索引 %2。")
                        .arg(componentIndex + 1)
                        .arg(itemIndex));
                }

                const RotarySurfaceGroup surface = itemAnalyses[itemIndex].surface;

                if (surface == RotarySurfaceGroup::Unknown
                    || (sharedSurface != RotarySurfaceGroup::Unknown && surface != sharedSurface))
                {
                    componentRequiresContinuousGroup[componentIndex] = true;
                    break;
                }

                sharedSurface = surface;
            }
        }

        const auto manualBoundaryAnalysis = [&manualBreakBoundaries](int clusterIndex) -> const RotaryCutBoundaryAnalysis*
            {
                const auto boundary = std::find_if
                (
                    manualBreakBoundaries.begin(),
                    manualBreakBoundaries.end(),
                    [clusterIndex](const ManualRotaryBreakBoundary& candidate)
                    {
                        return candidate.clusterIndex == clusterIndex;
                    }
                );
                return boundary != manualBreakBoundaries.end() ? &boundary->analysis : nullptr;
            };

        const auto assignManualComponentToSegment =
            [&](size_t componentIndex) -> bool
            {
                if (componentIndex >= components.size())
                {
                    return fail(QStringLiteral("普通分量索引无效：%1。").arg(componentIndex + 1));
                }

                const RotaryFeatureComponent& component = components[componentIndex];
                QVector<CadItem*> componentItems;
                int leftClusterIndex = -1;
                int rightClusterIndex = -1;

                for (size_t boundaryIndex = 0; boundaryIndex < manualBoundaryClusters.size(); ++boundaryIndex)
                {
                    const int boundaryClusterIndex = manualBoundaryClusters[boundaryIndex];
                    const RotaryCutBoundaryAnalysis* boundaryAnalysis = manualBoundaryAnalysis(boundaryClusterIndex);

                    if (boundaryAnalysis == nullptr)
                    {
                        return fail(QStringLiteral("断面 %1 的分析结果缺失。")
                            .arg(boundaryIndex + 1));
                    }

                    int beforeCount = 0;
                    int afterCount = 0;
                    int onBoundaryCount = 0;
                    int ambiguousCount = 0;

                    for (const size_t itemIndex : component.itemIndices)
                    {
                        if (itemIndex >= sortableItems.size() || sortableItems[itemIndex] == nullptr)
                        {
                            return fail(QStringLiteral("普通分量 %1 中存在无效图元索引 %2。")
                                .arg(componentIndex + 1)
                                .arg(itemIndex));
                        }

                        CadItem* item = sortableItems[itemIndex];
                        if (!componentItems.contains(item))
                        {
                            componentItems.push_back(item);
                        }
                        item->rebuildRawPathPoints3D();

                        if (item->rawPathPoints3D().empty())
                        {
                            return fail(QStringLiteral("普通分量 %1 的图元没有可用于侧别判断的路径点：%2。")
                                .arg(componentIndex + 1)
                                .arg(describeRotaryPathItems(QVector<CadItem*>{ item })));
                        }

                        for (const RawPathPoint3D& rawPoint : item->rawPathPoints3D())
                        {
                            const RotaryBoundarySide side = RotaryCutBoundaryAnalyzer::classifyPointRelativeToBoundary
                            (
                                *boundaryAnalysis,
                                QVector3D
                                (
                                    static_cast<float>(rawPoint.x),
                                    static_cast<float>(rawPoint.y),
                                    static_cast<float>(rawPoint.z)
                                ),
                                boundaryTolerance
                            );

                            beforeCount += side == RotaryBoundarySide::Before ? 1 : 0;
                            afterCount += side == RotaryBoundarySide::After ? 1 : 0;
                            onBoundaryCount += side == RotaryBoundarySide::OnBoundary ? 1 : 0;
                            ambiguousCount += side == RotaryBoundarySide::Ambiguous ? 1 : 0;
                        }
                    }

                    qInfo().noquote() << QStringLiteral
                    (
                        "[智能分段] 普通分量 %1 / 断面 %2：Before=%3 After=%4 OnBoundary=%5 Ambiguous=%6，图元=%7。"
                    )
                        .arg(componentIndex + 1)
                        .arg(boundaryIndex + 1)
                        .arg(beforeCount)
                        .arg(afterCount)
                        .arg(onBoundaryCount)
                        .arg(ambiguousCount)
                        .arg(describeRotaryPathItems(componentItems));

                    if (ambiguousCount > 0)
                    {
                        return fail(QStringLiteral
                        (
                            "普通分量 %1 的图元相对断面 %2 侧别不确定：Before=%3 After=%4 OnBoundary=%5 Ambiguous=%6，%7。"
                        )
                            .arg(componentIndex + 1)
                            .arg(boundaryIndex + 1)
                            .arg(beforeCount)
                            .arg(afterCount)
                            .arg(onBoundaryCount)
                            .arg(ambiguousCount)
                            .arg(describeRotaryPathItems(componentItems)));
                    }

                    if (beforeCount > 0 && afterCount > 0)
                    {
                        return fail(QStringLiteral
                        (
                            "普通分量 %1 真正横跨断面 %2：Before=%3 After=%4 OnBoundary=%5 Ambiguous=0，%6。"
                        )
                            .arg(componentIndex + 1)
                            .arg(boundaryIndex + 1)
                            .arg(beforeCount)
                            .arg(afterCount)
                            .arg(onBoundaryCount)
                            .arg(describeRotaryPathItems(componentItems)));
                    }

                    bool itemIsBefore = beforeCount > 0;
                    bool itemIsAfter = afterCount > 0;

                    if (!itemIsBefore && !itemIsAfter)
                    {
                        const double itemCenterX = rotaryBoundsCenterX(component.bounds);
                        const double boundaryCenterX = cutClusters[static_cast<size_t>(boundaryClusterIndex)].centerX;
                        itemIsBefore = itemCenterX <= boundaryCenterX;
                        itemIsAfter = !itemIsBefore;
                    }

                    if (!itemIsAfter)
                    {
                        leftClusterIndex = boundaryIndex > 0
                            ? manualBoundaryClusters[boundaryIndex - 1]
                            : -1;
                        rightClusterIndex = boundaryClusterIndex;
                        break;
                    }
                }

                if (rightClusterIndex < 0)
                {
                    leftClusterIndex = manualBoundaryClusters.back();
                }

                const auto segment = std::find_if
                (
                    segments.begin(),
                    segments.end(),
                    [leftClusterIndex, rightClusterIndex](const RotaryLazySegment& candidate)
                    {
                        return candidate.leftClusterIndex == leftClusterIndex
                            && candidate.rightClusterIndex == rightClusterIndex;
                    }
                );

                if (segment == segments.end())
                {
                    return fail(QStringLiteral("普通分量 %1 无法匹配加工区间：左断面=%2，右断面=%3。")
                        .arg(componentIndex + 1)
                        .arg(leftClusterIndex)
                        .arg(rightClusterIndex));
                }

                if (componentRequiresContinuousGroup[componentIndex])
                {
                    segment->continuousGroups.push_back(component.itemIndices);
                }
                else
                {
                    for (const size_t itemIndex : component.itemIndices)
                    {
                        segmentItemsForSurface(*segment, itemAnalyses[itemIndex].surface).push_back(itemIndex);
                    }
                }

                return true;
            };

        for (size_t componentIndex = 0; componentIndex < components.size(); ++componentIndex)
        {
            if (componentIsEndCut[componentIndex])
            {
                continue;
            }

            const RotaryFeatureComponent& component = components[componentIndex];
            const double componentCenterX = rotaryBoundsCenterX(component.bounds);
            int leftClusterIndex = -1;
            int rightClusterIndex = -1;

            if (useManualBreakBoundaries)
            {
                if (!assignManualComponentToSegment(componentIndex))
                {
                    return failCurrentOr(QStringLiteral("普通分量 %1 的图元分段失败。")
                        .arg(componentIndex + 1));
                }

                continue;
            }
            else
            {
                for (size_t clusterIndex = 0; clusterIndex < cutClusters.size(); ++clusterIndex)
                {
                    const double clusterCenterX = cutClusters[clusterIndex].centerX;

                    if (clusterCenterX < componentCenterX - boundaryTolerance)
                    {
                        leftClusterIndex = static_cast<int>(clusterIndex);
                        continue;
                    }

                    if (clusterCenterX > componentCenterX + boundaryTolerance)
                    {
                        rightClusterIndex = static_cast<int>(clusterIndex);
                        break;
                    }
                }
            }

            if ((!useManualBreakBoundaries
                    && (leftClusterIndex < 0 || rightClusterIndex < 0 || leftClusterIndex == rightClusterIndex))
                || (useManualBreakBoundaries && leftClusterIndex < 0 && rightClusterIndex < 0))
            {
                return fail(QStringLiteral("普通分量 %1 无法确定相邻加工断面：左=%2，右=%3。")
                    .arg(componentIndex + 1)
                    .arg(leftClusterIndex)
                    .arg(rightClusterIndex));
            }

            auto existingSegment = std::find_if
            (
                segments.begin(),
                segments.end(),
                [leftClusterIndex, rightClusterIndex](const RotaryLazySegment& segment)
                {
                    return segment.leftClusterIndex == leftClusterIndex && segment.rightClusterIndex == rightClusterIndex;
                }
            );

            if (existingSegment == segments.end())
            {
                RotaryLazySegment segment;
                segment.leftClusterIndex = leftClusterIndex;
                segment.rightClusterIndex = rightClusterIndex;
                segment.centerX = (cutClusters[static_cast<size_t>(leftClusterIndex)].centerX + cutClusters[static_cast<size_t>(rightClusterIndex)].centerX) * 0.5;
                segments.push_back(segment);
                existingSegment = std::prev(segments.end());
            }

            if (componentRequiresContinuousGroup[componentIndex])
            {
                existingSegment->continuousGroups.push_back(component.itemIndices);
                continue;
            }

            for (const size_t itemIndex : component.itemIndices)
            {
                if (itemIndex >= itemAnalyses.size())
                {
                    return fail(QStringLiteral("普通分量 %1 包含无效图元索引 %2。")
                        .arg(componentIndex + 1)
                        .arg(itemIndex));
                }

                segmentItemsForSurface(*existingSegment, itemAnalyses[itemIndex].surface).push_back(itemIndex);
            }
        }

        if (segments.empty())
        {
            return fail(QStringLiteral("没有生成任何加工区间。"));
        }

        std::sort
        (
            segments.begin(),
            segments.end(),
            [](const RotaryLazySegment& left, const RotaryLazySegment& right)
            {
                return left.centerX < right.centerX;
            }
        );

        processUpdates.clear();
        processUpdates.reserve(sortableItems.size());
        std::vector<ProcessConnectionSegment> processedSegments;
        processedSegments.reserve(sortableItems.size());
        std::vector<bool> scheduled(sortableItems.size(), false);
        int nextContinuousGroupId = 0;
        bool hasCurrentEndPoint = false;
        QVector3D currentEndPoint;

        auto appendGroup = [&](const std::vector<size_t>& rawIndices) -> bool
        {
            std::vector<size_t> filteredIndices;
            filteredIndices.reserve(rawIndices.size());

            for (const size_t index : rawIndices)
            {
                if (index < sortableItems.size() && !scheduled[index])
                {
                    filteredIndices.push_back(index);
                }
            }

            if (filteredIndices.empty())
            {
                return true;
            }

            if (!appendSorted3DGroup(sortableItems, filteredIndices, rotaryAxisConfig, processUpdates, processedSegments, hasCurrentEndPoint, currentEndPoint))
            {
                QVector<CadItem*> failedItems;

                for (const size_t index : filteredIndices)
                {
                    if (index < sortableItems.size())
                    {
                        failedItems.push_back(sortableItems[index]);
                    }
                }

                return fail(QStringLiteral("加工组排序失败：%1。")
                    .arg(describeRotaryPathItems(failedItems)));
            }

            for (const size_t index : filteredIndices)
            {
                scheduled[index] = true;
            }

            return true;
        };

        auto appendContinuousGroup = [&](const std::vector<size_t>& rawIndices) -> bool
        {
            std::vector<size_t> filteredIndices;
            filteredIndices.reserve(rawIndices.size());

            for (const size_t index : rawIndices)
            {
                if (index < sortableItems.size() && !scheduled[index])
                {
                    filteredIndices.push_back(index);
                }
            }

            if (filteredIndices.empty())
            {
                return true;
            }

            QString continuousFailureReason;
            const int continuousGroupId = isClosedProcessGroup(sortableItems, filteredIndices)
                ? nextContinuousGroupId++
                : -1;
            if (!appendContinuous3DGroup
            (
                sortableItems,
                filteredIndices,
                rotaryAxisConfig,
                processUpdates,
                processedSegments,
                hasCurrentEndPoint,
                currentEndPoint,
                continuousGroupId,
                &continuousFailureReason
            ))
            {
                QVector<CadItem*> failedItems;
                for (const size_t index : filteredIndices)
                {
                    failedItems.push_back(sortableItems[index]);
                }

                return fail(QStringLiteral("连续加工组排序失败：%1；%2。")
                    .arg(continuousFailureReason)
                    .arg(describeRotaryPathItems(failedItems)));
            }

            for (const size_t index : filteredIndices)
            {
                scheduled[index] = true;
            }
            return true;
        };

        const auto appendContinuousGroups = [&appendContinuousGroup]
            (const std::vector<std::vector<size_t>>& groups) -> bool
            {
                for (const std::vector<size_t>& group : groups)
                {
                    if (!appendContinuousGroup(group))
                    {
                        return false;
                    }
                }
                return true;
            };

        auto appendSurfaceSweepGroup =
            [&](const std::vector<size_t>& rawIndices, bool leftToRight) -> bool
            {
                std::vector<CadItem*> localItems;
                std::vector<size_t> localToGlobal;
                localItems.reserve(rawIndices.size());
                localToGlobal.reserve(rawIndices.size());

                for (const size_t index : rawIndices)
                {
                    if (index < sortableItems.size()
                        && !scheduled[index]
                        && sortableItems[index] != nullptr)
                    {
                        localItems.push_back(sortableItems[index]);
                        localToGlobal.push_back(index);
                    }
                }

                if (localItems.empty())
                {
                    return true;
                }

                const std::vector<int> componentIds = buildItemConnectivityComponents(localItems);
                if (componentIds.size() != localItems.size())
                {
                    return fail(QStringLiteral("平面加工组连通分组数量不一致：输入 %1，结果 %2。")
                        .arg(localItems.size())
                        .arg(componentIds.size()));
                }

                const int componentCount = componentIds.empty()
                    ? 0
                    : *std::max_element(componentIds.begin(), componentIds.end()) + 1;
                if (componentCount <= 0)
                {
                    return fail(QStringLiteral("平面加工组没有有效连通分量。"));
                }

                std::vector<std::vector<size_t>> componentIndices(static_cast<size_t>(componentCount));

                for (size_t localIndex = 0; localIndex < localToGlobal.size(); ++localIndex)
                {
                    const int componentId = componentIds[localIndex];
                    if (componentId < 0 || componentId >= componentCount)
                    {
                        return fail(QStringLiteral("平面加工组图元 %1 的连通分量编号无效：%2。")
                            .arg(localIndex + 1)
                            .arg(componentId));
                    }

                    componentIndices[static_cast<size_t>(componentId)].push_back(localToGlobal[localIndex]);
                }

                std::vector<std::vector<size_t>> separatedComponents;
                separatedComponents.reserve(componentIndices.size());
                for (const std::vector<size_t>& component : componentIndices)
                {
                    std::vector<size_t> remainingIndices;
                    remainingIndices.reserve(component.size());

                    for (const size_t itemIndex : component)
                    {
                        if (isIndividuallyClosedProcessItem(sortableItems[itemIndex]))
                        {
                            separatedComponents.push_back({ itemIndex });
                        }
                        else
                        {
                            remainingIndices.push_back(itemIndex);
                        }
                    }

                    if (!remainingIndices.empty())
                    {
                        separatedComponents.push_back(std::move(remainingIndices));
                    }
                }
                componentIndices = std::move(separatedComponents);

                const auto componentSweepBoundaryX = [&sortableItems, leftToRight](const std::vector<size_t>& indices)
                    {
                        bool hasPoint = false;
                        double boundaryX = 0.0;

                        for (const size_t index : indices)
                        {
                            if (index >= sortableItems.size() || sortableItems[index] == nullptr)
                            {
                                continue;
                            }

                            for (const QVector3D& point : sortableItems[index]->m_geometry.vertices)
                            {
                                if (!hasPoint)
                                {
                                    boundaryX = point.x();
                                    hasPoint = true;
                                    continue;
                                }

                                boundaryX = leftToRight
                                    ? std::min(boundaryX, static_cast<double>(point.x()))
                                    : std::max(boundaryX, static_cast<double>(point.x()));
                            }
                        }

                        return boundaryX;
                    };

                std::sort
                (
                    componentIndices.begin(),
                    componentIndices.end(),
                    [&componentSweepBoundaryX, leftToRight](const std::vector<size_t>& left, const std::vector<size_t>& right)
                    {
                        const double leftBoundaryX = componentSweepBoundaryX(left);
                        const double rightBoundaryX = componentSweepBoundaryX(right);

                        if (std::abs(leftBoundaryX - rightBoundaryX) <= kSortEpsilon)
                        {
                            return left < right;
                        }

                        return leftToRight ? leftBoundaryX < rightBoundaryX : leftBoundaryX > rightBoundaryX;
                    }
                );

                for (const std::vector<size_t>& component : componentIndices)
                {
                    const double sweepBoundaryX = componentSweepBoundaryX(component);
                    const int continuousGroupId = isClosedProcessGroup(sortableItems, component)
                        ? nextContinuousGroupId++
                        : -1;

                    if (component.size() == 1)
                    {
                        const size_t itemIndex = component.front();
                        const std::vector<ProcessPathOption> options = buildPathOptionsForItem
                        (
                            sortableItems[itemIndex],
                            SortStrategy::Smart
                        );

                        if (options.empty())
                        {
                            return fail(QStringLiteral("加工组排序失败：图元没有可用走刀方向，%1。")
                                .arg(describeRotaryPathItems(QVector<CadItem*>{ sortableItems[itemIndex] })));
                        }

                        const QVector3D referencePoint = hasCurrentEndPoint
                            ? currentEndPoint
                            : kRotaryInitialSortOrigin;
                        const QVector3D sweepDirection3D
                        (
                            leftToRight ? 1.0f : -1.0f,
                            0.0f,
                            0.0f
                        );
                        const bool preferSweepTangent = isCompleteCircleOrEllipse(sortableItems[itemIndex])
                            && std::any_of
                            (
                                options.cbegin(),
                                options.cend(),
                                [&sweepDirection3D](const ProcessPathOption& option)
                                {
                                    return std::abs(QVector3D::dotProduct(option.startTangent, sweepDirection3D))
                                        > kSurfaceSweepBoundaryTolerance;
                                }
                            );
                        const ProcessPathOption* selectedOption = nullptr;
                        double bestBoundaryDistance = std::numeric_limits<double>::max();
                        double bestTravelDistance = std::numeric_limits<double>::max();
                        double bestSweepTangentDot = -std::numeric_limits<double>::max();

                        for (const ProcessPathOption& option : options)
                        {
                            const double boundaryDistance = std::abs
                            (
                                static_cast<double>(option.startPoint.x()) - sweepBoundaryX
                            );
                            const double travelDistance = rotarySortTravelDistance
                            (
                                referencePoint,
                                option.startPoint,
                                rotaryAxisConfig
                            );
                            const double sweepTangentDot = QVector3D::dotProduct
                            (
                                option.startTangent,
                                sweepDirection3D
                            );

                            bool shouldReplace = selectedOption == nullptr;
                            if (!shouldReplace)
                            {
                                const bool betterSweepTangent = preferSweepTangent
                                    && sweepTangentDot > bestSweepTangentDot + kSortEpsilon;
                                const bool equivalentSweepTangent = !preferSweepTangent
                                    || std::abs(sweepTangentDot - bestSweepTangentDot) <= kSortEpsilon;
                                const bool betterFallback = boundaryDistance
                                        < bestBoundaryDistance - kSurfaceSweepBoundaryTolerance
                                    || (std::abs(boundaryDistance - bestBoundaryDistance)
                                            <= kSurfaceSweepBoundaryTolerance
                                        && (travelDistance < bestTravelDistance - kSortEpsilon
                                            || (std::abs(travelDistance - bestTravelDistance) <= kSortEpsilon
                                                && isPointLexicographicallyLess
                                                (
                                                    option.startPoint,
                                                    selectedOption->startPoint
                                                ))));
                                shouldReplace = betterSweepTangent
                                    || (equivalentSweepTangent && betterFallback);
                            }

                            if (shouldReplace)
                            {
                                selectedOption = &option;
                                bestBoundaryDistance = boundaryDistance;
                                bestTravelDistance = travelDistance;
                                bestSweepTangentDot = sweepTangentDot;
                            }
                        }

                        if (selectedOption == nullptr)
                        {
                            return fail(QStringLiteral("加工组排序失败：无法为图元选择走刀方向，%1。")
                                .arg(describeRotaryPathItems(QVector<CadItem*>{ sortableItems[itemIndex] })));
                        }

                        processUpdates.push_back
                        ({
                            sortableItems[itemIndex],
                            static_cast<int>(processUpdates.size()),
                            selectedOption->reverse,
                            selectedOption->hasCustomStart,
                            selectedOption->processStartParameter,
                            continuousGroupId
                        });
                        hasCurrentEndPoint = true;
                        currentEndPoint = selectedOption->endPoint;
                        processedSegments.push_back({ selectedOption->startPoint, selectedOption->endPoint });
                        scheduled[itemIndex] = true;
                        continue;
                    }

                    if (!appendSorted3DGroup
                    (
                        sortableItems,
                        component,
                        rotaryAxisConfig,
                        processUpdates,
                        processedSegments,
                        hasCurrentEndPoint,
                        currentEndPoint,
                        true,
                        sweepBoundaryX,
                        continuousGroupId
                    ))
                    {
                        QVector<CadItem*> failedItems;

                        for (const size_t index : component)
                        {
                            if (index < sortableItems.size())
                            {
                                failedItems.push_back(sortableItems[index]);
                            }
                        }

                        return fail(QStringLiteral("加工组排序失败：%1。")
                            .arg(describeRotaryPathItems(failedItems)));
                    }

                    for (const size_t index : component)
                    {
                        scheduled[index] = true;
                    }
                }

                return true;
            };

        for (const RotaryLazySegment& segment : segments)
        {
            if ((!useManualBreakBoundaries
                    && (segment.leftClusterIndex < 0 || segment.rightClusterIndex < 0))
                || (segment.leftClusterIndex >= 0
                    && static_cast<size_t>(segment.leftClusterIndex) >= cutClusters.size())
                || (segment.rightClusterIndex >= 0
                    && static_cast<size_t>(segment.rightClusterIndex) >= cutClusters.size()))
            {
                return fail(QStringLiteral("加工区间引用了无效断面组：左=%1，右=%2，断面组总数=%3。")
                    .arg(segment.leftClusterIndex)
                    .arg(segment.rightClusterIndex)
                    .arg(cutClusters.size()));
            }

            if ((segment.leftClusterIndex >= 0
                    && !appendContinuousGroup(cutClusters[static_cast<size_t>(segment.leftClusterIndex)].itemIndices))
                || !appendContinuousGroups(segment.continuousGroups)
                // S-shaped machining sweep: top L->R, right R->L, bottom L->R, left R->L.
                || !appendSurfaceSweepGroup(segment.topItems, true)
                || !appendSurfaceSweepGroup(segment.rightItems, false)
                || !appendSurfaceSweepGroup(segment.bottomItems, true)
                || !appendSurfaceSweepGroup(segment.leftItems, false)
                || !appendGroup(segment.unknownItems)
                || (segment.rightClusterIndex >= 0
                    && !appendContinuousGroup(cutClusters[static_cast<size_t>(segment.rightClusterIndex)].itemIndices)))
            {
                return failCurrentOr(QStringLiteral("加工组排序失败：区间左断面=%1，右断面=%2。")
                    .arg(segment.leftClusterIndex)
                    .arg(segment.rightClusterIndex));
            }
        }

        QHash<CadItem*, int> updateCounts;

        for (const CadEditer::ProcessStateUpdate& update : processUpdates)
        {
            updateCounts[update.item] += 1;
        }

        QVector<CadItem*> missingItems;
        QVector<CadItem*> duplicateItems;
        size_t sortableBoundaryCount = 0;
        size_t sortableOrdinaryCount = 0;

        for (CadItem* item : sortableItems)
        {
            if (item != nullptr
                && item->m_rotaryEndCutRole != RotaryEndCutRole::None
                && item->m_rotaryEndCutPairId >= 0)
            {
                ++sortableBoundaryCount;
            }
            else
            {
                ++sortableOrdinaryCount;
            }

            const int count = updateCounts.value(item, 0);

            if (count == 0)
            {
                missingItems.push_back(item);
            }
            else if (count > 1)
            {
                duplicateItems.push_back(item);
            }
        }

        if (processUpdates.size() != sortableItems.size()
            || !missingItems.isEmpty()
            || !duplicateItems.isEmpty())
        {
            return fail(QStringLiteral
            (
                "计划图元数量不一致：应生成=%1，已生成=%2，断面图元=%3，普通图元=%4，缺失=%5，重复=%6。"
            )
                .arg(sortableItems.size())
                .arg(processUpdates.size())
                .arg(sortableBoundaryCount)
                .arg(sortableOrdinaryCount)
                .arg(describeRotaryPathItems(missingItems))
                .arg(describeRotaryPathItems(duplicateItems)));
        }

        return true;
    }

    int nextProcessOrder(const CadDocument& document)
    {
        int maxOrder = -1;

        for (const std::unique_ptr<CadItem>& entity : document.m_entities)
        {
            if (entity != nullptr)
            {
                maxOrder = std::max(maxOrder, entity->m_processOrder);
            }
        }

        return maxOrder + 1;
    }
}

bool Gcode_postprocessing_system::sortEntitiesByCurrentMode(bool smartSort)
{
    const int excludedCount = refreshWasteProcessingExclusions();

    if (excludedCount > 0)
    {
        ui->openGLWidget->appendCommandMessage
        (
            QStringLiteral("废面规则已排除 %1 个图元，本次排序不会处理这些图元。").arg(excludedCount)
        );
    }

    const GGenerator::GenerationMode generationMode = resolveGenerationMode();
    return smartSort
        ? (generationMode == GGenerator::GenerationMode::Mode3D
            ? smartSortEntities3D()
            : smartSortEntities())
        : (generationMode == GGenerator::GenerationMode::Mode3D
            ? sortEntitiesByCurrentDirection3D()
            : sortEntitiesByCurrentDirection());
}

QVector<CadItem*> Gcode_postprocessing_system::expandedSelectedRotaryEndCut(QString* errorMessage) const
{
    const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();

    if (selectedItems.isEmpty())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("请先选中加工断面中的一个或部分图元。系统会自动扩展相连图元。");
        }

        return {};
    }

    QVector<CadItem*> sceneItems;
    sceneItems.reserve(static_cast<qsizetype>(m_document.m_entities.size()));

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity != nullptr
            && (!entity->m_excludedAsInternalGeometry || selectedItems.contains(entity.get())))
        {
            sceneItems.push_back(entity.get());
        }
    }

    qInfo().noquote() << QStringLiteral("[断面候选] 选择集：%1")
        .arg(describeRotaryPathItems(selectedItems));
    qInfo().noquote() << QStringLiteral("[断面候选] sceneItems 过滤后：%1")
        .arg(describeRotaryPathItems(sceneItems));
    const RotaryPathTopology topology
    (
        sceneItems,
        RotaryPathTopologyTolerance::fromConnectionTolerance(kEndCutConnectionTolerance)
    );
    QVector<CadItem*> connectedItems;
    const RotaryPathLoopResult loop = topology.extractSeededLoop(selectedItems, &connectedItems);
    qInfo().noquote() << QStringLiteral("[断面候选] 连通扩展：%1")
        .arg(describeRotaryPathItems(connectedItems));

    if (!loop.valid || loop.usedItems.isEmpty())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = loop.errorMessage;
        }

        return {};
    }

    const RotaryCutBoundaryAnalysis analysis = RotaryCutBoundaryAnalyzer::analyze
    (
        loop.usedItems,
        sceneItems,
        m_rotaryTubeSectionModel,
        kEndCutConnectionTolerance
    );

    if (!analysis.valid)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = analysis.errorMessage;
        }

        return {};
    }

    if (errorMessage != nullptr)
    {
        errorMessage->clear();
    }

    return analysis.boundaryItems.isEmpty() ? loop.usedItems : analysis.boundaryItems;
}

bool Gcode_postprocessing_system::assignSelectedRotaryEndCut()
{
    QString selectionError;
    const QVector<CadItem*> expandedItems = expandedSelectedRotaryEndCut(&selectionError);

    if (expandedItems.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("指定加工断面"), selectionError);
        return false;
    }

    for (const CadItem* item : expandedItems)
    {
        if (item != nullptr && item->m_rotaryEndCutRole != RotaryEndCutRole::None)
        {
            QMessageBox::warning
            (
                this,
                QStringLiteral("指定加工断面"),
                QStringLiteral("选中的图元已属于一个加工断面边界。如需重新指定，请先清除加工断面指定。")
            );
            return false;
        }
    }

    int highestBoundaryId = -1;

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity == nullptr || entity->m_rotaryEndCutRole == RotaryEndCutRole::None)
        {
            continue;
        }

        highestBoundaryId = std::max(highestBoundaryId, entity->m_rotaryEndCutPairId);
    }

    const int boundaryId = highestBoundaryId + 1;

    for (CadItem* item : expandedItems)
    {
        if (item == nullptr)
        {
            continue;
        }

        item->m_rotaryEndCutPairId = boundaryId;
        item->m_rotaryEndCutRole = RotaryEndCutRole::Break;
    }

    const QString message = QStringLiteral("已指定加工断面 断%1，共识别 %2 个相连图元，已通过方管周向分离验证。")
        .arg(boundaryId + 1)
        .arg(expandedItems.size());

    invalidateProcessOrdersAfterEndCutChange();
    refreshWasteProcessingExclusions();
    ui->openGLWidget->appendCommandMessage(message);
    ui->openGLWidget->update();
    statusBar()->showMessage(message, 5000);
    return true;
}

bool Gcode_postprocessing_system::assignSelectedWasteEndCut()
{
    QString selectionError;
    const QVector<CadItem*> expandedItems = expandedSelectedRotaryEndCut(&selectionError);

    if (expandedItems.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("指定废面"), selectionError);
        return false;
    }

    for (const CadItem* item : expandedItems)
    {
        if (item != nullptr && item->m_rotaryEndCutRole != RotaryEndCutRole::None)
        {
            QMessageBox::warning(this, QStringLiteral("指定废面"), QStringLiteral("选中的图元已属于一个加工断面边界，请先清除原加工断面指定。"));
            return false;
        }
    }

    int highestBoundaryId = -1;

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity != nullptr && entity->m_rotaryEndCutRole != RotaryEndCutRole::None)
        {
            highestBoundaryId = std::max(highestBoundaryId, entity->m_rotaryEndCutPairId);
        }
    }

    const int wasteId = highestBoundaryId + 1;

    for (CadItem* item : expandedItems)
    {
        if (item == nullptr)
        {
            continue;
        }

        item->m_rotaryEndCutPairId = wasteId;
        item->m_rotaryEndCutRole = RotaryEndCutRole::Waste;
    }

    invalidateProcessOrdersAfterEndCutChange();
    const int excludedCount = refreshWasteProcessingExclusions();
    const QString message = QStringLiteral("已指定废弃面 W%1，共识别 %2 个相连图元，已通过方管周向分离验证；当前废弃区共排除 %3 个图元。")
        .arg(wasteId + 1)
        .arg(expandedItems.size())
        .arg(excludedCount);
    ui->openGLWidget->appendCommandMessage(message);
    ui->openGLWidget->update();
    statusBar()->showMessage(message, 6000);
    return true;
}

bool Gcode_postprocessing_system::smartAssignSelectedRotaryEndCut()
{
    return assignSelectedRotaryEndCut();
}

bool Gcode_postprocessing_system::recognizeRotaryTubeSection(bool interactive)
{
    QVector<CadItem*> sceneItems;
    sceneItems.reserve(static_cast<qsizetype>(m_document.m_entities.size()));

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity != nullptr)
        {
            sceneItems.push_back(entity.get());
        }
    }

    RotaryTubeSectionModel recognized;
    const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();

    if (!interactive)
    {
        recognized = RotaryTubeGeometryAnalyzer::findBestSectionModel
        (
            sceneItems,
            kEndCutConnectionTolerance
        );

        QString diagnostic = QStringLiteral("自动截面识别：检查候选 %1 个，有效 %2 个，圆角候选 %3 个")
            .arg(recognized.inspectedCandidateCount)
            .arg(recognized.validCandidateCount)
            .arg(recognized.roundedCandidateCount);

        if (recognized.valid)
        {
            diagnostic += QStringLiteral("；选中 X=%1，Y长=%2，Z宽=%3，圆角数=%4，R=%5")
                .arg(recognized.centerX, 0, 'f', 3)
                .arg(recognized.yLength, 0, 'f', 3)
                .arg(recognized.zWidth, 0, 'f', 3)
                .arg(recognized.roundedCornerCount)
                .arg(recognized.cornerRadius, 0, 'f', 3);
        }

        ui->openGLWidget->appendCommandMessage(diagnostic);

        if (recognized.valid && recognized.roundedCornerCount == 0)
        {
            ui->openGLWidget->appendCommandMessage
            (
                QStringLiteral("未找到可靠圆角截面，已回退使用直角截面。")
            );
        }
    }
    else if (!selectedItems.isEmpty())
    {
        recognized = RotaryTubeGeometryAnalyzer::buildSectionModel
        (
            selectedItems,
            sceneItems,
            kEndCutConnectionTolerance
        );
    }
    else
    {
        recognized.errorMessage = QStringLiteral("请先选择方管垂直截面中的一个或部分图元。");
    }

    if (!recognized.valid)
    {
        const QString errorMessage = recognized.errorMessage.isEmpty()
            ? QStringLiteral("未找到有效的方管垂直截面。")
            : recognized.errorMessage;

        if (interactive)
        {
            QMessageBox::warning(this, QStringLiteral("识别方管垂直截面"), errorMessage);
        }

        ui->openGLWidget->appendCommandMessage(QStringLiteral("方管垂直截面识别失败：%1").arg(errorMessage));
        statusBar()->showMessage(QStringLiteral("方管垂直截面识别失败"), 5000);
        return false;
    }

    m_rotaryTubeSectionModel = recognized;
    syncToolPanelState();
    const QString message = QStringLiteral("方管垂直截面识别完成，共提取 %1 个外轮廓点；内部线和无用支线已忽略。")
        .arg(recognized.sectionHull.size());
    ui->openGLWidget->appendCommandMessage(message);
    statusBar()->showMessage(message, 5000);
    return true;
}

bool Gcode_postprocessing_system::removeInternalMachiningPaths(bool interactive)
{
    Q_UNUSED(interactive);
    QVector<CadItem*> sceneItems;
    sceneItems.reserve(static_cast<qsizetype>(m_document.m_entities.size()));

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity != nullptr)
        {
            entity->m_excludedAsInternalGeometry = false;
            sceneItems.push_back(entity.get());
        }
    }

    const RotaryInternalPathResult result = RotaryTubeGeometryAnalyzer::findInternalPaths
    (
        m_rotaryTubeSectionModel,
        sceneItems,
        kEndCutConnectionTolerance
    );
    QSet<CadItem*> internalItems;

    for (CadItem* item : result.physicalInteriorItems)
    {
        internalItems.insert(item);
    }

    for (CadItem* item : result.topologicalInteriorItems)
    {
        internalItems.insert(item);
    }

    int excludedInternalCount = 0;

    for (CadItem* item : internalItems)
    {
        if (item == nullptr || item->m_rotaryEndCutRole != RotaryEndCutRole::None)
        {
            continue;
        }

        item->m_excludedAsInternalGeometry = true;
        item->m_excludedFromProcessing = true;
        item->m_processOrder = -1;
        item->m_processContinuousGroupId = -1;
        ++excludedInternalCount;
    }

    invalidateProcessOrdersAfterEndCutChange();
    refreshWasteProcessingExclusions();
    const QString message = m_rotaryTubeSectionModel.valid
        ? QStringLiteral("内部线条识别完成：进入方管内部 %1 个，最大外轮廓之外 %2 个，共排除 %3 个图元。")
            .arg(result.physicalInteriorItems.size())
            .arg(result.topologicalInteriorItems.size())
            .arg(excludedInternalCount)
        : QStringLiteral("方管垂直截面尚未识别，本次仅执行拓扑过滤：最大外轮廓之外 %1 个，共排除 %2 个图元。")
            .arg(result.topologicalInteriorItems.size())
            .arg(excludedInternalCount);
    ui->openGLWidget->appendCommandMessage(message);
    ui->openGLWidget->update();
    statusBar()->showMessage(message, 6000);
    return true;
}

bool Gcode_postprocessing_system::restoreInternalMachiningPaths(bool interactive)
{
    Q_UNUSED(interactive);
    int restoredCount = 0;

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity != nullptr && entity->m_excludedAsInternalGeometry)
        {
            entity->m_excludedAsInternalGeometry = false;
            ++restoredCount;
        }
    }

    refreshWasteProcessingExclusions();
    ui->openGLWidget->update();

    const QString message = restoredCount > 0
        ? QStringLiteral("已恢复 %1 个由内部线识别排除的图元。").arg(restoredCount)
        : QStringLiteral("当前没有由内部线识别排除的图元。");
    ui->openGLWidget->appendCommandMessage(message);
    statusBar()->showMessage(message, 4000);
    return restoredCount > 0;
}

bool Gcode_postprocessing_system::toggleSelectedRotaryEndCutAssignment()
{
    const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();

    if (selectedItems.isEmpty())
    {
        return false;
    }

    const bool hasUnassignedItem = std::any_of(selectedItems.begin(), selectedItems.end(), [](const CadItem* item)
    {
        return item != nullptr && item->m_rotaryEndCutRole == RotaryEndCutRole::None;
    });

    if (!hasUnassignedItem)
    {
        return clearSelectedRotaryEndCutAssignments();
    }

    QString errorMessage;
    const QVector<CadItem*> boundaryItems = expandedSelectedRotaryEndCut(&errorMessage);

    if (boundaryItems.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("加工断面指定"), errorMessage);
        return false;
    }

    int nextBoundaryId = 0;

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity != nullptr && entity->m_rotaryEndCutPairId >= nextBoundaryId)
        {
            nextBoundaryId = entity->m_rotaryEndCutPairId + 1;
        }
    }

    for (CadItem* item : boundaryItems)
    {
        item->m_rotaryEndCutPairId = nextBoundaryId;
        item->m_rotaryEndCutRole = RotaryEndCutRole::Break;
    }

    invalidateProcessOrdersAfterEndCutChange();
    refreshWasteProcessingExclusions();
    const QString message = QStringLiteral("已指定加工断面 %1，共 %2 个图元。")
        .arg(nextBoundaryId + 1)
        .arg(boundaryItems.size());
    ui->openGLWidget->appendCommandMessage(message);
    ui->openGLWidget->update();
    statusBar()->showMessage(message, 5000);
    return true;
}

bool Gcode_postprocessing_system::recognizeAllRotaryEndCuts(bool interactive)
{
    QVector<CadItem*> documentItems;
    QVector<CadItem*> sceneItems;

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity != nullptr && !entity->m_excludedAsInternalGeometry)
        {
            documentItems.push_back(entity.get());
            sceneItems.push_back(entity.get());
        }
    }

    if (documentItems.isEmpty())
    {
        const QString message = QStringLiteral("当前文档为空，未识别到加工断面。");
        ui->openGLWidget->appendCommandMessage(message);
        statusBar()->showMessage(message, 4000);

        if (interactive)
        {
            QMessageBox::information(this, QStringLiteral("识别加工断面"), message);
        }

        return false;
    }

    qInfo().noquote() << QStringLiteral("[断面候选] sceneItems 过滤后：%1")
        .arg(describeRotaryPathItems(sceneItems));
    const RotaryPathTopology topology
    (
        sceneItems,
        RotaryPathTopologyTolerance::fromConnectionTolerance(kEndCutConnectionTolerance)
    );
    int nextBoundaryId = 0;

    for (CadItem* item : documentItems)
    {
        if (item->m_rotaryEndCutPairId >= nextBoundaryId)
        {
            nextBoundaryId = item->m_rotaryEndCutPairId + 1;
        }
    }

    int recognizedCount = 0;
    QSet<CadItem*> attemptedBoundaryItems;

    for (CadItem* seedItem : documentItems)
    {
        if (seedItem == nullptr
            || seedItem->m_rotaryEndCutRole != RotaryEndCutRole::None
            || attemptedBoundaryItems.contains(seedItem))
        {
            continue;
        }

        const QVector<CadItem*> seedItems{ seedItem };
        QVector<CadItem*> connectedItems;
        const RotaryPathLoopResult loop = topology.extractSeededLoop(seedItems, &connectedItems);

        if (!loop.valid || loop.usedItems.isEmpty())
        {
            qInfo().noquote() << QStringLiteral("[自动识别加工断面] 种子 %1 提取失败：%2")
                .arg(describeRotaryPathItems(seedItems))
                .arg(loop.errorMessage);
            continue;
        }

        qInfo().noquote() << QStringLiteral("[断面候选] 选择集：%1")
            .arg(describeRotaryPathItems(seedItems));
        qInfo().noquote() << QStringLiteral("[断面候选] 连通扩展：%1")
            .arg(describeRotaryPathItems(connectedItems));

        for (CadItem* item : loop.usedItems)
        {
            attemptedBoundaryItems.insert(item);
        }

        if (std::any_of(loop.usedItems.begin(), loop.usedItems.end(), [](const CadItem* item)
        {
            return item != nullptr && item->m_rotaryEndCutRole != RotaryEndCutRole::None;
        }))
        {
            continue;
        }

        const RotaryCutBoundaryAnalysis analysis = RotaryCutBoundaryAnalyzer::analyze
        (
            loop.usedItems,
            sceneItems,
            m_rotaryTubeSectionModel,
            kEndCutConnectionTolerance
        );

        if (!analysis.valid)
        {
            qInfo().noquote() << QStringLiteral("[自动识别加工断面] 周向验证失败：%1")
                .arg(analysis.errorMessage);
            continue;
        }

        const QVector<CadItem*>& recognizedItems = analysis.boundaryItems.isEmpty()
            ? loop.usedItems
            : analysis.boundaryItems;

        for (CadItem* item : recognizedItems)
        {
            item->m_rotaryEndCutPairId = nextBoundaryId;
            item->m_rotaryEndCutRole = RotaryEndCutRole::Break;
        }

        ++nextBoundaryId;
        ++recognizedCount;
    }

    const QString message = QStringLiteral("所有加工断面识别完成，共识别 %1 个有效加工断面。")
        .arg(recognizedCount);

    if (recognizedCount > 0)
    {
        invalidateProcessOrdersAfterEndCutChange();
        refreshWasteProcessingExclusions();
    }

    ui->openGLWidget->appendCommandMessage(message);
    ui->openGLWidget->update();
    statusBar()->showMessage(message, 5000);

    if (interactive && recognizedCount == 0)
    {
        QMessageBox::information(this, QStringLiteral("识别加工断面"), QStringLiteral("未识别到有效加工断面。"));
    }

    syncMachiningSettingsState();
    return recognizedCount > 0;
}

bool Gcode_postprocessing_system::toggleSelectedInternalPathAssignment()
{
    const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();

    if (selectedItems.isEmpty())
    {
        return false;
    }

    const bool hasOrdinaryItem = std::any_of(selectedItems.begin(), selectedItems.end(), [](const CadItem* item)
    {
        return item != nullptr && !item->m_excludedAsInternalGeometry;
    });

    for (CadItem* item : selectedItems)
    {
        if (item != nullptr)
        {
            item->m_excludedAsInternalGeometry = hasOrdinaryItem;
            item->m_processOrder = -1;
            item->m_processContinuousGroupId = -1;
        }
    }

    invalidateProcessOrdersAfterEndCutChange();
    refreshWasteProcessingExclusions();
    const QString message = hasOrdinaryItem
        ? QStringLiteral("已将选中图元指定为内部线条。")
        : QStringLiteral("已恢复选中的内部线条。");
    ui->openGLWidget->appendCommandMessage(message);
    ui->openGLWidget->update();
    statusBar()->showMessage(message, 4000);
    return true;
}

bool Gcode_postprocessing_system::clearAllMachiningFaceAndLineAssignments()
{
    const bool clearedSections = clearRotaryEndCutAssignments();
    const bool restoredLines = restoreInternalMachiningPaths();
    const QString message = QStringLiteral("已清空所有加工断面和内部线条状态，未删除 CAD 图元。");
    ui->openGLWidget->appendCommandMessage(message);
    statusBar()->showMessage(message, 5000);
    return clearedSections || restoredLines;
}

bool Gcode_postprocessing_system::clearSelectedRotaryEndCutAssignments()
{
    const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();
    QSet<int> pairIds;

    for (const CadItem* item : selectedItems)
    {
        if (item != nullptr
            && item->m_rotaryEndCutRole != RotaryEndCutRole::None
            && item->m_rotaryEndCutPairId >= 0)
        {
            pairIds.insert(item->m_rotaryEndCutPairId);
        }
    }

    if (pairIds.isEmpty())
    {
        QMessageBox::information(this, QStringLiteral("清除加工断面指定"), QStringLiteral("请先选中一个已指定的加工断面。"));
        return false;
    }

    int clearedCount = 0;

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity == nullptr || !pairIds.contains(entity->m_rotaryEndCutPairId))
        {
            continue;
        }

        entity->m_rotaryEndCutPairId = -1;
        entity->m_rotaryEndCutRole = RotaryEndCutRole::None;
        ++clearedCount;
    }

    const QString message = QStringLiteral("已清除 %1 个加工断面边界，共 %2 个图元。")
        .arg(pairIds.size())
        .arg(clearedCount);
    invalidateProcessOrdersAfterEndCutChange();
    refreshWasteProcessingExclusions();
    ui->openGLWidget->appendCommandMessage(message);
    ui->openGLWidget->update();
    statusBar()->showMessage(message, 5000);
    return true;
}

bool Gcode_postprocessing_system::clearRotaryEndCutAssignments()
{
    int clearedCount = 0;

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity == nullptr || entity->m_rotaryEndCutRole == RotaryEndCutRole::None)
        {
            continue;
        }

        entity->m_rotaryEndCutPairId = -1;
        entity->m_rotaryEndCutRole = RotaryEndCutRole::None;
        ++clearedCount;
    }

    if (clearedCount == 0)
    {
        statusBar()->showMessage(QStringLiteral("当前没有已指定的加工断面。"), 3000);
        return false;
    }

    const QString message = QStringLiteral("已清除 %1 个图元的加工断面指定。").arg(clearedCount);
    invalidateProcessOrdersAfterEndCutChange();
    refreshWasteProcessingExclusions();
    ui->openGLWidget->appendCommandMessage(message);
    ui->openGLWidget->update();
    statusBar()->showMessage(message, 5000);
    return true;
}

void Gcode_postprocessing_system::invalidateProcessOrdersAfterEndCutChange()
{
    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity != nullptr)
        {
            entity->m_processOrder = -1;
            entity->m_processContinuousGroupId = -1;
        }
    }
}

int Gcode_postprocessing_system::refreshWasteProcessingExclusions()
{
    struct BoundaryGroup
    {
        RotaryEndCutRole role = RotaryEndCutRole::None;
        QVector<CadItem*> items;
    };

    std::map<int, BoundaryGroup> boundaryGroups;
    QVector<CadItem*> sceneItems;
    sceneItems.reserve(static_cast<qsizetype>(m_document.m_entities.size()));

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity == nullptr)
        {
            continue;
        }

        const bool isBreakBoundary = entity->m_rotaryEndCutRole == RotaryEndCutRole::Break
            && entity->m_rotaryEndCutPairId >= 0;
        entity->m_excludedFromProcessing = entity->m_excludedAsInternalGeometry && !isBreakBoundary;
        sceneItems.push_back(entity.get());

        if (entity->m_rotaryEndCutRole == RotaryEndCutRole::None || entity->m_rotaryEndCutPairId < 0)
        {
            continue;
        }

        const int roleIndex = static_cast<int>(entity->m_rotaryEndCutRole);
        const int key = entity->m_rotaryEndCutPairId * 4 + roleIndex;
        BoundaryGroup& group = boundaryGroups[key];
        group.role = entity->m_rotaryEndCutRole;
        group.items.push_back(entity.get());

        if (entity->m_rotaryEndCutRole == RotaryEndCutRole::Waste)
        {
            entity->m_excludedFromProcessing = true;
            entity->m_processOrder = -1;
            entity->m_processContinuousGroupId = -1;
        }
    }

    struct BoundaryPosition
    {
        RotaryEndCutRole role = RotaryEndCutRole::None;
        double centerX = 0.0;
        RotaryCutBoundaryAnalysis analysis;
    };

    std::vector<BoundaryPosition> boundaries;
    boundaries.reserve(boundaryGroups.size());

    for (const auto& [key, group] : boundaryGroups)
    {
        Q_UNUSED(key);
        BoundaryPosition boundary;
        boundary.role = group.role;
        boundary.analysis = RotaryCutBoundaryAnalyzer::analyze
        (
            group.items,
            sceneItems,
            m_rotaryTubeSectionModel,
            kEndCutConnectionTolerance
        );

        if (!boundary.analysis.valid || boundary.analysis.orderedPath.isEmpty())
        {
            continue;
        }

        for (const QVector3D& point : boundary.analysis.orderedPath)
        {
            boundary.centerX += point.x();
        }

        boundary.centerX /= static_cast<double>(boundary.analysis.orderedPath.size());
        boundaries.push_back(std::move(boundary));
    }

    std::sort
    (
        boundaries.begin(),
        boundaries.end(),
        [](const BoundaryPosition& left, const BoundaryPosition& right)
        {
            return left.centerX < right.centerX;
        }
    );

    bool boundariesHaveStableOrder = true;

    for (size_t boundaryIndex = 1; boundaryIndex < boundaries.size() && boundariesHaveStableOrder; ++boundaryIndex)
    {
        const BoundaryPosition& leftBoundary = boundaries[boundaryIndex - 1];
        const BoundaryPosition& rightBoundary = boundaries[boundaryIndex];

        if (RotaryCutBoundaryAnalyzer::boundariesIntersect
        (
            leftBoundary.analysis,
            rightBoundary.analysis,
            kEndCutConnectionTolerance
        ))
        {
            boundariesHaveStableOrder = false;
            break;
        }

        for (const QVector3D& point : rightBoundary.analysis.orderedPath)
        {
            const RotaryBoundarySide side = RotaryCutBoundaryAnalyzer::classifyPointRelativeToBoundary
            (
                leftBoundary.analysis,
                point,
                kEndCutConnectionTolerance
            );

            if (side != RotaryBoundarySide::After && side != RotaryBoundarySide::OnBoundary)
            {
                boundariesHaveStableOrder = false;
                break;
            }
        }

        for (const QVector3D& point : leftBoundary.analysis.orderedPath)
        {
            const RotaryBoundarySide side = RotaryCutBoundaryAnalyzer::classifyPointRelativeToBoundary
            (
                rightBoundary.analysis,
                point,
                kEndCutConnectionTolerance
            );

            if (side != RotaryBoundarySide::Before && side != RotaryBoundarySide::OnBoundary)
            {
                boundariesHaveStableOrder = false;
                break;
            }
        }
    }

    if (!boundariesHaveStableOrder)
    {
        boundaries.clear();
    }

    int excludedCount = 0;

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity == nullptr)
        {
            continue;
        }

        if (entity->m_rotaryEndCutRole == RotaryEndCutRole::Waste)
        {
            ++excludedCount;
            continue;
        }

        if (entity->m_rotaryEndCutRole != RotaryEndCutRole::None || boundaries.size() < 2)
        {
            continue;
        }

        entity->rebuildRawPathPoints3D();

        if (entity->rawPathPoints3D().empty())
        {
            continue;
        }

        size_t intervalIndex = boundaries.size();
        bool crossesBoundary = false;

        for (size_t boundaryIndex = 0; boundaryIndex < boundaries.size(); ++boundaryIndex)
        {
            bool hasPointBefore = false;
            bool hasPointAfter = false;

            for (const RawPathPoint3D& rawPoint : entity->rawPathPoints3D())
            {
                const QVector3D point
                (
                    static_cast<float>(rawPoint.x),
                    static_cast<float>(rawPoint.y),
                    static_cast<float>(rawPoint.z)
                );
                const RotaryBoundarySide side = RotaryCutBoundaryAnalyzer::classifyPointRelativeToBoundary
                (
                    boundaries[boundaryIndex].analysis,
                    point,
                    kEndCutConnectionTolerance
                );

                if (side == RotaryBoundarySide::Ambiguous)
                {
                    crossesBoundary = true;
                    break;
                }

                hasPointBefore = hasPointBefore || side == RotaryBoundarySide::Before;
                hasPointAfter = hasPointAfter || side == RotaryBoundarySide::After;
            }

            if (crossesBoundary || (hasPointBefore && hasPointAfter))
            {
                crossesBoundary = true;
                break;
            }

            if (!hasPointAfter)
            {
                intervalIndex = boundaryIndex;
                break;
            }
        }

        if (crossesBoundary || intervalIndex == 0 || intervalIndex >= boundaries.size())
        {
            continue;
        }

        if (boundaries[intervalIndex - 1].role == RotaryEndCutRole::Waste
            || boundaries[intervalIndex].role == RotaryEndCutRole::Waste)
        {
            entity->m_excludedFromProcessing = true;
            entity->m_processOrder = -1;
            entity->m_processContinuousGroupId = -1;
            ++excludedCount;
        }
    }

    syncMachiningSettingsState();
    return excludedCount;
}

bool Gcode_postprocessing_system::sortEntitiesByCurrentDirection()
{
    if (m_document.m_entities.empty())
    {
        QMessageBox::warning(this, QStringLiteral("3轴排序"), QStringLiteral("当前文档为空，无法执行排序。"));
        return false;
    }

    std::vector<CadItem*> sortableItems;

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity == nullptr)
        {
            continue;
        }

        const CadProcessVisualInfo info = buildProcessVisualInfo(entity.get());

        if (!info.valid)
        {
            continue;
        }

        sortableItems.push_back(entity.get());
    }

    const SortableDedupResult dedupResult = deduplicateSortableItems(sortableItems);

    if (!dedupResult.duplicateItems.isEmpty() && !m_editer.deleteEntities(dedupResult.duplicateItems))
    {
        QMessageBox::warning(this, QStringLiteral("3轴排序"), QStringLiteral("重复图元自动去重失败，排序已中止。"));
        return false;
    }

    if (sortableItems.empty())
    {
        QMessageBox::warning(this, QStringLiteral("3轴排序"), QStringLiteral("当前文档中没有可参与 G 代码排序的图元。"));
        return false;
    }

    const QVector3D sweepDirection = computeSweepDirection(sortableItems);
    const GapStartSelectionContext gapStartContext = buildGapStartSelectionContext(sortableItems, kPreferredStartGapDistance2D);
    std::vector<CadEditer::ProcessStateUpdate> processUpdates;
    std::vector<ProcessConnectionSegment> processedSegments;
    std::vector<bool> visited(sortableItems.size(), false);

    processUpdates.reserve(sortableItems.size());
    processedSegments.reserve(sortableItems.size());

    bool hasCurrentEndPoint = false;
    int currentComponentId = -1;
    QVector3D currentEndPoint;

    for (size_t order = 0; order < sortableItems.size(); ++order)
    {
        SortCandidate bestCandidate = chooseNext2DSortCandidate
        (
            sortableItems,
            visited,
            processedSegments,
            gapStartContext,
            SortStrategy::KeepDirection,
            currentComponentId,
            -1,
            false,
            hasCurrentEndPoint,
            currentEndPoint,
            sweepDirection
        );

        const std::vector<bool> visitedComponents = buildVisitedComponentMask(visited, gapStartContext.componentIds);
        const int selectedComponentId =
            bestCandidate.index >= 0 && static_cast<size_t>(bestCandidate.index) < gapStartContext.componentIds.size()
            ? gapStartContext.componentIds[static_cast<size_t>(bestCandidate.index)]
            : -1;
        const bool enteringFreshPreferredComponent =
            selectedComponentId >= 0
            && static_cast<size_t>(selectedComponentId) < visitedComponents.size()
            && !visitedComponents[static_cast<size_t>(selectedComponentId)]
            && static_cast<size_t>(selectedComponentId) < gapStartContext.preferredStartPointsByComponent.size()
            && !gapStartContext.preferredStartPointsByComponent[static_cast<size_t>(selectedComponentId)].empty();

        if (enteringFreshPreferredComponent)
        {
            bestCandidate = chooseNext2DSortCandidate
            (
                sortableItems,
                visited,
                processedSegments,
                gapStartContext,
                SortStrategy::KeepDirection,
                currentComponentId,
                selectedComponentId,
                true,
                hasCurrentEndPoint,
                currentEndPoint,
                sweepDirection
            );
        }

        if (bestCandidate.index < 0)
        {
            QMessageBox::warning(this, QStringLiteral("3轴排序"), QStringLiteral("排序过程中出现无效图元，排序已中止。"));
            return false;
        }

        visited[static_cast<size_t>(bestCandidate.index)] = true;
        processUpdates.push_back
        ({
            sortableItems[static_cast<size_t>(bestCandidate.index)],
            static_cast<int>(order),
            sortableItems[static_cast<size_t>(bestCandidate.index)]->m_isReverse,
            sortableItems[static_cast<size_t>(bestCandidate.index)]->m_hasCustomProcessStart,
            sortableItems[static_cast<size_t>(bestCandidate.index)]->m_processStartParameter
        });
        hasCurrentEndPoint = true;
        currentComponentId =
            static_cast<size_t>(bestCandidate.index) < gapStartContext.componentIds.size()
            ? gapStartContext.componentIds[static_cast<size_t>(bestCandidate.index)]
            : -1;
        currentEndPoint = bestCandidate.endPoint;
        processedSegments.push_back({ bestCandidate.startPoint, bestCandidate.endPoint });
    }

    if (!m_editer.applyEntityProcessStates(processUpdates))
    {
        QMessageBox::warning(this, QStringLiteral("3轴排序"), QStringLiteral("排序结果写入失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage
    (
        QStringLiteral("3轴排序完成，共更新 %1 个图元的加工顺序，首件已按最接近原点的当前起点选取，并保留当前加工方向设置。%2")
            .arg(processUpdates.size())
            .arg(dedupResult.removedCount > 0 ? QStringLiteral("已自动删除 %1 个重复图元。").arg(dedupResult.removedCount) : QString())
    );
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("3轴排序完成，共更新 %1 个图元").arg(processUpdates.size()), 5000);
    return true;
}

bool Gcode_postprocessing_system::assignSelectedEntityProcessOrder()
{
    CadItem* selectedItem = ui->openGLWidget->selectedEntity();

    if (selectedItem == nullptr)
    {
        QMessageBox::warning(this, QStringLiteral("排序"), QStringLiteral("请先选择一个图元。"));
        return false;
    }

    if (!isProcessVisualizable(selectedItem))
    {
        QMessageBox::warning(this, QStringLiteral("排序"), QStringLiteral("当前图元类型暂不支持加工排序。"));
        return false;
    }

    const int processOrder = nextProcessOrder(m_document);

    if (!m_editer.setEntityProcessOrder(selectedItem, processOrder))
    {
        QMessageBox::warning(this, QStringLiteral("排序"), QStringLiteral("当前图元加工顺序设置失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage(QStringLiteral("当前选中图元已设置为第 %1 个加工对象。").arg(processOrder + 1));
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("已设置加工顺序 #%1").arg(processOrder + 1), 5000);
    return true;
}

bool Gcode_postprocessing_system::smartSortEntities()
{
    if (m_document.m_entities.empty())
    {
        QMessageBox::warning(this, QStringLiteral("3轴智能排序"), QStringLiteral("当前文档为空，无法执行智能排序。"));
        return false;
    }

    std::vector<CadItem*> sortableItems;

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity == nullptr)
        {
            continue;
        }

        const CadProcessVisualInfo info = buildProcessVisualInfo(entity.get());

        if (!info.valid)
        {
            continue;
        }

        sortableItems.push_back(entity.get());
    }

    const SortableDedupResult dedupResult = deduplicateSortableItems(sortableItems);

    if (!dedupResult.duplicateItems.isEmpty() && !m_editer.deleteEntities(dedupResult.duplicateItems))
    {
        QMessageBox::warning(this, QStringLiteral("3轴智能排序"), QStringLiteral("重复图元自动去重失败，排序已中止。"));
        return false;
    }

    if (sortableItems.empty())
    {
        QMessageBox::warning(this, QStringLiteral("3轴智能排序"), QStringLiteral("当前文档中没有可参与 G 代码排序的图元。"));
        return false;
    }

    const QVector3D sweepDirection = computeSweepDirection(sortableItems);
    const GapStartSelectionContext gapStartContext = buildGapStartSelectionContext(sortableItems, kPreferredStartGapDistance2D);
    std::vector<CadEditer::ProcessStateUpdate> processUpdates;
    std::vector<ProcessConnectionSegment> processedSegments;
    std::vector<bool> visited(sortableItems.size(), false);

    processUpdates.reserve(sortableItems.size());
    processedSegments.reserve(sortableItems.size());

    bool hasCurrentEndPoint = false;
    int currentComponentId = -1;
    QVector3D currentEndPoint;

    for (size_t order = 0; order < sortableItems.size(); ++order)
    {
        SortCandidate bestCandidate = chooseNext2DSortCandidate
        (
            sortableItems,
            visited,
            processedSegments,
            gapStartContext,
            SortStrategy::Smart,
            currentComponentId,
            -1,
            false,
            hasCurrentEndPoint,
            currentEndPoint,
            sweepDirection
        );

        const std::vector<bool> visitedComponents = buildVisitedComponentMask(visited, gapStartContext.componentIds);
        const int selectedComponentId =
            bestCandidate.index >= 0 && static_cast<size_t>(bestCandidate.index) < gapStartContext.componentIds.size()
            ? gapStartContext.componentIds[static_cast<size_t>(bestCandidate.index)]
            : -1;
        const bool enteringFreshPreferredComponent =
            selectedComponentId >= 0
            && static_cast<size_t>(selectedComponentId) < visitedComponents.size()
            && !visitedComponents[static_cast<size_t>(selectedComponentId)]
            && static_cast<size_t>(selectedComponentId) < gapStartContext.preferredStartPointsByComponent.size()
            && !gapStartContext.preferredStartPointsByComponent[static_cast<size_t>(selectedComponentId)].empty();

        if (enteringFreshPreferredComponent)
        {
            bestCandidate = chooseNext2DSortCandidate
            (
                sortableItems,
                visited,
                processedSegments,
                gapStartContext,
                SortStrategy::Smart,
                currentComponentId,
                selectedComponentId,
                true,
                hasCurrentEndPoint,
                currentEndPoint,
                sweepDirection
            );
        }

        if (bestCandidate.index < 0)
        {
            QMessageBox::warning(this, QStringLiteral("3轴智能排序"), QStringLiteral("智能排序过程中出现无效图元，排序已中止。"));
            return false;
        }

        visited[static_cast<size_t>(bestCandidate.index)] = true;
        processUpdates.push_back
        ({
            sortableItems[static_cast<size_t>(bestCandidate.index)],
            static_cast<int>(order),
            bestCandidate.reverse,
            bestCandidate.hasCustomStart,
            bestCandidate.processStartParameter
        });
        hasCurrentEndPoint = true;
        currentComponentId =
            static_cast<size_t>(bestCandidate.index) < gapStartContext.componentIds.size()
            ? gapStartContext.componentIds[static_cast<size_t>(bestCandidate.index)]
            : -1;
        currentEndPoint = bestCandidate.endPoint;
        processedSegments.push_back({ bestCandidate.startPoint, bestCandidate.endPoint });
    }

    if (!m_editer.applyEntityProcessStates(processUpdates))
    {
        QMessageBox::warning(this, QStringLiteral("3轴智能排序"), QStringLiteral("智能排序结果写入失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage
    (
        QStringLiteral("3轴智能排序完成，共更新 %1 个图元的加工顺序，并已对闭合图元的方向/起刀缝点做连续性优化。%2")
            .arg(processUpdates.size())
            .arg(dedupResult.removedCount > 0 ? QStringLiteral("已自动删除 %1 个重复图元。").arg(dedupResult.removedCount) : QString())
    );
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("3轴智能排序完成，共更新 %1 个图元").arg(processUpdates.size()), 5000);
    return true;
}

bool Gcode_postprocessing_system::sortEntitiesByCurrentDirection3D()
{
    if (m_document.m_entities.empty())
    {
        QMessageBox::warning(this, QStringLiteral("4轴(绕A)排序"), QStringLiteral("当前文档为空，无法执行排序。"));
        return false;
    }

    if (!documentContainsThreeDimensionalGeometry(m_document))
    {
        QMessageBox::warning(this, QStringLiteral("4轴(绕A)排序"), QStringLiteral("当前文档未检测到可用于4轴排序的有效路径。"));
        return false;
    }

    std::vector<CadItem*> sortableItems;

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity == nullptr)
        {
            continue;
        }

        const CadProcessVisualInfo info = buildProcessVisualInfo(entity.get());

        if (!info.valid)
        {
            if (entity->m_rotaryEndCutRole != RotaryEndCutRole::Break
                || entity->m_rotaryEndCutPairId < 0)
            {
                continue;
            }

            entity->rebuildRawPathPoints3D();

            if (entity->rawPathPoints3D().empty())
            {
                continue;
            }
        }

        sortableItems.push_back(entity.get());
    }

    const SortableDedupResult dedupResult = deduplicateSortableItems(sortableItems);

    if (!dedupResult.duplicateItems.isEmpty() && !m_editer.deleteEntities(dedupResult.duplicateItems))
    {
        QMessageBox::warning(this, QStringLiteral("4轴(绕A)排序"), QStringLiteral("重复图元自动去重失败，排序已中止。"));
        return false;
    }

    if (sortableItems.empty())
    {
        QMessageBox::warning(this, QStringLiteral("4轴(绕A)排序"), QStringLiteral("当前文档中没有可参与 4 轴 G 代码排序的图元。"));
        return false;
    }

    const GProfileRotaryAxisConfig& rotaryAxisConfig = m_activeProfile.rotaryAxisConfig();
    std::vector<CadEditer::ProcessStateUpdate> lazyRotaryUpdates;
    QVector<CadItem*> documentItems;
    documentItems.reserve(static_cast<qsizetype>(m_document.m_entities.size()));

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity != nullptr)
        {
            documentItems.push_back(entity.get());
        }
    }

    QString segmentationFailure;

    if (tryBuildSquareTubeLazyRotaryProcessUpdates
    (
        sortableItems,
        documentItems,
        rotaryAxisConfig,
        lazyRotaryUpdates,
        m_rotaryTubeSectionModel,
        &segmentationFailure
    ))
    {
        if (!m_editer.applyEntityProcessStates(lazyRotaryUpdates))
        {
            QMessageBox::warning(this, QStringLiteral("4轴(绕A)排序"), QStringLiteral("4轴排序结果写入失败。"));
            return false;
        }

        ui->openGLWidget->appendCommandMessage
        (
            QStringLiteral("4轴(绕A)排序完成，共更新 %1 个图元；已按加工断面分段，并在每个加工断面前完成其左侧图元的懒旋转加工。%2")
                .arg(lazyRotaryUpdates.size())
                .arg(dedupResult.removedCount > 0 ? QStringLiteral("已自动删除 %1 个重复图元。").arg(dedupResult.removedCount) : QString())
        );
        ui->openGLWidget->refreshCommandPrompt();
        statusBar()->showMessage(QStringLiteral("4轴(绕A)懒旋转排序完成，共更新 %1 个图元").arg(lazyRotaryUpdates.size()), 5000);
        return true;
    }

    if (std::any_of(documentItems.cbegin(), documentItems.cend(), [](const CadItem* item)
        {
            return item != nullptr
                && item->m_rotaryEndCutRole == RotaryEndCutRole::Break
                && item->m_rotaryEndCutPairId >= 0;
        }))
    {
        const QString message = segmentationFailure.isEmpty()
            ? QStringLiteral("已指定加工断面，但无法形成稳定加工分段。")
            : segmentationFailure;
        ui->openGLWidget->appendCommandMessage(QStringLiteral("[智能分段] %1").arg(message));
        ui->openGLWidget->refreshCommandPrompt();
        statusBar()->showMessage(message, 8000);
        QMessageBox::warning
        (
            this,
            QStringLiteral("4轴(绕A)排序"),
            message
        );
        return false;
    }

    const QVector3D sweepDirection = computeRotarySweepDirection(sortableItems, rotaryAxisConfig);
    const GapStartSelectionContext gapStartContext = buildGapStartSelectionContext(sortableItems, kPreferredStartGapDistance3D);
    std::vector<CadEditer::ProcessStateUpdate> processUpdates;
    std::vector<ProcessConnectionSegment> processedSegments;
    std::vector<bool> visited(sortableItems.size(), false);

    processUpdates.reserve(sortableItems.size());
    processedSegments.reserve(sortableItems.size());

    bool hasCurrentEndPoint = false;
    int currentComponentId = -1;
    QVector3D currentEndPoint;

    for (size_t order = 0; order < sortableItems.size(); ++order)
    {
        SortCandidate bestCandidate = chooseNext3DSortCandidate
        (
            sortableItems,
            visited,
            processedSegments,
            gapStartContext,
            SortStrategy::KeepDirection,
            currentComponentId,
            -1,
            false,
            hasCurrentEndPoint,
            currentEndPoint,
            sweepDirection,
            rotaryAxisConfig
        );

        const std::vector<bool> visitedComponents = buildVisitedComponentMask(visited, gapStartContext.componentIds);
        const int selectedComponentId =
            bestCandidate.index >= 0 && static_cast<size_t>(bestCandidate.index) < gapStartContext.componentIds.size()
            ? gapStartContext.componentIds[static_cast<size_t>(bestCandidate.index)]
            : -1;
        const bool enteringFreshPreferredComponent =
            selectedComponentId >= 0
            && static_cast<size_t>(selectedComponentId) < visitedComponents.size()
            && !visitedComponents[static_cast<size_t>(selectedComponentId)]
            && static_cast<size_t>(selectedComponentId) < gapStartContext.preferredStartPointsByComponent.size()
            && !gapStartContext.preferredStartPointsByComponent[static_cast<size_t>(selectedComponentId)].empty();

        if (enteringFreshPreferredComponent)
        {
            bestCandidate = chooseNext3DSortCandidate
            (
                sortableItems,
                visited,
                processedSegments,
                gapStartContext,
                SortStrategy::KeepDirection,
                currentComponentId,
                selectedComponentId,
                true,
                hasCurrentEndPoint,
                currentEndPoint,
                sweepDirection,
                rotaryAxisConfig
            );
        }

        if (bestCandidate.index < 0)
        {
            QMessageBox::warning(this, QStringLiteral("4轴(绕A)排序"), QStringLiteral("4轴排序过程中出现无效图元，排序已中止。"));
            return false;
        }

        visited[static_cast<size_t>(bestCandidate.index)] = true;
        processUpdates.push_back
        ({
            sortableItems[static_cast<size_t>(bestCandidate.index)],
            static_cast<int>(order),
            sortableItems[static_cast<size_t>(bestCandidate.index)]->m_isReverse,
            sortableItems[static_cast<size_t>(bestCandidate.index)]->m_hasCustomProcessStart,
            sortableItems[static_cast<size_t>(bestCandidate.index)]->m_processStartParameter
        });
        hasCurrentEndPoint = true;
        currentComponentId =
            static_cast<size_t>(bestCandidate.index) < gapStartContext.componentIds.size()
            ? gapStartContext.componentIds[static_cast<size_t>(bestCandidate.index)]
            : -1;
        currentEndPoint = bestCandidate.endPoint;
        processedSegments.push_back({ bestCandidate.startPoint, bestCandidate.endPoint });
    }

    assignClosedComponentGroupIds(sortableItems, gapStartContext.componentIds, processUpdates);

    if (!m_editer.applyEntityProcessStates(processUpdates))
    {
        QMessageBox::warning(this, QStringLiteral("4轴(绕A)排序"), QStringLiteral("4轴排序结果写入失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage
    (
        QStringLiteral("4轴(绕A)排序完成，共更新 %1 个图元的加工顺序，排序已按 X 与 A 轴联动连续性重新整理。%2")
            .arg(processUpdates.size())
            .arg(dedupResult.removedCount > 0 ? QStringLiteral("已自动删除 %1 个重复图元。").arg(dedupResult.removedCount) : QString())
    );
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("4轴(绕A)排序完成，共更新 %1 个图元").arg(processUpdates.size()), 5000);
    return true;
}

bool Gcode_postprocessing_system::smartSortEntities3D()
{
    if (m_document.m_entities.empty())
    {
        QMessageBox::warning(this, QStringLiteral("4轴(绕A)智能排序"), QStringLiteral("当前文档为空，无法执行智能排序。"));
        return false;
    }

    if (!documentContainsThreeDimensionalGeometry(m_document))
    {
        QMessageBox::warning(this, QStringLiteral("4轴(绕A)智能排序"), QStringLiteral("当前文档未检测到可用于4轴智能排序的有效路径。"));
        return false;
    }

    std::vector<CadItem*> sortableItems;

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity == nullptr)
        {
            continue;
        }

        const CadProcessVisualInfo info = buildProcessVisualInfo(entity.get());

        if (!info.valid)
        {
            if (entity->m_rotaryEndCutRole != RotaryEndCutRole::Break
                || entity->m_rotaryEndCutPairId < 0)
            {
                continue;
            }

            entity->rebuildRawPathPoints3D();

            if (entity->rawPathPoints3D().empty())
            {
                continue;
            }
        }

        sortableItems.push_back(entity.get());
    }

    const SortableDedupResult dedupResult = deduplicateSortableItems(sortableItems);

    if (!dedupResult.duplicateItems.isEmpty() && !m_editer.deleteEntities(dedupResult.duplicateItems))
    {
        QMessageBox::warning(this, QStringLiteral("4轴(绕A)智能排序"), QStringLiteral("重复图元自动去重失败，排序已中止。"));
        return false;
    }

    if (sortableItems.empty())
    {
        QMessageBox::warning(this, QStringLiteral("4轴(绕A)智能排序"), QStringLiteral("当前文档中没有可参与 4 轴 G 代码排序的图元。"));
        return false;
    }

    const GProfileRotaryAxisConfig& rotaryAxisConfig = m_activeProfile.rotaryAxisConfig();
    std::vector<CadEditer::ProcessStateUpdate> lazyRotaryUpdates;
    QVector<CadItem*> documentItems;
    documentItems.reserve(static_cast<qsizetype>(m_document.m_entities.size()));

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity != nullptr)
        {
            documentItems.push_back(entity.get());
        }
    }

    QString segmentationFailure;

    if (tryBuildSquareTubeLazyRotaryProcessUpdates
    (
        sortableItems,
        documentItems,
        rotaryAxisConfig,
        lazyRotaryUpdates,
        m_rotaryTubeSectionModel,
        &segmentationFailure
    ))
    {
        if (!m_editer.applyEntityProcessStates(lazyRotaryUpdates))
        {
            QMessageBox::warning(this, QStringLiteral("4轴(绕A)智能排序"), QStringLiteral("4轴智能排序结果写入失败。"));
            return false;
        }

        ui->openGLWidget->appendCommandMessage
        (
            QStringLiteral("4轴(绕A)智能排序完成，共更新 %1 个图元；已按加工断面分段，并在每个加工断面前完成其左侧图元的懒旋转加工。%2")
                .arg(lazyRotaryUpdates.size())
                .arg(dedupResult.removedCount > 0 ? QStringLiteral("已自动删除 %1 个重复图元。").arg(dedupResult.removedCount) : QString())
        );
        ui->openGLWidget->refreshCommandPrompt();
        statusBar()->showMessage(QStringLiteral("4轴(绕A)懒旋转智能排序完成，共更新 %1 个图元").arg(lazyRotaryUpdates.size()), 5000);
        return true;
    }

    if (std::any_of(documentItems.cbegin(), documentItems.cend(), [](const CadItem* item)
        {
            return item != nullptr
                && item->m_rotaryEndCutRole == RotaryEndCutRole::Break
                && item->m_rotaryEndCutPairId >= 0;
        }))
    {
        const QString message = segmentationFailure.isEmpty()
            ? QStringLiteral("已指定加工断面，但无法形成稳定加工分段。")
            : segmentationFailure;
        ui->openGLWidget->appendCommandMessage(QStringLiteral("[智能分段] %1").arg(message));
        ui->openGLWidget->refreshCommandPrompt();
        statusBar()->showMessage(message, 8000);
        QMessageBox::warning
        (
            this,
            QStringLiteral("4轴(绕A)智能排序"),
            message
        );
        return false;
    }

    const QVector3D sweepDirection = computeRotarySweepDirection(sortableItems, rotaryAxisConfig);
    const GapStartSelectionContext gapStartContext = buildGapStartSelectionContext(sortableItems, kPreferredStartGapDistance3D);
    std::vector<CadEditer::ProcessStateUpdate> processUpdates;
    std::vector<ProcessConnectionSegment> processedSegments;
    std::vector<bool> visited(sortableItems.size(), false);

    processUpdates.reserve(sortableItems.size());
    processedSegments.reserve(sortableItems.size());

    bool hasCurrentEndPoint = false;
    int currentComponentId = -1;
    QVector3D currentEndPoint;

    for (size_t order = 0; order < sortableItems.size(); ++order)
    {
        SortCandidate bestCandidate = chooseNext3DSortCandidate
        (
            sortableItems,
            visited,
            processedSegments,
            gapStartContext,
            SortStrategy::Smart,
            currentComponentId,
            -1,
            false,
            hasCurrentEndPoint,
            currentEndPoint,
            sweepDirection,
            rotaryAxisConfig
        );

        const std::vector<bool> visitedComponents = buildVisitedComponentMask(visited, gapStartContext.componentIds);
        const int selectedComponentId =
            bestCandidate.index >= 0 && static_cast<size_t>(bestCandidate.index) < gapStartContext.componentIds.size()
            ? gapStartContext.componentIds[static_cast<size_t>(bestCandidate.index)]
            : -1;
        const bool enteringFreshPreferredComponent =
            selectedComponentId >= 0
            && static_cast<size_t>(selectedComponentId) < visitedComponents.size()
            && !visitedComponents[static_cast<size_t>(selectedComponentId)]
            && static_cast<size_t>(selectedComponentId) < gapStartContext.preferredStartPointsByComponent.size()
            && !gapStartContext.preferredStartPointsByComponent[static_cast<size_t>(selectedComponentId)].empty();

        if (enteringFreshPreferredComponent)
        {
            bestCandidate = chooseNext3DSortCandidate
            (
                sortableItems,
                visited,
                processedSegments,
                gapStartContext,
                SortStrategy::Smart,
                currentComponentId,
                selectedComponentId,
                true,
                hasCurrentEndPoint,
                currentEndPoint,
                sweepDirection,
                rotaryAxisConfig
            );
        }

        if (bestCandidate.index < 0)
        {
            QMessageBox::warning(this, QStringLiteral("4轴(绕A)智能排序"), QStringLiteral("4轴智能排序过程中出现无效图元，排序已中止。"));
            return false;
        }

        visited[static_cast<size_t>(bestCandidate.index)] = true;
        processUpdates.push_back
        ({
            sortableItems[static_cast<size_t>(bestCandidate.index)],
            static_cast<int>(order),
            bestCandidate.reverse,
            bestCandidate.hasCustomStart,
            bestCandidate.processStartParameter
        });
        hasCurrentEndPoint = true;
        currentComponentId =
            static_cast<size_t>(bestCandidate.index) < gapStartContext.componentIds.size()
            ? gapStartContext.componentIds[static_cast<size_t>(bestCandidate.index)]
            : -1;
        currentEndPoint = bestCandidate.endPoint;
        processedSegments.push_back({ bestCandidate.startPoint, bestCandidate.endPoint });
    }

    assignClosedComponentGroupIds(sortableItems, gapStartContext.componentIds, processUpdates);

    if (!m_editer.applyEntityProcessStates(processUpdates))
    {
        QMessageBox::warning(this, QStringLiteral("4轴(绕A)智能排序"), QStringLiteral("4轴智能排序结果写入失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage
    (
        QStringLiteral("4轴(绕A)智能排序完成，共更新 %1 个图元的加工顺序，并已按 A 轴连续性优化方向与闭合图元缝点。").arg(processUpdates.size())
    );
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("4轴(绕A)智能排序完成，共更新 %1 个图元").arg(processUpdates.size()), 5000);
    return true;
}


GGenerator::GenerationMode Gcode_postprocessing_system::resolveGenerationMode() const
{
    switch (m_generationPreference)
    {
    case GCodeGenerationPreference::Force2D:
        return GGenerator::GenerationMode::Mode2D;
    case GCodeGenerationPreference::Force3D:
        return GGenerator::GenerationMode::Mode3D;
    case GCodeGenerationPreference::Auto:
    default:
        return documentContainsThreeDimensionalGeometry(m_document)
            ? GGenerator::GenerationMode::Mode3D
            : GGenerator::GenerationMode::Mode2D;
    }
}
