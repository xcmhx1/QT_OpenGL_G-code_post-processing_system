#include "platform/pch.h"

#include "desktop/Gcode_postprocessing_system.h"

#include "cad/items/CadItem.h"
#include "cad/items/CadCircleItem.h"
#include "cad/items/CadEllipseItem.h"
#include "cad/geometry/CadEllipseGeometry.h"
#include "cad/geometry/CadOcsGeometry.h"
#include "cad/view/rendering/CadProcessVisualUtils.h"
#include "application/machining/RotaryCutBoundaryAnalyzer.h"
#include "application/machining/RotaryPathTopology.h"
#include "application/planning/ProcessPlanningService.h"

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
    constexpr double kNearGapPriorityDistance3D = 1.0;
    constexpr double kPreferredStartGapDistance3D = 1.0;
    constexpr double kRotaryPlaneMatchToleranceDegrees = 3.0;
    constexpr double kSurfaceSweepBoundaryTolerance = 1.0e-4;
    constexpr double kSquareTubeSectionToleranceRatio = 0.015;
    constexpr double kSortDedupCoordinateTolerance = 1.0e-4;
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

    double spatialSegmentToSegmentDistance(const QVector3D& firstStart, const QVector3D& firstEnd, const QVector3D& secondStart, const QVector3D& secondEnd)
    {
        return std::sqrt(segmentToSegmentDistanceSquared(firstStart, firstEnd, secondStart, secondEnd));
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

    size_t effectiveClosedPolylineStartIndex(const CadItem*, size_t vertexCount)
    {
        if (vertexCount == 0)
        {
            return 0;
        }

        return 0;
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
                : std::initializer_list<bool>{ false };

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
                : std::initializer_list<bool>{ false };

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
                : std::initializer_list<bool>{ false };

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
                : std::initializer_list<bool>{ false };

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
                : std::initializer_list<bool>{ false };

                for (const bool reverse : reverseOptions)
                {
                    ProcessPathOption option;
                    option.reverse = reverse;
                    option.hasCustomStart = false;
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
                : std::initializer_list<bool>{ false };

                for (const bool reverse : reverseOptions)
                {
                    ProcessPathOption option;
                    option.reverse = reverse;
                    option.hasCustomStart = false;
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
                : std::initializer_list<bool>{ false };
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

}

bool Gcode_postprocessing_system::sortEntitiesByCurrentMode(bool smartSort)
{
    const GGenerator::GenerationMode generationMode = resolveGenerationMode();
    if (generationMode == GGenerator::GenerationMode::Mode3D)
    {
        return smartSort ? smartSortEntities3D() : sortEntitiesByCurrentDirection3D();
    }

    const int excludedCount = refreshWasteProcessingExclusions();

    if (excludedCount > 0)
    {
        ui->openGLWidget->appendCommandMessage
        (
            QStringLiteral("废面规则已排除 %1 个图元，本次排序不会处理这些图元。").arg(excludedCount)
        );
    }

    return smartSort ? smartSortEntities() : sortEntitiesByCurrentDirection();
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
            && (!m_processState.stateOrDefault(entity->m_entityId).analysis.excludedAsInternalGeometry
                || selectedItems.contains(entity.get())))
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
        if (item != nullptr && m_processState.stateOrDefault(item->m_entityId)
            .overrideData.boundaryRole != cadcam::planning::BoundaryRole::None)
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
        if (entity == nullptr)
        {
            continue;
        }
        const auto state = m_processState.stateOrDefault(entity->m_entityId);
        if (state.overrideData.boundaryRole != cadcam::planning::BoundaryRole::None)
            highestBoundaryId = std::max(highestBoundaryId, state.overrideData.boundaryPairId);
    }

    const int boundaryId = highestBoundaryId + 1;

    m_processState.beginBatch();
    for (CadItem* item : expandedItems)
    {
        if (item == nullptr)
        {
            continue;
        }

        m_processState.setBoundary(item->m_entityId, cadcam::planning::BoundaryRole::Break, boundaryId);
    }
    m_processState.endBatch();

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
        if (item != nullptr && m_processState.stateOrDefault(item->m_entityId)
            .overrideData.boundaryRole != cadcam::planning::BoundaryRole::None)
        {
            QMessageBox::warning(this, QStringLiteral("指定废面"), QStringLiteral("选中的图元已属于一个加工断面边界，请先清除原加工断面指定。"));
            return false;
        }
    }

    int highestBoundaryId = -1;

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity != nullptr)
        {
            const auto state = m_processState.stateOrDefault(entity->m_entityId);
            if (state.overrideData.boundaryRole != cadcam::planning::BoundaryRole::None)
                highestBoundaryId = std::max(highestBoundaryId, state.overrideData.boundaryPairId);
        }
    }

    const int wasteId = highestBoundaryId + 1;

    m_processState.beginBatch();
    for (CadItem* item : expandedItems)
    {
        if (item == nullptr)
        {
            continue;
        }

        m_processState.setBoundary(item->m_entityId, cadcam::planning::BoundaryRole::Waste, wasteId);
    }
    m_processState.endBatch();

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
            kEndCutConnectionTolerance,
            m_document.contentRevision()
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
            kEndCutConnectionTolerance,
            m_document.contentRevision()
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
    invalidateProcessOrdersAfterEndCutChange();
    syncToolPanelState();
    const QString message = QStringLiteral("方管垂直截面识别完成，共提取 %1 个外轮廓点；内部线和无用支线已忽略。")
        .arg(recognized.sectionBoundary.size());
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
            sceneItems.push_back(entity.get());
        }
    }

    const RotaryInternalPathResult result = RotaryTubeGeometryAnalyzer::findInternalPaths
    (
        m_rotaryTubeSectionModel,
        sceneItems,
        kEndCutConnectionTolerance
    );
    QSet<CadItem*> topologicalItems;
    QSet<CadItem*> physicalItems;

    for (CadItem* item : result.physicalInteriorItems)
    {
        if (item != nullptr && m_processState.stateOrDefault(item->m_entityId)
            .overrideData.boundaryRole == cadcam::planning::BoundaryRole::None)
        {
            physicalItems.insert(item);
        }
    }

    for (CadItem* item : result.topologicalInteriorItems)
    {
        if (item != nullptr && m_processState.stateOrDefault(item->m_entityId)
            .overrideData.boundaryRole == cadcam::planning::BoundaryRole::None)
        {
            topologicalItems.insert(item);
        }
    }

    QSet<CadItem*> internalItems = topologicalItems;
    internalItems.unite(physicalItems);
    m_processState.beginBatch();
    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity != nullptr)
        {
            const bool shouldExclude = internalItems.contains(entity.get());
            m_processState.setInternalGeometryExcluded(entity->m_entityId, shouldExclude);
        }
    }
    m_processState.endBatch();

    invalidateProcessOrdersAfterEndCutChange();
    refreshWasteProcessingExclusions();
    QString message = m_rotaryTubeSectionModel.valid
        ? QStringLiteral("内部线条识别完成：拓扑内部 %1 个，进入方管内部 %2 个，去重后共排除 %3 个图元。")
            .arg(topologicalItems.size())
            .arg(physicalItems.size())
            .arg(internalItems.size())
        : QStringLiteral("内部线条识别完成：未识别方管截面，仅执行拓扑分析；拓扑内部 %1 个，去重后共排除 %2 个图元，跳过开放组件 %3 个。")
            .arg(topologicalItems.size())
            .arg(internalItems.size())
            .arg(result.skippedComponentCount);
    if (!internalItems.isEmpty()
        && (!ui->openGLWidget->processVisualsVisible()
            || !ui->openGLWidget->excludedEntitiesDimmed()))
    {
        message += QStringLiteral(" 内部线状态已标记；当前排除图元显示已关闭。");
    }
    ui->openGLWidget->appendCommandMessage(message);
    for (const Diagnostic& diagnostic : result.diagnostics)
    {
        if (diagnostic.severity == DiagnosticSeverity::Warning
            && !diagnostic.userMessage.isEmpty())
        {
            ui->openGLWidget->appendCommandMessage(diagnostic.userMessage);
        }
    }
    ui->openGLWidget->update();
    statusBar()->showMessage(message, 6000);
    return true;
}

bool Gcode_postprocessing_system::restoreInternalMachiningPaths(bool interactive)
{
    Q_UNUSED(interactive);
    int restoredCount = 0;

    m_processState.beginBatch();
    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity != nullptr && m_processState.stateOrDefault(entity->m_entityId)
            .analysis.excludedAsInternalGeometry)
        {
            m_processState.setInternalGeometryExcluded(entity->m_entityId, false);
            ++restoredCount;
        }
    }
    m_processState.endBatch();

    if (restoredCount > 0) invalidateProcessOrdersAfterEndCutChange();
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

    const bool hasUnassignedItem = std::any_of(selectedItems.begin(), selectedItems.end(), [this](const CadItem* item)
    {
        return item != nullptr && m_processState.stateOrDefault(item->m_entityId)
            .overrideData.boundaryRole == cadcam::planning::BoundaryRole::None;
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
        if (entity != nullptr)
        {
            const auto state = m_processState.stateOrDefault(entity->m_entityId);
            if (state.overrideData.boundaryPairId >= nextBoundaryId)
                nextBoundaryId = state.overrideData.boundaryPairId + 1;
        }
    }

    m_processState.beginBatch();
    for (CadItem* item : boundaryItems)
    {
        if (item != nullptr) m_processState.setBoundary
            (item->m_entityId, cadcam::planning::BoundaryRole::Break, nextBoundaryId);
    }
    m_processState.endBatch();

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
        if (entity != nullptr && !m_processState.stateOrDefault(entity->m_entityId)
            .analysis.excludedAsInternalGeometry)
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
        const auto state = m_processState.stateOrDefault(item->m_entityId);
        if (state.overrideData.boundaryPairId >= nextBoundaryId)
        {
            nextBoundaryId = state.overrideData.boundaryPairId + 1;
        }
    }

    int recognizedCount = 0;
    QSet<CadItem*> attemptedBoundaryItems;

    for (CadItem* seedItem : documentItems)
    {
        if (seedItem == nullptr
            || m_processState.stateOrDefault(seedItem->m_entityId)
                .overrideData.boundaryRole != cadcam::planning::BoundaryRole::None
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

        if (std::any_of(loop.usedItems.begin(), loop.usedItems.end(), [this](const CadItem* item)
        {
            return item != nullptr && m_processState.stateOrDefault(item->m_entityId)
                .overrideData.boundaryRole != cadcam::planning::BoundaryRole::None;
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

        m_processState.beginBatch();
        for (CadItem* item : recognizedItems)
        {
            if (item != nullptr) m_processState.setBoundary
                (item->m_entityId, cadcam::planning::BoundaryRole::Break, nextBoundaryId);
        }
        m_processState.endBatch();

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

    const bool hasOrdinaryItem = std::any_of(selectedItems.begin(), selectedItems.end(), [this](const CadItem* item)
    {
        return item != nullptr && !m_processState.stateOrDefault(item->m_entityId)
            .analysis.excludedAsInternalGeometry;
    });

    m_processState.beginBatch();
    for (CadItem* item : selectedItems)
    {
        if (item != nullptr)
        {
            m_processState.setInternalGeometryExcluded(item->m_entityId, hasOrdinaryItem);
        }
    }
    m_processState.endBatch();

    invalidateProcessOrdersAfterEndCutChange();
    refreshWasteProcessingExclusions();
    QString message = hasOrdinaryItem
        ? QStringLiteral("已将选中图元指定为内部线条。")
        : QStringLiteral("已恢复选中的内部线条。");
    if (hasOrdinaryItem
        && (!ui->openGLWidget->processVisualsVisible()
            || !ui->openGLWidget->excludedEntitiesDimmed()))
    {
        message += QStringLiteral(" 内部线状态已标记；当前排除图元显示已关闭。");
    }
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
        if (item != nullptr)
        {
            const auto state = m_processState.stateOrDefault(item->m_entityId);
            if (state.overrideData.boundaryRole != cadcam::planning::BoundaryRole::None
                && state.overrideData.boundaryPairId >= 0)
                pairIds.insert(state.overrideData.boundaryPairId);
        }
    }

    if (pairIds.isEmpty())
    {
        QMessageBox::information(this, QStringLiteral("清除加工断面指定"), QStringLiteral("请先选中一个已指定的加工断面。"));
        return false;
    }

    int clearedCount = 0;

    m_processState.beginBatch();
    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity == nullptr)
        {
            continue;
        }
        const auto state = m_processState.stateOrDefault(entity->m_entityId);
        if (pairIds.contains(state.overrideData.boundaryPairId))
        {
            m_processState.setBoundary(entity->m_entityId, cadcam::planning::BoundaryRole::None, -1);
            ++clearedCount;
        }
    }
    m_processState.endBatch();

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

    m_processState.beginBatch();
    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity == nullptr || m_processState.stateOrDefault(entity->m_entityId)
            .overrideData.boundaryRole == cadcam::planning::BoundaryRole::None)
        {
            continue;
        }
        m_processState.setBoundary(entity->m_entityId, cadcam::planning::BoundaryRole::None, -1);
        ++clearedCount;
    }
    m_processState.endBatch();

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
    invalidateCurrentProcessPlan();
}

void Gcode_postprocessing_system::invalidateCurrentProcessPlan()
{
    m_currentProcessPlan.reset();
    m_processPresentation.reset();
    if (ui != nullptr && ui->openGLWidget != nullptr)
    {
        ui->openGLWidget->setProcessPresentation(nullptr);
        ui->openGLWidget->update();
    }
}

int Gcode_postprocessing_system::refreshWasteProcessingExclusions()
{
    struct BoundaryGroup
    {
        cadcam::planning::BoundaryRole role = cadcam::planning::BoundaryRole::None;
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

        const auto state = m_processState.stateOrDefault(entity->m_entityId);
        sceneItems.push_back(entity.get());

        if (state.overrideData.boundaryRole == cadcam::planning::BoundaryRole::None
            || state.overrideData.boundaryPairId < 0)
        {
            continue;
        }

        const int roleIndex = static_cast<int>(state.overrideData.boundaryRole);
        const int key = state.overrideData.boundaryPairId * 4 + roleIndex;
        BoundaryGroup& group = boundaryGroups[key];
        group.role = state.overrideData.boundaryRole;
        group.items.push_back(entity.get());
    }

    struct BoundaryPosition
    {
        cadcam::planning::BoundaryRole role = cadcam::planning::BoundaryRole::None;
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

        const auto state = m_processState.stateOrDefault(entity->m_entityId);
        if (state.overrideData.boundaryRole == cadcam::planning::BoundaryRole::Waste)
        {
            ++excludedCount;
            continue;
        }

        if (state.overrideData.boundaryRole != cadcam::planning::BoundaryRole::None
            || boundaries.size() < 2)
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

        if (boundaries[intervalIndex - 1].role == cadcam::planning::BoundaryRole::Waste
            || boundaries[intervalIndex].role == cadcam::planning::BoundaryRole::Waste)
        {
            ++excludedCount;
        }
    }

    syncMachiningSettingsState();
    return excludedCount;
}

bool Gcode_postprocessing_system::sortEntitiesByCurrentDirection()
{
    return sortEntitiesWithProcessPlan2D(QStringLiteral("3轴排序"));
}

bool Gcode_postprocessing_system::assignSelectedEntityProcessOrder()
{
    return sortEntitiesWithProcessPlan2D(QStringLiteral("3轴排序"));
}

bool Gcode_postprocessing_system::smartSortEntities()
{
    return sortEntitiesWithProcessPlan2D(QStringLiteral("3轴智能排序"));
}

bool Gcode_postprocessing_system::sortEntitiesWithProcessPlan2D(const QString& commandTitle)
{
    if (m_document.m_entities.empty())
    {
        QMessageBox::warning(this, commandTitle, QStringLiteral("当前文档为空，无法执行排序。"));
        return false;
    }

    std::vector<CadItem*> dedupCandidates;
    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity != nullptr && buildProcessVisualInfo(entity.get()).valid)
            dedupCandidates.push_back(entity.get());
    }
    const SortableDedupResult dedupResult = deduplicateSortableItems(dedupCandidates);
    if (!dedupResult.duplicateItems.isEmpty()
        && !m_editer.deleteEntities(dedupResult.duplicateItems))
    {
        QMessageBox::warning(this, commandTitle, QStringLiteral("重复图元自动去重失败，排序已中止。"));
        return false;
    }

    cadcam::planning::PlanarProcessPlanningPolicy policy;
    policy.allowReverse = true;
    policy.preserveUserDirection = true;
    policy.initialPosition = { 0.0, 0.0, 0.0 };
    policy.hasInitialPosition = true;
    policy.numericalEpsilon = 1.0e-5;
    const OperationContext context = createOperationContext(QStringLiteral("BuildAndApplyPlanarProcessPlan"));
    ProcessPlanningService service;
    auto plan = service.buildPlanarPlan(m_document, m_processState, policy, context);
    if (!plan.succeeded() || !plan.value.has_value())
    {
        QString message = QStringLiteral("无法生成三轴加工计划。");
        for (const Diagnostic& diagnostic : plan.diagnostics)
        {
            if (!diagnostic.userMessage.trimmed().isEmpty())
            {
                message = diagnostic.userMessage;
                break;
            }
        }
        ui->openGLWidget->appendCommandMessage(QStringLiteral("[加工计划] %1").arg(message));
        ui->openGLWidget->refreshCommandPrompt();
        statusBar()->showMessage(message, 8000);
        QMessageBox::warning(this, commandTitle, message);
        return false;
    }

    m_currentProcessPlan = std::move(*plan.value);
    auto presentation = cadcam::process::ProcessPresentationSnapshot::build
        (*m_currentProcessPlan, context);
    if (!presentation.succeeded() || !presentation.value.has_value())
    {
        invalidateCurrentProcessPlan();
        QMessageBox::warning(this, commandTitle, QStringLiteral("加工计划显示快照构建失败。"));
        return false;
    }
    m_processPresentation = std::move(*presentation.value);
    ui->openGLWidget->setProcessPresentation(&*m_processPresentation);
    ui->openGLWidget->update();
    const QString message = QStringLiteral("%1完成：最近距离排序，共安排 %2 个图元，排除 %3 个图元。%4")
        .arg(commandTitle)
        .arg(m_currentProcessPlan->assignments.size())
        .arg(m_currentProcessPlan->exclusions.size())
        .arg(dedupResult.removedCount > 0
            ? QStringLiteral("已自动删除 %1 个重复图元。").arg(dedupResult.removedCount)
            : QString());
    ui->openGLWidget->appendCommandMessage(message);
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(message, 5000);
    return true;
}

bool Gcode_postprocessing_system::sortEntitiesByCurrentDirection3D()
{
    return sortEntitiesWithProcessPlan3D(QStringLiteral("4轴(绕A)排序"));
}

bool Gcode_postprocessing_system::sortEntitiesWithProcessPlan3D(const QString& commandTitle)
{
    if (m_document.m_entities.empty())
    {
        QMessageBox::warning(this, commandTitle, QStringLiteral("当前文档为空，无法执行排序。"));
        return false;
    }
    if (!documentContainsThreeDimensionalGeometry(m_document))
    {
        QMessageBox::warning(this, commandTitle, QStringLiteral("当前文档未检测到可用于4轴排序的有效路径。"));
        return false;
    }

    std::vector<CadItem*> dedupCandidates;
    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity == nullptr) continue;
        const CadProcessVisualInfo info = buildProcessVisualInfo(entity.get());
        const auto state = m_processState.stateOrDefault(entity->m_entityId);
        const bool isBoundary = state.overrideData.boundaryRole != cadcam::planning::BoundaryRole::None
            && state.overrideData.boundaryPairId >= 0;
        if (info.valid || isBoundary) dedupCandidates.push_back(entity.get());
    }
    const SortableDedupResult dedupResult = deduplicateSortableItems(dedupCandidates);
    if (!dedupResult.duplicateItems.isEmpty()
        && !m_editer.deleteEntities(dedupResult.duplicateItems))
    {
        QMessageBox::warning(this, commandTitle, QStringLiteral("重复图元自动去重失败，排序已中止。"));
        return false;
    }

    cadcam::planning::ProcessPlanningPolicy policy;
    policy.orderingStrategy = m_activeProfile.rotaryAxisConfig().lazyRotationProcessing
        ? cadcam::planning::ProcessOrderingStrategy::LazyRotation
        : cadcam::planning::ProcessOrderingStrategy::NearestNext;
    policy.connectionTolerance = kEndCutConnectionTolerance;
    policy.allowReverse = true;
    policy.preserveClosedLoopsAsAtomicGroups = true;
    policy.initialPosition = { 0.0, 0.0, 500.0 };

    const OperationContext context = createOperationContext(QStringLiteral("BuildAndApplyProcessPlan3D"));
    const std::optional<cadcam::machining::TubeSectionModel> section =
        m_rotaryTubeSectionModel.coreModel;
    ProcessPlanningService service;
    auto planResult = service.buildRotaryPlan
        (m_document, m_processState, section, policy, context);
    if (!planResult.succeeded() || !planResult.value.has_value())
    {
        QString message = QStringLiteral("无法生成加工计划。");
        for (const Diagnostic& diagnostic : planResult.diagnostics)
        {
            if (!diagnostic.userMessage.trimmed().isEmpty())
            {
                message = diagnostic.userMessage;
                break;
            }
        }
        ui->openGLWidget->appendCommandMessage(QStringLiteral("[加工计划] %1").arg(message));
        ui->openGLWidget->refreshCommandPrompt();
        statusBar()->showMessage(message, 8000);
        QMessageBox::warning(this, commandTitle, message);
        return false;
    }

    m_currentProcessPlan = std::move(*planResult.value);
    auto presentation = cadcam::process::ProcessPresentationSnapshot::build
        (*m_currentProcessPlan, context);
    if (!presentation.succeeded() || !presentation.value.has_value())
    {
        invalidateCurrentProcessPlan();
        QMessageBox::warning(this, commandTitle, QStringLiteral("加工计划显示快照构建失败。"));
        return false;
    }
    m_processPresentation = std::move(*presentation.value);
    ui->openGLWidget->setProcessPresentation(&*m_processPresentation);
    ui->openGLWidget->update();
    const QString strategyText = policy.orderingStrategy == cadcam::planning::ProcessOrderingStrategy::LazyRotation
        ? QStringLiteral("懒旋转加工")
        : QStringLiteral("普通最近距离排序");
    const QString message = QStringLiteral("%1完成：%2，共安排 %3 个图元，排除 %4 个图元。%5")
        .arg(commandTitle)
        .arg(strategyText)
        .arg(m_currentProcessPlan->assignments.size())
        .arg(m_currentProcessPlan->exclusions.size())
        .arg(dedupResult.removedCount > 0
            ? QStringLiteral("已自动删除 %1 个重复图元。").arg(dedupResult.removedCount)
            : QString());
    ui->openGLWidget->appendCommandMessage(message);
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(message, 5000);
    return true;
}

bool Gcode_postprocessing_system::smartSortEntities3D()
{
    return sortEntitiesWithProcessPlan3D(QStringLiteral("4轴(绕A)智能排序"));
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
