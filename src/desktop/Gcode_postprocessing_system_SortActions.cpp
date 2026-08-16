#include "platform/pch.h"

#include "desktop/Gcode_postprocessing_system.h"
#include "core/diagnostics/SummaryLog.h"

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
#include <QElapsedTimer>
#include <QHash>
#include <QMessageBox>
#include <QSet>
#include <QStatusBar>
#include <QStringList>

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <limits>
#include <map>
#include <set>
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

    class BoundaryPerformanceTimer
    {
    public:
        explicit BoundaryPerformanceTimer(double* accumulator)
            : m_accumulator(accumulator)
        {
            if (m_accumulator != nullptr) m_timer.start();
        }

        ~BoundaryPerformanceTimer()
        {
            if (m_accumulator != nullptr)
            {
                *m_accumulator +=
                    static_cast<double>(m_timer.nsecsElapsed()) / 1000000.0;
            }
        }

    private:
        double* m_accumulator = nullptr;
        QElapsedTimer m_timer;
    };

    class BoundaryAssignmentPerformanceOperation
    {
    public:
        BoundaryAssignmentPerformanceOperation
        (
            BoundaryAssignmentPerformanceReport*& activeReport,
            const QString& operation,
            std::uint64_t documentEntityCount,
            std::uint64_t selectedEntityCount
        )
            : m_activeReport(activeReport)
        {
            if (m_activeReport != nullptr) return;
            m_ownedReport.emplace();
            m_ownedReport->operation = operation;
            m_ownedReport->documentEntityCount = documentEntityCount;
            m_ownedReport->selectedEntityCount = selectedEntityCount;
            m_activeReport = &*m_ownedReport;
            m_totalTimer.start();
            m_owner = true;
        }

        ~BoundaryAssignmentPerformanceOperation()
        {
            if (!m_owner || !m_ownedReport.has_value()) return;
            BoundaryAssignmentPerformanceReport& report = *m_ownedReport;
            report.totalMs =
                static_cast<double>(m_totalTimer.nsecsElapsed()) / 1000000.0;
            cadcam::core::emitSummaryLog
            (
                QStringLiteral("Performance"),
                QStringLiteral("BoundaryAssignment"),
                QStringLiteral("operation=%1 totalMs=%2 "
                "selectionExpansionMs=%3 boundaryAnalysisMs=%4 boundaryOrderingMs=%5 "
                "wasteRefreshMs=%6 topologyBuildMs=%7 topologyAdapterMs=%8 "
                "endpointCompileMs=%9 pathCleanupMs=%10 coreTopologyBuildMs=%11 "
                "connectivityScanMs=%12 recordBoundsBuildMs=%13 recordMappingMs=%14 "
                "pathRebuildMs=%15 pointClassificationMs=%16 processStateUpdateMs=%17 "
                "viewerRefreshMs=%18 settingsSyncMs=%19 documentEntityCount=%20 "
                "selectedEntityCount=%21 boundaryGroupCount=%22 analyzedBoundaryCount=%23 "
                "topologyBuildCount=%24 topologyReuseCount=%25 topologyRecordCount=%26 "
                "totalPathPointCount=%27 totalSegmentCount=%28 recordPairCount=%29 "
                "recordPairBroadPhaseRejectedCount=%30 recordPairPreciseTestCount=%31 "
                "endpointToPathTestCount=%32 segmentPairTestCount=%33 "
                "connectedRecordPairCount=%34 adjacencyEdgeCount=%35 rebuiltPathCount=%36 "
                "reusedPathCount=%37 classifiedEntityCount=%38 classificationCallCount=%39 "
                "samplePointCount=%40"
            )
                .arg(report.operation)
                .arg(report.totalMs, 0, 'f', 3)
                .arg(report.selectionExpansionMs, 0, 'f', 3)
                .arg(report.boundaryAnalysisMs, 0, 'f', 3)
                .arg(report.boundaryOrderingMs, 0, 'f', 3)
                .arg(report.wasteRefreshMs, 0, 'f', 3)
                .arg(report.topologyBuildMs, 0, 'f', 3)
                .arg(report.topologyMetrics.topologyAdapterMs, 0, 'f', 3)
                .arg(report.topologyMetrics.endpointCompileMs, 0, 'f', 3)
                .arg(report.topologyMetrics.pathCleanupMs, 0, 'f', 3)
                .arg(report.topologyMetrics.coreTopologyBuildMs, 0, 'f', 3)
                .arg(report.topologyMetrics.connectivityScanMs, 0, 'f', 3)
                .arg(report.topologyMetrics.recordBoundsBuildMs, 0, 'f', 3)
                .arg(report.topologyMetrics.recordMappingMs, 0, 'f', 3)
                .arg(report.pathRebuildMs, 0, 'f', 3)
                .arg(report.pointClassificationMs, 0, 'f', 3)
                .arg(report.processStateUpdateMs, 0, 'f', 3)
                .arg(report.viewerRefreshMs, 0, 'f', 3)
                .arg(report.settingsSyncMs, 0, 'f', 3)
                .arg(static_cast<qulonglong>(report.documentEntityCount))
                .arg(static_cast<qulonglong>(report.selectedEntityCount))
                .arg(static_cast<qulonglong>(report.boundaryGroupCount))
                .arg(static_cast<qulonglong>(report.analyzedBoundaryCount))
                .arg(static_cast<qulonglong>(report.topologyBuildCount))
                .arg(static_cast<qulonglong>(report.topologyReuseCount))
                .arg(static_cast<qulonglong>(report.topologyMetrics.topologyRecordCount))
                .arg(static_cast<qulonglong>(report.topologyMetrics.totalPathPointCount))
                .arg(static_cast<qulonglong>(report.topologyMetrics.totalSegmentCount))
                .arg(static_cast<qulonglong>(report.topologyMetrics.recordPairCount))
                .arg(static_cast<qulonglong>
                    (report.topologyMetrics.recordPairBroadPhaseRejectedCount))
                .arg(static_cast<qulonglong>
                    (report.topologyMetrics.recordPairPreciseTestCount))
                .arg(static_cast<qulonglong>(report.topologyMetrics.endpointToPathTestCount))
                .arg(static_cast<qulonglong>(report.topologyMetrics.segmentPairTestCount))
                .arg(static_cast<qulonglong>(report.topologyMetrics.connectedRecordPairCount))
                .arg(static_cast<qulonglong>(report.topologyMetrics.adjacencyEdgeCount))
                .arg(static_cast<qulonglong>(report.rebuiltPathCount))
                .arg(static_cast<qulonglong>(report.reusedPathCount))
                .arg(static_cast<qulonglong>(report.classifiedEntityCount))
                .arg(static_cast<qulonglong>(report.classificationCallCount))
                .arg(static_cast<qulonglong>(report.samplePointCount)));
            m_activeReport = nullptr;
        }

    private:
        BoundaryAssignmentPerformanceReport*& m_activeReport;
        std::optional<BoundaryAssignmentPerformanceReport> m_ownedReport;
        QElapsedTimer m_totalTimer;
        bool m_owner = false;
    };

    std::function<void(double)> pathRebuildObserver
        (BoundaryAssignmentPerformanceReport* report)
    {
        if (report == nullptr) return {};
        return [report](double elapsedMs)
        {
            report->pathRebuildMs += elapsedMs;
            ++report->rebuiltPathCount;
        };
    }

    QString firstDiagnosticMessage(const QVector<Diagnostic>& diagnostics)
    {
        for (const Diagnostic& diagnostic : diagnostics)
        {
            if (isErrorSeverity(diagnostic.severity))
            {
                return !diagnostic.userMessage.isEmpty()
                    ? diagnostic.userMessage : diagnostic.technicalDetail;
            }
        }
        return diagnostics.isEmpty() ? QString() : diagnostics.front().technicalDetail;
    }

    RotaryCutBoundaryAnalysis analyzeBoundary
    (
        const QVector<CadItem*>& candidateItems,
        const RotaryPathTopology& topology,
        const RotaryTubeSectionModel& sectionModel,
        double connectionTolerance,
        BoundaryAssignmentPerformanceReport* report
    )
    {
        BoundaryPerformanceTimer timer
            (report != nullptr ? &report->boundaryAnalysisMs : nullptr);
        if (report != nullptr)
        {
            ++report->analyzedBoundaryCount;
            ++report->topologyReuseCount;
        }
        return RotaryCutBoundaryAnalyzer::analyze
        (
            candidateItems,
            topology,
            sectionModel,
            connectionTolerance
        );
    }

    void updateBoundaryViewer
    (
        CadViewer* viewer,
        BoundaryAssignmentPerformanceReport* report
    )
    {
        BoundaryPerformanceTimer timer
            (report != nullptr ? &report->viewerRefreshMs : nullptr);
        if (viewer != nullptr) viewer->update();
    }

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

bool Gcode_postprocessing_system::sortEntitiesByCurrentMode
(cadcam::planning::ProcessSortIntent sortIntent)
{
    const bool rebuildSequence =
        sortIntent == cadcam::planning::ProcessSortIntent::RebuildSequence;
    const GGenerator::GenerationMode generationMode = resolveGenerationMode();
    if (generationMode == GGenerator::GenerationMode::Mode3D)
    {
        return sortEntitiesWithProcessPlan3D
        (
            rebuildSequence ? QStringLiteral("4轴(绕A)智能排序")
                : QStringLiteral("4轴(绕A)排序"),
            sortIntent
        );
    }

    const int excludedCount = refreshWasteProcessingExclusions();

    if (excludedCount > 0)
    {
        ui->openGLWidget->appendCommandMessage
        (
            QStringLiteral("废面规则已排除 %1 个图元，本次排序不会处理这些图元。").arg(excludedCount)
        );
    }

    return sortEntitiesWithProcessPlan2D
    (
        rebuildSequence ? QStringLiteral("3轴智能排序") : QStringLiteral("3轴排序"),
        sortIntent
    );
}

OperationResult<RotaryBoundaryOperationGeometry>
Gcode_postprocessing_system::buildRotaryBoundaryOperationGeometry
(
    const QVector<CadItem*>& requiredItems
) const
{
    OperationResult<RotaryBoundaryOperationGeometry> result;
    RotaryBoundaryOperationGeometry geometry;
    geometry.documentRevision = m_document.contentRevision();
    geometry.connectionTolerance = kEndCutConnectionTolerance;
    geometry.sceneItems.reserve(static_cast<qsizetype>(m_document.m_entities.size()));

    const QSet<CadItem*> requiredSet(requiredItems.begin(), requiredItems.end());
    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity == nullptr)
        {
            continue;
        }
        const auto processState = m_processState.stateOrDefault(entity->m_entityId);
        if (processState.overrideData.boundaryRole != cadcam::planning::BoundaryRole::None
            || requiredSet.contains(entity.get()))
        {
            geometry.sceneItems.push_back(entity.get());
        }
    }

    {
        BoundaryPerformanceTimer topologyTimer
            (m_boundaryAssignmentPerformanceReport != nullptr
                ? &m_boundaryAssignmentPerformanceReport->topologyBuildMs : nullptr);
        if (m_boundaryAssignmentPerformanceReport != nullptr)
        {
            ++m_boundaryAssignmentPerformanceReport->topologyBuildCount;
        }
        geometry.topology = std::make_unique<RotaryPathTopology>
        (
            geometry.sceneItems,
            RotaryPathTopologyTolerance::fromConnectionTolerance
                (geometry.connectionTolerance),
            pathRebuildObserver(m_boundaryAssignmentPerformanceReport),
            m_boundaryAssignmentPerformanceReport != nullptr
                ? &m_boundaryAssignmentPerformanceReport->topologyMetrics : nullptr
        );
    }

    if (geometry.documentRevision != m_document.contentRevision())
    {
        result.status = OperationStatus::Conflict;
        Diagnostic diagnostic;
        diagnostic.code = DiagnosticCode::TopologyBuildFailure;
        diagnostic.severity = DiagnosticSeverity::Error;
        diagnostic.component = QStringLiteral("BoundaryAssignment");
        diagnostic.operation = QStringLiteral("BuildOperationGeometry");
        diagnostic.stage = QStringLiteral("RevisionValidation");
        diagnostic.userMessage = QStringLiteral("文档已发生变化，请重新执行加工断面操作。");
        diagnostic.technicalDetail = QStringLiteral
            ("document revision changed while building operation topology");
        diagnostic.context.insert(QStringLiteral("capturedRevision"),
            static_cast<qulonglong>(geometry.documentRevision));
        diagnostic.context.insert(QStringLiteral("currentRevision"),
            static_cast<qulonglong>(m_document.contentRevision()));
        result.addDiagnostic(diagnostic);
        return result;
    }

    result.status = geometry.topology->status();
    result.diagnostics = geometry.topology->diagnostics();
    if (result.status != OperationStatus::Success
        && result.status != OperationStatus::PartialSuccess)
    {
        return result;
    }

    result.value.emplace(std::move(geometry));
    return result;
}

QVector<CadItem*> Gcode_postprocessing_system::expandedSelectedRotaryEndCut
(
    const RotaryBoundaryOperationGeometry& geometry,
    QString* errorMessage
) const
{
    BoundaryPerformanceTimer selectionTimer
        (m_boundaryAssignmentPerformanceReport != nullptr
            ? &m_boundaryAssignmentPerformanceReport->selectionExpansionMs : nullptr);
    const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();

    if (selectedItems.isEmpty())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("请先选中加工断面中的一个或部分图元。系统会自动扩展相连图元。");
        }

        return {};
    }

    cadcam::core::emitSummaryLog(QStringLiteral("断面候选"), QString(), QStringLiteral("选择集：%1")
        .arg(describeRotaryPathItems(selectedItems)));
    cadcam::core::emitSummaryLog(QStringLiteral("断面候选"), QString(), QStringLiteral("sceneItems 过滤后：%1")
        .arg(describeRotaryPathItems(geometry.sceneItems)));
    QVector<CadItem*> connectedItems;
    RotaryPathLoopResult loop;
    {
        BoundaryPerformanceTimer orderingTimer
            (m_boundaryAssignmentPerformanceReport != nullptr
                ? &m_boundaryAssignmentPerformanceReport->boundaryOrderingMs : nullptr);
        if (m_boundaryAssignmentPerformanceReport != nullptr)
        {
            ++m_boundaryAssignmentPerformanceReport->topologyReuseCount;
        }
        loop = geometry.topology->extractSeededLoop(selectedItems, &connectedItems);
    }
    cadcam::core::emitSummaryLog(QStringLiteral("断面候选"), QString(), QStringLiteral("连通扩展：%1")
        .arg(describeRotaryPathItems(connectedItems)));

    if (!loop.valid || loop.usedItems.isEmpty())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = loop.errorMessage;
        }

        return {};
    }

    const RotaryCutBoundaryAnalysis analysis = analyzeBoundary
    (
        loop.usedItems,
        *geometry.topology,
        m_rotaryTubeSectionModel,
        kEndCutConnectionTolerance,
        m_boundaryAssignmentPerformanceReport
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
    BoundaryAssignmentPerformanceOperation performance
    (
        m_boundaryAssignmentPerformanceReport,
        QStringLiteral("AssignRotaryEndCut"),
        static_cast<std::uint64_t>(m_document.m_entities.size()),
        static_cast<std::uint64_t>(ui->openGLWidget->selectedEntities().size())
    );
    const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();
    auto operationGeometry = buildRotaryBoundaryOperationGeometry(selectedItems);
    if (!operationGeometry.succeeded() || !operationGeometry.value.has_value())
    {
        QMessageBox::warning(this, QStringLiteral("指定加工断面"),
            firstDiagnosticMessage(operationGeometry.diagnostics));
        return false;
    }
    QString selectionError;
    const QVector<CadItem*> expandedItems = expandedSelectedRotaryEndCut
        (*operationGeometry.value, &selectionError);

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

    {
        BoundaryPerformanceTimer stateTimer
            (m_boundaryAssignmentPerformanceReport != nullptr
                ? &m_boundaryAssignmentPerformanceReport->processStateUpdateMs : nullptr);
        m_processState.beginBatch();
        for (CadItem* item : expandedItems)
        {
            if (item == nullptr)
            {
                continue;
            }

            m_processState.setBoundary
                (item->m_entityId, cadcam::planning::BoundaryRole::Break, boundaryId);
        }
        m_processState.endBatch();
    }

    const QString message = QStringLiteral("已指定加工断面 断%1，共识别 %2 个相连图元，已通过方管周向分离验证。")
        .arg(boundaryId + 1)
        .arg(expandedItems.size());

    invalidateProcessOrdersAfterEndCutChange();
    refreshWasteProcessingExclusions(*operationGeometry.value);
    ui->openGLWidget->appendCommandMessage(message);
    updateBoundaryViewer(ui->openGLWidget, m_boundaryAssignmentPerformanceReport);
    statusBar()->showMessage(message, 5000);
    return true;
}

bool Gcode_postprocessing_system::assignSelectedWasteEndCut()
{
    BoundaryAssignmentPerformanceOperation performance
    (
        m_boundaryAssignmentPerformanceReport,
        QStringLiteral("AssignWasteEndCut"),
        static_cast<std::uint64_t>(m_document.m_entities.size()),
        static_cast<std::uint64_t>(ui->openGLWidget->selectedEntities().size())
    );
    const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();
    auto operationGeometry = buildRotaryBoundaryOperationGeometry(selectedItems);
    if (!operationGeometry.succeeded() || !operationGeometry.value.has_value())
    {
        QMessageBox::warning(this, QStringLiteral("指定废面"),
            firstDiagnosticMessage(operationGeometry.diagnostics));
        return false;
    }
    QString selectionError;
    const QVector<CadItem*> expandedItems = expandedSelectedRotaryEndCut
        (*operationGeometry.value, &selectionError);

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

    {
        BoundaryPerformanceTimer stateTimer
            (m_boundaryAssignmentPerformanceReport != nullptr
                ? &m_boundaryAssignmentPerformanceReport->processStateUpdateMs : nullptr);
        m_processState.beginBatch();
        for (CadItem* item : expandedItems)
        {
            if (item == nullptr)
            {
                continue;
            }

            m_processState.setBoundary
                (item->m_entityId, cadcam::planning::BoundaryRole::Waste, wasteId);
        }
        m_processState.endBatch();
    }

    invalidateProcessOrdersAfterEndCutChange();
    const int excludedCount = refreshWasteProcessingExclusions(*operationGeometry.value);
    const QString message = QStringLiteral("已指定废弃面 W%1，共识别 %2 个相连图元，已通过方管周向分离验证；当前废弃区共排除 %3 个图元。")
        .arg(wasteId + 1)
        .arg(expandedItems.size())
        .arg(excludedCount);
    ui->openGLWidget->appendCommandMessage(message);
    updateBoundaryViewer(ui->openGLWidget, m_boundaryAssignmentPerformanceReport);
    statusBar()->showMessage(message, 6000);
    return true;
}

bool Gcode_postprocessing_system::smartAssignSelectedRotaryEndCut()
{
    BoundaryAssignmentPerformanceOperation performance
    (
        m_boundaryAssignmentPerformanceReport,
        QStringLiteral("SmartAssignRotaryEndCut"),
        static_cast<std::uint64_t>(m_document.m_entities.size()),
        static_cast<std::uint64_t>(ui->openGLWidget->selectedEntities().size())
    );
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

    recognized.setUserCenter(m_rotaryTubeSectionModel.userCenter);
    m_rotaryTubeSectionModel = recognized;
    invalidateProcessOrdersAfterEndCutChange();
    syncToolPanelState();
    const QString message = QStringLiteral("方管垂直截面识别完成，共提取 %1 个外轮廓点；内部线和无用支线已忽略。")
        .arg(recognized.sectionBoundary.size());
    ui->openGLWidget->appendCommandMessage(message);
    statusBar()->showMessage(message, 5000);
    return true;
}

bool Gcode_postprocessing_system::setRotaryTubeSectionUserCenter
    (std::optional<cadcam::geometry::Vector2d> center)
{
    const cadcam::geometry::Vector2d previous =
        m_rotaryTubeSectionModel.effectiveCenter();
    if (!m_rotaryTubeSectionModel.setUserCenter(center)) return false;
    const cadcam::geometry::Vector2d current =
        m_rotaryTubeSectionModel.effectiveCenter();
    if (previous.x == current.x && previous.y == current.y) return true;

    invalidateProcessOrdersAfterEndCutChange();
    syncToolPanelState();
    syncMachiningSettingsState();
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

    const RotaryInternalPathResult result =
        RotaryTubeGeometryAnalyzer::findInternalItemsByWindow
            (m_rotaryTubeSectionModel, sceneItems);
    if (!result.sectionAvailable)
    {
        const QString message =
            QStringLiteral("未识别有效方管截面，未执行内部线条清理。");
        cadcam::core::emitSummaryLog
        (
            QStringLiteral("InternalPathWindow"),
            QStringLiteral("Status"),
            QStringLiteral("sectionAvailable=false windowCollapsed=false "
                "removedEntityCount=0 candidatePathCount=0"));
        ui->openGLWidget->appendCommandMessage(message);
        statusBar()->showMessage(message, 6000);
        return false;
    }
    if (result.windowCollapsed)
    {
        for (const Diagnostic& diagnostic : result.diagnostics)
        {
            if (!diagnostic.userMessage.isEmpty())
            {
                ui->openGLWidget->appendCommandMessage(diagnostic.userMessage);
            }
        }
        cadcam::core::emitSummaryLog
        (
            QStringLiteral("InternalPathWindow"),
            QStringLiteral("Status"),
            QStringLiteral("sectionAvailable=true windowCollapsed=true "
                "insetDistance=%1 removedEntityCount=0")
                .arg(result.insetDistance, 0, 'f', 6));
        statusBar()->showMessage
            (QStringLiteral("内部线条清理窗口已坍缩，未删除任何图元。"), 6000);
        return false;
    }

    QVector<CadItem*> removableItems;
    for (CadItem* item : result.removableItems)
    {
        if (item != nullptr)
        {
            removableItems.push_back(item);
        }
    }

    if (!removableItems.isEmpty()
        && !m_editer.deleteEntities(removableItems))
    {
        const QString message = QStringLiteral("内部线条图元删除失败。");
        ui->openGLWidget->appendCommandMessage(message);
        statusBar()->showMessage(message, 6000);
        return false;
    }

    if (!removableItems.isEmpty())
    {
        cadcam::core::emitSummaryLog
        (
            QStringLiteral("InternalPathWindow"),
            QStringLiteral("DeleteResult"),
            QStringLiteral("deletedEntityCount=%1")
                .arg(removableItems.size())
        );
        invalidateProcessOrdersAfterEndCutChange();
        refreshWasteProcessingExclusions();
        updateBoundaryViewer(ui->openGLWidget, m_boundaryAssignmentPerformanceReport);
    }

    const QString message = removableItems.isEmpty()
        ? QStringLiteral("内部线条清理完成：内缩窗口内没有与其相交的图元（候选 %1，窗口外保留 %2，跳过 %3）。")
            .arg(result.candidatePathCount)
            .arg(result.outsideWindowCount)
            .arg(result.skippedPathCount)
        : QStringLiteral("内部线条清理完成：按最大圆角半径 %1 mm 内缩生成窗口（YZ 半宽 %2×%3），已删除 %4 个相交图元，可按 Ctrl+Z 撤销。")
            .arg(result.insetDistance, 0, 'f', 3)
            .arg(result.windowHalfY, 0, 'f', 3)
            .arg(result.windowHalfZ, 0, 'f', 3)
            .arg(removableItems.size());
    cadcam::core::emitSummaryLog
    (
        QStringLiteral("InternalPathWindow"),
        QStringLiteral("Status"),
        QStringLiteral("sectionAvailable=true windowCollapsed=false "
            "insetDistance=%1 windowHalfY=%2 windowHalfZ=%3 "
            "candidatePathCount=%4 skippedPathCount=%5 outsideWindowCount=%6 "
            "removedEntityCount=%7")
            .arg(result.insetDistance, 0, 'f', 6)
            .arg(result.windowHalfY, 0, 'f', 6)
            .arg(result.windowHalfZ, 0, 'f', 6)
            .arg(result.candidatePathCount)
            .arg(result.skippedPathCount)
            .arg(result.outsideWindowCount)
            .arg(removableItems.size()));
    for (const Diagnostic& diagnostic : result.diagnostics)
    {
        if (!diagnostic.userMessage.isEmpty())
        {
            ui->openGLWidget->appendCommandMessage(diagnostic.userMessage);
        }
    }
    ui->openGLWidget->appendCommandMessage(message);
    statusBar()->showMessage(message, 6000);
    return true;
}

bool Gcode_postprocessing_system::toggleSelectedRotaryEndCutAssignment()
{
    const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();
    const bool hasUnassignedItem = std::any_of
    (
        selectedItems.begin(), selectedItems.end(), [this](const CadItem* item)
        {
            return item != nullptr && m_processState.stateOrDefault(item->m_entityId)
                .overrideData.boundaryRole == cadcam::planning::BoundaryRole::None;
        }
    );
    const bool hasAssignedItem = std::any_of
    (
        selectedItems.begin(), selectedItems.end(), [this](const CadItem* item)
        {
            return item != nullptr && m_processState.stateOrDefault(item->m_entityId)
                .overrideData.boundaryRole != cadcam::planning::BoundaryRole::None;
        }
    );
    const QString operation = selectedItems.isEmpty()
        ? QStringLiteral("ToggleRotaryEndCut")
        : (!hasUnassignedItem ? QStringLiteral("ClearSelectedRotaryEndCut")
            : (hasAssignedItem ? QStringLiteral("ReassignRotaryEndCut")
                : QStringLiteral("AssignRotaryEndCut")));
    BoundaryAssignmentPerformanceOperation performance
    (
        m_boundaryAssignmentPerformanceReport,
        operation,
        static_cast<std::uint64_t>(m_document.m_entities.size()),
        static_cast<std::uint64_t>(selectedItems.size())
    );

    if (selectedItems.isEmpty())
    {
        return false;
    }

    if (!hasUnassignedItem)
    {
        return clearSelectedRotaryEndCutAssignments();
    }

    auto operationGeometry = buildRotaryBoundaryOperationGeometry(selectedItems);
    if (!operationGeometry.succeeded() || !operationGeometry.value.has_value())
    {
        QMessageBox::warning(this, QStringLiteral("加工断面指定"),
            firstDiagnosticMessage(operationGeometry.diagnostics));
        return false;
    }
    QString errorMessage;
    const QVector<CadItem*> boundaryItems = expandedSelectedRotaryEndCut
        (*operationGeometry.value, &errorMessage);

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

    {
        BoundaryPerformanceTimer stateTimer
            (m_boundaryAssignmentPerformanceReport != nullptr
                ? &m_boundaryAssignmentPerformanceReport->processStateUpdateMs : nullptr);
        m_processState.beginBatch();
        for (CadItem* item : boundaryItems)
        {
            if (item != nullptr) m_processState.setBoundary
                (item->m_entityId, cadcam::planning::BoundaryRole::Break, nextBoundaryId);
        }
        m_processState.endBatch();
    }

    invalidateProcessOrdersAfterEndCutChange();
    refreshWasteProcessingExclusions(*operationGeometry.value);
    const QString message = QStringLiteral("已指定加工断面 %1，共 %2 个图元。")
        .arg(nextBoundaryId + 1)
        .arg(boundaryItems.size());
    ui->openGLWidget->appendCommandMessage(message);
    updateBoundaryViewer(ui->openGLWidget, m_boundaryAssignmentPerformanceReport);
    statusBar()->showMessage(message, 5000);
    return true;
}

bool Gcode_postprocessing_system::recognizeAllRotaryEndCuts(bool interactive)
{
    BoundaryAssignmentPerformanceOperation performance
    (
        m_boundaryAssignmentPerformanceReport,
        QStringLiteral("RecognizeAllRotaryEndCuts"),
        static_cast<std::uint64_t>(m_document.m_entities.size()),
        static_cast<std::uint64_t>(ui->openGLWidget->selectedEntities().size())
    );
    auto operationGeometry = buildRotaryBoundaryOperationGeometry();
    if (!operationGeometry.succeeded() || !operationGeometry.value.has_value())
    {
        const QString message = firstDiagnosticMessage(operationGeometry.diagnostics);
        ui->openGLWidget->appendCommandMessage(message);
        statusBar()->showMessage(message, 5000);
        return false;
    }
    const QVector<CadItem*>& documentItems = operationGeometry.value->sceneItems;

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

    cadcam::core::emitSummaryLog(QStringLiteral("断面候选"), QString(), QStringLiteral("sceneItems 过滤后：%1")
        .arg(describeRotaryPathItems(operationGeometry.value->sceneItems)));
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
        RotaryPathLoopResult loop;
        {
            BoundaryPerformanceTimer orderingTimer
                (m_boundaryAssignmentPerformanceReport != nullptr
                    ? &m_boundaryAssignmentPerformanceReport->boundaryOrderingMs : nullptr);
            if (m_boundaryAssignmentPerformanceReport != nullptr)
            {
                ++m_boundaryAssignmentPerformanceReport->topologyReuseCount;
            }
            loop = operationGeometry.value->topology->extractSeededLoop
                (seedItems, &connectedItems);
        }

        if (!loop.valid || loop.usedItems.isEmpty())
        {
            cadcam::core::emitSummaryLog(QStringLiteral("自动识别加工断面"), QString(), QStringLiteral("种子 %1 提取失败：%2")
                .arg(describeRotaryPathItems(seedItems))
                .arg(loop.errorMessage));
            continue;
        }

        cadcam::core::emitSummaryLog(QStringLiteral("断面候选"), QString(), QStringLiteral("选择集：%1")
            .arg(describeRotaryPathItems(seedItems)));
        cadcam::core::emitSummaryLog(QStringLiteral("断面候选"), QString(), QStringLiteral("连通扩展：%1")
            .arg(describeRotaryPathItems(connectedItems)));

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

        const RotaryCutBoundaryAnalysis analysis = analyzeBoundary
        (
            loop.usedItems,
            *operationGeometry.value->topology,
            m_rotaryTubeSectionModel,
            kEndCutConnectionTolerance,
            m_boundaryAssignmentPerformanceReport
        );

        if (!analysis.valid)
        {
            cadcam::core::emitSummaryLog(QStringLiteral("自动识别加工断面"), QString(), QStringLiteral("周向验证失败：%1")
                .arg(analysis.errorMessage));
            continue;
        }

        const QVector<CadItem*>& recognizedItems = analysis.boundaryItems.isEmpty()
            ? loop.usedItems
            : analysis.boundaryItems;

        {
            BoundaryPerformanceTimer stateTimer
                (m_boundaryAssignmentPerformanceReport != nullptr
                    ? &m_boundaryAssignmentPerformanceReport->processStateUpdateMs : nullptr);
            m_processState.beginBatch();
            for (CadItem* item : recognizedItems)
            {
                if (item != nullptr) m_processState.setBoundary
                    (item->m_entityId, cadcam::planning::BoundaryRole::Break, nextBoundaryId);
            }
            m_processState.endBatch();
        }

        ++nextBoundaryId;
        ++recognizedCount;
    }

    const QString message = QStringLiteral("所有加工断面识别完成，共识别 %1 个有效加工断面。")
        .arg(recognizedCount);

    if (recognizedCount > 0)
    {
        invalidateProcessOrdersAfterEndCutChange();
        refreshWasteProcessingExclusions(*operationGeometry.value);
    }

    ui->openGLWidget->appendCommandMessage(message);
    updateBoundaryViewer(ui->openGLWidget, m_boundaryAssignmentPerformanceReport);
    statusBar()->showMessage(message, 5000);

    if (interactive && recognizedCount == 0)
    {
        QMessageBox::information(this, QStringLiteral("识别加工断面"), QStringLiteral("未识别到有效加工断面。"));
    }

    syncMachiningSettingsState();
    return recognizedCount > 0;
}

bool Gcode_postprocessing_system::clearSelectedRotaryEndCutAssignments()
{
    const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();
    BoundaryAssignmentPerformanceOperation performance
    (
        m_boundaryAssignmentPerformanceReport,
        QStringLiteral("ClearSelectedRotaryEndCut"),
        static_cast<std::uint64_t>(m_document.m_entities.size()),
        static_cast<std::uint64_t>(selectedItems.size())
    );
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

    {
        BoundaryPerformanceTimer stateTimer
            (m_boundaryAssignmentPerformanceReport != nullptr
                ? &m_boundaryAssignmentPerformanceReport->processStateUpdateMs : nullptr);
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
                m_processState.setBoundary
                    (entity->m_entityId, cadcam::planning::BoundaryRole::None, -1);
                ++clearedCount;
            }
        }
        m_processState.endBatch();
    }

    const QString message = QStringLiteral("已清除 %1 个加工断面边界，共 %2 个图元。")
        .arg(pairIds.size())
        .arg(clearedCount);
    invalidateProcessOrdersAfterEndCutChange();
    refreshWasteProcessingExclusions();
    ui->openGLWidget->appendCommandMessage(message);
    updateBoundaryViewer(ui->openGLWidget, m_boundaryAssignmentPerformanceReport);
    statusBar()->showMessage(message, 5000);
    return true;
}

bool Gcode_postprocessing_system::clearRotaryEndCutAssignments()
{
    BoundaryAssignmentPerformanceOperation performance
    (
        m_boundaryAssignmentPerformanceReport,
        QStringLiteral("ClearAllRotaryEndCuts"),
        static_cast<std::uint64_t>(m_document.m_entities.size()),
        static_cast<std::uint64_t>(ui->openGLWidget->selectedEntities().size())
    );
    int clearedCount = 0;

    {
        BoundaryPerformanceTimer stateTimer
            (m_boundaryAssignmentPerformanceReport != nullptr
                ? &m_boundaryAssignmentPerformanceReport->processStateUpdateMs : nullptr);
        m_processState.beginBatch();
        for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
        {
            if (entity == nullptr || m_processState.stateOrDefault(entity->m_entityId)
                .overrideData.boundaryRole == cadcam::planning::BoundaryRole::None)
            {
                continue;
            }
            m_processState.setBoundary
                (entity->m_entityId, cadcam::planning::BoundaryRole::None, -1);
            ++clearedCount;
        }
        m_processState.endBatch();
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
    updateBoundaryViewer(ui->openGLWidget, m_boundaryAssignmentPerformanceReport);
    statusBar()->showMessage(message, 5000);
    return true;
}

void Gcode_postprocessing_system::invalidateProcessOrdersAfterEndCutChange()
{
    invalidateCurrentProcessPlan();
}

void Gcode_postprocessing_system::invalidateCurrentProcessPlan()
{
    {
        BoundaryPerformanceTimer stateTimer
            (m_boundaryAssignmentPerformanceReport != nullptr
                ? &m_boundaryAssignmentPerformanceReport->processStateUpdateMs : nullptr);
        m_currentProcessPlan.reset();
        m_processPresentation.reset();
    }
    if (ui != nullptr && ui->openGLWidget != nullptr)
    {
        BoundaryPerformanceTimer viewerTimer
            (m_boundaryAssignmentPerformanceReport != nullptr
                ? &m_boundaryAssignmentPerformanceReport->viewerRefreshMs : nullptr);
        ui->openGLWidget->setProcessPresentation(nullptr);
        ui->openGLWidget->update();
    }
}

bool Gcode_postprocessing_system::applyProcessUnitSequenceToCurrentPlan
(
    const cadcam::planning::ProcessUnitSequence& sequence,
    QString* errorMessage
)
{
    const auto fail = [errorMessage](const QString& message)
    {
        if (errorMessage != nullptr) *errorMessage = message;
        return false;
    };

    if (!m_currentProcessPlan.has_value()
        || m_currentProcessPlan->contentRevision != m_document.contentRevision()
        || m_currentProcessPlan->processStateRevision != m_processState.revision()
        || m_currentProcessPlan->processUnitSequence.units
            != m_processState.processUnitSequence().units)
    {
        return fail(QStringLiteral("当前加工计划已失效，请重新排序后再调整加工单元。"));
    }

    cadcam::planning::ProcessUnitSequence targetSequence = sequence;
    targetSequence.revision = m_processState.processUnitSequence().revision + 1U;
    const OperationContext context = createOperationContext
        (QStringLiteral("ReorderCurrentProcessUnitSequence"));
    ProcessPlanningService service;
    auto reordered = service.reorderPlanByUnitSequence
        (*m_currentProcessPlan, targetSequence, context);
    if (!reordered.succeeded() || !reordered.value.has_value())
    {
        QString message = QStringLiteral("加工单元重排不符合当前加工约束。");
        for (const Diagnostic& diagnostic : reordered.diagnostics)
        {
            if (!diagnostic.userMessage.trimmed().isEmpty())
            {
                message = diagnostic.userMessage;
                break;
            }
        }
        return fail(message);
    }

    const std::uint64_t expectedProcessRevision = m_processState.revision() + 1U;
    cadcam::planning::ProcessPlan candidatePlan = std::move(*reordered.value);
    candidatePlan.processStateRevision = expectedProcessRevision;
    auto candidatePresentation = cadcam::process::ProcessPresentationSnapshot::build
        (candidatePlan, context);
    if (!candidatePresentation.succeeded() || !candidatePresentation.value.has_value())
    {
        return fail(QStringLiteral("加工单元重排后的显示快照构建失败。"));
    }

    if (!m_processState.setProcessUnitSequence(sequence.units))
    {
        return fail(QStringLiteral("加工单元序列未发生变化或状态更新失败。"));
    }

    candidatePlan.processUnitSequence = m_processState.processUnitSequence();
    candidatePlan.processStateRevision = m_processState.revision();
    candidatePresentation.value->processStateRevision = m_processState.revision();
    m_currentProcessPlan = std::move(candidatePlan);
    m_processPresentation = std::move(*candidatePresentation.value);
    ui->openGLWidget->setProcessPresentation(&*m_processPresentation);
    ui->openGLWidget->update();
    return true;
}

bool Gcode_postprocessing_system::applyProcessUnitTraversalToCurrentPlan
(
    const cadcam::planning::ProcessUnitKey& key,
    const cadcam::process::ProcessUnitTraversalOverride& traversal,
    const std::optional<cadcam::process::ProcessUnitTraversalOverride>& storedOverride,
    QString* errorMessage
)
{
    const auto fail = [errorMessage](const QString& message)
    {
        if (errorMessage != nullptr) *errorMessage = message;
        return false;
    };
    if (!m_currentProcessPlan.has_value()
        || m_currentProcessPlan->contentRevision != m_document.contentRevision()
        || m_currentProcessPlan->processStateRevision != m_processState.revision()
        || m_currentProcessPlan->processUnitSequence.units
            != m_processState.processUnitSequence().units)
    {
        return fail(QStringLiteral("当前加工计划已失效，请重新排序后再调整加工单元。"));
    }

    std::optional<cadcam::process::ProcessUnitTraversalOverride> currentStored;
    if (const auto* value = m_processState.findProcessUnitTraversalOverride(key))
        currentStored = *value;
    const bool stateWillChange = currentStored != storedOverride;
    const std::uint64_t expectedProcessRevision = m_processState.revision()
        + (stateWillChange ? 1U : 0U);

    const OperationContext context = createOperationContext
        (QStringLiteral("ReverseCurrentProcessUnitTraversal"));
    ProcessPlanningService service;
    auto updated = service.applyPlanUnitTraversal
        (*m_currentProcessPlan, key, traversal, context);
    if (!updated.succeeded() || !updated.value.has_value())
    {
        QString message = QStringLiteral("加工单元整组反向不符合当前加工约束。");
        for (const Diagnostic& diagnostic : updated.diagnostics)
        {
            if (!diagnostic.userMessage.trimmed().isEmpty())
            {
                message = diagnostic.userMessage;
                break;
            }
        }
        return fail(message);
    }

    cadcam::planning::ProcessPlan candidatePlan = std::move(*updated.value);
    candidatePlan.processStateRevision = expectedProcessRevision;
    auto candidatePresentation = cadcam::process::ProcessPresentationSnapshot::build
        (candidatePlan, context);
    if (!candidatePresentation.succeeded() || !candidatePresentation.value.has_value())
    {
        return fail(QStringLiteral("加工单元反向后的显示快照构建失败。"));
    }

    if (stateWillChange
        && !m_processState.setProcessUnitTraversalOverride(key, storedOverride))
    {
        return fail(QStringLiteral("加工单元遍历覆盖更新失败。"));
    }

    candidatePlan.processStateRevision = m_processState.revision();
    candidatePresentation.value->processStateRevision = m_processState.revision();
    m_currentProcessPlan = std::move(candidatePlan);
    m_processPresentation = std::move(*candidatePresentation.value);
    ui->openGLWidget->setProcessPresentation(&*m_processPresentation);
    ui->openGLWidget->update();
    return true;
}

void Gcode_postprocessing_system::handleProcessUnitReverseRequest
    (const cadcam::planning::ProcessUnitKey& unitKey)
{
    const auto announce = [this](const QString& message)
    {
        ui->openGLWidget->appendCommandMessage(message);
        ui->openGLWidget->refreshCommandPrompt();
        statusBar()->showMessage(message, 5000);
    };
    if (!m_currentProcessPlan.has_value())
    {
        announce(QStringLiteral("当前没有有效加工计划，请先排序。"));
        return;
    }

    const auto unit = std::find_if
    (
        m_currentProcessPlan->processUnits.begin(),
        m_currentProcessPlan->processUnits.end(),
        [&unitKey](const cadcam::planning::ProcessUnit& candidate)
        { return candidate.key == unitKey; }
    );
    if (unit == m_currentProcessPlan->processUnits.end())
    {
        announce(QStringLiteral("目标加工单元不属于当前计划。"));
        return;
    }

    std::map<cadcam::geometry::EntityId, const cadcam::planning::ProcessAssignment*>
        assignmentsByEntity;
    for (const cadcam::planning::ProcessAssignment& assignment
        : m_currentProcessPlan->assignments)
        assignmentsByEntity.emplace(assignment.entityId, &assignment);

    cadcam::process::ProcessUnitTraversalOverride beforeTraversal;
    beforeTraversal.members.reserve(unit->orderedMemberEntityIds.size());
    for (const cadcam::geometry::EntityId entityId : unit->orderedMemberEntityIds)
    {
        const auto assignment = assignmentsByEntity.find(entityId);
        if (assignment == assignmentsByEntity.end())
        {
            announce(QStringLiteral("加工单元成员缺少计划分配，无法整组反向。"));
            return;
        }
        beforeTraversal.members.push_back
        ({ entityId, assignment->second->reverse, assignment->second->startParameter });
    }

    cadcam::process::ProcessUnitTraversalOverride afterTraversal = beforeTraversal;
    std::reverse(afterTraversal.members.begin(), afterTraversal.members.end());
    for (auto& member : afterTraversal.members) member.reverse = !member.reverse;

    std::optional<cadcam::process::ProcessUnitTraversalOverride> beforeStored;
    if (const auto* value = m_processState.findProcessUnitTraversalOverride(unitKey))
        beforeStored = *value;
    const std::optional<cadcam::process::ProcessUnitTraversalOverride> afterStored =
        afterTraversal;

    const auto applyTraversal = [this, announce, unitKey]
    (
        const cadcam::process::ProcessUnitTraversalOverride& traversal,
        const std::optional<cadcam::process::ProcessUnitTraversalOverride>& stored
    )
    {
        QString errorMessage;
        const bool success = applyProcessUnitTraversalToCurrentPlan
            (unitKey, traversal, stored, &errorMessage);
        if (!success) announce(errorMessage);
        return success;
    };
    const bool applied = m_editer.executeUndoableAction
    (
        [applyTraversal, afterTraversal, afterStored]()
        {
            return applyTraversal(afterTraversal, afterStored);
        },
        [applyTraversal, beforeTraversal, beforeStored]()
        {
            return applyTraversal(beforeTraversal, beforeStored);
        }
    );
    if (applied)
        announce(QStringLiteral("加工单元已整组反向，起点与方向显示已刷新。"));
}

void Gcode_postprocessing_system::handleProcessUnitMoveToBackRequest
(
    const QVector<cadcam::planning::ProcessUnitKey>& selectedUnitKeys,
    const cadcam::planning::ProcessUnitKey& targetUnitKey
)
{
    const auto announce = [this](const QString& message)
    {
        ui->openGLWidget->appendCommandMessage(message);
        ui->openGLWidget->refreshCommandPrompt();
        statusBar()->showMessage(message, 5000);
    };

    if (!m_currentProcessPlan.has_value())
    {
        announce(QStringLiteral("当前没有有效加工计划，请先排序。"));
        return;
    }

    const cadcam::planning::ProcessUnitSequence before =
        m_processState.processUnitSequence();
    std::map<std::vector<cadcam::geometry::EntityId>, std::size_t> indicesByKey;
    for (std::size_t index = 0; index < before.units.size(); ++index)
    {
        indicesByKey.emplace(before.units[index].memberEntityIds, index);
    }

    std::set<std::size_t> selectedIndices;
    for (const cadcam::planning::ProcessUnitKey& key : selectedUnitKeys)
    {
        const auto found = indicesByKey.find(key.memberEntityIds);
        if (found == indicesByKey.end())
        {
            announce(QStringLiteral("选中的加工单元不属于当前权威序列。"));
            return;
        }
        selectedIndices.insert(found->second);
    }

    const auto target = indicesByKey.find(targetUnitKey.memberEntityIds);
    if (selectedIndices.empty() || target == indicesByKey.end()
        || selectedIndices.find(target->second) == selectedIndices.end())
    {
        announce(QStringLiteral("请先选中包含目标标签的连续加工单元范围。"));
        return;
    }

    const std::size_t rangeFirst = *selectedIndices.begin();
    const std::size_t rangeLast = *selectedIndices.rbegin();
    if (rangeLast - rangeFirst + 1U != selectedIndices.size())
    {
        announce(QStringLiteral("选中的加工单元编号不连续，无法执行块内移尾。"));
        return;
    }
    if (target->second == rangeLast)
    {
        announce(QStringLiteral("目标加工单元已经位于所选范围末位。"));
        return;
    }

    cadcam::planning::ProcessUnitSequence after = before;
    const cadcam::planning::ProcessUnitKey moved = after.units[target->second];
    after.units.erase(after.units.begin() + static_cast<std::ptrdiff_t>(target->second));
    after.units.insert(after.units.begin() + static_cast<std::ptrdiff_t>(rangeLast), moved);
    ++after.revision;

    const auto applySequence = [this, announce]
        (const cadcam::planning::ProcessUnitSequence& requested)
    {
        QString errorMessage;
        const bool success = applyProcessUnitSequenceToCurrentPlan
            (requested, &errorMessage);
        if (!success) announce(errorMessage);
        return success;
    };
    const bool applied = m_editer.executeUndoableAction
    (
        [applySequence, after]()
        {
            return applySequence(after);
        },
        [applySequence, before]()
        {
            return applySequence(before);
        }
    );
    if (!applied)
    {
        return;
    }

    announce(QStringLiteral("加工单元 %1 已移动到所选范围末位，编号已连续刷新。")
        .arg(static_cast<qulonglong>(target->second + 1U)));
}

int Gcode_postprocessing_system::refreshWasteProcessingExclusions()
{
    const bool hasBoundary = std::any_of
    (
        m_document.m_entities.begin(), m_document.m_entities.end(),
        [this](const std::unique_ptr<CadItem>& entity)
        {
            if (entity == nullptr) return false;
            const auto state = m_processState.stateOrDefault(entity->m_entityId);
            return state.overrideData.boundaryRole != cadcam::planning::BoundaryRole::None
                && state.overrideData.boundaryPairId >= 0;
        }
    );
    if (!hasBoundary)
    {
        syncMachiningSettingsState();
        return 0;
    }

    auto operationGeometry = buildRotaryBoundaryOperationGeometry();
    if (!operationGeometry.succeeded() || !operationGeometry.value.has_value())
    {
        syncMachiningSettingsState();
        int excludedCount = 0;
        for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
        {
            if (entity != nullptr && m_processState.stateOrDefault(entity->m_entityId)
                .overrideData.boundaryRole == cadcam::planning::BoundaryRole::Waste)
            {
                ++excludedCount;
            }
        }
        return excludedCount;
    }
    return refreshWasteProcessingExclusions(*operationGeometry.value);
}

int Gcode_postprocessing_system::refreshWasteProcessingExclusions
    (const RotaryBoundaryOperationGeometry& geometry)
{
    BoundaryPerformanceTimer wasteRefreshTimer
        (m_boundaryAssignmentPerformanceReport != nullptr
            ? &m_boundaryAssignmentPerformanceReport->wasteRefreshMs : nullptr);
    if (m_boundaryAssignmentPerformanceReport != nullptr)
    {
        ++m_boundaryAssignmentPerformanceReport->topologyReuseCount;
    }
    struct BoundaryGroup
    {
        cadcam::planning::BoundaryRole role = cadcam::planning::BoundaryRole::None;
        QVector<CadItem*> items;
    };

    std::map<int, BoundaryGroup> boundaryGroups;
    for (CadItem* entity : geometry.sceneItems)
    {
        if (entity == nullptr)
        {
            continue;
        }

        const auto state = m_processState.stateOrDefault(entity->m_entityId);

        if (state.overrideData.boundaryRole == cadcam::planning::BoundaryRole::None
            || state.overrideData.boundaryPairId < 0)
        {
            continue;
        }

        const int roleIndex = static_cast<int>(state.overrideData.boundaryRole);
        const int key = state.overrideData.boundaryPairId * 4 + roleIndex;
        BoundaryGroup& group = boundaryGroups[key];
        group.role = state.overrideData.boundaryRole;
        group.items.push_back(entity);
    }

    if (m_boundaryAssignmentPerformanceReport != nullptr)
    {
        m_boundaryAssignmentPerformanceReport->boundaryGroupCount = std::max
        (
            m_boundaryAssignmentPerformanceReport->boundaryGroupCount,
            static_cast<std::uint64_t>(boundaryGroups.size())
        );
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
        boundary.analysis = analyzeBoundary
        (
            group.items,
            *geometry.topology,
            m_rotaryTubeSectionModel,
            kEndCutConnectionTolerance,
            m_boundaryAssignmentPerformanceReport
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

    bool boundariesHaveStableOrder = true;
    {
        BoundaryPerformanceTimer orderingTimer
            (m_boundaryAssignmentPerformanceReport != nullptr
                ? &m_boundaryAssignmentPerformanceReport->boundaryOrderingMs : nullptr);
        std::sort
        (
            boundaries.begin(),
            boundaries.end(),
            [](const BoundaryPosition& left, const BoundaryPosition& right)
            {
                return left.centerX < right.centerX;
            }
        );

        for (size_t boundaryIndex = 1;
            boundaryIndex < boundaries.size() && boundariesHaveStableOrder;
            ++boundaryIndex)
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

            {
                BoundaryPerformanceTimer classificationTimer
                    (m_boundaryAssignmentPerformanceReport != nullptr
                        ? &m_boundaryAssignmentPerformanceReport->pointClassificationMs
                        : nullptr);
                for (const QVector3D& point : rightBoundary.analysis.orderedPath)
                {
                    if (m_boundaryAssignmentPerformanceReport != nullptr)
                    {
                        ++m_boundaryAssignmentPerformanceReport->classificationCallCount;
                        ++m_boundaryAssignmentPerformanceReport->samplePointCount;
                    }
                    const RotaryBoundarySide side =
                        RotaryCutBoundaryAnalyzer::classifyPointRelativeToBoundary
                        (leftBoundary.analysis, point, kEndCutConnectionTolerance);
                    if (side != RotaryBoundarySide::After
                        && side != RotaryBoundarySide::OnBoundary)
                    {
                        boundariesHaveStableOrder = false;
                        break;
                    }
                }

                for (const QVector3D& point : leftBoundary.analysis.orderedPath)
                {
                    if (m_boundaryAssignmentPerformanceReport != nullptr)
                    {
                        ++m_boundaryAssignmentPerformanceReport->classificationCallCount;
                        ++m_boundaryAssignmentPerformanceReport->samplePointCount;
                    }
                    const RotaryBoundarySide side =
                        RotaryCutBoundaryAnalyzer::classifyPointRelativeToBoundary
                        (rightBoundary.analysis, point, kEndCutConnectionTolerance);
                    if (side != RotaryBoundarySide::Before
                        && side != RotaryBoundarySide::OnBoundary)
                    {
                        boundariesHaveStableOrder = false;
                        break;
                    }
                }
            }

            if (!boundariesHaveStableOrder)
            {
                break;
            }
        }
    }

    if (!boundariesHaveStableOrder)
    {
        boundaries.clear();
    }

    int excludedCount = 0;
    QHash<CadItem*, const RotaryPathTopologyRecord*> topologyRecordByItem;
    for (const RotaryPathTopologyRecord& record : geometry.topology->records())
    {
        if (record.sourceItem != nullptr)
        {
            topologyRecordByItem.insert(record.sourceItem, &record);
        }
    }

    for (CadItem* entity : geometry.sceneItems)
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

        const auto topologyRecord = topologyRecordByItem.constFind(entity);
        if (topologyRecord == topologyRecordByItem.cend()
            || (*topologyRecord)->points.isEmpty())
        {
            continue;
        }
        const QVector<QVector3D>& pathPoints = (*topologyRecord)->points;
        if (m_boundaryAssignmentPerformanceReport != nullptr)
        {
            ++m_boundaryAssignmentPerformanceReport->reusedPathCount;
        }

        size_t intervalIndex = boundaries.size();
        bool crossesBoundary = false;
        if (m_boundaryAssignmentPerformanceReport != nullptr)
        {
            ++m_boundaryAssignmentPerformanceReport->classifiedEntityCount;
        }

        {
            BoundaryPerformanceTimer classificationTimer
                (m_boundaryAssignmentPerformanceReport != nullptr
                    ? &m_boundaryAssignmentPerformanceReport->pointClassificationMs : nullptr);
            for (size_t boundaryIndex = 0; boundaryIndex < boundaries.size(); ++boundaryIndex)
            {
                bool hasPointBefore = false;
                bool hasPointAfter = false;

                for (const QVector3D& point : pathPoints)
                {
                    if (m_boundaryAssignmentPerformanceReport != nullptr)
                    {
                        ++m_boundaryAssignmentPerformanceReport->classificationCallCount;
                        ++m_boundaryAssignmentPerformanceReport->samplePointCount;
                    }
                    const RotaryBoundarySide side =
                        RotaryCutBoundaryAnalyzer::classifyPointRelativeToBoundary
                        (boundaries[boundaryIndex].analysis, point,
                            kEndCutConnectionTolerance);

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
    return sortEntitiesWithProcessPlan2D
        (QStringLiteral("3轴排序"),
            cadcam::planning::ProcessSortIntent::PreserveCurrentSequence);
}

bool Gcode_postprocessing_system::assignSelectedEntityProcessOrder()
{
    return sortEntitiesWithProcessPlan2D
        (QStringLiteral("3轴排序"),
            cadcam::planning::ProcessSortIntent::PreserveCurrentSequence);
}

bool Gcode_postprocessing_system::smartSortEntities()
{
    return sortEntitiesWithProcessPlan2D
        (QStringLiteral("3轴智能排序"),
            cadcam::planning::ProcessSortIntent::RebuildSequence);
}

bool Gcode_postprocessing_system::sortEntitiesWithProcessPlan2D
(
    const QString& commandTitle,
    cadcam::planning::ProcessSortIntent sortIntent
)
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
    policy.sortIntent = sortIntent;
    policy.allowReverse = true;
    policy.preserveUserDirection =
        sortIntent == cadcam::planning::ProcessSortIntent::PreserveCurrentSequence;
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

    cadcam::planning::ProcessPlan resolvedPlan = std::move(*plan.value);
    m_processState.setProcessUnitSequence(resolvedPlan.processUnitSequence.units);
    resolvedPlan.processUnitSequence = m_processState.processUnitSequence();
    resolvedPlan.processStateRevision = m_processState.revision();
    m_currentProcessPlan = std::move(resolvedPlan);
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
    return sortEntitiesWithProcessPlan3D
        (QStringLiteral("4轴(绕A)排序"),
            cadcam::planning::ProcessSortIntent::PreserveCurrentSequence);
}

bool Gcode_postprocessing_system::sortEntitiesWithProcessPlan3D
(
    const QString& commandTitle,
    cadcam::planning::ProcessSortIntent sortIntent
)
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
    policy.sortIntent = sortIntent;
    policy.orderingStrategy = m_activeProfile.rotaryAxisConfig().lazyRotationProcessing
        ? cadcam::planning::ProcessOrderingStrategy::LazyRotation
        : cadcam::planning::ProcessOrderingStrategy::NearestNext;
    policy.connectionTolerance = kEndCutConnectionTolerance;
    policy.rotationSafetyClearance =
        m_activeProfile.toolTransferConfig().rotationSafetyClearance;
    policy.sameZoneTransferClearance =
        m_activeProfile.toolTransferConfig().sameZoneTransferClearance;
    policy.coordinatedTransferEnabled =
        m_activeProfile.toolTransferConfig().coordinatedTransferEnabled;
    policy.rotaryAxisY = m_activeProfile.rotaryAxisConfig().centerY;
    policy.rotaryAxisZ = m_activeProfile.rotaryAxisConfig().centerZ;
    policy.invertAAxisDirection =
        m_activeProfile.rotaryAxisConfig().invertAAxisDirection;
    policy.aAxisOffsetDegrees =
        m_activeProfile.rotaryAxisConfig().aAxisOffsetDegrees;
    policy.keepContinuousAngle =
        m_activeProfile.rotaryAxisConfig().keepContinuousAngle;
    policy.useInitialMachinePoint =
        m_activeProfile.rotaryAxisConfig().useInitialMachinePoint;
    policy.initialMachinePoint =
    {
        m_activeProfile.rotaryAxisConfig().initialMachineX,
        m_activeProfile.rotaryAxisConfig().initialMachineY,
        m_activeProfile.rotaryAxisConfig().initialMachineZ,
        0.0
    };
    policy.machiningPlaneZOffset =
        m_activeProfile.rotaryAxisConfig().machiningPlaneZOffset;
    policy.overcutDistance =
        m_activeProfile.rotaryAxisConfig().overcutDistance;
    policy.allowReverse = true;
    policy.preserveClosedLoopsAsAtomicGroups = true;
    policy.initialPosition = { 0.0, 0.0, 500.0 };
    policy.zone16Sweep.initialZone =
        cadcam::machining::TubeZone16::TopFace;
    policy.zone16Sweep.perimeterDirection =
        m_activeProfile.rotaryAxisConfig().perimeterSweepDirection
            == GProfilePerimeterSweepDirection::Clockwise
        ? cadcam::planning::PerimeterSweepDirection::Clockwise
        : cadcam::planning::PerimeterSweepDirection::CounterClockwise;
    policy.zone16Sweep.longitudinalDirection =
        m_activeProfile.rotaryAxisConfig().longitudinalSweepDirection
            == GProfileLongitudinalSweepDirection::PositiveX
        ? cadcam::planning::LongitudinalSweepDirection::PositiveX
        : cadcam::planning::LongitudinalSweepDirection::NegativeX;

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

    cadcam::planning::ProcessPlan resolvedPlan = std::move(*planResult.value);
    m_processState.setProcessUnitSequence(resolvedPlan.processUnitSequence.units);
    resolvedPlan.processUnitSequence = m_processState.processUnitSequence();
    resolvedPlan.processStateRevision = m_processState.revision();
    m_currentProcessPlan = std::move(resolvedPlan);
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
    return sortEntitiesWithProcessPlan3D
        (QStringLiteral("4轴(绕A)智能排序"),
            cadcam::planning::ProcessSortIntent::RebuildSequence);
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
