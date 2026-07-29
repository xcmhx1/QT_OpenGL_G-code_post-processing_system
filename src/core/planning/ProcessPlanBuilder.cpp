#include "core/planning/ProcessPlanBuilder.h"

#include "core/machining/TubeCutBoundary.h"
#include "core/machining/TubeSectionProjector.h"
#include "core/machine/RotaryKinematics.h"
#include "core/planning/SingleClosedEntryRefiner.h"

#include <QStringList>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>

namespace cadcam::planning
{
    namespace
    {
        using geometry::EntityId;
        using geometry::Vector2d;
        using geometry::Vector3d;
        using machining::TubeCutAnalysis;
        using machining::TubeCutBoundaryClassifier;
        using machining::TubeCutResult;

        constexpr double kCalculationEpsilon = 1.0e-12;

        struct BoundaryData
        {
            int groupId = -1;
            int pairId = -1;
            BoundaryRole role = BoundaryRole::None;
            TubeCutAnalysis analysis;
        };

        struct BoundaryIdentity
        {
            int pairId = -1;
            BoundaryRole role = BoundaryRole::None;
            std::vector<EntityId> entityIds;
            std::size_t stableSourceIndex = 0;
            EntityId stableEntityId = 0;
        };

        struct XBounds
        {
            bool valid = false;
            double minimum = 0.0;
            double maximum = 0.0;
        };

        struct DirectedEntity
        {
            const PlanningEntity* entity = nullptr;
            bool reverseRelativeToInput = false;
            std::optional<double> selectedStartParameter;
            Vector3d start;
            Vector3d end;
            int entryAxisReversalCount = 0;
            double entryTangentCost = 0.0;
        };

        enum class ZoneEntryCandidateKind
        {
            OpenEndpoint,
            ClosedCurveParameter,
            ClosedLoopConnection,
            ClosedLoopArcInterior,
            ClosedLoopEllipseInterior,
            ClosedLoopZoneRunMidpoint,
            BreakZoneMidpoint
        };

        struct ZoneEntryCandidate
        {
            machining::TubeZone16 zone =
                machining::TubeZone16::TopFace;
            ZoneEntryCandidateKind kind =
                ZoneEntryCandidateKind::OpenEndpoint;
            EntityId entityId = 0;
            geometry::SourceGeometryKind sourceKind =
                geometry::SourceGeometryKind::Unknown;
            std::optional<double> sourceParameter;
            bool reverse = false;
            Vector3d entryPosition;
            Vector3d firstCutPoint;
            Vector3d firstCutTangent;
            double entryX = 0.0;
            double confidence = 0.0;
            double distanceToMemberEndpoint = 0.0;
            double distanceToZoneBoundary = 0.0;
            bool ambiguous = false;
        };

        struct TraversalSelectionContext
        {
            std::optional<machining::TubeZone16> requiredEntryZone;
            int longitudinalDirection = 1;
            double zoneHitX = 0.0;
            double frontierX = 0.0;
            double projectionTolerance = 0.0;
            Vector3d previousEnd;
            Vector3d previousCutEnd;
            Vector3d previousTransferAnchor;
            bool hardZoneConstraint = false;
            bool allowZoneRunMidpointFallback = false;
        };

        struct GroupTraversal
        {
            int groupId = -1;
            std::vector<DirectedEntity> entities;
            Vector3d start;
            Vector3d end;
            double movementDistance = 0.0;
            double rotationCost = 0.0;
            int surfaceCost = 0;
            int entryAxisReversalCount = 0;
            double entryTangentCost = 0.0;
            int entryCandidateCount = 0;
            int connectionCandidateCount = 0;
            int arcInteriorCandidateCount = 0;
            int ellipseInteriorCandidateCount = 0;
            int zoneRunMidpointCandidateCount = 0;
            int curveCandidateRejectedCount = 0;
            int wrongZoneRejectedCount = 0;
            std::size_t stableSourceIndex = 0;
            EntityId stableEntityId = 0;
            std::vector<EntityId> arcInteriorCandidateEntityIds;
            std::vector<EntityId> ellipseInteriorCandidateEntityIds;
            std::optional<ZoneEntryCandidate> selectedEntry;
            std::vector<ProcessPathFragment> fragments;
            QString entryRefinementMode;
            Vector3d previousCutEnd;
            Vector3d previousTransferAnchor;
            int curveMemberCount = 0;
            int arcTangentRootCount = 0;
            int ellipseTangentRootCount = 0;
            int validTangentCount = 0;
            double entryTravelDistance = 0.0;
            double approachCutAngle = 0.0;
            double nearestConnectionDistance = 0.0;
            double forwardAngle = 0.0;
            double reverseAngle = 0.0;
            double tangentResidual = 0.0;
            double approachCutDot = 0.0;
        };

        struct ClosedLoopTraversalReport
        {
            int groupId = -1;
            std::vector<EntityId> memberEntityIds;
            int memberCount = 0;
            int nodeCount = 0;
            int connectedComponentCount = 0;
            int branchNodeCount = 0;
            int invalidDegreeNodeCount = 0;
            int candidateCount = 0;
            int wrongZoneRejectedCount = 0;
            std::vector<EntityId> selectedOrder;
            std::vector<bool> selectedReverse;
            bool simpleLoopValid = false;
            QString status = QStringLiteral("Failed");
            QString failureReason;
        };

        struct SectionProjection
        {
            bool valid = false;
            double perimeterPosition = 0.0;
            double distance = std::numeric_limits<double>::max();
        };

        struct SectionBounds
        {
            bool valid = false;
            double minimumY = 0.0;
            double maximumY = 0.0;
            double minimumZ = 0.0;
            double maximumZ = 0.0;
        };

        struct ProcessSurfaceFootprint
        {
            machining::TubeSurfaceRegion dominantRegion =
                machining::TubeSurfaceRegion::Unknown;
            machining::TubeSurfaceRegion entryRegion =
                machining::TubeSurfaceRegion::Unknown;
            machining::TubeSurfaceRegion exitRegion =
                machining::TubeSurfaceRegion::Unknown;
            double minimumX = 0.0;
            double maximumX = 0.0;
            double anchorX = 0.0;
            double minimumPerimeterPosition = 0.0;
            double maximumPerimeterPosition = 0.0;
        };

        struct SurfaceSweepState
        {
            machining::TubeSurfaceRegion currentRegion =
                machining::TubeSurfaceRegion::Unknown;
            int perimeterDirection = 0;
            int longitudinalDirection = 0;
            double currentX = 0.0;
            double currentPerimeterPosition = 0.0;
            bool initialized = false;
        };

        struct SurfaceSweepReport
        {
            int partitionId = -1;
            machining::TubeSurfaceRegion initialRegion =
                machining::TubeSurfaceRegion::Unknown;
            int perimeterDirection = 0;
            int longitudinalDirection = 0;
            int selectedUnitCount = 0;
            int regionTransitionCount = 0;
            int backtrackCount = 0;
            double longitudinalBacktrackDistance = 0.0;
            QStringList selectedUnits;
            bool active = false;
        };

        struct SchedulingCandidate
        {
            GroupTraversal traversal;
            ProcessSurfaceFootprint footprint;
            std::optional<ClosedLoopTraversalReport> closedLoopReport;
            struct BreakBoundaryTraversalReport
            {
                int groupId = -1;
                int boundaryRank = -1;
                int boundaryPairId = -1;
                bool forcedTopMidpoint = false;
                machining::TubeZone16 preferredStartZone =
                    machining::TubeZone16::TopFace;
                int candidateRunCount = 0;
                QString candidateRuns;
                machining::TubeZone16 startZone =
                    machining::TubeZone16::TopFace;
                double selectedRunLength = 0.0;
                double selectedMaximumShellDeviation = 0.0;
                double selectedConfidence = 0.0;
                Vector3d selectedMidpoint;
                EntityId selectedEntityId = 0;
                double selectedSourceParameter = 0.0;
                std::optional<machining::TubeZone16> exitZone;
                double exitConfidence = 0.0;
                double exitReliableLength = 0.0;
                EntityId finalEntityId = 0;
                double finalParameterBegin = 0.0;
                double finalParameterEnd = 0.0;
                bool exitUsedFallback = false;
                int fragmentCount = 0;
                int nextPartitionId = -1;
                bool partitionMappingFound = false;
                bool partitionStartSucceeded = false;
                QString direction;
                QString status = QStringLiteral("Failed");
                QString failureReason;
                DiagnosticCode failureCode =
                    DiagnosticCode::ProcessPlanningBreakFragmentTraversalInvalid;
                std::vector<ProcessPathFragment> fragments;
            };
            std::optional<BreakBoundaryTraversalReport> breakReport;
        };
        using BreakBoundaryTraversalReport =
            SchedulingCandidate::BreakBoundaryTraversalReport;

        struct ProcessGroupZoneProfile
        {
            machining::TubeZoneMask certainMask = 0U;
            machining::TubeZoneMask possibleMask = 0U;
            machining::TubeZoneMask connectionEntryMask = 0U;
            machining::TubeZoneMask curveInteriorEntryMask = 0U;
            machining::TubeZoneMask zoneRunMidpointEntryMask = 0U;
            machining::TubeZoneMask legalEntryMask = 0U;
            machining::TubeZoneMask schedulableMask = 0U;
            std::array<machining::TubeZoneSpan,
                machining::kTubeZone16Count> zoneSpans;
            std::array<std::vector<ZoneEntryCandidate>,
                machining::kTubeZone16Count> entryCandidates;
            std::array<int, machining::kTubeZone16Count>
                entryCandidateCounts{};
            std::array<std::vector<EntityId>,
                machining::kTubeZone16Count> arcMemberIdsByZone;
            std::array<std::vector<EntityId>,
                machining::kTubeZone16Count> ellipseMemberIdsByZone;
            bool closed = false;
            bool uncertain = false;
        };

        struct ZoneSweepSelection
        {
            int groupId = -1;
            machining::TubeZone16 zone = machining::TubeZone16::TopFace;
            machining::TubeZoneSpan span;
            double hitX = 0.0;
            double frontierBefore = 0.0;
            bool fallbackOwner = false;
        };

        struct ZoneSweepOwnership
        {
            int groupId = -1;
            machining::TubeZone16 ownerZone =
                machining::TubeZone16::TopFace;
            bool usedPossibleFallback = false;
            bool usedBoundaryFallback = false;
            machining::TubeZoneMask ownerCandidateMask = 0U;
            machining::TubeZoneMask legalEntryMaskBefore = 0U;
        };

        struct TubeZoneSweepPartition
        {
            int partitionId = -1;
            double minimumX = 0.0;
            double maximumX = 0.0;
            int longitudinalDirection = 1;
            machining::TubeZone16 initialZone =
                machining::TubeZone16::TopFace;
            int perimeterDirection = 1;
            std::array<std::vector<int>,
                machining::kTubeZone16Count> zoneBuckets;
            std::unordered_set<int> groupIds;
            std::unordered_map<int, ZoneSweepOwnership> ownerships;
        };

        struct Zone16SweepState
        {
            int partitionId = -1;
            machining::TubeZone16 initialZone =
                machining::TubeZone16::TopFace;
            int currentZoneOffset = 0;
            int longitudinalDirection = 1;
            double frontierX = 0.0;
            bool zoneEntered = false;
            bool active = false;
            std::array<bool, machining::kTubeZone16Count> enteredZones{};
            std::array<bool, machining::kTubeZone16Count> completedZones{};
        };

        struct Zone16SweepReport
        {
            int partitionId = -1;
            machining::TubeZone16 initialZone =
                machining::TubeZone16::TopFace;
            int perimeterDirection = 1;
            int longitudinalDirection = 1;
            double partitionMinimumX = 0.0;
            double partitionMaximumX = 0.0;
            int processedUnitCount = 0;
            int zoneTransitionCount = 0;
            int backtrackCount = 0;
            QStringList selectedUnits;
            QString status = QStringLiteral("Success");
            bool active = false;
        };

        double distance(const Vector3d& left, const Vector3d& right)
        {
            return std::sqrt
            (
                (left.x - right.x) * (left.x - right.x)
                + (left.y - right.y) * (left.y - right.y)
                + (left.z - right.z) * (left.z - right.z)
            );
        }

        Vector3d entrySubtract(const Vector3d& left, const Vector3d& right)
        {
            return { left.x - right.x, left.y - right.y, left.z - right.z };
        }

        double entryDot(const Vector3d& left, const Vector3d& right)
        {
            return left.x * right.x + left.y * right.y + left.z * right.z;
        }

        Vector3d entryCross(const Vector3d& left, const Vector3d& right)
        {
            return
            {
                left.y * right.z - left.z * right.y,
                left.z * right.x - left.x * right.z,
                left.x * right.y - left.y * right.x
            };
        }

        double entryLength(const Vector3d& value)
        {
            return std::sqrt(entryDot(value, value));
        }

        std::optional<Vector3d> entryNormalized(const Vector3d& value)
        {
            const double length = entryLength(value);
            if (!std::isfinite(length) || length <= kCalculationEpsilon)
                return std::nullopt;
            return Vector3d
            {
                value.x / length,
                value.y / length,
                value.z / length
            };
        }

        constexpr double kEntryTwoPi =
            6.283185307179586476925286766559;

        std::optional<double> parameterInPositiveSweep
        (
            double parameter,
            double start,
            double end,
            double tolerance
        )
        {
            if (!std::isfinite(parameter) || !std::isfinite(start)
                || !std::isfinite(end))
            {
                return std::nullopt;
            }
            while (end <= start) end += kEntryTwoPi;
            while (parameter < start - tolerance) parameter += kEntryTwoPi;
            while (parameter > end + tolerance
                && parameter - kEntryTwoPi >= start - tolerance)
            {
                parameter -= kEntryTwoPi;
            }
            if (parameter < start - tolerance || parameter > end + tolerance)
                return std::nullopt;
            return std::clamp(parameter, start, end);
        }

        struct ExactCurveTangentRoot
        {
            double parameter = 0.0;
            Vector3d point;
            double residual = 0.0;
        };

        std::vector<ExactCurveTangentRoot> arcTangentRoots
        (
            const geometry::ArcGeometry& arc,
            const Vector3d& externalPoint,
            double tolerance
        )
        {
            std::vector<ExactCurveTangentRoot> roots;
            const auto axisU = entryNormalized(arc.axisU);
            const auto axisV = entryNormalized(arc.axisV);
            if (!axisU.has_value() || !axisV.has_value()
                || !std::isfinite(arc.radius) || arc.radius <= tolerance)
            {
                return roots;
            }
            const Vector3d offset = entrySubtract(externalPoint, arc.center);
            const double localX = entryDot(offset, *axisU);
            const double localY = entryDot(offset, *axisV);
            const double radialDistance = std::hypot(localX, localY);
            if (!std::isfinite(radialDistance)
                || radialDistance <= arc.radius + tolerance)
            {
                return roots;
            }
            const double base = std::atan2(localY, localX);
            const double delta = std::acos(std::clamp
                (arc.radius / radialDistance, -1.0, 1.0));
            for (const double rawParameter : { base - delta, base + delta })
            {
                const auto parameter = parameterInPositiveSweep
                    (rawParameter, arc.startParameter, arc.endParameter,
                        tolerance);
                if (!parameter.has_value()) continue;
                const Vector3d radial
                {
                    axisU->x * arc.radius * std::cos(*parameter)
                        + axisV->x * arc.radius * std::sin(*parameter),
                    axisU->y * arc.radius * std::cos(*parameter)
                        + axisV->y * arc.radius * std::sin(*parameter),
                    axisU->z * arc.radius * std::cos(*parameter)
                        + axisV->z * arc.radius * std::sin(*parameter)
                };
                const Vector3d point
                {
                    arc.center.x + radial.x,
                    arc.center.y + radial.y,
                    arc.center.z + radial.z
                };
                const double scale = std::max
                    (1.0, arc.radius * distance(externalPoint, point));
                const double residual =
                    std::abs(entryDot(radial,
                        entrySubtract(externalPoint, point))) / scale;
                roots.push_back({ *parameter, point, residual });
            }
            return roots;
        }

        std::vector<ExactCurveTangentRoot> ellipseTangentRoots
        (
            const geometry::EllipseGeometry& ellipse,
            const Vector3d& externalPoint,
            double tolerance
        )
        {
            std::vector<ExactCurveTangentRoot> roots;
            const Vector3d normal = entryCross
                (ellipse.majorAxis, ellipse.minorAxis);
            const auto unitNormal = entryNormalized(normal);
            if (!unitNormal.has_value()) return roots;
            double start = ellipse.startParameter;
            double end = ellipse.endParameter;
            while (end <= start) end += kEntryTwoPi;
            const auto pointAt = [&ellipse](double parameter)
            {
                return Vector3d
                {
                    ellipse.center.x
                        + ellipse.majorAxis.x * std::cos(parameter)
                        + ellipse.minorAxis.x * std::sin(parameter),
                    ellipse.center.y
                        + ellipse.majorAxis.y * std::cos(parameter)
                        + ellipse.minorAxis.y * std::sin(parameter),
                    ellipse.center.z
                        + ellipse.majorAxis.z * std::cos(parameter)
                        + ellipse.minorAxis.z * std::sin(parameter)
                };
            };
            const auto derivativeAt = [&ellipse](double parameter)
            {
                return Vector3d
                {
                    -ellipse.majorAxis.x * std::sin(parameter)
                        + ellipse.minorAxis.x * std::cos(parameter),
                    -ellipse.majorAxis.y * std::sin(parameter)
                        + ellipse.minorAxis.y * std::cos(parameter),
                    -ellipse.majorAxis.z * std::sin(parameter)
                        + ellipse.minorAxis.z * std::cos(parameter)
                };
            };
            const auto equation = [&](double parameter)
            {
                const Vector3d point = pointAt(parameter);
                return entryDot(entryCross
                    (entrySubtract(externalPoint, point),
                        derivativeAt(parameter)), *unitNormal);
            };
            constexpr int kScanIntervals = 64;
            constexpr int kMaximumIterations = 64;
            const double parameterTolerance = std::max(1.0e-12, tolerance);
            const double residualTolerance = std::max
                (1.0e-10, tolerance * std::max
                    (entryLength(ellipse.majorAxis),
                        entryLength(ellipse.minorAxis)));
            double left = start;
            double leftValue = equation(left);
            for (int interval = 1; interval <= kScanIntervals; ++interval)
            {
                const double right = start + (end - start)
                    * static_cast<double>(interval)
                    / static_cast<double>(kScanIntervals);
                const double rightValue = equation(right);
                std::optional<double> root;
                if (std::abs(leftValue) <= residualTolerance)
                    root = left;
                else if (std::isfinite(leftValue)
                    && std::isfinite(rightValue)
                    && leftValue * rightValue < 0.0)
                {
                    double low = left;
                    double high = right;
                    double lowValue = leftValue;
                    for (int iteration = 0;
                        iteration < kMaximumIterations; ++iteration)
                    {
                        const double middle = (low + high) * 0.5;
                        const double middleValue = equation(middle);
                        if (std::abs(middleValue) <= residualTolerance
                            || high - low <= parameterTolerance)
                        {
                            low = high = middle;
                            break;
                        }
                        if (lowValue * middleValue <= 0.0)
                            high = middle;
                        else
                        {
                            low = middle;
                            lowValue = middleValue;
                        }
                    }
                    root = (low + high) * 0.5;
                }
                if (root.has_value())
                {
                    bool duplicate = false;
                    for (const auto& existing : roots)
                    {
                        duplicate = duplicate
                            || std::abs(existing.parameter - *root)
                                <= parameterTolerance * 8.0;
                    }
                    if (!duplicate)
                    {
                        roots.push_back
                        ({
                            *root,
                            pointAt(*root),
                            std::abs(equation(*root))
                        });
                    }
                }
                left = right;
                leftValue = rightValue;
            }
            if (std::abs(leftValue) <= residualTolerance)
            {
                bool duplicate = false;
                for (const auto& existing : roots)
                    duplicate = duplicate
                        || std::abs(existing.parameter - end)
                            <= parameterTolerance * 8.0;
                if (!duplicate)
                    roots.push_back({ end, pointAt(end), std::abs(leftValue) });
            }
            return roots;
        }

        QString strategyName(ProcessOrderingStrategy strategy)
        {
            return strategy == ProcessOrderingStrategy::LazyRotation
                ? QStringLiteral("LazyRotation")
                : QStringLiteral("NearestNext");
        }

        QString groupKindName(ProcessGroupKind kind)
        {
            switch (kind)
            {
            case ProcessGroupKind::SingleEntity: return QStringLiteral("SingleEntity");
            case ProcessGroupKind::ConnectedChain: return QStringLiteral("ConnectedChain");
            case ProcessGroupKind::ClosedLoop: return QStringLiteral("ClosedLoop");
            case ProcessGroupKind::BreakBoundary: return QStringLiteral("BreakBoundary");
            case ProcessGroupKind::WasteBoundary: return QStringLiteral("WasteBoundary");
            }
            return QStringLiteral("Unknown");
        }

        QVariantMap diagnosticValues
        (
            const ProcessPlanningInput& input,
            const ProcessPlanningPolicy& policy,
            EntityId entityId = 0,
            std::size_t sourceIndex = 0,
            int boundaryPairId = -1,
            int groupId = -1,
            int predecessorGroupId = -1,
            int successorGroupId = -1,
            int candidateCount = 0,
            int eligibleCount = 0,
            int groupCount = 0,
            int assignmentCount = 0,
            int excludedCount = 0,
            int processOrder = -1,
            int continuousGroupId = -1,
            bool initialSelection = false,
            const GroupTraversal* selected = nullptr,
            ProcessGroupKind selectedGroupKind = ProcessGroupKind::SingleEntity,
            int blockedNearestBoundaryGroupId = -1
        )
        {
            QVariantMap values;
            values.insert(QStringLiteral("contentRevision"), QVariant::fromValue<qulonglong>(input.contentRevision));
            values.insert(QStringLiteral("entityId"), QVariant::fromValue<qulonglong>(entityId));
            values.insert(QStringLiteral("sourceIndex"), QVariant::fromValue<qulonglong>(sourceIndex));
            values.insert(QStringLiteral("boundaryPairId"), boundaryPairId);
            values.insert(QStringLiteral("groupId"), groupId);
            values.insert(QStringLiteral("predecessorGroupId"), predecessorGroupId);
            values.insert(QStringLiteral("successorGroupId"), successorGroupId);
            values.insert(QStringLiteral("orderingStrategy"), strategyName(policy.orderingStrategy));
            values.insert(QStringLiteral("candidateCount"), candidateCount);
            values.insert(QStringLiteral("eligibleCount"), eligibleCount);
            values.insert(QStringLiteral("groupCount"), groupCount);
            values.insert(QStringLiteral("assignmentCount"), assignmentCount);
            values.insert(QStringLiteral("excludedCount"), excludedCount);
            values.insert(QStringLiteral("processOrder"), processOrder);
            values.insert(QStringLiteral("continuousGroupId"), continuousGroupId);
            values.insert(QStringLiteral("initialSelection"), initialSelection);
            values.insert(QStringLiteral("initialPositionX"), policy.initialPosition.x);
            values.insert(QStringLiteral("initialPositionY"), policy.initialPosition.y);
            values.insert(QStringLiteral("initialPositionZ"), policy.initialPosition.z);
            values.insert(QStringLiteral("selectedGroupId"), selected != nullptr ? selected->groupId : -1);
            values.insert(QStringLiteral("selectedGroupKind"), selected != nullptr
                ? groupKindName(selectedGroupKind) : QString());
            values.insert(QStringLiteral("selectedMovementDistance"), selected != nullptr
                ? selected->movementDistance : -1.0);
            values.insert(QStringLiteral("selectedRotationCost"), selected != nullptr
                ? selected->rotationCost : -1.0);
            values.insert(QStringLiteral("selectedSurfaceCost"), selected != nullptr
                ? selected->surfaceCost : -1);
            values.insert(QStringLiteral("blockedNearestBoundaryGroupId"), blockedNearestBoundaryGroupId);
            return values;
        }

        Diagnostic planningDiagnostic
        (
            const OperationContext& context,
            DiagnosticCode code,
            const QString& message,
            const QString& detail,
            const QVariantMap& values,
            DiagnosticSeverity severity = DiagnosticSeverity::Error
        )
        {
            Diagnostic diagnostic;
            diagnostic.code = code;
            diagnostic.severity = severity;
            diagnostic.component = QStringLiteral("ProcessPlanBuilder");
            diagnostic.operation = QStringLiteral("BuildProcessPlan");
            diagnostic.stage = QStringLiteral("ProcessPlanning");
            diagnostic.userMessage = message;
            diagnostic.technicalDetail = detail;
            diagnostic.correlationId = context.correlationId;
            diagnostic.context = values;
            const qulonglong entityId = values.value(QStringLiteral("entityId")).toULongLong();
            if (entityId != 0U) diagnostic.entityId = entityId;
            const int groupId = values.value(QStringLiteral("groupId"), -1).toInt();
            if (groupId >= 0) diagnostic.groupId = groupId;
            return diagnostic;
        }

        template<typename T>
        OperationResult<T> failure
        (
            OperationStatus status,
            const OperationContext& context,
            DiagnosticCode code,
            const QString& message,
            const QString& detail,
            const QVariantMap& values
        )
        {
            OperationResult<T> result;
            result.status = status;
            result.addDiagnostic(planningDiagnostic(context, code, message, detail, values));
            return result;
        }

        std::vector<double> cumulativeSectionLengths(const machining::TubeSectionGeometry& section)
        {
            std::vector<double> cumulative(section.boundary.size() + 1U, 0.0);
            for (std::size_t index = 0; index < section.boundary.size(); ++index)
            {
                const Vector2d& first = section.boundary[index];
                const Vector2d& second = section.boundary[(index + 1U) % section.boundary.size()];
                cumulative[index + 1U] = cumulative[index]
                    + std::hypot(second.x - first.x, second.y - first.y);
            }
            return cumulative;
        }

        SectionProjection projectToSection
        (
            const Vector2d& point,
            const machining::TubeSectionGeometry& section,
            const std::vector<double>& cumulative
        )
        {
            SectionProjection best;
            const Vector2d localPoint
                { point.x - section.centerY, point.y - section.centerZ };
            for (std::size_t index = 0; index < section.boundary.size(); ++index)
            {
                const Vector2d start
                    { section.boundary[index].x - section.centerY,
                      section.boundary[index].y - section.centerZ };
                const Vector2d& worldEnd = section.boundary[(index + 1U) % section.boundary.size()];
                const Vector2d end
                    { worldEnd.x - section.centerY, worldEnd.y - section.centerZ };
                const double dy = end.x - start.x;
                const double dz = end.y - start.y;
                const double lengthSquared = dy * dy + dz * dz;
                if (lengthSquared <= kCalculationEpsilon) continue;
                const double factor = std::clamp
                (
                    ((localPoint.x - start.x) * dy + (localPoint.y - start.y) * dz)
                        / lengthSquared,
                    0.0,
                    1.0
                );
                const double projectedY = start.x + dy * factor;
                const double projectedZ = start.y + dz * factor;
                const double candidateDistance = std::hypot
                    (localPoint.x - projectedY, localPoint.y - projectedZ);
                if (candidateDistance < best.distance)
                {
                    best.valid = true;
                    best.distance = candidateDistance;
                    best.perimeterPosition = cumulative[index] + std::sqrt(lengthSquared) * factor;
                }
            }
            return best;
        }

        QString surfaceRegionName(machining::TubeSurfaceRegion region)
        {
            using Region = machining::TubeSurfaceRegion;
            switch (region)
            {
            case Region::Top: return QStringLiteral("Top");
            case Region::TopRightCorner: return QStringLiteral("TopRightCorner");
            case Region::Right: return QStringLiteral("Right");
            case Region::BottomRightCorner: return QStringLiteral("BottomRightCorner");
            case Region::Bottom: return QStringLiteral("Bottom");
            case Region::BottomLeftCorner: return QStringLiteral("BottomLeftCorner");
            case Region::Left: return QStringLiteral("Left");
            case Region::TopLeftCorner: return QStringLiteral("TopLeftCorner");
            case Region::Mixed: return QStringLiteral("Mixed");
            case Region::Unknown: return QStringLiteral("Unknown");
            }
            return QStringLiteral("Unknown");
        }

        int surfaceRegionIndex(machining::TubeSurfaceRegion region)
        {
            using Region = machining::TubeSurfaceRegion;
            switch (region)
            {
            case Region::Top: return 0;
            case Region::TopRightCorner: return 1;
            case Region::Right: return 2;
            case Region::BottomRightCorner: return 3;
            case Region::Bottom: return 4;
            case Region::BottomLeftCorner: return 5;
            case Region::Left: return 6;
            case Region::TopLeftCorner: return 7;
            case Region::Mixed:
            case Region::Unknown: return -1;
            }
            return -1;
        }

        machining::TubeSurfaceRegion surfaceRegionAt(int index)
        {
            using Region = machining::TubeSurfaceRegion;
            static constexpr std::array<Region, 8> regions
            {{
                Region::Top,
                Region::TopRightCorner,
                Region::Right,
                Region::BottomRightCorner,
                Region::Bottom,
                Region::BottomLeftCorner,
                Region::Left,
                Region::TopLeftCorner
            }};
            const int wrapped = ((index % 8) + 8) % 8;
            return regions[static_cast<std::size_t>(wrapped)];
        }

        SectionBounds sectionBounds(const machining::TubeSectionGeometry& section)
        {
            SectionBounds bounds;
            if (section.boundary.empty()) return bounds;
            bounds.valid = true;
            bounds.minimumY = bounds.maximumY = section.boundary.front().x;
            bounds.minimumZ = bounds.maximumZ = section.boundary.front().y;
            for (const Vector2d& point : section.boundary)
            {
                if (!std::isfinite(point.x) || !std::isfinite(point.y))
                    return SectionBounds{};
                bounds.minimumY = std::min(bounds.minimumY, point.x);
                bounds.maximumY = std::max(bounds.maximumY, point.x);
                bounds.minimumZ = std::min(bounds.minimumZ, point.y);
                bounds.maximumZ = std::max(bounds.maximumZ, point.y);
            }
            return bounds;
        }

        double surfaceClassificationTolerance
        (
            const machining::TubeSectionGeometry& section
        )
        {
            const double maximumDimension = std::max
                (std::abs(section.yLength), std::abs(section.zWidth));
            return std::max({ kCalculationEpsilon, 1.0e-6,
                maximumDimension * 1.0e-8 });
        }

        machining::TubeSurfaceRegion cornerRegion
        (
            const machining::TubeCornerGeometry& corner
        )
        {
            using Region = machining::TubeSurfaceRegion;
            if (corner.yDirection > 0 && corner.zDirection > 0)
                return Region::TopRightCorner;
            if (corner.yDirection > 0 && corner.zDirection < 0)
                return Region::BottomRightCorner;
            if (corner.yDirection < 0 && corner.zDirection < 0)
                return Region::BottomLeftCorner;
            if (corner.yDirection < 0 && corner.zDirection > 0)
                return Region::TopLeftCorner;
            return Region::Unknown;
        }

        const machining::TubeCornerGeometry* matchingTubeCorner
        (
            const Vector3d& point,
            const machining::TubeSectionModel& section
        )
        {
            for (const auto& corner : section.corners)
            {
                if (!std::isfinite(corner.radius) || corner.radius <= 0.0) continue;
                const double tolerance = std::max(0.01, corner.radius * 0.01);
                const double dy = point.y - corner.center.x;
                const double dz = point.z - corner.center.y;
                if (dy * corner.yDirection < -tolerance
                    || dz * corner.zDirection < -tolerance) continue;
                if (std::abs(std::hypot(dy, dz) - corner.radius) <= tolerance)
                    return &corner;
            }
            return nullptr;
        }

        std::optional<machining::TubeSurfaceRegion> flatRegionForPoints
        (
            const std::vector<Vector3d>& points,
            const SectionBounds& bounds,
            double tolerance
        )
        {
            using Region = machining::TubeSurfaceRegion;
            if (points.empty() || !bounds.valid) return std::nullopt;
            struct Candidate
            {
                Region region = Region::Unknown;
                double maximumDistance = 0.0;
                double averageDistance = 0.0;
            };
            const std::array<std::pair<Region, double>, 4> planes
            {{
                { Region::Top, bounds.maximumZ },
                { Region::Right, bounds.maximumY },
                { Region::Bottom, bounds.minimumZ },
                { Region::Left, bounds.minimumY }
            }};
            std::vector<Candidate> candidates;
            for (const auto& [region, coordinate] : planes)
            {
                double maximumDistance = 0.0;
                double totalDistance = 0.0;
                for (const Vector3d& point : points)
                {
                    const double value = region == Region::Top || region == Region::Bottom
                        ? point.z : point.y;
                    const double candidateDistance = std::abs(value - coordinate);
                    maximumDistance = std::max(maximumDistance, candidateDistance);
                    totalDistance += candidateDistance;
                }
                if (maximumDistance <= tolerance)
                {
                    candidates.push_back
                    ({ region, maximumDistance,
                        totalDistance / static_cast<double>(points.size()) });
                }
            }
            if (candidates.empty()) return std::nullopt;
            std::sort(candidates.begin(), candidates.end(), [](const Candidate& left,
                const Candidate& right)
            {
                if (std::abs(left.maximumDistance - right.maximumDistance)
                    > kCalculationEpsilon)
                    return left.maximumDistance < right.maximumDistance;
                if (std::abs(left.averageDistance - right.averageDistance)
                    > kCalculationEpsilon)
                    return left.averageDistance < right.averageDistance;
                return surfaceRegionIndex(left.region) < surfaceRegionIndex(right.region);
            });
            if (candidates.size() == 1U) return candidates.front().region;
            if (candidates[0].maximumDistance
                    < candidates[1].maximumDistance - kCalculationEpsilon
                || (std::abs(candidates[0].maximumDistance
                        - candidates[1].maximumDistance) <= kCalculationEpsilon
                    && candidates[0].averageDistance
                        < candidates[1].averageDistance - kCalculationEpsilon))
                return candidates.front().region;
            return std::nullopt;
        }

        std::optional<machining::TubeSurfaceRegion> cornerRegionForPoints
        (
            const std::vector<Vector3d>& points,
            const machining::TubeSectionModel& section
        )
        {
            if (points.empty()) return std::nullopt;
            const machining::TubeCornerGeometry* selected = nullptr;
            for (const Vector3d& point : points)
            {
                const auto* corner = matchingTubeCorner(point, section);
                if (corner == nullptr || (selected != nullptr && selected != corner))
                    return std::nullopt;
                selected = corner;
            }
            if (selected == nullptr) return std::nullopt;
            const auto region = cornerRegion(*selected);
            return surfaceRegionIndex(region) >= 0
                ? std::optional<machining::TubeSurfaceRegion>(region) : std::nullopt;
        }

        machining::TubeSurfaceRegion classifySurfacePoints
        (
            const std::vector<Vector3d>& points,
            const machining::TubeSectionModel& section,
            const SectionBounds& bounds,
            double tolerance
        )
        {
            if (const auto flat = flatRegionForPoints(points, bounds, tolerance))
                return *flat;
            if (const auto corner = cornerRegionForPoints(points, section))
                return *corner;
            return points.empty() ? machining::TubeSurfaceRegion::Unknown
                : machining::TubeSurfaceRegion::Mixed;
        }

        machining::TubeSurfaceRegion classifySurfacePoint
        (
            const Vector3d& point,
            const machining::TubeSectionModel& section,
            const SectionBounds& bounds,
            double tolerance
        )
        {
            if (const auto* corner = matchingTubeCorner(point, section))
            {
                const auto region = cornerRegion(*corner);
                if (surfaceRegionIndex(region) >= 0) return region;
            }
            const std::vector<Vector3d> singlePoint{ point };
            if (const auto flat = flatRegionForPoints(singlePoint, bounds, tolerance))
                return *flat;
            return machining::TubeSurfaceRegion::Unknown;
        }

        double wrappedPerimeterDelta(double from, double to, double perimeter)
        {
            if (!std::isfinite(from) || !std::isfinite(to)
                || !std::isfinite(perimeter) || perimeter <= 0.0) return 0.0;
            return std::remainder(to - from, perimeter);
        }

        BoundarySide classifyPoint
        (
            const Vector3d& point,
            const TubeCutAnalysis& boundary,
            const machining::TubeSectionGeometry& section,
            double tolerance
        )
        {
            if (boundary.unwrappedBoundary.size() < 2U || section.boundary.size() < 3U || section.perimeter <= 0.0)
            {
                return BoundarySide::Indeterminate;
            }

            const std::vector<double> cumulative = cumulativeSectionLengths(section);
            const SectionProjection projection = projectToSection({ point.y, point.z }, section, cumulative);
            const double safeTolerance = std::max(1.0e-6, std::abs(tolerance));
            if (!projection.valid || projection.distance > safeTolerance)
            {
                return BoundarySide::Indeterminate;
            }

            const double perimeter = section.perimeter;
            double minimumBoundaryX = boundary.unwrappedBoundary.front().x;
            double maximumBoundaryX = minimumBoundaryX;
            for (const machining::UnwrappedBoundaryPoint& sample : boundary.unwrappedBoundary)
            {
                minimumBoundaryX = std::min(minimumBoundaryX, sample.x);
                maximumBoundaryX = std::max(maximumBoundaryX, sample.x);
            }
            const double referenceX = minimumBoundaryX
                + (maximumBoundaryX - minimumBoundaryX) * 0.5;
            const double localPointX = point.x - referenceX;
            double queryPosition = projection.perimeterPosition;
            double minimumPosition = boundary.unwrappedBoundary.front().perimeterPosition;
            double maximumPosition = minimumPosition;
            for (const machining::UnwrappedBoundaryPoint& sample : boundary.unwrappedBoundary)
            {
                minimumPosition = std::min(minimumPosition, sample.perimeterPosition);
                maximumPosition = std::max(maximumPosition, sample.perimeterPosition);
            }
            const double intervalCenter = (minimumPosition + maximumPosition) * 0.5;
            while (queryPosition - intervalCenter > perimeter * 0.5) queryPosition -= perimeter;
            while (queryPosition - intervalCenter < -perimeter * 0.5) queryPosition += perimeter;

            std::vector<double> intersections;
            bool onBoundary = false;
            for (const double shift : { -perimeter, 0.0, perimeter })
            {
                for (std::size_t index = 0; index + 1U < boundary.unwrappedBoundary.size(); ++index)
                {
                    const auto& first = boundary.unwrappedBoundary[index];
                    const auto& second = boundary.unwrappedBoundary[index + 1U];
                    const double firstX = first.x - referenceX;
                    const double secondX = second.x - referenceX;
                    const double firstS = first.perimeterPosition + shift;
                    const double secondS = second.perimeterPosition + shift;
                    const double edgeX = secondX - firstX;
                    const double edgeS = secondS - firstS;
                    const double lengthSquared = edgeX * edgeX + edgeS * edgeS;
                    if (lengthSquared > kCalculationEpsilon)
                    {
                        const double factor = std::clamp
                        (
                            ((localPointX - firstX) * edgeX
                                + (queryPosition - firstS) * edgeS) / lengthSquared,
                            0.0,
                            1.0
                        );
                        const double nearestX = firstX + edgeX * factor;
                        const double nearestS = firstS + edgeS * factor;
                        onBoundary = onBoundary
                            || std::hypot(localPointX - nearestX,
                                queryPosition - nearestS) <= safeTolerance;
                    }
                    const bool crosses = (firstS <= queryPosition && queryPosition < secondS)
                        || (secondS <= queryPosition && queryPosition < firstS);
                    if (!crosses || std::abs(edgeS) <= kCalculationEpsilon) continue;
                    const double factor = (queryPosition - firstS) / edgeS;
                    const double intersectionX = firstX + edgeX * factor;
                    intersections.push_back(intersectionX);
                    onBoundary = onBoundary
                        || std::abs(intersectionX - localPointX) <= safeTolerance;
                }
            }
            if (onBoundary) return BoundarySide::OnBoundary;

            std::sort(intersections.begin(), intersections.end());
            std::vector<double> unique;
            for (const double x : intersections)
            {
                if (!unique.empty() && std::abs(x - unique.back()) <= safeTolerance)
                {
                    unique.back() = (unique.back() + x) * 0.5;
                }
                else
                {
                    unique.push_back(x);
                }
            }
            const int crossings = static_cast<int>(std::count_if
            (
                unique.cbegin(), unique.cend(),
                [localPointX, safeTolerance](double x)
                { return x < localPointX - safeTolerance; }
            ));
            return crossings % 2 == 0 ? BoundarySide::Left : BoundarySide::Right;
        }

        BoundarySide classifyGroup
        (
            const ProcessGroup& group,
            const std::unordered_map<EntityId, const PlanningEntity*>& entities,
            const BoundaryData& boundary,
            const machining::TubeSectionGeometry& section,
            double tolerance
        )
        {
            double groupMinimumX = std::numeric_limits<double>::max();
            double groupMaximumX = std::numeric_limits<double>::lowest();
            for (const EntityId entityId : group.entityIds)
            {
                const auto found = entities.find(entityId);
                if (found == entities.end() || found->second->path.vertices.empty())
                {
                    return BoundarySide::Indeterminate;
                }
                for (const geometry::PathVertex3D& vertex : found->second->path.vertices)
                {
                    groupMinimumX = std::min(groupMinimumX, vertex.position.x);
                    groupMaximumX = std::max(groupMaximumX, vertex.position.x);
                }
            }

            double boundaryMinimumX = std::numeric_limits<double>::max();
            double boundaryMaximumX = std::numeric_limits<double>::lowest();
            for (const machining::UnwrappedBoundaryPoint& point : boundary.analysis.unwrappedBoundary)
            {
                boundaryMinimumX = std::min(boundaryMinimumX, point.x);
                boundaryMaximumX = std::max(boundaryMaximumX, point.x);
            }
            const double safeTolerance = std::max(1.0e-6, std::abs(tolerance));
            if (groupMaximumX < boundaryMinimumX - safeTolerance)
            {
                return BoundarySide::Left;
            }
            if (groupMinimumX > boundaryMaximumX + safeTolerance)
            {
                return BoundarySide::Right;
            }

            bool left = false;
            bool right = false;
            bool onBoundary = false;
            for (const EntityId entityId : group.entityIds)
            {
                const auto found = entities.find(entityId);
                if (found == entities.end()) return BoundarySide::Indeterminate;
                const auto& vertices = found->second->path.vertices;
                if (vertices.empty()) return BoundarySide::Indeterminate;
                for (std::size_t index = 0; index < vertices.size(); ++index)
                {
                    const BoundarySide side = classifyPoint(vertices[index].position, boundary.analysis, section, tolerance);
                    if (side == BoundarySide::Indeterminate) return side;
                    left = left || side == BoundarySide::Left;
                    right = right || side == BoundarySide::Right;
                    onBoundary = onBoundary || side == BoundarySide::OnBoundary;
                    if (index + 1U < vertices.size())
                    {
                        const Vector3d& first = vertices[index].position;
                        const Vector3d& second = vertices[index + 1U].position;
                        const Vector3d midpoint
                        {
                            first.x + (second.x - first.x) * 0.5,
                            first.y + (second.y - first.y) * 0.5,
                            first.z + (second.z - first.z) * 0.5
                        };
                        const BoundarySide middleSide = classifyPoint(midpoint, boundary.analysis, section, tolerance);
                        if (middleSide == BoundarySide::Indeterminate) return middleSide;
                        left = left || middleSide == BoundarySide::Left;
                        right = right || middleSide == BoundarySide::Right;
                        onBoundary = onBoundary || middleSide == BoundarySide::OnBoundary;
                    }
                }
            }
            if (left && right) return BoundarySide::Mixed;
            if (left) return BoundarySide::Left;
            if (right) return BoundarySide::Right;
            return onBoundary ? BoundarySide::OnBoundary : BoundarySide::Indeterminate;
        }

        QString sideName(BoundarySide side)
        {
            switch (side)
            {
            case BoundarySide::Left: return QStringLiteral("Left");
            case BoundarySide::OnBoundary: return QStringLiteral("OnBoundary");
            case BoundarySide::Right: return QStringLiteral("Right");
            case BoundarySide::Mixed: return QStringLiteral("Mixed");
            case BoundarySide::Indeterminate: return QStringLiteral("Indeterminate");
            }
            return QStringLiteral("Indeterminate");
        }

        XBounds groupXBounds
        (
            const ProcessGroup& group,
            const std::unordered_map<EntityId, const PlanningEntity*>& entities
        )
        {
            XBounds bounds;
            for (const EntityId entityId : group.entityIds)
            {
                const auto found = entities.find(entityId);
                if (found == entities.end()) return {};
                for (const geometry::PathVertex3D& vertex : found->second->path.vertices)
                {
                    if (!bounds.valid)
                    {
                        bounds.valid = true;
                        bounds.minimum = bounds.maximum = vertex.position.x;
                    }
                    else
                    {
                        bounds.minimum = std::min(bounds.minimum, vertex.position.x);
                        bounds.maximum = std::max(bounds.maximum, vertex.position.x);
                    }
                }
            }
            return bounds;
        }

        XBounds boundaryXBounds(const BoundaryData& boundary)
        {
            XBounds bounds;
            for (const machining::UnwrappedBoundaryPoint& point : boundary.analysis.unwrappedBoundary)
            {
                if (!bounds.valid)
                {
                    bounds.valid = true;
                    bounds.minimum = bounds.maximum = point.x;
                }
                else
                {
                    bounds.minimum = std::min(bounds.minimum, point.x);
                    bounds.maximum = std::max(bounds.maximum, point.x);
                }
            }
            return bounds;
        }

        QVariantMap boundaryDiagnosticValues
        (
            const ProcessPlanningInput& input,
            const ProcessPlanningPolicy& policy,
            const BoundaryData& boundary,
            const ProcessGroup& otherGroup,
            const std::unordered_map<EntityId, const PlanningEntity*>& entities,
            int boundarySpatialRank,
            BoundarySide relativeSide,
            BoundarySide reverseRelativeSide = BoundarySide::Indeterminate
        )
        {
            QVariantMap values = diagnosticValues
                (input, policy, 0U, 0U, boundary.pairId, otherGroup.groupId);
            const XBounds boundaryBounds = boundaryXBounds(boundary);
            const XBounds otherBounds = groupXBounds(otherGroup, entities);
            values.insert(QStringLiteral("boundaryGroupId"), boundary.groupId);
            values.insert(QStringLiteral("otherGroupId"), otherGroup.groupId);
            values.insert(QStringLiteral("boundarySpatialRank"), boundarySpatialRank);
            values.insert(QStringLiteral("relativeSide"), sideName(relativeSide));
            values.insert(QStringLiteral("reverseRelativeSide"), sideName(reverseRelativeSide));
            values.insert(QStringLiteral("boundaryMinimumX"),
                boundaryBounds.valid ? boundaryBounds.minimum : 0.0);
            values.insert(QStringLiteral("boundaryMaximumX"),
                boundaryBounds.valid ? boundaryBounds.maximum : 0.0);
            values.insert(QStringLiteral("otherMinimumX"),
                otherBounds.valid ? otherBounds.minimum : 0.0);
            values.insert(QStringLiteral("otherMaximumX"),
                otherBounds.valid ? otherBounds.maximum : 0.0);
            return values;
        }

        std::vector<Vector3d> directedPoints(const PlanningEntity& entity, bool reverse)
        {
            std::vector<Vector3d> points;
            points.reserve(entity.path.vertices.size());
            for (const geometry::PathVertex3D& vertex : entity.path.vertices) points.push_back(vertex.position);
            if (!reverse || points.size() < 2U) return points;
            if (entity.path.closed)
            {
                std::reverse(points.begin() + 1, points.end());
            }
            else
            {
                std::reverse(points.begin(), points.end());
            }
            return points;
        }

        std::vector<Vector3d> directedTraversalPoints(const DirectedEntity& directed)
        {
            if (directed.entity == nullptr) return {};
            const PlanningEntity& entity = *directed.entity;
            if (!entity.path.closed || entity.path.vertices.size() < 2U)
                return directedPoints(entity, directed.reverseRelativeToInput);

            std::size_t startIndex = 0U;
            if (directed.selectedStartParameter.has_value()
                && std::isfinite(*directed.selectedStartParameter))
            {
                double bestDifference = std::numeric_limits<double>::max();
                for (std::size_t index = 0U; index < entity.path.vertices.size(); ++index)
                {
                    const double difference = std::abs
                        (entity.path.vertices[index].sourceParameter
                            - *directed.selectedStartParameter);
                    if (difference < bestDifference)
                    {
                        bestDifference = difference;
                        startIndex = index;
                    }
                }
            }

            const std::size_t pointCount = entity.path.vertices.size();
            std::vector<Vector3d> points;
            points.reserve(pointCount + 1U);
            for (std::size_t offset = 0U; offset < pointCount; ++offset)
            {
                const std::size_t index = directed.reverseRelativeToInput
                    ? (startIndex + pointCount - offset) % pointCount
                    : (startIndex + offset) % pointCount;
                points.push_back(entity.path.vertices[index].position);
            }
            points.push_back(points.front());
            return points;
        }

        machining::TubeSurfaceRegion directedEndpointRegion
        (
            const DirectedEntity& directed,
            bool entry,
            const machining::TubeSectionModel& section,
            const SectionBounds& bounds,
            double tolerance
        )
        {
            const std::vector<Vector3d> points = directedPoints
                (*directed.entity, directed.reverseRelativeToInput);
            if (points.size() >= 2U)
            {
                const Vector3d& endpoint = entry ? points.front() : points.back();
                for (std::size_t offset = 1U; offset < points.size(); ++offset)
                {
                    const Vector3d& neighbor = entry
                        ? points[offset] : points[points.size() - 1U - offset];
                    if (distance(endpoint, neighbor) <= kCalculationEpsilon) continue;
                    const Vector3d middle
                    {
                        (endpoint.x + neighbor.x) * 0.5,
                        (endpoint.y + neighbor.y) * 0.5,
                        (endpoint.z + neighbor.z) * 0.5
                    };
                    const auto region = classifySurfacePoints
                        ({ endpoint, middle, neighbor }, section, bounds, tolerance);
                    if (region != machining::TubeSurfaceRegion::Unknown) return region;
                    break;
                }
            }
            return classifySurfacePoint
                (entry ? directed.start : directed.end, section, bounds, tolerance);
        }

        ProcessSurfaceFootprint buildSurfaceFootprint
        (
            const ProcessGroup& group,
            const std::unordered_map<EntityId, const PlanningEntity*>& entities,
            const machining::TubeSectionModel& section,
            const SectionBounds& bounds,
            const std::vector<double>& cumulative,
            double tolerance
        )
        {
            ProcessSurfaceFootprint footprint;
            std::vector<Vector3d> points;
            std::vector<double> xCoordinates;
            std::vector<double> perimeterPositions;
            const PlanningEntity* firstEntity = nullptr;
            for (const EntityId entityId : group.entityIds)
            {
                const auto found = entities.find(entityId);
                if (found == entities.end() || found->second == nullptr) continue;
                const PlanningEntity& entity = *found->second;
                if (firstEntity == nullptr
                    || entity.sourceIndex < firstEntity->sourceIndex
                    || (entity.sourceIndex == firstEntity->sourceIndex
                        && entity.entityId < firstEntity->entityId))
                    firstEntity = &entity;
                for (const auto& vertex : entity.path.vertices)
                {
                    const Vector3d& point = vertex.position;
                    if (!std::isfinite(point.x) || !std::isfinite(point.y)
                        || !std::isfinite(point.z)) continue;
                    points.push_back(point);
                    xCoordinates.push_back(point.x);
                    const SectionProjection projection = projectToSection
                        ({ point.y, point.z }, section.geometry, cumulative);
                    if (projection.valid) perimeterPositions.push_back
                        (projection.perimeterPosition);
                }
            }
            if (points.empty()) return footprint;

            footprint.dominantRegion = group.kind == ProcessGroupKind::BreakBoundary
                    || group.kind == ProcessGroupKind::WasteBoundary
                ? machining::TubeSurfaceRegion::Mixed
                : classifySurfacePoints(points, section, bounds, tolerance);
            std::sort(xCoordinates.begin(), xCoordinates.end());
            footprint.minimumX = xCoordinates.front();
            footprint.maximumX = xCoordinates.back();
            const std::size_t middle = xCoordinates.size() / 2U;
            footprint.anchorX = xCoordinates.size() % 2U == 0U
                ? (xCoordinates[middle - 1U] + xCoordinates[middle]) * 0.5
                : xCoordinates[middle];
            if (!perimeterPositions.empty())
            {
                const auto extrema = std::minmax_element
                    (perimeterPositions.begin(), perimeterPositions.end());
                footprint.minimumPerimeterPosition = *extrema.first;
                footprint.maximumPerimeterPosition = *extrema.second;
            }
            if (surfaceRegionIndex(footprint.dominantRegion) >= 0)
            {
                footprint.entryRegion = footprint.dominantRegion;
                footprint.exitRegion = footprint.dominantRegion;
            }
            else if (firstEntity != nullptr && !firstEntity->path.vertices.empty())
            {
                const Vector3d& first = firstEntity->path.vertices.front().position;
                const Vector3d& last = firstEntity->path.vertices.back().position;
                footprint.entryRegion = classifySurfacePoint
                    (first, section, bounds, tolerance);
                footprint.exitRegion = classifySurfacePoint
                    (last, section, bounds, tolerance);
            }
            return footprint;
        }

        ProcessSurfaceFootprint footprintForTraversal
        (
            ProcessSurfaceFootprint footprint,
            const GroupTraversal& traversal,
            const machining::TubeSectionModel& section,
            const SectionBounds& bounds,
            double tolerance
        )
        {
            if (surfaceRegionIndex(footprint.dominantRegion) >= 0)
            {
                footprint.entryRegion = footprint.dominantRegion;
                footprint.exitRegion = footprint.dominantRegion;
                return footprint;
            }
            if (!traversal.entities.empty())
            {
                footprint.entryRegion = directedEndpointRegion
                    (traversal.entities.front(), true, section, bounds, tolerance);
                footprint.exitRegion = directedEndpointRegion
                    (traversal.entities.back(), false, section, bounds, tolerance);
            }
            return footprint;
        }

        int traversalPerimeterDirection
        (
            const GroupTraversal& traversal,
            const machining::TubeSectionModel& section,
            const std::vector<double>& cumulative,
            double tolerance
        )
        {
            if (traversal.entities.empty() || section.geometry.perimeter <= 0.0) return 0;
            const DirectedEntity& last = traversal.entities.back();
            const std::vector<Vector3d> points = directedTraversalPoints(last);
            if (points.size() < 2U) return 0;
            const Vector3d& endpoint = points.back();
            for (std::size_t offset = 1U; offset < points.size(); ++offset)
            {
                const Vector3d& previous = points[points.size() - 1U - offset];
                if (distance(previous, endpoint) <= kCalculationEpsilon) continue;
                const SectionProjection from = projectToSection
                    ({ previous.y, previous.z }, section.geometry, cumulative);
                const SectionProjection to = projectToSection
                    ({ endpoint.y, endpoint.z }, section.geometry, cumulative);
                if (!from.valid || !to.valid) return 0;
                const double delta = wrappedPerimeterDelta
                    (from.perimeterPosition, to.perimeterPosition,
                        section.geometry.perimeter);
                if (delta > tolerance) return 1;
                if (delta < -tolerance) return -1;
                return 0;
            }
            return 0;
        }

        bool directionAllowed(const PlanningEntity& entity, bool reverse, bool allowReverse)
        {
            switch (entity.directionPreference)
            {
            case process::DirectionPreference::Forward: return !reverse;
            case process::DirectionPreference::Reverse: return reverse;
            case process::DirectionPreference::Auto: return !reverse || allowReverse;
            }
            return false;
        }

        bool manualDirectionAllowed
        (
            const PlanningEntity& entity,
            bool reverse,
            const TraversalSelectionContext* selection
        )
        {
            if (selection == nullptr || !selection->hardZoneConstraint)
                return true;
            switch (entity.manualDirectionPreference)
            {
            case process::DirectionPreference::Forward: return !reverse;
            case process::DirectionPreference::Reverse: return reverse;
            case process::DirectionPreference::Auto: return true;
            }
            return false;
        }

        bool hasManualEntryConstraint
        (
            const ProcessGroup& group,
            const std::unordered_map<EntityId, const PlanningEntity*>& entities
        )
        {
            return std::any_of
            (
                group.entityIds.cbegin(),
                group.entityIds.cend(),
                [&entities](EntityId entityId)
                {
                    const auto found = entities.find(entityId);
                    return found != entities.end()
                        && found->second != nullptr
                        && (found->second->manualStartParameter.has_value()
                            || found->second->manualDirectionPreference
                                != process::DirectionPreference::Auto);
                }
            );
        }

        std::optional<GroupTraversal> buildTraversal
        (
            const ProcessGroup& group,
            const std::unordered_map<EntityId, const PlanningEntity*>& entities,
            const Vector3d& currentPosition,
            bool allowReverse,
            double tolerance,
            std::optional<std::pair<EntityId, bool>> forcedStart = std::nullopt
        )
        {
            GroupTraversal traversal;
            traversal.groupId = group.groupId;
            std::unordered_set<EntityId> used;
            Vector3d cursor = currentPosition;

            while (used.size() < group.entityIds.size())
            {
                const PlanningEntity* selected = nullptr;
                bool selectedReverse = false;
                std::vector<Vector3d> selectedPoints;
                double bestDistance = std::numeric_limits<double>::max();

                for (const EntityId entityId : group.entityIds)
                {
                    if (used.find(entityId) != used.end()) continue;
                    const auto found = entities.find(entityId);
                    if (found == entities.end()) return std::nullopt;
                    const PlanningEntity& entity = *found->second;
                    for (const bool reverse : { false, true })
                    {
                        if (!directionAllowed(entity, reverse, allowReverse)) continue;
                        if (used.empty() && forcedStart.has_value()
                            && (forcedStart->first != entityId || forcedStart->second != reverse)) continue;
                        const std::vector<Vector3d> points = directedPoints(entity, reverse);
                        if (points.size() < 2U) continue;
                        const double candidateDistance = distance(cursor, points.front());
                        if (!used.empty() && candidateDistance > tolerance) continue;
                        const bool replace = selected == nullptr
                            || candidateDistance < bestDistance - kCalculationEpsilon
                            || (std::abs(candidateDistance - bestDistance) <= kCalculationEpsilon
                                && (entity.sourceIndex < selected->sourceIndex
                                    || (entity.sourceIndex == selected->sourceIndex && entity.entityId < selected->entityId)));
                        if (replace)
                        {
                            selected = &entity;
                            selectedReverse = reverse;
                            selectedPoints = points;
                            bestDistance = candidateDistance;
                        }
                    }
                }

                if (selected == nullptr) return std::nullopt;
                DirectedEntity directed;
                directed.entity = selected;
                directed.reverseRelativeToInput = selectedReverse;
                directed.selectedStartParameter = selected->startParameter;
                directed.start = selectedPoints.front();
                directed.end = selectedPoints.back();
                traversal.entities.push_back(directed);
                used.insert(selected->entityId);
                cursor = directed.end;
            }

            if (traversal.entities.empty()) return std::nullopt;
            traversal.start = traversal.entities.front().start;
            traversal.end = traversal.entities.back().end;
            if (group.closed && distance(traversal.end, traversal.start) > tolerance) return std::nullopt;
            if (group.closed) traversal.end = traversal.start;
            traversal.movementDistance = distance(currentPosition, traversal.start);
            traversal.entryAxisReversalCount =
                traversal.entities.front().entryAxisReversalCount;
            traversal.entryTangentCost = traversal.entities.front().entryTangentCost;
            traversal.stableSourceIndex = traversal.entities.front().entity->sourceIndex;
            traversal.stableEntityId = traversal.entities.front().entity->entityId;
            return traversal;
        }

        double angleDegrees(const Vector3d& point, const machining::TubeSectionModel& section)
        {
            return std::atan2
            (
                point.z - section.geometry.centerZ,
                point.y - section.geometry.centerY
            ) * 180.0 / 3.14159265358979323846;
        }

        int surfaceIndex(double degrees)
        {
            double normalized = std::fmod(degrees + 360.0, 360.0);
            return static_cast<int>(std::floor((normalized + 45.0) / 90.0)) % 4;
        }

        void scoreTraversal
        (
            GroupTraversal& traversal,
            const Vector3d& currentPosition,
            const std::optional<machining::TubeSectionModel>& section
        )
        {
            traversal.movementDistance = distance(currentPosition, traversal.start);
            if (!section.has_value()) return;
            const double currentAngle = angleDegrees(currentPosition, *section);
            const double startAngle = angleDegrees(traversal.start, *section);
            traversal.rotationCost = std::abs(std::remainder(startAngle - currentAngle, 360.0));
            const int currentSurface = surfaceIndex(currentAngle);
            const int targetSurface = surfaceIndex(startAngle);
            const int rawDifference = std::abs(currentSurface - targetSurface);
            traversal.surfaceCost = std::min(rawDifference, 4 - rawDifference);
        }

        double entryThreshold(double connectionTolerance)
        {
            return std::max(kCalculationEpsilon, connectionTolerance * 1.0e-6);
        }

        std::optional<double> rotaryTravelLength
        (
            const Vector3d& start,
            const Vector3d& end,
            const Vector2d& center,
            double threshold
        )
        {
            const double startRadius = std::hypot(start.y - center.x, start.z - center.y);
            const double endRadius = std::hypot(end.y - center.x, end.z - center.y);
            const double localRadius = (startRadius + endRadius) * 0.5;
            if (!std::isfinite(localRadius) || localRadius <= threshold) return std::nullopt;

            const double startAngle = std::atan2(start.z - center.y, start.y - center.x);
            const double endAngle = std::atan2(end.z - center.y, end.y - center.x);
            const double angleDelta = std::remainder
                (endAngle - startAngle, 2.0 * 3.14159265358979323846);
            const double travel = localRadius * angleDelta;
            return std::isfinite(travel) ? std::optional<double>(travel) : std::nullopt;
        }

        void scoreEntrySmoothness
        (
            DirectedEntity& directed,
            const Vector3d& currentPosition,
            const Vector3d& nextPoint,
            const std::optional<Vector2d>& tubeCenter,
            double connectionTolerance
        )
        {
            const double threshold = entryThreshold(connectionTolerance);
            const double approachDx = directed.start.x - currentPosition.x;
            const double cutDx = nextPoint.x - directed.start.x;
            if (std::abs(approachDx) > threshold && std::abs(cutDx) > threshold
                && approachDx * cutDx < 0.0)
            {
                ++directed.entryAxisReversalCount;
            }

            if (tubeCenter.has_value()
                && std::isfinite(tubeCenter->x) && std::isfinite(tubeCenter->y))
            {
                const std::optional<double> approachRotary = rotaryTravelLength
                    (currentPosition, directed.start, *tubeCenter, threshold);
                const std::optional<double> cutRotary = rotaryTravelLength
                    (directed.start, nextPoint, *tubeCenter, threshold);
                if (approachRotary.has_value() && cutRotary.has_value()
                    && std::abs(*approachRotary) > threshold
                    && std::abs(*cutRotary) > threshold
                    && *approachRotary * *cutRotary < 0.0)
                {
                    ++directed.entryAxisReversalCount;
                }

                const double approachA = approachRotary.value_or(0.0);
                const double cutA = cutRotary.value_or(0.0);
                const double approachLength = std::hypot(approachDx, approachA);
                const double cutLength = std::hypot(cutDx, cutA);
                if (approachLength > threshold && cutLength > threshold)
                {
                    const double dotValue = (approachDx * cutDx + approachA * cutA)
                        / (approachLength * cutLength);
                    directed.entryTangentCost = std::clamp(1.0 - dotValue, 0.0, 2.0);
                }
                return;
            }

            const Vector3d approach
            {
                directed.start.x - currentPosition.x,
                directed.start.y - currentPosition.y,
                directed.start.z - currentPosition.z
            };
            const Vector3d cut
            {
                nextPoint.x - directed.start.x,
                nextPoint.y - directed.start.y,
                nextPoint.z - directed.start.z
            };
            const double approachLength = std::sqrt
                (approach.x * approach.x + approach.y * approach.y + approach.z * approach.z);
            const double cutLength = std::sqrt
                (cut.x * cut.x + cut.y * cut.y + cut.z * cut.z);
            if (approachLength > threshold && cutLength > threshold)
            {
                const double dotValue = (approach.x * cut.x + approach.y * cut.y
                    + approach.z * cut.z) / (approachLength * cutLength);
                directed.entryTangentCost = std::clamp(1.0 - dotValue, 0.0, 2.0);
            }
        }

        bool strongEntryZone(machining::TubeZone16 zone)
        {
            return machining::tubeZoneIndex(zone) % 2U == 0U;
        }

        bool reliableEntryProjection
        (
            const machining::TubeSectionProjection& projection,
            double projectionTolerance
        )
        {
            if (!projection.valid || projection.ambiguous
                || projection.confidence < 0.5
                || projection.absoluteDistanceToShell
                    > projectionTolerance * 0.8)
            {
                return false;
            }

            return projection.onBoundary
                ? !strongEntryZone(projection.zone)
                : strongEntryZone(projection.zone);
        }

        Vector3d interpolateEntryPoint
        (
            const Vector3d& start,
            const Vector3d& end,
            double factor
        )
        {
            return
            {
                start.x + (end.x - start.x) * factor,
                start.y + (end.y - start.y) * factor,
                start.z + (end.z - start.z) * factor
            };
        }

        std::optional<ZoneEntryCandidate> classifyZoneEntry
        (
            ZoneEntryCandidateKind kind,
            EntityId entityId,
            std::optional<double> sourceParameter,
            bool reverse,
            const Vector3d& entryPosition,
            const Vector3d& firstCutPoint,
            geometry::SourceGeometryKind sourceKind,
            const machining::TubeSectionModel& section,
            double projectionTolerance
        )
        {
            const double cutLength = distance(entryPosition, firstCutPoint);
            if (!std::isfinite(cutLength)
                || cutLength <= kCalculationEpsilon)
            {
                return std::nullopt;
            }

            std::optional<machining::TubeZone16> reliableZone;
            double confidence = 1.0;
            double maximumDeviation = 0.0;
            for (const double factor : { 0.25, 0.5, 0.75 })
            {
                const Vector3d point = interpolateEntryPoint
                    (entryPosition, firstCutPoint, factor);
                const machining::TubeSectionProjection projection =
                    machining::TubeSectionProjector::project
                    (
                        section, { point.y, point.z },
                        projectionTolerance
                    );
                if (!reliableEntryProjection
                    (projection, projectionTolerance))
                {
                    return std::nullopt;
                }
                if (!reliableZone.has_value())
                    reliableZone = projection.zone;
                else if (*reliableZone != projection.zone)
                    return std::nullopt;
                confidence = std::min(confidence, projection.confidence);
                maximumDeviation = std::max(maximumDeviation,
                    projection.absoluteDistanceToShell);
            }
            if (!reliableZone.has_value()) return std::nullopt;

            const machining::TubeSectionProjection entryProjection =
                machining::TubeSectionProjector::project
                (
                    section, { entryPosition.y, entryPosition.z },
                    projectionTolerance
                );
            if (entryProjection.valid && !entryProjection.onBoundary
                && !entryProjection.ambiguous
                && strongEntryZone(entryProjection.zone)
                && entryProjection.zone != *reliableZone)
            {
                return std::nullopt;
            }

            ZoneEntryCandidate candidate;
            candidate.zone = *reliableZone;
            candidate.kind = kind;
            candidate.entityId = entityId;
            candidate.sourceKind = sourceKind;
            candidate.sourceParameter = sourceParameter;
            candidate.reverse = reverse;
            candidate.entryPosition = entryPosition;
            candidate.firstCutPoint = firstCutPoint;
            candidate.firstCutTangent =
            {
                firstCutPoint.x - entryPosition.x,
                firstCutPoint.y - entryPosition.y,
                firstCutPoint.z - entryPosition.z
            };
            candidate.entryX = entryPosition.x;
            candidate.confidence = confidence;
            candidate.distanceToZoneBoundary =
                (entryProjection.onBoundary || entryProjection.ambiguous)
                ? cutLength * 0.25 : cutLength;
            candidate.distanceToZoneBoundary =
                std::max(0.0, candidate.distanceToZoneBoundary
                    - maximumDeviation);
            candidate.ambiguous = false;
            return candidate;
        }

        void scoreUnwrappedEntry
        (
            DirectedEntity& directed,
            const Vector3d& previousEnd,
            const Vector3d& firstCutPoint,
            const machining::TubeSectionModel& section,
            double connectionTolerance,
            double projectionTolerance
        )
        {
            scoreEntrySmoothness(directed, previousEnd, firstCutPoint,
                Vector2d{ section.geometry.centerY, section.geometry.centerZ },
                connectionTolerance);
            const double previousProjectionTolerance = std::max
            (
                projectionTolerance,
                std::hypot
                (
                    previousEnd.y - section.geometry.centerY,
                    previousEnd.z - section.geometry.centerZ
                ) + std::max(section.geometry.yLength,
                    section.geometry.zWidth)
            );
            const auto previous = machining::TubeSectionProjector::project
                (section, { previousEnd.y, previousEnd.z },
                    previousProjectionTolerance);
            const auto entry = machining::TubeSectionProjector::project
                (section, { directed.start.y, directed.start.z },
                    projectionTolerance);
            const auto cut = machining::TubeSectionProjector::project
                (section, { firstCutPoint.y, firstCutPoint.z },
                    projectionTolerance);
            if (!previous.valid || !entry.valid || !cut.valid
                || section.geometry.perimeter <= kCalculationEpsilon)
            {
                return;
            }

            const double approachX = directed.start.x - previousEnd.x;
            const double approachS = wrappedPerimeterDelta
                (previous.perimeterPosition, entry.perimeterPosition,
                    section.geometry.perimeter);
            const double cutX = firstCutPoint.x - directed.start.x;
            const double cutS = wrappedPerimeterDelta
                (entry.perimeterPosition, cut.perimeterPosition,
                    section.geometry.perimeter);
            const double threshold = entryThreshold(connectionTolerance);
            directed.entryAxisReversalCount = 0;
            if (std::abs(approachX) > threshold
                && std::abs(cutX) > threshold
                && approachX * cutX < 0.0)
            {
                ++directed.entryAxisReversalCount;
            }
            if (std::abs(approachS) > threshold
                && std::abs(cutS) > threshold
                && approachS * cutS < 0.0)
            {
                ++directed.entryAxisReversalCount;
            }
            const double approachLength = std::hypot(approachX, approachS);
            const double cutLength = std::hypot(cutX, cutS);
            if (approachLength > threshold && cutLength > threshold)
            {
                const double dotValue = (approachX * cutX
                    + approachS * cutS) / (approachLength * cutLength);
                directed.entryTangentCost =
                    std::clamp(1.0 - dotValue, 0.0, 2.0);
            }
        }

        bool isSingleClosedEntryOptimizedCurve
        (
            const ProcessGroup& group,
            const PlanningEntity& entity
        )
        {
            return group.entityIds.size() == 1U && entity.path.closed
                && (entity.sourceKind == geometry::SourceGeometryKind::Circle
                    || entity.sourceKind == geometry::SourceGeometryKind::Ellipse
                    || entity.sourceKind == geometry::SourceGeometryKind::Spline);
        }

        std::optional<GroupTraversal> buildSingleClosedCurveTraversal
        (
            const ProcessGroup& group,
            const PlanningEntity& entity,
            const Vector3d& currentPosition,
            bool reverse,
            std::size_t startIndex,
            double connectionTolerance,
            const std::optional<machining::TubeSectionModel>& section,
            const std::optional<Vector2d>& tubeCenter
        )
        {
            const std::size_t pointCount = entity.path.vertices.size();
            if (pointCount < 2U || startIndex >= pointCount) return std::nullopt;

            std::vector<Vector3d> points;
            points.reserve(pointCount);
            for (std::size_t offset = 0U; offset < pointCount; ++offset)
            {
                const std::size_t index = reverse
                    ? (startIndex + pointCount - offset) % pointCount
                    : (startIndex + offset) % pointCount;
                points.push_back(entity.path.vertices[index].position);
            }

            const double threshold = entryThreshold(connectionTolerance);
            const auto next = std::find_if
            (
                points.cbegin() + 1,
                points.cend(),
                [&points, threshold](const Vector3d& point)
                {
                    return distance(points.front(), point) > threshold;
                }
            );
            if (next == points.cend()) return std::nullopt;

            const std::optional<double> selectedStartParameter = entity.startParameter.has_value()
                ? entity.startParameter
                : std::optional<double>(entity.path.vertices[startIndex].sourceParameter);
            if (!selectedStartParameter.has_value()
                || !std::isfinite(*selectedStartParameter)) return std::nullopt;

            DirectedEntity directed;
            directed.entity = &entity;
            directed.reverseRelativeToInput = reverse;
            directed.selectedStartParameter = selectedStartParameter;
            directed.start = points.front();
            directed.end = points.front();
            scoreEntrySmoothness
                (directed, currentPosition, *next, tubeCenter, connectionTolerance);

            GroupTraversal traversal;
            traversal.groupId = group.groupId;
            traversal.entities.push_back(directed);
            traversal.start = directed.start;
            traversal.end = directed.end;
            traversal.entryAxisReversalCount = directed.entryAxisReversalCount;
            traversal.entryTangentCost = directed.entryTangentCost;
            traversal.stableSourceIndex = entity.sourceIndex;
            traversal.stableEntityId = entity.entityId;
            scoreTraversal(traversal, currentPosition, section);
            return traversal;
        }

        bool selectedStartParameterLess
        (
            const std::optional<double>& left,
            const std::optional<double>& right
        )
        {
            if (left.has_value() != right.has_value()) return !left.has_value();
            if (!left.has_value()) return false;
            if (std::abs(*left - *right) > kCalculationEpsilon) return *left < *right;
            return false;
        }

        bool traversalLess
        (
            const GroupTraversal& left,
            const GroupTraversal& right,
            ProcessOrderingStrategy strategy
        )
        {
            if (strategy == ProcessOrderingStrategy::LazyRotation)
            {
                if (left.entryAxisReversalCount != right.entryAxisReversalCount)
                    return left.entryAxisReversalCount < right.entryAxisReversalCount;
                if (std::abs(left.entryTangentCost - right.entryTangentCost) > kCalculationEpsilon)
                    return left.entryTangentCost < right.entryTangentCost;
                if (std::abs(left.rotationCost - right.rotationCost) > kCalculationEpsilon)
                    return left.rotationCost < right.rotationCost;
                if (left.surfaceCost != right.surfaceCost) return left.surfaceCost < right.surfaceCost;
                if (std::abs(left.movementDistance - right.movementDistance) > kCalculationEpsilon)
                    return left.movementDistance < right.movementDistance;
            }
            else
            {
                if (std::abs(left.movementDistance - right.movementDistance) > kCalculationEpsilon)
                    return left.movementDistance < right.movementDistance;
                if (left.entryAxisReversalCount != right.entryAxisReversalCount)
                    return left.entryAxisReversalCount < right.entryAxisReversalCount;
                if (std::abs(left.entryTangentCost - right.entryTangentCost) > kCalculationEpsilon)
                    return left.entryTangentCost < right.entryTangentCost;
            }
            if (left.stableSourceIndex != right.stableSourceIndex)
                return left.stableSourceIndex < right.stableSourceIndex;
            if (left.stableEntityId != right.stableEntityId)
                return left.stableEntityId < right.stableEntityId;
            const std::optional<double> leftStart = left.entities.empty()
                ? std::nullopt : left.entities.front().selectedStartParameter;
            const std::optional<double> rightStart = right.entities.empty()
                ? std::nullopt : right.entities.front().selectedStartParameter;
            if (selectedStartParameterLess(leftStart, rightStart)) return true;
            if (selectedStartParameterLess(rightStart, leftStart)) return false;
            const bool leftReverse = !left.entities.empty()
                && left.entities.front().reverseRelativeToInput;
            const bool rightReverse = !right.entities.empty()
                && right.entities.front().reverseRelativeToInput;
            return leftReverse < rightReverse;
        }

        bool zoneConstrainedTraversalLess
        (
            const GroupTraversal& left,
            const GroupTraversal& right,
            const TraversalSelectionContext& selection,
            ProcessOrderingStrategy strategy
        )
        {
            if (!selection.hardZoneConstraint
                || !selection.requiredEntryZone.has_value()
                || !left.selectedEntry.has_value()
                || !right.selectedEntry.has_value())
            {
                return traversalLess(left, right, strategy);
            }
            const ZoneEntryCandidate& leftEntry = *left.selectedEntry;
            const ZoneEntryCandidate& rightEntry = *right.selectedEntry;
            if (leftEntry.ambiguous != rightEntry.ambiguous)
                return !leftEntry.ambiguous;
            if (std::abs(leftEntry.distanceToMemberEndpoint
                - rightEntry.distanceToMemberEndpoint)
                > kCalculationEpsilon)
            {
                return leftEntry.distanceToMemberEndpoint
                    > rightEntry.distanceToMemberEndpoint;
            }
            if (std::abs(leftEntry.distanceToZoneBoundary
                - rightEntry.distanceToZoneBoundary) > kCalculationEpsilon)
            {
                return leftEntry.distanceToZoneBoundary
                    > rightEntry.distanceToZoneBoundary;
            }
            if (left.entryAxisReversalCount != right.entryAxisReversalCount)
                return left.entryAxisReversalCount
                    < right.entryAxisReversalCount;
            if (std::abs(left.entryTangentCost
                - right.entryTangentCost) > kCalculationEpsilon)
            {
                return left.entryTangentCost < right.entryTangentCost;
            }
            const double leftHitDistance =
                std::abs(leftEntry.entryX - selection.zoneHitX);
            const double rightHitDistance =
                std::abs(rightEntry.entryX - selection.zoneHitX);
            if (std::abs(leftHitDistance - rightHitDistance)
                > kCalculationEpsilon)
            {
                return leftHitDistance < rightHitDistance;
            }
            if (std::abs(left.rotationCost - right.rotationCost)
                > kCalculationEpsilon)
            {
                return left.rotationCost < right.rotationCost;
            }
            if (std::abs(left.movementDistance - right.movementDistance)
                > kCalculationEpsilon)
            {
                return left.movementDistance < right.movementDistance;
            }
            return traversalLess(left, right, strategy);
        }

        bool surfaceSweepCandidateLess
        (
            const SchedulingCandidate& left,
            const SchedulingCandidate& right,
            const SurfaceSweepState& state,
            double tolerance
        )
        {
            const double leftDelta = left.footprint.anchorX - state.currentX;
            const double rightDelta = right.footprint.anchorX - state.currentX;
            const bool leftForward = state.longitudinalDirection >= 0
                ? leftDelta >= -tolerance : leftDelta <= tolerance;
            const bool rightForward = state.longitudinalDirection >= 0
                ? rightDelta >= -tolerance : rightDelta <= tolerance;
            if (leftForward != rightForward) return leftForward;

            const double leftBacktrack = leftForward ? 0.0 : std::abs(leftDelta);
            const double rightBacktrack = rightForward ? 0.0 : std::abs(rightDelta);
            if (std::abs(leftBacktrack - rightBacktrack) > kCalculationEpsilon)
                return leftBacktrack < rightBacktrack;
            if (std::abs(std::abs(leftDelta) - std::abs(rightDelta))
                > kCalculationEpsilon)
                return std::abs(leftDelta) < std::abs(rightDelta);
            if (std::abs(left.traversal.rotationCost - right.traversal.rotationCost)
                > kCalculationEpsilon)
                return left.traversal.rotationCost < right.traversal.rotationCost;
            if (left.traversal.entryAxisReversalCount
                != right.traversal.entryAxisReversalCount)
                return left.traversal.entryAxisReversalCount
                    < right.traversal.entryAxisReversalCount;
            if (std::abs(left.traversal.entryTangentCost
                    - right.traversal.entryTangentCost) > kCalculationEpsilon)
                return left.traversal.entryTangentCost
                    < right.traversal.entryTangentCost;
            if (std::abs(left.traversal.movementDistance
                    - right.traversal.movementDistance) > kCalculationEpsilon)
                return left.traversal.movementDistance
                    < right.traversal.movementDistance;
            if (left.traversal.stableSourceIndex != right.traversal.stableSourceIndex)
                return left.traversal.stableSourceIndex
                    < right.traversal.stableSourceIndex;
            if (left.traversal.stableEntityId != right.traversal.stableEntityId)
                return left.traversal.stableEntityId
                    < right.traversal.stableEntityId;
            return traversalLess(left.traversal, right.traversal,
                ProcessOrderingStrategy::NearestNext);
        }

        bool footprintUsesRegion
        (
            const ProcessSurfaceFootprint& footprint,
            machining::TubeSurfaceRegion region,
            bool allowMixed
        )
        {
            if (footprint.dominantRegion == region) return true;
            return allowMixed
                && footprint.dominantRegion == machining::TubeSurfaceRegion::Mixed
                && footprint.entryRegion == region;
        }

        machining::TubeZone16 zoneAtOffset
        (
            machining::TubeZone16 initialZone,
            int offset,
            int perimeterDirection
        )
        {
            const int initial = static_cast<int>(machining::tubeZoneIndex(initialZone));
            const int wrapped = (initial + offset * perimeterDirection
                + static_cast<int>(machining::kTubeZone16Count))
                % static_cast<int>(machining::kTubeZone16Count);
            return static_cast<machining::TubeZone16>(wrapped);
        }

        std::optional<machining::TubeZone16> firstZoneInSweep
        (
            machining::TubeZoneMask mask,
            machining::TubeZone16 initialZone,
            int perimeterDirection
        )
        {
            for (int offset = 0;
                offset < static_cast<int>(machining::kTubeZone16Count);
                ++offset)
            {
                const machining::TubeZone16 zone =
                    zoneAtOffset(initialZone, offset, perimeterDirection);
                if ((mask & machining::tubeZoneBit(zone)) != 0U)
                    return zone;
            }
            return std::nullopt;
        }

        machining::TubeZoneMask strongTubeZoneMask()
        {
            machining::TubeZoneMask mask = 0U;
            for (std::size_t index = 0U;
                index < machining::kTubeZone16Count; index += 2U)
            {
                mask |= machining::tubeZoneBit
                    (static_cast<machining::TubeZone16>(index));
            }
            return mask;
        }

        bool zoneCompleted
        (
            const TubeZoneSweepPartition& partition,
            machining::TubeZone16 zone,
            const std::unordered_set<int>& scheduled
        )
        {
            const auto& bucket =
                partition.zoneBuckets[machining::tubeZoneIndex(zone)];
            return std::all_of(bucket.cbegin(), bucket.cend(),
                [&scheduled](int groupId)
                {
                    return scheduled.find(groupId) != scheduled.end();
                });
        }

        QString zoneMaskText(machining::TubeZoneMask mask)
        {
            return QStringLiteral("0x%1").arg(static_cast<unsigned int>(mask),
                4, 16, QLatin1Char('0')).toUpper();
        }

        QString zoneSpansText(const ProcessGroupZoneProfile& profile)
        {
            QStringList spans;
            for (std::size_t index = 0U;
                index < machining::kTubeZone16Count; ++index)
            {
                const machining::TubeZone16 zone =
                    static_cast<machining::TubeZone16>(index);
                if ((profile.possibleMask & machining::tubeZoneBit(zone)) == 0U)
                    continue;
                const machining::TubeZoneSpan& span = profile.zoneSpans[index];
                spans.push_back(QStringLiteral("%1:[%2,%3]")
                    .arg(machining::tubeZoneName(zone))
                    .arg(span.minimumX, 0, 'f', 6)
                    .arg(span.maximumX, 0, 'f', 6));
            }
            return spans.join(QLatin1Char(';'));
        }

        QString processGroupKeyText(const ProcessGroup& group)
        {
            std::vector<EntityId> ids = group.entityIds;
            std::sort(ids.begin(), ids.end());
            QStringList values;
            values.reserve(static_cast<qsizetype>(ids.size()));
            for (const EntityId entityId : ids)
                values.push_back(QString::number(entityId));
            return values.join(QLatin1Char('+'));
        }

        bool processGroupStableLess
        (
            const ProcessGroup& left,
            const ProcessGroup& right
        )
        {
            std::vector<EntityId> leftIds = left.entityIds;
            std::vector<EntityId> rightIds = right.entityIds;
            std::sort(leftIds.begin(), leftIds.end());
            std::sort(rightIds.begin(), rightIds.end());
            return leftIds < rightIds;
        }

        std::optional<machining::TubeZone16> traversalExitZone
        (
            const GroupTraversal& traversal,
            const machining::TubeSectionModel& section,
            double projectionTolerance
        )
        {
            if (traversal.entities.empty()) return std::nullopt;
            const DirectedEntity& directed = traversal.entities.back();
            if (directed.entity == nullptr) return std::nullopt;
            const std::vector<Vector3d> points = directedTraversalPoints(directed);
            for (std::size_t offset = 1U; offset < points.size(); ++offset)
            {
                const Vector3d& end = points[points.size() - offset];
                const Vector3d& start = points[points.size() - offset - 1U];
                if (distance(start, end) <= kCalculationEpsilon) continue;

                std::array<machining::TubeSectionProjection, 3> samples;
                for (std::size_t sample = 0U; sample < samples.size(); ++sample)
                {
                    const double parameter = 0.25
                        + static_cast<double>(sample) * 0.25;
                    const Vector2d yz
                    {
                        start.y + (end.y - start.y) * parameter,
                        start.z + (end.z - start.z) * parameter
                    };
                    samples[sample] = machining::TubeSectionProjector::project
                        (section, yz, projectionTolerance);
                }

                std::array<int, machining::kTubeZone16Count> counts{};
                for (const auto& sample : samples)
                {
                    if (sample.valid && !sample.ambiguous)
                        ++counts[machining::tubeZoneIndex(sample.zone)];
                }
                int bestCount = 0;
                std::optional<machining::TubeZone16> bestZone;
                bool tied = false;
                for (std::size_t index = 0U; index < counts.size(); ++index)
                {
                    if (counts[index] > bestCount)
                    {
                        bestCount = counts[index];
                        bestZone = static_cast<machining::TubeZone16>(index);
                        tied = false;
                    }
                    else if (counts[index] > 0 && counts[index] == bestCount)
                    {
                        tied = true;
                    }
                }
                if (bestZone.has_value() && !tied) return bestZone;
                if (samples[1].valid && !samples[1].ambiguous)
                    return samples[1].zone;
                return std::nullopt;
            }
            return std::nullopt;
        }

        Diagnostic zone16SweepDiagnostic
        (
            const OperationContext& context,
            const Zone16SweepReport& report
        )
        {
            QVariantMap values;
            values.insert(QStringLiteral("zone16SweepSummary"), true);
            values.insert(QStringLiteral("partitionId"), report.partitionId);
            values.insert(QStringLiteral("initialZone"),
                machining::tubeZoneName(report.initialZone));
            values.insert(QStringLiteral("perimeterDirection"),
                report.perimeterDirection >= 0
                    ? QStringLiteral("Clockwise")
                    : QStringLiteral("CounterClockwise"));
            values.insert(QStringLiteral("longitudinalDirection"),
                report.longitudinalDirection);
            values.insert(QStringLiteral("partitionMinimumX"),
                report.partitionMinimumX);
            values.insert(QStringLiteral("partitionMaximumX"),
                report.partitionMaximumX);
            values.insert(QStringLiteral("processedUnitCount"),
                report.processedUnitCount);
            values.insert(QStringLiteral("zoneTransitions"),
                report.zoneTransitionCount);
            values.insert(QStringLiteral("backtrackCount"),
                report.backtrackCount);
            values.insert(QStringLiteral("selectedUnits"),
                report.selectedUnits);
            values.insert(QStringLiteral("status"), report.status);
            return planningDiagnostic
            (
                context,
                DiagnosticCode::ProcessPlanningZone16SweepSummary,
                QStringLiteral("四轴 16 区位扫描加工段已完成。"),
                QStringLiteral("Zone16 sweep partition completed."),
                values,
                report.status == QStringLiteral("Success")
                    ? DiagnosticSeverity::Info : DiagnosticSeverity::Warning
            );
        }

        Diagnostic zoneOwnershipDiagnostic
        (
            const OperationContext& context,
            const TubeZoneSweepPartition& partition,
            const ProcessGroup& group,
            const ProcessGroupZoneProfile& profile,
            const ZoneSweepOwnership& ownership
        )
        {
            QVariantMap values;
            values.insert(QStringLiteral("zoneOwnership"), true);
            values.insert(QStringLiteral("partitionId"), partition.partitionId);
            values.insert(QStringLiteral("unitKey"), processGroupKeyText(group));
            values.insert(QStringLiteral("certainMask"),
                zoneMaskText(profile.certainMask));
            values.insert(QStringLiteral("possibleMask"),
                zoneMaskText(profile.possibleMask));
            values.insert(QStringLiteral("ownerCandidateMask"),
                zoneMaskText(ownership.ownerCandidateMask));
            values.insert(QStringLiteral("ownerZone"),
                machining::tubeZoneName(ownership.ownerZone));
            values.insert(QStringLiteral("ownerBasis"),
                ownership.usedPossibleFallback
                ? QStringLiteral("PossibleOccupancyFallback")
                : QStringLiteral("CertainOccupancy"));
            values.insert(QStringLiteral("legalEntryMaskBefore"),
                zoneMaskText(ownership.legalEntryMaskBefore));
            values.insert(QStringLiteral("usedPossibleFallback"),
                ownership.usedPossibleFallback);
            values.insert(QStringLiteral("usedBoundaryFallback"),
                ownership.usedBoundaryFallback);
            return planningDiagnostic
            (
                context,
                ownership.usedPossibleFallback
                    || ownership.usedBoundaryFallback
                    ? DiagnosticCode::ProcessPlanningZoneSweepFallbackOwner
                    : DiagnosticCode::ProcessPlanningZone16SweepSummary,
                ownership.usedPossibleFallback
                    || ownership.usedBoundaryFallback
                    ? QStringLiteral("加工单元使用保守区位作为唯一生产归属。")
                    : QStringLiteral("加工单元已确定唯一生产区位。"),
                QStringLiteral("A single immutable owner zone was selected for this partition."),
                values,
                ownership.usedPossibleFallback
                    || ownership.usedBoundaryFallback
                    ? DiagnosticSeverity::Warning : DiagnosticSeverity::Info
            );
        }

        Diagnostic zonePhaseDiagnostic
        (
            const OperationContext& context,
            const TubeZoneSweepPartition& partition,
            machining::TubeZone16 zone,
            const std::unordered_set<int>& scheduled,
            const QString& event
        )
        {
            const auto& bucket =
                partition.zoneBuckets[machining::tubeZoneIndex(zone)];
            const int processedUnitCount = static_cast<int>(std::count_if
            (
                bucket.cbegin(), bucket.cend(),
                [&scheduled](int groupId)
                {
                    return scheduled.find(groupId) != scheduled.end();
                }
            ));
            QVariantMap values;
            values.insert(QStringLiteral("zonePhase"), true);
            values.insert(QStringLiteral("partitionId"), partition.partitionId);
            values.insert(QStringLiteral("zone"), machining::tubeZoneName(zone));
            values.insert(QStringLiteral("event"), event);
            values.insert(QStringLiteral("ownedUnitCount"),
                static_cast<int>(bucket.size()));
            values.insert(QStringLiteral("processedUnitCount"),
                processedUnitCount);
            values.insert(QStringLiteral("remainingUnitCount"),
                static_cast<int>(bucket.size()) - processedUnitCount);
            return planningDiagnostic
            (
                context,
                DiagnosticCode::ProcessPlanningZone16SweepSummary,
                event == QStringLiteral("Enter")
                    ? QStringLiteral("16 区位加工阶段已进入。")
                    : QStringLiteral("16 区位加工阶段已完成。"),
                QStringLiteral("Zone phase lifecycle event."),
                values,
                DiagnosticSeverity::Info
            );
        }

        QVariantMap closedLoopDiagnosticValues(const ClosedLoopTraversalReport& report)
        {
            auto entityIdsText = [](const std::vector<EntityId>& entityIds)
            {
                QStringList values;
                values.reserve(static_cast<qsizetype>(entityIds.size()));
                for (const EntityId entityId : entityIds)
                    values.push_back(QString::number(entityId));
                return values.join(QLatin1Char(','));
            };
            QStringList reverseValues;
            reverseValues.reserve(static_cast<qsizetype>(report.selectedReverse.size()));
            for (const bool reverse : report.selectedReverse)
                reverseValues.push_back(reverse ? QStringLiteral("1") : QStringLiteral("0"));

            QVariantMap values;
            values.insert(QStringLiteral("closedLoopSummary"), true);
            values.insert(QStringLiteral("groupId"), report.groupId);
            values.insert(QStringLiteral("memberCount"), report.memberCount);
            values.insert(QStringLiteral("memberEntityIds"), entityIdsText(report.memberEntityIds));
            values.insert(QStringLiteral("nodeCount"), report.nodeCount);
            values.insert(QStringLiteral("connectedComponentCount"), report.connectedComponentCount);
            values.insert(QStringLiteral("branchNodeCount"), report.branchNodeCount);
            values.insert(QStringLiteral("invalidDegreeNodeCount"), report.invalidDegreeNodeCount);
            values.insert(QStringLiteral("candidateCount"), report.candidateCount);
            values.insert(QStringLiteral("selectedOrder"), entityIdsText(report.selectedOrder));
            values.insert(QStringLiteral("selectedReverse"), reverseValues.join(QLatin1Char(',')));
            values.insert(QStringLiteral("status"), report.status);
            values.insert(QStringLiteral("failureReason"), report.failureReason);
            return values;
        }

        Diagnostic surfaceSweepDiagnostic
        (
            const OperationContext& context,
            const SurfaceSweepReport& report
        )
        {
            QVariantMap values;
            values.insert(QStringLiteral("surfaceSweepSummary"), true);
            values.insert(QStringLiteral("partitionId"), report.partitionId);
            values.insert(QStringLiteral("initialRegion"),
                surfaceRegionName(report.initialRegion));
            values.insert(QStringLiteral("perimeterDirection"),
                report.perimeterDirection);
            values.insert(QStringLiteral("longitudinalDirection"),
                report.longitudinalDirection);
            values.insert(QStringLiteral("selectedUnits"),
                report.selectedUnits.join(QLatin1Char(',')));
            values.insert(QStringLiteral("selectedUnitCount"),
                report.selectedUnitCount);
            values.insert(QStringLiteral("regionTransitions"),
                report.regionTransitionCount);
            values.insert(QStringLiteral("backtrackCount"),
                report.backtrackCount);
            values.insert(QStringLiteral("longitudinalBacktrackDistance"),
                report.longitudinalBacktrackDistance);
            values.insert(QStringLiteral("status"), QStringLiteral("Success"));
            return planningDiagnostic
            (
                context,
                DiagnosticCode::ProcessPlanningSurfaceSweepSummary,
                QStringLiteral("四轴按面扫描分区已完成。"),
                QStringLiteral("LazyRotation surface sweep partition completed."),
                values,
                DiagnosticSeverity::Info
            );
        }

        class ClosedLoopTraversalBuilder
        {
        public:
            struct Result
            {
                std::optional<GroupTraversal> traversal;
                ClosedLoopTraversalReport report;
            };

            static Result build
            (
                const ProcessGroup& group,
                const std::unordered_map<EntityId, const PlanningEntity*>& entities,
                const Vector3d& currentPosition,
                const ProcessPlanningPolicy& policy,
                const std::optional<machining::TubeSectionModel>& section,
                const std::optional<Vector2d>& tubeCenter,
                ProcessOrderingStrategy selectionStrategy,
                const TraversalSelectionContext* selection = nullptr
            )
            {
                Result result;
                result.report.groupId = group.groupId;
                result.report.memberEntityIds = group.entityIds;
                std::sort(result.report.memberEntityIds.begin(), result.report.memberEntityIds.end());
                result.report.memberCount = static_cast<int>(group.entityIds.size());

                struct Edge
                {
                    const PlanningEntity* entity = nullptr;
                    Vector3d sourceStart;
                    Vector3d sourceEnd;
                    int startNode = -1;
                    int endNode = -1;
                };
                std::vector<Edge> edges;
                edges.reserve(group.entityIds.size());
                std::set<EntityId> uniqueIds;
                for (const EntityId entityId : group.entityIds)
                {
                    const auto found = entities.find(entityId);
                    if (found == entities.end() || found->second == nullptr
                        || !uniqueIds.insert(entityId).second)
                    {
                        result.report.failureReason = QStringLiteral("Closed-loop member is missing or duplicated.");
                        return result;
                    }
                    const PlanningEntity& entity = *found->second;
                    if (entity.path.closed || entity.path.vertices.size() < 2U)
                    {
                        result.report.failureReason = entity.path.closed
                            ? QStringLiteral("Multi-entity closed loop contains a semantically closed member.")
                            : QStringLiteral("Closed-loop member has fewer than two path points.");
                        return result;
                    }
                    const Vector3d sourceStart = entity.path.vertices.front().position;
                    const Vector3d sourceEnd = entity.path.vertices.back().position;
                    if (!std::isfinite(sourceStart.x) || !std::isfinite(sourceStart.y)
                        || !std::isfinite(sourceStart.z) || !std::isfinite(sourceEnd.x)
                        || !std::isfinite(sourceEnd.y) || !std::isfinite(sourceEnd.z))
                    {
                        result.report.failureReason = QStringLiteral("Closed-loop member endpoint is not finite.");
                        return result;
                    }
                    edges.push_back({ &entity, sourceStart, sourceEnd });
                }
                std::sort(edges.begin(), edges.end(), [](const Edge& left, const Edge& right)
                {
                    if (left.entity->sourceIndex != right.entity->sourceIndex)
                        return left.entity->sourceIndex < right.entity->sourceIndex;
                    return left.entity->entityId < right.entity->entityId;
                });

                const std::size_t endpointCount = edges.size() * 2U;
                std::vector<std::size_t> parents(endpointCount);
                for (std::size_t index = 0; index < endpointCount; ++index) parents[index] = index;
                const auto findRoot = [&parents](std::size_t value)
                {
                    std::size_t root = value;
                    while (parents[root] != root) root = parents[root];
                    while (parents[value] != value)
                    {
                        const std::size_t next = parents[value];
                        parents[value] = root;
                        value = next;
                    }
                    return root;
                };
                const auto endpoint = [&edges](std::size_t index) -> const Vector3d&
                {
                    const Edge& edge = edges[index / 2U];
                    return index % 2U == 0U ? edge.sourceStart : edge.sourceEnd;
                };
                for (std::size_t left = 0; left < endpointCount; ++left)
                {
                    for (std::size_t right = left + 1U; right < endpointCount; ++right)
                    {
                        if (distance(endpoint(left), endpoint(right)) > policy.connectionTolerance) continue;
                        const std::size_t leftRoot = findRoot(left);
                        const std::size_t rightRoot = findRoot(right);
                        if (leftRoot != rightRoot) parents[rightRoot] = leftRoot;
                    }
                }

                std::map<std::size_t, int> nodeByRoot;
                for (std::size_t edgeIndex = 0; edgeIndex < edges.size(); ++edgeIndex)
                {
                    const std::size_t startRoot = findRoot(edgeIndex * 2U);
                    const std::size_t endRoot = findRoot(edgeIndex * 2U + 1U);
                    const auto nodeFor = [&nodeByRoot](std::size_t root)
                    {
                        const auto inserted = nodeByRoot.emplace
                            (root, static_cast<int>(nodeByRoot.size()));
                        return inserted.first->second;
                    };
                    edges[edgeIndex].startNode = nodeFor(startRoot);
                    edges[edgeIndex].endNode = nodeFor(endRoot);
                }
                result.report.nodeCount = static_cast<int>(nodeByRoot.size());

                std::vector<std::vector<std::size_t>> adjacency(nodeByRoot.size());
                for (std::size_t edgeIndex = 0; edgeIndex < edges.size(); ++edgeIndex)
                {
                    adjacency[static_cast<std::size_t>(edges[edgeIndex].startNode)].push_back(edgeIndex);
                    adjacency[static_cast<std::size_t>(edges[edgeIndex].endNode)].push_back(edgeIndex);
                }
                for (const auto& incidentEdges : adjacency)
                {
                    if (incidentEdges.size() > 2U) ++result.report.branchNodeCount;
                    if (incidentEdges.size() != 2U) ++result.report.invalidDegreeNodeCount;
                }

                std::vector<bool> visitedNodes(adjacency.size(), false);
                for (std::size_t node = 0; node < adjacency.size(); ++node)
                {
                    if (visitedNodes[node] || adjacency[node].empty()) continue;
                    ++result.report.connectedComponentCount;
                    std::vector<std::size_t> pending{ node };
                    visitedNodes[node] = true;
                    while (!pending.empty())
                    {
                        const std::size_t currentNode = pending.back();
                        pending.pop_back();
                        for (const std::size_t edgeIndex : adjacency[currentNode])
                        {
                            const Edge& edge = edges[edgeIndex];
                            const std::size_t nextNode = static_cast<std::size_t>
                                (edge.startNode == static_cast<int>(currentNode)
                                    ? edge.endNode : edge.startNode);
                            if (!visitedNodes[nextNode])
                            {
                                visitedNodes[nextNode] = true;
                                pending.push_back(nextNode);
                            }
                        }
                    }
                }

                result.report.simpleLoopValid = result.report.connectedComponentCount == 1
                    && result.report.branchNodeCount == 0
                    && result.report.invalidDegreeNodeCount == 0
                    && edges.size() == adjacency.size();
                if (!result.report.simpleLoopValid)
                {
                    result.report.failureReason = QStringLiteral("Closed-loop endpoint graph is not one simple cycle.");
                    return result;
                }

                std::optional<GroupTraversal> best;
                std::vector<GroupTraversal> connectionCandidates;
                int bestStartEndpoint = -1;
                int bestLoopDirection = -1;
                int wrongZoneRejectedCount = 0;
                for (std::size_t startEdgeIndex = 0; startEdgeIndex < edges.size(); ++startEdgeIndex)
                {
                    for (const bool startReverse : { false, true })
                    {
                        const Edge& startEdge = edges[startEdgeIndex];
                        if (!directionAllowed(*startEdge.entity, startReverse,
                            policy.allowReverse)
                            || !manualDirectionAllowed(*startEdge.entity,
                                startReverse, selection))
                        {
                            continue;
                        }

                        GroupTraversal candidate;
                        candidate.groupId = group.groupId;
                        std::vector<bool> used(edges.size(), false);
                        int currentNode = startReverse ? startEdge.endNode : startEdge.startNode;
                        const int initialNode = currentNode;
                        Vector3d previousEnd;
                        bool hasPreviousEnd = false;
                        Vector3d firstNextPoint;
                        bool candidateValid = true;

                        for (std::size_t step = 0; step < edges.size(); ++step)
                        {
                            std::vector<std::size_t> unusedIncident;
                            for (const std::size_t edgeIndex : adjacency[static_cast<std::size_t>(currentNode)])
                            {
                                if (!used[edgeIndex]
                                    && std::find(unusedIncident.begin(), unusedIncident.end(), edgeIndex)
                                        == unusedIncident.end())
                                    unusedIncident.push_back(edgeIndex);
                            }
                            const std::size_t edgeIndex = step == 0U
                                ? startEdgeIndex
                                : unusedIncident.size() == 1U
                                    ? unusedIncident.front() : edges.size();
                            if (edgeIndex >= edges.size() || used[edgeIndex])
                            {
                                candidateValid = false;
                                break;
                            }

                            const Edge& edge = edges[edgeIndex];
                            const bool reverse = edge.endNode == currentNode;
                            if ((edge.startNode != currentNode && edge.endNode != currentNode)
                                || !directionAllowed(*edge.entity, reverse,
                                    policy.allowReverse)
                                || !manualDirectionAllowed(*edge.entity,
                                    reverse, selection))
                            {
                                candidateValid = false;
                                break;
                            }
                            std::vector<Vector3d> points = directedPoints(*edge.entity, reverse);
                            if (points.size() < 2U
                                || (hasPreviousEnd
                                    && distance(previousEnd, points.front()) > policy.connectionTolerance))
                            {
                                candidateValid = false;
                                break;
                            }
                            const double threshold = entryThreshold(policy.connectionTolerance);
                            const auto nextPoint = std::find_if
                            (
                                points.cbegin() + 1,
                                points.cend(),
                                [&points, threshold](const Vector3d& point)
                                { return distance(points.front(), point) > threshold; }
                            );
                            if (nextPoint == points.cend())
                            {
                                candidateValid = false;
                                break;
                            }

                            DirectedEntity directed;
                            directed.entity = edge.entity;
                            directed.reverseRelativeToInput = reverse;
                            directed.selectedStartParameter = edge.entity->startParameter;
                            directed.start = points.front();
                            directed.end = points.back();
                            candidate.entities.push_back(directed);
                            if (step == 0U) firstNextPoint = *nextPoint;
                            previousEnd = directed.end;
                            hasPreviousEnd = true;
                            used[edgeIndex] = true;
                            currentNode = reverse ? edge.startNode : edge.endNode;
                        }

                        if (!candidateValid || currentNode != initialNode
                            || candidate.entities.size() != edges.size()) continue;
                        candidate.start = candidate.entities.front().start;
                        if (distance(candidate.entities.back().end, candidate.start)
                            > policy.connectionTolerance) continue;
                        candidate.end = candidate.start;
                        const DirectedEntity& firstDirected =
                            candidate.entities.front();
                        const auto firstVertices = firstDirected.reverseRelativeToInput
                            ? std::vector<geometry::PathVertex3D>
                                (firstDirected.entity->path.vertices.rbegin(),
                                    firstDirected.entity->path.vertices.rend())
                            : firstDirected.entity->path.vertices;
                        const std::optional<double> entryParameter =
                            firstVertices.empty() ? std::nullopt
                                : std::optional<double>
                                    (firstVertices.front().sourceParameter);
                        if (selection != nullptr
                            && selection->hardZoneConstraint
                            && selection->requiredEntryZone.has_value()
                            && section.has_value())
                        {
                            auto entry = classifyZoneEntry
                            (
                                ZoneEntryCandidateKind::ClosedLoopConnection,
                                firstDirected.entity->entityId,
                                entryParameter,
                                firstDirected.reverseRelativeToInput,
                                candidate.start, firstNextPoint,
                                firstDirected.entity->sourceKind, *section,
                                selection->projectionTolerance
                            );
                            const bool manualStartMatches =
                                !firstDirected.entity->manualStartParameter.has_value()
                                || (entryParameter.has_value()
                                    && std::abs(*entryParameter
                                        - *firstDirected.entity->manualStartParameter)
                                        <= 1.0e-10);
                            const bool otherManualStartExists = std::any_of
                            (
                                candidate.entities.cbegin() + 1,
                                candidate.entities.cend(),
                                [](const DirectedEntity& directed)
                                {
                                    return directed.entity != nullptr
                                        && directed.entity->manualStartParameter.has_value();
                                }
                            );
                            if (!entry.has_value()
                                || entry->zone
                                    != *selection->requiredEntryZone
                                || !manualStartMatches
                                || otherManualStartExists)
                            {
                                ++wrongZoneRejectedCount;
                                continue;
                            }
                            candidate.selectedEntry = std::move(entry);
                            scoreUnwrappedEntry(candidate.entities.front(),
                                selection->previousTransferAnchor, firstNextPoint,
                                *section, policy.connectionTolerance,
                                selection->projectionTolerance);
                            candidate.entryRefinementMode =
                                QStringLiteral("NearestConnection");
                            candidate.previousCutEnd =
                                selection->previousCutEnd;
                            candidate.previousTransferAnchor =
                                selection->previousTransferAnchor;
                            candidate.entryTravelDistance =
                                distance(selection->previousTransferAnchor,
                                    candidate.start);
                            candidate.nearestConnectionDistance =
                                candidate.entryTravelDistance;
                        }
                        else
                        {
                            scoreEntrySmoothness
                            (
                                candidate.entities.front(), currentPosition,
                                firstNextPoint, tubeCenter,
                                policy.connectionTolerance
                            );
                        }
                        candidate.entryAxisReversalCount =
                            candidate.entities.front().entryAxisReversalCount;
                        candidate.entryTangentCost = candidate.entities.front().entryTangentCost;
                        candidate.stableSourceIndex = candidate.entities.front().entity->sourceIndex;
                        candidate.stableEntityId = candidate.entities.front().entity->entityId;
                        scoreTraversal(candidate, currentPosition, section);
                        ++result.report.candidateCount;
                        if (selection != nullptr
                            && selection->hardZoneConstraint)
                        {
                            const double dotValue = std::clamp
                            (
                                1.0 - candidate.entryTangentCost,
                                -1.0,
                                1.0
                            );
                            candidate.approachCutDot = dotValue;
                            candidate.approachCutAngle =
                                std::acos(dotValue)
                                * 180.0 / 3.14159265358979323846;
                            connectionCandidates.push_back
                                (std::move(candidate));
                            continue;
                        }

                        const int startEndpoint = startReverse ? 1 : 0;
                        const int loopDirection = startReverse ? 1 : 0;
                        const auto stableLess = [&candidate, startEndpoint, loopDirection,
                            &best, bestStartEndpoint, bestLoopDirection,
                            selectionStrategy, selection]()
                        {
                            if (!best.has_value()) return true;
                            if (selection != nullptr
                                && zoneConstrainedTraversalLess(candidate,
                                    *best, *selection, selectionStrategy))
                            {
                                return true;
                            }
                            if (selection != nullptr
                                && zoneConstrainedTraversalLess(*best,
                                    candidate, *selection,
                                    selectionStrategy))
                            {
                                return false;
                            }
                            if (selection == nullptr
                                && traversalLess(candidate, *best,
                                    selectionStrategy))
                            {
                                return true;
                            }
                            if (selection == nullptr
                                && traversalLess(*best, candidate,
                                    selectionStrategy))
                            {
                                return false;
                            }
                            if (startEndpoint != bestStartEndpoint)
                                return startEndpoint < bestStartEndpoint;
                            if (loopDirection != bestLoopDirection)
                                return loopDirection < bestLoopDirection;
                            std::vector<EntityId> candidateOrder;
                            std::vector<EntityId> bestOrder;
                            for (const DirectedEntity& directed : candidate.entities)
                                candidateOrder.push_back(directed.entity->entityId);
                            for (const DirectedEntity& directed : best->entities)
                                bestOrder.push_back(directed.entity->entityId);
                            return candidateOrder < bestOrder;
                        };
                        if (stableLess())
                        {
                            best = std::move(candidate);
                            bestStartEndpoint = startEndpoint;
                            bestLoopDirection = loopDirection;
                        }
                    }
                }

                if (selection != nullptr
                    && selection->hardZoneConstraint
                    && !connectionCandidates.empty())
                {
                    const double tieTolerance = std::max
                    (
                        policy.connectionDistanceTieTolerance,
                        kCalculationEpsilon
                    );
                    const auto connectionLess =
                        [tieTolerance](const GroupTraversal& left,
                            const GroupTraversal& right)
                    {
                        if (std::abs(left.entryTravelDistance
                            - right.entryTravelDistance) > tieTolerance)
                        {
                            return left.entryTravelDistance
                                < right.entryTravelDistance;
                        }
                        if (std::abs(left.entryTangentCost
                            - right.entryTangentCost)
                            > kCalculationEpsilon)
                        {
                            return left.entryTangentCost
                                < right.entryTangentCost;
                        }
                        if (left.stableEntityId != right.stableEntityId)
                            return left.stableEntityId
                                < right.stableEntityId;
                        const bool leftReverse = !left.entities.empty()
                            && left.entities.front()
                                .reverseRelativeToInput;
                        const bool rightReverse = !right.entities.empty()
                            && right.entities.front()
                                .reverseRelativeToInput;
                        return leftReverse < rightReverse;
                    };
                    auto selected = std::min_element
                        (connectionCandidates.begin(),
                            connectionCandidates.end(), connectionLess);
                    best = *selected;
                    const double samePointTolerance = std::max
                        (policy.connectionDistanceTieTolerance, 1.0e-8);
                    best->forwardAngle =
                        std::numeric_limits<double>::quiet_NaN();
                    best->reverseAngle =
                        std::numeric_limits<double>::quiet_NaN();
                    for (const GroupTraversal& candidate :
                        connectionCandidates)
                    {
                        if (distance(candidate.start, best->start)
                            > samePointTolerance
                            || candidate.entities.empty())
                        {
                            continue;
                        }
                        if (candidate.entities.front()
                            .reverseRelativeToInput)
                        {
                            best->reverseAngle =
                                candidate.approachCutAngle;
                        }
                        else
                        {
                            best->forwardAngle =
                                candidate.approachCutAngle;
                        }
                    }
                }
                if (!best.has_value())
                {
                    result.report.wrongZoneRejectedCount =
                        wrongZoneRejectedCount;
                    result.report.failureReason = QStringLiteral("No complete loop traversal satisfies member direction constraints.");
                    return result;
                }
                result.report.status = QStringLiteral("Success");
                result.report.wrongZoneRejectedCount =
                    wrongZoneRejectedCount;
                best->wrongZoneRejectedCount = wrongZoneRejectedCount;
                for (const DirectedEntity& directed : best->entities)
                {
                    result.report.selectedOrder.push_back(directed.entity->entityId);
                    result.report.selectedReverse.push_back(directed.reverseRelativeToInput);
                }
                result.traversal = std::move(best);
                return result;
            }
        };

        class ClosedLoopZoneRunBuilder
        {
        public:
            struct BreakSegment;
            struct BreakZoneRun;

            struct Result
            {
                std::optional<GroupTraversal> traversal;
                ClosedLoopTraversalReport closedLoopReport;
                BreakBoundaryTraversalReport report;
            };

            static Result buildBreak
            (
                const ProcessGroup& group,
                const std::unordered_map<EntityId, const PlanningEntity*>& entities,
                const Vector3d& currentPosition,
                const ProcessPlanningPolicy& policy,
                const machining::TubeSectionModel& section,
                const std::optional<Vector2d>& tubeCenter,
                ProcessOrderingStrategy selectionStrategy,
                double projectionTolerance,
                int boundaryRank,
                machining::TubeZone16 preferredStartZone
            )
            {
                Result result;
                result.report.groupId = group.groupId;
                result.report.boundaryRank = boundaryRank;
                result.report.preferredStartZone = preferredStartZone;
                result.report.forcedTopMidpoint =
                    preferredStartZone == machining::TubeZone16::TopFace;
                if (!group.entityIds.empty())
                {
                    const auto found = entities.find(group.entityIds.front());
                    if (found != entities.end() && found->second != nullptr)
                        result.report.boundaryPairId =
                            found->second->boundaryPairId;
                }

                auto loop = ClosedLoopTraversalBuilder::build
                (
                    group, entities, currentPosition, policy, section,
                    tubeCenter, selectionStrategy
                );
                result.closedLoopReport = std::move(loop.report);
                if (!loop.traversal.has_value())
                {
                    result.report.failureReason =
                        result.closedLoopReport.failureReason;
                    result.report.failureCode =
                        DiagnosticCode::ProcessPlanningBreakFragmentTraversalInvalid;
                    return result;
                }

                std::vector<std::vector<DirectedEntity>> directions;
                directions.push_back(loop.traversal->entities);
                std::vector<DirectedEntity> reverseDirection;
                reverseDirection.reserve(loop.traversal->entities.size());
                bool reverseDirectionAllowed = true;
                for (auto iterator = loop.traversal->entities.rbegin();
                    iterator != loop.traversal->entities.rend(); ++iterator)
                {
                    DirectedEntity directed = *iterator;
                    directed.reverseRelativeToInput =
                        !directed.reverseRelativeToInput;
                    if (directed.entity == nullptr
                        || !directionAllowed(*directed.entity,
                            directed.reverseRelativeToInput,
                            policy.allowReverse))
                    {
                        reverseDirectionAllowed = false;
                        break;
                    }
                    const auto vertices = orientedVertices(directed);
                    if (vertices.size() < 2U)
                    {
                        reverseDirectionAllowed = false;
                        break;
                    }
                    directed.start = vertices.front().position;
                    directed.end = vertices.back().position;
                    reverseDirection.push_back(std::move(directed));
                }
                if (reverseDirectionAllowed)
                    directions.push_back(std::move(reverseDirection));

                std::optional<Candidate> best;
                QStringList runDescriptions;
                int candidateRunCount = 0;
                bool midpointLocated = false;
                bool fragmentTraversalBuilt = false;
                bool exitResolved = false;
                for (std::size_t directionIndex = 0U;
                    directionIndex < directions.size(); ++directionIndex)
                {
                    const std::vector<DirectedEntity>& cycle =
                        directions[directionIndex];
                    const std::vector<BreakSegment> segments =
                        buildSegments(cycle, section, projectionTolerance);
                    const std::vector<BreakZoneRun> runs =
                        buildRuns(segments);
                    for (const BreakZoneRun& run : runs)
                    {
                        runDescriptions.push_back(describeRun(run));
                        if (!run.strongZone
                            || run.zone != result.report.preferredStartZone)
                        {
                            continue;
                        }
                        ++candidateRunCount;
                        auto candidate = buildCandidate
                        (
                            group.groupId, cycle, segments, run,
                            entities, currentPosition, policy,
                            section, tubeCenter, projectionTolerance,
                            0.5, std::nullopt, std::nullopt, true,
                            directionIndex == 0U
                                ? QStringLiteral("Forward")
                                : QStringLiteral("Reverse"),
                            midpointLocated, fragmentTraversalBuilt,
                            exitResolved
                        );
                        if (!candidate.has_value()) continue;
                        if (!best.has_value()
                            || candidateLess(*candidate, *best,
                                result.report.forcedTopMidpoint,
                                projectionTolerance, selectionStrategy))
                        {
                            best = std::move(candidate);
                        }
                    }
                }
                result.report.candidateRunCount = candidateRunCount;
                result.report.candidateRuns =
                    runDescriptions.join(QLatin1Char(';'));
                if (!best.has_value())
                {
                    if (!midpointLocated)
                    {
                        result.report.failureReason =
                            QStringLiteral("No reliable run in the preferred sweep zone has an interior arc-length midpoint.");
                        result.report.failureCode =
                            DiagnosticCode::ProcessPlanningBreakMidpointCandidateMissing;
                    }
                    else if (!fragmentTraversalBuilt)
                    {
                        result.report.failureReason =
                            QStringLiteral("Break midpoint could not be represented as one continuous complementary fragment traversal.");
                        result.report.failureCode =
                            DiagnosticCode::ProcessPlanningBreakFragmentTraversalInvalid;
                    }
                    else if (!exitResolved)
                    {
                        result.report.failureReason =
                            QStringLiteral("Break final fragment did not resolve to the selected strong start zone.");
                        result.report.failureCode =
                            DiagnosticCode::ProcessPlanningBreakExitZoneUnresolved;
                    }
                    else
                    {
                        result.report.failureReason =
                            QStringLiteral("Break candidate selection failed after all traversal invariants passed.");
                        result.report.failureCode =
                            DiagnosticCode::ProcessPlanningBreakFragmentTraversalInvalid;
                    }
                    return result;
                }

                result.report.startZone = best->zone;
                result.report.selectedRunLength = best->runLength;
                result.report.selectedMaximumShellDeviation =
                    best->maximumShellDeviation;
                result.report.selectedConfidence = best->confidence;
                result.report.selectedMidpoint = best->midpoint;
                result.report.selectedEntityId = best->midpointEntityId;
                result.report.selectedSourceParameter =
                    best->midpointSourceParameter;
                result.report.exitZone = best->zone;
                result.report.exitConfidence = best->exitConfidence;
                result.report.exitReliableLength = best->exitReliableLength;
                result.report.finalEntityId = best->finalEntityId;
                result.report.finalParameterBegin =
                    best->finalParameterBegin;
                result.report.finalParameterEnd =
                    best->finalParameterEnd;
                result.report.exitUsedFallback = best->exitUsedFallback;
                result.report.fragmentCount =
                    static_cast<int>(best->fragments.size());
                result.report.direction = best->direction;
                result.report.fragments = best->fragments;
                result.report.status = QStringLiteral("Success");
                result.report.failureReason.clear();
                result.traversal = std::move(best->traversal);
                result.closedLoopReport.status = QStringLiteral("Success");
                result.closedLoopReport.selectedOrder.clear();
                result.closedLoopReport.selectedReverse.clear();
                for (const DirectedEntity& directed :
                    result.traversal->entities)
                {
                    result.closedLoopReport.selectedOrder.push_back
                        (directed.entity->entityId);
                    result.closedLoopReport.selectedReverse.push_back
                        (directed.reverseRelativeToInput);
                }
                return result;
            }

            struct OrdinaryResult
            {
                std::optional<GroupTraversal> traversal;
                std::optional<GroupTraversal> zoneRunMidpointTraversal;
                int candidateCount = 0;
                int arcInteriorCandidateCount = 0;
                int ellipseInteriorCandidateCount = 0;
                int zoneRunMidpointCandidateCount = 0;
                int curveCandidateRejectedCount = 0;
                int wrongZoneRejectedCount = 0;
                int curveMemberCount = 0;
                int arcTangentRootCount = 0;
                int ellipseTangentRootCount = 0;
                int validTangentCount = 0;
                std::vector<EntityId> arcCandidateEntityIds;
                std::vector<EntityId> ellipseCandidateEntityIds;
            };

            static bool ordinaryCurveInteriorEligible
            (
                const PlanningEntity& entity
            )
            {
                return !entity.path.closed
                    && (entity.sourceKind
                            == geometry::SourceGeometryKind::Arc
                        || entity.sourceKind
                            == geometry::SourceGeometryKind::Ellipse);
            }

            static bool ordinaryZoneRunMidpointEligible
            (
                const PlanningEntity& entity
            )
            {
                if (entity.path.closed) return false;
                switch (entity.sourceKind)
                {
                case geometry::SourceGeometryKind::Line:
                case geometry::SourceGeometryKind::Polyline:
                case geometry::SourceGeometryKind::Spline:
                case geometry::SourceGeometryKind::Arc:
                case geometry::SourceGeometryKind::Ellipse:
                    return true;
                default:
                    return false;
                }
            }

            static std::optional<double> runFactorForParameter
            (
                const BreakZoneRun& run,
                const std::vector<BreakSegment>& segments,
                EntityId entityId,
                double parameter
            )
            {
                double accumulated = 0.0;
                for (const std::size_t segmentIndex : run.segmentIndices)
                {
                    const BreakSegment& segment = segments[segmentIndex];
                    if (segment.entityId == entityId)
                    {
                        const double minimum = std::min
                            (segment.parameterBegin, segment.parameterEnd);
                        const double maximum = std::max
                            (segment.parameterBegin, segment.parameterEnd);
                        if (parameter >= minimum - 1.0e-10
                            && parameter <= maximum + 1.0e-10)
                        {
                            const double denominator =
                                segment.parameterEnd - segment.parameterBegin;
                            if (std::abs(denominator) <= 1.0e-12)
                                return std::nullopt;
                            const double localFactor = std::clamp
                            (
                                (parameter - segment.parameterBegin)
                                    / denominator,
                                0.0,
                                1.0
                            );
                            return std::clamp
                            (
                                (accumulated + segment.length * localFactor)
                                    / run.length,
                                0.0,
                                1.0
                            );
                        }
                    }
                    accumulated += segment.length;
                }
                return std::nullopt;
            }

            static OrdinaryResult buildOrdinary
            (
                const ProcessGroup& group,
                const GroupTraversal& canonicalTraversal,
                const std::unordered_map<EntityId,
                    const PlanningEntity*>& entities,
                const Vector3d& currentPosition,
                const ProcessPlanningPolicy& policy,
                const machining::TubeSectionModel& section,
                const std::optional<Vector2d>& tubeCenter,
                ProcessOrderingStrategy selectionStrategy,
                const TraversalSelectionContext& selection
            )
            {
                OrdinaryResult result;
                if (!selection.hardZoneConstraint
                    || !selection.requiredEntryZone.has_value())
                {
                    return result;
                }

                std::vector<std::vector<DirectedEntity>> directions;
                directions.push_back(canonicalTraversal.entities);
                std::vector<DirectedEntity> reverseDirection;
                reverseDirection.reserve(canonicalTraversal.entities.size());
                bool reverseAllowed = true;
                for (auto iterator = canonicalTraversal.entities.rbegin();
                    iterator != canonicalTraversal.entities.rend(); ++iterator)
                {
                    DirectedEntity directed = *iterator;
                    directed.reverseRelativeToInput =
                        !directed.reverseRelativeToInput;
                    if (directed.entity == nullptr
                        || !directionAllowed(*directed.entity,
                            directed.reverseRelativeToInput,
                            policy.allowReverse)
                        || !manualDirectionAllowed(*directed.entity,
                            directed.reverseRelativeToInput, &selection))
                    {
                        reverseAllowed = false;
                        break;
                    }
                    const auto vertices = orientedVertices(directed);
                    if (vertices.size() < 2U)
                    {
                        reverseAllowed = false;
                        break;
                    }
                    directed.start = vertices.front().position;
                    directed.end = vertices.back().position;
                    reverseDirection.push_back(std::move(directed));
                }
                if (reverseAllowed)
                    directions.push_back(std::move(reverseDirection));

                std::set<EntityId> curveMemberIds;
                for (const DirectedEntity& directed :
                    canonicalTraversal.entities)
                {
                    if (directed.entity != nullptr
                        && ordinaryCurveInteriorEligible(*directed.entity)
                        && directed.entity->sourceEntity.has_value())
                    {
                        curveMemberIds.insert(directed.entity->entityId);
                    }
                }
                result.curveMemberCount =
                    static_cast<int>(curveMemberIds.size());

                for (std::size_t directionIndex = 0U;
                    directionIndex < directions.size(); ++directionIndex)
                {
                    const auto& cycle = directions[directionIndex];
                    if (!std::all_of(cycle.cbegin(), cycle.cend(),
                        [&selection](const DirectedEntity& directed)
                        {
                            return directed.entity != nullptr
                                && manualDirectionAllowed
                                (
                                    *directed.entity,
                                    directed.reverseRelativeToInput,
                                    &selection
                                );
                        }))
                    {
                        continue;
                    }
                    const std::vector<BreakSegment> segments =
                        buildSegments(cycle, section,
                            selection.projectionTolerance);
                    const std::vector<BreakZoneRun> runs =
                        buildRuns(segments);
                    for (const BreakZoneRun& run : runs)
                    {
                        if (!run.strongZone
                            || run.zone != *selection.requiredEntryZone
                            || run.length <= kCalculationEpsilon)
                        {
                            continue;
                        }

                        struct RootCandidate
                        {
                            EntityId entityId = 0;
                            geometry::SourceGeometryKind sourceKind =
                                geometry::SourceGeometryKind::Unknown;
                            double parameter = 0.0;
                            Vector3d point;
                            double residual = 0.0;
                            double runFactor = 0.0;
                        };
                        std::vector<RootCandidate> roots;
                        std::set<EntityId> examinedMembers;
                        for (const std::size_t segmentIndex :
                            run.segmentIndices)
                        {
                            const BreakSegment& segment =
                                segments[segmentIndex];
                            const auto entity = entities.find
                                (segment.entityId);
                            if (entity == entities.end()
                                || entity->second == nullptr
                                || !ordinaryCurveInteriorEligible
                                    (*entity->second)
                                || !entity->second->sourceEntity.has_value()
                                || !examinedMembers.insert
                                    (segment.entityId).second)
                            {
                                continue;
                            }
                            const PlanningEntity& curveEntity =
                                *entity->second;
                            std::vector<ExactCurveTangentRoot>
                                memberRoots;
                            if (curveEntity.sourceKind
                                == geometry::SourceGeometryKind::Arc)
                            {
                                if (const auto* arc =
                                    std::get_if<geometry::ArcGeometry>
                                    (&curveEntity.sourceEntity->geometry))
                                {
                                    memberRoots = arcTangentRoots
                                        (*arc,
                                            selection.previousTransferAnchor,
                                            std::max(1.0e-10,
                                                selection
                                                    .projectionTolerance
                                                    * 1.0e-6));
                                }
                            }
                            else if (curveEntity.sourceKind
                                == geometry::SourceGeometryKind::Ellipse)
                            {
                                if (const auto* ellipse =
                                    std::get_if<geometry::EllipseGeometry>
                                    (&curveEntity.sourceEntity->geometry))
                                {
                                    memberRoots = ellipseTangentRoots
                                        (*ellipse,
                                            selection.previousTransferAnchor,
                                            std::max(1.0e-10,
                                                selection
                                                    .projectionTolerance
                                                    * 1.0e-6));
                                }
                            }
                            if (directionIndex == 0U)
                            {
                                if (curveEntity.sourceKind
                                    == geometry::SourceGeometryKind::Arc)
                                {
                                    result.arcTangentRootCount +=
                                        static_cast<int>(memberRoots.size());
                                }
                                else
                                {
                                    result.ellipseTangentRootCount +=
                                        static_cast<int>(memberRoots.size());
                                }
                            }
                            for (const auto& root : memberRoots)
                            {
                                const auto runFactor =
                                    runFactorForParameter
                                    (
                                        run, segments,
                                        curveEntity.entityId,
                                        root.parameter
                                    );
                                if (!runFactor.has_value()
                                    || *runFactor <= kCalculationEpsilon
                                    || *runFactor
                                        >= 1.0 - kCalculationEpsilon)
                                {
                                    continue;
                                }
                                roots.push_back
                                ({
                                    curveEntity.entityId,
                                    curveEntity.sourceKind,
                                    root.parameter,
                                    root.point,
                                    root.residual,
                                    *runFactor
                                });
                            }
                        }

                        for (const RootCandidate& root : roots)
                        {
                            bool midpointLocated = false;
                            bool fragmentsBuilt = false;
                            bool exitResolved = false;
                            auto candidate = buildCandidate
                            (
                                group.groupId, cycle, segments, run,
                                entities, currentPosition, policy, section,
                                tubeCenter, selection.projectionTolerance,
                                root.runFactor, root.point, root.parameter,
                                false,
                                directionIndex == 0U
                                    ? QStringLiteral("Forward")
                                    : QStringLiteral("Reverse"),
                                midpointLocated, fragmentsBuilt, exitResolved
                            );
                            if (!candidate.has_value()
                                || !fragmentsBuilt)
                            {
                                ++result.curveCandidateRejectedCount;
                                continue;
                            }
                            const auto entryEntityFound = entities.find
                                (candidate->midpointEntityId);
                            if (entryEntityFound == entities.end()
                                || entryEntityFound->second == nullptr
                                || !ordinaryCurveInteriorEligible
                                    (*entryEntityFound->second))
                            {
                                ++result.curveCandidateRejectedCount;
                                continue;
                            }
                            const PlanningEntity& entryEntity =
                                *entryEntityFound->second;
                            const double endpointTolerance = std::max
                            (
                                1.0e-8,
                                selection.projectionTolerance * 1.0e-3
                            );
                            const double distanceToMemberEndpoint = std::min
                            (
                                distance(candidate->midpoint,
                                    entryEntity.path.vertices.front()
                                        .position),
                                distance(candidate->midpoint,
                                    entryEntity.path.vertices.back()
                                        .position)
                            );
                            if (distanceToMemberEndpoint
                                <= endpointTolerance)
                            {
                                ++result.curveCandidateRejectedCount;
                                continue;
                            }
                            const ZoneEntryCandidateKind candidateKind =
                                entryEntity.sourceKind
                                    == geometry::SourceGeometryKind::Arc
                                ? ZoneEntryCandidateKind::
                                    ClosedLoopArcInterior
                                : ZoneEntryCandidateKind::
                                    ClosedLoopEllipseInterior;
                            auto entry = classifyZoneEntry
                            (
                                candidateKind,
                                candidate->midpointEntityId,
                                candidate->midpointSourceParameter,
                                candidate->traversal.entities.front()
                                    .reverseRelativeToInput,
                                candidate->midpoint,
                                candidate->firstCutPoint,
                                entryEntity.sourceKind,
                                section,
                                selection.projectionTolerance
                            );
                            if (!entry.has_value()
                                || entry->zone
                                    != *selection.requiredEntryZone)
                            {
                                ++result.wrongZoneRejectedCount;
                                ++result.curveCandidateRejectedCount;
                                continue;
                            }
                            entry->distanceToMemberEndpoint =
                                distanceToMemberEndpoint;
                            if (entry->distanceToZoneBoundary
                                <= endpointTolerance)
                            {
                                ++result.curveCandidateRejectedCount;
                                continue;
                            }

                            const PlanningEntity* traversalEntryEntity =
                                candidate->traversal.entities.front().entity;
                            if (traversalEntryEntity == nullptr)
                            {
                                ++result.curveCandidateRejectedCount;
                                continue;
                            }
                            const bool manualStartMatches =
                                !traversalEntryEntity
                                    ->manualStartParameter.has_value()
                                || std::abs(*traversalEntryEntity
                                    ->manualStartParameter
                                    - candidate->midpointSourceParameter)
                                    <= 1.0e-10;
                            const bool otherManualStartExists = std::any_of
                            (
                                candidate->traversal.entities.cbegin() + 1,
                                candidate->traversal.entities.cend(),
                                [](const DirectedEntity& directed)
                                {
                                    return directed.entity != nullptr
                                        && directed.entity
                                            ->manualStartParameter.has_value();
                                }
                            );
                            if (!manualStartMatches
                                || otherManualStartExists)
                            {
                                ++result.wrongZoneRejectedCount;
                                ++result.curveCandidateRejectedCount;
                                continue;
                            }

                            candidate->traversal.selectedEntry =
                                std::move(entry);
                            candidate->traversal.fragments =
                                candidate->fragments;
                            scoreUnwrappedEntry
                            (
                                candidate->traversal.entities.front(),
                                selection.previousTransferAnchor,
                                candidate->firstCutPoint, section,
                                policy.connectionTolerance,
                                selection.projectionTolerance
                            );
                            candidate->traversal.entryAxisReversalCount =
                                candidate->traversal.entities.front()
                                    .entryAxisReversalCount;
                            candidate->traversal.entryTangentCost =
                                candidate->traversal.entities.front()
                                    .entryTangentCost;
                            candidate->traversal.entryRefinementMode =
                                QStringLiteral("ExactCurveTangent");
                            candidate->traversal.previousCutEnd =
                                selection.previousCutEnd;
                            candidate->traversal.previousTransferAnchor =
                                selection.previousTransferAnchor;
                            candidate->traversal.entryTravelDistance =
                                distance(selection.previousTransferAnchor,
                                    candidate->midpoint);
                            candidate->traversal.approachCutDot =
                                std::clamp
                                (
                                    1.0 - candidate->traversal
                                        .entryTangentCost,
                                    -1.0,
                                    1.0
                                );
                            candidate->traversal.approachCutAngle =
                                std::acos(candidate->traversal
                                    .approachCutDot)
                                * 180.0 / 3.14159265358979323846;
                            candidate->traversal.tangentResidual =
                                root.residual;
                            scoreTraversal(candidate->traversal,
                                currentPosition, section);
                            ++result.candidateCount;
                            ++result.validTangentCount;
                            if (entryEntity.sourceKind
                                == geometry::SourceGeometryKind::Arc)
                            {
                                ++result.arcInteriorCandidateCount;
                                result.arcCandidateEntityIds.push_back
                                    (entryEntity.entityId);
                            }
                            else
                            {
                                ++result.ellipseInteriorCandidateCount;
                                result.ellipseCandidateEntityIds.push_back
                                    (entryEntity.entityId);
                            }
                            const auto tangentLess =
                                [](const GroupTraversal& left,
                                    const GroupTraversal& right)
                            {
                                const bool leftSameDirection =
                                    left.approachCutDot > 0.0;
                                const bool rightSameDirection =
                                    right.approachCutDot > 0.0;
                                if (leftSameDirection != rightSameDirection)
                                    return leftSameDirection;
                                if (std::abs(left.entryTravelDistance
                                    - right.entryTravelDistance)
                                    > kCalculationEpsilon)
                                {
                                    return left.entryTravelDistance
                                        < right.entryTravelDistance;
                                }
                                if (left.entryAxisReversalCount
                                    != right.entryAxisReversalCount)
                                {
                                    return left.entryAxisReversalCount
                                        < right.entryAxisReversalCount;
                                }
                                if (std::abs(left.rotationCost
                                    - right.rotationCost)
                                    > kCalculationEpsilon)
                                {
                                    return left.rotationCost
                                        < right.rotationCost;
                                }
                                if (left.stableEntityId
                                    != right.stableEntityId)
                                {
                                    return left.stableEntityId
                                        < right.stableEntityId;
                                }
                                const double leftParameter =
                                    left.selectedEntry
                                        ->sourceParameter.value_or(0.0);
                                const double rightParameter =
                                    right.selectedEntry
                                        ->sourceParameter.value_or(0.0);
                                if (std::abs(leftParameter - rightParameter)
                                    > 1.0e-12)
                                {
                                    return leftParameter < rightParameter;
                                }
                                return left.entities.front()
                                    .reverseRelativeToInput
                                    < right.entities.front()
                                        .reverseRelativeToInput;
                            };
                            if (!result.traversal.has_value()
                                || tangentLess(candidate->traversal,
                                    *result.traversal))
                            {
                                result.traversal =
                                    std::move(candidate->traversal);
                            }
                        }
                    }
                }
                if (selection.allowZoneRunMidpointFallback)
                {
                    std::optional<Candidate> bestMidpoint;
                    for (std::size_t directionIndex = 0U;
                        directionIndex < directions.size(); ++directionIndex)
                    {
                        const auto& cycle = directions[directionIndex];
                        const std::vector<BreakSegment> segments =
                            buildSegments(cycle, section,
                                selection.projectionTolerance);
                        const std::vector<BreakZoneRun> runs =
                            buildRuns(segments);
                        for (const BreakZoneRun& run : runs)
                        {
                            if (!run.strongZone || run.touchesBoundary
                                || run.zone
                                    != *selection.requiredEntryZone
                                || run.length <= kCalculationEpsilon)
                            {
                                continue;
                            }
                            bool midpointLocated = false;
                            bool fragmentsBuilt = false;
                            bool exitResolved = false;
                            auto candidate = buildCandidate
                            (
                                group.groupId, cycle, segments, run,
                                entities, currentPosition, policy, section,
                                tubeCenter, selection.projectionTolerance,
                                0.5, std::nullopt, std::nullopt, false,
                                directionIndex == 0U
                                    ? QStringLiteral("Forward")
                                    : QStringLiteral("Reverse"),
                                midpointLocated, fragmentsBuilt, exitResolved
                            );
                            if (!candidate.has_value() || !midpointLocated
                                || !fragmentsBuilt)
                            {
                                continue;
                            }
                            const auto entryEntityFound = entities.find
                                (candidate->midpointEntityId);
                            if (entryEntityFound == entities.end()
                                || entryEntityFound->second == nullptr
                                || !ordinaryZoneRunMidpointEligible
                                    (*entryEntityFound->second))
                            {
                                continue;
                            }
                            const PlanningEntity& entryEntity =
                                *entryEntityFound->second;
                            const double endpointTolerance = std::max
                            (
                                1.0e-8,
                                selection.projectionTolerance * 1.0e-3
                            );
                            const double distanceToMemberEndpoint = std::min
                            (
                                distance(candidate->midpoint,
                                    entryEntity.path.vertices.front()
                                        .position),
                                distance(candidate->midpoint,
                                    entryEntity.path.vertices.back()
                                        .position)
                            );
                            if (distanceToMemberEndpoint
                                <= endpointTolerance)
                            {
                                continue;
                            }
                            auto entry = classifyZoneEntry
                            (
                                ZoneEntryCandidateKind::
                                    ClosedLoopZoneRunMidpoint,
                                candidate->midpointEntityId,
                                candidate->midpointSourceParameter,
                                candidate->traversal.entities.front()
                                    .reverseRelativeToInput,
                                candidate->midpoint,
                                candidate->firstCutPoint,
                                entryEntity.sourceKind,
                                section,
                                selection.projectionTolerance
                            );
                            if (!entry.has_value() || entry->ambiguous
                                || entry->zone
                                    != *selection.requiredEntryZone
                                || entry->distanceToZoneBoundary
                                    <= endpointTolerance)
                            {
                                continue;
                            }
                            const PlanningEntity* traversalEntryEntity =
                                candidate->traversal.entities.front().entity;
                            if (traversalEntryEntity == nullptr)
                                continue;
                            const bool manualStartMatches =
                                !traversalEntryEntity
                                    ->manualStartParameter.has_value()
                                || std::abs(*traversalEntryEntity
                                    ->manualStartParameter
                                    - candidate->midpointSourceParameter)
                                    <= 1.0e-10;
                            const bool otherManualStartExists = std::any_of
                            (
                                candidate->traversal.entities.cbegin() + 1,
                                candidate->traversal.entities.cend(),
                                [](const DirectedEntity& directed)
                                {
                                    return directed.entity != nullptr
                                        && directed.entity
                                            ->manualStartParameter.has_value();
                                }
                            );
                            if (!manualStartMatches
                                || otherManualStartExists)
                            {
                                continue;
                            }
                            entry->distanceToMemberEndpoint =
                                distanceToMemberEndpoint;
                            candidate->traversal.selectedEntry =
                                std::move(entry);
                            candidate->traversal.fragments =
                                candidate->fragments;
                            scoreUnwrappedEntry
                            (
                                candidate->traversal.entities.front(),
                                selection.previousTransferAnchor,
                                candidate->firstCutPoint, section,
                                policy.connectionTolerance,
                                selection.projectionTolerance
                            );
                            candidate->traversal.entryAxisReversalCount =
                                candidate->traversal.entities.front()
                                    .entryAxisReversalCount;
                            candidate->traversal.entryTangentCost =
                                candidate->traversal.entities.front()
                                    .entryTangentCost;
                            candidate->traversal.entryRefinementMode =
                                QStringLiteral("ZoneRunMidpointFallback");
                            candidate->traversal.previousCutEnd =
                                selection.previousCutEnd;
                            candidate->traversal.previousTransferAnchor =
                                selection.previousTransferAnchor;
                            candidate->traversal.entryTravelDistance =
                                distance(selection.previousTransferAnchor,
                                    candidate->midpoint);
                            candidate->traversal.approachCutDot =
                                std::clamp
                                (
                                    1.0 - candidate->traversal
                                        .entryTangentCost,
                                    -1.0,
                                    1.0
                                );
                            candidate->traversal.approachCutAngle =
                                std::acos(candidate->traversal
                                    .approachCutDot)
                                * 180.0 / 3.14159265358979323846;
                            scoreTraversal(candidate->traversal,
                                currentPosition, section);
                            ++result.zoneRunMidpointCandidateCount;
                            const double lengthTolerance = std::max
                                (1.0e-9,
                                    selection.projectionTolerance * 1.0e-3);
                            const auto midpointLess =
                                [lengthTolerance](const Candidate& left,
                                    const Candidate& right)
                            {
                                if (std::abs(left.runLength
                                    - right.runLength) > lengthTolerance)
                                {
                                    return left.runLength > right.runLength;
                                }
                                if (std::abs(left.maximumShellDeviation
                                    - right.maximumShellDeviation)
                                    > kCalculationEpsilon)
                                {
                                    return left.maximumShellDeviation
                                        < right.maximumShellDeviation;
                                }
                                if (left.midpointEntityId
                                    != right.midpointEntityId)
                                {
                                    return left.midpointEntityId
                                        < right.midpointEntityId;
                                }
                                if (left.midpointSourceParameter
                                    != right.midpointSourceParameter)
                                {
                                    return left.midpointSourceParameter
                                        < right.midpointSourceParameter;
                                }
                                return left.direction < right.direction;
                            };
                            if (!bestMidpoint.has_value()
                                || midpointLess(*candidate, *bestMidpoint))
                            {
                                bestMidpoint = std::move(candidate);
                            }
                        }
                    }
                    if (bestMidpoint.has_value())
                    {
                        bestMidpoint->traversal.entryCandidateCount =
                            result.zoneRunMidpointCandidateCount;
                        bestMidpoint->traversal
                            .zoneRunMidpointCandidateCount =
                            result.zoneRunMidpointCandidateCount;
                        result.zoneRunMidpointTraversal =
                            std::move(bestMidpoint->traversal);
                    }
                }
                if (result.traversal.has_value())
                {
                    std::sort(result.arcCandidateEntityIds.begin(),
                        result.arcCandidateEntityIds.end());
                    result.arcCandidateEntityIds.erase(std::unique
                    (
                        result.arcCandidateEntityIds.begin(),
                        result.arcCandidateEntityIds.end()
                    ), result.arcCandidateEntityIds.end());
                    std::sort(result.ellipseCandidateEntityIds.begin(),
                        result.ellipseCandidateEntityIds.end());
                    result.ellipseCandidateEntityIds.erase(std::unique
                    (
                        result.ellipseCandidateEntityIds.begin(),
                        result.ellipseCandidateEntityIds.end()
                    ), result.ellipseCandidateEntityIds.end());
                    result.traversal->entryCandidateCount =
                        result.candidateCount;
                    result.traversal->arcInteriorCandidateCount =
                        result.arcInteriorCandidateCount;
                    result.traversal->ellipseInteriorCandidateCount =
                        result.ellipseInteriorCandidateCount;
                    result.traversal->curveCandidateRejectedCount =
                        result.curveCandidateRejectedCount;
                    result.traversal->arcInteriorCandidateEntityIds =
                        result.arcCandidateEntityIds;
                    result.traversal->ellipseInteriorCandidateEntityIds =
                        result.ellipseCandidateEntityIds;
                    result.traversal->wrongZoneRejectedCount =
                        result.wrongZoneRejectedCount;
                    result.traversal->curveMemberCount =
                        result.curveMemberCount;
                    result.traversal->arcTangentRootCount =
                        result.arcTangentRootCount;
                    result.traversal->ellipseTangentRootCount =
                        result.ellipseTangentRootCount;
                    result.traversal->validTangentCount =
                        result.validTangentCount;
                }
                if (result.zoneRunMidpointTraversal.has_value())
                {
                    result.zoneRunMidpointTraversal->curveMemberCount =
                        result.curveMemberCount;
                    result.zoneRunMidpointTraversal->arcTangentRootCount =
                        result.arcTangentRootCount;
                    result.zoneRunMidpointTraversal
                        ->ellipseTangentRootCount =
                        result.ellipseTangentRootCount;
                    result.zoneRunMidpointTraversal->validTangentCount =
                        result.validTangentCount;
                }
                return result;
            }

        public:
            struct BreakSegment
            {
                std::size_t entityOrder = 0U;
                std::size_t segmentOrder = 0U;
                EntityId entityId = 0;
                std::size_t sourceIndex = 0U;
                Vector3d start;
                Vector3d end;
                double parameterBegin = 0.0;
                double parameterEnd = 0.0;
                double length = 0.0;
                std::optional<machining::TubeZone16> strongZone;
                std::optional<machining::TubeZone16> possibleZone;
                double maximumShellDeviation = 0.0;
                double confidence = 0.0;
                bool touchesBoundary = false;
            };

            struct BreakZoneRun
            {
                machining::TubeZone16 zone =
                    machining::TubeZone16::TopFace;
                std::vector<std::size_t> segmentIndices;
                double length = 0.0;
                double maximumShellDeviation = 0.0;
                double confidence = 0.0;
                bool strongZone = false;
                bool touchesBoundary = false;
            };

            struct FragmentLocation
            {
                std::size_t segmentIndex = 0U;
                geometry::PathVertex3D vertex;
            };

            struct ExitResult
            {
                std::optional<machining::TubeZone16> zone;
                double confidence = 0.0;
                double reliableLength = 0.0;
                EntityId finalEntityId = 0;
                double finalParameterBegin = 0.0;
                double finalParameterEnd = 0.0;
                bool usedFallback = false;
            };

            struct Candidate
            {
                GroupTraversal traversal;
                std::vector<ProcessPathFragment> fragments;
                machining::TubeZone16 zone =
                    machining::TubeZone16::TopFace;
                double runLength = 0.0;
                double maximumShellDeviation = 0.0;
                double confidence = 0.0;
                Vector3d midpoint;
                EntityId midpointEntityId = 0;
                std::size_t midpointSourceIndex = 0;
                double midpointSourceParameter = 0.0;
                Vector3d firstCutPoint;
                double exitConfidence = 0.0;
                double exitReliableLength = 0.0;
                EntityId finalEntityId = 0;
                double finalParameterBegin = 0.0;
                double finalParameterEnd = 0.0;
                bool exitUsedFallback = false;
                QString direction;
            };

            static bool finite(const Vector3d& value)
            {
                return std::isfinite(value.x) && std::isfinite(value.y)
                    && std::isfinite(value.z);
            }

            static Vector3d interpolate
            (
                const Vector3d& start,
                const Vector3d& end,
                double factor
            )
            {
                return
                {
                    start.x + (end.x - start.x) * factor,
                    start.y + (end.y - start.y) * factor,
                    start.z + (end.z - start.z) * factor
                };
            }

            static std::vector<geometry::PathVertex3D> orientedVertices
            (
                const DirectedEntity& directed
            )
            {
                if (directed.entity == nullptr) return {};
                std::vector<geometry::PathVertex3D> vertices =
                    directed.entity->path.vertices;
                if (directed.reverseRelativeToInput)
                    std::reverse(vertices.begin(), vertices.end());
                return vertices;
            }

            static bool strongZone(machining::TubeZone16 zone)
            {
                return machining::tubeZoneIndex(zone) % 2U == 0U;
            }

            static BreakSegment classifySegment
            (
                const DirectedEntity& directed,
                std::size_t entityOrder,
                std::size_t segmentOrder,
                const geometry::PathVertex3D& start,
                const geometry::PathVertex3D& end,
                const machining::TubeSectionModel& section,
                double projectionTolerance
            )
            {
                BreakSegment segment;
                segment.entityOrder = entityOrder;
                segment.segmentOrder = segmentOrder;
                segment.entityId = directed.entity->entityId;
                segment.sourceIndex = directed.entity->sourceIndex;
                segment.start = start.position;
                segment.end = end.position;
                segment.parameterBegin = start.sourceParameter;
                segment.parameterEnd = end.sourceParameter;
                segment.length = distance(segment.start, segment.end);
                segment.maximumShellDeviation = 0.0;
                segment.confidence = 1.0;
                std::array<int, machining::kTubeZone16Count> possibleCounts{};
                std::optional<machining::TubeZone16> reliableZone;
                bool allReliable = segment.length > kCalculationEpsilon;
                for (const double factor : { 0.25, 0.5, 0.75 })
                {
                    const Vector3d point =
                        interpolate(segment.start, segment.end, factor);
                    const machining::TubeSectionProjection projection =
                        machining::TubeSectionProjector::project
                        (
                            section, { point.y, point.z },
                            projectionTolerance
                        );
                    if (projection.valid)
                    {
                        ++possibleCounts[machining::tubeZoneIndex
                            (projection.zone)];
                    }
                    segment.maximumShellDeviation = std::max
                        (segment.maximumShellDeviation,
                            projection.absoluteDistanceToShell);
                    segment.confidence = std::min
                        (segment.confidence, projection.confidence);
                    segment.touchesBoundary = segment.touchesBoundary
                        || projection.onBoundary || projection.ambiguous
                        || !projection.valid
                        || !strongZone(projection.zone);
                    if (!projection.valid || projection.ambiguous
                        || projection.onBoundary
                        || !strongZone(projection.zone)
                        || projection.confidence < 0.5
                        || projection.absoluteDistanceToShell
                            > projectionTolerance * 0.8)
                    {
                        allReliable = false;
                        continue;
                    }
                    if (!reliableZone.has_value())
                        reliableZone = projection.zone;
                    else if (*reliableZone != projection.zone)
                        allReliable = false;
                }
                int bestCount = 0;
                bool tied = false;
                for (std::size_t index = 0U;
                    index < possibleCounts.size(); ++index)
                {
                    if (possibleCounts[index] > bestCount)
                    {
                        bestCount = possibleCounts[index];
                        segment.possibleZone =
                            static_cast<machining::TubeZone16>(index);
                        tied = false;
                    }
                    else if (possibleCounts[index] > 0
                        && possibleCounts[index] == bestCount)
                    {
                        tied = true;
                    }
                }
                if (tied) segment.possibleZone.reset();
                if (allReliable && reliableZone.has_value())
                    segment.strongZone = reliableZone;
                return segment;
            }

            static std::vector<BreakSegment> buildSegments
            (
                const std::vector<DirectedEntity>& cycle,
                const machining::TubeSectionModel& section,
                double projectionTolerance
            )
            {
                std::vector<BreakSegment> segments;
                for (std::size_t entityOrder = 0U;
                    entityOrder < cycle.size(); ++entityOrder)
                {
                    const DirectedEntity& directed = cycle[entityOrder];
                    const auto vertices = orientedVertices(directed);
                    for (std::size_t index = 1U;
                        index < vertices.size(); ++index)
                    {
                        if (!finite(vertices[index - 1U].position)
                            || !finite(vertices[index].position)
                            || distance(vertices[index - 1U].position,
                                vertices[index].position)
                                <= kCalculationEpsilon)
                        {
                            continue;
                        }
                        segments.push_back(classifySegment
                        (
                            directed, entityOrder, index - 1U,
                            vertices[index - 1U], vertices[index],
                            section, projectionTolerance
                        ));
                    }
                }
                return segments;
            }

            static void addSegmentToRun
            (
                BreakZoneRun& run,
                const BreakSegment& segment,
                std::size_t segmentIndex
            )
            {
                if (run.segmentIndices.empty())
                {
                    run.zone = *segment.strongZone;
                    run.confidence = segment.confidence;
                }
                run.segmentIndices.push_back(segmentIndex);
                run.length += segment.length;
                run.maximumShellDeviation = std::max
                    (run.maximumShellDeviation,
                        segment.maximumShellDeviation);
                run.confidence = std::min
                    (run.confidence, segment.confidence);
                run.strongZone = true;
                run.touchesBoundary = run.touchesBoundary
                    || segment.touchesBoundary;
            }

            static std::vector<BreakZoneRun> buildRuns
            (
                const std::vector<BreakSegment>& segments
            )
            {
                std::vector<BreakZoneRun> runs;
                for (std::size_t index = 0U; index < segments.size(); ++index)
                {
                    const BreakSegment& segment = segments[index];
                    if (!segment.strongZone.has_value())
                        continue;
                    if (runs.empty() || runs.back().zone != *segment.strongZone
                        || (!runs.back().segmentIndices.empty()
                            && runs.back().segmentIndices.back() + 1U != index))
                    {
                        runs.emplace_back();
                    }
                    addSegmentToRun(runs.back(), segment, index);
                }
                if (runs.size() > 1U
                    && !runs.front().segmentIndices.empty()
                    && !runs.back().segmentIndices.empty()
                    && runs.front().segmentIndices.front() == 0U
                    && runs.back().segmentIndices.back() + 1U
                        == segments.size()
                    && runs.front().zone == runs.back().zone)
                {
                    BreakZoneRun combined = runs.back();
                    for (const std::size_t index :
                        runs.front().segmentIndices)
                    {
                        addSegmentToRun(combined, segments[index], index);
                    }
                    runs.front() = std::move(combined);
                    runs.pop_back();
                }
                return runs;
            }

            static QString describeRun(const BreakZoneRun& run)
            {
                return QStringLiteral("%1|length=%2|deviation=%3|confidence=%4|strong=%5|boundary=%6")
                    .arg(machining::tubeZoneName(run.zone))
                    .arg(run.length, 0, 'g', 15)
                    .arg(run.maximumShellDeviation, 0, 'g', 15)
                    .arg(run.confidence, 0, 'g', 15)
                    .arg(run.strongZone ? 1 : 0)
                    .arg(run.touchesBoundary ? 1 : 0);
            }

            static std::optional<FragmentLocation> locateParameter
            (
                const std::vector<geometry::PathVertex3D>& vertices,
                double parameter
            )
            {
                if (vertices.size() < 2U || !std::isfinite(parameter))
                    return std::nullopt;
                const double epsilon = 1.0e-10;
                for (std::size_t index = 1U; index < vertices.size(); ++index)
                {
                    const double left = vertices[index - 1U].sourceParameter;
                    const double right = vertices[index].sourceParameter;
                    const double minimum = std::min(left, right) - epsilon;
                    const double maximum = std::max(left, right) + epsilon;
                    if (parameter < minimum || parameter > maximum)
                        continue;
                    const double denominator = right - left;
                    const double factor = std::abs(denominator) <= epsilon
                        ? 0.0 : std::clamp
                            ((parameter - left) / denominator, 0.0, 1.0);
                    FragmentLocation location;
                    location.segmentIndex = index - 1U;
                    location.vertex.position = interpolate
                        (vertices[index - 1U].position,
                            vertices[index].position, factor);
                    location.vertex.sourceParameter = parameter;
                    return location;
                }
                return std::nullopt;
            }

            static std::optional<std::vector<geometry::PathVertex3D>>
                fragmentVertices
            (
                const PlanningEntity& entity,
                double parameterBegin,
                double parameterEnd,
                bool reverse
            )
            {
                DirectedEntity directed;
                directed.entity = &entity;
                directed.reverseRelativeToInput = reverse;
                const auto vertices = orientedVertices(directed);
                const auto begin = locateParameter(vertices, parameterBegin);
                const auto end = locateParameter(vertices, parameterEnd);
                if (!begin.has_value() || !end.has_value()
                    || begin->segmentIndex > end->segmentIndex)
                {
                    return std::nullopt;
                }
                std::vector<geometry::PathVertex3D> result;
                result.push_back(begin->vertex);
                for (std::size_t index = begin->segmentIndex + 1U;
                    index <= end->segmentIndex && index < vertices.size();
                    ++index)
                {
                    if (distance(result.back().position,
                        vertices[index].position) > kCalculationEpsilon)
                    {
                        result.push_back(vertices[index]);
                    }
                }
                if (distance(result.back().position, end->vertex.position)
                    > kCalculationEpsilon)
                {
                    result.push_back(end->vertex);
                }
                else
                {
                    result.back() = end->vertex;
                }
                if (result.size() < 2U
                    || distance(result.front().position,
                        result.back().position) <= kCalculationEpsilon)
                {
                    return std::nullopt;
                }
                return result;
            }

            static double pathLength
            (
                const std::vector<geometry::PathVertex3D>& vertices
            )
            {
                double length = 0.0;
                for (std::size_t index = 1U;
                    index < vertices.size(); ++index)
                {
                    length += distance(vertices[index - 1U].position,
                        vertices[index].position);
                }
                return length;
            }

            static ExitResult resolveExit
            (
                const std::vector<ProcessPathFragment>& fragments,
                const std::unordered_map<EntityId,
                    const PlanningEntity*>& entities,
                const machining::TubeSectionModel& section,
                double projectionTolerance,
                machining::TubeZone16 startZone
            )
            {
                ExitResult result;
                std::optional<machining::TubeZone16> fallbackZone;
                double fallbackConfidence = 0.0;
                bool reliableZoneFound = false;
                for (auto iterator = fragments.rbegin();
                    iterator != fragments.rend(); ++iterator)
                {
                    const auto entity = entities.find(iterator->entityId);
                    if (entity == entities.end() || entity->second == nullptr)
                        continue;
                    const auto vertices = fragmentVertices
                    (
                        *entity->second, iterator->sourceParameterBegin,
                        iterator->sourceParameterEnd, iterator->reverse
                    );
                    if (!vertices.has_value()) continue;
                    for (std::size_t offset = 1U;
                        offset < vertices->size(); ++offset)
                    {
                        const auto& end =
                            (*vertices)[vertices->size() - offset];
                        const auto& begin =
                            (*vertices)[vertices->size() - offset - 1U];
                        const double segmentLength =
                            distance(begin.position, end.position);
                        if (segmentLength <= kCalculationEpsilon) continue;
                        DirectedEntity directed;
                        directed.entity = entity->second;
                        const BreakSegment classified = classifySegment
                        (
                            directed, 0U, 0U, begin, end,
                            section, projectionTolerance
                        );
                        if (classified.strongZone.has_value())
                        {
                            if (*classified.strongZone != startZone)
                                return reliableZoneFound
                                    ? result : ExitResult{};
                            if (!reliableZoneFound)
                            {
                                result.zone = *classified.strongZone;
                                result.confidence =
                                    classified.confidence;
                                result.finalEntityId =
                                    iterator->entityId;
                                result.finalParameterBegin =
                                    iterator->sourceParameterBegin;
                                result.finalParameterEnd =
                                    iterator->sourceParameterEnd;
                                reliableZoneFound = true;
                            }
                            else
                            {
                                result.confidence = std::min
                                    (result.confidence,
                                        classified.confidence);
                            }
                            result.reliableLength += segmentLength;
                            continue;
                        }
                        if (reliableZoneFound
                            && segmentLength > projectionTolerance)
                        {
                            return result;
                        }
                        if (!fallbackZone.has_value()
                            && classified.possibleZone.has_value()
                            && strongZone(*classified.possibleZone))
                        {
                            fallbackZone = classified.possibleZone;
                            fallbackConfidence = classified.confidence;
                        }
                    }
                }
                if (reliableZoneFound) return result;
                if (fallbackZone.has_value() && *fallbackZone == startZone)
                {
                    result.zone = fallbackZone;
                    result.confidence = fallbackConfidence;
                    result.usedFallback = true;
                    result.finalEntityId = fragments.empty()
                        ? 0U : fragments.back().entityId;
                    if (!fragments.empty())
                    {
                        result.finalParameterBegin =
                            fragments.back().sourceParameterBegin;
                        result.finalParameterEnd =
                            fragments.back().sourceParameterEnd;
                    }
                }
                return result;
            }

            static std::optional<Candidate> buildCandidate
            (
                int groupId,
                const std::vector<DirectedEntity>& cycle,
                const std::vector<BreakSegment>& segments,
                const BreakZoneRun& run,
                const std::unordered_map<EntityId,
                    const PlanningEntity*>& entities,
                const Vector3d& currentPosition,
                const ProcessPlanningPolicy& policy,
                const machining::TubeSectionModel& section,
                const std::optional<Vector2d>& tubeCenter,
                double projectionTolerance,
                double runFactor,
                std::optional<Vector3d> exactPoint,
                std::optional<double> exactParameter,
                bool requireMatchingExit,
                const QString& direction,
                bool& midpointLocated,
                bool& fragmentTraversalBuilt,
                bool& exitResolved
            )
            {
                if (run.segmentIndices.empty()
                    || run.length <= kCalculationEpsilon)
                {
                    return std::nullopt;
                }
                const double target = run.length * std::clamp
                    (runFactor, 0.0, 1.0);
                double accumulated = 0.0;
                const BreakSegment* midpointSegment = nullptr;
                double midpointFactor = 0.0;
                for (const std::size_t index : run.segmentIndices)
                {
                    const BreakSegment& segment = segments[index];
                    if (accumulated + segment.length
                        >= target - kCalculationEpsilon)
                    {
                        midpointSegment = &segment;
                        midpointFactor = std::clamp
                            ((target - accumulated) / segment.length,
                                0.0, 1.0);
                        break;
                    }
                    accumulated += segment.length;
                }
                if (midpointSegment == nullptr
                    || midpointFactor <= kCalculationEpsilon
                    || midpointFactor >= 1.0 - kCalculationEpsilon)
                {
                    return std::nullopt;
                }
                midpointLocated = true;

                const Vector3d midpoint = exactPoint.value_or(interpolate
                    (midpointSegment->start, midpointSegment->end,
                        midpointFactor));
                const double midpointParameter = exactParameter.value_or
                (
                    midpointSegment->parameterBegin
                    + (midpointSegment->parameterEnd
                        - midpointSegment->parameterBegin)
                        * midpointFactor
                );
                const std::size_t startEntityOrder =
                    midpointSegment->entityOrder;

                Candidate candidate;
                candidate.zone = run.zone;
                candidate.runLength = run.length;
                candidate.maximumShellDeviation =
                    run.maximumShellDeviation;
                candidate.confidence = run.confidence;
                candidate.midpoint = midpoint;
                candidate.midpointEntityId =
                    midpointSegment->entityId;
                candidate.midpointSourceIndex =
                    midpointSegment->sourceIndex;
                candidate.midpointSourceParameter =
                    midpointParameter;
                candidate.firstCutPoint = midpointSegment->end;
                candidate.direction = direction;
                candidate.traversal.groupId = groupId;

                std::vector<DirectedEntity> rotated;
                rotated.reserve(cycle.size());
                for (std::size_t offset = 0U;
                    offset < cycle.size(); ++offset)
                {
                    DirectedEntity directed =
                        cycle[(startEntityOrder + offset) % cycle.size()];
                    const auto vertices = orientedVertices(directed);
                    if (vertices.size() < 2U) return std::nullopt;
                    directed.start = vertices.front().position;
                    directed.end = vertices.back().position;
                    if (offset == 0U)
                    {
                        directed.start = midpoint;
                        directed.selectedStartParameter =
                            midpointParameter;
                    }
                    rotated.push_back(std::move(directed));
                }
                candidate.traversal.entities = rotated;
                candidate.traversal.start = midpoint;
                candidate.traversal.end = midpoint;

                const auto firstVertices =
                    orientedVertices(rotated.front());
                if (firstVertices.size() < 2U) return std::nullopt;
                ProcessPathFragment first;
                first.entityId = rotated.front().entity->entityId;
                first.fragmentOrder = 0;
                first.sourceParameterBegin = midpointParameter;
                first.sourceParameterEnd =
                    firstVertices.back().sourceParameter;
                first.reverse =
                    rotated.front().reverseRelativeToInput;
                candidate.fragments.push_back(first);

                for (std::size_t offset = 1U;
                    offset < rotated.size(); ++offset)
                {
                    const auto vertices = orientedVertices(rotated[offset]);
                    if (vertices.size() < 2U) return std::nullopt;
                    ProcessPathFragment fragment;
                    fragment.entityId =
                        rotated[offset].entity->entityId;
                    fragment.fragmentOrder =
                        static_cast<int>(candidate.fragments.size());
                    fragment.sourceParameterBegin =
                        vertices.front().sourceParameter;
                    fragment.sourceParameterEnd =
                        vertices.back().sourceParameter;
                    fragment.reverse =
                        rotated[offset].reverseRelativeToInput;
                    candidate.fragments.push_back(fragment);
                }

                ProcessPathFragment last;
                last.entityId = rotated.front().entity->entityId;
                last.fragmentOrder =
                    static_cast<int>(candidate.fragments.size());
                last.sourceParameterBegin =
                    firstVertices.front().sourceParameter;
                last.sourceParameterEnd = midpointParameter;
                last.reverse =
                    rotated.front().reverseRelativeToInput;
                candidate.fragments.push_back(last);

                std::map<EntityId, int> fragmentCounts;
                std::map<EntityId, double> fragmentLengths;
                std::vector<std::vector<geometry::PathVertex3D>>
                    fragmentPaths;
                for (const ProcessPathFragment& fragment :
                    candidate.fragments)
                {
                    auto path = fragmentVertices
                    (
                        *entities.at(fragment.entityId),
                        fragment.sourceParameterBegin,
                        fragment.sourceParameterEnd,
                        fragment.reverse
                    );
                    if (!path.has_value())
                    {
                        return std::nullopt;
                    }
                    ++fragmentCounts[fragment.entityId];
                    fragmentLengths[fragment.entityId] +=
                        pathLength(*path);
                    fragmentPaths.push_back(std::move(*path));
                }
                for (const DirectedEntity& directed : rotated)
                {
                    const int expected =
                        directed.entity->entityId
                            == midpointSegment->entityId ? 2 : 1;
                    if (fragmentCounts[directed.entity->entityId]
                        != expected)
                    {
                        return std::nullopt;
                    }
                    const double fullLength =
                        pathLength(directed.entity->path.vertices);
                    const double lengthTolerance =
                        std::max(1.0e-8, fullLength * 1.0e-9);
                    if (std::abs(fragmentLengths[directed.entity->entityId]
                        - fullLength) > lengthTolerance)
                    {
                        return std::nullopt;
                    }
                }
                for (std::size_t index = 1U;
                    index < fragmentPaths.size(); ++index)
                {
                    if (distance(fragmentPaths[index - 1U].back().position,
                        fragmentPaths[index].front().position)
                        > policy.connectionTolerance)
                    {
                        return std::nullopt;
                    }
                }
                if (fragmentPaths.empty()
                    || distance(fragmentPaths.back().back().position,
                        fragmentPaths.front().front().position)
                        > policy.connectionTolerance)
                {
                    return std::nullopt;
                }
                fragmentTraversalBuilt = true;

                if (requireMatchingExit)
                {
                    const ExitResult exit = resolveExit
                    (
                        candidate.fragments, entities, section,
                        projectionTolerance, candidate.zone
                    );
                    if (!exit.zone.has_value()
                        || *exit.zone != candidate.zone)
                    {
                        return std::nullopt;
                    }
                    exitResolved = true;
                    candidate.exitConfidence = exit.confidence;
                    candidate.exitReliableLength = exit.reliableLength;
                    candidate.finalEntityId = exit.finalEntityId;
                    candidate.finalParameterBegin =
                        exit.finalParameterBegin;
                    candidate.finalParameterEnd =
                        exit.finalParameterEnd;
                    candidate.exitUsedFallback = exit.usedFallback;
                }

                DirectedEntity& entry = candidate.traversal.entities.front();
                scoreEntrySmoothness
                (
                    entry, currentPosition, midpointSegment->end,
                    tubeCenter, policy.connectionTolerance
                );
                candidate.traversal.entryAxisReversalCount =
                    entry.entryAxisReversalCount;
                candidate.traversal.entryTangentCost =
                    entry.entryTangentCost;
                candidate.traversal.stableSourceIndex =
                    midpointSegment->sourceIndex;
                candidate.traversal.stableEntityId =
                    midpointSegment->entityId;
                scoreTraversal(candidate.traversal,
                    currentPosition, section);
                return candidate;
            }

            template<typename Candidate>
            static bool candidateLess
            (
                const Candidate& left,
                const Candidate& right,
                bool forcedTop,
                double projectionTolerance,
                ProcessOrderingStrategy selectionStrategy
            )
            {
                const double lengthTieTolerance =
                    std::max(1.0e-9, projectionTolerance * 1.0e-3);
                if (forcedTop
                    && std::abs(left.runLength - right.runLength)
                        > lengthTieTolerance)
                {
                    return left.runLength > right.runLength;
                }
                if (!forcedTop)
                {
                    if (std::abs(left.confidence - right.confidence)
                        > kCalculationEpsilon)
                    {
                        return left.confidence > right.confidence;
                    }
                    if (std::abs(left.runLength - right.runLength)
                        > kCalculationEpsilon)
                    {
                        return left.runLength > right.runLength;
                    }
                }
                if (std::abs(left.maximumShellDeviation
                    - right.maximumShellDeviation) > kCalculationEpsilon)
                {
                    return left.maximumShellDeviation
                        < right.maximumShellDeviation;
                }
                if (forcedTop
                    && std::abs(left.confidence - right.confidence)
                        > kCalculationEpsilon)
                {
                    return left.confidence > right.confidence;
                }
                if (forcedTop
                    && std::abs(left.runLength - right.runLength)
                        > kCalculationEpsilon)
                {
                    return left.runLength > right.runLength;
                }
                if (!forcedTop)
                {
                    if (traversalLess(left.traversal,
                        right.traversal, selectionStrategy))
                    {
                        return true;
                    }
                    if (traversalLess(right.traversal,
                        left.traversal, selectionStrategy))
                    {
                        return false;
                    }
                }
                if (left.midpointSourceIndex !=
                    right.midpointSourceIndex)
                {
                    return left.midpointSourceIndex
                        < right.midpointSourceIndex;
                }
                if (left.midpointEntityId != right.midpointEntityId)
                    return left.midpointEntityId < right.midpointEntityId;
                if (left.midpointSourceParameter
                    != right.midpointSourceParameter)
                {
                    return left.midpointSourceParameter
                        < right.midpointSourceParameter;
                }
                return left.direction < right.direction;
            }
        };

        QVariantMap breakDiagnosticValues
        (
            const BreakBoundaryTraversalReport& report
        )
        {
            QVariantMap values;
            values.insert(QStringLiteral("breakStartSummary"), true);
            values.insert(QStringLiteral("groupId"), report.groupId);
            values.insert(QStringLiteral("boundaryRank"), report.boundaryRank);
            values.insert(QStringLiteral("boundaryPairId"),
                report.boundaryPairId);
            values.insert(QStringLiteral("forcedTopMidpoint"),
                report.forcedTopMidpoint);
            values.insert(QStringLiteral("preferredStartZone"),
                machining::tubeZoneName(report.preferredStartZone));
            values.insert(QStringLiteral("candidateRunCount"),
                report.candidateRunCount);
            values.insert(QStringLiteral("candidateRuns"),
                report.candidateRuns);
            values.insert(QStringLiteral("strategy"),
                report.forcedTopMidpoint
                    ? QStringLiteral("ForcedTopMidpoint")
                    : QStringLiteral("StrongZoneMidpoint"));
            values.insert(QStringLiteral("selectedZone"),
                machining::tubeZoneName(report.startZone));
            values.insert(QStringLiteral("selectedRunLength"),
                report.selectedRunLength);
            values.insert(QStringLiteral("selectedMaximumShellDeviation"),
                report.selectedMaximumShellDeviation);
            values.insert(QStringLiteral("selectedConfidence"),
                report.selectedConfidence);
            values.insert(QStringLiteral("selectedMidpoint"),
                QStringLiteral("%1,%2,%3")
                    .arg(report.selectedMidpoint.x, 0, 'g', 15)
                    .arg(report.selectedMidpoint.y, 0, 'g', 15)
                    .arg(report.selectedMidpoint.z, 0, 'g', 15));
            values.insert(QStringLiteral("selectedEntityId"),
                QVariant::fromValue<qulonglong>(report.selectedEntityId));
            values.insert(QStringLiteral("selectedSourceParameter"),
                report.selectedSourceParameter);
            values.insert(QStringLiteral("exitZone"),
                report.exitZone.has_value()
                    ? machining::tubeZoneName(*report.exitZone)
                    : QStringLiteral("Unknown"));
            values.insert(QStringLiteral("exitConfidence"),
                report.exitConfidence);
            values.insert(QStringLiteral("exitReliableLength"),
                report.exitReliableLength);
            values.insert(QStringLiteral("finalEntityId"),
                QVariant::fromValue<qulonglong>(report.finalEntityId));
            values.insert(QStringLiteral("finalParameterBegin"),
                report.finalParameterBegin);
            values.insert(QStringLiteral("finalParameterEnd"),
                report.finalParameterEnd);
            values.insert(QStringLiteral("exitUsedFallback"),
                report.exitUsedFallback);
            values.insert(QStringLiteral("direction"), report.direction);
            values.insert(QStringLiteral("fragmentCount"),
                report.fragmentCount);
            values.insert(QStringLiteral("midpointFragmentUsed"),
                report.fragmentCount > 0);
            values.insert(QStringLiteral("nextPartitionId"),
                report.nextPartitionId);
            values.insert(QStringLiteral("partitionMappingFound"),
                report.partitionMappingFound);
            values.insert(QStringLiteral("partitionStartSucceeded"),
                report.partitionStartSucceeded);
            values.insert(QStringLiteral("status"), report.status);
            values.insert(QStringLiteral("failureReason"),
                report.failureReason);
            return values;
        }

        Diagnostic breakStartDiagnostic
        (
            const OperationContext& context,
            const BreakBoundaryTraversalReport& report
        )
        {
            return planningDiagnostic
            (
                context,
                DiagnosticCode::ProcessPlanningBreakStartSummary,
                QStringLiteral("加工断面已从可靠强区位边段中点建立闭环遍历。"),
                QStringLiteral("Break boundary traversal uses complementary source fragments around a strong-zone arc-length midpoint."),
                breakDiagnosticValues(report),
                report.exitUsedFallback
                    ? DiagnosticSeverity::Warning
                    : DiagnosticSeverity::Info
            );
        }

        QString entryCandidateKindName(ZoneEntryCandidateKind kind)
        {
            switch (kind)
            {
            case ZoneEntryCandidateKind::OpenEndpoint:
                return QStringLiteral("OpenEndpoint");
            case ZoneEntryCandidateKind::ClosedCurveParameter:
                return QStringLiteral("ClosedCurveParameter");
            case ZoneEntryCandidateKind::ClosedLoopConnection:
                return QStringLiteral("ClosedLoopConnection");
            case ZoneEntryCandidateKind::ClosedLoopArcInterior:
                return QStringLiteral("ClosedLoopArcInterior");
            case ZoneEntryCandidateKind::ClosedLoopEllipseInterior:
                return QStringLiteral("ClosedLoopEllipseInterior");
            case ZoneEntryCandidateKind::ClosedLoopZoneRunMidpoint:
                return QStringLiteral("ClosedLoopZoneRunMidpoint");
            case ZoneEntryCandidateKind::BreakZoneMidpoint:
                return QStringLiteral("BreakZoneMidpoint");
            }
            return QStringLiteral("Unknown");
        }

        QString planningSourceKindName(geometry::SourceGeometryKind kind)
        {
            switch (kind)
            {
            case geometry::SourceGeometryKind::Line:
                return QStringLiteral("Line");
            case geometry::SourceGeometryKind::Arc:
                return QStringLiteral("Arc");
            case geometry::SourceGeometryKind::Circle:
                return QStringLiteral("Circle");
            case geometry::SourceGeometryKind::Ellipse:
                return QStringLiteral("Ellipse");
            case geometry::SourceGeometryKind::Polyline:
                return QStringLiteral("Polyline");
            case geometry::SourceGeometryKind::Spline:
                return QStringLiteral("Spline");
            case geometry::SourceGeometryKind::Point:
                return QStringLiteral("Point");
            case geometry::SourceGeometryKind::Unknown:
                return QStringLiteral("Unknown");
            }
            return QStringLiteral("Unknown");
        }

        QString vectorText(const Vector3d& value)
        {
            return QStringLiteral("%1,%2,%3")
                .arg(value.x, 0, 'g', 15)
                .arg(value.y, 0, 'g', 15)
                .arg(value.z, 0, 'g', 15);
        }

        Diagnostic entrySelectionDiagnostic
        (
            const OperationContext& context,
            const ProcessGroup& group,
            const GroupTraversal& traversal,
            std::optional<machining::TubeZone16> scheduledZone
        )
        {
            QVariantMap values;
            values.insert(QStringLiteral("entrySelectionSummary"), true);
            values.insert(QStringLiteral("entryRefinementSummary"),
                !traversal.entryRefinementMode.isEmpty());
            values.insert(QStringLiteral("unitKey"), processGroupKeyText(group));
            values.insert(QStringLiteral("groupKind"), groupKindName(group.kind));
            values.insert(QStringLiteral("candidateCount"),
                traversal.entryCandidateCount);
            values.insert(QStringLiteral("connectionCandidateCount"),
                traversal.connectionCandidateCount);
            values.insert(QStringLiteral("arcInteriorCandidateCount"),
                traversal.arcInteriorCandidateCount);
            values.insert(QStringLiteral("ellipseInteriorCandidateCount"),
                traversal.ellipseInteriorCandidateCount);
            values.insert(QStringLiteral("zoneRunMidpointCandidateCount"),
                traversal.zoneRunMidpointCandidateCount);
            values.insert(QStringLiteral("curveCandidateRejectedCount"),
                traversal.curveCandidateRejectedCount);
            values.insert(QStringLiteral("wrongZoneRejectedCount"),
                traversal.wrongZoneRejectedCount);
            values.insert(QStringLiteral("selectedStart"),
                vectorText(traversal.start));
            values.insert(QStringLiteral("selectedReverse"),
                !traversal.entities.empty()
                    && traversal.entities.front().reverseRelativeToInput);
            values.insert(QStringLiteral("axisReversalCount"),
                traversal.entryAxisReversalCount);
            values.insert(QStringLiteral("tangentCost"),
                traversal.entryTangentCost);
            values.insert(QStringLiteral("rotationCost"),
                traversal.rotationCost);
            values.insert(QStringLiteral("movementDistance"),
                traversal.movementDistance);
            values.insert(QStringLiteral("scheduledZone"),
                scheduledZone.has_value()
                ? machining::tubeZoneName(*scheduledZone)
                : QStringLiteral("None"));
            values.insert(QStringLiteral("ownerZone"),
                scheduledZone.has_value()
                ? machining::tubeZoneName(*scheduledZone)
                : QStringLiteral("None"));
            values.insert(QStringLiteral("selectedEntryZone"),
                traversal.selectedEntry.has_value()
                ? machining::tubeZoneName
                    (traversal.selectedEntry->zone)
                : QStringLiteral("None"));
            values.insert(QStringLiteral("candidateKind"),
                traversal.selectedEntry.has_value()
                ? entryCandidateKindName
                    (traversal.selectedEntry->kind)
                : QStringLiteral("Unconstrained"));
            const bool curveInteriorSelected =
                traversal.selectedEntry.has_value()
                && (traversal.selectedEntry->kind
                        == ZoneEntryCandidateKind::ClosedLoopArcInterior
                    || traversal.selectedEntry->kind
                        == ZoneEntryCandidateKind::
                            ClosedLoopEllipseInterior);
            const bool zoneRunMidpointSelected =
                traversal.selectedEntry.has_value()
                && traversal.selectedEntry->kind
                    == ZoneEntryCandidateKind::
                        ClosedLoopZoneRunMidpoint;
            values.insert(QStringLiteral("selectionMode"),
                !traversal.entryRefinementMode.isEmpty()
                ? traversal.entryRefinementMode
                : curveInteriorSelected
                    ? QStringLiteral("ExactCurveTangent")
                    : zoneRunMidpointSelected
                        ? QStringLiteral("ZoneRunMidpointFallback")
                        : QStringLiteral("NearestConnection"));
            values.insert(QStringLiteral("selectedEntityId"),
                QVariant::fromValue<qulonglong>
                    (traversal.selectedEntry.has_value()
                    ? traversal.selectedEntry->entityId : 0U));
            values.insert(QStringLiteral("selectedSourceKind"),
                traversal.selectedEntry.has_value()
                ? planningSourceKindName
                    (traversal.selectedEntry->sourceKind)
                : QStringLiteral("Unknown"));
            values.insert(QStringLiteral("selectedSourceParameter"),
                traversal.selectedEntry.has_value()
                    && traversal.selectedEntry->sourceParameter.has_value()
                ? *traversal.selectedEntry->sourceParameter : 0.0);
            values.insert(QStringLiteral("entryPosition"),
                traversal.selectedEntry.has_value()
                ? vectorText(traversal.selectedEntry->entryPosition)
                : vectorText(traversal.start));
            values.insert(QStringLiteral("firstCutTangent"),
                traversal.selectedEntry.has_value()
                ? vectorText(traversal.selectedEntry->firstCutTangent)
                : QStringLiteral("0,0,0"));
            values.insert(QStringLiteral("distanceToZoneBoundary"),
                traversal.selectedEntry.has_value()
                ? traversal.selectedEntry->distanceToZoneBoundary
                : 0.0);
            values.insert(QStringLiteral("distanceToMemberEndpoint"),
                traversal.selectedEntry.has_value()
                ? traversal.selectedEntry->distanceToMemberEndpoint
                : 0.0);
            values.insert(QStringLiteral("fragmentCount"),
                static_cast<int>(traversal.fragments.size()));
            values.insert(QStringLiteral("midpointFragmentUsed"),
                !traversal.fragments.empty());
            values.insert(QStringLiteral("previousCutEnd"),
                vectorText(traversal.previousCutEnd));
            values.insert(QStringLiteral("previousTransferAnchor"),
                vectorText(traversal.previousTransferAnchor));
            values.insert(QStringLiteral("curveMemberCount"),
                traversal.curveMemberCount);
            values.insert(QStringLiteral("arcTangentRootCount"),
                traversal.arcTangentRootCount);
            values.insert(QStringLiteral("ellipseTangentRootCount"),
                traversal.ellipseTangentRootCount);
            values.insert(QStringLiteral("validTangentCount"),
                traversal.validTangentCount);
            values.insert(QStringLiteral("travelDistance"),
                traversal.entryTravelDistance);
            values.insert(QStringLiteral("approachCutAngle"),
                traversal.approachCutAngle);
            values.insert(QStringLiteral("nearestConnectionDistance"),
                traversal.nearestConnectionDistance);
            values.insert(QStringLiteral("forwardAngle"),
                traversal.forwardAngle);
            values.insert(QStringLiteral("reverseAngle"),
                traversal.reverseAngle);
            values.insert(QStringLiteral("tangentResidual"),
                traversal.tangentResidual);
            values.insert(QStringLiteral("approachCutDot"),
                traversal.approachCutDot);
            return planningDiagnostic
            (
                context,
                DiagnosticCode::ProcessPlanningEntrySelectionSummary,
                QStringLiteral("普通四轴加工单元已选择合法平滑入口。"),
                QStringLiteral("Ordinary rotary unit selected a legal entry after zone and longitudinal ordering."),
                values,
                DiagnosticSeverity::Info
            );
        }

        std::optional<GroupTraversal> bestTraversal
        (
            const ProcessGroup& group,
            const std::unordered_map<EntityId, const PlanningEntity*>& entities,
            const Vector3d& currentPosition,
            const ProcessPlanningPolicy& policy,
            const std::optional<machining::TubeSectionModel>& section,
            const std::optional<Vector2d>& tubeCenter,
            ProcessOrderingStrategy selectionStrategy,
            ClosedLoopTraversalReport* closedLoopReport = nullptr,
            const TraversalSelectionContext* selection = nullptr
        )
        {
            if (closedLoopReport != nullptr) *closedLoopReport = ClosedLoopTraversalReport{};
            if (group.entityIds.size() == 1U)
            {
                const auto found = entities.find(group.entityIds.front());
                if (found == entities.end()) return std::nullopt;
                const PlanningEntity& entity = *found->second;
                if (isSingleClosedEntryOptimizedCurve(group, entity))
                {
                    std::optional<GroupTraversal> best;
                    int candidateCount = 0;
                    int wrongZoneRejectedCount = 0;
                    const std::size_t startCandidateCount =
                        entity.startParameter.has_value()
                        ? 1U : entity.path.vertices.size();
                    for (std::size_t startIndex = 0U;
                        startIndex < startCandidateCount; ++startIndex)
                    {
                        for (const bool reverse : { false, true })
                        {
                            if (!directionAllowed(entity, reverse,
                                policy.allowReverse)
                                || !manualDirectionAllowed(entity, reverse,
                                    selection))
                            {
                                continue;
                            }
                            const double candidateParameter =
                                entity.path.vertices[startIndex]
                                    .sourceParameter;
                            if (selection != nullptr
                                && selection->hardZoneConstraint
                                && entity.manualStartParameter.has_value()
                                && std::abs(candidateParameter
                                    - *entity.manualStartParameter)
                                    > 1.0e-10)
                            {
                                continue;
                            }
                            auto candidate = buildSingleClosedCurveTraversal
                            (
                                group, entity, currentPosition, reverse, startIndex,
                                policy.connectionTolerance, section, tubeCenter
                            );
                            if (!candidate.has_value()) continue;
                            if (selection != nullptr
                                && selection->hardZoneConstraint
                                && selection->requiredEntryZone.has_value()
                                && section.has_value())
                            {
                                const std::size_t pointCount =
                                    entity.path.vertices.size();
                                const double threshold = entryThreshold
                                    (policy.connectionTolerance);
                                std::optional<Vector3d> firstCutPoint;
                                for (std::size_t offset = 1U;
                                    offset < pointCount; ++offset)
                                {
                                    const std::size_t index = reverse
                                        ? (startIndex + pointCount - offset)
                                            % pointCount
                                        : (startIndex + offset) % pointCount;
                                    const Vector3d& point =
                                        entity.path.vertices[index].position;
                                    if (distance(candidate->start, point)
                                        > threshold)
                                    {
                                        firstCutPoint = point;
                                        break;
                                    }
                                }
                                if (!firstCutPoint.has_value())
                                    continue;
                                auto entry = classifyZoneEntry
                                (
                                    ZoneEntryCandidateKind::
                                        ClosedCurveParameter,
                                    entity.entityId, candidateParameter,
                                    reverse, candidate->start,
                                    *firstCutPoint, entity.sourceKind, *section,
                                    selection->projectionTolerance
                                );
                                if (!entry.has_value()
                                    || entry->zone
                                        != *selection->requiredEntryZone)
                                {
                                    ++wrongZoneRejectedCount;
                                    continue;
                                }
                                candidate->selectedEntry =
                                    std::move(entry);
                                scoreUnwrappedEntry
                                (
                                    candidate->entities.front(),
                                    selection->previousEnd,
                                    *firstCutPoint, *section,
                                    policy.connectionTolerance,
                                    selection->projectionTolerance
                                );
                                candidate->entryAxisReversalCount =
                                    candidate->entities.front()
                                        .entryAxisReversalCount;
                                candidate->entryTangentCost =
                                    candidate->entities.front()
                                        .entryTangentCost;
                            }
                            ++candidateCount;
                            if (!best.has_value()
                                || (selection != nullptr
                                    ? zoneConstrainedTraversalLess
                                        (*candidate, *best, *selection,
                                            selectionStrategy)
                                    : traversalLess(*candidate, *best,
                                        selectionStrategy)))
                            {
                                best = std::move(candidate);
                            }
                        }
                    }
                    if (best.has_value())
                    {
                        best->entryCandidateCount = candidateCount;
                        best->wrongZoneRejectedCount =
                            wrongZoneRejectedCount;
                    }
                    return best;
                }
            }

            if (group.kind == ProcessGroupKind::ClosedLoop && group.entityIds.size() > 1U)
            {
                auto canonicalLoop = ClosedLoopTraversalBuilder::build
                (
                    group, entities, currentPosition, policy, section,
                    tubeCenter, selectionStrategy
                );
                if (!canonicalLoop.traversal.has_value())
                {
                    if (closedLoopReport != nullptr)
                        *closedLoopReport =
                            std::move(canonicalLoop.report);
                    return std::nullopt;
                }
                if (selection == nullptr
                    || !selection->hardZoneConstraint
                    || !section.has_value())
                {
                    canonicalLoop.traversal->entryCandidateCount =
                        canonicalLoop.report.candidateCount;
                    if (closedLoopReport != nullptr)
                        *closedLoopReport =
                            std::move(canonicalLoop.report);
                    return std::move(canonicalLoop.traversal);
                }

                auto connectionLoop = ClosedLoopTraversalBuilder::build
                (
                    group, entities, currentPosition, policy, section,
                    tubeCenter, selectionStrategy, selection
                );
                auto interior = ClosedLoopZoneRunBuilder::buildOrdinary
                (
                    group, *canonicalLoop.traversal, entities,
                    currentPosition, policy, *section, tubeCenter,
                    selectionStrategy, *selection
                );
                const bool curveInteriorMode =
                    interior.traversal.has_value();
                const bool connectionMode =
                    !curveInteriorMode
                    && connectionLoop.traversal.has_value();
                std::optional<GroupTraversal> best;
                if (curveInteriorMode)
                    best = std::move(interior.traversal);
                else if (connectionMode)
                    best = std::move(connectionLoop.traversal);
                else if (selection->allowZoneRunMidpointFallback)
                    best = std::move(interior.zoneRunMidpointTraversal);
                const int connectionCandidateCount =
                    connectionLoop.report.candidateCount;
                const int totalCandidateCount =
                    connectionCandidateCount + interior.candidateCount
                    + interior.zoneRunMidpointCandidateCount;
                const int wrongZoneRejectedCount =
                    connectionLoop.report.wrongZoneRejectedCount
                    + interior.wrongZoneRejectedCount;
                if (best.has_value())
                {
                    best->entryCandidateCount = totalCandidateCount;
                    best->connectionCandidateCount =
                        connectionCandidateCount;
                    best->arcInteriorCandidateCount =
                        interior.arcInteriorCandidateCount;
                    best->ellipseInteriorCandidateCount =
                        interior.ellipseInteriorCandidateCount;
                    best->zoneRunMidpointCandidateCount =
                        interior.zoneRunMidpointCandidateCount;
                    best->curveCandidateRejectedCount =
                        interior.curveCandidateRejectedCount;
                    best->curveMemberCount =
                        interior.curveMemberCount;
                    best->arcTangentRootCount =
                        interior.arcTangentRootCount;
                    best->ellipseTangentRootCount =
                        interior.ellipseTangentRootCount;
                    best->validTangentCount =
                        interior.validTangentCount;
                    best->arcInteriorCandidateEntityIds =
                        std::move(interior.arcCandidateEntityIds);
                    best->ellipseInteriorCandidateEntityIds =
                        std::move(interior.ellipseCandidateEntityIds);
                    best->wrongZoneRejectedCount =
                        wrongZoneRejectedCount;
                }
                canonicalLoop.report.candidateCount =
                    totalCandidateCount;
                if (best.has_value())
                {
                    canonicalLoop.report.selectedOrder.clear();
                    canonicalLoop.report.selectedReverse.clear();
                    for (const DirectedEntity& directed :
                        best->entities)
                    {
                        canonicalLoop.report.selectedOrder.push_back
                            (directed.entity->entityId);
                        canonicalLoop.report.selectedReverse.push_back
                            (directed.reverseRelativeToInput);
                    }
                }
                if (closedLoopReport != nullptr)
                    *closedLoopReport =
                        std::move(canonicalLoop.report);
                return best;
            }

            std::optional<GroupTraversal> best;
            int candidateCount = 0;
            int wrongZoneRejectedCount = 0;
            for (const EntityId entityId : group.entityIds)
            {
                for (const bool reverse : { false, true })
                {
                    const auto found = entities.find(entityId);
                    if (found == entities.end()
                        || !directionAllowed(*found->second, reverse,
                            policy.allowReverse)
                        || !manualDirectionAllowed(*found->second,
                            reverse, selection))
                    {
                        continue;
                    }
                    auto candidate = buildTraversal
                    (
                        group, entities, currentPosition, policy.allowReverse,
                        policy.connectionTolerance, std::make_pair(entityId, reverse)
                    );
                    if (!candidate.has_value()) continue;
                    if (selection != nullptr
                        && selection->hardZoneConstraint
                        && selection->requiredEntryZone.has_value()
                        && section.has_value())
                    {
                        const DirectedEntity& entryDirected =
                            candidate->entities.front();
                        const bool manualDirectionsMatch = std::all_of
                        (
                            candidate->entities.cbegin(),
                            candidate->entities.cend(),
                            [&selection](const DirectedEntity& directed)
                            {
                                return directed.entity != nullptr
                                    && manualDirectionAllowed
                                    (
                                        *directed.entity,
                                        directed.reverseRelativeToInput,
                                        selection
                                    );
                            }
                        );
                        if (!manualDirectionsMatch) continue;
                        const std::vector<Vector3d> entryPoints =
                            directedPoints(*entryDirected.entity,
                                entryDirected.reverseRelativeToInput);
                        const double threshold = entryThreshold
                            (policy.connectionTolerance);
                        const auto next = std::find_if
                        (
                            entryPoints.cbegin() + 1,
                            entryPoints.cend(),
                            [&entryPoints, threshold]
                            (const Vector3d& point)
                            {
                                return distance(entryPoints.front(), point)
                                    > threshold;
                            }
                        );
                        if (next == entryPoints.cend()) continue;
                        const auto entryVertices =
                            entryDirected.reverseRelativeToInput
                            ? std::vector<geometry::PathVertex3D>
                                (entryDirected.entity->path.vertices.rbegin(),
                                    entryDirected.entity->path.vertices.rend())
                            : entryDirected.entity->path.vertices;
                        const std::optional<double> entryParameter =
                            entryVertices.empty() ? std::nullopt
                                : std::optional<double>
                                    (entryVertices.front().sourceParameter);
                        auto entry = classifyZoneEntry
                        (
                            ZoneEntryCandidateKind::OpenEndpoint,
                            entryDirected.entity->entityId,
                            entryParameter,
                            entryDirected.reverseRelativeToInput,
                            candidate->start, *next,
                            entryDirected.entity->sourceKind, *section,
                            selection->projectionTolerance
                        );
                        const bool manualStartMatches =
                            !entryDirected.entity
                                ->manualStartParameter.has_value()
                            || (entryParameter.has_value()
                                && std::abs(*entryParameter
                                    - *entryDirected.entity
                                        ->manualStartParameter)
                                    <= 1.0e-10);
                        const bool otherManualStartExists = std::any_of
                        (
                            candidate->entities.cbegin() + 1,
                            candidate->entities.cend(),
                            [](const DirectedEntity& directed)
                            {
                                return directed.entity != nullptr
                                    && directed.entity
                                        ->manualStartParameter.has_value();
                            }
                        );
                        if (!entry.has_value()
                            || entry->zone
                                != *selection->requiredEntryZone
                            || !manualStartMatches
                            || otherManualStartExists)
                        {
                            ++wrongZoneRejectedCount;
                            continue;
                        }
                        candidate->selectedEntry = std::move(entry);
                        scoreUnwrappedEntry
                        (
                            candidate->entities.front(),
                            selection->previousEnd, *next, *section,
                            policy.connectionTolerance,
                            selection->projectionTolerance
                        );
                        candidate->entryAxisReversalCount =
                            candidate->entities.front()
                                .entryAxisReversalCount;
                        candidate->entryTangentCost =
                            candidate->entities.front()
                                .entryTangentCost;
                    }
                    ++candidateCount;
                    scoreTraversal(*candidate, currentPosition, section);
                    if (!best.has_value()
                        || (selection != nullptr
                            ? zoneConstrainedTraversalLess
                                (*candidate, *best, *selection,
                                    selectionStrategy)
                            : traversalLess(*candidate, *best,
                                selectionStrategy)))
                        best = std::move(candidate);
                }
            }
            if (best.has_value())
            {
                best->entryCandidateCount = candidateCount;
                best->wrongZoneRejectedCount =
                    wrongZoneRejectedCount;
            }
            return best;
        }

        bool sameEntitySet(std::vector<EntityId> left, std::vector<EntityId> right)
        {
            std::sort(left.begin(), left.end());
            std::sort(right.begin(), right.end());
            return left == right;
        }

        struct ClosedLoopValidationFailure
        {
            int groupId = -1;
            EntityId previousEntityId = 0;
            EntityId currentEntityId = 0;
            double joinGap = 0.0;
            QString reason;
        };

        bool validateMultiEntityClosedLoopUnits
        (
            const ProcessPlan& plan,
            const std::unordered_map<EntityId, const PlanningEntity*>& entities,
            double connectionTolerance,
            ClosedLoopValidationFailure& failure
        )
        {
            std::map<EntityId, const ProcessAssignment*> assignments;
            for (const ProcessAssignment& assignment : plan.assignments)
                assignments.emplace(assignment.entityId, &assignment);
            std::map<int, std::vector<const ProcessPathFragment*>>
                fragmentsByUnit;
            for (const ProcessPathFragment& fragment :
                plan.plannedFragments)
            {
                fragmentsByUnit[fragment.processUnitIndex]
                    .push_back(&fragment);
            }

            for (const ProcessGroup& group : plan.groups)
            {
                if (group.kind != ProcessGroupKind::ClosedLoop
                    || group.entityIds.size() <= 1U) continue;
                std::vector<EntityId> key = group.entityIds;
                std::sort(key.begin(), key.end());
                const auto unit = std::find_if
                (
                    plan.processUnits.cbegin(), plan.processUnits.cend(),
                    [&key](const ProcessUnit& candidate)
                    { return candidate.key.memberEntityIds == key; }
                );
                if (unit == plan.processUnits.cend()
                    || unit->orderedMemberEntityIds.size() != group.entityIds.size())
                {
                    failure.groupId = group.groupId;
                    failure.reason = QStringLiteral("Closed-loop ProcessUnit is missing or incomplete.");
                    return false;
                }
                const int processUnitIndex = static_cast<int>
                    (std::distance(plan.processUnits.cbegin(), unit));
                const auto fragmented = fragmentsByUnit.find
                    (processUnitIndex);
                if (fragmented != fragmentsByUnit.end())
                {
                    auto fragments = fragmented->second;
                    std::sort(fragments.begin(), fragments.end(),
                        [](const ProcessPathFragment* left,
                            const ProcessPathFragment* right)
                        {
                            return left->fragmentOrder
                                < right->fragmentOrder;
                        });
                    std::map<EntityId, int> fragmentCounts;
                    std::map<EntityId, double> fragmentLengths;
                    std::vector<std::vector<geometry::PathVertex3D>>
                        fragmentPaths;
                    for (std::size_t index = 0U;
                        index < fragments.size(); ++index)
                    {
                        const ProcessPathFragment* fragment =
                            fragments[index];
                        const auto entity = fragment != nullptr
                            ? entities.find(fragment->entityId)
                            : entities.end();
                        if (fragment == nullptr
                            || fragment->fragmentOrder
                                != static_cast<int>(index)
                            || entity == entities.end()
                            || entity->second == nullptr)
                        {
                            failure.groupId = group.groupId;
                            failure.reason = QStringLiteral(
                                "Closed-loop fragment metadata is incomplete.");
                            return false;
                        }
                        auto path = ClosedLoopZoneRunBuilder::
                            fragmentVertices
                            (
                                *entity->second,
                                fragment->sourceParameterBegin,
                                fragment->sourceParameterEnd,
                                fragment->reverse
                            );
                        if (!path.has_value())
                        {
                            failure.groupId = group.groupId;
                            failure.currentEntityId =
                                fragment->entityId;
                            failure.reason = QStringLiteral(
                                "Closed-loop fragment parameter interval is invalid.");
                            return false;
                        }
                        ++fragmentCounts[fragment->entityId];
                        fragmentLengths[fragment->entityId] +=
                            ClosedLoopZoneRunBuilder::pathLength(*path);
                        fragmentPaths.push_back(std::move(*path));
                    }
                    int splitMemberCount = 0;
                    for (const EntityId entityId :
                        unit->key.memberEntityIds)
                    {
                        const auto entity = entities.find(entityId);
                        if (entity == entities.end()
                            || entity->second == nullptr)
                        {
                            failure.groupId = group.groupId;
                            failure.currentEntityId = entityId;
                            failure.reason = QStringLiteral(
                                "Closed-loop fragment source entity is missing.");
                            return false;
                        }
                        const int count = fragmentCounts[entityId];
                        if (count == 2) ++splitMemberCount;
                        else if (count != 1)
                        {
                            failure.groupId = group.groupId;
                            failure.currentEntityId = entityId;
                            failure.reason = QStringLiteral(
                                "Closed-loop fragments do not cover each member exactly once.");
                            return false;
                        }
                        const double fullLength =
                            ClosedLoopZoneRunBuilder::pathLength
                                (entity->second->path.vertices);
                        const double lengthTolerance =
                            std::max(1.0e-8, fullLength * 1.0e-9);
                        if (std::abs(fragmentLengths[entityId]
                            - fullLength) > lengthTolerance)
                        {
                            failure.groupId = group.groupId;
                            failure.currentEntityId = entityId;
                            failure.reason = QStringLiteral(
                                "Closed-loop fragment coverage has a gap or overlap.");
                            return false;
                        }
                    }
                    if (splitMemberCount != 1
                        || fragmentCounts.size()
                            != unit->key.memberEntityIds.size())
                    {
                        failure.groupId = group.groupId;
                        failure.reason = QStringLiteral(
                            "Closed-loop internal entry must split exactly one member.");
                        return false;
                    }
                    for (std::size_t index = 1U;
                        index < fragmentPaths.size(); ++index)
                    {
                        const double gap = distance
                            (fragmentPaths[index - 1U].back().position,
                                fragmentPaths[index].front().position);
                        if (gap > connectionTolerance)
                        {
                            failure = { group.groupId,
                                fragments[index - 1U]->entityId,
                                fragments[index]->entityId, gap,
                                QStringLiteral(
                                    "Adjacent closed-loop fragments are not physically connected.") };
                            return false;
                        }
                    }
                    if (fragmentPaths.empty()
                        || distance(fragmentPaths.back().back().position,
                            fragmentPaths.front().front().position)
                            > connectionTolerance)
                    {
                        failure.groupId = group.groupId;
                        failure.reason = QStringLiteral(
                            "Closed-loop fragment traversal does not return to its entry.");
                        return false;
                    }
                    continue;
                }

                Vector3d firstStart;
                Vector3d previousEnd;
                EntityId previousEntityId = 0;
                bool first = true;
                for (const EntityId entityId : unit->orderedMemberEntityIds)
                {
                    const auto entity = entities.find(entityId);
                    const auto assignment = assignments.find(entityId);
                    if (entity == entities.end() || assignment == assignments.end()
                        || entity->second == nullptr || entity->second->path.closed)
                    {
                        failure.groupId = group.groupId;
                        failure.currentEntityId = entityId;
                        failure.reason = QStringLiteral("Closed-loop member or assignment is invalid.");
                        return false;
                    }
                    const std::vector<Vector3d> points = directedPoints
                        (*entity->second, assignment->second->reverse);
                    if (points.size() < 2U)
                    {
                        failure.groupId = group.groupId;
                        failure.currentEntityId = entityId;
                        failure.reason = QStringLiteral("Closed-loop member has no physical endpoints.");
                        return false;
                    }
                    if (first)
                    {
                        firstStart = points.front();
                        first = false;
                    }
                    else
                    {
                        const double gap = distance(previousEnd, points.front());
                        if (gap > connectionTolerance)
                        {
                            failure = { group.groupId, previousEntityId, entityId, gap,
                                QStringLiteral("Adjacent closed-loop members are not physically connected.") };
                            return false;
                        }
                    }
                    previousEnd = points.back();
                    previousEntityId = entityId;
                }
                const double closureGap = distance(previousEnd, firstStart);
                if (closureGap > connectionTolerance)
                {
                    failure = { group.groupId, previousEntityId,
                        unit->orderedMemberEntityIds.front(), closureGap,
                        QStringLiteral("Closed-loop traversal does not return to its physical start.") };
                    return false;
                }
            }
            return true;
        }
    }

    OperationResult<ProcessPlan> ProcessPlanBuilder::build
    (
        const ProcessPlanningInput& input,
        const ProcessPlanningPolicy& policy,
        const OperationContext& context
    )
    {
        if (input.contentRevision == 0U || input.topology == nullptr
            || input.entities.empty() || policy.connectionTolerance <= 0.0
            || !std::isfinite(policy.connectionTolerance)
            || !std::isfinite(policy.rotationSafetyClearance)
            || policy.rotationSafetyClearance <= 0.0
            || !std::isfinite(policy.sameZoneTransferClearance)
            || policy.sameZoneTransferClearance < 0.0
            || !std::isfinite(policy.connectionDistanceTieTolerance)
            || policy.connectionDistanceTieTolerance <= 0.0)
        {
            return failure<ProcessPlan>
            (
                OperationStatus::InvalidInput, context, DiagnosticCode::ProcessPlanningInputInvalid,
                QStringLiteral("加工计划输入无效。"), QStringLiteral("Revision, topology, entities, or policy is invalid."),
                diagnosticValues(input, policy)
            );
        }
        if (input.topologyInput.contentRevision != input.contentRevision)
        {
            return failure<ProcessPlan>
            (
                OperationStatus::Conflict, context, DiagnosticCode::ProcessPlanningRevisionMismatch,
                QStringLiteral("加工计划输入版本不一致。"), QStringLiteral("TopologyInput revision does not match planning input."),
                diagnosticValues(input, policy)
            );
        }
        if (input.tubeSection.has_value()
            && input.tubeSection->contentRevision != input.contentRevision)
        {
            return failure<ProcessPlan>
            (
                OperationStatus::Conflict, context, DiagnosticCode::ProcessPlanningRevisionMismatch,
                QStringLiteral("方管截面已过期，请重新识别后再排序。"),
                QStringLiteral("TubeSectionModel revision does not match planning input."),
                diagnosticValues(input, policy)
            );
        }
        if (policy.orderingStrategy == ProcessOrderingStrategy::LazyRotation && !input.tubeSection.has_value())
        {
            return failure<ProcessPlan>
            (
                OperationStatus::InvalidInput, context, DiagnosticCode::ProcessPlanningInputInvalid,
                QStringLiteral("启用懒旋转加工前需要先识别方管截面。"), QStringLiteral("LazyRotation requires TubeSectionModel."),
                diagnosticValues(input, policy)
            );
        }

        std::optional<machining::TubeSectionGeometry> planningSection;
        std::optional<machining::TubeSectionModel> surfaceSweepSection;
        if (input.tubeSection.has_value())
        {
            auto preparedSection = TubeCutBoundaryClassifier::prepareSection
                (input.tubeSection->geometry, context);
            if (!preparedSection.succeeded() || !preparedSection.value.has_value())
            {
                auto result = failure<ProcessPlan>
                (
                    OperationStatus::InvalidInput, context,
                    DiagnosticCode::ProcessPlanningInputInvalid,
                    QStringLiteral("已识别的方管截面无法用于加工计划。"),
                    QStringLiteral("Tube section normalization failed before planning."),
                    diagnosticValues(input, policy)
                );
                result.mergeDiagnostics(preparedSection.diagnostics);
                return result;
            }
            planningSection = std::move(*preparedSection.value);
            surfaceSweepSection = *input.tubeSection;
            surfaceSweepSection->geometry = *planningSection;
        }
        const bool zone16SweepEnabled = policy.sortIntent
                == ProcessSortIntent::RebuildSequence
            && policy.orderingStrategy == ProcessOrderingStrategy::LazyRotation
            && surfaceSweepSection.has_value();
        const double zoneProjectionTolerance = surfaceSweepSection.has_value()
            ? std::max(1.0e-5, std::max(surfaceSweepSection->geometry.yLength,
                surfaceSweepSection->geometry.zWidth) * 1.0e-6)
            : 0.0;

        ProcessPlan plan;
        plan.contentRevision = input.contentRevision;
        plan.processStateRevision = input.processStateRevision;
        plan.mode = ProcessPlanMode::Rotary4Axis;
        plan.orderingStrategy = policy.orderingStrategy;
        std::unordered_map<EntityId, const PlanningEntity*> entities;
        std::unordered_set<EntityId> seen;

        std::vector<EntityId> ordinaryIds;
        std::map<std::pair<int, BoundaryRole>, std::vector<EntityId>> boundaryIds;
        for (const PlanningEntity& entity : input.entities)
        {
            if (entity.entityId == 0U || !seen.insert(entity.entityId).second)
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::InvalidInput, context, DiagnosticCode::ProcessPlanningInputInvalid,
                    QStringLiteral("加工计划包含无效或重复图元编号。"), QStringLiteral("EntityId is zero or duplicated."),
                    diagnosticValues(input, policy, entity.entityId, entity.sourceIndex)
                );
            }
            entities.emplace(entity.entityId, &entity);
            const ProcessExclusionReason reason = !entity.visible
                ? ProcessExclusionReason::Hidden
                : !entity.processEnabled
                    ? ProcessExclusionReason::UserDisabled
                    : entity.excludedAsInternalGeometry && entity.boundaryRole == BoundaryRole::None
                        ? ProcessExclusionReason::InternalGeometry
                        : entity.sourceKind == geometry::SourceGeometryKind::Point
                            || entity.sourceKind == geometry::SourceGeometryKind::Unknown
                            ? ProcessExclusionReason::UnsupportedGeometry
                            : entity.path.vertices.size() < 2U
                                ? ProcessExclusionReason::InvalidPath
                                : ProcessExclusionReason::InvalidPath;
            const bool excluded = !entity.visible || !entity.processEnabled
                || (entity.excludedAsInternalGeometry && entity.boundaryRole == BoundaryRole::None)
                || entity.sourceKind == geometry::SourceGeometryKind::Point
                || entity.sourceKind == geometry::SourceGeometryKind::Unknown
                || entity.path.vertices.size() < 2U;
            if (excluded)
            {
                plan.exclusions.push_back({ entity.entityId, reason });
                continue;
            }
            if (entity.boundaryRole != BoundaryRole::None && entity.boundaryPairId >= 0)
                boundaryIds[{ entity.boundaryPairId, entity.boundaryRole }].push_back(entity.entityId);
            else
                ordinaryIds.push_back(entity.entityId);
        }

        std::sort(ordinaryIds.begin(), ordinaryIds.end(), [&entities](EntityId left, EntityId right)
        {
            const PlanningEntity* l = entities.at(left);
            const PlanningEntity* r = entities.at(right);
            return l->sourceIndex != r->sourceIndex ? l->sourceIndex < r->sourceIndex : left < right;
        });

        std::vector<BoundaryIdentity> boundaryIdentities;
        boundaryIdentities.reserve(boundaryIds.size());
        for (const auto& [key, ids] : boundaryIds)
        {
            BoundaryIdentity identity;
            identity.pairId = key.first;
            identity.role = key.second;
            identity.entityIds = ids;
            identity.stableSourceIndex = std::numeric_limits<std::size_t>::max();
            identity.stableEntityId = std::numeric_limits<EntityId>::max();
            for (const EntityId entityId : ids)
            {
                identity.stableSourceIndex = std::min
                    (identity.stableSourceIndex, entities.at(entityId)->sourceIndex);
                identity.stableEntityId = std::min(identity.stableEntityId, entityId);
            }
            boundaryIdentities.push_back(std::move(identity));
        }
        std::sort(boundaryIdentities.begin(), boundaryIdentities.end(),
            [](const BoundaryIdentity& left, const BoundaryIdentity& right)
            {
                if (left.stableSourceIndex != right.stableSourceIndex)
                    return left.stableSourceIndex < right.stableSourceIndex;
                if (left.stableEntityId != right.stableEntityId)
                    return left.stableEntityId < right.stableEntityId;
                return static_cast<int>(left.role) < static_cast<int>(right.role);
            });

        std::vector<BoundaryData> boundaries;
        for (const BoundaryIdentity& identity : boundaryIdentities)
        {
            ProcessGroup group;
            group.groupId = static_cast<int>(plan.groups.size());
            group.kind = identity.role == BoundaryRole::Break
                ? ProcessGroupKind::BreakBoundary
                : ProcessGroupKind::WasteBoundary;
            group.closed = true;
            group.entityIds = identity.entityIds;
            std::sort(group.entityIds.begin(), group.entityIds.end(), [&entities](EntityId left, EntityId right)
            {
                return entities.at(left)->sourceIndex < entities.at(right)->sourceIndex;
            });
            plan.groups.push_back(group);
            if (identity.role == BoundaryRole::Waste)
            {
                for (const EntityId entityId : identity.entityIds)
                    plan.exclusions.push_back({ entityId, ProcessExclusionReason::WasteRegion });
            }

            if (!input.tubeSection.has_value())
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::InvalidInput, context, DiagnosticCode::ProcessPlanningBoundaryInvalid,
                    QStringLiteral("验证加工断面前需要先识别方管截面。"), QStringLiteral("Boundary validation requires TubeSectionModel."),
                    diagnosticValues(input, policy, 0U, 0U, identity.pairId, group.groupId)
                );
            }
            const auto loop = input.topology->extractBestLoop
                (identity.entityIds, identity.entityIds);
            if (!loop.succeeded() || !loop.value.has_value() || !loop.value->connectedLoop
                || !sameEntitySet(loop.value->usedEntityIds, identity.entityIds))
            {
                auto result = failure<ProcessPlan>
                (
                    OperationStatus::Failed, context, DiagnosticCode::ProcessPlanningBoundaryInvalid,
                    QStringLiteral("加工断面无法形成包含全部指定图元的严格闭环。"), QStringLiteral("Boundary strict-loop extraction failed."),
                    diagnosticValues(input, policy, 0U, 0U, identity.pairId, group.groupId)
                );
                result.mergeDiagnostics(loop.diagnostics);
                return result;
            }
            const double surfaceMappingTolerance = std::clamp
                (policy.connectionTolerance * 0.01, 1.0e-4, 0.01);
            const auto analysis = TubeCutBoundaryClassifier::analyze
            (
                loop.value->orderedPath,
                loop.value->usedEntityIds,
                loop.value->maximumJoinGap,
                *planningSection,
                context,
                surfaceMappingTolerance
            );
            if (!analysis.succeeded() || !analysis.value.has_value())
            {
                auto result = failure<ProcessPlan>
                (
                    OperationStatus::Failed, context, DiagnosticCode::ProcessPlanningBoundaryInvalid,
                    QStringLiteral("加工断面重新验证失败，无法建立安全加工计划。"), QStringLiteral("TubeCutBoundaryClassifier analysis failed."),
                    diagnosticValues(input, policy, 0U, 0U, identity.pairId, group.groupId)
                );
                result.mergeDiagnostics(analysis.diagnostics);
                return result;
            }
            if (analysis.value->result == TubeCutResult::Indeterminate)
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::Failed, context, DiagnosticCode::ProcessPlanningBoundaryInvalid,
                    QStringLiteral("加工断面判定不确定，无法建立安全加工计划。"), QStringLiteral("TubeCutBoundaryClassifier returned Indeterminate."),
                    diagnosticValues(input, policy, 0U, 0U, identity.pairId, group.groupId)
                );
            }
            if (analysis.value->result == TubeCutResult::KeepsLeftAndRight)
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::Failed, context, DiagnosticCode::ProcessPlanningBoundaryKeepsConnected,
                    QStringLiteral("加工断面仍会保留左右材料桥，不能作为中断切面。"), QStringLiteral("Boundary winding is zero."),
                    diagnosticValues(input, policy, 0U, 0U, identity.pairId, group.groupId)
                );
            }
            boundaries.push_back
                ({ group.groupId, identity.pairId, identity.role, *analysis.value });
        }

        if (!ordinaryIds.empty())
        {
            const std::vector<int> componentIds = input.topology->componentIds(ordinaryIds);
            if (componentIds.size() != ordinaryIds.size())
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::Failed, context, DiagnosticCode::ProcessPlanningGroupBuildFailed,
                    QStringLiteral("普通加工图元的拓扑分组失败。"), QStringLiteral("componentIds size mismatch."),
                    diagnosticValues(input, policy)
                );
            }
            std::map<int, std::vector<EntityId>> components;
            int syntheticComponent = -1;
            for (std::size_t index = 0; index < ordinaryIds.size(); ++index)
            {
                const int componentId = componentIds[index] >= 0 ? componentIds[index] : syntheticComponent--;
                components[componentId].push_back(ordinaryIds[index]);
            }
            const auto appendOrdinaryGroup =
                [&plan, &entities]
                (const std::vector<EntityId>& ids, bool closed)
            {
                if (ids.empty()) return;
                ProcessGroup group;
                group.groupId = static_cast<int>(plan.groups.size());
                group.entityIds = ids;
                group.closed = closed
                    || (ids.size() == 1U
                        && entities.at(ids.front())->path.closed);
                group.kind = group.closed
                    ? ProcessGroupKind::ClosedLoop
                    : ids.size() == 1U
                        ? ProcessGroupKind::SingleEntity
                        : ProcessGroupKind::ConnectedChain;
                plan.groups.push_back(std::move(group));
            };
            for (auto& [componentId, ids] : components)
            {
                (void)componentId;
                if (!policy.preserveClosedLoopsAsAtomicGroups)
                {
                    appendOrdinaryGroup(ids, false);
                    continue;
                }

                std::vector<std::vector<EntityId>> pendingComponents
                    { std::move(ids) };
                for (std::size_t pendingIndex = 0U;
                    pendingIndex < pendingComponents.size(); ++pendingIndex)
                {
                    std::vector<EntityId> currentIds =
                        std::move(pendingComponents[pendingIndex]);
                    const auto loop = input.topology->extractBestLoop
                        (currentIds, currentIds);
                    if (!loop.succeeded() || !loop.value.has_value()
                        || !loop.value->connectedLoop
                        || loop.value->usedEntityIds.empty())
                    {
                        appendOrdinaryGroup(currentIds, false);
                        continue;
                    }

                    const std::unordered_set<EntityId> loopIds
                        (loop.value->usedEntityIds.cbegin(),
                            loop.value->usedEntityIds.cend());
                    std::vector<EntityId> closedIds;
                    std::vector<EntityId> remainingIds;
                    closedIds.reserve(loopIds.size());
                    remainingIds.reserve(currentIds.size() - loopIds.size());
                    for (const EntityId entityId : currentIds)
                    {
                        (loopIds.count(entityId) != 0U
                            ? closedIds : remainingIds).push_back(entityId);
                    }
                    if (closedIds.empty())
                    {
                        appendOrdinaryGroup(currentIds, false);
                        continue;
                    }
                    appendOrdinaryGroup(closedIds, true);
                    if (remainingIds.empty()) continue;

                    const std::vector<int> remainingComponentIds =
                        input.topology->componentIds(remainingIds);
                    if (remainingComponentIds.size() != remainingIds.size())
                    {
                        appendOrdinaryGroup(remainingIds, false);
                        continue;
                    }
                    std::map<int, std::vector<EntityId>> remainingComponents;
                    int remainingSyntheticComponent = -1;
                    for (std::size_t index = 0U;
                        index < remainingIds.size(); ++index)
                    {
                        const int remainingComponentId =
                            remainingComponentIds[index] >= 0
                            ? remainingComponentIds[index]
                            : remainingSyntheticComponent--;
                        remainingComponents[remainingComponentId].push_back
                            (remainingIds[index]);
                    }
                    for (auto& [remainingComponentId, remaining] :
                        remainingComponents)
                    {
                        (void)remainingComponentId;
                        pendingComponents.push_back(std::move(remaining));
                    }
                }
            }
        }

        std::unordered_set<int> excludedGroups;
        for (const ProcessGroup& group : plan.groups)
            if (group.kind == ProcessGroupKind::WasteBoundary) excludedGroups.insert(group.groupId);

        QVector<Diagnostic> zoneSweepDiagnostics;
        std::unordered_map<int, ProcessGroupZoneProfile> groupZoneProfiles;
        if (zone16SweepEnabled)
        {
            groupZoneProfiles.reserve(plan.groups.size());
            for (const ProcessGroup& group : plan.groups)
            {
                if (group.kind == ProcessGroupKind::BreakBoundary
                    || group.kind == ProcessGroupKind::WasteBoundary)
                    continue;

                std::vector<geometry::Path3D> paths;
                paths.reserve(group.entityIds.size());
                for (const EntityId entityId : group.entityIds)
                {
                    paths.push_back(entities.at(entityId)->path);
                }
                auto profileResult = machining::TubeSectionProjector::buildProfile
                    (*surfaceSweepSection, paths, group.closed,
                        zoneProjectionTolerance, context);
                if (!profileResult.succeeded() || !profileResult.value.has_value())
                {
                    auto failed = failure<ProcessPlan>
                    (
                        OperationStatus::Failed, context,
                        DiagnosticCode::ProcessPlanningZoneSweepProfileInvalid,
                        QStringLiteral("加工单元无法建立可靠的方管 16 区位画像。"),
                        QStringLiteral("Static ProcessGroup zone profile construction failed."),
                        diagnosticValues(input, policy, 0U, 0U, -1,
                            group.groupId)
                    );
                    failed.mergeDiagnostics(profileResult.diagnostics);
                    return failed;
                }
                zoneSweepDiagnostics += profileResult.diagnostics;

                const machining::ProcessUnitZoneProfile& sourceProfile =
                    *profileResult.value;
                ProcessGroupZoneProfile profile;
                profile.certainMask = sourceProfile.certainMask;
                profile.possibleMask = sourceProfile.possibleMask;
                profile.zoneSpans = sourceProfile.zoneSpans;
                profile.closed = group.closed;
                profile.uncertain = sourceProfile.uncertain;
                for (std::size_t zoneIndex = 0U;
                    zoneIndex < machining::kTubeZone16Count;
                    ++zoneIndex)
                {
                    TraversalSelectionContext selection;
                    selection.requiredEntryZone =
                        static_cast<machining::TubeZone16>(zoneIndex);
                    selection.longitudinalDirection =
                        policy.zone16Sweep.longitudinalDirection
                            == LongitudinalSweepDirection::PositiveX
                        ? 1 : -1;
                    selection.zoneHitX = selection.longitudinalDirection >= 0
                        ? profile.zoneSpans[zoneIndex].minimumX
                        : profile.zoneSpans[zoneIndex].maximumX;
                    selection.frontierX = selection.zoneHitX;
                    selection.projectionTolerance =
                        zoneProjectionTolerance;
                    selection.previousEnd = policy.initialPosition;
                    selection.hardZoneConstraint = true;
                    ClosedLoopTraversalReport ignoredLoopReport;
                    auto entryTraversal = bestTraversal
                    (
                        group, entities, policy.initialPosition, policy,
                        input.tubeSection, input.tubeSectionCenter,
                        ProcessOrderingStrategy::LazyRotation,
                        &ignoredLoopReport, &selection
                    );
                    if (!entryTraversal.has_value()
                        || !entryTraversal->selectedEntry.has_value())
                    {
                        continue;
                    }
                    profile.entryCandidates[zoneIndex].push_back
                        (*entryTraversal->selectedEntry);
                    profile.entryCandidateCounts[zoneIndex] =
                        entryTraversal->entryCandidateCount;
                    const machining::TubeZone16 zone =
                        static_cast<machining::TubeZone16>(zoneIndex);
                    const machining::TubeZoneMask zoneBit =
                        machining::tubeZoneBit(zone);
                    if (group.kind == ProcessGroupKind::ClosedLoop
                        && group.entityIds.size() > 1U)
                    {
                        if (entryTraversal->connectionCandidateCount > 0)
                            profile.connectionEntryMask |= zoneBit;
                        if (entryTraversal->arcInteriorCandidateCount > 0
                            || entryTraversal
                                ->ellipseInteriorCandidateCount > 0)
                        {
                            profile.curveInteriorEntryMask |= zoneBit;
                        }
                        profile.arcMemberIdsByZone[zoneIndex] =
                            entryTraversal
                                ->arcInteriorCandidateEntityIds;
                        profile.ellipseMemberIdsByZone[zoneIndex] =
                            entryTraversal
                                ->ellipseInteriorCandidateEntityIds;
                        profile.legalEntryMask =
                            profile.connectionEntryMask
                            | profile.curveInteriorEntryMask
                            | profile.zoneRunMidpointEntryMask;
                    }
                    else
                    {
                        profile.legalEntryMask |= zoneBit;
                    }
                }
                if ((profile.certainMask & ~profile.possibleMask) != 0U
                    || profile.possibleMask == 0U)
                {
                    QVariantMap values = diagnosticValues
                        (input, policy, 0U, 0U, -1, group.groupId);
                    values.insert(QStringLiteral("unitKey"),
                        processGroupKeyText(group));
                    values.insert(QStringLiteral("certainMask"),
                        zoneMaskText(profile.certainMask));
                    values.insert(QStringLiteral("possibleMask"),
                        zoneMaskText(profile.possibleMask));
                    values.insert(QStringLiteral("legalEntryMask"),
                        zoneMaskText(profile.legalEntryMask));
                    return failure<ProcessPlan>
                    (
                        OperationStatus::Failed,
                        context,
                        DiagnosticCode::ProcessPlanningZoneSweepProfileInvalid,
                        QStringLiteral("加工单元没有可用于 16 区位调度的可靠投影。"),
                        QStringLiteral("Zone occupancy masks are inconsistent or empty."),
                        values
                    );
                }
                profile.schedulableMask = profile.certainMask != 0U
                    ? profile.certainMask : profile.possibleMask;
                QVariantMap entryValues;
                entryValues.insert(QStringLiteral("entryZoneProfile"), true);
                entryValues.insert(QStringLiteral("unitKey"),
                    processGroupKeyText(group));
                entryValues.insert(QStringLiteral("groupKind"),
                    groupKindName(group.kind));
                entryValues.insert(QStringLiteral("certainMask"),
                    zoneMaskText(profile.certainMask));
                entryValues.insert(QStringLiteral("possibleMask"),
                    zoneMaskText(profile.possibleMask));
                entryValues.insert(QStringLiteral("legalEntryMask"),
                    zoneMaskText(profile.legalEntryMask));
                entryValues.insert(QStringLiteral("connectionEntryMask"),
                    zoneMaskText(profile.connectionEntryMask));
                entryValues.insert(QStringLiteral("curveInteriorEntryMask"),
                    zoneMaskText(profile.curveInteriorEntryMask));
                entryValues.insert(QStringLiteral("zoneRunMidpointEntryMask"),
                    zoneMaskText(profile.zoneRunMidpointEntryMask));
                QStringList memberSourceKinds;
                for (const EntityId entityId : group.entityIds)
                {
                    const auto entity = entities.find(entityId);
                    memberSourceKinds.push_back(QStringLiteral("%1:%2")
                        .arg(entityId)
                        .arg(entity != entities.end()
                            && entity->second != nullptr
                            ? planningSourceKindName
                                (entity->second->sourceKind)
                            : QStringLiteral("Missing")));
                }
                entryValues.insert(QStringLiteral("memberSourceKinds"),
                    memberSourceKinds.join(QLatin1Char(',')));
                QStringList counts;
                QStringList arcMembers;
                QStringList ellipseMembers;
                for (std::size_t zoneIndex = 0U;
                    zoneIndex < machining::kTubeZone16Count;
                    ++zoneIndex)
                {
                    const QString zoneName = machining::tubeZoneName
                        (static_cast<machining::TubeZone16>(zoneIndex));
                    counts.push_back(QStringLiteral("%1:%2")
                        .arg(zoneName)
                        .arg(profile.entryCandidateCounts[zoneIndex]));
                    auto idsText = [](const std::vector<EntityId>& ids)
                    {
                        QStringList values;
                        for (const EntityId id : ids)
                            values.push_back(QString::number(id));
                        return values.join(QLatin1Char(','));
                    };
                    arcMembers.push_back(QStringLiteral("%1:%2")
                        .arg(zoneName)
                        .arg(idsText
                            (profile.arcMemberIdsByZone[zoneIndex])));
                    ellipseMembers.push_back(QStringLiteral("%1:%2")
                        .arg(zoneName)
                        .arg(idsText
                            (profile.ellipseMemberIdsByZone[zoneIndex])));
                }
                entryValues.insert(QStringLiteral("candidateCountsByZone"),
                    counts.join(QLatin1Char(',')));
                entryValues.insert(QStringLiteral("arcMemberIdsByZone"),
                    arcMembers.join(QLatin1Char(';')));
                entryValues.insert(QStringLiteral("ellipseMemberIdsByZone"),
                    ellipseMembers.join(QLatin1Char(';')));
                zoneSweepDiagnostics.push_back(planningDiagnostic
                (
                    context,
                    DiagnosticCode::ProcessPlanningEntrySelectionSummary,
                    QStringLiteral("加工单元已建立合法入口区位画像。"),
                    QStringLiteral("Executable entry candidates were classified independently from occupancy."),
                    entryValues,
                    DiagnosticSeverity::Info
                ));
                groupZoneProfiles.emplace(group.groupId, std::move(profile));
            }
        }

        std::vector<std::vector<int>> boundarySuccessors(boundaries.size());
        std::vector<int> boundaryIndegree(boundaries.size(), 0);
        for (std::size_t leftIndex = 0; leftIndex < boundaries.size(); ++leftIndex)
        {
            for (std::size_t rightIndex = leftIndex + 1U;
                rightIndex < boundaries.size(); ++rightIndex)
            {
                const BoundaryData& leftBoundary = boundaries[leftIndex];
                const BoundaryData& rightBoundary = boundaries[rightIndex];
                const ProcessGroup& leftGroup = plan.groups
                    [static_cast<std::size_t>(leftBoundary.groupId)];
                const ProcessGroup& rightGroup = plan.groups
                    [static_cast<std::size_t>(rightBoundary.groupId)];
                const BoundarySide leftRelativeToRight = classifyGroup
                (
                    leftGroup, entities, rightBoundary,
                    *planningSection, policy.connectionTolerance
                );
                const BoundarySide rightRelativeToLeft = classifyGroup
                (
                    rightGroup, entities, leftBoundary,
                    *planningSection, policy.connectionTolerance
                );
                const bool leftBeforeRight = leftRelativeToRight == BoundarySide::Left
                    && rightRelativeToLeft == BoundarySide::Right;
                const bool rightBeforeLeft = leftRelativeToRight == BoundarySide::Right
                    && rightRelativeToLeft == BoundarySide::Left;
                if (!leftBeforeRight && !rightBeforeLeft)
                {
                    const bool crossing = leftRelativeToRight == BoundarySide::Mixed
                        || rightRelativeToLeft == BoundarySide::Mixed;
                    const bool overlapping = leftRelativeToRight == BoundarySide::OnBoundary
                        || rightRelativeToLeft == BoundarySide::OnBoundary;
                    return failure<ProcessPlan>
                    (
                        OperationStatus::Failed, context,
                        DiagnosticCode::ProcessPlanningBoundaryClassificationFailed,
                        crossing
                            ? QStringLiteral("两个加工断面相互交叉，无法确定空间顺序。")
                            : overlapping
                                ? QStringLiteral("两个加工断面重合，无法建立独立工艺屏障。")
                                : QStringLiteral("两个加工断面无法严格区分左右空间关系。"),
                        QStringLiteral("Bidirectional boundary classification is not a strict Left/Right pair."),
                        boundaryDiagnosticValues
                        (
                            input, policy, rightBoundary, leftGroup, entities, -1,
                            leftRelativeToRight, rightRelativeToLeft
                        )
                    );
                }

                const std::size_t predecessor = leftBeforeRight ? leftIndex : rightIndex;
                const std::size_t successor = leftBeforeRight ? rightIndex : leftIndex;
                boundarySuccessors[predecessor].push_back(static_cast<int>(successor));
                ++boundaryIndegree[successor];
            }
        }

        std::vector<int> boundaryOrder;
        boundaryOrder.reserve(boundaries.size());
        std::vector<bool> boundaryScheduled(boundaries.size(), false);
        while (boundaryOrder.size() < boundaries.size())
        {
            std::vector<int> eligibleBoundaries;
            for (std::size_t index = 0; index < boundaries.size(); ++index)
                if (!boundaryScheduled[index] && boundaryIndegree[index] == 0)
                    eligibleBoundaries.push_back(static_cast<int>(index));
            if (eligibleBoundaries.size() != 1U)
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::Failed, context,
                    DiagnosticCode::ProcessPlanningBoundaryClassificationFailed,
                    eligibleBoundaries.empty()
                        ? QStringLiteral("多断面空间关系形成循环，无法生成加工计划。")
                        : QStringLiteral("多个加工断面缺少唯一左右顺序，无法生成加工计划。"),
                    QStringLiteral("Boundary spatial relation graph is cyclic or not uniquely ordered."),
                    diagnosticValues(input, policy, 0U, 0U, -1, -1, -1, -1,
                        static_cast<int>(boundaries.size()),
                        static_cast<int>(eligibleBoundaries.size()))
                );
            }
            const int selectedBoundary = eligibleBoundaries.front();
            boundaryScheduled[static_cast<std::size_t>(selectedBoundary)] = true;
            boundaryOrder.push_back(selectedBoundary);
            for (const int successor : boundarySuccessors[static_cast<std::size_t>(selectedBoundary)])
                --boundaryIndegree[static_cast<std::size_t>(successor)];
        }

        std::unordered_map<int, int> boundaryRankByGroup;
        for (std::size_t rank = 0; rank < boundaryOrder.size(); ++rank)
        {
            const std::size_t boundaryIndex = static_cast<std::size_t>(boundaryOrder[rank]);
            boundaryRankByGroup[boundaries[boundaryIndex].groupId] = static_cast<int>(rank);
        }

        std::unordered_map<int, std::vector<BoundarySide>> groupBoundarySides;
        for (const ProcessGroup& group : plan.groups)
        {
            if (group.kind == ProcessGroupKind::BreakBoundary
                || group.kind == ProcessGroupKind::WasteBoundary) continue;
            std::vector<BoundarySide> sides(boundaries.size(), BoundarySide::Indeterminate);
            for (std::size_t boundaryIndex = 0; boundaryIndex < boundaries.size(); ++boundaryIndex)
            {
                const BoundaryData& boundary = boundaries[boundaryIndex];
                const BoundarySide side = classifyGroup
                (
                    group, entities, boundary, *planningSection,
                    policy.connectionTolerance
                );
                if (side == BoundarySide::Mixed || side == BoundarySide::Indeterminate
                    || side == BoundarySide::OnBoundary)
                {
                    return failure<ProcessPlan>
                    (
                        OperationStatus::Failed, context,
                        DiagnosticCode::ProcessPlanningBoundaryClassificationFailed,
                        side == BoundarySide::Mixed
                            ? QStringLiteral("加工组跨越加工断面，无法建立安全加工计划。")
                            : side == BoundarySide::OnBoundary
                                ? QStringLiteral("普通加工组落在加工断面上，无法建立安全加工计划。")
                                : QStringLiteral("加工组相对加工断面的侧别无法确定。"),
                        QStringLiteral("Ordinary group classification is not strictly Left or Right."),
                        boundaryDiagnosticValues
                        (
                            input, policy, boundary, group, entities,
                            boundaryRankByGroup[boundary.groupId], side
                        )
                    );
                }
                sides[boundaryIndex] = side;
            }
            bool enteredLeftSide = false;
            for (const int orderedIndex : boundaryOrder)
            {
                const BoundarySide side = sides[static_cast<std::size_t>(orderedIndex)];
                if (side == BoundarySide::Left) enteredLeftSide = true;
                else if (enteredLeftSide)
                {
                    const BoundaryData& boundary = boundaries
                        [static_cast<std::size_t>(orderedIndex)];
                    return failure<ProcessPlan>
                    (
                        OperationStatus::Failed, context,
                        DiagnosticCode::ProcessPlanningBoundaryClassificationFailed,
                        QStringLiteral("加工组相对多个断面的侧别不满足连续空间分区。"),
                        QStringLiteral("Boundary side pattern is not monotonic Right* then Left*."),
                        boundaryDiagnosticValues
                        (
                            input, policy, boundary, group, entities,
                            boundaryRankByGroup[boundary.groupId], side
                        )
                    );
                }
            }
            groupBoundarySides.emplace(group.groupId, std::move(sides));
        }

        // Waste intervals reuse the same strict spatial order as production barriers.
        if (boundaries.size() >= 2U)
        {
            for (const ProcessGroup& group : plan.groups)
            {
                if (group.kind == ProcessGroupKind::BreakBoundary
                    || group.kind == ProcessGroupKind::WasteBoundary) continue;
                const auto sides = groupBoundarySides.find(group.groupId);
                if (sides == groupBoundarySides.end()) continue;
                for (std::size_t rank = 0; rank + 1U < boundaryOrder.size(); ++rank)
                {
                    const std::size_t leftIndex = static_cast<std::size_t>(boundaryOrder[rank]);
                    const std::size_t rightIndex = static_cast<std::size_t>(boundaryOrder[rank + 1U]);
                    const BoundaryData& leftBoundary = boundaries[leftIndex];
                    const BoundaryData& rightBoundary = boundaries[rightIndex];
                    if (leftBoundary.role != BoundaryRole::Waste
                        && rightBoundary.role != BoundaryRole::Waste) continue;
                    if (sides->second[leftIndex] == BoundarySide::Right
                        && sides->second[rightIndex] == BoundarySide::Left)
                    {
                        excludedGroups.insert(group.groupId);
                        for (const EntityId entityId : group.entityIds)
                            plan.exclusions.push_back
                                ({ entityId, ProcessExclusionReason::WasteRegion });
                        break;
                    }
                }
            }
        }

        std::set<std::pair<int, int>> precedencePairs;
        for (std::size_t boundaryIndex = 0; boundaryIndex < boundaries.size(); ++boundaryIndex)
        {
            const BoundaryData& boundary = boundaries[boundaryIndex];
            if (boundary.role != BoundaryRole::Break) continue;
            for (const ProcessGroup& group : plan.groups)
            {
                if (group.groupId == boundary.groupId || excludedGroups.find(group.groupId) != excludedGroups.end()) continue;
                BoundarySide side = BoundarySide::Indeterminate;
                if (group.kind == ProcessGroupKind::BreakBoundary
                    || group.kind == ProcessGroupKind::WasteBoundary)
                {
                    side = boundaryRankByGroup.at(group.groupId)
                        < boundaryRankByGroup.at(boundary.groupId)
                        ? BoundarySide::Left : BoundarySide::Right;
                }
                else
                {
                    const auto sides = groupBoundarySides.find(group.groupId);
                    if (sides != groupBoundarySides.end()) side = sides->second[boundaryIndex];
                }
                const int predecessor = side == BoundarySide::Left
                    ? group.groupId : boundary.groupId;
                const int successor = side == BoundarySide::Left
                    ? boundary.groupId : group.groupId;
                if (precedencePairs.insert({ predecessor, successor }).second)
                    plan.precedenceConstraints.push_back
                        ({ predecessor, successor, boundary.pairId });
            }
        }

        std::unordered_map<int, int> indegree;
        std::unordered_map<int, std::vector<int>> successors;
        std::unordered_set<int> schedulable;
        for (const ProcessGroup& group : plan.groups)
        {
            if (excludedGroups.find(group.groupId) == excludedGroups.end())
            {
                schedulable.insert(group.groupId);
                indegree[group.groupId] = 0;
            }
        }
        for (const ProcessPrecedence& precedence : plan.precedenceConstraints)
        {
            ++indegree[precedence.successorGroupId];
            successors[precedence.predecessorGroupId].push_back(precedence.successorGroupId);
        }

        std::vector<TubeZoneSweepPartition> zoneSweepPartitions;
        std::unordered_map<int, int> partitionAfterBreakGroup;
        if (zone16SweepEnabled)
        {
            std::vector<std::size_t> orderedBreakBoundaryIndices;
            for (const int orderedIndex : boundaryOrder)
            {
                const std::size_t boundaryIndex =
                    static_cast<std::size_t>(orderedIndex);
                if (boundaries[boundaryIndex].role == BoundaryRole::Break)
                    orderedBreakBoundaryIndices.push_back(boundaryIndex);
            }
            zoneSweepPartitions.resize(orderedBreakBoundaryIndices.size() + 1U);
            std::vector<bool> partitionBoundsInitialized
                (zoneSweepPartitions.size(), false);
            for (std::size_t partitionIndex = 0U;
                partitionIndex < zoneSweepPartitions.size(); ++partitionIndex)
            {
                zoneSweepPartitions[partitionIndex].partitionId =
                    static_cast<int>(partitionIndex);
                zoneSweepPartitions[partitionIndex].initialZone =
                    policy.zone16Sweep.initialZone;
                zoneSweepPartitions[partitionIndex].perimeterDirection =
                    policy.zone16Sweep.perimeterDirection
                        == PerimeterSweepDirection::Clockwise ? 1 : -1;
                zoneSweepPartitions[partitionIndex].longitudinalDirection =
                    policy.zone16Sweep.longitudinalDirection
                        == LongitudinalSweepDirection::PositiveX ? 1 : -1;
            }
            for (std::size_t breakIndex = 0U;
                breakIndex < orderedBreakBoundaryIndices.size(); ++breakIndex)
            {
                partitionAfterBreakGroup.emplace
                (
                    boundaries[orderedBreakBoundaryIndices[breakIndex]].groupId,
                    static_cast<int>(breakIndex + 1U)
                );
            }

            for (const ProcessGroup& group : plan.groups)
            {
                if (group.kind == ProcessGroupKind::BreakBoundary
                    || group.kind == ProcessGroupKind::WasteBoundary
                    || excludedGroups.find(group.groupId) != excludedGroups.end())
                    continue;
                const auto sides = groupBoundarySides.find(group.groupId);
                if (sides == groupBoundarySides.end()
                    || groupZoneProfiles.find(group.groupId)
                        == groupZoneProfiles.end())
                {
                    return failure<ProcessPlan>
                    (
                        OperationStatus::Failed, context,
                        DiagnosticCode::ProcessPlanningZoneSweepProfileInvalid,
                        QStringLiteral("加工单元缺少 16 区位加工段数据。"),
                        QStringLiteral("Zone16 partition input is missing group side or profile data."),
                        diagnosticValues(input, policy, 0U, 0U, -1,
                            group.groupId)
                    );
                }

                std::size_t partitionIndex = 0U;
                for (const std::size_t boundaryIndex :
                    orderedBreakBoundaryIndices)
                {
                    if (sides->second[boundaryIndex] == BoundarySide::Right)
                        ++partitionIndex;
                }
                if (partitionIndex >= zoneSweepPartitions.size())
                {
                    return failure<ProcessPlan>
                    (
                        OperationStatus::InternalError, context,
                        DiagnosticCode::ProcessPlanningInvariantViolation,
                        QStringLiteral("加工单元无法映射到有效的 16 区位加工段。"),
                        QStringLiteral("Computed partition index exceeds partition count."),
                        diagnosticValues(input, policy, 0U, 0U, -1,
                            group.groupId)
                    );
                }

                TubeZoneSweepPartition& partition =
                    zoneSweepPartitions[partitionIndex];
                partition.groupIds.insert(group.groupId);
                const ProcessGroupZoneProfile& profile =
                    groupZoneProfiles.at(group.groupId);
                const machining::TubeZoneMask boundsMask =
                    profile.schedulableMask;
                for (std::size_t zoneIndex = 0U;
                    zoneIndex < machining::kTubeZone16Count; ++zoneIndex)
                {
                    const auto zone =
                        static_cast<machining::TubeZone16>(zoneIndex);
                    if ((boundsMask & machining::tubeZoneBit(zone)) == 0U)
                        continue;
                    const machining::TubeZoneSpan& span =
                        profile.zoneSpans[zoneIndex];
                    if (!partitionBoundsInitialized[partitionIndex])
                    {
                        partition.minimumX = span.minimumX;
                        partition.maximumX = span.maximumX;
                        partitionBoundsInitialized[partitionIndex] = true;
                    }
                    else
                    {
                        partition.minimumX = std::min
                            (partition.minimumX, span.minimumX);
                        partition.maximumX = std::max
                            (partition.maximumX, span.maximumX);
                    }
                }
            }
        }
        if (schedulable.empty())
        {
            return failure<ProcessPlan>
            (
                OperationStatus::Failed, context, DiagnosticCode::ProcessPlanningNoProcessableEntities,
                QStringLiteral("当前文档没有可加工图元。"), QStringLiteral("Every entity is excluded."),
                diagnosticValues(input, policy, 0U, 0U, -1, -1, -1, -1, 0, 0,
                    static_cast<int>(plan.groups.size()), 0, static_cast<int>(plan.exclusions.size()))
            );
        }

        std::unordered_set<int> scheduled;
        std::unordered_map<int, int> schedulingOccurrences;
        QVector<Diagnostic> closedLoopDiagnostics;
        Zone16SweepState zoneSweepState;
        Zone16SweepReport zoneSweepReport;
        machining::TubeZone16 currentSweepZone =
            policy.zone16Sweep.initialZone;
        Vector3d currentPosition = policy.initialPosition;
        std::optional<machining::TubeZone16> previousTransferZone;
        int processOrder = 0;
        std::vector<bool> startedPartitions(zoneSweepPartitions.size(), false);
        std::vector<bool> finishedPartitions(zoneSweepPartitions.size(), false);
        std::optional<OperationResult<ProcessPlan>>
            zoneSweepLifecycleFailure;
        const auto setZoneSweepLifecycleFailure =
            [&](DiagnosticCode code, const QString& userMessage,
                const QString& technicalDetail, QVariantMap values)
        {
            zoneSweepLifecycleFailure = failure<ProcessPlan>
            (
                OperationStatus::InternalError, context, code,
                userMessage, technicalDetail, std::move(values)
            );
        };
        const auto finishZoneSweepPartition = [&]() -> bool
        {
            if (!zoneSweepState.active) return true;
            if (zoneSweepState.partitionId < 0
                || static_cast<std::size_t>(zoneSweepState.partitionId)
                    >= zoneSweepPartitions.size())
            {
                setZoneSweepLifecycleFailure
                (
                    DiagnosticCode::ProcessPlanningInvariantViolation,
                    QStringLiteral("16 区位加工段状态无效。"),
                    QStringLiteral("Active zone sweep references an invalid partition."),
                    diagnosticValues(input, policy)
                );
                return false;
            }
            const std::size_t partitionIndex =
                static_cast<std::size_t>(zoneSweepState.partitionId);
            const TubeZoneSweepPartition& partition =
                zoneSweepPartitions[partitionIndex];
            const bool allGroupsScheduled = std::all_of
            (
                partition.groupIds.cbegin(), partition.groupIds.cend(),
                [&scheduled](int groupId)
                {
                    return scheduled.find(groupId) != scheduled.end();
                }
            );
            bool pairedPhases = true;
            for (std::size_t zoneIndex = 0U;
                zoneIndex < machining::kTubeZone16Count; ++zoneIndex)
            {
                if (zoneSweepState.enteredZones[zoneIndex]
                    != zoneSweepState.completedZones[zoneIndex])
                {
                    pairedPhases = false;
                    break;
                }
            }
            if (!allGroupsScheduled || !pairedPhases
                || zoneSweepReport.processedUnitCount
                    != static_cast<int>(partition.groupIds.size())
                || zoneSweepReport.backtrackCount != 0)
            {
                QVariantMap values = diagnosticValues(input, policy);
                values.insert(QStringLiteral("partitionId"),
                    zoneSweepState.partitionId);
                values.insert(QStringLiteral("allGroupsScheduled"),
                    allGroupsScheduled);
                values.insert(QStringLiteral("pairedPhases"), pairedPhases);
                values.insert(QStringLiteral("processedUnitCount"),
                    zoneSweepReport.processedUnitCount);
                values.insert(QStringLiteral("expectedUnitCount"),
                    static_cast<int>(partition.groupIds.size()));
                values.insert(QStringLiteral("backtrackCount"),
                    zoneSweepReport.backtrackCount);
                setZoneSweepLifecycleFailure
                (
                    DiagnosticCode::ProcessPlanningZoneIncomplete,
                    QStringLiteral("16 区位加工段尚未完整完成，不能结束该加工段。"),
                    QStringLiteral("Partition completion requires every owner group and every entered zone phase to be complete."),
                    std::move(values)
                );
                return false;
            }
            if (finishedPartitions[partitionIndex])
            {
                QVariantMap values = diagnosticValues(input, policy);
                values.insert(QStringLiteral("partitionId"),
                    zoneSweepState.partitionId);
                setZoneSweepLifecycleFailure
                (
                    DiagnosticCode::ProcessPlanningInvariantViolation,
                    QStringLiteral("16 区位加工段被重复结束。"),
                    QStringLiteral("A zone sweep partition may finish only once."),
                    std::move(values)
                );
                return false;
            }
            finishedPartitions[partitionIndex] = true;
            zoneSweepDiagnostics.push_back
                (zone16SweepDiagnostic(context, zoneSweepReport));
            zoneSweepState = Zone16SweepState{};
            zoneSweepReport = Zone16SweepReport{};
            return true;
        };
        const auto startZoneSweepPartition =
            [&](int partitionId, machining::TubeZone16 initialZone) -> bool
        {
            zoneSweepLifecycleFailure.reset();
            if (partitionId < 0
                || static_cast<std::size_t>(partitionId)
                    >= zoneSweepPartitions.size())
                return false;
            const std::size_t partitionIndex =
                static_cast<std::size_t>(partitionId);
            if (zoneSweepState.active || startedPartitions[partitionIndex])
            {
                QVariantMap values = diagnosticValues(input, policy);
                values.insert(QStringLiteral("partitionId"), partitionId);
                values.insert(QStringLiteral("partitionActive"),
                    zoneSweepState.active);
                values.insert(QStringLiteral("partitionStarted"),
                    static_cast<bool>(startedPartitions[partitionIndex]));
                values.insert(QStringLiteral("partitionFinished"),
                    static_cast<bool>(finishedPartitions[partitionIndex]));
                setZoneSweepLifecycleFailure
                (
                    zoneSweepState.active
                        ? DiagnosticCode::ProcessPlanningZoneIncomplete
                        : DiagnosticCode::ProcessPlanningInvariantViolation,
                    zoneSweepState.active
                        ? QStringLiteral("当前 16 区位加工段尚未完成，不能启动其他加工段。")
                        : QStringLiteral("同一 16 区位加工段不能重复启动。"),
                    QStringLiteral("A zone sweep partition cannot restart or replace an active partition."),
                    std::move(values)
                );
                return false;
            }

            TubeZoneSweepPartition& partition =
                zoneSweepPartitions[partitionIndex];
            partition.initialZone = initialZone;
            std::vector<int> orderedGroupIds
                (partition.groupIds.cbegin(), partition.groupIds.cend());
            std::sort(orderedGroupIds.begin(), orderedGroupIds.end(),
                [&plan](int left, int right)
            {
                return processGroupStableLess
                (
                    plan.groups[static_cast<std::size_t>(left)],
                    plan.groups[static_cast<std::size_t>(right)]
                );
            });
            for (const int groupId : orderedGroupIds)
            {
                const ProcessGroup& group =
                    plan.groups[static_cast<std::size_t>(groupId)];
                ProcessGroupZoneProfile& profile =
                    groupZoneProfiles.at(groupId);
                const machining::TubeZoneMask strongZones =
                    strongTubeZoneMask();
                const bool canCreateOwnerZoneEntry =
                    group.kind == ProcessGroupKind::ClosedLoop
                    && group.entityIds.size() > 1U;
                const machining::TubeZoneMask requiredEntryMask =
                    canCreateOwnerZoneEntry
                    ? std::numeric_limits<machining::TubeZoneMask>::max()
                    : profile.legalEntryMask;
                machining::TubeZoneMask ownerCandidateMask =
                    profile.certainMask & strongZones & requiredEntryMask;
                bool usedPossibleFallback = false;
                bool usedBoundaryFallback = false;
                if (ownerCandidateMask == 0U)
                {
                    ownerCandidateMask =
                        profile.possibleMask & strongZones
                        & requiredEntryMask;
                    usedPossibleFallback = ownerCandidateMask != 0U;
                }
                if (ownerCandidateMask == 0U
                    && profile.certainMask != 0U)
                {
                    ownerCandidateMask =
                        profile.certainMask & requiredEntryMask;
                    usedBoundaryFallback = true;
                }
                if (ownerCandidateMask == 0U
                    && profile.possibleMask != 0U)
                {
                    ownerCandidateMask =
                        profile.possibleMask & requiredEntryMask;
                    usedPossibleFallback = true;
                    usedBoundaryFallback = true;
                }
                std::optional<machining::TubeZone16> ownerZone =
                    firstZoneInSweep(ownerCandidateMask, initialZone,
                        partition.perimeterDirection);
                if (!ownerZone.has_value())
                {
                    QVariantMap values = diagnosticValues
                        (input, policy, 0U, 0U, -1, groupId);
                    values.insert(QStringLiteral("partitionId"), partitionId);
                    values.insert(QStringLiteral("unitKey"),
                        processGroupKeyText(plan.groups
                            [static_cast<std::size_t>(groupId)]));
                    values.insert(QStringLiteral("groupKind"),
                        groupKindName(group.kind));
                    values.insert(QStringLiteral("memberCount"),
                        static_cast<int>(group.entityIds.size()));
                    values.insert(QStringLiteral("certainMask"),
                        zoneMaskText(profile.certainMask));
                    values.insert(QStringLiteral("possibleMask"),
                        zoneMaskText(profile.possibleMask));
                    values.insert(QStringLiteral("legalEntryMask"),
                        zoneMaskText(profile.legalEntryMask));
                    values.insert(QStringLiteral("requiredEntryMask"),
                        zoneMaskText(requiredEntryMask));
                    values.insert(QStringLiteral("ownerCandidateMask"),
                        zoneMaskText(ownerCandidateMask));
                    setZoneSweepLifecycleFailure
                    (
                        DiagnosticCode::ProcessPlanningZoneSweepProfileInvalid,
                        QStringLiteral("加工单元没有可用的唯一生产区位。"),
                        QStringLiteral("Neither certain nor possible occupancy contains a zone in the partition sweep."),
                        std::move(values)
                    );
                    return false;
                }
                ZoneSweepOwnership ownership;
                ownership.groupId = groupId;
                ownership.ownerZone = *ownerZone;
                ownership.usedPossibleFallback = usedPossibleFallback;
                ownership.usedBoundaryFallback = usedBoundaryFallback;
                ownership.ownerCandidateMask = ownerCandidateMask;
                ownership.legalEntryMaskBefore = profile.legalEntryMask;
                const machining::TubeZoneMask ownerBit =
                    machining::tubeZoneBit(*ownerZone);
                if ((profile.legalEntryMask & ownerBit) == 0U)
                {
                    if (group.kind == ProcessGroupKind::ClosedLoop
                        && group.entityIds.size() > 1U
                        && surfaceSweepSection.has_value())
                    {
                        TraversalSelectionContext selection;
                        selection.requiredEntryZone = *ownerZone;
                        selection.longitudinalDirection =
                            partition.longitudinalDirection;
                        const machining::TubeZoneSpan& ownerSpan =
                            profile.zoneSpans[machining::tubeZoneIndex
                                (*ownerZone)];
                        selection.zoneHitX =
                            selection.longitudinalDirection >= 0
                            ? ownerSpan.minimumX : ownerSpan.maximumX;
                        selection.frontierX = selection.zoneHitX;
                        selection.projectionTolerance =
                            zoneProjectionTolerance;
                        selection.previousEnd = currentPosition;
                        selection.hardZoneConstraint = true;
                        selection.allowZoneRunMidpointFallback = true;
                        ClosedLoopTraversalReport ignoredLoopReport;
                        auto ownerEntry = bestTraversal
                        (
                            group, entities, currentPosition, policy,
                            input.tubeSection, input.tubeSectionCenter,
                            ProcessOrderingStrategy::LazyRotation,
                            &ignoredLoopReport, &selection
                        );
                        if (ownerEntry.has_value()
                            && ownerEntry->selectedEntry.has_value()
                            && ownerEntry->selectedEntry->zone == *ownerZone)
                        {
                            const std::size_t ownerIndex =
                                machining::tubeZoneIndex(*ownerZone);
                            profile.entryCandidates[ownerIndex].push_back
                                (*ownerEntry->selectedEntry);
                            profile.entryCandidateCounts[ownerIndex] =
                                ownerEntry->entryCandidateCount;
                            switch (ownerEntry->selectedEntry->kind)
                            {
                            case ZoneEntryCandidateKind::
                                ClosedLoopArcInterior:
                            case ZoneEntryCandidateKind::
                                ClosedLoopEllipseInterior:
                                profile.curveInteriorEntryMask |= ownerBit;
                                break;
                            case ZoneEntryCandidateKind::
                                ClosedLoopConnection:
                                profile.connectionEntryMask |= ownerBit;
                                break;
                            case ZoneEntryCandidateKind::
                                ClosedLoopZoneRunMidpoint:
                                profile.zoneRunMidpointEntryMask |= ownerBit;
                                break;
                            default:
                                break;
                            }
                            profile.legalEntryMask =
                                profile.connectionEntryMask
                                | profile.curveInteriorEntryMask
                                | profile.zoneRunMidpointEntryMask;
                            const QString unitKey =
                                processGroupKeyText(group);
                            for (Diagnostic& diagnostic :
                                zoneSweepDiagnostics)
                            {
                                if (!diagnostic.context.value
                                        (QStringLiteral("entryZoneProfile"))
                                        .toBool()
                                    || diagnostic.context.value
                                        (QStringLiteral("unitKey"))
                                        .toString() != unitKey)
                                {
                                    continue;
                                }
                                diagnostic.context.insert
                                (
                                    QStringLiteral(
                                        "zoneRunMidpointEntryMask"),
                                    zoneMaskText(profile
                                        .zoneRunMidpointEntryMask)
                                );
                                diagnostic.context.insert
                                (
                                    QStringLiteral("legalEntryMask"),
                                    zoneMaskText(profile.legalEntryMask)
                                );
                                break;
                            }
                        }
                    }
                    if ((profile.legalEntryMask & ownerBit) == 0U)
                    {
                        QVariantMap values = diagnosticValues
                            (input, policy, 0U, 0U, -1, groupId);
                        values.insert(QStringLiteral("partitionId"),
                            partitionId);
                        values.insert(QStringLiteral("unitKey"),
                            processGroupKeyText(plan.groups
                                [static_cast<std::size_t>(groupId)]));
                        values.insert(QStringLiteral("ownerZone"),
                            machining::tubeZoneName(*ownerZone));
                        values.insert(QStringLiteral("certainMask"),
                            zoneMaskText(profile.certainMask));
                        values.insert(QStringLiteral("possibleMask"),
                            zoneMaskText(profile.possibleMask));
                        values.insert(QStringLiteral("legalEntryMaskBefore"),
                            zoneMaskText(ownership.legalEntryMaskBefore));
                        setZoneSweepLifecycleFailure
                        (
                            DiagnosticCode::
                                ProcessPlanningOwnerZoneEntryUnavailable,
                            QStringLiteral("加工单元无法在所属区位建立合法起刀点。"),
                            QStringLiteral("The immutable owner zone has no legal connection, curve-interior, or reliable run-midpoint entry."),
                            std::move(values)
                        );
                        return false;
                    }
                }
                partition.ownerships.emplace(groupId, ownership);
                partition.zoneBuckets[machining::tubeZoneIndex(*ownerZone)]
                    .push_back(groupId);
                zoneSweepDiagnostics.push_back(zoneOwnershipDiagnostic
                (
                    context, partition,
                    plan.groups[static_cast<std::size_t>(groupId)],
                    profile, ownership
                ));
            }
            std::unordered_map<int, int> bucketOccurrences;
            for (auto& bucket : partition.zoneBuckets)
            {
                std::sort(bucket.begin(), bucket.end(),
                    [&plan](int left, int right)
                {
                    return processGroupStableLess
                    (
                        plan.groups[static_cast<std::size_t>(left)],
                        plan.groups[static_cast<std::size_t>(right)]
                    );
                });
                for (const int groupId : bucket)
                    ++bucketOccurrences[groupId];
            }
            for (const int groupId : orderedGroupIds)
            {
                if (partition.ownerships.count(groupId) != 1U
                    || bucketOccurrences[groupId] != 1)
                {
                    QVariantMap values = diagnosticValues
                        (input, policy, 0U, 0U, -1, groupId);
                    values.insert(QStringLiteral("partitionId"), partitionId);
                    values.insert(QStringLiteral("ownershipCount"),
                        static_cast<int>(partition.ownerships.count(groupId)));
                    values.insert(QStringLiteral("bucketOccurrenceCount"),
                        bucketOccurrences[groupId]);
                    setZoneSweepLifecycleFailure
                    (
                        DiagnosticCode::ProcessPlanningInvariantViolation,
                        QStringLiteral("加工单元的 16 区位生产归属不唯一。"),
                        QStringLiteral("Every ordinary group must have one owner and occur in exactly one production bucket."),
                        std::move(values)
                    );
                    return false;
                }
            }

            startedPartitions[partitionIndex] = true;
            zoneSweepState.partitionId = partitionId;
            zoneSweepState.initialZone = initialZone;
            zoneSweepState.currentZoneOffset = 0;
            zoneSweepState.longitudinalDirection =
                partition.longitudinalDirection;
            zoneSweepState.frontierX =
                partition.longitudinalDirection >= 0
                ? partition.minimumX : partition.maximumX;
            zoneSweepState.zoneEntered = false;
            zoneSweepState.active = !partition.groupIds.empty();

            zoneSweepReport.partitionId = partitionId;
            zoneSweepReport.initialZone = initialZone;
            zoneSweepReport.perimeterDirection =
                partition.perimeterDirection;
            zoneSweepReport.longitudinalDirection =
                partition.longitudinalDirection;
            zoneSweepReport.partitionMinimumX = partition.minimumX;
            zoneSweepReport.partitionMaximumX = partition.maximumX;
            zoneSweepReport.active = true;
            if (!zoneSweepState.active)
            {
                finishedPartitions[partitionIndex] = true;
                zoneSweepDiagnostics.push_back
                    (zone16SweepDiagnostic(context, zoneSweepReport));
                zoneSweepState = Zone16SweepState{};
                zoneSweepReport = Zone16SweepReport{};
            }
            return true;
        };
        const auto zoneBlockedFailure =
            [&](const TubeZoneSweepPartition& partition,
                machining::TubeZone16 zone)
                -> OperationResult<ProcessPlan>
        {
            QStringList ownedUnitKeys;
            QStringList unfinishedUnitKeys;
            QStringList eligibleUnitKeys;
            QStringList blockedUnitKeys;
            QStringList remainingPredecessors;
            const auto& bucket =
                partition.zoneBuckets[machining::tubeZoneIndex(zone)];
            for (const int groupId : bucket)
            {
                const QString unitKey = processGroupKeyText
                    (plan.groups[static_cast<std::size_t>(groupId)]);
                ownedUnitKeys.push_back(unitKey);
                if (scheduled.find(groupId) != scheduled.end())
                    continue;
                unfinishedUnitKeys.push_back(unitKey);
                if (indegree[groupId] == 0)
                    eligibleUnitKeys.push_back(unitKey);
                else
                    blockedUnitKeys.push_back(unitKey);
                remainingPredecessors.push_back
                    (QStringLiteral("%1:%2").arg(unitKey)
                        .arg(indegree[groupId]));
            }
            QVariantMap values = diagnosticValues(input, policy);
            values.insert(QStringLiteral("zoneBlocked"), true);
            values.insert(QStringLiteral("partitionId"),
                partition.partitionId);
            values.insert(QStringLiteral("zone"),
                machining::tubeZoneName(zone));
            values.insert(QStringLiteral("frontierX"),
                zoneSweepState.frontierX);
            values.insert(QStringLiteral("ownedUnitKeys"),
                ownedUnitKeys.join(QLatin1Char(',')));
            values.insert(QStringLiteral("unfinishedUnitKeys"),
                unfinishedUnitKeys.join(QLatin1Char(',')));
            values.insert(QStringLiteral("eligibleUnitKeys"),
                eligibleUnitKeys.join(QLatin1Char(',')));
            values.insert(QStringLiteral("blockedUnitKeys"),
                blockedUnitKeys.join(QLatin1Char(',')));
            values.insert(QStringLiteral("remainingPredecessors"),
                remainingPredecessors.join(QLatin1Char(',')));
            OperationResult<ProcessPlan> blocked = failure<ProcessPlan>
            (
                OperationStatus::Failed, context,
                DiagnosticCode::ProcessPlanningZoneBlockedByPrecedence,
                QStringLiteral("当前区位仍有未完成加工单元，但其前置约束尚未满足，不能提前切换区位。"),
                QStringLiteral("The current owner-zone bucket is unfinished and has no eligible unit."),
                values
            );
            blocked.mergeDiagnostics(zoneSweepDiagnostics);
            return blocked;
        };
        while (scheduled.size() < schedulable.size())
        {
            std::vector<int> eligible;
            for (const int groupId : schedulable)
                if (scheduled.find(groupId) == scheduled.end() && indegree[groupId] == 0) eligible.push_back(groupId);
            std::sort(eligible.begin(), eligible.end());
            if (eligible.empty())
            {
                if (zone16SweepEnabled && zoneSweepState.active)
                {
                    const TubeZoneSweepPartition& partition =
                        zoneSweepPartitions[static_cast<std::size_t>
                            (zoneSweepState.partitionId)];
                    const machining::TubeZone16 zone = zoneAtOffset
                    (
                        zoneSweepState.initialZone,
                        zoneSweepState.currentZoneOffset,
                        partition.perimeterDirection
                    );
                    if (!zoneCompleted(partition, zone, scheduled))
                        return zoneBlockedFailure(partition, zone);
                }
                return failure<ProcessPlan>
                (
                    OperationStatus::Failed, context, DiagnosticCode::ProcessPlanningPrecedenceCycle,
                    QStringLiteral("中断切面前置约束形成循环，无法生成加工计划。"), QStringLiteral("No eligible group remains while unscheduled groups exist."),
                    diagnosticValues(input, policy, 0U, 0U, -1, -1, -1, -1,
                        static_cast<int>(schedulable.size() - scheduled.size()), 0,
                        static_cast<int>(plan.groups.size()), static_cast<int>(plan.assignments.size()),
                        static_cast<int>(plan.exclusions.size()), -1, -1, scheduled.empty())
                );
            }

            if (zone16SweepEnabled && zoneSweepState.active)
            {
                const TubeZoneSweepPartition& partition =
                    zoneSweepPartitions[static_cast<std::size_t>
                        (zoneSweepState.partitionId)];
                const bool complete = std::all_of
                (
                    partition.groupIds.cbegin(), partition.groupIds.cend(),
                    [&scheduled](int groupId)
                    { return scheduled.find(groupId) != scheduled.end(); }
                );
                if (complete)
                {
                    if (zoneSweepReport.processedUnitCount
                        != static_cast<int>(partition.groupIds.size()))
                    {
                        QVariantMap values = diagnosticValues
                            (input, policy, 0U, 0U, -1, -1);
                        values.insert(QStringLiteral("partitionId"),
                            partition.partitionId);
                        values.insert(QStringLiteral("processedUnitCount"),
                            zoneSweepReport.processedUnitCount);
                        values.insert(QStringLiteral("expectedUnitCount"),
                            static_cast<int>(partition.groupIds.size()));
                        return failure<ProcessPlan>
                        (
                            OperationStatus::InternalError, context,
                            DiagnosticCode::ProcessPlanningInvariantViolation,
                            QStringLiteral("16 区位加工段计数与实际加工单元不一致。"),
                            QStringLiteral("Zone sweep report did not account for every partition group exactly once."),
                            values
                        );
                    }
                    if (!finishZoneSweepPartition())
                        return std::move(*zoneSweepLifecycleFailure);
                }
            }

            const bool hasEligibleBreak = std::any_of
            (
                eligible.cbegin(), eligible.cend(),
                [&plan](int groupId)
                {
                    return plan.groups[static_cast<std::size_t>(groupId)].kind
                        == ProcessGroupKind::BreakBoundary;
                }
            );
            if (zone16SweepEnabled && !zoneSweepState.active
                && !hasEligibleBreak)
            {
                for (const TubeZoneSweepPartition& partition :
                    zoneSweepPartitions)
                {
                    const bool hasEligibleUnit = std::any_of
                    (
                        eligible.cbegin(), eligible.cend(),
                        [&partition](int groupId)
                        {
                            return partition.groupIds.find(groupId)
                                != partition.groupIds.end();
                        }
                    );
                    if (hasEligibleUnit)
                    {
                        if (!startZoneSweepPartition
                            (partition.partitionId, currentSweepZone))
                        {
                            if (zoneSweepLifecycleFailure.has_value())
                                return std::move(*zoneSweepLifecycleFailure);
                            return failure<ProcessPlan>
                            (
                                OperationStatus::Failed, context,
                                DiagnosticCode::ProcessPlanningZoneSweepProfileInvalid,
                                QStringLiteral("无法以当前工艺区位启动 16 区位扫描。"),
                                QStringLiteral("The eligible partition rejected the current sweep zone."),
                                diagnosticValues(input, policy)
                            );
                        }
                        break;
                    }
                }
            }

            const bool initialSelection = scheduled.empty();
            const ProcessOrderingStrategy selectionStrategy =
                policy.sortIntent == ProcessSortIntent::RebuildSequence
                    && policy.orderingStrategy
                        == ProcessOrderingStrategy::LazyRotation
                ? ProcessOrderingStrategy::LazyRotation
                : ProcessOrderingStrategy::NearestNext;
            std::vector<int> candidateGroupIds = eligible;
            std::unordered_map<int, ZoneSweepSelection> zoneSelections;
            if (zone16SweepEnabled && zoneSweepState.active)
            {
                candidateGroupIds.clear();
                const TubeZoneSweepPartition& partition =
                    zoneSweepPartitions[static_cast<std::size_t>
                        (zoneSweepState.partitionId)];
                const std::unordered_set<int> eligibleSet
                    (eligible.cbegin(), eligible.cend());
                while (zoneSweepState.currentZoneOffset
                    < static_cast<int>(machining::kTubeZone16Count))
                {
                    const machining::TubeZone16 zone = zoneAtOffset
                        (zoneSweepState.initialZone,
                            zoneSweepState.currentZoneOffset,
                            partition.perimeterDirection);
                    const std::size_t zoneIndex =
                        machining::tubeZoneIndex(zone);
                    if (!zoneSweepState.zoneEntered)
                    {
                        if (zoneSweepState.enteredZones[zoneIndex])
                        {
                            QVariantMap values = diagnosticValues
                                (input, policy);
                            values.insert(QStringLiteral("partitionId"),
                                zoneSweepState.partitionId);
                            values.insert(QStringLiteral("zone"),
                                machining::tubeZoneName(zone));
                            values.insert(QStringLiteral("currentZoneOffset"),
                                zoneSweepState.currentZoneOffset);
                            return failure<ProcessPlan>
                            (
                                OperationStatus::InternalError, context,
                                DiagnosticCode::ProcessPlanningZoneReentered,
                                QStringLiteral("16 区位加工阶段被重复进入。"),
                                QStringLiteral("A completed or previously entered zone cannot be entered again."),
                                values
                            );
                        }
                        zoneSweepState.frontierX =
                            zoneSweepState.longitudinalDirection >= 0
                            ? partition.minimumX : partition.maximumX;
                        zoneSweepState.zoneEntered = true;
                        zoneSweepState.enteredZones[zoneIndex] = true;
                        zoneSweepDiagnostics.push_back(zonePhaseDiagnostic
                        (
                            context, partition, zone, scheduled,
                            QStringLiteral("Enter")
                        ));
                    }

                    if (zoneCompleted(partition, zone, scheduled))
                    {
                        if (!zoneSweepState.zoneEntered
                            || zoneSweepState.completedZones[zoneIndex])
                        {
                            QVariantMap values = diagnosticValues
                                (input, policy);
                            values.insert(QStringLiteral("partitionId"),
                                zoneSweepState.partitionId);
                            values.insert(QStringLiteral("zone"),
                                machining::tubeZoneName(zone));
                            return failure<ProcessPlan>
                            (
                                OperationStatus::InternalError, context,
                                DiagnosticCode::ProcessPlanningZoneReentered,
                                QStringLiteral("16 区位加工阶段完成状态重复。"),
                                QStringLiteral("A zone phase may complete only once after entering."),
                                values
                            );
                        }
                        zoneSweepState.completedZones[zoneIndex] = true;
                        zoneSweepDiagnostics.push_back(zonePhaseDiagnostic
                        (
                            context, partition, zone, scheduled,
                            QStringLiteral("Complete")
                        ));
                        ++zoneSweepState.currentZoneOffset;
                        ++zoneSweepReport.zoneTransitionCount;
                        zoneSweepState.zoneEntered = false;
                        continue;
                    }

                    std::vector<ZoneSweepSelection> available;
                    for (const int groupId :
                        partition.zoneBuckets[zoneIndex])
                    {
                        if (scheduled.find(groupId) != scheduled.end()
                            || eligibleSet.find(groupId) == eligibleSet.end())
                            continue;
                        const ProcessGroupZoneProfile& profile =
                            groupZoneProfiles.at(groupId);
                        const machining::TubeZoneSpan& span =
                            profile.zoneSpans[zoneIndex];
                        const bool behind =
                            zoneSweepState.longitudinalDirection >= 0
                            ? span.maximumX < zoneSweepState.frontierX
                                - zoneProjectionTolerance
                            : span.minimumX > zoneSweepState.frontierX
                                + zoneProjectionTolerance;
                        if (behind)
                        {
                            QVariantMap values = diagnosticValues
                                (input, policy, 0U, 0U, -1, groupId);
                            values.insert(QStringLiteral("partitionId"),
                                zoneSweepState.partitionId);
                            values.insert(QStringLiteral("zone"),
                                machining::tubeZoneName(zone));
                            values.insert(QStringLiteral("unitKey"),
                                processGroupKeyText(plan.groups
                                    [static_cast<std::size_t>(groupId)]));
                            values.insert(QStringLiteral("spanMinimumX"),
                                span.minimumX);
                            values.insert(QStringLiteral("spanMaximumX"),
                                span.maximumX);
                            values.insert(QStringLiteral("frontierX"),
                                zoneSweepState.frontierX);
                            values.insert(QStringLiteral("remainingPredecessors"),
                                indegree[groupId]);
                            return failure<ProcessPlan>
                            (
                                OperationStatus::Failed, context,
                                DiagnosticCode::ProcessPlanningZoneSweepBacktrackRequired,
                                QStringLiteral("16 区位扫描发现必须回头的加工单元，已停止排序。"),
                                QStringLiteral("An eligible unit lies completely behind the monotonic zone frontier."),
                                values
                            );
                        }

                        ZoneSweepSelection selection;
                        selection.groupId = groupId;
                        selection.zone = zone;
                        selection.span = span;
                        selection.hitX =
                            zoneSweepState.longitudinalDirection >= 0
                            ? span.minimumX : span.maximumX;
                        selection.frontierBefore =
                            zoneSweepState.frontierX;
                        selection.fallbackOwner = partition.ownerships.at
                            (groupId).usedPossibleFallback;
                        available.push_back(selection);
                    }

                    if (!available.empty())
                    {
                        std::sort(available.begin(), available.end(),
                            [&](const ZoneSweepSelection& left,
                                const ZoneSweepSelection& right)
                        {
                            if (std::abs(left.hitX - right.hitX)
                                > zoneProjectionTolerance)
                            {
                                return zoneSweepState.longitudinalDirection >= 0
                                    ? left.hitX < right.hitX
                                    : left.hitX > right.hitX;
                            }
                            return processGroupStableLess
                            (
                                plan.groups[static_cast<std::size_t>(left.groupId)],
                                plan.groups[static_cast<std::size_t>(right.groupId)]
                            );
                        });
                        const ZoneSweepSelection& selection =
                            available.front();
                        candidateGroupIds.push_back(selection.groupId);
                        zoneSelections.emplace
                            (selection.groupId, selection);
                        break;
                    }

                    return zoneBlockedFailure(partition, zone);
                }

                if (candidateGroupIds.empty())
                {
                    QVariantMap values = diagnosticValues(input, policy);
                    values.insert(QStringLiteral("partitionId"),
                        zoneSweepState.partitionId);
                    values.insert(QStringLiteral("currentZoneOffset"),
                        zoneSweepState.currentZoneOffset);
                    return failure<ProcessPlan>
                    (
                        OperationStatus::InternalError, context,
                        DiagnosticCode::ProcessPlanningZoneIncomplete,
                        QStringLiteral("16 区位加工段未生成下一加工单元。"),
                        QStringLiteral("The sweep exhausted its zone phases without completing the active partition."),
                        values
                    );
                }
            }

            std::vector<SchedulingCandidate> candidates;
            candidates.reserve(candidateGroupIds.size());
            for (const int groupId : candidateGroupIds)
            {
                const ProcessGroup& group = plan.groups[static_cast<std::size_t>(groupId)];
                ClosedLoopTraversalReport candidateClosedLoopReport;
                std::optional<GroupTraversal> candidate;
                std::optional<BreakBoundaryTraversalReport>
                    candidateBreakReport;
                if (group.kind == ProcessGroupKind::BreakBoundary)
                {
                    const auto boundaryRank =
                        boundaryRankByGroup.find(group.groupId);
                    if (!surfaceSweepSection.has_value()
                        || boundaryRank == boundaryRankByGroup.end())
                    {
                        QVariantMap values = diagnosticValues
                            (input, policy, 0U, 0U, -1, groupId);
                        values.insert(QStringLiteral("boundaryRank"),
                            boundaryRank == boundaryRankByGroup.end()
                                ? -1 : boundaryRank->second);
                        return failure<ProcessPlan>
                        (
                            OperationStatus::Failed, context,
                            DiagnosticCode::ProcessPlanningBreakMidpointCandidateMissing,
                            QStringLiteral("加工断面缺少有效截面或空间排序，无法选择强区位中点。"),
                            QStringLiteral("Break midpoint selection requires a prepared TubeSectionModel and boundary rank."),
                            values
                        );
                    }
                    auto breakTraversal =
                        ClosedLoopZoneRunBuilder::buildBreak
                        (
                            group, entities, currentPosition, policy,
                            *surfaceSweepSection, input.tubeSectionCenter,
                            selectionStrategy, zoneProjectionTolerance,
                            boundaryRank->second,
                            currentSweepZone
                        );
                    candidateClosedLoopReport =
                        std::move(breakTraversal.closedLoopReport);
                    candidateBreakReport =
                        std::move(breakTraversal.report);
                    candidate =
                        std::move(breakTraversal.traversal);
                }
                else
                {
                    std::optional<TraversalSelectionContext>
                        traversalSelection;
                    const auto zoneSelection =
                        zoneSelections.find(groupId);
                    if (zoneSelection != zoneSelections.end())
                    {
                        traversalSelection.emplace();
                        traversalSelection->requiredEntryZone =
                            zoneSelection->second.zone;
                        traversalSelection->longitudinalDirection =
                            zoneSweepState.longitudinalDirection;
                        traversalSelection->zoneHitX =
                            zoneSelection->second.hitX;
                        traversalSelection->frontierX =
                            zoneSelection->second.frontierBefore;
                        traversalSelection->projectionTolerance =
                            zoneProjectionTolerance;
                        traversalSelection->previousCutEnd =
                            currentPosition;
                        machine::ToolTransferPolicy transferPolicy;
                        transferPolicy.rotationSafetyClearance =
                            policy.rotationSafetyClearance;
                        transferPolicy.sameZoneTransferClearance =
                            policy.sameZoneTransferClearance;
                        transferPolicy.coordinatedTransferEnabled =
                            policy.coordinatedTransferEnabled;
                        traversalSelection->previousTransferAnchor =
                            initialSelection
                            ? currentPosition
                            : machine::RotaryKinematics
                                ::sourceTransferAnchor
                                (
                                    currentPosition,
                                    previousTransferZone,
                                    zoneSelection->second.zone,
                                    transferPolicy,
                                    input.tubeSection,
                                    input.tubeSectionCenter.has_value()
                                        ? input.tubeSectionCenter->x
                                        : surfaceSweepSection
                                            ->geometry.centerY,
                                    input.tubeSectionCenter.has_value()
                                        ? input.tubeSectionCenter->y
                                        : surfaceSweepSection
                                            ->geometry.centerZ,
                                    zoneProjectionTolerance
                                );
                        traversalSelection->previousEnd =
                            traversalSelection->previousTransferAnchor;
                        traversalSelection->hardZoneConstraint = true;
                        const ProcessGroupZoneProfile& profile =
                            groupZoneProfiles.at(groupId);
                        traversalSelection
                            ->allowZoneRunMidpointFallback =
                            (profile.zoneRunMidpointEntryMask
                                & machining::tubeZoneBit
                                    (zoneSelection->second.zone)) != 0U;
                    }
                    candidate = bestTraversal
                        (group, entities, currentPosition, policy,
                            input.tubeSection, input.tubeSectionCenter,
                            selectionStrategy,
                            &candidateClosedLoopReport,
                            traversalSelection.has_value()
                                ? &*traversalSelection : nullptr);
                }
                if (!candidate.has_value())
                {
                    const bool manualEntryConstraint =
                        hasManualEntryConstraint(group, entities);
                    const bool hardZoneConstraint =
                        zoneSelections.find(groupId)
                            != zoneSelections.end();
                    QVariantMap values = diagnosticValues
                    (
                        input, policy, 0U, 0U, -1, groupId, -1, -1,
                        static_cast<int>(schedulable.size() - scheduled.size()),
                        static_cast<int>(eligible.size()), static_cast<int>(plan.groups.size()),
                        static_cast<int>(plan.assignments.size()),
                        static_cast<int>(plan.exclusions.size()), -1, -1, initialSelection,
                        nullptr, group.kind
                    );
                    values.insert(QStringLiteral("manualEntryConstraint"),
                        manualEntryConstraint);
                    values.insert(QStringLiteral("hardZoneConstraint"),
                        hardZoneConstraint);
                    const auto requiredZone =
                        zoneSelections.find(groupId);
                    if (requiredZone != zoneSelections.end())
                    {
                        values.insert(QStringLiteral("requiredEntryZone"),
                            machining::tubeZoneName
                                (requiredZone->second.zone));
                    }
                    if (candidateClosedLoopReport.groupId >= 0)
                    {
                        const QVariantMap closedLoopValues =
                            closedLoopDiagnosticValues(candidateClosedLoopReport);
                        for (auto iterator = closedLoopValues.cbegin();
                            iterator != closedLoopValues.cend(); ++iterator)
                            values.insert(iterator.key(), iterator.value());
                    }
                    if (candidateBreakReport.has_value())
                    {
                        const QVariantMap breakValues =
                            breakDiagnosticValues(*candidateBreakReport);
                        for (auto iterator = breakValues.cbegin();
                            iterator != breakValues.cend(); ++iterator)
                        {
                            values.insert(iterator.key(),
                                iterator.value());
                        }
                    }
                    return failure<ProcessPlan>
                    (
                        manualEntryConstraint && hardZoneConstraint
                            ? OperationStatus::Conflict
                            : OperationStatus::Failed, context,
                        candidateBreakReport.has_value()
                            ? candidateBreakReport->failureCode
                            : candidateClosedLoopReport.groupId >= 0
                            && !candidateClosedLoopReport.simpleLoopValid
                            ? DiagnosticCode::ProcessPlanningGroupBuildFailed
                            : DiagnosticCode::ProcessPlanningDirectionFailed,
                        manualEntryConstraint && hardZoneConstraint
                            ? QStringLiteral("人工起点或方向无法在当前扫描区位形成合法入口。")
                            : candidateBreakReport.has_value()
                            ? QStringLiteral("加工断面无法从可靠强区位边段中点建立闭环遍历。")
                            : candidateClosedLoopReport.groupId >= 0
                            && !candidateClosedLoopReport.simpleLoopValid
                            ? QStringLiteral("多图元闭合加工单元不是唯一简单环，无法生成加工计划。")
                            : QStringLiteral("连续加工组无法建立有效入口和加工方向。"),
                        manualEntryConstraint && hardZoneConstraint
                            ? QStringLiteral("Manual entry constraints conflict with requiredEntryZone.")
                            : candidateBreakReport.has_value()
                            ? candidateBreakReport->failureReason
                            : candidateClosedLoopReport.groupId >= 0
                            ? candidateClosedLoopReport.failureReason
                            : QStringLiteral("No connected traversal covers every group entity."),
                        values
                    );
                }
                SchedulingCandidate schedulingCandidate;
                schedulingCandidate.traversal = std::move(*candidate);
                if (candidateClosedLoopReport.groupId >= 0)
                    schedulingCandidate.closedLoopReport =
                        std::move(candidateClosedLoopReport);
                if (candidateBreakReport.has_value())
                    schedulingCandidate.breakReport =
                        std::move(candidateBreakReport);
                candidates.push_back(std::move(schedulingCandidate));
            }
            if (candidates.empty())
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::Failed, context, DiagnosticCode::ProcessPlanningOrderingFailed,
                    QStringLiteral("无法从可调度加工组中选择下一组。"), QStringLiteral("eligibleGroups produced no candidate."),
                    diagnosticValues(input, policy, 0U, 0U, -1, -1, -1, -1,
                        static_cast<int>(schedulable.size() - scheduled.size()),
                        static_cast<int>(eligible.size()), static_cast<int>(plan.groups.size()),
                        static_cast<int>(plan.assignments.size()), static_cast<int>(plan.exclusions.size()),
                        -1, -1, initialSelection)
                );
            }

            std::size_t selectedIndex = 0U;
            for (std::size_t candidateIndex = 1U;
                candidateIndex < candidates.size(); ++candidateIndex)
            {
                bool replace = false;
                const auto leftZone = zoneSelections.find
                    (candidates[candidateIndex].traversal.groupId);
                const auto rightZone = zoneSelections.find
                    (candidates[selectedIndex].traversal.groupId);
                if (leftZone != zoneSelections.end()
                    && rightZone != zoneSelections.end())
                {
                    const ZoneSweepSelection& left = leftZone->second;
                    const ZoneSweepSelection& right = rightZone->second;
                    if (std::abs(left.hitX - right.hitX)
                        > zoneProjectionTolerance)
                    {
                        replace = zoneSweepState.longitudinalDirection >= 0
                            ? left.hitX < right.hitX
                            : left.hitX > right.hitX;
                    }
                    else
                    {
                        const auto intersectsFrontier =
                            [&](const ZoneSweepSelection& selection)
                        {
                            return selection.span.minimumX
                                    <= selection.frontierBefore
                                        + zoneProjectionTolerance
                                && selection.span.maximumX
                                    >= selection.frontierBefore
                                        - zoneProjectionTolerance;
                        };
                        const bool leftIntersects = intersectsFrontier(left);
                        const bool rightIntersects = intersectsFrontier(right);
                        if (leftIntersects != rightIntersects)
                        {
                            replace = leftIntersects;
                        }
                        else
                        {
                            replace = processGroupStableLess
                            (
                                plan.groups[static_cast<std::size_t>
                                    (candidates[candidateIndex]
                                        .traversal.groupId)],
                                plan.groups[static_cast<std::size_t>
                                    (candidates[selectedIndex]
                                        .traversal.groupId)]
                            );
                        }
                    }
                }
                else
                {
                    replace = traversalLess(candidates[candidateIndex].traversal,
                        candidates[selectedIndex].traversal, selectionStrategy);
                }
                if (replace) selectedIndex = candidateIndex;
            }
            SchedulingCandidate selectedCandidate =
                std::move(candidates[selectedIndex]);
            GroupTraversal& selected = selectedCandidate.traversal;
            std::optional<ClosedLoopTraversalReport>& selectedClosedLoopReport =
                selectedCandidate.closedLoopReport;
            std::optional<BreakBoundaryTraversalReport>& selectedBreakReport =
                selectedCandidate.breakReport;

            const ProcessGroup& selectedGroup = plan.groups[static_cast<std::size_t>(selected.groupId)];
            const auto scheduledZoneSelection =
                zoneSelections.find(selected.groupId);
            if (zone16SweepEnabled
                && selectedGroup.kind != ProcessGroupKind::BreakBoundary
                && scheduledZoneSelection == zoneSelections.end())
            {
                QVariantMap values = diagnosticValues
                    (input, policy, 0U, 0U, -1, selected.groupId);
                values.insert(QStringLiteral("unitKey"),
                    processGroupKeyText(selectedGroup));
                values.insert(QStringLiteral("activePartitionId"),
                    zoneSweepState.partitionId);
                return failure<ProcessPlan>
                (
                    OperationStatus::InternalError, context,
                    DiagnosticCode::ProcessPlanningInvariantViolation,
                    QStringLiteral("普通加工单元未经过唯一生产区位桶调度。"),
                    QStringLiteral("Every ordinary group in a Zone16 sweep must be selected from its owner bucket."),
                    values
                );
            }
            if (scheduledZoneSelection != zoneSelections.end())
            {
                if (!selected.selectedEntry.has_value()
                    || selected.selectedEntry->zone
                        != scheduledZoneSelection->second.zone)
                {
                    QVariantMap values = diagnosticValues
                        (input, policy, 0U, 0U, -1,
                            selected.groupId);
                    values.insert(QStringLiteral("unitKey"),
                        processGroupKeyText(selectedGroup));
                    values.insert(QStringLiteral("scheduledZone"),
                        machining::tubeZoneName
                            (scheduledZoneSelection->second.zone));
                    values.insert(QStringLiteral("selectedEntryZone"),
                        selected.selectedEntry.has_value()
                        ? machining::tubeZoneName
                            (selected.selectedEntry->zone)
                        : QStringLiteral("None"));
                    return failure<ProcessPlan>
                    (
                        OperationStatus::Failed, context,
                        DiagnosticCode::
                            ProcessPlanningInvariantViolation,
                        QStringLiteral("加工单元实际起刀区位与当前扫描区位不一致。"),
                        QStringLiteral("scheduledZone must equal selectedEntryZone before plan publication."),
                        values
                    );
                }
            }
            const bool continuous = selectedGroup.kind == ProcessGroupKind::ConnectedChain
                || selectedGroup.kind == ProcessGroupKind::ClosedLoop
                || selectedGroup.kind == ProcessGroupKind::BreakBoundary;
            const int continuousGroupId = continuous ? selectedGroup.groupId : -1;
            ProcessUnit processUnit;
            processUnit.key.memberEntityIds = selectedGroup.entityIds;
            std::sort(processUnit.key.memberEntityIds.begin(), processUnit.key.memberEntityIds.end());
            processUnit.closed = selectedGroup.closed;
            processUnit.ownerZone =
                scheduledZoneSelection != zoneSelections.end()
                ? std::optional<machining::TubeZone16>
                    (scheduledZoneSelection->second.zone)
                : selected.selectedEntry.has_value()
                    ? std::optional<machining::TubeZone16>
                        (selected.selectedEntry->zone)
                    : std::nullopt;
            processUnit.orderedMemberEntityIds.reserve(selected.entities.size());
            for (const DirectedEntity& directed : selected.entities)
                processUnit.orderedMemberEntityIds.push_back(directed.entity->entityId);
            const int processUnitIndex = static_cast<int>(plan.processUnits.size());
            plan.processUnits.push_back(processUnit);
            plan.processUnitSequence.units.push_back(processUnit.key);
            for (const DirectedEntity& directed : selected.entities)
            {
                ProcessAssignment assignment;
                assignment.entityId = directed.entity->entityId;
                assignment.processOrder = processOrder++;
                assignment.processUnitIndex = processUnitIndex;
                assignment.continuousGroupId = continuousGroupId;
                assignment.reverse = directed.reverseRelativeToInput;
                assignment.startParameter = directed.selectedStartParameter;
                plan.assignments.push_back(assignment);
            }
            const std::vector<ProcessPathFragment>* selectedFragments =
                selectedBreakReport.has_value()
                ? &selectedBreakReport->fragments
                : &selected.fragments;
            if (selectedFragments != nullptr
                && !selectedFragments->empty())
            {
                for (ProcessPathFragment fragment :
                    *selectedFragments)
                {
                    fragment.processUnitIndex = processUnitIndex;
                    plan.plannedFragments.push_back(std::move(fragment));
                }
            }
            currentPosition = selected.end;
            previousTransferZone = processUnit.ownerZone;
            if (selectedClosedLoopReport.has_value())
            {
                closedLoopDiagnostics.push_back(planningDiagnostic
                (
                    context,
                    DiagnosticCode::ProcessPlanningClosedLoopSummary,
                    QStringLiteral("多图元闭合加工单元已建立确定遍历。"),
                    QStringLiteral("Closed-loop traversal selected from complete loop candidates."),
                    closedLoopDiagnosticValues(*selectedClosedLoopReport),
                    DiagnosticSeverity::Info
                ));
            }
            if (policy.sortIntent == ProcessSortIntent::RebuildSequence
                && policy.orderingStrategy
                    == ProcessOrderingStrategy::LazyRotation
                && selectedGroup.kind != ProcessGroupKind::BreakBoundary)
            {
                closedLoopDiagnostics.push_back
                    (entrySelectionDiagnostic
                    (
                        context, selectedGroup, selected,
                        scheduledZoneSelection != zoneSelections.end()
                        ? std::optional<machining::TubeZone16>
                            (scheduledZoneSelection->second.zone)
                        : std::nullopt
                    ));
            }
            if (!scheduled.insert(selected.groupId).second)
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::InternalError, context,
                    DiagnosticCode::ProcessPlanningInvariantViolation,
                    QStringLiteral("加工单元被重复加入加工计划。"),
                    QStringLiteral("A ProcessGroup may be scheduled exactly once."),
                    diagnosticValues(input, policy, 0U, 0U, -1,
                        selected.groupId)
                );
            }
            ++schedulingOccurrences[selected.groupId];
            for (const int successor : successors[selected.groupId]) --indegree[successor];

            if (zone16SweepEnabled && zoneSweepState.active
                && selectedGroup.kind != ProcessGroupKind::BreakBoundary)
            {
                const TubeZoneSweepPartition& partition =
                    zoneSweepPartitions[static_cast<std::size_t>
                        (zoneSweepState.partitionId)];
                const machining::TubeZone16 zone = zoneAtOffset
                (
                    zoneSweepState.initialZone,
                    zoneSweepState.currentZoneOffset,
                    partition.perimeterDirection
                );
                const std::size_t zoneIndex =
                    machining::tubeZoneIndex(zone);
                if (zoneCompleted(partition, zone, scheduled))
                {
                    if (!zoneSweepState.zoneEntered
                        || !zoneSweepState.enteredZones[zoneIndex]
                        || zoneSweepState.completedZones[zoneIndex])
                    {
                        QVariantMap values = diagnosticValues
                            (input, policy, 0U, 0U, -1,
                                selected.groupId);
                        values.insert(QStringLiteral("partitionId"),
                            zoneSweepState.partitionId);
                        values.insert(QStringLiteral("zone"),
                            machining::tubeZoneName(zone));
                        return failure<ProcessPlan>
                        (
                            OperationStatus::InternalError, context,
                            DiagnosticCode::ProcessPlanningZoneIncomplete,
                            QStringLiteral("16 区位加工阶段完成状态无效。"),
                            QStringLiteral("The owner-zone bucket completed outside a valid entered phase."),
                            values
                        );
                    }
                    zoneSweepState.completedZones[zoneIndex] = true;
                    zoneSweepDiagnostics.push_back(zonePhaseDiagnostic
                    (
                        context, partition, zone, scheduled,
                        QStringLiteral("Complete")
                    ));
                    ++zoneSweepState.currentZoneOffset;
                    ++zoneSweepReport.zoneTransitionCount;
                    zoneSweepState.zoneEntered = false;
                }
            }

            if (zone16SweepEnabled)
            {
                if (selectedGroup.kind == ProcessGroupKind::BreakBoundary)
                {
                    const auto partition = partitionAfterBreakGroup.find
                        (selected.groupId);
                    if (!selectedBreakReport.has_value()
                        || !selectedBreakReport->exitZone.has_value())
                    {
                        QVariantMap values = diagnosticValues
                            (input, policy, 0U, 0U, -1, selected.groupId);
                        values.insert(QStringLiteral("unitKey"),
                            processGroupKeyText(selectedGroup));
                        if (selectedBreakReport.has_value())
                        {
                            const QVariantMap breakValues =
                                breakDiagnosticValues(*selectedBreakReport);
                            for (auto iterator = breakValues.cbegin();
                                iterator != breakValues.cend(); ++iterator)
                            {
                                values.insert(iterator.key(),
                                    iterator.value());
                            }
                        }
                        return failure<ProcessPlan>
                        (
                            OperationStatus::Failed, context,
                            DiagnosticCode::ProcessPlanningBreakExitZoneUnresolved,
                            QStringLiteral("加工断面的最终计划片段无法解析可靠出口区位。"),
                            QStringLiteral("Break exit zone was not resolved from the final planned fragment."),
                            values
                        );
                    }
                    currentSweepZone =
                        *selectedBreakReport->exitZone;
                    if (partition == partitionAfterBreakGroup.end())
                    {
                        QVariantMap values =
                            breakDiagnosticValues(*selectedBreakReport);
                        values.insert(QStringLiteral("nextPartitionId"), -1);
                        values.insert(QStringLiteral("partitionMappingFound"),
                            false);
                        values.insert(QStringLiteral("partitionStartSucceeded"),
                            false);
                        return failure<ProcessPlan>
                        (
                            OperationStatus::Failed, context,
                            DiagnosticCode::ProcessPlanningBreakPartitionMappingMissing,
                            QStringLiteral("加工断面缺少下一加工段的分区映射。"),
                            QStringLiteral("No zone-sweep partition is mapped after the selected Break boundary."),
                            values
                        );
                    }
                    if (!startZoneSweepPartition
                        (partition->second,
                            *selectedBreakReport->exitZone))
                    {
                        if (zoneSweepLifecycleFailure.has_value())
                            return std::move(*zoneSweepLifecycleFailure);
                        QVariantMap values =
                            breakDiagnosticValues(*selectedBreakReport);
                        values.insert(QStringLiteral("nextPartitionId"),
                            partition->second);
                        values.insert(QStringLiteral("partitionMappingFound"),
                            true);
                        values.insert(QStringLiteral("partitionStartSucceeded"),
                            false);
                        return failure<ProcessPlan>
                        (
                            OperationStatus::Failed, context,
                            DiagnosticCode::ProcessPlanningBreakPartitionStartFailed,
                            QStringLiteral("加工断面出口区位无法启动下一加工段。"),
                            QStringLiteral("The mapped partition rejected the resolved Break exit zone."),
                            values
                        );
                    }
                    selectedBreakReport->nextPartitionId =
                        partition->second;
                    selectedBreakReport->partitionMappingFound = true;
                    selectedBreakReport->partitionStartSucceeded = true;
                }
                else
                {
                    const auto activeSelection =
                        zoneSelections.find(selected.groupId);
                    if (!zoneSweepState.active)
                    {
                        if (!startZoneSweepPartition
                            (0, currentSweepZone))
                        {
                            if (zoneSweepLifecycleFailure.has_value())
                                return std::move(*zoneSweepLifecycleFailure);
                            return failure<ProcessPlan>
                            (
                                OperationStatus::Failed, context,
                                DiagnosticCode::ProcessPlanningZoneSweepProfileInvalid,
                                QStringLiteral("首个加工单元无法初始化 16 区位扫描。"),
                                QStringLiteral("The configured initial sweep zone could not start partition zero."),
                                diagnosticValues(input, policy, 0U, 0U, -1,
                                    selected.groupId)
                            );
                        }
                    }

                    ZoneSweepSelection selection;
                    bool selectionAvailable = false;
                    if (activeSelection != zoneSelections.end())
                    {
                        selection = activeSelection->second;
                        selectionAvailable = true;
                    }
                    else if (zoneSweepState.active)
                    {
                        const ProcessGroupZoneProfile& profile =
                            groupZoneProfiles.at(selected.groupId);
                        const TubeZoneSweepPartition& partition =
                            zoneSweepPartitions[static_cast<std::size_t>
                                (zoneSweepState.partitionId)];
                        const auto ownership =
                            partition.ownerships.find(selected.groupId);
                        if (ownership == partition.ownerships.end())
                        {
                            QVariantMap values = diagnosticValues
                                (input, policy, 0U, 0U, -1, selected.groupId);
                            values.insert(QStringLiteral("unitKey"),
                                processGroupKeyText(selectedGroup));
                            values.insert(QStringLiteral("partitionId"),
                                zoneSweepState.partitionId);
                            values.insert(QStringLiteral("certainMask"),
                                zoneMaskText(profile.certainMask));
                            values.insert(QStringLiteral("possibleMask"),
                                zoneMaskText(profile.possibleMask));
                            values.insert(QStringLiteral("legalEntryMask"),
                                zoneMaskText(profile.legalEntryMask));
                            return failure<ProcessPlan>
                            (
                                OperationStatus::Failed, context,
                                DiagnosticCode::ProcessPlanningInvariantViolation,
                                QStringLiteral("加工单元缺少唯一生产区位归属。"),
                                QStringLiteral("An ordinary selected group is absent from the partition ownership table."),
                                values
                            );
                        }
                        const machining::TubeZone16 zone =
                            ownership->second.ownerZone;
                        const machining::TubeZone16 activeZone = zoneAtOffset
                        (
                            zoneSweepState.initialZone,
                            zoneSweepState.currentZoneOffset,
                            partition.perimeterDirection
                        );
                        if (zone != activeZone
                            || !selected.selectedEntry.has_value()
                            || selected.selectedEntry->zone != zone)
                        {
                            QVariantMap values = diagnosticValues
                                (input, policy, 0U, 0U, -1,
                                    selected.groupId);
                            values.insert(QStringLiteral("partitionId"),
                                zoneSweepState.partitionId);
                            values.insert(QStringLiteral("ownerZone"),
                                machining::tubeZoneName(zone));
                            values.insert(QStringLiteral("activeZone"),
                                machining::tubeZoneName(activeZone));
                            values.insert(QStringLiteral("selectedEntryZone"),
                                selected.selectedEntry.has_value()
                                ? machining::tubeZoneName
                                    (selected.selectedEntry->zone)
                                : QStringLiteral("None"));
                            return failure<ProcessPlan>
                            (
                                OperationStatus::InternalError, context,
                                DiagnosticCode::ProcessPlanningInvariantViolation,
                                QStringLiteral("加工单元未从当前唯一生产区位进入。"),
                                QStringLiteral("Owner zone, active phase, and selected entry zone must match."),
                                values
                            );
                        }
                        selection.groupId = selected.groupId;
                        selection.zone = zone;
                        selection.span = profile.zoneSpans
                            [machining::tubeZoneIndex(zone)];
                        selection.hitX =
                            zoneSweepState.longitudinalDirection >= 0
                            ? selection.span.minimumX
                            : selection.span.maximumX;
                        selection.frontierBefore =
                            zoneSweepState.frontierX;
                        selection.fallbackOwner =
                            ownership->second.usedPossibleFallback
                            || ownership->second.usedBoundaryFallback;
                        selectionAvailable = true;
                    }

                    if (selectionAvailable && zoneSweepReport.active)
                    {
                        currentSweepZone = selection.zone;
                        const double frontierAfter =
                            zoneSweepState.longitudinalDirection >= 0
                            ? std::max(zoneSweepState.frontierX,
                                selection.span.maximumX)
                            : std::min(zoneSweepState.frontierX,
                                selection.span.minimumX);
                        const ProcessGroupZoneProfile& profile =
                            groupZoneProfiles.at(selected.groupId);
                        zoneSweepReport.selectedUnits.push_back
                        (
                            QStringLiteral("unitKey=%1|zone=%2|certainMask=%3|possibleMask=%4|span=[%5,%6]|hitX=%7|frontierBefore=%8|frontierAfter=%9|startX=%10|endX=%11|fallbackOwner=%12|order=%13")
                                .arg(processGroupKeyText(selectedGroup))
                                .arg(machining::tubeZoneName(selection.zone))
                                .arg(zoneMaskText(profile.certainMask))
                                .arg(zoneMaskText(profile.possibleMask))
                                .arg(selection.span.minimumX, 0, 'f', 6)
                                .arg(selection.span.maximumX, 0, 'f', 6)
                                .arg(selection.hitX, 0, 'f', 6)
                                .arg(selection.frontierBefore, 0, 'f', 6)
                                .arg(frontierAfter, 0, 'f', 6)
                                .arg(selected.start.x, 0, 'f', 6)
                                .arg(selected.end.x, 0, 'f', 6)
                                .arg(selection.fallbackOwner
                                    ? QStringLiteral("true")
                                    : QStringLiteral("false"))
                                .arg(processUnitIndex + 1)
                        );
                        zoneSweepState.frontierX = frontierAfter;
                        ++zoneSweepReport.processedUnitCount;
                    }
                }
            }
            if (selectedBreakReport.has_value())
            {
                closedLoopDiagnostics.push_back
                    (breakStartDiagnostic(context, *selectedBreakReport));
            }
        }

        if (zone16SweepEnabled && zoneSweepState.active)
        {
            const TubeZoneSweepPartition& partition =
                zoneSweepPartitions[static_cast<std::size_t>
                    (zoneSweepState.partitionId)];
            if (zoneSweepReport.processedUnitCount
                != static_cast<int>(partition.groupIds.size()))
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::InternalError, context,
                    DiagnosticCode::ProcessPlanningInvariantViolation,
                    QStringLiteral("最终 16 区位加工段未完整记录全部加工单元。"),
                    QStringLiteral("Final zone sweep report count does not match its partition."),
                    diagnosticValues(input, policy)
                );
            }
        }
        if (zone16SweepEnabled && !finishZoneSweepPartition())
            return std::move(*zoneSweepLifecycleFailure);
        if (zone16SweepEnabled)
        {
            for (std::size_t partitionIndex = 0U;
                partitionIndex < zoneSweepPartitions.size();
                ++partitionIndex)
            {
                const TubeZoneSweepPartition& partition =
                    zoneSweepPartitions[partitionIndex];
                if (partition.groupIds.empty()) continue;
                std::unordered_map<int, int> bucketOccurrences;
                for (const auto& bucket : partition.zoneBuckets)
                {
                    for (const int groupId : bucket)
                        ++bucketOccurrences[groupId];
                }
                for (const int groupId : partition.groupIds)
                {
                    if (partition.ownerships.count(groupId) != 1U
                        || bucketOccurrences[groupId] != 1
                        || schedulingOccurrences[groupId] != 1)
                    {
                        QVariantMap values = diagnosticValues
                            (input, policy, 0U, 0U, -1, groupId);
                        values.insert(QStringLiteral("partitionId"),
                            static_cast<int>(partitionIndex));
                        values.insert(QStringLiteral("ownershipCount"),
                            static_cast<int>
                                (partition.ownerships.count(groupId)));
                        values.insert(QStringLiteral("bucketOccurrenceCount"),
                            bucketOccurrences[groupId]);
                        values.insert(QStringLiteral("scheduledOccurrenceCount"),
                            schedulingOccurrences[groupId]);
                        return failure<ProcessPlan>
                        (
                            OperationStatus::InternalError, context,
                            DiagnosticCode::ProcessPlanningInvariantViolation,
                            QStringLiteral("16 区位加工单元的归属或调度次数不唯一。"),
                            QStringLiteral("Every ordinary group must have one owner, one production bucket, and one scheduling occurrence."),
                            values
                        );
                    }
                }
                if (!startedPartitions[partitionIndex]
                    || !finishedPartitions[partitionIndex])
                {
                    QVariantMap values = diagnosticValues(input, policy);
                    values.insert(QStringLiteral("partitionId"),
                        static_cast<int>(partitionIndex));
                    values.insert(QStringLiteral("started"),
                        static_cast<bool>(startedPartitions[partitionIndex]));
                    values.insert(QStringLiteral("finished"),
                        static_cast<bool>(finishedPartitions[partitionIndex]));
                    return failure<ProcessPlan>
                    (
                        OperationStatus::InternalError, context,
                        DiagnosticCode::ProcessPlanningZoneIncomplete,
                        QStringLiteral("16 区位加工段生命周期未完整结束。"),
                        QStringLiteral("Every non-empty partition must start and finish exactly once."),
                        values
                    );
                }
            }
        }

        OperationReport entryRefinement =
            SingleClosedEntryRefiner::refine
            (plan, input, policy, context);
        if (!entryRefinement.succeeded())
        {
            OperationResult<ProcessPlan> result;
            result.status = entryRefinement.status;
            result.mergeDiagnostics(entryRefinement.diagnostics);
            return result;
        }
        closedLoopDiagnostics += entryRefinement.diagnostics;

        std::unordered_set<EntityId> assignedIds;
        std::unordered_set<EntityId> excludedIds;
        std::unordered_map<EntityId, int> groupByEntity;
        std::unordered_map<int, int> firstOrderByGroup;
        std::unordered_map<int, int> lastOrderByGroup;
        for (const ProcessGroup& group : plan.groups)
        {
            for (const EntityId entityId : group.entityIds)
            {
                if (!groupByEntity.emplace(entityId, group.groupId).second)
                {
                    return failure<ProcessPlan>
                    (
                        OperationStatus::InternalError, context, DiagnosticCode::ProcessPlanningInvariantViolation,
                        QStringLiteral("加工图元被重复分配到多个加工组。"),
                        QStringLiteral("EntityId belongs to more than one ProcessGroup."),
                        diagnosticValues(input, policy, entityId, 0U, -1, group.groupId)
                    );
                }
            }
        }
        for (std::size_t index = 0; index < plan.assignments.size(); ++index)
        {
            const ProcessAssignment& assignment = plan.assignments[index];
            if (!assignedIds.insert(assignment.entityId).second
                || assignment.processOrder != static_cast<int>(index)
                || assignment.processUnitIndex < 0
                || static_cast<std::size_t>(assignment.processUnitIndex) >= plan.processUnits.size()
                || groupByEntity.find(assignment.entityId) == groupByEntity.end())
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::InternalError, context, DiagnosticCode::ProcessPlanningInvariantViolation,
                    QStringLiteral("加工计划完整性校验失败。"), QStringLiteral("Duplicate assignment, non-contiguous order, or missing group."),
                    diagnosticValues(input, policy, assignment.entityId, 0U, -1,
                        groupByEntity.find(assignment.entityId) != groupByEntity.end() ? groupByEntity[assignment.entityId] : -1,
                        -1, -1, 0, 0, static_cast<int>(plan.groups.size()),
                        static_cast<int>(plan.assignments.size()), static_cast<int>(plan.exclusions.size()),
                        assignment.processOrder, assignment.continuousGroupId)
                );
            }
            const int groupId = groupByEntity[assignment.entityId];
            if (firstOrderByGroup.find(groupId) == firstOrderByGroup.end()) firstOrderByGroup[groupId] = assignment.processOrder;
            lastOrderByGroup[groupId] = assignment.processOrder;
        }
        if (!validateProcessUnitStructure(plan))
        {
            return failure<ProcessPlan>
            (
                OperationStatus::InternalError, context, DiagnosticCode::ProcessPlanningInvariantViolation,
                QStringLiteral("加工单元成员、顺序或分配关系校验失败。"),
                QStringLiteral("ProcessUnit structure is inconsistent with assignments or sequence."),
                diagnosticValues(input, policy)
            );
        }
        ClosedLoopValidationFailure closedLoopFailure;
        if (!validateMultiEntityClosedLoopUnits
            (plan, entities, policy.connectionTolerance, closedLoopFailure))
        {
            QVariantMap values = diagnosticValues
                (input, policy, closedLoopFailure.currentEntityId, 0U, -1,
                    closedLoopFailure.groupId);
            values.insert(QStringLiteral("previousEntityId"),
                QVariant::fromValue<qulonglong>(closedLoopFailure.previousEntityId));
            values.insert(QStringLiteral("joinGap"), closedLoopFailure.joinGap);
            values.insert(QStringLiteral("connectionTolerance"), policy.connectionTolerance);
            return failure<ProcessPlan>
            (
                OperationStatus::InternalError, context,
                DiagnosticCode::ProcessPlanningInvariantViolation,
                QStringLiteral("多图元闭合加工单元的最终遍历不连续。"),
                closedLoopFailure.reason, values
            );
        }
        for (const ProcessExclusion& exclusion : plan.exclusions)
        {
            if (!excludedIds.insert(exclusion.entityId).second || assignedIds.find(exclusion.entityId) != assignedIds.end())
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::InternalError, context, DiagnosticCode::ProcessPlanningInvariantViolation,
                    QStringLiteral("加工计划排除项校验失败。"), QStringLiteral("Exclusions overlap assignments or contain duplicates."),
                    diagnosticValues(input, policy, exclusion.entityId)
                );
            }
        }
        for (const ProcessPrecedence& precedence : plan.precedenceConstraints)
        {
            if (lastOrderByGroup.find(precedence.predecessorGroupId) == lastOrderByGroup.end()
                || firstOrderByGroup.find(precedence.successorGroupId) == firstOrderByGroup.end()
                || lastOrderByGroup[precedence.predecessorGroupId] >= firstOrderByGroup[precedence.successorGroupId])
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::InternalError, context, DiagnosticCode::ProcessPlanningInvariantViolation,
                    QStringLiteral("加工计划违反加工断面屏障约束。"), QStringLiteral("Break boundary precedence was not satisfied."),
                    diagnosticValues(input, policy, 0U, 0U, precedence.boundaryPairId, -1,
                        precedence.predecessorGroupId, precedence.successorGroupId)
                );
            }
        }
        for (std::size_t rank = 0; rank < boundaryOrder.size(); ++rank)
        {
            const std::size_t boundaryIndex = static_cast<std::size_t>(boundaryOrder[rank]);
            const BoundaryData& boundary = boundaries[boundaryIndex];
            if (boundary.role != BoundaryRole::Break) continue;
            const ProcessGroup& boundaryGroup = plan.groups
                [static_cast<std::size_t>(boundary.groupId)];
            const auto boundaryFirst = firstOrderByGroup.find(boundary.groupId);
            const auto boundaryLast = lastOrderByGroup.find(boundary.groupId);
            if (boundaryFirst == firstOrderByGroup.end()
                || boundaryLast == lastOrderByGroup.end()
                || boundaryLast->second - boundaryFirst->second + 1
                    != static_cast<int>(boundaryGroup.entityIds.size()))
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::InternalError, context,
                    DiagnosticCode::ProcessPlanningInvariantViolation,
                    QStringLiteral("加工断面组在最终计划中不连续。"),
                    QStringLiteral("Break boundary assignments are missing or not contiguous."),
                    boundaryDiagnosticValues
                    (
                        input, policy, boundary, boundaryGroup, entities,
                        static_cast<int>(rank), BoundarySide::OnBoundary
                    )
                );
            }

            int maximumLeftLastOrder = -1;
            for (const ProcessGroup& otherGroup : plan.groups)
            {
                if (otherGroup.groupId == boundary.groupId
                    || excludedGroups.find(otherGroup.groupId) != excludedGroups.end()) continue;
                BoundarySide side = BoundarySide::Indeterminate;
                if (otherGroup.kind == ProcessGroupKind::BreakBoundary
                    || otherGroup.kind == ProcessGroupKind::WasteBoundary)
                {
                    side = boundaryRankByGroup.at(otherGroup.groupId)
                        < static_cast<int>(rank)
                        ? BoundarySide::Left : BoundarySide::Right;
                }
                else
                {
                    side = groupBoundarySides.at(otherGroup.groupId)[boundaryIndex];
                }
                const auto otherFirst = firstOrderByGroup.find(otherGroup.groupId);
                const auto otherLast = lastOrderByGroup.find(otherGroup.groupId);
                const bool validSideOrder = otherFirst != firstOrderByGroup.end()
                    && otherLast != lastOrderByGroup.end()
                    && (side == BoundarySide::Left
                        ? otherLast->second < boundaryFirst->second
                        : side == BoundarySide::Right
                            && otherFirst->second > boundaryLast->second);
                if (!validSideOrder)
                {
                    return failure<ProcessPlan>
                    (
                        OperationStatus::InternalError, context,
                        DiagnosticCode::ProcessPlanningInvariantViolation,
                        QStringLiteral("最终加工计划违反断面左右工艺屏障。"),
                        QStringLiteral("A group was scheduled on the wrong side of a Break boundary."),
                        boundaryDiagnosticValues
                        (
                            input, policy, boundary, otherGroup, entities,
                            static_cast<int>(rank), side
                        )
                    );
                }
                if (side == BoundarySide::Left)
                    maximumLeftLastOrder = std::max
                        (maximumLeftLastOrder, otherLast->second);
            }
            if (maximumLeftLastOrder + 1 != boundaryFirst->second)
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::InternalError, context,
                    DiagnosticCode::ProcessPlanningInvariantViolation,
                    QStringLiteral("左侧加工组完成后未立即加工对应断面。"),
                    QStringLiteral("Break boundary does not immediately follow its final left-side group."),
                    boundaryDiagnosticValues
                    (
                        input, policy, boundary, boundaryGroup, entities,
                        static_cast<int>(rank), BoundarySide::OnBoundary
                    )
                );
            }
        }
        if (assignedIds.size() + excludedIds.size() != seen.size())
        {
            return failure<ProcessPlan>
            (
                OperationStatus::InternalError, context, DiagnosticCode::ProcessPlanningInvariantViolation,
                QStringLiteral("加工计划未完整覆盖文档图元。"),
                QStringLiteral("Assignments and exclusions do not cover every PlanningEntity exactly once."),
                diagnosticValues(input, policy, 0U, 0U, -1, -1, -1, -1, 0, 0,
                    static_cast<int>(plan.groups.size()), static_cast<int>(plan.assignments.size()),
                    static_cast<int>(plan.exclusions.size()))
            );
        }

        plan.assignments.shrink_to_fit();
        OperationResult<ProcessPlan> result;
        result.status = OperationStatus::Success;
        result.value = std::move(plan);
        result.mergeDiagnostics(closedLoopDiagnostics);
        result.mergeDiagnostics(zoneSweepDiagnostics);
        return result;
    }
}
